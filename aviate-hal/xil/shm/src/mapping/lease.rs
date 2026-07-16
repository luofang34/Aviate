//! The writer lease: single-writer ownership, crash liveness, and
//! writer identity — two kernel-released locks.
//!
//! The writer holds two exclusive `flock`s for its entire life, and
//! the kernel releases both on ANY process exit, including a crash —
//! while the block's name, ready flag and incarnation all survive
//! one.
//!
//! * **The global lease** (`/tmp/<name>.lease`) serializes writers:
//!   creation takes it FIRST, non-blockingly, so a second writer
//!   fails loudly instead of unlinking a live peer's object out from
//!   under every consumer — and a slow, late cleanup cannot destroy
//!   a successor's object, because a writer that is still alive
//!   still holds its lease and no successor can exist while it does.
//!   Its first 8 bytes are the little-endian `writer_incarnation`
//!   counter, advanced under the lock by every grant
//!   (`wrapping_add(1)`, zero skipped — zero is the block's "not
//!   stamped" sentinel). The counter is persisted BEFORE the value
//!   is ever stamped into a block, so a writer that crashes between
//!   grant and stamp merely burns a number; two writers can never
//!   share one. A pid⊕clock identity cannot promise that: pids
//!   recycle and clocks have granularity, and a same-process
//!   same-instant restart is exactly the case a consumer needs to
//!   detect as replaced. This file is never unlinked: it carries the
//!   counter, and removing a lock file races its next locker (the
//!   classic open-then-unlink hole, where two processes lock two
//!   different inodes behind one path).
//!
//! * **The incarnation token** (`/tmp/<name>.lease.<incarnation>`)
//!   answers the only question a consumer ever asks: "is the writer
//!   that stamped THIS incarnation alive?". The grant locks it
//!   immediately after allocating the value — BEFORE the value can
//!   reach any block header — so a header naming an incarnation
//!   proves its writer once held the token, and a probe finding the
//!   token unlocked proves that writer is dead. The global lease
//!   alone could never say this: between a successor's grant and its
//!   object creation the NAME still resolves to the predecessor's
//!   block while the global lease is genuinely held — and held with
//!   the predecessor's counter still on file until the successor's
//!   pwrite lands, so even "held + counter" revives the corpse if a
//!   probe lands inside that window. The token closes it by
//!   construction: no grant ever touches another writer's token.
//!   The successor unlinks its predecessor's token — no writer can
//!   hold that token again (a value is never granted twice), and a
//!   probe racing the unlink reads "unlocked", the same verdict the
//!   unlink preserves.
//!
//! Every `flock` result is classified by errno, never collapsed to
//! a boolean: `EINTR` is retried, only `EWOULDBLOCK`/`EAGAIN`
//! proves a live exclusive holder, and anything else is a real
//! failure that propagates so callers fail closed instead of
//! mistaking a broken probe for a verdict.

use std::ffi::CString;
use std::io;

/// The global lease path for a shm name: `/aviate_gz_bridge` →
/// `/tmp/aviate_gz_bridge.lease`. `/tmp` because both sides of the
/// contract (this crate and the C++ plugin) must derive the same
/// path without sharing code, and per-user temp dirs differ between
/// processes started from different environments.
pub(crate) fn lease_path(shm_name: &str) -> String {
    format!("/tmp/{}.lease", shm_name.trim_start_matches('/'))
}

/// The incarnation-token path: the global lease path with the
/// incarnation appended (`/tmp/aviate_gz_bridge.lease.7`).
pub(crate) fn token_path(shm_name: &str, incarnation: u64) -> String {
    format!("{}.{incarnation}", lease_path(shm_name))
}

/// One non-blocking `flock` result, classified by errno. Pure so the
/// classification itself is testable without provoking each errno.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FlockVerdict {
    /// The lock was granted.
    Acquired,
    /// A conflicting lock is held (`EWOULDBLOCK`/`EAGAIN`) — the
    /// ONLY outcome that proves a live exclusive holder.
    Held,
    /// A signal landed mid-call (`EINTR`); the call says nothing
    /// about the lock and must be retried.
    Interrupted,
    /// Any other errno: the probe itself is broken (bad descriptor,
    /// no lock table, unsupported file). Treating this as either
    /// "held" or "free" would turn an environment fault into a
    /// silent protocol decision.
    Failed,
}

fn classify_flock(rc: libc::c_int, errno: libc::c_int) -> FlockVerdict {
    if rc == 0 {
        return FlockVerdict::Acquired;
    }
    if errno == libc::EINTR {
        return FlockVerdict::Interrupted;
    }
    // EWOULDBLOCK and EAGAIN are one value on Linux and macOS, but
    // POSIX allows them to differ; accept both by value.
    if errno == libc::EWOULDBLOCK || errno == libc::EAGAIN {
        return FlockVerdict::Held;
    }
    FlockVerdict::Failed
}

/// A classified non-blocking `flock`, with `EINTR` already retried.
enum FlockOutcome {
    Acquired,
    Held,
    Failed(io::Error),
}

fn flock_nb(fd: libc::c_int, op: libc::c_int) -> FlockOutcome {
    loop {
        // SAFETY: flock on a caller-supplied fd has no memory
        // preconditions; an invalid fd is reported through errno and
        // classified below.
        let rc = unsafe { libc::flock(fd, op | libc::LOCK_NB) };
        let errno = if rc == 0 {
            0
        } else {
            io::Error::last_os_error().raw_os_error().unwrap_or(0)
        };
        match classify_flock(rc, errno) {
            FlockVerdict::Acquired => return FlockOutcome::Acquired,
            FlockVerdict::Interrupted => continue,
            FlockVerdict::Held => return FlockOutcome::Held,
            FlockVerdict::Failed => {
                return FlockOutcome::Failed(io::Error::from_raw_os_error(errno))
            }
        }
    }
}

/// Read the counter in the lease file's first 8 bytes
/// (little-endian). A file shorter than 8 bytes (canonically:
/// freshly created and empty) reads as zero-padded.
fn read_counter(fd: libc::c_int) -> io::Result<u64> {
    let mut buf = [0u8; 8];
    let mut n = 0usize;
    while n < buf.len() {
        // SAFETY: pread into the remaining span of a stack buffer
        // whose length bounds the request.
        let rc = unsafe {
            libc::pread(
                fd,
                buf[n..].as_mut_ptr().cast(),
                buf.len() - n,
                n as libc::off_t,
            )
        };
        if rc < 0 {
            let e = io::Error::last_os_error();
            if e.raw_os_error() == Some(libc::EINTR) {
                continue;
            }
            return Err(e);
        }
        if rc == 0 {
            break; // EOF — the remaining bytes stay zero.
        }
        n += rc as usize;
    }
    Ok(u64::from_le_bytes(buf))
}

/// Advance the incarnation counter in the lease file, returning
/// `(previous, granted)`. Caller must hold the exclusive lock on
/// `fd` — the lock is what makes read-increment-write atomic across
/// processes; the first grant on a fresh (empty) file yields 1.
fn advance_counter(fd: libc::c_int) -> io::Result<(u64, u64)> {
    let prev = read_counter(fd)?;
    let mut next = prev.wrapping_add(1);
    if next == 0 {
        next = 1;
    }
    let out = next.to_le_bytes();
    let mut w = 0usize;
    while w < out.len() {
        // SAFETY: pwrite from the remaining span of a stack buffer
        // whose length bounds the request.
        let rc = unsafe {
            libc::pwrite(
                fd,
                out[w..].as_ptr().cast(),
                out.len() - w,
                w as libc::off_t,
            )
        };
        if rc < 0 {
            let e = io::Error::last_os_error();
            if e.raw_os_error() == Some(libc::EINTR) {
                continue;
            }
            return Err(e);
        }
        if rc == 0 {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                "lease counter write made no progress",
            ));
        }
        w += rc as usize;
    }
    // No fsync: the counter must survive PROCESS death (page cache
    // does), and a host reboot clears /tmp and every shm object with
    // it, so there is no state to stay consistent with.
    Ok((prev, next))
}

/// How long an exclusive acquisition keeps retrying a held lock
/// before calling it a real conflict. A `fork()` anywhere in a
/// process holding a lock fd duplicates the fd table and the child's
/// reference pins the flock until its exec — even with `O_CLOEXEC` —
/// so a just-released lock can transiently read as held (~0.4% of
/// acquires on a loaded macOS host). That window clears in well
/// under a millisecond; a live writer holds its locks for its whole
/// life. One hundred milliseconds separates the two cleanly.
const ACQUIRE_ATTEMPTS: u32 = 25;
const ACQUIRE_RETRY_SPACING: std::time::Duration = std::time::Duration::from_millis(4);

/// Open a lock file for exclusive acquisition, creating it if
/// needed.
///
/// `O_CLOEXEC` is load-bearing: `flock` locks ride the open file
/// description, which survives fork+exec. Without it, any process
/// the writer spawns inherits the lock fd and keeps the lock alive
/// after the writer exits — turning the crash signal into a lie for
/// as long as the child runs.
fn open_lock_file(path: &str) -> io::Result<libc::c_int> {
    let cpath = CString::new(path)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in lock path"))?;
    // SAFETY: plain open; the caller owns the returned fd.
    let fd = unsafe {
        libc::open(
            cpath.as_ptr(),
            libc::O_CREAT | libc::O_RDWR | libc::O_CLOEXEC,
            0o666,
        )
    };
    if fd == -1 {
        return Err(io::Error::last_os_error());
    }
    Ok(fd)
}

/// Take an exclusive lock on `fd`, seeing through a fork-window pin
/// (see [`ACQUIRE_ATTEMPTS`]) but never waiting out a live holder —
/// that would serialize a new writer behind a hung one instead of
/// surfacing the conflict. Any other `flock` failure propagates
/// immediately: it is an environment fault, and retrying cannot turn
/// it into an answer.
fn flock_exclusive(fd: libc::c_int, what: &str) -> io::Result<()> {
    for attempt in 0..ACQUIRE_ATTEMPTS {
        match flock_nb(fd, libc::LOCK_EX) {
            FlockOutcome::Acquired => return Ok(()),
            FlockOutcome::Held => {
                if attempt + 1 < ACQUIRE_ATTEMPTS {
                    std::thread::sleep(ACQUIRE_RETRY_SPACING);
                }
            }
            FlockOutcome::Failed(e) => return Err(e),
        }
    }
    Err(io::Error::new(
        io::ErrorKind::WouldBlock,
        format!("another live process holds {what}"),
    ))
}

/// An exclusively held writer lease: the global lock plus the token
/// of the incarnation the grant advanced to. Dropping it releases
/// both; so does any process death.
#[derive(Debug)]
pub(crate) struct WriterLease {
    global_fd: libc::c_int,
    token_fd: libc::c_int,
    incarnation: u64,
}

impl WriterLease {
    /// Take the lease, or fail if a live writer holds it.
    ///
    /// Grant order is the identity invariant: global lock, counter
    /// advance (persisted), token lock — all BEFORE the incarnation
    /// is returned, let alone stamped into a block. A liveness probe
    /// landing anywhere inside this sequence finds the PREDECESSOR's
    /// token already kernel-released and reads Dead; at no instant
    /// does one writer's grant make another writer's incarnation
    /// look alive.
    pub(crate) fn acquire(shm_name: &str) -> io::Result<Self> {
        let global_fd = open_lock_file(&lease_path(shm_name))?;
        let acquired = flock_exclusive(global_fd, "the writer lease")
            .and_then(|()| advance_counter(global_fd));
        let (prev, incarnation) = match acquired {
            Ok(pair) => pair,
            Err(e) => {
                // SAFETY: closing the fd this function opened.
                unsafe { libc::close(global_fd) };
                return Err(e);
            }
        };
        let token_fd = match take_token(shm_name, incarnation) {
            Ok(fd) => fd,
            Err(e) => {
                // SAFETY: closing the fd this function opened.
                unsafe { libc::close(global_fd) };
                return Err(e);
            }
        };
        // The predecessor's token is dead weight once ours is held:
        // its writer is dead (a live one would still hold the global
        // lease this grant just won), its value will never be
        // granted again, and any probe racing this unlink reads
        // "unlocked" — the verdict the unlink preserves. Best
        // effort: a leftover file is litter, not a hazard.
        if prev != 0 && prev != incarnation {
            if let Ok(cpath) = CString::new(token_path(shm_name, prev)) {
                // SAFETY: unlink by owned path; affects no fd.
                unsafe { libc::unlink(cpath.as_ptr()) };
            }
        }
        Ok(Self {
            global_fd,
            token_fd,
            incarnation,
        })
    }

    /// The incarnation this grant advanced the lease counter to:
    /// nonzero, and distinct from every value any earlier grant on
    /// this name received.
    pub(crate) fn incarnation(&self) -> u64 {
        self.incarnation
    }
}

/// Lock the token for a freshly granted incarnation. No process can
/// hold it exclusively — the value has never been granted before —
/// but a consumer's transient shared probe or a fork pin can briefly
/// conflict, so it gets the same bounded retry as the global lease.
fn take_token(shm_name: &str, incarnation: u64) -> io::Result<libc::c_int> {
    let fd = open_lock_file(&token_path(shm_name, incarnation))?;
    match flock_exclusive(fd, "the incarnation token") {
        Ok(()) => Ok(fd),
        Err(e) => {
            // SAFETY: closing the fd this function opened.
            unsafe { libc::close(fd) };
            Err(e)
        }
    }
}

impl Drop for WriterLease {
    fn drop(&mut self) {
        // SAFETY: closing our own fds releases each flock exactly
        // once. Token first: from that instant probes of this
        // incarnation read Dead, while the still-held global lease
        // bars a successor from unlinking a name that may still
        // resolve to this writer's block.
        unsafe {
            libc::close(self.token_fd);
            libc::close(self.global_fd);
        }
    }
}

/// What the locks say about ONE writer — the one that stamped the
/// probed incarnation — with the probe's own health kept separate
/// from its verdict.
#[derive(Debug)]
pub(crate) enum WriterLiveness {
    /// A live process holds the token for the probed incarnation:
    /// the very writer the caller asked about, because no other
    /// process ever locks that token.
    Alive,
    /// The probed incarnation's writer holds no lock — it exited or
    /// crashed, or never ran at all.
    Dead {
        /// Whether the GLOBAL lease is held, i.e. a successor's
        /// grant is live while its own object may not exist yet.
        /// This qualifies the death (a consumer maps it to
        /// Replaced-vs-Gone); it never contributes to liveness.
        takeover_in_progress: bool,
    },
    /// A probe failed; the writer's state is unknowable through it.
    /// Callers must fail closed: an unknown writer is not a live
    /// one, and data must not flow on a broken probe.
    Unknown(io::Error),
}

/// One lock file probed with a non-blocking SHARED lock.
enum LockProbe {
    /// `EWOULDBLOCK`/`EAGAIN` — nothing else: a live exclusive
    /// holder exists.
    HeldExclusively,
    /// The probe lock was granted (and released immediately): no
    /// exclusive holder.
    Unlocked,
    /// No such file.
    Missing,
    /// The probe itself is broken; it carries no verdict.
    Broken(io::Error),
}

fn probe_lock(path: String) -> LockProbe {
    let Ok(cpath) = CString::new(path) else {
        return LockProbe::Broken(io::Error::new(
            io::ErrorKind::InvalidInput,
            "NUL in lock path",
        ));
    };
    // SAFETY: plain libc calls; the fd is always closed before
    // returning.
    unsafe {
        let fd = libc::open(cpath.as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC);
        if fd == -1 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::NotFound {
                LockProbe::Missing
            } else {
                LockProbe::Broken(e)
            };
        }
        let probe = match flock_nb(fd, libc::LOCK_SH) {
            FlockOutcome::Held => LockProbe::HeldExclusively,
            FlockOutcome::Acquired => LockProbe::Unlocked,
            FlockOutcome::Failed(e) => LockProbe::Broken(e),
        };
        libc::close(fd); // also releases the probe lock if taken
        probe
    }
}

/// Probe the liveness of the writer that stamped `incarnation` into
/// the block behind `shm_name`.
///
/// The verdict comes from the incarnation TOKEN alone: held — by the
/// only process that ever locks it — is [`WriterLiveness::Alive`];
/// unlocked or missing is Dead. The global lease is consulted only
/// to qualify a death with "a successor's grant is already live",
/// never to prove liveness: inside a takeover the global lease is
/// genuinely held while the name still resolves to the corpse, and
/// reading that as "my writer lives" is exactly the revival the
/// token exists to prevent.
pub(crate) fn writer_liveness(shm_name: &str, incarnation: u64) -> WriterLiveness {
    match probe_lock(token_path(shm_name, incarnation)) {
        LockProbe::HeldExclusively => WriterLiveness::Alive,
        LockProbe::Broken(e) => WriterLiveness::Unknown(e),
        LockProbe::Unlocked | LockProbe::Missing => match probe_lock(lease_path(shm_name)) {
            LockProbe::HeldExclusively => WriterLiveness::Dead {
                takeover_in_progress: true,
            },
            LockProbe::Unlocked | LockProbe::Missing => WriterLiveness::Dead {
                takeover_in_progress: false,
            },
            LockProbe::Broken(e) => WriterLiveness::Unknown(e),
        },
    }
}

#[cfg(test)]
mod test_support;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#[path = "lease/tests.rs"]
mod tests;
