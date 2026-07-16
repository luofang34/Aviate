//! The writer lease: single-writer ownership, crash liveness, and
//! writer identity in one primitive.
//!
//! The writer holds an exclusive `flock` on a small lease file for
//! its entire life. That gives three properties nothing inside the
//! shm block can give:
//!
//! * **Liveness.** The kernel releases the lock on ANY process exit,
//!   including a crash — while the block's name, ready flag and
//!   incarnation all survive one. "Is the lease held?" is therefore
//!   the only trustworthy answer to "is the writer alive?".
//! * **Single ownership.** Creation takes the lease FIRST,
//!   non-blockingly. A second writer fails loudly instead of
//!   unlinking a live peer's object out from under every consumer —
//!   and a slow, late cleanup cannot destroy a successor's object,
//!   because a writer that is still alive still holds its lease and
//!   no successor can exist while it does.
//! * **Identity.** The lease file's first 8 bytes are a
//!   little-endian `writer_incarnation` counter, advanced under the
//!   exclusive lock by every grant (`wrapping_add(1)`, zero skipped
//!   — zero is the block's "not stamped" sentinel). The counter is
//!   persisted BEFORE the value is ever stamped into a block, so a
//!   writer that crashes between grant and stamp merely burns a
//!   number; two writers can never share one. A pid⊕clock identity
//!   cannot promise that: pids recycle and clocks have granularity,
//!   and a same-process same-instant restart is exactly the case a
//!   consumer needs to detect as [`Replaced`].
//!
//! [`Replaced`]: aviate_xil_contract::WriterState::Replaced
//!
//! The lease file is never unlinked: it carries the counter, and
//! removing a lock file races its next locker (the classic
//! open-then-unlink hole, where two processes lock two different
//! inodes behind one path).
//!
//! Every `flock` result is classified by errno, never collapsed to
//! a boolean: `EINTR` is retried, only `EWOULDBLOCK`/`EAGAIN`
//! proves a live exclusive holder, and anything else is a real
//! failure that propagates so callers fail closed instead of
//! mistaking a broken probe for a verdict.

use std::ffi::CString;
use std::io;

/// The lease path for a shm name: `/aviate_gz_bridge` →
/// `/tmp/aviate_gz_bridge.lease`. `/tmp` because both sides of the
/// contract (this crate and the C++ plugin) must derive the same
/// path without sharing code, and per-user temp dirs differ between
/// processes started from different environments.
pub(crate) fn lease_path(shm_name: &str) -> String {
    format!("/tmp/{}.lease", shm_name.trim_start_matches('/'))
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

/// Advance the incarnation counter stored in the lease file's first
/// 8 bytes (little-endian). Caller must hold the exclusive lock on
/// `fd` — the lock is what makes read-increment-write atomic across
/// processes. A file shorter than 8 bytes (canonically: freshly
/// created and empty) reads as zero-padded, so the first grant on a
/// fresh file yields 1.
fn advance_counter(fd: libc::c_int) -> io::Result<u64> {
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
    let mut next = u64::from_le_bytes(buf).wrapping_add(1);
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
    Ok(next)
}

/// How long `acquire` keeps retrying a held lock before calling it a
/// real conflict. A `fork()` anywhere in a process holding a lease fd
/// duplicates the fd table and the child's reference pins the flock
/// until its exec — even with `O_CLOEXEC` — so a just-released lock
/// can transiently read as held (~0.4% of acquires on a loaded macOS
/// host). That window clears in well under a millisecond; a live
/// writer holds its lease for its whole life. One hundred
/// milliseconds separates the two cleanly.
const ACQUIRE_ATTEMPTS: u32 = 25;
const ACQUIRE_RETRY_SPACING: std::time::Duration = std::time::Duration::from_millis(4);

/// An exclusively held writer lease carrying the incarnation its
/// grant advanced to. Dropping it releases the lock; so does any
/// process death.
#[derive(Debug)]
pub(crate) struct WriterLease {
    fd: libc::c_int,
    incarnation: u64,
}

impl WriterLease {
    /// Take the lease, or fail if a live writer holds it. Retries a
    /// HELD verdict only long enough to see through a fork-window
    /// pin (see [`ACQUIRE_ATTEMPTS`]); it never waits out a live
    /// writer — that would serialize a new writer behind a hung one
    /// instead of surfacing the conflict. Any other `flock` failure
    /// propagates immediately: it is an environment fault, and
    /// retrying cannot turn it into an answer.
    pub(crate) fn acquire(shm_name: &str) -> io::Result<Self> {
        let path = CString::new(lease_path(shm_name))
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in lease path"))?;
        // SAFETY: plain libc calls; the fd is closed on every failure
        // path and owned by the returned value on success.
        //
        // O_CLOEXEC is load-bearing: `flock` locks ride the open file
        // description, which survives fork+exec. Without it, any
        // process the writer spawns inherits the lease fd and keeps
        // the lock alive after the writer exits — turning the crash
        // signal into a lie for as long as the child runs.
        unsafe {
            let fd = libc::open(
                path.as_ptr(),
                libc::O_CREAT | libc::O_RDWR | libc::O_CLOEXEC,
                0o666,
            );
            if fd == -1 {
                return Err(io::Error::last_os_error());
            }
            for attempt in 0..ACQUIRE_ATTEMPTS {
                match flock_nb(fd, libc::LOCK_EX) {
                    FlockOutcome::Acquired => match advance_counter(fd) {
                        Ok(incarnation) => return Ok(Self { fd, incarnation }),
                        Err(e) => {
                            libc::close(fd);
                            return Err(e);
                        }
                    },
                    FlockOutcome::Held => {
                        if attempt + 1 < ACQUIRE_ATTEMPTS {
                            std::thread::sleep(ACQUIRE_RETRY_SPACING);
                        }
                    }
                    FlockOutcome::Failed(e) => {
                        libc::close(fd);
                        return Err(e);
                    }
                }
            }
            libc::close(fd);
            Err(io::Error::new(
                io::ErrorKind::WouldBlock,
                "another live writer holds the lease",
            ))
        }
    }

    /// The incarnation this grant advanced the lease counter to:
    /// nonzero, and distinct from every value any earlier grant on
    /// this name received.
    pub(crate) fn incarnation(&self) -> u64 {
        self.incarnation
    }
}

impl Drop for WriterLease {
    fn drop(&mut self) {
        // SAFETY: closing our own fd releases the flock exactly once.
        unsafe {
            libc::close(self.fd);
        }
    }
}

/// What the lease says about the writer, with the probe's own health
/// kept separate from its verdict.
#[derive(Debug)]
pub(crate) enum WriterLiveness {
    /// A live process holds the exclusive writer lock.
    Alive,
    /// No process holds it — the writer exited or crashed, or none
    /// has ever run on this host (no lease file).
    Dead,
    /// The probe itself failed; the writer's state is unknowable
    /// through it. Callers must fail closed: an unknown writer is
    /// not a live one, and data must not flow on a broken probe.
    Unknown(io::Error),
}

/// Probe the lease for `shm_name` with a non-blocking SHARED lock:
/// a HELD verdict (`EWOULDBLOCK`/`EAGAIN` — nothing else) proves a
/// live exclusive holder; a granted probe lock proves there is none
/// and is released immediately. A missing lease file means no writer
/// has ever run on this host — equally dead. Every other outcome is
/// [`WriterLiveness::Unknown`].
pub(crate) fn writer_liveness(shm_name: &str) -> WriterLiveness {
    let Ok(path) = CString::new(lease_path(shm_name)) else {
        return WriterLiveness::Unknown(io::Error::new(
            io::ErrorKind::InvalidInput,
            "NUL in lease path",
        ));
    };
    // SAFETY: plain libc calls; the fd is always closed before
    // returning.
    unsafe {
        let fd = libc::open(path.as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC);
        if fd == -1 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::NotFound {
                WriterLiveness::Dead
            } else {
                WriterLiveness::Unknown(e)
            };
        }
        let outcome = flock_nb(fd, libc::LOCK_SH);
        libc::close(fd); // also releases the probe lock if taken
        match outcome {
            FlockOutcome::Held => WriterLiveness::Alive,
            FlockOutcome::Acquired => WriterLiveness::Dead,
            FlockOutcome::Failed(e) => WriterLiveness::Unknown(e),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#[path = "lease/tests.rs"]
mod tests;
