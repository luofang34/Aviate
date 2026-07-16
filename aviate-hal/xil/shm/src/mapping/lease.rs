//! The writer lease: single-writer ownership and crash liveness in
//! one primitive.
//!
//! The writer holds an exclusive `flock` on a small lease file for
//! its entire life. That gives two properties nothing inside the shm
//! block can give:
//!
//! * **Liveness.** The kernel releases the lock on ANY process exit,
//!   including a crash — while the block's name, ready flag and
//!   incarnation all survive one. "Is the lease held?" is therefore
//!   the only trustworthy answer to "is the writer alive?".
//! * **Single ownership.** Creation takes the lease FIRST,
//!   non-blockingly. A second writer fails loudly instead of
//!   unlinking a live peer's object out from under every consumer —
//!   and a late, slow cleanup by an OLD writer cannot destroy a new
//!   writer's object, because an old writer that is still alive
//!   still holds its lease and no new writer can have been created.
//!
//! The lease file is never unlinked: removing a lock file races its
//! next locker (the classic open-then-unlink hole, where two
//! processes lock two different inodes behind one path).

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

/// An exclusively held writer lease. Dropping it releases the lock;
/// so does any process death.
#[derive(Debug)]
pub(crate) struct WriterLease {
    fd: libc::c_int,
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

impl WriterLease {
    /// Take the lease, or fail if a live writer holds it. Retries
    /// only long enough to see through a fork-window pin (see
    /// [`ACQUIRE_ATTEMPTS`]); it never waits out a live writer —
    /// that would serialize a new writer behind a hung one instead
    /// of surfacing the conflict.
    pub(crate) fn acquire(shm_name: &str) -> io::Result<Self> {
        let path = CString::new(lease_path(shm_name))
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in lease path"))?;
        // SAFETY: plain libc calls; the fd is closed on the failure
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
            let mut last = io::Error::from(io::ErrorKind::WouldBlock);
            for attempt in 0..ACQUIRE_ATTEMPTS {
                if libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) == 0 {
                    return Ok(Self { fd });
                }
                last = io::Error::last_os_error();
                if attempt + 1 < ACQUIRE_ATTEMPTS {
                    std::thread::sleep(ACQUIRE_RETRY_SPACING);
                }
            }
            libc::close(fd);
            Err(io::Error::new(
                io::ErrorKind::WouldBlock,
                format!("another live writer holds the lease: {last}"),
            ))
        }
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

/// Whether a live writer currently holds the lease for `shm_name`.
///
/// Probes with a non-blocking SHARED lock: if the probe succeeds no
/// process holds the exclusive writer lock, so the writer is dead or
/// absent; the probe lock is released immediately. A missing lease
/// file means no writer has ever run on this host — equally dead.
pub(crate) fn writer_alive(shm_name: &str) -> bool {
    let Ok(path) = CString::new(lease_path(shm_name)) else {
        return false;
    };
    // SAFETY: plain libc calls; the fd is always closed before
    // returning.
    unsafe {
        let fd = libc::open(path.as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC);
        if fd == -1 {
            return false;
        }
        let probe = libc::flock(fd, libc::LOCK_SH | libc::LOCK_NB);
        libc::close(fd); // also releases the probe lock if taken
        probe == -1
    }
}
