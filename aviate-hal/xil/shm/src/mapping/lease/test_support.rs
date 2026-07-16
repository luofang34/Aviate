//! Test-only lease shapes: the grant windows the liveness rules
//! must survive, manufactured on demand because the real windows are
//! microseconds wide and unhittable on cue.

use std::io;

use super::{flock_exclusive, lease_path, open_lock_file, WriterLease};

/// A grant frozen at its most dangerous instant: the global lease is
/// held, but the counter has NOT been advanced and no token has been
/// taken — the file still carries the predecessor's incarnation, and
/// the name may still resolve to the predecessor's block. A probe of
/// the predecessor's incarnation must read Dead here; before the
/// token existed, this exact window read "held + predecessor's
/// counter" and revived the corpse.
#[derive(Debug)]
pub(crate) struct PreCounterGrant {
    fd: libc::c_int,
}

impl WriterLease {
    /// Acquire ONLY the global lease and stop — no counter advance,
    /// no token. Models a successor paused between winning the lock
    /// and publishing its identity; dropping the value releases the
    /// lock and lets a real grant proceed.
    pub(crate) fn acquire_global_only_for_test(shm_name: &str) -> io::Result<PreCounterGrant> {
        let fd = open_lock_file(&lease_path(shm_name))?;
        match flock_exclusive(fd, "the writer lease") {
            Ok(()) => Ok(PreCounterGrant { fd }),
            Err(e) => {
                // SAFETY: closing the fd this function opened.
                unsafe { libc::close(fd) };
                Err(e)
            }
        }
    }
}

impl Drop for PreCounterGrant {
    fn drop(&mut self) {
        // SAFETY: closing our own fd releases the flock exactly once.
        unsafe {
            libc::close(self.fd);
        }
    }
}
