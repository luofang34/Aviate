//! Test-only shm object factories: the creation-window shapes the
//! attach rules must survive, manufactured on demand because the
//! real windows are microseconds wide and unhittable on cue.

use std::ffi::CString;
use std::io;

use aviate_xil_contract::{SharedStateV2, EXPECTED_SIZE};

use super::super::lease;
use super::super::Mapping;
use super::cstring;

impl Mapping {
    /// Create the object at SIZE ZERO — the `shm_open`-published,
    /// not-yet-`ftruncate`d window. Test-only, like
    /// [`Mapping::create_mid_init_for_test`]: the real window is
    /// microseconds wide, so the regression test manufactures it.
    /// The returned value holds the lease and the (unsized) object;
    /// dropping it unlinks the name.
    pub(crate) fn create_zero_sized_for_test(name: &str) -> io::Result<ZeroSizedObject> {
        let lease = lease::WriterLease::acquire(name)?;
        let cname = cstring(name)?;
        // SAFETY: plain libc calls; the fd is closed immediately (the
        // named object persists until unlinked in Drop).
        unsafe {
            libc::shm_unlink(cname.as_ptr());
            let fd = libc::shm_open(
                cname.as_ptr(),
                libc::O_CREAT | libc::O_RDWR,
                0o666 as libc::c_uint,
            );
            if fd == -1 {
                return Err(io::Error::last_os_error());
            }
            libc::close(fd);
        }
        Ok(ZeroSizedObject {
            name: cname,
            _lease: lease,
        })
    }

    /// Create the object and stamp NOTHING — models a writer caught
    /// mid-initialisation (block zeroed, fingerprint not yet
    /// written, `plugin_ready` still 0). Test-only: it exists so the
    /// attach order has a regression test, because that window is
    /// otherwise microseconds wide and unhittable on demand.
    pub(crate) fn create_mid_init_for_test(name: &str) -> io::Result<Self> {
        let lease = lease::WriterLease::acquire(name)?;
        let cname = cstring(name)?;
        // SAFETY: same libc sequence as `create`, minus the
        // fingerprint and readiness publication.
        unsafe {
            libc::shm_unlink(cname.as_ptr());
            let fd = libc::shm_open(
                cname.as_ptr(),
                libc::O_CREAT | libc::O_RDWR,
                0o666 as libc::c_uint,
            );
            if fd == -1 {
                return Err(io::Error::last_os_error());
            }
            if libc::ftruncate(fd, EXPECTED_SIZE as libc::off_t) == -1 {
                let e = io::Error::last_os_error();
                libc::close(fd);
                return Err(e);
            }
            let ptr = libc::mmap(
                core::ptr::null_mut(),
                EXPECTED_SIZE,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            if ptr == libc::MAP_FAILED {
                return Err(io::Error::last_os_error());
            }
            core::ptr::write_bytes(ptr.cast::<u8>(), 0, EXPECTED_SIZE);
            Ok(Self {
                base: ptr.cast::<SharedStateV2>(),
                name: cname,
                owner: true,
                incarnation: 0,
                lease: Some(lease),
            })
        }
    }
}

/// Test-only handle for a zero-sized shm object (see
/// [`Mapping::create_zero_sized_for_test`]).
#[derive(Debug)]
pub(crate) struct ZeroSizedObject {
    name: CString,
    _lease: lease::WriterLease,
}

impl Drop for ZeroSizedObject {
    fn drop(&mut self) {
        // SAFETY: unlinking the name we created; mappings (none) are
        // unaffected.
        unsafe {
            libc::shm_unlink(self.name.as_ptr());
        }
    }
}
