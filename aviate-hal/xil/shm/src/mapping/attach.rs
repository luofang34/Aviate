//! Construction and fail-closed attach: how a mapping comes to
//! exist, and every rule it must satisfy before a single payload
//! field is interpreted.
//!
//! Split out of `mapping.rs` so the block's lifecycle rules read in
//! one place, separate from the per-cycle publish/read paths.

use core::sync::atomic::{AtomicU32, Ordering};
use std::ffi::CString;
use std::io;

use aviate_xil_contract::{
    validate_attach, AttachError, SharedStateV2, EXPECTED_SIZE, LAYOUT_VERSION, MAGIC,
};

use super::lanes::{load_u32, load_u64, store_u32, store_u64};
use super::Mapping;

/// Why an attach was refused.
#[derive(Debug)]
pub enum AttachFailure {
    /// The shm object does not exist yet (writer not up) or the OS
    /// refused the mapping.
    Io(io::Error),
    /// The object exists but is not a valid contract block.
    Contract(AttachError),
    /// The block validates but the simulation writer has not
    /// published `plugin_ready` — retry once the writer is up. An
    /// attacher must never read payload fields from a block whose
    /// writer is absent or mid-initialization.
    NotReady,
}

impl core::fmt::Display for AttachFailure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AttachFailure::Io(e) => write!(f, "shm attach I/O failure: {e}"),
            AttachFailure::Contract(e) => write!(f, "shm attach contract violation: {e:?}"),
            AttachFailure::NotReady => write!(f, "shm block present but writer not ready"),
        }
    }
}

impl std::error::Error for AttachFailure {}

impl Mapping {
    /// Create (or re-create) the shm object and initialize the
    /// header: unlink any stale object (macOS refuses `ftruncate` on
    /// an existing one), zero the block, stamp the fingerprint, set
    /// `reset_generation = 1`, publish `plugin_ready` last.
    pub(crate) fn create(name: &str) -> io::Result<Self> {
        let cname = cstring(name)?;
        // SAFETY: plain libc calls on an owned CString; failure
        // paths close what was opened; the block is exclusively ours
        // between shm_open(O_CREAT after unlink) and plugin_ready.
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
                libc::shm_unlink(cname.as_ptr());
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
            // The fd is not needed after mmap; the mapping keeps the
            // object alive.
            libc::close(fd);
            if ptr == libc::MAP_FAILED {
                let e = io::Error::last_os_error();
                libc::shm_unlink(cname.as_ptr());
                return Err(e);
            }
            let base = ptr.cast::<SharedStateV2>();
            core::ptr::write_bytes(ptr.cast::<u8>(), 0, EXPECTED_SIZE);
            // The block is exclusively ours until plugin_ready is
            // published, so the fingerprint stores need no atomics —
            // the Release store below is what makes them visible.
            store_u64(core::ptr::addr_of_mut!((*base).header.magic), MAGIC);
            store_u32(
                core::ptr::addr_of_mut!((*base).header.layout_version),
                LAYOUT_VERSION,
            );
            store_u32(
                core::ptr::addr_of_mut!((*base).header.declared_size),
                EXPECTED_SIZE as u32,
            );
            store_u32(core::ptr::addr_of_mut!((*base).header.reset_generation), 1);
            let incarnation = fresh_incarnation();
            store_u64(
                core::ptr::addr_of_mut!((*base).header.writer_incarnation),
                incarnation,
            );
            AtomicU32::from_ptr(core::ptr::addr_of_mut!((*base).header.plugin_ready))
                .store(1, Ordering::Release);
            Ok(Self {
                base,
                name: cname,
                owner: true,
                incarnation,
            })
        }
    }

    /// Attach to an existing object, failing closed on any contract
    /// mismatch (#262) and on a writer that has not published
    /// readiness. `read_only` maps `PROT_READ` over an `O_RDONLY`
    /// descriptor, so the OS itself refuses consumer writes.
    pub(crate) fn attach(name: &str, read_only: bool) -> Result<Self, AttachFailure> {
        let cname = cstring(name).map_err(AttachFailure::Io)?;
        // SAFETY: plain libc calls; payload fields are interpreted
        // only after validate_attach passes on the fingerprint and
        // plugin_ready is observed non-zero.
        unsafe {
            let oflag = if read_only {
                libc::O_RDONLY
            } else {
                libc::O_RDWR
            };
            let fd = libc::shm_open(cname.as_ptr(), oflag, 0);
            if fd == -1 {
                return Err(AttachFailure::Io(io::Error::last_os_error()));
            }
            let mut st: libc::stat = core::mem::zeroed();
            if libc::fstat(fd, &mut st) == -1 {
                let e = io::Error::last_os_error();
                libc::close(fd);
                return Err(AttachFailure::Io(e));
            }
            let actual = st.st_size.max(0) as usize;
            if actual < EXPECTED_SIZE {
                libc::close(fd);
                return Err(AttachFailure::Contract(AttachError::MappingTooSmall {
                    actual,
                }));
            }
            let prot = if read_only {
                libc::PROT_READ
            } else {
                libc::PROT_READ | libc::PROT_WRITE
            };
            let ptr = libc::mmap(
                core::ptr::null_mut(),
                EXPECTED_SIZE,
                prot,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            if ptr == libc::MAP_FAILED {
                return Err(AttachFailure::Io(io::Error::last_os_error()));
            }
            let base = ptr.cast::<SharedStateV2>();
            // READINESS FIRST, fingerprint second. The writer zeroes
            // the block, stamps the fingerprint, and only THEN
            // publishes plugin_ready with Release — so an Acquire
            // load that observes a non-zero ready proves the
            // fingerprint is already visible. Reading the fingerprint
            // first would inspect the zeroed block of a writer that
            // is merely mid-initialisation and report BadMagic: a
            // permanent ContractMismatch (which callers correctly do
            // not retry) for what is a normal, retryable startup
            // window.
            let ready = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*base).header.plugin_ready))
                .load(Ordering::Acquire);
            if ready == 0 {
                libc::munmap(ptr, EXPECTED_SIZE);
                return Err(AttachFailure::NotReady);
            }
            let magic = load_u64(core::ptr::addr_of!((*base).header.magic));
            let version = load_u32(core::ptr::addr_of!((*base).header.layout_version));
            let declared = load_u32(core::ptr::addr_of!((*base).header.declared_size));
            if let Err(e) = validate_attach(magic, version, declared, actual) {
                libc::munmap(ptr, EXPECTED_SIZE);
                return Err(AttachFailure::Contract(e));
            }
            let incarnation = load_u64(core::ptr::addr_of!((*base).header.writer_incarnation));
            Ok(Self {
                base,
                name: cname,
                owner: false,
                incarnation,
            })
        }
    }
}

impl Mapping {
    /// Create the object and stamp NOTHING — models a writer caught
    /// mid-initialisation (block zeroed, fingerprint not yet
    /// written, `plugin_ready` still 0). Test-only: it exists so the
    /// attach order has a regression test, because that window is
    /// otherwise microseconds wide and unhittable on demand.
    #[cfg(test)]
    pub(crate) fn create_mid_init_for_test(name: &str) -> io::Result<Self> {
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
            })
        }
    }
}

/// The `writer_incarnation` of whatever object the name resolves to
/// RIGHT NOW, or `None` if it does not resolve. Maps the live object
/// briefly rather than trusting this session's mapping, which is the
/// whole point: a crashed writer's orphaned memory still answers
/// every question about itself perfectly.
///
/// Inode identity is not an option — macOS reports `st_dev = 0` and
/// `st_ino = 0` for every POSIX shm object, so two distinct objects
/// are indistinguishable by stat.
///
/// Slow path: three syscalls plus a transient mapping. Consumers
/// poll it at staleness-check rates (~1 Hz), never per frame.
pub(super) fn live_incarnation(name: &CString) -> Option<u64> {
    // SAFETY: plain libc calls on an owned CString; the fd is closed
    // and the transient mapping unmapped on every path. Only the
    // header's incarnation lane is read, atomically.
    unsafe {
        let fd = libc::shm_open(name.as_ptr(), libc::O_RDONLY, 0);
        if fd == -1 {
            return None;
        }
        let mut st: libc::stat = core::mem::zeroed();
        if libc::fstat(fd, &mut st) == -1 || (st.st_size.max(0) as usize) < EXPECTED_SIZE {
            libc::close(fd);
            return None;
        }
        let ptr = libc::mmap(
            core::ptr::null_mut(),
            EXPECTED_SIZE,
            libc::PROT_READ,
            libc::MAP_SHARED,
            fd,
            0,
        );
        libc::close(fd);
        if ptr == libc::MAP_FAILED {
            return None;
        }
        let base = ptr.cast::<SharedStateV2>();
        let live = load_u64(core::ptr::addr_of!((*base).header.writer_incarnation));
        libc::munmap(ptr, EXPECTED_SIZE);
        Some(live)
    }
}

/// A value that never repeats across writers on this host: the
/// creating process's id folded with the wall clock, so a restarted
/// writer (same pid reused, or same instant) cannot collide with the
/// object it replaced. Zero is reserved for "not stamped".
fn fresh_incarnation() -> u64 {
    let pid = std::process::id() as u64;
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    (nanos.rotate_left(16) ^ pid) | 1
}

fn cstring(name: &str) -> io::Result<CString> {
    CString::new(name).map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in shm name"))
}
