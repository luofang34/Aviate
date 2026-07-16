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
    validate_attach, AttachError, SharedStateV2, WriterState, EXPECTED_SIZE, LAYOUT_VERSION, MAGIC,
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
    /// an existing one), size it, stamp the fingerprint, publish
    /// `plugin_ready` last.
    ///
    /// The block is NOT cleared here. `shm_open(O_CREAT)` publishes
    /// the NAME before `ftruncate` sizes it, so from that instant an
    /// attacher may be mapping and atomically loading `plugin_ready`
    /// — a bulk non-atomic clear would race those loads (Rust and
    /// C++ both forbid mixing conflicting atomic and non-atomic
    /// accesses without synchronisation). It is also unnecessary: a
    /// freshly created POSIX shm object is zero-filled, and
    /// `ftruncate` zero-extends. The creation window is visible to
    /// attachers as a zero-sized object and is reported as
    /// retryable [`AttachError::Initializing`].
    pub(crate) fn create(name: &str) -> io::Result<Self> {
        let cname = cstring(name)?;
        // The lease comes FIRST. Unlinking a name without owning its
        // lease is how one writer destroys another's live object out
        // from under every consumer; with the lease held, the unlink
        // below can only ever remove a DEAD writer's leftover. The
        // grant also advances the persisted incarnation counter, so
        // the identity stamped below is already burned before any
        // block carries it.
        let lease = super::lease::WriterLease::acquire(name)?;
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
            // Every store is atomic even though nothing should be
            // reading yet: an attacher is already permitted to load
            // `plugin_ready` (the name is public), and mixing a
            // non-atomic store with that concurrent atomic load is a
            // data race by definition, not merely in practice.
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
            let incarnation = lease.incarnation();
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
                lease: Some(lease),
            })
        }
    }

    /// Attach to an existing object, failing closed on any contract
    /// mismatch and on a writer that has not published
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
            // Zero size is the writer's shm_open-before-ftruncate
            // window: retryable, not a foreign object. It must travel
            // as NotReady — callers translate the Contract channel
            // into a permanent, never-retried failure, which would
            // turn this microsecond of normal startup into a
            // deadlock. Anything else short of the block IS
            // foreign.
            if actual == 0 {
                libc::close(fd);
                return Err(AttachFailure::NotReady);
            }
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
            // Re-check readiness AFTER reading the fingerprint and
            // incarnation: a writer tearing down or being replaced
            // mid-inspection cleared the flag, and the values just
            // read may straddle two objects' lifetimes. Same bracket
            // `inspect` uses.
            let ready_again =
                AtomicU32::from_ptr(core::ptr::addr_of_mut!((*base).header.plugin_ready))
                    .load(Ordering::Acquire);
            if ready_again == 0 {
                libc::munmap(ptr, EXPECTED_SIZE);
                return Err(AttachFailure::NotReady);
            }
            // And demand that THIS block's writer is the live one.
            // A crashed writer's block passes every in-memory check
            // above — name, size, fingerprint, ready, incarnation
            // all survive the crash — so without the lease probe a
            // consumer happily attaches to a corpse and trusts its
            // frozen final snapshot. And a held lease alone is not
            // enough: between a successor's grant and its object
            // creation, the name still resolves to the corpse while
            // "some writer" is genuinely alive — the grant's counter
            // must equal the incarnation just read from the header,
            // or this mapping belongs to a writer the lease does not
            // vouch for. Mismatch is NotReady (the successor's
            // object is coming); a probe that cannot run is not a
            // verdict, and its error propagates instead of being
            // read as either answer.
            match super::lease::writer_liveness(name) {
                super::lease::WriterLiveness::Alive(lease_incarnation)
                    if lease_incarnation == incarnation => {}
                super::lease::WriterLiveness::Alive(_) | super::lease::WriterLiveness::Dead => {
                    libc::munmap(ptr, EXPECTED_SIZE);
                    return Err(AttachFailure::NotReady);
                }
                super::lease::WriterLiveness::Unknown(e) => {
                    libc::munmap(ptr, EXPECTED_SIZE);
                    return Err(AttachFailure::Io(e));
                }
            }
            Ok(Self {
                base,
                name: cname,
                owner: false,
                incarnation,
                lease: None,
            })
        }
    }
}

/// What the name resolves to RIGHT NOW, relative to the incarnation
/// the caller attached to. Maps the live object briefly rather than
/// trusting the caller's mapping, which is the whole point: a
/// crashed writer's orphaned memory answers every question about
/// itself perfectly.
///
/// Applies the same fail-closed order as `attach`: readiness first
/// (Acquire), then the fingerprint, then a re-check of readiness —
/// so a block that is being torn down or re-created mid-inspection
/// is reported as `Initializing`, never mistaken for a healthy peer.
///
/// Inode identity is not an option — macOS reports `st_dev = 0` and
/// `st_ino = 0` for every POSIX shm object, so two distinct objects
/// are indistinguishable by stat.
///
/// Slow path: a few syscalls plus a transient mapping. Consumers
/// poll it at staleness-check rates (~1 Hz), never per frame.
pub(super) fn writer_state(name: &CString, attached_incarnation: u64) -> WriterState {
    // SAFETY: plain libc calls on an owned CString; the fd is closed
    // and the transient mapping unmapped on every path. Only header
    // lanes are read, atomically.
    unsafe {
        let fd = libc::shm_open(name.as_ptr(), libc::O_RDONLY, 0);
        if fd == -1 {
            // The name is gone: the writer exited and unlinked. The
            // caller's mapping is an orphan it keeps alive itself.
            return WriterState::Gone;
        }
        let mut st: libc::stat = core::mem::zeroed();
        if libc::fstat(fd, &mut st) == -1 {
            libc::close(fd);
            return WriterState::Gone;
        }
        let actual = st.st_size.max(0) as usize;
        if actual == 0 {
            libc::close(fd);
            return WriterState::Initializing;
        }
        if actual < EXPECTED_SIZE {
            libc::close(fd);
            return WriterState::ContractMismatch;
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
            return WriterState::Gone;
        }
        let base = ptr.cast::<SharedStateV2>();
        let state = inspect(base, attached_incarnation);
        libc::munmap(ptr, EXPECTED_SIZE);
        state
    }
}

/// Classify a transiently-mapped block. Readiness brackets the
/// fingerprint read on both sides, so a writer that is still
/// stamping (or already tearing down) is `Initializing` rather than
/// a bogus `ContractMismatch` or a false `Current`.
///
/// # Safety
/// `base` must point at an `EXPECTED_SIZE` mapping of a shm object.
unsafe fn inspect(base: *const SharedStateV2, attached_incarnation: u64) -> WriterState {
    let ready_first =
        AtomicU32::from_ptr(core::ptr::addr_of!((*base).header.plugin_ready) as *mut u32)
            .load(Ordering::Acquire);
    if ready_first == 0 {
        return WriterState::Initializing;
    }
    let magic = load_u64(core::ptr::addr_of!((*base).header.magic));
    let version = load_u32(core::ptr::addr_of!((*base).header.layout_version));
    let declared = load_u32(core::ptr::addr_of!((*base).header.declared_size));
    let incarnation = load_u64(core::ptr::addr_of!((*base).header.writer_incarnation));
    let ready_again =
        AtomicU32::from_ptr(core::ptr::addr_of!((*base).header.plugin_ready) as *mut u32)
            .load(Ordering::Acquire);
    if ready_again == 0 {
        // Readiness dropped while we looked: the values we just read
        // may belong to a block being replaced. Say so instead of
        // guessing.
        return WriterState::Initializing;
    }
    if validate_attach(magic, version, declared, EXPECTED_SIZE).is_err() {
        return WriterState::ContractMismatch;
    }
    if incarnation != attached_incarnation {
        return WriterState::Replaced;
    }
    WriterState::Current
}

/// Bind a raw `Current` to the lease before letting it stand:
/// every in-block signal survives a crash, so only the
/// kernel-released lock can distinguish "same healthy writer" from
/// "corpse of the writer I attached to" — and only the lock's
/// COUNTER can distinguish "my writer is alive" from "somebody is
/// alive". A raw `Current` proves the name still resolves to the
/// attached incarnation; the lease must vouch for that same
/// incarnation, or a successor mid-takeover (grant persisted,
/// object not yet replaced) would revive the corpse as healthy.
///
/// * lease incarnation == attachment incarnation → `Current`.
/// * held by a different incarnation → `Replaced`: this
///   attachment's writer has a live successor; re-attach.
/// * lease free → `Gone`.
/// * probe broken → `Gone`, fail closed. `Current` is a promise
///   that reads are trustworthy, and a broken probe cannot back
///   that promise; the re-attach this triggers runs the same probe
///   through [`Mapping::attach`], which propagates the underlying
///   error as [`AttachFailure::Io`] instead of swallowing it.
pub(super) fn confirm_alive(
    name: &CString,
    attached_incarnation: u64,
    state: WriterState,
) -> WriterState {
    match state {
        WriterState::Current => {
            let liveness = name
                .to_str()
                .map(super::lease::writer_liveness)
                .unwrap_or_else(|_| {
                    super::lease::WriterLiveness::Unknown(io::Error::new(
                        io::ErrorKind::InvalidInput,
                        "shm name is not UTF-8",
                    ))
                });
            match liveness {
                super::lease::WriterLiveness::Alive(lease_incarnation)
                    if lease_incarnation == attached_incarnation =>
                {
                    WriterState::Current
                }
                super::lease::WriterLiveness::Alive(_) => WriterState::Replaced,
                super::lease::WriterLiveness::Dead | super::lease::WriterLiveness::Unknown(_) => {
                    WriterState::Gone
                }
            }
        }
        other => other,
    }
}

fn cstring(name: &str) -> io::Result<CString> {
    CString::new(name).map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in shm name"))
}

#[cfg(test)]
mod test_support;
