//! The mapped session: creation, fail-closed attach, and typed
//! access to every contract field.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::ffi::CString;
use std::io;

use aviate_xil_contract::{
    pack_fc_status, pack_lifecycle_request, seqlock_read, seqlock_write, unpack_fc_status,
    unpack_lifecycle_request, validate_attach, AttachError, FcState, LifecycleRequest,
    SharedStateV2, EXPECTED_SIZE, LAYOUT_VERSION, MAGIC,
};

/// Why an [`ShmSession::attach`] was refused.
#[derive(Debug)]
pub enum AttachFailure {
    /// The shm object does not exist yet (writer not up) or the OS
    /// refused the mapping.
    Io(io::Error),
    /// The object exists but is not a valid contract block.
    Contract(AttachError),
}

impl core::fmt::Display for AttachFailure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AttachFailure::Io(e) => write!(f, "shm attach I/O failure: {e}"),
            AttachFailure::Contract(e) => write!(f, "shm attach contract violation: {e:?}"),
        }
    }
}

impl std::error::Error for AttachFailure {}

/// One coherent `{step, time, state}` snapshot taken under the model
/// seqlock (#265: `sim_step`/`time_us` are the sim-time authority).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ModelStateSnapshot {
    /// Physics step counter (monotonic across resets).
    pub sim_step: u64,
    /// Simulation time (µs) (rewinds on world reset).
    pub time_us: u64,
    /// Position (m), world ENU.
    pub pos: [f64; 3],
    /// Orientation quaternion [w, x, y, z], ENU/FLU.
    pub quat: [f64; 4],
    /// Linear velocity [m/s], world ENU.
    pub vel: [f64; 3],
    /// Angular velocity [rad/s], body FLU.
    pub ang_vel: [f64; 3],
}

/// A mapped contract block. `Send` but deliberately not `Sync`
/// (one session per thread; cheap to attach another).
#[derive(Debug)]
pub struct ShmSession {
    base: *mut SharedStateV2,
    name: CString,
    /// Creator unlinks the object on drop; attachers never do.
    owner: bool,
}

// SAFETY: the mapping is process-shared memory accessed only through
// atomic/volatile operations below; moving the session between
// threads moves only the pointer and ownership flag.
unsafe impl Send for ShmSession {}

macro_rules! atomic_u32_accessor {
    ($(#[$doc:meta])* $get:ident, $set:ident, $field:ident) => {
        $(#[$doc])*
        pub fn $get(&self) -> u32 {
            // SAFETY: field is a naturally-aligned u32 inside the
            // validated mapping; AtomicU32 has the same layout.
            unsafe {
                AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).control.$field))
                    .load(Ordering::Acquire)
            }
        }
        /// Atomically store the field (see the getter's docs).
        pub fn $set(&self, v: u32) {
            // SAFETY: as above.
            unsafe {
                AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).control.$field))
                    .store(v, Ordering::Release)
            }
        }
    };
}

impl ShmSession {
    /// Create (or re-create) the shm object and initialize the
    /// header. Simulation-writer role: unlinks any stale object
    /// first (macOS refuses `ftruncate` on an existing one), zeroes
    /// the block, stamps the fingerprint, and sets
    /// `reset_generation = 1` and `plugin_ready = 1`.
    pub fn create(name: &str) -> io::Result<Self> {
        let cname = cstring(name)?;
        // SAFETY: plain libc calls on an owned CString; failure
        // paths close what was opened.
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
            // Header fingerprint last, ready flag after everything
            // else is in place.
            core::ptr::addr_of_mut!((*base).header.magic).write_volatile(MAGIC);
            core::ptr::addr_of_mut!((*base).header.layout_version).write_volatile(LAYOUT_VERSION);
            core::ptr::addr_of_mut!((*base).header.declared_size)
                .write_volatile(EXPECTED_SIZE as u32);
            core::ptr::addr_of_mut!((*base).header.reset_generation).write_volatile(1);
            AtomicU32::from_ptr(core::ptr::addr_of_mut!((*base).header.plugin_ready))
                .store(1, Ordering::Release);
            Ok(Self {
                base,
                name: cname,
                owner: true,
            })
        }
    }

    /// Attach to an existing object, failing closed on any contract
    /// mismatch (#262): missing object, short mapping (macOS rounds
    /// `st_size` up to the page, so only `actual < expected` fails),
    /// wrong magic, wrong layout version, wrong declared size.
    pub fn attach(name: &str) -> Result<Self, AttachFailure> {
        let cname = cstring(name).map_err(AttachFailure::Io)?;
        // SAFETY: plain libc calls; the block is only interpreted
        // after validate_attach passes on the fingerprint fields.
        unsafe {
            let fd = libc::shm_open(cname.as_ptr(), libc::O_RDWR, 0);
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
                return Err(AttachFailure::Io(io::Error::last_os_error()));
            }
            let base = ptr.cast::<SharedStateV2>();
            let magic = core::ptr::addr_of!((*base).header.magic).read_volatile();
            let version = core::ptr::addr_of!((*base).header.layout_version).read_volatile();
            let declared = core::ptr::addr_of!((*base).header.declared_size).read_volatile();
            if let Err(e) = validate_attach(magic, version, declared, actual) {
                libc::munmap(ptr, EXPECTED_SIZE);
                return Err(AttachFailure::Contract(e));
            }
            Ok(Self {
                base,
                name: cname,
                owner: false,
            })
        }
    }

    /// Simulation-world epoch (bumps on every world reset; 1 at
    /// creation). Consumers re-establish freshness tracking when
    /// this changes instead of quarantining the source (#265).
    pub fn reset_generation(&self) -> u32 {
        // SAFETY: naturally-aligned u32 in the validated mapping.
        unsafe {
            AtomicU32::from_ptr(core::ptr::addr_of_mut!(
                (*self.base).header.reset_generation
            ))
            .load(Ordering::Acquire)
        }
    }

    /// Bump the world epoch (simulation-writer role only).
    pub fn bump_reset_generation(&self) -> u32 {
        // SAFETY: as above.
        unsafe {
            AtomicU32::from_ptr(core::ptr::addr_of_mut!(
                (*self.base).header.reset_generation
            ))
            .fetch_add(1, Ordering::AcqRel)
            .wrapping_add(1)
        }
    }

    /// Whether the simulation writer currently owns the block.
    pub fn plugin_ready(&self) -> bool {
        // SAFETY: as above.
        unsafe {
            AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).header.plugin_ready))
                .load(Ordering::Acquire)
                != 0
        }
    }

    /// One coherent model-state snapshot, or `None` while no valid
    /// state exists or the writer kept the seqlock busy.
    pub fn read_model_state(&self) -> Option<ModelStateSnapshot> {
        // SAFETY: seq is a naturally-aligned u32; payload fields are
        // copied with volatile reads inside the seqlock window, so a
        // torn copy is discarded by the seq re-check.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).state.seq));
            let valid = seqlock_read(seq, || {
                core::ptr::addr_of!((*self.base).state.valid).read_volatile()
            })?;
            if valid == 0 {
                return None;
            }
            seqlock_read(seq, || ModelStateSnapshot {
                sim_step: core::ptr::addr_of!((*self.base).state.sim_step).read_volatile(),
                time_us: core::ptr::addr_of!((*self.base).state.time_us).read_volatile(),
                pos: core::ptr::addr_of!((*self.base).state.pos).read_volatile(),
                quat: core::ptr::addr_of!((*self.base).state.quat).read_volatile(),
                vel: core::ptr::addr_of!((*self.base).state.vel).read_volatile(),
                ang_vel: core::ptr::addr_of!((*self.base).state.ang_vel).read_volatile(),
            })
        }
    }

    /// Publish a model-state snapshot (simulation-writer role).
    pub fn write_model_state(&self, s: &ModelStateSnapshot) {
        // SAFETY: writer-side counterpart of read_model_state; the
        // seqlock write protocol makes concurrent readers discard
        // any torn view.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).state.seq));
            seqlock_write(seq, || {
                core::ptr::addr_of_mut!((*self.base).state.sim_step).write_volatile(s.sim_step);
                core::ptr::addr_of_mut!((*self.base).state.time_us).write_volatile(s.time_us);
                core::ptr::addr_of_mut!((*self.base).state.pos).write_volatile(s.pos);
                core::ptr::addr_of_mut!((*self.base).state.quat).write_volatile(s.quat);
                core::ptr::addr_of_mut!((*self.base).state.vel).write_volatile(s.vel);
                core::ptr::addr_of_mut!((*self.base).state.ang_vel).write_volatile(s.ang_vel);
                core::ptr::addr_of_mut!((*self.base).state.valid).write_volatile(1);
            });
        }
    }

    /// Publish motor boundary commands (FC role). Values are
    /// boundary rotor-speed commands — the actuator curve is applied
    /// BEFORE this call (#140).
    pub fn write_motor_command(&self, velocities: &[f64]) {
        let n = velocities.len().min(8);
        let mut lanes = [0.0_f64; 8];
        lanes[..n].copy_from_slice(&velocities[..n]);
        // SAFETY: command block writes under its own seqlock.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).command.seq));
            seqlock_write(seq, || {
                core::ptr::addr_of_mut!((*self.base).command.motor_vel).write_volatile(lanes);
                core::ptr::addr_of_mut!((*self.base).command.num_motors).write_volatile(n as u32);
            });
        }
    }

    /// One coherent motor-command snapshot: `(velocities, count)`
    /// (simulation-writer role).
    pub fn read_motor_command(&self) -> Option<([f64; 8], u32)> {
        // SAFETY: reader-side counterpart of write_motor_command.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).command.seq));
            seqlock_read(seq, || {
                (
                    core::ptr::addr_of!((*self.base).command.motor_vel).read_volatile(),
                    core::ptr::addr_of!((*self.base).command.num_motors).read_volatile(),
                )
            })
        }
    }

    /// Acknowledge a processed step (FC role): the lockstep gate the
    /// plugin blocks on, and the FC liveness heartbeat. Deliberately
    /// outside the command seqlock — the FC acks every step even on
    /// cycles that publish no new motor values.
    pub fn ack_step(&self, step: u64) {
        // SAFETY: naturally-aligned u64 written atomically.
        unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).command.fc_step_ack))
                .store(step, Ordering::Release)
        }
    }

    /// FC liveness / lockstep acknowledgement as last published.
    pub fn fc_step_ack(&self) -> u64 {
        // SAFETY: naturally-aligned u64 read atomically; the ack is
        // a monotonic heartbeat where a slightly stale value is
        // harmless.
        unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).command.fc_step_ack))
                .load(Ordering::Acquire)
        }
    }

    atomic_u32_accessor!(
        /// Ack nonce (set by the simulation writer once the action
        /// succeeded).
        lifecycle_ack_nonce,
        set_lifecycle_ack_nonce,
        lifecycle_ack_nonce
    );
    atomic_u32_accessor!(
        /// FC per-process nonce (consumers detect FC restarts).
        fc_session_nonce,
        set_fc_session_nonce,
        fc_session_nonce
    );
    atomic_u32_accessor!(
        /// Runtime lockstep toggle (#265).
        lockstep_enabled_raw,
        set_lockstep_enabled_raw,
        lockstep_enabled
    );
    atomic_u32_accessor!(
        /// Target real-time factor in percent (0 = unlimited).
        target_rtf_percent,
        set_target_rtf_percent,
        target_rtf_percent
    );

    /// The pending lifecycle request as one coherent `(nonce,
    /// request)` pair — a single packed atomic word, so no caller
    /// can pair a fresh nonce with a stale request (#267).
    pub fn lifecycle_request(&self) -> (u32, LifecycleRequest) {
        // SAFETY: naturally-aligned u64 in the validated mapping.
        let packed = unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!(
                (*self.base).control.lifecycle_request
            ))
            .load(Ordering::Acquire)
        };
        unpack_lifecycle_request(packed)
    }

    /// Post a lifecycle request as one packed atomic word and
    /// return its nonce. Single-requester protocol: completion (or
    /// duplication) is `lifecycle_ack_nonce == nonce`; the executor
    /// never re-runs an acked nonce, and nonce comparison is
    /// equality-based so wrapping is harmless.
    pub fn post_lifecycle_request(&self, req: LifecycleRequest) -> u32 {
        let (prev_nonce, _) = self.lifecycle_request();
        let nonce = prev_nonce.wrapping_add(1);
        // SAFETY: as above.
        unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!(
                (*self.base).control.lifecycle_request
            ))
            .store(pack_lifecycle_request(nonce, req), Ordering::Release);
        }
        nonce
    }

    /// The FC status as one coherent `(generation, state)` pair —
    /// a single packed atomic word, so `Ready` can never be
    /// observed with a stale generation (#267). `Ready` counts only
    /// when the generation equals [`Self::reset_generation`].
    pub fn fc_status(&self) -> (u32, FcState) {
        // SAFETY: as above.
        let packed = unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).control.fc_status))
                .load(Ordering::Acquire)
        };
        unpack_fc_status(packed)
    }

    /// Publish the FC state and the generation it refers to as one
    /// packed atomic word.
    pub fn set_fc_status(&self, state: FcState, generation: u32) {
        // SAFETY: as above.
        unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).control.fc_status))
                .store(pack_fc_status(generation, state), Ordering::Release);
        }
    }
}

impl Drop for ShmSession {
    fn drop(&mut self) {
        // SAFETY: base came from a successful EXPECTED_SIZE mmap and
        // is unmapped exactly once; only the creator unlinks the
        // name.
        unsafe {
            if self.owner {
                AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).header.plugin_ready))
                    .store(0, Ordering::Release);
            }
            libc::munmap(self.base.cast(), EXPECTED_SIZE);
            if self.owner {
                libc::shm_unlink(self.name.as_ptr());
            }
        }
    }
}

fn cstring(name: &str) -> io::Result<CString> {
    CString::new(name).map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in shm name"))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests;
