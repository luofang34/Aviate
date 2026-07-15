//! The raw mapping: creation, fail-closed attach, and every
//! volatile/atomic access primitive. ALL unsafe code in the SITL
//! shm data plane lives in this module; `roles.rs` composes these
//! safe methods into role-specific endpoints without any unsafe.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::ffi::CString;
use std::io;

use aviate_xil_contract::{
    seqlock_read, seqlock_write, validate_attach, AttachError, SharedStateV2, EXPECTED_SIZE,
    LAYOUT_VERSION, MAGIC,
};

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

/// One coherent `{generation, step, time, state}` snapshot taken
/// under the model seqlock (#265: `sim_step`/`time_us` are the
/// sim-time authority; the generation rides inside the same read so
/// a snapshot can never be attributed to the wrong epoch).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ModelStateSnapshot {
    /// Simulation-world epoch this snapshot belongs to.
    pub reset_generation: u32,
    /// Physics step counter (monotonic across resets).
    pub sim_step: u64,
    /// Simulation time (µs); rewinds on world reset.
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

/// A validated mapping of the contract block. Construction chooses
/// the protection; the role wrappers in `roles.rs` decide which
/// methods are reachable.
#[derive(Debug)]
pub(crate) struct Mapping {
    base: *mut SharedStateV2,
    name: CString,
    /// Creator unlinks the object on drop; attachers never do.
    owner: bool,
}

// SAFETY: the mapping is process-shared memory accessed only through
// the atomic/volatile operations below; moving it between threads
// moves only the pointer and ownership flag.
unsafe impl Send for Mapping {}

macro_rules! control_u32 {
    ($(#[$doc:meta])* $get:ident, $set:ident, $field:ident) => {
        $(#[$doc])*
        pub(crate) fn $get(&self) -> u32 {
            // SAFETY: naturally-aligned u32 inside the validated
            // mapping; AtomicU32 has the same layout.
            unsafe {
                AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).control.$field))
                    .load(Ordering::Acquire)
            }
        }
        /// Atomically store the field (see the getter's docs).
        pub(crate) fn $set(&self, v: u32) {
            // SAFETY: as above.
            unsafe {
                AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).control.$field))
                    .store(v, Ordering::Release)
            }
        }
    };
}

macro_rules! control_u64 {
    ($(#[$doc:meta])* $get:ident, $set:ident, $field:ident) => {
        $(#[$doc])*
        pub(crate) fn $get(&self) -> u64 {
            // SAFETY: naturally-aligned u64 inside the validated
            // mapping; AtomicU64 has the same layout.
            unsafe {
                AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).control.$field))
                    .load(Ordering::Acquire)
            }
        }
        /// Atomically store the field (see the getter's docs).
        pub(crate) fn $set(&self, v: u64) {
            // SAFETY: as above.
            unsafe {
                AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).control.$field))
                    .store(v, Ordering::Release)
            }
        }
    };
}

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
            let magic = core::ptr::addr_of!((*base).header.magic).read_volatile();
            let version = core::ptr::addr_of!((*base).header.layout_version).read_volatile();
            let declared = core::ptr::addr_of!((*base).header.declared_size).read_volatile();
            if let Err(e) = validate_attach(magic, version, declared, actual) {
                libc::munmap(ptr, EXPECTED_SIZE);
                return Err(AttachFailure::Contract(e));
            }
            let ready = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*base).header.plugin_ready))
                .load(Ordering::Acquire);
            if ready == 0 {
                libc::munmap(ptr, EXPECTED_SIZE);
                return Err(AttachFailure::NotReady);
            }
            Ok(Self {
                base,
                name: cname,
                owner: false,
            })
        }
    }

    /// Simulation-world epoch (header authority; the same value
    /// rides inside every model snapshot).
    pub(crate) fn reset_generation(&self) -> u32 {
        // SAFETY: naturally-aligned u32 in the validated mapping.
        unsafe {
            AtomicU32::from_ptr(core::ptr::addr_of_mut!(
                (*self.base).header.reset_generation
            ))
            .load(Ordering::Acquire)
        }
    }

    /// Bump the world epoch (simulation-writer role only).
    pub(crate) fn bump_reset_generation(&self) -> u32 {
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
    pub(crate) fn plugin_ready(&self) -> bool {
        // SAFETY: as above.
        unsafe {
            AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).header.plugin_ready))
                .load(Ordering::Acquire)
                != 0
        }
    }

    /// One coherent model snapshot, or `None` while no valid state
    /// exists or the writer kept the seqlock busy.
    pub(crate) fn read_model_state(&self) -> Option<ModelStateSnapshot> {
        // SAFETY: seq is a naturally-aligned u32; payload fields are
        // copied with volatile reads inside the seqlock window, so a
        // torn copy is discarded by the seq re-check.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).state.seq));
            let snap = seqlock_read(seq, || {
                (
                    core::ptr::addr_of!((*self.base).state.valid).read_volatile(),
                    ModelStateSnapshot {
                        reset_generation: core::ptr::addr_of!((*self.base).state.reset_generation)
                            .read_volatile(),
                        sim_step: core::ptr::addr_of!((*self.base).state.sim_step).read_volatile(),
                        time_us: core::ptr::addr_of!((*self.base).state.time_us).read_volatile(),
                        pos: core::ptr::addr_of!((*self.base).state.pos).read_volatile(),
                        quat: core::ptr::addr_of!((*self.base).state.quat).read_volatile(),
                        vel: core::ptr::addr_of!((*self.base).state.vel).read_volatile(),
                        ang_vel: core::ptr::addr_of!((*self.base).state.ang_vel).read_volatile(),
                    },
                )
            })?;
            let (valid, snapshot) = snap;
            if valid == 0 {
                return None;
            }
            Some(snapshot)
        }
    }

    /// Publish a model snapshot (simulation-writer role). The
    /// snapshot's `reset_generation` should mirror the header's
    /// current epoch.
    pub(crate) fn write_model_state(&self, s: &ModelStateSnapshot) {
        // SAFETY: writer-side counterpart of read_model_state; the
        // seqlock write protocol makes concurrent readers discard
        // any torn view.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).state.seq));
            seqlock_write(seq, || {
                core::ptr::addr_of_mut!((*self.base).state.reset_generation)
                    .write_volatile(s.reset_generation);
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
    pub(crate) fn write_motor_command(&self, velocities: &[f64]) {
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
    pub(crate) fn read_motor_command(&self) -> Option<([f64; 8], u32)> {
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

    /// Acknowledge a processed step (FC role): a bare aligned atomic
    /// outside the command seqlock — the FC acks every step even on
    /// cycles that publish no new motor values.
    pub(crate) fn ack_step(&self, step: u64) {
        // SAFETY: naturally-aligned u64 written atomically.
        unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).command.fc_step_ack))
                .store(step, Ordering::Release)
        }
    }

    /// FC liveness / lockstep acknowledgement as last published.
    pub(crate) fn fc_step_ack(&self) -> u64 {
        // SAFETY: naturally-aligned u64 read atomically; the ack is
        // a monotonic heartbeat where a slightly stale value is
        // harmless.
        unsafe {
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*self.base).command.fc_step_ack))
                .load(Ordering::Acquire)
        }
    }

    control_u64!(
        /// Packed lifecycle request word (hi = nonce, lo = request).
        lifecycle_request_packed,
        set_lifecycle_request_packed,
        lifecycle_request
    );
    control_u64!(
        /// Packed FC status word (hi = generation, lo = state).
        fc_status_packed,
        set_fc_status_packed,
        fc_status
    );
    control_u32!(
        /// Ack nonce (set by the simulation writer on success).
        lifecycle_ack_nonce,
        set_lifecycle_ack_nonce,
        lifecycle_ack_nonce
    );
    control_u32!(
        /// FC per-process nonce (consumers detect FC restarts).
        fc_session_nonce,
        set_fc_session_nonce,
        fc_session_nonce
    );
    control_u32!(
        /// Runtime lockstep toggle (#265).
        lockstep_enabled_raw,
        set_lockstep_enabled_raw,
        lockstep_enabled
    );
    control_u32!(
        /// Target real-time factor in percent (0 = unlimited).
        target_rtf_percent,
        set_target_rtf_percent,
        target_rtf_percent
    );
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: base came from a successful EXPECTED_SIZE mmap and
        // is unmapped exactly once; only the creator clears the
        // ready flag and unlinks the name.
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
