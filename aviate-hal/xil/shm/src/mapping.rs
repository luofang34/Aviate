//! The raw mapping: creation, fail-closed attach, and every
//! volatile/atomic access primitive. ALL unsafe code in the SITL
//! shm data plane lives in this module; `roles.rs` composes these
//! safe methods into role-specific endpoints without any unsafe.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::ffi::CString;

use aviate_xil_contract::{seqlock_read, seqlock_write, SharedStateV2, WriterState, EXPECTED_SIZE};

mod attach;
mod lanes;
pub(crate) mod lease;

pub use attach::AttachFailure;
use attach::{confirm_alive, writer_state};

use lanes::{load_f64_lanes, load_u32, load_u64, store_f64_lanes, store_u32, store_u64};

/// One coherent `{generation, step, time, state}` snapshot taken
/// under the model seqlock (`sim_step`/`time_us` are the
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
    /// Angular velocity [rad/s], world ENU (gz's
    /// `WorldAngularVelocity` verbatim, NOT a body gyro; known
    /// unreliable — see the contract's `ModelStateBlock::ang_vel`).
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
    /// Held for the writer's whole life; `None` for attachers. The
    /// kernel releases it on any exit, so its held/free state is the
    /// liveness signal `writer_liveness` probes.
    lease: Option<lease::WriterLease>,
    /// The `writer_incarnation` of the object this mapping was taken
    /// from. A writer that dies and re-creates the object leaves
    /// this mapping pointing at the dead object's (still-mapped)
    /// memory, whose last snapshot looks valid forever;
    /// [`Mapping::writer_state`] re-reads the live name's
    /// incarnation and compares, so a consumer can re-attach instead
    /// of serving a frozen world.
    incarnation: u64,
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

    /// Retire the current snapshot: publish `valid = 0` through the
    /// state seqlock so no reader can consume the previous epoch's
    /// pose while the new world spins up. Simulation-writer role.
    pub(crate) fn invalidate_model_state(&self, generation: u32) {
        // SAFETY: writer-side seqlock publish, same as
        // write_model_state.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).state.seq));
            seqlock_write(seq, || {
                store_u32(core::ptr::addr_of_mut!((*self.base).state.valid), 0);
                store_u32(
                    core::ptr::addr_of_mut!((*self.base).state.reset_generation),
                    generation,
                );
            });
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

    /// One coherent model snapshot, or `None` when the writer is
    /// gone, no valid state exists yet, or the writer kept the
    /// seqlock busy for the whole retry budget.
    pub(crate) fn read_model_state(&self) -> Option<ModelStateSnapshot> {
        // A writer that dropped or is mid-restart must not keep
        // feeding its last snapshot forever: the payload would look
        // perfectly valid and perfectly still. Consumers combine this
        // with `writer_state()` for the crash-and-recreate case.
        if !self.plugin_ready() {
            return None;
        }
        // SAFETY: seq is a naturally-aligned u32; every payload lane
        // is read atomically inside the seqlock window, so a torn
        // copy is discarded by the seq re-check rather than being a
        // data race.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).state.seq));
            let (valid, snapshot) = seqlock_read(seq, || {
                (
                    load_u32(core::ptr::addr_of!((*self.base).state.valid)),
                    ModelStateSnapshot {
                        reset_generation: load_u32(core::ptr::addr_of!(
                            (*self.base).state.reset_generation
                        )),
                        sim_step: load_u64(core::ptr::addr_of!((*self.base).state.sim_step)),
                        time_us: load_u64(core::ptr::addr_of!((*self.base).state.time_us)),
                        pos: load_f64_lanes(core::ptr::addr_of!((*self.base).state.pos_bits)),
                        quat: load_f64_lanes(core::ptr::addr_of!((*self.base).state.quat_bits)),
                        vel: load_f64_lanes(core::ptr::addr_of!((*self.base).state.vel_bits)),
                        ang_vel: load_f64_lanes(core::ptr::addr_of!(
                            (*self.base).state.ang_vel_bits
                        )),
                    },
                )
            })?;
            if valid == 0 {
                return None;
            }
            // Double-check the epoch. The snapshot carries the
            // generation it was published under; the header carries
            // the generation the world is in NOW. Between a reset
            // bumping the header and the next publish landing, the
            // block still holds the PREVIOUS epoch's pose — valid,
            // coherent, and from a world that no longer exists.
            // Serving it would teleport a consumer back into the
            // pre-reset flight.
            if snapshot.reset_generation != self.reset_generation() {
                return None;
            }
            Some(snapshot)
        }
    }

    /// Whether the shm object this mapping was taken from has been
    /// replaced — the writer crashed (leaving `plugin_ready` set in
    /// the orphaned mapping) and created a fresh object under the
    /// same name. The consumer must re-attach; this mapping can only
    /// ever serve the dead world's last snapshot.
    pub(crate) fn writer_state(&self) -> WriterState {
        confirm_alive(&self.name, writer_state(&self.name, self.incarnation))
    }

    /// The incarnation this mapping was stamped with or attached to.
    #[cfg(test)]
    pub(crate) fn incarnation(&self) -> u64 {
        self.incarnation
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
                store_u32(
                    core::ptr::addr_of_mut!((*self.base).state.reset_generation),
                    s.reset_generation,
                );
                store_u64(
                    core::ptr::addr_of_mut!((*self.base).state.sim_step),
                    s.sim_step,
                );
                store_u64(
                    core::ptr::addr_of_mut!((*self.base).state.time_us),
                    s.time_us,
                );
                store_f64_lanes(core::ptr::addr_of_mut!((*self.base).state.pos_bits), &s.pos);
                store_f64_lanes(
                    core::ptr::addr_of_mut!((*self.base).state.quat_bits),
                    &s.quat,
                );
                store_f64_lanes(core::ptr::addr_of_mut!((*self.base).state.vel_bits), &s.vel);
                store_f64_lanes(
                    core::ptr::addr_of_mut!((*self.base).state.ang_vel_bits),
                    &s.ang_vel,
                );
                store_u32(core::ptr::addr_of_mut!((*self.base).state.valid), 1);
            });
        }
    }

    /// Publish motor boundary commands (FC role). Values are
    /// boundary rotor-speed commands — the actuator curve is applied
    /// BEFORE this call.
    pub(crate) fn write_motor_command(&self, velocities: &[f64]) {
        let n = velocities.len().min(8);
        let mut lanes = [0.0_f64; 8];
        lanes[..n].copy_from_slice(&velocities[..n]);
        // SAFETY: command block writes under its own seqlock.
        unsafe {
            let seq = AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).command.seq));
            seqlock_write(seq, || {
                store_f64_lanes(
                    core::ptr::addr_of_mut!((*self.base).command.motor_vel_bits),
                    &lanes,
                );
                store_u32(
                    core::ptr::addr_of_mut!((*self.base).command.num_motors),
                    n as u32,
                );
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
                    load_f64_lanes(core::ptr::addr_of!((*self.base).command.motor_vel_bits)),
                    load_u32(core::ptr::addr_of!((*self.base).command.num_motors)),
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
        /// Runtime lockstep toggle.
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
        // ready flag and unlinks the name — and only after
        // re-resolving the NAME and finding its own object there.
        // Reading the incarnation through `self.base` could not
        // check that: our own mapping always answers with the value
        // we stamped, replaced or not.
        unsafe {
            if self.owner {
                // `Initializing` at owner-drop can only be our own
                // never-readied object (the mid-init test helper):
                // while we hold the lease no successor can exist,
                // and a foreign protocol writer would have needed
                // the lease too.
                let ours = matches!(
                    writer_state(&self.name, self.incarnation),
                    WriterState::Current | WriterState::Initializing
                );
                AtomicU32::from_ptr(core::ptr::addr_of_mut!((*self.base).header.plugin_ready))
                    .store(0, Ordering::Release);
                libc::munmap(self.base.cast(), EXPECTED_SIZE);
                if ours {
                    libc::shm_unlink(self.name.as_ptr());
                }
                // Release the lease only after the unlink: freeing
                // it earlier would let a new writer win the lease
                // while the name still resolves to our dying object.
                drop(self.lease.take());
            } else {
                libc::munmap(self.base.cast(), EXPECTED_SIZE);
            }
        }
    }
}
