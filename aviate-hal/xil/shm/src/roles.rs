//! Role-specific endpoints over the mapped block. No unsafe here:
//! each role exposes exactly the methods its actor may perform, and
//! the consumer's mapping is additionally read-only at the OS level.

use aviate_xil_contract::{
    pack_fc_status, pack_lifecycle_request, unpack_fc_status, unpack_lifecycle_request, FcState,
    LifecycleRequest,
};

use crate::mapping::{AttachFailure, Mapping, ModelStateSnapshot};

macro_rules! shared_readers {
    () => {
        /// Simulation-world epoch (header authority; every snapshot
        /// carries the same value inside its seqlock payload).
        pub fn reset_generation(&self) -> u32 {
            self.mapping.reset_generation()
        }

        /// Whether the simulation writer currently owns the block.
        pub fn plugin_ready(&self) -> bool {
            self.mapping.plugin_ready()
        }

        /// Whether the shm object behind this mapping has been
        /// REPLACED — the writer crashed (so `plugin_ready` stayed
        /// set in the now-orphaned memory) and created a fresh
        /// object under the same name. This mapping can then only
        /// serve the dead world's final snapshot forever, so the
        /// consumer must re-attach.
        ///
        /// The full staleness protocol for a consumer is: stop
        /// trusting the source when `plugin_ready()` goes false
        /// (clean writer exit — `read_model_state` already returns
        /// `None`), and re-attach when this returns true (writer
        /// crash + recreate).
        pub fn writer_replaced(&self) -> bool {
            self.mapping.writer_replaced()
        }

        /// One coherent `{generation, step, time, state}` snapshot.
        pub fn read_model_state(&self) -> Option<ModelStateSnapshot> {
            self.mapping.read_model_state()
        }

        /// FC liveness / lockstep acknowledgement as last published.
        pub fn fc_step_ack(&self) -> u64 {
            self.mapping.fc_step_ack()
        }

        /// The pending lifecycle request as one coherent
        /// `(nonce, request)` pair (single packed atomic word).
        pub fn lifecycle_request(&self) -> (u32, LifecycleRequest) {
            unpack_lifecycle_request(self.mapping.lifecycle_request_packed())
        }

        /// Last lifecycle request nonce the simulation writer
        /// acknowledged (ack means the action SUCCEEDED).
        pub fn lifecycle_ack_nonce(&self) -> u32 {
            self.mapping.lifecycle_ack_nonce()
        }

        /// FC status as one coherent `(generation, state)` pair.
        /// `Ready` counts only when the generation equals
        /// [`Self::reset_generation`].
        pub fn fc_status(&self) -> (u32, FcState) {
            unpack_fc_status(self.mapping.fc_status_packed())
        }

        /// FC per-process nonce — consumers detect an FC restart by
        /// a change here even though the shm object identity is
        /// unchanged.
        pub fn fc_session_nonce(&self) -> u32 {
            self.mapping.fc_session_nonce()
        }

        /// Whether lockstep is currently requested.
        pub fn lockstep_enabled(&self) -> bool {
            self.mapping.lockstep_enabled_raw() != 0
        }

        /// Target real-time factor in percent (0 = unlimited).
        pub fn target_rtf_percent(&self) -> u32 {
            self.mapping.target_rtf_percent()
        }
    };
}

/// Simulation-side writer: creates and owns the block. In production
/// this actor is the C++ gz plugin over the identical layout; the
/// Rust writer serves headless harnesses and tests.
#[derive(Debug)]
pub struct SimWriterSession {
    mapping: Mapping,
}

impl SimWriterSession {
    /// Create (or re-create) the block and publish readiness.
    pub fn create(name: &str) -> std::io::Result<Self> {
        Ok(Self {
            mapping: Mapping::create(name)?,
        })
    }

    /// Create the block WITHOUT stamping the fingerprint or
    /// publishing readiness — a writer frozen mid-initialisation.
    /// Test-only; see [`Mapping::create_mid_init_for_test`].
    #[cfg(test)]
    pub(crate) fn create_mid_init_for_test(name: &str) -> std::io::Result<Self> {
        Ok(Self {
            mapping: Mapping::create_mid_init_for_test(name)?,
        })
    }

    shared_readers!();

    /// Publish one model snapshot under the seqlock.
    pub fn write_model_state(&self, s: &ModelStateSnapshot) {
        self.mapping.write_model_state(s);
    }

    /// Bump the world epoch (world reset) and return the new value.
    pub fn bump_reset_generation(&self) -> u32 {
        self.mapping.bump_reset_generation()
    }

    /// One coherent motor-command snapshot: `(velocities, count)`.
    pub fn read_motor_command(&self) -> Option<([f64; 8], u32)> {
        self.mapping.read_motor_command()
    }

    /// Acknowledge a lifecycle request nonce AFTER the requested
    /// action succeeded — never before.
    pub fn set_lifecycle_ack_nonce(&self, nonce: u32) {
        self.mapping.set_lifecycle_ack_nonce(nonce);
    }
}

/// Flight-controller endpoint: motor commands, step acks, FC status.
#[derive(Debug)]
pub struct FcSession {
    mapping: Mapping,
}

impl FcSession {
    /// Attach read-write; fails closed on fingerprint or readiness.
    pub fn attach(name: &str) -> Result<Self, AttachFailure> {
        Ok(Self {
            mapping: Mapping::attach(name, false)?,
        })
    }

    shared_readers!();

    /// Publish motor boundary commands (rotor speed, rad/s — the
    /// actuator curve is applied BEFORE this call, #140).
    pub fn write_motor_command(&self, velocities: &[f64]) {
        self.mapping.write_motor_command(velocities);
    }

    /// Acknowledge a processed step: heartbeat + lockstep gate.
    pub fn ack_step(&self, step: u64) {
        self.mapping.ack_step(step);
    }

    /// Publish the FC state and the generation it refers to as one
    /// packed atomic word — `Ready` can never pair with a stale
    /// generation.
    pub fn set_fc_status(&self, state: FcState, generation: u32) {
        self.mapping
            .set_fc_status_packed(pack_fc_status(generation, state));
    }

    /// Stamp this FC process's session nonce (once at startup).
    pub fn set_fc_session_nonce(&self, nonce: u32) {
        self.mapping.set_fc_session_nonce(nonce);
    }
}

/// Session-host / test-harness endpoint: lifecycle requests and
/// runtime time controls. Never touches model state or motor
/// commands.
#[derive(Debug)]
pub struct HostSession {
    mapping: Mapping,
}

impl HostSession {
    /// Attach read-write; fails closed on fingerprint or readiness.
    pub fn attach(name: &str) -> Result<Self, AttachFailure> {
        Ok(Self {
            mapping: Mapping::attach(name, false)?,
        })
    }

    shared_readers!();

    /// Post a lifecycle request as one packed atomic word and
    /// return its nonce. Single-requester lane: completion (or
    /// duplication) is `lifecycle_ack_nonce == nonce`; comparison is
    /// equality-based, so nonce wrapping is harmless.
    pub fn post_lifecycle_request(&self, req: LifecycleRequest) -> u32 {
        let (prev_nonce, _) = self.lifecycle_request();
        let nonce = prev_nonce.wrapping_add(1);
        self.mapping
            .set_lifecycle_request_packed(pack_lifecycle_request(nonce, req));
        nonce
    }

    /// Toggle lockstep at runtime (#265).
    pub fn set_lockstep(&self, enabled: bool) {
        self.mapping
            .set_lockstep_enabled_raw(if enabled { 1 } else { 0 });
    }

    /// Request a real-time factor (percent: 100 = 1×, 400 = 4×,
    /// 0 = as-fast-as-possible).
    pub fn set_target_rtf_percent(&self, percent: u32) {
        self.mapping.set_target_rtf_percent(percent);
    }
}

/// Read-only telemetry observer. The mapping itself is `PROT_READ`
/// over an `O_RDONLY` descriptor: even unsafe code behind this
/// endpoint could not write the block without faulting.
#[derive(Debug)]
pub struct ConsumerSession {
    mapping: Mapping,
}

impl ConsumerSession {
    /// Attach read-only; fails closed on fingerprint or readiness.
    pub fn attach(name: &str) -> Result<Self, AttachFailure> {
        Ok(Self {
            mapping: Mapping::attach(name, true)?,
        })
    }

    shared_readers!();
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#[path = "roles/tests.rs"]
mod tests;
