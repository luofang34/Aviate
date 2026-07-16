//! Role-specific endpoints over the mapped block. No unsafe here:
//! each role exposes exactly the methods its actor may perform, and
//! the consumer's mapping is additionally read-only at the OS level.

use aviate_xil_contract::{
    pack_fc_status, pack_lifecycle_request, unpack_fc_status, unpack_lifecycle_request, FcState,
    LifecycleRequest, WriterState,
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

        /// What the name behind this mapping resolves to right now.
        ///
        /// This is the whole staleness protocol in one call, and it
        /// cannot be a boolean: a writer that exited (`Gone`) and a
        /// writer that is still the same one (`Current`) are
        /// opposite conclusions that a `replaced: bool` collapses
        /// into the same `false`, leaving an orphaned mapping
        /// looking healthy forever.
        ///
        /// * [`WriterState::Current`] — reads are trustworthy.
        /// * [`WriterState::Replaced`] / [`WriterState::Gone`] —
        ///   re-attach; this mapping only serves a dead world.
        /// * [`WriterState::Initializing`] — retry shortly.
        /// * [`WriterState::ContractMismatch`] — fail closed.
        pub fn writer_state(&self) -> WriterState {
            self.mapping.writer_state()
        }

        /// One coherent `{generation, step, time, state}` snapshot.
        ///
        /// Generation authority: the HEADER `reset_generation` is
        /// the epoch of record, and every snapshot carries a copy
        /// inside its seqlock payload. This read serves a snapshot
        /// only when the two agree — a pose whose payload generation
        /// trails the header belongs to a world that no longer
        /// exists and is answered with `None`, exactly like an
        /// unpublished or retired snapshot. Consumers therefore
        /// never need to cross-check the pair themselves; comparing
        /// a returned snapshot's `reset_generation` against their
        /// OWN last-seen value tells them a reset happened, not
        /// whether the data is trustworthy.
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

        /// FC session nonce — watchers detect an FC restart or
        /// re-attach by a change here even though the shm object
        /// identity is unchanged. Owned by the FC endpoint:
        /// [`FcSession::attach`] stamps `previous + 1` (wrapping,
        /// zero skipped — zero means no FC has ever attached to
        /// this object). Compare for equality only; the value
        /// wraps.
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
    ///
    /// This is one half of the identity contract, and the two halves
    /// must not be confused:
    ///
    /// * **Writer (re)start — THIS call.** A new shm object with a
    ///   fresh `writer_incarnation`. Consumers attached to the
    ///   previous object observe [`WriterState::Replaced`] and must
    ///   re-attach; the orphaned mapping can only ever serve the
    ///   dead world's final snapshot.
    /// * **World reset — [`Self::bump_reset_generation`].** The SAME
    ///   object, epoch bumped in place. Consumers keep their
    ///   attachment (`writer_state()` stays
    ///   [`WriterState::Current`]) and re-key on the new generation.
    ///
    /// Creation also takes the writer lease first: if a live writer
    /// holds this name, creation fails with `WouldBlock` instead of
    /// unlinking the peer's object out from under its consumers.
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
    ///
    /// The other half of the identity contract stated on
    /// [`Self::create`]: a reset happens IN PLACE. The object, its
    /// `writer_incarnation`, and every consumer's attachment all
    /// survive — `writer_state()` stays [`WriterState::Current`] —
    /// and consumers re-key their freshness tracking on the new
    /// generation instead of re-attaching.
    ///
    /// Retires the outgoing snapshot in the same act: the previous
    /// epoch's pose stays in the block until the new world publishes
    /// its first step, and it is valid and coherent — just from a
    /// world that no longer exists.
    pub fn bump_reset_generation(&self) -> u32 {
        let generation = self.mapping.bump_reset_generation();
        self.mapping.invalidate_model_state(generation);
        generation
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
    ///
    /// An attachment IS a session: attaching stamps the next
    /// `fc_session_nonce` — the previous value advanced by
    /// `wrapping_add(1)` with zero skipped, since zero is reserved
    /// for "no FC has ever attached". The FC endpoint is the only
    /// writer of the word, and stamping here (rather than trusting
    /// each binary to remember) is what lets a watcher tell "the
    /// same FC, still alive" from "an FC re-attached behind my
    /// back". Watchers compare for equality only: the counter
    /// wraps, and its ordering carries no meaning across a writer
    /// restart (a fresh object resets it to zero).
    pub fn attach(name: &str) -> Result<Self, AttachFailure> {
        let session = Self {
            mapping: Mapping::attach(name, false)?,
        };
        let mut next = session.mapping.fc_session_nonce().wrapping_add(1);
        if next == 0 {
            next = 1;
        }
        session.mapping.set_fc_session_nonce(next);
        Ok(session)
    }

    shared_readers!();

    /// Publish motor boundary commands (rotor speed, rad/s — the
    /// actuator curve is applied BEFORE this call).
    pub fn write_motor_command(&self, velocities: &[f64]) {
        self.mapping.write_motor_command(velocities);
    }

    /// Acknowledge a processed step: heartbeat + lockstep gate.
    ///
    /// One gate, one acker: when lockstep is armed the simulator
    /// blocks each physics step on this word, and exactly one
    /// session — whichever drives the lockstep session — may write
    /// it. A second acker races the owner and re-opens the gate
    /// while the owner is still processing. Endpoints that merely
    /// consume steps heartbeat this only in free-run.
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

    /// Toggle lockstep at runtime.
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
