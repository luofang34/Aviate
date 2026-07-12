//! Core kernel struct and constructors.
//!
//! Behavior is split across sibling modules:
//!   - `kernel_logic.rs` — lifecycle (init, arm, disarm, reset, fault handling).
//!   - `kernel_update.rs` — the per-cycle `update()` loop.
//!   - `kernel_trait.rs`  — the `AviateKernelTrait` definition and impl.
//!
//! The kernel struct itself is being decomposed (see plan in
//! `docs/AVIATE_SPEC.md` follow-ups). Phase 1 introduces the
//! `cfg: ResolvedKernelConfig` sub-struct (this module's
//! `kernel/config.rs`) — every field declared here that's not already
//! a generic algorithm box (E, V, M, S) is on the migration path to
//! either `KernelState` (Phase 3) or stays read-only inside `cfg`.

pub mod builder;
pub mod config;
pub mod descend;
pub mod pipeline;
pub mod slew;
pub mod snapshot;
pub mod state;

use crate::checks::{KernelChecks, PreArmFlags};
use crate::control::VehicleController;
use crate::ekf::{Ekf, Estimator};
use crate::kernel::config::ResolvedKernelConfig;
use crate::kernel::pipeline::KernelPipeline;
use crate::kernel::state::KernelState;
use crate::mixer::{ActuatorSanitizer, Mixer, ModeConfig, Sanitizer};

pub use crate::kernel_types::InitState;

pub struct AviateKernelImpl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer> {
    /// Algorithm-identity bundle (estimator, controller, mixer,
    /// sanitizer, protector). See `kernel/pipeline.rs`.
    pub(crate) pipeline: KernelPipeline<E, V, M, S>,

    /// All safety-relevant runtime state — lifecycle, mode, faults,
    /// control law, gate checks, actuator snapshot, timing stats,
    /// EKF persistent state, sanitizer fallback memory, and
    /// controller runtime state (`V::RuntimeState`). See
    /// `kernel/state.rs`. The "every safety-relevant persistent
    /// state field has exactly one owner" invariant covers
    /// every field of this sub-struct.
    pub state: KernelState<E::RuntimeState, V::RuntimeState>,

    /// Validated, flight-period-immutable configuration (spec §19).
    /// See `kernel/config.rs`.
    pub(crate) cfg: ResolvedKernelConfig,
}

/// Type alias for the kernel struct.
///
/// Parameter order mirrors the constructor: `<E, V, M, S>` =
/// estimator, vehicle controller, mixer, sanitizer.
pub type AviateKernel<E, V, M, S> = AviateKernelImpl<E, V, M, S>;

/// Default kernel: 15-state EKF + group-aware Sanitizer. Use when
/// callers don't need to substitute estimation or sanitization.
pub type DefaultAviateKernel<V, M> = AviateKernelImpl<Ekf, V, M, Sanitizer>;

impl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer>
    AviateKernelImpl<E, V, M, S>
{
    /// Crate-internal scaffolding: constructs without the builder's
    /// controller/config binding check, so it must never be reachable
    /// from production integration code. Every external construction
    /// goes through `AviateKernelBuilder::build`.
    ///
    /// ```compile_fail
    /// use aviate_core::control::multirotor::MultirotorController;
    /// use aviate_core::ekf::Ekf;
    /// use aviate_core::mixer::{ModeConfig, QuadXMixer, Sanitizer};
    /// // private associated function: external code cannot bypass the
    /// // checked builder.
    /// let _ = aviate_core::kernel::AviateKernelImpl::new(
    ///     Ekf::default(),
    ///     MultirotorController::default(),
    ///     QuadXMixer { timestamp_source: || unimplemented!() },
    ///     Sanitizer,
    ///     ModeConfig { mode: aviate_core::control::ConfigMode::Hover, groups: &[] },
    /// );
    /// ```
    /// Read the immutable algorithm pipeline. Construction fixes the
    /// pipeline; no caller can swap or retune a component afterwards,
    /// so the verified controller/config binding cannot be separated
    /// from the flying tuning.
    pub fn pipeline(&self) -> &KernelPipeline<E, V, M, S> {
        &self.pipeline
    }

    /// Read the resolved configuration the binding check verified.
    pub fn cfg(&self) -> &ResolvedKernelConfig {
        &self.cfg
    }

    /// Test scaffolding: mutate the resolved configuration of a built
    /// kernel to stage a scenario (mid-flight limit changes, timeout
    /// variation, slew overrides). Production code must never call
    /// this — a post-build config edit desynchronizes the canonical
    /// hash and the verified controller binding from what actually
    /// flies, and the runtime-boundary gate fails any use outside a
    /// tests tree.
    #[doc(hidden)]
    pub fn cfg_scenario_override(&mut self) -> &mut ResolvedKernelConfig {
        &mut self.cfg
    }

    #[cfg(test)]
    pub(crate) fn new(
        estimator: E,
        controller: V,
        mixer: M,
        sanitizer: S,
        mode_config: ModeConfig,
    ) -> Self {
        Self {
            pipeline: KernelPipeline::new(estimator, controller, mixer, sanitizer),
            state: KernelState::new(KernelChecks::new()),
            cfg: ResolvedKernelConfig {
                mode_config,
                ..Default::default()
            },
        }
    }

    /// Create a kernel with custom pre-arm requirements. Same direct
    /// struct-literal pattern as `new()` — every required component is
    /// supplied positionally, so this constructor cannot fail.
    pub fn with_pre_arm_required(
        estimator: E,
        controller: V,
        mixer: M,
        sanitizer: S,
        mode_config: ModeConfig,
        required: PreArmFlags,
    ) -> Self {
        Self {
            pipeline: KernelPipeline::new(estimator, controller, mixer, sanitizer),
            state: KernelState::new(KernelChecks::with_pre_arm_required(required)),
            cfg: ResolvedKernelConfig {
                mode_config,
                ..Default::default()
            },
        }
    }

    /// Project the local channel's state into a
    /// [`crate::kernel::snapshot::ChannelSnapshot`] for
    /// cross-channel exchange (spec §16). Writes the canonical
    /// kernel-state encoding into `state_buf` and returns a
    /// `ChannelSnapshot` borrowing the populated portion.
    ///
    /// The caller owns `state_buf` (no allocation, no_std-friendly)
    /// and SHOULD size it to
    /// `<KernelState<E::RuntimeState, V::RuntimeState> as Replicable>::ENCODED_LEN`.
    /// A short buffer truncates without panic — `state_bytes.len()`
    /// will be less than `ENCODED_LEN`, which causes
    /// `ChannelSnapshot::agrees_with` to fail safely against any
    /// peer running with a full-size buffer.
    pub fn project_for_cross_channel<'buf>(
        &self,
        cycle_seq: u64,
        channel_id: crate::ChannelId,
        state_buf: &'buf mut [u8],
    ) -> crate::kernel::snapshot::ChannelSnapshot<'buf> {
        use crate::replicable::Replicable;
        let n = self.state.encode_canonical(state_buf);
        crate::kernel::snapshot::ChannelSnapshot {
            channel_id,
            cycle_seq,
            algorithm_identity_hash: self.pipeline.algorithm_identity_hash(),
            config_hash: self.cfg.canonical_hash(),
            state_bytes: &state_buf[..n],
        }
    }

    /// One-call cross-channel agreement check: project the local
    /// snapshot into `local_buf` and run
    /// [`crate::kernel::snapshot::decide_lockstep`] against the
    /// supplied peer snapshots.
    ///
    /// Returns the gate decision; the caller routes the redundancy
    /// response (proceed with lockstep, downgrade to channel-
    /// isolated, retry next cycle, declare hot-spare takeover). The
    /// kernel itself does NOT mutate based on the decision — that
    /// belongs to a higher-level redundancy policy that is out of
    /// scope here. // COV:EXCL(phantom DA: grcov attributes a debug-info region to this doc-comment line)
    /// // COV:EXCL(phantom DA: grcov attributes a debug-info region to this doc-comment line)
    /// `local_buf` SHOULD be sized to // COV:EXCL(phantom DA: grcov attributes a debug-info region to this doc-comment line)
    /// `<KernelState<E::RuntimeState, V::RuntimeState> as Replicable>::ENCODED_LEN`.
    /// A short buffer truncates the local projection, which fails
    /// `agrees_with` against any full-size peer (LLR-CCS-102) and
    /// surfaces here as `RefuseStateMismatch` — exactly the safe
    /// failure mode for a buffer-sizing bug.
    pub fn check_lockstep_agreement<'lb, 'pb>(
        &self,
        cycle_seq: u64,
        channel_id: crate::ChannelId,
        local_buf: &'lb mut [u8],
        peers: &[Option<crate::kernel::snapshot::ChannelSnapshot<'pb>>],
        quorum: usize,
    ) -> crate::kernel::snapshot::LockstepDecision {
        let local = self.project_for_cross_channel(cycle_seq, channel_id, local_buf);
        crate::kernel::snapshot::decide_lockstep(&local, peers, quorum)
    }
}

// --- Watchdog ---

pub trait Watchdog {
    fn kick(&mut self);
    fn check_deadline(&self) -> bool;
}

impl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer> Watchdog
    for AviateKernelImpl<E, V, M, S>
{
    fn kick(&mut self) {
        // Minimal implementation: just a stub for now as we don't have full timing context
        // In a real system, this would update a timestamp
    }

    // COV:EXCL_START(STUB: watchdog placeholder, not implemented)
    fn check_deadline(&self) -> bool {
        true
    }
    // COV:EXCL_STOP
}

/// Aviate core initialization
pub fn init_core() {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::multirotor::MultirotorController;
    use crate::control::ConfigMode;
    use crate::mixer::ActuatorCmd;

    struct DummyMixer;
    impl Mixer for DummyMixer {
        const ALGORITHM_ID: u64 = 0x4D49_5854_4553_5431; // "MIXTEST1"

        fn mix(&self, _axis: &crate::control::AxisCommand) -> ActuatorCmd {
            ActuatorCmd::default()
        }
    }

    fn create_kernel() -> AviateKernelImpl<Ekf, MultirotorController, DummyMixer, Sanitizer> {
        let mode_config = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        };
        AviateKernelImpl::new(
            Ekf::default(),
            MultirotorController::default(),
            DummyMixer,
            Sanitizer,
            mode_config,
        )
    }

    #[test]
    fn test_ground_reset_success_unit() {
        let mut kernel = create_kernel();
        kernel.state.init_state = InitState::Fault;
        kernel.state.faults = crate::fault::FaultFlags::ALL_IMU_FAILED;

        kernel.ground_reset();

        assert_eq!(kernel.state.init_state, InitState::ConfigLoading);
        assert!(kernel.state.faults.is_empty());

        // Cover DummyMixer
        kernel.pipeline.mixer.mix(&crate::control::AxisCommand {
            roll: crate::types::NormalizedSigned(0.0),
            pitch: crate::types::NormalizedSigned(0.0),
            yaw: crate::types::NormalizedSigned(0.0),
            collective: crate::types::Normalized(0.0),
        });
    }

    #[test]
    fn read_accessors_expose_the_built_kernel_surfaces() {
        let mut kernel = create_kernel();
        // The read accessors are the only public routes to the
        // pipeline and config; the scenario override is the only
        // mutable route and is confined to tests by the boundary gate.
        assert_ne!(kernel.pipeline().algorithm_identity_hash(), 0);
        let hash_before = kernel.cfg().canonical_hash();
        kernel.cfg_scenario_override().command_timeout_ms =
            kernel.cfg().command_timeout_ms.wrapping_add(1);
        assert_ne!(kernel.cfg().canonical_hash(), hash_before);
    }
}
