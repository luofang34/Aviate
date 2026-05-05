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
pub mod pipeline;
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
    pub pipeline: KernelPipeline<E, V, M, S>,

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
    pub cfg: ResolvedKernelConfig,
}

/// Type alias for the kernel struct.
///
/// Parameter order mirrors the constructor: `<E, V, M, S>` =
/// estimator, vehicle controller, mixer, sanitizer.
pub type AviateKernel<E, V, M, S> = AviateKernelImpl<E, V, M, S>;

/// Default kernel: 18-state EKF + group-aware Sanitizer. Use when
/// callers don't need to substitute estimation or sanitization.
pub type DefaultAviateKernel<V, M> = AviateKernelImpl<Ekf, V, M, Sanitizer>;

impl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer>
    AviateKernelImpl<E, V, M, S>
{
    /// Construct a kernel with default config and default pre-arm
    /// requirements. Direct struct-literal initialization — bypasses
    /// the `Result`-returning builder because every required component
    /// is supplied positionally, so the build can never fail.
    ///
    /// New code should prefer `AviateKernelBuilder` directly when any
    /// non-default field (custom limits, custom command_timeout, etc.)
    /// is needed.
    pub fn new(
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
}
