//! Core kernel struct and constructors.
//!
//! Behavior is split across sibling modules:
//!   - `kernel_logic.rs` — lifecycle (init, arm, disarm, reset, fault handling).
//!   - `kernel_update.rs` — the per-cycle `update()` loop.
//!   - `kernel_trait.rs`  — the `AviateKernelTrait` definition and impl.

use crate::checks::{KernelChecks, PreArmFlags};
use crate::control::envelope::SimpleEnvelopeProtector;
use crate::control::{ConfigMode, ControlLawV1, Limits, VehicleController};
use crate::ekf::{Ekf, Estimator};
use crate::fault::{FaultFlags, FaultHandlingTable};
use crate::kernel_types::{Config, TimingStats, DEFAULT_COMMAND_TIMEOUT_MS};
use crate::mixer::{ActuatorSanitizer, ActuatorState, Mixer, ModeConfig, Sanitizer};
use crate::types::Normalized;

pub use crate::kernel_types::InitState;

pub struct AviateKernelImpl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer> {
    pub estimator: E,
    pub controller: V,
    pub mixer: M,
    pub sanitizer: S,
    pub protector: SimpleEnvelopeProtector,
    pub limits: Limits,
    pub mode: ConfigMode,
    pub mode_config: ModeConfig,

    // State Machine
    pub init_state: InitState,
    pub faults: FaultFlags,
    pub fault_table: FaultHandlingTable,
    pub control_law: ControlLawV1,

    // Unified Check System (§17, §14, §4.5)
    pub checks: KernelChecks,

    // Actuator state tracking for transition checks
    pub actuator_state: ActuatorState,

    // Command timeout threshold (ms)
    pub command_timeout_ms: u32,

    // Configuration (spec §19)
    pub config: Config,

    // Timing tracking (spec §18)
    pub timing_stats: TimingStats,

    // Safety
    pub safe_output: [Normalized; 16], // MAX_ACTUATORS = 16
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
    pub fn new(
        estimator: E,
        controller: V,
        mixer: M,
        sanitizer: S,
        mode_config: ModeConfig,
    ) -> Self {
        Self {
            estimator,
            controller,
            mixer,
            sanitizer,
            protector: SimpleEnvelopeProtector,
            limits: Limits {
                max_roll: crate::types::Radians(0.78), // ~45 deg
                max_pitch: crate::types::Radians(0.78),
                max_roll_rate: crate::types::RadiansPerSecond(3.0),
                max_pitch_rate: crate::types::RadiansPerSecond(3.0),
                max_yaw_rate: crate::types::RadiansPerSecond(3.0),
                max_horizontal_speed: crate::types::MetersPerSecond(10.0),
                max_climb_rate: crate::types::MetersPerSecond(2.0),
                max_descent_rate: crate::types::MetersPerSecond(2.0),
                max_altitude: crate::types::Meters(100.0),
                min_altitude: crate::types::Meters(0.0),
                min_airspeed: None,
                max_airspeed: None,
                max_load_factor: 2.0,
                min_load_factor: 0.0,
            },
            mode: ConfigMode::Hover,
            mode_config,

            init_state: InitState::PowerOn,
            faults: FaultFlags::empty(),
            fault_table: FaultHandlingTable::DEFAULT,
            control_law: ControlLawV1::Primary,
            checks: KernelChecks::new(),
            actuator_state: ActuatorState::default(),
            command_timeout_ms: DEFAULT_COMMAND_TIMEOUT_MS,
            config: Config::default(),
            timing_stats: TimingStats::default(),
            safe_output: [Normalized(0.0); 16],
        }
    }

    /// Create kernel with custom pre-arm requirements
    pub fn with_pre_arm_required(
        estimator: E,
        controller: V,
        mixer: M,
        sanitizer: S,
        mode_config: ModeConfig,
        required: PreArmFlags,
    ) -> Self {
        let mut kernel = Self::new(estimator, controller, mixer, sanitizer, mode_config);
        kernel.checks = KernelChecks::with_pre_arm_required(required);
        kernel
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
    use crate::mixer::ActuatorCmd;

    struct DummyMixer;
    impl Mixer for DummyMixer {
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
            Sanitizer::default(),
            mode_config,
        )
    }

    #[test]
    fn test_ground_reset_success_unit() {
        let mut kernel = create_kernel();
        kernel.init_state = InitState::Fault;
        kernel.faults = FaultFlags::ALL_IMU_FAILED;

        kernel.ground_reset();

        assert_eq!(kernel.init_state, InitState::ConfigLoading);
        assert!(kernel.faults.is_empty());

        // Cover DummyMixer
        kernel.mixer.mix(&crate::control::AxisCommand {
            roll: crate::types::NormalizedSigned(0.0),
            pitch: crate::types::NormalizedSigned(0.0),
            yaw: crate::types::NormalizedSigned(0.0),
            collective: crate::types::Normalized(0.0),
        });
    }
}
