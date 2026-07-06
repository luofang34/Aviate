//! Phase-0 mock-pipeline smoke test: the four substitutable trait
//! surfaces (Estimator / VehicleController / Mixer / ActuatorSanitizer)
//! actually drive a kernel cycle when every implementor is replaced
//! by a custom-built mock. Proves end-to-end that the trait dispatch
//! paths work — not just that each individual trait *compiles* against
//! a non-default impl.
//!
//! Completion criterion (from the Phase-0 plan): swapping any single
//! trait's implementation does not require touching `kernel_update.rs`
//! or any other internal kernel module — only the type parameter.
//! This file is the structural witness for that property.

use aviate_core::checks::{KernelChecks, PreArmFlags};
use aviate_core::control::runtime::ControllerRuntimeState;
use aviate_core::control::{
    AxisCommand, Command, CommandSource, ConfigMode, ControlMode, Limits, Setpoint,
    VehicleControlMode, VehicleController,
};
use aviate_core::ekf::runtime::EstimatorRuntimeState;
use aviate_core::ekf::Estimator;
use aviate_core::kernel::config::ResolvedKernelConfig;
use aviate_core::kernel::pipeline::KernelPipeline;
use aviate_core::kernel::state::KernelState;
use aviate_core::kernel::AviateKernelImpl;
use aviate_core::mixer::{
    ActuatorCmd, ActuatorFallbackState, ActuatorSanitizer, ActuatorState, Mixer, ModeConfig,
    SanitizeReport,
};
use aviate_core::sensor::{
    AirspeedData, BaroData, GnssData, ImuData, MagData, SensorHealth, SensorReading, SensorSet,
};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    MetersPerSecondSquared, Normalized, NormalizedSigned, RadiansPerSecond, Scalar,
};
use aviate_core::ChannelId;

// ----- Mock Estimator -----
//
// Selects its own `RuntimeState` distinct from `EkfState` to prove
// the kernel does not assume the EKF's 18-state-+-18×18-covariance
// shape. `MockEstimatorRuntime` is a 6-byte struct with no
// covariance — totally incompatible with EkfState. If the kernel
// were to inadvertently downcast to `&EkfState` somewhere, this
// fails to compile.

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct MockEstimatorRuntime {
    predict_calls: u32,
    initialized: bool,
}

impl EstimatorRuntimeState for MockEstimatorRuntime {
    fn reset(&mut self) {
        *self = Self::default();
    }
}

impl aviate_core::replicable::Replicable for MockEstimatorRuntime {
    const ENCODED_LEN: usize = 5; // u32 (predict_calls) + u8 (initialized)
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = 0usize;
        w += aviate_core::replicable::copy_into(buf, w, &self.predict_calls.to_le_bytes());
        w += aviate_core::replicable::copy_into(buf, w, &[if self.initialized { 1 } else { 0 }]);
        w
    }
}

#[derive(Default)]
struct MockEstimator;

impl Estimator for MockEstimator {
    type RuntimeState = MockEstimatorRuntime;

    const ALGORITHM_ID: u64 = 0x4553_544D_4F43_4B00; // "ESTMOCK\0"

    fn observe(
        &self,
        state: &mut MockEstimatorRuntime,
        _sensors: &SensorSet,
        _overrides: Option<&aviate_core::control::SensorOverrides>,
        _dt: Scalar,
    ) {
        state.predict_calls = state.predict_calls.wrapping_add(1);
        state.initialized = true;
    }

    fn estimate(&self, state: &MockEstimatorRuntime) -> aviate_core::state::StateEstimate {
        // Project the mock's tiny runtime onto the kernel-facing
        // `StateEstimate` summary. Quality is `Good` once the mock
        // has seen at least one predict() call — that's all the
        // kernel needs to know to gate its in-flight checks.
        use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
        if state.initialized {
            StateEstimate {
                attitude: aviate_core::math::Quaternion::IDENTITY,
                angular_velocity: [aviate_core::types::RadiansPerSecond(0.0); 3],
                position_ned: [aviate_core::types::Meters(0.0); 3],
                velocity_ned: [aviate_core::types::MetersPerSecond(0.0); 3],
                quality: EstimateQuality::Good,
                valid_flags: StateValidFlags::all(),
            }
        } else {
            StateEstimate {
                attitude: aviate_core::math::Quaternion::IDENTITY,
                angular_velocity: [aviate_core::types::RadiansPerSecond(0.0); 3],
                position_ned: [aviate_core::types::Meters(0.0); 3],
                velocity_ned: [aviate_core::types::MetersPerSecond(0.0); 3],
                quality: EstimateQuality::Unusable,
                valid_flags: StateValidFlags::empty(),
            }
        }
    }
}

// ----- Mock Controller -----

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct MockControllerRuntime {
    step_count: u32,
}

impl ControllerRuntimeState for MockControllerRuntime {
    fn reset(&mut self) {
        self.step_count = 0;
    }
}

impl aviate_core::replicable::Replicable for MockControllerRuntime {
    const ENCODED_LEN: usize = 4;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        aviate_core::replicable::copy_into(buf, 0, &self.step_count.to_le_bytes())
    }
}

struct MockController;

impl VehicleController for MockController {
    type RuntimeState = MockControllerRuntime;

    const ALGORITHM_ID: u64 = 0x4354_4C4D_4F43_4B00; // "CTLMOCK\0"

    fn step(
        &self,
        runtime: &mut MockControllerRuntime,
        _state: &aviate_core::state::StateEstimate,
        _command: &Command,
        _flags: &VehicleControlMode,
        _mode: ConfigMode,
        _limits: &Limits,
    ) -> AxisCommand {
        runtime.step_count = runtime.step_count.wrapping_add(1);
        AxisCommand {
            roll: NormalizedSigned(0.0),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.0),
        }
    }
}

// ----- Mock Mixer -----

struct MockMixer;

impl Mixer for MockMixer {
    const ALGORITHM_ID: u64 = 0x4D49_584D_4F43_4B00; // "MIXMOCK\0"

    fn mix(&self, _axis: &AxisCommand) -> ActuatorCmd {
        ActuatorCmd::default()
    }
}

// ----- Mock Sanitizer -----

struct MockSanitizer;

impl ActuatorSanitizer for MockSanitizer {
    const ALGORITHM_ID: u64 = 0x5341_4E4D_4F43_4B00; // "SANMOCK\0"

    fn sanitize(
        &self,
        _cmd: &mut ActuatorCmd,
        _mode: &ModeConfig,
        _fallback: &mut ActuatorFallbackState,
    ) -> SanitizeReport {
        SanitizeReport::default()
    }
}

// ----- Test fixtures -----

fn make_sensors() -> SensorSet {
    let ts = Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    };
    let valid_imu = SensorReading {
        value: ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
            gyro: [
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ],
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };
    let mut imus: [SensorReading<ImuData>; 3] =
        core::array::from_fn(|_| SensorReading::<ImuData>::default());
    imus[0] = valid_imu;
    SensorSet {
        imus,
        gnss: core::array::from_fn(|_| SensorReading::<GnssData>::default()),
        mags: core::array::from_fn(|_| SensorReading::<MagData>::default()),
        baros: core::array::from_fn(|_| SensorReading::<BaroData>::default()),
        airspeeds: core::array::from_fn(|_| SensorReading::<AirspeedData>::default()),
        geometry: None,
    }
}

fn make_command() -> Command {
    Command {
        source: CommandSource::Pilot,
        mode: ControlMode::Rate,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
    }
}

fn make_kernel() -> AviateKernelImpl<MockEstimator, MockController, MockMixer, MockSanitizer> {
    AviateKernelImpl {
        pipeline: KernelPipeline::new(MockEstimator, MockController, MockMixer, MockSanitizer),
        state: KernelState::new(KernelChecks::with_pre_arm_required(PreArmFlags::empty())),
        cfg: ResolvedKernelConfig::default(),
    }
}

// ----- Tests -----

#[test]
fn mock_pipeline_drives_full_update_cycle_without_panic() {
    // Smoke test: the kernel can be constructed AND step through
    // update() with all four trait impls replaced. If trait dispatch
    // wired any of the four wrong, this fails to compile or panics
    // mid-cycle.
    let mut kernel = make_kernel();
    let sensors = make_sensors();
    let cmd = make_command();
    let actuator_state = ActuatorState::default();

    let _result = kernel.update(
        ChannelId(0),
        TimeDelta {
            dt_sec: aviate_core::types::Seconds(0.001),
            tick_delta: 1000,
        },
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );
}

#[test]
fn mock_estimator_predict_is_actually_invoked() {
    // Verify the kernel calls into Estimator::predict during update().
    // If the kernel were to bypass the trait (e.g. cast to a concrete
    // type), this would fail. The mock counts predict_calls via a
    // Cell so we can read it back through &kernel.
    let mut kernel = make_kernel();
    // Bypass the safety gate (init_state == PowerOn returns early).
    // We're not exercising the lifecycle state machine here — that's
    // tests/kernel.rs's job — only the trait-dispatch path.
    kernel.state.init_state = aviate_core::kernel::InitState::Armed;
    let sensors = make_sensors();
    let cmd = make_command();
    let actuator_state = ActuatorState::default();

    assert_eq!(kernel.state.estimator.predict_calls, 0);

    let _ = kernel.update(
        ChannelId(0),
        TimeDelta {
            dt_sec: aviate_core::types::Seconds(0.001),
            tick_delta: 1000,
        },
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );

    let n = kernel.state.estimator.predict_calls;
    assert!(
        n >= 1,
        "MockEstimator::predict should have been called at least once during update(); was {}",
        n
    );
}

#[test]
fn mock_controller_runtime_state_is_owned_by_kernel_state() {
    // Verify KernelState owns the controller's runtime state via the
    // associated-type wiring: `state: KernelState<V::RuntimeState>`.
    // Mutating `kernel.state.controller` directly should be visible
    // when the kernel runs the controller (full cycle), then
    // ground-resetting should clear it back to zero.
    let mut kernel = make_kernel();
    kernel.state.controller.step_count = 99;
    kernel.ground_reset();
    assert_eq!(
        kernel.state.controller.step_count, 0,
        "ground_reset must clear MockControllerRuntime via the trait's reset hook"
    );
}
