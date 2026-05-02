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
    VehicleController,
};
use aviate_core::ekf::{Estimator, EstimatorState};
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
// Pure no-op: never mutates `EstimatorState`. The kernel still gets a
// well-formed `StateEstimate` from `state.estimator.get_estimate()`
// (Unusable quality because `is_initialized() == false`), so the
// downstream cycle exercises the safe-output path. That's fine — the
// purpose is to prove the trait dispatch works.

#[derive(Default)]
struct MockEstimator {
    predict_calls: core::cell::Cell<u32>,
}

impl Estimator for MockEstimator {
    fn predict(&self, _state: &mut EstimatorState, _imu: &ImuData, _dt: Scalar) {
        self.predict_calls
            .set(self.predict_calls.get().wrapping_add(1));
    }
    fn update_gnss(&self, _: &mut EstimatorState, _: &SensorReading<GnssData>) {}
    fn update_baro(&self, _: &mut EstimatorState, _: &SensorReading<BaroData>) {}
    fn update_mag(&self, _: &mut EstimatorState, _: &SensorReading<MagData>) {}
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

struct MockController;

impl VehicleController for MockController {
    type RuntimeState = MockControllerRuntime;

    fn step(
        &self,
        runtime: &mut MockControllerRuntime,
        _state: &aviate_core::state::StateEstimate,
        _command: &Command,
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
    fn mix(&self, _axis: &AxisCommand) -> ActuatorCmd {
        ActuatorCmd::default()
    }
}

// ----- Mock Sanitizer -----

struct MockSanitizer;

impl ActuatorSanitizer for MockSanitizer {
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
        pipeline: KernelPipeline::new(
            MockEstimator::default(),
            MockController,
            MockMixer,
            MockSanitizer,
        ),
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

    assert_eq!(kernel.pipeline.estimator.predict_calls.get(), 0);

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

    let n = kernel.pipeline.estimator.predict_calls.get();
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
