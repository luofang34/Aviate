//! Controller cycle interval tests through the production kernel path.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::checks::PreArmFlags;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::multirotor::{MultirotorController, MultirotorRuntimeState};
use aviate_core::control::runtime::{ControllerRuntimeState, NoControllerState};
use aviate_core::control::vtol::VtolController;
use aviate_core::control::{
    AxisCommand, Command, CommandSource, ConfigMode, ControlLawV1, ControlMode, IntegratorAction,
    MultirotorControllerObservation, Setpoint, VehicleControlMode, VehicleController,
};
use aviate_core::ekf::runtime::EstimatorRuntimeState;
use aviate_core::ekf::Estimator;
use aviate_core::kernel::config::ResolvedKernelConfig;
use aviate_core::kernel::state::KernelState;
use aviate_core::kernel::{AviateKernelImpl, InitState};
use aviate_core::math::Quaternion;
use aviate_core::mixer::{ActuatorState, ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::replicable::Replicable;
use aviate_core::sensor::{ImuData, SensorHealth, SensorReading, SensorSet};
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, NormalizedThrust, RadiansPerSecond, Seconds,
};
use aviate_core::ChannelId;

#[derive(Clone, Debug, Default)]
struct FixedEstimatorState(StateEstimate);

impl EstimatorRuntimeState for FixedEstimatorState {
    fn reset(&mut self) {
        *self = Self::default();
    }
}

impl Replicable for FixedEstimatorState {
    const ENCODED_LEN: usize = 1;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        aviate_core::replicable::copy_into(buf, 0, &[0])
    }
}

#[derive(Default)]
struct FixedEstimator;

impl Estimator for FixedEstimator {
    type RuntimeState = FixedEstimatorState;

    const ALGORITHM_ID: u64 = 0x4649_5845_4553_5431; // "FIXEEST1"

    fn observe(
        &self,
        _state: &mut Self::RuntimeState,
        _sensors: &SensorSet,
        _overrides: Option<&aviate_core::control::SensorOverrides>,
        _dt: aviate_core::types::Scalar,
    ) {
    }

    fn estimate(&self, state: &Self::RuntimeState) -> StateEstimate {
        state.0.clone()
    }
}

type TimedKernel = AviateKernelImpl<FixedEstimator, MultirotorController, QuadXMixer, Sanitizer>;
type TimedState = KernelState<FixedEstimatorState, MultirotorRuntimeState>;

fn timestamp_source() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn estimate() -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-10.0)],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    }
}

fn make_kernel() -> TimedKernel {
    let mut kernel = aviate_core::kernel::builder::AviateKernelBuilder::new()
        .estimator(FixedEstimator)
        .controller(MultirotorController::default())
        .mixer(QuadXMixer { timestamp_source })
        .sanitizer(Sanitizer)
        .pre_arm_required(PreArmFlags::empty())
        .config(ResolvedKernelConfig {
            mode_config: ModeConfig {
                mode: ConfigMode::Hover,
                groups: &[],
            },
            ..Default::default()
        })
        .build()
        .expect("the default controller and configuration must match");
    kernel.state.init_state = InitState::Armed;
    kernel.state.control_law = ControlLawV1::Primary;
    kernel.state.estimator.0 = estimate();
    kernel
}

fn sensors() -> SensorSet {
    let mut sensors = SensorSet {
        imus: core::array::from_fn(|_| SensorReading::<ImuData>::default()),
        gnss: core::array::from_fn(|_| SensorReading::default()),
        mags: core::array::from_fn(|_| SensorReading::default()),
        baros: core::array::from_fn(|_| SensorReading::default()),
        airspeeds: core::array::from_fn(|_| SensorReading::default()),
        geometry: None,
    };
    sensors.imus[0] = SensorReading {
        value: ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
            gyro: [RadiansPerSecond(0.0); 3],
        },
        valid: true,
        source_id: 0,
        timestamp: timestamp_source(),
        health: SensorHealth::Good,
    };
    sensors
}

fn velocity_command(sequence: u32, north_mps: f32) -> Command {
    Command {
        mode: ControlMode::VelocityControl,
        setpoint: Setpoint {
            velocity: Some([
                MetersPerSecond(north_mps),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Autopilot,
    }
}

fn cycle(dt_sec: f32) -> TimeDelta {
    TimeDelta {
        dt_sec: Seconds(dt_sec),
        tick_delta: 10_000,
    }
}

fn update(kernel: &mut TimedKernel, dt_sec: f32, sequence: u32) -> aviate_core::UpdateResult {
    update_with_north_velocity(kernel, dt_sec, sequence, 0.2)
}

fn update_with_north_velocity(
    kernel: &mut TimedKernel,
    dt_sec: f32,
    sequence: u32,
    north_mps: f32,
) -> aviate_core::UpdateResult {
    kernel.update(
        ChannelId::PRIMARY,
        cycle(dt_sec),
        &sensors(),
        &velocity_command(sequence, north_mps),
        0,
        &ActuatorState::default(),
        None,
    )
}

fn observation(result: &aviate_core::UpdateResult) -> MultirotorControllerObservation {
    result
        .controller_observation
        .multirotor
        .expect("the multirotor controller must produce an observation")
}

fn actuator_bits(result: &aviate_core::UpdateResult) -> [u32; 16] {
    result.actuator.outputs.map(|output| output.0.to_bits())
}

#[test]
fn kernel_interval_activates_the_velocity_integrator_across_two_cycles() {
    let mut kernel = make_kernel();
    let first = update(&mut kernel, 0.01, 1);
    let first_observation = observation(&first);
    let first_after = first_observation.velocity_loop.integrator_after[0];

    assert_eq!(kernel.state.controller.dt_sec.to_bits(), 0.01_f32.to_bits());
    assert_eq!(
        first_observation.velocity_loop.integrator_action[0],
        IntegratorAction::Integrated
    );
    assert!(first_after > 0.0);

    let second = update(&mut kernel, 0.01, 2);
    let second_observation = observation(&second);
    assert_eq!(
        second_observation.velocity_loop.integrator_before[0].to_bits(),
        first_after.to_bits()
    );
    assert!(second_observation.velocity_loop.i[0] > 0.0);
    assert!(second_observation.velocity_loop.integrator_after[0] > first_after);
}

#[test]
fn invalid_or_zero_interval_suppresses_timed_terms_and_keeps_outputs_finite() {
    for invalid in [0.0, -0.01, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut kernel = make_kernel();
        let _ = update(&mut kernel, 0.01, 1);
        let retained = kernel.state.controller.velocity_loop.integrator_ned;
        kernel.state.estimator.0.angular_velocity[0] = RadiansPerSecond(0.25);
        kernel.state.estimator.0.velocity_ned[2] = MetersPerSecond(0.25);

        let result = update_with_north_velocity(&mut kernel, invalid, 2, 0.4);
        let observation = observation(&result);
        assert_eq!(kernel.state.controller.dt_sec.to_bits(), 0.0_f32.to_bits());
        assert_eq!(
            kernel.state.controller.velocity_loop.integrator_ned,
            retained
        );
        assert_eq!(
            observation.velocity_loop.integrator_action,
            [IntegratorAction::FrozenInactive; 3]
        );
        assert_eq!(observation.velocity_loop.d, [0.0; 3]);
        assert_eq!(observation.rate_loop.d, [0.0; 3]);
        assert_eq!(
            observation.setpoints.acceleration_ned,
            [MetersPerSecondSquared(0.0); 3]
        );
        assert!(result
            .actuator
            .outputs
            .iter()
            .all(|output| output.0.is_finite()));
    }
}

#[test]
fn cloned_snapshot_resumes_with_the_same_next_cycle_result() {
    let mut primary = make_kernel();
    let _ = update(&mut primary, 0.02, 1);
    let mut restored = make_kernel();
    restored.state = primary.state.clone();

    let mut primary_buf = [0u8; <TimedState as Replicable>::ENCODED_LEN];
    let mut restored_buf = [0u8; <TimedState as Replicable>::ENCODED_LEN];
    let primary_snapshot =
        primary.project_for_cross_channel(1, ChannelId::PRIMARY, &mut primary_buf);
    let restored_snapshot =
        restored.project_for_cross_channel(1, ChannelId::SECONDARY, &mut restored_buf);
    assert!(primary_snapshot.agrees_with(&restored_snapshot));

    let primary_result = update(&mut primary, 0.01, 2);
    let restored_result = update(&mut restored, 0.01, 2);
    assert_eq!(
        primary_result.controller_observation,
        restored_result.controller_observation
    );
    assert_eq!(
        actuator_bits(&primary_result),
        actuator_bits(&restored_result)
    );
    assert_eq!(primary.state.controller, restored.state.controller);
}

fn direct_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: NormalizedThrust(0.37),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    }
}

fn assert_default_interval_hook_preserves_output<C>(controller: C)
where
    C: VehicleController<RuntimeState = NoControllerState>,
{
    let mut runtime = NoControllerState;
    let command = direct_command();
    let flags = VehicleControlMode::from_control_mode(command.mode);
    let limits = ResolvedKernelConfig::default().limits;
    let baseline = controller.step(
        &mut runtime,
        &estimate(),
        &command,
        &flags,
        ConfigMode::Hover,
        &limits,
    );
    runtime.set_cycle_interval(Seconds(0.01));
    let timed = controller.step(
        &mut runtime,
        &estimate(),
        &command,
        &flags,
        ConfigMode::Hover,
        &limits,
    );
    assert_eq!(axis_bits(baseline), axis_bits(timed));
}

fn axis_bits(command: AxisCommand) -> [u32; 4] {
    [
        command.roll.0.to_bits(),
        command.pitch.0.to_bits(),
        command.yaw.0.to_bits(),
        command.collective.0.to_bits(),
    ]
}

#[test]
fn fixed_wing_default_interval_hook_preserves_output() {
    assert_default_interval_hook_preserves_output(FixedWingController);
}

#[test]
fn vtol_default_interval_hook_preserves_output() {
    assert_default_interval_hook_preserves_output(VtolController);
}
