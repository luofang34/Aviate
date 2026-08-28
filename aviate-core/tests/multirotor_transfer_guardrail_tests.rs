//! Multirotor transfer guardrails for production mode changes.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::multirotor::{MultirotorController, MultirotorRuntimeState};
use aviate_core::control::{
    AxisCommand, Command, CommandSource, ConfigMode, ControlMode, EffectiveControlTopology, Limits,
    Setpoint, VehicleControlMode, VehicleController,
};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{Mixer, QuadXMixerReversedSpin};
use aviate_core::replicable::Replicable;
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, NormalizedThrust, RadiansPerSecond};

fn state() -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 0.2),
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(1.0), Meters(2.0), Meters(-10.0)],
        velocity_ned: [
            MetersPerSecond(0.1),
            MetersPerSecond(-0.2),
            MetersPerSecond(0.05),
        ],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    }
}

fn limits() -> Limits {
    aviate_core::kernel::config::ResolvedKernelConfig::default().limits
}

fn command(mode: ControlMode, setpoint: Setpoint) -> Command {
    Command {
        mode,
        setpoint,
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 7,
        source: CommandSource::Autopilot,
    }
}

fn velocity_setpoint(x: f32) -> Setpoint {
    Setpoint {
        velocity: Some([
            MetersPerSecond(x),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ]),
        ..Default::default()
    }
}

fn axis_bits(value: AxisCommand) -> [u32; 4] {
    [
        value.roll.0.to_bits(),
        value.pitch.0.to_bits(),
        value.yaw.0.to_bits(),
        value.collective.0.to_bits(),
    ]
}

fn actuator_bits(value: &aviate_core::mixer::ActuatorCmd) -> [u32; 4] {
    core::array::from_fn(|index| value.outputs[index].0.to_bits())
}

fn timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

#[path = "multirotor_transfer_guardrail_tests/fallback.rs"]
mod fallback;

#[test]
fn supported_modes_select_the_declared_topology() {
    let controller = MultirotorController::default();
    let mut cases = [
        (
            command(
                ControlMode::Attitude,
                Setpoint {
                    attitude: Some(Quaternion::IDENTITY),
                    collective_thrust: NormalizedThrust(0.4),
                    ..Default::default()
                },
            ),
            EffectiveControlTopology::Attitude,
        ),
        (
            command(
                ControlMode::AltitudeHold,
                Setpoint {
                    vertical_speed: Some(MetersPerSecond(0.0)),
                    ..Default::default()
                },
            ),
            EffectiveControlTopology::Vertical,
        ),
        (
            command(
                ControlMode::PositionHold,
                Setpoint {
                    position: Some([Meters(1.0), Meters(2.0), Meters(-10.0)]),
                    ..Default::default()
                },
            ),
            EffectiveControlTopology::Position,
        ),
        (
            command(ControlMode::VelocityControl, velocity_setpoint(0.5)),
            EffectiveControlTopology::Velocity,
        ),
    ];
    for (command, expected) in &mut cases {
        let mut runtime = MultirotorRuntimeState {
            dt_sec: 0.01,
            ..Default::default()
        };
        let result = controller.step_with_observation(
            &mut runtime,
            &state(),
            command,
            &VehicleControlMode::from_control_mode(command.mode),
            ConfigMode::Hover,
            &limits(),
        );
        let observation = result.observation.multirotor.expect("multirotor witness");
        assert_eq!(observation.current_topology, *expected);
    }
}

#[test]
fn unsupported_modes_produce_a_safe_zero_command() {
    let controller = MultirotorController::default();
    for mode in [ControlMode::Rate, ControlMode::DeviationTracking] {
        let mut runtime = MultirotorRuntimeState {
            dt_sec: 0.01,
            ..Default::default()
        };
        let seed = command(ControlMode::VelocityControl, velocity_setpoint(0.5));
        let _ = controller.step(
            &mut runtime,
            &state(),
            &seed,
            &VehicleControlMode::from_control_mode(seed.mode),
            ConfigMode::Hover,
            &limits(),
        );
        let retained_velocity = runtime.velocity_loop;
        let retained_rate = runtime.rate_loop;
        let unsupported = command(
            mode,
            Setpoint {
                angular_rate: Some([RadiansPerSecond(1.0); 3]),
                collective_thrust: NormalizedThrust(0.7),
                ..Default::default()
            },
        );
        let result = controller.step_with_observation(
            &mut runtime,
            &state(),
            &unsupported,
            &VehicleControlMode::from_control_mode(mode),
            ConfigMode::Hover,
            &limits(),
        );
        assert_eq!(axis_bits(result.axis_command), [0; 4]);
        let observation = result.observation.multirotor.expect("multirotor witness");
        assert_eq!(
            observation.current_topology,
            EffectiveControlTopology::Unsupported
        );
        assert_eq!(runtime.velocity_loop, retained_velocity);
        assert_eq!(runtime.rate_loop, retained_rate);
        assert_eq!(runtime.previous_effective_mode, Some(mode));
        assert_eq!(
            runtime.previous_topology,
            Some(EffectiveControlTopology::Unsupported)
        );
        assert_eq!(axis_bits(runtime.last_axis_command), [0; 4]);
        assert!(runtime.axis_command_primed);

        let reentry = command(ControlMode::VelocityControl, velocity_setpoint(0.4));
        let reentry = controller.step_with_observation(
            &mut runtime,
            &state(),
            &reentry,
            &VehicleControlMode::from_control_mode(reentry.mode),
            ConfigMode::Hover,
            &limits(),
        );
        let reentry = reentry.observation.multirotor.expect("reentry witness");
        assert_eq!(
            reentry.previous_topology,
            Some(EffectiveControlTopology::Unsupported)
        );
        assert_eq!(reentry.setpoints.acceleration_ned, [Default::default(); 3]);
        assert_eq!(reentry.rate_loop.d, [0.0; 3]);
    }
}

#[test]
fn topology_edge_hygiene_is_output_neutral_for_alia_gains() {
    let controller = MultirotorController::default();
    assert_eq!(controller.velocity_gains().vel_accel_ff.to_bits(), 0);
    assert_eq!(controller.velocity_gains().vel_d.map(f32::to_bits), [0; 3]);

    let mut clean = MultirotorRuntimeState {
        dt_sec: 0.01,
        previous_topology: Some(EffectiveControlTopology::Attitude),
        ..Default::default()
    };
    let mut stale = clean;
    stale.vel_sp_primed = true;
    stale.last_vel_sp_ned = Vector3::new(
        MetersPerSecond(90.0),
        MetersPerSecond(-80.0),
        MetersPerSecond(70.0),
    );
    stale.velocity_loop.d_primed = true;
    stale.velocity_loop.last_vel_filt_ned = Vector3::new(
        MetersPerSecond(-60.0),
        MetersPerSecond(50.0),
        MetersPerSecond(-40.0),
    );
    let command = command(ControlMode::VelocityControl, velocity_setpoint(0.7));
    let flags = VehicleControlMode::from_control_mode(command.mode);
    let clean_step = controller.step_with_observation(
        &mut clean,
        &state(),
        &command,
        &flags,
        ConfigMode::Hover,
        &limits(),
    );
    let stale_step = controller.step_with_observation(
        &mut stale,
        &state(),
        &command,
        &flags,
        ConfigMode::Hover,
        &limits(),
    );
    assert_eq!(
        axis_bits(clean_step.axis_command),
        axis_bits(stale_step.axis_command)
    );
    let clean_observation = clean_step.observation.multirotor.expect("clean witness");
    let stale_observation = stale_step.observation.multirotor.expect("stale witness");
    assert_eq!(
        clean_observation.setpoints.acceleration_ned,
        [Default::default(); 3]
    );
    assert_eq!(clean_observation.velocity_loop.d, [0.0; 3]);
    assert_eq!(clean_observation, stale_observation);

    let mixer = QuadXMixerReversedSpin {
        timestamp_source: timestamp,
    };
    let clean_actuator = mixer.mix(&clean_step.axis_command);
    let stale_actuator = mixer.mix(&stale_step.axis_command);
    assert_eq!(
        actuator_bits(&clean_actuator),
        actuator_bits(&stale_actuator)
    );
}

#[test]
fn vertical_topology_entry_restarts_the_derivative_sample() {
    let mut gains = CascadeGains::x500_defaults();
    gains.vel_d[2] = 0.2;
    let controller = MultirotorController::from_gains(gains, 0.5);
    let mut clean = MultirotorRuntimeState {
        dt_sec: 0.01,
        previous_topology: Some(EffectiveControlTopology::Attitude),
        ..Default::default()
    };
    let mut stale = clean;
    stale.velocity_loop.d_primed = true;
    stale.velocity_loop.last_vel_filt_ned.z = MetersPerSecond(-40.0);
    let command = command(
        ControlMode::AltitudeHold,
        Setpoint {
            vertical_speed: Some(MetersPerSecond(0.0)),
            ..Default::default()
        },
    );
    let flags = VehicleControlMode::from_control_mode(command.mode);
    let clean_step = controller.step_with_observation(
        &mut clean,
        &state(),
        &command,
        &flags,
        ConfigMode::Hover,
        &limits(),
    );
    let stale_step = controller.step_with_observation(
        &mut stale,
        &state(),
        &command,
        &flags,
        ConfigMode::Hover,
        &limits(),
    );
    let clean_observation = clean_step.observation.multirotor.expect("clean witness");
    let stale_observation = stale_step.observation.multirotor.expect("stale witness");
    assert_eq!(clean_observation.velocity_loop.d[2].to_bits(), 0);
    assert_eq!(stale_observation.velocity_loop.d[2].to_bits(), 0);
    assert_eq!(
        axis_bits(clean_step.axis_command),
        axis_bits(stale_step.axis_command)
    );
}

#[test]
fn cloned_runtime_restores_a_topology_edge_deterministically() {
    let controller = MultirotorController::default();
    let mut primary = MultirotorRuntimeState {
        dt_sec: 0.01,
        ..Default::default()
    };
    let position = command(
        ControlMode::PositionHold,
        Setpoint {
            position: Some([Meters(3.0), Meters(2.0), Meters(-10.0)]),
            ..Default::default()
        },
    );
    let _ = controller.step(
        &mut primary,
        &state(),
        &position,
        &VehicleControlMode::from_control_mode(position.mode),
        ConfigMode::Hover,
        &limits(),
    );
    let mut restored = primary;
    let mut primary_bytes = [0u8; MultirotorRuntimeState::ENCODED_LEN];
    let mut restored_bytes = [0u8; MultirotorRuntimeState::ENCODED_LEN];
    primary.encode_canonical(&mut primary_bytes);
    restored.encode_canonical(&mut restored_bytes);
    assert_eq!(primary_bytes, restored_bytes);

    let velocity = command(ControlMode::VelocityControl, velocity_setpoint(0.6));
    let flags = VehicleControlMode::from_control_mode(velocity.mode);
    let primary_step = controller.step_with_observation(
        &mut primary,
        &state(),
        &velocity,
        &flags,
        ConfigMode::Hover,
        &limits(),
    );
    let restored_step = controller.step_with_observation(
        &mut restored,
        &state(),
        &velocity,
        &flags,
        ConfigMode::Hover,
        &limits(),
    );
    assert_eq!(primary_step, restored_step);
}
