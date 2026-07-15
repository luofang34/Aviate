//! Tests for MultirotorController with position and velocity control paths
//!
//! Covers uncovered lines in control/multirotor.rs:
//! - Position control path (lines 42-55)
//! - Velocity control path (lines 57-65)

use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::multirotor::{MultirotorController, MultirotorRuntimeState};
use aviate_core::control::{
    Command, CommandSource, ConfigMode, ControlMode, Limits, Setpoint, VehicleControlMode,
    VehicleController,
};
use aviate_core::math::Quaternion;
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_core::types::{Meters, MetersPerSecond, Normalized, Radians, RadiansPerSecond};

fn make_state() -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-10.0)],
        velocity_ned: [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    }
}

fn make_limits() -> Limits {
    Limits {
        max_roll: Radians(0.5),
        max_pitch: Radians(0.5),
        max_roll_rate: RadiansPerSecond(2.0),
        max_pitch_rate: RadiansPerSecond(2.0),
        max_yaw_rate: RadiansPerSecond(1.5),
        max_horizontal_speed: MetersPerSecond(10.0),
        max_climb_rate: MetersPerSecond(3.0),
        max_descent_rate: MetersPerSecond(2.0),
        max_altitude: Meters(100.0),
        min_altitude: Meters(5.0),
        min_airspeed: None,
        max_airspeed: None,
        max_load_factor: 2.5,
        min_load_factor: 0.0,
    }
}

// =============================================================================
// Position Control Path Tests
// =============================================================================

#[test]
fn mc_controller_position_control_path() {
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();

    // Command with position setpoint triggers position control path
    let cmd = Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some([Meters(5.0), Meters(5.0), Meters(-15.0)]),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = MultirotorRuntimeState::default();
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );

    // Controller should produce output (position error -> velocity -> attitude -> rate)
    // Exact values depend on gains, but should be non-zero when there's position error
    let collective = axis_cmd.collective;
    assert!(
        collective.0 >= 0.0 && collective.0 <= 1.0,
        "Collective should be valid: {}",
        collective.0
    );
}

#[test]
fn mc_controller_position_control_with_offset() {
    let controller = MultirotorController::default();
    let mut state = make_state();
    state.position_ned = [Meters(0.0), Meters(0.0), Meters(-10.0)];
    let limits = make_limits();

    // Command to move to a different position
    let cmd = Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some([Meters(10.0), Meters(0.0), Meters(-10.0)]),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = MultirotorRuntimeState::default();
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );

    // With X position error, should produce some roll/pitch command
    // to generate horizontal acceleration
    let _roll = axis_cmd.roll.0;
    let _pitch = axis_cmd.pitch.0;
    // Just verify it runs without panicking
}

// =============================================================================
// Velocity Control Path Tests
// =============================================================================

#[test]
fn mc_controller_velocity_control_path() {
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();

    // Command with velocity setpoint (no position) triggers velocity control path
    let cmd = Command {
        mode: ControlMode::VelocityControl,
        setpoint: Setpoint {
            velocity: Some([
                MetersPerSecond(2.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = MultirotorRuntimeState::default();
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );

    // Should produce valid output
    assert!(axis_cmd.collective.0 >= 0.0 && axis_cmd.collective.0 <= 1.0);
}

#[test]
fn mc_controller_velocity_control_vertical() {
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();

    // Command vertical velocity (climb)
    let cmd = Command {
        mode: ControlMode::VelocityControl,
        setpoint: Setpoint {
            velocity: Some([
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(-2.0),
            ]), // NED: -Z is up
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = MultirotorRuntimeState::default();
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );

    // Climbing should increase collective
    // (depends on controller gains, but should be reasonable)
    assert!(axis_cmd.collective.0 >= 0.0);
}

// =============================================================================
// Attitude-Only Control Path Tests
// =============================================================================

#[test]
fn mc_controller_attitude_only_path() {
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();

    // Command with only attitude and collective (no position or velocity)
    let tilted = Quaternion::from_axis_angle(aviate_core::math::Vector3::new(1.0, 0.0, 0.0), 0.2);

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(tilted),
            collective_thrust: Normalized(0.6),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = MultirotorRuntimeState::default();
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );

    // Collective should be passed through
    assert!(
        (axis_cmd.collective.0 - 0.6).abs() < 0.1,
        "Collective {} should be ~0.6",
        axis_cmd.collective.0
    );
}

#[test]
fn mc_controller_no_setpoint_uses_defaults() {
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();

    // Minimal command with no optional setpoints
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = MultirotorRuntimeState::default();
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );

    // Should use default level attitude (identity quaternion)
    // and produce minimal roll/pitch/yaw commands
    let _roll = axis_cmd.roll.0;
    let _pitch = axis_cmd.pitch.0;
    let _yaw = axis_cmd.yaw.0;
    // Just verify it runs
}

#[test]
fn mc_controller_position_feedforward_fires_on_primed_cycle() {
    // The position→velocity acceleration feedforward is a finite
    // difference of vel_sp across cycles; it runs only once the runtime
    // is primed (second cycle) and dt > 0. Two position-control cycles
    // exercise that path.
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();
    let cmd = Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some([Meters(8.0), Meters(-3.0), Meters(-12.0)]),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = MultirotorRuntimeState::default();
    // Cycle 1 primes vel_sp (dt_sec = 0 → feedforward skipped).
    let _ = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );
    // Cycle 2 with dt > 0 → the accel-feedforward finite difference runs.
    runtime.dt_sec = 0.01;
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );
    assert!(axis_cmd.collective.0 >= 0.0 && axis_cmd.collective.0 <= 1.0);
}

// =============================================================================
// Flag-Driven Loop Selection (issue #67)
//
// Loop selection follows the control-mode flags, not setpoint-field
// presence: an identical position setpoint engages the position loop
// only under a mode whose flags authorize it.
// =============================================================================

/// Build a command carrying a fixed horizontal position setpoint under
/// the given mode. The state is level with a large along-track error,
/// so the position loop (when selected) commands a non-level attitude
/// and hence a non-zero roll/pitch torque.
fn cmd_with_position(mode: ControlMode) -> Command {
    Command {
        mode,
        setpoint: Setpoint {
            position: Some([Meters(20.0), Meters(0.0), Meters(-10.0)]),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    }
}

fn step_roll_pitch(mode: ControlMode) -> (f32, f32) {
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();
    let cmd = cmd_with_position(mode);
    let mut runtime = MultirotorRuntimeState::default();
    let axis = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );
    (axis.roll.0, axis.pitch.0)
}

#[test]
fn position_mode_engages_position_loop() {
    // Position mode authorizes the position loop: the along-track
    // error tilts the vehicle, producing a non-zero pitch torque.
    let (_roll, pitch) = step_roll_pitch(ControlMode::PositionHold);
    assert!(
        pitch.abs() > 1e-3,
        "position loop should tilt the vehicle in PositionHold, pitch={pitch}"
    );
}

#[test]
fn stabilized_mode_ignores_identical_position_setpoint() {
    // Same position setpoint, Stabilized (Attitude) mode: the position
    // flag is clear, so the loop is NOT selected and the vehicle holds
    // the (identity) commanded attitude — level, near-zero torque.
    let (roll, pitch) = step_roll_pitch(ControlMode::Attitude);
    assert!(
        roll.abs() < 1e-3 && pitch.abs() < 1e-3,
        "Stabilized must not run the position loop, roll={roll} pitch={pitch}"
    );
}

#[test]
fn altitude_mode_ignores_horizontal_position_setpoint() {
    // Altitude mode enables the altitude/climb-rate flags but NOT the
    // horizontal position flag, so the horizontal position loop stays
    // unselected: no roll/pitch tilt from the along-track error.
    let (roll, pitch) = step_roll_pitch(ControlMode::AltitudeHold);
    assert!(
        roll.abs() < 1e-3 && pitch.abs() < 1e-3,
        "Altitude must not run the horizontal position loop, roll={roll} pitch={pitch}"
    );
}

#[test]
fn position_mode_without_position_setpoint_runs_open_loop() {
    // Position flag set but no position setpoint present: the outer
    // loop is not selected (no fabricated hold target), so the cascade
    // tracks the commanded attitude directly — level, near-zero torque.
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();
    let cmd = Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };
    let mut runtime = MultirotorRuntimeState::default();
    let axis = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );
    assert!(
        axis.roll.0.abs() < 1e-3 && axis.pitch.0.abs() < 1e-3,
        "missing position setpoint must not tilt the vehicle"
    );
}

#[test]
fn velocity_mode_without_velocity_setpoint_runs_open_loop() {
    // Velocity flag set but no velocity setpoint present: outer loop
    // not selected; cascade tracks commanded attitude directly.
    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();
    let cmd = Command {
        mode: ControlMode::VelocityControl,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };
    let mut runtime = MultirotorRuntimeState::default();
    let axis = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );
    assert!(
        axis.roll.0.abs() < 1e-3 && axis.pitch.0.abs() < 1e-3,
        "missing velocity setpoint must not tilt the vehicle"
    );
}

// =============================================================================
// Altitude / climb-rate hold (issue #66)
//
// AltitudeHold drives only the vertical branch of the velocity loop
// (collective around hover trim) while roll/pitch stay manual and yaw
// slaves to the heading setpoint. Thrust must respond to the vertical
// setpoint — not pass raw `collective_thrust` through.
// =============================================================================

/// Run one AltitudeHold step and return the resulting collective.
fn step_altitude(
    controller: &MultirotorController,
    state: &StateEstimate,
    limits: &Limits,
    setpoint: Setpoint,
) -> f32 {
    let cmd = Command {
        mode: ControlMode::AltitudeHold,
        setpoint,
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };
    let mut runtime = MultirotorRuntimeState::default();
    controller
        .step(
            &mut runtime,
            state,
            &cmd,
            &VehicleControlMode::from_control_mode(cmd.mode),
            ConfigMode::Hover,
            limits,
        )
        .collective
        .0
}

#[test]
fn altitude_mode_climb_rate_drives_collective_not_passthrough() {
    // A commanded climb rate must raise collective above a commanded
    // descent, and neither must equal the raw manual collective_thrust —
    // proving the vertical loop is engaged rather than passing thrust
    // straight through.
    let controller = MultirotorController::from_gains(CascadeGains::x500_defaults(), 0.5);
    let state = make_state();
    let limits = make_limits();
    let manual = 0.3;

    // NED: -Z is up, so a negative vertical_speed is a climb.
    let climb = step_altitude(
        &controller,
        &state,
        &limits,
        Setpoint {
            vertical_speed: Some(MetersPerSecond(-2.0)),
            collective_thrust: Normalized(manual),
            ..Default::default()
        },
    );
    let descend = step_altitude(
        &controller,
        &state,
        &limits,
        Setpoint {
            vertical_speed: Some(MetersPerSecond(2.0)),
            collective_thrust: Normalized(manual),
            ..Default::default()
        },
    );

    assert!(
        climb > descend,
        "climb collective {climb} must exceed descend collective {descend}"
    );
    assert!(
        (climb - manual).abs() > 1e-3,
        "climb collective {climb} must not be raw pass-through of {manual}"
    );
}

#[test]
fn altitude_mode_altitude_error_drives_thrust() {
    // Vehicle sits at 10 m (NED z = -10). Commanding a higher altitude
    // must climb (collective above hover trim); a lower altitude must
    // descend (below trim). Exercises the altitude→climb-rate shaper.
    let controller = MultirotorController::from_gains(CascadeGains::x500_defaults(), 0.5);
    let state = make_state();
    let limits = make_limits();

    let up = step_altitude(
        &controller,
        &state,
        &limits,
        Setpoint {
            altitude: Some(Meters(11.0)),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
    );
    let down = step_altitude(
        &controller,
        &state,
        &limits,
        Setpoint {
            altitude: Some(Meters(9.0)),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
    );

    assert!(
        up > down,
        "higher altitude {up} must out-thrust lower {down}"
    );
    assert!(
        up > 0.5 && down < 0.5,
        "climb must exceed hover trim and descent fall below it: up={up} down={down}"
    );
}

#[test]
fn altitude_mode_keeps_manual_roll_pitch() {
    // In AltitudeHold the horizontal attitude stays manual: a tilted
    // manual attitude setpoint must still produce roll torque even while
    // the vertical loop owns collective.
    let controller = MultirotorController::from_gains(CascadeGains::x500_defaults(), 0.5);
    let state = make_state();
    let limits = make_limits();
    let tilted = Quaternion::from_axis_angle(aviate_core::math::Vector3::new(1.0, 0.0, 0.0), 0.2);

    let cmd = Command {
        mode: ControlMode::AltitudeHold,
        setpoint: Setpoint {
            attitude: Some(tilted),
            vertical_speed: Some(MetersPerSecond(0.0)),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };
    let mut runtime = MultirotorRuntimeState::default();
    let axis = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Hover,
        &limits,
    );
    assert!(
        axis.roll.0.abs() > 1e-3,
        "manual roll must pass through in altitude mode, roll={}",
        axis.roll.0
    );
}

#[test]
fn altitude_mode_slaves_yaw_to_heading() {
    // With the vehicle at yaw 0 and a level manual attitude, a heading
    // setpoint must drive a yaw command; without it, yaw stays zero.
    let controller = MultirotorController::from_gains(CascadeGains::x500_defaults(), 0.5);
    let state = make_state();
    let limits = make_limits();

    let yaw_no_heading = {
        let axis_setpoint = Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        };
        let cmd = Command {
            mode: ControlMode::AltitudeHold,
            setpoint: axis_setpoint,
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 1,
            source: CommandSource::Pilot,
        };
        let mut runtime = MultirotorRuntimeState::default();
        controller
            .step(
                &mut runtime,
                &state,
                &cmd,
                &VehicleControlMode::from_control_mode(cmd.mode),
                ConfigMode::Hover,
                &limits,
            )
            .yaw
            .0
    };

    let yaw_with_heading = {
        let cmd = Command {
            mode: ControlMode::AltitudeHold,
            setpoint: Setpoint {
                attitude: Some(Quaternion::IDENTITY),
                heading: Some(Radians(0.5)),
                collective_thrust: Normalized(0.5),
                ..Default::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 1,
            source: CommandSource::Pilot,
        };
        let mut runtime = MultirotorRuntimeState::default();
        controller
            .step(
                &mut runtime,
                &state,
                &cmd,
                &VehicleControlMode::from_control_mode(cmd.mode),
                ConfigMode::Hover,
                &limits,
            )
            .yaw
            .0
    };

    assert!(
        yaw_no_heading.abs() < 1e-6,
        "level attitude with no heading setpoint must not command yaw, got {yaw_no_heading}"
    );
    assert!(
        yaw_with_heading.abs() > 1e-3,
        "heading setpoint must drive a yaw command, got {yaw_with_heading}"
    );
}

#[test]
fn altitude_hold_tracks_commanded_altitude_in_sim() {
    // Closed-loop against a vertical point-mass plant: collective maps to
    // upward acceleration around hover trim. Starting at 10 m and holding
    // a 20 m altitude setpoint, the commanded altitude must be tracked.
    let hover = 0.5_f32;
    let g = 9.81_f32;
    let dt = 0.01_f32;
    let controller = MultirotorController::from_gains(CascadeGains::x500_defaults(), hover);
    let limits = make_limits();

    let mut runtime = MultirotorRuntimeState {
        dt_sec: dt,
        ..Default::default()
    };

    let target_alt = 20.0_f32;
    let mut pos_ned_z = -10.0_f32;
    let mut vel_ned_z = 0.0_f32;
    let mut state = make_state();

    for _ in 0..6000 {
        state.position_ned[2] = Meters(pos_ned_z);
        state.velocity_ned[2] = MetersPerSecond(vel_ned_z);
        let cmd = Command {
            mode: ControlMode::AltitudeHold,
            setpoint: Setpoint {
                attitude: Some(Quaternion::IDENTITY),
                altitude: Some(Meters(target_alt)),
                collective_thrust: Normalized(hover),
                ..Default::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 1,
            source: CommandSource::Pilot,
        };
        let axis = controller.step(
            &mut runtime,
            &state,
            &cmd,
            &VehicleControlMode::from_control_mode(cmd.mode),
            ConfigMode::Hover,
            &limits,
        );
        // Upward thrust acceleration is zero at hover trim and scales
        // linearly with collective; NED z is down-positive.
        let accel_up = (axis.collective.0 / hover - 1.0) * g;
        vel_ned_z += -accel_up * dt;
        pos_ned_z += vel_ned_z * dt;
    }

    let final_alt = -pos_ned_z;
    assert!(
        (final_alt - target_alt).abs() < 1.0,
        "altitude hold must converge to {target_alt} m, settled at {final_alt} m"
    );
}

// =============================================================================
// Disarmed-gate boundary (law_invariants::DISARMED_COLLECTIVE_THRESHOLD)
// =============================================================================

#[test]
fn disarm_gate_boundary_is_the_named_invariant() {
    use aviate_core::control::law_invariants::DISARMED_COLLECTIVE_THRESHOLD;

    let controller = MultirotorController::default();
    let state = make_state();
    let limits = make_limits();
    let tilted = Quaternion::from_axis_angle(aviate_core::math::Vector3::new(1.0, 0.0, 0.0), 0.2);
    let cmd_with_collective = |c: f32| Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(tilted),
            collective_thrust: Normalized(c),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };
    let flags = VehicleControlMode::from_control_mode(ControlMode::Attitude);

    // Just below the threshold: axes silenced and loop memory reset,
    // even with a pending attitude error and a wound integrator.
    let mut runtime = MultirotorRuntimeState {
        vel_sp_primed: true,
        ..Default::default()
    };
    runtime.velocity_loop.integrator_ned.x = MetersPerSecond(1.5);
    let below = controller.step(
        &mut runtime,
        &state,
        &cmd_with_collective(DISARMED_COLLECTIVE_THRESHOLD - 1e-4),
        &flags,
        ConfigMode::Hover,
        &limits,
    );
    assert_eq!(below.roll.0, 0.0);
    assert_eq!(below.pitch.0, 0.0);
    assert_eq!(below.yaw.0, 0.0);
    assert_eq!(
        runtime.velocity_loop.integrator_ned.x.0, 0.0,
        "below the gate the velocity integrator must reset"
    );
    assert!(!runtime.vel_sp_primed);

    // At the threshold (the gate compares with strict `<`): the
    // cascade runs and the attitude error yields a live roll command.
    let mut runtime = MultirotorRuntimeState::default();
    let at = controller.step(
        &mut runtime,
        &state,
        &cmd_with_collective(DISARMED_COLLECTIVE_THRESHOLD),
        &flags,
        ConfigMode::Hover,
        &limits,
    );
    assert!(
        at.roll.0.abs() > 1e-3,
        "at the threshold the axes must be live, got roll {}",
        at.roll.0
    );
}
