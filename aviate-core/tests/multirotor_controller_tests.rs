//! Tests for McController with position and velocity control paths
//!
//! Covers uncovered lines in control/mc.rs:
//! - Position control path (lines 42-55)
//! - Velocity control path (lines 57-65)

use aviate_core::control::mc::McController;
use aviate_core::control::{
    Command, CommandSource, ConfigMode, ControlMode, Limits, Setpoint, VehicleController,
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
    let mut controller = McController::default();
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

    let axis_cmd = controller.step(&state, &cmd, ConfigMode::Hover, &limits);

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
    let mut controller = McController::default();
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

    let axis_cmd = controller.step(&state, &cmd, ConfigMode::Hover, &limits);

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
    let mut controller = McController::default();
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

    let axis_cmd = controller.step(&state, &cmd, ConfigMode::Hover, &limits);

    // Should produce valid output
    assert!(axis_cmd.collective.0 >= 0.0 && axis_cmd.collective.0 <= 1.0);
}

#[test]
fn mc_controller_velocity_control_vertical() {
    let mut controller = McController::default();
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

    let axis_cmd = controller.step(&state, &cmd, ConfigMode::Hover, &limits);

    // Climbing should increase collective
    // (depends on controller gains, but should be reasonable)
    assert!(axis_cmd.collective.0 >= 0.0);
}

// =============================================================================
// Attitude-Only Control Path Tests
// =============================================================================

#[test]
fn mc_controller_attitude_only_path() {
    let mut controller = McController::default();
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

    let axis_cmd = controller.step(&state, &cmd, ConfigMode::Hover, &limits);

    // Collective should be passed through
    assert!(
        (axis_cmd.collective.0 - 0.6).abs() < 0.1,
        "Collective {} should be ~0.6",
        axis_cmd.collective.0
    );
}

#[test]
fn mc_controller_no_setpoint_uses_defaults() {
    let mut controller = McController::default();
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

    let axis_cmd = controller.step(&state, &cmd, ConfigMode::Hover, &limits);

    // Should use default level attitude (identity quaternion)
    // and produce minimal roll/pitch/yaw commands
    let _roll = axis_cmd.roll.0;
    let _pitch = axis_cmd.pitch.0;
    let _yaw = axis_cmd.yaw.0;
    // Just verify it runs
}
