//! Tests for VtolController
//!
//! Covers uncovered lines in control/vtol.rs:
//! - Lines 8, 20: VtolController step method

use aviate_core::control::runtime::NoControllerState;
use aviate_core::control::vtol::VtolController;
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

#[test]
fn vtol_controller_step_returns_collective() {
    let controller = VtolController;
    let state = make_state();
    let limits = make_limits();

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.7),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let mut runtime = NoControllerState;
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Cruise,
        &limits,
    );

    // VtolController returns collective from setpoint
    assert_eq!(axis_cmd.collective.0, 0.7);
    // Placeholder returns zeros for roll/pitch/yaw
    assert_eq!(axis_cmd.roll.0, 0.0);
    assert_eq!(axis_cmd.pitch.0, 0.0);
    assert_eq!(axis_cmd.yaw.0, 0.0);
}

#[test]
fn vtol_controller_step_in_transition_mode() {
    let controller = VtolController;
    let state = make_state();
    let limits = make_limits();

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

    // Test with Transition mode
    let mut runtime = NoControllerState;
    let axis_cmd = controller.step(
        &mut runtime,
        &state,
        &cmd,
        &VehicleControlMode::from_control_mode(cmd.mode),
        ConfigMode::Transition,
        &limits,
    );

    assert_eq!(axis_cmd.collective.0, 0.5);
}
