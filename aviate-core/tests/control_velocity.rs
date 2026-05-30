//! Tests for Velocity Controller (public-API)
//!
//! The velocity controller converts a NED velocity error into a
//! collective thrust command plus an attitude setpoint for the
//! multirotor cascade.
//!
//! These checks pin the proportional contract: with the integral,
//! derivative and acceleration-feedforward terms disabled
//! (`vel_i = vel_d = vel_accel_ff = 0`) and `dt = 0` (integrator
//! frozen), the loop is a pure per-axis P controller around the
//! hover trim. The current attitude is held level (identity), so the
//! tilt-compensation factor `1/cos(tilt)` is unity and does not enter
//! the assertions.
//!
//! Covers:
//! - Hover (zero velocity error)
//! - Vertical velocity control (climb/descend)
//! - Horizontal velocity control (produces roll/pitch)
//! - Thrust clamping
//! - Roll/pitch angle limiting

use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::velocity::{
    AccelFeedforward, VelocityCommand, VelocityController, VelocityLoopState,
};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::types::MetersPerSecond;

/// P-only velocity controller with the given horizontal/vertical P
/// gains, tilt cap, and hover trim. Integral, derivative and
/// feedforward are zeroed so the response is a clean proportional law.
fn p_only(vel_p: [f32; 3], max_roll_pitch: f32, hover: f32) -> VelocityController {
    let mut g = CascadeGains::x500_defaults();
    g.vel_p = vel_p;
    g.vel_i = [0.0; 3];
    g.vel_d = [0.0; 3];
    g.vel_accel_ff = 0.0;
    g.vel_max_roll_pitch = max_roll_pitch;
    VelocityController::new(g, hover)
}

/// One P-only step from a fresh loop state with no feedforward and a
/// frozen integrator (`dt = 0`).
fn p_step(
    ctrl: &VelocityController,
    setpoint: Vector3<MetersPerSecond>,
    current: Vector3<MetersPerSecond>,
    current_att: &Quaternion,
) -> VelocityCommand {
    let mut state = VelocityLoopState::default();
    ctrl.step(
        &mut state,
        setpoint,
        current,
        AccelFeedforward::default(),
        current_att,
        0.0,
    )
}

// =============================================================================
// Hover - Zero Velocity Error
// =============================================================================

#[test]
fn zero_velocity_error_produces_hover_thrust() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // Hover thrust is nominally 0.5
    assert!(
        (cmd.collective.0 - 0.5).abs() < 0.1,
        "Collective should be ~0.5 at hover"
    );
}

#[test]
fn zero_horizontal_error_produces_level_attitude() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // Attitude setpoint should be close to level (identity)
    assert!(
        (cmd.attitude.w - 1.0).abs() < 0.1,
        "Attitude should be near level"
    );
}

// =============================================================================
// Vertical Velocity Control
// =============================================================================

#[test]
fn descending_too_fast_increases_collective() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    // NED: positive Z is down, so +2.0 means descending at 2 m/s
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(2.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // Error is -2.0, should increase collective above hover
    assert!(
        cmd.collective.0 > 0.5,
        "Collective should increase to arrest descent, got {}",
        cmd.collective.0
    );
}

#[test]
fn climbing_too_fast_decreases_collective() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    // NED: negative Z velocity means climbing
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(-2.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // Error is +2.0 (want to go less negative), should decrease collective
    assert!(
        cmd.collective.0 < 0.5,
        "Collective should decrease to reduce climb rate, got {}",
        cmd.collective.0
    );
}

#[test]
fn commanded_descent_rate() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    // Command 1 m/s descent (positive Z in NED)
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(1.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // Want to descend faster (positive error in Z), reduce collective
    assert!(
        cmd.collective.0 < 0.5,
        "Collective should decrease to initiate descent"
    );
}

// =============================================================================
// Horizontal Velocity Control
//
// Sign convention (verified end-to-end in SITL, pinned by the
// in-source unit test `horizontal_velocity_error_drives_consistent_
// tilt_direction`): a north (+X) velocity command yields a NEGATIVE
// to_euler pitch and an east (+Y) command a POSITIVE to_euler roll.
// The rate→mixer half of the loop closes the sign so the vehicle
// accelerates toward the commanded velocity.
// =============================================================================

#[test]
fn forward_velocity_error_produces_pitch_down() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    // Want to go forward (positive X in NED = North)
    let setpoint = Vector3::new(
        MetersPerSecond(5.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // North velocity command tilts the thrust vector forward via a
    // negative to_euler pitch (see the sign-convention note above).
    let (_, pitch, _) = cmd.attitude.to_euler();
    assert!(
        pitch < 0.0,
        "North vel_sp must produce negative-pitch setpoint, got {}",
        pitch
    );
}

#[test]
fn rightward_velocity_error_produces_roll_right() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    // Want to go right (positive Y in NED = East)
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(5.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // East velocity command tilts the thrust vector right via a
    // positive to_euler roll (right-wing-down).
    let (roll, _, _) = cmd.attitude.to_euler();
    assert!(
        roll > 0.0,
        "East vel_sp must produce positive-roll setpoint, got {}",
        roll
    );
}

// =============================================================================
// Thrust Clamping
// =============================================================================

#[test]
fn collective_clamps_at_zero() {
    let ctrl = p_only([0.1, 0.1, 0.5], 0.5, 0.5);
    // Very high climb rate error
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(-10.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    assert!(cmd.collective.0 >= 0.0, "Collective should not go negative");
}

#[test]
fn collective_clamps_at_one() {
    let ctrl = p_only([0.1, 0.1, 0.5], 0.5, 0.5);
    // Very high descent rate error
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(10.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    assert!(cmd.collective.0 <= 1.0, "Collective should not exceed 1.0");
}

// =============================================================================
// Roll/Pitch Limiting
// =============================================================================

#[test]
fn roll_pitch_limited_to_max() {
    let max_angle = 0.5; // ~28 degrees
    let ctrl = p_only([1.0, 1.0, 0.2], max_angle, 0.5);
    // Large velocity error
    let setpoint = Vector3::new(
        MetersPerSecond(50.0),
        MetersPerSecond(50.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    let (roll, pitch, _) = cmd.attitude.to_euler();
    assert!(
        roll.abs() <= max_angle + 0.1,
        "Roll {} should be limited to {}",
        roll,
        max_angle
    );
    assert!(
        pitch.abs() <= max_angle + 0.1,
        "Pitch {} should be limited to {}",
        pitch,
        max_angle
    );
}

// =============================================================================
// Gain Scaling
// =============================================================================

#[test]
fn higher_horizontal_gain_produces_larger_tilt() {
    let setpoint = Vector3::new(
        MetersPerSecond(5.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let ctrl_low = p_only([0.05, 0.05, 0.2], 0.5, 0.5);
    let ctrl_high = p_only([0.1, 0.1, 0.2], 0.5, 0.5);

    let cmd_low = p_step(&ctrl_low, setpoint, current, &Quaternion::IDENTITY);
    let cmd_high = p_step(&ctrl_high, setpoint, current, &Quaternion::IDENTITY);

    let (_, pitch_low, _) = cmd_low.attitude.to_euler();
    let (_, pitch_high, _) = cmd_high.attitude.to_euler();

    assert!(
        pitch_high.abs() > pitch_low.abs(),
        "Higher gain should produce larger tilt: {} vs {}",
        pitch_high,
        pitch_low
    );
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn small_velocity_error() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    let setpoint = Vector3::new(
        MetersPerSecond(0.1),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let current = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );

    let cmd = p_step(&ctrl, setpoint, current, &Quaternion::IDENTITY);

    // Should produce small adjustments
    assert!((cmd.collective.0 - 0.5).abs() < 0.1);
    let (roll, pitch, _) = cmd.attitude.to_euler();
    assert!(roll.abs() < 0.1);
    assert!(pitch.abs() < 0.1);
}

#[test]
fn matching_velocities_at_non_zero() {
    let ctrl = p_only([0.1, 0.1, 0.2], 0.5, 0.5);
    let velocity = Vector3::new(
        MetersPerSecond(3.0),
        MetersPerSecond(2.0),
        MetersPerSecond(-1.0),
    );

    let cmd = p_step(&ctrl, velocity, velocity, &Quaternion::IDENTITY);

    // No error -> hover thrust
    assert!((cmd.collective.0 - 0.5).abs() < 0.1);
}
