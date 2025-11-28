//! Tests for Velocity Controller
//!
//! The velocity controller converts velocity error to collective thrust
//! and attitude setpoints for the multicopter.
//!
//! Covers:
//! - Hover (zero velocity error)
//! - Vertical velocity control (climb/descend)
//! - Horizontal velocity control (produces roll/pitch)
//! - Thrust clamping
//! - Roll/pitch angle limiting

use aviate_core::control::velocity::VelocityController;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::types::MetersPerSecond;

// =============================================================================
// Hover - Zero Velocity Error
// =============================================================================

#[test]
fn zero_velocity_error_produces_hover_thrust() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, _) = ctrl.step(setpoint, current, &current_att);

    // Hover thrust is nominally 0.5
    assert!((collective.0 - 0.5).abs() < 0.1, "Collective should be ~0.5 at hover");
}

#[test]
fn zero_horizontal_error_produces_level_attitude() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let (_, att_sp) = ctrl.step(setpoint, current, &current_att);

    // Attitude setpoint should be close to level (identity)
    assert!((att_sp.w - 1.0).abs() < 0.1, "Attitude should be near level");
}

// =============================================================================
// Vertical Velocity Control
// =============================================================================

#[test]
fn descending_too_fast_increases_collective() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    // NED: positive Z is down, so +2.0 means descending at 2 m/s
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(2.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, _) = ctrl.step(setpoint, current, &current_att);

    // Error is -2.0, should increase collective above hover
    assert!(collective.0 > 0.5, "Collective should increase to arrest descent, got {}", collective.0);
}

#[test]
fn climbing_too_fast_decreases_collective() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    // NED: negative Z velocity means climbing
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(-2.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, _) = ctrl.step(setpoint, current, &current_att);

    // Error is +2.0 (want to go less negative), should decrease collective
    assert!(collective.0 < 0.5, "Collective should decrease to reduce climb rate, got {}", collective.0);
}

#[test]
fn commanded_descent_rate() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    // Command 1 m/s descent (positive Z in NED)
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(1.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, _) = ctrl.step(setpoint, current, &current_att);

    // Want to descend faster (positive error in Z), reduce collective
    assert!(collective.0 < 0.5, "Collective should decrease to initiate descent");
}

// =============================================================================
// Horizontal Velocity Control
// =============================================================================

#[test]
fn forward_velocity_error_produces_pitch_down() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    // Want to go forward (positive X in NED = North)
    let setpoint = Vector3::new(MetersPerSecond(5.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let (_, att_sp) = ctrl.step(setpoint, current, &current_att);

    // To accelerate forward, need positive pitch (nose down in NED)
    let (_, pitch, _) = att_sp.to_euler();
    assert!(pitch > 0.0, "Pitch should be positive (nose down) to accelerate forward, got {}", pitch);
}

#[test]
fn rightward_velocity_error_produces_roll_right() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    // Want to go right (positive Y in NED = East)
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(5.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let (_, att_sp) = ctrl.step(setpoint, current, &current_att);

    // To accelerate right, need negative roll (right wing down)
    let (roll, _, _) = att_sp.to_euler();
    assert!(roll < 0.0, "Roll should be negative (right wing down) to accelerate right, got {}", roll);
}

// =============================================================================
// Thrust Clamping
// =============================================================================

#[test]
fn collective_clamps_at_zero() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.5], 0.5);
    // Very high climb rate error
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(-10.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, _) = ctrl.step(setpoint, current, &current_att);

    assert!(collective.0 >= 0.0, "Collective should not go negative");
}

#[test]
fn collective_clamps_at_one() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.5], 0.5);
    // Very high descent rate error
    let setpoint = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(10.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, _) = ctrl.step(setpoint, current, &current_att);

    assert!(collective.0 <= 1.0, "Collective should not exceed 1.0");
}

// =============================================================================
// Roll/Pitch Limiting
// =============================================================================

#[test]
fn roll_pitch_limited_to_max() {
    let max_angle = 0.5; // ~28 degrees
    let ctrl = VelocityController::new([1.0, 1.0, 0.2], max_angle);
    // Large velocity error
    let setpoint = Vector3::new(MetersPerSecond(50.0), MetersPerSecond(50.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let (_, att_sp) = ctrl.step(setpoint, current, &current_att);

    let (roll, pitch, _) = att_sp.to_euler();
    assert!(roll.abs() <= max_angle + 0.1, "Roll {} should be limited to {}", roll, max_angle);
    assert!(pitch.abs() <= max_angle + 0.1, "Pitch {} should be limited to {}", pitch, max_angle);
}

// =============================================================================
// Gain Scaling
// =============================================================================

#[test]
fn higher_horizontal_gain_produces_larger_tilt() {
    let setpoint = Vector3::new(MetersPerSecond(5.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let ctrl_low = VelocityController::new([0.05, 0.05, 0.2], 0.5);
    let ctrl_high = VelocityController::new([0.1, 0.1, 0.2], 0.5);

    let (_, att_low) = ctrl_low.step(setpoint, current, &current_att);
    let (_, att_high) = ctrl_high.step(setpoint, current, &current_att);

    let (_, pitch_low, _) = att_low.to_euler();
    let (_, pitch_high, _) = att_high.to_euler();

    assert!(pitch_high.abs() > pitch_low.abs(),
            "Higher gain should produce larger pitch: {} vs {}", pitch_high, pitch_low);
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn small_velocity_error() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    let setpoint = Vector3::new(MetersPerSecond(0.1), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current = Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, att_sp) = ctrl.step(setpoint, current, &current_att);

    // Should produce small adjustments
    assert!((collective.0 - 0.5).abs() < 0.1);
    let (roll, pitch, _) = att_sp.to_euler();
    assert!(roll.abs() < 0.1);
    assert!(pitch.abs() < 0.1);
}

#[test]
fn matching_velocities_at_non_zero() {
    let ctrl = VelocityController::new([0.1, 0.1, 0.2], 0.5);
    let velocity = Vector3::new(MetersPerSecond(3.0), MetersPerSecond(2.0), MetersPerSecond(-1.0));
    let current_att = Quaternion::IDENTITY;

    let (collective, _) = ctrl.step(velocity, velocity, &current_att);

    // No error -> hover thrust
    assert!((collective.0 - 0.5).abs() < 0.1);
}
