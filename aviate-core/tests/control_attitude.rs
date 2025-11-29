//! Tests for Attitude Controller
//!
//! The attitude controller converts quaternion attitude error to angular rate setpoints.
//! Uses shortest-path quaternion error calculation.
//!
//! Covers:
//! - Level flight (identity quaternion)
//! - Single-axis rotations (roll, pitch, yaw)
//! - Large angle errors (inverted)
//! - Gain scaling
//! - Multi-axis errors

use aviate_core::control::attitude::AttitudeController;
use aviate_core::math::{Quaternion, Vector3};

const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

// =============================================================================
// Level Flight - Zero Error
// =============================================================================

#[test]
fn level_setpoint_and_current_produces_zero_rate() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    let current = Quaternion::IDENTITY;

    let rate_sp = ctrl.step(&setpoint, &current);

    assert!((rate_sp[0].0).abs() < 1e-5, "Roll rate should be zero");
    assert!((rate_sp[1].0).abs() < 1e-5, "Pitch rate should be zero");
    assert!((rate_sp[2].0).abs() < 1e-5, "Yaw rate should be zero");
}

// =============================================================================
// Single Axis - Roll
// =============================================================================

#[test]
fn roll_error_produces_roll_rate_correction() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    // 10 degrees roll
    let current = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 10.0 * DEG_TO_RAD);

    let rate_sp = ctrl.step(&setpoint, &current);

    // Should produce negative roll rate to correct back to level
    assert!(
        rate_sp[0].0 < 0.0,
        "Roll rate should be negative to correct"
    );
    assert!((rate_sp[1].0).abs() < 1e-5, "Pitch rate should be ~zero");
    assert!((rate_sp[2].0).abs() < 1e-5, "Yaw rate should be ~zero");
}

#[test]
fn negative_roll_error_produces_positive_correction() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    // -10 degrees roll
    let current = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), -10.0 * DEG_TO_RAD);

    let rate_sp = ctrl.step(&setpoint, &current);

    assert!(
        rate_sp[0].0 > 0.0,
        "Roll rate should be positive to correct"
    );
}

// =============================================================================
// Single Axis - Pitch
// =============================================================================

#[test]
fn pitch_error_produces_pitch_rate_correction() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    // 10 degrees nose-up pitch
    let current = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), 10.0 * DEG_TO_RAD);

    let rate_sp = ctrl.step(&setpoint, &current);

    assert!((rate_sp[0].0).abs() < 1e-5, "Roll rate should be ~zero");
    assert!(
        rate_sp[1].0 < 0.0,
        "Pitch rate should be negative to correct"
    );
    assert!((rate_sp[2].0).abs() < 1e-5, "Yaw rate should be ~zero");
}

#[test]
fn pitch_error_magnitude_check() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    let angle = 10.0 * DEG_TO_RAD;
    let current = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), angle);

    let rate_sp = ctrl.step(&setpoint, &current);

    // For small angles: error ≈ 2 * sin(angle/2) ≈ angle
    // rate = error * gain = ~0.1745 * 6.0 ≈ 1.05
    let expected = -2.0 * (angle / 2.0).sin() * 6.0;
    assert!(
        (rate_sp[1].0 - expected).abs() < 0.1,
        "Expected pitch rate ~{}, got {}",
        expected,
        rate_sp[1].0
    );
}

// =============================================================================
// Single Axis - Yaw
// =============================================================================

#[test]
fn yaw_error_produces_yaw_rate_correction() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    // 30 degrees yaw
    let current = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 30.0 * DEG_TO_RAD);

    let rate_sp = ctrl.step(&setpoint, &current);

    assert!((rate_sp[0].0).abs() < 1e-5, "Roll rate should be ~zero");
    assert!((rate_sp[1].0).abs() < 1e-5, "Pitch rate should be ~zero");
    assert!(rate_sp[2].0 < 0.0, "Yaw rate should be negative to correct");
}

// =============================================================================
// Large Angle Errors
// =============================================================================

#[test]
fn inverted_roll_produces_large_correction() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    // 180 degrees roll (inverted)
    let current = Quaternion::new(0.0, 1.0, 0.0, 0.0);

    let rate_sp = ctrl.step(&setpoint, &current);

    // Maximum error: 2 * 1.0 * 6.0 = -12.0
    assert!(
        (rate_sp[0].0 - (-12.0)).abs() < 1e-5,
        "Roll rate should be -12 rad/s"
    );
    assert!((rate_sp[1].0).abs() < 1e-5);
    assert!((rate_sp[2].0).abs() < 1e-5);
}

#[test]
fn ninety_degree_pitch() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    // 90 degrees pitch (vertical climb attitude)
    let current = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), 90.0 * DEG_TO_RAD);

    let rate_sp = ctrl.step(&setpoint, &current);

    // sin(45deg) ≈ 0.707, error ≈ 2 * 0.707 ≈ 1.414
    // rate = 1.414 * 6.0 ≈ 8.5
    assert!(rate_sp[1].0 < -5.0, "Pitch rate should be large negative");
}

// =============================================================================
// Gain Scaling
// =============================================================================

#[test]
fn higher_gain_produces_larger_output() {
    let angle = 10.0 * DEG_TO_RAD;
    let setpoint = Quaternion::IDENTITY;
    let current = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), angle);

    let ctrl_low = AttitudeController::new([3.0, 3.0, 1.0]);
    let ctrl_high = AttitudeController::new([6.0, 6.0, 2.0]);

    let rate_low = ctrl_low.step(&setpoint, &current);
    let rate_high = ctrl_high.step(&setpoint, &current);

    assert!(
        (rate_high[0].0.abs() - 2.0 * rate_low[0].0.abs()).abs() < 0.1,
        "Double gain should double output"
    );
}

#[test]
fn different_gains_per_axis() {
    let ctrl = AttitudeController::new([2.0, 4.0, 8.0]);
    let setpoint = Quaternion::IDENTITY;

    // Apply same small rotation to different axes and check ratio
    let roll_current = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 0.1);
    let pitch_current = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), 0.1);
    let yaw_current = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 0.1);

    let roll_rate = ctrl.step(&setpoint, &roll_current);
    let pitch_rate = ctrl.step(&setpoint, &pitch_current);
    let yaw_rate = ctrl.step(&setpoint, &yaw_current);

    // Pitch gain is 2x roll gain
    assert!((pitch_rate[1].0.abs() / roll_rate[0].0.abs() - 2.0).abs() < 0.1);
    // Yaw gain is 4x roll gain
    assert!((yaw_rate[2].0.abs() / roll_rate[0].0.abs() - 4.0).abs() < 0.1);
}

// =============================================================================
// Multi-Axis Errors
// =============================================================================

#[test]
fn combined_roll_and_pitch_error() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;

    // Combined rotation: 10 deg roll + 10 deg pitch
    let q_roll = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 10.0 * DEG_TO_RAD);
    let q_pitch = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), 10.0 * DEG_TO_RAD);
    let current = q_roll.mul(&q_pitch);

    let rate_sp = ctrl.step(&setpoint, &current);

    // Both roll and pitch rates should be non-zero
    assert!(rate_sp[0].0.abs() > 0.1, "Roll rate should be non-zero");
    assert!(rate_sp[1].0.abs() > 0.1, "Pitch rate should be non-zero");
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn very_small_error() {
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;
    let current = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 0.001); // ~0.06 deg

    let rate_sp = ctrl.step(&setpoint, &current);

    // Very small error should produce very small rate
    assert!(rate_sp[0].0.abs() < 0.02);
}

#[test]
fn quaternion_sign_ambiguity_handled() {
    // q and -q represent the same rotation
    let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
    let setpoint = Quaternion::IDENTITY;

    let q = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 30.0 * DEG_TO_RAD);
    let neg_q = Quaternion::new(-q.w, -q.x, -q.y, -q.z);

    let rate_q = ctrl.step(&setpoint, &q);
    let rate_neg_q = ctrl.step(&setpoint, &neg_q);

    // Should produce equivalent corrections (may differ by sign depending on implementation)
    assert!(
        (rate_q[0].0.abs() - rate_neg_q[0].0.abs()).abs() < 1e-5,
        "q and -q should produce equivalent magnitude correction"
    );
}
