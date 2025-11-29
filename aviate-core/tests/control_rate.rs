//! Tests for Rate Controller
//!
//! The rate controller is the innermost loop in the control cascade.
//! It converts angular rate setpoints to normalized torque commands.
//!
//! Covers:
//! - Zero error produces zero output
//! - Positive/negative error tracking
//! - Gain scaling behavior
//! - Output saturation at [-1, 1]
//! - Independent axis control

use aviate_core::control::rate::RateController;
use aviate_core::types::RadiansPerSecond;

// =============================================================================
// Zero Error Cases
// =============================================================================

#[test]
fn zero_error_produces_zero_output() {
    let ctrl = RateController::new([1.0, 1.0, 1.0]);
    let setpoint = [RadiansPerSecond(0.0); 3];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    assert!((output[0].0).abs() < 1e-6, "Roll output should be zero");
    assert!((output[1].0).abs() < 1e-6, "Pitch output should be zero");
    assert!((output[2].0).abs() < 1e-6, "Yaw output should be zero");
}

#[test]
fn matching_setpoint_and_current_produces_zero() {
    let ctrl = RateController::new([2.0, 2.0, 2.0]);
    let setpoint = [
        RadiansPerSecond(1.5),
        RadiansPerSecond(-0.5),
        RadiansPerSecond(0.3),
    ];
    let current = [
        RadiansPerSecond(1.5),
        RadiansPerSecond(-0.5),
        RadiansPerSecond(0.3),
    ];

    let output = ctrl.step(setpoint, current);

    assert!((output[0].0).abs() < 1e-6);
    assert!((output[1].0).abs() < 1e-6);
    assert!((output[2].0).abs() < 1e-6);
}

// =============================================================================
// Positive Error (setpoint > current)
// =============================================================================

#[test]
fn positive_roll_error_produces_positive_output() {
    let ctrl = RateController::new([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(1.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    assert!(
        output[0].0 > 0.0,
        "Positive error should produce positive output"
    );
    assert!(
        (output[0].0 - 1.0).abs() < 1e-6,
        "Output should equal error * gain"
    );
}

#[test]
fn positive_error_scales_with_gain() {
    let ctrl = RateController::new([0.5, 0.5, 0.5]);
    let setpoint = [
        RadiansPerSecond(2.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    // error = 2.0, gain = 0.5 -> output = 1.0 (clamped)
    assert!((output[0].0 - 1.0).abs() < 1e-6);
}

// =============================================================================
// Negative Error (setpoint < current)
// =============================================================================

#[test]
fn negative_roll_error_produces_negative_output() {
    let ctrl = RateController::new([1.0, 1.0, 1.0]);
    let setpoint = [RadiansPerSecond(0.0); 3];
    let current = [
        RadiansPerSecond(1.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];

    let output = ctrl.step(setpoint, current);

    assert!(
        output[0].0 < 0.0,
        "Negative error should produce negative output"
    );
    assert!((output[0].0 - (-1.0)).abs() < 1e-6);
}

#[test]
fn negative_pitch_error() {
    let ctrl = RateController::new([1.0, 0.8, 1.0]);
    let setpoint = [
        RadiansPerSecond(0.0),
        RadiansPerSecond(-1.0),
        RadiansPerSecond(0.0),
    ];
    let current = [
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];

    let output = ctrl.step(setpoint, current);

    // error = -1.0, gain = 0.8 -> output = -0.8
    assert!((output[1].0 - (-0.8)).abs() < 1e-6);
}

// =============================================================================
// Output Saturation
// =============================================================================

#[test]
fn output_saturates_at_positive_one() {
    let ctrl = RateController::new([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(5.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    assert!(
        (output[0].0 - 1.0).abs() < 1e-6,
        "Output should clamp to 1.0"
    );
}

#[test]
fn output_saturates_at_negative_one() {
    let ctrl = RateController::new([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(-5.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    assert!(
        (output[0].0 - (-1.0)).abs() < 1e-6,
        "Output should clamp to -1.0"
    );
}

#[test]
fn high_gain_saturates_earlier() {
    let ctrl = RateController::new([10.0, 10.0, 10.0]);
    let setpoint = [
        RadiansPerSecond(0.2),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    // error = 0.2, gain = 10 -> raw = 2.0, clamped to 1.0
    assert!((output[0].0 - 1.0).abs() < 1e-6);
}

// =============================================================================
// Independent Axis Control
// =============================================================================

#[test]
fn axes_are_independent() {
    let ctrl = RateController::new([1.0, 2.0, 3.0]);
    let setpoint = [
        RadiansPerSecond(0.5),
        RadiansPerSecond(0.25),
        RadiansPerSecond(0.1),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    assert!((output[0].0 - 0.5).abs() < 1e-6, "Roll: 0.5 * 1.0 = 0.5");
    assert!((output[1].0 - 0.5).abs() < 1e-6, "Pitch: 0.25 * 2.0 = 0.5");
    assert!((output[2].0 - 0.3).abs() < 1e-6, "Yaw: 0.1 * 3.0 = 0.3");
}

#[test]
fn different_gains_per_axis() {
    let ctrl = RateController::new([0.1, 0.2, 0.3]);
    let setpoint = [
        RadiansPerSecond(1.0),
        RadiansPerSecond(1.0),
        RadiansPerSecond(1.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    assert!((output[0].0 - 0.1).abs() < 1e-6);
    assert!((output[1].0 - 0.2).abs() < 1e-6);
    assert!((output[2].0 - 0.3).abs() < 1e-6);
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn very_small_error() {
    let ctrl = RateController::new([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(1e-6),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = ctrl.step(setpoint, current);

    assert!((output[0].0 - 1e-6).abs() < 1e-9);
}

#[test]
fn mixed_positive_and_negative_errors() {
    let ctrl = RateController::new([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(0.5),
        RadiansPerSecond(-0.3),
        RadiansPerSecond(0.0),
    ];
    let current = [
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.2),
        RadiansPerSecond(-0.1),
    ];

    let output = ctrl.step(setpoint, current);

    // Roll: 0.5 - 0.0 = 0.5
    // Pitch: -0.3 - 0.2 = -0.5
    // Yaw: 0.0 - (-0.1) = 0.1
    assert!((output[0].0 - 0.5).abs() < 1e-6);
    assert!((output[1].0 - (-0.5)).abs() < 1e-6);
    assert!((output[2].0 - 0.1).abs() < 1e-6);
}
