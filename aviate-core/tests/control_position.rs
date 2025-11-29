//! Tests for Position Controller
//!
//! The position controller is the outermost loop in the control cascade.
//! It converts position error to velocity setpoints.
//!
//! Covers:
//! - Zero error (position hold)
//! - Single axis position errors
//! - Velocity output clamping
//! - Gain scaling
//! - 3D position tracking

use aviate_core::control::position::PositionController;
use aviate_core::math::Vector3;
use aviate_core::types::Meters;

// =============================================================================
// Position Hold - Zero Error
// =============================================================================

#[test]
fn zero_error_produces_zero_velocity() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let setpoint = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

#[test]
fn at_setpoint_produces_zero_velocity() {
    let ctrl = PositionController::new([0.5, 0.5, 0.5]);
    let position = Vector3::new(Meters(10.0), Meters(-5.0), Meters(-20.0));

    let vel_sp = ctrl.step(position, position);

    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

// =============================================================================
// Single Axis - X (North)
// =============================================================================

#[test]
fn positive_x_error_produces_positive_x_velocity() {
    let ctrl = PositionController::new([0.5, 0.5, 0.5]);
    let setpoint = Vector3::new(Meters(10.0), Meters(0.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    // error = 10, gain = 0.5 -> velocity = 5.0
    assert!((vel_sp.x.0 - 5.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

#[test]
fn negative_x_error_produces_negative_x_velocity() {
    let ctrl = PositionController::new([0.5, 0.5, 0.5]);
    let setpoint = Vector3::new(Meters(-10.0), Meters(0.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!((vel_sp.x.0 - (-5.0)).abs() < 1e-6);
}

// =============================================================================
// Single Axis - Y (East)
// =============================================================================

#[test]
fn positive_y_error_produces_positive_y_velocity() {
    let ctrl = PositionController::new([1.0, 0.8, 1.0]);
    let setpoint = Vector3::new(Meters(0.0), Meters(5.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    // error = 5, gain = 0.8 -> velocity = 4.0
    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0 - 4.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

// =============================================================================
// Single Axis - Z (Down in NED)
// =============================================================================

#[test]
fn altitude_error_produces_z_velocity() {
    let ctrl = PositionController::new([1.0, 1.0, 0.5]);
    // Want to climb (more negative Z in NED)
    let setpoint = Vector3::new(Meters(0.0), Meters(0.0), Meters(-20.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(-10.0));

    let vel_sp = ctrl.step(setpoint, current);

    // error = -20 - (-10) = -10, gain = 0.5 -> velocity = -5.0
    assert!((vel_sp.z.0 - (-5.0)).abs() < 1e-6);
}

#[test]
fn descent_command() {
    let ctrl = PositionController::new([1.0, 1.0, 0.5]);
    // Want to descend (less negative Z in NED)
    let setpoint = Vector3::new(Meters(0.0), Meters(0.0), Meters(-5.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(-15.0));

    let vel_sp = ctrl.step(setpoint, current);

    // error = -5 - (-15) = 10, gain = 0.5 -> velocity = 5.0 (descend)
    assert!((vel_sp.z.0 - 5.0).abs() < 1e-6);
}

// =============================================================================
// Velocity Clamping
// =============================================================================

#[test]
fn large_error_clamps_velocity_positive() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let setpoint = Vector3::new(Meters(100.0), Meters(0.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    // error = 100, but should clamp to 10 m/s
    assert!((vel_sp.x.0 - 10.0).abs() < 1e-6, "Should clamp to 10 m/s");
}

#[test]
fn large_error_clamps_velocity_negative() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let setpoint = Vector3::new(Meters(-100.0), Meters(0.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!(
        (vel_sp.x.0 - (-10.0)).abs() < 1e-6,
        "Should clamp to -10 m/s"
    );
}

#[test]
fn clamping_per_axis() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let setpoint = Vector3::new(Meters(50.0), Meters(-50.0), Meters(50.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!((vel_sp.x.0 - 10.0).abs() < 1e-6);
    assert!((vel_sp.y.0 - (-10.0)).abs() < 1e-6);
    assert!((vel_sp.z.0 - 10.0).abs() < 1e-6);
}

// =============================================================================
// Gain Scaling
// =============================================================================

#[test]
fn gain_affects_output_linearly() {
    let error = 4.0;
    let setpoint = Vector3::new(Meters(error), Meters(0.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let ctrl_low = PositionController::new([0.25, 0.25, 0.25]);
    let ctrl_high = PositionController::new([0.5, 0.5, 0.5]);

    let vel_low = ctrl_low.step(setpoint, current);
    let vel_high = ctrl_high.step(setpoint, current);

    // Double gain should double velocity (before clamping)
    assert!((vel_high.x.0 / vel_low.x.0 - 2.0).abs() < 1e-6);
}

#[test]
fn different_gains_per_axis() {
    let ctrl = PositionController::new([0.1, 0.2, 0.4]);
    let setpoint = Vector3::new(Meters(10.0), Meters(10.0), Meters(10.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!((vel_sp.x.0 - 1.0).abs() < 1e-6, "X: 10 * 0.1 = 1.0");
    assert!((vel_sp.y.0 - 2.0).abs() < 1e-6, "Y: 10 * 0.2 = 2.0");
    assert!((vel_sp.z.0 - 4.0).abs() < 1e-6, "Z: 10 * 0.4 = 4.0");
}

// =============================================================================
// 3D Position Tracking
// =============================================================================

#[test]
fn diagonal_error_produces_diagonal_velocity() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let setpoint = Vector3::new(Meters(6.0), Meters(6.0), Meters(-6.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!((vel_sp.x.0 - 6.0).abs() < 1e-6);
    assert!((vel_sp.y.0 - 6.0).abs() < 1e-6);
    assert!((vel_sp.z.0 - (-6.0)).abs() < 1e-6);
}

#[test]
fn tracking_moving_setpoint() {
    let ctrl = PositionController::new([0.5, 0.5, 0.5]);

    // Simulated: setpoint is 5m ahead, we're catching up
    let setpoint = Vector3::new(Meters(15.0), Meters(0.0), Meters(-10.0));
    let current = Vector3::new(Meters(10.0), Meters(0.0), Meters(-10.0));

    let vel_sp = ctrl.step(setpoint, current);

    // X error = 5, gain = 0.5 -> velocity = 2.5
    assert!((vel_sp.x.0 - 2.5).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn very_small_error() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let setpoint = Vector3::new(Meters(0.001), Meters(0.0), Meters(0.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!((vel_sp.x.0 - 0.001).abs() < 1e-6);
}

#[test]
fn zero_gain_produces_zero_output() {
    let ctrl = PositionController::new([0.0, 0.0, 0.0]);
    let setpoint = Vector3::new(Meters(100.0), Meters(100.0), Meters(100.0));
    let current = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));

    let vel_sp = ctrl.step(setpoint, current);

    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}
