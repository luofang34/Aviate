//! Tests for the Position Controller (public-API).
//!
//! The position controller is the outermost loop in the control
//! cascade. It converts a position error into a velocity setpoint
//! using a square-root deceleration profile (the "sqrt shaper" used
//! by ArduPilot `AC_PosControl` / PX4): a plain-P slope near zero,
//! a `√(2·a·Δ)` branch once the error exceeds the linear distance
//! `d_lin = a/p²`, and an absolute `vel_cap` clamp. This guarantees
//! the vehicle can always decelerate to rest at the setpoint within
//! its braking authority `a`, instead of commanding a velocity it
//! cannot arrest (the failure mode of a plain-P position loop).
//!
//! These public-API checks pin the observable contract:
//! - zero error → zero velocity
//! - sign of velocity matches sign of position error, per axis
//! - inside the linear region (`|err| ≤ a/p²`) output equals `p·err`
//! - large errors saturate at `vel_cap`, never the linear value
//! - the sqrt branch undershoots the linear extrapolation `p·err`
//! - output is monotonic non-decreasing in error magnitude
//!
//! `PositionController::new([p,p,p])` ships the X500 defaults
//! `accel_limits = [1.5,1.5,1.5]`, `vel_caps = [2.0,2.0,3.0]`; tests
//! that need a different braking/cap envelope use `with_limits`.

use aviate_core::control::position::PositionController;
use aviate_core::math::Vector3;
use aviate_core::types::Meters;

fn pos(x: f32, y: f32, z: f32) -> Vector3<Meters> {
    Vector3::new(Meters(x), Meters(y), Meters(z))
}

const ORIGIN: Vector3<Meters> = Vector3 {
    x: Meters(0.0),
    y: Meters(0.0),
    z: Meters(0.0),
};

// =============================================================================
// Position Hold - Zero Error
// =============================================================================

#[test]
fn zero_error_produces_zero_velocity() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let vel_sp = ctrl.step(ORIGIN, ORIGIN);

    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

#[test]
fn at_setpoint_produces_zero_velocity() {
    let ctrl = PositionController::new([0.5, 0.5, 0.5]);
    let position = pos(10.0, -5.0, -20.0);

    let vel_sp = ctrl.step(position, position);

    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

// =============================================================================
// Single Axis - linear region: vel = p · err
//
// d_lin = a/p² = 1.5/0.25 = 6 m for p = 0.5, so a 2 m error is well
// inside the linear region and the output is exactly p·err.
// =============================================================================

#[test]
fn positive_x_error_produces_positive_x_velocity() {
    let ctrl = PositionController::new([0.5, 0.5, 0.5]);
    let vel_sp = ctrl.step(pos(2.0, 0.0, 0.0), ORIGIN);

    // 2 m error < d_lin (6 m): vel = 0.5 · 2 = 1.0 m/s.
    assert!(vel_sp.x.0 > 0.0, "positive error → positive velocity");
    assert!((vel_sp.x.0 - 1.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

#[test]
fn negative_x_error_produces_negative_x_velocity() {
    let ctrl = PositionController::new([0.5, 0.5, 0.5]);
    let vel_sp = ctrl.step(pos(-2.0, 0.0, 0.0), ORIGIN);

    assert!(vel_sp.x.0 < 0.0, "negative error → negative velocity");
    assert!((vel_sp.x.0 - (-1.0)).abs() < 1e-6);
}

#[test]
fn positive_y_error_produces_positive_y_velocity() {
    let ctrl = PositionController::new([1.0, 0.8, 1.0]);
    // d_lin_y = 1.5/0.64 ≈ 2.34 m; a 1 m error is linear: 0.8 · 1.
    let vel_sp = ctrl.step(pos(0.0, 1.0, 0.0), ORIGIN);

    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0 - 0.8).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

// =============================================================================
// Single Axis - Z (Down in NED)
// =============================================================================

#[test]
fn altitude_error_produces_z_velocity() {
    let ctrl = PositionController::new([1.0, 1.0, 0.5]);
    // Want to climb 2 m (more negative Z). err = -12 - (-10) = -2,
    // inside d_lin_z = 6 m: vel = 0.5 · -2 = -1.0 m/s (climb).
    let vel_sp = ctrl.step(pos(0.0, 0.0, -12.0), pos(0.0, 0.0, -10.0));

    assert!(vel_sp.z.0 < 0.0, "climb command → negative NED z velocity");
    assert!((vel_sp.z.0 - (-1.0)).abs() < 1e-6);
}

#[test]
fn descent_command() {
    let ctrl = PositionController::new([1.0, 1.0, 0.5]);
    // Want to descend 2 m (less negative Z). err = -13 - (-15) = +2,
    // linear: vel = 0.5 · 2 = +1.0 m/s (descend).
    let vel_sp = ctrl.step(pos(0.0, 0.0, -13.0), pos(0.0, 0.0, -15.0));

    assert!(
        vel_sp.z.0 > 0.0,
        "descend command → positive NED z velocity"
    );
    assert!((vel_sp.z.0 - 1.0).abs() < 1e-6);
}

// =============================================================================
// Velocity Cap Saturation
//
// A large error must saturate at vel_cap (the speed the shaper can
// still arrest within braking authority), NOT the linear value p·err.
// =============================================================================

#[test]
fn large_error_clamps_velocity_positive() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]); // cap_x = 2.0
    let vel_sp = ctrl.step(pos(100.0, 0.0, 0.0), ORIGIN);

    assert!(
        (vel_sp.x.0 - 2.0).abs() < 1e-6,
        "large +error should saturate at vel_cap_x = 2.0, got {}",
        vel_sp.x.0
    );
}

#[test]
fn large_error_clamps_velocity_negative() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]); // cap_x = 2.0
    let vel_sp = ctrl.step(pos(-100.0, 0.0, 0.0), ORIGIN);

    assert!(
        (vel_sp.x.0 - (-2.0)).abs() < 1e-6,
        "large -error should saturate at -vel_cap_x = -2.0, got {}",
        vel_sp.x.0
    );
}

#[test]
fn clamping_per_axis() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]); // caps = [2,2,3]
    let vel_sp = ctrl.step(pos(50.0, -50.0, 50.0), ORIGIN);

    assert!((vel_sp.x.0 - 2.0).abs() < 1e-6, "x saturates at +2.0");
    assert!((vel_sp.y.0 - (-2.0)).abs() < 1e-6, "y saturates at -2.0");
    assert!((vel_sp.z.0 - 3.0).abs() < 1e-6, "z saturates at +3.0");
}

// =============================================================================
// Gain Scaling (linear region)
// =============================================================================

#[test]
fn gain_affects_output_linearly() {
    // err = 4 m is inside d_lin for both gains (d_lin = 1.5/p²: 24 m
    // at p=0.25, 6 m at p=0.5), so both outputs are linear and the
    // ratio is exactly the gain ratio.
    let setpoint = pos(4.0, 0.0, 0.0);

    let ctrl_low = PositionController::new([0.25, 0.25, 0.25]);
    let ctrl_high = PositionController::new([0.5, 0.5, 0.5]);

    let vel_low = ctrl_low.step(setpoint, ORIGIN);
    let vel_high = ctrl_high.step(setpoint, ORIGIN);

    assert!((vel_low.x.0 - 1.0).abs() < 1e-6); // 0.25 · 4
    assert!((vel_high.x.0 - 2.0).abs() < 1e-6); // 0.5 · 4
    assert!((vel_high.x.0 / vel_low.x.0 - 2.0).abs() < 1e-6);
}

#[test]
fn different_gains_per_axis() {
    let ctrl = PositionController::new([0.1, 0.2, 0.4]);
    // err = 5 m is linear on every axis (d_lin = 1.5/p²: 150, 37.5,
    // 9.375 m); outputs are p·err and stay under the caps.
    let vel_sp = ctrl.step(pos(5.0, 5.0, 5.0), ORIGIN);

    assert!((vel_sp.x.0 - 0.5).abs() < 1e-6, "X: 5 * 0.1 = 0.5");
    assert!((vel_sp.y.0 - 1.0).abs() < 1e-6, "Y: 5 * 0.2 = 1.0");
    assert!((vel_sp.z.0 - 2.0).abs() < 1e-6, "Z: 5 * 0.4 = 2.0");
}

// =============================================================================
// 3D Tracking
// =============================================================================

#[test]
fn diagonal_error_produces_diagonal_velocity() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    // 1 m per axis is inside d_lin (1.5 m): vel = 1·err per axis.
    let vel_sp = ctrl.step(pos(1.0, 1.0, -1.0), ORIGIN);

    assert!((vel_sp.x.0 - 1.0).abs() < 1e-6);
    assert!((vel_sp.y.0 - 1.0).abs() < 1e-6);
    assert!((vel_sp.z.0 - (-1.0)).abs() < 1e-6);
}

#[test]
fn tracking_moving_setpoint() {
    // Lift the cap so a 5 m lead stays in the linear region and the
    // exact tracking velocity is observable: with_limits(p, a, cap).
    let ctrl = PositionController::with_limits([0.5, 0.5, 0.5], [1.5, 1.5, 1.5], [5.0, 5.0, 5.0]);
    // Setpoint 5 m ahead in X. d_lin = 6 m, so err = 5 is linear:
    // vel = 0.5 · 5 = 2.5 m/s, under the 5 m/s cap.
    let vel_sp = ctrl.step(pos(15.0, 0.0, -10.0), pos(10.0, 0.0, -10.0));

    assert!((vel_sp.x.0 - 2.5).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}

// =============================================================================
// Sqrt-shaper defining behaviour
// =============================================================================

#[test]
fn sqrt_region_undershoots_linear_extrapolation() {
    // Beyond the linear region the kinematic shaper commands LESS
    // than the naive p·err a plain-P loop would — that headroom is
    // exactly the braking margin that lets the vehicle stop without
    // overshoot. Use a high cap so the comparison isn't masked by
    // clamping.
    let p = 0.5;
    let ctrl = PositionController::with_limits([p, p, p], [1.5, 1.5, 1.5], [100.0, 100.0, 100.0]);
    let err = 30.0; // >> d_lin = 6 m
    let vel_sp = ctrl.step(pos(err, 0.0, 0.0), ORIGIN);

    let linear_extrapolation = p * err; // 15 m/s
                                        // Exact sqrt-shaper value: sign·√(2·a·(|err|−d_lin/2)) with
                                        // d_lin = a/p² = 1.5/0.25 = 6, i.e. √(2·1.5·(30−3)) = √81 = 9.0.
                                        // Pin the magnitude, not just the bound — a bound-only check would
                                        // pass for any value in (0, 15) and miss a sign/scale error.
    let expected = 9.0_f32;
    assert!(
        (vel_sp.x.0 - expected).abs() < 1e-3,
        "sqrt branch should command exactly {expected} m/s, got {}",
        vel_sp.x.0
    );
    assert!(
        vel_sp.x.0 < linear_extrapolation,
        "and must undershoot the linear extrapolation p·err = {linear_extrapolation}"
    );
}

#[test]
fn output_monotonic_in_error_magnitude() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]); // cap_x = 2.0
    let mut prev = 0.0;
    for e in [0.1_f32, 0.5, 1.0, 2.0, 5.0, 50.0] {
        let v = ctrl.step(pos(e, 0.0, 0.0), ORIGIN).x.0;
        assert!(
            v + 1e-6 >= prev,
            "velocity must not decrease as error grows: {prev} → {v} at err={e}"
        );
        assert!(v <= 2.0 + 1e-6, "never exceeds vel_cap, got {v} at err={e}");
        prev = v;
    }
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn very_small_error() {
    let ctrl = PositionController::new([1.0, 1.0, 1.0]);
    let vel_sp = ctrl.step(pos(0.001, 0.0, 0.0), ORIGIN);

    // Deep in the linear region: vel = 1.0 · 0.001.
    assert!((vel_sp.x.0 - 0.001).abs() < 1e-6);
}

#[test]
fn zero_gain_produces_zero_output() {
    let ctrl = PositionController::new([0.0, 0.0, 0.0]);
    let vel_sp = ctrl.step(pos(100.0, 100.0, 100.0), ORIGIN);

    assert!((vel_sp.x.0).abs() < 1e-6);
    assert!((vel_sp.y.0).abs() < 1e-6);
    assert!((vel_sp.z.0).abs() < 1e-6);
}
