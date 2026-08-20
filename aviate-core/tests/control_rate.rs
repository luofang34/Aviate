//! Tests for Rate Controller (public-API)
//!
//! The rate controller is the innermost loop in the control cascade.
//! It converts angular rate setpoints to normalized torque commands.
//!
//! These public-API tests pin the proportional contract: with the
//! derivative term disabled (`rate_d = 0`), the loop reduces to a
//! plain per-axis P controller, `out = clamp(error · rate_p, -1, 1)`.
//! The derivative-on-measurement behaviour is exercised separately
//! (it requires two cycles and a measurement delta) in
//! `d_term_damps_against_measurement_rise` and in the in-source unit
//! tests in `src/control/rate.rs`.
//!
//! Covers:
//! - Zero error produces zero output
//! - Positive/negative error tracking
//! - Gain scaling behavior
//! - Output saturation at [-1, 1]
//! - Independent axis control
//! - Derivative-on-measurement damping

use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::rate::{RateController, RateLoopState};
use aviate_core::types::RadiansPerSecond;

/// Build a P-only rate controller with the given per-axis P gains.
/// `rate_d = 0` collapses the loop to a pure proportional law so the
/// assertions can pin an exact `error · gain` response without the
/// derivative term participating.
fn p_only(rate_p: [f32; 3]) -> RateController {
    let mut g = CascadeGains::x500_defaults();
    g.rate_p = rate_p;
    g.rate_d = [0.0; 3];
    RateController::new(g)
}

/// One P-only step from a fresh loop state. `dt` is irrelevant when
/// `rate_d = 0` (the D branch is gated on `rate_d[i] > 0`).
fn p_step(
    ctrl: &RateController,
    setpoint: [RadiansPerSecond; 3],
    current: [RadiansPerSecond; 3],
) -> [aviate_core::types::NormalizedSigned; 3] {
    let mut state = RateLoopState::default();
    ctrl.step(&mut state, setpoint, current, 0.001)
}

// =============================================================================
// Zero Error Cases
// =============================================================================

#[test]
fn zero_error_produces_zero_output() {
    let ctrl = p_only([1.0, 1.0, 1.0]);
    let setpoint = [RadiansPerSecond(0.0); 3];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    assert!((output[0].0).abs() < 1e-6, "Roll output should be zero");
    assert!((output[1].0).abs() < 1e-6, "Pitch output should be zero");
    assert!((output[2].0).abs() < 1e-6, "Yaw output should be zero");
}

#[test]
fn matching_setpoint_and_current_produces_zero() {
    let ctrl = p_only([2.0, 2.0, 2.0]);
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

    let output = p_step(&ctrl, setpoint, current);

    assert!((output[0].0).abs() < 1e-6);
    assert!((output[1].0).abs() < 1e-6);
    assert!((output[2].0).abs() < 1e-6);
}

// =============================================================================
// Positive Error (setpoint > current)
// =============================================================================

#[test]
fn positive_roll_error_produces_positive_output() {
    let ctrl = p_only([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(1.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

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
    let ctrl = p_only([0.5, 0.5, 0.5]);
    let setpoint = [
        RadiansPerSecond(2.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    // error = 2.0, gain = 0.5 -> output = 1.0 (clamped)
    assert!((output[0].0 - 1.0).abs() < 1e-6);
}

// =============================================================================
// Negative Error (setpoint < current)
// =============================================================================

#[test]
fn negative_roll_error_produces_negative_output() {
    let ctrl = p_only([1.0, 1.0, 1.0]);
    let setpoint = [RadiansPerSecond(0.0); 3];
    let current = [
        RadiansPerSecond(1.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];

    let output = p_step(&ctrl, setpoint, current);

    assert!(
        output[0].0 < 0.0,
        "Negative error should produce negative output"
    );
    assert!((output[0].0 - (-1.0)).abs() < 1e-6);
}

#[test]
fn negative_pitch_error() {
    let ctrl = p_only([1.0, 0.8, 1.0]);
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

    let output = p_step(&ctrl, setpoint, current);

    // error = -1.0, gain = 0.8 -> output = -0.8
    assert!((output[1].0 - (-0.8)).abs() < 1e-6);
}

// =============================================================================
// Output Saturation
// =============================================================================

#[test]
fn output_saturates_at_positive_one() {
    let ctrl = p_only([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(5.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    assert!(
        (output[0].0 - 1.0).abs() < 1e-6,
        "Output should clamp to 1.0"
    );
}

#[test]
fn output_saturates_at_negative_one() {
    let ctrl = p_only([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(-5.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    assert!(
        (output[0].0 - (-1.0)).abs() < 1e-6,
        "Output should clamp to -1.0"
    );
}

#[test]
fn high_gain_saturates_earlier() {
    let ctrl = p_only([10.0, 10.0, 10.0]);
    let setpoint = [
        RadiansPerSecond(0.2),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    // error = 0.2, gain = 10 -> raw = 2.0, clamped to 1.0
    assert!((output[0].0 - 1.0).abs() < 1e-6);
}

// =============================================================================
// Independent Axis Control
// =============================================================================

#[test]
fn axes_are_independent() {
    let ctrl = p_only([1.0, 2.0, 3.0]);
    let setpoint = [
        RadiansPerSecond(0.5),
        RadiansPerSecond(0.25),
        RadiansPerSecond(0.1),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    assert!((output[0].0 - 0.5).abs() < 1e-6, "Roll: 0.5 * 1.0 = 0.5");
    assert!((output[1].0 - 0.5).abs() < 1e-6, "Pitch: 0.25 * 2.0 = 0.5");
    assert!((output[2].0 - 0.3).abs() < 1e-6, "Yaw: 0.1 * 3.0 = 0.3");
}

#[test]
fn different_gains_per_axis() {
    let ctrl = p_only([0.1, 0.2, 0.3]);
    let setpoint = [
        RadiansPerSecond(1.0),
        RadiansPerSecond(1.0),
        RadiansPerSecond(1.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    assert!((output[0].0 - 0.1).abs() < 1e-6);
    assert!((output[1].0 - 0.2).abs() < 1e-6);
    assert!((output[2].0 - 0.3).abs() < 1e-6);
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn very_small_error() {
    let ctrl = p_only([1.0, 1.0, 1.0]);
    let setpoint = [
        RadiansPerSecond(1e-6),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let current = [RadiansPerSecond(0.0); 3];

    let output = p_step(&ctrl, setpoint, current);

    assert!((output[0].0 - 1e-6).abs() < 1e-9);
}

#[test]
fn mixed_positive_and_negative_errors() {
    let ctrl = p_only([1.0, 1.0, 1.0]);
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

    let output = p_step(&ctrl, setpoint, current);

    // Roll: 0.5 - 0.0 = 0.5
    // Pitch: -0.3 - 0.2 = -0.5
    // Yaw: 0.0 - (-0.1) = 0.1
    assert!((output[0].0 - 0.5).abs() < 1e-6);
    assert!((output[1].0 - (-0.5)).abs() < 1e-6);
    assert!((output[2].0 - 0.1).abs() < 1e-6);
}

// =============================================================================
// Derivative-on-measurement (new cascade behaviour)
// =============================================================================

#[test]
fn d_term_damps_against_measurement_rise() {
    // With a non-zero `rate_d`, a measurement that rises between
    // cycles (setpoint held at zero) must drive the output MORE
    // negative than the P term alone — the derivative-on-measurement
    // term opposes the motion. The first cycle primes the filter and
    // emits no D contribution (no previous sample to difference).
    let mut g = CascadeGains::x500_defaults();
    g.rate_p = [2.5, 2.5, 2.5];
    g.rate_d = [0.05, 0.05, 0.0];
    let ctrl = RateController::new(g);
    let mut state = RateLoopState::default();

    let zero = [RadiansPerSecond(0.0); 3];
    // Prime against a zero sample.
    let _ = ctrl.step(&mut state, zero, zero, 0.001);

    // Roll measurement rises to 0.1 rad/s; setpoint stays at zero.
    let risen = [
        RadiansPerSecond(0.1),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let out = ctrl.step(&mut state, zero, risen, 0.001);

    // P term alone: error = -0.1, out_p = -0.25. The D term adds
    // further negative torque (damping the rising rate), so the
    // total must be strictly below the P-only value.
    let p_only_value = -0.1 * 2.5;
    assert!(
        out[0].0 < p_only_value,
        "D term should add damping: got {}, P-only would be {}",
        out[0].0,
        p_only_value
    );
    // Yaw has rate_d = 0, so it stays pure-P (here exactly zero error).
    assert!((out[2].0).abs() < 1e-6);
}

// =============================================================================
// Integral term
// =============================================================================

/// An I-enabled controller with P and D disabled, so the output reads
/// the integral state directly: out = clamp(integral, -1, 1).
fn i_only(rate_i: [f32; 3]) -> RateController {
    let mut g = CascadeGains::x500_defaults();
    g.rate_p = [0.0; 3];
    g.rate_d = [0.0; 3];
    g.rate_i = rate_i;
    RateController::new(g)
}

#[test]
fn i_term_accumulates_a_steady_error() {
    let ctrl = i_only([2.0, 0.0, 0.0]);
    let mut state = RateLoopState::default();
    let setpoint = [
        RadiansPerSecond(0.1),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let zero = [RadiansPerSecond(0.0); 3];
    let mut last = 0.0_f32;
    for _ in 0..100 {
        let out = ctrl.step(&mut state, setpoint, zero, 0.001);
        assert!(
            out[0].0 >= last,
            "a steady error must ratchet the integral upward"
        );
        last = out[0].0;
    }
    // 100 cycles × 0.1 rad/s × 2.0 × 0.001 s = 0.02 accumulated.
    assert!((last - 0.02).abs() < 1e-4, "integral reached {last}");
    // The axes with rate_i = 0 must stay untouched.
    let out = ctrl.step(&mut state, setpoint, zero, 0.001);
    assert!((out[1].0).abs() < 1e-6);
    assert!((out[2].0).abs() < 1e-6);
}

#[test]
fn a_saturated_command_cannot_wind_the_integral() {
    // P saturates the axis from the first cycle, so the integral may
    // not grow: torque the actuators never delivered must not become
    // a stored correction that overshoots the recovery.
    let mut g = CascadeGains::x500_defaults();
    g.rate_p = [10.0, 0.0, 0.0];
    g.rate_d = [0.0; 3];
    g.rate_i = [2.0, 0.0, 0.0];
    let ctrl = RateController::new(g);
    let mut state = RateLoopState::default();
    let big = [
        RadiansPerSecond(1.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let zero = [RadiansPerSecond(0.0); 3];
    for _ in 0..50 {
        let out = ctrl.step(&mut state, big, zero, 0.001);
        assert!((out[0].0 - 1.0).abs() < 1e-6, "the command holds the clamp");
    }
    // Error removed: the output is the integral alone, which the
    // anti-windup gate kept at zero through the saturated hold.
    let out = ctrl.step(&mut state, zero, zero, 0.001);
    assert!(
        out[0].0.abs() < 1e-6,
        "a saturated hold must not wind up, got {}",
        out[0].0
    );
}

#[test]
fn the_integral_may_shrink_even_while_saturated() {
    // Accumulate a positive integral unsaturated, then jam the error
    // hard negative: the command saturates at -1, and the gate must
    // still let the integral SHRINK — freezing it would carry a stale
    // correction through the transient and bias the recovery.
    let ctrl = i_only([2.0, 0.0, 0.0]);
    let mut state = RateLoopState::default();
    let positive = [
        RadiansPerSecond(0.1),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    let zero = [RadiansPerSecond(0.0); 3];
    for _ in 0..100 {
        ctrl.step(&mut state, positive, zero, 0.001);
    }
    let before = ctrl.step(&mut state, zero, zero, 0.001)[0].0;
    assert!(before > 0.01, "the integral accumulated, got {before}");
    // A P-saturating negative error with P gain restored.
    let mut g = CascadeGains::x500_defaults();
    g.rate_p = [10.0, 0.0, 0.0];
    g.rate_d = [0.0; 3];
    g.rate_i = [2.0, 0.0, 0.0];
    let saturating = RateController::new(g);
    let negative = [
        RadiansPerSecond(-1.0),
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.0),
    ];
    for _ in 0..10 {
        ctrl.step(&mut state, negative, zero, 0.001);
        saturating.step(&mut state, negative, zero, 0.001);
    }
    let after = ctrl.step(&mut state, zero, zero, 0.001)[0].0;
    assert!(
        after < before,
        "the integral must shrink under an opposing saturated error: {before} -> {after}"
    );
}
