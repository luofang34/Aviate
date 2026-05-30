//! Cascade step-response tests against LLR-CTL-201/202 bounds.
//!
//! These tests simulate the attitude + rate cascade against a
//! kinematic-surrogate plant (single-axis double integrator with
//! finite torque authority) and assert the closed-loop response
//! against the LLR's overshoot / settle / steady-state bounds.
//! Bounds are taken from `cert/trace/llr.toml`, not picked by
//! eye.
//!
//! Plant model:
//!   ω̇ = torque_norm · K_PLANT_RAD_S2     (rad/s²)
//!   θ̇ = ω
//!
//! `K_PLANT_RAD_S2` is the per-axis angular-acceleration
//! authority of one unit of normalized torque. For the X500 the
//! mixer produces ≈ 1.49 N·m of body torque at full roll (one
//! side at max thrust, the other at zero — i.e. `arm·F_motor =
//! 0.174 m · 8.55 N ≈ 1.49 N·m`) against roll inertia ≈ 0.02
//! kg·m², so `K ≈ 74 rad/s²`. The gz physics test is a separate
//! behavioural verifier (LLR-CTL-202 lists Gazebo XIL as the
//! flight-grade evidence, with this unit harness as the
//! structural witness).
//!
//! Single-axis (roll) simulation — the rate loop receives a
//! zero pitch / yaw setpoint and the surrogate plant is
//! axis-decoupled, so the per-axis LLR claim is isolated from
//! cross-axis dynamics.

use crate::control::attitude::AttitudeController;
use crate::control::cascade_gains::CascadeGains;
use crate::control::rate::{RateController, RateLoopState};
use crate::math::{Quaternion, Vector3};
use crate::types::RadiansPerSecond;

const DT_SEC: f32 = 0.001;
const K_PLANT_RAD_S2: f32 = 74.0;

/// LLR-CTL-202: attitude-loop step response from a 10° roll
/// setpoint step. Overshoot ≤ 30 %, settle (±5 %) ≤ 1.0 s.
#[test]
fn attitude_step_response_meets_llr_ctl_202() {
    let gains = CascadeGains::x500_defaults();
    gains.validate().unwrap();
    let att = AttitudeController::new(gains.att_p);
    let rate = RateController::new(gains);
    let mut rate_state = RateLoopState::default();

    let setpoint_deg = 10.0_f32;
    let setpoint_rad = setpoint_deg.to_radians();
    let setpoint_quat = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), setpoint_rad);

    let band = setpoint_deg * 0.05;
    let lo = setpoint_deg - band;
    let hi = setpoint_deg + band;

    let n_steps = 2500_usize; // 2.5 s observation window.
    let mut theta: f32 = 0.0;
    let mut omega: f32 = 0.0;
    let mut peak_deg: f32 = 0.0;
    let mut last_exit_idx: usize = 0;
    let mut final_deg: f32 = 0.0;

    for i in 0..n_steps {
        let current_quat = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), theta);
        let rate_sp = att.step(&setpoint_quat, &current_quat);
        let cur_rate = [
            RadiansPerSecond(omega),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ];
        let torque = rate.step(&mut rate_state, rate_sp, cur_rate, DT_SEC);
        let alpha = torque[0].0 * K_PLANT_RAD_S2;
        omega += alpha * DT_SEC;
        theta += omega * DT_SEC;

        let theta_deg = theta.to_degrees();
        if theta_deg > peak_deg {
            peak_deg = theta_deg;
        }
        if !(lo..=hi).contains(&theta_deg) {
            last_exit_idx = i;
        }
        final_deg = theta_deg;
    }

    let overshoot_pct = (peak_deg - setpoint_deg) / setpoint_deg * 100.0;
    assert!(
        overshoot_pct <= 30.0,
        "LLR-CTL-202 violation: overshoot {overshoot_pct:.1}% exceeds 30%"
    );

    let settle_s = (last_exit_idx + 1) as f32 * DT_SEC;
    assert!(
        settle_s <= 1.0,
        "LLR-CTL-202 violation: settle (±5%) {settle_s:.2}s exceeds 1.0s\nfinal angle {final_deg:.2}°, band [{lo:.2}, {hi:.2}]"
    );

    assert!(
        (lo..=hi).contains(&final_deg),
        "LLR-CTL-202 violation: final angle {final_deg:.2}° outside ±5% band [{lo:.2}, {hi:.2}]"
    );
}

/// LLR-CTL-201: rate-loop tracking under a ramp-and-hold rate
/// command (±60 °/s). Steady-state error ≤ 0.5 °/s, sampled over
/// the last 200 ms of the hold.
#[test]
fn rate_loop_tracks_setpoint_meets_llr_ctl_201() {
    let gains = CascadeGains::x500_defaults();
    let rate = RateController::new(gains);

    for &target_dps in &[60.0_f32, -60.0] {
        let mut rate_state = RateLoopState::default();
        let mut omega: f32 = 0.0;
        let n_steps = 1500_usize; // 1.5 s
        let ramp_n = 500_usize; // ramp in 0.5 s
        let target_rad = target_dps.to_radians();
        let mut max_ss_err_dps: f32 = 0.0;

        for i in 0..n_steps {
            let sp_value = if i < ramp_n {
                target_rad * (i as f32 / ramp_n as f32)
            } else {
                target_rad
            };
            let sp = [
                RadiansPerSecond(sp_value),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ];
            let cur = [
                RadiansPerSecond(omega),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ];
            let torque = rate.step(&mut rate_state, sp, cur, DT_SEC);
            let alpha = torque[0].0 * K_PLANT_RAD_S2;
            omega += alpha * DT_SEC;
            // Last 200 ms = last 200 samples at 1 kHz.
            if i >= n_steps - 200 {
                let err_dps = (sp_value - omega).to_degrees().abs();
                if err_dps > max_ss_err_dps {
                    max_ss_err_dps = err_dps;
                }
            }
        }

        assert!(
            max_ss_err_dps <= 0.5,
            "LLR-CTL-201 violation at {target_dps} °/s: steady-state error {max_ss_err_dps:.3} °/s exceeds 0.5 °/s"
        );
    }
}

/// LLR-CTL-205: cascade time-scale separation. The
/// inner-loop (rate) bandwidth must be at least 5× the
/// outer-loop (attitude) bandwidth for the cascade's
/// stability margins to be analyzable via single-loop
/// classical-control techniques — the structural
/// precondition for the LLR-CTL-202 step-response bounds.
/// Below 5:1, the outer loop "sees" the inner loop's
/// dynamics directly and the cascade's pole positions
/// migrate unfavourably under tuning perturbations.
///
/// Sensitivity-analysis derivation (per-axis, roll example):
/// closing the rate loop around the plant `K / s` gives a
/// closed-loop transfer of `1 / (1 + s/(K · rate_p))`. Its
/// 3 dB bandwidth is `ω_inner = K · rate_p`. The outer
/// (attitude) loop sees the closed inner loop in series
/// with an integrator (gyro → angle), and its open-loop
/// crossover sits at `ω_outer ≈ att_p`. The separation
/// ratio is therefore `(K · rate_p) / att_p`.
///
/// Per-axis assertion: ratio ≥ 5. Yaw is skipped where
/// `rate_p[2] = 0` (some airframes ride on aerodynamic
/// stability), since the inner loop is then unmodelled.
#[test]
fn cascade_time_scale_separation_at_least_five_to_one() {
    let gains = CascadeGains::x500_defaults();
    gains.validate().unwrap();
    let min_ratio: f32 = 5.0;
    for axis in 0..3 {
        if gains.rate_p[axis] == 0.0 {
            continue; // open-loop yaw, no inner-loop pole to separate.
        }
        let ratio = (K_PLANT_RAD_S2 * gains.rate_p[axis]) / gains.att_p[axis];
        assert!(
            ratio >= min_ratio,
            "LLR-CTL-205 violation: axis {axis} time-scale separation ratio \
             (K·rate_p/att_p) = {ratio:.2}× — must be ≥ {min_ratio}× for the \
             cascade to be classically analyzable. K = {K_PLANT_RAD_S2}, \
             rate_p[{axis}] = {}, att_p[{axis}] = {}",
            gains.rate_p[axis],
            gains.att_p[axis]
        );
    }
}

/// Sanity test that the simulation harness itself behaves
/// physically: with zero controller gains everywhere, the
/// vehicle never moves. Guards against the more elaborate
/// tests above passing because the surrogate plant somehow
/// integrated motion not driven by the controller output.
#[test]
fn zero_gains_zero_motion() {
    let mut gains = CascadeGains::x500_defaults();
    gains.att_p = [0.0; 3];
    gains.rate_p = [0.0; 3];
    gains.rate_d = [0.0; 3];
    let att = AttitudeController::new(gains.att_p);
    let rate = RateController::new(gains);
    let mut rate_state = RateLoopState::default();

    let setpoint_quat =
        Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 10.0_f32.to_radians());
    let mut theta: f32 = 0.0;
    let mut omega: f32 = 0.0;
    let mut max_abs_deg: f32 = 0.0;

    for _ in 0..500 {
        let current_quat = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), theta);
        let rate_sp = att.step(&setpoint_quat, &current_quat);
        let cur = [
            RadiansPerSecond(omega),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ];
        let torque = rate.step(&mut rate_state, rate_sp, cur, DT_SEC);
        let alpha = torque[0].0 * K_PLANT_RAD_S2;
        omega += alpha * DT_SEC;
        theta += omega * DT_SEC;
        let d = theta.to_degrees().abs();
        if d > max_abs_deg {
            max_abs_deg = d;
        }
    }
    assert!(
        max_abs_deg < 1e-6,
        "surrogate plant integrated motion with zero controller output: max |angle| = {max_abs_deg:.3e}°",
    );
}
