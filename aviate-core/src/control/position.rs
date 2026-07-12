use crate::math::Vector3;
#[allow(unused_imports)] // FloatExt provides sqrt/abs/signum in no_std builds
use crate::types::{FloatExt, Meters, MetersPerSecond, Scalar};

/// Kinematically-shaped position controller.
///
/// Converts position error into a velocity setpoint such that the
/// vehicle can always decelerate to zero velocity exactly at the
/// setpoint within its brake-authority envelope. The output is the
/// canonical "sqrt-controller" used by ArduPilot's `AC_PosControl`
/// and PX4's position trajectory generator:
///
/// ```text
///                        ┌ p · err                              for |err| ≤ d_lin
/// vel_sp(err, p, a) =   ─┤
///                        └ sign(err) · √( 2·a · (|err| − d_lin/2) )  otherwise
///
///   d_lin = a / p²
/// ```
///
/// * `p` (gain) sets the slope near zero — acts like a plain P
///   controller for small errors.
/// * `a` (`accel_limits`) caps the deceleration profile so the
///   velocity loop can always track the requested vel_sp with the
///   thrust authority available. Pick *below* the cascade's
///   measured brake authority (max-thrust net upward acceleration
///   for Z, max-tilt-angle horizontal acceleration for X/Y).
/// * `vel_cap` (`vel_caps`) is an absolute speed clamp; the sqrt
///   shape still ensures the vehicle can stop without overshoot
///   when commanded between zero and the cap.
///
/// I-term is **not** the right fix for kinematic overshoot. At
/// max thrust the velocity loop is already saturated, and
/// integrating error while saturated only winds the controller
/// up into the opposite overshoot. Energy management belongs in
/// the velocity setpoint generation, not in the feedback path.
#[derive(Clone, Debug)]
pub struct PositionController {
    /// Per-axis P gain (slope near zero).
    pub gains: [Scalar; 3],
    /// Per-axis deceleration limit `a` in m/s².
    pub accel_limits: [Scalar; 3],
    /// Per-axis absolute speed cap in m/s.
    pub vel_caps: [Scalar; 3],
}

impl PositionController {
    /// Constructor with sensible defaults for a multirotor like
    /// the X500: horizontal `(p=0.3, a=1.5, vel=2.0)`, vertical
    /// `(p=0.6, a=1.5, vel=3.0)`. The Z `a=1.5` is below the
    /// X500's measured ~2.9 m/s² max upward braking authority at
    /// thrust=1.0; `vel_cap_z = 3.0` is above `a/p = 2.5` so the
    /// sqrt branch actually engages during cruise descent.
    /// Explicit per-axis tuning. Reach for this when the airframe's
    /// brake authority differs from the X500 defaults (heavier
    /// payload, lower max thrust, etc).
    pub fn with_limits(
        gains: [Scalar; 3],
        accel_limits: [Scalar; 3],
        vel_caps: [Scalar; 3],
    ) -> Self {
        Self {
            gains,
            accel_limits,
            vel_caps,
        }
    }

    pub fn step(
        &self,
        setpoint: Vector3<Meters>,
        current: Vector3<Meters>,
    ) -> Vector3<MetersPerSecond> {
        let error = Vector3 {
            x: setpoint.x.0 - current.x.0,
            y: setpoint.y.0 - current.y.0,
            z: setpoint.z.0 - current.z.0,
        };
        Vector3 {
            x: MetersPerSecond(sqrt_shape(
                error.x,
                self.gains[0],
                self.accel_limits[0],
                self.vel_caps[0],
            )),
            y: MetersPerSecond(sqrt_shape(
                error.y,
                self.gains[1],
                self.accel_limits[1],
                self.vel_caps[1],
            )),
            z: MetersPerSecond(sqrt_shape(
                error.z,
                self.gains[2],
                self.accel_limits[2],
                self.vel_caps[2],
            )),
        }
    }
}

/// Square-root deceleration profile with a linear region near zero.
///
/// Continuous at `|err| = a/p²` because the linear segment is
/// offset by `a/(2·p²)` from the origin — see PX4 / ArduPilot for
/// the same construction. Falls back to plain P when either `a`
/// or `p` is non-positive so the caller can disable shaping
/// per-axis without a separate code path.
#[inline]
pub(crate) fn sqrt_shape(err: Scalar, p: Scalar, a_max: Scalar, vel_cap: Scalar) -> Scalar {
    if a_max <= 0.0 || p <= 0.0 {
        return (err * p).clamp(-vel_cap, vel_cap);
    }
    let linear_dist = a_max / (p * p);
    let abs_err = err.abs();
    let raw = if abs_err > linear_dist {
        let inner = 2.0 * a_max * (abs_err - 0.5 * linear_dist);
        // `inner` is positive because the branch guarantees
        // `abs_err > linear_dist > linear_dist/2`, so the sqrt is
        // always well-defined here.
        let mag = inner.sqrt();
        if err >= 0.0 {
            mag
        } else {
            -mag
        }
    } else {
        p * err
    };
    raw.clamp(-vel_cap, vel_cap)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Default-ish multirotor parameters used across the tests.
    const P: Scalar = 0.6;
    const A: Scalar = 1.5;
    /// `cap` chosen larger than `A/P` so the sqrt branch is
    /// reachable for at least some inputs.
    const CAP: Scalar = 10.0;

    #[test]
    fn linear_region_matches_p_times_err() {
        // For |err| ≤ a/p², output equals p · err exactly.
        let d_lin = A / (P * P);
        for err in [-d_lin * 0.5, -0.1, 0.0, 0.1, d_lin * 0.5, d_lin - 1e-6] {
            let v = sqrt_shape(err, P, A, CAP);
            let want = P * err;
            assert!(
                (v - want).abs() < 1e-6,
                "linear branch mismatch at err={err}: got {v}, want {want}"
            );
        }
    }

    #[test]
    fn sqrt_region_matches_closed_form() {
        // For |err| > a/p², output equals sign(err)·√(2·a·(|err|−d/2)).
        let d_lin = A / (P * P);
        for err_mag in [d_lin + 0.01, d_lin * 2.0, d_lin * 5.0] {
            for sign in [-1.0_f32, 1.0] {
                let err = sign * err_mag;
                let v = sqrt_shape(err, P, A, CAP);
                let want = sign * (2.0 * A * (err_mag - 0.5 * d_lin)).sqrt();
                assert!(
                    (v - want).abs() < 1e-5,
                    "sqrt branch mismatch at err={err}: got {v}, want {want}"
                );
            }
        }
    }

    #[test]
    fn continuous_at_crossover() {
        // Linear and sqrt branches agree at |err| = a/p².
        let d_lin = A / (P * P);
        let v_below = sqrt_shape(d_lin - 1e-6, P, A, CAP);
        let v_above = sqrt_shape(d_lin + 1e-6, P, A, CAP);
        assert!(
            (v_below - v_above).abs() < 1e-3,
            "discontinuity: below={v_below}, above={v_above}"
        );
        // Both branches must give p·d_lin = a/p at the crossover.
        let predicted = A / P;
        assert!((v_below - predicted).abs() < 1e-3);
        assert!((v_above - predicted).abs() < 1e-3);
    }

    #[test]
    fn slope_continuous_at_crossover() {
        // d/d(err) of `p·err` is `p`. d/d(err) of
        // `√(2·a·(err−d/2))` evaluated at err=d is
        // `a / √(2·a·(d/2)) = a / √(a²/p²) = p`. Same slope; the
        // function is C¹. Verify by finite difference.
        let d_lin = A / (P * P);
        let h = 1e-4;
        let slope_below =
            (sqrt_shape(d_lin - h * 0.5, P, A, CAP) - sqrt_shape(d_lin - 1.5 * h, P, A, CAP)) / h;
        let slope_above =
            (sqrt_shape(d_lin + 1.5 * h, P, A, CAP) - sqrt_shape(d_lin + h * 0.5, P, A, CAP)) / h;
        assert!(
            (slope_below - slope_above).abs() < 5e-3,
            "slope discontinuity at crossover: below={slope_below}, above={slope_above}"
        );
        assert!((slope_below - P).abs() < 5e-3);
    }

    #[test]
    fn output_bounded_by_vel_cap() {
        let cap = 2.0;
        for err in [-1000.0, -10.0, -1.0, 0.0, 1.0, 10.0, 1000.0_f32] {
            let v = sqrt_shape(err, P, A, cap);
            assert!(v.abs() <= cap + 1e-6, "cap violated at err={err}: v={v}");
        }
    }

    #[test]
    fn output_sign_matches_err_sign() {
        for err in [-100.0, -5.0, -0.5, 0.5, 5.0, 100.0_f32] {
            let v = sqrt_shape(err, P, A, CAP);
            assert!(v.signum() == err.signum(), "sign mismatch err={err} v={v}");
        }
    }

    #[test]
    fn output_monotonic_in_err_magnitude() {
        let cap = 10.0;
        let mut prev = sqrt_shape(0.1, P, A, cap);
        for err in [0.5, 1.0, 2.0, 5.0, 10.0, 50.0_f32] {
            let v = sqrt_shape(err, P, A, cap);
            assert!(v + 1e-6 >= prev, "non-monotonic at err={err}: {prev} → {v}");
            prev = v;
            // capped or growing; never decreasing
        }
    }

    #[test]
    fn zero_err_zero_out() {
        assert_eq!(sqrt_shape(0.0, P, A, CAP), 0.0);
    }

    #[test]
    fn non_positive_params_fall_back_to_linear() {
        // a ≤ 0 disables shaping; output is plain p·err clamped.
        assert!((sqrt_shape(1.0, P, 0.0, CAP) - P).abs() < 1e-6);
        assert!((sqrt_shape(1.0, P, -1.0, CAP) - P).abs() < 1e-6);
        // p ≤ 0: output is 0 for any err (still clamped).
        assert_eq!(sqrt_shape(1.0, 0.0, A, CAP), 0.0);
    }

    #[test]
    fn brake_distance_matches_accel_limit() {
        // At |err| = cap²/(2·a) + d_lin/2 the sqrt branch outputs
        // exactly `cap`. This is the predicted stopping distance:
        // a body braking at `a` from `cap` to 0 covers `cap²/(2·a)`,
        // plus the linear-region offset of `d_lin/2`.
        let p = 0.5;
        let a = 1.5;
        let cap = 4.0;
        let d_lin = a / (p * p);
        let predicted = cap * cap / (2.0 * a) + d_lin / 2.0;
        assert!(predicted > d_lin, "test params must enter sqrt branch");
        let v = sqrt_shape(predicted, p, a, cap * 10.0);
        assert!(
            (v - cap).abs() < 1e-3,
            "brake-distance prediction off: v={v}"
        );
    }

    #[test]
    fn x500_tuning_is_kinematically_consistent() {
        // The X500 brake authority at thrust=1.0 (net upward
        // acceleration) is ~2.9 m/s² with hover trim 0.77:
        //   a_brake = ((1 - hover) / hover) · g = (0.23 / 0.77) · 9.81 ≈ 2.93
        // Our `accel_limits[2] = 1.5` must stay below that with
        // margin so the velocity loop can always track the
        // requested deceleration without saturating.
        let hover = 0.77_f32;
        let g = 9.81_f32;
        let measured = ((1.0 - hover) / hover) * g;
        let configured = 1.5_f32;
        assert!(
            configured < measured,
            "configured a_max_z ({configured}) ≥ measured brake authority ({measured})"
        );
        // And the cap must be larger than a/p for the sqrt branch
        // to actually engage; otherwise the linear region clamps
        // before the sqrt does anything useful.
        let p_z = 0.6_f32;
        let cap_z = 3.0_f32;
        let min_cap = configured / p_z;
        assert!(
            cap_z > min_cap,
            "vel_cap_z ({cap_z}) ≤ a/p ({min_cap}); sqrt branch never engages"
        );
    }

    #[test]
    fn touchdown_velocity_at_ground_threshold() {
        // For a vehicle landing with the X500 defaults
        // (target z=0, vehicle 30 cm above ground), the
        // commanded vertical velocity must satisfy the
        // `touchdown_velocity ≤ 1 m/s` criterion. Within the
        // linear region (err < a/p²), vel_sp = p · err, so
        // vel_sp at err=0.30m = 0.18 m/s. Well under 1 m/s.
        let ctrl =
            PositionController::with_limits([0.3, 0.3, 0.6], [1.5, 1.5, 1.5], [2.0, 2.0, 3.0]);
        let setpoint = Vector3 {
            x: Meters(0.0),
            y: Meters(0.0),
            z: Meters(0.0),
        };
        let current = Vector3 {
            x: Meters(0.0),
            y: Meters(0.0),
            z: Meters(-0.30),
        };
        let v = ctrl.step(setpoint, current);
        assert!(
            v.z.0 < 1.0,
            "commanded touchdown vel {} ≥ 1 m/s — controller can't satisfy the criterion",
            v.z.0
        );
    }
}
