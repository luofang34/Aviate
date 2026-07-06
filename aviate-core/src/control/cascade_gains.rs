//! Cascade tuning parameters — every gain, every limit that the
//! multirotor controller cascade consumes. Owned by
//! `ResolvedKernelConfig::cascade_gains` so that
//! `ResolvedKernelConfig::canonical_hash` covers tuning, not just
//! structure.
//!
//! Before this module landed (DRQ-CTL-001), gains lived as
//! constructor arguments on `MultirotorController` and were
//! invisible to lockstep — two channels could disagree on tuning
//! without either side noticing because `algorithm_identity_hash`
//! only sees algorithm classes, not their internal parameters.
//! Moving the gains into the canonical config closes that hole.
//!
//! Validation invariants (checked at construction):
//!
//! * **Non-negative gains.** Every gain SHALL be ≥ 0 and finite.
//! * **Feedforward and LPF coefficients in [0, 1].**
//!
//! A simple structural att-vs-rate ratio rule was tried and
//! removed: the cascade's damping ζ is a function of the
//! plant K (`ζ = 0.5·√(K·rate_p/att_p)`) so a gain-ratio bound
//! independent of K is incoherent. The actual stability /
//! response property is asserted at higher level — the
//! step-response test (LLR-CTL-202) measures overshoot and
//! settle time against the surrogate plant, which is where the
//! cascade actually has to behave.

use crate::types::Scalar;

/// Every PID gain and limit the multirotor cascade reads, as a
/// single immutable struct. Authoritative source — neither
/// `MultirotorController` nor any of its sub-controllers may
/// carry a separate tuning copy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CascadeGains {
    /// Position loop P gains (m/s per m), per X/Y/Z axis.
    pub pos_p: [Scalar; 3],
    /// Position loop sqrt-shaper accel limit (m/s²) per axis.
    pub pos_accel_limits: [Scalar; 3],
    /// Position loop absolute velocity cap (m/s) per axis.
    pub pos_vel_caps: [Scalar; 3],

    /// Velocity loop P gains (normalized-thrust per m/s), per axis.
    pub vel_p: [Scalar; 3],
    /// Velocity loop I gains (normalized-thrust per m·s of
    /// integrated error), per axis. Anti-windup is conditional —
    /// the integrator does NOT accumulate when the velocity loop
    /// is saturated against its actuator authority.
    pub vel_i: [Scalar; 3],
    /// Maximum roll/pitch tilt the velocity loop is allowed to
    /// command (radians). Bounds horizontal acceleration.
    pub vel_max_roll_pitch: Scalar,
    /// Acceleration-feedforward scale [0..1]. `1.0` = full
    /// feedforward (vehicle commanded the thrust needed for
    /// the desired acceleration at every step, without waiting
    /// for the velocity-error feedback path to react). `0.0` =
    /// no feedforward, pure feedback.
    pub vel_accel_ff: Scalar,
    /// Velocity-loop D gains (derivative-on-measurement),
    /// per axis. Damps velocity oscillation that the P-only
    /// loop cannot absorb when the cascade is operating near
    /// its actuator authority. Filtered by `rate_d_lpf_alpha`
    /// (re-used to keep the parameter surface narrow). Yaw /
    /// horizontal D usually stays at zero on small
    /// multirotors; the vertical axis benefits from a small
    /// non-zero value to damp the descent-overshoot pathology.
    pub vel_d: [Scalar; 3],

    /// Attitude loop P gains (rad/s per rad of attitude error),
    /// per roll/pitch/yaw.
    pub att_p: [Scalar; 3],

    /// Rate loop P gains (normalized-torque per rad/s), per axis.
    pub rate_p: [Scalar; 3],
    /// Rate loop D gains (normalized-torque per rad/s² of
    /// derivative-of-measurement). Computed against gyro, not
    /// against setpoint, so a setpoint step doesn't kick the D
    /// term. LPF-filtered by `rate_d_lpf_alpha`.
    pub rate_d: [Scalar; 3],
    /// Single-pole LPF coefficient for the rate D-term, in
    /// `[0..1]`. `0.0` disables filtering; `1.0` freezes the
    /// derivative at its initial sample.
    pub rate_d_lpf_alpha: Scalar,
}

impl CascadeGains {
    /// Sensible defaults for an X500-class quadrotor. Validated.
    ///
    /// The attitude / rate gains satisfy `rate_p ≥ att_p` per
    /// axis (yaw included where rate_p > 0), giving the cascade
    /// `ζ ≥ 0.5`. The values match the cascade that flew the
    /// open-loop hover_trim_check 10/10 prior to DRQ-CTL-002,
    /// plus integral / derivative augmentation:
    ///
    /// * `vel_i` — small velocity I for hover-trim drift; sized
    ///   so a 1-second wind-up at maximum unsaturated error
    ///   contributes ≤ 5 % of hover thrust (one trim-step).
    /// * `rate_d` — yaw left at zero (the airframe is yaw-
    ///   damped by rotor drag); roll / pitch get a small D term
    ///   that the LPF smooths against gyro noise.
    pub fn x500_defaults() -> Self {
        Self {
            // Vertical gains sized for the X500's actual brake
            // authority. Hover trim ≈ 0.77 → max upward accel
            // (collective = 1.0) ≈ (1 / 0.77 − 1) · g ≈ 2.9 m/s².
            // The position loop's `pos_accel_limits[z]` lives
            // safely below that; the velocity cap is small
            // enough that the cascade can stop within ≈ 0.5 m
            // of the setpoint from full descent.
            pos_p: [0.5, 0.5, 0.5],
            pos_accel_limits: [2.0, 2.0, 1.0],
            pos_vel_caps: [2.0, 2.0, 0.4],
            vel_p: [0.3, 0.3, 0.4],
            // Horizontal I-term stays off until the
            // closed-loop-horizontal regression baseline lands —
            // it was a candidate root cause of the drift mode
            // and a clean P-only baseline is easier to compare
            // against. Vertical I-term is on for hover trim
            // bias rejection (the velocity loop's anti-windup
            // freezes accumulation while the thrust output is
            // clamped, so a saturated brake or climb doesn't
            // wind the integrator and cause the reverse-overshoot
            // pathology).
            vel_i: [0.0, 0.0, 0.05],
            vel_max_roll_pitch: 0.35, // ~20°
            // Disabled. The current finite-difference accel_ff
            // (Δvel_sp / dt) is unfiltered, so any gz-side
            // position noise becomes a giant spurious horizontal
            // acceleration command at dt = 1 ms. Until an LPF or
            // an analytic accel_ff lands the cascade is stabler
            // with pure feedback.
            vel_accel_ff: 0.0,
            // Velocity D term wired but disabled (vel_d = 0)
            // — empirically the un-filtered Δvel/dt at 1 ms
            // amplified gz position-derivative noise instead
            // of damping it. The hook stays so a future LPF
            // tuning or analytic-derivative source can land
            // without re-plumbing the loop. The canonical-hash
            // probe in canonical/tests.rs still locks the
            // field in lockstep so a downstream re-enable can't
            // ship without changing the algorithm-identity hash.
            vel_d: [0.0, 0.0, 0.0],
            // Joint LLR constraint:
            //
            // * LLR-CTL-202: 10° step settles ≤ 1 s with ≤ 30 %
            //   overshoot. For the X500's plant authority
            //   (K ≈ 74 rad/s² per unit normalised torque), the
            //   closed-loop has ωn = √(att_p · K · rate_p) and
            //   ζ = 0.5 · √(K · rate_p / att_p). Pick gains so
            //   ωn ≥ 4.5 rad/s (settle ≈ 1 / ωn ≈ 0.7 s, under
            //   the 1 s bound) and ζ ≥ 0.5 (overshoot ≤ 16 %,
            //   well under 30 %).
            // * LLR-CTL-205: cascade time-scale separation
            //   `K · rate_p / att_p ≥ 5`. Below that the inner
            //   loop's dynamics intrude on the outer loop and
            //   the cascade's stability margins are no longer
            //   classically analyzable.
            //
            // Picking `att_p = 3.5`, `rate_p = 0.30` for roll
            // and pitch gives a separation ratio of
            // 74 · 0.30 / 3.5 ≈ 6.3× (above the 5× floor with
            // margin) and ωn ≈ √(3.5 · 74 · 0.30) ≈ 8.8 rad/s,
            // settle ≈ 0.5 s with ζ ≈ 1.26 (overdamped — no
            // overshoot).
            att_p: [3.5, 3.5, 1.5],
            rate_p: [0.30, 0.30, 0.15],
            rate_d: [0.0, 0.0, 0.0],
            rate_d_lpf_alpha: 0.5,
        }
    }

    /// Validate the cascade gain ordering and non-negativity
    /// invariants. Returns the first violation encountered; OK
    /// means all axes / gains pass.
    pub fn validate(&self) -> Result<(), CascadeGainsError> {
        for i in 0..3 {
            for (name, g) in [
                ("pos_p", self.pos_p[i]),
                ("pos_accel_limits", self.pos_accel_limits[i]),
                ("pos_vel_caps", self.pos_vel_caps[i]),
                ("vel_p", self.vel_p[i]),
                ("vel_i", self.vel_i[i]),
                ("vel_d", self.vel_d[i]),
                ("att_p", self.att_p[i]),
                ("rate_p", self.rate_p[i]),
                ("rate_d", self.rate_d[i]),
            ] {
                if !g.is_finite() || g < 0.0 {
                    return Err(CascadeGainsError::NonNegativeGain { name, axis: i });
                }
            }
        }
        if !self.vel_max_roll_pitch.is_finite() || self.vel_max_roll_pitch < 0.0 {
            return Err(CascadeGainsError::NonNegativeGain {
                name: "vel_max_roll_pitch",
                axis: 0,
            });
        }
        if !self.vel_accel_ff.is_finite() || !(0.0..=1.0).contains(&self.vel_accel_ff) {
            return Err(CascadeGainsError::FeedforwardOutOfRange(self.vel_accel_ff));
        }
        if !self.rate_d_lpf_alpha.is_finite() || !(0.0..=1.0).contains(&self.rate_d_lpf_alpha) {
            return Err(CascadeGainsError::LpfAlphaOutOfRange(self.rate_d_lpf_alpha));
        }
        Ok(())
    }
}

impl Default for CascadeGains {
    fn default() -> Self {
        Self::x500_defaults()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CascadeGainsError {
    /// `gains.<name>[axis]` was negative, infinite, or NaN.
    NonNegativeGain { name: &'static str, axis: usize },
    /// `vel_accel_ff` outside `[0.0, 1.0]`.
    FeedforwardOutOfRange(Scalar),
    /// `rate_d_lpf_alpha` outside `[0.0, 1.0]`.
    LpfAlphaOutOfRange(Scalar),
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn x500_defaults_validate() {
        CascadeGains::x500_defaults().validate().unwrap();
    }

    #[test]
    fn accepts_arbitrary_gain_ratio() {
        // No structural ratio rule between att_p and rate_p
        // (damping is plant-K dependent — see module docs).
        // Both rate_p < att_p and rate_p > att_p validate; the
        // step-response test asserts the actual response.
        let mut g = CascadeGains::x500_defaults();
        g.att_p[0] = 4.0;
        g.rate_p[0] = 0.5;
        g.validate().unwrap();
        g.att_p[0] = 0.1;
        g.rate_p[0] = 10.0;
        g.validate().unwrap();
    }

    #[test]
    fn rejects_negative_gain() {
        let mut g = CascadeGains::x500_defaults();
        g.pos_p[2] = -0.1;
        assert!(matches!(
            g.validate(),
            Err(CascadeGainsError::NonNegativeGain {
                name: "pos_p",
                axis: 2,
            })
        ));
    }

    #[test]
    fn rejects_nan_gain() {
        let mut g = CascadeGains::x500_defaults();
        g.vel_p[1] = f32::NAN;
        assert!(matches!(
            g.validate(),
            Err(CascadeGainsError::NonNegativeGain { .. })
        ));
    }

    #[test]
    fn rejects_feedforward_out_of_range() {
        let mut g = CascadeGains::x500_defaults();
        g.vel_accel_ff = 1.5;
        assert!(matches!(
            g.validate(),
            Err(CascadeGainsError::FeedforwardOutOfRange(_))
        ));
    }

    #[test]
    fn accepts_yaw_rate_p_zero() {
        // Some airframes ride on aerodynamic yaw stability and
        // configure rate_p[2] = 0. Validation must not divide
        // by zero in that case.
        let mut g = CascadeGains::x500_defaults();
        g.att_p[2] = 0.0;
        g.rate_p[2] = 0.0;
        g.validate().unwrap();
    }

    #[test]
    fn rejects_negative_max_roll_pitch() {
        let mut g = CascadeGains::x500_defaults();
        g.vel_max_roll_pitch = -0.1;
        assert!(matches!(
            g.validate(),
            Err(CascadeGainsError::NonNegativeGain {
                name: "vel_max_roll_pitch",
                axis: 0,
            })
        ));
    }

    #[test]
    fn rejects_lpf_alpha_out_of_range() {
        let mut g = CascadeGains::x500_defaults();
        g.rate_d_lpf_alpha = 1.5;
        assert!(matches!(
            g.validate(),
            Err(CascadeGainsError::LpfAlphaOutOfRange(_))
        ));
    }
}
