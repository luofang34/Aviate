//! Algorithm-invariant numeric limits of the control law.
//!
//! Airframe-tunable limits live in
//! [`CascadeGains`](crate::control::cascade_gains::CascadeGains): they
//! are validated at construction and covered by
//! `ResolvedKernelConfig::canonical_hash`, so lockstep channels cannot
//! disagree on them. The constants here are the other category — fixed
//! properties of the control algorithm itself: model-validity bounds,
//! numeric-safety floors, and mode-gate epsilons. Changing one of them
//! changes the control law, not a tune, so this module lives inside
//! the controller-owned tree that `scripts/check_algorithm_identity.sh`
//! adjudicates against the controller identities.
//!
//! The tuning/invariant classification is executable, not prose:
//! `scripts/check_control_limits.sh` checks every float literal in the
//! production control tree against `cert/control_limits_registry.toml`,
//! so a new behavior-shaping constant must be registered as either a
//! hash-covered `CascadeGains` field or a named invariant here.

use crate::types::Scalar;

/// Standard gravity [m/s²] — the vertical-force scale the velocity
/// loop linearizes against. The hover-trim model assumes thrust equals
/// weight at trim, so acceleration feedforward and tilt-angle
/// conversion both divide by this value. A physical constant, not a
/// knob: no airframe flies under a different gravity.
pub const STANDARD_GRAVITY_MPS2: Scalar = 9.81;

/// Horizontal-acceleration command bound [m/s²] — the tilt-model
/// validity clamp. The velocity loop converts commanded horizontal
/// acceleration to tilt via `atan(a / g)`; at 1 g that is 45°, the
/// edge of the region where the decoupled horizontal/vertical thrust
/// split and the roll·pitch attitude composition stay honest. Derived
/// from gravity, not from airframe authority — the authority knob is
/// `CascadeGains::vel_max_roll_pitch`, which is tuning and
/// hash-covered.
pub const MAX_HORIZONTAL_ACCEL_CMD_MPS2: Scalar = STANDARD_GRAVITY_MPS2;

/// Commanded-collective threshold below which the cascade treats the
/// vehicle as on the ground: axis commands are silenced and loop
/// memory (integrators, derivative history, feedforward priming) is
/// reset. Collective is a `Normalized` command, so this epsilon is
/// scale-free across airframes — any flying multirotor commands far
/// more than 2 %, and a commanded value this low means "no thrust
/// requested", not "very gentle descent". The threshold gates mode
/// logic (whether the loops run and reset), not loop strength, which
/// is why it is an algorithm invariant rather than tuning.
pub const DISARMED_COLLECTIVE_THRESHOLD: Scalar = 0.02;

/// Floor on the tilt-compensation cosine `cos(tilt) = R[2,2]`. The
/// vertical loop divides collective by `cos(tilt)` to hold vertical
/// force constant while tilted; past 60° the divisor would amplify
/// collective without bound and a recovering vehicle would pin its
/// motors on numeric grounds alone. Flooring the cosine at 0.5 caps
/// the amplification at 2× and leaves the residual to the mixer's
/// per-motor clamp. A numeric-safety floor, not a preference.
pub const TILT_COMP_COS_FLOOR: Scalar = 0.5;

// Compile-time bounds. Violating any of these is a build error, which
// is stronger than a runtime test: the constants cannot even compile
// outside their valid domain.
//
// The disarmed threshold must be strictly positive (the gate compares
// with strict `<`, so a zero threshold would never trigger on a zero
// command) and far below any plausible flying collective — an order
// of magnitude under the uncalibrated 0.5 hover default.
const _: () = assert!(DISARMED_COLLECTIVE_THRESHOLD > 0.0);
const _: () = assert!(DISARMED_COLLECTIVE_THRESHOLD <= 0.05);
// The tilt-compensation floor must be a valid cosine; at or below
// zero the collective division would explode or flip sign.
const _: () = assert!(TILT_COMP_COS_FLOOR > 0.0);
const _: () = assert!(TILT_COMP_COS_FLOOR <= 1.0);

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    #[allow(unused_imports)] // atan2 comes from FloatExt in no_std builds
    use crate::types::FloatExt;

    #[test]
    fn standard_gravity_matches_the_physical_constant() {
        assert!((STANDARD_GRAVITY_MPS2 - 9.81).abs() < 1e-6);
    }

    #[test]
    fn horizontal_accel_clamp_bounds_the_tilt_model_to_45_degrees() {
        // The clamp exists to keep `atan(a / g)` inside the region
        // where the decoupled-thrust model is valid; its boundary is
        // exactly 45° of commanded tilt.
        let tilt = MAX_HORIZONTAL_ACCEL_CMD_MPS2.atan2(STANDARD_GRAVITY_MPS2);
        assert!((tilt - core::f32::consts::FRAC_PI_4).abs() < 1e-6);
    }

    #[test]
    fn tilt_comp_floor_caps_amplification_at_two() {
        // The floor bounds the 1/cos(tilt) collective amplification
        // at exactly 2×.
        assert!((1.0 / TILT_COMP_COS_FLOOR - 2.0).abs() < 1e-6);
    }
}
