//! Actuation contract carried by `ResolvedKernelConfig` (#140): the
//! registered mixer geometry the configuration was resolved for, and
//! the actuator curve that converts the cascade's force-domain
//! collective into the boundary command.
//!
//! Both are DATA declarations, not code selection: a preset TOML can
//! never supply an `ALGORITHM_ID` or pick a Rust type. The app maps
//! these variants onto its compiled mixer/adapter types; the variants
//! live here so `canonical_hash` covers them and two lockstep
//! channels cannot disagree about geometry or curve silently.

#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::{FloatExt, Normalized, NormalizedThrust};

/// Registered mixer geometries a resolved configuration may declare.
/// Mirrors the compiled mixer families in `crate::mixer`; the mixer
/// TYPE identity is separately witnessed by
/// `KernelPipeline::algorithm_identity_hash` — this field is the
/// configuration-side declaration the canonical hash folds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MixerGeometry {
    /// Generic quad-X (CW on the FR+RL diagonal).
    QuadX,
    /// PX4-gazebo-models X500 pattern (CW on the FL+RR diagonal) —
    /// opposite yaw signs from [`MixerGeometry::QuadX`].
    QuadXX500,
}

impl MixerGeometry {
    /// Motor count this geometry drives.
    pub fn motor_count(self) -> u8 {
        match self {
            MixerGeometry::QuadX | MixerGeometry::QuadXX500 => 4,
        }
    }
}

/// Plant curve between the cascade's force-domain collective
/// ([`NormalizedThrust`]) and the boundary actuator command
/// (rotor-speed / PWM fraction).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorCurveKind {
    /// Thrust proportional to the boundary command: the boundary
    /// command IS the thrust fraction (direct-force plants, ideal
    /// thrust-controlled ESCs).
    Linear,
    /// Thrust proportional to boundary command² (rotor-speed
    /// commands into quadratic rotors — the gz
    /// `MulticopterMotorModel`, most ESC+prop stacks): the boundary
    /// command is `sqrt(thrust)`.
    QuadraticRotor,
}

impl ActuatorCurveKind {
    /// Convert a force-domain command into the boundary actuator
    /// command. Applied EXACTLY ONCE, at the board/simulator edge —
    /// never inside the controller or mixer, which reason purely in
    /// force (#140). Input is clamped to `[0, 1]`; a non-finite
    /// input maps to zero output (the safe side: no thrust rather
    /// than full thrust from a NaN).
    pub fn boundary_command(self, thrust: NormalizedThrust) -> Normalized {
        let t = if thrust.0.is_finite() {
            thrust.0.clamp(0.0, 1.0)
        } else {
            0.0
        };
        match self {
            ActuatorCurveKind::Linear => Normalized(t),
            ActuatorCurveKind::QuadraticRotor => Normalized(t.sqrt()),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn quadratic_curve_maps_force_to_speed_fraction() {
        // #140 guardrail: force 0 → speed 0, 0.25 → 0.5, 1 → 1.
        let c = ActuatorCurveKind::QuadraticRotor;
        assert_eq!(c.boundary_command(NormalizedThrust(0.0)).0, 0.0);
        assert!((c.boundary_command(NormalizedThrust(0.25)).0 - 0.5).abs() < 1e-6);
        assert!((c.boundary_command(NormalizedThrust(1.0)).0 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn linear_curve_is_identity_on_the_unit_interval() {
        let c = ActuatorCurveKind::Linear;
        for t in [0.0, 0.25, 0.5929, 1.0] {
            assert_eq!(c.boundary_command(NormalizedThrust(t)).0, t);
        }
    }

    #[test]
    fn out_of_range_and_non_finite_inputs_are_safe() {
        // Finite out-of-range clamps; ANY non-finite input (NaN or
        // ±Inf — both mean an upstream numeric fault) maps to zero
        // thrust, never to full thrust.
        for c in [ActuatorCurveKind::Linear, ActuatorCurveKind::QuadraticRotor] {
            assert_eq!(c.boundary_command(NormalizedThrust(-0.5)).0, 0.0);
            assert_eq!(c.boundary_command(NormalizedThrust(2.0)).0, 1.0);
            assert_eq!(c.boundary_command(NormalizedThrust(f32::NAN)).0, 0.0);
            assert_eq!(c.boundary_command(NormalizedThrust(f32::INFINITY)).0, 0.0);
        }
    }

    #[test]
    fn v1_speed_seed_squared_round_trips_through_the_quadratic_curve() {
        // The explicit V1 migration story: the X500's legacy
        // speed-domain hover seed 0.77 becomes force-domain
        // 0.77² = 0.5929; the quadratic boundary curve must map it
        // back to the identical boundary command, so the migrated
        // kernel commands the same rotor speed at trim.
        let force = NormalizedThrust(0.77 * 0.77);
        let boundary = ActuatorCurveKind::QuadraticRotor.boundary_command(force);
        assert!((boundary.0 - 0.77).abs() < 1e-6);
    }

    #[test]
    fn geometry_motor_counts() {
        assert_eq!(MixerGeometry::QuadX.motor_count(), 4);
        assert_eq!(MixerGeometry::QuadXX500.motor_count(), 4);
    }
}
