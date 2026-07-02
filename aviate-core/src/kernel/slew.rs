//! Per-cycle actuator slew limiter (DRQ-FLT-001 / DRQ-MORPH-001).
//!
//! The kernel writes per-actuator `Normalized` outputs every cycle.
//! For command-loss-safe mode transitions (HLR-FLT-204 / LLR-FLT-208)
//! and morph-induced geometry deltas (HLR-MORPH-202 / LLR-MORPH-202),
//! the per-actuator delta between consecutive cycles must respect a
//! configured slew limit — a sudden jump from a saturated motor down
//! to idle can shed lift faster than the body can rotate to
//! compensate, so the contract is to clamp the per-cycle delta and
//! let several cycles complete the transition.
//!
//! Disabled by default: a `slew_limit_per_cycle[i]` of `0.0` (or any
//! non-positive / non-finite value) means "no slew constraint on
//! channel `i`" — the writer is a no-op. Existing airframes don't
//! configure slew limits and continue to operate identically.
//!
//! Scope: applies ONLY to the normal control path. The severe-fault
//! early-return paths in `kernel_update.rs` (numeric-error backup,
//! enum-corruption backup) bypass the slew limiter and emit the safe
//! pattern immediately — slew-limiting a severe-fault response would
//! keep dangerous outputs alive for additional cycles, which is the
//! opposite of what LLR-FLT-205 requires.

use crate::mixer::{ActuatorCmd, MAX_ACTUATORS};
use crate::types::Normalized;

/// Clamp the per-actuator delta in `cmd` against `previous` to at
/// most `slew_per_cycle[i]` per channel.
///
/// Behavior per channel `i`:
///   - `slew_per_cycle[i] > 0` and finite → clamp `cmd.outputs[i]` to
///     within `±slew_per_cycle[i]` of `previous[i]`.
///   - `slew_per_cycle[i] <= 0` or non-finite → channel unconstrained
///     (pass through). Default for all airframes that don't opt in.
///
/// Direction reversal is handled by the symmetric clamp range
/// `[-limit, +limit]` — a sign-flipped target still respects the
/// same per-cycle magnitude bound.
pub fn apply_slew_limit(
    cmd: &mut ActuatorCmd,
    previous: &[Normalized; MAX_ACTUATORS],
    slew_per_cycle: &[Normalized; MAX_ACTUATORS],
) {
    for i in 0..MAX_ACTUATORS {
        let limit = slew_per_cycle[i].0;
        if !limit.is_finite() || limit <= 0.0 {
            continue;
        }
        let prev = previous[i].0;
        let target = cmd.outputs[i].0;
        let delta = target - prev;
        let clamped = delta.clamp(-limit, limit);
        cmd.outputs[i] = Normalized(prev + clamped);
    }
}

#[cfg(test)]
#[allow(clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::time::{TimeSource, Timestamp};

    fn make_cmd(outputs: [f32; MAX_ACTUATORS]) -> ActuatorCmd {
        ActuatorCmd {
            outputs: outputs.map(Normalized),
            active_mask: 0xFFFF,
            sequence: 0,
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            fallback_mask: 0,
            sanitized: false,
        }
    }

    #[test]
    fn zero_limit_disables_slew() {
        let mut cmd = make_cmd([1.0; MAX_ACTUATORS]);
        let prev = [Normalized(0.0); MAX_ACTUATORS];
        let limits = [Normalized(0.0); MAX_ACTUATORS];

        apply_slew_limit(&mut cmd, &prev, &limits);

        for n in &cmd.outputs {
            assert!(
                (n.0 - 1.0).abs() < 1e-6,
                "expected passthrough, got {}",
                n.0
            );
        }
    }

    #[test]
    fn positive_limit_caps_positive_delta() {
        let mut cmd = make_cmd([1.0; MAX_ACTUATORS]);
        let prev = [Normalized(0.0); MAX_ACTUATORS];
        let limits = [Normalized(0.1); MAX_ACTUATORS];

        apply_slew_limit(&mut cmd, &prev, &limits);

        for n in &cmd.outputs {
            assert!((n.0 - 0.1).abs() < 1e-6, "expected 0.1, got {}", n.0);
        }
    }

    #[test]
    fn positive_limit_caps_negative_delta() {
        let mut cmd = make_cmd([-1.0; MAX_ACTUATORS]);
        let prev = [Normalized(0.5); MAX_ACTUATORS];
        let limits = [Normalized(0.1); MAX_ACTUATORS];

        apply_slew_limit(&mut cmd, &prev, &limits);

        for n in &cmd.outputs {
            assert!((n.0 - 0.4).abs() < 1e-6, "expected 0.4, got {}", n.0);
        }
    }

    #[test]
    fn delta_within_limit_passes_through() {
        let mut cmd = make_cmd([0.55; MAX_ACTUATORS]);
        let prev = [Normalized(0.5); MAX_ACTUATORS];
        let limits = [Normalized(0.1); MAX_ACTUATORS];

        apply_slew_limit(&mut cmd, &prev, &limits);

        for n in &cmd.outputs {
            assert!(
                (n.0 - 0.55).abs() < 1e-6,
                "expected passthrough 0.55, got {}",
                n.0
            );
        }
    }

    #[test]
    fn direction_reversal_uses_symmetric_clamp() {
        let mut cmd = make_cmd([0.0; MAX_ACTUATORS]);
        cmd.outputs[0] = Normalized(-0.5);
        cmd.outputs[1] = Normalized(0.5);
        let prev = {
            let mut p = [Normalized(0.0); MAX_ACTUATORS];
            p[0] = Normalized(0.3);
            p[1] = Normalized(-0.3);
            p
        };
        let limits = [Normalized(0.2); MAX_ACTUATORS];

        apply_slew_limit(&mut cmd, &prev, &limits);

        let ch0 = cmd.outputs[0].0;
        assert!(
            (ch0 - 0.1).abs() < 1e-6,
            "ch0: 0.3 → -0.5 clamped to 0.3-0.2=0.1, got {ch0}"
        );
        let ch1 = cmd.outputs[1].0;
        assert!(
            (ch1 + 0.1).abs() < 1e-6,
            "ch1: -0.3 → 0.5 clamped to -0.3+0.2=-0.1, got {ch1}"
        );
    }

    #[test]
    fn multi_cycle_ramp_converges() {
        let mut prev = [Normalized(0.0); MAX_ACTUATORS];
        let target = [Normalized(1.0); MAX_ACTUATORS];
        let limits = [Normalized(0.2); MAX_ACTUATORS];

        for cycle in 1..=6 {
            let mut cmd = make_cmd([1.0; MAX_ACTUATORS]);
            apply_slew_limit(&mut cmd, &prev, &limits);
            let expected = (0.2 * cycle as f32).min(1.0);
            for n in &cmd.outputs {
                assert!(
                    (n.0 - expected).abs() < 1e-6,
                    "cycle {}: expected {}, got {}",
                    cycle,
                    expected,
                    n.0
                );
            }
            prev = cmd.outputs;
        }

        for (i, n) in target.iter().zip(prev.iter()) {
            assert!(
                (i.0 - n.0).abs() < 1e-6,
                "ramp did not converge: target {}, got {}",
                i.0,
                n.0
            );
        }
    }

    #[test]
    fn non_finite_limit_treated_as_disabled() {
        let mut cmd = make_cmd([1.0; MAX_ACTUATORS]);
        cmd.outputs[1] = Normalized(0.5);
        let prev = [Normalized(0.0); MAX_ACTUATORS];
        let mut limits = [Normalized(0.1); MAX_ACTUATORS];
        limits[0] = Normalized(f32::NAN);
        limits[1] = Normalized(f32::INFINITY);

        apply_slew_limit(&mut cmd, &prev, &limits);

        let ch0 = cmd.outputs[0].0;
        assert!(
            (ch0 - 1.0).abs() < 1e-6,
            "NaN limit should leave channel unconstrained, got {ch0}"
        );
        let ch1 = cmd.outputs[1].0;
        assert!(
            (ch1 - 0.5).abs() < 1e-6,
            "Inf limit should leave channel unconstrained, got {ch1}"
        );
        for n in cmd.outputs.iter().skip(2) {
            assert!((n.0 - 0.1).abs() < 1e-6, "expected 0.1, got {}", n.0);
        }
    }

    #[test]
    fn negative_limit_treated_as_disabled() {
        let mut cmd = make_cmd([1.0; MAX_ACTUATORS]);
        let prev = [Normalized(0.0); MAX_ACTUATORS];
        let limits = [Normalized(-0.5); MAX_ACTUATORS];

        apply_slew_limit(&mut cmd, &prev, &limits);

        for n in &cmd.outputs {
            assert!(
                (n.0 - 1.0).abs() < 1e-6,
                "negative limit should disable, got {}",
                n.0
            );
        }
    }
}
