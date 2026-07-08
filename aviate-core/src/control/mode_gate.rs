//! Mode-entry gating: refuses a commanded outer loop the estimator
//! can't back and falls back to the highest capable rung — the
//! kernel-side analog of PX4's `mode_requirements.cpp`.
//!
//! [`ControlMode::required_validity`] declares what each mode needs;
//! [`gate_mode_entry`] is the loop-selection-time check that enforces
//! it, and [`apply_mode_entry`] carries the decision into the command
//! the cascade actually flies. The decision is carried into
//! `ChannelStatus` (`kernel_types.rs`) so a mode is never silently
//! swapped without the requested mode and the missing validity being
//! visible downstream.

use crate::control::enums::ControlMode;
use crate::control::Command;
use crate::state::StateValidFlags;
use crate::types::MetersPerSecond;

impl ControlMode {
    /// Estimator validity this mode's outer loop needs to run.
    ///
    /// `Rate` needs nothing (the rate loop tracks raw gyro). Every
    /// other mode needs at least `ATTITUDE`, since the cascade always
    /// closes the attitude→rate loops underneath whichever outer
    /// loop is selected. `AltitudeHold`/`VelocityControl` additionally
    /// need `VELOCITY` (the climb-rate/velocity source their vertical
    /// or velocity loop closes against); `PositionHold`/
    /// `DeviationTracking` need `POSITION` on top of that for their
    /// horizontal loop.
    pub fn required_validity(self) -> StateValidFlags {
        match self {
            ControlMode::Rate => StateValidFlags::empty(),
            ControlMode::Attitude => StateValidFlags::ATTITUDE,
            ControlMode::AltitudeHold | ControlMode::VelocityControl => {
                StateValidFlags::ATTITUDE | StateValidFlags::VELOCITY
            }
            ControlMode::PositionHold | ControlMode::DeviationTracking => {
                StateValidFlags::ATTITUDE | StateValidFlags::VELOCITY | StateValidFlags::POSITION
            }
        }
    }
}

/// Rungs considered for mode-entry fallback, most to least
/// estimator-demanding. `Rate` has nothing to fall back from (its
/// requirement is always met) and is excluded; `VelocityControl` and
/// `DeviationTracking` share a validity tier with `AltitudeHold` and
/// `PositionHold` respectively and fall through these same rungs
/// rather than needing entries of their own.
const FALLBACK_RUNGS: [ControlMode; 3] = [
    ControlMode::PositionHold,
    ControlMode::AltitudeHold,
    ControlMode::Attitude,
];

/// Outcome of gating a commanded mode's entry against current
/// estimator validity.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ModeEntryDecision {
    /// The requested mode's validity requirement is met; it runs as
    /// requested.
    Granted(ControlMode),
    /// The requested mode's requirement is unmet; `effective` is the
    /// highest rung whose own requirement current validity does
    /// satisfy. `missing` is what the *requested* mode was short of.
    FallenBack {
        requested: ControlMode,
        effective: ControlMode,
        missing: StateValidFlags,
    },
    /// Even the least-demanding rung's requirement (bare attitude) is
    /// unmet — i.e. `ATTITUDE` itself is invalid. In the full update
    /// cycle this coincides exactly with
    /// `DegradationReason::AttitudeLost`, which forces
    /// `ControlLawV1::Backup` earlier in the same cycle and always
    /// wins first; this variant exists so the gate's own contract is
    /// total (every input has an answer), not because the kernel
    /// acts on it directly.
    Refused {
        requested: ControlMode,
        missing: StateValidFlags,
    },
}

impl ModeEntryDecision {
    /// Mode whose flags should drive loop selection this cycle.
    ///
    /// `Refused` has no mode that is safe to fly; callers on the
    /// `Backup` path never reach here, and off that path this value
    /// is a defensive fallback, not a claim that `Attitude` is safe
    /// to actually run against invalid attitude.
    pub fn effective(self) -> ControlMode {
        match self {
            ModeEntryDecision::Granted(m) => m,
            ModeEntryDecision::FallenBack { effective, .. } => effective,
            ModeEntryDecision::Refused { .. } => ControlMode::Attitude,
        }
    }
}

/// Gate a commanded mode's entry against current estimator validity.
///
/// Walks the fallback rungs from most to least demanding, skipping
/// any rung that is not strictly less demanding than what was
/// requested (a fallback never runs sideways or up in capability),
/// and returns the first rung whose own requirement `valid`
/// satisfies. Deterministic, allocation-free, and bounded by the
/// fixed rung count — at most three comparisons.
pub fn gate_mode_entry(requested: ControlMode, valid: StateValidFlags) -> ModeEntryDecision {
    let required = requested.required_validity();
    let missing = required - valid;
    if missing.is_empty() {
        return ModeEntryDecision::Granted(requested);
    }
    for &rung in &FALLBACK_RUNGS {
        let rung_required = rung.required_validity();
        let strictly_less_demanding = required.contains(rung_required) && rung_required != required;
        if strictly_less_demanding && valid.contains(rung_required) {
            return ModeEntryDecision::FallenBack {
                requested,
                effective: rung,
                missing,
            };
        }
    }
    ModeEntryDecision::Refused { requested, missing }
}

/// Apply a mode-entry decision to the command about to be flown.
///
/// Retags `command.mode` to the gated rung; a `Granted` decision
/// leaves the command untouched. Falling back into `AltitudeHold`
/// also needs a vertical target: a bare `PositionHold`/
/// `VelocityControl`/`DeviationTracking` setpoint carries its
/// vertical target inside `position` (an axis the fallback no longer
/// trusts, since `POSITION` validity covers the whole 3-vector), not
/// in `vertical_speed`/`altitude`. Left unset, the vertical loop
/// would find no target and the collective would fall through to
/// `Setpoint::collective_thrust`'s manual-passthrough default of
/// zero — commanding motors off mid-flight instead of holding
/// altitude. Defaulting to a zero climb rate keeps the vehicle
/// station-keeping on the validated velocity estimate instead.
pub fn apply_mode_entry(mut command: Command, decision: ModeEntryDecision) -> Command {
    let effective = decision.effective();
    if effective == command.mode {
        return command;
    }
    if effective == ControlMode::AltitudeHold
        && command.setpoint.vertical_speed.is_none()
        && command.setpoint.altitude.is_none()
    {
        command.setpoint.vertical_speed = Some(MetersPerSecond(0.0));
    }
    command.mode = effective;
    command
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: StateValidFlags = StateValidFlags::all();
    const ATT: StateValidFlags = StateValidFlags::ATTITUDE;
    const ATT_VEL: StateValidFlags = StateValidFlags::ATTITUDE.union(StateValidFlags::VELOCITY);
    const ATT_VEL_POS: StateValidFlags = StateValidFlags::ATTITUDE
        .union(StateValidFlags::VELOCITY)
        .union(StateValidFlags::POSITION);

    #[test]
    fn required_validity_table() {
        assert_eq!(
            ControlMode::Rate.required_validity(),
            StateValidFlags::empty()
        );
        assert_eq!(ControlMode::Attitude.required_validity(), ATT);
        assert_eq!(ControlMode::AltitudeHold.required_validity(), ATT_VEL);
        assert_eq!(ControlMode::VelocityControl.required_validity(), ATT_VEL);
        assert_eq!(ControlMode::PositionHold.required_validity(), ATT_VEL_POS);
        assert_eq!(
            ControlMode::DeviationTracking.required_validity(),
            ATT_VEL_POS
        );
    }

    #[test]
    fn rate_mode_always_granted() {
        assert_eq!(
            gate_mode_entry(ControlMode::Rate, StateValidFlags::empty()),
            ModeEntryDecision::Granted(ControlMode::Rate)
        );
    }

    #[test]
    fn position_granted_when_fully_valid() {
        assert_eq!(
            gate_mode_entry(ControlMode::PositionHold, ALL),
            ModeEntryDecision::Granted(ControlMode::PositionHold)
        );
    }

    #[test]
    fn position_falls_back_to_altitude_when_only_position_invalid() {
        let decision = gate_mode_entry(ControlMode::PositionHold, ATT_VEL);
        assert_eq!(
            decision,
            ModeEntryDecision::FallenBack {
                requested: ControlMode::PositionHold,
                effective: ControlMode::AltitudeHold,
                missing: StateValidFlags::POSITION,
            }
        );
    }

    #[test]
    fn position_falls_back_to_attitude_when_position_and_velocity_invalid() {
        let decision = gate_mode_entry(ControlMode::PositionHold, ATT);
        assert_eq!(
            decision,
            ModeEntryDecision::FallenBack {
                requested: ControlMode::PositionHold,
                effective: ControlMode::Attitude,
                missing: StateValidFlags::POSITION | StateValidFlags::VELOCITY,
            }
        );
    }

    #[test]
    fn position_refused_when_attitude_invalid() {
        let decision = gate_mode_entry(ControlMode::PositionHold, StateValidFlags::empty());
        assert_eq!(
            decision,
            ModeEntryDecision::Refused {
                requested: ControlMode::PositionHold,
                missing: ATT_VEL_POS,
            }
        );
    }

    #[test]
    fn deviation_tracking_falls_back_like_position() {
        let decision = gate_mode_entry(ControlMode::DeviationTracking, ATT_VEL);
        assert_eq!(
            decision,
            ModeEntryDecision::FallenBack {
                requested: ControlMode::DeviationTracking,
                effective: ControlMode::AltitudeHold,
                missing: StateValidFlags::POSITION,
            }
        );
    }

    #[test]
    fn altitude_falls_back_to_attitude_when_velocity_invalid() {
        let decision = gate_mode_entry(ControlMode::AltitudeHold, ATT);
        assert_eq!(
            decision,
            ModeEntryDecision::FallenBack {
                requested: ControlMode::AltitudeHold,
                effective: ControlMode::Attitude,
                missing: StateValidFlags::VELOCITY,
            }
        );
    }

    #[test]
    fn velocity_control_skips_the_altitude_rung_and_falls_to_attitude() {
        // VelocityControl and AltitudeHold share a validity tier
        // (ATTITUDE|VELOCITY); when that tier's requirement isn't
        // met, re-trying AltitudeHold would fail identically, so the
        // gate should skip straight to Attitude rather than wasting
        // a rung on a guaranteed-to-fail retry.
        let decision = gate_mode_entry(ControlMode::VelocityControl, ATT);
        assert_eq!(
            decision,
            ModeEntryDecision::FallenBack {
                requested: ControlMode::VelocityControl,
                effective: ControlMode::Attitude,
                missing: StateValidFlags::VELOCITY,
            }
        );
    }

    #[test]
    fn attitude_refused_when_its_own_requirement_unmet() {
        let decision = gate_mode_entry(ControlMode::Attitude, StateValidFlags::empty());
        assert_eq!(
            decision,
            ModeEntryDecision::Refused {
                requested: ControlMode::Attitude,
                missing: ATT,
            }
        );
    }

    #[test]
    fn effective_reads_through_each_variant() {
        assert_eq!(
            ModeEntryDecision::Granted(ControlMode::PositionHold).effective(),
            ControlMode::PositionHold
        );
        assert_eq!(
            ModeEntryDecision::FallenBack {
                requested: ControlMode::PositionHold,
                effective: ControlMode::AltitudeHold,
                missing: StateValidFlags::POSITION,
            }
            .effective(),
            ControlMode::AltitudeHold
        );
        assert_eq!(
            ModeEntryDecision::Refused {
                requested: ControlMode::PositionHold,
                missing: ATT_VEL_POS,
            }
            .effective(),
            ControlMode::Attitude
        );
    }

    fn position_command() -> Command {
        use crate::control::{CommandSource, Setpoint};
        use crate::types::Meters;
        Command {
            mode: ControlMode::PositionHold,
            setpoint: Setpoint {
                position: Some([Meters(10.0), Meters(0.0), Meters(-5.0)]),
                ..Default::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 1,
            source: CommandSource::Autopilot,
        }
    }

    #[test]
    fn apply_mode_entry_leaves_command_untouched_when_granted() {
        let cmd = position_command();
        let out = apply_mode_entry(cmd.clone(), ModeEntryDecision::Granted(cmd.mode));
        assert_eq!(out.mode, cmd.mode);
        assert_eq!(
            out.setpoint.position.map(|p| p[0].0),
            cmd.setpoint.position.map(|p| p[0].0)
        );
    }

    #[test]
    fn apply_mode_entry_retags_mode_and_defaults_vertical_speed_on_altitude_fallback() {
        let cmd = position_command();
        let decision = ModeEntryDecision::FallenBack {
            requested: cmd.mode,
            effective: ControlMode::AltitudeHold,
            missing: StateValidFlags::POSITION,
        };
        let out = apply_mode_entry(cmd, decision);
        assert_eq!(out.mode, ControlMode::AltitudeHold);
        assert_eq!(
            out.setpoint.vertical_speed,
            Some(MetersPerSecond(0.0)),
            "AltitudeHold fallback must synthesize a vertical target so \
             the vertical loop doesn't fall through to manual collective \
             passthrough on a setpoint that never carried one"
        );
    }

    #[test]
    fn apply_mode_entry_preserves_an_existing_vertical_speed_on_altitude_fallback() {
        let mut cmd = position_command();
        cmd.setpoint.vertical_speed = Some(MetersPerSecond(-1.5));
        let decision = ModeEntryDecision::FallenBack {
            requested: cmd.mode,
            effective: ControlMode::AltitudeHold,
            missing: StateValidFlags::POSITION,
        };
        let out = apply_mode_entry(cmd, decision);
        assert_eq!(out.setpoint.vertical_speed, Some(MetersPerSecond(-1.5)));
    }

    #[test]
    fn apply_mode_entry_does_not_synthesize_a_vertical_speed_on_attitude_fallback() {
        let cmd = position_command();
        let decision = ModeEntryDecision::FallenBack {
            requested: cmd.mode,
            effective: ControlMode::Attitude,
            missing: StateValidFlags::POSITION | StateValidFlags::VELOCITY,
        };
        let out = apply_mode_entry(cmd, decision);
        assert_eq!(out.mode, ControlMode::Attitude);
        assert_eq!(
            out.setpoint.vertical_speed, None,
            "Attitude has no closed-loop collective path to synthesize a \
             target for; this fallback rung is a known residual risk for \
             autonomous commands, not something this gate papers over"
        );
    }
}
