//! Orthogonal control-mode flags — the explicit contract that
//! selects which cascade loops run.
//!
//! Modeled on PX4's `VehicleControlMode.msg`: an orthogonal set of
//! `flag_control_*_enabled` bits published to the controllers, where
//! the active loops are an explicit, gated decision rather than an
//! emergent property of which [`Setpoint`] fields happen to be
//! populated.
//!
//! The kernel derives a [`VehicleControlMode`] from the requested
//! [`ControlMode`] every cycle (see [`VehicleControlMode::from_control_mode`])
//! and hands it to the controller. Outer-loop selection is then a
//! single, auditable decision ([`VehicleControlMode::outer_loop`])
//! driven by these flags. Mode *requests* (RC-switch decode,
//! mission-driven mode changes) stay outside the cert boundary; the
//! kernel owns only the mode→loop mapping defined here.

use crate::control::enums::ControlMode;
use crate::control::Setpoint;
use crate::types::{Meters, MetersPerSecond};

/// Orthogonal control-mode flags handed to the controller each cycle.
///
/// Each `flag_control_*_enabled` bit authorizes one control concern.
/// The bits are orthogonal: an outer loop layers on top of the inner
/// loops it depends on (position implies velocity implies attitude
/// implies rates), but the struct stores each independently so a mode
/// manager can express any legal combination without a hidden
/// hierarchy.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct VehicleControlMode {
    /// Pilot stick inputs feed the active loops directly.
    pub flag_control_manual_enabled: bool,
    /// Setpoints originate from an external (offboard) computer.
    pub flag_control_offboard_enabled: bool,
    /// Body-rate loop is active.
    pub flag_control_rates_enabled: bool,
    /// Attitude loop is active (produces rate setpoints).
    pub flag_control_attitude_enabled: bool,
    /// Altitude hold is active (produces a climb-rate setpoint).
    pub flag_control_altitude_enabled: bool,
    /// Climb-rate loop is active.
    pub flag_control_climb_rate_enabled: bool,
    /// Velocity loop is active (produces attitude/thrust setpoints).
    pub flag_control_velocity_enabled: bool,
    /// Position loop is active (produces velocity setpoints).
    pub flag_control_position_enabled: bool,
    /// Flight termination is engaged; outputs are forced safe.
    pub flag_control_termination_enabled: bool,
}

/// Which outer loop the cascade runs this cycle, carrying the
/// mode-legal setpoint it acts on.
///
/// This is the single selection authority: the cascade matches on it
/// instead of inspecting setpoint-field presence directly, so the
/// Option inspection happens in exactly one place.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum OuterLoopSelection {
    /// Position → velocity → attitude → rate, tracking the NED
    /// position setpoint.
    Position([Meters; 3]),
    /// Velocity → attitude → rate, tracking the NED velocity
    /// setpoint.
    Velocity([MetersPerSecond; 3]),
    /// No outer loop: attitude/rate track the commanded attitude
    /// setpoint directly (manual / stabilized).
    None,
}

impl VehicleControlMode {
    /// Map a requested [`ControlMode`] to its authorized flag set.
    ///
    /// This is the deliberate default mapping the kernel applies when
    /// an OEM mode manager has not supplied flags of its own. The
    /// rate and attitude inner loops run in every powered mode; outer
    /// loops layer on per mode.
    pub const fn from_control_mode(mode: ControlMode) -> Self {
        match mode {
            ControlMode::Rate => Self {
                flag_control_manual_enabled: true,
                flag_control_offboard_enabled: false,
                flag_control_rates_enabled: true,
                flag_control_attitude_enabled: false,
                flag_control_altitude_enabled: false,
                flag_control_climb_rate_enabled: false,
                flag_control_velocity_enabled: false,
                flag_control_position_enabled: false,
                flag_control_termination_enabled: false,
            },
            ControlMode::Attitude => Self {
                flag_control_manual_enabled: true,
                flag_control_offboard_enabled: false,
                flag_control_rates_enabled: true,
                flag_control_attitude_enabled: true,
                flag_control_altitude_enabled: false,
                flag_control_climb_rate_enabled: false,
                flag_control_velocity_enabled: false,
                flag_control_position_enabled: false,
                flag_control_termination_enabled: false,
            },
            ControlMode::AltitudeHold => Self {
                flag_control_manual_enabled: true,
                flag_control_offboard_enabled: false,
                flag_control_rates_enabled: true,
                flag_control_attitude_enabled: true,
                flag_control_altitude_enabled: true,
                flag_control_climb_rate_enabled: true,
                flag_control_velocity_enabled: false,
                flag_control_position_enabled: false,
                flag_control_termination_enabled: false,
            },
            ControlMode::PositionHold => Self {
                flag_control_manual_enabled: false,
                flag_control_offboard_enabled: false,
                flag_control_rates_enabled: true,
                flag_control_attitude_enabled: true,
                flag_control_altitude_enabled: true,
                flag_control_climb_rate_enabled: true,
                flag_control_velocity_enabled: true,
                flag_control_position_enabled: true,
                flag_control_termination_enabled: false,
            },
            ControlMode::VelocityControl => Self {
                flag_control_manual_enabled: false,
                flag_control_offboard_enabled: true,
                flag_control_rates_enabled: true,
                flag_control_attitude_enabled: true,
                flag_control_altitude_enabled: false,
                flag_control_climb_rate_enabled: true,
                flag_control_velocity_enabled: true,
                flag_control_position_enabled: false,
                flag_control_termination_enabled: false,
            },
            ControlMode::DeviationTracking => Self {
                flag_control_manual_enabled: false,
                flag_control_offboard_enabled: false,
                flag_control_rates_enabled: true,
                flag_control_attitude_enabled: true,
                flag_control_altitude_enabled: true,
                flag_control_climb_rate_enabled: true,
                flag_control_velocity_enabled: true,
                flag_control_position_enabled: true,
                flag_control_termination_enabled: false,
            },
        }
    }

    /// Decide which outer loop runs, from the flags alone.
    ///
    /// A setpoint field is honored only when the corresponding flag
    /// authorizes its loop: a position setpoint present under a mode
    /// whose `flag_control_position_enabled` is clear is rejected
    /// here (not consumed downstream). This is the one place mode
    /// legality gates loop selection.
    pub fn outer_loop(&self, setpoint: &Setpoint) -> OuterLoopSelection {
        if self.flag_control_position_enabled {
            if let Some(position) = setpoint.position {
                return OuterLoopSelection::Position(position);
            }
        }
        if self.flag_control_velocity_enabled {
            if let Some(velocity) = setpoint.velocity {
                return OuterLoopSelection::Velocity(velocity);
            }
        }
        OuterLoopSelection::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Quaternion;
    use crate::types::{Meters, MetersPerSecond};

    fn pos_setpoint() -> Setpoint {
        Setpoint {
            position: Some([Meters(1.0), Meters(2.0), Meters(-3.0)]),
            ..Default::default()
        }
    }

    fn vel_setpoint() -> Setpoint {
        Setpoint {
            velocity: Some([
                MetersPerSecond(1.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            ..Default::default()
        }
    }

    fn att_setpoint() -> Setpoint {
        Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            ..Default::default()
        }
    }

    #[test]
    fn rate_mode_enables_only_rates() {
        let m = VehicleControlMode::from_control_mode(ControlMode::Rate);
        assert!(m.flag_control_rates_enabled);
        assert!(m.flag_control_manual_enabled);
        assert!(!m.flag_control_attitude_enabled);
        assert!(!m.flag_control_velocity_enabled);
        assert!(!m.flag_control_position_enabled);
    }

    #[test]
    fn attitude_mode_enables_attitude_not_outer_loops() {
        let m = VehicleControlMode::from_control_mode(ControlMode::Attitude);
        assert!(m.flag_control_attitude_enabled);
        assert!(m.flag_control_rates_enabled);
        assert!(!m.flag_control_altitude_enabled);
        assert!(!m.flag_control_velocity_enabled);
        assert!(!m.flag_control_position_enabled);
    }

    #[test]
    fn altitude_mode_enables_altitude_not_horizontal() {
        let m = VehicleControlMode::from_control_mode(ControlMode::AltitudeHold);
        assert!(m.flag_control_altitude_enabled);
        assert!(m.flag_control_climb_rate_enabled);
        assert!(m.flag_control_attitude_enabled);
        assert!(!m.flag_control_velocity_enabled);
        assert!(!m.flag_control_position_enabled);
    }

    #[test]
    fn position_mode_enables_full_stack() {
        let m = VehicleControlMode::from_control_mode(ControlMode::PositionHold);
        assert!(m.flag_control_position_enabled);
        assert!(m.flag_control_velocity_enabled);
        assert!(m.flag_control_altitude_enabled);
        assert!(m.flag_control_attitude_enabled);
        assert!(m.flag_control_rates_enabled);
        assert!(!m.flag_control_manual_enabled);
    }

    #[test]
    fn velocity_mode_enables_velocity_not_position() {
        let m = VehicleControlMode::from_control_mode(ControlMode::VelocityControl);
        assert!(m.flag_control_velocity_enabled);
        assert!(m.flag_control_offboard_enabled);
        assert!(!m.flag_control_position_enabled);
    }

    #[test]
    fn deviation_mode_enables_position_stack() {
        let m = VehicleControlMode::from_control_mode(ControlMode::DeviationTracking);
        assert!(m.flag_control_position_enabled);
        assert!(m.flag_control_velocity_enabled);
        assert!(m.flag_control_attitude_enabled);
    }

    #[test]
    fn outer_loop_selects_position_when_authorized() {
        let m = VehicleControlMode::from_control_mode(ControlMode::PositionHold);
        assert!(matches!(
            m.outer_loop(&pos_setpoint()),
            OuterLoopSelection::Position(_)
        ));
    }

    #[test]
    fn outer_loop_selects_velocity_when_authorized() {
        let m = VehicleControlMode::from_control_mode(ControlMode::VelocityControl);
        assert!(matches!(
            m.outer_loop(&vel_setpoint()),
            OuterLoopSelection::Velocity(_)
        ));
    }

    #[test]
    fn outer_loop_none_for_attitude_mode() {
        let m = VehicleControlMode::from_control_mode(ControlMode::Attitude);
        assert_eq!(m.outer_loop(&att_setpoint()), OuterLoopSelection::None);
    }

    #[test]
    fn outer_loop_rejects_position_setpoint_when_flag_clear() {
        // A position setpoint present under Attitude mode must not
        // select the position loop — rejection by mode legality.
        let m = VehicleControlMode::from_control_mode(ControlMode::Attitude);
        assert_eq!(m.outer_loop(&pos_setpoint()), OuterLoopSelection::None);
    }

    #[test]
    fn outer_loop_falls_through_to_velocity_when_position_absent() {
        // Position authorized but no position setpoint, velocity
        // present: the velocity loop is selected.
        let m = VehicleControlMode::from_control_mode(ControlMode::PositionHold);
        assert!(matches!(
            m.outer_loop(&vel_setpoint()),
            OuterLoopSelection::Velocity(_)
        ));
    }

    #[test]
    fn outer_loop_none_when_authorized_but_no_setpoint() {
        let m = VehicleControlMode::from_control_mode(ControlMode::PositionHold);
        assert_eq!(m.outer_loop(&Setpoint::default()), OuterLoopSelection::None);
    }

    #[test]
    fn default_flags_are_all_clear() {
        assert_eq!(
            VehicleControlMode::default(),
            VehicleControlMode {
                flag_control_manual_enabled: false,
                flag_control_offboard_enabled: false,
                flag_control_rates_enabled: false,
                flag_control_attitude_enabled: false,
                flag_control_altitude_enabled: false,
                flag_control_climb_rate_enabled: false,
                flag_control_velocity_enabled: false,
                flag_control_position_enabled: false,
                flag_control_termination_enabled: false,
            }
        );
    }
}
