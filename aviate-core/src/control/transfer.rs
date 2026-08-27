//! Multirotor mode capability and effective control topology.

use super::{ControlMode, Setpoint, VehicleControlMode};

/// Production support for one multirotor control mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MultirotorModeCapability {
    /// An external command can request this mode.
    External,
    /// The flight-control kernel can request this mode.
    Internal,
    /// The multirotor production controller does not support this mode.
    Unsupported,
}

/// The effective cascade topology for one controller cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EffectiveControlTopology {
    /// The position loop supplies the velocity loop.
    Position,
    /// A velocity setpoint supplies the velocity loop.
    Velocity,
    /// The vertical velocity loop supplies collective.
    Vertical,
    /// The attitude and rate loops use a direct attitude command.
    Attitude,
    /// The thrust gate resets controller memory and silences torque.
    ZeroThrust,
    /// The requested mode is outside the production capability.
    Unsupported,
}

/// Return the production capability for a multirotor mode.
pub const fn multirotor_mode_capability(mode: ControlMode) -> MultirotorModeCapability {
    match mode {
        ControlMode::Attitude | ControlMode::PositionHold | ControlMode::VelocityControl => {
            MultirotorModeCapability::External
        }
        ControlMode::AltitudeHold => MultirotorModeCapability::Internal,
        ControlMode::Rate | ControlMode::DeviationTracking => MultirotorModeCapability::Unsupported,
    }
}

impl VehicleControlMode {
    /// Return the cascade topology that the controller can run.
    pub fn effective_topology(
        &self,
        mode: ControlMode,
        setpoint: &Setpoint,
    ) -> EffectiveControlTopology {
        if multirotor_mode_capability(mode) == MultirotorModeCapability::Unsupported {
            return EffectiveControlTopology::Unsupported;
        }
        match self.outer_loop(setpoint) {
            super::OuterLoopSelection::Position(_) => EffectiveControlTopology::Position,
            super::OuterLoopSelection::Velocity(_) => EffectiveControlTopology::Velocity,
            super::OuterLoopSelection::None
                if self.flag_control_altitude_enabled
                    && (setpoint.altitude.is_some() || setpoint.vertical_speed.is_some()) =>
            {
                EffectiveControlTopology::Vertical
            }
            super::OuterLoopSelection::None => EffectiveControlTopology::Attitude,
        }
    }
}

/// Gated with its only caller. The multirotor controller hashes these tags
/// into its transition identity, and that controller is behind `mc`; without
/// the gate a build without it carries two functions nobody can reach, which
/// the lint set refuses.
#[cfg(feature = "mc")]
pub(crate) const fn mode_option_tag(mode: Option<ControlMode>) -> u8 {
    match mode {
        None => 0,
        Some(ControlMode::Attitude) => 1,
        Some(ControlMode::AltitudeHold) => 2,
        Some(ControlMode::PositionHold) => 3,
        Some(ControlMode::VelocityControl) => 4,
        Some(ControlMode::Rate) => 5,
        Some(ControlMode::DeviationTracking) => 6,
    }
}

#[cfg(feature = "mc")]
pub(crate) const fn topology_option_tag(topology: Option<EffectiveControlTopology>) -> u8 {
    match topology {
        None => 0,
        Some(EffectiveControlTopology::Attitude) => 1,
        Some(EffectiveControlTopology::Vertical) => 2,
        Some(EffectiveControlTopology::Velocity) => 3,
        Some(EffectiveControlTopology::Position) => 4,
        Some(EffectiveControlTopology::ZeroThrust) => 5,
        Some(EffectiveControlTopology::Unsupported) => 6,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Meters, MetersPerSecond};

    #[test]
    fn production_capability_is_explicit() {
        assert_eq!(
            multirotor_mode_capability(ControlMode::Attitude),
            MultirotorModeCapability::External
        );
        assert_eq!(
            multirotor_mode_capability(ControlMode::PositionHold),
            MultirotorModeCapability::External
        );
        assert_eq!(
            multirotor_mode_capability(ControlMode::VelocityControl),
            MultirotorModeCapability::External
        );
        assert_eq!(
            multirotor_mode_capability(ControlMode::AltitudeHold),
            MultirotorModeCapability::Internal
        );
        assert_eq!(
            multirotor_mode_capability(ControlMode::Rate),
            MultirotorModeCapability::Unsupported
        );
        assert_eq!(
            multirotor_mode_capability(ControlMode::DeviationTracking),
            MultirotorModeCapability::Unsupported
        );
    }

    #[test]
    fn topology_uses_mode_and_complete_setpoint() {
        let position = Setpoint {
            position: Some([Meters(1.0); 3]),
            velocity: Some([MetersPerSecond(2.0); 3]),
            ..Default::default()
        };
        let flags = VehicleControlMode::from_control_mode(ControlMode::PositionHold);
        assert_eq!(
            flags.effective_topology(ControlMode::PositionHold, &position),
            EffectiveControlTopology::Position
        );

        let velocity = Setpoint {
            velocity: Some([MetersPerSecond(2.0); 3]),
            ..Default::default()
        };
        assert_eq!(
            flags.effective_topology(ControlMode::PositionHold, &velocity),
            EffectiveControlTopology::Velocity
        );
    }
}
