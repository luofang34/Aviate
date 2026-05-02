use crate::control::runtime::NoControllerState;
use crate::control::{AxisCommand, Command, ConfigMode, Limits, VehicleController};
use crate::state::StateEstimate;
use crate::types::NormalizedSigned;

pub struct FixedWingController;

impl VehicleController for FixedWingController {
    type RuntimeState = NoControllerState;

    fn step(
        &self,
        _runtime: &mut NoControllerState,
        _state: &StateEstimate,
        command: &Command,
        _mode: ConfigMode,
        _limits: &Limits,
    ) -> AxisCommand {
        // Placeholder FW logic
        AxisCommand {
            roll: NormalizedSigned(0.0),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: command.setpoint.collective_thrust,
        }
    }
}
