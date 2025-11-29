use crate::control::{AxisCommand, Command, ConfigMode, Limits, VehicleController};
use crate::state::StateEstimate;
use crate::types::NormalizedSigned;

pub struct FwController;

impl VehicleController for FwController {
    fn step(
        &mut self,
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
