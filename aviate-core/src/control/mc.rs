use crate::control::{VehicleController, AxisCommand, Command, ConfigMode, Limits};
use crate::state::StateEstimate;
use crate::types::NormalizedSigned;

pub struct McController;

impl VehicleController for McController {
    fn step(
        &mut self,
        _state: &StateEstimate,
        command: &Command,
        _mode: ConfigMode,
        _limits: &Limits,
    ) -> AxisCommand {
        // Placeholder MC logic
        AxisCommand {
            roll: NormalizedSigned(0.0),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: command.collective_thrust,
        }
    }
}
