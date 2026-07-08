use crate::control::runtime::NoControllerState;
use crate::control::{
    AxisCommand, Command, ConfigMode, Limits, VehicleControlMode, VehicleController,
};
use crate::state::StateEstimate;
use crate::types::NormalizedSigned;

pub struct FixedWingController;

impl VehicleController for FixedWingController {
    type RuntimeState = NoControllerState;

    // Registered in cert/algorithm_id_registry.toml as
    // "controller.fixed_wing.v1".
    const ALGORITHM_ID: u64 = 0x4354_4C46_5747_5631; // "CTLFWGV1"

    fn step(
        &self,
        _runtime: &mut NoControllerState,
        _state: &StateEstimate,
        command: &Command,
        _flags: &VehicleControlMode,
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
