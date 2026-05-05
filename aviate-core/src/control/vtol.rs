use crate::control::runtime::NoControllerState;
use crate::control::{AxisCommand, Command, ConfigMode, Limits, VehicleController};
use crate::state::StateEstimate;
use crate::types::NormalizedSigned;

pub struct VtolController;

impl VehicleController for VtolController {
    type RuntimeState = NoControllerState;

    // Registered in cert/algorithm_id_registry.toml as
    // "controller.vtol.v1".
    const ALGORITHM_ID: u64 = 0x4354_4C56_544F_4C31; // "CTLVTOL1"

    fn step(
        &self,
        _runtime: &mut NoControllerState,
        _state: &StateEstimate,
        command: &Command,
        _mode: ConfigMode,
        _limits: &Limits,
    ) -> AxisCommand {
        // Placeholder VTOL logic (hybrid)
        AxisCommand {
            roll: NormalizedSigned(0.0),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: command.setpoint.collective_thrust,
        }
    }
}
