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

    // Copies no tuning from the resolved configuration: every value
    // this stub uses is compiled in, so there is nothing the config
    // hash could vouch for or contradict. Stated explicitly rather
    // than inherited — a future tunable version must replace this
    // with a real identity comparison.
    fn verify_config_binding(
        &self,
        _cfg: &crate::kernel::config::ResolvedKernelConfig,
    ) -> Result<(), crate::control::ControllerConfigMismatch> {
        Ok(())
    }

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::VehicleController;

    #[test]
    fn stub_declares_no_config_binding() {
        // The stub copies nothing from the resolved configuration, so
        // binding verification accepts any config — including default.
        let cfg = crate::kernel::config::ResolvedKernelConfig::default();
        assert!(FixedWingController.verify_config_binding(&cfg).is_ok());
    }
}
