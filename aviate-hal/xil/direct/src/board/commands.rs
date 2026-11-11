//! Inbound command dispatch of [`super::DirectXilBoard`]: geodetic
//! targets into the present frame, the external-mode rule, and the
//! discrete arm, disarm and terminate commands.

use aviate_core::hal::ActuatorHal;
use aviate_core::types::Meters;
use aviate_hal_io::{CommandOutcome, SystemCommand};
use aviate_link::mavlink::MavState;

use super::ingress::{init_state_to_mav_state, DirectCommandRefusal};
use super::DirectXilBoard;

impl<C, M> DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    pub(super) fn drain_inbound_commands(&mut self, now_us: u64) {
        while let Some(mut sys_cmd) = self.endpoint.poll_command() {
            if let (Some(target), SystemCommand::FlightControl(flight)) =
                (self.endpoint.polled_geodetic_target(), &mut sys_cmd)
            {
                let Some(position) = self.geodetic_in_frame(&target) else {
                    log::warn!(
                        "DirectXilBoard refuses a geodetic target without a geoid separation"
                    );
                    self.last_refusal = Some(DirectCommandRefusal::GeoidSeparationUnknown);
                    continue;
                };
                flight.setpoint.position = Some(position.map(Meters));
            }
            if let SystemCommand::FlightControl(ref flight) = sys_cmd {
                let commandable = self
                    .kernel
                    .pipeline()
                    .controller
                    .accepts_external_mode(flight.mode);
                if self.endpoint.polled_from_link() && !commandable {
                    log::warn!("DirectXilBoard refuses link command in {:?}", flight.mode);
                    self.last_refusal = Some(DirectCommandRefusal::ModeNotCommandable(flight.mode));
                    continue;
                }
                self.kernel.state.checks.pre_arm.update_throttle(
                    flight.setpoint.collective_thrust
                        < aviate_core::kernel_types::THROTTLE_LOW_MAX_COLLECTIVE,
                );
            }
            if let Some(outcome) = self.enact_discrete(&sys_cmd) {
                self.endpoint.report_outcome(&sys_cmd, outcome);
            }
            self.ingress.receive(&sys_cmd, now_us);
        }
    }

    fn enact_discrete(&mut self, sys_cmd: &SystemCommand) -> Option<CommandOutcome> {
        match sys_cmd {
            SystemCommand::Arm => match self.kernel.arm() {
                Ok(()) => {
                    self.board_hal.arm();
                    self.endpoint.set_armed(true);
                    self.endpoint.set_system_status(MavState::Active);
                    Some(CommandOutcome::Accepted)
                }
                Err(error) => {
                    let missing = self.kernel.state.checks.pre_arm.missing();
                    Some(CommandOutcome::ArmRejected { error, missing })
                }
            },
            SystemCommand::Disarm => match self.kernel.disarm() {
                Ok(()) => {
                    self.board_hal.disarm();
                    self.endpoint.set_armed(false);
                    self.endpoint
                        .set_system_status(init_state_to_mav_state(self.kernel.state.init_state));
                    Some(CommandOutcome::Accepted)
                }
                Err(error) => Some(CommandOutcome::DisarmRejected(error)),
            },
            SystemCommand::EmergencyTerminate => {
                self.kernel.terminate();
                self.board_hal.disarm();
                self.endpoint.set_armed(false);
                self.endpoint.set_system_status(MavState::Critical);
                Some(CommandOutcome::Accepted)
            }
            SystemCommand::FlightControl(_) => None,
        }
    }
}
