//! MAVLink command input and operator response handling.

use log::{info, warn};

use aviate_hal_io::{CommandHal, CommandOutcome, SystemCommand};
use aviate_link::mavlink::protocol::{
    CommandAck, CommandLong, Heartbeat, SetAttitudeTarget, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{
    mav_cmd, mav_result, parse_mavlink_frame, serialize_mavlink, MavAutopilot, MavMessage,
    MavModeFlag, MavState, MavType, FORCE_ARM_DISARM_MAGIC,
};

use crate::bridge;
use crate::command_provenance::{MavlinkCommandFamily, MavlinkCommandProvenance};
use crate::XilConfig;

use super::{ReceivedCommand, SitlIO};

impl SitlIO {
    pub(super) fn process_mavlink_data(&mut self, data: &[u8], source: std::net::SocketAddr) {
        match parse_mavlink_frame(data) {
            Ok(parsed) => {
                self.rx_count = self.rx_count.wrapping_add(1);
                self.handle_message(
                    parsed.message,
                    source,
                    parsed.header,
                    &data[..parsed.consumed],
                );
            }
            Err(error) => warn!(
                "MAVLink parse error from {source}: {error:?} (length={}, prefix={:02x?})",
                data.len(),
                &data[..data.len().min(10)]
            ),
        }
    }

    fn handle_message(
        &mut self,
        message: MavMessage,
        source: std::net::SocketAddr,
        header: aviate_link::mavlink::protocol::MavHeader,
        frame: &[u8],
    ) {
        match message {
            MavMessage::SetAttitudeTarget(target) => {
                self.gcs_addr = Some(source);
                self.handle_set_attitude_target(target, source, header, frame);
            }
            MavMessage::SetPositionTargetLocalNed(target) => {
                self.gcs_addr = Some(source);
                self.handle_set_position_target(target, source, header, frame);
            }
            MavMessage::CommandLong(command) => {
                self.gcs_addr = Some(source);
                self.handle_command_long(command);
            }
            MavMessage::Heartbeat(_) => self.gcs_addr = Some(source),
            _ => {}
        }
    }

    fn handle_set_attitude_target(
        &mut self,
        target: SetAttitudeTarget,
        source: std::net::SocketAddr,
        header: aviate_link::mavlink::protocol::MavHeader,
        frame: &[u8],
    ) {
        let mut command = bridge::mavlink_to_command(&target);
        command.sequence = u32::from(header.seq);
        let provenance = MavlinkCommandProvenance::new(
            &mut self.command_source_epochs,
            source,
            header,
            target.time_boot_ms,
            MavlinkCommandFamily::AttitudeTarget,
            frame,
        );
        self.flight_cmd = Some(ReceivedCommand {
            command: SystemCommand::FlightControl(command),
            provenance: Some(provenance),
        });
    }

    fn handle_set_position_target(
        &mut self,
        target: SetPositionTargetLocalNed,
        source: std::net::SocketAddr,
        header: aviate_link::mavlink::protocol::MavHeader,
        frame: &[u8],
    ) {
        let mut command = bridge::mavlink_position_to_command(&target);
        command.sequence = u32::from(header.seq);
        let provenance = MavlinkCommandProvenance::new(
            &mut self.command_source_epochs,
            source,
            header,
            target.time_boot_ms,
            MavlinkCommandFamily::PositionTargetLocalNed,
            frame,
        );
        self.flight_cmd = Some(ReceivedCommand {
            command: SystemCommand::FlightControl(command),
            provenance: Some(provenance),
        });
    }

    #[cfg(test)]
    pub(super) fn inject_attitude_target(&mut self, target: SetAttitudeTarget) {
        let header = aviate_link::mavlink::protocol::MavHeader {
            payload_len: 39,
            incompat_flags: 0,
            compat_flags: 0,
            seq: 0,
            sysid: 255,
            compid: 190,
            msgid: 82,
        };
        self.handle_set_attitude_target(
            target,
            std::net::SocketAddr::from(([127, 0, 0, 1], 30_000)),
            header,
            &[1; 51],
        );
    }

    pub(super) fn handle_command_long(&mut self, command: CommandLong) {
        if command.command != mav_cmd::COMPONENT_ARM_DISARM {
            self.send_command_ack(command.command, mav_result::UNSUPPORTED, 0);
            return;
        }
        let forced = command.param2 == FORCE_ARM_DISARM_MAGIC;
        if command.param1 == 1.0 {
            info!("received Arm command");
            self.command = Some(SystemCommand::Arm);
        } else if command.param1 == 0.0 && forced {
            warn!("received emergency terminate command");
            self.command = Some(SystemCommand::EmergencyTerminate);
        } else if command.param1 == 0.0 {
            info!("received Disarm command");
            self.command = Some(SystemCommand::Disarm);
        } else {
            warn!("Arm command has invalid parameter {}", command.param1);
            self.send_command_ack(command.command, mav_result::DENIED, 0);
        }
    }

    pub(super) fn send_command_ack(&mut self, command: u16, result: u8, detail: i32) {
        let acknowledgement = CommandAck {
            command,
            result,
            progress: 0,
            result_param2: detail,
            target_system: 255,
            target_component: 0,
        };
        if let Some(destination) = self.gcs_addr {
            self.send_message_to(&MavMessage::CommandAck(acknowledgement), destination);
        } else {
            warn!("cannot send command acknowledgement without a GCS address");
        }
    }

    pub(super) fn send_heartbeat(&mut self) {
        let heartbeat = Heartbeat {
            mav_type: MavType::Quadrotor as u8,
            autopilot: MavAutopilot::Aviate as u8,
            base_mode: if self.armed {
                MavModeFlag::SAFETY_ARMED.0 | MavModeFlag::HIL_ENABLED.0
            } else {
                MavModeFlag::HIL_ENABLED.0
            },
            custom_mode: 0,
            system_status: self.system_status,
            mavlink_version: 3,
        };
        self.send_message_to(&MavMessage::Heartbeat(heartbeat), self.config.gcs_addr);
        if let Some(destination) = self.gcs_addr {
            if destination != self.config.gcs_addr {
                self.send_message_to(&MavMessage::Heartbeat(heartbeat), destination);
            }
        }
    }

    fn send_message_to(&mut self, message: &MavMessage, destination: std::net::SocketAddr) {
        let mut buffer = [0_u8; 300];
        let system_id = self.config.instance.wrapping_add(1);
        if let Some(length) = serialize_mavlink(message, self.seq, system_id, 1, &mut buffer) {
            self.seq = self.seq.wrapping_add(1);
            self.socket.send_to(&buffer[..length], destination).ok();
            self.tx_count = self.tx_count.wrapping_add(1);
        }
    }

    /// Set the MAVLink armed state.
    pub fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
        info!("MAVLink armed state is {armed}");
    }

    /// Set the MAVLink system state.
    pub fn set_system_status(&mut self, status: MavState) {
        self.system_status = status as u8;
    }

    /// Return the MAVLink armed state.
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Get received and sent frame counts.
    #[must_use]
    pub fn stats(&self) -> (u64, u64) {
        (self.rx_count, self.tx_count)
    }

    /// Get the simulator actuator endpoint.
    #[must_use]
    pub fn simulator_addr(&self) -> std::net::SocketAddr {
        self.config.simulator_addr()
    }

    /// Get the local sensor port.
    #[must_use]
    pub fn sensor_port(&self) -> u16 {
        self.config.sensor_port()
    }

    /// Get the current GCS endpoint.
    #[must_use]
    pub fn gcs_addr(&self) -> Option<std::net::SocketAddr> {
        self.gcs_addr
    }

    /// Get the immutable XIL configuration.
    #[must_use]
    pub const fn config(&self) -> &XilConfig {
        &self.config
    }

    /// Receive one command with its raw-frame identity.
    pub fn recv_command_with_provenance(&mut self) -> Option<ReceivedCommand> {
        self.poll();
        self.command
            .take()
            .map(|command| ReceivedCommand {
                command,
                provenance: None,
            })
            .or_else(|| self.flight_cmd.take())
    }
}

impl CommandHal for SitlIO {
    fn recv_command(&mut self) -> Option<SystemCommand> {
        self.recv_command_with_provenance()
            .map(|received| received.command)
    }

    fn report_outcome(&mut self, command: &SystemCommand, outcome: CommandOutcome) {
        let mav_command = match command {
            SystemCommand::Arm | SystemCommand::Disarm | SystemCommand::EmergencyTerminate => {
                mav_cmd::COMPONENT_ARM_DISARM
            }
            SystemCommand::FlightControl(_) => return,
        };
        let (result, detail) = match outcome {
            CommandOutcome::Accepted => (mav_result::ACCEPTED, 0),
            CommandOutcome::ArmRejected { missing, .. } if outcome.is_retryable() => (
                mav_result::TEMPORARILY_REJECTED,
                i32::try_from(missing.bits()).unwrap_or(i32::MAX),
            ),
            CommandOutcome::ArmRejected { missing, .. } => (
                mav_result::DENIED,
                i32::try_from(missing.bits()).unwrap_or(i32::MAX),
            ),
            CommandOutcome::DisarmRejected(_) => (mav_result::DENIED, 0),
        };
        self.send_command_ack(mav_command, result, detail);
    }
}

#[cfg(test)]
mod tests;
