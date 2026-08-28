//! Build typed flight setpoints for backend directives.

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::{Meters, NormalizedThrust, Radians};

use crate::SimulatorBackend;

use super::MissionRunner;

impl<B: SimulatorBackend> MissionRunner<B> {
    pub(super) fn attitude_command(&mut self, quaternion: [f32; 4], thrust: f32) -> Command {
        let command = Command {
            mode: ControlMode::Attitude,
            setpoint: Setpoint {
                attitude: Some(Quaternion::new(
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                )),
                collective_thrust: NormalizedThrust(thrust),
                ..Setpoint::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: self.next_command_sequence,
            source: CommandSource::Gcs,
        };
        self.next_command_sequence = self.next_command_sequence.wrapping_add(1);
        command
    }

    pub(super) fn position_command(&mut self, position: [f32; 3], heading: f32) -> Command {
        let command = Command {
            mode: ControlMode::PositionHold,
            setpoint: Setpoint {
                position: Some([
                    Meters(position[0]),
                    Meters(position[1]),
                    Meters(position[2]),
                ]),
                heading: Some(Radians(heading)),
                ..Setpoint::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: self.next_command_sequence,
            source: CommandSource::Gcs,
        };
        self.next_command_sequence = self.next_command_sequence.wrapping_add(1);
        command
    }
}
