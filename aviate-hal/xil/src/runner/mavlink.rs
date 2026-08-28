//! MAVLink command client for simulator adapters.

use std::net::UdpSocket;
use std::time::Duration;

use aviate_link::mavlink::protocol::{
    CommandLong, Heartbeat, MavHeader, SetAttitudeTarget, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{
    mav_cmd, parse_mavlink, serialize_mavlink, MavAutopilot, MavMessage, MavState, MavType,
};

use crate::{SimulatorError, SimulatorOperation, XilNetConfig};

/// Send MAVLink ground-control commands to a flight controller.
pub struct MavClient {
    socket: UdpSocket,
    target_addr: std::net::SocketAddr,
    seq: u8,
    target_system: u8,
    target_component: u8,
}

impl MavClient {
    /// Create a MAVLink client for one simulator instance.
    pub fn new(instance: u8) -> Result<Self, SimulatorError> {
        Self::new_with_net(instance, XilNetConfig::default())
    }

    /// Create a MAVLink client with the specified network configuration.
    pub fn new_with_net(instance: u8, net: XilNetConfig) -> Result<Self, SimulatorError> {
        let socket = UdpSocket::bind("127.0.0.1:0").map_err(|source| SimulatorError::Io {
            operation: SimulatorOperation::Connect,
            source,
        })?;
        socket
            .set_nonblocking(true)
            .map_err(|source| SimulatorError::Io {
                operation: SimulatorOperation::Connect,
                source,
            })?;

        let gcs_port = net.port(instance as u16, crate::PortSlot::SensorIn);
        let target_addr = std::net::SocketAddr::from(([127, 0, 0, 1], gcs_port));

        Ok(Self {
            socket,
            target_addr,
            seq: 0,
            target_system: instance.wrapping_add(1),
            target_component: 1,
        })
    }

    /// Send a MAVLink message.
    fn send(&mut self, msg: &MavMessage) -> bool {
        let mut buf = [0u8; 300];
        if let Some(len) = serialize_mavlink(msg, self.seq, 255, 190, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            self.socket.send_to(&buf[..len], self.target_addr).is_ok()
        } else {
            false
        }
    }

    /// Receive one available MAVLink message.
    fn recv(&mut self) -> Option<(MavHeader, MavMessage)> {
        let mut buf = [0u8; 512];
        match self.socket.recv_from(&mut buf) {
            Ok((len, src)) => {
                if self.target_addr.port() == 0 {
                    self.target_addr = src;
                }
                match parse_mavlink(&buf[..len]) {
                    Ok((msg, _sig, _consumed)) => {
                        // The parser validates these header bytes but does not return them.
                        let header = MavHeader {
                            payload_len: buf[1],
                            incompat_flags: buf[2],
                            compat_flags: buf[3],
                            seq: buf[4],
                            sysid: buf[5],
                            compid: buf[6],
                            msgid: (buf[7] as u32)
                                | ((buf[8] as u32) << 8)
                                | ((buf[9] as u32) << 16),
                        };
                        Some((header, msg))
                    }
                    Err(_) => None,
                }
            }
            Err(_) => None,
        }
    }

    /// Send a heartbeat.
    pub fn send_heartbeat(&mut self) -> bool {
        let hb = Heartbeat {
            mav_type: MavType::Gcs as u8,
            autopilot: MavAutopilot::Generic as u8,
            base_mode: 0,
            custom_mode: 0,
            system_status: MavState::Active as u8,
            mavlink_version: 3,
        };
        self.send(&MavMessage::Heartbeat(hb))
    }

    /// Send an arm command.
    pub fn send_arm(&mut self) -> bool {
        let cmd = CommandLong {
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            target_system: self.target_system,
            target_component: self.target_component,
            confirmation: 0,
        };
        self.send(&MavMessage::CommandLong(cmd))
    }

    /// Send a disarm command.
    pub fn send_disarm(&mut self) -> bool {
        let cmd = CommandLong {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            target_system: self.target_system,
            target_component: self.target_component,
            confirmation: 0,
        };
        self.send(&MavMessage::CommandLong(cmd))
    }

    /// Send an attitude and thrust target.
    pub fn send_attitude_target(&mut self, q: [f32; 4], thrust: f32) -> bool {
        let tgt = SetAttitudeTarget {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            type_mask: 0x07,
            q,
            body_roll_rate: 0.0,
            body_pitch_rate: 0.0,
            body_yaw_rate: 0.0,
            thrust,
            thrust_body: [0.0, 0.0, 0.0],
        };
        self.send(&MavMessage::SetAttitudeTarget(tgt))
    }

    /// Send a position target in the NED frame.
    pub fn send_position_target(&mut self, x: f32, y: f32, z: f32, yaw: f32) -> bool {
        let tgt = SetPositionTargetLocalNed {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            coordinate_frame: 1,
            type_mask: 0x0DF8,
            x,
            y,
            z,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw,
            yaw_rate: 0.0,
        };
        self.send(&MavMessage::SetPositionTargetLocalNed(tgt))
    }

    /// Connect when the target flight controller sends a heartbeat.
    pub fn try_connect(&mut self) -> bool {
        if self.socket.set_nonblocking(false).is_err()
            || self
                .socket
                .set_read_timeout(Some(Duration::from_millis(100)))
                .is_err()
        {
            return false;
        }
        let mut connected = false;
        for _ in 0..100 {
            self.send_heartbeat();
            while let Some((header, MavMessage::Heartbeat(_))) = self.recv() {
                if header.sysid == self.target_system {
                    connected = true;
                    break;
                }
            }
            if connected {
                break;
            }
        }
        if self.socket.set_read_timeout(None).is_err() || self.socket.set_nonblocking(true).is_err()
        {
            return false;
        }
        connected
    }
}
