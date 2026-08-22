//! Simulator-neutral MAVLink HIL execution.

use std::io;
use std::net::SocketAddr;
use std::time::Duration;

use aviate_hal_xil::{
    SimActuatorCmd, SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
};

use crate::geodetic::NedOrigin;
use crate::link::Link;
use crate::messages::{Heartbeat, HilActuatorControls, HilGps, HilSensor, HilStateQuaternion};
use crate::sensor_fields::SensorFields;
use crate::transport::{HilTransport, HilTransportConfig};
use crate::transport_tcp::{HilTcpConfig, HilTcpTransport};

/// Lockstep reply flag in `HIL_ACTUATOR_CONTROLS.flags`.
pub const LOCKSTEP_ACTUATOR_FLAG: u64 = 1;

/// Transport-confirmed fields from one actuator send.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ActuatorSendReceipt {
    /// Timestamp in the sent reply.
    pub echoed_timestamp_us: u64,
    /// True when the sent reply has the lockstep flag.
    pub lockstep: bool,
}

/// MAVLink HIL endpoint configuration.
#[derive(Clone, Debug)]
pub struct HilBackendConfig {
    /// Local UDP input port.
    pub local_port: u16,
    /// Simulator endpoint.
    pub simulator_addr: SocketAddr,
    /// Outgoing MAVLink system identifier.
    pub sys_id: u8,
    /// Outgoing MAVLink component identifier.
    pub comp_id: u8,
}

impl Default for HilBackendConfig {
    fn default() -> Self {
        Self {
            local_port: 14_560,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], 14_560)),
            sys_id: 1,
            comp_id: 1,
        }
    }
}

/// MAVLink HIL adapter for one simulator session.
pub struct HilBackend {
    link: Link,
    origin: NedOrigin,
    last_sensor_time_us: u64,
}

impl HilBackend {
    /// Create a UDP HIL adapter.
    pub fn new(config: HilBackendConfig) -> io::Result<Self> {
        let transport = HilTransport::new(HilTransportConfig {
            local_port: config.local_port,
            simulator_addr: config.simulator_addr,
            sys_id: config.sys_id,
            comp_id: config.comp_id,
        })?;
        Ok(Self {
            link: Link::Udp(transport),
            origin: NedOrigin::default(),
            last_sensor_time_us: 0,
        })
    }

    /// Create a TCP HIL adapter. The adapter reconnects during polling.
    #[must_use]
    pub fn connect_tcp(config: HilBackendConfig) -> Self {
        Self {
            link: Link::Tcp(HilTcpTransport::new(HilTcpConfig {
                simulator_addr: config.simulator_addr,
                sys_id: config.sys_id,
                comp_id: config.comp_id,
            })),
            origin: NedOrigin::default(),
            last_sensor_time_us: 0,
        }
    }

    /// Poll the link and convert one available sensor group.
    pub fn poll(&mut self) -> Option<SimSensorPacket> {
        self.link.poll();
        let sensor = self.link.take_sensor();
        let gps = self.link.take_gps();
        if sensor.is_none() && gps.is_none() {
            return None;
        }
        let mut packet = SimSensorPacket::default();
        if let Some(sensor) = sensor {
            self.last_sensor_time_us = sensor.time_usec;
            apply_sensor(&mut packet, sensor);
        }
        if let Some(gps) = gps {
            apply_gps(&mut packet, &mut self.origin, gps);
        }
        Some(packet)
    }

    /// Take the newest simulator truth message.
    pub fn take_state_quaternion(&mut self) -> Option<HilStateQuaternion> {
        self.link.take_state_quaternion()
    }

    /// Send one actuator command.
    pub fn send_actuators(&mut self, command: &SimActuatorCmd) -> io::Result<()> {
        self.send_actuators_with_receipt(command).map(|_| ())
    }

    /// Send one actuator command and return its encoded evidence.
    pub fn send_actuators_with_receipt(
        &mut self,
        command: &SimActuatorCmd,
    ) -> io::Result<ActuatorSendReceipt> {
        let message = actuator_message(command, self.last_sensor_time_us, self.link.now_us());
        self.link.send_actuator_controls(&message)?;
        Ok(ActuatorSendReceipt {
            echoed_timestamp_us: message.time_usec,
            lockstep: message.flags & LOCKSTEP_ACTUATOR_FLAG != 0,
        })
    }

    /// Send one heartbeat.
    pub fn send_heartbeat(&mut self, armed: bool) -> io::Result<()> {
        self.link
            .send_heartbeat(&Heartbeat::new_quadrotor_hil(armed))
    }

    /// Get the link clock in microseconds.
    #[must_use]
    pub fn now_us(&self) -> u64 {
        self.link.now_us()
    }

    /// Return true when a link can exchange frames.
    #[must_use]
    pub fn connected(&self) -> bool {
        match &self.link {
            Link::Udp(_) => true,
            Link::Tcp(transport) => transport.connected(),
        }
    }

    /// Wait until a TCP sample is available or the timeout expires.
    pub fn wait_readable(&mut self, timeout: Duration) -> bool {
        match &mut self.link {
            Link::Tcp(transport) => transport.wait_readable(timeout),
            Link::Udp(_) => false,
        }
    }

    /// Get TCP counters or compatible UDP counters.
    #[must_use]
    pub fn tcp_stats(&self) -> (u64, u64, u64, u64, u64) {
        match &self.link {
            Link::Tcp(transport) => transport.stats(),
            Link::Udp(transport) => {
                let (received, sent, crc_failures) = transport.stats();
                (received, sent, crc_failures, 0, 0)
            }
        }
    }

    /// Get received, sent, and CRC-failure counters.
    #[must_use]
    pub fn stats(&self) -> (u64, u64, u64) {
        let (received, sent, crc_failures, _, _) = self.tcp_stats();
        (received, sent, crc_failures)
    }

    /// Get the UDP input port. TCP links return zero.
    #[must_use]
    pub fn local_port(&self) -> u16 {
        match &self.link {
            Link::Udp(transport) => transport.local_port(),
            Link::Tcp(_) => 0,
        }
    }
}

fn apply_sensor(packet: &mut SimSensorPacket, sensor: HilSensor) {
    packet.timestamp_us = sensor.time_usec;
    let updated = SensorFields::from_bits(sensor.fields_updated);
    packet.presence_mask = updated.known_presence_mask();
    if updated.imu() {
        packet.imu = Some(SimImuData {
            accel: [sensor.xacc, sensor.yacc, sensor.zacc],
            gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
            temperature: Some(sensor.temperature),
        });
    }
    if updated.baro() {
        packet.baro = Some(SimBaroData {
            pressure_pa: sensor.abs_pressure * 100.0,
            differential_pressure_pa: updated
                .differential_pressure()
                .then_some(sensor.diff_pressure * 100.0),
            pressure_altitude_m: updated.pressure_altitude().then_some(sensor.pressure_alt),
            temperature_c: sensor.temperature,
        });
    }
    if updated.mag() {
        packet.mag = Some(SimMagData {
            field_ut: [
                sensor.xmag * 100.0,
                sensor.ymag * 100.0,
                sensor.zmag * 100.0,
            ],
        });
    }
}

fn apply_gps(packet: &mut SimSensorPacket, origin: &mut NedOrigin, gps: HilGps) {
    if packet.timestamp_us == 0 {
        packet.timestamp_us = gps.time_usec;
    }
    let fix = decode_fix(gps.fix_type);
    let lat_deg = f64::from(gps.lat) / 1e7;
    let lon_deg = f64::from(gps.lon) / 1e7;
    let alt_m = gps.alt as f32 / 1_000.0;
    packet.gnss = Some(SimGnssData {
        lat_deg,
        lon_deg,
        alt_m,
        position_ned: origin.project(lat_deg, lon_deg, alt_m, fix),
        vel_ned: [
            f32::from(gps.vn) / 100.0,
            f32::from(gps.ve) / 100.0,
            f32::from(gps.vd) / 100.0,
        ],
        fix,
        h_acc: f32::from(gps.eph) / 100.0,
        v_acc: f32::from(gps.epv) / 100.0,
        satellites: gps.satellites_visible,
    });
}

fn decode_fix(fix_type: u8) -> SimGnssFix {
    match fix_type {
        2 => SimGnssFix::TwoD,
        3 | 4 => SimGnssFix::ThreeD,
        5 => SimGnssFix::RtkFloat,
        6 => SimGnssFix::RtkFixed,
        _ => SimGnssFix::None,
    }
}

fn actuator_message(
    command: &SimActuatorCmd,
    last_sensor_time_us: u64,
    link_time_us: u64,
) -> HilActuatorControls {
    let mut controls = [0.0_f32; 16];
    let count = usize::from(command.count).min(controls.len());
    controls[..count].copy_from_slice(&command.outputs[..count]);
    HilActuatorControls {
        time_usec: if last_sensor_time_us == 0 {
            link_time_us
        } else {
            last_sensor_time_us
        },
        controls,
        flags: LOCKSTEP_ACTUATOR_FLAG,
        mode: if command.armed {
            HilActuatorControls::MODE_FLAG_ARMED
        } else {
            0
        },
    }
}

#[cfg(test)]
mod tests;
