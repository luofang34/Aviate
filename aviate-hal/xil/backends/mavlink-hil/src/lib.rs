//! MAVLink HIL Backend
//!
//! Provides Hardware-In-The-Loop simulation support for legacy simulators
//! that use the standard MAVLink HIL protocol (jMAVSim, X-Plane, FlightGear, etc.).
//!
//! ## Protocol
//!
//! This backend implements the standard MAVLink v2 HIL messages:
//! - **HIL_SENSOR (107)**: Simulator → FC, IMU/baro/mag sensor data
//! - **HIL_GPS (113)**: Simulator → FC, GPS data
//! - **HIL_STATE_QUATERNION (115)**: Simulator → FC, ground truth state
//! - **HIL_ACTUATOR_CONTROLS (93)**: FC → Simulator, motor/servo commands
//!
//! ## Usage with SitlIO
//!
//! This backend is designed to integrate with SitlIO, the simulator-neutral
//! middleware. The typical usage pattern is:
//!
//! ```ignore
//! // Create backend
//! let mut hil = HilBackend::new(config)?;
//!
//! // In control loop:
//! // 1. Poll for sensor data and feed to SitlIO
//! if let Some(packet) = hil.poll() {
//!     sitl_io.feed_sensor_packet(&packet);
//! }
//!
//! // 2. Get actuator commands from SitlIO and send to simulator
//! if let Some(cmd) = sitl_io.take_actuator_cmd() {
//!     hil.send_actuators(&cmd)?;
//! }
//! ```
//!
//! ## Coordinate Frames
//!
//! All sensor data is expected in NED (North-East-Down) body frame,
//! which is the standard MAVLink convention.

#![forbid(unsafe_code)]
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]

pub mod messages;
pub mod transport;
pub mod wire;

use std::io;
use std::net::SocketAddr;

use aviate_hal_xil::{
    SimActuatorCmd, SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
};

pub use messages::{
    Heartbeat, HilActuatorControls, HilGps, HilMessage, HilSensor, HilStateQuaternion,
};
pub use transport::{HilTransport, HilTransportConfig};
pub use wire::{parse_frame, serialize_frame, MavFrame, ParseError};

/// HIL backend configuration
#[derive(Clone, Debug)]
pub struct HilBackendConfig {
    /// Local port to bind for receiving HIL data (default: 14560)
    pub local_port: u16,
    /// Remote simulator address (default: 127.0.0.1:14560)
    pub simulator_addr: SocketAddr,
    /// System ID for outgoing MAVLink messages (default: 1)
    pub sys_id: u8,
    /// Component ID for outgoing MAVLink messages (default: 1)
    pub comp_id: u8,
}

impl Default for HilBackendConfig {
    fn default() -> Self {
        Self {
            local_port: 14560,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], 14560)),
            sys_id: 1,
            comp_id: 1,
        }
    }
}

/// MAVLink HIL backend
///
/// Bridges legacy HIL simulators to the Aviate SITL infrastructure.
/// Converts between MAVLink HIL messages and simulator-neutral types.
///
/// This backend handles:
/// - Receiving HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION from the simulator
/// - Sending HIL_ACTUATOR_CONTROLS to the simulator
/// - Converting between MAVLink HIL format and SimSensorPacket/SimActuatorCmd
pub struct HilBackend {
    transport: HilTransport,
}

impl HilBackend {
    /// Create a new HIL backend
    pub fn new(config: HilBackendConfig) -> io::Result<Self> {
        let transport_config = HilTransportConfig {
            local_port: config.local_port,
            simulator_addr: config.simulator_addr,
            sys_id: config.sys_id,
            comp_id: config.comp_id,
        };

        let transport = HilTransport::new(transport_config)?;

        Ok(Self { transport })
    }

    /// Poll for incoming data
    ///
    /// Call this regularly to receive HIL messages from the simulator.
    /// Returns a sensor packet if new sensor data was received.
    ///
    /// The returned packet should be fed to SitlIO via `feed_sensor_packet()`.
    pub fn poll(&mut self) -> Option<SimSensorPacket> {
        self.transport.poll();

        let sensor = self.transport.take_sensor();
        let gps = self.transport.take_gps();

        // If no new data, return None
        if sensor.is_none() && gps.is_none() {
            return None;
        }

        let mut packet = SimSensorPacket::default();

        // Convert HIL_SENSOR to simulator-neutral types
        if let Some(sensor) = sensor {
            packet.timestamp_us = sensor.time_usec;

            packet.imu = Some(SimImuData {
                accel: [sensor.xacc, sensor.yacc, sensor.zacc],
                gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
                temperature: Some(sensor.temperature),
            });

            packet.baro = Some(SimBaroData {
                // Convert hPa to Pa
                pressure_pa: sensor.abs_pressure * 100.0,
                temperature_c: sensor.temperature,
            });

            packet.mag = Some(SimMagData {
                // Convert Gauss to microTesla (1 Gauss = 100 uT)
                field_ut: [
                    sensor.xmag * 100.0,
                    sensor.ymag * 100.0,
                    sensor.zmag * 100.0,
                ],
            });
        }

        // Convert HIL_GPS to simulator-neutral types
        if let Some(gps) = gps {
            if packet.timestamp_us == 0 {
                packet.timestamp_us = gps.time_usec;
            }

            let fix = match gps.fix_type {
                0 | 1 => SimGnssFix::None,
                2 => SimGnssFix::TwoD,
                3 => SimGnssFix::ThreeD,
                4 => SimGnssFix::ThreeD, // DGPS maps to 3D
                5 => SimGnssFix::RtkFloat,
                6 => SimGnssFix::RtkFixed,
                _ => SimGnssFix::None,
            };

            packet.gnss = Some(SimGnssData {
                lat_deg: (gps.lat as f64) / 1e7,
                lon_deg: (gps.lon as f64) / 1e7,
                alt_m: (gps.alt as f32) / 1000.0, // mm to m
                vel_ned: [
                    (gps.vn as f32) / 100.0, // cm/s to m/s
                    (gps.ve as f32) / 100.0,
                    (gps.vd as f32) / 100.0,
                ],
                fix,
                h_acc: (gps.eph as f32) / 100.0, // hdop * 100 to meters (approx)
                v_acc: (gps.epv as f32) / 100.0,
                satellites: gps.satellites_visible,
            });
        }

        Some(packet)
    }

    /// Take the last received state quaternion data
    ///
    /// HIL_STATE_QUATERNION contains ground truth vehicle state (attitude,
    /// position, velocity, acceleration) useful for simulation validation
    /// but not raw sensor data.
    pub fn take_state_quaternion(&mut self) -> Option<HilStateQuaternion> {
        self.transport.take_state_quaternion()
    }

    /// Send actuator command to simulator
    ///
    /// Converts the simulator-neutral actuator command to HIL_ACTUATOR_CONTROLS
    /// and sends it to the legacy simulator.
    pub fn send_actuators(&mut self, cmd: &SimActuatorCmd) -> io::Result<()> {
        let mut controls = [0.0f32; 16];
        for (i, &output) in cmd.outputs.iter().enumerate().take(cmd.count as usize) {
            controls[i] = output;
        }

        let hil_cmd = HilActuatorControls {
            time_usec: self.transport.now_us(),
            controls,
            flags: 0,
            mode: if cmd.armed {
                HilActuatorControls::MODE_FLAG_ARMED
            } else {
                0
            },
        };

        self.transport.send_actuator_controls(&hil_cmd)
    }

    /// Send a heartbeat to the simulator
    ///
    /// Required by some simulators (like jMAVSim) to initialize HIL communication.
    /// Should be called periodically (typically 1Hz) to maintain the connection.
    pub fn send_heartbeat(&mut self, armed: bool) -> io::Result<()> {
        let heartbeat = Heartbeat::new_quadrotor_hil(armed);
        self.transport.send_heartbeat(&heartbeat)
    }

    /// Get current timestamp in microseconds
    pub fn now_us(&self) -> u64 {
        self.transport.now_us()
    }

    /// Get statistics (rx_count, tx_count, crc_errors)
    pub fn stats(&self) -> (u64, u64, u64) {
        self.transport.stats()
    }

    /// Get the local port
    pub fn local_port(&self) -> u16 {
        self.transport.local_port()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::UdpSocket;
    use std::thread;
    use std::time::Duration;

    fn find_available_port() -> Option<u16> {
        let socket = UdpSocket::bind("127.0.0.1:0"); // COV:EXCL(TEST)
        assert!(socket.is_ok());
        let Ok(socket) = socket else {
            return None;
        };

        let addr = socket.local_addr(); // COV:EXCL(TEST)
        assert!(addr.is_ok());
        let Ok(addr) = addr else {
            return None;
        };
        Some(addr.port())
    }

    fn bind_sim_socket(port: u16) -> Option<UdpSocket> {
        let socket = UdpSocket::bind(("127.0.0.1", port)); // COV:EXCL(TEST)
        assert!(socket.is_ok());
        let Ok(socket) = socket else {
            return None;
        };
        Some(socket)
    }

    fn create_backend(config: HilBackendConfig) -> Option<HilBackend> {
        let backend = HilBackend::new(config); // COV:EXCL(TEST)
        assert!(backend.is_ok());
        let Ok(backend) = backend else {
            return None;
        };
        Some(backend)
    }

    fn serialize_test_frame(msg: &HilMessage, buf: &mut [u8]) -> Option<usize> {
        let len = serialize_frame(msg, 1, 1, 1, buf); // COV:EXCL(TEST)
        assert!(len.is_some());
        len
    }

    #[test]
    fn test_backend_create() {
        let Some(port) = find_available_port() else {
            return;
        };
        let config = HilBackendConfig {
            local_port: port,
            ..Default::default()
        };
        let backend = HilBackend::new(config);
        assert!(backend.is_ok());
    }

    #[test]
    fn test_backend_sensor_conversion() {
        let Some(port1) = find_available_port() else {
            return;
        };
        let Some(port2) = find_available_port() else {
            return;
        };

        let config = HilBackendConfig {
            local_port: port1,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], port2)),
            ..Default::default()
        };
        let Some(mut backend) = create_backend(config) else {
            return;
        };

        // Send HIL_SENSOR from "simulator"
        let Some(sim_socket) = bind_sim_socket(port2) else {
            return;
        };

        let sensor = HilSensor {
            time_usec: 1000000,
            xacc: 0.0,
            yacc: 0.0,
            zacc: -9.81,
            xgyro: 0.01,
            ygyro: 0.02,
            zgyro: 0.03,
            xmag: 0.2, // Gauss
            ymag: 0.0,
            zmag: 0.4,
            abs_pressure: 1013.25, // hPa
            diff_pressure: 0.0,
            pressure_alt: 0.0,
            temperature: 25.0,
            fields_updated: 0xFFFFFFFF,
            id: 0,
        };

        let msg = HilMessage::Sensor(sensor);
        let mut buf = [0u8; 256];
        let Some(len) = serialize_test_frame(&msg, &mut buf) else {
            return;
        };

        assert!(sim_socket
            .send_to(&buf[..len], ("127.0.0.1", port1))
            .is_ok()); // COV:EXCL(TEST)

        thread::sleep(Duration::from_millis(10));

        let packet = backend.poll(); // COV:EXCL(TEST)
        assert!(packet.is_some());
        let Some(packet) = packet else {
            return;
        };
        assert!(packet.imu.is_some());
        assert!(packet.baro.is_some());
        assert!(packet.mag.is_some());

        // Check IMU conversion
        let Some(imu) = packet.imu else {
            return;
        }; // COV:EXCL(TEST)
        assert!((imu.accel[2] - (-9.81)).abs() < 1e-6);
        assert!((imu.gyro[0] - 0.01).abs() < 1e-6);

        // Check baro conversion (hPa to Pa)
        let Some(baro) = packet.baro else {
            return;
        }; // COV:EXCL(TEST)
        assert!((baro.pressure_pa - 101325.0).abs() < 1.0);

        // Check mag conversion (Gauss to uT)
        let Some(mag) = packet.mag else {
            return;
        }; // COV:EXCL(TEST)
        assert!((mag.field_ut[0] - 20.0).abs() < 1e-6); // 0.2 Gauss = 20 uT
    }

    #[test]
    fn test_backend_actuator_send() {
        let Some(port1) = find_available_port() else {
            return;
        };
        let Some(port2) = find_available_port() else {
            return;
        };

        let config = HilBackendConfig {
            local_port: port1,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], port2)),
            ..Default::default()
        };
        let Some(mut backend) = create_backend(config) else {
            return;
        };

        // Set up "simulator" to receive
        let Some(sim_socket) = bind_sim_socket(port2) else {
            return;
        };
        assert!(sim_socket.set_nonblocking(true).is_ok()); // COV:EXCL(TEST)

        // Send actuator command
        let cmd = SimActuatorCmd {
            timestamp_us: 1000000,
            outputs: [
                0.5, 0.6, 0.7, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ],
            count: 4,
            armed: true,
        };

        assert!(backend.send_actuators(&cmd).is_ok()); // COV:EXCL(TEST)

        thread::sleep(Duration::from_millis(10));

        // Receive on simulator side
        let mut buf = [0u8; 256];
        let received = sim_socket.recv_from(&mut buf); // COV:EXCL(TEST)
        assert!(received.is_ok());
        let Ok((len, _)) = received else {
            return;
        };

        let frame = parse_frame(&buf[..len]); // COV:EXCL(TEST)
        assert!(frame.is_ok());
        let Ok((frame, _)) = frame else {
            return;
        };
        assert!(matches!(&frame.message, HilMessage::ActuatorControls(_)));
        let HilMessage::ActuatorControls(ctrl) = frame.message else {
            return;
        };
        assert!(ctrl.is_armed());
        assert!((ctrl.controls[0] - 0.5).abs() < 1e-6);
        assert!((ctrl.controls[1] - 0.6).abs() < 1e-6);
    }

    #[test]
    fn test_backend_gps_conversion() {
        let Some(port1) = find_available_port() else {
            return;
        };
        let Some(port2) = find_available_port() else {
            return;
        };

        let config = HilBackendConfig {
            local_port: port1,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], port2)),
            ..Default::default()
        };
        let Some(mut backend) = create_backend(config) else {
            return;
        };

        let Some(sim_socket) = bind_sim_socket(port2) else {
            return;
        };

        let gps = HilGps {
            time_usec: 1000000,
            lat: 473977420, // 47.3977420 deg
            lon: 85455940,  // 8.5455940 deg
            alt: 488000,    // 488m in mm
            eph: 100,
            epv: 150,
            vel: 500,
            vn: 100, // 1 m/s north
            ve: 200, // 2 m/s east
            vd: -50, // -0.5 m/s down (climbing)
            cog: 9000,
            fix_type: 3,
            satellites_visible: 12,
            id: 0,
            yaw: 0,
        };

        let msg = HilMessage::Gps(gps);
        let mut buf = [0u8; 256];
        let Some(len) = serialize_test_frame(&msg, &mut buf) else {
            return;
        };

        assert!(sim_socket
            .send_to(&buf[..len], ("127.0.0.1", port1))
            .is_ok()); // COV:EXCL(TEST)

        thread::sleep(Duration::from_millis(10));

        let packet = backend.poll(); // COV:EXCL(TEST)
        assert!(packet.is_some());
        let Some(packet) = packet else {
            return;
        };
        assert!(packet.gnss.is_some());

        let Some(gnss) = packet.gnss else {
            return;
        }; // COV:EXCL(TEST)
        assert!((gnss.lat_deg - 47.397742).abs() < 1e-6);
        assert!((gnss.lon_deg - 8.545594).abs() < 1e-6);
        assert!((gnss.alt_m - 488.0).abs() < 0.1);
        assert!((gnss.vel_ned[0] - 1.0).abs() < 0.01);
        assert!((gnss.vel_ned[1] - 2.0).abs() < 0.01);
        assert!(matches!(gnss.fix, SimGnssFix::ThreeD));
    }

    #[test]
    fn test_poll_returns_none_when_no_data() {
        let Some(port) = find_available_port() else {
            return;
        };
        let config = HilBackendConfig {
            local_port: port,
            ..Default::default()
        };
        let Some(mut backend) = create_backend(config) else {
            return;
        };

        // Poll without sending any data - should return None
        assert!(backend.poll().is_none());
    }
}
