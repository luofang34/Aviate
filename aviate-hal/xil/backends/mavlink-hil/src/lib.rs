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

pub mod geodetic;
pub mod messages;
pub mod transport;
pub mod transport_tcp;
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
pub use transport_tcp::{HilTcpConfig, HilTcpTransport};
pub use wire::{parse_frame, serialize_frame, MavFrame, ParseError};

/// Bit 0 of `HIL_ACTUATOR_CONTROLS.flags`: this command answers the
/// sensor sample that produced it.
pub const LOCKSTEP_ACTUATOR_FLAG: u64 = 1;

/// The `HIL_SENSOR.fields_updated` bitmap, which declares WHICH lanes a
/// sample actually carries.
#[derive(Debug, Clone, Copy)]
pub struct SensorFields(u32);

impl SensorFields {
    /// Bits 0..=2 accelerometer, 3..=5 gyro, 6..=8 magnetometer,
    /// 9 absolute pressure, 10 differential pressure, 11 pressure
    /// altitude, 12 temperature.
    const ACCEL: u32 = 0b111;
    const GYRO: u32 = 0b111 << 3;
    const MAG: u32 = 0b111 << 6;
    const BARO: u32 = 1 << 9;

    /// Reads the bitmap. A sample that declares NOTHING is treated as
    /// declaring everything: some simulators leave the field zero, and
    /// refusing their every sample would be a worse failure than
    /// trusting a lane they did populate.
    #[must_use]
    pub fn from_bits(bits: u32) -> Self {
        Self(if bits == 0 { u32::MAX } else { bits })
    }

    /// Whether the sample carries accelerometer and gyro lanes.
    #[must_use]
    pub fn imu(self) -> bool {
        self.0 & Self::ACCEL != 0 && self.0 & Self::GYRO != 0
    }

    /// Whether the sample carries magnetometer lanes.
    #[must_use]
    pub fn mag(self) -> bool {
        self.0 & Self::MAG != 0
    }

    /// Whether the sample carries an absolute-pressure lane.
    #[must_use]
    pub fn baro(self) -> bool {
        self.0 & Self::BARO != 0
    }
}

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
    link: Link,
    origin: geodetic::NedOrigin,
    /// Timestamp of the newest sensor sample received, echoed into the
    /// actuator answer. Lockstep bridges pair an answer with the sample
    /// it answers BY THIS ECHO: the bridge compares the answer's clock
    /// against its own sensor clock, so an answer stamped from the
    /// flight controller's clock is a cross-clock comparison that
    /// rejects the answer whenever the controller's clock happens to
    /// lag the bridge's — a wedge that ends in the bridge's response
    /// timeout and a mid-flight actuator zeroing.
    last_sensor_time_us: u64,
}

/// Which socket carries this backend's HIL stream. A datagram bridge is
/// peer-symmetric and always "up"; a stream bridge is dialed, can close,
/// and reports that.
enum Link {
    /// Datagram HIL (jMAVSim and the classic simulators).
    Udp(HilTransport),
    /// Stream HIL, dialed by the flight controller (the X-Plane bridge).
    Tcp(HilTcpTransport),
}

impl Link {
    fn poll(&mut self) {
        match self {
            Self::Udp(transport) => transport.poll(),
            Self::Tcp(transport) => transport.poll(),
        }
    }

    fn take_sensor(&mut self) -> Option<HilSensor> {
        match self {
            Self::Udp(transport) => transport.take_sensor(),
            Self::Tcp(transport) => transport.take_sensor(),
        }
    }

    fn take_gps(&mut self) -> Option<HilGps> {
        match self {
            Self::Udp(transport) => transport.take_gps(),
            Self::Tcp(transport) => transport.take_gps(),
        }
    }

    fn take_state_quaternion(&mut self) -> Option<HilStateQuaternion> {
        match self {
            Self::Udp(transport) => transport.take_state_quaternion(),
            Self::Tcp(transport) => transport.take_state_quaternion(),
        }
    }

    fn now_us(&self) -> u64 {
        match self {
            Self::Udp(transport) => transport.now_us(),
            Self::Tcp(transport) => transport.now_us(),
        }
    }

    fn send_actuator_controls(&mut self, controls: &HilActuatorControls) -> io::Result<()> {
        match self {
            Self::Udp(transport) => transport.send_actuator_controls(controls),
            Self::Tcp(transport) => transport.send_actuator_controls(controls),
        }
    }

    fn send_heartbeat(&mut self, heartbeat: &Heartbeat) -> io::Result<()> {
        match self {
            Self::Udp(transport) => transport.send_heartbeat(heartbeat),
            Self::Tcp(transport) => transport.send_heartbeat(heartbeat),
        }
    }
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

        Ok(Self {
            link: Link::Udp(transport),
            origin: geodetic::NedOrigin::default(),
            last_sensor_time_us: 0,
        })
    }

    /// Poll for incoming data
    ///
    /// Call this regularly to receive HIL messages from the simulator.
    /// Returns a sensor packet if new sensor data was received.
    ///
    /// The returned packet should be fed to SitlIO via `feed_sensor_packet()`.
    pub fn poll(&mut self) -> Option<SimSensorPacket> {
        self.link.poll();

        let sensor = self.link.take_sensor();
        let gps = self.link.take_gps();

        // If no new data, return None
        if sensor.is_none() && gps.is_none() {
            return None;
        }

        let mut packet = SimSensorPacket::default();

        // Convert HIL_SENSOR to simulator-neutral types. A bridge that
        // interpolates IMU substeps sends most samples with the
        // secondary sensors ABSENT, marking them in `fields_updated`;
        // publishing those zeroed lanes as measurements would feed the
        // estimator a magnetometer and barometer reading of zero at the
        // IMU rate. Only the lanes the sample declares are published.
        if let Some(sensor) = sensor {
            packet.timestamp_us = sensor.time_usec;
            self.last_sensor_time_us = sensor.time_usec;
            let updated = SensorFields::from_bits(sensor.fields_updated);

            if updated.imu() {
                packet.imu = Some(SimImuData {
                    accel: [sensor.xacc, sensor.yacc, sensor.zacc],
                    gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
                    temperature: Some(sensor.temperature),
                });
            }

            if updated.baro() {
                packet.baro = Some(SimBaroData {
                    // Convert hPa to Pa
                    pressure_pa: sensor.abs_pressure * 100.0,
                    temperature_c: sensor.temperature,
                });
            }

            if updated.mag() {
                packet.mag = Some(SimMagData {
                    // Convert Gauss to microTesla (1 Gauss = 100 uT)
                    field_ut: [
                        sensor.xmag * 100.0,
                        sensor.ymag * 100.0,
                        sensor.zmag * 100.0,
                    ],
                });
            }
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

            let lat_deg = (gps.lat as f64) / 1e7;
            let lon_deg = (gps.lon as f64) / 1e7;
            let alt_m = (gps.alt as f32) / 1000.0; // mm to m
                                                   // HIL_GPS carries WGS84 lat/lon/alt only, but the kernel
                                                   // consumes `position_ned` — an unprojected fix would hold the
                                                   // estimator at the origin forever. Project against an origin
                                                   // latched at the first usable fix.
            let position_ned = self.origin.project(lat_deg, lon_deg, alt_m, fix);

            packet.gnss = Some(SimGnssData {
                lat_deg,
                lon_deg,
                alt_m,
                position_ned,
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
        self.link.take_state_quaternion()
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
            // Echo the answered sample's clock; the transport clock is
            // only a fallback for an answer sent before any sample.
            time_usec: if self.last_sensor_time_us > 0 {
                self.last_sensor_time_us
            } else {
                self.link.now_us()
            },
            controls,
            // Bit 0 marks this command as the lockstep response to the
            // sensor sample that produced it. A bridge that paces its
            // sensor stream on actuator feedback drops the link when the
            // bit is absent, reading the flight controller as dead.
            flags: LOCKSTEP_ACTUATOR_FLAG,
            mode: if cmd.armed {
                HilActuatorControls::MODE_FLAG_ARMED
            } else {
                0
            },
        };

        self.link.send_actuator_controls(&hil_cmd)
    }

    /// Send a heartbeat to the simulator
    ///
    /// Required by some simulators (like jMAVSim) to initialize HIL communication.
    /// Should be called periodically (typically 1Hz) to maintain the connection.
    pub fn send_heartbeat(&mut self, armed: bool) -> io::Result<()> {
        let heartbeat = Heartbeat::new_quadrotor_hil(armed);
        self.link.send_heartbeat(&heartbeat)
    }

    /// Get current timestamp in microseconds
    pub fn now_us(&self) -> u64 {
        self.link.now_us()
    }

    /// Dials a STREAM HIL bridge that listens for the flight
    /// controller. A bridge that is not listening yet is not an error;
    /// the link retries on every poll.
    #[must_use]
    pub fn connect_tcp(config: HilBackendConfig) -> Self {
        Self {
            link: Link::Tcp(HilTcpTransport::new(HilTcpConfig {
                simulator_addr: config.simulator_addr,
                sys_id: config.sys_id,
                comp_id: config.comp_id,
            })),
            origin: geodetic::NedOrigin::default(),
            last_sensor_time_us: 0,
        }
    }

    /// Whether a stream link is up. A datagram link is always reported
    /// up: it has no connection to lose.
    #[must_use]
    pub fn connected(&self) -> bool {
        match &self.link {
            Link::Udp(_) => true,
            Link::Tcp(transport) => transport.connected(),
        }
    }

    /// Blocks until a stream link has a sample waiting or `timeout`
    /// elapses. A datagram link has nothing to wait on and returns
    /// immediately, leaving its caller to pace itself.
    pub fn wait_readable(&mut self, timeout: std::time::Duration) -> bool {
        match &mut self.link {
            Link::Tcp(transport) => transport.wait_readable(timeout),
            Link::Udp(_) => false,
        }
    }

    /// Stream-link statistics: received frames, sent frames, CRC
    /// failures, unsent commands, and successful connections.
    #[must_use]
    pub fn tcp_stats(&self) -> (u64, u64, u64, u64, u64) {
        match &self.link {
            Link::Tcp(transport) => transport.stats(),
            Link::Udp(transport) => {
                let (rx, tx, crc) = transport.stats();
                (rx, tx, crc, 0, 0)
            }
        }
    }

    /// Get statistics (rx_count, tx_count, crc_errors)
    pub fn stats(&self) -> (u64, u64, u64) {
        let (rx, tx, crc, _, _) = self.tcp_stats();
        (rx, tx, crc)
    }

    /// Get the local port (datagram links only; a dialed stream link
    /// has no meaningful local port to report).
    pub fn local_port(&self) -> u16 {
        match &self.link {
            Link::Udp(transport) => transport.local_port(),
            Link::Tcp(_) => 0,
        }
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

#[cfg(test)]
mod hil_contract_tests {
    use super::{SensorFields, LOCKSTEP_ACTUATOR_FLAG};

    #[test]
    fn a_sample_declaring_only_the_imu_publishes_only_the_imu() {
        // Bits 0..=5 are accelerometer and gyro; a bridge that
        // interpolates IMU substeps sends most samples exactly like
        // this, and reading the absent lanes would publish zeros as
        // measurements.
        let fields = SensorFields::from_bits(0b11_1111);
        assert!(fields.imu());
        assert!(!fields.mag(), "an undeclared magnetometer is not a reading");
        assert!(!fields.baro(), "an undeclared barometer is not a reading");
    }

    #[test]
    fn a_sample_declaring_everything_publishes_everything() {
        let fields = SensorFields::from_bits(0xFFFF_FFFF);
        assert!(fields.imu() && fields.mag() && fields.baro());
    }

    #[test]
    fn a_sample_declaring_nothing_is_trusted_whole() {
        // Some simulators leave the field zero. Refusing every sample
        // from them would be a worse failure than trusting the lanes
        // they populate.
        let fields = SensorFields::from_bits(0);
        assert!(fields.imu() && fields.mag() && fields.baro());
    }

    #[test]
    fn the_lockstep_flag_is_bit_zero() {
        // A bridge that paces its sensor stream on actuator feedback
        // drops the link when this bit is absent.
        assert_eq!(LOCKSTEP_ACTUATOR_FLAG, 1);
    }
}
