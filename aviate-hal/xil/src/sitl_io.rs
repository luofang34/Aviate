//! SITL Transport Layer
//!
//! Simulator-neutral middleware between flight controller and simulator backends.
//! This is the transport layer for SITL - it buffers sensor and actuator data,
//! but does NOT implement HAL traits. HAL abstraction lives in `aviate-hal-io`.
//!
//! ## Responsibilities
//!
//! - **Sensor input**: Receives sensor data from simulator backend via Rust API
//! - **Actuator output**: Provides actuator commands to simulator backend via Rust API
//! - **Command input**: Receives arm/disarm and setpoint commands via MAVLink
//! - **Heartbeat**: Maintains connection with GCS/mission_runner
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────────────┐
//! │                            aviate-hal-io                                │
//! │  BoardHal<I,B,M,G,T,A> implements SensorHal + ActuatorHal              │
//! │  - FakeImu, FakeBaro, FakeMag, FakeGnss (sensor drivers)              │
//! │  - FakeActuator (actuator driver)                                      │
//! └─────────────────────────────────────────────────────────────────────────┘
//!                    ↑ feed()                      ↓ set_actuator_cmd()
//! ┌─────────────────────────────────────────────────────────────────────────┐
//! │                     aviate-hal-xil (this module)                        │
//! │  SitlIO - Simulator-neutral middleware                                 │
//! │  - feed_sensor_packet() ← receives from backend                        │
//! │  - take_actuator_cmd() → provides to backend                           │
//! └─────────────────────────────────────────────────────────────────────────┘
//!                    ↑ Rust API                    ↓ Rust API
//! ┌─────────────────────────────────────────────────────────────────────────┐
//! │                  Simulator Backend (gazebo_bridge.rs)                   │
//! │  - ENU→NED coordinate conversion                                       │
//! │  - C FFI for Gazebo plugin integration                                 │
//! └─────────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Note on MAVLink
//!
//! MAVLink is used only for GCS commands and mission_runner test harness.
//! Sensor/actuator data uses direct Rust API (feed_sensor_packet, take_actuator_cmd)
//! for lower latency and cleaner architecture.

use std::io;
use std::net::UdpSocket;

use log::{info, warn};

use aviate_core::hal::SystemHal;
use aviate_core::time::{TimeSource, Timestamp};

use aviate_hal_io::{
    CommandHal, GnssFix, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading,
    SystemCommand,
};

use aviate_link::mavlink::protocol::{
    CommandAck, CommandLong, Heartbeat, SetAttitudeTarget, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{
    mav_cmd, mav_result, parse_mavlink, serialize_mavlink, MavAutopilot, MavMessage, MavModeFlag,
    MavState, MavType,
};

use crate::sim_types::{SimActuatorCmd, SimGnssFix, SimSensorPacket};
use crate::{bridge, XilConfig};

/// Raw sensor data from simulator (IMU, baro, mag)
#[derive(Debug, Clone, Default)]
pub struct HilSensorData {
    pub imu: RawImuReading,
    pub baro: RawBaroReading,
    pub mag: RawMagReading,
}

/// Raw GPS data from simulator
#[derive(Debug, Clone, Default)]
pub struct HilGpsData {
    pub gnss: RawGnssReading,
}

/// SITL I/O transport layer
///
/// Handles communication with the simulator. Does NOT implement HAL traits -
/// those are implemented by `BoardHal` in `aviate-hal-io` using fake drivers.
///
/// ## Data Flow
///
/// **Sensors (input):**
/// ```text
/// Simulator → SitlIO.poll() → take_sensor_data() → board feeds fake sensors
/// ```
///
/// **Actuators (output):**
/// ```text
/// BoardHal.write() → FakeActuator → board takes cmd → SitlIO.send_actuator()
/// ```
///
/// **Commands (input):**
/// ```text
/// GCS → SitlIO.recv_command() → board processes arm/disarm/setpoints
/// ```
pub struct SitlIO {
    /// Combined MAVLink socket (GCS commands + Telem + Legacy)
    /// Binds to Port 20000 + i*16 (Slot 0)
    socket: UdpSocket,
    config: XilConfig,
    start_time: std::time::Instant,
    armed: bool,
    seq: u8,

    // Buffered sensor data (from last poll)
    sensor_data: Option<HilSensorData>,
    gps_data: Option<HilGpsData>,

    // Discrete command slot (arm/disarm). Kept separate from the
    // setpoint slot: poll() drains every pending datagram into these
    // slots latest-wins, so a same-batch setpoint stream would
    // otherwise overwrite an Arm/Disarm and silently drop it.
    command: Option<SystemCommand>,
    // High-rate setpoint slot (latest-wins is the correct semantics
    // for a stream — only the newest setpoint matters).
    flight_cmd: Option<SystemCommand>,

    // Heartbeat timing
    last_heartbeat_us: u64,

    // GCS/client address (for responding to commands)
    gcs_addr: Option<std::net::SocketAddr>,

    // Statistics
    rx_count: u64,
    tx_count: u64,

    // Buffered actuator command for Rust API (direct FFI path)
    actuator_cmd: Option<SimActuatorCmd>,

    /// Current system status for heartbeat (MAV_STATE value)
    /// Updated by runtime via set_system_status()
    system_status: u8,
}

impl SitlIO {
    /// Create a new SITL I/O transport
    pub fn new(config: XilConfig) -> io::Result<Self> {
        // Bind to instance base port (Slot 0, e.g., 20000)
        // Used for MAVLink GCS communication (Command/Telem)
        info!("SitlIO: Binding MAVLink/GCS port {}", config.sensor_port());
        let socket = UdpSocket::bind(("0.0.0.0", config.sensor_port()))?;
        socket.set_nonblocking(true)?;

        Ok(Self {
            socket,
            config,
            start_time: std::time::Instant::now(),
            armed: false,
            seq: 0,
            sensor_data: None,
            gps_data: None,
            command: None,
            flight_cmd: None,
            last_heartbeat_us: 0,
            gcs_addr: None,
            rx_count: 0,
            tx_count: 0,
            actuator_cmd: None,
            system_status: MavState::Boot as u8, // Start in BOOT state
        })
    }

    /// Poll for incoming MAVLink messages
    ///
    /// Receives all available messages, updates internal buffers, and sends heartbeat.
    /// Call this at the start of each control loop iteration.
    pub fn poll(&mut self) {
        let mut buf = [0u8; 1024]; // Increased buffer size

        // Process all available messages from MAVLink socket
        loop {
            match self.socket.recv_from(&mut buf) {
                Ok((len, src)) => {
                    self.process_mavlink_data(&buf[..len], src);
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        // Send heartbeat at 1 Hz
        let now_us = self.now_us();
        if now_us - self.last_heartbeat_us >= 1_000_000 {
            self.send_heartbeat();
            self.last_heartbeat_us = now_us;
        }
    }

    /// Take buffered sensor data (IMU, baro, mag)
    ///
    /// Returns None if no new sensor data received since last take.
    pub fn take_sensor_data(&mut self) -> Option<HilSensorData> {
        self.sensor_data.take()
    }

    /// Take buffered GPS data
    ///
    /// Returns None if no new GPS data received since last take.
    pub fn take_gps_data(&mut self) -> Option<HilGpsData> {
        self.gps_data.take()
    }

    // =========================================================================
    // Rust API for direct simulator integration (bypasses MAVLink)
    // =========================================================================

    /// Feed sensor data from simulator via Rust API
    ///
    /// This is the direct path for simulator backends (like gazebo_bridge) to
    /// provide sensor data without going through MAVLink. The data is buffered
    /// and can be retrieved via `take_sensor_data()` and `take_gps_data()`.
    ///
    /// ## Coordinate Frame
    ///
    /// All data must be in NED (North-East-Down) frame. Backend-specific code
    /// is responsible for coordinate conversion (e.g., ENU→NED for Gazebo).
    pub fn feed_sensor_packet(&mut self, packet: &SimSensorPacket) {
        // Convert IMU/Baro/Mag to HilSensorData
        if packet.imu.is_some() || packet.baro.is_some() || packet.mag.is_some() {
            let imu = packet
                .imu
                .map_or_else(RawImuReading::default, |d| RawImuReading {
                    accel: d.accel,
                    gyro: d.gyro,
                    temperature: d.temperature,
                });

            let baro = packet
                .baro
                .map_or_else(RawBaroReading::default, |d| RawBaroReading {
                    pressure_pa: d.pressure_pa,
                    temperature_c: d.temperature_c,
                });

            let mag = packet
                .mag
                .map_or_else(RawMagReading::default, |d| RawMagReading {
                    field_ut: d.field_ut,
                });

            self.sensor_data = Some(HilSensorData { imu, baro, mag });
        }

        // Convert GNSS to HilGpsData
        if let Some(gnss) = packet.gnss {
            let fix = match gnss.fix {
                SimGnssFix::None => GnssFix::None,
                SimGnssFix::TwoD => GnssFix::TwoD,
                SimGnssFix::ThreeD => GnssFix::ThreeD,
                SimGnssFix::RtkFloat => GnssFix::RtkFloat,
                SimGnssFix::RtkFixed => GnssFix::RtkFixed,
            };

            self.gps_data = Some(HilGpsData {
                gnss: RawGnssReading {
                    lat_deg: gnss.lat_deg,
                    lon_deg: gnss.lon_deg,
                    alt_m: gnss.alt_m,
                    position_ned: gnss.position_ned,
                    vel_ned: gnss.vel_ned,
                    fix,
                    h_acc: gnss.h_acc,
                    v_acc: gnss.v_acc,
                    satellites: gnss.satellites,
                },
            });
        }
    }

    /// Set actuator command for Rust API consumers
    ///
    /// Called by the board layer after getting actuator commands from the mixer.
    /// Simulator backends (like gazebo_bridge) can retrieve this via `take_actuator_cmd()`.
    pub fn set_actuator_cmd(&mut self, cmd: SimActuatorCmd) {
        self.actuator_cmd = Some(cmd);
    }

    /// Take buffered actuator command (for Rust API)
    ///
    /// Returns None if no new actuator command since last take.
    /// Used by simulator backends (like gazebo_bridge) to get motor commands.
    pub fn take_actuator_cmd(&mut self) -> Option<SimActuatorCmd> {
        self.actuator_cmd.take()
    }

    /// Check if there's a pending actuator command
    pub fn has_actuator_cmd(&self) -> bool {
        self.actuator_cmd.is_some()
    }

    /// Process received MAVLink data
    fn process_mavlink_data(&mut self, data: &[u8], src: std::net::SocketAddr) {
        match parse_mavlink(data) {
            Ok((msg, _sig, _consumed)) => {
                self.rx_count += 1;
                self.handle_message(msg, src);
            }
            Err(e) => {
                // Log parse errors to help debug GCS communication issues
                warn!(
                    "MAVLink parse error from {}: {:?} (len={}, first_bytes={:02x?})",
                    src,
                    e,
                    data.len(),
                    &data[..data.len().min(10)]
                );
            }
        }
    }

    /// Handle a parsed MAVLink message
    ///
    /// Handles GCS commands (arm/disarm, setpoints). Sensor data is provided
    /// via the Rust API (feed_sensor_packet) from the simulator backend.
    fn handle_message(&mut self, msg: MavMessage, src: std::net::SocketAddr) {
        match msg {
            MavMessage::SetAttitudeTarget(tgt) => {
                self.gcs_addr = Some(src);
                self.handle_set_attitude_target(tgt);
            }
            MavMessage::SetPositionTargetLocalNed(tgt) => {
                self.gcs_addr = Some(src);
                self.handle_set_position_target(tgt);
            }
            MavMessage::CommandLong(cmd) => {
                self.gcs_addr = Some(src);
                self.handle_command_long(cmd);
            }
            MavMessage::Heartbeat(_) => {
                self.gcs_addr = Some(src);
            }
            _ => {}
        }
    }

    fn handle_set_attitude_target(&mut self, tgt: SetAttitudeTarget) {
        let cmd = bridge::mavlink_to_command(&tgt);
        self.flight_cmd = Some(SystemCommand::FlightControl(cmd));
    }

    fn handle_set_position_target(&mut self, tgt: SetPositionTargetLocalNed) {
        let cmd = bridge::mavlink_position_to_command(&tgt);
        self.flight_cmd = Some(SystemCommand::FlightControl(cmd));
    }

    fn handle_command_long(&mut self, cmd: CommandLong) {
        info!(
            "Received COMMAND_LONG: cmd={}, param1={}, target=({},{})",
            cmd.command, cmd.param1, cmd.target_system, cmd.target_component
        );

        let result = if cmd.command == mav_cmd::COMPONENT_ARM_DISARM {
            if cmd.param1 == 1.0 {
                info!("Processing ARM command");
                self.command = Some(SystemCommand::Arm);
                mav_result::ACCEPTED
            } else if cmd.param1 == 0.0 {
                info!("Processing DISARM command");
                self.command = Some(SystemCommand::Disarm);
                mav_result::ACCEPTED
            } else {
                warn!("Invalid ARM param1: {}", cmd.param1);
                mav_result::DENIED
            }
        } else {
            warn!("Unsupported command: {}", cmd.command);
            mav_result::UNSUPPORTED
        };

        // Send COMMAND_ACK to GCS
        self.send_command_ack(cmd.command, result);
    }

    /// Send COMMAND_ACK response to GCS
    fn send_command_ack(&mut self, command: u16, result: u8) {
        let ack = CommandAck {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: 255, // Broadcast
            target_component: 0,
        };

        if let Some(gcs_addr) = self.gcs_addr {
            info!(
                "Sending COMMAND_ACK to {}: cmd={}, result={}",
                gcs_addr, command, result
            );
            self.send_message_to(&MavMessage::CommandAck(ack), gcs_addr);
        } else {
            warn!("Cannot send COMMAND_ACK - no GCS address known");
        }
    }

    /// Send heartbeat message to GCS
    ///
    /// For Gazebo, sensor/actuator data flows via FFI bridge - no MAVLink to simulator.
    /// Heartbeat is only sent to GCS so it learns our port and can send commands back.
    fn send_heartbeat(&mut self) {
        let hb = Heartbeat {
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

        // Send heartbeat to GCS so it can discover our port
        self.send_message_to(&MavMessage::Heartbeat(hb), self.config.gcs_addr);

        // Also send to learned GCS address (if active/different)
        if let Some(gcs_addr) = self.gcs_addr {
            if gcs_addr != self.config.gcs_addr {
                self.send_message_to(&MavMessage::Heartbeat(hb), gcs_addr);
            }
        }
    }

    /// Send a MAVLink message to a specific address via GCS socket
    ///
    /// Uses the MAVLink socket (port 20000+...) so responses come from the same port
    /// that we're listening on for commands.
    fn send_message_to(&mut self, msg: &MavMessage, addr: std::net::SocketAddr) {
        let mut buf = [0u8; 300];
        // System ID = instance + 1, Component ID = 1 (Autopilot)
        if let Some(len) = serialize_mavlink(msg, self.seq, self.config.instance + 1, 1, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            let _ = self.socket.send_to(&buf[..len], addr);
            self.tx_count += 1;
        }
    }

    /// Set armed state (for MAVLink mode flags)
    pub fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
        if armed {
            info!("MAVLink armed");
        } else {
            info!("MAVLink disarmed");
        }
    }

    /// Set system status for heartbeat
    ///
    /// Maps FC init states to MAV_STATE:
    /// - `MavState::Boot` (1): PowerOn, ConfigLoading
    /// - `MavState::Calibrating` (2): SensorInit, EstimatorConverging
    /// - `MavState::Standby` (3): Ready (disarmed, can be armed)
    /// - `MavState::Active` (4): Armed
    ///
    /// Called by runtime when init state changes.
    pub fn set_system_status(&mut self, status: MavState) {
        self.system_status = status as u8;
    }

    /// Check if armed
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Get statistics
    pub fn stats(&self) -> (u64, u64) {
        (self.rx_count, self.tx_count)
    }

    /// Get the address actuator commands are sent to
    pub fn simulator_addr(&self) -> std::net::SocketAddr {
        self.config.simulator_addr()
    }

    /// Get the sensor port we're listening on
    pub fn sensor_port(&self) -> u16 {
        self.config.sensor_port()
    }

    /// Get the connected GCS address (if any)
    pub fn gcs_addr(&self) -> Option<std::net::SocketAddr> {
        self.gcs_addr
    }

    /// Borrow the underlying network/instance configuration. Used by
    /// the SitlRunner to spin up auxiliary listeners (fault command,
    /// etc.) on the same instance number.
    pub fn config(&self) -> &XilConfig {
        &self.config
    }
}

// Implement SystemHal - timing and system functions
impl SystemHal for SitlIO {
    fn now(&self) -> Timestamp {
        Timestamp {
            ticks: self.now_us(),
            source: TimeSource::Internal,
        }
    }

    fn now_us(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
    }

    fn delay_us(&self, us: u32) {
        std::thread::sleep(std::time::Duration::from_micros(us as u64));
    }

    fn kick_watchdog(&mut self) {}

    fn reboot(&mut self) -> ! {
        info!("Reboot requested");
        std::process::exit(0);
    }

    fn enter_bootloader(&mut self) -> ! {
        warn!("Bootloader not supported in SITL");
        std::process::exit(1);
    }
}

// Implement CommandHal - receives commands from GCS
impl CommandHal for SitlIO {
    fn recv_command(&mut self) -> Option<SystemCommand> {
        self.poll();
        // Discrete commands (arm/disarm) first: they must never be
        // starved or dropped by the setpoint stream.
        self.command.take().or_else(|| self.flight_cmd.take())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::XilNetConfig;

    /// Ephemeral-port config so tests never collide on a fixed port
    /// (base_port 0 + SensorIn slot 0 → OS-assigned bind).
    fn test_io() -> SitlIO {
        let net = XilNetConfig {
            base_port: 0,
            stride: 16,
        };
        SitlIO::new(XilConfig::for_instance_with_net(0, net)).expect("bind ephemeral UDP")
    }

    fn arm_msg() -> CommandLong {
        CommandLong {
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        }
    }

    fn attitude_msg(thrust: f32) -> SetAttitudeTarget {
        SetAttitudeTarget {
            time_boot_ms: 0,
            target_system: 1,
            target_component: 1,
            type_mask: 0,
            q: [1.0, 0.0, 0.0, 0.0],
            body_roll_rate: 0.0,
            body_pitch_rate: 0.0,
            body_yaw_rate: 0.0,
            thrust,
            thrust_body: [0.0, 0.0, 0.0],
        }
    }

    /// A setpoint parsed in the same poll batch after an Arm must not
    /// clobber it: both survive, discrete command first.
    #[test]
    fn setpoint_in_same_batch_does_not_clobber_arm() {
        let mut io = test_io();
        io.handle_command_long(arm_msg());
        io.handle_set_attitude_target(attitude_msg(0.4));

        assert!(matches!(io.recv_command(), Some(SystemCommand::Arm)));
        match io.recv_command() {
            Some(SystemCommand::FlightControl(cmd)) => {
                assert!((cmd.setpoint.collective_thrust.0 - 0.4).abs() < 1e-6);
            }
            other => panic!("expected buffered FlightControl, got {other:?}"),
        }
    }

    /// Arm parsed after a setpoint in the same batch: discrete command
    /// still drains first, the setpoint is preserved behind it.
    #[test]
    fn arm_after_setpoint_in_same_batch_preserves_both() {
        let mut io = test_io();
        io.handle_set_attitude_target(attitude_msg(0.7));
        io.handle_command_long(arm_msg());

        assert!(matches!(io.recv_command(), Some(SystemCommand::Arm)));
        assert!(matches!(
            io.recv_command(),
            Some(SystemCommand::FlightControl(_))
        ));
        assert!(io.recv_command().is_none());
    }

    /// Setpoints remain latest-wins: only the newest survives a batch.
    #[test]
    fn setpoint_slot_is_latest_wins() {
        let mut io = test_io();
        io.handle_set_attitude_target(attitude_msg(0.2));
        io.handle_set_attitude_target(attitude_msg(0.9));

        match io.recv_command() {
            Some(SystemCommand::FlightControl(cmd)) => {
                assert!((cmd.setpoint.collective_thrust.0 - 0.9).abs() < 1e-6);
            }
            other => panic!("expected latest setpoint, got {other:?}"),
        }
        assert!(io.recv_command().is_none());
    }
}
