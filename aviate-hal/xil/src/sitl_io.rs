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

mod command_link;

use std::io;
use std::net::UdpSocket;

use log::{info, warn};

use aviate_core::hal::SystemHal;
use aviate_core::time::{TimeSource, Timestamp};

use aviate_hal_io::{
    GnssFix, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading, SystemCommand,
};

use aviate_link::mavlink::MavState;

use crate::command_provenance::{MavlinkCommandProvenance, SourceEpochTracker};
use crate::sim_types::{SimActuatorCmd, SimGnssFix, SimSensorPacket};
use crate::XilConfig;

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

/// One received command with exact raw provenance for a MAVLink setpoint.
#[derive(Clone, Debug)]
pub struct ReceivedCommand {
    /// Typed command supplied to the runtime.
    pub command: SystemCommand,
    /// Exact raw identity. Discrete commands do not carry this value.
    pub provenance: Option<MavlinkCommandProvenance>,
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
    flight_cmd: Option<ReceivedCommand>,
    command_source_epochs: SourceEpochTracker,

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
        let command_source_epochs = SourceEpochTracker::new()?;

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
            command_source_epochs,
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
                    differential_pressure_pa: d.differential_pressure_pa,
                    pressure_altitude_m: d.pressure_altitude_m,
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

    /// Clear data that must not cross a reset generation.
    pub fn clear_generation_state(&mut self) {
        let mut buffer = [0_u8; 1_024];
        loop {
            match self.socket.recv_from(&mut buffer) {
                Ok(_) => {}
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        self.sensor_data = None;
        self.gps_data = None;
        self.command = None;
        self.flight_cmd = None;
        self.actuator_cmd = None;
        self.armed = false;
        self.system_status = MavState::Boot as u8;
    }

    /// Check if there's a pending actuator command
    pub fn has_actuator_cmd(&self) -> bool {
        self.actuator_cmd.is_some()
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
