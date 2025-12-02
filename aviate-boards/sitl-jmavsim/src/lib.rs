//! SITL jMAVSim Board Configuration
//!
//! This board represents a simulated quadcopter using jMAVSim via MAVLink HIL protocol.
//! It uses the existing `aviate-backend-mavlink-hil` for communication.
//!
//! ## Architecture
//!
//! ```text
//! SENSORS (Input):
//! jMAVSim → UDP/MAVLink → HilBackend → FakeImu/Baro/... → BoardHal → SensorHal
//!                                           ↓
//!                                    Same kernel code
//!                                           ↓
//! ACTUATORS (Output):
//! Kernel → BoardHal → FakeActuator → HilBackend → UDP/MAVLink → jMAVSim
//! ```
//!
//! ## MAVLink HIL Protocol
//!
//! | Message | Direction | Content |
//! |---------|-----------|---------|
//! | HIL_SENSOR (107) | Sim → FC | IMU, Baro, Mag |
//! | HIL_GPS (113) | Sim → FC | GNSS data |
//! | HIL_ACTUATOR_CONTROLS (93) | FC → Sim | Motor commands |
//!
//! ## Usage
//!
//! ```ignore
//! let mut board = JmavSimBoard::new(config)?;
//! board.arm()?;  // Arm manually after init
//! loop {
//!     board.step();
//! }
//! ```

#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

use std::io;

use aviate_backend_mavlink_hil::{HilBackend, HilBackendConfig};
use aviate_core::control::mc::McController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::hal::{ActuatorHal, SensorHal};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ActuatorCmd, ActuatorState, ModeConfig, QuadXMixer};
use aviate_core::sensor::{BaroData, GnssData, ImuData, MagData, SensorReading, SensorSet};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, Normalized, Seconds};
use aviate_core::{ArmError, AviateKernel, ChannelId, InitState};

use aviate_hal_io::{
    BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag, GnssFix, RawBaroReading,
    RawGnssReading, RawImuReading, RawMagReading,
};
use aviate_hal_xil::{SimActuatorCmd, SimGnssFix};

/// Time source for SITL using std::time
pub struct SitlTime {
    start: std::time::Instant,
}

impl SitlTime {
    fn new() -> Self {
        Self {
            start: std::time::Instant::now(),
        }
    }
}

impl aviate_hal_io::TimeSource for SitlTime {
    fn now_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}

/// Type alias for the SITL board's HAL
pub type SitlBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime, FakeActuator>;

/// jMAVSim board configuration
///
/// ## Port Configuration
///
/// jMAVSim in UDP mode binds to `simulator_port` (default 14560) and waits for
/// incoming messages. The flight controller binds to `local_port` (default 0 = ephemeral)
/// and sends messages to jMAVSim to initiate the connection. Once connected, jMAVSim
/// sends sensor data back to the FC's port.
#[derive(Clone, Debug)]
pub struct JmavSimConfig {
    /// Local port to bind for receiving HIL data (default: 0 = ephemeral)
    /// Use 0 to let the OS assign an available port, avoiding conflicts with jMAVSim.
    pub local_port: u16,
    /// Remote simulator port where jMAVSim listens (default: 14560)
    pub simulator_port: u16,
    /// Simulator host (default: 127.0.0.1)
    pub simulator_host: [u8; 4],
    /// MAVLink system ID (default: 1)
    pub sys_id: u8,
    /// MAVLink component ID (default: 1)
    pub comp_id: u8,
}

impl Default for JmavSimConfig {
    fn default() -> Self {
        Self {
            local_port: 0,         // Ephemeral port - let OS assign
            simulator_port: 14560, // jMAVSim default UDP port
            simulator_host: [127, 0, 0, 1],
            sys_id: 1,
            comp_id: 1,
        }
    }
}

/// jMAVSim SITL board
///
/// Uses MAVLink HIL protocol to communicate with jMAVSim simulator.
/// Follows the same BoardHal pattern as real hardware boards.
pub struct JmavSimBoard {
    /// MAVLink HIL backend for communication with jMAVSim
    hil_backend: HilBackend,

    /// Board HAL with fake sensors (same interface as real hardware)
    board_hal: SitlBoardHal,

    /// Flight controller kernel
    kernel: AviateKernel<McController, QuadXMixer>,

    /// Last command received
    last_cmd: Command,

    /// Last IMU timestamp for dt calculation
    last_imu_time: Option<u64>,

    /// Cached sensor readings for kernel
    sensor_cache: SensorCache,

    /// EKF initialization flag
    ekf_initialized: bool,

    /// Armed state
    armed: bool,
}

/// Cached sensor readings for kernel init
struct SensorCache {
    imu: Option<SensorReading<ImuData>>,
    gnss: Option<SensorReading<GnssData>>,
    baro: Option<SensorReading<BaroData>>,
    mag: Option<SensorReading<MagData>>,
}

impl SensorCache {
    fn new() -> Self {
        Self {
            imu: None,
            gnss: None,
            baro: None,
            mag: None,
        }
    }

    fn to_sensor_set(&self) -> SensorSet {
        SensorSet {
            imus: [
                self.imu.unwrap_or_default(),
                SensorReading::default(),
                SensorReading::default(),
            ],
            gnss: [self.gnss.unwrap_or_default(), SensorReading::default()],
            mags: [self.mag.unwrap_or_default(), SensorReading::default()],
            baros: [self.baro.unwrap_or_default(), SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
        }
    }
}

impl JmavSimBoard {
    /// Create a new jMAVSim board with default configuration
    pub fn new() -> io::Result<Self> {
        Self::with_config(JmavSimConfig::default())
    }

    /// Create a new jMAVSim board with custom configuration
    pub fn with_config(config: JmavSimConfig) -> io::Result<Self> {
        let hil_config = HilBackendConfig {
            local_port: config.local_port,
            simulator_addr: std::net::SocketAddr::from((
                config.simulator_host,
                config.simulator_port,
            )),
            sys_id: config.sys_id,
            comp_id: config.comp_id,
        };

        let hil_backend = HilBackend::new(hil_config)?;

        // Create fake sensors and actuator - same interface as real hardware drivers
        let fake_imu = FakeImu::new();
        let fake_baro = FakeBaro::new();
        let fake_mag = FakeMag::new();
        let fake_gnss = FakeGnss::new();
        let time = SitlTime::new();
        let fake_actuator = FakeActuator::new();

        // Create BoardHal with fake sensors and actuator
        let board_hal = BoardHal::new(
            fake_imu,
            fake_baro,
            fake_mag,
            fake_gnss,
            time,
            fake_actuator,
        );

        let kernel = Self::create_kernel();
        let last_cmd = Self::default_command();

        Ok(Self {
            hil_backend,
            board_hal,
            kernel,
            last_cmd,
            last_imu_time: None,
            sensor_cache: SensorCache::new(),
            ekf_initialized: false,
            armed: false,
        })
    }

    fn create_kernel() -> AviateKernel<McController, QuadXMixer> {
        let controller = McController::default();
        let mixer = QuadXMixer {
            timestamp_source: sitl_timestamp,
        };
        let mode_config = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        };

        let mut kernel = AviateKernel::new(controller, mixer, mode_config);

        // Initialize throttle check as satisfied (default command has low throttle)
        kernel.checks.pre_arm.update_throttle(true);

        kernel
    }

    fn default_command() -> Command {
        Command {
            mode: ControlMode::Attitude,
            setpoint: Setpoint {
                collective_thrust: Normalized(0.0),
                ..Default::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 0,
            source: CommandSource::Failsafe,
        }
    }

    /// Run one iteration of the control loop
    ///
    /// This:
    /// 1. Polls HilBackend for MAVLink HIL messages
    /// 2. Feeds fake sensors with HIL data
    /// 3. Reads sensors via BoardHal's SensorHal implementation
    /// 4. Runs the kernel
    /// 5. Sends actuator commands via MAVLink HIL
    ///
    /// Returns the actuator command that was sent.
    pub fn step(&mut self) -> ActuatorCmd {
        // 1. Poll HilBackend for incoming MAVLink HIL messages
        if let Some(packet) = self.hil_backend.poll() {
            // Feed IMU data (convert SimImuData to RawImuReading)
            if let Some(imu_data) = packet.imu {
                self.board_hal.imu_mut().feed(RawImuReading {
                    accel: imu_data.accel,
                    gyro: imu_data.gyro,
                    temperature: imu_data.temperature,
                });
            }
            // Feed Baro data (convert SimBaroData to RawBaroReading)
            if let Some(baro_data) = packet.baro {
                self.board_hal.baro_mut().feed(RawBaroReading {
                    pressure_pa: baro_data.pressure_pa,
                    temperature_c: baro_data.temperature_c,
                });
            }
            // Feed Mag data (convert SimMagData to RawMagReading)
            if let Some(mag_data) = packet.mag {
                self.board_hal.mag_mut().feed(RawMagReading {
                    field_ut: mag_data.field_ut,
                });
            }
            // Feed GNSS data (convert SimGnssData to RawGnssReading)
            if let Some(gnss_data) = packet.gnss {
                self.board_hal.gnss_mut().feed(RawGnssReading {
                    lat_deg: gnss_data.lat_deg,
                    lon_deg: gnss_data.lon_deg,
                    alt_m: gnss_data.alt_m,
                    vel_ned: gnss_data.vel_ned,
                    fix: convert_gnss_fix(gnss_data.fix),
                    h_acc: gnss_data.h_acc,
                    v_acc: gnss_data.v_acc,
                    satellites: gnss_data.satellites,
                });
            }
        }

        // 2. Read sensors via BoardHal's SensorHal implementation
        let mut current_dt = 0.001;
        let mut current_delta_us = 1000u64;

        if let Some(imu) = self.board_hal.read_imu() {
            let current_time = imu.timestamp.ticks;
            let delta_us_val = if let Some(last) = self.last_imu_time {
                current_time.saturating_sub(last)
            } else {
                1000
            };
            current_dt = (delta_us_val as f32) * 1e-6;
            current_delta_us = delta_us_val;
            self.last_imu_time = Some(current_time);
            current_dt = current_dt.clamp(0.0001, 0.1);
            self.sensor_cache.imu = Some(imu);
        }

        if let Some(gnss) = self.board_hal.read_gnss() {
            self.sensor_cache.gnss = Some(gnss);
        }

        if let Some(baro) = self.board_hal.read_baro() {
            self.sensor_cache.baro = Some(baro);
        }

        if let Some(mag) = self.board_hal.read_mag() {
            self.sensor_cache.mag = Some(mag);
        }

        let time_delta = TimeDelta {
            dt_sec: Seconds(current_dt),
            tick_delta: current_delta_us,
        };

        // 3. Initialize EKF once we have sensor data
        if !self.ekf_initialized && self.sensor_cache.imu.is_some() {
            self.kernel.ekf.init(
                Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
                Vector3::new(
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                ),
                Quaternion::IDENTITY,
            );
            self.ekf_initialized = true;
        }

        // 4. Run init state machine
        let sensors = self.sensor_cache.to_sensor_set();
        if !self.kernel.is_ready() {
            let ts = Timestamp {
                ticks: self.hil_backend.now_us(),
                source: TimeSource::Internal,
            };
            let prev_state = self.kernel.init_state;
            self.kernel.init_step(&sensors, ts);

            // Log state transitions
            if self.kernel.init_state != prev_state {
                eprintln!(
                    "[FC] Init state: {:?} -> {:?}",
                    prev_state, self.kernel.init_state
                );
            }
        }

        // 5. Step kernel
        let result = self.kernel.update(
            ChannelId(0),
            time_delta,
            &sensors,
            &self.last_cmd,
            &ActuatorState::default(),
            None,
        );
        let actuator_cmd = result.actuator;

        // 6. Write outputs via BoardHal (ActuatorHal implementation)
        self.board_hal.write(&actuator_cmd);

        // 7. Forward actuator command to jMAVSim via MAVLink HIL
        if let Some(raw_cmd) = self.board_hal.actuator_mut().take_cmd() {
            let sim_cmd = SimActuatorCmd {
                timestamp_us: self.hil_backend.now_us(),
                outputs: raw_cmd.outputs,
                count: raw_cmd.count,
                armed: self.armed,
            };
            // Ignore send errors (jMAVSim might not be connected yet)
            let _ = self.hil_backend.send_actuators(&sim_cmd);
        }

        actuator_cmd
    }

    /// Run the main control loop indefinitely
    pub fn run(&mut self) -> ! {
        let loop_period_us = 2500; // 400Hz to match jMAVSim default rate
        let mut last_tick = self.hil_backend.now_us();

        loop {
            let now = self.hil_backend.now_us();
            let elapsed = now.saturating_sub(last_tick);

            if elapsed >= loop_period_us {
                last_tick = now;
                self.step();
            } else {
                let remaining_us = loop_period_us - elapsed;
                if remaining_us > 100 {
                    std::thread::sleep(std::time::Duration::from_micros(remaining_us - 100));
                }
            }
        }
    }

    /// Arm the flight controller
    pub fn arm(&mut self) -> Result<(), ArmError> {
        eprintln!("[INFO] Arm command (state={:?})", self.kernel.init_state);
        eprintln!("[INFO] Faults: {:?}", self.kernel.faults);

        self.kernel.arm()?;

        eprintln!("[INFO] Armed successfully");
        self.board_hal.arm();
        self.armed = true;
        Ok(())
    }

    /// Disarm the flight controller
    pub fn disarm(&mut self) {
        eprintln!("[INFO] Disarm command");
        self.kernel.disarm();
        self.board_hal.disarm();
        self.armed = false;
    }

    /// Set the flight command (attitude/thrust setpoint)
    pub fn set_command(&mut self, cmd: Command) {
        self.kernel
            .checks
            .pre_arm
            .update_throttle(cmd.setpoint.collective_thrust.0 < 0.1);
        self.last_cmd = cmd;
    }

    /// Check if the kernel is ready for flight
    pub fn is_ready(&self) -> bool {
        self.kernel.is_ready()
    }

    /// Check if the kernel is armed
    pub fn is_armed(&self) -> bool {
        self.kernel.init_state == InitState::Armed
    }

    /// Get a reference to the kernel
    pub fn kernel(&self) -> &AviateKernel<McController, QuadXMixer> {
        &self.kernel
    }

    /// Get a mutable reference to the kernel
    pub fn kernel_mut(&mut self) -> &mut AviateKernel<McController, QuadXMixer> {
        &mut self.kernel
    }

    /// Get current timestamp in microseconds
    pub fn now_us(&self) -> u64 {
        self.hil_backend.now_us()
    }

    /// Get statistics (rx_count, tx_count, crc_errors)
    pub fn stats(&self) -> (u64, u64, u64) {
        self.hil_backend.stats()
    }

    /// Get the local port being used
    pub fn local_port(&self) -> u16 {
        self.hil_backend.local_port()
    }

    /// Send a HEARTBEAT message to initialize jMAVSim
    ///
    /// jMAVSim requires a HEARTBEAT message to initialize HIL communication.
    /// Call this periodically (typically 1Hz) to maintain the connection.
    pub fn send_heartbeat(&mut self) {
        let _ = self.hil_backend.send_heartbeat(self.armed);
    }

    /// Send initial handshake to trigger jMAVSim connection
    ///
    /// jMAVSim in UDP mode waits for the first HEARTBEAT message before starting
    /// to send sensor data. Call this after creating the board to initiate the connection.
    pub fn send_handshake(&mut self) {
        // Send HEARTBEAT - required by jMAVSim to initialize
        self.send_heartbeat();
    }

    /// Get the airframe ID
    pub fn airframe_id() -> &'static str {
        aviate_airframe_quadcopter::airframe_id()
    }

    /// Get board ID
    pub fn board_id() -> &'static str {
        "sitl-jmavsim"
    }
}

fn sitl_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

/// Convert SimGnssFix to GnssFix
fn convert_gnss_fix(fix: SimGnssFix) -> GnssFix {
    match fix {
        SimGnssFix::None => GnssFix::None,
        SimGnssFix::TwoD => GnssFix::TwoD,
        SimGnssFix::ThreeD => GnssFix::ThreeD,
        SimGnssFix::RtkFloat => GnssFix::RtkFloat,
        SimGnssFix::RtkFixed => GnssFix::RtkFixed,
    }
}

/// Board info for jMAVSim SITL
pub const BOARD_INFO: BoardInfo = BoardInfo {
    name: "sitl-jmavsim",
    airframe: "quadcopter",
    description: "jMAVSim quadcopter via MAVLink HIL protocol",
    motor_count: 4,
    motor_layout: MotorLayout::QuadX,
};

/// Board information structure
#[derive(Clone, Debug)]
pub struct BoardInfo {
    pub name: &'static str,
    pub airframe: &'static str,
    pub description: &'static str,
    pub motor_count: u8,
    pub motor_layout: MotorLayout,
}

/// Motor layout configuration
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorLayout {
    QuadX,    // X configuration (45 rotated)
    QuadPlus, // + configuration
    Hex,      // Hexacopter
    Octo,     // Octocopter
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_board_info() {
        assert_eq!(BOARD_INFO.name, "sitl-jmavsim");
        assert_eq!(BOARD_INFO.airframe, "quadcopter");
        assert_eq!(BOARD_INFO.motor_count, 4);
    }

    #[test]
    fn test_airframe_id() {
        assert_eq!(JmavSimBoard::airframe_id(), "quadcopter");
    }

    #[test]
    fn test_board_id() {
        assert_eq!(JmavSimBoard::board_id(), "sitl-jmavsim");
    }

    #[test]
    fn test_default_config() {
        let config = JmavSimConfig::default();
        assert_eq!(config.local_port, 0); // Ephemeral port
        assert_eq!(config.simulator_port, 14560); // jMAVSim default
        assert_eq!(config.simulator_host, [127, 0, 0, 1]);
    }
}
