//! SITL X500 Board Configuration
//!
//! This board represents a simulated x500 quadcopter in Gazebo SITL.
//! It combines the XIL HAL with quadcopter airframe configuration.
//!
//! ## Sensor Configuration (simulated)
//!
//! | Sensor | Model | Interface |
//! |--------|-------|-----------|
//! | IMU    | Gazebo physics | HIL_SENSOR |
//! | GNSS   | Gazebo plugin  | HIL_GPS |
//! | Baro   | Gazebo plugin  | HIL_SENSOR |
//! | Mag    | Gazebo plugin  | HIL_SENSOR |
//!
//! ## Motor Configuration (x500 layout)
//!
//! ```text
//!     Front
//!   1 (CW)   2 (CCW)
//!       \   /
//!        [X]
//!       /   \
//!   4 (CCW)  3 (CW)
//!     Rear
//! ```

#![forbid(unsafe_code)]

use std::io;

use aviate_core::control::mc::McController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::hal::{ActuatorHal, CommandHal, SensorHal, SystemCommand, SystemHal};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ActuatorCmd, ModeConfig, QuadXMixer};
use aviate_core::sensor::{BaroData, GnssData, ImuData, MagData, SensorReading, SensorSet};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, Normalized, Seconds};
use aviate_core::AviateKernel;

use aviate_hal_xil::{SitlConfig, UdpMavlinkHal};

/// X500 SITL board configuration
pub struct X500SitlBoard {
    hal: UdpMavlinkHal,
    kernel: AviateKernel<McController, QuadXMixer>,
    last_cmd: Command,
    last_imu_time: Option<u64>,
    sensor_cache: SensorCache,
    ekf_initialized: bool,
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

impl X500SitlBoard {
    /// Create a new X500 SITL board with default configuration
    pub fn new() -> io::Result<Self> {
        Self::with_config(SitlConfig::default())
    }

    /// Create a new X500 SITL board with custom configuration
    pub fn with_config(config: SitlConfig) -> io::Result<Self> {
        let hal = UdpMavlinkHal::new(config)?;
        let kernel = Self::create_kernel();
        let last_cmd = Self::default_command();

        Ok(Self {
            hal,
            kernel,
            last_cmd,
            last_imu_time: None,
            sensor_cache: SensorCache::new(),
            ekf_initialized: false,
        })
    }

    /// Create a new X500 SITL board with retry on port binding
    pub fn new_with_retry(max_retries: u32, retry_delay_ms: u64) -> io::Result<Self> {
        let config = SitlConfig::default();
        for i in 0..max_retries {
            match Self::with_config(config.clone()) {
                Ok(board) => return Ok(board),
                Err(e) => {
                    if i < max_retries - 1 {
                        eprintln!(
                            "[WARN] Failed to bind port: {}. Retrying in {}ms...",
                            e, retry_delay_ms
                        );
                        std::thread::sleep(std::time::Duration::from_millis(retry_delay_ms));
                    } else {
                        return Err(e);
                    }
                }
            }
        }
        unreachable!()
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
    /// This reads sensors, runs the kernel, and writes actuator outputs.
    /// Returns the actuator command that was sent.
    pub fn step(&mut self) -> ActuatorCmd {
        // 1. Read sensors and calculate dt
        let mut current_dt = 0.001; // Default 1ms
        let mut current_delta_us = 1000;

        if let Some(imu) = self.hal.read_imu() {
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

        if let Some(gnss) = self.hal.read_gnss() {
            self.sensor_cache.gnss = Some(gnss);
        }

        if let Some(baro) = self.hal.read_baro() {
            self.sensor_cache.baro = Some(baro);
        }

        if let Some(mag) = self.hal.read_mag() {
            self.sensor_cache.mag = Some(mag);
        }

        let time_delta = TimeDelta {
            dt_sec: Seconds(current_dt),
            tick_delta: current_delta_us,
        };

        // 2. Receive commands
        if let Some(sys_cmd) = self.hal.recv_command() {
            match sys_cmd {
                SystemCommand::FlightControl(cmd) => {
                    self.kernel
                        .checks
                        .pre_arm
                        .update_throttle(cmd.setpoint.collective_thrust.0 < 0.1);
                    self.last_cmd = cmd;
                }
                SystemCommand::Arm => {
                    eprintln!("[INFO] Arm command (state={:?})", self.kernel.init_state);
                    if let Err(e) = self.kernel.arm() {
                        let pre_arm = &self.kernel.checks.pre_arm;
                        eprintln!("[WARN] Arming failed: {:?}", e);
                        eprintln!("[WARN] Missing pre-arm: {:?}", pre_arm.missing());
                    } else {
                        eprintln!("[INFO] Armed successfully");
                    }
                    self.hal.arm();
                }
                SystemCommand::Disarm => {
                    eprintln!("[INFO] Disarm command");
                    self.kernel.disarm();
                    self.hal.disarm();
                }
            }
        }

        // 3. Initialize EKF once we have sensor data
        if !self.ekf_initialized && self.sensor_cache.imu.is_some() {
            eprintln!("[INFO] Initializing EKF with sensor data");
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
            let ts = self.hal.now();
            self.kernel.init_step(&sensors, ts);
        }

        // 4. Step kernel
        let actuator_cmd = self.kernel.step(time_delta, &self.last_cmd, &sensors, 0);

        // 5. Write outputs
        self.hal.write(&actuator_cmd);

        // 6. Watchdog
        self.hal.kick_watchdog();

        actuator_cmd
    }

    /// Run the main control loop indefinitely
    pub fn run(&mut self) -> ! {
        let loop_period_us = 1000; // 1kHz
        let mut last_tick = self.hal.now_us();

        loop {
            let now = self.hal.now_us();
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

    /// Check if the kernel is ready for flight
    pub fn is_ready(&self) -> bool {
        self.kernel.is_ready()
    }

    /// Check if the kernel is armed
    pub fn is_armed(&self) -> bool {
        self.kernel.init_state == aviate_core::InitState::Armed
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
        self.hal.now_us()
    }

    /// Get the airframe ID
    pub fn airframe_id() -> &'static str {
        aviate_airframe_quadcopter::airframe_id()
    }

    /// Get board ID
    pub fn board_id() -> &'static str {
        "sitl-x500"
    }
}

fn sitl_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

/// Board info for the X500 SITL
pub const BOARD_INFO: BoardInfo = BoardInfo {
    name: "sitl-x500",
    airframe: "quadcopter",
    description: "PX4 X500 quadcopter in Gazebo SITL",
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
    QuadX,    // X configuration (45° rotated)
    QuadPlus, // + configuration
    Hex,      // Hexacopter
    Octo,     // Octocopter
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_board_info() {
        assert_eq!(BOARD_INFO.name, "sitl-x500");
        assert_eq!(BOARD_INFO.airframe, "quadcopter");
        assert_eq!(BOARD_INFO.motor_count, 4);
    }

    #[test]
    fn test_airframe_id() {
        assert_eq!(X500SitlBoard::airframe_id(), "quadcopter");
    }

    #[test]
    fn test_board_id() {
        assert_eq!(X500SitlBoard::board_id(), "sitl-x500");
    }
}
