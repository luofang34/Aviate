//! Simulation runtime for SITL/HITL
//!
//! Extracted from sitl-gazebo board to eliminate ~165 lines of stepping logic duplication.
//!
//! The `SitlRunner` struct encapsulates the control loop stepping logic.
//!
//! ## Shared Components
//!
//! This module provides factory functions and types that are shared across all SITL boards:
//! - `create_kernel()` - Creates an AviateKernel with MultirotorController + QuadXMixer
//! - `default_command()` - Creates a safe failsafe command with zero thrust
//! - `sitl_timestamp()` - Returns a SITL timestamp
//! - `BoardInfo` - Common board information structure
//! - `run_control_loop()` - Configurable control loop with specified period
//!
//! ## Backend Support
//!
//! - **Gazebo (SitlIO)**: Shared memory FFI + MAVLink commands
//! - **jMAVSim (HilBackend)**: Full MAVLink HIL (sensor + actuator)
//!
//! Both backends use `SitlRunner` for the core stepping logic but have different
//! transport layer integrations.
//!
//! This module is only available when env-sitl or env-hitl features are enabled.

#![cfg(any(feature = "env-sitl", feature = "env-hitl"))]

use crate::sensor_cache::SensorCache;

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::Command;
use aviate_core::hal::{ActuatorHal, CommandHal, SensorHal, SystemCommand, SystemHal};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ActuatorCmd, QuadXMixer};
use aviate_core::time::TimeDelta;
use aviate_core::types::{Meters, MetersPerSecond, Seconds};
use aviate_core::{AviateKernel, ChannelId, InitState};
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SimActuatorCmd, SitlIO};

/// Time source for SITL (re-exported for convenience)
pub struct SitlTime {
    start: std::time::Instant,
}

impl SitlTime {
    pub fn new() -> Self {
        Self {
            start: std::time::Instant::now(),
        }
    }
}

impl Default for SitlTime {
    fn default() -> Self {
        Self::new()
    }
}

impl aviate_hal_io::TimeSource for SitlTime {
    fn now_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}

/// SITL Board HAL type (Gazebo/SitlIO)
pub type SitlBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime, FakeActuator>;

/// SITL Kernel type
pub type SitlKernel = AviateKernel<MultirotorController, QuadXMixer>;

/// SITL runner encapsulating the stepping logic (Gazebo/SitlIO-specific for Phase 1)
///
/// This struct wraps SitlIO transport, BoardHal, and kernel, providing the control loop
/// stepping logic that was previously duplicated in GazeboSitlBoard.
///
/// **Future**: Make this generic over transport types to support both SitlIO and HilBackend.
pub struct SitlRunner {
    /// Simulator transport (SitlIO for Gazebo)
    pub transport: SitlIO,

    /// Board HAL with fake sensors (same interface as real hardware)
    pub board_hal: SitlBoardHal,

    /// Flight controller kernel
    pub kernel: SitlKernel,

    /// Last command received
    pub last_cmd: Command,

    /// Last IMU timestamp for dt calculation
    pub last_imu_time: Option<u64>,

    /// Cached sensor readings for kernel initialization
    pub sensor_cache: SensorCache,

    /// EKF initialization flag
    pub ekf_initialized: bool,
}

impl SitlRunner {
    /// Create a new SITL runner
    pub fn new(
        transport: SitlIO,
        board_hal: SitlBoardHal,
        kernel: SitlKernel,
        default_command: Command,
    ) -> Self {
        Self {
            transport,
            board_hal,
            kernel,
            last_cmd: default_command,
            last_imu_time: None,
            sensor_cache: SensorCache::new(),
            ekf_initialized: false,
        }
    }

    /// Step the flight controller (extracted from GazeboSitlBoard::step)
    ///
    /// This is the ~165 lines of stepping logic that was duplicated across SITL boards.
    ///
    /// ## Steps:
    /// 1. Poll transport for incoming messages
    /// 2. Feed fake sensors with HIL data (via BoardHal accessors)
    /// 3. Read sensors via BoardHal's SensorHal implementation
    /// 4. Calculate dt from IMU timestamps
    /// 5. Cache sensor readings for EKF init
    /// 6. Receive commands via transport
    /// 7. Initialize EKF once we have sensor data (one-time)
    /// 8. Run kernel initialization state machine
    /// 9. Step kernel with sensor data and commands
    /// 10. Write actuator outputs via BoardHal
    /// 11. Forward actuator commands to simulator
    /// 12. Kick watchdog
    pub fn step(&mut self) -> ActuatorCmd {
        // 1. Poll transport for incoming messages
        self.transport.poll();

        // 2. Feed fake sensors with HIL data (via BoardHal accessors)
        //    This is the key integration point - same pattern as real HW feeding real sensors
        if let Some(sensor_data) = self.transport.take_sensor_data() {
            // Feed IMU
            self.board_hal.imu_mut().feed(sensor_data.imu);
            // Feed Baro
            self.board_hal.baro_mut().feed(sensor_data.baro);
            // Feed Mag
            self.board_hal.mag_mut().feed(sensor_data.mag);
        }

        if let Some(gps_data) = self.transport.take_gps_data() {
            // Feed GNSS
            self.board_hal.gnss_mut().feed(gps_data.gnss);
        }

        // 3. Read sensors via BoardHal's SensorHal implementation
        //    This is the SAME code path that real hardware uses!
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

        // 4. Receive commands via transport
        if let Some(sys_cmd) = self.transport.recv_command() {
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
                    eprintln!("[INFO] Faults: {:?}", self.kernel.faults);
                    if let Err(e) = self.kernel.arm() {
                        let pre_arm = &self.kernel.checks.pre_arm;
                        eprintln!("[WARN] Arming failed: {:?}", e);
                        eprintln!("[WARN] Missing pre-arm: {:?}", pre_arm.missing());
                        eprintln!("[WARN] Faults: {:?}", self.kernel.faults);
                    } else {
                        eprintln!("[INFO] Armed successfully");
                        // Only arm HAL and transport if kernel arm succeeded
                        self.board_hal.arm();
                        self.transport.set_armed(true);
                    }
                }
                SystemCommand::Disarm => {
                    eprintln!("[INFO] Disarm command");
                    self.kernel.disarm();
                    // Disarm through BoardHal and notify transport
                    self.board_hal.disarm();
                    self.transport.set_armed(false);
                }
            }
        }

        // 5. Initialize EKF once we have sensor data
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

        // 6. Run init state machine
        let sensors = self.sensor_cache.to_sensor_set();
        if !self.kernel.is_ready() {
            let ts = self.transport.now();
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

        // 7. Step kernel
        let result = self.kernel.update(
            ChannelId(0),
            time_delta,
            &sensors,
            &self.last_cmd,
            &aviate_core::mixer::ActuatorState::default(),
            None,
        );
        let actuator_cmd = result.actuator;

        // 8. Write outputs via BoardHal (ActuatorHal implementation)
        //    This writes to FakeActuator, same path as real hardware
        self.board_hal.write(&actuator_cmd);

        // 9. Forward actuator command to simulator
        //    Take command from FakeActuator and set for backend to retrieve
        if let Some(raw_cmd) = self.board_hal.actuator_mut().take_cmd() {
            let sim_cmd = SimActuatorCmd {
                timestamp_us: self.transport.now_us(),
                outputs: raw_cmd.outputs,
                count: raw_cmd.count,
                armed: self.is_armed(),
            };
            self.transport.set_actuator_cmd(sim_cmd);
        }

        // 10. Watchdog
        self.transport.kick_watchdog();

        actuator_cmd
    }

    /// Check if system is armed
    pub fn is_armed(&self) -> bool {
        self.kernel.init_state == InitState::Armed
    }

    /// Get access to transport
    pub fn transport_mut(&mut self) -> &mut SitlIO {
        &mut self.transport
    }

    /// Get system uptime in microseconds
    pub fn now_us(&self) -> u64 {
        self.transport.now_us()
    }
}

// ============================================================================
// Shared Factory Functions (used by all SITL boards)
// ============================================================================

use aviate_core::control::{CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::mixer::ModeConfig;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::Normalized;

/// Create an AviateKernel configured for SITL multirotor simulation
///
/// This is shared by all SITL boards (Gazebo, jMAVSim, etc.) to ensure
/// consistent kernel initialization.
pub fn create_kernel() -> SitlKernel {
    let controller = MultirotorController::default();
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

/// Create a safe default/failsafe command with zero thrust
///
/// This is shared by all SITL boards to ensure consistent failsafe behavior.
pub fn default_command() -> Command {
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

/// SITL timestamp function for mixer
pub fn sitl_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

/// Board information structure (shared across SITL boards)
#[derive(Clone, Debug)]
pub struct SitlBoardInfo {
    pub name: &'static str,
    pub description: &'static str,
}

/// Run a control loop with configurable period
///
/// This is the shared control loop implementation used by all SITL boards.
/// The only difference between boards is the `loop_period_us` parameter:
/// - Gazebo: 1000us (1kHz)
/// - jMAVSim: 2500us (400Hz)
///
/// # Arguments
/// * `runner` - The SitlRunner to step
/// * `loop_period_us` - Control loop period in microseconds
pub fn run_control_loop(runner: &mut SitlRunner, loop_period_us: u64) -> ! {
    let mut last_tick = runner.now_us();

    loop {
        let now = runner.now_us();
        let elapsed = now.saturating_sub(last_tick);

        if elapsed >= loop_period_us {
            last_tick = now;
            runner.step();
        } else {
            let remaining_us = loop_period_us - elapsed;
            if remaining_us > 100 {
                std::thread::sleep(std::time::Duration::from_micros(remaining_us - 100));
            }
        }
    }
}

/// Default loop periods for different simulators
pub mod loop_periods {
    /// Gazebo SITL loop period (1kHz)
    pub const GAZEBO_US: u64 = 1000;
    /// jMAVSim SITL loop period (400Hz to match jMAVSim default rate)
    pub const JMAVSIM_US: u64 = 2500;
}

// Re-export for convenience (Phase 1 stub - Phase 2+ will use this for AppRuntime)
use aviate_config::AppConfig;

/// Application runtime for SITL/HITL (Phase 1 stub)
///
/// Phase 2+: This will wrap SitlRunner and provide AppRuntime::run(config)
pub struct AppRuntime<Board, Airframe> {
    _board: core::marker::PhantomData<Board>,
    _airframe: core::marker::PhantomData<Airframe>,
}

impl<Board, Airframe> AppRuntime<Board, Airframe> {
    /// Run the simulation application (never returns)
    ///
    /// Phase 1: Stub (boards use SitlRunner directly)
    /// Phase 2+: Full implementation using AppConfig
    pub fn run(_config: &AppConfig) -> ! {
        unimplemented!("Phase 1: Use SitlRunner directly. Phase 2+: Full AppRuntime::run()")
    }
}
