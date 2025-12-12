//! Gazebo SITL Board Configuration
//!
//! Generic board for Gazebo-based SITL simulation. The specific airframe
//! (e.g., X500, GenericQuadX) is specified by the application.
//!
//! ## Architecture
//!
//! This board uses the same `BoardHal` that real hardware boards use,
//! ensuring the dataflow is identical between SITL and real hardware:
//!
//! ```text
//! SENSORS (Input):
//! SITL:  Gazebo → gazebo_bridge (FFI) → SitlIO → FakeImu/Baro/... → BoardHal → SensorHal
//! Real:  SPI/I2C → BMI088/BMP390/... → BoardHal → SensorHal
//!                                           ↓
//!                                    Same kernel code
//!                                           ↓
//! ACTUATORS (Output):
//! SITL:  Kernel → BoardHal → FakeActuator → SitlIO → gazebo_bridge (FFI) → Gazebo
//! Real:  Kernel → BoardHal → PwmMotors → PWM signals → ESCs
//! ```
//!
//! ## Sensor Configuration (simulated)
//!
//! | Sensor | Model | Interface |
//! |--------|-------|-----------|
//! | IMU    | Gazebo physics | FFI |
//! | GNSS   | Gazebo plugin  | FFI |
//! | Baro   | Gazebo plugin  | FFI |
//! | Mag    | Gazebo plugin  | FFI |

#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

use std::io;

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::mixer::{ActuatorCmd, ModeConfig, QuadXMixer};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::Normalized;
use aviate_core::AviateKernel;

use aviate_runtime::SitlRunner;
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SitlConfig, SitlIO};

/// Gazebo SITL board configuration
///
/// Uses the same BoardHal abstraction as real hardware boards, ensuring
/// that SITL tests exercise the same code paths as real hardware.
///
/// **Phase 1**: Delegates to SitlRunner for control loop execution.
/// This eliminates ~165 lines of duplication from the board implementation.
pub struct GazeboSitlBoard {
    /// SITL runner (encapsulates transport, board HAL, and kernel)
    runner: SitlRunner,
}

impl GazeboSitlBoard {
    /// Create a new X500 SITL board with default configuration
    pub fn new() -> io::Result<Self> {
        Self::with_config(SitlConfig::default())
    }

    /// Create a new X500 SITL board with custom configuration
    pub fn with_config(config: SitlConfig) -> io::Result<Self> {
        let transport = SitlIO::new(config)?;

        // Create fake sensors and actuator - same interface as real hardware drivers
        let fake_imu = FakeImu::new();
        let fake_baro = FakeBaro::new();
        let fake_mag = FakeMag::new();
        let fake_gnss = FakeGnss::new();
        let time = aviate_runtime::sim::SitlTime::new();
        let fake_actuator = FakeActuator::new();

        // Create BoardHal with fake sensors and actuator
        // This is the SAME BoardHal that real hardware would use!
        let board_hal = BoardHal::new(
            fake_imu,
            fake_baro,
            fake_mag,
            fake_gnss,
            time,
            fake_actuator,
        );

        let kernel = Self::create_kernel();
        let default_cmd = Self::default_command();

        // Create SitlRunner with all components
        let runner = SitlRunner::new(transport, board_hal, kernel, default_cmd);

        Ok(Self { runner })
    }

    /// Create a new X500 SITL board with retry on port binding
    pub fn new_with_retry(max_retries: u32, retry_delay_ms: u64) -> io::Result<Self> {
        Self::with_config_retry(SitlConfig::default(), max_retries, retry_delay_ms)
    }

    /// Create a new X500 SITL board with custom config and retry on port binding
    pub fn with_config_retry(
        config: SitlConfig,
        max_retries: u32,
        retry_delay_ms: u64,
    ) -> io::Result<Self> {
        for i in 0..max_retries {
            match Self::with_config(config.clone()) {
                Ok(board) => return Ok(board),
                Err(e) => {
                    if i < max_retries - 1 {
                        eprintln!(
                            "[WARN] Failed to bind port {}: {}. Retrying in {}ms...",
                            config.sensor_port(),
                            e,
                            retry_delay_ms
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

    fn create_kernel() -> AviateKernel<MultirotorController, QuadXMixer> {
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
    /// Delegates to SitlRunner for execution. See aviate-runtime/src/sim.rs
    /// for the 12-step control loop implementation.
    ///
    /// Returns the actuator command that was sent.
    pub fn step(&mut self) -> ActuatorCmd {
        self.runner.step()
    }

    /// Run the main control loop indefinitely
    pub fn run(&mut self) -> ! {
        let loop_period_us = 1000; // 1kHz
        let mut last_tick = self.runner.now_us();

        loop {
            let now = self.runner.now_us();
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
        self.runner.kernel.is_ready()
    }

    /// Check if the kernel is armed
    pub fn is_armed(&self) -> bool {
        self.runner.is_armed()
    }

    /// Get a reference to the kernel
    pub fn kernel(&self) -> &AviateKernel<MultirotorController, QuadXMixer> {
        &self.runner.kernel
    }

    /// Get a mutable reference to the kernel
    pub fn kernel_mut(&mut self) -> &mut AviateKernel<MultirotorController, QuadXMixer> {
        &mut self.runner.kernel
    }

    /// Get current timestamp in microseconds
    pub fn now_us(&self) -> u64 {
        self.runner.now_us()
    }

    /// Get a reference to the transport layer
    pub fn transport(&self) -> &SitlIO {
        &self.runner.transport
    }

    /// Get a mutable reference to the transport layer
    ///
    /// Use this to feed sensor data from simulator backends or
    /// read actuator commands for forwarding to the simulator.
    pub fn transport_mut(&mut self) -> &mut SitlIO {
        self.runner.transport_mut()
    }

    /// Get board ID
    pub fn board_id() -> &'static str {
        "sitl-gazebo"
    }
}

fn sitl_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

/// Board info for Gazebo SITL
pub const BOARD_INFO: BoardInfo = BoardInfo {
    name: "sitl-gazebo",
    description: "Gazebo SITL simulation board",
};

/// Board information structure
#[derive(Clone, Debug)]
pub struct BoardInfo {
    pub name: &'static str,
    pub description: &'static str,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_board_info() {
        assert_eq!(BOARD_INFO.name, "sitl-gazebo");
    }

    #[test]
    fn test_board_id() {
        assert_eq!(GazeboSitlBoard::board_id(), "sitl-gazebo");
    }
}
