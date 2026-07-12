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
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]

use std::io;

use log::warn;

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::mixer::{ActuatorCmd, QuadXMixerX500};
use aviate_core::DefaultAviateKernel;

use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::ConfigMode;
use aviate_core::mixer::ModeConfig;
use aviate_core::AviateKernel;
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SitlConfig, SitlIO};
use aviate_runtime::{loop_periods, sitl_timestamp, SitlBoardInfo, SitlRunner};

/// X500 kernel construction, owned by this app.
///
/// Airframe selection is an application decision: the runtime provides
/// only the generic `SitlRunner`, and this board states visibly that
/// it flies the X500 controller/mixer pair. One tuning source: the
/// same `CascadeGains` value and hover trim construct the flying
/// controller AND land in the lockstep-hashed `ResolvedKernelConfig`;
/// the builder-level binding check makes a divergent pair
/// unconstructible. Preset-file loading replaces these literals with
/// configuration when app-owned preset construction lands.
pub fn create_x500_kernel() -> DefaultAviateKernel<MultirotorController, QuadXMixerX500> {
    let gains = CascadeGains::x500_defaults();
    let hover: f32 = 0.77;
    let controller = MultirotorController::from_gains(gains, hover);
    let mixer = QuadXMixerX500 {
        timestamp_source: sitl_timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };

    let mut kernel = AviateKernel::new(
        aviate_core::ekf::Ekf::default(),
        controller,
        mixer,
        aviate_core::mixer::Sanitizer,
        mode_config,
    );
    kernel.cfg.cascade_gains = gains;
    kernel.cfg.hover_thrust_norm = aviate_core::types::Normalized(hover);

    // Default command carries low throttle, so the throttle pre-arm
    // check starts satisfied.
    kernel.state.checks.pre_arm.update_throttle(true);

    kernel
}

/// Gazebo SITL board configuration
///
/// Uses the same BoardHal abstraction as real hardware boards, ensuring
/// that SITL tests exercise the same code paths as real hardware.
///
/// **Phase 1**: Delegates to SitlRunner for control loop execution.
/// This eliminates ~165 lines of duplication from the board implementation.
pub struct GazeboSitlBoard {
    /// SITL runner (encapsulates transport, board HAL, and kernel)
    runner: SitlRunner<MultirotorController, QuadXMixerX500>,
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

        // Use shared factory functions from aviate-runtime
        let kernel = create_x500_kernel();

        // Create SitlRunner with all components
        let runner = SitlRunner::new(transport, board_hal, kernel);

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
                        warn!(
                            "Failed to bind port {}: {}. Retrying in {}ms...",
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
    ///
    /// Uses the shared control loop from aviate-runtime with Gazebo's 1kHz rate.
    pub fn run(&mut self) -> ! {
        aviate_runtime::run_control_loop(&mut self.runner, loop_periods::GAZEBO_US)
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
    pub fn kernel(&self) -> &DefaultAviateKernel<MultirotorController, QuadXMixerX500> {
        &self.runner.kernel
    }

    /// Get a mutable reference to the kernel
    pub fn kernel_mut(&mut self) -> &mut DefaultAviateKernel<MultirotorController, QuadXMixerX500> {
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

    /// Initialize telemetry from application config
    ///
    /// Call this after creating the board to enable GCS telemetry output.
    ///
    /// # Arguments
    /// * `cfg` - Application configuration (from AviateApp.toml)
    /// * `loop_hz` - Control loop frequency in Hz (e.g., 1000 for 1kHz)
    pub fn init_telemetry(&mut self, cfg: &aviate_config::AppConfig, loop_hz: u32) {
        self.runner.init_telemetry(cfg, loop_hz);
    }
}

/// Board info for Gazebo SITL
pub const BOARD_INFO: SitlBoardInfo = SitlBoardInfo {
    name: "sitl-gazebo",
    description: "Gazebo SITL simulation board",
};

/// Re-export BoardInfo type for backwards compatibility
pub type BoardInfo = SitlBoardInfo;

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
