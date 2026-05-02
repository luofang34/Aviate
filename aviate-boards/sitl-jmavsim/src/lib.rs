//! SITL jMAVSim Board Configuration
//!
//! This board represents a simulated quadcopter using jMAVSim via MAVLink HIL protocol.
//! It uses the shared `SitlRunner` from `aviate-runtime` for the control loop,
//! with `HilBackend` providing the MAVLink HIL transport layer to jMAVSim.
//!
//! ## Architecture
//!
//! ```text
//! SENSORS (Input):
//! jMAVSim → UDP/MAVLink HIL → HilBackend → SitlIO.feed_sensor_packet() → SitlRunner
//!                                                    ↓
//!                                              Same kernel code
//!                                                    ↓
//! ACTUATORS (Output):
//! SitlRunner → SitlIO.take_actuator_cmd() → HilBackend → UDP/MAVLink → jMAVSim
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
//! let mut board = JmavSimBoard::new()?;
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

use log::info;

use aviate_backend_mavlink_hil::{HilBackend, HilBackendConfig};
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::Command;
use aviate_core::mixer::{ActuatorCmd, QuadXMixer};
use aviate_core::{ArmError, DefaultAviateKernel, InitState};

use aviate_core::hal::ActuatorHal;
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{
    SimBaroData, SimGnssData, SimImuData, SimMagData, SimSensorPacket, SitlConfig, SitlIO,
};
use aviate_runtime::{
    create_kernel, default_command, loop_periods, SitlBoardInfo, SitlRunner, SitlTime,
};

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
/// Delegates control loop to `SitlRunner` from `aviate-runtime`.
pub struct JmavSimBoard {
    /// MAVLink HIL backend for communication with jMAVSim
    hil_backend: HilBackend,

    /// SITL runner (encapsulates transport, board HAL, and kernel)
    runner: SitlRunner,

    /// Armed state (for MAVLink heartbeat)
    armed: bool,
}

impl JmavSimBoard {
    /// Create a new jMAVSim board with default configuration
    pub fn new() -> io::Result<Self> {
        Self::with_config(JmavSimConfig::default())
    }

    /// Create a new jMAVSim board with custom configuration
    pub fn with_config(config: JmavSimConfig) -> io::Result<Self> {
        // Create HilBackend for MAVLink HIL communication with jMAVSim
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

        // Create SitlIO for transport abstraction (same as Gazebo)
        let sitl_config = SitlConfig::default();
        let transport = SitlIO::new(sitl_config)?;

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

        let kernel = create_kernel();
        let default_cmd = default_command();

        // Create SitlRunner with all components
        let runner = SitlRunner::new(transport, board_hal, kernel, default_cmd);

        Ok(Self {
            hil_backend,
            runner,
            armed: false,
        })
    }

    /// Run one iteration of the control loop
    ///
    /// This:
    /// 1. Polls HilBackend for MAVLink HIL messages from jMAVSim
    /// 2. Converts HIL data to SimSensorPacket and feeds to SitlIO
    /// 3. Runs SitlRunner.step() (handles control loop)
    /// 4. Gets actuator commands from SitlIO and sends to jMAVSim
    ///
    /// Returns the actuator command that was sent.
    pub fn step(&mut self) -> ActuatorCmd {
        // 1. Poll HilBackend for incoming MAVLink HIL messages from jMAVSim
        if let Some(packet) = self.hil_backend.poll() {
            // 2. Convert HIL data to SimSensorPacket and feed to SitlIO
            let mut sensor_packet = SimSensorPacket::default();

            if let Some(imu) = packet.imu {
                sensor_packet.imu = Some(SimImuData {
                    accel: imu.accel,
                    gyro: imu.gyro,
                    temperature: imu.temperature,
                });
            }

            if let Some(baro) = packet.baro {
                sensor_packet.baro = Some(SimBaroData {
                    pressure_pa: baro.pressure_pa,
                    temperature_c: baro.temperature_c,
                });
            }

            if let Some(mag) = packet.mag {
                sensor_packet.mag = Some(SimMagData {
                    field_ut: mag.field_ut,
                });
            }

            if let Some(gnss) = packet.gnss {
                sensor_packet.gnss = Some(SimGnssData {
                    lat_deg: gnss.lat_deg,
                    lon_deg: gnss.lon_deg,
                    alt_m: gnss.alt_m,
                    vel_ned: gnss.vel_ned,
                    fix: gnss.fix,
                    h_acc: gnss.h_acc,
                    v_acc: gnss.v_acc,
                    satellites: gnss.satellites,
                });
            }

            // Feed sensor packet to SitlIO
            self.runner.transport.feed_sensor_packet(&sensor_packet);
        }

        // 3. Run SitlRunner step (handles control loop, same as Gazebo)
        let actuator_cmd = self.runner.step();

        // 4. Get actuator commands and send to jMAVSim via HilBackend
        if let Some(sim_cmd) = self.runner.transport.take_actuator_cmd() {
            // Forward to jMAVSim via MAVLink HIL
            let _ = self.hil_backend.send_actuators(&sim_cmd);
        }

        actuator_cmd
    }

    /// Run the main control loop indefinitely
    ///
    /// Uses the shared control loop from aviate-runtime with jMAVSim's 400Hz rate.
    /// Note: This method cannot use `run_control_loop` directly because jMAVSim
    /// requires the HilBackend bridge in each step. Uses the same timing logic.
    pub fn run(&mut self) -> ! {
        let mut last_tick = self.runner.now_us();

        loop {
            let now = self.runner.now_us();
            let elapsed = now.saturating_sub(last_tick);

            if elapsed >= loop_periods::JMAVSIM_US {
                last_tick = now;
                self.step();
            } else {
                let remaining_us = loop_periods::JMAVSIM_US - elapsed;
                if remaining_us > 100 {
                    std::thread::sleep(std::time::Duration::from_micros(remaining_us - 100));
                }
            }
        }
    }

    /// Arm the flight controller
    pub fn arm(&mut self) -> Result<(), ArmError> {
        info!(
            "Arm command (state={:?})",
            self.runner.kernel.state.init_state
        );
        info!("Faults: {:?}", self.runner.kernel.state.faults);

        self.runner.kernel.arm()?;

        info!("Armed successfully");
        self.runner.board_hal.arm();
        self.runner.transport.set_armed(true);
        self.armed = true;
        Ok(())
    }

    /// Disarm the flight controller
    pub fn disarm(&mut self) {
        info!("Disarm command");
        self.runner.kernel.disarm();
        self.runner.board_hal.disarm();
        self.runner.transport.set_armed(false);
        self.armed = false;
    }

    /// Set the flight command (attitude/thrust setpoint)
    pub fn set_command(&mut self, cmd: Command) {
        self.runner
            .kernel
            .state
            .checks
            .pre_arm
            .update_throttle(cmd.setpoint.collective_thrust.0 < 0.1);
        self.runner.last_cmd = cmd;
    }

    /// Check if the kernel is ready for flight
    pub fn is_ready(&self) -> bool {
        self.runner.kernel.is_ready()
    }

    /// Check if the kernel is armed
    pub fn is_armed(&self) -> bool {
        self.runner.kernel.state.init_state == InitState::Armed
    }

    /// Get a reference to the kernel
    pub fn kernel(&self) -> &DefaultAviateKernel<MultirotorController, QuadXMixer> {
        &self.runner.kernel
    }

    /// Get a mutable reference to the kernel
    pub fn kernel_mut(&mut self) -> &mut DefaultAviateKernel<MultirotorController, QuadXMixer> {
        &mut self.runner.kernel
    }

    /// Get current timestamp in microseconds
    pub fn now_us(&self) -> u64 {
        self.runner.now_us()
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

    /// Get board ID
    pub fn board_id() -> &'static str {
        "sitl-jmavsim"
    }
}

/// Board info for jMAVSim SITL
pub const BOARD_INFO: SitlBoardInfo = SitlBoardInfo {
    name: "sitl-jmavsim",
    description: "jMAVSim SITL via MAVLink HIL protocol",
};

/// Re-export BoardInfo type for backwards compatibility
pub type BoardInfo = SitlBoardInfo;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_board_info() {
        assert_eq!(BOARD_INFO.name, "sitl-jmavsim");
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
