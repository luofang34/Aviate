//! SITL (Software-In-The-Loop) platform implementation
//!
//! Provides HAL implementation for running aviate-core in simulation.
//! Supports both mock mode (for testing) and UDP MAVLink mode (for external simulators).
//!
//! ## Features
//!
//! - `gz-plugin`: Enable Gazebo bridge via shared memory FFI (zero-copy, requires libaviate_gz_bridge.so)
//!
//! ## Architecture
//!
//! The SITL platform consists of:
//!
//! - **GzBridge**: Bridges Gazebo physics to MAVLink HIL protocol
//!   - Reads model state via shared memory (AviateGzPlugin)
//!   - Sends HIL_SENSOR/HIL_GPS to Aviate autopilot
//!   - Receives HIL_ACTUATOR_CONTROLS and forwards to Gazebo
//!
//! - **UdpMavlinkHal**: HAL implementation for receiving simulator data
//!   - Binds to UDP ports for MAVLink communication
//!   - Parses HIL_SENSOR/HIL_GPS messages
//!   - Sends HIL_ACTUATOR_CONTROLS

// Allow unsafe code only for gz-plugin FFI
#![cfg_attr(not(feature = "gz-plugin"), forbid(unsafe_code))]

pub mod mock;
pub mod udp;
pub mod bridge;
pub mod gz_bridge;
pub mod flight_log;

#[cfg(feature = "gz-plugin")]
pub mod gz_plugin;

pub use mock::SitlHal;
pub use udp::UdpMavlinkHal;

// gz_bridge exports (requires gz-plugin feature for full functionality)
pub use gz_bridge::{GzBridge, GzBridgeConfig, GzBridgeError};

// flight_log exports
pub use flight_log::{FlightLog, FlightLogConfig, FlightSample, FlightStats};

#[cfg(feature = "gz-plugin")]
pub use gz_plugin::{GzPluginBridge, GzPluginError, AviateModelState, enu_to_ned, enu_vel_to_ned};

/// Default ports for MAVLink HIL communication
pub const DEFAULT_SENSOR_PORT: u16 = 14560;  // Receive HIL_SENSOR/HIL_GPS from simulator
pub const DEFAULT_ACTUATOR_PORT: u16 = 14561; // Send HIL_ACTUATOR_CONTROLS to simulator

/// SITL configuration
#[derive(Clone, Debug)]
pub struct SitlConfig {
    /// UDP port to receive sensor data from simulator
    pub sensor_port: u16,
    /// Address to send actuator commands to
    pub simulator_addr: std::net::SocketAddr,
    /// Address to send telemetry to (GCS)
    pub gcs_addr: std::net::SocketAddr,
    /// Loop rate in Hz (default 1000)
    pub loop_rate_hz: u32,
}

impl Default for SitlConfig {
    fn default() -> Self {
        Self {
            sensor_port: DEFAULT_SENSOR_PORT,
            simulator_addr: std::net::SocketAddr::from(([127, 0, 0, 1], DEFAULT_ACTUATOR_PORT)),
            gcs_addr: std::net::SocketAddr::from(([127, 0, 0, 1], 14550)),
            loop_rate_hz: 1000,
        }
    }
}
