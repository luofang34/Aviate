#![forbid(unsafe_code)]

//! SITL (Software-In-The-Loop) platform implementation
//!
//! Provides HAL implementation for running aviate-core in simulation.
//! Supports both mock mode (for testing) and UDP MAVLink mode (for external simulators).

pub mod mock;
pub mod udp;

pub use mock::SitlHal;
pub use udp::UdpMavlinkHal;

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
    /// Loop rate in Hz (default 1000)
    pub loop_rate_hz: u32,
}

impl Default for SitlConfig {
    fn default() -> Self {
        Self {
            sensor_port: DEFAULT_SENSOR_PORT,
            simulator_addr: std::net::SocketAddr::from(([127, 0, 0, 1], DEFAULT_ACTUATOR_PORT)),
            loop_rate_hz: 1000,
        }
    }
}
