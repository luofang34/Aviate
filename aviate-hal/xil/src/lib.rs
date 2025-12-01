//! X-In-Loop (XIL) Platform Core
//!
//! Backend-agnostic platform for SITL (Software-In-The-Loop) and HITL (Hardware-In-The-Loop)
//! simulation. This crate provides:
//!
//! - **Backend trait**: Interface for kinematics backends (Gazebo, Unity, Chrono, etc.)
//! - **World state**: Backend-agnostic representation of simulation world
//! - **Test infrastructure**: Mission framework, test runner, config parsing
//! - **SITL transport**: Network communication with simulators (MAVLink/UDP)
//!
//! ## Architecture
//!
//! ```text
//! aviate-hal-xil (this crate, no backend deps)
//!        ↑
//! aviate-backend-gz (implements KinematicsBackend)
//!        ↑ (FFI/IPC)
//! aviate_gz_plugin (C++, Gazebo)
//! ```
//!
//! The xil core does NOT depend on any specific backend. Backends implement
//! traits defined here and are selected at runtime via configuration.

#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod backend;
pub mod bridge;
pub mod flight_log;
pub mod mock;
pub mod sitl_io;
pub mod test;
pub mod udp;
pub mod world;

// Core exports
pub use backend::{BackendConfig, BackendError, KinematicsBackend, LockstepMode, TimingMode};
pub use world::{
    AngularVelocity, Entity, EntityId, EntityState, Position, Quaternion, Velocity, World,
};

// Transport exports
pub use mock::SitlHal;
pub use sitl_io::{HilGpsData, HilSensorData, SitlIO};
pub use udp::UdpMavlinkHal;

// Re-export legacy name for compatibility
pub use SitlIO as SitlMavlink;

// Flight log exports
pub use flight_log::{FlightLog, FlightLogConfig, FlightSample, FlightStats};

// Test infrastructure exports
pub use test::config::{parse_test_config, parse_test_config_str, TestConfig, VehicleTestConfig};
pub use test::{
    Action, Criterion, CriterionResult, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, VehicleConfig,
};

/// Default ports for MAVLink HIL communication
pub const DEFAULT_SENSOR_PORT: u16 = 14560;
pub const DEFAULT_ACTUATOR_PORT: u16 = 14561;

/// XIL configuration
///
/// Supports multi-vehicle simulation via instance IDs.
/// Each instance uses separate UDP ports:
/// - Instance 0: sensor=14560, actuator=14561
/// - Instance N: sensor=14560+N*10, actuator=14561+N*10
#[derive(Clone, Debug)]
pub struct XilConfig {
    /// Instance ID for multi-vehicle simulation (0 for single vehicle)
    pub instance: u8,
    /// UDP port to receive sensor data from simulator
    pub sensor_port: u16,
    /// Address to send actuator commands to
    pub simulator_addr: std::net::SocketAddr,
    /// Address to send telemetry to (GCS)
    pub gcs_addr: std::net::SocketAddr,
    /// Loop rate in Hz (default 1000)
    pub loop_rate_hz: u32,
}

impl XilConfig {
    /// Create config for a specific instance ID
    ///
    /// Port allocation: base + instance * 10
    pub fn for_instance(instance: u8) -> Self {
        let base_port = DEFAULT_SENSOR_PORT + (instance as u16) * 10;
        Self {
            instance,
            sensor_port: base_port,
            simulator_addr: std::net::SocketAddr::from(([127, 0, 0, 1], base_port + 1)),
            gcs_addr: std::net::SocketAddr::from(([127, 0, 0, 1], 14550)),
            loop_rate_hz: 1000,
        }
    }
}

impl Default for XilConfig {
    fn default() -> Self {
        Self::for_instance(0)
    }
}

// Re-export legacy name for compatibility
pub type SitlConfig = XilConfig;
