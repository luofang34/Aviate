//! X-In-Loop (XIL) Platform Core
//!
//! Backend-agnostic platform for SITL (Software-In-The-Loop) and HITL (Hardware-In-The-Loop)
//! simulation. This crate provides:
//!
//! - **Backend trait**: Interface for kinematics backends (Gazebo, Unity, Chrono, etc.)
//! - **World state**: Backend-agnostic representation of simulation world
//! - **Test infrastructure**: Mission framework, test runner, config parsing
//! - **HAL implementations**: Mock and UDP MAVLink HAL for testing
//!
//! ## Architecture
//!
//! ```text
//! aviate-platform-xil (this crate, no backend deps)
//!        ↑
//! aviate-backend-gz (implements KinematicsBackend)
//!        ↑ (FFI/IPC)
//! aviate_gz_plugin (C++, Gazebo)
//! ```
//!
//! The xil core does NOT depend on any specific backend. Backends implement
//! traits defined here and are selected at runtime via configuration.

#![forbid(unsafe_code)]

pub mod backend;
pub mod bridge;
pub mod flight_log;
pub mod mock;
pub mod test;
pub mod udp;
pub mod world;

// Core exports
pub use backend::{BackendConfig, BackendError, KinematicsBackend, LockstepMode, TimingMode};
pub use world::{
    AngularVelocity, Entity, EntityId, EntityState, Position, Quaternion, Velocity, World,
};

// HAL exports
pub use mock::SitlHal;
pub use udp::UdpMavlinkHal;

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
#[derive(Clone, Debug)]
pub struct XilConfig {
    /// UDP port to receive sensor data from simulator
    pub sensor_port: u16,
    /// Address to send actuator commands to
    pub simulator_addr: std::net::SocketAddr,
    /// Address to send telemetry to (GCS)
    pub gcs_addr: std::net::SocketAddr,
    /// Loop rate in Hz (default 1000)
    pub loop_rate_hz: u32,
}

impl Default for XilConfig {
    fn default() -> Self {
        Self {
            sensor_port: DEFAULT_SENSOR_PORT,
            simulator_addr: std::net::SocketAddr::from(([127, 0, 0, 1], DEFAULT_ACTUATOR_PORT)),
            gcs_addr: std::net::SocketAddr::from(([127, 0, 0, 1], 14550)),
            loop_rate_hz: 1000,
        }
    }
}

// Re-export legacy name for compatibility
pub type SitlConfig = XilConfig;
