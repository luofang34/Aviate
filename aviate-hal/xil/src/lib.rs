//! X-In-Loop (XIL) Platform Core
//!
//! Backend-agnostic platform for SITL (Software-In-The-Loop) and HITL (Hardware-In-The-Loop)
//! simulation. This crate provides:
//!
//! - **Backend trait**: Typed simulator directives, receipts, and frames
//! - **World state**: Backend-agnostic representation of simulation world
//! - **Test infrastructure**: Mission framework, test runner, config parsing
//! - **SITL transport**: Network communication with simulators (MAVLink/UDP)
//!
//! ## Architecture
//!
//! ```text
//! aviate-hal-xil (this crate, no backend deps)
//!        ↑
//! aviate-backend-gz (implements SimulatorBackend)
//!        ↑ (FFI/IPC)
//! aviate_gz_plugin (C++, Gazebo)
//! ```
//!
//! The xil core does NOT depend on any specific backend. Backends implement
//! traits defined here and are selected at runtime via configuration.

#![forbid(unsafe_code)]
// deny (not forbid) so a `#[cfg(test)]` module can `#[allow]` these for
// assertion helpers; forbid cannot be relaxed and breaks `clippy
// --all-targets` on the test build.
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod backend;
pub mod bridge;
pub mod command_provenance;
pub mod config;
pub mod fault_ctrl;
pub mod fault_protocol;
pub mod flight_log;
pub mod mission;
pub mod mock;
mod network;
pub mod perturbation;
pub mod runner;
pub mod sim_types;
pub mod sitl_io;
pub mod world;

// Core exports
pub use backend::{
    BackendStatus, DirectiveId, DirectiveOutcome, DirectiveReceipt, FrameEvent, ResetGeneration,
    SimulatorBackend, SimulatorDirective, SimulatorDirectiveKind, SimulatorError, SimulatorFrame,
    SimulatorLifecycle, SimulatorOperation, VehicleState,
};
pub use world::{
    AngularVelocity, Entity, EntityId, EntityState, Position, Quaternion, Velocity, World,
};

// Transport exports
pub use command_provenance::{MavlinkCommandFamily, MavlinkCommandProvenance};
pub use mock::SitlHal;
pub use network::{PortSlot, SitlConfig, XilConfig, XilNetConfig};
pub use sitl_io::{HilGpsData, HilSensorData, ReceivedCommand, SitlIO};

// Simulator-neutral data types (for direct FFI integration)
pub use sim_types::{
    SimActuatorCmd, SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
    SimTimestampUs,
};

// Re-export legacy name for compatibility
pub use SitlIO as SitlMavlink;

// Flight log exports
pub use flight_log::{FlightLog, FlightLogConfig, FlightSample, FlightStats};

// Test infrastructure exports
pub use config::{parse_test_config, parse_test_config_str, TestConfig, VehicleTestConfig};
pub use mission::{
    Action, Criterion, CriterionResult, FaultSpec, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, SensorTarget, VehicleConfig,
};

// Runner exports (backend-agnostic mission execution)
pub use runner::{
    run_test_config, run_test_config_from_current_state, MavClient, MissionRunner, TestResult,
};

// Fault injection protocol exports
pub use fault_protocol::{
    AckStatus, FaultAck, FaultAction, FaultClient, FaultCommand, FAULT_ACK_MAGIC, FAULT_ACK_SIZE,
    FAULT_CMD_MAGIC, FAULT_CMD_SIZE,
};

// Fault controller exports
pub use fault_ctrl::{FaultController, FaultCtrlError, FaultSensors};
