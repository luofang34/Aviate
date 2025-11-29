//! Aviate Quadcopter SITL Application
//!
//! This crate provides the SITL (Software-In-The-Loop) simulation environment
//! for testing the Aviate flight controller with Gazebo.
//!
//! ## Components
//!
//! - **Mission Framework**: Define and execute test missions
//! - **Lockstep Synchronization**: Deterministic simulation support
//! - **Gazebo Integration**: Zero-copy physics data via shared memory
//! - **Test Configuration**: TOML-based test scenario definitions
//! - **World Generation**: Dynamic SDF world file generation

pub mod mission;
pub mod mission_runner;
pub mod router_gen;
pub mod test_config;
pub mod world_gen;

#[cfg(feature = "multi-vehicle")]
pub mod multi_vehicle;

pub use mission::{
    Action, Criterion, CriterionResult, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, VehicleConfig,
};

pub use router_gen::{
    generate_router_config, generate_router_config_file, generate_temp_router_config, vehicle_port,
    RouterParams, GCS_PORT, VEHICLE_BASE_PORT,
};
pub use test_config::{parse_test_config, parse_test_config_str, TestConfig, VehicleTestConfig};
pub use world_gen::{generate_temp_world, generate_world, generate_world_file, WorldParams};

#[cfg(feature = "gz-plugin")]
pub use mission_runner::{run_mission_suite, run_mission_suite_for_instance, MissionRunner};

#[cfg(feature = "multi-vehicle")]
pub use multi_vehicle::{start_router, RouterError, RouterHandle};
