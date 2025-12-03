//! Aviate Gazebo SITL Application
//!
//! This crate provides the SITL (Software-In-The-Loop) simulation environment
//! for testing the Aviate flight controller with Gazebo.
//!
//! ## Components
//!
//! - **Mission Framework**: Shared mission types from aviate-hal-xil
//! - **Lockstep Synchronization**: Deterministic simulation support
//! - **Gazebo Integration**: Zero-copy physics data via shared memory
//! - **World Generation**: Dynamic SDF world file generation

pub mod mission_runner;
pub mod router_gen;
pub mod world_gen;

#[cfg(feature = "multi-vehicle")]
pub mod multi_vehicle;

// Re-export mission types from aviate-hal-xil (shared across all simulators)
pub use aviate_hal_xil::{
    parse_test_config, parse_test_config_str, Action, Criterion, CriterionResult, FaultSpec,
    Mission, MissionResult, MultiVehicleCriterion, MultiVehicleMission, MultiVehiclePhase, Phase,
    PhaseResult, SensorTarget, TestConfig, VehicleConfig, VehicleTestConfig,
};

pub use router_gen::{
    generate_router_config, generate_router_config_file, generate_temp_router_config, vehicle_port,
    RouterParams, GCS_PORT,
};
pub use world_gen::{generate_temp_world, generate_world, generate_world_file, WorldParams};

#[cfg(feature = "gz-plugin")]
pub use mission_runner::{run_mission_suite, run_mission_suite_for_instance, MissionRunner};

#[cfg(feature = "multi-vehicle")]
pub use multi_vehicle::{start_router, RouterError, RouterHandle};
