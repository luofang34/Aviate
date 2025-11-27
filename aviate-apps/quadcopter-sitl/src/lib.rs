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
pub mod test_config;
pub mod world_gen;

pub use mission::{
    Action, Criterion, CriterionResult, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, VehicleConfig,
};

pub use test_config::{parse_test_config, parse_test_config_str, TestConfig, VehicleTestConfig};
pub use world_gen::{generate_world, generate_world_file, generate_temp_world, WorldParams};

#[cfg(feature = "gz-plugin")]
pub use mission_runner::{run_mission_suite, run_mission_suite_for_instance, MissionRunner};
