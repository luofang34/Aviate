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

pub mod mission;
pub mod mission_runner;

pub use mission::{
    Action, Criterion, CriterionResult, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, VehicleConfig,
};

#[cfg(feature = "gz-plugin")]
pub use mission_runner::{run_mission_suite, MissionRunner};
