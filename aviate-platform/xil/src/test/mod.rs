//! Test Infrastructure
//!
//! Mission framework, test configuration parsing, and test runner.
//! Backend-agnostic - works with any kinematics backend.

pub mod mission;
pub mod config;

// Re-export mission types
pub use mission::{
    Action, Criterion, CriterionResult, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, VehicleConfig,
};
