//! Test Infrastructure
//!
//! Mission framework, test configuration parsing, and test runner.
//! Backend-agnostic - works with any kinematics backend.

pub mod config;
pub mod mission;

// Re-export mission types
pub use mission::{
    Action, Criterion, CriterionResult, FaultSpec, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, SensorTarget, VehicleConfig,
};
