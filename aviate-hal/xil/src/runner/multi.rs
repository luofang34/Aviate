//! Parallel execution for one test configuration.

use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use tracing::error;

use crate::config::TestConfig;
use crate::mission::{Mission, MissionResult};
use crate::{SimulatorBackend, SimulatorError};

use super::MissionRunner;

/// Result from one complete test configuration.
#[derive(Debug)]
pub struct TestResult {
    /// Test name.
    pub name: String,
    /// Mission results for each vehicle.
    pub vehicle_results: Vec<MissionResult>,
    /// True when all vehicle missions passed.
    pub passed: bool,
    /// Total test duration.
    pub duration: Duration,
}

/// Run one test configuration with concurrent vehicles.
///
/// The `backend_factory` creates one backend for each vehicle instance.
pub fn run_test_config<B, F>(config: &TestConfig, backend_factory: F) -> TestResult
where
    B: SimulatorBackend + 'static,
    F: Fn(u8) -> Result<B, SimulatorError> + Send + Sync + 'static,
{
    run_test_config_with_preparation(config, backend_factory, true)
}

/// Run one test from the current generation of each simulator.
///
/// The caller must own simulator startup. The caller must guarantee that each
/// current generation contains the declared clean initial state. This function
/// does not send a reset directive.
pub fn run_test_config_from_current_state<B, F>(
    config: &TestConfig,
    backend_factory: F,
) -> TestResult
where
    B: SimulatorBackend + 'static,
    F: Fn(u8) -> Result<B, SimulatorError> + Send + Sync + 'static,
{
    run_test_config_with_preparation(config, backend_factory, false)
}

fn run_test_config_with_preparation<B, F>(
    config: &TestConfig,
    backend_factory: F,
    reset: bool,
) -> TestResult
where
    B: SimulatorBackend + 'static,
    F: Fn(u8) -> Result<B, SimulatorError> + Send + Sync + 'static,
{
    let start = Instant::now();
    let factory = Arc::new(backend_factory);

    let handles: Vec<_> = config
        .vehicles
        .iter()
        .map(|vehicle| {
            let vehicle_id = vehicle.id.clone();
            let instance = vehicle.instance;
            let mission = vehicle.mission.clone();
            let factory = Arc::clone(&factory);

            thread::spawn(move || {
                run_vehicle(factory.as_ref(), instance, &vehicle_id, &mission, reset)
            })
        })
        .collect();

    let vehicle_results: Vec<MissionResult> = handles
        .into_iter()
        .map(|h| {
            h.join().unwrap_or_else(|_| MissionResult {
                mission_name: "unknown".to_string(),
                passed: false,
                phases: vec![],
                total_duration: Duration::ZERO,
                max_altitude: 0.0,
            })
        })
        .collect();

    let all_passed = vehicle_results.iter().all(|r| r.passed);

    TestResult {
        name: config.name.clone(),
        vehicle_results,
        passed: all_passed,
        duration: start.elapsed(),
    }
}

fn run_vehicle<B, F>(
    factory: &F,
    instance: u8,
    vehicle_id: &str,
    mission: &Mission,
    reset: bool,
) -> MissionResult
where
    B: SimulatorBackend + 'static,
    F: Fn(u8) -> Result<B, SimulatorError> + Send + Sync + 'static,
{
    let backend = match factory(instance) {
        Ok(backend) => backend,
        Err(error) => {
            error!("[{vehicle_id}:{instance}] Failed to create backend: {error}");
            return failed_result(mission);
        }
    };
    match MissionRunner::new(backend, vehicle_id) {
        Ok(mut runner) if reset => runner.run(mission),
        Ok(mut runner) => runner.run_from_current_state(mission),
        Err(error) => {
            error!("[{vehicle_id}:{instance}] Failed to create runner: {error}");
            failed_result(mission)
        }
    }
}

fn failed_result(mission: &Mission) -> MissionResult {
    MissionResult {
        mission_name: mission.name.clone(),
        passed: false,
        phases: Vec::new(),
        total_duration: Duration::ZERO,
        max_altitude: 0.0,
    }
}
