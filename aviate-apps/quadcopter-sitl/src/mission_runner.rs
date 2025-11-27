//! Mission Runner - Executes missions under lockstep simulation
//!
//! This module provides the runtime for executing missions defined in mission.rs.
//! All test missions run under lockstep for deterministic, reproducible results.
//!
//! ## Multi-Vehicle Support
//!
//! Each vehicle connects to its own shared memory segment (instance-based).
//! For multi-vehicle tests, create separate MissionRunner instances with
//! different instance IDs.

#[cfg(feature = "gz-plugin")]
use std::time::{Duration, Instant};

#[cfg(feature = "gz-plugin")]
use crate::mission::{
    Action, Criterion, Mission, MissionResult, Phase, PhaseResult, CriterionResult,
};

#[cfg(feature = "gz-plugin")]
use aviate_platform_sitl::gz_plugin::{GzPluginBridge, AviateModelState, enu_to_ned_f32};

/// Mission runner state
///
/// Each MissionRunner connects to a specific vehicle instance via shared memory.
/// Instance 0 uses /aviate_gz_bridge, instance N uses /aviate_gz_bridge_N.
#[cfg(feature = "gz-plugin")]
pub struct MissionRunner {
    bridge: GzPluginBridge,
    instance: u8,
    vehicle_id: String,
    last_step: u64,
    current_position: [f32; 3],
    current_velocity: [f32; 3],
    start_position: [f32; 3],
    armed: bool,
    max_altitude: f32,
}

#[cfg(feature = "gz-plugin")]
impl MissionRunner {
    /// Create a new mission runner connected to Gazebo (instance 0)
    pub fn new() -> Result<Self, String> {
        Self::for_instance(0, "x500")
    }

    /// Create a new mission runner for a specific vehicle instance
    ///
    /// Each vehicle instance has its own shared memory segment.
    /// Use this for multi-vehicle testing.
    pub fn for_instance(instance: u8, vehicle_id: &str) -> Result<Self, String> {
        let bridge = GzPluginBridge::connect_instance_with_retry(instance, 20, 500)
            .map_err(|e| format!("Failed to connect to instance {}: {:?}", instance, e))?;

        Ok(Self {
            bridge,
            instance,
            vehicle_id: vehicle_id.to_string(),
            last_step: 0,
            current_position: [0.0; 3],
            current_velocity: [0.0; 3],
            start_position: [0.0; 3],
            armed: false,
            max_altitude: 0.0,
        })
    }

    /// Get the instance ID this runner is connected to
    pub fn instance(&self) -> u8 {
        self.instance
    }

    /// Get the vehicle ID
    pub fn vehicle_id(&self) -> &str {
        &self.vehicle_id
    }

    /// Log a message with vehicle prefix
    fn log(&self, msg: &str) {
        println!("[{}:{}] {}", self.vehicle_id, self.instance, msg);
    }

    /// Run a complete mission
    pub fn run(&mut self, mission: &Mission) -> MissionResult {
        self.log(&format!("=== Mission: {} ===", mission.name));
        self.log(&format!("Description: {}", mission.description));
        self.log(&format!("Lockstep: {}", if mission.lockstep { "YES" } else { "NO" }));
        println!();

        // Enable lockstep if required
        if mission.lockstep {
            self.bridge.set_lockstep(true);
            self.log("Lockstep enabled");
        }

        // Wait for simulation to be ready and model to be found
        // The plugin needs time to locate the included model entity
        let timeout = Duration::from_secs(10);
        let start = Instant::now();
        let mut found_model = false;

        while start.elapsed() < timeout {
            self.last_step = self.bridge.sim_step();
            if let Some(state) = self.bridge.get_model_state() {
                // Check for valid position (valid != 0 and time_us > 0 means plugin found model)
                if state.valid != 0 && state.time_us > 0 {
                    self.start_position = enu_to_ned_f32(state.pos);
                    self.current_position = self.start_position;
                    found_model = true;
                    self.log(&format!("Model state ready (time={}us, step={})", state.time_us, self.last_step));
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }

        if !found_model {
            self.log("WARNING: Model state not available, simulation may not work correctly");
        }

        let mission_start = Instant::now();
        let mut phase_results = Vec::new();
        let mut mission_passed = true;

        // Execute each phase
        for (i, phase) in mission.phases.iter().enumerate() {
            self.log(&format!("[Phase {}/{}] {}", i + 1, mission.phases.len(), phase.name));

            let result = self.run_phase(phase);

            if result.passed {
                self.log(&format!("  PASSED (alt: {:.2}m)", result.max_altitude));
            } else {
                self.log("  FAILED");
                for cr in &result.criteria_results {
                    if !cr.passed {
                        self.log(&format!("    - {}: expected {}, got {}",
                            cr.criterion, cr.expected, cr.actual_value));
                    }
                }
                mission_passed = false;
            }

            phase_results.push(result);
        }

        // Disable lockstep
        if mission.lockstep {
            self.bridge.set_lockstep(false);
        }

        let total_duration = mission_start.elapsed();

        println!();
        self.log(&format!("=== Mission {} ===", if mission_passed { "PASSED" } else { "FAILED" }));
        self.log(&format!("Duration: {:.2}s", total_duration.as_secs_f32()));
        self.log(&format!("Max altitude: {:.2}m", self.max_altitude));

        MissionResult {
            mission_name: mission.name.clone(),
            passed: mission_passed,
            phases: phase_results,
            total_duration,
            max_altitude: self.max_altitude,
        }
    }

    /// Run a single phase
    fn run_phase(&mut self, phase: &Phase) -> PhaseResult {
        let phase_start = Instant::now();
        let mut phase_max_altitude = 0.0f32;
        let _step_timeout_us = 50000; // 50ms timeout per step (future: use with wait_and_process)

        // Execute for the phase duration
        while phase_start.elapsed() < phase.duration {
            // Wait for next simulation step
            let current_step = self.bridge.sim_step();

            if current_step > self.last_step {
                // Read state
                if let Some(state) = self.bridge.get_model_state() {
                    self.update_state(&state);
                    if -self.current_position[2] > phase_max_altitude {
                        phase_max_altitude = -self.current_position[2];
                    }
                    if phase_max_altitude > self.max_altitude {
                        self.max_altitude = phase_max_altitude;
                    }
                }

                // Execute action
                self.execute_action(&phase.action);

                // Acknowledge step
                self.bridge.ack_step(current_step);
                self.last_step = current_step;
            }

            std::thread::sleep(Duration::from_micros(100));
        }

        // Verify criteria
        let criteria_results: Vec<CriterionResult> = phase.verify
            .iter()
            .map(|c| self.verify_criterion(c, phase_max_altitude))
            .collect();

        let passed = criteria_results.iter().all(|r| r.passed);

        PhaseResult {
            name: phase.name.clone(),
            passed,
            duration_actual: phase_start.elapsed(),
            max_altitude: phase_max_altitude,
            final_position: self.current_position,
            criteria_results,
        }
    }

    /// Update internal state from simulation
    fn update_state(&mut self, state: &AviateModelState) {
        self.current_position = enu_to_ned_f32(state.pos);
        self.current_velocity = enu_to_ned_f32(state.vel);
    }

    /// Execute an action
    fn execute_action(&mut self, action: &Action) {
        match action {
            Action::Wait => {}
            Action::Arm => {
                self.armed = true;
                // In a full implementation, send MAVLink arm command
            }
            Action::Disarm => {
                self.armed = false;
                let _ = self.bridge.set_motor_speeds(&[0.0, 0.0, 0.0, 0.0]);
            }
            Action::Thrust(t) => {
                if self.armed {
                    let speed = t * 1000.0; // Scale to rad/s
                    let _ = self.bridge.set_motor_speeds(&[speed as f64, speed as f64, speed as f64, speed as f64]);
                }
            }
            Action::AttitudeTarget { q: _, thrust } => {
                if self.armed {
                    let speed = thrust * 1000.0;
                    let _ = self.bridge.set_motor_speeds(&[speed as f64, speed as f64, speed as f64, speed as f64]);
                }
            }
            Action::GoTo { position: _, heading: _ } => {
                // Future: implement position control
            }
        }
    }

    /// Verify a criterion
    fn verify_criterion(&self, criterion: &Criterion, phase_max_alt: f32) -> CriterionResult {
        match criterion {
            Criterion::Armed(expected) => CriterionResult {
                criterion: "armed".to_string(),
                passed: self.armed == *expected,
                actual_value: self.armed.to_string(),
                expected: expected.to_string(),
            },
            Criterion::MinAltitude(min) => {
                let alt = phase_max_alt;
                CriterionResult {
                    criterion: "min_altitude".to_string(),
                    passed: alt >= *min,
                    actual_value: format!("{:.2}m", alt),
                    expected: format!(">= {:.2}m", min),
                }
            }
            Criterion::MaxAltitude(max) => {
                let alt = -self.current_position[2]; // NED z is negative up
                CriterionResult {
                    criterion: "max_altitude".to_string(),
                    passed: alt <= *max,
                    actual_value: format!("{:.2}m", alt),
                    expected: format!("<= {:.2}m", max),
                }
            }
            Criterion::AltitudeHold { target, tolerance } => {
                let alt = -self.current_position[2];
                let error = (alt - target).abs();
                CriterionResult {
                    criterion: "altitude_hold".to_string(),
                    passed: error <= *tolerance,
                    actual_value: format!("{:.2}m (error: {:.2}m)", alt, error),
                    expected: format!("{:.2}m +/- {:.2}m", target, tolerance),
                }
            }
            Criterion::PositionHold { target, tolerance } => {
                let dx = self.current_position[0] - target[0];
                let dy = self.current_position[1] - target[1];
                let dz = self.current_position[2] - target[2];
                let error = (dx*dx + dy*dy + dz*dz).sqrt();
                CriterionResult {
                    criterion: "position_hold".to_string(),
                    passed: error <= *tolerance,
                    actual_value: format!("error: {:.2}m", error),
                    expected: format!("<= {:.2}m", tolerance),
                }
            }
            Criterion::MaxDrift(max) => {
                let dx = self.current_position[0] - self.start_position[0];
                let dy = self.current_position[1] - self.start_position[1];
                let drift = (dx*dx + dy*dy).sqrt();
                CriterionResult {
                    criterion: "max_drift".to_string(),
                    passed: drift <= *max,
                    actual_value: format!("{:.2}m", drift),
                    expected: format!("<= {:.2}m", max),
                }
            }
            Criterion::SensorDataReceived => {
                // Check if we've received any state updates
                CriterionResult {
                    criterion: "sensor_data".to_string(),
                    passed: self.last_step > 0,
                    actual_value: format!("{} steps", self.last_step),
                    expected: "> 0 steps".to_string(),
                }
            }
        }
    }
}

/// Run multiple missions in sequence on instance 0
#[cfg(feature = "gz-plugin")]
pub fn run_mission_suite(missions: &[Mission]) -> Vec<MissionResult> {
    run_mission_suite_for_instance(0, "x500", missions)
}

/// Run multiple missions in sequence on a specific instance
#[cfg(feature = "gz-plugin")]
pub fn run_mission_suite_for_instance(
    instance: u8,
    vehicle_id: &str,
    missions: &[Mission],
) -> Vec<MissionResult> {
    let mut results = Vec::new();

    for mission in missions {
        // Create fresh runner for each mission
        match MissionRunner::for_instance(instance, vehicle_id) {
            Ok(mut runner) => {
                let result = runner.run(mission);
                results.push(result);
            }
            Err(e) => {
                eprintln!("[{}:{}] Failed to create runner for {}: {}",
                    vehicle_id, instance, mission.name, e);
                results.push(MissionResult {
                    mission_name: mission.name.clone(),
                    passed: false,
                    phases: vec![],
                    total_duration: Duration::ZERO,
                    max_altitude: 0.0,
                });
            }
        }
    }

    results
}

#[cfg(test)]
mod tests {
    use crate::mission::CriterionResult;

    #[test]
    fn test_criterion_result() {
        let cr = CriterionResult {
            criterion: "test".to_string(),
            passed: true,
            actual_value: "1.0".to_string(),
            expected: "1.0".to_string(),
        };
        assert!(cr.passed);
    }
}
