//! Mission Runner - Backend-agnostic test execution
//!
//! This module provides infrastructure for running missions across different simulators.
//! All simulator-specific code is abstracted behind the `SimulatorBackend` trait.
//!
//! ## Architecture
//!
//! ```text
//! TestConfig (TOML)
//!       ↓
//! MissionRunner<B: SimulatorBackend>
//!       ↓
//! ┌─────────────────────────────────────────────┐
//! │  B = GazeboBackend  │  B = JMavSimBackend   │
//! │  (gz-plugin FFI)    │  (MAVLink HIL)        │
//! └─────────────────────────────────────────────┘
//! ```
//!
//! ## Multi-Vehicle Support
//!
//! Each vehicle runs in its own thread with a separate backend instance.
//! The `run_test_config` function orchestrates parallel execution.

use std::net::UdpSocket;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use crate::config::TestConfig;
use crate::mission::{
    Action, Criterion, CriterionResult, Mission, MissionResult, Phase, PhaseResult,
};
use crate::XilNetConfig;

use aviate_link::mavlink::protocol::{
    CommandLong, Heartbeat, SetAttitudeTarget, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{
    mav_cmd, parse_mavlink, serialize_mavlink, MavAutopilot, MavMessage, MavState, MavType,
};

// ============================================================================
// SimulatorBackend Trait
// ============================================================================

/// Backend-agnostic vehicle state
#[derive(Clone, Debug, Default)]
pub struct VehicleState {
    /// Position in NED frame [m]
    pub position: [f32; 3],
    /// Velocity in NED frame [m/s]
    pub velocity: [f32; 3],
    /// Orientation quaternion [w, x, y, z]
    pub orientation: [f32; 4],
    /// Angular velocity [rad/s]
    pub angular_velocity: [f32; 3],
    /// Simulation time in microseconds
    pub time_us: u64,
    /// Is state valid/available
    pub valid: bool,
}

/// Error type for simulator backend operations
#[derive(Debug)]
pub enum SimulatorError {
    /// Connection failed
    ConnectionFailed(String),
    /// Backend not available
    NotAvailable(String),
    /// Timeout waiting for simulator
    Timeout,
    /// IO error
    Io(std::io::Error),
}

impl std::fmt::Display for SimulatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ConnectionFailed(msg) => write!(f, "Connection failed: {}", msg),
            Self::NotAvailable(msg) => write!(f, "Not available: {}", msg),
            Self::Timeout => write!(f, "Timeout"),
            Self::Io(e) => write!(f, "IO error: {}", e),
        }
    }
}

impl std::error::Error for SimulatorError {}

impl From<std::io::Error> for SimulatorError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Backend-agnostic simulator interface
///
/// Each simulator backend (Gazebo, jMAVSim, etc.) implements this trait
/// to provide a unified interface for mission execution.
pub trait SimulatorBackend: Send {
    /// Backend name (for logging)
    fn name(&self) -> &str;

    /// Connect to the simulator for a specific vehicle instance
    fn connect(&mut self, instance: u8, timeout_ms: u64) -> Result<(), SimulatorError>;

    /// Check if connected to the simulator
    fn is_connected(&self) -> bool;

    /// Get current vehicle state (NED frame)
    fn get_vehicle_state(&self) -> Option<VehicleState>;

    /// Set motor speeds (normalized 0.0-1.0 or rad/s depending on backend)
    fn set_motor_speeds(&mut self, speeds: &[f64]) -> Result<(), SimulatorError>;

    /// Enable/disable lockstep mode
    fn set_lockstep(&mut self, enabled: bool);

    /// Get current simulation step number
    fn sim_step(&self) -> u64;

    /// Acknowledge a simulation step (for lockstep)
    fn ack_step(&mut self, step: u64);

    /// Get the instance ID this backend is connected to
    fn instance(&self) -> u8;
}

// ============================================================================
// MAVLink Client (generic, works with any backend)
// ============================================================================

/// MAVLink GCS client for sending commands to the FC
///
/// This is backend-agnostic - it just sends MAVLink over UDP.
pub struct MavClient {
    socket: UdpSocket,
    target_addr: std::net::SocketAddr,
    seq: u8,
    target_system: u8,
    target_component: u8,
}

impl MavClient {
    /// Create a new MAVLink client for a specific instance
    pub fn new(instance: u8) -> Result<Self, SimulatorError> {
        Self::new_with_net(instance, XilNetConfig::default())
    }

    /// Create a new MAVLink client with custom network configuration
    ///
    /// Binds to port 14550 (standard GCS port) to receive telemetry from the FC.
    /// The FC sends telemetry to this port.
    pub fn new_with_net(_instance: u8, _net: XilNetConfig) -> Result<Self, SimulatorError> {
        // GCS listens on port 14550 - FC broadcasts to this port
        let socket = UdpSocket::bind("127.0.0.1:14550")?;
        socket.set_nonblocking(true)?;

        // We don't know the FC's ephemeral port yet - it will be learned when we receive a packet
        let target_addr = std::net::SocketAddr::from(([127, 0, 0, 1], 0));

        Ok(Self {
            socket,
            target_addr,
            seq: 0,
            target_system: 1,
            target_component: 1,
        })
    }

    /// Send a MAVLink message
    fn send(&mut self, msg: &MavMessage) -> bool {
        let mut buf = [0u8; 300];
        if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            self.socket.send_to(&buf[..len], self.target_addr).is_ok()
        } else {
            false
        }
    }

    /// Try to receive a MAVLink message (non-blocking)
    /// Also learns the FC's address from the first received packet
    fn recv(&mut self) -> Option<MavMessage> {
        let mut buf = [0u8; 512];
        match self.socket.recv_from(&mut buf) {
            Ok((len, src)) => {
                // Learn FC address from first packet
                if self.target_addr.port() == 0 {
                    self.target_addr = src;
                }
                match parse_mavlink(&buf[..len]) {
                    Ok((msg, _sig, _consumed)) => Some(msg),
                    Err(_) => None,
                }
            }
            Err(_) => None,
        }
    }

    /// Send heartbeat
    pub fn send_heartbeat(&mut self) -> bool {
        let hb = Heartbeat {
            mav_type: MavType::Gcs as u8,
            autopilot: MavAutopilot::Generic as u8,
            base_mode: 0,
            custom_mode: 0,
            system_status: MavState::Active as u8,
            mavlink_version: 3,
        };
        self.send(&MavMessage::Heartbeat(hb))
    }

    /// Send arm command
    pub fn send_arm(&mut self) -> bool {
        let cmd = CommandLong {
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            target_system: self.target_system,
            target_component: self.target_component,
            confirmation: 0,
        };
        self.send(&MavMessage::CommandLong(cmd))
    }

    /// Send disarm command
    pub fn send_disarm(&mut self) -> bool {
        let cmd = CommandLong {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            target_system: self.target_system,
            target_component: self.target_component,
            confirmation: 0,
        };
        self.send(&MavMessage::CommandLong(cmd))
    }

    /// Send attitude target (quaternion + thrust)
    pub fn send_attitude_target(&mut self, q: [f32; 4], thrust: f32) -> bool {
        let tgt = SetAttitudeTarget {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            type_mask: 0x07,
            q,
            body_roll_rate: 0.0,
            body_pitch_rate: 0.0,
            body_yaw_rate: 0.0,
            thrust,
            thrust_body: [0.0, 0.0, 0.0],
        };
        self.send(&MavMessage::SetAttitudeTarget(tgt))
    }

    /// Send position target (NED frame)
    pub fn send_position_target(&mut self, x: f32, y: f32, z: f32, yaw: f32) -> bool {
        let tgt = SetPositionTargetLocalNed {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            coordinate_frame: 1, // MAV_FRAME_LOCAL_NED
            type_mask: 0x0DF8,
            x,
            y,
            z,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw,
            yaw_rate: 0.0,
        };
        self.send(&MavMessage::SetPositionTargetLocalNed(tgt))
    }

    /// Try to connect to FC (wait for heartbeat from FC broadcast)
    ///
    /// The FC broadcasts telemetry to port 14550 continuously.
    /// We just need to wait to receive a heartbeat.
    pub fn try_connect(&mut self) -> bool {
        // Wait for FC heartbeat (FC broadcasts continuously)
        for _ in 0..50 {
            if let Some(MavMessage::Heartbeat(_)) = self.recv() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        false
    }
}

// ============================================================================
// MissionRunner (backend-agnostic)
// ============================================================================

/// Mission runner state
///
/// Generic over the simulator backend. Each vehicle instance gets its own runner.
/// Requires FC to be running - does not support direct motor control bypass.
pub struct MissionRunner<B: SimulatorBackend> {
    backend: B,
    mav: MavClient,
    vehicle_id: String,
    last_step: u64,
    current_state: VehicleState,
    start_position: [f32; 3],
    armed: bool,
    max_altitude: f32,
}

impl<B: SimulatorBackend> MissionRunner<B> {
    /// Create a new mission runner with the given backend
    pub fn new(backend: B, vehicle_id: &str) -> Result<Self, SimulatorError> {
        let instance = backend.instance();
        let mav = MavClient::new(instance)?;

        Ok(Self {
            backend,
            mav,
            vehicle_id: vehicle_id.to_string(),
            last_step: 0,
            current_state: VehicleState::default(),
            start_position: [0.0; 3],
            armed: false,
            max_altitude: 0.0,
        })
    }

    /// Get the vehicle ID
    pub fn vehicle_id(&self) -> &str {
        &self.vehicle_id
    }

    /// Get the instance ID
    pub fn instance(&self) -> u8 {
        self.backend.instance()
    }

    /// Log a message with vehicle prefix
    fn log(&self, msg: &str) {
        println!("[{}:{}] {}", self.vehicle_id, self.backend.instance(), msg);
    }

    /// Run a complete mission
    pub fn run(&mut self, mission: &Mission) -> MissionResult {
        self.log(&format!("=== Mission: {} ===", mission.name));
        self.log(&format!("Description: {}", mission.description));
        self.log(&format!(
            "Lockstep: {}",
            if mission.lockstep { "YES" } else { "NO" }
        ));
        println!();

        // Connect to FC via MAVLink (required)
        if !self.mav.try_connect() {
            self.log("ERROR: FC not connected - mission runner requires FC");
            return MissionResult {
                mission_name: mission.name.clone(),
                passed: false,
                phases: vec![],
                total_duration: Duration::ZERO,
                max_altitude: 0.0,
            };
        }
        self.log("FC connected via MAVLink");

        // Enable lockstep if required
        if mission.lockstep {
            self.backend.set_lockstep(true);
            self.log("Lockstep enabled");
        }

        // Wait for simulation to be ready
        let timeout = Duration::from_secs(10);
        let start = Instant::now();
        let mut found_model = false;

        while start.elapsed() < timeout {
            self.last_step = self.backend.sim_step();
            if let Some(state) = self.backend.get_vehicle_state() {
                if state.valid && state.time_us > 0 {
                    self.start_position = state.position;
                    self.current_state = state;
                    found_model = true;
                    self.log(&format!(
                        "Model state ready (time={}us, step={})",
                        self.current_state.time_us, self.last_step
                    ));
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }

        if !found_model {
            self.log("WARNING: Model state not available");
        }

        let mission_start = Instant::now();
        let mut phase_results = Vec::new();
        let mut mission_passed = true;

        // Execute each phase
        for (i, phase) in mission.phases.iter().enumerate() {
            self.log(&format!(
                "[Phase {}/{}] {}",
                i + 1,
                mission.phases.len(),
                phase.name
            ));

            let result = self.run_phase(phase);

            if result.passed {
                self.log(&format!("  PASSED (alt: {:.2}m)", result.max_altitude));
            } else {
                self.log("  FAILED");
                for cr in &result.criteria_results {
                    if !cr.passed {
                        self.log(&format!(
                            "    - {}: expected {}, got {}",
                            cr.criterion, cr.expected, cr.actual_value
                        ));
                    }
                }
                mission_passed = false;
            }

            phase_results.push(result);
        }

        // Disable lockstep
        if mission.lockstep {
            self.backend.set_lockstep(false);
        }

        let total_duration = mission_start.elapsed();

        println!();
        self.log(&format!(
            "=== Mission {} ===",
            if mission_passed { "PASSED" } else { "FAILED" }
        ));
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

        while phase_start.elapsed() < phase.duration {
            let current_step = self.backend.sim_step();

            if current_step > self.last_step {
                // Read state
                if let Some(state) = self.backend.get_vehicle_state() {
                    self.current_state = state;
                    let alt = -self.current_state.position[2]; // NED: -z is up
                    if alt > phase_max_altitude {
                        phase_max_altitude = alt;
                    }
                    if phase_max_altitude > self.max_altitude {
                        self.max_altitude = phase_max_altitude;
                    }
                }

                // Execute action
                self.execute_action(&phase.action);

                // Acknowledge step
                self.backend.ack_step(current_step);
                self.last_step = current_step;
            }

            std::thread::sleep(Duration::from_micros(100));
        }

        // Verify criteria
        let criteria_results: Vec<CriterionResult> = phase
            .verify
            .iter()
            .map(|c| self.verify_criterion(c, phase_max_altitude))
            .collect();

        let passed = criteria_results.iter().all(|r| r.passed);

        PhaseResult {
            name: phase.name.clone(),
            passed,
            duration_actual: phase_start.elapsed(),
            max_altitude: phase_max_altitude,
            final_position: self.current_state.position,
            criteria_results,
        }
    }

    /// Execute an action via MAVLink
    fn execute_action(&mut self, action: &Action) {
        match action {
            Action::Wait => {
                self.mav.send_heartbeat();
            }
            Action::Arm => {
                if !self.armed {
                    self.mav.send_arm();
                    self.armed = true;
                    self.log("ARM via MAVLink");
                }
            }
            Action::Disarm => {
                if self.armed {
                    self.mav.send_attitude_target([1.0, 0.0, 0.0, 0.0], 0.0);
                    self.mav.send_disarm();
                    self.armed = false;
                    self.log("DISARM via MAVLink");
                }
            }
            Action::Thrust(t) => {
                if self.armed {
                    self.mav.send_attitude_target([1.0, 0.0, 0.0, 0.0], *t);
                }
            }
            Action::AttitudeTarget { q, thrust } => {
                if self.armed {
                    self.mav.send_attitude_target(*q, *thrust);
                }
            }
            Action::GoTo { position, heading } => {
                if self.armed {
                    self.mav
                        .send_position_target(position[0], position[1], position[2], *heading);
                }
            }
            Action::InjectFault { sensor, fault } => {
                self.log(&format!(
                    "INJECT_FAULT {:?} {:?} (not yet implemented)",
                    sensor, fault
                ));
            }
            Action::ClearFaults => {
                self.log("CLEAR_FAULTS (not yet implemented)");
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
            Criterion::MinAltitude(min) => CriterionResult {
                criterion: "min_altitude".to_string(),
                passed: phase_max_alt >= *min,
                actual_value: format!("{:.2}m", phase_max_alt),
                expected: format!(">= {:.2}m", min),
            },
            Criterion::MaxAltitude(max) => {
                let alt = -self.current_state.position[2];
                CriterionResult {
                    criterion: "max_altitude".to_string(),
                    passed: alt <= *max,
                    actual_value: format!("{:.2}m", alt),
                    expected: format!("<= {:.2}m", max),
                }
            }
            Criterion::AltitudeHold { target, tolerance } => {
                let alt = -self.current_state.position[2];
                let error = (alt - target).abs();
                CriterionResult {
                    criterion: "altitude_hold".to_string(),
                    passed: error <= *tolerance,
                    actual_value: format!("{:.2}m (error: {:.2}m)", alt, error),
                    expected: format!("{:.2}m +/- {:.2}m", target, tolerance),
                }
            }
            Criterion::PositionHold { target, tolerance } => {
                let dx = self.current_state.position[0] - target[0];
                let dy = self.current_state.position[1] - target[1];
                let dz = self.current_state.position[2] - target[2];
                let error = (dx * dx + dy * dy + dz * dz).sqrt();
                CriterionResult {
                    criterion: "position_hold".to_string(),
                    passed: error <= *tolerance,
                    actual_value: format!("error: {:.2}m", error),
                    expected: format!("<= {:.2}m", tolerance),
                }
            }
            Criterion::MaxDrift(max) => {
                let dx = self.current_state.position[0] - self.start_position[0];
                let dy = self.current_state.position[1] - self.start_position[1];
                let drift = (dx * dx + dy * dy).sqrt();
                CriterionResult {
                    criterion: "max_drift".to_string(),
                    passed: drift <= *max,
                    actual_value: format!("{:.2}m", drift),
                    expected: format!("<= {:.2}m", max),
                }
            }
            Criterion::SensorDataReceived => CriterionResult {
                criterion: "sensor_data".to_string(),
                passed: self.last_step > 0,
                actual_value: format!("{} steps", self.last_step),
                expected: "> 0 steps".to_string(),
            },
        }
    }
}

// ============================================================================
// Multi-Vehicle Execution
// ============================================================================

/// Result from running a complete test config
#[derive(Debug)]
pub struct TestResult {
    /// Test name
    pub name: String,
    /// Per-vehicle mission results
    pub vehicle_results: Vec<MissionResult>,
    /// Overall pass/fail
    pub passed: bool,
    /// Total test duration
    pub duration: Duration,
}

/// Run a test configuration with multiple vehicles in parallel
///
/// The `backend_factory` creates a new backend for each vehicle instance.
/// Vehicles run their missions concurrently in separate threads.
pub fn run_test_config<B, F>(config: &TestConfig, backend_factory: F) -> TestResult
where
    B: SimulatorBackend + 'static,
    F: Fn(u8) -> Result<B, SimulatorError> + Send + Sync + 'static,
{
    let start = Instant::now();
    let factory = Arc::new(backend_factory);

    // Spawn a thread for each vehicle
    let handles: Vec<_> = config
        .vehicles
        .iter()
        .map(|vehicle| {
            let vehicle_id = vehicle.id.clone();
            let instance = vehicle.instance;
            let mission = vehicle.mission.clone();
            let factory = Arc::clone(&factory);

            thread::spawn(move || -> MissionResult {
                match factory(instance) {
                    Ok(backend) => match MissionRunner::new(backend, &vehicle_id) {
                        Ok(mut runner) => runner.run(&mission),
                        Err(e) => {
                            eprintln!(
                                "[{}:{}] Failed to create runner: {}",
                                vehicle_id, instance, e
                            );
                            MissionResult {
                                mission_name: mission.name.clone(),
                                passed: false,
                                phases: vec![],
                                total_duration: Duration::ZERO,
                                max_altitude: 0.0,
                            }
                        }
                    },
                    Err(e) => {
                        eprintln!(
                            "[{}:{}] Failed to create backend: {}",
                            vehicle_id, instance, e
                        );
                        MissionResult {
                            mission_name: mission.name.clone(),
                            passed: false,
                            phases: vec![],
                            total_duration: Duration::ZERO,
                            max_altitude: 0.0,
                        }
                    }
                }
            })
        })
        .collect();

    // Collect results
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

    // Check global verification criteria
    let all_passed = vehicle_results.iter().all(|r| r.passed);

    // TODO: Check global criteria like min_separation
    // if let Some(ref global) = config.global_verification {
    //     if let Some(min_sep) = global.min_separation {
    //         // Check positions...
    //     }
    // }

    TestResult {
        name: config.name.clone(),
        vehicle_results,
        passed: all_passed,
        duration: start.elapsed(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vehicle_state_default() {
        let state = VehicleState::default();
        assert!(!state.valid);
        assert_eq!(state.time_us, 0);
    }

    #[test]
    fn test_simulator_error_display() {
        let err = SimulatorError::ConnectionFailed("test".to_string());
        assert!(err.to_string().contains("test"));
    }
}
