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

use log::error;

use crate::config::TestConfig;
use crate::mission::{
    Action, Criterion, CriterionResult, Mission, MissionResult, Phase, PhaseResult,
};
use crate::XilNetConfig;

use aviate_link::mavlink::protocol::{
    CommandLong, Heartbeat, MavHeader, SetAttitudeTarget, SetPositionTargetLocalNed,
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
    /// Position in NED frame \[m\]
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
    /// Binds to ephemeral port and connects to target instance GCS port.
    pub fn new_with_net(instance: u8, net: XilNetConfig) -> Result<Self, SimulatorError> {
        // Bind to ephemeral port to avoid conflicts in multi-vehicle tests
        let socket = UdpSocket::bind("127.0.0.1:0")?;
        socket.set_nonblocking(true)?;

        // Target the dedicated GCS port for this instance (Slot 0 / SensorIn)
        let gcs_port = net.port(instance as u16, crate::PortSlot::SensorIn);
        let target_addr = std::net::SocketAddr::from(([127, 0, 0, 1], gcs_port));
        // info!("MavClient targeting {}", target_addr);

        Ok(Self {
            socket,
            target_addr,
            seq: 0,
            target_system: instance + 1, // System ID = Instance + 1
            target_component: 1,
        })
    }

    /// Send a MAVLink message
    fn send(&mut self, msg: &MavMessage) -> bool {
        let mut buf = [0u8; 300];
        // Send as GCS (SysID 255, CompID 190)
        if let Some(len) = serialize_mavlink(msg, self.seq, 255, 190, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            self.socket.send_to(&buf[..len], self.target_addr).is_ok()
        } else {
            false
        }
    }

    /// Try to receive a MAVLink message (non-blocking)
    /// Also learns the FC's address from the first received packet
    fn recv(&mut self) -> Option<(MavHeader, MavMessage)> {
        let mut buf = [0u8; 512];
        match self.socket.recv_from(&mut buf) {
            Ok((len, src)) => {
                // Learn FC address from first packet
                if self.target_addr.port() == 0 {
                    self.target_addr = src;
                }
                match parse_mavlink(&buf[..len]) {
                    Ok((msg, _sig, _consumed)) => {
                        // Reconstruct header (parse_mavlink checks validity but drops header)
                        // Safety: parse_mavlink ensured buffer is valid MAVLink v2
                        let header = MavHeader {
                            payload_len: buf[1],
                            incompat_flags: buf[2],
                            compat_flags: buf[3],
                            seq: buf[4],
                            sysid: buf[5],
                            compid: buf[6],
                            msgid: (buf[7] as u32)
                                | ((buf[8] as u32) << 8)
                                | ((buf[9] as u32) << 16),
                        };
                        Some((header, msg))
                    }
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
    /// The FC broadcasts telemetry to port 14550.
    /// We send heartbeats to register ourselves with the FC/Router.
    /// Returns true if we receive a heartbeat from our TARGET SYSTEM.
    pub fn try_connect(&mut self) -> bool {
        // Send HB to register, then check for response
        for _ in 0..100 {
            // Increased retry count (10s total)
            self.send_heartbeat();

            while let Some((header, MavMessage::Heartbeat(_))) = self.recv() {
                if header.sysid == self.target_system {
                    return true;
                }
                // Ignore heartbeats from other systems (e.g. other vehicles)
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        false
    }
}

// ============================================================================
// MissionRunner (backend-agnostic)
// ============================================================================

/// A single per-step entry in the phase trace.
///
/// Carries ground-truth position (NED, metres) and attitude
/// quaternion (`[w, x, y, z]`, body→world) sampled at each
/// `sim_step` advance. Criteria walk this trace to enforce
/// throughout-phase and trace-walking constraints.
#[derive(Debug, Clone, Copy)]
pub struct TraceSample {
    pub elapsed: f32,
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub attitude: [f32; 4],
    pub angular_velocity: [f32; 3],
}

/// Mission runner state
///
/// Generic over the simulator backend. Each vehicle instance gets its own runner.
/// Requires FC to be running - does not support direct motor control bypass.
pub struct MissionRunner<B: SimulatorBackend> {
    backend: B,
    mav: MavClient,
    /// Per-instance fault command UDP client. Constructed lazily; we
    /// only need it when a mission contains `Action::InjectFault` or
    /// `Action::ClearFaults`. Once constructed it owns its own
    /// ephemeral receive port for ACKs.
    fault_client: Option<crate::fault_protocol::FaultClient>,
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
            fault_client: None,
            vehicle_id: vehicle_id.to_string(),
            last_step: 0,
            current_state: VehicleState::default(),
            start_position: [0.0; 3],
            armed: false,
            max_altitude: 0.0,
        })
    }

    /// Lazily create or return the `FaultClient` for this instance.
    /// Returns `None` if socket binding fails — the caller logs the
    /// outcome and continues so a missing FaultController on the FC
    /// side does not abort the entire mission run.
    fn fault_client_mut(&mut self) -> Option<&mut crate::fault_protocol::FaultClient> {
        if self.fault_client.is_none() {
            let cfg = crate::XilConfig::for_instance(self.backend.instance());
            match crate::fault_protocol::FaultClient::new(&cfg) {
                Ok(c) => self.fault_client = Some(c),
                Err(e) => {
                    self.log(&format!(
                        "WARN: FaultClient bind failed for instance {}: {:?}",
                        self.backend.instance(),
                        e
                    ));
                    return None;
                }
            }
        }
        self.fault_client.as_mut()
    }

    /// Get the vehicle ID
    pub fn vehicle_id(&self) -> &str {
        &self.vehicle_id
    }

    /// Get the instance ID
    pub fn instance(&self) -> u8 {
        self.backend.instance()
    }

    /// Log an info message with vehicle target for env_logger formatting
    fn log(&self, msg: &str) {
        let target = format!("fc{}", self.backend.instance());
        log::info!(target: &target, "{}", msg);
    }

    /// Log an error message with vehicle target for env_logger formatting
    fn log_error(&self, msg: &str) {
        let target = format!("fc{}", self.backend.instance());
        log::error!(target: &target, "{}", msg);
    }

    /// Run a complete mission
    pub fn run(&mut self, mission: &Mission) -> MissionResult {
        self.log(&format!(
            "==> {} | lockstep={} <==",
            mission.name,
            if mission.lockstep { "yes" } else { "no" }
        ));

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
        let total_phases = mission.phases.len();
        for (i, phase) in mission.phases.iter().enumerate() {
            let phase_num = i + 1;
            self.log(&format!(
                "[Phase {}/{}] {}",
                phase_num, total_phases, phase.name
            ));

            let result = self.run_phase(phase);

            if result.passed {
                self.log(&format!(
                    "[Phase {}/{}] {} PASSED (alt: {:.2}m)",
                    phase_num, total_phases, phase.name, result.max_altitude
                ));
            } else {
                self.log_error(&format!(
                    "[Phase {}/{}] {} FAILED",
                    phase_num, total_phases, phase.name
                ));
                for cr in &result.criteria_results {
                    if !cr.passed {
                        self.log_error(&format!(
                            "  - {}: expected {}, got {}",
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

        // Dump per-step trace to CSV for post-mortem inspection.
        // Aircraft-agnostic columns (position, velocity, attitude
        // quaternion, body-frame angular rate, derived Euler) so the
        // same writer works for multirotor, fixed-wing, or VTOL —
        // the CSV reader can plot whichever axes a given vehicle
        // class cares about. One file per mission, overwritten on
        // each run, lives next to the world SDF in the OS temp
        // directory.
        let csv_path = std::env::temp_dir().join(format!(
            "aviate_trace_{}.csv",
            mission.name.replace(['/', ' '], "_")
        ));
        if let Err(e) = write_trace_csv(&csv_path, &mission.name, &phase_results) {
            self.log_error(&format!(
                "Failed to write trace CSV ({}): {}",
                csv_path.display(),
                e
            ));
        } else {
            self.log(&format!("Trace CSV: {}", csv_path.display()));
        }

        if mission_passed {
            self.log(&format!(
                "==> PASSED | duration={:.2}s max_alt={:.2}m <==",
                total_duration.as_secs_f32(),
                self.max_altitude
            ));
        } else {
            self.log_error(&format!(
                "==> FAILED | duration={:.2}s max_alt={:.2}m <==",
                total_duration.as_secs_f32(),
                self.max_altitude
            ));
        }

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
        // Per-step trace: each sample carries `(elapsed_s,
        // position_ned, attitude_quat[w,x,y,z])`. The third field
        // is what makes attitude-aware criteria possible; without
        // it a tumbling vehicle could still pass a position-only
        // check.
        let mut trace: Vec<TraceSample> = Vec::new();

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
                    let elapsed = phase_start.elapsed().as_secs_f32();
                    trace.push(TraceSample {
                        elapsed,
                        position: self.current_state.position,
                        velocity: self.current_state.velocity,
                        attitude: self.current_state.orientation,
                        angular_velocity: self.current_state.angular_velocity,
                    });
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
            .map(|c| self.verify_criterion(c, phase_max_altitude, &trace))
            .collect();

        let passed = criteria_results.iter().all(|r| r.passed);

        PhaseResult {
            name: phase.name.clone(),
            passed,
            duration_actual: phase_start.elapsed(),
            max_altitude: phase_max_altitude,
            final_position: self.current_state.position,
            criteria_results,
            trace,
            action_tag: format!("{:?}", phase.action),
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
                let target = *sensor;
                let spec = *fault;
                if let Some(client) = self.fault_client_mut() {
                    match client.inject(target, spec) {
                        Ok(ack) => self.log(&format!(
                            "INJECT_FAULT {:?} {:?} ack={:?}",
                            target, spec, ack.status
                        )),
                        Err(e) => self.log_error(&format!(
                            "INJECT_FAULT {:?} {:?} failed: {:?}",
                            target, spec, e
                        )),
                    }
                } else {
                    self.log_error(&format!(
                        "INJECT_FAULT {:?} {:?} skipped: FaultClient unavailable",
                        target, spec
                    ));
                }
            }
            Action::ClearFaults => {
                if let Some(client) = self.fault_client_mut() {
                    match client.clear_all() {
                        Ok(ack) => self.log(&format!("CLEAR_FAULTS ack={:?}", ack.status)),
                        Err(e) => self.log_error(&format!("CLEAR_FAULTS failed: {:?}", e)),
                    }
                } else {
                    self.log_error("CLEAR_FAULTS skipped: FaultClient unavailable");
                }
            }
        }
    }

    /// Verify a criterion
    fn verify_criterion(
        &self,
        criterion: &Criterion,
        phase_max_alt: f32,
        trace: &[TraceSample],
    ) -> CriterionResult {
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
            Criterion::ReachedWaypoint { target, tolerance } => {
                let min_err = trace
                    .iter()
                    .map(|s| {
                        let dx = s.position[0] - target[0];
                        let dy = s.position[1] - target[1];
                        let dz = s.position[2] - target[2];
                        (dx * dx + dy * dy + dz * dz).sqrt()
                    })
                    .fold(f32::INFINITY, f32::min);
                CriterionResult {
                    criterion: "reached_waypoint".to_string(),
                    passed: min_err <= *tolerance,
                    actual_value: format!("min error: {:.2}m", min_err),
                    expected: format!("<= {:.2}m at any point", tolerance),
                }
            }
            Criterion::StableHover {
                altitude,
                tolerance,
                hold_secs,
            } => {
                // Sliding-window check: find the longest contiguous
                // run of samples whose altitude (positive up) is in
                // band. Pass iff that run is at least `hold_secs`.
                let in_band = |z_ned: f32| {
                    let alt = -z_ned;
                    (alt - altitude).abs() <= *tolerance
                };
                let mut best_run = 0.0_f32;
                let mut run_start: Option<f32> = None;
                for s in trace {
                    if in_band(s.position[2]) {
                        if run_start.is_none() {
                            run_start = Some(s.elapsed);
                        }
                        if let Some(t0) = run_start {
                            best_run = best_run.max(s.elapsed - t0);
                        }
                    } else {
                        run_start = None;
                    }
                }
                CriterionResult {
                    criterion: "stable_hover".to_string(),
                    passed: best_run >= *hold_secs,
                    actual_value: format!("best continuous run: {:.2}s", best_run),
                    expected: format!(
                        ">= {:.2}s in [{:.2},{:.2}]m band",
                        hold_secs,
                        altitude - tolerance,
                        altitude + tolerance,
                    ),
                }
            }
            Criterion::StationKeeping {
                center_ned,
                xy_tolerance,
                z_tolerance,
            } => {
                // Throughout-phase: every sample must be inside the
                // box. We report the WORST sample (the one that
                // pushed furthest outside) — that's what a debugger
                // wants to see when the criterion fails.
                let mut worst_xy = 0.0_f32;
                let mut worst_z = 0.0_f32;
                let mut worst_t = 0.0_f32;
                for s in trace {
                    let dx = s.position[0] - center_ned[0];
                    let dy = s.position[1] - center_ned[1];
                    let dz = s.position[2] - center_ned[2];
                    let xy = (dx * dx + dy * dy).sqrt();
                    let z = dz.abs();
                    if xy > worst_xy {
                        worst_xy = xy;
                        worst_t = s.elapsed;
                    }
                    if z > worst_z {
                        worst_z = z;
                    }
                }
                let passed =
                    worst_xy <= *xy_tolerance && worst_z <= *z_tolerance && !trace.is_empty();
                CriterionResult {
                    criterion: "station_keeping".to_string(),
                    passed,
                    actual_value: format!(
                        "worst xy={:.2}m z={:.2}m at t={:.2}s ({} samples)",
                        worst_xy,
                        worst_z,
                        worst_t,
                        trace.len(),
                    ),
                    expected: format!(
                        "every sample within xy<={:.2}m, z<={:.2}m of {:?}",
                        xy_tolerance, z_tolerance, center_ned
                    ),
                }
            }
            Criterion::MaxExcursion {
                center_ned,
                xy_max,
                z_max,
            } => {
                let mut worst_xy = 0.0_f32;
                let mut worst_z = 0.0_f32;
                for s in trace {
                    let dx = s.position[0] - center_ned[0];
                    let dy = s.position[1] - center_ned[1];
                    let dz = s.position[2] - center_ned[2];
                    worst_xy = worst_xy.max((dx * dx + dy * dy).sqrt());
                    worst_z = worst_z.max(dz.abs());
                }
                let passed = worst_xy <= *xy_max && worst_z <= *z_max;
                CriterionResult {
                    criterion: "max_excursion".to_string(),
                    passed,
                    actual_value: format!("xy={:.2}m z={:.2}m", worst_xy, worst_z),
                    expected: format!("xy<={:.2}m, z<={:.2}m", xy_max, z_max),
                }
            }
            Criterion::TrajectoryTracking {
                waypoints,
                tolerance,
                max_time_s,
            } => {
                // Walk the trace; advance to the next waypoint each
                // time we land inside the tolerance ball. The
                // criterion passes only if every waypoint is
                // visited in order before `max_time_s` elapses.
                let mut idx = 0;
                let mut visit_time = None;
                for s in trace {
                    if idx >= waypoints.len() {
                        break;
                    }
                    if s.elapsed > *max_time_s {
                        break;
                    }
                    let w = &waypoints[idx];
                    let dx = s.position[0] - w[0];
                    let dy = s.position[1] - w[1];
                    let dz = s.position[2] - w[2];
                    let err = (dx * dx + dy * dy + dz * dz).sqrt();
                    if err <= *tolerance {
                        idx += 1;
                        visit_time = Some(s.elapsed);
                    }
                }
                let passed = idx == waypoints.len();
                CriterionResult {
                    criterion: "trajectory_tracking".to_string(),
                    passed,
                    actual_value: format!(
                        "visited {}/{} at last t={:?}",
                        idx,
                        waypoints.len(),
                        visit_time
                    ),
                    expected: format!(
                        "every waypoint reached within {:.2}m, total time <= {:.2}s",
                        tolerance, max_time_s
                    ),
                }
            }
            Criterion::ReturnedNear {
                target_ned,
                tolerance,
            } => {
                let dx = self.current_state.position[0] - target_ned[0];
                let dy = self.current_state.position[1] - target_ned[1];
                let dz = self.current_state.position[2] - target_ned[2];
                let err = (dx * dx + dy * dy + dz * dz).sqrt();
                CriterionResult {
                    criterion: "returned_near".to_string(),
                    passed: err <= *tolerance,
                    actual_value: format!("end-of-phase error: {:.2}m", err),
                    expected: format!("<= {:.2}m of {:?}", tolerance, target_ned),
                }
            }
            Criterion::AttitudeBounded { roll_pitch_max_deg } => {
                let limit_rad = roll_pitch_max_deg.to_radians();
                let mut worst_roll_rad = 0.0_f32;
                let mut worst_pitch_rad = 0.0_f32;
                let mut worst_t = 0.0_f32;
                for s in trace {
                    let (roll, pitch, _) = quat_to_rpy(s.attitude);
                    if roll.abs() > worst_roll_rad.abs() {
                        worst_roll_rad = roll;
                        worst_t = s.elapsed;
                    }
                    if pitch.abs() > worst_pitch_rad.abs() {
                        worst_pitch_rad = pitch;
                    }
                }
                let passed = worst_roll_rad.abs() <= limit_rad
                    && worst_pitch_rad.abs() <= limit_rad
                    && !trace.is_empty();
                CriterionResult {
                    criterion: "attitude_bounded".to_string(),
                    passed,
                    actual_value: format!(
                        "worst roll={:.1}° pitch={:.1}° at t={:.2}s",
                        worst_roll_rad.to_degrees(),
                        worst_pitch_rad.to_degrees(),
                        worst_t,
                    ),
                    expected: format!("|roll|, |pitch| <= {:.1}°", roll_pitch_max_deg),
                }
            }
            Criterion::SensorDataReceived => CriterionResult {
                criterion: "sensor_data".to_string(),
                passed: self.last_step > 0,
                actual_value: format!("{} steps", self.last_step),
                expected: "> 0 steps".to_string(),
            },
            Criterion::TouchdownVelocity {
                max_descent_mps,
                ground_tolerance,
            } => {
                // Find the index of the first sample within
                // `ground_tolerance` of the ground. gz's
                // `WorldLinearVelocity` component returns zero on
                // macOS — same gap that forced the quaternion-
                // derived gyro in the synth path — so we derive
                // the touchdown vertical speed from a finite
                // difference over the position samples just
                // before contact instead of reading the velocity
                // field.
                let idx = trace
                    .iter()
                    .position(|s| s.position[2] >= -ground_tolerance);
                match idx {
                    Some(i) if i > 0 => {
                        // Use the highest-altitude sample within
                        // the previous 100 ms as the back step so a
                        // single-sample contact-damping spike does
                        // not hide the real approach speed.
                        let s_now = &trace[i];
                        let lookback = trace[..i]
                            .iter()
                            .rev()
                            .take_while(|s| s_now.elapsed - s.elapsed < 0.1)
                            .min_by(|a, b| a.position[2].partial_cmp(&b.position[2]).unwrap())
                            .unwrap_or(&trace[i - 1]);
                        let dt = (s_now.elapsed - lookback.elapsed).max(1e-3);
                        let v_down = (s_now.position[2] - lookback.position[2]) / dt;
                        CriterionResult {
                            criterion: "touchdown_velocity".to_string(),
                            passed: v_down <= *max_descent_mps,
                            actual_value: format!(
                                "v_down={:.2} m/s at t={:.2}s (alt={:.2}m, derived from Δz/Δt over {:.0}ms)",
                                v_down,
                                s_now.elapsed,
                                -s_now.position[2],
                                dt * 1000.0,
                            ),
                            expected: format!(
                                "v_down ≤ {:.2} m/s within {:.2}m of ground",
                                max_descent_mps, ground_tolerance,
                            ),
                        }
                    }
                    Some(_) => CriterionResult {
                        criterion: "touchdown_velocity".to_string(),
                        passed: false,
                        actual_value: "vehicle already on ground at phase start".to_string(),
                        expected: format!(
                            "vehicle reaches within {:.2}m of ground during the phase",
                            ground_tolerance
                        ),
                    },
                    None => CriterionResult {
                        criterion: "touchdown_velocity".to_string(),
                        passed: false,
                        actual_value: format!(
                            "no touchdown sample (min alt {:.2}m)",
                            trace
                                .iter()
                                .map(|s| -s.position[2])
                                .fold(f32::INFINITY, f32::min)
                        ),
                        expected: format!(
                            "vehicle reaches within {:.2}m of ground",
                            ground_tolerance
                        ),
                    },
                }
            }
        }
    }
}

/// Body-axis roll, pitch, yaw (radians) from a unit quaternion
/// `[w, x, y, z]` representing body→world (NED+FRD) rotation.
/// Standard Z-Y-X (yaw-pitch-roll) extraction.
fn quat_to_rpy(q: [f32; 4]) -> (f32, f32, f32) {
    let [w, x, y, z] = q;
    // roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);
    // pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f32::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };
    // yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);
    (roll, pitch, yaw)
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
                            error!(
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
                        error!(
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

/// Write the per-step flight trace to a CSV.
///
/// Columns are aircraft-agnostic so the same file format works for
/// multirotor / fixed-wing / VTOL: `t,phase,action` then NED
/// position, NED velocity, body→world attitude quaternion + derived
/// Euler angles, body-frame angular rate. Each row is one
/// gz-physics step (~1 kHz). The file is overwritten per mission.
fn write_trace_csv(
    path: &std::path::Path,
    mission_name: &str,
    phases: &[PhaseResult],
) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::fs::File::create(path)?;
    writeln!(
        f,
        "# mission={}\nt_s,phase,action,x_ned_m,y_ned_m,z_ned_m,vx_mps,vy_mps,vz_mps,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,p_radps,q_radps,r_radps"
    , mission_name)?;
    let mut t_offset = 0.0f32;
    for phase in phases {
        // CSV `action` column is the TOML action with whitespace
        // collapsed so the column doesn't break on commas inside
        // the debug-formatted enum.
        let action = phase
            .action_tag
            .replace([',', '\n', '\r'], ";")
            .replace("  ", " ");
        for s in &phase.trace {
            let (roll, pitch, yaw) = quat_to_rpy(s.attitude);
            writeln!(
                f,
                "{:.4},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.6},{:.6},{:.6},{:.6},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4}",
                t_offset + s.elapsed,
                phase.name,
                action,
                s.position[0], s.position[1], s.position[2],
                s.velocity[0], s.velocity[1], s.velocity[2],
                s.attitude[0], s.attitude[1], s.attitude[2], s.attitude[3],
                roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees(),
                s.angular_velocity[0], s.angular_velocity[1], s.angular_velocity[2],
            )?;
        }
        t_offset += phase.duration_actual.as_secs_f32();
    }
    Ok(())
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
