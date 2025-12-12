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
//!
//! ## MAVLink Integration
//!
//! Commands flow through the full Aviate control stack:
//! ```text
//! MissionRunner → MAVLink UDP → Aviate SITL (main.rs) → Controller → Mixer
//!     → GzPluginBridge (shared memory) → Gazebo Motors
//! ```

#[cfg(feature = "gz-plugin")]
use std::net::UdpSocket;
#[cfg(feature = "gz-plugin")]
use std::time::{Duration, Instant};

#[cfg(feature = "gz-plugin")]
use aviate_hal_xil::{
    Action, Criterion, CriterionResult, Mission, MissionResult, Phase, PhaseResult,
};

#[cfg(feature = "gz-plugin")]
use aviate_backend_gz::{enu_to_ned_f32, AviateModelState, GzPluginBridge};

#[cfg(feature = "gz-plugin")]
use aviate_hal_xil::{PortSlot, XilNetConfig};

#[cfg(feature = "gz-plugin")]
use aviate_link::mavlink::{
    mav_cmd, parse_mavlink, serialize_mavlink, MavAutopilot, MavMessage, MavState, MavType,
};

#[cfg(feature = "gz-plugin")]
use aviate_link::mavlink::protocol::{
    CommandLong, Heartbeat, SetAttitudeTarget, SetPositionTargetLocalNed,
};

/// MAVLink GCS client for sending commands to the FC
///
/// Sends commands via UDP to the Aviate SITL autopilot (main.rs).
/// The FC then processes commands through the control loop and mixer.
#[cfg(feature = "gz-plugin")]
pub struct MavClient {
    socket: UdpSocket,
    target_addr: std::net::SocketAddr,
    seq: u8,
    target_system: u8,
    target_component: u8,
}

#[cfg(feature = "gz-plugin")]
impl MavClient {
    /// Create a new MAVLink client connected to the FC
    pub fn new(instance: u8) -> Result<Self, String> {
        Self::new_with_net(instance, XilNetConfig::default())
    }

    /// Create a new MAVLink client with custom network configuration
    pub fn new_with_net(instance: u8, net: XilNetConfig) -> Result<Self, String> {
        // Bind to ephemeral port (OS assigns)
        let socket = UdpSocket::bind("0.0.0.0:0")
            .map_err(|e| format!("Failed to bind UDP socket: {}", e))?;
        socket
            .set_nonblocking(true)
            .map_err(|e| format!("Failed to set nonblocking: {}", e))?;

        // FC address: sensor port is where FC listens for HIL data
        let port = net.port(instance as u16, PortSlot::SensorIn);
        let target_addr = std::net::SocketAddr::from(([127, 0, 0, 1], port));

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
    fn recv(&mut self) -> Option<MavMessage> {
        let mut buf = [0u8; 512];
        match self.socket.recv_from(&mut buf) {
            Ok((len, _src)) => match parse_mavlink(&buf[..len]) {
                Ok((msg, _sig, _consumed)) => Some(msg),
                Err(_) => None,
            },
            Err(_) => None,
        }
    }

    /// Send heartbeat (Level 1)
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

    /// Send arm command (Level 2)
    pub fn send_arm(&mut self) -> bool {
        let cmd = CommandLong {
            param1: 1.0, // 1 = arm
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

    /// Send disarm command (Level 2)
    pub fn send_disarm(&mut self) -> bool {
        let cmd = CommandLong {
            param1: 0.0, // 0 = disarm
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

    /// Send attitude target (Level 3-4)
    /// q: quaternion [w, x, y, z], thrust: 0.0 to 1.0
    pub fn send_attitude_target(&mut self, q: [f32; 4], thrust: f32) -> bool {
        let tgt = SetAttitudeTarget {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            type_mask: 0x07, // Ignore body rates, use attitude + thrust
            q,
            body_roll_rate: 0.0,
            body_pitch_rate: 0.0,
            body_yaw_rate: 0.0,
            thrust,
            thrust_body: [0.0, 0.0, 0.0],
        };
        self.send(&MavMessage::SetAttitudeTarget(tgt))
    }

    /// Send position target (Level 5)
    pub fn send_position_target(&mut self, x: f32, y: f32, z: f32, yaw: f32) -> bool {
        let tgt = SetPositionTargetLocalNed {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            coordinate_frame: 1, // MAV_FRAME_LOCAL_NED
            type_mask: 0x0DF8,   // Position + yaw only (ignore velocity, accel, yaw_rate)
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

    /// Wait for COMMAND_ACK
    pub fn wait_ack(&mut self, timeout_ms: u64) -> Option<u8> {
        let start = Instant::now();
        while start.elapsed().as_millis() < timeout_ms as u128 {
            if let Some(MavMessage::CommandAck(ack)) = self.recv() {
                return Some(ack.result);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        None
    }
}

/// Mission runner state
///
/// Each MissionRunner connects to a specific vehicle instance via shared memory.
/// Instance 0 uses /aviate_gz_bridge, instance N uses /aviate_gz_bridge_N.
///
/// ## Control Modes
///
/// - **MAVLink mode** (when FC is running): Commands flow through the full stack:
///   `MissionRunner → MAVLink → FC → Controller → Mixer → GzPluginBridge → Gazebo`
///
/// - **Direct mode** (fallback): Direct motor control via shared memory:
///   `MissionRunner → GzPluginBridge → Gazebo` (bypasses FC, for testing only)
#[cfg(feature = "gz-plugin")]
pub struct MissionRunner {
    bridge: GzPluginBridge,
    mav: MavClient,
    instance: u8,
    vehicle_id: String,
    last_step: u64,
    current_position: [f32; 3],
    current_velocity: [f32; 3],
    start_position: [f32; 3],
    armed: bool,
    max_altitude: f32,
    /// True if FC is responding to MAVLink commands
    fc_connected: bool,
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

        let mav = MavClient::new(instance)?;

        Ok(Self {
            bridge,
            mav,
            instance,
            vehicle_id: vehicle_id.to_string(),
            last_step: 0,
            current_position: [0.0; 3],
            current_velocity: [0.0; 3],
            start_position: [0.0; 3],
            armed: false,
            max_altitude: 0.0,
            fc_connected: false, // Will be detected when FC responds
        })
    }

    /// Check if FC is responding to MAVLink commands
    fn try_connect_fc(&mut self) -> bool {
        // Send heartbeats to establish GCS connection
        // Sensor data is fed from Gazebo by the FC (main.rs) directly
        for _ in 0..10 {
            self.mav.send_heartbeat();
            std::thread::sleep(Duration::from_millis(10));
        }

        // Check for heartbeat response
        for _ in 0..10 {
            if let Some(MavMessage::Heartbeat(_)) = self.mav.recv() {
                self.log("FC connected via MAVLink");
                return true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        false
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
        self.log(&format!(
            "Lockstep: {}",
            if mission.lockstep { "YES" } else { "NO" }
        ));
        println!();

        // Try to connect to FC via MAVLink
        self.fc_connected = self.try_connect_fc();
        if self.fc_connected {
            self.log("Control mode: MAVLink (FC connected)");
        } else {
            self.log("Control mode: Direct (FC not running, using shared memory)");
        }

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
                    self.log(&format!(
                        "Model state ready (time={}us, step={})",
                        state.time_us, self.last_step
                    ));
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
            self.bridge.set_lockstep(false);
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
    ///
    /// When FC is connected (fc_connected=true), commands flow via MAVLink:
    ///   MissionRunner → MAVLink → FC → Controller → Mixer → GzPluginBridge → Gazebo
    ///
    /// When FC is not running (fc_connected=false), direct motor control:
    ///   MissionRunner → GzPluginBridge → Gazebo (bypasses FC, for testing)
    fn execute_action(&mut self, action: &Action) {
        if self.fc_connected {
            self.execute_action_mavlink(action);
        } else {
            self.execute_action_direct(action);
        }
    }

    /// Execute action via MAVLink commands to FC
    fn execute_action_mavlink(&mut self, action: &Action) {
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
                // TODO: Send fault injection via UDP to FC
                self.log(&format!(
                    "INJECT_FAULT {:?} {:?} (not yet implemented)",
                    sensor, fault
                ));
            }
            Action::ClearFaults => {
                // TODO: Send clear faults command via UDP to FC
                self.log("CLEAR_FAULTS (not yet implemented)");
            }
        }
    }

    /// Execute action via direct motor control (bypass FC)
    fn execute_action_direct(&mut self, action: &Action) {
        match action {
            Action::Wait => {}
            Action::Arm => {
                self.armed = true;
            }
            Action::Disarm => {
                self.armed = false;
                let _ = self.bridge.set_motor_speeds(&[0.0, 0.0, 0.0, 0.0]);
            }
            Action::Thrust(t) => {
                if self.armed {
                    let speed = (*t * 1000.0) as f64;
                    let _ = self.bridge.set_motor_speeds(&[speed, speed, speed, speed]);
                }
            }
            Action::AttitudeTarget { q: _, thrust } => {
                if self.armed {
                    let speed = (*thrust * 1000.0) as f64;
                    let _ = self.bridge.set_motor_speeds(&[speed, speed, speed, speed]);
                }
            }
            Action::GoTo {
                position: _,
                heading: _,
            } => {
                // Position control requires FC
            }
            Action::InjectFault { .. } | Action::ClearFaults => {
                // Fault injection requires FC connection
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
                let error = (dx * dx + dy * dy + dz * dz).sqrt();
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
                let drift = (dx * dx + dy * dy).sqrt();
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
                eprintln!(
                    "[{}:{}] Failed to create runner for {}: {}",
                    vehicle_id, instance, mission.name, e
                );
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
    use aviate_hal_xil::CriterionResult;

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
