//! Mission execution over typed backend directives.

use aviate_core::control::Command;
use std::time::Duration;

use crate::mission::{Action, CriterionResult, Mission, MissionResult, Phase, PhaseResult};
use crate::{
    DirectiveId, DirectiveOutcome, DirectiveReceipt, ResetGeneration, SimulatorBackend,
    SimulatorDirective, SimulatorDirectiveKind, SimulatorError, SimulatorOperation, VehicleState,
};

use super::trace::write_trace_csv;
use super::{MissionRunner, TraceSample};

impl<B: SimulatorBackend> MissionRunner<B> {
    /// Create a mission runner with one backend.
    pub fn new(backend: B, vehicle_id: &str) -> Result<Self, SimulatorError> {
        Ok(Self {
            backend,
            fault_client: None,
            vehicle_id: vehicle_id.to_string(),
            last_step: 0,
            last_simulation_time: Duration::ZERO,
            current_state: VehicleState::default(),
            start_position: [0.0; 3],
            armed: false,
            max_altitude: 0.0,
            generation: ResetGeneration::INITIAL,
            next_directive_id: 0,
            next_command_sequence: 0,
        })
    }

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

    /// Return the vehicle identifier.
    pub fn vehicle_id(&self) -> &str {
        &self.vehicle_id
    }

    /// Return the simulator instance.
    pub fn instance(&self) -> u8 {
        self.backend.instance()
    }

    /// Record one vehicle message.
    fn log(&self, msg: &str) {
        tracing::info!(
            target: "aviate_hal_xil::runner",
            instance = self.backend.instance(),
            vehicle = %self.vehicle_id,
            "{msg}"
        );
    }

    /// Record one vehicle failure.
    fn log_error(&self, msg: &str) {
        tracing::error!(
            target: "aviate_hal_xil::runner",
            instance = self.backend.instance(),
            vehicle = %self.vehicle_id,
            "{msg}"
        );
    }

    /// Run one complete mission.
    pub fn run(&mut self, mission: &Mission) -> MissionResult {
        self.run_with_preparation(mission, true)
    }

    /// Run one mission from the current simulator generation.
    ///
    /// The caller must own simulator startup. The caller must guarantee that
    /// the current generation contains the declared clean initial state.
    /// This function does not send a reset directive.
    pub fn run_from_current_state(&mut self, mission: &Mission) -> MissionResult {
        self.run_with_preparation(mission, false)
    }

    fn run_with_preparation(&mut self, mission: &Mission, reset: bool) -> MissionResult {
        self.log(&format!("==> {} | acknowledged=yes <==", mission.name));
        let timeout = Duration::from_secs(10);
        let mission_start = match self.prepare_mission(timeout, reset) {
            Ok(start) => start,
            Err((operation, error)) => return self.failed_mission(mission, operation, &error),
        };
        let (phase_results, mut mission_passed) = self.run_phases(mission);
        if let Err(error) = self.execute_directive(SimulatorDirectiveKind::Stop, timeout) {
            self.log_error(&format!("stop failed: {error}"));
            mission_passed = false;
        }
        let total_duration = self
            .backend
            .status()
            .simulation_time
            .saturating_sub(mission_start);
        self.write_trace(mission, &phase_results);
        self.complete_mission(mission, phase_results, mission_passed, total_duration)
    }

    fn prepare_mission(
        &mut self,
        timeout: Duration,
        reset: bool,
    ) -> Result<Duration, (&'static str, SimulatorError)> {
        let instance = self.backend.instance();
        let connected = self
            .backend
            .connect(instance, timeout)
            .map_err(|error| ("connect", error))?;
        self.generation = connected.generation;
        if reset {
            let receipt = self
                .execute_directive(SimulatorDirectiveKind::Reset, timeout)
                .map_err(|error| ("reset", error))?;
            self.generation = receipt.generation;
        }
        self.execute_directive(SimulatorDirectiveKind::Start, timeout)
            .map_err(|error| ("start", error))?;
        let initial = self
            .next_ready_frame(timeout)
            .map_err(|error| ("read initial frame", error))?;
        self.last_step = initial.step;
        self.last_simulation_time = initial.simulation_time;
        self.start_position = initial.vehicle.position;
        self.current_state = initial.vehicle;
        self.armed = false;
        self.max_altitude = 0.0;
        self.log(&format!(
            "backend ready (generation={}, step={}, time={:?})",
            self.generation.get(),
            self.last_step,
            initial.simulation_time
        ));
        Ok(initial.simulation_time)
    }

    fn run_phases(&mut self, mission: &Mission) -> (Vec<PhaseResult>, bool) {
        let mut results = Vec::new();
        let mut passed = true;
        let total = mission.phases.len();
        for (index, phase) in mission.phases.iter().enumerate() {
            let number = index.wrapping_add(1);
            self.log(&format!("[Phase {number}/{total}] {}", phase.name));
            let result = match self.run_phase(phase) {
                Ok(result) => result,
                Err(error) => {
                    self.log_error(&format!("phase {} failed: {error}", phase.name));
                    passed = false;
                    break;
                }
            };
            self.log_phase_result(number, total, phase, &result);
            passed &= result.passed;
            results.push(result);
        }
        (results, passed)
    }

    fn log_phase_result(&self, number: usize, total: usize, phase: &Phase, result: &PhaseResult) {
        if result.passed {
            self.log(&format!(
                "[Phase {number}/{total}] {} PASSED (alt: {:.2}m)",
                phase.name, result.max_altitude
            ));
            return;
        }
        self.log_error(&format!("[Phase {number}/{total}] {} FAILED", phase.name));
        for criterion in result.criteria_results.iter().filter(|item| !item.passed) {
            self.log_error(&format!(
                "  - {}: expected {}, got {}",
                criterion.criterion, criterion.expected, criterion.actual_value
            ));
        }
    }

    fn write_trace(&self, mission: &Mission, phase_results: &[PhaseResult]) {
        let csv_path = std::env::temp_dir().join(format!(
            "aviate_trace_{}.csv",
            mission.name.replace(['/', ' '], "_")
        ));
        if let Err(error) = write_trace_csv(&csv_path, &mission.name, phase_results) {
            self.log_error(&format!(
                "Failed to write trace CSV ({}): {}",
                csv_path.display(),
                error
            ));
        } else {
            self.log(&format!("Trace CSV: {}", csv_path.display()));
        }
    }

    fn complete_mission(
        &self,
        mission: &Mission,
        phases: Vec<PhaseResult>,
        passed: bool,
        total_duration: Duration,
    ) -> MissionResult {
        if passed {
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
            passed,
            phases,
            total_duration,
            max_altitude: self.max_altitude,
        }
    }

    fn failed_mission(
        &self,
        mission: &Mission,
        operation: &str,
        error: &SimulatorError,
    ) -> MissionResult {
        self.log_error(&format!("{operation} failed: {error}"));
        MissionResult {
            mission_name: mission.name.clone(),
            passed: false,
            phases: Vec::new(),
            total_duration: Duration::ZERO,
            max_altitude: 0.0,
        }
    }

    fn execute_directive(
        &mut self,
        kind: SimulatorDirectiveKind,
        timeout: Duration,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        let id = DirectiveId(self.next_directive_id);
        self.next_directive_id = self.next_directive_id.wrapping_add(1);
        let operation = kind.operation();
        let receipt = self.backend.execute(
            SimulatorDirective {
                id,
                generation: self.generation,
                kind,
            },
            timeout,
        )?;
        if receipt.id != id {
            return Err(SimulatorError::NotAvailable {
                operation,
                detail: "the receipt has another directive identity".to_owned(),
            });
        }
        let expected_generation = if operation == SimulatorOperation::Reset {
            self.generation.next()
        } else {
            self.generation
        };
        if receipt.generation != expected_generation {
            return Err(SimulatorError::StaleGeneration {
                expected: expected_generation,
                received: receipt.generation,
            });
        }
        if Some(receipt.outcome) != expected_outcome(operation) {
            return Err(SimulatorError::NotAvailable {
                operation,
                detail: "the receipt does not acknowledge the requested operation".to_owned(),
            });
        }
        Ok(receipt)
    }

    /// Run a single phase.
    fn run_phase(&mut self, phase: &Phase) -> Result<PhaseResult, SimulatorError> {
        let phase_start = self.backend.status().simulation_time;
        let mut phase_max_altitude = 0.0f32;
        let mut trace: Vec<TraceSample> = Vec::new();

        loop {
            self.execute_action(&phase.action)?;
            let frame = self.next_frame(Duration::from_secs(1))?;
            self.accept_phase_frame(&frame, matches!(&phase.action, Action::Disarm))?;
            let elapsed = frame.simulation_time.saturating_sub(phase_start);
            self.current_state = frame.vehicle;
            let altitude = -self.current_state.position[2];
            phase_max_altitude = phase_max_altitude.max(altitude);
            self.max_altitude = self.max_altitude.max(phase_max_altitude);
            trace.push(TraceSample {
                elapsed: elapsed.as_secs_f32(),
                sim_time_us: u64::try_from(frame.simulation_time.as_micros()).unwrap_or(u64::MAX),
                position: self.current_state.position,
                velocity: self.current_state.velocity,
                attitude: self.current_state.orientation,
                angular_velocity: self.current_state.angular_velocity,
            });
            if elapsed >= phase.duration {
                break;
            }
        }

        let criteria_results: Vec<CriterionResult> = phase
            .verify
            .iter()
            .map(|c| self.verify_criterion(c, phase_max_altitude, &trace))
            .collect();

        let passed = criteria_results.iter().all(|r| r.passed);

        Ok(PhaseResult {
            name: phase.name.clone(),
            passed,
            duration_actual: self
                .backend
                .status()
                .simulation_time
                .saturating_sub(phase_start),
            max_altitude: phase_max_altitude,
            final_position: self.current_state.position,
            criteria_results,
            trace,
            action_tag: format!("{:?}", phase.action),
        })
    }

    /// Execute an action through the backend contract.
    fn execute_action(&mut self, action: &Action) -> Result<(), SimulatorError> {
        match action {
            Action::Wait => {}
            Action::Arm => self.arm_if_ready()?,
            Action::Disarm => self.disarm_if_armed()?,
            Action::Thrust(t) => {
                if self.armed {
                    let command = self.attitude_command([1.0, 0.0, 0.0, 0.0], *t);
                    self.send_setpoint(command)?;
                }
            }
            Action::AttitudeTarget { q, thrust } => {
                if self.armed {
                    let command = self.attitude_command(*q, *thrust);
                    self.send_setpoint(command)?;
                }
            }
            Action::GoTo { position, heading } => {
                if self.armed {
                    let command = self.position_command(*position, *heading);
                    self.send_setpoint(command)?;
                }
            }
            Action::InjectFault { sensor, fault } => self.inject_fault(*sensor, *fault),
            Action::ClearFaults => self.clear_faults(),
        }
        Ok(())
    }

    fn arm_if_ready(&mut self) -> Result<(), SimulatorError> {
        if self.armed {
            return Ok(());
        }
        let ready = self.execute_directive(
            SimulatorDirectiveKind::CheckArmReadiness,
            Duration::from_secs(1),
        )?;
        if ready.outcome != DirectiveOutcome::ArmReady {
            return Err(SimulatorError::ReadinessFailed {
                generation: ready.generation,
                detail: "the arm-readiness receipt does not confirm readiness".to_owned(),
            });
        }
        self.execute_directive(SimulatorDirectiveKind::Arm, Duration::from_secs(1))?;
        self.armed = true;
        self.log("ARM acknowledged");
        Ok(())
    }

    fn disarm_if_armed(&mut self) -> Result<(), SimulatorError> {
        if !self.armed {
            return Ok(());
        }
        self.execute_directive(SimulatorDirectiveKind::Disarm, Duration::from_secs(1))?;
        self.armed = false;
        self.log("DISARM acknowledged");
        Ok(())
    }

    fn send_setpoint(&mut self, command: Command) -> Result<(), SimulatorError> {
        self.execute_directive(
            SimulatorDirectiveKind::Setpoint(command),
            Duration::from_secs(1),
        )?;
        Ok(())
    }

    fn inject_fault(
        &mut self,
        target: crate::mission::SensorTarget,
        spec: crate::mission::FaultSpec,
    ) {
        if let Some(client) = self.fault_client_mut() {
            match client.inject(target, spec) {
                Ok(ack) => self.log(&format!(
                    "INJECT_FAULT {target:?} {spec:?} ack={:?}",
                    ack.status
                )),
                Err(error) => {
                    self.log_error(&format!(
                        "INJECT_FAULT {target:?} {spec:?} failed: {error:?}"
                    ));
                }
            }
        } else {
            self.log_error(&format!(
                "INJECT_FAULT {target:?} {spec:?} skipped: FaultClient unavailable"
            ));
        }
    }

    fn clear_faults(&mut self) {
        if let Some(client) = self.fault_client_mut() {
            match client.clear_all() {
                Ok(ack) => self.log(&format!("CLEAR_FAULTS ack={:?}", ack.status)),
                Err(error) => self.log_error(&format!("CLEAR_FAULTS failed: {error:?}")),
            }
        } else {
            self.log_error("CLEAR_FAULTS skipped: FaultClient unavailable");
        }
    }
}

fn expected_outcome(operation: SimulatorOperation) -> Option<DirectiveOutcome> {
    match operation {
        SimulatorOperation::Start => Some(DirectiveOutcome::Started),
        SimulatorOperation::Stop => Some(DirectiveOutcome::Stopped),
        SimulatorOperation::Reset => Some(DirectiveOutcome::ResetAccepted),
        SimulatorOperation::CheckArmReadiness => Some(DirectiveOutcome::ArmReady),
        SimulatorOperation::Arm => Some(DirectiveOutcome::Armed),
        SimulatorOperation::Setpoint => Some(DirectiveOutcome::SetpointAccepted),
        SimulatorOperation::Disarm => Some(DirectiveOutcome::Disarmed),
        SimulatorOperation::Connect | SimulatorOperation::NextFrame => None,
    }
}
