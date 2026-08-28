//! Deterministic directive-session tests.
#![allow(clippy::expect_used, clippy::panic)]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::mission::{Action, Criterion, Mission, Phase, VehicleConfig};
use crate::{
    BackendStatus, DirectiveOutcome, DirectiveReceipt, FrameEvent, MissionRunner, ResetGeneration,
    SimulatorBackend, SimulatorDirective, SimulatorDirectiveKind, SimulatorError, SimulatorFrame,
    SimulatorLifecycle, SimulatorOperation, VehicleState,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DirectiveTag {
    Start,
    Stop,
    Reset,
    CheckArmReadiness,
    Arm,
    Setpoint,
    Disarm,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Observation {
    generation: ResetGeneration,
    tag: DirectiveTag,
}

struct ScriptBackend {
    status: BackendStatus,
    observations: Arc<Mutex<Vec<Observation>>>,
    fail_after_arm: bool,
}

impl ScriptBackend {
    fn new(observations: Arc<Mutex<Vec<Observation>>>) -> Self {
        Self {
            status: BackendStatus::default(),
            observations,
            fail_after_arm: false,
        }
    }

    fn with_post_arm_failure(observations: Arc<Mutex<Vec<Observation>>>) -> Self {
        Self {
            fail_after_arm: true,
            ..Self::new(observations)
        }
    }

    fn receipt(
        &self,
        directive: &SimulatorDirective,
        outcome: DirectiveOutcome,
    ) -> DirectiveReceipt {
        DirectiveReceipt {
            id: directive.id,
            generation: self.status.generation,
            step: self.status.step,
            simulation_time: self.status.simulation_time,
            outcome,
        }
    }

    fn observe(&self, generation: ResetGeneration, tag: DirectiveTag) {
        self.observations
            .lock()
            .expect("observation lock must be available")
            .push(Observation { generation, tag });
    }
}

impl SimulatorBackend for ScriptBackend {
    fn name(&self) -> &str {
        "script"
    }

    fn connect(
        &mut self,
        _instance: u8,
        _timeout: Duration,
    ) -> Result<BackendStatus, SimulatorError> {
        Ok(self.status)
    }

    fn status(&self) -> BackendStatus {
        self.status
    }

    fn execute(
        &mut self,
        directive: SimulatorDirective,
        _timeout: Duration,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        if directive.generation != self.status.generation {
            return Err(SimulatorError::StaleGeneration {
                expected: self.status.generation,
                received: directive.generation,
            });
        }
        let (tag, outcome) = match &directive.kind {
            SimulatorDirectiveKind::Start => {
                self.status.lifecycle = SimulatorLifecycle::Ready;
                (DirectiveTag::Start, DirectiveOutcome::Started)
            }
            SimulatorDirectiveKind::Stop => {
                self.status.lifecycle = SimulatorLifecycle::Stopped;
                (DirectiveTag::Stop, DirectiveOutcome::Stopped)
            }
            SimulatorDirectiveKind::Reset => {
                self.status = BackendStatus {
                    generation: self.status.generation.next(),
                    lifecycle: SimulatorLifecycle::Ready,
                    ..BackendStatus::default()
                };
                (DirectiveTag::Reset, DirectiveOutcome::ResetAccepted)
            }
            SimulatorDirectiveKind::CheckArmReadiness => {
                (DirectiveTag::CheckArmReadiness, DirectiveOutcome::ArmReady)
            }
            SimulatorDirectiveKind::Arm => {
                self.status.armed = true;
                (DirectiveTag::Arm, DirectiveOutcome::Armed)
            }
            SimulatorDirectiveKind::Setpoint(_) => {
                (DirectiveTag::Setpoint, DirectiveOutcome::SetpointAccepted)
            }
            SimulatorDirectiveKind::Disarm => {
                self.status.armed = false;
                self.status.lifecycle = SimulatorLifecycle::Converging;
                (DirectiveTag::Disarm, DirectiveOutcome::Disarmed)
            }
        };
        self.observe(self.status.generation, tag);
        Ok(self.receipt(&directive, outcome))
    }

    fn next_frame(&mut self, _timeout: Duration) -> Result<FrameEvent, SimulatorError> {
        if !matches!(
            self.status.lifecycle,
            SimulatorLifecycle::Ready | SimulatorLifecycle::Converging
        ) {
            return Err(SimulatorError::InvalidLifecycle {
                operation: SimulatorOperation::NextFrame,
                lifecycle: self.status.lifecycle,
            });
        }
        self.status.step = self.status.step.wrapping_add(1);
        self.status.simulation_time = self
            .status
            .simulation_time
            .saturating_add(Duration::from_millis(10));
        if self.fail_after_arm && self.status.armed {
            self.status.armed = false;
            self.status.lifecycle = SimulatorLifecycle::Converging;
        }
        Ok(FrameEvent::Frame(SimulatorFrame {
            generation: self.status.generation,
            step: self.status.step,
            simulation_time: self.status.simulation_time,
            lifecycle: self.status.lifecycle,
            vehicle: initial_vehicle_state(),
            armed: self.status.armed,
        }))
    }

    fn instance(&self) -> u8 {
        0
    }
}

#[test]
fn repeated_scripted_sessions_start_identically() {
    let observations = Arc::new(Mutex::new(Vec::new()));
    let backend = ScriptBackend::new(Arc::clone(&observations));
    let mut runner = MissionRunner::new(backend, "alia").expect("runner must be valid");
    let mission = scripted_mission();

    for _ in 0..3 {
        let result = runner.run(&mission);
        assert!(result.passed);
        assert_eq!(result.phases[0].trace[0].position, [1.0, 2.0, -3.0]);
    }

    let observations = observations
        .lock()
        .expect("observation lock must be available");
    for generation in 2..=4 {
        assert_generation_sequence(&observations, ResetGeneration::new(generation));
    }
}

#[test]
fn post_arm_lifecycle_loss_fails_the_mission() {
    let observations = Arc::new(Mutex::new(Vec::new()));
    let backend = ScriptBackend::with_post_arm_failure(observations);
    let mut runner = MissionRunner::new(backend, "alia").expect("runner must be valid");

    let result = runner.run(&scripted_mission());

    assert!(!result.passed);
    assert!(result.phases.is_empty());
}

#[test]
fn current_state_session_does_not_claim_a_reset() {
    let observations = Arc::new(Mutex::new(Vec::new()));
    let backend = ScriptBackend::new(Arc::clone(&observations));
    let mut runner = MissionRunner::new(backend, "gazebo").expect("runner must be valid");

    let result = runner.run_from_current_state(&scripted_mission());

    assert!(result.passed);
    let observations = observations
        .lock()
        .expect("observation lock must be available");
    assert!(!observations
        .iter()
        .any(|item| item.tag == DirectiveTag::Reset));
    assert_generation_sequence_without_reset(&observations, ResetGeneration::INITIAL);
}

fn assert_generation_sequence(observations: &[Observation], generation: ResetGeneration) {
    let tags: Vec<_> = observations
        .iter()
        .filter(|item| item.generation == generation)
        .map(|item| item.tag)
        .collect();
    assert_eq!(
        tags,
        [
            DirectiveTag::Reset,
            DirectiveTag::Start,
            DirectiveTag::CheckArmReadiness,
            DirectiveTag::Arm,
            DirectiveTag::Setpoint,
            DirectiveTag::Setpoint,
            DirectiveTag::Disarm,
            DirectiveTag::Stop,
        ]
    );
}

fn assert_generation_sequence_without_reset(
    observations: &[Observation],
    generation: ResetGeneration,
) {
    let tags: Vec<_> = observations
        .iter()
        .filter(|item| item.generation == generation)
        .map(|item| item.tag)
        .collect();
    assert_eq!(
        tags,
        [
            DirectiveTag::Start,
            DirectiveTag::CheckArmReadiness,
            DirectiveTag::Arm,
            DirectiveTag::Setpoint,
            DirectiveTag::Setpoint,
            DirectiveTag::Disarm,
            DirectiveTag::Stop,
        ]
    );
}

fn initial_vehicle_state() -> VehicleState {
    VehicleState {
        position: [1.0, 2.0, -3.0],
        orientation: [1.0, 0.0, 0.0, 0.0],
        valid: true,
        ..VehicleState::default()
    }
}

fn scripted_mission() -> Mission {
    Mission {
        name: "scripted-backend-contract".to_owned(),
        description: "Exercise the directive contract.".to_owned(),
        vehicle: VehicleConfig::default(),
        lockstep: true,
        phases: vec![
            phase("arm", Action::Arm, Criterion::Armed(true), 10),
            phase("setpoint", Action::Thrust(0.5), Criterion::Armed(true), 20),
            phase("disarm", Action::Disarm, Criterion::Armed(false), 10),
        ],
        reset_between_runs: true,
    }
}

fn phase(name: &str, action: Action, criterion: Criterion, duration_ms: u64) -> Phase {
    Phase {
        name: name.to_owned(),
        duration: Duration::from_millis(duration_ms),
        action,
        verify: vec![criterion],
    }
}
