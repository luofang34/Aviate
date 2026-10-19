//! The typed simulator backend over the JSBSim board.

use std::time::{Duration, Instant};

use aviate_hal_xil::{
    BackendStatus, DirectiveOutcome, DirectiveReceipt, FaultAck, FaultCommand, FrameEvent,
    SimulatorBackend, SimulatorDirective, SimulatorDirectiveKind, SimulatorError, SimulatorFrame,
    SimulatorLifecycle, SimulatorOperation,
};

use crate::JsbsimBoard;

/// A `SimulatorBackend` over the lockstep JSBSim board.
///
/// The sidecar starts a new world for each TCP session, so a reset ends
/// the session and waits for the first state of the new one.
pub struct JsbsimSimulatorBackend<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    board: JsbsimBoard<C, M>,
    instance: u8,
}

impl<C, M> JsbsimSimulatorBackend<C, M>
where
    C: aviate_core::control::VehicleController + Send,
    C::RuntimeState: Send,
    M: aviate_core::mixer::Mixer + Send,
{
    /// Wrap a board for SITL instance `instance`.
    #[must_use]
    pub fn new(board: JsbsimBoard<C, M>, instance: u8) -> Self {
        Self { board, instance }
    }

    /// The wrapped board.
    #[must_use]
    pub fn board(&self) -> &JsbsimBoard<C, M> {
        &self.board
    }

    /// The wrapped board, for interactive control.
    pub fn board_mut(&mut self) -> &mut JsbsimBoard<C, M> {
        &mut self.board
    }

    fn check_generation(&self, directive: &SimulatorDirective) -> Result<(), SimulatorError> {
        let active = self.board.lifecycle_status().generation;
        if directive.generation == active {
            Ok(())
        } else {
            Err(SimulatorError::StaleGeneration {
                expected: active,
                received: directive.generation,
            })
        }
    }

    fn receipt(
        &self,
        directive: &SimulatorDirective,
        outcome: DirectiveOutcome,
    ) -> DirectiveReceipt {
        let status = self.board.lifecycle_status();
        DirectiveReceipt {
            id: directive.id,
            generation: status.generation,
            step: status.step,
            simulation_time: status.simulation_time,
            outcome,
        }
    }

    fn require_ready(&self) -> Result<(), SimulatorError> {
        let status = self.board.lifecycle_status();
        if status.lifecycle == SimulatorLifecycle::Ready {
            Ok(())
        } else {
            Err(SimulatorError::ReadinessFailed {
                generation: status.generation,
                detail: format!("the session is {:?}, not ready", status.lifecycle),
            })
        }
    }

    /// Step the board until a state of the new world arrives.
    fn await_new_world(&mut self, timeout: Duration) -> Result<(), SimulatorError> {
        let deadline = Instant::now() + timeout;
        while self.board.lifecycle_status().lifecycle == SimulatorLifecycle::Resetting {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(SimulatorError::Timeout {
                    operation: SimulatorOperation::Reset,
                    generation: self.board.lifecycle_status().generation,
                    timeout,
                });
            }
            self.board
                .wait_for_truth(remaining.min(Duration::from_millis(50)));
            while self.board.step().is_some() {}
        }
        Ok(())
    }

    fn timed_out_or_lost(&self, timeout: Duration) -> Result<FrameEvent, SimulatorError> {
        let status = self.board.lifecycle_status();
        if self.board.connected() {
            Ok(FrameEvent::TimedOut {
                generation: status.generation,
                last_step: status.step,
                timeout,
            })
        } else {
            Err(SimulatorError::BridgeLost {
                generation: status.generation,
                last_step: status.step,
            })
        }
    }
}

impl<C, M> SimulatorBackend for JsbsimSimulatorBackend<C, M>
where
    C: aviate_core::control::VehicleController + Send,
    C::RuntimeState: Send,
    M: aviate_core::mixer::Mixer + Send,
{
    fn name(&self) -> &str {
        "jsbsim-lockstep"
    }

    fn connect(
        &mut self,
        instance: u8,
        timeout: Duration,
    ) -> Result<BackendStatus, SimulatorError> {
        if instance != self.instance {
            return Err(SimulatorError::ConnectionFailed {
                backend: self.name().to_owned(),
                detail: format!("instance {instance} does not match {}", self.instance),
            });
        }
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            self.board.step();
            if self.board.connected() {
                return Ok(self.board.lifecycle_status());
            }
            self.board.wait_for_truth(Duration::from_millis(10));
        }
        Err(SimulatorError::Timeout {
            operation: SimulatorOperation::Connect,
            generation: self.board.lifecycle_status().generation,
            timeout,
        })
    }

    fn status(&self) -> BackendStatus {
        self.board.lifecycle_status()
    }

    fn execute(
        &mut self,
        directive: SimulatorDirective,
        timeout: Duration,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        self.check_generation(&directive)?;
        let generation = self.board.lifecycle_status().generation;
        match &directive.kind {
            SimulatorDirectiveKind::Start => {
                self.board.start_session();
                Ok(self.receipt(&directive, DirectiveOutcome::Started))
            }
            SimulatorDirectiveKind::Stop => {
                self.board.stop_session();
                Ok(self.receipt(&directive, DirectiveOutcome::Stopped))
            }
            SimulatorDirectiveKind::Reset => {
                self.board
                    .begin_reset()
                    .map_err(|error| SimulatorError::ResetFailed {
                        generation,
                        detail: error.to_string(),
                    })?;
                self.await_new_world(timeout)?;
                Ok(self.receipt(&directive, DirectiveOutcome::ResetAccepted))
            }
            SimulatorDirectiveKind::CheckArmReadiness => {
                self.require_ready()?;
                self.board.check_arm_readiness().map_err(|error| {
                    SimulatorError::ReadinessFailed {
                        generation,
                        detail: format!("{error:?}"),
                    }
                })?;
                Ok(self.receipt(&directive, DirectiveOutcome::ArmReady))
            }
            SimulatorDirectiveKind::Arm => {
                self.require_ready()?;
                self.board
                    .arm()
                    .map_err(|error| SimulatorError::ArmRefused {
                        generation,
                        detail: format!("{error:?}"),
                    })?;
                Ok(self.receipt(&directive, DirectiveOutcome::Armed))
            }
            SimulatorDirectiveKind::Setpoint(command) => {
                self.require_ready()?;
                self.board.set_command(command.clone());
                Ok(self.receipt(&directive, DirectiveOutcome::SetpointAccepted))
            }
            SimulatorDirectiveKind::Disarm => {
                self.board
                    .disarm()
                    .map_err(|error| SimulatorError::NotAvailable {
                        operation: SimulatorOperation::Disarm,
                        detail: format!("{error:?}"),
                    })?;
                Ok(self.receipt(&directive, DirectiveOutcome::Disarmed))
            }
        }
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<FrameEvent, SimulatorError> {
        let lifecycle = self.board.lifecycle_status().lifecycle;
        if matches!(
            lifecycle,
            SimulatorLifecycle::Stopped | SimulatorLifecycle::Resetting
        ) {
            return Err(SimulatorError::InvalidLifecycle {
                operation: SimulatorOperation::NextFrame,
                lifecycle,
            });
        }
        let deadline = Instant::now() + timeout;
        loop {
            if self.board.step().is_some() {
                let status = self.board.lifecycle_status();
                return Ok(FrameEvent::Frame(SimulatorFrame {
                    generation: status.generation,
                    step: status.step,
                    simulation_time: status.simulation_time,
                    lifecycle: status.lifecycle,
                    vehicle: self.board.vehicle().clone(),
                    armed: status.armed,
                }));
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() || !self.board.wait_for_truth(remaining) {
                return self.timed_out_or_lost(timeout);
            }
        }
    }

    fn instance(&self) -> u8 {
        self.instance
    }

    fn apply_fault(&mut self, command: &FaultCommand) -> Option<FaultAck> {
        Some(self.board.apply_fault(command))
    }
}
