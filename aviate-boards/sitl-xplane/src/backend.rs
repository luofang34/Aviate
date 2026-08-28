//! XIL backend adapter for the X-Plane board.

use std::time::{Duration, Instant};

use aviate_hal_xil::{
    BackendStatus, DirectiveOutcome, DirectiveReceipt, FrameEvent, SimulatorBackend,
    SimulatorDirective, SimulatorDirectiveKind, SimulatorError, SimulatorFrame, SimulatorLifecycle,
    SimulatorOperation, VehicleState,
};

use crate::XPlaneBoard;

/// A typed backend over the existing sample-paced X-Plane board.
pub struct XPlaneSimulatorBackend<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    board: XPlaneBoard<C, M>,
    instance: u8,
    declared_initial: VehicleState,
    last_vehicle: VehicleState,
    has_fix: bool,
    has_truth: bool,
    reset_validation_pending: bool,
}

impl<C, M> XPlaneSimulatorBackend<C, M>
where
    C: aviate_core::control::VehicleController + Send,
    C::RuntimeState: Send,
    M: aviate_core::mixer::Mixer + Send,
{
    /// Wrap a board and bind one declared initial vehicle state.
    #[must_use]
    pub fn new(board: XPlaneBoard<C, M>, instance: u8, declared_initial: VehicleState) -> Self {
        Self {
            board,
            instance,
            declared_initial,
            last_vehicle: VehicleState::default(),
            has_fix: false,
            has_truth: false,
            reset_validation_pending: false,
        }
    }

    /// Return the wrapped board.
    #[must_use]
    pub fn board(&self) -> &XPlaneBoard<C, M> {
        &self.board
    }

    /// Return the wrapped board for interactive control.
    pub fn board_mut(&mut self) -> &mut XPlaneBoard<C, M> {
        &mut self.board
    }

    /// Consume the adapter and return the board.
    #[must_use]
    pub fn into_board(self) -> XPlaneBoard<C, M> {
        self.board
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

    fn validate_initial_state(&self, actual: &VehicleState) -> Result<(), SimulatorError> {
        if same_initial_state(actual, &self.declared_initial) && !self.board.is_armed() {
            Ok(())
        } else {
            Err(SimulatorError::ReadinessFailed {
                generation: self.board.lifecycle_status().generation,
                detail: "the reset state does not match the declared initial state".to_owned(),
            })
        }
    }

    fn update_vehicle_state(&mut self) {
        if let Some(fix) = self.board.last_fix() {
            self.last_vehicle.position = fix.position_ned;
            self.last_vehicle.velocity = fix.vel_ned;
            self.has_fix = true;
        }
        if let Some(truth) = self.board.take_truth() {
            self.last_vehicle.orientation = truth.attitude_quaternion;
            self.last_vehicle.angular_velocity =
                [truth.rollspeed, truth.pitchspeed, truth.yawspeed];
            self.last_vehicle.velocity = [
                f32::from(truth.vx) / 100.0,
                f32::from(truth.vy) / 100.0,
                f32::from(truth.vz) / 100.0,
            ];
            self.has_truth = true;
        }
        self.last_vehicle.valid = self.has_fix && self.has_truth;
    }

    fn reset_cached_vehicle(&mut self) {
        self.last_vehicle = VehicleState::default();
        self.has_fix = false;
        self.has_truth = false;
    }

    fn require_validated_ready(&self) -> Result<(), SimulatorError> {
        let status = self.board.lifecycle_status();
        if status.lifecycle != SimulatorLifecycle::Ready || self.reset_validation_pending {
            return Err(SimulatorError::ReadinessFailed {
                generation: status.generation,
                detail: "the reset generation is not validated and ready".to_owned(),
            });
        }
        Ok(())
    }
}

impl<C, M> SimulatorBackend for XPlaneSimulatorBackend<C, M>
where
    C: aviate_core::control::VehicleController + Send,
    C::RuntimeState: Send,
    M: aviate_core::mixer::Mixer + Send,
{
    fn name(&self) -> &str {
        "xplane-alia-sample-paced"
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
            std::thread::yield_now();
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
                        generation: self.board.lifecycle_status().generation,
                        detail: error.to_string(),
                    })?;
                self.reset_cached_vehicle();
                self.reset_validation_pending = true;
                self.await_reset_ack(timeout)?;
                Ok(self.receipt(&directive, DirectiveOutcome::ResetAccepted))
            }
            SimulatorDirectiveKind::CheckArmReadiness => {
                self.require_validated_ready()?;
                self.board.check_arm_readiness().map_err(|error| {
                    SimulatorError::ReadinessFailed {
                        generation: self.board.lifecycle_status().generation,
                        detail: format!("{error:?}"),
                    }
                })?;
                Ok(self.receipt(&directive, DirectiveOutcome::ArmReady))
            }
            SimulatorDirectiveKind::Arm => {
                self.require_validated_ready()?;
                self.board
                    .arm()
                    .map_err(|error| SimulatorError::ArmRefused {
                        generation: self.board.lifecycle_status().generation,
                        detail: format!("{error:?}"),
                    })?;
                Ok(self.receipt(&directive, DirectiveOutcome::Armed))
            }
            SimulatorDirectiveKind::Setpoint(command) => {
                self.require_validated_ready()?;
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
        let before = self.board.lifecycle_status();
        if before.lifecycle == SimulatorLifecycle::Stopped {
            return Err(SimulatorError::InvalidLifecycle {
                operation: SimulatorOperation::NextFrame,
                lifecycle: before.lifecycle,
            });
        }
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() || !self.board.wait_for_sample(remaining) {
                return self.frame_timeout_or_bridge_loss(timeout);
            }
            self.board.step();
            let status = self.board.lifecycle_status();
            if status.step <= before.step {
                continue;
            }
            self.update_vehicle_state();
            if status.lifecycle == SimulatorLifecycle::Ready && self.reset_validation_pending {
                self.validate_initial_state(&self.last_vehicle)?;
                self.reset_validation_pending = false;
            }
            return Ok(FrameEvent::Frame(SimulatorFrame {
                generation: status.generation,
                step: status.step,
                simulation_time: status.simulation_time,
                lifecycle: status.lifecycle,
                vehicle: self.last_vehicle.clone(),
                armed: status.armed,
            }));
        }
    }

    fn instance(&self) -> u8 {
        self.instance
    }
}

impl<C, M> XPlaneSimulatorBackend<C, M>
where
    C: aviate_core::control::VehicleController + Send,
    C::RuntimeState: Send,
    M: aviate_core::mixer::Mixer + Send,
{
    fn await_reset_ack(&mut self, timeout: Duration) -> Result<(), SimulatorError> {
        let deadline = Instant::now() + timeout;
        while self.board.lifecycle_status().lifecycle == SimulatorLifecycle::Resetting {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() || !self.board.wait_for_sample(remaining) {
                return Err(self.reset_wait_error(timeout));
            }
            self.board.step();
        }
        let lifecycle = self.board.lifecycle_status().lifecycle;
        if matches!(
            lifecycle,
            SimulatorLifecycle::Converging | SimulatorLifecycle::Ready
        ) {
            self.update_vehicle_state();
            Ok(())
        } else {
            Err(SimulatorError::InvalidLifecycle {
                operation: SimulatorOperation::Reset,
                lifecycle,
            })
        }
    }

    fn reset_wait_error(&self, timeout: Duration) -> SimulatorError {
        let status = self.board.lifecycle_status();
        if self.board.connected() {
            SimulatorError::Timeout {
                operation: SimulatorOperation::Reset,
                generation: status.generation,
                timeout,
            }
        } else {
            SimulatorError::BridgeLost {
                generation: status.generation,
                last_step: status.step,
            }
        }
    }

    fn frame_timeout_or_bridge_loss(
        &self,
        timeout: Duration,
    ) -> Result<FrameEvent, SimulatorError> {
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

fn same_initial_state(actual: &VehicleState, expected: &VehicleState) -> bool {
    actual.valid
        && expected.valid
        && close_array(&actual.position, &expected.position, 0.01)
        && close_array(&actual.velocity, &expected.velocity, 0.01)
        && close_array(&actual.orientation, &expected.orientation, 0.001)
        && close_array(&actual.angular_velocity, &expected.angular_velocity, 0.001)
}

fn close_array<const N: usize>(actual: &[f32; N], expected: &[f32; N], tolerance: f32) -> bool {
    actual
        .iter()
        .zip(expected)
        .all(|(left, right)| (*left - *right).abs() <= tolerance)
}

#[cfg(test)]
mod tests {
    use super::same_initial_state;
    use aviate_hal_xil::VehicleState;

    #[test]
    fn initial_state_requires_all_declared_fields() {
        let state = VehicleState {
            orientation: [1.0, 0.0, 0.0, 0.0],
            valid: true,
            ..VehicleState::default()
        };
        assert!(same_initial_state(&state, &state));
        let mut moved = state.clone();
        moved.position[0] = 0.02;
        assert!(!same_initial_state(&moved, &state));
    }
}
