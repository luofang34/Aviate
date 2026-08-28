//! Typed simulator backend for Gazebo.

use std::time::Duration;

use aviate_hal_xil::{
    BackendStatus, DirectiveReceipt, FrameEvent, SimulatorBackend, SimulatorDirective,
    SimulatorError, SimulatorOperation,
};
#[cfg(feature = "gz-plugin")]
use aviate_hal_xil::{
    DirectiveOutcome, MavClient, ResetGeneration, SimulatorDirectiveKind, SimulatorFrame,
    SimulatorLifecycle, VehicleState,
};

#[cfg(feature = "gz-plugin")]
use crate::plugin::{
    enu_quat_to_ned_f32, enu_to_ned_f32, flu_to_frd_f32, GzPluginBridge, GzPluginError,
};

/// Gazebo backend for the XIL directive contract.
pub struct GazeboSimBackend {
    #[cfg(feature = "gz-plugin")]
    bridge: Option<GzPluginBridge>,
    #[cfg(feature = "gz-plugin")]
    mav: Option<MavClient>,
    instance: u8,
    status: BackendStatus,
}

impl GazeboSimBackend {
    /// Create a disconnected backend.
    #[must_use]
    pub fn new(instance: u8) -> Self {
        Self {
            #[cfg(feature = "gz-plugin")]
            bridge: None,
            #[cfg(feature = "gz-plugin")]
            mav: None,
            instance,
            status: BackendStatus::default(),
        }
    }

    /// Create and connect a backend.
    pub fn connect_new(instance: u8, timeout_ms: u64) -> Result<Self, SimulatorError> {
        let mut backend = Self::new(instance);
        backend.connect(instance, Duration::from_millis(timeout_ms))?;
        Ok(backend)
    }

    #[cfg(feature = "gz-plugin")]
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

    #[cfg(feature = "gz-plugin")]
    fn check_generation(&self, directive: &SimulatorDirective) -> Result<(), SimulatorError> {
        if directive.generation == self.status.generation {
            Ok(())
        } else {
            Err(SimulatorError::StaleGeneration {
                expected: self.status.generation,
                received: directive.generation,
            })
        }
    }

    #[cfg(feature = "gz-plugin")]
    fn mav_mut(&mut self, operation: SimulatorOperation) -> Result<&mut MavClient, SimulatorError> {
        self.mav
            .as_mut()
            .ok_or_else(|| SimulatorError::NotAvailable {
                operation,
                detail: "the command link is not connected".to_owned(),
            })
    }

    #[cfg(feature = "gz-plugin")]
    fn send_setpoint(&mut self, command: &aviate_core::control::Command) -> bool {
        use aviate_core::control::ControlMode;

        let Some(mav) = self.mav.as_mut() else {
            return false;
        };
        if command.mode == ControlMode::PositionHold {
            let Some(position) = command.setpoint.position else {
                return false;
            };
            mav.send_position_target(
                position[0].0,
                position[1].0,
                position[2].0,
                command.setpoint.heading.map_or(0.0, |value| value.0),
            )
        } else {
            let attitude = command
                .setpoint
                .attitude
                .unwrap_or(aviate_core::math::Quaternion::IDENTITY);
            mav.send_attitude_target(
                [attitude.w, attitude.x, attitude.y, attitude.z],
                command.setpoint.collective_thrust.0,
            )
        }
    }
}

impl Default for GazeboSimBackend {
    fn default() -> Self {
        Self::new(0)
    }
}

#[cfg(feature = "gz-plugin")]
impl GazeboSimBackend {
    fn start(
        &mut self,
        directive: &SimulatorDirective,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        let bridge = self
            .bridge
            .as_ref()
            .ok_or_else(|| SimulatorError::NotAvailable {
                operation: SimulatorOperation::Start,
                detail: "the Gazebo bridge is not connected".to_owned(),
            })?;
        bridge
            .set_lockstep(true)
            .map_err(|error| SimulatorError::ConnectionFailed {
                backend: self.name().to_owned(),
                detail: error.to_string(),
            })?;
        self.status.lifecycle = SimulatorLifecycle::Converging;
        Ok(self.receipt(directive, DirectiveOutcome::Started))
    }

    fn stop(&mut self, directive: &SimulatorDirective) -> Result<DirectiveReceipt, SimulatorError> {
        if let Some(bridge) = self.bridge.as_ref() {
            bridge
                .set_lockstep(false)
                .map_err(|error| SimulatorError::ConnectionFailed {
                    backend: self.name().to_owned(),
                    detail: error.to_string(),
                })?;
        }
        self.status.lifecycle = SimulatorLifecycle::Stopped;
        self.status.armed = false;
        Ok(self.receipt(directive, DirectiveOutcome::Stopped))
    }

    fn arm_ready(
        &self,
        directive: &SimulatorDirective,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        if self.status.lifecycle != SimulatorLifecycle::Ready {
            return Err(SimulatorError::ReadinessFailed {
                generation: self.status.generation,
                detail: "the Gazebo model is not ready".to_owned(),
            });
        }
        Ok(self.receipt(directive, DirectiveOutcome::ArmReady))
    }

    fn arm(&mut self, directive: &SimulatorDirective) -> Result<DirectiveReceipt, SimulatorError> {
        if !self.mav_mut(SimulatorOperation::Arm)?.send_arm() {
            return Err(SimulatorError::ArmRefused {
                generation: self.status.generation,
                detail: "the command link did not send the arm request".to_owned(),
            });
        }
        self.status.armed = true;
        Ok(self.receipt(directive, DirectiveOutcome::Armed))
    }

    fn setpoint(
        &mut self,
        directive: &SimulatorDirective,
        command: &aviate_core::control::Command,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        if !self.send_setpoint(command) {
            return Err(SimulatorError::NotAvailable {
                operation: SimulatorOperation::Setpoint,
                detail: "the command link did not send the setpoint".to_owned(),
            });
        }
        Ok(self.receipt(directive, DirectiveOutcome::SetpointAccepted))
    }

    fn disarm(
        &mut self,
        directive: &SimulatorDirective,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        if !self.mav_mut(SimulatorOperation::Disarm)?.send_disarm() {
            return Err(SimulatorError::NotAvailable {
                operation: SimulatorOperation::Disarm,
                detail: "the command link did not send the disarm request".to_owned(),
            });
        }
        self.status.armed = false;
        Ok(self.receipt(directive, DirectiveOutcome::Disarmed))
    }
}

#[cfg(feature = "gz-plugin")]
impl SimulatorBackend for GazeboSimBackend {
    fn name(&self) -> &str {
        "gazebo"
    }

    fn connect(
        &mut self,
        instance: u8,
        timeout: Duration,
    ) -> Result<BackendStatus, SimulatorError> {
        let interval_ms = 500_u64;
        let attempts =
            u32::try_from((timeout.as_millis() as u64 / interval_ms).max(1)).unwrap_or(u32::MAX);
        let bridge = GzPluginBridge::connect_instance_with_retry(instance, attempts, interval_ms)
            .map_err(|error| match error {
            GzPluginError::PluginNotRunning => SimulatorError::NotAvailable {
                operation: SimulatorOperation::Connect,
                detail: "the Gazebo plugin is not running".to_owned(),
            },
            _ => SimulatorError::ConnectionFailed {
                backend: self.name().to_owned(),
                detail: error.to_string(),
            },
        })?;
        self.status.generation = ResetGeneration::new(bridge.reset_generation());
        self.status.lifecycle = SimulatorLifecycle::Converging;
        self.bridge = Some(bridge);
        self.mav = Some(MavClient::new(instance)?);
        self.instance = instance;
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
        self.check_generation(&directive)?;
        match &directive.kind {
            SimulatorDirectiveKind::Start => self.start(&directive),
            SimulatorDirectiveKind::Stop => self.stop(&directive),
            SimulatorDirectiveKind::Reset => Err(SimulatorError::NotAvailable {
                operation: SimulatorOperation::Reset,
                detail: "the Gazebo world-reset acknowledgement is not available".to_owned(),
            }),
            SimulatorDirectiveKind::CheckArmReadiness => self.arm_ready(&directive),
            SimulatorDirectiveKind::Arm => self.arm(&directive),
            SimulatorDirectiveKind::Setpoint(command) => self.setpoint(&directive, command),
            SimulatorDirectiveKind::Disarm => self.disarm(&directive),
        }
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<FrameEvent, SimulatorError> {
        let bridge = self
            .bridge
            .as_ref()
            .ok_or_else(|| SimulatorError::NotAvailable {
                operation: SimulatorOperation::NextFrame,
                detail: "the Gazebo bridge is not connected".to_owned(),
            })?;
        let Some(model_state) = bridge.wait_for_state(
            self.status.step,
            u64::try_from(timeout.as_micros()).unwrap_or(u64::MAX),
        ) else {
            return Ok(FrameEvent::TimedOut {
                generation: self.status.generation,
                last_step: self.status.step,
                timeout,
            });
        };
        let reset_generation = ResetGeneration::new(model_state.reset_generation);
        if reset_generation != self.status.generation {
            return Err(SimulatorError::StaleGeneration {
                expected: self.status.generation,
                received: reset_generation,
            });
        }
        let step = model_state.sim_step;
        let state = vehicle_state(&model_state);
        let simulation_time = Duration::from_micros(model_state.time_us);
        bridge.ack_step(step);
        self.status.step = step;
        self.status.simulation_time = simulation_time;
        if state.valid {
            self.status.lifecycle = SimulatorLifecycle::Ready;
        }
        Ok(FrameEvent::Frame(SimulatorFrame {
            generation: self.status.generation,
            step,
            simulation_time,
            lifecycle: self.status.lifecycle,
            vehicle: state,
            armed: self.status.armed,
        }))
    }

    fn instance(&self) -> u8 {
        self.instance
    }
}

#[cfg(feature = "gz-plugin")]
fn vehicle_state(state: &crate::plugin::AviateModelState) -> VehicleState {
    VehicleState {
        position: enu_to_ned_f32(state.pos),
        velocity: enu_to_ned_f32(state.vel),
        orientation: enu_quat_to_ned_f32(state.quat),
        angular_velocity: flu_to_frd_f32(state.ang_vel),
        valid: state.valid != 0,
    }
}

#[cfg(not(feature = "gz-plugin"))]
impl SimulatorBackend for GazeboSimBackend {
    fn name(&self) -> &str {
        "gazebo"
    }

    fn connect(
        &mut self,
        _instance: u8,
        _timeout: Duration,
    ) -> Result<BackendStatus, SimulatorError> {
        Err(SimulatorError::NotAvailable {
            operation: SimulatorOperation::Connect,
            detail: "the gz-plugin feature is not enabled".to_owned(),
        })
    }

    fn status(&self) -> BackendStatus {
        self.status
    }

    fn execute(
        &mut self,
        directive: SimulatorDirective,
        _timeout: Duration,
    ) -> Result<DirectiveReceipt, SimulatorError> {
        Err(SimulatorError::NotAvailable {
            operation: directive.kind.operation(),
            detail: "the gz-plugin feature is not enabled".to_owned(),
        })
    }

    fn next_frame(&mut self, _timeout: Duration) -> Result<FrameEvent, SimulatorError> {
        Err(SimulatorError::NotAvailable {
            operation: SimulatorOperation::NextFrame,
            detail: "the gz-plugin feature is not enabled".to_owned(),
        })
    }

    fn instance(&self) -> u8 {
        self.instance
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use super::GazeboSimBackend;
    use aviate_hal_xil::{
        DirectiveId, ResetGeneration, SimulatorBackend, SimulatorDirective, SimulatorDirectiveKind,
        SimulatorError, SimulatorOperation,
    };
    use std::time::Duration;

    #[test]
    fn new_backend_has_the_requested_instance() {
        assert_eq!(GazeboSimBackend::new(3).instance(), 3);
    }

    #[test]
    fn reset_fails_without_a_world_acknowledgement() {
        let mut backend = GazeboSimBackend::new(0);
        let error = backend
            .execute(
                SimulatorDirective {
                    id: DirectiveId(1),
                    generation: ResetGeneration::INITIAL,
                    kind: SimulatorDirectiveKind::Reset,
                },
                Duration::from_secs(1),
            )
            .expect_err("reset must fail closed");
        assert!(matches!(
            error,
            SimulatorError::NotAvailable {
                operation: SimulatorOperation::Reset,
                ..
            }
        ));
    }
}
