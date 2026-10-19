//! The board's simulator session: its lifecycle, its reset, and the
//! flight controller's arm state.

use std::time::Duration;

use aviate_core::control::Command;
use aviate_core::hal::{ActuatorHal, SystemHal};
use aviate_core::{ArmError, DisarmError, InitState};
use aviate_hal_xil::{BackendStatus, ResetGeneration, SimulatorLifecycle, VehicleState};
use log::info;

use super::JsbsimBoard;

impl<C, M> JsbsimBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// One coherent view of the session: its world generation,
    /// lifecycle, step and simulation time.
    #[must_use]
    pub fn lifecycle_status(&self) -> BackendStatus {
        BackendStatus {
            generation: self.reset_generation(),
            lifecycle: self.lifecycle,
            step: self.step_index,
            simulation_time: Duration::from_micros(self.time_us),
            armed: self.is_armed(),
        }
    }

    /// The vehicle the latest truth state states, in the world origin's
    /// NED frame.
    #[must_use]
    pub fn vehicle(&self) -> &VehicleState {
        &self.vehicle
    }

    /// Start running the kernel on truth states.
    pub fn start_session(&mut self) {
        if self.lifecycle == SimulatorLifecycle::Stopped {
            self.lifecycle = SimulatorLifecycle::Converging;
            self.observe_health();
        }
    }

    /// Stop running the kernel; truth states are answered idle.
    pub fn stop_session(&mut self) {
        self.terminate();
        self.lifecycle = SimulatorLifecycle::Stopped;
    }

    /// End the sidecar session and dial a new one: the sidecar starts a
    /// new world, with the vehicle back at its start, for each session.
    /// The session waits in `Resetting` until a state of the new world
    /// arrives.
    ///
    /// # Errors
    ///
    /// The transport's refusal to end the session.
    pub fn begin_reset(&mut self) -> std::io::Result<()> {
        self.terminate();
        self.vehicle = VehicleState::default();
        self.lifecycle = SimulatorLifecycle::Resetting;
        self.backend.restart_session()
    }

    /// Arm the flight controller.
    ///
    /// # Errors
    ///
    /// The kernel's refusal, so a harness sees a refused arm rather than
    /// a silent no-op.
    pub fn arm(&mut self) -> Result<(), ArmError> {
        self.check_arm_readiness()?;
        info!(
            "Arm command (state={:?})",
            self.runner.kernel.state.init_state
        );
        self.runner.kernel.arm()?;
        self.runner.board_hal.arm();
        self.runner.transport.set_armed(true);
        Ok(())
    }

    /// The live conditions that permit an arm.
    ///
    /// # Errors
    ///
    /// The current arm refusal.
    pub fn check_arm_readiness(&self) -> Result<(), ArmError> {
        if self.runner.kernel.state.init_state == InitState::Armed {
            return Err(ArmError::AlreadyArmed);
        }
        if !self.runner.kernel.is_ready() {
            return Err(ArmError::NotReady);
        }
        if !self.runner.kernel.state.faults.is_empty() {
            return Err(ArmError::Faulted);
        }
        Ok(())
    }

    /// Disarm the flight controller.
    ///
    /// # Errors
    ///
    /// The kernel's refusal (an in-flight disarm, for example).
    pub fn disarm(&mut self) -> Result<(), DisarmError> {
        self.runner.kernel.disarm()?;
        self.runner.board_hal.disarm();
        self.runner.transport.set_armed(false);
        Ok(())
    }

    /// Cut outputs immediately, in any flight phase.
    pub fn terminate(&mut self) {
        self.runner.kernel.terminate();
        self.runner.board_hal.disarm();
        self.runner.transport.set_armed(false);
    }

    /// Set the flight command, through the shared ingress so the
    /// setpoint carries a real receive time.
    pub fn set_command(&mut self, command: Command) {
        self.runner.kernel.state.checks.pre_arm.update_throttle(
            command.setpoint.collective_thrust
                < aviate_core::kernel_types::THROTTLE_LOW_MAX_COLLECTIVE,
        );
        let now_ticks = self.runner.transport.now().ticks;
        self.runner.ingress.receive(
            aviate_hal_io::SystemCommand::FlightControl(command),
            now_ticks,
        );
    }

    /// A state of a new world arrived: the runner ground-resets on its
    /// first sample, and the session converges again.
    pub(super) fn enter_new_world(&mut self) {
        if self.lifecycle != SimulatorLifecycle::Stopped {
            self.lifecycle = SimulatorLifecycle::Converging;
        }
    }

    /// Ready once the kernel is ready to fly; back to converging when it
    /// is neither ready nor flying.
    pub(super) fn observe_health(&mut self) {
        let ready = self.runner.kernel.is_ready();
        match self.lifecycle {
            SimulatorLifecycle::Converging if ready => self.lifecycle = SimulatorLifecycle::Ready,
            SimulatorLifecycle::Ready if !ready && !self.is_armed() => {
                self.lifecycle = SimulatorLifecycle::Converging;
            }
            _ => {}
        }
    }

    fn reset_generation(&self) -> ResetGeneration {
        ResetGeneration::new(
            ResetGeneration::INITIAL
                .get()
                .wrapping_add(self.generation.unwrap_or(0)),
        )
    }
}
