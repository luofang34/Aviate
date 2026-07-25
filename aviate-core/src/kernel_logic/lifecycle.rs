//! Armed-state transitions: the guarded entry to and exit from flight.
//!
//! Split from `kernel_logic.rs` to keep each file under the 500-line
//! cap. Grouped by what they decide rather than by call order: `arm`,
//! `disarm`, and `terminate` are the three ways the armed state
//! changes, and they share the precondition vocabulary and the
//! flight-period teardown.

use crate::control::{ControlLawV1, VehicleController};
use crate::ekf::Estimator;
use crate::kernel::{AviateKernelImpl, InitState};
use crate::kernel_types::{ArmError, DisarmError, TerminalCause};
use crate::mixer::{ActuatorSanitizer, Mixer};

impl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer>
    AviateKernelImpl<E, V, M, S>
{
    pub fn is_ready(&self) -> bool {
        self.state.init_state == InitState::Ready
    }

    pub fn arm(&mut self) -> Result<(), ArmError> {
        if self.state.init_state == InitState::Armed {
            return Err(ArmError::AlreadyArmed);
        }
        if self.state.init_state != InitState::Ready {
            return Err(ArmError::NotReady);
        }
        if !self.state.faults.is_empty() {
            return Err(ArmError::Faulted);
        }

        // Capture the datum this flight period's height is measured
        // against. Read through the same trait surface the rest of the
        // kernel uses, so the airborne determination sees exactly the
        // estimate the control loop does.
        let estimate = self.pipeline.estimator.estimate(&self.state.estimator);
        self.state.flight_phase.begin_flight_period(&estimate);

        self.state.init_state = InitState::Armed;
        Ok(())
    }

    /// Ordinary disarm: end the flight period and cut to the safe
    /// actuator pattern.
    ///
    /// Refused with [`DisarmError::Airborne`] while the vehicle is
    /// holding itself up. The refusal leaves every piece of state
    /// untouched, so a refused disarm is observably a no-op rather than
    /// a partial transition. [`Self::terminate`] is the path that cuts
    /// outputs regardless of phase.
    pub fn disarm(&mut self) -> Result<(), DisarmError> {
        if self.state.flight_phase.is_airborne() {
            return Err(DisarmError::Airborne);
        }
        self.end_flight_period();
        Ok(())
    }

    /// Emergency terminate: cut outputs immediately, in any phase.
    ///
    /// This is the deliberate motor cut, for the cases where cutting
    /// power *is* the correct action — fly-away, imminent injury. It is
    /// separate from ordinary disarm at the command level precisely so
    /// that an ordinary control can never become a kill: reaching it
    /// requires a distinct command, not a differently-timed press of the
    /// same one.
    ///
    /// Infallible by design. A terminate that could be refused would be
    /// useless in the situations it exists for.
    pub fn terminate(&mut self) {
        self.end_flight_period();
    }

    /// Shared tail of both paths: leave `Armed`, drop to the backup law,
    /// and invalidate everything accumulated during the flight period.
    fn end_flight_period(&mut self) {
        self.state.init_state = InitState::Disarmed;
        self.state.control_law = ControlLawV1::Backup; // Was Frozen, now Backup
        self.state.terminal_cause = TerminalCause::None;
        self.state.checks.in_flight.reset();
        self.state.flight_phase.reset();
        // Reset controller persistent runtime state — disarm
        // invalidates accumulated integrators / anti-windup / mode
        // latches the same way ground_reset does (LLR-CTL-101).
        self.pipeline.controller.reset(&mut self.state.controller);
    }
}
