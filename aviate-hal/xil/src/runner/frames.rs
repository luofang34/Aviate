//! Validate simulator frames for mission execution.

use std::time::{Duration, Instant};

use crate::{
    FrameEvent, SimulatorBackend, SimulatorError, SimulatorFrame, SimulatorLifecycle,
    SimulatorOperation,
};

use super::MissionRunner;

impl<B: SimulatorBackend> MissionRunner<B> {
    pub(super) fn next_frame(
        &mut self,
        timeout: Duration,
    ) -> Result<SimulatorFrame, SimulatorError> {
        match self.backend.next_frame(timeout)? {
            FrameEvent::Frame(frame) if frame.generation == self.generation => Ok(frame),
            FrameEvent::Frame(frame) => Err(SimulatorError::StaleGeneration {
                expected: self.generation,
                received: frame.generation,
            }),
            FrameEvent::TimedOut {
                generation,
                timeout,
                ..
            } => Err(SimulatorError::Timeout {
                operation: SimulatorOperation::NextFrame,
                generation,
                timeout,
            }),
        }
    }

    pub(super) fn next_ready_frame(
        &mut self,
        timeout: Duration,
    ) -> Result<SimulatorFrame, SimulatorError> {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(SimulatorError::Timeout {
                    operation: SimulatorOperation::NextFrame,
                    generation: self.generation,
                    timeout,
                });
            }
            let frame = self.next_frame(remaining)?;
            match frame.lifecycle {
                SimulatorLifecycle::Ready if frame.vehicle.valid => return Ok(frame),
                SimulatorLifecycle::Resetting | SimulatorLifecycle::Converging => {}
                lifecycle => {
                    return Err(SimulatorError::InvalidLifecycle {
                        operation: SimulatorOperation::NextFrame,
                        lifecycle,
                    });
                }
            }
        }
    }

    pub(super) fn accept_phase_frame(
        &mut self,
        frame: &SimulatorFrame,
        disarm_settling: bool,
    ) -> Result<(), SimulatorError> {
        let lifecycle_accepted = frame.lifecycle == SimulatorLifecycle::Ready
            || (disarm_settling && frame.lifecycle == SimulatorLifecycle::Converging);
        let detail = if frame.step <= self.last_step {
            Some("the simulation step did not advance")
        } else if frame.simulation_time <= self.last_simulation_time {
            Some("the simulation time did not advance")
        } else if !lifecycle_accepted {
            Some("the backend left the ready lifecycle")
        } else if frame.armed != self.armed {
            self.armed = frame.armed;
            Some("the frame arm state does not match the directive receipt")
        } else {
            None
        };
        if let Some(detail) = detail {
            return Err(SimulatorError::ReadinessFailed {
                generation: frame.generation,
                detail: detail.to_owned(),
            });
        }
        self.last_step = frame.step;
        self.last_simulation_time = frame.simulation_time;
        self.armed = frame.armed;
        Ok(())
    }
}
