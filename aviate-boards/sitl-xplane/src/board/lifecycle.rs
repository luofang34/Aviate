//! X-Plane board lifecycle for one simulator session.

use aviate_config::xplane_model::XPlaneWireModel;
use aviate_hal_xil::perturbation::{PerturbationConfig, PerturbationEngine, PerturbationError};
use aviate_hal_xil::{BackendStatus, ResetGeneration, SimulatorLifecycle};

use super::{wire::WireConstraints, XPlaneBoard};

/// A reset request that the X-Plane session cannot start.
#[derive(Debug, thiserror::Error)]
pub enum XPlaneResetError {
    /// Another reset request is not complete.
    #[error("reset generation {generation:?} is still in progress")]
    ResetInProgress {
        /// Generation that waits for an acknowledgment.
        generation: ResetGeneration,
    },
    /// The condition engine cannot start a new generation.
    #[error("the condition engine cannot reset: {source}")]
    Condition {
        /// Source error.
        #[source]
        source: PerturbationError,
    },
    /// The HIL stream did not accept the reset request.
    #[error("the HIL stream cannot request a reset: {source}")]
    Transport {
        /// Source error.
        #[source]
        source: std::io::Error,
    },
}

pub(super) struct XPlaneSessionState {
    generation: ResetGeneration,
    lifecycle: SimulatorLifecycle,
    wire_model: XPlaneWireModel,
    sample_dt_sec: f32,
    perturbation: Option<PerturbationConfig>,
}

impl XPlaneSessionState {
    pub(super) fn new(
        wire_model: XPlaneWireModel,
        sample_rate_hz: u16,
        perturbation: Option<PerturbationConfig>,
    ) -> Self {
        Self {
            generation: ResetGeneration::INITIAL,
            lifecycle: SimulatorLifecycle::Converging,
            wire_model,
            sample_dt_sec: 1.0 / f32::from(sample_rate_hz),
            perturbation,
        }
    }

    pub(super) fn is_stopped(&self) -> bool {
        self.lifecycle == SimulatorLifecycle::Stopped
    }

    pub(super) fn observe_health(&mut self, ready: bool, healthy: bool) {
        match self.lifecycle {
            SimulatorLifecycle::Converging if ready => {
                self.lifecycle = SimulatorLifecycle::Ready;
            }
            SimulatorLifecycle::Ready if !healthy => {
                self.lifecycle = SimulatorLifecycle::Converging;
            }
            _ => {}
        }
    }

    fn begin_reset(&mut self) {
        self.lifecycle = SimulatorLifecycle::Resetting;
    }

    fn accept_reset_request(&mut self) -> ResetGeneration {
        self.generation = self.generation.next();
        self.generation
    }

    pub(super) fn reset_acknowledged(&mut self) {
        if self.lifecycle == SimulatorLifecycle::Resetting {
            self.lifecycle = SimulatorLifecycle::Converging;
        }
    }

    pub(super) fn is_resetting(&self) -> bool {
        self.lifecycle == SimulatorLifecycle::Resetting
    }
}

impl<C, M> XPlaneBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Return one coherent lifecycle status.
    #[must_use]
    pub fn lifecycle_status(&self) -> BackendStatus {
        BackendStatus {
            generation: self.session.generation,
            lifecycle: self.session.lifecycle,
            step: self.sample_sequence,
            simulation_time: std::time::Duration::from_micros(
                self.last_sample_time_us.unwrap_or(0),
            ),
            armed: self.is_armed(),
        }
    }

    /// Start sample processing.
    pub fn start_session(&mut self) {
        if self.session.lifecycle == SimulatorLifecycle::Stopped {
            self.session.lifecycle = if self.is_ready() {
                SimulatorLifecycle::Ready
            } else {
                SimulatorLifecycle::Converging
            };
        }
    }

    /// Stop sample processing and make outputs safe.
    pub fn stop_session(&mut self) {
        self.terminate();
        self.session.lifecycle = SimulatorLifecycle::Stopped;
    }

    /// Clear all state for a new reset generation.
    ///
    /// # Errors
    ///
    /// Returns an error if the condition or HIL stream cannot reset.
    pub fn begin_reset(&mut self) -> Result<ResetGeneration, XPlaneResetError> {
        if self.session.is_resetting() || self.hil_backend.reset_request_pending() {
            return Err(XPlaneResetError::ResetInProgress {
                generation: self.session.generation,
            });
        }
        self.session.begin_reset();
        self.terminate();
        self.runner.reset_for_simulator_generation();
        self.armed = false;
        self.last_fix = None;
        self.last_imu = None;
        self.lane_injection = [0.0; 4];
        self.wire = WireConstraints::new(self.session.wire_model);
        self.last_packet_at = None;
        self.last_sample_time_us = None;
        self.sample_dt_sec = self.session.sample_dt_sec;
        self.runtime_identity.reset_sample_evidence();
        self.runtime_failure = None;
        self.control_observations.clear();
        self.perturbation_failure = None;
        self.perturbation = self
            .session
            .perturbation
            .clone()
            .map(PerturbationEngine::new)
            .transpose()
            .map_err(|source| XPlaneResetError::Condition { source })?;
        self.artifact_failure
            .store(false, std::sync::atomic::Ordering::Release);
        self.sample_sequence = 0;
        self.last_answer_armed = None;
        self.hil_backend
            .request_reset()
            .map_err(|source| XPlaneResetError::Transport { source })?;
        Ok(self.session.accept_reset_request())
    }

    /// Return the TCP transport counters.
    #[must_use]
    pub fn stats(&self) -> (u64, u64, u64, u64, u64) {
        self.hil_backend.tcp_stats()
    }

    /// Send one heartbeat to the bridge.
    pub fn send_heartbeat(&mut self) {
        self.hil_backend.send_heartbeat(self.armed).ok();
    }

    /// Return microseconds on the board clock.
    #[must_use]
    pub fn now_us(&self) -> u64 {
        self.runner.now_us()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use aviate_config::xplane_model::XPlaneSimulatorModel;
    use aviate_hal_xil::{ResetGeneration, SimulatorLifecycle};

    use super::XPlaneSessionState;

    const MODEL: &str = include_str!("../../../../presets/alia250-xplane.toml");

    #[test]
    fn reset_generation_converges_before_ready() {
        let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
        let mut session = XPlaneSessionState::new(model.wire(), model.sample_rate_hz(), None);
        session.observe_health(true, true);
        assert_eq!(session.lifecycle, SimulatorLifecycle::Ready);

        session.begin_reset();
        assert_eq!(session.lifecycle, SimulatorLifecycle::Resetting);
        assert_eq!(session.accept_reset_request(), ResetGeneration::new(2));
        assert_eq!(session.lifecycle, SimulatorLifecycle::Resetting);
        session.reset_acknowledged();
        assert_eq!(session.lifecycle, SimulatorLifecycle::Converging);

        session.observe_health(false, true);
        assert_eq!(session.lifecycle, SimulatorLifecycle::Converging);
        session.observe_health(true, true);
        assert_eq!(session.lifecycle, SimulatorLifecycle::Ready);
        session.observe_health(false, false);
        assert_eq!(session.lifecycle, SimulatorLifecycle::Converging);
    }
}
