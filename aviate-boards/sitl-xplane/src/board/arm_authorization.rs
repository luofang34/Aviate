//! Live authorization for direct and inbound X-Plane Arm requests.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use aviate_core::ArmError;
use aviate_hal_xil::perturbation::LiveArtifactGuard;

use super::XPlaneBoard;

pub(super) struct XPlaneArmAuthorizer {
    pub(super) runtime_identity_ready: bool,
    pub(super) tuning_trace_ready: bool,
    pub(super) perturbation_ready: bool,
    pub(super) perturbation_configured: bool,
    pub(super) perturbation_identity_bound: bool,
    pub(super) artifact_guard: Option<LiveArtifactGuard>,
    pub(super) artifact_failure: Arc<AtomicBool>,
}

impl aviate_runtime::ArmAuthorizer for XPlaneArmAuthorizer {
    fn authorize_arm(&self) -> Result<(), ArmError> {
        let artifact_ready = if self.perturbation_configured {
            self.perturbation_identity_bound
                && self.artifact_guard.as_ref().is_some_and(|guard| {
                    let verified = guard.verify().is_ok();
                    if !verified {
                        self.artifact_failure.store(true, Ordering::Release);
                    }
                    verified
                })
        } else {
            true
        };
        if self.runtime_identity_ready
            && self.tuning_trace_ready
            && self.perturbation_ready
            && artifact_ready
        {
            Ok(())
        } else {
            Err(ArmError::NotReady)
        }
    }
}

impl<C, M> XPlaneBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    pub(super) fn arm_authorizer(&self) -> XPlaneArmAuthorizer {
        XPlaneArmAuthorizer {
            runtime_identity_ready: self.runtime_identity.is_verified(),
            tuning_trace_ready: self
                .tuning_trace
                .as_ref()
                .is_none_or(super::tuning_trace::TuningTracePublisher::is_ready),
            perturbation_ready: self.perturbation_failure.is_none(),
            perturbation_configured: self.perturbation.is_some(),
            perturbation_identity_bound: self.perturbation_identity_bound,
            artifact_guard: self.perturbation_guard.clone(),
            artifact_failure: Arc::clone(&self.artifact_failure),
        }
    }
}
