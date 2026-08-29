//! Live authorization for direct and inbound X-Plane Arm requests.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use aviate_core::ArmError;
use aviate_hal_xil::perturbation::{LiveArtifactGuard, PerturbationCapability};

#[cfg(test)]
mod tests;

use super::config::PerturbationBinding;
use super::XPlaneBoard;

pub(super) struct XPlaneArmAuthorizer {
    pub(super) runtime_identity_ready: bool,
    pub(super) tuning_trace_ready: bool,
    pub(super) perturbation_ready: bool,
    pub(super) perturbation_configured: bool,
    pub(super) binding: Option<PerturbationBinding>,
    pub(super) hover_scale_basis_points: u16,
    pub(super) artifact_guard: Option<LiveArtifactGuard>,
    pub(super) artifact_failure: Arc<AtomicBool>,
}

impl XPlaneArmAuthorizer {
    /// Compares the run manifest identity with the loaded artifact identity.
    fn manifest_identity_ready(&self) -> bool {
        self.binding
            .as_ref()
            .is_some_and(|binding| binding.artifact == binding.manifest)
    }

    /// Compares the negotiated capability set with the set this run executes.
    ///
    /// The set is derived again from the live factors and the hover scale the
    /// kernel was built with, so a declared capability that the run does not
    /// execute, or a factor the artifact never declared, refuses the request.
    fn capability_identity_ready(&self) -> bool {
        self.binding.as_ref().is_some_and(|binding| {
            let declared = sorted(&binding.artifact.required_capabilities);
            let executed = sorted(
                &binding
                    .config
                    .executed_capabilities(self.hover_scale_basis_points),
            );
            declared == executed
        })
    }

    /// Re-reads the artifact file and compares its hash with the loaded bytes.
    fn artifact_hash_ready(&self) -> bool {
        self.artifact_guard.as_ref().is_some_and(|guard| {
            let verified = guard.verify().is_ok();
            if !verified {
                self.artifact_failure.store(true, Ordering::Release);
            }
            verified
        })
    }
}

fn sorted(capabilities: &[PerturbationCapability]) -> Vec<PerturbationCapability> {
    let mut sorted = capabilities.to_vec();
    sorted.sort_unstable();
    sorted
}

impl aviate_runtime::ArmAuthorizer for XPlaneArmAuthorizer {
    fn authorize_arm(&self) -> Result<(), ArmError> {
        let condition_ready = if self.perturbation_configured {
            self.manifest_identity_ready()
                && self.capability_identity_ready()
                && self.artifact_hash_ready()
        } else {
            true
        };
        if self.runtime_identity_ready
            && self.tuning_trace_ready
            && self.perturbation_ready
            && condition_ready
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
            binding: self.perturbation_binding.clone(),
            hover_scale_basis_points: self.hover_initialization.scale_basis_points,
            artifact_guard: self.perturbation_guard.clone(),
            artifact_failure: Arc::clone(&self.artifact_failure),
        }
    }
}
