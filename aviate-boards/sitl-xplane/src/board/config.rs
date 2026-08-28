//! X-Plane board configuration.

use std::io;
use std::net::SocketAddr;

use aviate_config::xplane_model::XPlaneSimulatorModel;
use aviate_core::DefaultAviateKernel;
use aviate_hal_xil::perturbation::{
    LiveArtifactGuard, LoadedPerturbationArtifact, PerturbationArtifactIdentity, PerturbationConfig,
};
use aviate_runtime::SitlBoardInfo;

use super::XPlaneTuningTraceConfig;

/// Board configuration.
#[derive(Debug, Clone)]
pub struct XPlaneConfig {
    /// The bridge plugin's listening address the board dials.
    pub simulator_addr: SocketAddr,
    /// System ID for outgoing messages.
    pub sys_id: u8,
    /// Component ID for outgoing messages.
    pub comp_id: u8,
    /// Versioned plant-protection and actuator-lane model.
    pub model: XPlaneSimulatorModel,
    pub(super) tuning_trace: Option<XPlaneTuningTraceConfig>,
    pub(super) perturbation: Option<PerturbationConfig>,
    pub(super) perturbation_guard: Option<LiveArtifactGuard>,
    pub(super) perturbation_identity_bound: bool,
    pub(super) hover_initialization: Option<XPlaneHoverInitialization>,
}

impl XPlaneConfig {
    /// Build a bridge configuration for one validated simulator model.
    #[must_use]
    pub fn new(simulator_addr: SocketAddr, model: XPlaneSimulatorModel) -> Self {
        Self {
            simulator_addr,
            sys_id: 1,
            comp_id: 1,
            model,
            tuning_trace: None,
            perturbation: None,
            perturbation_guard: None,
            perturbation_identity_bound: true,
            hover_initialization: None,
        }
    }

    /// Enable one required packet-synchronous tuning trace.
    #[must_use]
    pub fn with_tuning_trace(mut self, config: XPlaneTuningTraceConfig) -> Self {
        self.tuning_trace = Some(config);
        self
    }

    /// Enable one identity-bound calibration perturbation.
    #[must_use]
    pub fn with_perturbation(mut self, config: PerturbationConfig) -> Self {
        self.perturbation = Some(config);
        self.perturbation_identity_bound = false;
        self
    }

    /// Enable one artifact-backed perturbation bound to the run manifest.
    ///
    /// # Errors
    ///
    /// Returns an error when the run manifest names another artifact.
    pub fn with_verified_perturbation(
        mut self,
        artifact: &LoadedPerturbationArtifact,
        manifest_identity: &PerturbationArtifactIdentity,
    ) -> Result<Self, XPlanePerturbationBindingError> {
        if artifact.identity() != manifest_identity {
            return Err(XPlanePerturbationBindingError::ManifestIdentityMismatch);
        }
        self.perturbation = Some(artifact.config().clone());
        self.perturbation_guard = Some(artifact.live_guard());
        self.perturbation_identity_bound = true;
        Ok(self)
    }

    /// Bind immutable hover initialization evidence to each observation.
    #[must_use]
    pub fn with_hover_initialization(mut self, evidence: XPlaneHoverInitialization) -> Self {
        self.hover_initialization = Some(evidence);
        self
    }
}

/// Immutable hover initialization at kernel construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct XPlaneHoverInitialization {
    /// Preset force-domain baseline as IEEE-754 bits.
    pub baseline_force_bits: u32,
    /// Effective force-domain value as IEEE-754 bits.
    pub effective_force_bits: u32,
    /// Applied baseline scale in basis points.
    pub scale_basis_points: u16,
    /// Canonical hash of the effective kernel configuration.
    pub kernel_config_hash: u64,
}

/// A condition artifact does not match another run identity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum XPlanePerturbationBindingError {
    /// The run manifest contains another artifact identity.
    ManifestIdentityMismatch,
}

impl core::fmt::Display for XPlanePerturbationBindingError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ManifestIdentityMismatch => {
                formatter.write_str("condition artifact does not match the run manifest")
            }
        }
    }
}

impl std::error::Error for XPlanePerturbationBindingError {}

pub(super) fn validate_hover_initialization<C, M>(
    kernel: &DefaultAviateKernel<C, M>,
    evidence: XPlaneHoverInitialization,
) -> io::Result<()>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let baseline = f32::from_bits(evidence.baseline_force_bits);
    if !baseline.is_finite()
        || baseline <= 0.0
        || evidence.effective_force_bits != kernel.cfg().hover_thrust_norm.0.to_bits()
        || evidence.kernel_config_hash != kernel.cfg().canonical_hash()
        || !(8_000..=12_000).contains(&evidence.scale_basis_points)
    {
        Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "hover initialization does not match the kernel",
        ))
    } else {
        Ok(())
    }
}

/// Board info for the X-Plane SITL board.
pub const BOARD_INFO: SitlBoardInfo = SitlBoardInfo {
    name: "sitl-xplane",
    description: "X-Plane SITL via the MAVLink HIL bridge over TCP",
};
