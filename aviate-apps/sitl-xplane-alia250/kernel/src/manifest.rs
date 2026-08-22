//! Machine-readable identity for each Alia X-Plane run.

use std::string::String;
use std::{fmt, path::PathBuf};

use aviate_config::airframe_preset::{CandidateIdentity, ContentDigest};
use aviate_hal_xil::perturbation::PerturbationArtifactIdentity;

use crate::{AliaKernel, CalibrationRunManifest, HoverInitializationEvidence};

mod source_identity;
use source_identity::application_source_identity;

const MANIFEST_SCHEMA_VERSION: u16 = 3;
const APPLICATION_ID: &str = "sitl-xplane-alia250";

/// The purpose of one flight-controller run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunPurpose {
    /// Normal flight with the shipped preset.
    Normal,
    /// Candidate evaluation.
    Candidate,
    /// Plant-identification flight.
    Identify,
    /// Collective sweep.
    Sweep,
    /// Mixer response-sign probe.
    YawSign,
}

impl RunPurpose {
    fn as_str(self) -> &'static str {
        match self {
            Self::Normal => "normal",
            Self::Candidate => "candidate",
            Self::Identify => "identify",
            Self::Sweep => "sweep",
            Self::YawSign => "yaw_sign",
        }
    }
}

/// Complete immutable identity for one Alia run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AliaRunManifest {
    /// Manifest schema version.
    pub schema_version: u16,
    /// Application identity.
    pub application_id: &'static str,
    /// Identity of the application-owned build inputs.
    pub build_identity: ContentDigest,
    /// Identity of the application source bundle.
    pub source_identity: ContentDigest,
    /// Identity of the exact dependency lock file.
    pub lock_identity: ContentDigest,
    /// Rust target used by the executable.
    pub build_target: String,
    /// Cargo profile class used by the executable.
    pub build_profile: &'static str,
    /// Rust compiler release used by the executable.
    pub build_compiler: &'static str,
    /// Flight-relevant Cargo feature set.
    pub build_features: &'static str,
    /// Run purpose.
    pub purpose: RunPurpose,
    /// Embedded base preset identity.
    pub base_preset: ContentDigest,
    /// Simulator plant-protection model identity.
    pub simulator_model: ContentDigest,
    /// Exact verified runtime handshake document identity.
    pub runtime_handshake: ContentDigest,
    /// Flight algorithm identity.
    pub algorithm_identity_hash: u64,
    /// Resolved kernel configuration identity.
    pub kernel_config_hash: u64,
    /// Canonical fold state immediately before the effective hover force.
    pub hover_kernel_prefix_hash: u64,
    /// Immutable force-domain hover initialization.
    pub hover_initialization: HoverInitializationEvidence,
    /// Optional condition artifact identity for a perturbation run.
    pub perturbation: Option<ManifestPerturbationIdentity>,
    perturbation_artifact: Option<PerturbationArtifactIdentity>,
    /// Optional candidate name.
    pub candidate_id: Option<String>,
    /// Optional candidate and plant identities.
    pub candidate_identity: Option<CandidateIdentity>,
}

/// Condition artifact identity encoded into one run manifest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ManifestPerturbationIdentity {
    /// Artifact path supplied by the harness.
    pub artifact_path: String,
    /// SHA-256 of the exact artifact bytes.
    pub artifact_sha256: String,
    /// SHA-256 of canonical condition JSON.
    pub condition_digest: String,
    /// Run seed supplied by the harness.
    pub run_seed: u64,
    /// Exact Aviate-owned capability names.
    pub required_capabilities: Vec<&'static str>,
}

/// Simulator and condition identities used to build one run manifest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RunExecutionIdentity {
    /// Simulator plant-protection identity.
    pub simulator_model: ContentDigest,
    /// Exact verified runtime handshake identity.
    pub runtime_handshake: ContentDigest,
    /// Immutable force-domain hover initialization.
    pub hover_initialization: HoverInitializationEvidence,
    /// Optional verified condition artifact identity.
    pub perturbation: Option<PerturbationArtifactIdentity>,
}

/// Complete build provenance calculated from the running executable.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BuildIdentity {
    /// SHA-256 of the executable opened by the running process.
    pub executable: ContentDigest,
    /// SHA-256 of the embedded source bundle.
    pub source: ContentDigest,
    /// SHA-256 of `Cargo.lock`.
    pub lock: ContentDigest,
    /// Target operating system and architecture.
    pub target: String,
    /// Cargo profile class.
    pub profile: &'static str,
    /// Rust compiler release.
    pub compiler: &'static str,
    /// Flight-relevant features.
    pub features: &'static str,
}

/// Run-manifest construction failure.
#[derive(Debug)]
pub enum RunManifestError {
    /// The process executable path is not available.
    CurrentExecutable(std::io::Error),
    /// The process executable cannot be read.
    ReadExecutable {
        /// Executable path returned by the operating system.
        path: PathBuf,
        /// File read failure.
        source: std::io::Error,
    },
    /// Candidate configuration does not match the supplied kernel.
    CandidateKernelMismatch,
    /// Candidate simulator model does not match the run model.
    CandidateModelMismatch,
    /// Candidate base preset does not match the run base.
    CandidateBaseMismatch,
    /// Candidate presence does not match the run purpose.
    CandidatePurposeMismatch,
    /// Hover evidence does not match the checked kernel.
    HoverInitializationMismatch,
    /// The hover prefix does not reconstruct the checked kernel identity.
    HoverKernelIdentityMismatch,
    /// The condition artifact path cannot be encoded without loss.
    InvalidConditionArtifactPath(PathBuf),
    /// The condition artifact identity repeats one capability.
    DuplicateConditionCapability(&'static str),
}

impl fmt::Display for RunManifestError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CurrentExecutable(error) => {
                write!(formatter, "cannot locate executable: {error}")
            }
            Self::ReadExecutable { path, source } => {
                write!(formatter, "cannot read executable {path:?}: {source}")
            }
            Self::CandidateKernelMismatch => {
                formatter.write_str("candidate kernel identity does not match the run kernel")
            }
            Self::CandidateModelMismatch => {
                formatter.write_str("candidate simulator model does not match the run model")
            }
            Self::CandidateBaseMismatch => {
                formatter.write_str("candidate base preset does not match the run base")
            }
            Self::CandidatePurposeMismatch => {
                formatter.write_str("candidate presence does not match the run purpose")
            }
            Self::HoverInitializationMismatch => {
                formatter.write_str("hover initialization does not match the run kernel")
            }
            Self::HoverKernelIdentityMismatch => {
                formatter.write_str("hover kernel prefix does not reconstruct the run kernel")
            }
            Self::InvalidConditionArtifactPath(path) => {
                write!(formatter, "condition artifact path {path:?} is not UTF-8")
            }
            Self::DuplicateConditionCapability(capability) => {
                write!(formatter, "condition capability {capability} repeats")
            }
        }
    }
}

impl std::error::Error for RunManifestError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::CurrentExecutable(error) => Some(error),
            Self::ReadExecutable { source, .. } => Some(source),
            Self::CandidateKernelMismatch
            | Self::CandidateModelMismatch
            | Self::CandidateBaseMismatch
            | Self::CandidatePurposeMismatch
            | Self::HoverInitializationMismatch
            | Self::HoverKernelIdentityMismatch
            | Self::InvalidConditionArtifactPath(_)
            | Self::DuplicateConditionCapability(_) => None,
        }
    }
}

impl BuildIdentity {
    /// Hash the actual running executable and its embedded build inputs.
    ///
    /// # Errors
    ///
    /// Returns an error when the executable cannot be located or read.
    pub fn current() -> Result<Self, RunManifestError> {
        let path = std::env::current_exe().map_err(RunManifestError::CurrentExecutable)?;
        let bytes = std::fs::read(&path)
            .map_err(|source| RunManifestError::ReadExecutable { path, source })?;
        Ok(Self {
            executable: ContentDigest::calculate(&bytes),
            source: application_source_identity(),
            lock: ContentDigest::calculate(include_bytes!("../../../../Cargo.lock")),
            target: env!("AVIATE_BUILD_TARGET").to_owned(),
            profile: env!("AVIATE_BUILD_PROFILE"),
            compiler: env!("AVIATE_BUILD_COMPILER"),
            features: env!("AVIATE_BUILD_FEATURES"),
        })
    }
}

impl AliaRunManifest {
    /// Build a manifest from a checked kernel and its immutable inputs.
    pub fn new(
        kernel: &AliaKernel,
        purpose: RunPurpose,
        candidate: Option<&CalibrationRunManifest>,
        build: BuildIdentity,
        execution: RunExecutionIdentity,
    ) -> Result<Self, RunManifestError> {
        let base_preset = ContentDigest::calculate(crate::construct::ALIA250_PRESET.as_bytes());
        if (purpose == RunPurpose::Candidate) != candidate.is_some() {
            return Err(RunManifestError::CandidatePurposeMismatch);
        }
        if candidate.is_some_and(|value| value.identity.base_preset != base_preset) {
            return Err(RunManifestError::CandidateBaseMismatch);
        }
        if candidate.is_some_and(|value| value.kernel_config_hash != kernel.cfg().canonical_hash())
        {
            return Err(RunManifestError::CandidateKernelMismatch);
        }
        if candidate.is_some_and(|value| value.simulator_model != execution.simulator_model) {
            return Err(RunManifestError::CandidateModelMismatch);
        }
        validate_hover_initialization(kernel, execution.hover_initialization)?;
        let hover_kernel_prefix_hash = kernel.cfg().hover_kernel_prefix_hash();
        let reconstructed =
            aviate_core::kernel::config::ResolvedKernelConfig::canonical_hash_from_hover_prefix(
                hover_kernel_prefix_hash,
                kernel.cfg().hover_thrust_norm.0,
                kernel.cfg().mixer_geometry,
                kernel.cfg().actuator_curve,
            );
        if reconstructed != kernel.cfg().canonical_hash() {
            return Err(RunManifestError::HoverKernelIdentityMismatch);
        }
        let perturbation = execution
            .perturbation
            .as_ref()
            .map(ManifestPerturbationIdentity::try_from)
            .transpose()?;
        let perturbation_artifact = execution.perturbation;
        let (candidate_id, candidate_identity) = candidate.map_or((None, None), |value| {
            (Some(value.candidate_id.clone()), Some(value.identity))
        });
        Ok(Self {
            schema_version: MANIFEST_SCHEMA_VERSION,
            application_id: APPLICATION_ID,
            build_identity: build.executable,
            source_identity: build.source,
            lock_identity: build.lock,
            build_target: build.target,
            build_profile: build.profile,
            build_compiler: build.compiler,
            build_features: build.features,
            purpose,
            base_preset,
            simulator_model: execution.simulator_model,
            runtime_handshake: execution.runtime_handshake,
            algorithm_identity_hash: kernel.pipeline().algorithm_identity_hash(),
            kernel_config_hash: kernel.cfg().canonical_hash(),
            hover_kernel_prefix_hash,
            hover_initialization: execution.hover_initialization,
            perturbation,
            perturbation_artifact,
            candidate_id,
            candidate_identity,
        })
    }

    /// Encode the manifest as strict TOML text.
    #[must_use]
    pub fn to_toml(&self) -> String {
        let mut text = format!(
            "schema_version = {}\napplication_id = {:?}\nbuild_identity = {:?}\nsource_identity = {:?}\nlock_identity = {:?}\nbuild_target = {:?}\nbuild_profile = {:?}\nbuild_compiler = {:?}\nbuild_features = {:?}\npurpose = {:?}\nbase_preset_digest = {:?}\nsimulator_model_digest = {:?}\nruntime_handshake_digest = {:?}\nalgorithm_identity_hash = {:?}\nkernel_config_hash = {:?}\nhover_baseline_force_bits = {:?}\nhover_effective_force_bits = {:?}\nhover_scale_basis_points = {}\nhover_estimator_mode = {:?}\nhover_kernel_prefix_hash = {:?}\nhover_mixer_geometry = {:?}\nhover_actuator_curve = {:?}\nhover_kernel_config_hash = {:?}\n",
            self.schema_version,
            self.application_id,
            self.build_identity.to_string(),
            self.source_identity.to_string(),
            self.lock_identity.to_string(),
            self.build_target,
            self.build_profile,
            self.build_compiler,
            self.build_features,
            self.purpose.as_str(),
            self.base_preset.to_string(),
            self.simulator_model.to_string(),
            self.runtime_handshake.to_string(),
            format!("{:016x}", self.algorithm_identity_hash),
            format!("{:016x}", self.kernel_config_hash),
            format!("{:08x}", self.hover_initialization.baseline_force_bits),
            format!("{:08x}", self.hover_initialization.effective_force_bits),
            self.hover_initialization.scale_basis_points,
            self.hover_initialization.estimator_mode.as_str(),
            format!("{:016x}", self.hover_kernel_prefix_hash),
            "quad-x-x500-reversed-spin",
            "quadratic",
            format!("{:016x}", self.hover_kernel_config_hash()),
        );
        if let Some(identity) = &self.perturbation {
            text.push_str(&format!(
                "condition_artifact_path = {:?}\ncondition_artifact_sha256 = {:?}\ncondition_digest = {:?}\ncondition_run_seed = {}\ncondition_required_capabilities = {:?}\n",
                identity.artifact_path,
                identity.artifact_sha256,
                identity.condition_digest,
                identity.run_seed,
                identity.required_capabilities,
            ));
        }
        if let (Some(candidate_id), Some(identity)) = (&self.candidate_id, self.candidate_identity)
        {
            text.push_str(&format!(
                "candidate_id = {candidate_id:?}\ncandidate_digest = {:?}\nplant_artifact_digest = {:?}\ncandidate_lineage_digest = {:?}\n",
                identity.candidate.to_string(),
                identity.plant_artifact.to_string(),
                identity.lineage.to_string(),
            ));
        }
        text
    }

    /// Return the effective full kernel configuration identity.
    #[must_use]
    pub const fn hover_kernel_config_hash(&self) -> u64 {
        self.kernel_config_hash
    }

    /// Return the verified artifact identity encoded in this manifest.
    #[must_use]
    pub const fn perturbation_artifact_identity(&self) -> Option<&PerturbationArtifactIdentity> {
        self.perturbation_artifact.as_ref()
    }
}

impl TryFrom<&PerturbationArtifactIdentity> for ManifestPerturbationIdentity {
    type Error = RunManifestError;

    fn try_from(value: &PerturbationArtifactIdentity) -> Result<Self, Self::Error> {
        let artifact_path = value
            .artifact_path
            .to_str()
            .ok_or_else(|| {
                RunManifestError::InvalidConditionArtifactPath(value.artifact_path.clone())
            })?
            .to_owned();
        let mut required_capabilities = value
            .required_capabilities
            .iter()
            .map(|capability| capability.as_str())
            .collect::<Vec<_>>();
        required_capabilities.sort_unstable();
        if let Some(repeated) = required_capabilities
            .windows(2)
            .find(|pair| pair[0] == pair[1])
        {
            return Err(RunManifestError::DuplicateConditionCapability(repeated[0]));
        }
        Ok(Self {
            artifact_path,
            artifact_sha256: hex_digest(value.artifact_sha256),
            condition_digest: hex_digest(value.condition_digest),
            run_seed: value.run_seed,
            required_capabilities,
        })
    }
}

fn validate_hover_initialization(
    kernel: &AliaKernel,
    evidence: HoverInitializationEvidence,
) -> Result<(), RunManifestError> {
    if evidence.effective_force_bits != kernel.cfg().hover_thrust_norm.0.to_bits()
        || evidence.effective_kernel_config_hash != kernel.cfg().canonical_hash()
        || !(8_000..=12_000).contains(&evidence.scale_basis_points)
    {
        Err(RunManifestError::HoverInitializationMismatch)
    } else {
        Ok(())
    }
}

fn hex_digest(value: [u8; 32]) -> String {
    let mut encoded = String::with_capacity(64);
    for byte in value {
        encoded.push_str(&format!("{byte:02x}"));
    }
    encoded
}

#[cfg(test)]
mod tests;
