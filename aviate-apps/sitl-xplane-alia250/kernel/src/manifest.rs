//! Machine-readable identity for each Alia X-Plane run.

use std::string::String;
use std::{fmt, path::PathBuf};

use aviate_config::airframe_preset::{CandidateIdentity, ContentDigest};

use crate::{AliaKernel, CalibrationRunManifest};

mod source_identity;
use source_identity::application_source_identity;

const MANIFEST_SCHEMA_VERSION: u16 = 2;
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
    /// Optional candidate name.
    pub candidate_id: Option<String>,
    /// Optional candidate and plant identities.
    pub candidate_identity: Option<CandidateIdentity>,
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
            | Self::CandidatePurposeMismatch => None,
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
        simulator_model: ContentDigest,
        purpose: RunPurpose,
        candidate: Option<&CalibrationRunManifest>,
        build: BuildIdentity,
        runtime_handshake: ContentDigest,
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
        if candidate.is_some_and(|value| value.simulator_model != simulator_model) {
            return Err(RunManifestError::CandidateModelMismatch);
        }
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
            simulator_model,
            runtime_handshake,
            algorithm_identity_hash: kernel.pipeline().algorithm_identity_hash(),
            kernel_config_hash: kernel.cfg().canonical_hash(),
            candidate_id,
            candidate_identity,
        })
    }

    /// Encode the manifest as strict TOML text.
    #[must_use]
    pub fn to_toml(&self) -> String {
        let mut text = format!(
            "schema_version = {}\napplication_id = {:?}\nbuild_identity = {:?}\nsource_identity = {:?}\nlock_identity = {:?}\nbuild_target = {:?}\nbuild_profile = {:?}\nbuild_compiler = {:?}\nbuild_features = {:?}\npurpose = {:?}\nbase_preset_digest = {:?}\nsimulator_model_digest = {:?}\nruntime_handshake_digest = {:?}\nalgorithm_identity_hash = {:?}\nkernel_config_hash = {:?}\n",
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
        );
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
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use super::*;

    #[test]
    fn normal_manifest_contains_all_non_candidate_identities() {
        let kernel = crate::build_alia250_kernel().expect("valid kernel");
        let model = ContentDigest::calculate(b"model");
        let runtime = ContentDigest::calculate(b"runtime");
        let build = BuildIdentity::current().expect("build identity");
        let manifest =
            AliaRunManifest::new(&kernel, model, RunPurpose::Normal, None, build, runtime)
                .expect("manifest");
        let text = manifest.to_toml();
        assert!(text.contains("application_id = \"sitl-xplane-alia250\""));
        assert!(text.contains(&format!("simulator_model_digest = \"{model}\"")));
        assert!(text.contains(&format!("runtime_handshake_digest = \"{runtime}\"")));
        assert!(text.contains("algorithm_identity_hash = \""));
        assert!(!text.contains("candidate_id"));
    }

    #[test]
    fn candidate_manifest_must_match_the_kernel_and_model() {
        let kernel = crate::build_alia250_kernel().expect("valid kernel");
        let model = ContentDigest::calculate(b"model");
        let runtime = ContentDigest::calculate(b"runtime");
        let identity = CandidateIdentity {
            base_preset: ContentDigest::calculate(crate::construct::ALIA250_PRESET.as_bytes()),
            candidate: ContentDigest::calculate(b"candidate"),
            plant_artifact: ContentDigest::calculate(b"plant"),
            lineage: ContentDigest::calculate(b"lineage"),
        };
        let build = BuildIdentity::current().expect("build identity");
        let mut candidate = CalibrationRunManifest {
            candidate_id: "candidate-a".to_owned(),
            identity,
            simulator_model: model,
            kernel_config_hash: kernel.cfg().canonical_hash().wrapping_add(1),
        };
        let mut wrong_base = candidate.clone();
        wrong_base.identity.base_preset = ContentDigest::calculate(b"wrong-base");
        assert!(matches!(
            AliaRunManifest::new(
                &kernel,
                model,
                RunPurpose::Candidate,
                Some(&wrong_base),
                build.clone(),
                runtime,
            ),
            Err(RunManifestError::CandidateBaseMismatch)
        ));
        assert!(matches!(
            AliaRunManifest::new(
                &kernel,
                model,
                RunPurpose::Normal,
                Some(&candidate),
                build.clone(),
                runtime,
            ),
            Err(RunManifestError::CandidatePurposeMismatch)
        ));
        assert!(matches!(
            AliaRunManifest::new(
                &kernel,
                model,
                RunPurpose::Candidate,
                Some(&candidate),
                build.clone(),
                runtime,
            ),
            Err(RunManifestError::CandidateKernelMismatch)
        ));
        candidate.kernel_config_hash = kernel.cfg().canonical_hash();
        candidate.simulator_model = ContentDigest::calculate(b"other-model");
        assert!(matches!(
            AliaRunManifest::new(
                &kernel,
                model,
                RunPurpose::Candidate,
                Some(&candidate),
                build,
                runtime,
            ),
            Err(RunManifestError::CandidateModelMismatch)
        ));
    }
}
