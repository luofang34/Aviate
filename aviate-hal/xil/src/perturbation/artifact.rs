//! Strict loading and live verification of one condition artifact.

mod schema;

use std::fs::{File, OpenOptions};
use std::io::{Read as _, Take};
use std::path::{Path, PathBuf};

use sha2::{Digest as _, Sha256};
use thiserror::Error;

use super::{PerturbationConfig, PerturbationEngine, PerturbationError};
use schema::ConditionSet;

const MAX_ARTIFACT_BYTES: u64 = 256 * 1024;

/// One calibration feature that Aviate executes from a condition artifact.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum PerturbationCapability {
    /// Deterministic bounded sensor noise.
    SensorPerturbation,
    /// Force-domain actuator authority scaling.
    ActuatorAuthority,
    /// Deterministic actuator command hold.
    CommandHold,
    /// Controller hover-force initialization scaling.
    HoverTrimUncertainty,
}

impl PerturbationCapability {
    /// Return the stable Pilotage capability name.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SensorPerturbation => "sensor_perturbation",
            Self::ActuatorAuthority => "actuator_authority",
            Self::CommandHold => "command_hold",
            Self::HoverTrimUncertainty => "hover_trim_uncertainty",
        }
    }

    /// Parse one stable Pilotage capability name.
    pub fn parse(value: &str) -> Result<Self, ArtifactError> {
        match value {
            "sensor_perturbation" => Ok(Self::SensorPerturbation),
            "actuator_authority" => Ok(Self::ActuatorAuthority),
            "command_hold" => Ok(Self::CommandHold),
            "hover_trim_uncertainty" => Ok(Self::HoverTrimUncertainty),
            _ => Err(ArtifactError::UnsupportedCapability(value.to_owned())),
        }
    }
}

/// Immutable condition inputs bound to one Aviate run.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PerturbationArtifactIdentity {
    /// Artifact path supplied by the harness.
    pub artifact_path: PathBuf,
    /// SHA-256 of the exact artifact bytes.
    pub artifact_sha256: [u8; 32],
    /// SHA-256 of the canonical condition JSON.
    pub condition_digest: [u8; 32],
    /// Run seed supplied by the harness.
    pub run_seed: u64,
    /// Exact condition capabilities that Aviate must execute.
    pub required_capabilities: Vec<PerturbationCapability>,
}

/// A verified artifact and its executable Aviate factors.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadedPerturbationArtifact {
    identity: PerturbationArtifactIdentity,
    config: PerturbationConfig,
    hover_scale_basis_points: u16,
}

impl LoadedPerturbationArtifact {
    /// Load one strict artifact and compare all harness-supplied identities.
    pub fn load(
        path: &Path,
        expected_artifact_sha256: [u8; 32],
        expected_condition_digest: [u8; 32],
        run_seed: u64,
        expected_capabilities: &[PerturbationCapability],
    ) -> Result<Self, ArtifactError> {
        reject_zero_digest("artifact SHA-256", expected_artifact_sha256)?;
        reject_zero_digest("condition digest", expected_condition_digest)?;
        let bytes = read_regular_file(path)?;
        let artifact_sha256 = digest(&bytes);
        if artifact_sha256 != expected_artifact_sha256 {
            return Err(ArtifactError::ArtifactDigestMismatch);
        }
        let condition: ConditionSet = serde_json::from_slice(&bytes)?;
        condition.validate()?;
        let canonical = serde_json::to_vec(&condition)?;
        let condition_digest = digest(&canonical);
        if condition_digest != expected_condition_digest {
            return Err(ArtifactError::ConditionDigestMismatch);
        }
        let required_capabilities = condition.required_capabilities();
        validate_capability_set(expected_capabilities, &required_capabilities)?;
        let config = condition.perturbation_config(condition_digest, run_seed);
        PerturbationEngine::new(config.clone()).map_err(ArtifactError::Perturbation)?;
        Ok(Self {
            identity: PerturbationArtifactIdentity {
                artifact_path: path.to_owned(),
                artifact_sha256,
                condition_digest,
                run_seed,
                required_capabilities,
            },
            config,
            hover_scale_basis_points: condition.hover_scale_basis_points(),
        })
    }

    /// Return the immutable artifact identity.
    #[must_use]
    pub const fn identity(&self) -> &PerturbationArtifactIdentity {
        &self.identity
    }

    /// Return the validated simulator-neutral execution factors.
    #[must_use]
    pub const fn config(&self) -> &PerturbationConfig {
        &self.config
    }

    /// Return the force-domain hover baseline scale.
    #[must_use]
    pub const fn hover_scale_basis_points(&self) -> u16 {
        self.hover_scale_basis_points
    }

    /// Create an Arm-time verifier for the original artifact path.
    #[must_use]
    pub fn live_guard(&self) -> LiveArtifactGuard {
        LiveArtifactGuard {
            path: self.identity.artifact_path.clone(),
            expected_sha256: self.identity.artifact_sha256,
        }
    }
}

/// Arm-time content guard for one verified artifact path.
#[derive(Clone, Debug)]
pub struct LiveArtifactGuard {
    path: PathBuf,
    expected_sha256: [u8; 32],
}

impl LiveArtifactGuard {
    /// Reopen the artifact and require its original byte identity.
    pub fn verify(&self) -> Result<(), ArtifactError> {
        let bytes = read_regular_file(&self.path)?;
        if digest(&bytes) == self.expected_sha256 {
            Ok(())
        } else {
            Err(ArtifactError::LiveArtifactChanged {
                path: self.path.clone(),
            })
        }
    }
}

/// A strict condition-artifact failure.
#[derive(Debug, Error)]
pub enum ArtifactError {
    /// The artifact file could not be opened without following its final link.
    #[error("cannot open condition artifact {path:?}: {source}")]
    Open {
        /// Artifact path.
        path: PathBuf,
        /// File-system failure.
        #[source]
        source: std::io::Error,
    },
    /// The opened artifact is not a regular file.
    #[error("condition artifact {0:?} is not a regular file")]
    NotRegular(PathBuf),
    /// The artifact could not be read.
    #[error("cannot read condition artifact {path:?}: {source}")]
    Read {
        /// Artifact path.
        path: PathBuf,
        /// File-system failure.
        #[source]
        source: std::io::Error,
    },
    /// The artifact exceeds the fixed input limit.
    #[error("condition artifact {path:?} has more than {limit} bytes")]
    TooLarge {
        /// Artifact path.
        path: PathBuf,
        /// Maximum accepted byte count.
        limit: u64,
    },
    /// Strict JSON decoding or canonical encoding failed.
    #[error("condition artifact JSON is invalid: {0}")]
    Json(#[from] serde_json::Error),
    /// A condition value is outside its contract.
    #[error("invalid condition artifact: {0}")]
    Invalid(&'static str),
    /// The exact file digest does not match the harness claim.
    #[error("condition artifact SHA-256 does not match")]
    ArtifactDigestMismatch,
    /// The canonical condition digest does not match the harness claim.
    #[error("canonical condition digest does not match")]
    ConditionDigestMismatch,
    /// The harness capability set does not match the artifact.
    #[error("condition capability set does not match")]
    CapabilityMismatch,
    /// The harness repeated one capability.
    #[error("condition capability set repeats {0}")]
    DuplicateCapability(&'static str),
    /// Aviate does not implement a named condition capability.
    #[error("unsupported condition capability {0}")]
    UnsupportedCapability(String),
    /// Engine validation rejected the decoded factors.
    #[error("condition perturbation is invalid: {0}")]
    Perturbation(#[source] PerturbationError),
    /// The artifact bytes changed after initial verification.
    #[error("condition artifact {path:?} changed after verification")]
    LiveArtifactChanged {
        /// Artifact path.
        path: PathBuf,
    },
}

fn read_regular_file(path: &Path) -> Result<Vec<u8>, ArtifactError> {
    let mut options = OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt as _;
        options.custom_flags(libc::O_CLOEXEC | libc::O_NOFOLLOW);
    }
    let file = options.open(path).map_err(|source| ArtifactError::Open {
        path: path.to_owned(),
        source,
    })?;
    if !file
        .metadata()
        .map_err(|source| ArtifactError::Read {
            path: path.to_owned(),
            source,
        })?
        .is_file()
    {
        return Err(ArtifactError::NotRegular(path.to_owned()));
    }
    read_bounded(file.take(MAX_ARTIFACT_BYTES.wrapping_add(1)), path)
}

fn read_bounded(mut reader: Take<File>, path: &Path) -> Result<Vec<u8>, ArtifactError> {
    let mut bytes = Vec::new();
    reader
        .read_to_end(&mut bytes)
        .map_err(|source| ArtifactError::Read {
            path: path.to_owned(),
            source,
        })?;
    if bytes.len() as u64 > MAX_ARTIFACT_BYTES {
        Err(ArtifactError::TooLarge {
            path: path.to_owned(),
            limit: MAX_ARTIFACT_BYTES,
        })
    } else {
        Ok(bytes)
    }
}

fn validate_capability_set(
    expected: &[PerturbationCapability],
    actual: &[PerturbationCapability],
) -> Result<(), ArtifactError> {
    let mut normalized = expected.to_vec();
    normalized.sort_unstable();
    if let Some(repeated) = normalized.windows(2).find(|pair| pair[0] == pair[1]) {
        return Err(ArtifactError::DuplicateCapability(repeated[0].as_str()));
    }
    let mut actual = actual.to_vec();
    actual.sort_unstable();
    if normalized == actual {
        Ok(())
    } else {
        Err(ArtifactError::CapabilityMismatch)
    }
}

fn reject_zero_digest(field: &'static str, value: [u8; 32]) -> Result<(), ArtifactError> {
    if value == [0; 32] {
        Err(ArtifactError::Invalid(field))
    } else {
        Ok(())
    }
}

fn digest(bytes: &[u8]) -> [u8; 32] {
    Sha256::digest(bytes).into()
}

#[cfg(test)]
mod tests;
