//! Versioned wire values for the simulator tuning trace.

mod command;
mod observation;
mod perturbation;

pub use command::{
    TuningCommand, TuningCommandSource, TuningConfigMode, TuningControlMode, TuningEstimate,
    TuningEstimateQuality, TuningEstimateValidity, TuningImu, TuningSetpoint,
};
pub use observation::{TuningConstraintFlags, TuningControlObservation};
pub use perturbation::{
    TuningActuatorApplication, TuningActuatorBypassReason, TuningActuatorEligibility,
    TuningHoverEstimatorMode, TuningHoverInitialization, TuningPerturbationCapability,
    TuningSendEvidence, TuningSensorApplication,
};

use serde::{Deserialize, Serialize};

/// Frame names in the tuning trace protocol.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningFrameType {
    /// Aviate identity sent before observations.
    AviateTuningHandshake,
    /// Runner acceptance of the Aviate identity.
    AviateTuningReady,
    /// One causal simulator-packet observation.
    AviateControlObservation,
    /// Runner acceptance of one observation sequence.
    AviateTuningObservationAck,
}

/// One exact run identity sent before the trace starts.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningHandshake {
    /// Must be `aviate-tuning-handshake`.
    #[serde(rename = "type")]
    pub frame_type: TuningFrameType,
    /// Protocol schema version.
    pub schema_version: u16,
    /// SHA-256 of the exact run manifest text.
    pub run_manifest_digest: String,
    /// SHA-256 of the running executable.
    pub build_identity: String,
    /// SHA-256 of the embedded source bundle.
    pub source_identity: String,
    /// SHA-256 of the dependency lock file.
    pub lock_identity: String,
    /// SHA-256 of the simulator model.
    pub simulator_model_digest: String,
    /// SHA-256 of the consumed runtime handshake.
    pub runtime_handshake_digest: String,
    /// SHA-256 of the candidate document. Omitted for a base run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub candidate_digest: Option<String>,
    /// SHA-256 of the resolved overlay lineage. Omitted for a base run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub candidate_lineage_digest: Option<String>,
    /// SHA-256 of the plant artifact. Omitted for a base run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plant_artifact_digest: Option<String>,
    /// Flight algorithm identity as 16 lowercase hexadecimal digits.
    pub algorithm_identity_hash: String,
    /// Resolved kernel identity as 16 lowercase hexadecimal digits.
    pub kernel_config_hash: String,
    /// Condition artifact path. Omitted when no condition is active.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub condition_artifact_path: Option<String>,
    /// SHA-256 of the exact condition artifact bytes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub condition_artifact_sha256: Option<String>,
    /// SHA-256 of the canonical condition JSON.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub condition_digest: Option<String>,
    /// Seed for deterministic condition decisions.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub condition_run_seed: Option<u64>,
    /// Exact sorted Aviate-owned condition capabilities.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub condition_required_capabilities: Option<Vec<TuningPerturbationCapability>>,
    /// Preset hover-force baseline as IEEE-754 bits.
    pub hover_baseline_force_bits: u32,
    /// Effective hover force as IEEE-754 bits.
    pub hover_effective_force_bits: u32,
    /// Applied hover-force scale in basis points.
    pub hover_scale_basis_points: u16,
    /// Online hover estimator state for this run.
    pub hover_estimator_mode: TuningHoverEstimatorMode,
    /// Effective hover kernel hash as 16 lowercase hexadecimal digits.
    pub hover_kernel_config_hash: String,
}

/// Runner acceptance of one handshake.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningReady {
    /// Must be `aviate-tuning-ready`.
    #[serde(rename = "type")]
    pub frame_type: TuningFrameType,
    /// Protocol schema version.
    pub schema_version: u16,
    /// SHA-256 of the accepted run manifest text.
    pub run_manifest_digest: String,
}

/// Runner acceptance of one observation.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningObservationAck {
    /// Must be `aviate-tuning-observation-ack`.
    #[serde(rename = "type")]
    pub frame_type: TuningFrameType,
    /// Protocol schema version.
    pub schema_version: u16,
    /// SHA-256 of the active run manifest text.
    pub run_manifest_digest: String,
    /// Exact observation sequence accepted by the runner.
    pub sequence: u64,
}
