//! Immutable trace identity validation and handshake construction.

use std::net::SocketAddr;

use super::protocol::{
    TuningFrameType, TuningHandshake, TuningHoverEstimatorMode, TuningPerturbationCapability,
};
use super::{TuningTraceError, TUNING_TRACE_SCHEMA_VERSION};

/// Immutable identities for one tuning trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XPlaneTuningTraceIdentity {
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
    /// Candidate document identity for a candidate run.
    pub candidate_digest: Option<String>,
    /// Final candidate lineage identity for a candidate run.
    pub candidate_lineage_digest: Option<String>,
    /// Plant artifact identity for a candidate run.
    pub plant_artifact_digest: Option<String>,
    /// Flight algorithm identity as 16 lowercase hexadecimal digits.
    pub algorithm_identity_hash: String,
    /// Resolved kernel identity as 16 lowercase hexadecimal digits.
    pub kernel_config_hash: String,
    /// Condition artifact path for a condition-bound run.
    pub condition_artifact_path: Option<String>,
    /// SHA-256 of the exact condition artifact bytes.
    pub condition_artifact_sha256: Option<String>,
    /// SHA-256 of the canonical condition JSON.
    pub condition_digest: Option<String>,
    /// Seed for deterministic condition decisions.
    pub condition_run_seed: Option<u64>,
    /// Exact sorted Aviate-owned condition capabilities.
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

/// Configuration for the packet-synchronous tuning trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XPlaneTuningTraceConfig {
    pub(super) endpoint: SocketAddr,
    pub(super) identity: XPlaneTuningTraceIdentity,
}

impl XPlaneTuningTraceConfig {
    /// Validate one loopback endpoint and its immutable identity.
    ///
    /// # Errors
    ///
    /// Returns an error for a non-loopback address or invalid identity.
    pub fn new(
        endpoint: SocketAddr,
        identity: XPlaneTuningTraceIdentity,
    ) -> Result<Self, TuningTraceError> {
        if !endpoint.ip().is_loopback() {
            return Err(TuningTraceError::NonLoopbackEndpoint(endpoint));
        }
        validate_identity(&identity)?;
        Ok(Self { endpoint, identity })
    }
}

pub(super) fn handshake_from_identity(identity: XPlaneTuningTraceIdentity) -> TuningHandshake {
    TuningHandshake {
        frame_type: TuningFrameType::AviateTuningHandshake,
        schema_version: TUNING_TRACE_SCHEMA_VERSION,
        run_manifest_digest: identity.run_manifest_digest,
        build_identity: identity.build_identity,
        source_identity: identity.source_identity,
        lock_identity: identity.lock_identity,
        simulator_model_digest: identity.simulator_model_digest,
        runtime_handshake_digest: identity.runtime_handshake_digest,
        candidate_digest: identity.candidate_digest,
        candidate_lineage_digest: identity.candidate_lineage_digest,
        plant_artifact_digest: identity.plant_artifact_digest,
        algorithm_identity_hash: identity.algorithm_identity_hash,
        kernel_config_hash: identity.kernel_config_hash,
        condition_artifact_path: identity.condition_artifact_path,
        condition_artifact_sha256: identity.condition_artifact_sha256,
        condition_digest: identity.condition_digest,
        condition_run_seed: identity.condition_run_seed,
        condition_required_capabilities: identity.condition_required_capabilities,
        hover_baseline_force_bits: identity.hover_baseline_force_bits,
        hover_effective_force_bits: identity.hover_effective_force_bits,
        hover_scale_basis_points: identity.hover_scale_basis_points,
        hover_estimator_mode: identity.hover_estimator_mode,
        hover_kernel_config_hash: identity.hover_kernel_config_hash,
    }
}

fn validate_identity(identity: &XPlaneTuningTraceIdentity) -> Result<(), TuningTraceError> {
    validate_digests(identity)?;
    validate_candidate(identity)?;
    validate_condition(identity)?;
    validate_hover(identity)
}

fn validate_digests(identity: &XPlaneTuningTraceIdentity) -> Result<(), TuningTraceError> {
    for (field, value) in [
        ("run_manifest_digest", identity.run_manifest_digest.as_str()),
        ("build_identity", identity.build_identity.as_str()),
        ("source_identity", identity.source_identity.as_str()),
        ("lock_identity", identity.lock_identity.as_str()),
        (
            "simulator_model_digest",
            identity.simulator_model_digest.as_str(),
        ),
        (
            "runtime_handshake_digest",
            identity.runtime_handshake_digest.as_str(),
        ),
    ] {
        validate_hex(field, value, 64)?;
    }
    validate_hex(
        "algorithm_identity_hash",
        &identity.algorithm_identity_hash,
        16,
    )?;
    validate_hex("kernel_config_hash", &identity.kernel_config_hash, 16)?;
    validate_hex(
        "hover_kernel_config_hash",
        &identity.hover_kernel_config_hash,
        16,
    )
}

fn validate_candidate(identity: &XPlaneTuningTraceIdentity) -> Result<(), TuningTraceError> {
    let fields = [
        identity.candidate_digest.as_deref(),
        identity.candidate_lineage_digest.as_deref(),
        identity.plant_artifact_digest.as_deref(),
    ];
    if fields.iter().any(Option::is_some) && !fields.iter().all(Option::is_some) {
        return Err(TuningTraceError::InvalidIdentity("candidate identity"));
    }
    for value in fields.into_iter().flatten() {
        validate_hex("candidate identity", value, 64)?;
    }
    Ok(())
}

fn validate_condition(identity: &XPlaneTuningTraceIdentity) -> Result<(), TuningTraceError> {
    let fields = [
        identity.condition_artifact_path.is_some(),
        identity.condition_artifact_sha256.is_some(),
        identity.condition_digest.is_some(),
        identity.condition_run_seed.is_some(),
        identity.condition_required_capabilities.is_some(),
    ];
    if !fields.into_iter().all(|present| present) && !fields.into_iter().all(|present| !present) {
        return Err(TuningTraceError::InvalidIdentity("condition identity"));
    }
    let Some(path) = identity.condition_artifact_path.as_deref() else {
        return Ok(());
    };
    if path.is_empty() {
        return Err(TuningTraceError::InvalidIdentity("condition artifact path"));
    }
    validate_hex(
        "condition artifact SHA-256",
        identity
            .condition_artifact_sha256
            .as_deref()
            .unwrap_or_default(),
        64,
    )?;
    validate_hex(
        "condition digest",
        identity.condition_digest.as_deref().unwrap_or_default(),
        64,
    )?;
    let capabilities = identity
        .condition_required_capabilities
        .as_deref()
        .unwrap_or_default();
    if !capabilities
        .windows(2)
        .all(|pair| pair[0].as_str() < pair[1].as_str())
    {
        return Err(TuningTraceError::InvalidIdentity("condition capabilities"));
    }
    Ok(())
}

fn validate_hover(identity: &XPlaneTuningTraceIdentity) -> Result<(), TuningTraceError> {
    let baseline = f32::from_bits(identity.hover_baseline_force_bits);
    let effective = f32::from_bits(identity.hover_effective_force_bits);
    let reference =
        (f64::from(baseline) * f64::from(identity.hover_scale_basis_points) / 10_000.0) as f32;
    if !baseline.is_finite()
        || !effective.is_finite()
        || !(0.0..1.0).contains(&baseline)
        || !(0.0..1.0).contains(&effective)
        || effective.to_bits() != reference.to_bits()
        || !(8_000..=12_000).contains(&identity.hover_scale_basis_points)
        || identity.hover_estimator_mode != TuningHoverEstimatorMode::Disabled
        || identity.hover_kernel_config_hash != identity.kernel_config_hash
    {
        Err(TuningTraceError::InvalidIdentity("hover initialization"))
    } else {
        Ok(())
    }
}

fn validate_hex(field: &'static str, value: &str, length: usize) -> Result<(), TuningTraceError> {
    if value.len() != length
        || !value
            .as_bytes()
            .iter()
            .all(|byte| byte.is_ascii_digit() || matches!(*byte, b'a'..=b'f'))
    {
        return Err(TuningTraceError::InvalidIdentity(field));
    }
    Ok(())
}
