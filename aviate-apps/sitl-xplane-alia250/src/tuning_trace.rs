//! Construction of the board-owned tuning trace identity.

use std::net::SocketAddr;

use aviate_app_sitl_xplane_alia250_kernel::AliaRunManifest;
use aviate_board_sitl_xplane::{
    TuningHoverEstimatorMode, TuningPerturbationCapability, TuningTraceError,
    XPlaneTuningTraceConfig, XPlaneTuningTraceIdentity,
};
use aviate_config::airframe_preset::ContentDigest;

pub(super) fn config(
    endpoint: Option<SocketAddr>,
    manifest: &AliaRunManifest,
    manifest_digest: ContentDigest,
) -> Result<Option<XPlaneTuningTraceConfig>, TuningTraceError> {
    let Some(endpoint) = endpoint else {
        return Ok(None);
    };
    let (candidate_digest, candidate_lineage_digest, plant_artifact_digest) = manifest
        .candidate_identity
        .map_or((None, None, None), |identity| {
            (
                Some(identity.candidate.to_string()),
                Some(identity.lineage.to_string()),
                Some(identity.plant_artifact.to_string()),
            )
        });
    let condition = manifest
        .perturbation
        .as_ref()
        .map(condition_identity)
        .transpose()?;
    let (condition_artifact_path, condition_artifact_sha256, condition_digest) =
        condition.as_ref().map_or((None, None, None), |identity| {
            (
                Some(identity.artifact_path.clone()),
                Some(identity.artifact_sha256.clone()),
                Some(identity.condition_digest.clone()),
            )
        });
    let condition_run_seed = condition.as_ref().map(|identity| identity.run_seed);
    let condition_required_capabilities = condition.map(|identity| identity.capabilities);
    let hover_estimator_mode = match manifest.hover_initialization.estimator_mode {
        aviate_app_sitl_xplane_alia250_kernel::HoverEstimatorMode::Disabled => {
            TuningHoverEstimatorMode::Disabled
        }
    };
    let identity = XPlaneTuningTraceIdentity {
        run_manifest_digest: manifest_digest.to_string(),
        build_identity: manifest.build_identity.to_string(),
        source_identity: manifest.source_identity.to_string(),
        lock_identity: manifest.lock_identity.to_string(),
        simulator_model_digest: manifest.simulator_model.to_string(),
        runtime_handshake_digest: manifest.runtime_handshake.to_string(),
        candidate_digest,
        candidate_lineage_digest,
        plant_artifact_digest,
        algorithm_identity_hash: format!("{:016x}", manifest.algorithm_identity_hash),
        kernel_config_hash: format!("{:016x}", manifest.kernel_config_hash),
        condition_artifact_path,
        condition_artifact_sha256,
        condition_digest,
        condition_run_seed,
        condition_required_capabilities,
        hover_baseline_force_bits: manifest.hover_initialization.baseline_force_bits,
        hover_effective_force_bits: manifest.hover_initialization.effective_force_bits,
        hover_scale_basis_points: manifest.hover_initialization.scale_basis_points,
        hover_estimator_mode,
        hover_kernel_config_hash: format!("{:016x}", manifest.hover_kernel_config_hash()),
    };
    XPlaneTuningTraceConfig::new(endpoint, identity).map(Some)
}

struct ConditionIdentity {
    artifact_path: String,
    artifact_sha256: String,
    condition_digest: String,
    run_seed: u64,
    capabilities: Vec<TuningPerturbationCapability>,
}

fn condition_identity(
    value: &aviate_app_sitl_xplane_alia250_kernel::ManifestPerturbationIdentity,
) -> Result<ConditionIdentity, TuningTraceError> {
    let capabilities = value
        .required_capabilities
        .iter()
        .map(|name| match *name {
            "actuator_authority" => Ok(TuningPerturbationCapability::ActuatorAuthority),
            "command_hold" => Ok(TuningPerturbationCapability::CommandHold),
            "hover_trim_uncertainty" => Ok(TuningPerturbationCapability::HoverTrimUncertainty),
            "sensor_perturbation" => Ok(TuningPerturbationCapability::SensorPerturbation),
            _ => Err(TuningTraceError::InvalidIdentity("condition capabilities")),
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(ConditionIdentity {
        artifact_path: value.artifact_path.clone(),
        artifact_sha256: value.artifact_sha256.clone(),
        condition_digest: value.condition_digest.clone(),
        run_seed: value.run_seed,
        capabilities,
    })
}

#[cfg(test)]
mod tests;
