//! Construction of the board-owned tuning trace identity.

use std::net::SocketAddr;

use aviate_app_sitl_xplane_alia250_kernel::AliaRunManifest;
use aviate_board_sitl_xplane::{
    TuningTraceError, XPlaneTuningTraceConfig, XPlaneTuningTraceIdentity,
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
    };
    XPlaneTuningTraceConfig::new(endpoint, identity).map(Some)
}
