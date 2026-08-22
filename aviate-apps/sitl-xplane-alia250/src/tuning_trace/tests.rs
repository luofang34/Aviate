//! Condition identity mapping tests.

#![allow(clippy::expect_used)]

use aviate_app_sitl_xplane_alia250_kernel::ManifestPerturbationIdentity;
use aviate_board_sitl_xplane::{TuningPerturbationCapability, TuningTraceError};

use super::condition_identity;

fn identity(capabilities: Vec<&'static str>) -> ManifestPerturbationIdentity {
    ManifestPerturbationIdentity {
        artifact_path: "/tmp/condition.json".to_owned(),
        artifact_sha256: "a".repeat(64),
        condition_digest: "b".repeat(64),
        run_seed: 41,
        required_capabilities: capabilities,
    }
}

#[test]
fn exact_capability_names_map_without_reordering() {
    let mapped = condition_identity(&identity(vec![
        "actuator_authority",
        "command_hold",
        "hover_trim_uncertainty",
        "sensor_perturbation",
    ]))
    .expect("mapped identity");
    assert_eq!(
        mapped.capabilities,
        vec![
            TuningPerturbationCapability::ActuatorAuthority,
            TuningPerturbationCapability::CommandHold,
            TuningPerturbationCapability::HoverTrimUncertainty,
            TuningPerturbationCapability::SensorPerturbation,
        ]
    );
}

#[test]
fn unknown_capability_fails_closed() {
    assert!(matches!(
        condition_identity(&identity(vec!["unknown"])),
        Err(TuningTraceError::InvalidIdentity("condition capabilities"))
    ));
}
