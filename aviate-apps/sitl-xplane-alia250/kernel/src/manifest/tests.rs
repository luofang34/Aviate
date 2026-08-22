#![allow(clippy::expect_used)]

use super::*;

#[test]
fn normal_manifest_contains_all_non_candidate_identities() {
    let built = crate::build_alia250_kernel_with_hover_scale(10_000).expect("valid kernel");
    let kernel = built.kernel;
    let model = ContentDigest::calculate(b"model");
    let runtime = ContentDigest::calculate(b"runtime");
    let build = BuildIdentity::current().expect("build identity");
    let manifest = AliaRunManifest::new(
        &kernel,
        RunPurpose::Normal,
        None,
        build,
        RunExecutionIdentity {
            simulator_model: model,
            runtime_handshake: runtime,
            hover_initialization: built.hover_initialization,
            perturbation: None,
        },
    )
    .expect("manifest");
    let text = manifest.to_toml();
    assert_eq!(manifest.hover_kernel_prefix_hash, 0xf899_1c7f_ac65_1ee8);
    assert_eq!(manifest.kernel_config_hash, 0x396d_fd92_ca64_b405);
    assert!(text.contains("application_id = \"sitl-xplane-alia250\""));
    assert!(text.contains(&format!("simulator_model_digest = \"{model}\"")));
    assert!(text.contains(&format!("runtime_handshake_digest = \"{runtime}\"")));
    assert!(text.contains("hover_baseline_force_bits = \""));
    assert!(text.contains("hover_effective_force_bits = \""));
    assert!(text.contains(&format!(
        "hover_kernel_config_hash = \"{:016x}\"",
        manifest.hover_kernel_config_hash()
    )));
    assert!(!text.contains("hover_force_baseline_bits"));
    assert!(!text.contains("hover_force_effective_bits"));
    assert!(text.contains("hover_estimator_mode = \"disabled\""));
    assert!(!text.contains("candidate_id"));
}

#[test]
fn candidate_manifest_must_match_the_kernel_and_model() {
    let built = crate::build_alia250_kernel_with_hover_scale(10_000).expect("valid kernel");
    let kernel = built.kernel;
    let hover = built.hover_initialization;
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
            RunPurpose::Candidate,
            Some(&wrong_base),
            build.clone(),
            execution(model, runtime, hover),
        ),
        Err(RunManifestError::CandidateBaseMismatch)
    ));
    assert!(matches!(
        AliaRunManifest::new(
            &kernel,
            RunPurpose::Normal,
            Some(&candidate),
            build.clone(),
            execution(model, runtime, hover),
        ),
        Err(RunManifestError::CandidatePurposeMismatch)
    ));
    assert!(matches!(
        AliaRunManifest::new(
            &kernel,
            RunPurpose::Candidate,
            Some(&candidate),
            build.clone(),
            execution(model, runtime, hover),
        ),
        Err(RunManifestError::CandidateKernelMismatch)
    ));
    candidate.kernel_config_hash = kernel.cfg().canonical_hash();
    candidate.simulator_model = ContentDigest::calculate(b"other-model");
    assert!(matches!(
        AliaRunManifest::new(
            &kernel,
            RunPurpose::Candidate,
            Some(&candidate),
            build,
            execution(model, runtime, hover),
        ),
        Err(RunManifestError::CandidateModelMismatch)
    ));
}

#[test]
fn condition_capabilities_are_sorted_and_unique() {
    use aviate_hal_xil::perturbation::PerturbationCapability::{
        ActuatorAuthority, CommandHold, HoverTrimUncertainty, SensorPerturbation,
    };

    let identity = PerturbationArtifactIdentity {
        artifact_path: "condition.json".into(),
        artifact_sha256: [1; 32],
        condition_digest: [2; 32],
        run_seed: 3,
        required_capabilities: vec![
            SensorPerturbation,
            HoverTrimUncertainty,
            CommandHold,
            ActuatorAuthority,
        ],
    };
    let encoded = ManifestPerturbationIdentity::try_from(&identity).expect("identity");
    assert_eq!(
        encoded.required_capabilities,
        vec![
            "actuator_authority",
            "command_hold",
            "hover_trim_uncertainty",
            "sensor_perturbation"
        ]
    );

    let mut duplicate = identity;
    duplicate.required_capabilities.push(CommandHold);
    assert!(matches!(
        ManifestPerturbationIdentity::try_from(&duplicate),
        Err(RunManifestError::DuplicateConditionCapability(
            "command_hold"
        ))
    ));
}

fn execution(
    simulator_model: ContentDigest,
    runtime_handshake: ContentDigest,
    hover_initialization: HoverInitializationEvidence,
) -> RunExecutionIdentity {
    RunExecutionIdentity {
        simulator_model,
        runtime_handshake,
        hover_initialization,
        perturbation: None,
    }
}
