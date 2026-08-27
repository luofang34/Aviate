//! Calibration candidate resolution tests.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_config::{
    airframe_preset::{
        calculate_overlay_lineage_digest, resolve_candidate, CalibrationOverlay, CalibrationStage,
        CandidateError, ContentDigest, GainOverrides, InnerLoopDesign,
    },
    xplane_model::XPlaneSimulatorModel,
};

const ALIA250: &str = include_str!("../../presets/alia250.toml");
const X500: &str = include_str!("../../presets/x500.toml");
const MODEL: &str = include_str!("../../presets/alia250-xplane.toml");

fn model_digest() -> ContentDigest {
    let model = model();
    ContentDigest::from_hex(&model.canonical_digest().expect("model digest").to_string())
        .expect("compatible digest")
}

fn model() -> XPlaneSimulatorModel {
    XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model")
}

/// The rate the MODEL declares.
///
/// A plant states the rate it was fitted at and the two must agree, so a
/// fixture that types the number is the same fact written twice — and pins the
/// declaration, because moving it then fails tests with nothing to say about
/// the change.
fn model_sample_rate_hz() -> f64 {
    f64::from(model().sample_rate_hz())
}

fn plant(extra: &str) -> String {
    plant_at(model_sample_rate_hz(), extra)
}

/// A plant fitted at `rate_hz`, for tests that need it to disagree.
fn plant_at(rate_hz: f64, extra: &str) -> String {
    format!(
        "schema_version = 1\nartifact_id = \"plant-a\"\nairframe_id = \"alia250\"\nsimulator_model_digest = \"{}\"\nrun_manifest_digest = \"{}\"\ntrace_digest = \"{}\"\nsample_clock = \"simulator-microseconds\"\noperating_hover_force = 0.43\nprobe_rad_s = [1.0, 2.5]\nsample_rate_hz = {}\nauthority_k = [5.3, 3.1, 1.0]\ndelay_s = [0.02, 0.02, 0.03]\ndelay_ci95_s = [0.005, 0.005, 0.005]\nr_squared = [0.96, 0.95, 0.94]\nauthority_ci95 = [0.2, 0.15, 0.05]\ncoherence = [0.96, 0.95, 0.94]\napplied_input_max = [0.2, 0.2, 0.2]\nsample_count = [500, 500, 500]\nsaturation_fraction = [0.0, 0.0, 0.0]\nresponse_sign = [1, 1, 1]\n{extra}",
        model_digest(),
        "a".repeat(64),
        "b".repeat(64),
        rate_hz,
    )
}

fn candidate(plant_text: &str, extra: &str) -> String {
    format!(
        "schema_version = 1\ncandidate_id = \"test-1\"\nbase_preset_digest = \"{}\"\nplant_artifact_digest = \"{}\"\nstage = \"inner-loop\"\n[inner_loop]\nnatural_frequency_rad_s = [2.0, 2.0, 0.9]\nloop_separation = [6.25, 6.25, 6.25]\n{extra}",
        ContentDigest::calculate(ALIA250.as_bytes()),
        ContentDigest::calculate(plant_text.as_bytes())
    )
}

fn staged_candidate(plant_text: &str, stage: &str, body: &str) -> String {
    format!(
        "schema_version = 1\ncandidate_id = \"test-stage\"\nbase_preset_digest = \"{}\"\nplant_artifact_digest = \"{}\"\nstage = \"{stage}\"\n{body}",
        ContentDigest::calculate(ALIA250.as_bytes()),
        ContentDigest::calculate(plant_text.as_bytes())
    )
}

#[test]
fn candidate_derives_inner_gains_from_the_verified_plant() {
    let plant = plant("");
    let text = candidate(&plant, "");
    let resolved = resolve_candidate(ALIA250, &text, &plant, &model()).expect("valid candidate");
    let (preset, candidate_id, identity, artifact) = resolved.into_parts();
    assert_eq!(preset.hover_thrust_force_seed(), 0.43);
    assert_eq!(preset.gains.pos_p, [0.5, 0.5, 0.5]);
    assert!((preset.gains.att_p[0] - 0.8).abs() < 1.0e-6);
    assert!((preset.gains.rate_p[0] - 0.943_396_2).abs() < 1.0e-6);
    assert_eq!(candidate_id, "test-1");
    assert_eq!(artifact.authority_k, [5.3, 3.1, 1.0]);
    assert_eq!(
        identity.candidate,
        ContentDigest::calculate(text.as_bytes())
    );
}

#[test]
fn candidate_rejects_a_different_base_preset() {
    let plant = plant("");
    let text = candidate(&plant, "").replace(
        &ContentDigest::calculate(ALIA250.as_bytes()).to_string(),
        &"0".repeat(64),
    );
    assert_eq!(
        resolve_candidate(ALIA250, &text, &plant, &model()),
        Err(CandidateError::BasePresetMismatch)
    );
}

#[test]
fn candidate_rejects_a_different_plant_artifact() {
    let original = plant("");
    let text = candidate(&original, "");
    let other = plant("\n# different exact evidence\n");
    assert_eq!(
        resolve_candidate(ALIA250, &text, &other, &model()),
        Err(CandidateError::PlantArtifactMismatch)
    );
}

#[test]
fn candidate_rejects_a_different_simulator_model() {
    let plant = plant("");
    let text = candidate(&plant, "");
    let other = MODEL.replace("model_id = \"alia250-xplane-12\"", "model_id = \"other\"");
    let other = XPlaneSimulatorModel::from_toml_str(&other).expect("valid other model");
    assert_eq!(
        resolve_candidate(ALIA250, &text, &plant, &other),
        Err(CandidateError::PlantSimulatorModelMismatch)
    );
}

#[test]
fn candidate_rejects_schema_one_force_seed_semantics() {
    let plant = plant("").replace("airframe_id = \"alia250\"", "airframe_id = \"x500\"");
    let text = candidate(&plant, "").replace(
        &ContentDigest::calculate(ALIA250.as_bytes()).to_string(),
        &ContentDigest::calculate(X500.as_bytes()).to_string(),
    );
    assert_eq!(
        resolve_candidate(X500, &text, &plant, &model()),
        Err(CandidateError::UnsupportedBaseSchema(1))
    );
}

#[test]
fn direct_inner_gain_overrides_are_not_in_the_allowlist() {
    let plant = plant("");
    let text = candidate(&plant, "[gains]\natt_p = [0.7, 0.7, 0.3]\n");
    assert!(matches!(
        resolve_candidate(ALIA250, &text, &plant, &model()),
        Err(CandidateError::Parse(_))
    ));
}

#[test]
fn candidate_rejects_a_bandwidth_that_the_delay_cannot_support() {
    let plant = plant("").replace(
        "delay_s = [0.02, 0.02, 0.03]",
        "delay_s = [0.20, 0.02, 0.03]",
    );
    let text = candidate(&plant, "");
    assert_eq!(
        resolve_candidate(ALIA250, &text, &plant, &model()),
        Err(CandidateError::InvalidRelation(
            "bandwidth must hold across the delay interval"
        ))
    );
}

#[test]
fn saturation_bar_admits_the_census_bound_and_refuses_just_past_it() {
    let at_bar = plant("").replace(
        "saturation_fraction = [0.0, 0.0, 0.0]",
        "saturation_fraction = [0.12, 0.12, 0.12]",
    );
    let text = candidate(&at_bar, "");
    assert!(resolve_candidate(ALIA250, &text, &at_bar, &model()).is_ok());

    let past_bar = plant("").replace(
        "saturation_fraction = [0.0, 0.0, 0.0]",
        "saturation_fraction = [0.121, 0.0, 0.0]",
    );
    let text = candidate(&past_bar, "");
    assert!(matches!(
        resolve_candidate(ALIA250, &text, &past_bar, &model()),
        Err(CandidateError::PlantArtifact(_))
    ));
}

#[test]
fn candidate_rejects_weak_or_clipped_plant_evidence() {
    for replacement in [
        (
            "r_squared = [0.96, 0.95, 0.94]",
            "r_squared = [0.24, 0.95, 0.94]",
        ),
        (
            "saturation_fraction = [0.0, 0.0, 0.0]",
            "saturation_fraction = [0.13, 0.0, 0.0]",
        ),
        (
            "sample_count = [500, 500, 500]",
            "sample_count = [99, 500, 500]",
        ),
    ] {
        let plant = plant("").replace(replacement.0, replacement.1);
        let text = candidate(&plant, "");
        assert!(matches!(
            resolve_candidate(ALIA250, &text, &plant, &model()),
            Err(CandidateError::PlantArtifact(_))
        ));
    }
}

#[test]
fn candidate_rejects_unknown_fields_and_invalid_bounds() {
    let plant = plant("");
    for extra in [
        "mixer = \"quad-x\"\n",
        "[gains]\nvel_max_roll_pitch = 1.3\n",
        "[inner_loop.extra]\nvalue = 1\n",
    ] {
        let text = candidate(&plant, extra);
        assert!(matches!(
            resolve_candidate(ALIA250, &text, &plant, &model()),
            Err(CandidateError::FieldOutOfRange(_))
                | Err(CandidateError::InvalidRelation(_))
                | Err(CandidateError::Parse(_))
        ));
    }
}

#[test]
fn each_stage_requires_one_owned_change_and_freezes_other_fields() {
    let plant = plant("");
    for text in [
        staged_candidate(&plant, "outer-loop", ""),
        staged_candidate(&plant, "outer-loop", "[gains]\nrate_i = [0.1, 0.1, 0.3]\n"),
    ] {
        assert_eq!(
            resolve_candidate(ALIA250, &text, &plant, &model()),
            Err(CandidateError::InvalidRelation(
                "candidate fields must belong to one calibration stage"
            ))
        );
    }
}

#[test]
fn hover_requires_wire_headroom() {
    let plant = plant("");
    let text = staged_candidate(&plant, "hover", "hover_thrust_seed = 0.51\n");
    assert_eq!(
        resolve_candidate(ALIA250, &text, &plant, &model()),
        Err(CandidateError::InvalidRelation(
            "hover force must keep wire collective headroom"
        ))
    );
}

#[test]
fn authority_interval_must_preserve_loop_separation() {
    let plant = plant("").replace(
        "authority_ci95 = [0.2, 0.15, 0.05]",
        "authority_ci95 = [1.3, 0.15, 0.05]",
    );
    let text = candidate(&plant, "");
    assert_eq!(
        resolve_candidate(ALIA250, &text, &plant, &model()),
        Err(CandidateError::InvalidRelation(
            "loop separation must hold across the authority interval"
        ))
    );
}

#[test]
fn gain_steps_must_be_adjacent_to_the_champion() {
    let plant = plant("");
    let text = staged_candidate(&plant, "outer-loop", "[gains]\npos_p = [0.7, 0.5, 0.5]\n");
    assert_eq!(
        resolve_candidate(ALIA250, &text, &plant, &model()),
        Err(CandidateError::InvalidRelation(
            "candidate gain step must be adjacent to the champion"
        ))
    );
}

#[test]
fn plant_sample_clock_rate_must_match_the_model() {
    // Names its own disagreement rather than editing the text of a fixture,
    // so it stays a disagreement whatever the model comes to declare.
    let plant = plant_at(model_sample_rate_hz() - 20.0, "");
    let text = candidate(&plant, "");
    assert_eq!(
        resolve_candidate(ALIA250, &text, &plant, &model()),
        Err(CandidateError::InvalidRelation(
            "plant sample rate must match the simulator model"
        ))
    );
}

#[test]
fn cumulative_overlays_rebuild_the_final_winner_and_bind_each_parent() {
    let plant = plant("");
    let base = ContentDigest::calculate(ALIA250.as_bytes());
    let plant_digest = ContentDigest::calculate(plant.as_bytes());
    let inner = CalibrationOverlay {
        overlay_id: "inner-winner".to_owned(),
        parent_digest: base.to_string(),
        stage: CalibrationStage::InnerLoop,
        hover_thrust_seed: None,
        inner_loop: Some(InnerLoopDesign {
            natural_frequency_rad_s: [1.75, 1.75, 0.75],
            loop_separation: [6.25; 3],
        }),
        gains: GainOverrides::default(),
    };
    let inner_lineage = calculate_overlay_lineage_digest(base, plant_digest, &inner);
    let rate = CalibrationOverlay {
        overlay_id: "rate-id-winner".to_owned(),
        parent_digest: inner_lineage.to_string(),
        stage: CalibrationStage::RateIntegralDerivative,
        hover_thrust_seed: None,
        inner_loop: None,
        gains: GainOverrides {
            rate_i: Some([0.11, 0.1, 0.3]),
            ..GainOverrides::default()
        },
    };
    let final_lineage = calculate_overlay_lineage_digest(inner_lineage, plant_digest, &rate);
    let text = format!(
        "schema_version = 2\ncandidate_id = \"cumulative-winner\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"{plant_digest}\"\n\n[[overlays]]\noverlay_id = \"inner-winner\"\nparent_digest = \"{base}\"\nstage = \"inner-loop\"\n[overlays.inner_loop]\nnatural_frequency_rad_s = [1.75, 1.75, 0.75]\nloop_separation = [6.25, 6.25, 6.25]\n\n[[overlays]]\noverlay_id = \"rate-id-winner\"\nparent_digest = \"{inner_lineage}\"\nstage = \"rate-integral-derivative\"\n[overlays.gains]\nrate_i = [0.11, 0.1, 0.3]\n"
    );
    let resolved = resolve_candidate(ALIA250, &text, &plant, &model()).expect("valid lineage");
    let (preset, _, identity, _) = resolved.into_parts();
    assert!((preset.gains.att_p[0] - 0.7).abs() < 1.0e-6);
    assert_eq!(preset.gains.rate_i, [0.11, 0.1, 0.3]);
    assert_eq!(identity.lineage, final_lineage);

    let changed_prefix = text.replace(
        "natural_frequency_rad_s = [1.75, 1.75, 0.75]",
        "natural_frequency_rad_s = [1.7, 1.75, 0.75]",
    );
    assert_eq!(
        resolve_candidate(ALIA250, &changed_prefix, &plant, &model()),
        Err(CandidateError::LineageMismatch(1))
    );

    let duplicate = format!(
        "{text}\n[[overlays]]\noverlay_id = \"rate-id-again\"\nparent_digest = \"{final_lineage}\"\nstage = \"rate-integral-derivative\"\n[overlays.gains]\nrate_d = [0.01, 0.01, 0.01]\n"
    );
    assert_eq!(
        resolve_candidate(ALIA250, &duplicate, &plant, &model()),
        Err(CandidateError::DuplicateStage(
            CalibrationStage::RateIntegralDerivative
        ))
    );
}
