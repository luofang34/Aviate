//! Kernel construction from a validated preset, and what it refuses.

#![allow(clippy::expect_used)]

use super::*;

const XPLANE_MODEL: &str = include_str!("../../../../../presets/alia250-xplane.toml");

fn model_digest() -> ContentDigest {
    let model = aviate_config::xplane_model::XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL)
        .expect("valid model");
    ContentDigest::from_hex(&model.canonical_digest().expect("model digest").to_string())
        .expect("compatible digest")
}

/// The rate the MODEL declares, so the fixture cannot drift from it.
///
/// A plant states the rate it was fitted at, and the two are required to
/// agree. Typing the number here made the preset's rate unchangeable: the
/// declaration and the fixture were the same fact written twice, and
/// moving one broke tests that had nothing to say about the change.
fn model_sample_rate_hz() -> u16 {
    XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL)
        .expect("valid model")
        .sample_rate_hz()
}

fn plant_artifact() -> String {
    format!(
        "schema_version = 1\nartifact_id = \"plant-a\"\nairframe_id = \"alia250\"\nsimulator_model_digest = \"{}\"\nrun_manifest_digest = \"{}\"\ntrace_digest = \"{}\"\nsample_clock = \"simulator-microseconds\"\noperating_hover_force = 0.43\nprobe_rad_s = [1.0, 2.5]\nsample_rate_hz = {}\nauthority_k = [5.3, 3.1, 1.0]\ndelay_s = [0.02, 0.02, 0.03]\ndelay_ci95_s = [0.005, 0.005, 0.005]\nr_squared = [0.96, 0.95, 0.94]\nauthority_ci95 = [0.2, 0.15, 0.05]\ncoherence = [0.96, 0.95, 0.94]\napplied_input_max = [0.2, 0.2, 0.2]\nsample_count = [500, 500, 500]\nsaturation_fraction = [0.0, 0.0, 0.0]\nresponse_sign = [1, 1, 1]\n",
        model_digest(),
        "a".repeat(64),
        "b".repeat(64),
        f64::from(model_sample_rate_hz()),
    )
}

#[test]
fn normal_kernel_uses_the_shipped_preset() {
    let preset = load_preset().expect("valid preset");
    let kernel = build_alia250_kernel().expect("valid kernel");
    assert_eq!(kernel.cfg().cascade_gains, gains_from_preset(preset.gains));
    assert_eq!(kernel.cfg().hover_thrust_norm.0, 0.43);
    assert_eq!(
        kernel.cfg().mixer_geometry,
        MixerGeometry::QuadXX500ReversedSpin
    );
}

#[test]
fn identification_kernel_hosts_the_airframes_own_law_at_reduced_authority() {
    let preset = load_preset().expect("valid preset");
    let flight = gains_from_preset(preset.gains);
    let kernel = build_alia250_identification_kernel().expect("valid kernel");
    let host = kernel.cfg().cascade_gains;
    let rate_scale = [0.5, 0.5, 0.2];
    let att_scale = [0.7, 0.7, 0.3];
    for axis in 0..3 {
        assert_eq!(host.rate_p[axis], flight.rate_p[axis] * rate_scale[axis]);
        assert_eq!(host.rate_d[axis], flight.rate_d[axis] * rate_scale[axis]);
        assert_eq!(host.rate_i[axis], 0.0);
        assert_eq!(host.att_p[axis], flight.att_p[axis] * att_scale[axis]);
    }
    // Everything the scaling does not name stays the flight law's own.
    assert_eq!(host.pos_p, flight.pos_p);
    assert_eq!(host.vel_p, flight.vel_p);
}

#[test]
fn hover_scale_changes_only_the_force_domain_initialization() {
    let model = XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL).expect("valid model");
    let model_identity = model.canonical_digest().expect("model identity");
    let low = build_alia250_kernel_with_hover_scale(9_000).expect("0.9 hover scale");
    let nominal = build_alia250_kernel_with_hover_scale(10_000).expect("nominal hover scale");
    let high = build_alia250_kernel_with_hover_scale(11_000).expect("1.1 hover scale");

    assert_eq!(
        f32::from_bits(low.hover_initialization.baseline_force_bits),
        0.43
    );
    assert_eq!(
        low.hover_initialization.baseline_force_bits,
        nominal.hover_initialization.baseline_force_bits
    );
    assert_eq!(
        nominal.hover_initialization.baseline_force_bits,
        high.hover_initialization.baseline_force_bits
    );
    assert_eq!(
        low.kernel.cfg().hover_thrust_norm.0,
        (f64::from(0.43_f32) * 0.9) as f32
    );
    assert_eq!(nominal.kernel.cfg().hover_thrust_norm.0, 0.43);
    assert_eq!(
        high.kernel.cfg().hover_thrust_norm.0,
        (f64::from(0.43_f32) * 1.1) as f32
    );
    assert_eq!(
        low.hover_initialization.estimator_mode,
        HoverEstimatorMode::Disabled
    );
    assert_eq!(
        low.hover_initialization.effective_kernel_config_hash,
        low.kernel.cfg().canonical_hash()
    );
    assert_eq!(
        model.canonical_digest().expect("unchanged model identity"),
        model_identity
    );
}

#[test]
fn hover_scale_bounds_fail_before_kernel_construction() {
    for scale in [7_999, 12_001] {
        assert!(matches!(
            build_alia250_kernel_with_hover_scale(scale),
            Err(AliaKernelBuildError::InvalidHoverScale(value)) if value == scale
        ));
    }
}

#[test]
fn candidate_build_reports_all_input_identities() {
    let base = aviate_config::airframe_preset::ContentDigest::calculate(ALIA250_PRESET.as_bytes());
    let plant = plant_artifact();
    let plant_digest = ContentDigest::calculate(plant.as_bytes());
    let text = format!(
        "schema_version = 1\ncandidate_id = \"candidate-a\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"{plant_digest}\"\nstage = \"inner-loop\"\n[inner_loop]\nnatural_frequency_rad_s = [1.75, 1.75, 0.75]\nloop_separation = [6.25, 6.25, 6.25]\n"
    );
    let model = aviate_config::xplane_model::XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL)
        .expect("valid model");
    let built =
        build_alia250_kernel_with_candidate(&text, &plant, &model).expect("valid candidate kernel");
    assert_eq!(built.manifest.candidate_id, "candidate-a");
    assert_eq!(built.kernel.cfg().cascade_gains.att_p, [0.7, 0.7, 0.3]);
    assert_eq!(built.manifest.identity.plant_artifact, plant_digest);
    assert_eq!(built.manifest.simulator_model, model_digest());
    assert_eq!(
        built.manifest.kernel_config_hash,
        built.kernel.cfg().canonical_hash()
    );
}

#[test]
fn allowed_candidate_field_changes_both_resolved_identities() {
    let base = ContentDigest::calculate(ALIA250_PRESET.as_bytes());
    let plant = plant_artifact();
    let plant_digest = ContentDigest::calculate(plant.as_bytes());
    let original = format!(
        "schema_version = 1\ncandidate_id = \"candidate-identity\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"{plant_digest}\"\nstage = \"rate-integral-derivative\"\n[gains]\nrate_d_lpf_alpha = 0.45\n"
    );
    let changed = original.replace("rate_d_lpf_alpha = 0.45", "rate_d_lpf_alpha = 0.55");
    let model = XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL).expect("valid model");

    let first = build_alia250_kernel_with_candidate(&original, &plant, &model)
        .expect("valid original candidate");
    let repeated = build_alia250_kernel_with_candidate(&original, &plant, &model)
        .expect("repeat original candidate");
    let mutated = build_alia250_kernel_with_candidate(&changed, &plant, &model)
        .expect("valid mutated candidate");

    assert_eq!(first.manifest.identity, repeated.manifest.identity);
    assert_eq!(
        first.manifest.kernel_config_hash,
        repeated.manifest.kernel_config_hash
    );
    assert_ne!(
        first.manifest.identity.candidate,
        mutated.manifest.identity.candidate
    );
    assert_ne!(
        first.manifest.kernel_config_hash,
        mutated.manifest.kernel_config_hash
    );
}

#[test]
fn pilotage_candidate_prefix_has_a_pinned_vector() {
    let base = ContentDigest::calculate(ALIA250_PRESET.as_bytes());
    let plant = plant_artifact();
    let plant_digest = ContentDigest::calculate(plant.as_bytes());
    let candidate = format!(
        "schema_version = 1\ncandidate_id = \"pilotage-vector\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"{plant_digest}\"\nstage = \"inner-loop\"\n[inner_loop]\nnatural_frequency_rad_s = [1.7, 1.8, 0.8]\nloop_separation = [6.0, 6.0, 6.2]\n"
    );
    let model = XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL).expect("valid model");
    let built = build_alia250_kernel_with_candidate(&candidate, &plant, &model)
        .expect("valid candidate kernel");
    assert_eq!(
        built.kernel.cfg().hover_kernel_prefix_hash(),
        0x262e_ca7c_3a3b_18bd
    );
}
