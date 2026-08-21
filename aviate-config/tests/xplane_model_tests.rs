#![allow(clippy::expect_used)]

use aviate_config::{
    airframe_preset::ContentDigest,
    xplane_model::{XPlaneModelError, XPlaneSimulatorModel},
};

const MODEL: &str = include_str!("../../presets/alia250-xplane.toml");
const AIRFRAME: &str = include_str!("../../presets/alia250.toml");

#[test]
fn alia_model_has_the_measured_protection_boundary() {
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid Alia model");
    assert_eq!(model.airframe_id(), "alia250");
    assert_eq!(model.lane_order(), [0, 2, 1, 3]);
    assert_eq!(model.wire().rise_per_s, 0.035);
    assert_eq!(model.wire().mean_ceiling, 0.55);
    assert_eq!(model.motor_count(), 4);
    assert_eq!(model.sample_rate_hz(), 100);
    assert_eq!(
        model.airframe_preset_digest(),
        ContentDigest::calculate(AIRFRAME.as_bytes()).to_string()
    );
}

#[test]
fn signed_zero_has_one_canonical_identity() {
    let positive = MODEL.replace("ground_squeeze = 0.5", "ground_squeeze = 0.0");
    let negative = MODEL.replace("ground_squeeze = 0.5", "ground_squeeze = -0.0");
    let positive = XPlaneSimulatorModel::from_toml_str(&positive).expect("positive zero");
    let negative = XPlaneSimulatorModel::from_toml_str(&negative).expect("negative zero");
    assert_eq!(
        positive.canonical_digest().expect("positive digest"),
        negative.canonical_digest().expect("negative digest")
    );
}

#[test]
fn alia_model_identity_is_pinned() {
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    assert_eq!(
        model.canonical_digest().expect("model digest").to_string(),
        "74ead3977eb71ba0fbcd075a926a3de3d628c19573e88cf6899471e19e28e3f8"
    );
}

#[test]
fn formatting_does_not_change_the_semantic_identity() {
    let first = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let second = XPlaneSimulatorModel::from_toml_str(&MODEL.replace(" = ", "=")).expect("valid");
    assert_eq!(
        first.canonical_digest().expect("first digest"),
        second.canonical_digest().expect("second digest")
    );
}

#[test]
fn every_model_field_changes_the_identity() {
    let base = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let base_digest = base.canonical_digest().expect("base digest");
    let mutations = [
        MODEL.replace(
            "model_id = \"alia250-xplane-12\"",
            "model_id = \"alia250-xplane-12b\"",
        ),
        MODEL.replace("airframe_id = \"alia250\"", "airframe_id = \"alia250b\""),
        MODEL.replace(
            "73e66d145271b45b7ab637333c6c217a339b22e0b15eeed0e9b6256183c9e068",
            "83e66d145271b45b7ab637333c6c217a339b22e0b15eeed0e9b6256183c9e068",
        ),
        MODEL.replace(
            "aircraft_id = \"xplane12-laminar-alia250\"",
            "aircraft_id = \"xplane12-laminar-alia250b\"",
        ),
        MODEL.replace(
            "simulator_id = \"xplane-12\"",
            "simulator_id = \"xplane-12b\"",
        ),
        MODEL.replace(
            "3dd71fafb0f887eec94805d5365ae90c33b67e8a54266f20c79f6a198b5769f1",
            "4dd71fafb0f887eec94805d5365ae90c33b67e8a54266f20c79f6a198b5769f1",
        ),
        MODEL.replace(
            "mixer_geometry = \"quad-x-x500-reversed-spin\"",
            "mixer_geometry = \"quad-x-x500\"",
        ),
        MODEL.replace(
            "actuator_curve = \"quadratic-rotor\"",
            "actuator_curve = \"linear\"",
        ),
        MODEL.replace("sample_rate_hz = 100", "sample_rate_hz = 101"),
        MODEL.replace(
            "max_samples_per_iteration = 32",
            "max_samples_per_iteration = 33",
        ),
        MODEL.replace("lane_order = [0, 2, 1, 3]", "lane_order = [0, 1, 2, 3]"),
        MODEL.replace("rise_per_s = 0.035", "rise_per_s = 0.036"),
        MODEL.replace("band_boundary = 0.40", "band_boundary = 0.41"),
        MODEL.replace("low_band_rise_per_s = 0.15", "low_band_rise_per_s = 0.16"),
        MODEL.replace("fall_per_s = 0.30", "fall_per_s = 0.31"),
        MODEL.replace("mean_ceiling = 0.55", "mean_ceiling = 0.56"),
        MODEL.replace("lane_ceiling = 0.85", "lane_ceiling = 0.86"),
        MODEL.replace("airborne_clearance_m = 0.5", "airborne_clearance_m = 0.6"),
        MODEL.replace("ground_squeeze = 0.5", "ground_squeeze = 0.6"),
        MODEL.replace("max_sample_dt_s = 0.05", "max_sample_dt_s = 0.06"),
    ];
    for text in mutations {
        let model = XPlaneSimulatorModel::from_toml_str(&text).expect("valid mutation");
        assert_ne!(model.canonical_digest().expect("digest"), base_digest);
    }
}

#[test]
fn invariant_transport_fields_fail_closed() {
    for text in [
        MODEL.replace("motor_count = 4", "motor_count = 3"),
        MODEL.replace(
            "bridge_protocol = \"mavlink-hil-tcp-v1\"",
            "bridge_protocol = \"udp\"",
        ),
    ] {
        assert!(XPlaneSimulatorModel::from_toml_str(&text).is_err());
    }
}

#[test]
fn invalid_lane_order_fails_closed() {
    let text = MODEL.replace("lane_order = [0, 2, 1, 3]", "lane_order = [0, 0, 1, 3]");
    assert_eq!(
        XPlaneSimulatorModel::from_toml_str(&text),
        Err(XPlaneModelError::InvalidLaneOrder)
    );
}

#[test]
fn protection_relations_fail_closed() {
    let text = MODEL.replace("mean_ceiling = 0.55", "mean_ceiling = 0.90");
    assert!(matches!(
        XPlaneSimulatorModel::from_toml_str(&text),
        Err(XPlaneModelError::InvalidRelation(_))
    ));
}

#[test]
fn unknown_fields_fail_closed() {
    let text = format!("{MODEL}\noptimizer_may_change_wire = true\n");
    assert!(matches!(
        XPlaneSimulatorModel::from_toml_str(&text),
        Err(XPlaneModelError::Parse(_))
    ));
}
