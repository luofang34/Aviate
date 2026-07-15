//! AirframePreset contract tests (#75, #140): the shipped x500
//! preset parses and validates; every rejection rule actually
//! rejects; the hover seed reaches consumers force-domain only.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_config::airframe_preset::{preset_from_toml_str, ActuatorCurve, MixerKind, PresetError};

const X500: &str = include_str!("../../presets/x500.toml");

#[test]
fn shipped_x500_preset_parses_and_validates() {
    let p = preset_from_toml_str(X500).expect("x500 preset must be valid");
    assert_eq!(p.schema_version, 1);
    assert_eq!(p.name, "x500");
    assert_eq!(p.mixer, MixerKind::QuadXX500);
    assert_eq!(p.motor_count, 4);
    assert_eq!(p.actuator_curve, ActuatorCurve::Quadratic);
    // V1 quadratic seed 0.77 is a boundary (rotor-speed) value; the
    // only accessor converts it explicitly to force: 0.77² = 0.5929.
    assert!((p.hover_thrust_force_seed() - 0.5929).abs() < 1e-6);
    assert_eq!(p.gains.att_p, [3.5, 3.5, 2.5]);
    assert!((p.limits.max_altitude - 100.0).abs() < 1e-6);
}

#[test]
fn unknown_fields_are_rejected() {
    // An ALGORITHM_ID smuggled through the preset must fail parsing —
    // algorithm identity is never data.
    let text = X500.replace(
        "schema_version = 1",
        "schema_version = 1\nalgorithm_id = 42",
    );
    let err = preset_from_toml_str(&text).expect_err("unknown field must fail");
    assert!(err.contains("algorithm_id"), "error names the field: {err}");
}

#[test]
fn unknown_mixer_kind_is_rejected() {
    let text = X500.replace("mixer = \"quad-x-x500\"", "mixer = \"hex-plus\"");
    preset_from_toml_str(&text).expect_err("unregistered mixer must fail");
}

#[test]
fn unknown_schema_version_is_rejected() {
    let text = X500.replace("schema_version = 1", "schema_version = 3");
    let err = preset_from_toml_str(&text).expect_err("schema 3 must fail this parser");
    assert!(err.contains("schema_version"), "{err}");
}

#[test]
fn schema_2_seed_is_force_domain_verbatim() {
    // Schema 2 defines the seed as force: no conversion applies.
    let text = X500
        .replace("schema_version = 1", "schema_version = 2")
        .replace("hover_thrust_seed = 0.77", "hover_thrust_seed = 0.5929");
    let p = preset_from_toml_str(&text).expect("schema 2 must parse");
    assert!((p.hover_thrust_force_seed() - 0.5929).abs() < 1e-6);
}

#[test]
fn v1_linear_seed_passes_through_unconverted() {
    // A linear plant's V1 boundary command already IS the thrust
    // fraction — the explicit conversion is the identity there.
    let text = X500.replace(
        "actuator_curve = \"quadratic\"",
        "actuator_curve = \"linear\"",
    );
    let p = preset_from_toml_str(&text).expect("linear V1 must parse");
    assert!((p.hover_thrust_force_seed() - 0.77).abs() < 1e-6);
}

#[test]
fn motor_count_must_match_mixer_geometry() {
    let text = X500.replace("motor_count = 4", "motor_count = 6");
    let err = preset_from_toml_str(&text).expect_err("mismatched motor count must fail");
    assert!(err.contains("motor_count"), "{err}");
}

#[test]
fn non_finite_and_out_of_range_values_are_rejected() {
    for (from, to) in [
        ("hover_thrust_seed = 0.77", "hover_thrust_seed = nan"),
        ("hover_thrust_seed = 0.77", "hover_thrust_seed = 1.2"),
        ("rate_d_lpf_alpha = 0.5", "rate_d_lpf_alpha = inf"),
        ("att_p = [3.5, 3.5, 2.5]", "att_p = [3.5, -1.0, 2.5]"),
        ("max_altitude = 100.0", "max_altitude = -5.0"),
    ] {
        let text = X500.replace(from, to);
        assert_ne!(text, X500, "replacement must apply: {from}");
        preset_from_toml_str(&text)
            .map(|_| panic!("must reject: {to}"))
            .ok();
    }
}

#[test]
fn validation_error_variants_render_their_context() {
    assert!(PresetError::UnsupportedSchema { found: 3 }
        .to_string()
        .contains('3'));
    assert!(PresetError::MotorCountMismatch {
        declared: 6,
        required: 4
    }
    .to_string()
    .contains('6'));
}

/// Until the app-owned kernel construction (#120A) makes the preset
/// file the authoritative source, the shipped x500 preset and the
/// compiled `CascadeGains::x500_defaults()` are two copies of the
/// same tuning — this pins them equal so they cannot drift apart in
/// the interim.
#[test]
fn x500_preset_matches_compiled_defaults() {
    use aviate_core::control::cascade_gains::CascadeGains;
    let p = preset_from_toml_str(X500).expect("valid");
    let d = CascadeGains::x500_defaults();
    assert_eq!(p.gains.pos_p, d.pos_p);
    assert_eq!(p.gains.pos_accel_limits, d.pos_accel_limits);
    assert_eq!(p.gains.pos_vel_caps, d.pos_vel_caps);
    assert_eq!(p.gains.vel_p, d.vel_p);
    assert_eq!(p.gains.vel_i, d.vel_i);
    assert_eq!(p.gains.vel_d, d.vel_d);
    assert_eq!(p.gains.vel_max_roll_pitch, d.vel_max_roll_pitch);
    assert_eq!(p.gains.vel_accel_ff, d.vel_accel_ff);
    assert_eq!(p.gains.att_p, d.att_p);
    assert_eq!(p.gains.rate_p, d.rate_p);
    assert_eq!(p.gains.rate_d, d.rate_d);
    assert_eq!(p.gains.rate_d_lpf_alpha, d.rate_d_lpf_alpha);
}
