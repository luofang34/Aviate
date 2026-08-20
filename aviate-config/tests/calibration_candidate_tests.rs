//! Calibration candidate resolution tests.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_config::airframe_preset::{resolve_candidate, CandidateError, ContentDigest};

const ALIA250: &str = include_str!("../../presets/alia250.toml");
const PLANT_DIGEST: &str = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

fn candidate(extra: &str) -> String {
    format!(
        "schema_version = 1\ncandidate_id = \"test-1\"\nbase_preset_digest = \"{}\"\nplant_artifact_digest = \"{PLANT_DIGEST}\"\n{extra}",
        ContentDigest::calculate(ALIA250.as_bytes())
    )
}

#[test]
fn candidate_changes_only_allowlisted_fields() {
    let text = candidate(
        "hover_thrust_seed = 0.44\n[gains]\natt_p = [0.7, 0.7, 0.3]\nrate_p = [0.8, 1.4, 2.0]\n",
    );
    let resolved = resolve_candidate(ALIA250, &text).expect("valid candidate");
    assert_eq!(resolved.preset.hover_thrust_force_seed(), 0.44);
    assert_eq!(resolved.preset.gains.att_p, [0.7, 0.7, 0.3]);
    assert_eq!(resolved.preset.gains.rate_p, [0.8, 1.4, 2.0]);
    assert_eq!(resolved.preset.mixer.motor_count(), 4);
    assert_eq!(
        resolved.identity.candidate,
        ContentDigest::calculate(text.as_bytes())
    );
}

#[test]
fn candidate_rejects_a_different_base_preset() {
    let text = candidate("").replace(
        &ContentDigest::calculate(ALIA250.as_bytes()).to_string(),
        PLANT_DIGEST,
    );
    assert_eq!(
        resolve_candidate(ALIA250, &text),
        Err(CandidateError::BasePresetMismatch)
    );
}

#[test]
fn candidate_rejects_unknown_fields() {
    let text = candidate("mixer = \"quad-x\"\n");
    assert!(matches!(
        resolve_candidate(ALIA250, &text),
        Err(CandidateError::Parse(_))
    ));
}

#[test]
fn candidate_rejects_non_finite_and_excessive_values() {
    for extra in [
        "hover_thrust_seed = nan\n",
        "[gains]\natt_p = [0.8, 0.8, 21.0]\n",
        "[gains]\nvel_max_roll_pitch = 1.3\n",
    ] {
        let text = candidate(extra);
        assert!(matches!(
            resolve_candidate(ALIA250, &text),
            Err(CandidateError::FieldOutOfRange(_)) | Err(CandidateError::Parse(_))
        ));
    }
}

#[test]
fn candidate_rejects_an_invalid_plant_identity() {
    let text = candidate("").replace(PLANT_DIGEST, "not-a-digest");
    assert_eq!(
        resolve_candidate(ALIA250, &text),
        Err(CandidateError::InvalidDigest {
            field: "plant_artifact_digest"
        })
    );
}
