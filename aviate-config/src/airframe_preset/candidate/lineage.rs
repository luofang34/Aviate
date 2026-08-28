//! Immutable cumulative calibration overlay lineage.

use alloc::vec::Vec;

use crate::xplane_model::XPlaneSimulatorModel;

use super::design::apply_validated_layer;
use super::{
    parse_digest, AirframePreset, CalibrationCandidate, CalibrationOverlay, CandidateError,
    CandidateLayer, ContentDigest, GainOverrides, PlantIdentificationArtifact,
    LEGACY_CANDIDATE_SCHEMA_VERSION,
};

pub(super) fn apply_candidate_layers(
    preset: &mut AirframePreset,
    candidate: &CalibrationCandidate,
    base_digest: ContentDigest,
    plant_digest: ContentDigest,
    plant: &PlantIdentificationArtifact,
    model: &XPlaneSimulatorModel,
    document_digest: ContentDigest,
) -> Result<ContentDigest, CandidateError> {
    if candidate.schema_version == LEGACY_CANDIDATE_SCHEMA_VERSION {
        apply_validated_layer(preset, CandidateLayer::legacy(candidate)?, plant, model)?;
        return Ok(document_digest);
    }
    let mut lineage = base_digest;
    for (index, overlay) in candidate.overlays.iter().enumerate() {
        let parent = parse_digest(&overlay.parent_digest, "overlays.parent_digest")?;
        if parent != lineage {
            return Err(CandidateError::LineageMismatch(index));
        }
        apply_validated_layer(preset, CandidateLayer::overlay(overlay), plant, model)?;
        lineage = calculate_overlay_lineage_digest(lineage, plant_digest, overlay);
    }
    Ok(lineage)
}

/// Calculate the next immutable cumulative overlay identity.
#[must_use]
pub fn calculate_overlay_lineage_digest(
    parent: ContentDigest,
    plant_artifact: ContentDigest,
    overlay: &CalibrationOverlay,
) -> ContentDigest {
    let mut bytes = b"aviate-candidate-overlay-v1".to_vec();
    bytes.extend_from_slice(parent.as_bytes());
    bytes.extend_from_slice(plant_artifact.as_bytes());
    push_text(&mut bytes, &overlay.overlay_id);
    bytes.push(overlay.stage as u8);
    push_option_scalar(&mut bytes, overlay.hover_thrust_seed);
    if let Some(design) = overlay.inner_loop {
        bytes.push(1);
        push_triple(&mut bytes, design.natural_frequency_rad_s);
        push_triple(&mut bytes, design.loop_separation);
    } else {
        bytes.push(0);
    }
    push_gain_overrides(&mut bytes, overlay.gains);
    ContentDigest::calculate(&bytes)
}

fn push_gain_overrides(bytes: &mut Vec<u8>, gains: GainOverrides) {
    for values in [
        gains.pos_p,
        gains.pos_accel_limits,
        gains.pos_vel_caps,
        gains.vel_p,
        gains.vel_i,
        gains.vel_d,
        gains.rate_i,
        gains.rate_d,
    ] {
        push_option_triple(bytes, values);
    }
    for value in [
        gains.vel_max_roll_pitch,
        gains.vel_max_yaw_step,
        gains.vel_accel_ff,
        gains.att_max_rate_cmd,
        gains.rate_d_lpf_alpha,
    ] {
        push_option_scalar(bytes, value);
    }
}

fn push_option_triple(bytes: &mut Vec<u8>, value: Option<[f32; 3]>) {
    if let Some(values) = value {
        bytes.push(1);
        push_triple(bytes, values);
    } else {
        bytes.push(0);
    }
}

fn push_option_scalar(bytes: &mut Vec<u8>, value: Option<f32>) {
    if let Some(value) = value {
        bytes.push(1);
        push_float(bytes, value);
    } else {
        bytes.push(0);
    }
}

fn push_triple(bytes: &mut Vec<u8>, values: [f32; 3]) {
    for value in values {
        push_float(bytes, value);
    }
}

fn push_float(bytes: &mut Vec<u8>, value: f32) {
    let canonical = if value == 0.0 { 0.0 } else { value };
    bytes.extend_from_slice(&canonical.to_bits().to_le_bytes());
}

fn push_text(bytes: &mut Vec<u8>, value: &str) {
    bytes.extend_from_slice(&(value.len() as u64).to_le_bytes());
    bytes.extend_from_slice(value.as_bytes());
}
