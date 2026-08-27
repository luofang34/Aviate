//! Immutable calibration candidates for one airframe preset.

use alloc::{
    string::{String, ToString},
    vec::Vec,
};
use core::fmt;

use serde::Deserialize;

use super::{preset_from_toml_str, AirframePreset, PresetError};

mod design;
mod digest;
mod lineage;
mod plant;
use digest::parse_digest;
pub use digest::ContentDigest;
pub use lineage::calculate_overlay_lineage_digest;
pub use plant::{
    PlantArtifactError, PlantIdentificationArtifact, PlantSampleClock, MAX_SATURATION_FRACTION,
    MIN_COHERENCE,
};

use crate::xplane_model::XPlaneSimulatorModel;
use lineage::apply_candidate_layers;

const LEGACY_CANDIDATE_SCHEMA_VERSION: u16 = 1;
const LINEAGE_CANDIDATE_SCHEMA_VERSION: u16 = 2;
const MAX_ID_BYTES: usize = 64;
const MAX_OVERLAYS: usize = 5;

/// Optional gain changes permitted for a calibration candidate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GainOverrides {
    /// Position P gains.
    pub pos_p: Option<[f32; 3]>,
    /// Position acceleration limits.
    pub pos_accel_limits: Option<[f32; 3]>,
    /// Position velocity caps.
    pub pos_vel_caps: Option<[f32; 3]>,
    /// Velocity P gains.
    pub vel_p: Option<[f32; 3]>,
    /// Velocity I gains.
    pub vel_i: Option<[f32; 3]>,
    /// Velocity D gains.
    pub vel_d: Option<[f32; 3]>,
    /// Velocity-loop tilt limit.
    pub vel_max_roll_pitch: Option<f32>,
    /// Applied yaw-error limit.
    pub vel_max_yaw_step: Option<f32>,
    /// Acceleration feedforward scale.
    pub vel_accel_ff: Option<f32>,
    /// Attitude-loop rate limit.
    pub att_max_rate_cmd: Option<f32>,
    /// Rate I gains.
    pub rate_i: Option<[f32; 3]>,
    /// Rate D gains.
    pub rate_d: Option<[f32; 3]>,
    /// Rate D-term filter coefficient.
    pub rate_d_lpf_alpha: Option<f32>,
}

/// One immutable stage overlay in a cumulative candidate lineage.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibrationOverlay {
    /// Stable overlay name.
    pub overlay_id: String,
    /// Base preset or previous overlay lineage identity.
    pub parent_digest: String,
    /// Tuning stage owned by this overlay.
    pub stage: CalibrationStage,
    /// Optional force-domain hover seed.
    pub hover_thrust_seed: Option<f32>,
    /// Plant-based inner-loop design coordinates.
    pub inner_loop: Option<InnerLoopDesign>,
    /// Allowlisted controller-gain changes.
    #[serde(default)]
    pub gains: GainOverrides,
}

#[derive(Clone, Copy)]
pub(super) struct CandidateLayer {
    pub(super) stage: CalibrationStage,
    pub(super) hover_thrust_seed: Option<f32>,
    pub(super) inner_loop: Option<InnerLoopDesign>,
    pub(super) gains: GainOverrides,
}

impl CandidateLayer {
    fn legacy(candidate: &CalibrationCandidate) -> Result<Self, CandidateError> {
        Ok(Self {
            stage: candidate.stage.ok_or(CandidateError::InvalidRelation(
                "schema one candidate requires one stage",
            ))?,
            hover_thrust_seed: candidate.hover_thrust_seed,
            inner_loop: candidate.inner_loop,
            gains: candidate.gains,
        })
    }

    fn overlay(overlay: &CalibrationOverlay) -> Self {
        Self {
            stage: overlay.stage,
            hover_thrust_seed: overlay.hover_thrust_seed,
            inner_loop: overlay.inner_loop,
            gains: overlay.gains,
        }
    }
}

/// Evidence-based roll, pitch, and yaw loop design coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InnerLoopDesign {
    /// Target closed-loop natural frequency in radians per second.
    pub natural_frequency_rad_s: [f32; 3],
    /// Required rate-loop to attitude-loop separation.
    pub loop_separation: [f32; 3],
}

/// One bounded calibration stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
#[repr(u8)]
pub enum CalibrationStage {
    /// Derive attitude and rate P gains from plant evidence.
    InnerLoop,
    /// Change the rate I, D, and filter values.
    RateIntegralDerivative,
    /// Change position and velocity loop values.
    OuterLoop,
    /// Change command-envelope values.
    CommandEnvelope,
    /// Change the hover force seed.
    Hover,
}

/// One strict calibration candidate document.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibrationCandidate {
    /// Candidate schema version.
    pub schema_version: u16,
    /// Stable candidate name.
    pub candidate_id: String,
    /// SHA-256 identity of the exact base preset text.
    pub base_preset_digest: String,
    /// SHA-256 identity of the plant-identification artifact.
    pub plant_artifact_digest: String,
    /// Tuning stage that owns all changes in this candidate.
    pub stage: Option<CalibrationStage>,
    /// Optional force-domain hover seed.
    pub hover_thrust_seed: Option<f32>,
    /// Plant-based inner-loop design coordinates.
    pub inner_loop: Option<InnerLoopDesign>,
    /// Allowlisted controller-gain changes.
    #[serde(default)]
    pub gains: GainOverrides,
    /// Ordered cumulative overlays for schema version two.
    #[serde(default)]
    pub overlays: Vec<CalibrationOverlay>,
}

/// Typed identities that bind a resolved candidate to its inputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CandidateIdentity {
    /// Base preset identity.
    pub base_preset: ContentDigest,
    /// Candidate document identity.
    pub candidate: ContentDigest,
    /// Plant-identification artifact identity.
    pub plant_artifact: ContentDigest,
    /// Final cumulative overlay lineage identity.
    pub lineage: ContentDigest,
}

/// One validated preset with immutable candidate identity.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedCandidate {
    preset: AirframePreset,
    candidate_id: String,
    identity: CandidateIdentity,
    plant_artifact: PlantIdentificationArtifact,
}

impl ResolvedCandidate {
    /// Consume the validated value into immutable kernel-construction inputs.
    #[must_use]
    pub fn into_parts(
        self,
    ) -> (
        AirframePreset,
        String,
        CandidateIdentity,
        PlantIdentificationArtifact,
    ) {
        (
            self.preset,
            self.candidate_id,
            self.identity,
            self.plant_artifact,
        )
    }
}

/// A calibration candidate error.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CandidateError {
    /// The candidate document cannot be decoded.
    Parse(String),
    /// The candidate schema version is not supported.
    UnsupportedSchema(u16),
    /// The candidate name is not portable ASCII.
    InvalidCandidateId,
    /// A digest is not one SHA-256 value.
    InvalidDigest {
        /// Name of the invalid digest field.
        field: &'static str,
    },
    /// The declared base identity does not match the supplied preset.
    BasePresetMismatch,
    /// The base preset schema is not valid for force-domain calibration.
    UnsupportedBaseSchema(u32),
    /// The declared plant identity does not match the supplied artifact.
    PlantArtifactMismatch,
    /// The plant artifact is invalid.
    PlantArtifact(PlantArtifactError),
    /// The plant evidence names another airframe.
    PlantAirframeMismatch,
    /// The plant evidence names another simulator model.
    PlantSimulatorModelMismatch,
    /// A candidate field exceeds the calibration boundary.
    FieldOutOfRange(&'static str),
    /// Related candidate fields violate a dynamic constraint.
    InvalidRelation(&'static str),
    /// The resolved preset violates its base contract.
    ResolvedPreset(PresetError),
    /// One cumulative overlay does not name its exact parent.
    LineageMismatch(usize),
    /// One stage appears more than once in a cumulative candidate.
    DuplicateStage(CalibrationStage),
    /// The base preset cannot be decoded.
    InvalidBasePreset(String),
}

impl fmt::Display for CandidateError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(error) => write!(formatter, "cannot parse calibration candidate: {error}"),
            Self::UnsupportedSchema(version) => {
                write!(formatter, "unsupported candidate schema version {version}")
            }
            Self::InvalidCandidateId => formatter.write_str("invalid calibration candidate_id"),
            Self::InvalidDigest { field } => {
                write!(formatter, "invalid calibration artifact digest in {field}")
            }
            Self::BasePresetMismatch => formatter.write_str("candidate base preset does not match"),
            Self::UnsupportedBaseSchema(version) => {
                write!(formatter, "unsupported calibration base schema {version}")
            }
            Self::PlantArtifactMismatch => {
                formatter.write_str("candidate plant artifact does not match")
            }
            Self::PlantArtifact(error) => write!(formatter, "plant artifact is invalid: {error}"),
            Self::PlantAirframeMismatch => {
                formatter.write_str("plant artifact airframe does not match the preset")
            }
            Self::PlantSimulatorModelMismatch => {
                formatter.write_str("plant artifact simulator model does not match the run")
            }
            Self::FieldOutOfRange(field) => {
                write!(
                    formatter,
                    "candidate field {field} is outside its permitted range"
                )
            }
            Self::InvalidRelation(relation) => {
                write!(formatter, "candidate violates dynamic relation {relation}")
            }
            Self::ResolvedPreset(error) => write!(formatter, "resolved preset is invalid: {error}"),
            Self::LineageMismatch(index) => {
                write!(
                    formatter,
                    "candidate overlay {index} does not match its parent"
                )
            }
            Self::DuplicateStage(stage) => {
                write!(
                    formatter,
                    "candidate stage {stage:?} appears more than once"
                )
            }
            Self::InvalidBasePreset(error) => write!(formatter, "invalid base preset: {error}"),
        }
    }
}

impl core::error::Error for CandidateError {}

impl From<PlantArtifactError> for CandidateError {
    fn from(error: PlantArtifactError) -> Self {
        Self::PlantArtifact(error)
    }
}

/// Resolve a candidate against the exact supplied base preset.
///
/// # Errors
///
/// Returns an error for malformed identities, non-allowlisted fields,
/// invalid bounds, or a base preset mismatch.
pub fn resolve_candidate(
    base_preset_text: &str,
    candidate_text: &str,
    plant_artifact_text: &str,
    simulator_model: &XPlaneSimulatorModel,
) -> Result<ResolvedCandidate, CandidateError> {
    let mut preset =
        preset_from_toml_str(base_preset_text).map_err(CandidateError::InvalidBasePreset)?;
    if preset.schema_version != 2 {
        return Err(CandidateError::UnsupportedBaseSchema(preset.schema_version));
    }
    let candidate: CalibrationCandidate =
        toml::from_str(candidate_text).map_err(|error| CandidateError::Parse(error.to_string()))?;
    validate_candidate(&candidate)?;

    let plant_artifact = PlantIdentificationArtifact::from_toml_str(plant_artifact_text)?;
    let actual_plant_digest = PlantIdentificationArtifact::content_digest(plant_artifact_text);
    if parse_digest(&candidate.plant_artifact_digest, "plant_artifact_digest")?
        != actual_plant_digest
    {
        return Err(CandidateError::PlantArtifactMismatch);
    }
    if plant_artifact.airframe_id != preset.name {
        return Err(CandidateError::PlantAirframeMismatch);
    }
    let expected_simulator_model_digest = ContentDigest::from_hex(
        &simulator_model
            .canonical_digest()
            .map_err(|error| CandidateError::InvalidBasePreset(error.to_string()))?
            .to_string(),
    )?;
    if parse_digest(
        &plant_artifact.simulator_model_digest,
        "simulator_model_digest",
    )? != expected_simulator_model_digest
    {
        return Err(CandidateError::PlantSimulatorModelMismatch);
    }

    let base_digest = ContentDigest::calculate(base_preset_text.as_bytes());
    if parse_digest(&candidate.base_preset_digest, "base_preset_digest")? != base_digest {
        return Err(CandidateError::BasePresetMismatch);
    }
    let candidate_document_digest = ContentDigest::calculate(candidate_text.as_bytes());
    let lineage = apply_candidate_layers(
        &mut preset,
        &candidate,
        base_digest,
        actual_plant_digest,
        &plant_artifact,
        simulator_model,
        candidate_document_digest,
    )?;

    Ok(ResolvedCandidate {
        preset,
        candidate_id: candidate.candidate_id,
        identity: CandidateIdentity {
            base_preset: base_digest,
            candidate: candidate_document_digest,
            plant_artifact: parse_digest(
                &candidate.plant_artifact_digest,
                "plant_artifact_digest",
            )?,
            lineage,
        },
        plant_artifact,
    })
}

fn validate_candidate(candidate: &CalibrationCandidate) -> Result<(), CandidateError> {
    if !matches!(
        candidate.schema_version,
        LEGACY_CANDIDATE_SCHEMA_VERSION | LINEAGE_CANDIDATE_SCHEMA_VERSION
    ) {
        return Err(CandidateError::UnsupportedSchema(candidate.schema_version));
    }
    validate_candidate_id(&candidate.candidate_id)?;
    parse_digest(&candidate.base_preset_digest, "base_preset_digest")?;
    parse_digest(&candidate.plant_artifact_digest, "plant_artifact_digest")?;
    match candidate.schema_version {
        LEGACY_CANDIDATE_SCHEMA_VERSION => validate_legacy_shape(candidate),
        LINEAGE_CANDIDATE_SCHEMA_VERSION => validate_lineage_shape(candidate),
        _ => Err(CandidateError::UnsupportedSchema(candidate.schema_version)),
    }
}

fn validate_legacy_shape(candidate: &CalibrationCandidate) -> Result<(), CandidateError> {
    if candidate.stage.is_none() || !candidate.overlays.is_empty() {
        return Err(CandidateError::InvalidRelation(
            "schema one candidate must contain one legacy stage",
        ));
    }
    Ok(())
}

fn validate_lineage_shape(candidate: &CalibrationCandidate) -> Result<(), CandidateError> {
    if candidate.stage.is_some()
        || candidate.hover_thrust_seed.is_some()
        || candidate.inner_loop.is_some()
        || candidate.gains != GainOverrides::default()
        || candidate.overlays.is_empty()
        || candidate.overlays.len() > MAX_OVERLAYS
    {
        return Err(CandidateError::InvalidRelation(
            "schema two candidate must contain only cumulative overlays",
        ));
    }
    let mut stages = [false; 5];
    for overlay in &candidate.overlays {
        validate_candidate_id(&overlay.overlay_id)?;
        parse_digest(&overlay.parent_digest, "overlays.parent_digest")?;
        let slot = overlay.stage as usize;
        if stages[slot] {
            return Err(CandidateError::DuplicateStage(overlay.stage));
        }
        stages[slot] = true;
    }
    Ok(())
}

fn validate_candidate_id(value: &str) -> Result<(), CandidateError> {
    let id = value.as_bytes();
    if id.is_empty()
        || id.len() > MAX_ID_BYTES
        || !id
            .iter()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'-' | b'_' | b'.'))
    {
        return Err(CandidateError::InvalidCandidateId);
    }
    Ok(())
}
