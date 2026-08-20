//! Immutable calibration candidates for one airframe preset.

use alloc::string::{String, ToString};
use core::fmt;

use serde::Deserialize;
use sha2::{Digest, Sha256};

use super::{preset_from_toml_str, AirframePreset, PresetError};

const CANDIDATE_SCHEMA_VERSION: u16 = 1;
const MAX_ID_BYTES: usize = 64;

/// SHA-256 identity for one exact artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ContentDigest([u8; 32]);

impl ContentDigest {
    /// Calculate the identity of an artifact.
    #[must_use]
    pub fn calculate(bytes: &[u8]) -> Self {
        Self(Sha256::digest(bytes).into())
    }

    /// Parse a lowercase or uppercase hexadecimal identity.
    ///
    /// # Errors
    ///
    /// Returns an error when the text is not one SHA-256 value.
    pub fn from_hex(text: &str) -> Result<Self, CandidateError> {
        if text.len() != 64 || !text.as_bytes().iter().all(u8::is_ascii_hexdigit) {
            return Err(CandidateError::InvalidDigest { field: "digest" });
        }
        let mut bytes = [0_u8; 32];
        for (index, byte) in bytes.iter_mut().enumerate() {
            let offset = index * 2;
            *byte =
                (hex_value(text.as_bytes()[offset]) << 4) | hex_value(text.as_bytes()[offset + 1]);
        }
        Ok(Self(bytes))
    }

    /// Return the raw digest bytes.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl fmt::Display for ContentDigest {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(formatter, "{byte:02x}")?;
        }
        Ok(())
    }
}

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
    /// Attitude P gains.
    pub att_p: Option<[f32; 3]>,
    /// Attitude-loop rate limit.
    pub att_max_rate_cmd: Option<f32>,
    /// Rate P gains.
    pub rate_p: Option<[f32; 3]>,
    /// Rate I gains.
    pub rate_i: Option<[f32; 3]>,
    /// Rate D gains.
    pub rate_d: Option<[f32; 3]>,
    /// Rate D-term filter coefficient.
    pub rate_d_lpf_alpha: Option<f32>,
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
    /// Optional force-domain hover seed.
    pub hover_thrust_seed: Option<f32>,
    /// Allowlisted controller-gain changes.
    #[serde(default)]
    pub gains: GainOverrides,
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
}

/// One validated preset with immutable candidate identity.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedCandidate {
    /// Fully resolved preset.
    pub preset: AirframePreset,
    /// Candidate name.
    pub candidate_id: String,
    /// Identities required for a trial manifest.
    pub identity: CandidateIdentity,
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
    /// A candidate field exceeds the calibration boundary.
    FieldOutOfRange(&'static str),
    /// The resolved preset violates its base contract.
    ResolvedPreset(PresetError),
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
            Self::FieldOutOfRange(field) => {
                write!(
                    formatter,
                    "candidate field {field} is outside its permitted range"
                )
            }
            Self::ResolvedPreset(error) => write!(formatter, "resolved preset is invalid: {error}"),
            Self::InvalidBasePreset(error) => write!(formatter, "invalid base preset: {error}"),
        }
    }
}

impl core::error::Error for CandidateError {}

/// Resolve a candidate against the exact supplied base preset.
///
/// # Errors
///
/// Returns an error for malformed identities, non-allowlisted fields,
/// invalid bounds, or a base preset mismatch.
pub fn resolve_candidate(
    base_preset_text: &str,
    candidate_text: &str,
) -> Result<ResolvedCandidate, CandidateError> {
    let mut preset =
        preset_from_toml_str(base_preset_text).map_err(CandidateError::InvalidBasePreset)?;
    let candidate: CalibrationCandidate =
        toml::from_str(candidate_text).map_err(|error| CandidateError::Parse(error.to_string()))?;
    validate_candidate(&candidate)?;

    let base_digest = ContentDigest::calculate(base_preset_text.as_bytes());
    if parse_digest(&candidate.base_preset_digest, "base_preset_digest")? != base_digest {
        return Err(CandidateError::BasePresetMismatch);
    }
    apply_overrides(&mut preset, &candidate);
    validate_candidate_bounds(&preset)?;
    preset.validate().map_err(CandidateError::ResolvedPreset)?;

    Ok(ResolvedCandidate {
        preset,
        candidate_id: candidate.candidate_id,
        identity: CandidateIdentity {
            base_preset: base_digest,
            candidate: ContentDigest::calculate(candidate_text.as_bytes()),
            plant_artifact: parse_digest(
                &candidate.plant_artifact_digest,
                "plant_artifact_digest",
            )?,
        },
    })
}

fn validate_candidate(candidate: &CalibrationCandidate) -> Result<(), CandidateError> {
    if candidate.schema_version != CANDIDATE_SCHEMA_VERSION {
        return Err(CandidateError::UnsupportedSchema(candidate.schema_version));
    }
    let id = candidate.candidate_id.as_bytes();
    if id.is_empty()
        || id.len() > MAX_ID_BYTES
        || !id
            .iter()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'-' | b'_' | b'.'))
    {
        return Err(CandidateError::InvalidCandidateId);
    }
    parse_digest(&candidate.plant_artifact_digest, "plant_artifact_digest")?;
    Ok(())
}

fn apply_overrides(preset: &mut AirframePreset, candidate: &CalibrationCandidate) {
    if let Some(value) = candidate.hover_thrust_seed {
        preset.hover_thrust_seed = value;
    }
    let source = candidate.gains;
    let target = &mut preset.gains;
    assign(&mut target.pos_p, source.pos_p);
    assign(&mut target.pos_accel_limits, source.pos_accel_limits);
    assign(&mut target.pos_vel_caps, source.pos_vel_caps);
    assign(&mut target.vel_p, source.vel_p);
    assign(&mut target.vel_i, source.vel_i);
    assign(&mut target.vel_d, source.vel_d);
    assign(&mut target.vel_max_roll_pitch, source.vel_max_roll_pitch);
    assign(&mut target.vel_max_yaw_step, source.vel_max_yaw_step);
    assign(&mut target.vel_accel_ff, source.vel_accel_ff);
    assign(&mut target.att_p, source.att_p);
    assign(&mut target.att_max_rate_cmd, source.att_max_rate_cmd);
    assign(&mut target.rate_p, source.rate_p);
    assign(&mut target.rate_i, source.rate_i);
    assign(&mut target.rate_d, source.rate_d);
    assign(&mut target.rate_d_lpf_alpha, source.rate_d_lpf_alpha);
}

fn assign<T: Copy>(target: &mut T, source: Option<T>) {
    if let Some(value) = source {
        *target = value;
    }
}

fn validate_candidate_bounds(preset: &AirframePreset) -> Result<(), CandidateError> {
    bounded("hover_thrust_seed", preset.hover_thrust_seed, 0.1, 0.9)?;
    triples("gains.pos_p", preset.gains.pos_p, 0.0, 10.0)?;
    triples(
        "gains.pos_accel_limits",
        preset.gains.pos_accel_limits,
        0.05,
        20.0,
    )?;
    triples("gains.pos_vel_caps", preset.gains.pos_vel_caps, 0.05, 30.0)?;
    triples("gains.vel_p", preset.gains.vel_p, 0.0, 20.0)?;
    triples("gains.vel_i", preset.gains.vel_i, 0.0, 20.0)?;
    triples("gains.vel_d", preset.gains.vel_d, 0.0, 20.0)?;
    bounded(
        "gains.vel_max_roll_pitch",
        preset.gains.vel_max_roll_pitch,
        0.05,
        1.2,
    )?;
    bounded(
        "gains.vel_max_yaw_step",
        preset.gains.vel_max_yaw_step,
        0.0,
        3.2,
    )?;
    bounded("gains.vel_accel_ff", preset.gains.vel_accel_ff, 0.0, 1.0)?;
    triples("gains.att_p", preset.gains.att_p, 0.0, 20.0)?;
    bounded(
        "gains.att_max_rate_cmd",
        preset.gains.att_max_rate_cmd,
        0.1,
        10.0,
    )?;
    triples("gains.rate_p", preset.gains.rate_p, 0.0, 20.0)?;
    triples("gains.rate_i", preset.gains.rate_i, 0.0, 20.0)?;
    triples("gains.rate_d", preset.gains.rate_d, 0.0, 20.0)?;
    bounded(
        "gains.rate_d_lpf_alpha",
        preset.gains.rate_d_lpf_alpha,
        0.0,
        1.0,
    )
}

fn triples(
    field: &'static str,
    values: [f32; 3],
    lower: f32,
    upper: f32,
) -> Result<(), CandidateError> {
    for value in values {
        bounded(field, value, lower, upper)?;
    }
    Ok(())
}

fn bounded(field: &'static str, value: f32, lower: f32, upper: f32) -> Result<(), CandidateError> {
    if !value.is_finite() || !(lower..=upper).contains(&value) {
        return Err(CandidateError::FieldOutOfRange(field));
    }
    Ok(())
}

fn hex_value(byte: u8) -> u8 {
    match byte {
        b'0'..=b'9' => byte - b'0',
        b'a'..=b'f' => byte - b'a' + 10,
        b'A'..=b'F' => byte - b'A' + 10,
        _ => 0,
    }
}

fn parse_digest(text: &str, field: &'static str) -> Result<ContentDigest, CandidateError> {
    ContentDigest::from_hex(text).map_err(|_| CandidateError::InvalidDigest { field })
}
