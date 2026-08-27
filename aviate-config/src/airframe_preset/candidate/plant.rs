//! Validated plant-identification evidence for calibration candidates.

use alloc::{
    format,
    string::{String, ToString},
};
use core::fmt;

use serde::Deserialize;

use super::ContentDigest;

const PLANT_ARTIFACT_SCHEMA_VERSION: u16 = 1;
const MAX_ID_BYTES: usize = 64;

/// Per-axis evidence from one plant-identification run.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlantIdentificationArtifact {
    /// Plant artifact schema version.
    pub schema_version: u16,
    /// Stable artifact name.
    pub artifact_id: String,
    /// Airframe preset name used by the experiment.
    pub airframe_id: String,
    /// Simulator model identity used by the experiment.
    pub simulator_model_digest: String,
    /// Exact run manifest used by the identification process.
    pub run_manifest_digest: String,
    /// Exact trace used to calculate this artifact.
    pub trace_digest: String,
    /// Sample clock used by the trace.
    pub sample_clock: PlantSampleClock,
    /// Mean force-domain collective during the probe windows.
    pub operating_hover_force: f32,
    /// Probe frequencies in radians per second.
    pub probe_rad_s: [f32; 2],
    /// Observed sample rate in hertz.
    pub sample_rate_hz: f32,
    /// Positive angular authority for roll, pitch, and yaw.
    pub authority_k: [f32; 3],
    /// Estimated transport delay in seconds for roll, pitch, and yaw.
    pub delay_s: [f32; 3],
    /// Half-width of the 95 percent delay interval for each axis.
    pub delay_ci95_s: [f32; 3],
    /// Coefficient of determination for each fitted axis.
    pub r_squared: [f32; 3],
    /// Half-width of the 95 percent authority interval for each axis.
    pub authority_ci95: [f32; 3],
    /// Block coherence for each fitted axis.
    pub coherence: [f32; 3],
    /// Largest applied normalized axis input for each axis.
    pub applied_input_max: [f32; 3],
    /// Number of valid samples for each axis.
    pub sample_count: [u32; 3],
    /// Fraction of samples affected by wire or actuator saturation.
    pub saturation_fraction: [f32; 3],
    /// Response sign for roll, pitch, and yaw. Each value is minus one or one.
    pub response_sign: [i8; 3],
}

/// Clock domain used by a plant-identification trace.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PlantSampleClock {
    /// Microseconds from the simulator sensor stream.
    SimulatorMicroseconds,
}

/// A plant-identification artifact error.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlantArtifactError {
    /// The TOML document cannot be decoded.
    Parse(String),
    /// The artifact schema is not supported.
    UnsupportedSchema(u16),
    /// A stable name is invalid.
    InvalidId(&'static str),
    /// An artifact identity is not one SHA-256 value.
    InvalidDigest(&'static str),
    /// A numeric field is outside its valid range.
    FieldOutOfRange(&'static str),
    /// An axis has too few valid samples.
    InsufficientSamples(usize),
    /// An axis has an invalid response sign.
    InvalidResponseSign(usize),
    /// An authority estimate is too uncertain.
    ExcessiveUncertainty(usize),
}

impl fmt::Display for PlantArtifactError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(error) => write!(formatter, "cannot parse plant artifact: {error}"),
            Self::UnsupportedSchema(version) => {
                write!(formatter, "unsupported plant artifact schema {version}")
            }
            Self::InvalidId(field) => write!(formatter, "invalid plant artifact field {field}"),
            Self::InvalidDigest(field) => {
                write!(formatter, "invalid plant artifact digest {field}")
            }
            Self::FieldOutOfRange(field) => {
                write!(
                    formatter,
                    "plant artifact field {field} is outside its range"
                )
            }
            Self::InsufficientSamples(axis) => {
                write!(formatter, "plant artifact axis {axis} has too few samples")
            }
            Self::InvalidResponseSign(axis) => {
                write!(
                    formatter,
                    "plant artifact axis {axis} has an invalid response sign"
                )
            }
            Self::ExcessiveUncertainty(axis) => {
                write!(
                    formatter,
                    "plant artifact axis {axis} has excessive uncertainty"
                )
            }
        }
    }
}

impl core::error::Error for PlantArtifactError {}

impl PlantIdentificationArtifact {
    /// Decode and validate a strict plant artifact.
    ///
    /// # Errors
    ///
    /// Returns an error for unknown fields, weak evidence, or invalid values.
    pub fn from_toml_str(text: &str) -> Result<Self, PlantArtifactError> {
        let artifact: Self =
            toml::from_str(text).map_err(|error| PlantArtifactError::Parse(error.to_string()))?;
        artifact.validate()?;
        Ok(artifact)
    }

    /// Validate the evidence required by the candidate resolver.
    ///
    /// # Errors
    ///
    /// Returns the first evidence failure.
    pub fn validate(&self) -> Result<(), PlantArtifactError> {
        if self.schema_version != PLANT_ARTIFACT_SCHEMA_VERSION {
            return Err(PlantArtifactError::UnsupportedSchema(self.schema_version));
        }
        validate_id("artifact_id", &self.artifact_id)?;
        validate_id("airframe_id", &self.airframe_id)?;
        ContentDigest::from_hex(&self.simulator_model_digest)
            .map_err(|_| PlantArtifactError::InvalidDigest("simulator_model_digest"))?;
        for (field, digest) in [
            ("run_manifest_digest", &self.run_manifest_digest),
            ("trace_digest", &self.trace_digest),
        ] {
            ContentDigest::from_hex(digest)
                .map_err(|_| PlantArtifactError::InvalidDigest(field))?;
        }
        bounded(
            "operating_hover_force",
            self.operating_hover_force,
            0.05,
            0.95,
        )?;
        bounded("sample_rate_hz", self.sample_rate_hz, 20.0, 1_000.0)?;
        for frequency in self.probe_rad_s {
            bounded("probe_rad_s", frequency, 0.1, 20.0)?;
        }
        if self.probe_rad_s[0] >= self.probe_rad_s[1] {
            return Err(PlantArtifactError::FieldOutOfRange("probe_rad_s"));
        }
        for axis in 0..3 {
            bounded("authority_k", self.authority_k[axis], 0.01, 100.0)?;
            bounded("delay_s", self.delay_s[axis], 0.0, 0.5)?;
            bounded("delay_ci95_s", self.delay_ci95_s[axis], 0.0, 0.5)?;
            if self.delay_s[axis] + self.delay_ci95_s[axis] > 0.5 {
                return Err(PlantArtifactError::FieldOutOfRange(
                    "delay_s + delay_ci95_s",
                ));
            }
            bounded("r_squared", self.r_squared[axis], 0.8, 1.0)?;
            bounded("authority_ci95", self.authority_ci95[axis], 0.0, 100.0)?;
            bounded("coherence", self.coherence[axis], 0.8, 1.0)?;
            bounded("applied_input_max", self.applied_input_max[axis], 0.05, 1.0)?;
            bounded(
                "saturation_fraction",
                self.saturation_fraction[axis],
                0.0,
                0.05,
            )?;
            if self.sample_count[axis] < 100 {
                return Err(PlantArtifactError::InsufficientSamples(axis));
            }
            if !matches!(self.response_sign[axis], -1 | 1) {
                return Err(PlantArtifactError::InvalidResponseSign(axis));
            }
            if self.authority_ci95[axis] > self.authority_k[axis] * 0.25 {
                return Err(PlantArtifactError::ExcessiveUncertainty(axis));
            }
        }
        Ok(())
    }

    /// Calculate the identity of the exact artifact text.
    #[must_use]
    pub fn content_digest(text: &str) -> ContentDigest {
        ContentDigest::calculate(text.as_bytes())
    }

    /// Encode the validated artifact as strict TOML.
    ///
    /// # Errors
    ///
    /// Returns an error when the artifact is not valid.
    pub fn to_toml(&self) -> Result<String, PlantArtifactError> {
        self.validate()?;
        Ok(format!(
            "schema_version = {}\nartifact_id = {:?}\nairframe_id = {:?}\nsimulator_model_digest = {:?}\nrun_manifest_digest = {:?}\ntrace_digest = {:?}\nsample_clock = \"simulator-microseconds\"\noperating_hover_force = {:?}\nprobe_rad_s = {:?}\nsample_rate_hz = {:?}\nauthority_k = {:?}\ndelay_s = {:?}\ndelay_ci95_s = {:?}\nr_squared = {:?}\nauthority_ci95 = {:?}\ncoherence = {:?}\napplied_input_max = {:?}\nsample_count = {:?}\nsaturation_fraction = {:?}\nresponse_sign = {:?}\n",
            self.schema_version,
            self.artifact_id,
            self.airframe_id,
            self.simulator_model_digest,
            self.run_manifest_digest,
            self.trace_digest,
            self.operating_hover_force,
            self.probe_rad_s,
            self.sample_rate_hz,
            self.authority_k,
            self.delay_s,
            self.delay_ci95_s,
            self.r_squared,
            self.authority_ci95,
            self.coherence,
            self.applied_input_max,
            self.sample_count,
            self.saturation_fraction,
            self.response_sign,
        ))
    }
}

fn validate_id(field: &'static str, value: &str) -> Result<(), PlantArtifactError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_ID_BYTES
        || !bytes
            .iter()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'-' | b'_' | b'.'))
    {
        return Err(PlantArtifactError::InvalidId(field));
    }
    Ok(())
}

fn bounded(
    field: &'static str,
    value: f32,
    lower: f32,
    upper: f32,
) -> Result<(), PlantArtifactError> {
    if !value.is_finite() || !(lower..=upper).contains(&value) {
        return Err(PlantArtifactError::FieldOutOfRange(field));
    }
    Ok(())
}
