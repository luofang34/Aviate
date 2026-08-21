//! Versioned X-Plane plant-protection configuration.

use alloc::string::{String, ToString};
use core::fmt;

use serde::Deserialize;
use sha2::{Digest, Sha256};

/// The supported X-Plane model schema.
pub const XPLANE_MODEL_SCHEMA_VERSION: u16 = 1;

const MAX_ID_BYTES: usize = 64;

/// Plant-protection limits between the mixer and the simulator bridge.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct XPlaneWireModel {
    /// Maximum collective rise above the low-load band, per second.
    pub rise_per_s: f32,
    /// Boundary between the low-load and working bands.
    pub band_boundary: f32,
    /// Maximum collective rise in the low-load band, per second.
    pub low_band_rise_per_s: f32,
    /// Maximum collective fall while armed, per second.
    pub fall_per_s: f32,
    /// Maximum mean collective.
    pub mean_ceiling: f32,
    /// Maximum command on one actuator lane.
    pub lane_ceiling: f32,
    /// Required climb above the arming altitude before full authority.
    pub airborne_clearance_m: f32,
    /// Differential authority fraction while the vehicle is on its gear.
    pub ground_squeeze: f32,
}

/// A strict simulator model for one airframe.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct XPlaneSimulatorModel {
    /// Model schema version.
    pub schema_version: u16,
    /// Stable simulator model name.
    pub model_id: String,
    /// Airframe preset name that this model protects.
    pub airframe_id: String,
    /// Mixer lane index for each simulator actuator lane.
    pub lane_order: [u8; 4],
    /// Fixed plant-protection limits.
    pub wire: XPlaneWireModel,
}

/// SHA-256 identity of the validated simulator model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct XPlaneModelDigest([u8; 32]);

impl XPlaneModelDigest {
    /// Return the raw digest bytes.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl fmt::Display for XPlaneModelDigest {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(formatter, "{byte:02x}")?;
        }
        Ok(())
    }
}

/// A simulator-model configuration error.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum XPlaneModelError {
    /// The TOML document cannot be decoded.
    Parse(String),
    /// The schema version is not supported.
    UnsupportedSchema(u16),
    /// A stable name is empty, too long, or not portable ASCII.
    InvalidId(&'static str),
    /// A numeric field is outside its permitted range.
    FieldOutOfRange(&'static str),
    /// The lane order is not a permutation of lanes zero through three.
    InvalidLaneOrder,
    /// Two related values violate the model contract.
    InvalidRelation(&'static str),
}

impl fmt::Display for XPlaneModelError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(error) => write!(formatter, "cannot parse X-Plane model: {error}"),
            Self::UnsupportedSchema(version) => {
                write!(formatter, "unsupported X-Plane model schema {version}")
            }
            Self::InvalidId(field) => write!(formatter, "invalid X-Plane model field {field}"),
            Self::FieldOutOfRange(field) => {
                write!(
                    formatter,
                    "X-Plane model field {field} is outside its range"
                )
            }
            Self::InvalidLaneOrder => {
                formatter.write_str("X-Plane lane_order must be a four-lane permutation")
            }
            Self::InvalidRelation(relation) => {
                write!(formatter, "invalid X-Plane model relation {relation}")
            }
        }
    }
}

impl core::error::Error for XPlaneModelError {}

impl XPlaneSimulatorModel {
    /// Decode and validate one strict model document.
    ///
    /// # Errors
    ///
    /// Returns an error for unknown fields, invalid values, or an unsupported schema.
    pub fn from_toml_str(text: &str) -> Result<Self, XPlaneModelError> {
        let model: Self =
            toml::from_str(text).map_err(|error| XPlaneModelError::Parse(error.to_string()))?;
        model.validate()?;
        Ok(model)
    }

    /// Validate the complete simulator model.
    ///
    /// # Errors
    ///
    /// Returns the first contract violation.
    pub fn validate(&self) -> Result<(), XPlaneModelError> {
        if self.schema_version != XPLANE_MODEL_SCHEMA_VERSION {
            return Err(XPlaneModelError::UnsupportedSchema(self.schema_version));
        }
        validate_id("model_id", &self.model_id)?;
        validate_id("airframe_id", &self.airframe_id)?;
        validate_lane_order(self.lane_order)?;
        self.wire.validate()
    }

    /// Calculate a canonical identity from all semantic fields.
    ///
    /// # Errors
    ///
    /// Returns an error when the model is invalid.
    pub fn canonical_digest(&self) -> Result<XPlaneModelDigest, XPlaneModelError> {
        self.validate()?;
        let mut hasher = Sha256::new();
        hasher.update(self.schema_version.to_le_bytes());
        update_text(&mut hasher, &self.model_id);
        update_text(&mut hasher, &self.airframe_id);
        hasher.update(self.lane_order);
        for value in self.wire.canonical_values() {
            hasher.update(value.to_bits().to_le_bytes());
        }
        Ok(XPlaneModelDigest(hasher.finalize().into()))
    }
}

impl XPlaneWireModel {
    fn validate(self) -> Result<(), XPlaneModelError> {
        bounded("wire.rise_per_s", self.rise_per_s, 0.001, 5.0)?;
        bounded("wire.band_boundary", self.band_boundary, 0.0, 1.0)?;
        bounded(
            "wire.low_band_rise_per_s",
            self.low_band_rise_per_s,
            0.001,
            5.0,
        )?;
        bounded("wire.fall_per_s", self.fall_per_s, 0.001, 10.0)?;
        bounded("wire.mean_ceiling", self.mean_ceiling, 0.05, 1.0)?;
        bounded("wire.lane_ceiling", self.lane_ceiling, 0.05, 1.0)?;
        bounded(
            "wire.airborne_clearance_m",
            self.airborne_clearance_m,
            0.0,
            100.0,
        )?;
        bounded("wire.ground_squeeze", self.ground_squeeze, 0.0, 1.0)?;
        if self.low_band_rise_per_s < self.rise_per_s {
            return Err(XPlaneModelError::InvalidRelation(
                "low_band_rise_per_s >= rise_per_s",
            ));
        }
        if self.band_boundary > self.mean_ceiling {
            return Err(XPlaneModelError::InvalidRelation(
                "band_boundary <= mean_ceiling",
            ));
        }
        if self.mean_ceiling > self.lane_ceiling {
            return Err(XPlaneModelError::InvalidRelation(
                "mean_ceiling <= lane_ceiling",
            ));
        }
        Ok(())
    }

    fn canonical_values(self) -> [f32; 8] {
        [
            self.rise_per_s,
            self.band_boundary,
            self.low_band_rise_per_s,
            self.fall_per_s,
            self.mean_ceiling,
            self.lane_ceiling,
            self.airborne_clearance_m,
            self.ground_squeeze,
        ]
    }
}

fn validate_id(field: &'static str, value: &str) -> Result<(), XPlaneModelError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_ID_BYTES
        || !bytes
            .iter()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'-' | b'_' | b'.'))
    {
        return Err(XPlaneModelError::InvalidId(field));
    }
    Ok(())
}

fn validate_lane_order(mut lanes: [u8; 4]) -> Result<(), XPlaneModelError> {
    lanes.sort_unstable();
    if lanes != [0, 1, 2, 3] {
        return Err(XPlaneModelError::InvalidLaneOrder);
    }
    Ok(())
}

fn bounded(
    field: &'static str,
    value: f32,
    lower: f32,
    upper: f32,
) -> Result<(), XPlaneModelError> {
    if !value.is_finite() || !(lower..=upper).contains(&value) {
        return Err(XPlaneModelError::FieldOutOfRange(field));
    }
    Ok(())
}

fn update_text(hasher: &mut Sha256, text: &str) {
    hasher.update((text.len() as u64).to_le_bytes());
    hasher.update(text.as_bytes());
}
