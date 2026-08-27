//! Verified runtime binding for one X-Plane trial session.

use alloc::string::{String, ToString};
use core::fmt;

use serde::Deserialize;

use crate::airframe_preset::ContentDigest;
use crate::xplane_model::XPlaneBridgeProtocol;

const RUNTIME_HANDSHAKE_SCHEMA_VERSION: u16 = 1;
const VERIFIER_ID: &str = "pilotage-xplane-trial-v1";
const MAX_ID_BYTES: usize = 64;

/// One single-use identity assertion from a verified X-Plane trial.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct XPlaneRuntimeHandshake {
    /// Runtime handshake schema version.
    pub schema_version: u16,
    /// Verifier that measured the running X-Plane session.
    pub verifier_id: String,
    /// Unique identity of the verified trial session.
    pub session_binding_digest: String,
    /// TCP bridge endpoint verified by the trial runner.
    pub bridge_endpoint: String,
    /// Bridge protocol in use.
    pub bridge_protocol: XPlaneBridgeProtocol,
    /// Exact bridge plugin binary identity.
    pub bridge_build_digest: String,
    /// Exact identity of the active bridge configuration.
    pub bridge_config_digest: String,
    /// Simulator build name.
    pub simulator_id: String,
    /// Current aircraft name.
    pub aircraft_id: String,
    /// Exact current aircraft file identity.
    pub aircraft_file_digest: String,
    /// Declared sensor sample rate.
    pub sample_rate_hz: u16,
    /// Actuator lane count accepted by the bridge.
    pub motor_count: u8,
    /// Mixer-to-simulator lane order.
    pub lane_order: [u8; 4],
}

/// Runtime handshake document failure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum XPlaneRuntimeHandshakeError {
    /// The TOML document cannot be decoded.
    Parse(String),
    /// The schema version is not supported.
    UnsupportedSchema(u16),
    /// The verifier is not the supported trial verifier.
    UnsupportedVerifier,
    /// A stable name is not portable ASCII.
    InvalidId(&'static str),
    /// An identity is not one SHA-256 value.
    InvalidDigest(&'static str),
    /// A numeric field is outside its permitted range.
    FieldOutOfRange(&'static str),
    /// The lane order is not one four-lane permutation.
    InvalidLaneOrder,
}

impl fmt::Display for XPlaneRuntimeHandshakeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(error) => {
                write!(formatter, "cannot parse X-Plane runtime handshake: {error}")
            }
            Self::UnsupportedSchema(version) => {
                write!(
                    formatter,
                    "unsupported X-Plane runtime handshake schema {version}"
                )
            }
            Self::UnsupportedVerifier => {
                formatter.write_str("unsupported X-Plane runtime verifier")
            }
            Self::InvalidId(field) => write!(formatter, "invalid runtime handshake field {field}"),
            Self::InvalidDigest(field) => {
                write!(formatter, "invalid runtime handshake digest {field}")
            }
            Self::FieldOutOfRange(field) => {
                write!(
                    formatter,
                    "runtime handshake field {field} is outside its range"
                )
            }
            Self::InvalidLaneOrder => {
                formatter.write_str("runtime handshake lane_order is not a permutation")
            }
        }
    }
}

impl core::error::Error for XPlaneRuntimeHandshakeError {}

impl XPlaneRuntimeHandshake {
    /// Decode and validate one strict runtime handshake.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown field or invalid identity.
    pub fn from_toml_str(text: &str) -> Result<Self, XPlaneRuntimeHandshakeError> {
        let value: Self = toml::from_str(text)
            .map_err(|error| XPlaneRuntimeHandshakeError::Parse(error.to_string()))?;
        value.validate()?;
        Ok(value)
    }

    /// Validate all fields that do not depend on a simulator model.
    ///
    /// # Errors
    ///
    /// Returns the first invalid field.
    pub fn validate(&self) -> Result<(), XPlaneRuntimeHandshakeError> {
        if self.schema_version != RUNTIME_HANDSHAKE_SCHEMA_VERSION {
            return Err(XPlaneRuntimeHandshakeError::UnsupportedSchema(
                self.schema_version,
            ));
        }
        if self.verifier_id != VERIFIER_ID {
            return Err(XPlaneRuntimeHandshakeError::UnsupportedVerifier);
        }
        validate_id("bridge_endpoint", &self.bridge_endpoint)?;
        validate_id("simulator_id", &self.simulator_id)?;
        validate_id("aircraft_id", &self.aircraft_id)?;
        for (field, digest) in [
            ("session_binding_digest", &self.session_binding_digest),
            ("bridge_build_digest", &self.bridge_build_digest),
            ("bridge_config_digest", &self.bridge_config_digest),
            ("aircraft_file_digest", &self.aircraft_file_digest),
        ] {
            validate_digest(field, digest)?;
        }
        if !(20..=1_000).contains(&self.sample_rate_hz) {
            return Err(XPlaneRuntimeHandshakeError::FieldOutOfRange(
                "sample_rate_hz",
            ));
        }
        if self.motor_count != 4 {
            return Err(XPlaneRuntimeHandshakeError::FieldOutOfRange("motor_count"));
        }
        let mut lanes = self.lane_order;
        lanes.sort_unstable();
        if lanes != [0, 1, 2, 3] {
            return Err(XPlaneRuntimeHandshakeError::InvalidLaneOrder);
        }
        Ok(())
    }

    /// Calculate the identity of the exact runtime handshake document.
    #[must_use]
    pub fn content_digest(text: &str) -> ContentDigest {
        ContentDigest::calculate(text.as_bytes())
    }
}

fn validate_id(field: &'static str, value: &str) -> Result<(), XPlaneRuntimeHandshakeError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_ID_BYTES
        || !bytes
            .iter()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'-' | b'_' | b'.' | b':'))
    {
        return Err(XPlaneRuntimeHandshakeError::InvalidId(field));
    }
    Ok(())
}

fn validate_digest(field: &'static str, value: &str) -> Result<(), XPlaneRuntimeHandshakeError> {
    if value.len() != 64 || !value.as_bytes().iter().all(u8::is_ascii_hexdigit) {
        return Err(XPlaneRuntimeHandshakeError::InvalidDigest(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use alloc::format;

    use super::*;

    const VALID: &str = "schema_version = 1\nverifier_id = \"pilotage-xplane-trial-v1\"\nsession_binding_digest = \"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\"\nbridge_endpoint = \"127.0.0.1:4560\"\nbridge_protocol = \"mavlink-hil-tcp-v1\"\nbridge_build_digest = \"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\"\nbridge_config_digest = \"cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\"\nsimulator_id = \"xplane-12\"\naircraft_id = \"xplane12-laminar-alia250\"\naircraft_file_digest = \"dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd\"\nsample_rate_hz = 100\nmotor_count = 4\nlane_order = [0, 2, 1, 3]\n";

    #[test]
    fn strict_verified_binding_parses() {
        let value = XPlaneRuntimeHandshake::from_toml_str(VALID).expect("valid binding");
        assert_eq!(value.sample_rate_hz, 100);
    }

    #[test]
    fn unknown_and_malformed_fields_fail_closed() {
        for text in [
            format!("{VALID}extra = true\n"),
            VALID.replace(&"a".repeat(64), "short"),
            VALID.replace("lane_order = [0, 2, 1, 3]", "lane_order = [0, 0, 1, 3]"),
        ] {
            assert!(XPlaneRuntimeHandshake::from_toml_str(&text).is_err());
        }
    }
}
