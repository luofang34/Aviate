//! The geodetic origin of the simulated world's local frame.
//!
//! gz places the world's local ENU frame at the SDF
//! `<spherical_coordinates>` origin, and a launcher that places
//! several vehicles moves that origin per world. The shared-memory
//! contract does not carry it, so the launcher states it to this FC
//! through `AVIATE_GNSS_ORIGIN`, and GNSS and baro readings are made
//! around it.

use core::fmt;

use world_core::{GeodeticPosition, LocalTangentPlane, PositionError};

/// Name of the environment variable that states the origin as
/// `latitude_deg,longitude_deg,elevation_m`.
pub const ORIGIN_ENV: &str = "AVIATE_GNSS_ORIGIN";

/// The world origin a launcher states: a WGS-84 position whose height
/// is the stated elevation.
///
/// The elevation is stated above mean sea level and taken as the height
/// above the WGS-84 ellipsoid: no geoid separation is applied, so a
/// simulated receiver's MSL altitude equals its ellipsoidal height. The
/// tens of meters between them move neither the Earth field nor the
/// pressure noticeably.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoOrigin {
    /// The origin position.
    pub position: GeodeticPosition,
}

impl Default for GeoOrigin {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl GeoOrigin {
    /// The origin of the Aviate SITL worlds when no launcher states
    /// one.
    pub const DEFAULT: Self = Self {
        position: GeodeticPosition::wgs84(47.397_741_9, 8.545_593_8, 488.0),
    };

    /// Parse `latitude_deg,longitude_deg,elevation_m`.
    pub fn parse(text: &str) -> Result<Self, OriginError> {
        let fields: Vec<&str> = text.split(',').map(str::trim).collect();
        let [lat, lon, elev] = fields.as_slice() else {
            return Err(OriginError::FieldCount(fields.len()));
        };
        let latitude_deg = number(lat, "latitude")?;
        let longitude_deg = number(lon, "longitude")?;
        let elevation_m = number(elev, "elevation")?;
        if !(-90.0..=90.0).contains(&latitude_deg) {
            return Err(OriginError::OutOfRange("latitude", latitude_deg));
        }
        if !(-180.0..=180.0).contains(&longitude_deg) {
            return Err(OriginError::OutOfRange("longitude", longitude_deg));
        }
        Ok(Self {
            position: GeodeticPosition::wgs84(latitude_deg, longitude_deg, elevation_m),
        })
    }

    /// The local NED frame at the origin.
    pub fn plane(&self) -> Result<LocalTangentPlane, PositionError> {
        LocalTangentPlane::new(self.position)
    }

    /// The origin `AVIATE_GNSS_ORIGIN` states, or [`Self::DEFAULT`]
    /// when it is not set.
    pub fn from_env() -> Result<Self, OriginError> {
        match std::env::var(ORIGIN_ENV) {
            Ok(text) => Self::parse(&text),
            Err(_) => Ok(Self::DEFAULT),
        }
    }
}

fn number(text: &str, field: &'static str) -> Result<f64, OriginError> {
    text.parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| OriginError::NotANumber(field, text.to_owned()))
}

/// Why an origin text was refused.
#[derive(Debug, Clone, PartialEq)]
pub enum OriginError {
    /// The text did not have exactly three comma-separated fields.
    FieldCount(usize),
    /// A field was not a finite number.
    NotANumber(&'static str, String),
    /// A coordinate was outside its valid range.
    OutOfRange(&'static str, f64),
}

impl fmt::Display for OriginError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FieldCount(count) => write!(
                f,
                "{ORIGIN_ENV} needs latitude_deg,longitude_deg,elevation_m; got {count} fields"
            ),
            Self::NotANumber(field, text) => {
                write!(f, "{ORIGIN_ENV} {field} {text:?} is not a finite number")
            }
            Self::OutOfRange(field, value) => {
                write!(f, "{ORIGIN_ENV} {field} {value} is out of range")
            }
        }
    }
}

impl std::error::Error for OriginError {}

#[cfg(test)]
mod tests;
