//! Geodetic origin and physical environment evaluation for direct XIL synthesis.

use core::fmt;

use world_atmosphere::{Altitude, AtmosphereError, AtmosphereState, Isa};
use world_core::{
    normal_gravity, DecimalYear, GeodeticPosition, LocalTangentPlane, Ned, PositionError,
};
use world_geomag::{wmm2025, GeomagError};

/// World reference point and geomagnetic epoch for local NED synthesis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectOrigin {
    /// WGS-84 geodetic origin of the local NED frame.
    pub position: GeodeticPosition,
    /// Decimal year used to evaluate the geomagnetic model.
    pub epoch: DecimalYear,
}

impl DirectOrigin {
    /// Create a [`DirectOrigin`] from a WGS-84 position and a decimal year.
    #[must_use]
    pub const fn new(position: GeodeticPosition, epoch: DecimalYear) -> Self {
        Self { position, epoch }
    }
}

/// Reason why [`DirectEnvironment`] or [`super::DirectSynthesizer`] refused startup.
#[derive(Debug, Clone, PartialEq)]
pub enum DirectSynthesisError {
    /// The geodetic origin is outside WGS-84 bounds.
    Origin(PositionError),
    /// The geomagnetic model rejected the position or the epoch.
    EarthField(GeomagError),
    /// The standard atmosphere model rejected the altitude.
    Atmosphere(AtmosphereError),
}

impl fmt::Display for DirectSynthesisError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Origin(err) => write!(f, "invalid world origin: {err}"),
            Self::EarthField(err) => write!(f, "geomagnetic model error at origin: {err}"),
            Self::Atmosphere(err) => write!(f, "atmosphere model error at origin: {err}"),
        }
    }
}

impl core::error::Error for DirectSynthesisError {}

impl From<PositionError> for DirectSynthesisError {
    fn from(err: PositionError) -> Self {
        Self::Origin(err)
    }
}

impl From<GeomagError> for DirectSynthesisError {
    fn from(err: GeomagError) -> Self {
        Self::EarthField(err)
    }
}

impl From<AtmosphereError> for DirectSynthesisError {
    fn from(err: AtmosphereError) -> Self {
        Self::Atmosphere(err)
    }
}

/// Physical environment at one geodetic position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectEnvironment {
    /// Current geodetic position on WGS-84.
    pub position: GeodeticPosition,
    /// Normal gravity magnitude in meters per second squared.
    pub gravity_mps2: f64,
    /// International Standard Atmosphere state at the current altitude.
    pub air: AtmosphereState,
    /// WMM2025 geomagnetic field vector in local NED microtesla.
    pub field_ned_ut: [f32; 3],
}

impl DirectEnvironment {
    /// Evaluate gravity, atmosphere, and Earth magnetic field at `position` and `epoch`.
    ///
    /// # Errors
    ///
    /// Returns [`DirectSynthesisError`] when a world model rejects `position` or `epoch`.
    pub fn at(
        position: GeodeticPosition,
        epoch: DecimalYear,
    ) -> Result<Self, DirectSynthesisError> {
        Ok(Self {
            position,
            gravity_mps2: normal_gravity(position)?,
            air: air_at(position)?,
            field_ned_ut: field_at(position, epoch)?,
        })
    }

    /// Update the environment for local `ned` meters in `plane`.
    ///
    /// If a model rejects an out-of-range excursion, this method retains the last valid value.
    pub fn follow(&mut self, plane: &LocalTangentPlane, ned: [f32; 3], epoch: DecimalYear) {
        let position = plane.to_geodetic(Ned {
            north_m: f64::from(ned[0]),
            east_m: f64::from(ned[1]),
            down_m: f64::from(ned[2]),
        });
        self.position = position;
        if let Ok(gravity) = normal_gravity(position) {
            self.gravity_mps2 = gravity;
        }
        if let Ok(air) = air_at(position) {
            self.air = air;
        }
        if let Ok(field) = field_at(position, epoch) {
            self.field_ned_ut = field;
        }
    }

    /// Return the magnetic declination in radians (positive east of true north).
    #[must_use]
    pub fn declination_rad(&self) -> f32 {
        libm::atan2f(self.field_ned_ut[1], self.field_ned_ut[0])
    }

    /// Return the magnetic declination in degrees (positive east of true north).
    #[must_use]
    pub fn declination_deg(&self) -> f32 {
        self.declination_rad().to_degrees()
    }
}

fn air_at(position: GeodeticPosition) -> Result<AtmosphereState, AtmosphereError> {
    Isa.at(Altitude::GeometricMsl(position.ellipsoidal_height_m))
}

fn field_at(position: GeodeticPosition, epoch: DecimalYear) -> Result<[f32; 3], GeomagError> {
    let field = wmm2025(position, epoch)?;
    Ok([
        (field.north_nt / 1_000.0) as f32,
        (field.east_nt / 1_000.0) as f32,
        (field.down_nt / 1_000.0) as f32,
    ])
}
