//! The environment a simulated vehicle's sensors read, from the shared
//! world models.

use world_atmosphere::{Altitude, AtmosphereError, AtmosphereState, Isa};
use world_core::{
    normal_gravity, DecimalYear, GeodeticPosition, LocalTangentPlane, Ned, PositionError,
};
use world_geomag::{wmm2025, GeomagError};

/// Why the environment at the world origin could not be made.
#[derive(Debug, thiserror::Error)]
pub enum SynthesisError {
    /// The origin is not a usable geodetic position.
    #[error("world origin: {0}")]
    Origin(#[from] PositionError),
    /// The geomagnetic model refused the origin or the date.
    #[error("Earth field at the world origin: {0}")]
    EarthField(#[from] GeomagError),
    /// The origin's elevation is outside the standard atmosphere.
    #[error("atmosphere at the world origin: {0}")]
    Atmosphere(#[from] AtmosphereError),
}

/// The environment at one position: where it is, the gravity, the air
/// and the Earth field there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Environment {
    /// The position, on the origin's datum.
    pub position: GeodeticPosition,
    /// Normal gravity, m/s².
    pub gravity_mps2: f64,
    /// The standard atmosphere at the position's height.
    pub air: AtmosphereState,
    /// The WMM Earth field, NED microtesla.
    pub field_ned_ut: [f32; 3],
}

impl Environment {
    /// The environment at `position` on `epoch`.
    pub fn at(position: GeodeticPosition, epoch: DecimalYear) -> Result<Self, SynthesisError> {
        Ok(Self {
            position,
            gravity_mps2: normal_gravity(position)?,
            air: air_at(position)?,
            field_ned_ut: field_at(position, epoch)?,
        })
    }

    /// Move to local NED position `ned` in `plane`. A model that refuses
    /// the new position keeps its last value, so a sensor never reads a
    /// substituted environment.
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

    /// The model's declination at the position, degrees.
    pub fn declination_deg(&self) -> f32 {
        self.field_ned_ut[1]
            .atan2(self.field_ned_ut[0])
            .to_degrees()
    }
}

/// The standard atmosphere at the position's height, taken as its height
/// above mean sea level (see [`super::GeoOrigin`]).
fn air_at(position: GeodeticPosition) -> Result<AtmosphereState, AtmosphereError> {
    Isa.at(Altitude::GeometricMsl(position.ellipsoidal_height_m))
}

fn field_at(position: GeodeticPosition, epoch: DecimalYear) -> Result<[f32; 3], GeomagError> {
    let field = wmm2025(position, epoch)?;
    // The model gives nanotesla.
    Ok([
        (field.north_nt / 1_000.0) as f32,
        (field.east_nt / 1_000.0) as f32,
        (field.down_nt / 1_000.0) as f32,
    ])
}
