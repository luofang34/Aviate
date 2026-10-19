//! The Earth's magnetic field the simulated magnetometer measures.
//!
//! The field comes from the same World Magnetic Model the flight
//! controller uses for its declination, evaluated at the vehicle's
//! position on the host clock's date, so a simulated vehicle meets the
//! same field a real one at that place would.

use aviate_geomag::{decimal_year_from_unix_seconds, wmm2025, GeodeticPoint, GeomagError};

use crate::origin::GeoOrigin;

/// Meters per degree of latitude in the flat-earth projection the GNSS
/// synthesis uses.
const METERS_PER_DEGREE: f64 = 111_111.0;

/// How far, in meters, the vehicle must move before the field is
/// evaluated again. Over this distance the field direction changes far
/// less than the magnetometer noise.
const REEVALUATE_M: f32 = 1_000.0;

/// The Earth field near the vehicle, in NED microtesla.
#[derive(Debug, Clone)]
pub struct EarthField {
    origin: GeoOrigin,
    decimal_year: f64,
    /// Local north/east position the field was last evaluated at.
    evaluated_at: [f32; 2],
    ned_ut: [f32; 3],
}

impl EarthField {
    /// The field at the world origin, today.
    ///
    /// # Errors
    ///
    /// The model's refusal: today outside its validity window, or the
    /// origin outside its domain.
    pub fn at_origin(origin: GeoOrigin) -> Result<Self, GeomagError> {
        let decimal_year = decimal_year_from_unix_seconds(unix_seconds_now());
        let ned_ut = evaluate(&origin, [0.0; 3], decimal_year)?;
        Ok(Self {
            origin,
            decimal_year,
            evaluated_at: [0.0, 0.0],
            ned_ut,
        })
    }

    /// Track the vehicle at local NED position `ned`, re-evaluating the
    /// model once it has moved far from the last evaluation point. A
    /// refusal keeps the last field.
    pub fn follow(&mut self, ned: [f32; 3]) {
        let moved = (ned[0] - self.evaluated_at[0]).hypot(ned[1] - self.evaluated_at[1]);
        if moved < REEVALUATE_M {
            return;
        }
        if let Ok(ned_ut) = evaluate(&self.origin, ned, self.decimal_year) {
            self.ned_ut = ned_ut;
            self.evaluated_at = [ned[0], ned[1]];
        }
    }

    /// The field, NED microtesla.
    pub fn ned_ut(&self) -> [f32; 3] {
        self.ned_ut
    }

    /// The model's declination at the last evaluation point, degrees.
    pub fn declination_deg(&self) -> f32 {
        self.ned_ut[1].atan2(self.ned_ut[0]).to_degrees()
    }
}

fn evaluate(origin: &GeoOrigin, ned: [f32; 3], decimal_year: f64) -> Result<[f32; 3], GeomagError> {
    let lat_cos = origin.latitude_deg.to_radians().cos();
    let point = GeodeticPoint {
        latitude_deg: origin.latitude_deg + f64::from(ned[0]) / METERS_PER_DEGREE,
        longitude_deg: origin.longitude_deg + f64::from(ned[1]) / (METERS_PER_DEGREE * lat_cos),
        height_m: f64::from(origin.elevation_m - ned[2]),
    };
    let field = wmm2025(point, decimal_year)?;
    // The model gives nanotesla.
    Ok([
        (field.north_nt / 1_000.0) as f32,
        (field.east_nt / 1_000.0) as f32,
        (field.down_nt / 1_000.0) as f32,
    ])
}

fn unix_seconds_now() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests;
