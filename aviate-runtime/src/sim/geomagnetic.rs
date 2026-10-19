//! The geomagnetic reference the SITL runner supplies to the estimator.
//!
//! The declination comes from the World Magnetic Model at the simulated
//! receiver's fix and the host clock's date. Without a usable fix, or
//! with the date outside the model's validity, a configured declination
//! serves instead; with neither, the estimator's yaw stays magnetic.

use aviate_core::sensor::{GeomagneticReference, GeomagneticSource};
use aviate_core::types::Radians;
use aviate_hal_io::{GnssFix, RawGnssReading};
use log::{info, warn};
use world_core::{DecimalYear, GeodeticPosition};
use world_geomag::wmm2025;

use super::SitlRunner;

/// How far, in degrees of latitude or longitude, the fix must move
/// before the model is evaluated again. Declination changes by far less
/// than the heading noise over this distance (about 1 km).
const REEVALUATE_DEG: f64 = 0.01;

/// The model reference and the fix it was evaluated at.
#[derive(Clone, Copy, Debug)]
struct ModelReference {
    reference: GeomagneticReference,
    latitude_deg: f64,
    longitude_deg: f64,
}

/// The declination sources the runner holds.
#[derive(Debug, Default)]
pub(crate) struct GeomagneticContext {
    /// The model value at the latest usable fix. Kept through a GNSS
    /// dropout: the declination does not change because the fix is lost.
    model: Option<ModelReference>,
    /// The fallback declination, when one is configured.
    configured: Option<Radians>,
    /// Whether a model refusal has been reported, so it is reported once.
    refusal_reported: bool,
}

impl GeomagneticContext {
    /// Evaluate the model at `fix` when it is usable and the vehicle has
    /// moved far enough since the last evaluation.
    pub(crate) fn observe_fix(&mut self, fix: &RawGnssReading) {
        if matches!(fix.fix, GnssFix::None | GnssFix::TwoD) {
            return;
        }
        let near_last = self.model.is_some_and(|m| {
            (m.latitude_deg - fix.lat_deg).abs() < REEVALUATE_DEG
                && (m.longitude_deg - fix.lon_deg).abs() < REEVALUATE_DEG
        });
        if near_last {
            return;
        }
        // The receiver reports one height; the few tens of meters between
        // sea level and the ellipsoid do not move the field.
        let position = GeodeticPosition::wgs84(fix.lat_deg, fix.lon_deg, f64::from(fix.alt_m));
        match wmm2025(position, decimal_year_now()) {
            Ok(field) => {
                if self.model.is_none() {
                    info!(
                        "declination {:.2} deg from {} at {:.5}, {:.5}",
                        field.declination_deg,
                        field.model.dataset.edition,
                        fix.lat_deg,
                        fix.lon_deg
                    );
                }
                self.model = Some(ModelReference {
                    reference: GeomagneticReference {
                        declination: Radians(field.declination_rad() as f32),
                        source: GeomagneticSource::Model,
                    },
                    latitude_deg: fix.lat_deg,
                    longitude_deg: fix.lon_deg,
                });
            }
            Err(error) => {
                if !self.refusal_reported {
                    warn!("geomagnetic model refused the fix: {error}");
                    self.refusal_reported = true;
                }
            }
        }
    }

    /// The reference for the estimator: the model value when there is
    /// one, else the configured declination.
    pub(crate) fn reference(&self) -> Option<GeomagneticReference> {
        self.model
            .map(|m| m.reference)
            .or(self.configured.map(|declination| GeomagneticReference {
                declination,
                source: GeomagneticSource::Configured,
            }))
    }

    /// The declination in radians, 0 when there is no reference.
    pub(crate) fn declination(&self) -> f32 {
        self.reference().map_or(0.0, |r| r.declination.0)
    }

    /// Drop the model value, for a simulator that may now be elsewhere.
    pub(crate) fn forget_model(&mut self) {
        self.model = None;
    }
}

/// Today's date as a decimal year, from the host clock.
fn decimal_year_now() -> DecimalYear {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX));
    DecimalYear::from_unix_seconds(seconds)
}

impl<C, M> SitlRunner<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Configure the declination, positive east, used while no model
    /// value is available.
    pub fn set_configured_declination(&mut self, declination: Radians) {
        self.geomagnetic.configured = Some(declination);
    }

    /// The geomagnetic reference the estimator currently receives.
    pub fn geomagnetic_reference(&self) -> Option<GeomagneticReference> {
        self.geomagnetic.reference()
    }
}

#[cfg(test)]
mod tests;
