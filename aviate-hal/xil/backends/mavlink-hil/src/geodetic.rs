//! Geodetic-to-local-NED projection with a latched origin.
//!
//! A HIL bridge reports position as WGS84 latitude, longitude and
//! altitude, while the kernel navigates in local NED metres. The
//! conversion needs an ORIGIN, and the origin must be latched once: a
//! per-sample origin would hold the vehicle at zero forever, and a
//! re-latch mid-flight would teleport the estimator.
//!
//! The projection is flat-earth. Over the tens of kilometres a SITL
//! flight covers, the error is far below the fix noise; it is not a
//! survey-grade transform and does not claim to be.

use aviate_hal_xil::sim_types::SimGnssFix;

/// Metres per degree of latitude on a sphere of the WGS84 mean radius.
const METRES_PER_DEGREE: f64 = 111_111.0;

/// The latched local-tangent-plane origin for a session.
#[derive(Debug, Clone, Copy, Default)]
pub struct NedOrigin {
    latched: Option<Origin>,
}

#[derive(Debug, Clone, Copy)]
struct Origin {
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f32,
    /// Longitude scale at the origin latitude, held so every sample
    /// projects against the SAME scale — recomputing it per sample
    /// would make the east axis stretch as the vehicle moves.
    lon_scale: f64,
}

impl NedOrigin {
    /// Projects one fix into local NED metres, latching the origin on
    /// the first usable fix.
    ///
    /// A fix with no 3D lock returns the origin (all zeros) rather than
    /// a position derived from a lock the receiver does not claim.
    #[must_use]
    pub fn project(&mut self, lat_deg: f64, lon_deg: f64, alt_m: f32, fix: SimGnssFix) -> [f32; 3] {
        let usable = matches!(
            fix,
            SimGnssFix::ThreeD | SimGnssFix::RtkFloat | SimGnssFix::RtkFixed
        ) && lat_deg.is_finite()
            && lon_deg.is_finite()
            && alt_m.is_finite();
        if !usable {
            return [0.0; 3];
        }
        let origin = *self.latched.get_or_insert_with(|| Origin {
            lat_deg,
            lon_deg,
            alt_m,
            lon_scale: METRES_PER_DEGREE * lat_deg.to_radians().cos(),
        });
        // Subtract in f64 before narrowing: two degrees near 47 differ
        // in their last f32 digits by metres, so an f32 subtraction
        // would quantize the position to a coarse grid.
        let north = (lat_deg - origin.lat_deg) * METRES_PER_DEGREE;
        let east = (lon_deg - origin.lon_deg) * origin.lon_scale;
        let down = f64::from(origin.alt_m - alt_m);
        [north as f32, east as f32, down as f32]
    }

    /// Whether an origin has been latched.
    #[must_use]
    pub fn is_latched(&self) -> bool {
        self.latched.is_some()
    }
}

#[cfg(test)]
mod tests;
