//! Geodetic-to-local-NED projection with a latched origin.
//!
//! A HIL bridge reports position as WGS84 latitude, longitude and
//! altitude, while the kernel navigates in local NED metres. The
//! conversion needs an ORIGIN, and the origin must be latched once: a
//! per-sample origin would hold the vehicle at zero forever, and a
//! re-latch mid-flight would teleport the estimator.
//!
//! The projection is `world_core::LocalTangentPlane`: NED on the WGS84
//! tangent plane at the origin, through ECEF, exact at any distance.

use aviate_hal_xil::sim_types::SimGnssFix;
use world_core::{GeodeticPosition, LocalTangentPlane};

/// The latched local-tangent-plane origin for a session.
#[derive(Debug, Clone, Copy, Default)]
pub struct NedOrigin {
    latched: Option<LocalTangentPlane>,
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
        // The bridge reports one altitude; it is taken as the height
        // above the WGS84 ellipsoid, the same surface the origin's is on.
        let position = GeodeticPosition::wgs84(lat_deg, lon_deg, f64::from(alt_m));
        let plane = match self.latched {
            Some(plane) => plane,
            None => match LocalTangentPlane::new(position) {
                Ok(plane) => *self.latched.insert(plane),
                Err(_) => return [0.0; 3],
            },
        };
        // The offset stays in f64 until here: two latitudes near 47
        // differ only in their last f32 digits by metres.
        match plane.to_ned(position) {
            Ok(ned) => [ned.north_m as f32, ned.east_m as f32, ned.down_m as f32],
            Err(_) => [0.0; 3],
        }
    }

    /// Whether an origin has been latched.
    #[must_use]
    pub fn is_latched(&self) -> bool {
        self.latched.is_some()
    }
}

#[cfg(test)]
mod tests;
