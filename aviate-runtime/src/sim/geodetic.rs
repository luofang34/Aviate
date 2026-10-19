//! The geodetic context telemetry states the simulated vehicle in.

use aviate_hal_io::RawGnssReading;

use super::SitlRunner;
use crate::telemetry::GeodeticOrigin;

/// The world origin and the receiver's latest fix.
#[derive(Debug, Default)]
pub(crate) struct GeodeticContext {
    /// The geodetic point the simulated world's local frame, and so the
    /// estimate's, is measured from.
    pub(crate) origin: Option<GeodeticOrigin>,
    /// The latest fix the simulated receiver reported.
    pub(crate) last_fix: Option<RawGnssReading>,
}

impl<C, M> SitlRunner<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Declare the geodetic point the simulated world's local frame is
    /// measured from. Telemetry states the global position estimate only
    /// once this is known.
    pub fn set_geodetic_origin(&mut self, origin: GeodeticOrigin) {
        self.geodetic.origin = Some(origin);
    }
}
