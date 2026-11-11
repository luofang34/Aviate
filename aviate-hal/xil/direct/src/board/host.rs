//! Stepping [`super::DirectXilBoard`] with sensor samples that the host built.

use aviate_core::sensor::{GeomagneticReference, GeomagneticSource};
use aviate_core::types::Radians;
use aviate_hal_io::{GnssFix, RawGnssReading};
use world_core::GeodeticPosition;

use crate::host_sensors::DirectHostSensors;
use crate::synthesis::declination_rad_at;

use super::step::advance_board_hal_time;
use super::{DirectActuatorCmd, DirectXilBoard};

impl<C, M> DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Advance the flight controller by one step of host-built sensor samples.
    ///
    /// The kernel, the command path, and the telemetry path are those of
    /// [`Self::step_truth`]. The built-in synthesizer does not run: the board
    /// sees only the samples in `sensors`, as a flight board sees only its
    /// hardware.
    ///
    /// The first call of a `reset_generation` sets the time reference. It runs
    /// no kernel cycle and does not use the samples. A call whose `time_us`
    /// does not increase also runs no kernel cycle. Each later call runs one
    /// kernel cycle whose interval is the increase of `time_us`.
    ///
    /// A lane that is `None` keeps its last value and adds no new measurement.
    /// The estimator therefore ages a silent sensor by its own rules. The
    /// kernel does not use the sample timestamps to compensate latency.
    ///
    /// Use one step entry for each `reset_generation`. The two entries keep
    /// separate time references.
    pub fn step_sensors(&mut self, sensors: &DirectHostSensors) -> DirectActuatorCmd {
        self.handle_generation_transition(sensors.reset_generation);
        let anchor_us = *self.boot_anchor_us.get_or_insert(sensors.time_us);
        advance_board_hal_time(&mut self.board_hal, sensors.time_us);

        let delta_us = match self.host_clock_us.replace(sensors.time_us) {
            Some(previous_us) if sensors.time_us > previous_us => sensors.time_us - previous_us,
            _ => return self.idle_step(sensors.time_us),
        };

        if let Some(gnss) = sensors.gnss {
            self.follow_model_declination(&gnss.reading);
        }
        let geomag = sensors.geomagnetic.unwrap_or(GeomagneticReference {
            declination: Radians(self.model_declination_rad),
            source: GeomagneticSource::Model,
        });
        self.run_cycle(sensors, delta_us, geomag, anchor_us)
    }

    /// Evaluate the geomagnetic model at a GNSS fix.
    ///
    /// A reading without a fix, or a position that the model refuses, keeps
    /// the last declination: the declination does not change because the fix
    /// is lost.
    fn follow_model_declination(&mut self, fix: &RawGnssReading) {
        if fix.fix == GnssFix::None {
            return;
        }
        let position = GeodeticPosition::wgs84(fix.lat_deg, fix.lon_deg, f64::from(fix.alt_m));
        if !position.is_well_formed() {
            return;
        }
        if let Ok(declination_rad) = declination_rad_at(position, self.config.epoch) {
            self.model_declination_rad = declination_rad;
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests;
