//! Outbound telemetry of [`super::DirectXilBoard`] for one cycle.

use aviate_core::kernel::config::MixerGeometry;
use aviate_core::mixer::fixed_wing::THROTTLE_LANE;
use aviate_core::sensor::GeomagneticReference;
use aviate_core::state::StateValidFlags;
use aviate_link::telemetry::GeodeticOrigin;

use crate::link::DirectTelemetryFrameInput;

use super::DirectXilBoard;

impl<C, M> DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    pub(super) fn emit_cycle_telemetry(
        &mut self,
        sim_time_us: u64,
        time_boot_ms: u32,
        estimate: &aviate_core::state::StateEstimate,
        geomag: GeomagneticReference,
    ) {
        let frame = self.frame_origin();
        let origin_msl_m =
            self.config.origin.ellipsoidal_height_m - self.geoid_separation_m.unwrap_or(0.0);
        let origin = GeodeticOrigin {
            latitude_deg: frame.latitude_deg,
            longitude_deg: frame.longitude_deg,
            altitude_msl_m: origin_msl_m as f32,
        };
        if self.geoid_separation_m.is_some() && !self.origin_announced {
            self.endpoint.emit_global_origin(
                origin.latitude_deg,
                origin.longitude_deg,
                origin_msl_m,
                u64::from(time_boot_ms) * 1_000,
            );
            self.origin_announced = true;
        }
        let input = DirectTelemetryFrameInput {
            sim_time_us,
            time_boot_ms,
            faults: self.kernel.state.faults,
            state: estimate,
            baro_pressure_pa: self.sensor_cache.baro_pressure_pa,
            baro_temperature_c: self.sensor_cache.baro_temperature_c,
            geomagnetic: Some(geomag),
            geodetic_origin: origin,
            gnss_fix: self.last_gnss_raw.as_ref(),
            indicated_airspeed_mps: self
                .sensor_cache
                .airspeed
                .and_then(|reading| reading.value.air.pitot_static())
                .map(|probe| probe.indicated_mps),
            throttle: self.commanded_throttle(),
            altitude_msl_m: self
                .geoid_separation_m
                .filter(|_| estimate.valid_flags.contains(StateValidFlags::HEIGHT))
                .map(|_| (origin_msl_m - f64::from(estimate.position_ned[2].0)) as f32),
        };
        self.endpoint.emit_telemetry(&input);
    }

    /// Throttle of the last actuator command, 0 to 1: the throttle lane
    /// of a fixed wing, the mean motor output of a multirotor.
    fn commanded_throttle(&self) -> f32 {
        let cmd = &self.last_actuator_cmd;
        match self.kernel.cfg().mixer_geometry {
            MixerGeometry::FixedWing => cmd.outputs.get(THROTTLE_LANE).copied().unwrap_or(0.0),
            MixerGeometry::QuadX
            | MixerGeometry::QuadXX500
            | MixerGeometry::QuadXX500ReversedSpin => {
                let motors = cmd.outputs.iter().take(usize::from(cmd.count));
                let count = f32::from(cmd.count.max(1));
                motors.sum::<f32>() / count
            }
        }
    }
}
