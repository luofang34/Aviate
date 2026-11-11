//! Rascal 110 class small unmanned aeroplane.

use aviate_core::control::Limits;
use aviate_core::ekf::air_aiding::AirAidingConfig;
use aviate_core::kernel::config::FixedWingParams;
use aviate_core::types::{
    Meters, MetersPerSecond, NormalizedThrust, Radians, RadiansPerSecond, Seconds,
};

use crate::description::FixedWingAirframe;

/// Rascal 110 class aeroplane: 2.8 m span, about 7 kg.
///
/// The numbers are first values for a simulated vehicle of this class
/// and have no flight evidence.
#[must_use]
pub fn rascal() -> FixedWingAirframe {
    FixedWingAirframe {
        id: "rascal",
        params: FixedWingParams {
            stall_airspeed: MetersPerSecond(11.0),
            min_airspeed: MetersPerSecond(14.0),
            cruise_airspeed: MetersPerSecond(22.0),
            max_airspeed: MetersPerSecond(32.0),
            min_throttle: NormalizedThrust(0.0),
            cruise_throttle: NormalizedThrust(0.5),
            max_throttle: NormalizedThrust(1.0),
            max_climb_rate: MetersPerSecond(4.0),
            max_sink_rate: MetersPerSecond(3.0),
            trim_pitch: Radians(0.03),
            height_time_constant: Seconds(4.0),
            speed_time_constant: Seconds(4.0),
            energy_rate_p: 0.05,
            energy_rate_i: 0.03,
            balance_rate_p: 0.5,
            balance_rate_i: 0.2,
            route_bank_angle: Radians(0.52),
            course_time_constant: Seconds(3.0),
            course_integral_time: Seconds(15.0),
            track_lookahead_time: Seconds(9.0),
            max_intercept_angle: Radians(0.785),
            roll_time_constant: Seconds(0.4),
            pitch_time_constant: Seconds(0.4),
            rate_p: [0.15, 0.25, 0.2],
            rate_i: [0.1, 0.2, 0.0],
            rate_ff: [0.4, 0.5, 0.3],
            rate_integrator_limit: 0.3,
            airspeed_scale_min: 0.5,
            airspeed_scale_max: 2.0,
            airspeed_filter_time_constant: Seconds(0.6),
        },
        limits: Limits {
            max_roll: Radians(0.785),
            max_pitch: Radians(0.45),
            max_roll_rate: RadiansPerSecond(1.5),
            max_pitch_rate: RadiansPerSecond(0.8),
            max_yaw_rate: RadiansPerSecond(0.8),
            max_horizontal_speed: MetersPerSecond(45.0),
            max_climb_rate: MetersPerSecond(5.0),
            max_descent_rate: MetersPerSecond(4.0),
            max_altitude: Meters(1_500.0),
            min_altitude: Meters(0.0),
            min_airspeed: None,
            max_airspeed: None,
            max_load_factor: 3.0,
            min_load_factor: 0.0,
        },
        air_aiding: AirAidingConfig {
            meas_noise_tas: 1.0,
            sideslip_sigma: 0.05,
            wind_prior_var: 64.0,
            process_noise_wind: 0.05,
            min_true_airspeed: 7.0,
        },
    }
}
