//! Cessna 172 class light aeroplane.

use aviate_core::control::Limits;
use aviate_core::kernel::config::FixedWingParams;
use aviate_core::types::{
    Meters, MetersPerSecond, NormalizedThrust, Radians, RadiansPerSecond, Seconds,
};

use crate::description::FixedWingAirframe;

/// Cessna 172 class aeroplane at normal-category weight, flaps up.
///
/// The airspeeds follow the published speeds of the type: 48 KCAS stall,
/// 127 KIAS maximum structural cruise. The loop tuning is a first value
/// for a simulated vehicle of this class and has no flight evidence.
#[must_use]
pub fn c172() -> FixedWingAirframe {
    FixedWingAirframe {
        id: "c172",
        params: FixedWingParams {
            stall_airspeed: MetersPerSecond(25.0),
            // 1.3 times the stall speed.
            min_airspeed: MetersPerSecond(33.0),
            cruise_airspeed: MetersPerSecond(55.0),
            max_airspeed: MetersPerSecond(65.0),
            min_throttle: NormalizedThrust(0.0),
            cruise_throttle: NormalizedThrust(0.65),
            max_throttle: NormalizedThrust(1.0),
            max_climb_rate: MetersPerSecond(2.5),
            max_sink_rate: MetersPerSecond(5.0),
            trim_pitch: Radians(0.02),
            height_time_constant: Seconds(5.0),
            speed_time_constant: Seconds(5.0),
            energy_rate_p: 0.05,
            energy_rate_i: 0.02,
            balance_rate_p: 0.5,
            balance_rate_i: 0.2,
            course_time_constant: Seconds(3.0),
            track_lookahead_time: Seconds(10.0),
            max_intercept_angle: Radians(0.785),
            roll_time_constant: Seconds(0.5),
            pitch_time_constant: Seconds(0.5),
            rate_p: [0.4, 0.6, 0.5],
            rate_i: [0.2, 0.4, 0.1],
            rate_ff: [1.0, 1.0, 0.5],
            rate_integrator_limit: 0.3,
            airspeed_scale_min: 0.5,
            airspeed_scale_max: 2.0,
            airspeed_filter_time_constant: Seconds(0.5),
        },
        limits: Limits {
            max_roll: Radians(0.785),
            max_pitch: Radians(0.35),
            max_roll_rate: RadiansPerSecond(0.8),
            max_pitch_rate: RadiansPerSecond(0.3),
            max_yaw_rate: RadiansPerSecond(0.3),
            max_horizontal_speed: MetersPerSecond(85.0),
            max_climb_rate: MetersPerSecond(5.0),
            max_descent_rate: MetersPerSecond(5.0),
            max_altitude: Meters(4_000.0),
            min_altitude: Meters(0.0),
            min_airspeed: None,
            max_airspeed: None,
            max_load_factor: 2.5,
            min_load_factor: 0.0,
        },
    }
}
