//! Fixed-wing parameter block shared by the integration tests.

use aviate_core::kernel::config::FixedWingParams;
use aviate_core::types::{MetersPerSecond, NormalizedThrust, Radians, Seconds};

/// A light single-engine aeroplane: 25 m/s stall, 55 m/s cruise.
pub fn reference_params() -> FixedWingParams {
    FixedWingParams {
        stall_airspeed: MetersPerSecond(25.0),
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
        course_time_constant: Seconds(5.0),
        track_lookahead_time: Seconds(8.0),
        max_intercept_angle: Radians(0.785),
        roll_time_constant: Seconds(0.5),
        pitch_time_constant: Seconds(0.5),
        rate_p: [0.4, 0.6, 0.5],
        rate_i: [0.2, 0.4, 0.1],
        rate_ff: [0.8, 1.0, 0.5],
        rate_integrator_limit: 0.3,
        airspeed_scale_min: 0.5,
        airspeed_scale_max: 2.0,
        airspeed_filter_time_constant: Seconds(0.5),
    }
}
