//! Cessna 172 class light aeroplane.

use aviate_core::control::Limits;
use aviate_core::ekf::air_aiding::AirAidingConfig;
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
///
/// The roll and pitch feedforward are the inverse of the body rate that
/// a unit deflection holds at the cruise airspeed. The stability
/// derivatives of the JSBSim Cessna 172 models give these rates:
///
/// - Roll: `(Clda / -Clp) * (2 V / b) * travel` with `Clda` = 0.229,
///   `Clp` = -0.484, b = 11.0 m and 0.3054 rad of aileron travel is
///   1.445 rad/s, so the feedforward is 0.69.
/// - Pitch: a steady pull-up holds `Mde * travel / (-Mq - Ma / Za)`
///   = 1.31 rad/s with 0.4887 rad of up elevator, so the feedforward is
///   0.77. The pitch rate leads the angle of attack: after a step of the
///   demand it is above the demand by 0.23 of the step for a short time.
///
/// In that pull-up the pitch damping `-Mq` = 4.00 /s is 0.33 of what
/// resists the rate and the stability term `-Ma / Za` = 8.14 /s is the
/// other part, which is the pitch damping share.
///
/// The yaw feedforward and the feedback gains have no such source.
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
            cruise_throttle: NormalizedThrust(0.84),
            max_throttle: NormalizedThrust(1.0),
            max_climb_rate: MetersPerSecond(2.5),
            max_sink_rate: MetersPerSecond(5.0),
            trim_pitch: Radians(0.029),
            height_time_constant: Seconds(5.0),
            speed_time_constant: Seconds(5.0),
            energy_rate_p: 0.05,
            energy_rate_i: 0.02,
            balance_rate_p: 0.5,
            balance_rate_i: 0.2,
            route_bank_angle: Radians(0.44),
            course_time_constant: Seconds(3.0),
            course_integral_time: Seconds(20.0),
            track_lookahead_time: Seconds(10.0),
            max_intercept_angle: Radians(0.785),
            roll_time_constant: Seconds(0.5),
            pitch_time_constant: Seconds(0.5),
            rate_p: [0.4, 0.6, 0.5],
            rate_i: [0.2, 0.4, 0.0],
            rate_ff: [0.69, 0.77, 0.5],
            pitch_damping_share: 0.33,
            rate_integrator_limit: 0.3,
            airspeed_scale_min: 0.5,
            airspeed_scale_max: 2.0,
            airspeed_filter_time_constant: Seconds(1.0),
        },
        limits: Limits {
            max_roll: Radians(0.785),
            max_pitch: Radians(0.35),
            max_roll_rate: RadiansPerSecond(0.5),
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
        air_aiding: AirAidingConfig {
            meas_noise_tas: 1.0,
            sideslip_sigma: 0.05,
            wind_prior_var: 64.0,
            process_noise_wind: 0.05,
            min_true_airspeed: 15.0,
        },
    }
}
