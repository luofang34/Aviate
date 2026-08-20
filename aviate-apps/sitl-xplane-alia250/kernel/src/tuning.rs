//! Conversion from validated airframe data to kernel types.

use aviate_config::airframe_preset::{GainsPreset, LimitsPreset};
use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::Limits;
use aviate_core::types::{Meters, MetersPerSecond, Radians, RadiansPerSecond};

/// Convert all preset gain fields to the immutable kernel configuration.
pub(crate) fn gains_from_preset(gains: GainsPreset) -> CascadeGains {
    CascadeGains {
        pos_p: gains.pos_p,
        pos_accel_limits: gains.pos_accel_limits,
        pos_vel_caps: gains.pos_vel_caps,
        vel_p: gains.vel_p,
        vel_i: gains.vel_i,
        vel_max_roll_pitch: gains.vel_max_roll_pitch,
        vel_max_yaw_step: gains.vel_max_yaw_step,
        vel_accel_ff: gains.vel_accel_ff,
        vel_d: gains.vel_d,
        att_p: gains.att_p,
        att_max_rate_cmd: gains.att_max_rate_cmd,
        rate_p: gains.rate_p,
        rate_i: gains.rate_i,
        rate_d: gains.rate_d,
        rate_d_lpf_alpha: gains.rate_d_lpf_alpha,
    }
}

/// Convert all preset envelope fields to the kernel limit types.
pub(crate) fn limits_from_preset(limits: LimitsPreset) -> Limits {
    Limits {
        max_roll: Radians(limits.max_roll),
        max_pitch: Radians(limits.max_pitch),
        max_roll_rate: RadiansPerSecond(limits.max_roll_rate),
        max_pitch_rate: RadiansPerSecond(limits.max_pitch_rate),
        max_yaw_rate: RadiansPerSecond(limits.max_yaw_rate),
        max_horizontal_speed: MetersPerSecond(limits.max_horizontal_speed),
        max_climb_rate: MetersPerSecond(limits.max_climb_rate),
        max_descent_rate: MetersPerSecond(limits.max_descent_rate),
        max_altitude: Meters(limits.max_altitude),
        min_altitude: Meters(limits.min_altitude),
        min_airspeed: None,
        max_airspeed: None,
        max_load_factor: 2.0,
        min_load_factor: 0.0,
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use aviate_config::airframe_preset::preset_from_toml_str;

    use super::*;

    const ALIA250: &str = include_str!("../../../../presets/alia250.toml");

    #[test]
    fn preset_mapping_consumes_all_gain_fields() {
        let preset = preset_from_toml_str(ALIA250).expect("valid Alia 250 preset");
        let mapped = gains_from_preset(preset.gains);
        assert_eq!(mapped.pos_p, preset.gains.pos_p);
        assert_eq!(mapped.pos_accel_limits, preset.gains.pos_accel_limits);
        assert_eq!(mapped.pos_vel_caps, preset.gains.pos_vel_caps);
        assert_eq!(mapped.vel_p, preset.gains.vel_p);
        assert_eq!(mapped.vel_i, preset.gains.vel_i);
        assert_eq!(mapped.vel_d, preset.gains.vel_d);
        assert_eq!(mapped.vel_max_roll_pitch, preset.gains.vel_max_roll_pitch);
        assert_eq!(mapped.vel_max_yaw_step, preset.gains.vel_max_yaw_step);
        assert_eq!(mapped.vel_accel_ff, preset.gains.vel_accel_ff);
        assert_eq!(mapped.att_p, preset.gains.att_p);
        assert_eq!(mapped.att_max_rate_cmd, preset.gains.att_max_rate_cmd);
        assert_eq!(mapped.rate_p, preset.gains.rate_p);
        assert_eq!(mapped.rate_i, preset.gains.rate_i);
        assert_eq!(mapped.rate_d, preset.gains.rate_d);
        assert_eq!(mapped.rate_d_lpf_alpha, preset.gains.rate_d_lpf_alpha);
    }
}
