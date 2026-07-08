//! Kernel-owned controlled Descend/Land failsafe terminal.
//!
//! When a terminal failsafe fires mid-air the kernel must not cut
//! motors — an all-zeros `safe_output` for a hovering multirotor is an
//! uncontrolled fall, not a survivable landing. This module synthesizes
//! the setpoint the cascade rides down on instead: a level attitude
//! holding the current heading, plus a fixed descent rate fed through
//! the existing vertical velocity loop (via [`ControlMode::AltitudeHold`]).
//!
//! Scope matches the inner-loop stabilization contract — attitude +
//! vertical-rate hold. Where the vehicle lands (RTL, geofence, site
//! selection) and any horizontal position-hold upgrade when the
//! position estimate is valid stay with the OEM mode manager; the
//! kernel guarantees only the last-resort survivable descent.
//!
//! `safe_output` (all motors zero) is reserved for the cases where a
//! controlled descent is impossible: on the ground (not armed), total
//! loss of attitude, and unrecoverable numeric/estimator divergence.

use crate::control::{Command, CommandSource, ControlMode, Limits, Setpoint};
use crate::math::{Quaternion, Vector3};
use crate::state::StateEstimate;
use crate::types::{MetersPerSecond, Radians};

/// Build the controlled descent setpoint for the terminal failsafe.
///
/// The synthesized command drives the normal cascade, not a bespoke
/// control law:
///
/// - **Vertical**: `vertical_speed = limits.max_descent_rate` (NED z is
///   down-positive, so a positive value is a descent — a negative climb
///   rate). The vertical velocity loop regulates collective around the
///   hover trim to hold this rate rather than free-falling.
/// - **Attitude**: level (zero roll/pitch) holding the current heading,
///   so the vehicle rides down wings-level instead of at whatever bank
///   it held when the failsafe fired.
///
/// `sequence` carries through from the command that would otherwise
/// have flown, so downstream sequence tracking is uninterrupted. The
/// source is tagged [`CommandSource::Failsafe`] to mark the setpoint as
/// kernel-synthesized.
pub fn descend_command(state: &StateEstimate, limits: &Limits, sequence: u32) -> Command {
    // Hold the heading the vehicle already flies; only roll/pitch are
    // driven to level. A yaw snap during a failsafe descent buys
    // nothing and can excite the yaw loop needlessly.
    let (_roll, _pitch, yaw) = state.attitude.to_euler();
    let level = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), yaw).normalize();

    Command {
        mode: ControlMode::AltitudeHold,
        setpoint: Setpoint {
            attitude: Some(level),
            vertical_speed: Some(MetersPerSecond(limits.max_descent_rate.0)),
            heading: Some(Radians(yaw)),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Failsafe,
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::control::ControlMode;
    use crate::state::{EstimateQuality, StateValidFlags};
    use crate::types::Meters;

    fn state_with_yaw(yaw: crate::types::Scalar) -> StateEstimate {
        let attitude = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), yaw);
        StateEstimate {
            attitude,
            angular_velocity: [crate::types::RadiansPerSecond(0.0); 3],
            position_ned: [Meters(0.0); 3],
            velocity_ned: [MetersPerSecond(0.0); 3],
            quality: EstimateQuality::Good,
            valid_flags: StateValidFlags::ATTITUDE | StateValidFlags::VELOCITY,
        }
    }

    fn limits() -> Limits {
        crate::kernel::config::ResolvedKernelConfig::default().limits
    }

    #[test]
    fn descends_at_the_envelope_descent_rate() {
        let lim = limits();
        let cmd = descend_command(&state_with_yaw(0.0), &lim, 7);
        // NED down-positive: a positive vertical_speed is a descent.
        let vspeed = cmd
            .setpoint
            .vertical_speed
            .expect("descend command carries a vertical-speed setpoint");
        assert_eq!(vspeed.0, lim.max_descent_rate.0);
        assert!(
            vspeed.0 > 0.0,
            "descent must command a downward (negative-climb) rate"
        );
    }

    #[test]
    fn commands_altitude_hold_from_the_failsafe_source() {
        let cmd = descend_command(&state_with_yaw(0.0), &limits(), 3);
        assert_eq!(cmd.mode, ControlMode::AltitudeHold);
        assert_eq!(cmd.source, CommandSource::Failsafe);
        assert_eq!(cmd.sequence, 3);
    }

    #[test]
    fn holds_level_attitude_at_current_heading() {
        let yaw = 1.2;
        let cmd = descend_command(&state_with_yaw(yaw), &limits(), 0);
        let att = cmd
            .setpoint
            .attitude
            .expect("descend command carries a level attitude setpoint");
        let (roll, pitch, out_yaw) = att.to_euler();
        assert!(roll.abs() < 1e-4, "descent attitude must be level in roll");
        assert!(
            pitch.abs() < 1e-4,
            "descent attitude must be level in pitch"
        );
        assert!(
            (out_yaw - yaw).abs() < 1e-4,
            "descent attitude must hold the current heading"
        );
        let heading = cmd
            .setpoint
            .heading
            .expect("descend command slaves yaw to the held heading");
        assert!((heading.0 - yaw).abs() < 1e-4);
    }
}
