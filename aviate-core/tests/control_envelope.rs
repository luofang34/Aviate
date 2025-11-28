//! Tests for §13 Envelope Protection
//!
//! The envelope protector constrains setpoints to safe limits.
//!
//! Covers:
//! - Attitude limits (roll/pitch)
//! - Angular rate limits
//! - Altitude limits (min/max)
//! - Velocity limits (horizontal/vertical)
//! - ProtectionStatus flags
//! - No limiting when within bounds

use aviate_core::control::envelope::{
    SimpleEnvelopeProtector, EnvelopeProtector, AxisLimitFlags,
};
use aviate_core::control::{Setpoint, Limits, AuthorityProfile};
use aviate_core::state::{StateEstimate, EstimateQuality, StateValidFlags};
use aviate_core::math::Quaternion;
use aviate_core::types::{Radians, RadiansPerSecond, Meters, MetersPerSecond};

fn make_limits() -> Limits {
    Limits {
        max_roll: Radians(0.5),           // ~28 deg
        max_pitch: Radians(0.5),          // ~28 deg
        max_roll_rate: RadiansPerSecond(2.0),
        max_pitch_rate: RadiansPerSecond(2.0),
        max_yaw_rate: RadiansPerSecond(1.5),
        max_horizontal_speed: MetersPerSecond(10.0),
        max_climb_rate: MetersPerSecond(3.0),
        max_descent_rate: MetersPerSecond(2.0),
        max_altitude: Meters(100.0),
        min_altitude: Meters(5.0),
        min_airspeed: None,
        max_airspeed: None,
        max_load_factor: 2.5,
        min_load_factor: 0.0,
    }
}

fn make_state() -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-50.0)],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    }
}

// =============================================================================
// No Limiting - Within Bounds
// =============================================================================

#[test]
fn within_bounds_not_limited() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        altitude: Some(Meters(50.0)),
        angular_rate: Some([
            RadiansPerSecond(1.0),
            RadiansPerSecond(1.0),
            RadiansPerSecond(1.0),
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(!status.saturated, "Should not be saturated");
    assert!(status.limited_axes.is_empty(), "No axes should be limited");
    assert_eq!(constrained.altitude.unwrap().0, 50.0);
}

#[test]
fn empty_setpoint_not_limited() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint::default();

    let (_, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(!status.saturated);
    assert!(status.limited_axes.is_empty());
}

// =============================================================================
// Altitude Limiting
// =============================================================================

#[test]
fn altitude_exceeds_max_clamped() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        altitude: Some(Meters(150.0)), // Exceeds max 100m
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert!(status.limited_axes.contains(AxisLimitFlags::ALTITUDE));
    assert_eq!(constrained.altitude.unwrap().0, 100.0);
}

#[test]
fn altitude_below_min_clamped() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        altitude: Some(Meters(2.0)), // Below min 5m
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert!(status.limited_axes.contains(AxisLimitFlags::ALTITUDE));
    assert_eq!(constrained.altitude.unwrap().0, 5.0);
}

#[test]
fn altitude_at_boundary_not_limited() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        altitude: Some(Meters(100.0)), // Exactly at max
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(!status.limited_axes.contains(AxisLimitFlags::ALTITUDE));
    assert_eq!(constrained.altitude.unwrap().0, 100.0);
}

// =============================================================================
// Angular Rate Limiting
// =============================================================================

#[test]
fn roll_rate_exceeds_max_clamped() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        angular_rate: Some([
            RadiansPerSecond(5.0), // Exceeds max 2.0
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert!(status.limited_axes.contains(AxisLimitFlags::ROLL));
    assert_eq!(constrained.angular_rate.unwrap()[0].0, 2.0);
}

#[test]
fn negative_pitch_rate_exceeds_max_clamped() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        angular_rate: Some([
            RadiansPerSecond(0.0),
            RadiansPerSecond(-5.0), // Exceeds -max_pitch_rate
            RadiansPerSecond(0.0),
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert!(status.limited_axes.contains(AxisLimitFlags::PITCH));
    assert_eq!(constrained.angular_rate.unwrap()[1].0, -2.0);
}

#[test]
fn yaw_rate_exceeds_max_clamped() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        angular_rate: Some([
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(3.0), // Exceeds max 1.5
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert!(status.limited_axes.contains(AxisLimitFlags::YAW));
    assert_eq!(constrained.angular_rate.unwrap()[2].0, 1.5);
}

// =============================================================================
// Velocity Limiting
// =============================================================================

#[test]
fn horizontal_speed_exceeds_max_scaled() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        velocity: Some([
            MetersPerSecond(20.0), // sqrt(20^2 + 0^2) = 20 > 10 m/s
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert!(status.limited_axes.contains(AxisLimitFlags::SPEED));
    // Should scale down to 10 m/s
    assert!((constrained.velocity.unwrap()[0].0 - 10.0).abs() < 0.1);
}

#[test]
fn diagonal_horizontal_speed_scaled_proportionally() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    // 14.14 m/s at 45 degrees (10, 10) magnitude = sqrt(200) ≈ 14.14
    let setpoint = Setpoint {
        velocity: Some([
            MetersPerSecond(10.0),
            MetersPerSecond(10.0),
            MetersPerSecond(0.0),
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    // Scaled to magnitude 10, direction preserved
    let vx = constrained.velocity.unwrap()[0].0;
    let vy = constrained.velocity.unwrap()[1].0;
    let mag = (vx * vx + vy * vy).sqrt();
    assert!((mag - 10.0).abs() < 0.1, "Magnitude should be 10, got {}", mag);
    assert!((vx - vy).abs() < 0.1, "Direction should be preserved (45 deg)");
}

#[test]
fn vertical_speed_climb_clamped() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        vertical_speed: Some(MetersPerSecond(-5.0)), // NED: negative is climb
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    // max_climb_rate = 3.0, so min vertical_speed = -3.0
    assert_eq!(constrained.vertical_speed.unwrap().0, -3.0);
}

#[test]
fn vertical_speed_descent_clamped() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        vertical_speed: Some(MetersPerSecond(5.0)), // NED: positive is descent
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    // max_descent_rate = 2.0
    assert_eq!(constrained.vertical_speed.unwrap().0, 2.0);
}

// =============================================================================
// Multiple Limits Exceeded
// =============================================================================

#[test]
fn multiple_limits_exceeded_all_flagged() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        altitude: Some(Meters(200.0)),
        angular_rate: Some([
            RadiansPerSecond(10.0),
            RadiansPerSecond(10.0),
            RadiansPerSecond(10.0),
        ]),
        ..Default::default()
    };

    let (_, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert!(status.limited_axes.contains(AxisLimitFlags::ALTITUDE));
    assert!(status.limited_axes.contains(AxisLimitFlags::ROLL));
    assert!(status.limited_axes.contains(AxisLimitFlags::PITCH));
    assert!(status.limited_axes.contains(AxisLimitFlags::YAW));
}

// =============================================================================
// Edge Cases
// =============================================================================

#[test]
fn very_small_exceedance() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        altitude: Some(Meters(100.001)), // Just barely over
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert_eq!(constrained.altitude.unwrap().0, 100.0);
}

#[test]
fn negative_altitude_clamped_to_min() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        altitude: Some(Meters(-10.0)), // Negative altitude
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.saturated);
    assert_eq!(constrained.altitude.unwrap().0, 5.0); // Clamped to min
}
