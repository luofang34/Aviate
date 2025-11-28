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

// =============================================================================
// Attitude Limiting (Roll/Pitch from Quaternion)
// =============================================================================

#[test]
fn attitude_roll_exceeds_positive_limit() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits(); // max_roll = 0.5 rad
    let state = make_state();

    // Create quaternion with large roll (1.0 rad > 0.5 limit)
    let roll_quat = Quaternion::from_axis_angle(
        aviate_core::math::Vector3::new(1.0, 0.0, 0.0),
        1.0 // rad, exceeds 0.5 limit
    );

    let setpoint = Setpoint {
        attitude: Some(roll_quat),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.limited_axes.contains(AxisLimitFlags::ROLL));

    // Check that roll is clamped
    if let Some(att) = constrained.attitude {
        let (r, _p, _y) = att.to_euler();
        assert!(r.abs() <= limits.max_roll.0 + 0.01, "Roll {} should be <= {}", r, limits.max_roll.0);
    }
}

#[test]
fn attitude_roll_exceeds_negative_limit() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    // Create quaternion with large negative roll
    let roll_quat = Quaternion::from_axis_angle(
        aviate_core::math::Vector3::new(1.0, 0.0, 0.0),
        -1.0 // rad, exceeds -0.5 limit
    );

    let setpoint = Setpoint {
        attitude: Some(roll_quat),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.limited_axes.contains(AxisLimitFlags::ROLL));

    if let Some(att) = constrained.attitude {
        let (r, _p, _y) = att.to_euler();
        assert!(r.abs() <= limits.max_roll.0 + 0.01, "Roll {} should be within limit", r);
    }
}

#[test]
fn attitude_pitch_exceeds_positive_limit() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits(); // max_pitch = 0.5 rad
    let state = make_state();

    // Create quaternion with large pitch
    let pitch_quat = Quaternion::from_axis_angle(
        aviate_core::math::Vector3::new(0.0, 1.0, 0.0),
        1.0 // rad, exceeds 0.5 limit
    );

    let setpoint = Setpoint {
        attitude: Some(pitch_quat),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.limited_axes.contains(AxisLimitFlags::PITCH));

    if let Some(att) = constrained.attitude {
        let (_r, p, _y) = att.to_euler();
        assert!(p.abs() <= limits.max_pitch.0 + 0.01, "Pitch {} should be <= {}", p, limits.max_pitch.0);
    }
}

#[test]
fn attitude_pitch_exceeds_negative_limit() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    // Create quaternion with large negative pitch
    let pitch_quat = Quaternion::from_axis_angle(
        aviate_core::math::Vector3::new(0.0, 1.0, 0.0),
        -1.0 // rad
    );

    let setpoint = Setpoint {
        attitude: Some(pitch_quat),
        ..Default::default()
    };

    let (_constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.limited_axes.contains(AxisLimitFlags::PITCH));
}

#[test]
fn attitude_within_limits_not_modified() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    // Create quaternion with small roll/pitch within limits
    let small_roll = Quaternion::from_axis_angle(
        aviate_core::math::Vector3::new(1.0, 0.0, 0.0),
        0.3 // rad, within 0.5 limit
    );

    let setpoint = Setpoint {
        attitude: Some(small_roll),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    // Should not be limited
    assert!(!status.limited_axes.contains(AxisLimitFlags::ROLL));
    assert!(!status.limited_axes.contains(AxisLimitFlags::PITCH));

    // Attitude should be unchanged (within tolerance)
    if let Some(att) = constrained.attitude {
        let (r, _p, _y) = att.to_euler();
        assert!((r - 0.3).abs() < 0.05, "Roll should be ~0.3, got {}", r);
    }
}

#[test]
fn attitude_roll_and_pitch_both_exceed() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    // Create quaternion with both roll and pitch exceeding limits
    // Use Euler angles to ensure both are clearly over limit
    let qr = Quaternion::from_axis_angle(aviate_core::math::Vector3::new(1.0, 0.0, 0.0), 0.8);
    let qp = Quaternion::from_axis_angle(aviate_core::math::Vector3::new(0.0, 1.0, 0.0), 0.8);
    let combined = qp.mul(&qr); // Y then X rotation for proper Euler ordering

    let setpoint = Setpoint {
        attitude: Some(combined),
        ..Default::default()
    };

    let (_, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    // At least one should be limited (combined angles may interact)
    assert!(
        status.limited_axes.contains(AxisLimitFlags::ROLL) ||
        status.limited_axes.contains(AxisLimitFlags::PITCH),
        "At least one axis should be limited"
    );
}

// =============================================================================
// Angular Rate Negative Limits
// =============================================================================

#[test]
fn roll_rate_negative_exceeds_limit() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        angular_rate: Some([
            RadiansPerSecond(-5.0), // Exceeds -max_roll_rate
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.limited_axes.contains(AxisLimitFlags::ROLL));
    assert_eq!(constrained.angular_rate.unwrap()[0].0, -2.0);
}

#[test]
fn pitch_rate_positive_exceeds_limit() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        angular_rate: Some([
            RadiansPerSecond(0.0),
            RadiansPerSecond(5.0), // Exceeds max_pitch_rate
            RadiansPerSecond(0.0),
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.limited_axes.contains(AxisLimitFlags::PITCH));
    assert_eq!(constrained.angular_rate.unwrap()[1].0, 2.0);
}

#[test]
fn yaw_rate_negative_exceeds_limit() {
    let protector = SimpleEnvelopeProtector;
    let limits = make_limits();
    let state = make_state();

    let setpoint = Setpoint {
        angular_rate: Some([
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(-3.0), // Exceeds -max_yaw_rate
        ]),
        ..Default::default()
    };

    let (constrained, status) = protector.constrain(&setpoint, &state, &limits, AuthorityProfile::HardEnvelope);

    assert!(status.limited_axes.contains(AxisLimitFlags::YAW));
    assert_eq!(constrained.angular_rate.unwrap()[2].0, -1.5);
}
