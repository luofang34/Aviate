//! Unit tests for the velocity loop (split from velocity.rs at the
//! 500-line cap, following the cascade_tests.rs precedent).
use super::*;
use crate::math::Quaternion;

fn ctrl(hover: Scalar) -> VelocityController {
    VelocityController::new(CascadeGains::x500_defaults(), hover)
}

fn zero_vel() -> Vector3<MetersPerSecond> {
    Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    )
}

#[test]
fn p_term_alone_matches_old_behaviour_at_zero_integral() {
    // With zero integrator and zero feedforward, the new
    // controller's output for the same (setpoint, current,
    // hover) must reduce to the legacy P-only formula. This
    // guards against the upgrade silently changing the
    // baseline response.
    let c = ctrl(0.77);
    let mut s = VelocityLoopState::default();
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(1.0), // 1 m/s descend
    );
    let out = c.step(
        &mut s,
        setpoint,
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        None,
        0.0,
    );
    // Legacy formula: trim + (-(setpoint.z - 0) * vel_p[2]),
    // clamped. setpoint.z = 1.0, vel_p[2] from x500 defaults.
    let gain_z = CascadeGains::x500_defaults().vel_p[2];
    let expected: f32 = (0.77 - 1.0 * gain_z).clamp(0.0, 1.0);
    assert!((out.collective.0 - expected).abs() < 1e-5);
}

#[test]
fn integrator_freezes_when_saturated() {
    // If the output saturates at max_up, the integrator must
    // not grow. Force a large velocity error and confirm.
    let c = ctrl(0.5); // makes saturation easy to hit
    let mut s = VelocityLoopState::default();
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(-10.0),
    );
    // First tick — saturated at max_up.
    let _ = c.step(
        &mut s,
        setpoint,
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        None,
        0.01,
    );
    assert_eq!(
        s.integrator_ned.z.0, 0.0,
        "integrator must not grow while saturated"
    );
}

#[test]
fn integrator_grows_when_not_saturated() {
    // Small error keeps the output far from saturation; the
    // integrator must accumulate at error · dt.
    let c = ctrl(0.5);
    let mut s = VelocityLoopState::default();
    let setpoint = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.1),
    );
    let _ = c.step(
        &mut s,
        setpoint,
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        None,
        0.01,
    );
    // error.z = 0.1 - 0 = 0.1. integrator += error · dt = 0.001.
    assert!((s.integrator_ned.z.0 - 0.001).abs() < 1e-6);
}

/// Pin the empirical horizontal sign convention — the
/// cascade-chain consistency tested in SITL. With the wrong
/// pitch / roll sign the horizontal loop closes in positive
/// feedback and the vehicle drifts away from the setpoint;
/// the failure mode only surfaces downstream in gz-physics
/// where root cause is hard to attribute, so this unit test
/// pins the sign with no simulator dependency.
///
/// "Need to move south" = vel_sp_x = −1 m/s, current = 0.
/// The cascade then commands a quaternion whose to_euler
/// pitch is NEGATIVE. The chain's mixer + plant convert that
/// to a south push, even though the to_euler doc-comment
/// names positive pitch "nose up" — the rate-to-mixer half
/// of the loop encodes a sign that completes the cycle in
/// the right direction. Verified end-to-end in SITL.
#[test]
fn horizontal_velocity_error_drives_consistent_tilt_direction() {
    let c = ctrl(0.5);
    let mut s = VelocityLoopState::default();
    let sp_south = Vector3::new(
        MetersPerSecond(-1.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let out_s = c.step(
        &mut s,
        sp_south,
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        None,
        0.0,
    );
    let (_roll_s, pitch_s, _yaw_s) = out_s.attitude.to_euler();
    assert!(
            pitch_s > 0.0,
            "south-bound vel_sp must produce positive-pitch quaternion (nose-up tilts thrust south); got pitch={pitch_s}"
        );

    let mut s2 = VelocityLoopState::default();
    let sp_east = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(1.0),
        MetersPerSecond(0.0),
    );
    let out_e = c.step(
        &mut s2,
        sp_east,
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        None,
        0.0,
    );
    let (roll_e, _pitch_e, _yaw_e) = out_e.attitude.to_euler();
    assert!(
            roll_e > 0.0,
            "east-bound vel_sp must produce positive-roll quaternion (right-wing-down tilts thrust east); got roll={roll_e}"
        );
}

#[test]
fn feedforward_offsets_thrust_when_accel_commanded() {
    // A commanded downward NED acceleration (positive z)
    // should reduce thrust below the trim by ff·trim/g, since
    // less thrust is needed to achieve faster descent. Test
    // forces `vel_accel_ff = 1.0` locally so it isn't
    // affected by tuning changes in the default gains.
    let mut gains = CascadeGains::x500_defaults();
    gains.vel_accel_ff = 1.0;
    let c = VelocityController::new(gains, 0.77);
    let mut s = VelocityLoopState::default();
    let baseline = c.step(
        &mut s,
        zero_vel(),
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        None,
        0.0,
    );
    let mut s2 = VelocityLoopState::default();
    let with_ff = c.step(
        &mut s2,
        zero_vel(),
        zero_vel(),
        AccelFeedforward {
            accel_ned: Vector3::new(
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(1.0), // commanded +1 m/s² down
            ),
        },
        &Quaternion::IDENTITY,
        None,
        0.0,
    );
    assert!(with_ff.collective.0 < baseline.collective.0);
}

fn yaw_of(q: &Quaternion) -> Scalar {
    (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

#[test]
fn commanded_heading_steers_the_attitude_setpoint_yaw() {
    // DRQ: guided modes honor commanded heading. At rest, level,
    // heading north, a 0.3 rad heading setpoint (inside the
    // per-step clamp) must appear directly in the attitude
    // setpoint's yaw.
    let c = ctrl(0.77);
    let mut s = VelocityLoopState::default();
    let out = c.step(
        &mut s,
        zero_vel(),
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        Some(Radians(0.3)),
        0.0,
    );
    assert!((yaw_of(&out.attitude) - 0.3).abs() < 1e-4);
}

#[test]
fn large_heading_error_is_slew_clamped() {
    // A 3 rad heading change applies at most MAX_YAW_ERROR_STEP
    // per cycle so the vehicle turns smoothly instead of the
    // attitude loop being stepped half a revolution.
    let c = ctrl(0.77);
    let mut s = VelocityLoopState::default();
    let out = c.step(
        &mut s,
        zero_vel(),
        zero_vel(),
        AccelFeedforward::default(),
        &Quaternion::IDENTITY,
        Some(Radians(3.0)),
        0.0,
    );
    assert!((yaw_of(&out.attitude) - MAX_YAW_ERROR_STEP).abs() < 1e-4);
}

#[test]
fn absent_heading_setpoint_holds_current_yaw() {
    // No heading in the command: today's hold-current behavior
    // is preserved bit-for-bit in intent (yaw from current
    // attitude).
    let c = ctrl(0.77);
    let mut s = VelocityLoopState::default();
    let current = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 0.9);
    let out = c.step(
        &mut s,
        zero_vel(),
        zero_vel(),
        AccelFeedforward::default(),
        &current,
        None,
        0.0,
    );
    assert!((yaw_of(&out.attitude) - 0.9).abs() < 1e-4);
}

#[test]
fn heading_error_wraps_across_pi() {
    // Current yaw near +178°, command near −178°: the shortest path is
    // +4° through the ±π seam, not −356° the long way round. Covers the
    // err < −π wrap branch.
    let c = ctrl(0.77);
    let mut s = VelocityLoopState::default();
    let current = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 3.1);
    let out = c.step(
        &mut s,
        zero_vel(),
        zero_vel(),
        AccelFeedforward::default(),
        &current,
        Some(Radians(-3.1)),
        0.0,
    );
    let yaw = yaw_of(&out.attitude);
    // Applied yaw continues PAST +π (wraps to negative side), i.e. the
    // vehicle turns the short way: |applied| stays near π, not near 0.
    assert!(
        yaw.abs() > 3.0,
        "short-way wrap expected, got applied yaw {yaw}"
    );
}

#[test]
fn heading_error_wraps_across_minus_pi() {
    // Mirror case: current −178°, command +178° covers the err > π
    // wrap branch.
    let c = ctrl(0.77);
    let mut s = VelocityLoopState::default();
    let current = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), -3.1);
    let out = c.step(
        &mut s,
        zero_vel(),
        zero_vel(),
        AccelFeedforward::default(),
        &current,
        Some(Radians(3.1)),
        0.0,
    );
    let yaw = yaw_of(&out.attitude);
    assert!(
        yaw.abs() > 3.0,
        "short-way wrap expected, got applied yaw {yaw}"
    );
}

/// The NED acceleration command must rotate into the heading frame
/// before becoming roll/pitch: those tilts compose AFTER the yaw
/// quaternion, so they are body-frame quantities. At yaw = +90°
/// (nose east) a NORTH velocity demand lies off the vehicle's LEFT
/// wing — the tilt must be a negative (left-wing-down) roll with no
/// meaningful pitch. The unrotated mapping produces nose-down pitch
/// instead (a push toward the vehicle's nose = east — 90° wrong),
/// which turns the horizontal loop into positive feedback past 90°
/// of heading and spirals position holds away (#110).
#[test]
fn tilt_command_rotates_with_heading() {
    let c = ctrl(0.5);
    let mut s = VelocityLoopState::default();
    let east = Quaternion::from_axis_angle(
        crate::math::Vector3::new(0.0, 0.0, 1.0),
        core::f32::consts::FRAC_PI_2,
    );
    let sp_north = Vector3::new(
        MetersPerSecond(1.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let out = c.step(
        &mut s,
        sp_north,
        zero_vel(),
        AccelFeedforward::default(),
        &east,
        None,
        0.0,
    );
    // Decompose the attitude setpoint as yaw ⊗ tilt: tilt = yaw⁻¹ ⊗ q.
    let yaw_inv = Quaternion::from_axis_angle(
        crate::math::Vector3::new(0.0, 0.0, 1.0),
        -core::f32::consts::FRAC_PI_2,
    );
    let tilt = yaw_inv.mul(&out.attitude);
    let (roll, pitch, _) = tilt.to_euler();
    assert!(
        roll < -0.01,
        "north demand at nose-east must be left-wing-down roll, got roll={roll}"
    );
    assert!(
        pitch.abs() < 0.01,
        "north demand at nose-east must not pitch, got pitch={pitch}"
    );
}

/// Same probe at yaw = 180°: a north demand must become nose-UP
/// pitch (the target is behind the vehicle). The unrotated mapping
/// commands nose-down — directly away from the setpoint, the
/// anti-corrective regime.
#[test]
fn tilt_command_inverts_at_180_deg_heading() {
    let c = ctrl(0.5);
    let mut s = VelocityLoopState::default();
    let south_facing = Quaternion::from_axis_angle(
        crate::math::Vector3::new(0.0, 0.0, 1.0),
        core::f32::consts::PI,
    );
    let sp_north = Vector3::new(
        MetersPerSecond(1.0),
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
    );
    let out = c.step(
        &mut s,
        sp_north,
        zero_vel(),
        AccelFeedforward::default(),
        &south_facing,
        None,
        0.0,
    );
    let yaw_inv = Quaternion::from_axis_angle(
        crate::math::Vector3::new(0.0, 0.0, 1.0),
        -core::f32::consts::PI,
    );
    let tilt = yaw_inv.mul(&out.attitude);
    let (roll, pitch, _) = tilt.to_euler();
    assert!(
        pitch > 0.01,
        "north demand at nose-south must be nose-up pitch, got pitch={pitch}"
    );
    assert!(
        roll.abs() < 0.01,
        "north demand at nose-south must not roll, got roll={roll}"
    );
}
