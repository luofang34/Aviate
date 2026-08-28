//! Where the nose is told to point, and how fast it may be turned there.

use super::*;

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
    // A 3 rad heading change applies at most `vel_max_yaw_step`
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
    assert!((yaw_of(&out.attitude) - c.gains.vel_max_yaw_step).abs() < 1e-4);
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
