//! Regression tests for the ESKF attitude-error frame convention.
//!
//! The filter's covariance Jacobians define the attitude error in the
//! GLOBAL (nav) frame: `dVel/dAtt = -[R·a]×`, `dAtt/dGyroBias = -R·dt`,
//! and no `-[ω]×` term in the attitude block. A correction computed
//! from an innovation is therefore a nav-frame rotation and must be
//! applied as `q ← δq ⊗ q`. Applying it about BODY axes instead is
//! indistinguishable while the attitude is near identity — which is
//! where every square mission flies, since the SITL vehicle spawns at
//! NED yaw 0 — but at 90° of yaw a horizontal correction lands on the
//! wrong axis, and past 180° it lands sign-flipped: every GNSS fusion
//! then *amplifies* the tilt error it measured, and sustained-yaw
//! flight diverges the filter (the #123 crash signature: vertical
//! velocity running away at ~2 g while the vehicle sits on the deck).

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::ekf::{Ekf, EkfConfig, EkfState};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{GnssData, GnssFix, GnssHealth, ImuData, SensorHealth, SensorReading};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond};

fn gnss_at_rest() -> SensorReading<GnssData> {
    SensorReading {
        value: GnssData {
            position_ned: [Meters(0.0), Meters(0.0), Meters(0.0)],
            velocity_ned: [
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ],
            fix: GnssFix::ThreeD,
            health: GnssHealth::Good,
        },
        timestamp: Timestamp {
            ticks: 0,
            source: TimeSource::Internal,
        },
        health: SensorHealth::Good,
        valid: true,
        source_id: 0,
    }
}

/// Angle between the two attitudes' body-z axes in the nav frame —
/// the tilt component of attitude error, which is what gravity
/// mis-projection feeds on.
fn tilt_between(a: &Quaternion, b: &Quaternion) -> f32 {
    let za = a.rotate_vector(Vector3::new(0.0, 0.0, 1.0));
    let zb = b.rotate_vector(Vector3::new(0.0, 0.0, 1.0));
    let dot = (za.x * zb.x + za.y * zb.y + za.z * zb.z).clamp(-1.0, 1.0);
    dot.acos()
}

/// At 180° of yaw, GNSS fusion must still *shrink* a tilt error.
///
/// The vehicle rests level, heading south (NED yaw = π). The filter
/// is seeded with a 0.08 rad nav-frame roll error. At rest the IMU
/// reports pure gravity; the tilted estimate mis-projects it, the
/// velocity estimate drifts, and the zero-velocity GNSS innovations
/// carry the tilt information back through the covariance
/// cross-terms. With the correction applied in the nav frame the
/// tilt error converges; applied about body axes it is sign-flipped
/// at this heading and grows instead — this test diverges within a
/// second of simulated time on the pre-fix code.
#[test]
fn gnss_fusion_shrinks_tilt_error_at_180_deg_yaw() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    let q_true = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), core::f32::consts::PI);
    // Nav-frame roll error, composed globally: q_est = δq ⊗ q_true.
    let q_err = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 0.08);
    let q_est = q_err.mul(&q_true);

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        q_est,
    );

    // At rest the accelerometer measures the specific force opposing
    // gravity, rotated into the body frame by the TRUE attitude:
    // f_b = R_trueᵀ·(0,0,-g). Heading south and level, that is still
    // (0,0,-g). No rotation.
    let imu = ImuData {
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
    };

    let initial_tilt = tilt_between(&state.quat, &q_true);
    let gnss = gnss_at_rest();

    // 3 s of simulated flight loop: predict at 200 Hz, GNSS at 40 Hz.
    for i in 0..600 {
        ekf.predict_state(&mut state, &imu, 0.005);
        if i % 5 == 0 {
            ekf.update_gnss_state(&mut state, &gnss);
        }
    }

    let final_tilt = tilt_between(&state.quat, &q_true);
    assert!(
        final_tilt < initial_tilt * 0.5,
        "tilt error must converge at yaw=π: initial {initial_tilt:.4} rad, final {final_tilt:.4} rad"
    );
    let speed_sq = state.vel.x.0 * state.vel.x.0
        + state.vel.y.0 * state.vel.y.0
        + state.vel.z.0 * state.vel.z.0;
    assert!(
        speed_sq < 0.25,
        "velocity must stay bounded at rest, got |v|² = {speed_sq:.3}"
    );
}

/// Direct convention probe: a velocity innovation's attitude
/// correction rotates about NAV axes, not body axes.
///
/// The estimate sits at exactly NED yaw = π with a hand-planted
/// covariance cross-term between vel-N (state 3) and the pitch
/// error (state 7). A +1 m/s velocity-N innovation then commands a
/// positive nav-frame pitch-error correction δθ_y. Composed
/// globally, `Ry(δ)·Rz(π)` decomposes as yaw π with euler pitch −δ
/// (the euler pitch is measured in the yawed frame, so the sign
/// flips). The broken body-frame application produces `Rz(π)·Ry(δ)`
/// = euler pitch +δ — opposite sign, which is exactly the
/// anti-corrective behavior that diverges sustained-yaw flight.
#[test]
fn attitude_correction_is_applied_in_the_nav_frame() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    let q_yaw_pi = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), core::f32::consts::PI);
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        q_yaw_pi,
    );

    // Positive cross-covariance vel-N ↔ pitch-error: a positive
    // velocity-N innovation implies a positive global pitch-error
    // correction.
    state.p_cov.set(3, 7, 0.05);
    state.p_cov.set(7, 3, 0.05);

    let mut gnss = gnss_at_rest();
    gnss.value.velocity_ned[0] = MetersPerSecond(1.0);
    ekf.update_gnss_state(&mut state, &gnss);

    let (roll, pitch, yaw) = state.quat.to_euler();
    assert!(
        pitch < -1e-3,
        "global +pitch-error correction at yaw=π must show as euler pitch −δ, got {pitch:.5}"
    );
    assert!(
        roll.abs() < 1e-3 && yaw.abs() > 3.0,
        "correction must not disturb roll or yaw: roll {roll:.5}, yaw {yaw:.5}"
    );
}
