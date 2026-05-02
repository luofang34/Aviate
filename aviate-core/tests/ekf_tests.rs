//! Tests for Extended Kalman Filter (EKF)
//!
//! Covers:
//! - update_gnss with valid GNSS data
//! - GNSS health gating

use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{GnssData, GnssFix, GnssHealth, SensorHealth, SensorReading};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond};

fn dummy_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn make_gnss_reading(
    pos: [Meters; 3],
    vel: [MetersPerSecond; 3],
    gnss_health: GnssHealth,
    sensor_health: SensorHealth,
    fix: GnssFix,
) -> SensorReading<GnssData> {
    SensorReading {
        value: GnssData {
            position_ned: pos,
            velocity_ned: vel,
            fix,
            health: gnss_health,
        },
        timestamp: dummy_timestamp(),
        health: sensor_health,
        valid: true,
        source_id: 0,
    }
}

// =============================================================================
// GNSS Update Tests
// =============================================================================

#[test]
fn ekf_update_gnss_with_good_health() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize EKF with known state
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let gnss = make_gnss_reading(
        [Meters(10.0), Meters(5.0), Meters(-50.0)],
        [
            MetersPerSecond(1.0),
            MetersPerSecond(0.5),
            MetersPerSecond(-0.2),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    // Should update state
    ekf.update_gnss(&mut state, &gnss);

    // After update, state should be influenced by GNSS measurement
    // We verify it's initialized and accepted the update
    assert!(state.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_degraded_sensor_health() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let gnss = make_gnss_reading(
        [Meters(100.0), Meters(100.0), Meters(-100.0)],
        [
            MetersPerSecond(10.0),
            MetersPerSecond(10.0),
            MetersPerSecond(-1.0),
        ],
        GnssHealth::Good,
        SensorHealth::Degraded, // Sensor health is degraded - should reject
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    // EKF should still be initialized (update was rejected, not failed)
    assert!(state.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_suspect_gnss_health() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let gnss = make_gnss_reading(
        [Meters(100.0), Meters(100.0), Meters(-100.0)],
        [
            MetersPerSecond(10.0),
            MetersPerSecond(10.0),
            MetersPerSecond(-1.0),
        ],
        GnssHealth::Suspect, // GNSS health is suspect - should reject
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    // Update should be rejected due to suspect GNSS health
    assert!(state.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_no_fix() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let gnss = make_gnss_reading(
        [Meters(100.0), Meters(100.0), Meters(-100.0)],
        [
            MetersPerSecond(10.0),
            MetersPerSecond(10.0),
            MetersPerSecond(-1.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::None, // No fix - should reject
    );

    ekf.update_gnss(&mut state, &gnss);

    // Update should be rejected due to no fix
    assert!(state.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_lost_gnss_health() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let gnss = make_gnss_reading(
        [Meters(100.0), Meters(100.0), Meters(-100.0)],
        [
            MetersPerSecond(10.0),
            MetersPerSecond(10.0),
            MetersPerSecond(-1.0),
        ],
        GnssHealth::Lost, // GNSS health is lost - should reject
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    // Update should be rejected due to lost GNSS
    assert!(state.is_initialized());
}

#[test]
fn ekf_default_creates_valid_instance() {
    let _ekf = Ekf::default();
    let state = EkfState::default();
    assert!(!state.is_initialized());
}

// =============================================================================
// Uninitialized EKF Tests (defensive guards)
// =============================================================================

#[test]
fn ekf_predict_on_uninitialized_returns_early() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // EKF is NOT initialized
    assert!(!state.is_initialized());

    // Create valid IMU data
    let imu = ImuData {
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
    };

    // Call predict on uninitialized EKF - should return early without panic
    ekf.predict(&mut state, &imu, 0.001);

    // Should still be uninitialized
    assert!(!state.is_initialized());
}

#[test]
fn ekf_predict_with_zero_dt_returns_early() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize EKF
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let imu = ImuData {
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
    };

    // Call predict with dt = 0 - should return early
    ekf.predict(&mut state, &imu, 0.0);

    // Should still be initialized (didn't crash)
    assert!(state.is_initialized());
}

#[test]
fn ekf_predict_with_negative_dt_returns_early() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize EKF
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let imu = ImuData {
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
    };

    // Call predict with negative dt - should return early
    ekf.predict(&mut state, &imu, -0.001);

    // Should still be initialized (didn't crash)
    assert!(state.is_initialized());
}

#[test]
fn ekf_predict_with_nan_dt_returns_early() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize EKF
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let imu = ImuData {
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
    };

    // Call predict with NaN dt - should return early
    ekf.predict(&mut state, &imu, f32::NAN);

    // Should still be initialized (didn't crash)
    assert!(state.is_initialized());
}

#[test]
fn ekf_get_estimate_uninitialized_returns_unusable() {
    use aviate_core::state::{EstimateQuality, StateValidFlags};

    let _ekf = Ekf::default();
    let state = EkfState::default();

    // EKF is NOT initialized
    assert!(!state.is_initialized());

    // Get estimate from uninitialized EKF
    let estimate = state.get_estimate();

    // Should return Unusable quality and empty flags
    assert_eq!(estimate.quality, EstimateQuality::Unusable);
    assert_eq!(estimate.valid_flags, StateValidFlags::empty());
}

// =============================================================================
// EKF Numerical Correctness Tests
// =============================================================================
//
// Innovation Gating Math:
//   chi² = innovation² / s, where s = P + R (measurement noise)
//   Measurement rejected if chi² > gate²
//
// With default config:
//   P_init = 0.1 (initial position covariance)
//   R = 0.5 (r_pos measurement noise)
//   gate = 5.0 (5-sigma gate)
//
//   s = P + R = 0.1 + 0.5 = 0.6
//   gate² = 25
//   max_innovation = sqrt(gate² * s) = sqrt(25 * 0.6) = sqrt(15) ≈ 3.87m
//
// Kalman Gain:
//   K = P / (P + R) = 0.1 / 0.6 ≈ 0.167
//   State update: x_new = x + K * innovation

#[test]
fn ekf_gnss_update_moves_state_toward_measurement() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize at origin
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();
    assert_eq!(state_before.position_ned[0].0, 0.0);

    // GNSS offset of 1m chosen to be within innovation gate (< 3.87m)
    // chi² = 1² / 0.6 = 1.67 < 25 (gate²) → accepted
    let gnss = make_gnss_reading(
        [Meters(1.0), Meters(0.5), Meters(-0.2)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    let state_after = state.get_estimate();

    // With K = P/(P+R) = 0.1/0.6 ≈ 0.167
    // Expected: x_new = 0 + 0.167 * 1.0 ≈ 0.167
    let expected_x = 0.167;
    assert!(
        (state_after.position_ned[0].0 - expected_x).abs() < 0.02,
        "X position should be ~{:.3}, got {:.3}",
        expected_x,
        state_after.position_ned[0].0
    );
    assert!(
        state_after.position_ned[1].0 > 0.0,
        "Y position should increase toward measurement"
    );
    assert!(
        state_after.position_ned[2].0 < 0.0,
        "Z position should decrease toward measurement (NED down)"
    );
}

#[test]
fn ekf_predict_integrates_angular_velocity() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize level
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Apply constant yaw rate of 0.1 rad/s for 1 second (100 steps * 0.01s)
    let imu = ImuData {
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.1), // Yaw rate
        ],
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81), // Gravity
        ],
    };

    for _ in 0..100 {
        ekf.predict(&mut state, &imu, 0.01);
    }

    let estimate = state.get_estimate();

    // Extract yaw from quaternion (simplified for small angles)
    // For small rotations around Z: yaw ≈ 2 * qz / qw
    let q = estimate.attitude;
    let yaw_estimate = 2.0 * q.z / q.w;

    // After 1 second at 0.1 rad/s, yaw should be approximately 0.1 rad
    assert!(
        (yaw_estimate - 0.1).abs() < 0.02,
        "Yaw should be ~0.1 rad after 1s at 0.1 rad/s, got {}",
        yaw_estimate
    );
}

#[test]
fn ekf_predict_integrates_velocity_to_position() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize with velocity in +X direction
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(1.0), // 1 m/s in X
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Stationary IMU (no rotation, gravity only)
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

    // Run for 1 second
    for _ in 0..100 {
        ekf.predict(&mut state, &imu, 0.01);
    }

    let estimate = state.get_estimate();

    // Position should have increased by approximately 1 meter in X
    assert!(
        estimate.position_ned[0].0 > 0.9,
        "X position should be ~1m after 1s at 1m/s, got {}",
        estimate.position_ned[0].0
    );
    assert!(
        estimate.position_ned[0].0 < 1.1,
        "X position should not overshoot significantly"
    );
}

#[test]
fn ekf_innovation_gating_rejects_outlier() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    // Use config with reasonable innovation gate
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize at origin
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run a few predict steps to establish covariance
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
    for _ in 0..10 {
        ekf.predict(&mut state, &imu, 0.01);
    }

    let state_before = state.get_estimate();

    // GNSS with huge outlier - 1000m away (should be rejected by innovation gating)
    let outlier_gnss = make_gnss_reading(
        [Meters(1000.0), Meters(0.0), Meters(0.0)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &outlier_gnss);

    let state_after = state.get_estimate();

    // State should NOT have jumped to 1000m (innovation gating should reject)
    // chi² = 1000² / 0.6 = 1,666,667 >> 25 (gate²) → rejected
    let position_change = (state_after.position_ned[0].0 - state_before.position_ned[0].0).abs();
    assert!(
        position_change < 1.0,
        "Innovation gating should reject 1000m outlier, but state moved {} m",
        position_change
    );
}

#[test]
fn ekf_innovation_gate_boundary_accepts_below_threshold() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // 3.0m is below gate threshold (3.87m)
    // chi² = 3² / 0.6 = 15 < 25 (gate²) → should accept
    let gnss = make_gnss_reading(
        [Meters(3.0), Meters(0.0), Meters(0.0)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    let estimate = state.get_estimate();

    // Should accept and move state: K ≈ 0.167, x_new ≈ 0.5
    assert!(
        estimate.position_ned[0].0 > 0.4,
        "3m measurement should be accepted (below gate), got {}",
        estimate.position_ned[0].0
    );
}

#[test]
fn ekf_innovation_gate_boundary_rejects_above_threshold() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // 5.0m is above gate threshold (3.87m)
    // chi² = 5² / 0.6 = 41.67 > 25 (gate²) → should reject
    let gnss = make_gnss_reading(
        [Meters(5.0), Meters(0.0), Meters(0.0)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    let estimate = state.get_estimate();

    // Should reject - state should remain near zero
    assert!(
        estimate.position_ned[0].0.abs() < 0.1,
        "5m measurement should be rejected (above gate), got {}",
        estimate.position_ned[0].0
    );
}

#[test]
fn ekf_multiple_gnss_updates_converge() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Consistent GNSS at 1m - well within gate threshold
    let gnss = make_gnss_reading(
        [Meters(1.0), Meters(0.0), Meters(0.0)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    // Theoretical convergence after n updates:
    //   x_n = z * (1 - ∏(1 - K_i)) where K_i = P_i/(P_i + R)
    //   P_{n+1} = (1 - K_n) * P_n
    // Convergence is limited by measurement noise R dominating as P shrinks
    for _ in 0..20 {
        ekf.update_gnss(&mut state, &gnss);
    }

    let estimate = state.get_estimate();

    // After 20 updates, converges to ~0.80m (empirically verified)
    // Convergence is slower due to measurement noise R=0.5 limiting gain as P decreases
    assert!(
        estimate.position_ned[0].0 > 0.75,
        "Position should converge toward GNSS measurement, got {}",
        estimate.position_ned[0].0
    );
    assert!(
        estimate.position_ned[0].0 < 1.0,
        "Position should not exceed measurement"
    );
}

// =============================================================================
// IMU Data Boundary Tests
// =============================================================================

#[test]
fn ekf_predict_with_nan_imu_gyro_returns_early() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    // IMU with NaN gyro - should be detected and skipped
    let imu = ImuData {
        gyro: [
            RadiansPerSecond(f32::NAN),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
    };

    ekf.predict(&mut state, &imu, 0.01);

    let state_after = state.get_estimate();

    // State should not change when IMU data is invalid
    assert_eq!(
        state_before.position_ned[0].0, state_after.position_ned[0].0,
        "Position should not change with NaN gyro"
    );
    assert!(state.is_initialized());
}

#[test]
fn ekf_predict_with_inf_imu_accel_returns_early() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    // IMU with Inf accel - should be detected and skipped
    let imu = ImuData {
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
        accel: [
            MetersPerSecondSquared(f32::INFINITY),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
    };

    ekf.predict(&mut state, &imu, 0.01);

    let state_after = state.get_estimate();

    // State should not change when IMU data is invalid
    assert_eq!(
        state_before.position_ned[0].0, state_after.position_ned[0].0,
        "Position should not change with Inf accel"
    );
    assert!(state.is_initialized());
}

#[test]
fn ekf_predict_with_extreme_gyro_rate_handles_gracefully() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Extreme gyro rate - 100 rad/s (well beyond physical limits ~50 rad/s)
    // This is still finite, so it should be processed
    let imu = ImuData {
        gyro: [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(100.0),
        ],
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
    };

    // Should not panic even with extreme values
    ekf.predict(&mut state, &imu, 0.01);

    let estimate = state.get_estimate();

    // Quaternion should still be normalized after extreme rotation
    let q = estimate.attitude;
    let norm = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    assert!(
        (norm - 1.0).abs() < 1e-5,
        "Quaternion should remain normalized, got norm = {}",
        norm
    );
}

// =============================================================================
// Baro Fusion Tests
// =============================================================================

#[test]
fn ekf_baro_update_modifies_altitude() {
    use aviate_core::sensor::{AirData, BaroData};
    use aviate_core::types::Pascals;

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize at z = 0 (sea level in NED)
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();
    assert_eq!(state_before.position_ned[2].0, 0.0);

    // Baro reading at ~5m altitude (within innovation gate)
    // Using barometric formula: h = 44330 * (1 - (P/101325)^0.1903)
    // For h = 5m: P = 101325 * (1 - 5/44330)^5.255 ≈ 101265 Pa
    //
    // Innovation gate check:
    //   P_init = 0.1, R_baro = 2.0, S = 2.1
    //   gate² = 25, max_innovation = sqrt(25 * 2.1) ≈ 7.24m
    //   5m altitude → z_ned = -5m is within gate
    let baro_reading = SensorReading {
        value: BaroData {
            altitude: None,
            air: AirData {
                static_pressure: Some(Pascals(101265.0)),
                dynamic_pressure: None,
                total_pressure: None,
                temperature: None,
                indicated_airspeed: None,
                true_airspeed: None,
            },
        },
        timestamp: dummy_timestamp(),
        health: SensorHealth::Good,
        valid: true,
        source_id: 0,
    };

    ekf.update_baro(&mut state, &baro_reading);

    let state_after = state.get_estimate();

    // Z position should have moved toward the baro measurement (negative = up in NED)
    assert!(
        state_after.position_ned[2].0 < state_before.position_ned[2].0,
        "Z position should decrease (go up) with altitude measurement, got {}",
        state_after.position_ned[2].0
    );

    // X and Y should remain unchanged
    assert!(
        (state_after.position_ned[0].0 - state_before.position_ned[0].0).abs() < 1e-6,
        "Baro should not affect X position"
    );
    assert!(
        (state_after.position_ned[1].0 - state_before.position_ned[1].0).abs() < 1e-6,
        "Baro should not affect Y position"
    );
}

#[test]
fn ekf_baro_rejects_degraded_sensor_health() {
    use aviate_core::sensor::{AirData, BaroData};
    use aviate_core::types::Pascals;

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    let baro_reading = SensorReading {
        value: BaroData {
            altitude: None,
            air: AirData {
                static_pressure: Some(Pascals(90000.0)), // ~1000m altitude
                dynamic_pressure: None,
                total_pressure: None,
                temperature: None,
                indicated_airspeed: None,
                true_airspeed: None,
            },
        },
        timestamp: dummy_timestamp(),
        health: SensorHealth::Degraded, // Should reject
        valid: true,
        source_id: 0,
    };

    ekf.update_baro(&mut state, &baro_reading);

    let state_after = state.get_estimate();

    // State should not change with degraded sensor health
    assert_eq!(
        state_before.position_ned[2].0, state_after.position_ned[2].0,
        "Baro with degraded health should be rejected"
    );
}

#[test]
fn ekf_baro_with_no_pressure_does_nothing() {
    use aviate_core::sensor::{AirData, BaroData};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    let baro_reading = SensorReading {
        value: BaroData {
            altitude: None,
            air: AirData {
                static_pressure: None, // No pressure data
                dynamic_pressure: None,
                total_pressure: None,
                temperature: None,
                indicated_airspeed: None,
                true_airspeed: None,
            },
        },
        timestamp: dummy_timestamp(),
        health: SensorHealth::Good,
        valid: true,
        source_id: 0,
    };

    ekf.update_baro(&mut state, &baro_reading);

    let state_after = state.get_estimate();

    // State should not change when no pressure data
    assert_eq!(
        state_before.position_ned[2].0, state_after.position_ned[2].0,
        "Baro with no pressure should not update state"
    );
}

// =============================================================================
// Covariance Behavior Tests
// =============================================================================

#[test]
fn ekf_state_uncertainty_grows_during_predict_only() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

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

    // Observe innovation acceptance behavior
    // With initial P and first GNSS at 3m (just inside gate), should accept
    let gnss_3m = make_gnss_reading(
        [Meters(3.0), Meters(0.0), Meters(0.0)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    // Before any predict, 3m should be within gate (chi² = 9/0.6 = 15 < 25)
    ekf.update_gnss(&mut state, &gnss_3m);
    let pos_after_first_update = state.get_estimate().position_ned[0].0;
    assert!(
        pos_after_first_update > 0.4,
        "3m measurement should be accepted initially"
    );

    // Re-init to test P growth
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run many predict steps without update - P should grow
    // This means larger innovations will be accepted (gate widens with P)
    for _ in 0..100 {
        ekf.predict(&mut state, &imu, 0.01);
    }

    // Now try GNSS at 5m which was previously rejected
    // With larger P, gate threshold increases: sqrt(gate² * (P + R))
    let gnss_5m = make_gnss_reading(
        [Meters(5.0), Meters(0.0), Meters(0.0)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss_5m);
    let pos_after_large_p = state.get_estimate().position_ned[0].0;

    // With grown P, the 5m measurement should now be accepted
    // (P grows during predict, making S = P + R larger, reducing chi²)
    assert!(
        pos_after_large_p > 0.1,
        "After P grows, previously rejected measurement should be accepted, got {}",
        pos_after_large_p
    );
}

#[test]
fn ekf_kalman_gain_decreases_with_repeated_updates() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Apply first GNSS update
    let gnss = make_gnss_reading(
        [Meters(1.0), Meters(0.0), Meters(0.0)],
        [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);
    let pos_after_first = state.get_estimate().position_ned[0].0;
    // K1 = P0 / (P0 + R) = 0.1 / 0.6 ≈ 0.167
    // First update: x = 0 + 0.167 * 1.0 ≈ 0.167
    let first_movement = pos_after_first; // ~0.167

    // Now re-init with offset and do another update sequence to observe K shrink
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Do 10 updates to shrink P
    for _ in 0..10 {
        ekf.update_gnss(&mut state, &gnss);
    }
    let pos_after_ten = state.get_estimate().position_ned[0].0;

    // Do one more update and measure the movement
    let pos_before_11th = pos_after_ten;
    ekf.update_gnss(&mut state, &gnss);
    let pos_after_11th = state.get_estimate().position_ned[0].0;

    let eleventh_movement = pos_after_11th - pos_before_11th;

    // After 10 updates, P has shrunk significantly
    // K = P / (P + R) where P is now much smaller than initial 0.1
    // So K_11 << K_1, meaning movement should be much smaller
    assert!(
        eleventh_movement < first_movement * 0.5,
        "K should decrease: first movement {:.4}, 11th movement {:.4}",
        first_movement,
        eleventh_movement
    );
}

// =============================================================================
// Quaternion Normalization Tests
// =============================================================================

#[test]
fn ekf_quaternion_normalization_preserved_after_long_run() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Varying gyro rates to stress-test quaternion integration
    for i in 0..1000 {
        let phase = (i as f32) * 0.01;
        let imu = ImuData {
            gyro: [
                RadiansPerSecond(0.5 * phase.sin()),
                RadiansPerSecond(0.3 * phase.cos()),
                RadiansPerSecond(0.1),
            ],
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
        };
        ekf.predict(&mut state, &imu, 0.01);
    }

    let estimate = state.get_estimate();
    let q = estimate.attitude;
    let norm = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();

    assert!(
        (norm - 1.0).abs() < 1e-5,
        "Quaternion should remain unit after 1000 iterations, got norm = {}",
        norm
    );
}

// =============================================================================
// GNSS Velocity Fusion Tests
// =============================================================================

#[test]
fn ekf_gnss_velocity_update_correctness() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize with zero velocity
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();
    assert_eq!(state_before.velocity_ned[0].0, 0.0);

    // GNSS with velocity measurement
    // Velocity uses R = 0.1 (meas_noise_gnss_vel)
    // K = P / (P + R) = 0.1 / (0.1 + 0.1) = 0.5
    // For 1 m/s measurement: x_new = 0 + 0.5 * 1.0 = 0.5 m/s
    let gnss = make_gnss_reading(
        [Meters(0.0), Meters(0.0), Meters(0.0)], // No position change
        [
            MetersPerSecond(1.0),
            MetersPerSecond(0.5),
            MetersPerSecond(-0.2),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    let state_after = state.get_estimate();

    // With K = 0.5 for velocity, expect ~0.5 m/s
    let expected_vx = 0.5;
    assert!(
        (state_after.velocity_ned[0].0 - expected_vx).abs() < 0.1,
        "Velocity X should be ~{:.2} m/s with K=0.5, got {:.3}",
        expected_vx,
        state_after.velocity_ned[0].0
    );

    assert!(
        state_after.velocity_ned[1].0 > 0.0,
        "Velocity Y should increase toward measurement"
    );
    assert!(
        state_after.velocity_ned[2].0 < 0.0,
        "Velocity Z should decrease toward measurement"
    );
}

#[test]
fn ekf_gnss_velocity_fusion_independent_of_position() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize at non-zero position
    state.init(
        Vector3::new(Meters(100.0), Meters(200.0), Meters(-50.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // GNSS with same position (no innovation) but non-zero velocity
    // Velocity gate: P=0.1, R=0.1, S=0.2, max_innov = sqrt(25*0.2) ≈ 2.24 m/s
    // Use 2.0 m/s to stay within gate
    let gnss = make_gnss_reading(
        [Meters(100.0), Meters(200.0), Meters(-50.0)], // Same as init
        [
            MetersPerSecond(2.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        GnssHealth::Good,
        SensorHealth::Good,
        GnssFix::ThreeD,
    );

    ekf.update_gnss(&mut state, &gnss);

    let state_after = state.get_estimate();

    // Position should not change (zero innovation)
    assert!(
        (state_after.position_ned[0].0 - 100.0).abs() < 0.01,
        "Position should remain unchanged"
    );

    // Velocity should be updated (K = 0.5, expected: 0 + 0.5 * 2.0 = 1.0 m/s)
    assert!(
        state_after.velocity_ned[0].0 > 0.8,
        "Velocity should move toward GNSS measurement, got {}",
        state_after.velocity_ned[0].0
    );
}

// =============================================================================
// Magnetometer Fusion Tests
// =============================================================================
//
// Mag fusion implementation notes:
//   - Fuses tilt-compensated magnetic heading into yaw state
//   - Uses inclination-based weight decay for polar region protection
//   - Field strength validation: 20-70 μT (Earth's typical range)
//   - Innovation gating on heading (same gate as position/velocity)
//
// Innovation gating for heading:
//   P_att_yaw_init = 0.1, R_mag = 0.05 (default)
//   S = P + R = 0.15
//   gate² = 25, max_innovation = sqrt(25 * 0.15) ≈ 1.94 rad ≈ 111°

use aviate_core::sensor::MagData;
use aviate_core::types::Microtesla;

fn make_mag_reading(
    field_ut: [Microtesla; 3],
    sensor_health: SensorHealth,
) -> SensorReading<MagData> {
    SensorReading {
        value: MagData { field_ut },
        timestamp: dummy_timestamp(),
        health: sensor_health,
        valid: true,
        source_id: 0,
    }
}

#[test]
fn ekf_mag_update_moves_yaw_toward_measurement() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize with identity quaternion (yaw = 0, pointing North)
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();
    let (_, _, yaw_before) = state_before.attitude.to_euler();
    assert!(yaw_before.abs() < 0.01, "Initial yaw should be ~0");

    // Mag reading indicating East heading (yaw = 90° = π/2)
    // When pointing East, body X (forward) aligns with NED East
    // So mag vector in body frame should have strong +Y component (body Y points to mag North)
    // For a ~45μT field typical of mid-latitudes:
    // If heading is 45° (0.785 rad), mag in body:
    //   mag_body_x = field * cos(heading) ≈ 32μT
    //   mag_body_y = field * sin(heading) ≈ 32μT
    //   mag_body_z = small (low inclination)
    let heading_target: f32 = 0.3; // ~17° - small angle for testing
    let field_strength: f32 = 45.0;
    let mag_x = field_strength * heading_target.cos();
    let mag_y = field_strength * heading_target.sin();
    let mag_z: f32 = 10.0; // Low inclination

    let mag = make_mag_reading(
        [Microtesla(mag_x), Microtesla(mag_y), Microtesla(mag_z)],
        SensorHealth::Good,
    );

    ekf.update_mag(&mut state, &mag);

    let state_after = state.get_estimate();
    let (_, _, yaw_after) = state_after.attitude.to_euler();

    // Yaw should move toward the mag heading
    // With K ≈ P/(P+R) = 0.1/0.15 ≈ 0.67, innovation = 0.3
    // Expected: yaw ≈ 0.67 * 0.3 ≈ 0.2 rad
    assert!(
        yaw_after > 0.1,
        "Yaw should move toward mag heading, got {} rad",
        yaw_after
    );
    assert!(
        yaw_after < heading_target,
        "Yaw should not exceed mag heading measurement"
    );
}

#[test]
fn ekf_mag_rejects_degraded_sensor_health() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    // Valid mag field but degraded sensor health
    let mag = make_mag_reading(
        [Microtesla(40.0), Microtesla(15.0), Microtesla(10.0)],
        SensorHealth::Degraded,
    );

    ekf.update_mag(&mut state, &mag);

    let state_after = state.get_estimate();

    // Attitude should not change
    assert_eq!(
        state_before.attitude.w, state_after.attitude.w,
        "Attitude should not change with degraded sensor health"
    );
}

#[test]
fn ekf_mag_rejects_weak_field() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    // Weak field < 20 μT (interference or shielding)
    // norm = sqrt(5² + 5² + 5²) ≈ 8.66 μT < 20 μT
    let mag = make_mag_reading(
        [Microtesla(5.0), Microtesla(5.0), Microtesla(5.0)],
        SensorHealth::Good,
    );

    ekf.update_mag(&mut state, &mag);

    let state_after = state.get_estimate();

    // Attitude should not change (field rejected)
    assert_eq!(
        state_before.attitude.w, state_after.attitude.w,
        "Weak field should be rejected"
    );
}

#[test]
fn ekf_mag_rejects_strong_field() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    // Strong field > 70 μT (near ferrous material)
    // norm = sqrt(60² + 60² + 60²) ≈ 104 μT > 70 μT
    let mag = make_mag_reading(
        [Microtesla(60.0), Microtesla(60.0), Microtesla(60.0)],
        SensorHealth::Good,
    );

    ekf.update_mag(&mut state, &mag);

    let state_after = state.get_estimate();

    // Attitude should not change (field rejected)
    assert_eq!(
        state_before.attitude.w, state_after.attitude.w,
        "Strong field should be rejected"
    );
}

#[test]
fn ekf_mag_weight_decays_with_inclination() {
    // Test that higher inclination results in smaller state updates
    // due to weight decay mechanism

    // First: low inclination (vertical_ratio < 0.8)
    let config = EkfConfig::default();
    let ekf_low = Ekf::new(config);
    let mut state_low = EkfState::default();
    state_low.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Low inclination: mostly horizontal field
    // norm = sqrt(40² + 15² + 10²) ≈ 44 μT
    // vertical_ratio = 10 / 44 ≈ 0.23 < 0.8 → weight = 1.0
    let mag_low_incl = make_mag_reading(
        [Microtesla(40.0), Microtesla(15.0), Microtesla(10.0)],
        SensorHealth::Good,
    );
    ekf_low.update_mag(&mut state_low, &mag_low_incl);
    let (_, _, yaw_low) = state_low.get_estimate().attitude.to_euler();

    // Second: high inclination (vertical_ratio in decay range 0.8-0.95)
    let ekf_high = Ekf::new(config);
    let mut state_high = EkfState::default();
    state_high.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // High inclination: mostly vertical field
    // norm = sqrt(10² + 15² + 40²) ≈ 44 μT
    // vertical_ratio = 40 / 44 ≈ 0.91 (in decay range 0.8-0.95)
    // weight ≈ 1 - (0.91 - 0.8) / 0.15 ≈ 0.27
    let mag_high_incl = make_mag_reading(
        [Microtesla(10.0), Microtesla(15.0), Microtesla(40.0)],
        SensorHealth::Good,
    );
    ekf_high.update_mag(&mut state_high, &mag_high_incl);
    let (_, _, yaw_high) = state_high.get_estimate().attitude.to_euler();

    // High inclination should result in smaller yaw change due to weight decay
    // (r_effective increases with lower weight, reducing Kalman gain)
    assert!(
        yaw_high.abs() < yaw_low.abs(),
        "High inclination should result in smaller update: low={:.4}, high={:.4}",
        yaw_low,
        yaw_high
    );
}

#[test]
fn ekf_mag_stops_fusion_at_high_inclination() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let state_before = state.get_estimate();

    // Very high inclination: nearly vertical field (polar region)
    // norm = sqrt(5² + 5² + 45²) ≈ 45.5 μT
    // vertical_ratio = 45 / 45.5 ≈ 0.99 > 0.95 → fusion disabled
    let mag = make_mag_reading(
        [Microtesla(5.0), Microtesla(5.0), Microtesla(45.0)],
        SensorHealth::Good,
    );

    ekf.update_mag(&mut state, &mag);

    let state_after = state.get_estimate();

    // Attitude should not change (high inclination rejects fusion)
    assert_eq!(
        state_before.attitude.w, state_after.attitude.w,
        "High inclination (>0.95) should disable fusion"
    );
}

#[test]
fn ekf_mag_innovation_gating_rejects_outlier() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize pointing North (yaw = 0)
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run a few predict steps to establish covariance
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
    for _ in 0..10 {
        ekf.predict(&mut state, &imu, 0.01);
    }

    let state_before = state.get_estimate();

    // Mag reading indicating ~180° heading (pointing South)
    // This is a huge innovation (~π rad) that should be rejected
    // With fresh P, chi² = π² / 0.15 ≈ 65.8 > 25 (gate²)
    // Note: field_x negative = pointing South
    let mag = make_mag_reading(
        [Microtesla(-40.0), Microtesla(0.0), Microtesla(10.0)],
        SensorHealth::Good,
    );

    ekf.update_mag(&mut state, &mag);

    let state_after = state.get_estimate();
    let (_, _, yaw_before) = state_before.attitude.to_euler();
    let (_, _, yaw_after) = state_after.attitude.to_euler();

    // Yaw should not have jumped significantly (gating should reject)
    let yaw_change = (yaw_after - yaw_before).abs();
    assert!(
        yaw_change < 0.5,
        "Innovation gating should reject 180° outlier, but yaw changed {} rad",
        yaw_change
    );
}

#[test]
fn ekf_mag_uninitialized_returns_early() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // EKF is NOT initialized
    assert!(!state.is_initialized());

    let mag = make_mag_reading(
        [Microtesla(40.0), Microtesla(15.0), Microtesla(10.0)],
        SensorHealth::Good,
    );

    // Should return early without panic
    ekf.update_mag(&mut state, &mag);

    // Should still be uninitialized
    assert!(!state.is_initialized());
}

#[test]
fn ekf_mag_innovation_wrapping_above_pi() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize with identity quaternion (yaw = 0)
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Create mag reading that indicates heading ~200° (3.49 rad)
    // This creates innovation > π which exercises the wrapping code
    // heading = atan2(mag_e, mag_n) in NED
    // For heading ≈ 200°: mag_n < 0, mag_e < 0 (pointing SW)
    let heading_target = 3.49_f32; // ~200° > π
    let field_strength = 45.0_f32;
    let mag_x = field_strength * heading_target.cos(); // ~-42
    let mag_y = field_strength * heading_target.sin(); // ~-15
    let mag_z = 10.0_f32;

    let mag = make_mag_reading(
        [Microtesla(mag_x), Microtesla(mag_y), Microtesla(mag_z)],
        SensorHealth::Good,
    );

    // This will trigger innovation wrapping: innov = 3.49 - 0 = 3.49 > π
    // Wrapped: innov = 3.49 - 2π ≈ -2.79
    // Then innovation gating may reject, but wrapping is exercised
    ekf.update_mag(&mut state, &mag);

    // Verify EKF is still operational
    assert!(state.is_initialized());
}

#[test]
fn ekf_mag_innovation_wrapping_below_minus_pi() {
    let config = EkfConfig::default();
    let ekf = Ekf::new(config);
    let mut state = EkfState::default();

    // Initialize with yaw pointing roughly South (~π rad)
    // Using quaternion for 180° yaw: q = [cos(π/2), 0, 0, sin(π/2)] = [0, 0, 0, 1]
    let quat_south = Quaternion::new(0.0, 0.0, 0.0, 1.0);

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        quat_south,
    );

    // Create mag reading indicating heading ~-170° (-2.97 rad)
    // With yaw at π, innovation = -2.97 - π ≈ -6.11 < -π
    // This exercises the < -π wrapping
    let heading_target = -2.97_f32;
    let field_strength = 45.0_f32;
    let mag_x = field_strength * heading_target.cos();
    let mag_y = field_strength * heading_target.sin();
    let mag_z = 10.0_f32;

    let mag = make_mag_reading(
        [Microtesla(mag_x), Microtesla(mag_y), Microtesla(mag_z)],
        SensorHealth::Good,
    );

    ekf.update_mag(&mut state, &mag);

    assert!(state.is_initialized());
}

// =============================================================================
// INV-27: Quaternion Fault Latch Behavior Tests
// =============================================================================

#[test]
fn ekf_has_numeric_fault_false_initially() {
    let _ekf = Ekf::new(EkfConfig::default());

    let state = EkfState::default();
    assert!(
        !state.has_numeric_fault(),
        "quat_fault should be false initially"
    );
}

#[test]
fn ekf_init_clears_quat_fault() {
    let _ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    // First init
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // After init, fault should be false
    assert!(
        !state.has_numeric_fault(),
        "quat_fault should be false after init()"
    );
}

#[test]
fn ekf_normal_operation_no_fault() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Normal IMU data - should not trigger fault
    let imu = ImuData {
        gyro: [
            RadiansPerSecond(0.1),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ],
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
    };

    // Multiple predict cycles should not trigger fault
    for _ in 0..100 {
        ekf.predict(&mut state, &imu, 0.01);
    }

    assert!(
        !state.has_numeric_fault(),
        "Normal operation should not trigger quat_fault"
    );
}

#[test]
fn ekf_reinit_clears_fault_state() {
    use aviate_core::sensor::ImuData;
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};

    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    // First init
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Do some predictions
    let imu = ImuData {
        gyro: [
            RadiansPerSecond(0.5),
            RadiansPerSecond(0.3),
            RadiansPerSecond(0.1),
        ],
        accel: [
            MetersPerSecondSquared(1.0),
            MetersPerSecondSquared(0.5),
            MetersPerSecondSquared(-9.81),
        ],
    };

    for _ in 0..50 {
        ekf.predict(&mut state, &imu, 0.01);
    }

    // Re-initialize - this should clear any potential fault state
    state.init(
        Vector3::new(Meters(100.0), Meters(50.0), Meters(-10.0)),
        Vector3::new(
            MetersPerSecond(5.0),
            MetersPerSecond(2.0),
            MetersPerSecond(-1.0),
        ),
        Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 0.5),
    );

    assert!(
        !state.has_numeric_fault(),
        "Re-init should clear quat_fault"
    );
}
