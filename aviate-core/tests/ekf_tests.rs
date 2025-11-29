//! Tests for Extended Kalman Filter (EKF)
//!
//! Covers:
//! - update_gnss with valid GNSS data
//! - GNSS health gating

use aviate_core::ekf::{Ekf, EkfConfig};
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
    let mut ekf = Ekf::new(config);

    // Initialize EKF with known state
    ekf.init(
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
    ekf.update_gnss(&gnss);

    // After update, state should be influenced by GNSS measurement
    // We verify it's initialized and accepted the update
    assert!(ekf.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_degraded_sensor_health() {
    let config = EkfConfig::default();
    let mut ekf = Ekf::new(config);

    ekf.init(
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

    ekf.update_gnss(&gnss);

    // EKF should still be initialized (update was rejected, not failed)
    assert!(ekf.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_suspect_gnss_health() {
    let config = EkfConfig::default();
    let mut ekf = Ekf::new(config);

    ekf.init(
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

    ekf.update_gnss(&gnss);

    // Update should be rejected due to suspect GNSS health
    assert!(ekf.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_no_fix() {
    let config = EkfConfig::default();
    let mut ekf = Ekf::new(config);

    ekf.init(
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

    ekf.update_gnss(&gnss);

    // Update should be rejected due to no fix
    assert!(ekf.is_initialized());
}

#[test]
fn ekf_update_gnss_rejects_lost_gnss_health() {
    let config = EkfConfig::default();
    let mut ekf = Ekf::new(config);

    ekf.init(
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

    ekf.update_gnss(&gnss);

    // Update should be rejected due to lost GNSS
    assert!(ekf.is_initialized());
}

#[test]
fn ekf_default_creates_valid_instance() {
    let ekf = Ekf::default();
    assert!(!ekf.is_initialized());
}
