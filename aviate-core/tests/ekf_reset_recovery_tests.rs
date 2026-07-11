//! Reset-to-measurement recovery (#80): a diverged state block whose
//! innovations the gate keeps rejecting must snap back to a healthy
//! measurement once its aiding age goes stale, instead of being
//! rejected forever. Without this, a braked climb that outruns the
//! vertical-velocity estimate leaves the filter permanently pinned
//! (collapsed covariance → tight gate → every innovation rejected)
//! and the controller flies the phantom velocity into the ground.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{
    GnssData, GnssFix, GnssHealth, ImuData, SensorHealth, SensorReading, SensorSet,
};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond};

fn gnss(vel_d: f32) -> SensorReading<GnssData> {
    SensorReading {
        value: GnssData {
            position_ned: [Meters(0.0), Meters(0.0), Meters(0.0)],
            velocity_ned: [
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(vel_d),
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

fn resting_sensors(vel_d: f32) -> SensorSet {
    let imu = SensorReading {
        value: ImuData {
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
        },
        timestamp: Timestamp {
            ticks: 0,
            source: TimeSource::Internal,
        },
        health: SensorHealth::Good,
        valid: true,
        source_id: 0,
    };
    SensorSet {
        imus: [imu, SensorReading::default(), SensorReading::default()],
        gnss: [gnss(vel_d), SensorReading::default()],
        mags: [SensorReading::default(), SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    }
}

/// Diverge vz far past the gate with a collapsed variance, then feed
/// healthy GNSS through `observe` for just over the reset age: the
/// velocity block must snap to the measurement and validity recover.
#[test]
fn stale_rejected_velocity_snaps_back_to_gnss() {
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

    // Converge once so the aiding ages leave their "never fused" seed.
    let sensors = resting_sensors(0.0);
    for _ in 0..50 {
        ekf.observe(&mut state, &sensors, None, 0.01);
    }
    assert!(state.gnss_vel_age_s < 0.1, "converged and freshly aided");

    // Force the divergence shape: vz pinned far from truth with a
    // collapsed variance so the 5σ gate rejects the ~6 m/s innovation
    // (s ≈ P_vv + 0.1; even P_vv = 0.01 needs |innov| < ~1.7).
    state.vel.z = MetersPerSecond(-6.0);
    for r in 0..15 {
        state.p_cov.set(r, 5, 0.0);
        state.p_cov.set(5, r, 0.0);
    }
    state.p_cov.set(5, 5, 0.01);

    // Under the reset age the gate must still reject — the estimate
    // stays wrong and the aiding age climbs.
    for _ in 0..50 {
        ekf.observe(&mut state, &sensors, None, 0.01);
    }
    assert!(
        state.vel.z.0 < -5.0,
        "gate still rejecting inside the reset window, vz = {}",
        state.vel.z.0
    );

    // Past the reset age the healthy measurement must win.
    for _ in 0..60 {
        ekf.observe(&mut state, &sensors, None, 0.01);
    }
    assert!(
        state.vel.z.0.abs() < 0.5,
        "velocity block must snap back to the GNSS measurement, vz = {}",
        state.vel.z.0
    );
    assert!(state.gnss_vel_age_s < 0.5, "aiding age recovered");
}

/// The recovery must not weaken the outlier gate for a filter that
/// has never been aided: the first reading after init still fuses
/// through the ordinary gate, so a wild outlier cannot capture the
/// state wholesale.
#[test]
fn never_aided_filter_keeps_the_outlier_gate() {
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

    // First-ever reading is a 50 m/s outlier: must be gate-rejected,
    // not snapped to.
    ekf.update_gnss_state(&mut state, &gnss(50.0));
    assert!(
        state.vel.z.0.abs() < 1.0,
        "outlier must not capture a never-aided filter, vz = {}",
        state.vel.z.0
    );
}

/// Same recovery for the position block: a pinned, gate-rejected
/// position must snap back to healthy GNSS once its aiding age goes
/// stale.
#[test]
fn stale_rejected_position_snaps_back_to_gnss() {
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

    let sensors = resting_sensors(0.0);
    for _ in 0..50 {
        ekf.observe(&mut state, &sensors, None, 0.01);
    }
    assert!(state.gnss_pos_age_s < 0.1, "converged and freshly aided");

    // Pin pos.z 40 m off with a collapsed variance: innovation 40,
    // s ≈ 0.01 + 0.5 → hopelessly outside the 5σ gate.
    state.pos.z = Meters(-40.0);
    for r in 0..15 {
        state.p_cov.set(r, 2, 0.0);
        state.p_cov.set(2, r, 0.0);
    }
    state.p_cov.set(2, 2, 0.01);

    for _ in 0..120 {
        ekf.observe(&mut state, &sensors, None, 0.01);
    }
    assert!(
        state.pos.z.0.abs() < 1.0,
        "position block must snap back to the GNSS measurement, pos.z = {}",
        state.pos.z.0
    );
    assert!(state.gnss_pos_age_s < 0.5, "aiding age recovered");
}
