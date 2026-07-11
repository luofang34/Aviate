//! #135 guardrails: elapsed time is never a reason to trust a source.
//!
//! PR #132 briefly carried an age-triggered reset-to-measurement:
//! once a channel's aiding age went stale, the whole block snapped to
//! the current reading. That let a previously aided but persistently
//! bad GNSS source (step fault, multipath jump, well-formed spoof —
//! all still marked `Good`) wait out the timer and capture the state
//! past the innovation gate. The branch is removed; these tests pin
//! the properties that must survive any future recovery design (which
//! per #135 requires an independently qualified trusted reference and
//! the #81 reset events).

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{
    GnssData, GnssFix, GnssHealth, ImuData, SensorHealth, SensorReading, SensorSet,
};
use aviate_core::state::StateValidFlags;
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

fn converged_state(ekf: &Ekf) -> EkfState {
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
    assert!(state.gnss_vel_age_s < 0.1, "converged and freshly aided");
    state
}

/// #135 acceptance: valid aiding first, then a persistent large but
/// finite jump still flagged `Good`. The bad source must NEVER
/// capture the state merely because time elapsed — and the honest
/// consequence of rejecting it is that velocity validity drops, so
/// the failsafe path (not a silent state capture) owns the outcome.
#[test]
fn previously_aided_bad_source_never_captures_the_state() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = converged_state(&ekf);

    // The source jumps to a persistent 40 m/s vertical velocity while
    // staying healthy and well-formed. 10 s of insistence.
    let bad = resting_sensors(40.0);
    for _ in 0..1_000 {
        ekf.observe(&mut state, &bad, None, 0.01);
    }
    assert!(
        state.vel.z.0.abs() < 1.0,
        "persistent bad source must not capture vz, got {}",
        state.vel.z.0
    );
    let est = ekf.estimate(&state);
    assert!(
        !est.valid_flags.contains(StateValidFlags::VELOCITY),
        "rejected aiding must surface as lost validity, not silence"
    );
}

/// An isolated outlier is rejected and stays rejected — one bad
/// sample among good ones triggers no recovery behavior.
#[test]
fn isolated_outlier_is_rejected_without_side_effects() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = converged_state(&ekf);

    ekf.update_gnss_state(&mut state, &gnss(35.0));
    assert!(
        state.vel.z.0.abs() < 0.5,
        "outlier must not move the state, got vz {}",
        state.vel.z.0
    );

    // Good aiding continues unharmed.
    let sensors = resting_sensors(0.0);
    for _ in 0..20 {
        ekf.observe(&mut state, &sensors, None, 0.01);
    }
    assert!(state.gnss_vel_age_s < 0.1, "good aiding still fuses");
}

/// A never-aided filter keeps the ordinary outlier gate: the first
/// reading cannot capture the state wholesale either.
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

    ekf.update_gnss_state(&mut state, &gnss(50.0));
    assert!(
        state.vel.z.0.abs() < 1.0,
        "outlier must not capture a never-aided filter, vz = {}",
        state.vel.z.0
    );
}
