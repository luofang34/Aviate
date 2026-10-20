//! An IMU outage must leave the filter less certain, not frozen.
//!
//! While the IMU delivers non-finite samples the vehicle keeps moving and
//! turning, unmeasured. A filter that skips those cycles entirely keeps
//! its pre-outage confidence, so when the IMU returns, the GNSS evidence
//! of where the vehicle went sits outside the innovation gate and the
//! estimate never recovers. The outage has to grow the uncertainty the
//! missing measurements would have bounded.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{
    GnssData, GnssFix, GnssHealth, ImuData, SensorHealth, SensorReading, SensorSet,
};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond};

const DT: f32 = 0.01;
const G: f32 = 9.81;
/// The roll the vehicle takes, unmeasured, while the IMU is out.
const ROLL_RAD: f32 = 30.0 * core::f32::consts::PI / 180.0;

fn reading<T>(value: T) -> SensorReading<T> {
    SensorReading {
        value,
        timestamp: Timestamp {
            ticks: 0,
            source: TimeSource::Internal,
        },
        health: SensorHealth::Good,
        valid: true,
        source_id: 0,
    }
}

/// The sensors of a vehicle at truth position and velocity (NED), its
/// IMU stating `accel` (body FRD specific force) and zero rotation rate.
fn sensors(pos: [f32; 3], vel: [f32; 3], accel: [f32; 3]) -> SensorSet {
    let imu = reading(ImuData {
        gyro: [RadiansPerSecond(0.0); 3],
        accel: accel.map(MetersPerSecondSquared),
    });
    let gnss = reading(GnssData {
        position_ned: pos.map(Meters),
        velocity_ned: vel.map(MetersPerSecond),
        fix: GnssFix::ThreeD,
        health: GnssHealth::Good,
    });
    SensorSet {
        imus: [imu, SensorReading::default(), SensorReading::default()],
        gnss: [gnss, SensorReading::default()],
        mags: [SensorReading::default(), SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
        geomagnetic: None,
    }
}

fn roll_of(q: Quaternion) -> f32 {
    (2.0 * (q.w * q.x + q.y * q.z)).atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y))
}

/// Hover level for 10 s, lose the IMU for 3 s while the vehicle rolls
/// 30° and starts accelerating east under hover thrust, then get the
/// IMU back. Within 5 s of its return the estimate has recovered the
/// roll and the east velocity.
#[test]
fn the_estimate_recovers_after_a_three_second_imu_outage() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(-10.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    let hover = sensors([0.0, 0.0, -10.0], [0.0; 3], [0.0, 0.0, -G]);
    for _ in 0..1_000 {
        ekf.observe(&mut state, &hover, None, DT);
    }

    // Rolled, the hover thrust holds altitude and accelerates the
    // vehicle east at g·tan(roll).
    let east_accel = G * ROLL_RAD.tan();
    let rolled_force = [0.0, 0.0, -G / ROLL_RAD.cos()];
    let truth = |t: f32| {
        (
            [0.0, 0.5 * east_accel * t * t, -10.0],
            [0.0, east_accel * t, 0.0],
        )
    };

    let mut t = 0.0;
    for _ in 0..300 {
        t += DT;
        let (pos, vel) = truth(t);
        ekf.observe(&mut state, &sensors(pos, vel, [f32::NAN; 3]), None, DT);
    }
    for _ in 0..500 {
        t += DT;
        let (pos, vel) = truth(t);
        ekf.observe(&mut state, &sensors(pos, vel, rolled_force), None, DT);
    }

    let (_, vel) = truth(t);
    let roll_deg = roll_of(state.quat).to_degrees();
    assert!(
        (roll_deg - ROLL_RAD.to_degrees()).abs() < 5.0,
        "roll estimate {roll_deg:.1}° after the outage, truth 30°"
    );
    assert!(
        (state.vel.y.0 - vel[1]).abs() < 1.0,
        "east velocity {:.2} m/s, truth {:.2} m/s",
        state.vel.y.0,
        vel[1]
    );
}
