//! A clipped accelerometer must leave the filter less certain, not wrong.
//!
//! A hard touchdown decelerates the vehicle at hundreds of g for a few
//! milliseconds, far past any accelerometer's range. The clipped samples
//! integrate only part of the impulse, so the velocity estimate keeps
//! most of the fall speed while the vehicle sits on the ground. A filter
//! that trusts those samples keeps its pre-impact confidence, every GNSS
//! velocity of zero then sits outside the innovation gate, and the land
//! detector never sees the vehicle stop.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{
    GnssData, GnssFix, GnssHealth, ImuData, SensorHealth, SensorReading, SensorSet,
};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond};

/// The SITL kernel step.
const DT: f32 = 0.004;
const G: f32 = 9.81;
/// GNSS arrives every 25th step (10 Hz).
const GNSS_EVERY: u32 = 25;
/// The accelerometer's clamp: the i16 milli-g SITL truth saturates here.
const CLAMP_MPS2: f32 = 321.3;
/// Height the vehicle drops from, so it lands at about 17 m/s.
const DROP_M: f32 = 15.0;

fn reading<T>(value: T, valid: bool) -> SensorReading<T> {
    SensorReading {
        value,
        timestamp: Timestamp {
            ticks: 0,
            source: TimeSource::Internal,
        },
        health: SensorHealth::Good,
        valid,
        source_id: 0,
    }
}

/// One step's sensors: the IMU reading `accel_z` (body FRD, level, no
/// rotation) and, on GNSS steps, the truth height and vertical speed.
fn sensors(step: u32, z: f32, vz: f32, accel_z: f32) -> SensorSet {
    let imu = reading(
        ImuData {
            gyro: [RadiansPerSecond(0.0); 3],
            accel: [0.0, 0.0, accel_z].map(MetersPerSecondSquared),
        },
        true,
    );
    let gnss = reading(
        GnssData {
            position_ned: [0.0, 0.0, z].map(Meters),
            velocity_ned: [0.0, 0.0, vz].map(MetersPerSecond),
            fix: GnssFix::ThreeD,
            health: GnssHealth::Good,
        },
        step.is_multiple_of(GNSS_EVERY),
    );
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

/// Hover for 10 s, fall 15 m on zero thrust, stop on the ground within
/// four samples clipped at the SITL clamp, then rest. Within 2 s of the
/// touchdown the estimate is at rest on the ground, as the land detector
/// needs it to be.
#[test]
fn the_estimate_comes_to_rest_after_a_clipped_hard_touchdown() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(-DROP_M)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    let mut step = 0u32;
    let mut observe = |state: &mut EkfState, z: f32, vz: f32, accel_z: f32| {
        ekf.observe(state, &sensors(step, z, vz, accel_z), None, DT);
        step = step.wrapping_add(1);
    };

    for _ in 0..2_500 {
        observe(&mut state, -DROP_M, 0.0, -G);
    }
    let mut t = 0.0;
    loop {
        t += DT;
        let z = -DROP_M + 0.5 * G * t * t;
        if z >= 0.0 {
            break;
        }
        observe(&mut state, z, G * t, 0.0);
    }
    for _ in 0..4 {
        observe(&mut state, 0.0, 0.0, -CLAMP_MPS2);
    }
    for _ in 0..500 {
        observe(&mut state, 0.0, 0.0, -G);
    }

    assert!(
        state.vel.z.0.abs() < 0.3,
        "vertical velocity {:.2} m/s on the ground",
        state.vel.z.0
    );
    assert!(
        state.pos.z.0.abs() < 0.3,
        "height error {:.2} m on the ground",
        state.pos.z.0
    );
}
