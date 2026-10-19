//! The estimator's heading is referenced to true north.
//!
//! A magnetometer measures magnetic heading. Where magnetic north lies
//! `D` east of true north, a filter that fuses that heading as yaw is off
//! by `D` — about 12° in parts of the world at 51° N. Every case below
//! drives the real `Estimator::observe` path with a stationary IMU and a
//! physically consistent magnetometer reading, and reads the converged
//! yaw.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{
    GeomagneticReference, GeomagneticSource, ImuData, MagData, SensorHealth, SensorReading,
    SensorSet,
};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Radians, RadiansPerSecond,
};

/// A 51° N field: declination 12° east, inclination 66°, 50 µT.
const DECLINATION_DEG: f32 = 12.0;
const INCLINATION_DEG: f32 = 66.0;
const INTENSITY_UT: f32 = 50.0;

/// The vehicle's true heading in every case.
const TRUE_YAW_DEG: f32 = 30.0;

/// Converged-heading tolerance.
const TOL_DEG: f32 = 0.5;

fn timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn reading<T>(value: T) -> SensorReading<T> {
    SensorReading {
        value,
        timestamp: timestamp(),
        health: SensorHealth::Good,
        valid: true,
        source_id: 0,
    }
}

/// The earth field in NED, µT.
fn field_ned() -> [f32; 3] {
    let (d, i) = (DECLINATION_DEG.to_radians(), INCLINATION_DEG.to_radians());
    let h = INTENSITY_UT * i.cos();
    [h * d.cos(), h * d.sin(), INTENSITY_UT * i.sin()]
}

/// The body-frame field a level vehicle at true heading `yaw` measures.
fn mag_body(yaw: f32) -> [Microtesla; 3] {
    let q = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), yaw);
    let r = q.to_rotation_matrix();
    let f = field_ned();
    let b = |col: usize| r.get(0, col) * f[0] + r.get(1, col) * f[1] + r.get(2, col) * f[2];
    [Microtesla(b(0)), Microtesla(b(1)), Microtesla(b(2))]
}

fn stationary_sensors(geomagnetic: Option<GeomagneticReference>) -> SensorSet {
    let imu = reading(ImuData {
        accel: [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(-9.81),
        ],
        gyro: [RadiansPerSecond(0.0); 3],
    });
    let mag = reading(MagData {
        field_ut: mag_body(TRUE_YAW_DEG.to_radians()),
    });
    SensorSet {
        imus: [imu, SensorReading::default(), SensorReading::default()],
        gnss: [SensorReading::default(), SensorReading::default()],
        mags: [mag, SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
        geomagnetic,
    }
}

/// Yaw, in degrees, after the filter has settled on `sensors`.
fn converged_yaw_deg(sensors: &SensorSet) -> f32 {
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
    // 20 s of 400 Hz cycles.
    for _ in 0..8_000 {
        ekf.observe(&mut state, sensors, None, 0.0025);
    }
    let (_, _, yaw) = ekf.estimate(&state).attitude.to_euler();
    yaw.to_degrees()
}

fn reference(source: GeomagneticSource) -> GeomagneticReference {
    GeomagneticReference {
        declination: Radians(DECLINATION_DEG.to_radians()),
        source,
    }
}

#[test]
fn a_supplied_declination_makes_yaw_true_heading() {
    for source in [GeomagneticSource::Model, GeomagneticSource::Configured] {
        let yaw = converged_yaw_deg(&stationary_sensors(Some(reference(source))));
        assert!(
            (yaw - TRUE_YAW_DEG).abs() <= TOL_DEG,
            "{source:?}: yaw {yaw:.2}° is not the true heading {TRUE_YAW_DEG}°"
        );
    }
}

#[test]
fn without_a_reference_yaw_stays_magnetic() {
    let yaw = converged_yaw_deg(&stationary_sensors(None));
    let magnetic = TRUE_YAW_DEG - DECLINATION_DEG;
    assert!(
        (yaw - magnetic).abs() <= TOL_DEG,
        "yaw {yaw:.2}° is not the magnetic heading {magnetic}°"
    );
}
