//! LLR-EST-204(b): a cycle with a non-finite IMU sample or `dt` leaves
//! the filter untouched.
//!
//! Without a usable inertial sample the cycle has no prediction to
//! correct, and an aiding update fused onto the un-propagated state would
//! still move the position, velocity, attitude, biases and covariance
//! the requirement freezes. The IMU-gap widening on the next usable
//! sample is what accounts for the time such cycles leave uncovered.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator, STATE_DIM};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::{
    GnssData, GnssFix, GnssHealth, ImuData, SensorHealth, SensorReading, SensorSet,
};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond};

const DT: f32 = 0.01;
const G: f32 = 9.81;

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

/// A level IMU reading `accel_z` and a GNSS fix at `pos` and `vel`.
fn sensors(accel_z: f32, pos: [f32; 3], vel: [f32; 3]) -> SensorSet {
    let imu = reading(ImuData {
        gyro: [RadiansPerSecond(0.0); 3],
        accel: [0.0, 0.0, accel_z].map(MetersPerSecondSquared),
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

/// A filter that has hovered at rest at 10 m on GNSS for 10 s.
fn hovering(ekf: &Ekf) -> EkfState {
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
    let hover = sensors(-G, [0.0, 0.0, -10.0], [0.0; 3]);
    for _ in 0..1_000 {
        ekf.observe(&mut state, &hover, None, DT);
    }
    state
}

/// Every quantity LLR-EST-204(b) freezes, bit for bit.
fn frozen_quantities(state: &EkfState) -> Vec<u32> {
    let q = state.quat;
    let mut bits: Vec<u32> = [q.w, q.x, q.y, q.z]
        .into_iter()
        .chain([state.pos.x.0, state.pos.y.0, state.pos.z.0])
        .chain([state.vel.x.0, state.vel.y.0, state.vel.z.0])
        .chain([
            state.gyro_bias.x.0,
            state.gyro_bias.y.0,
            state.gyro_bias.z.0,
        ])
        .chain([
            state.accel_bias.x.0,
            state.accel_bias.y.0,
            state.accel_bias.z.0,
        ])
        .map(f32::to_bits)
        .collect();
    for r in 0..STATE_DIM {
        for c in 0..STATE_DIM {
            bits.push(state.p_cov.get(r, c).to_bits());
        }
    }
    bits
}

/// A GNSS fix 1 m east of the estimate, moving north at 0.5 m/s: inside
/// the gate, so fusing it would move the state.
const OFFSET_POS: [f32; 3] = [0.0, 1.0, -10.0];
const OFFSET_VEL: [f32; 3] = [0.5, 0.0, 0.0];

#[test]
fn a_non_finite_imu_cycle_fuses_no_aiding() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = hovering(&ekf);
    let before = frozen_quantities(&state);

    ekf.observe(
        &mut state,
        &sensors(f32::NAN, OFFSET_POS, OFFSET_VEL),
        None,
        DT,
    );

    assert!(
        frozen_quantities(&state) == before,
        "a non-finite IMU cycle changed the filter"
    );
}

#[test]
fn a_non_finite_dt_cycle_fuses_no_aiding() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = hovering(&ekf);
    let before = frozen_quantities(&state);

    ekf.observe(
        &mut state,
        &sensors(-G, OFFSET_POS, OFFSET_VEL),
        None,
        f32::NAN,
    );

    assert!(
        frozen_quantities(&state) == before,
        "a non-finite dt cycle changed the filter"
    );
}

/// The same GNSS fix on a cycle with a usable IMU sample does move the
/// filter, so the two tests above witness the gate, not a fix the gate
/// would have rejected anyway.
#[test]
fn the_same_fix_moves_the_filter_on_a_usable_cycle() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = hovering(&ekf);
    let before = frozen_quantities(&state);

    ekf.observe(&mut state, &sensors(-G, OFFSET_POS, OFFSET_VEL), None, DT);

    assert!(frozen_quantities(&state) != before);
}
