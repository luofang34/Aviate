//! A rebase changes coordinates, not the estimate: every measurement
//! sees the same innovation and the same innovation variance before and
//! after, and the inverse shift returns the original state.

use super::{FrameShift, NavOrigin};
use crate::ekf::air_aiding::{airspeed_measurement, sideslip_measurement, AirAidingConfig};
use crate::ekf::joint::JointMeasurement;
use crate::ekf::{EkfState, IDX_POS, STATE_DIM};
use crate::math::{Quaternion, Vector3};
use crate::types::{Meters, MetersPerSecond, Scalar};

/// Rotation from local north-east-down at `(lat, lon)` to Earth-fixed axes.
fn ned_to_ecef(latitude_deg: f64, longitude_deg: f64) -> [[f64; 3]; 3] {
    let (sp, cp) = latitude_deg.to_radians().sin_cos();
    let (sl, cl) = longitude_deg.to_radians().sin_cos();
    [
        [-sp * cl, -sl, -cp * cl],
        [-sp * sl, cl, -cp * sl],
        [cp, 0.0, -sp],
    ]
}

/// The shift between two tangent planes 1 km apart at mid latitude.
fn shift() -> FrameShift {
    let old = ned_to_ecef(39.1582, -75.5244);
    let new = ned_to_ecef(39.1672, -75.5144);
    let mut rotation = [[0.0; 3]; 3];
    for (row, out) in rotation.iter_mut().enumerate() {
        for (column, value) in out.iter_mut().enumerate() {
            *value = (0..3).map(|k| new[k][row] * old[k][column]).sum();
        }
    }
    FrameShift {
        rotation,
        offset_m: [1_000.0, 862.0, 0.13],
        origin: NavOrigin {
            latitude_deg: 39.1672,
            longitude_deg: -75.5144,
        },
    }
}

fn inverse(shift: &FrameShift) -> FrameShift {
    let r = shift.rotation;
    let transposed = [
        [r[0][0], r[1][0], r[2][0]],
        [r[0][1], r[1][1], r[2][1]],
        [r[0][2], r[1][2], r[2][2]],
    ];
    let [x, y, z] = shift.direction(shift.offset_m);
    FrameShift {
        rotation: transposed,
        offset_m: [-x, -y, -z],
        origin: shift.origin,
    }
}

fn config() -> AirAidingConfig {
    AirAidingConfig {
        meas_noise_tas: 1.0,
        sideslip_sigma: 0.05,
        wind_prior_var: 64.0,
        process_noise_wind: 0.05,
        min_true_airspeed: 15.0,
    }
}

/// A banked vehicle 1.1 km from the origin with a correlated covariance,
/// a wind and a barometer datum.
fn state() -> EkfState {
    let mut state = EkfState::new();
    state.init(
        Vector3::new(Meters(1_020.0), Meters(870.0), Meters(-300.0)),
        Vector3::new(
            MetersPerSecond(40.0),
            MetersPerSecond(35.0),
            MetersPerSecond(-1.0),
        ),
        Quaternion::from_axis_angle(Vector3::new(0.6, 0.0, 0.8), 0.9).normalize(),
    );
    for i in 0..STATE_DIM {
        for j in 0..STATE_DIM {
            let coupling = 0.01 * ((i * 7 + j * 3) % 5) as Scalar;
            let value = if i == j {
                0.5 + 0.1 * i as Scalar
            } else {
                coupling * 0.01
            };
            state.p_cov.set(i, j, value);
        }
    }
    state.p_cov.make_symmetric();
    state.air = crate::ekf::air_aiding::AirAidingState::activated(9.0);
    state.air.wind_ne = [4.0, -3.0];
    state.air.p_xw[3] = [2.0, 0.3];
    state.air.p_xw[4] = [0.2, 2.5];
    state.baro_ref = Some(317.0);
    state
}

/// Innovation variance of a joint measurement on `state`.
fn variance(state: &EkfState, m: &JointMeasurement) -> Scalar {
    let mut total = m.noise;
    for i in 0..STATE_DIM {
        for j in 0..STATE_DIM {
            total += m.h_x[i] * state.p_cov.get(i, j) * m.h_x[j];
        }
        for w in 0..2 {
            total += 2.0 * m.h_x[i] * state.air.p_xw[i][w] * m.h_w[w];
        }
    }
    for a in 0..2 {
        for b in 0..2 {
            total += m.h_w[a] * state.air.p_ww[a][b] * m.h_w[b];
        }
    }
    total
}

fn close(a: Scalar, b: Scalar, tolerance: Scalar) -> bool {
    (a - b).abs() <= tolerance * a.abs().max(b.abs()).max(1.0)
}

#[test]
fn the_rotation_quaternion_is_the_rotation() {
    let shift = shift();
    let matrix = shift.rotation_quaternion().to_rotation_matrix();
    for (row, expected) in shift.rotation.iter().enumerate() {
        for (column, value) in expected.iter().enumerate() {
            assert!((f64::from(matrix.get(row, column)) - value).abs() < 1e-6);
        }
    }
}

#[test]
fn air_measurements_see_the_same_innovation_and_variance() {
    let config = config();
    let before = state();
    let mut after = before.clone();
    after.rebase(&shift());
    for measure in [airspeed_measurement, sideslip_measurement] {
        let (a, b) = (
            measure(&before, 52.0, &config),
            measure(&after, 52.0, &config),
        );
        // The wind is a horizontal state, so the frame tilt of 1 km
        // (0.009 degrees) leaves out its vertical part: 1 mm/s here.
        assert!((a.innovation - b.innovation).abs() < 1e-3, "{a:?} {b:?}");
        assert!(close(variance(&before, &a), variance(&after, &b), 1e-4));
    }
}

#[test]
fn a_gnss_fix_sees_the_same_innovation_and_the_barometer_datum_stays() {
    let shift = shift();
    let before = state();
    let mut after = before.clone();
    after.rebase(&shift);
    // A fix 3 m from the estimate, expressed in each frame.
    let fix_old = [1_023.0, 868.0, -301.0];
    let fix_new = shift.position(fix_old);
    let residual = |state: &EkfState, fix: [f64; 3]| {
        [
            fix[0] - f64::from(state.pos.x.0),
            fix[1] - f64::from(state.pos.y.0),
            fix[2] - f64::from(state.pos.z.0),
        ]
    };
    let length = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let (r_old, r_new) = (residual(&before, fix_old), residual(&after, fix_new));
    assert!((length(r_old) - length(r_new)).abs() < 1e-3);
    // The trace of the position block is invariant under rotation.
    let trace = |s: &EkfState| {
        (0..3)
            .map(|i| s.p_cov.get(IDX_POS + i, IDX_POS + i))
            .sum::<f32>()
    };
    assert!(close(trace(&before), trace(&after), 1e-5));
    // The barometer datum is a height reference and does not move.
    assert_eq!(before.baro_ref, after.baro_ref);
}

#[test]
fn the_inverse_shift_returns_the_original_state() {
    let shift = shift();
    let original = state();
    let mut state = original.clone();
    state.rebase(&shift);
    state.rebase(&inverse(&shift));
    let near = |a: Scalar, b: Scalar| close(a, b, 1e-5);
    assert!(near(state.pos.x.0, original.pos.x.0) && near(state.pos.y.0, original.pos.y.0));
    assert!(near(state.pos.z.0, original.pos.z.0));
    assert!(near(state.vel.x.0, original.vel.x.0) && near(state.vel.z.0, original.vel.z.0));
    let dot = state.quat.w * original.quat.w
        + state.quat.x * original.quat.x
        + state.quat.y * original.quat.y
        + state.quat.z * original.quat.z;
    assert!(dot.abs() > 1.0 - 1e-6);
    for i in 0..STATE_DIM {
        for j in 0..STATE_DIM {
            assert!(
                near(state.p_cov.get(i, j), original.p_cov.get(i, j)),
                "P[{i}][{j}]"
            );
        }
    }
    assert!(near(state.air.wind_ne[0], original.air.wind_ne[0]));
    assert!(near(state.air.p_xw[3][0], original.air.p_xw[3][0]));
    assert!(near(state.baro_ref.unwrap(), original.baro_ref.unwrap()));
    assert_eq!(state.frame.rebases, 2);
}
