//! The Jacobians of the joint measurements against finite differences
//! of their predicted values, with the error-state conventions of the
//! filter: additive velocity and wind, and an attitude error in the
//! navigation frame that multiplies from the left.

use super::{heading_measurement, JointMeasurement};
use crate::ekf::air_aiding::{airspeed_measurement, sideslip_measurement, AirAidingConfig};
use crate::ekf::update::heading_innovation;
use crate::ekf::{EkfState, IDX_ATT, IDX_VEL};
use crate::math::{Quaternion, Vector3};
use crate::types::{Meters, MetersPerSecond, Scalar};

const STEP: Scalar = 1e-3;

fn config() -> AirAidingConfig {
    AirAidingConfig {
        meas_noise_tas: 1.0,
        sideslip_sigma: 0.05,
        wind_prior_var: 64.0,
        process_noise_wind: 0.05,
        min_true_airspeed: 15.0,
    }
}

/// A banked, pitched, yawed vehicle climbing through a crosswind.
fn state() -> EkfState {
    let mut state = EkfState::new();
    let attitude = Quaternion::from_axis_angle(Vector3::new(0.3, 0.2, 0.9), 0.8).normalize();
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(30.0),
            MetersPerSecond(40.0),
            MetersPerSecond(-2.0),
        ),
        attitude,
    );
    state.air.active = true;
    state.air.wind_ne = [4.0, -3.0];
    state
}

/// Perturbation `which` of size `step`: 0..3 velocity, 3..6 attitude,
/// 6..8 wind.
fn perturbed(base: &EkfState, which: usize, step: Scalar) -> EkfState {
    let mut state = base.clone();
    let mut delta = [0.0; 3];
    match which {
        0..=2 => {
            let v = [&mut state.vel.x, &mut state.vel.y, &mut state.vel.z];
            v.into_iter().nth(which).unwrap().0 += step;
        }
        3..=5 => {
            delta[which - 3] = step;
            let small = Quaternion::new(1.0, 0.5 * delta[0], 0.5 * delta[1], 0.5 * delta[2]);
            state.quat = small.normalize().mul(&state.quat).normalize();
        }
        _ => state.air.wind_ne[which - 6] += step,
    }
    state
}

fn jacobian_entry(measurement: &JointMeasurement, which: usize) -> Scalar {
    match which {
        0..=2 => measurement.h_x[IDX_VEL + which],
        3..=5 => measurement.h_x[IDX_ATT + which - 3],
        _ => measurement.h_w[which - 6],
    }
}

/// Compare each Jacobian entry with a central difference of `predicted`.
fn check(
    predicted: impl Fn(&EkfState) -> Scalar,
    measured: impl Fn(&EkfState) -> JointMeasurement,
) {
    let base = state();
    let measurement = measured(&base);
    for which in 0..8 {
        let plus = predicted(&perturbed(&base, which, STEP));
        let minus = predicted(&perturbed(&base, which, -STEP));
        let numeric = (plus - minus) / (2.0 * STEP);
        let analytic = jacobian_entry(&measurement, which);
        let tolerance = 2e-2 * numeric.abs().max(1.0);
        assert!(
            (numeric - analytic).abs() < tolerance,
            "entry {which}: numeric {numeric}, analytic {analytic}"
        );
    }
}

#[test]
fn airspeed_jacobian_matches_the_predicted_airspeed() {
    let config = config();
    check(
        |state| 50.0 - airspeed_measurement(state, 50.0, &config).innovation,
        |state| airspeed_measurement(state, 50.0, &config),
    );
}

#[test]
fn sideslip_jacobian_matches_the_predicted_lateral_air_velocity() {
    let config = config();
    check(
        |state| -sideslip_measurement(state, 50.0, &config).innovation,
        |state| sideslip_measurement(state, 50.0, &config),
    );
}

#[test]
fn heading_jacobian_sees_tilt_through_the_levelled_field() {
    // A field with a strong vertical part, as at mid latitudes.
    let field_nav = Vector3::new(20.0, -2.0, 45.0);
    let base = state();
    let q = base.quat;
    let field_body = Quaternion::new(q.w, -q.x, -q.y, -q.z).rotate_vector(field_nav);
    let body = [field_body.x, field_body.y, field_body.z];
    let measurement = |state: &EkfState| {
        let innovation = heading_innovation(&state.quat, body, 0.0);
        heading_measurement(state.quat.rotate_vector(field_body), innovation, 0.05).unwrap()
    };
    check(
        |state| -heading_innovation(&state.quat, body, 0.0),
        measurement,
    );
    // The tilt entries are not zero: a roll or pitch error moves the heading.
    let row = measurement(&base).h_x;
    assert!(row[IDX_ATT].abs() > 0.1 || row[IDX_ATT + 1].abs() > 0.1);
}
