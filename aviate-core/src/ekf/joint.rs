//! Scalar update of the filter states together with the wind states.
//!
//! The 15 filter states and the 2 wind states of `ekf/air_aiding.rs`
//! form one joint state. A measurement that sees the wind, or a
//! measurement whose Jacobian has more than one non-zero entry, uses
//! this update. It keeps the 15-state covariance, the cross block and
//! the wind block consistent with one another.

use super::scalar::COV_VAR_FLOOR;
use super::{Ekf, EkfState, IDX_AB, IDX_ATT, IDX_GB, IDX_POS, IDX_VEL, STATE_DIM};
use crate::math::{Quaternion, Vector3};
use crate::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond, Scalar};

// The literal rows of `state_row` and the destructuring in
// `apply_correction` spell out this layout.
const _: () = assert!(IDX_POS == 0 && IDX_VEL == 3 && IDX_ATT == 6);
const _: () = assert!(IDX_GB == 9 && IDX_AB == 12 && STATE_DIM == 15);

/// Smallest innovation variance that the update divides by.
const MIN_INNOVATION_VARIANCE: Scalar = 1e-9;

/// One scalar measurement of the joint state: its Jacobian on the
/// filter states and on the wind states, its innovation and its noise.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JointMeasurement {
    pub(crate) h_x: [Scalar; STATE_DIM],
    pub(crate) h_w: [Scalar; 2],
    pub(crate) innovation: Scalar,
    pub(crate) noise: Scalar,
}

/// Jacobian row that sees only the velocity and the attitude error.
pub(crate) const fn state_row(velocity: [Scalar; 3], attitude: [Scalar; 3]) -> [Scalar; STATE_DIM] {
    let [vn, ve, vd] = velocity;
    let [an, ae, ad] = attitude;
    [
        0.0, 0.0, 0.0, vn, ve, vd, an, ae, ad, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ]
}

impl Ekf {
    /// One scalar update of the filter states and the wind states.
    /// Returns whether the innovation gate accepted the measurement.
    pub(crate) fn fuse_joint(&self, state: &mut EkfState, measurement: &JointMeasurement) -> bool {
        let JointMeasurement {
            h_x,
            h_w,
            innovation,
            noise,
        } = *measurement;
        let air = &state.air;
        // Covariance of each filter state and each wind state with the
        // measurement.
        let mut ph_x = [0.0; STATE_DIM];
        for ((value, p_row), cross) in ph_x.iter_mut().zip(&state.p_cov.data).zip(&air.p_xw) {
            *value = dot(p_row, &h_x) + dot(cross, &h_w);
        }
        let from_states = air.p_xw.iter().zip(&h_x).fold([0.0; 2], |sum, (cross, h)| {
            [sum[0] + cross[0] * h, sum[1] + cross[1] * h]
        });
        let ph_w = [
            from_states[0] + dot(&air.p_ww[0], &h_w),
            from_states[1] + dot(&air.p_ww[1], &h_w),
        ];
        let s = dot(&h_x, &ph_x) + dot(&h_w, &ph_w) + noise;
        let gate = self.config.innovation_gate;
        if !s.is_finite()
            || s < MIN_INNOVATION_VARIANCE
            || !innovation.is_finite()
            || innovation * innovation > gate * gate * s
        {
            return false;
        }

        let correction = ph_x.map(|ph| ph / s * innovation);
        self.apply_correction(state, &correction);

        for ((p_row, ph_row), cross) in state
            .p_cov
            .data
            .iter_mut()
            .zip(ph_x)
            .zip(state.air.p_xw.iter_mut())
        {
            let gain = ph_row / s;
            for (value, ph_column) in p_row.iter_mut().zip(ph_x) {
                *value -= gain * ph_column;
            }
            cross[0] -= gain * ph_w[0];
            cross[1] -= gain * ph_w[1];
        }
        state.p_cov.make_symmetric();
        state.p_cov.floor_diagonal(COV_VAR_FLOOR);
        let air = &mut state.air;
        for ((wind, row), ph) in air.wind_ne.iter_mut().zip(&mut air.p_ww).zip(ph_w) {
            let gain = ph / s;
            *wind += gain * innovation;
            row[0] -= gain * ph_w[0];
            row[1] -= gain * ph_w[1];
        }
        air.repair_wind_block();
        true
    }

    /// Add `correction` to the filter states. The attitude part is an
    /// error in the navigation frame, so it multiplies from the left.
    fn apply_correction(&self, state: &mut EkfState, correction: &[Scalar; STATE_DIM]) {
        let [pn, pe, pd, vn, ve, vd, an, ae, ad, gx, gy, gz, bx, by, bz] = *correction;
        state.pos = Vector3::new(
            Meters(state.pos.x.0 + pn),
            Meters(state.pos.y.0 + pe),
            Meters(state.pos.z.0 + pd),
        );
        state.vel = Vector3::new(
            MetersPerSecond(state.vel.x.0 + vn),
            MetersPerSecond(state.vel.y.0 + ve),
            MetersPerSecond(state.vel.z.0 + vd),
        );
        state.gyro_bias = Vector3::new(
            RadiansPerSecond(state.gyro_bias.x.0 + gx),
            RadiansPerSecond(state.gyro_bias.y.0 + gy),
            RadiansPerSecond(state.gyro_bias.z.0 + gz),
        );
        state.accel_bias = Vector3::new(
            MetersPerSecondSquared(state.accel_bias.x.0 + bx),
            MetersPerSecondSquared(state.accel_bias.y.0 + by),
            MetersPerSecondSquared(state.accel_bias.z.0 + bz),
        );
        self.clamp_biases(state);
        let small = state.sanitize_quat(Quaternion::new(1.0, 0.5 * an, 0.5 * ae, 0.5 * ad));
        let rotated = small.mul(&state.quat);
        state.quat = state.sanitize_quat(rotated);
    }
}

/// Heading measurement for a filter that fuses its heading jointly.
///
/// The heading comes from the magnetometer field rotated into the
/// navigation frame by the estimated attitude. A roll or pitch error
/// therefore moves the measured heading as well as a heading error
/// does. The Jacobian gives the filter both paths, so a heading
/// innovation in a bank corrects the tilt and the gyro bias that
/// caused it, and not the heading alone.
///
/// `field_nav` is the measured field in the navigation frame through
/// the estimated attitude. `None` when the field has no horizontal part.
pub(crate) fn heading_measurement(
    field_nav: Vector3<Scalar>,
    innovation: Scalar,
    noise: Scalar,
) -> Option<JointMeasurement> {
    let horizontal_sq = field_nav.x * field_nav.x + field_nav.y * field_nav.y;
    if !horizontal_sq.is_finite() || horizontal_sq <= Scalar::EPSILON {
        return None;
    }
    let attitude = [
        -field_nav.z * field_nav.x / horizontal_sq,
        -field_nav.z * field_nav.y / horizontal_sq,
        1.0,
    ];
    Some(JointMeasurement {
        h_x: state_row([0.0; 3], attitude),
        h_w: [0.0; 2],
        innovation,
        noise,
    })
}

fn dot<const N: usize>(a: &[Scalar; N], b: &[Scalar; N]) -> Scalar {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests;
