//! Scalar EKF update kernel and heading specialization.
//!
//! `update_scalar` is the workhorse used by `update_gnss` and
//! `update_baro`; `update_heading` is a yaw-specific variant used by
//! `update_mag` after it has resolved the heading innovation. Both live
//! here to keep the `impl Ekf` block under the 500-line cap per file.
//! No re-exports — impl block split only.

use super::{Ekf, IDX_AB, IDX_ATT, IDX_GB, IDX_MB, IDX_POS, IDX_VEL, STATE_DIM};
use crate::math::{Quaternion, Vector3};
use crate::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, RadiansPerSecond, Scalar,
};

impl Ekf {
    /// Internal: Update yaw/heading using scalar observation.
    ///
    /// H = [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, ..., 0] (observes z-component of attitude error)
    pub(crate) fn update_heading(&mut self, innov: Scalar, r_noise: Scalar) {
        // For heading, we observe the z-component of attitude error (yaw)
        // H is a row vector with 1 at IDX_ATT+2 (yaw error state)
        let state_idx = IDX_ATT + 2; // Yaw error state

        // Innovation variance: S = H * P * H' + R = P[yaw][yaw] + R
        let s = self.p_cov.get(state_idx, state_idx) + r_noise;
        if s < 1e-9 {
            return; // COV:EXCL(DEFENSIVE: prevent division by zero)
        }

        // Innovation gating
        let gate_sq = self.config.innovation_gate * self.config.innovation_gate;
        if (innov * innov) / s > gate_sq {
            return; // Reject measurement
        }

        // Kalman gain: K = P * H' / S
        // Since H is sparse (only 1 at state_idx), K[i] = P[i][state_idx] / S
        let k_gain_factor = 1.0 / s;
        let mut k_vector = [0.0; STATE_DIM];
        for (i, val) in k_vector.iter_mut().enumerate().take(STATE_DIM) {
            *val = self.p_cov.get(i, state_idx) * k_gain_factor;
        }

        // Update position states
        self.pos.x = Meters(self.pos.x.0 + k_vector[IDX_POS] * innov);
        self.pos.y = Meters(self.pos.y.0 + k_vector[IDX_POS + 1] * innov);
        self.pos.z = Meters(self.pos.z.0 + k_vector[IDX_POS + 2] * innov);

        // Update velocity states
        self.vel.x = MetersPerSecond(self.vel.x.0 + k_vector[IDX_VEL] * innov);
        self.vel.y = MetersPerSecond(self.vel.y.0 + k_vector[IDX_VEL + 1] * innov);
        self.vel.z = MetersPerSecond(self.vel.z.0 + k_vector[IDX_VEL + 2] * innov);

        // Update attitude (apply small angle rotation to quaternion)
        let d_ang = Vector3::new(
            k_vector[IDX_ATT] * innov,
            k_vector[IDX_ATT + 1] * innov,
            k_vector[IDX_ATT + 2] * innov,
        );
        let dq_small = self.sanitize_quat(Quaternion::new(
            1.0,
            d_ang.x * 0.5,
            d_ang.y * 0.5,
            d_ang.z * 0.5,
        ));
        self.quat = self.sanitize_quat(self.quat.mul(&dq_small));

        // Update gyro bias
        self.gyro_bias.x = RadiansPerSecond(self.gyro_bias.x.0 + k_vector[IDX_GB] * innov);
        self.gyro_bias.y = RadiansPerSecond(self.gyro_bias.y.0 + k_vector[IDX_GB + 1] * innov);
        self.gyro_bias.z = RadiansPerSecond(self.gyro_bias.z.0 + k_vector[IDX_GB + 2] * innov);

        // Update accel bias
        self.accel_bias.x = MetersPerSecondSquared(self.accel_bias.x.0 + k_vector[IDX_AB] * innov);
        self.accel_bias.y =
            MetersPerSecondSquared(self.accel_bias.y.0 + k_vector[IDX_AB + 1] * innov);
        self.accel_bias.z =
            MetersPerSecondSquared(self.accel_bias.z.0 + k_vector[IDX_AB + 2] * innov);

        // Update mag bias
        self.mag_bias.x = Microtesla(self.mag_bias.x.0 + k_vector[IDX_MB] * innov);
        self.mag_bias.y = Microtesla(self.mag_bias.y.0 + k_vector[IDX_MB + 1] * innov);
        self.mag_bias.z = Microtesla(self.mag_bias.z.0 + k_vector[IDX_MB + 2] * innov);

        // Update covariance: P = (I - K*H) * P
        // H*P is row state_idx of P
        let mut p_row_h = [0.0; STATE_DIM];
        for (c, val) in p_row_h.iter_mut().enumerate().take(STATE_DIM) {
            *val = self.p_cov.get(state_idx, c);
        }

        for (r, &k_val) in k_vector.iter().enumerate().take(STATE_DIM) {
            for (c, &p_val) in p_row_h.iter().enumerate().take(STATE_DIM) {
                let val = self.p_cov.get(r, c) - k_val * p_val;
                self.p_cov.set(r, c, val);
            }
        }

        self.p_cov.make_symmetric();
    }

    #[doc(hidden)]
    pub fn update_scalar(&mut self, state_idx: usize, meas: Scalar, r_noise: Scalar) {
        // Standard EKF scalar update: H = [0, ..., 1, ... 0] at state_idx
        let pred = match state_idx {
            0 => self.pos.x.0,
            1 => self.pos.y.0,
            2 => self.pos.z.0,
            3 => self.vel.x.0,
            4 => self.vel.y.0,
            5 => self.vel.z.0,
            _ => return, // COV:EXCL(DEFENSIVE: invalid state_idx guard)
        };
        let innov = meas - pred;

        // Innovation Gating
        let s = self.p_cov.get(state_idx, state_idx) + r_noise;
        if s < 1e-9 {
            return; // COV:EXCL(DEFENSIVE: prevent division by zero)
        }

        let gate_sq = self.config.innovation_gate * self.config.innovation_gate;
        if (innov * innov) / s > gate_sq {
            return; // Reject measurement
        }

        // 2. Kalman Gain K = PH' / S
        // PH' is the column of P at state_idx
        let k_gain_factor = 1.0 / s;
        // We can compute K and update state & P directly to avoid allocating K vector explicitly

        // Update State: x = x + K * innov
        // K[i] = P[i][state_idx] / S
        let mut k_vector = [0.0; STATE_DIM];
        for (i, val) in k_vector.iter_mut().enumerate().take(STATE_DIM) {
            *val = self.p_cov.get(i, state_idx) * k_gain_factor;
        }

        self.pos.x = Meters(self.pos.x.0 + k_vector[IDX_POS] * innov);
        self.pos.y = Meters(self.pos.y.0 + k_vector[IDX_POS + 1] * innov);
        self.pos.z = Meters(self.pos.z.0 + k_vector[IDX_POS + 2] * innov);

        self.vel.x = MetersPerSecond(self.vel.x.0 + k_vector[IDX_VEL] * innov);
        self.vel.y = MetersPerSecond(self.vel.y.0 + k_vector[IDX_VEL + 1] * innov);
        self.vel.z = MetersPerSecond(self.vel.z.0 + k_vector[IDX_VEL + 2] * innov);

        // Update other states (biases, etc.)
        self.gyro_bias.x = RadiansPerSecond(self.gyro_bias.x.0 + k_vector[IDX_GB] * innov);
        self.gyro_bias.y = RadiansPerSecond(self.gyro_bias.y.0 + k_vector[IDX_GB + 1] * innov);
        self.gyro_bias.z = RadiansPerSecond(self.gyro_bias.z.0 + k_vector[IDX_GB + 2] * innov);

        self.accel_bias.x = MetersPerSecondSquared(self.accel_bias.x.0 + k_vector[IDX_AB] * innov);
        self.accel_bias.y =
            MetersPerSecondSquared(self.accel_bias.y.0 + k_vector[IDX_AB + 1] * innov);
        self.accel_bias.z =
            MetersPerSecondSquared(self.accel_bias.z.0 + k_vector[IDX_AB + 2] * innov);

        // Attitude update (linearized error)
        let d_ang = Vector3::new(
            k_vector[IDX_ATT] * innov,
            k_vector[IDX_ATT + 1] * innov,
            k_vector[IDX_ATT + 2] * innov,
        );
        // Apply small angle rotation to quaternion
        // Better: use small angle approx dq = [1, dx/2, dy/2, dz/2]
        let dq_small = self.sanitize_quat(Quaternion::new(
            1.0,
            d_ang.x * 0.5,
            d_ang.y * 0.5,
            d_ang.z * 0.5,
        ));
        self.quat = self.sanitize_quat(self.quat.mul(&dq_small));

        // 4. Update P = (I - KH) * P
        // P_new = P - K * H * P
        // H*P is row state_idx of P.
        // (K * (H*P))[r][c] = K[r] * P[state_idx][c]

        // Create new P to avoid in-place corruption during calc?
        // P[r][c] -= K[r] * P[state_idx][c]
        // This can be done in place safely if we iterate carefully?
        // Yes, P[state_idx][c] is constant for the row 'r' loop if we extract row first.

        let mut p_row_h = [0.0; STATE_DIM];
        for (c, val) in p_row_h.iter_mut().enumerate().take(STATE_DIM) {
            *val = self.p_cov.get(state_idx, c);
        }

        for (r, &k_val) in k_vector.iter().enumerate().take(STATE_DIM) {
            for (c, &p_val) in p_row_h.iter().enumerate().take(STATE_DIM) {
                let val = self.p_cov.get(r, c) - k_val * p_val;
                self.p_cov.set(r, c, val);
            }
        }

        self.p_cov.make_symmetric();
    }
}
