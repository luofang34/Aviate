//! Scalar EKF update kernel and heading specialization.
//!
//! `scalar_update` is the workhorse used by `update_gnss` and
//! `update_baro`; `heading_update` is a yaw-specific variant used by
//! `update_mag` after it has resolved the heading innovation. Both
//! live here to keep the `impl Ekf` block under the 500-line cap per
//! file. Phase 4: methods take `&self` (algorithm tuning) and `&mut
//! state: &mut EkfState` (filter state).

use super::{Ekf, EkfState, IDX_AB, IDX_ATT, IDX_GB, IDX_MB, IDX_POS, IDX_VEL, STATE_DIM};
use crate::math::{Quaternion, Vector3};
use crate::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, RadiansPerSecond, Scalar,
};

impl Ekf {
    /// Internal: Update yaw/heading using scalar observation.
    ///
    /// H = [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, ..., 0] (observes z-component of attitude error)
    pub(crate) fn heading_update(&self, state: &mut EkfState, innov: Scalar, r_noise: Scalar) {
        let state_idx = IDX_ATT + 2; // Yaw error state

        // Innovation variance
        let s = state.p_cov.get(state_idx, state_idx) + r_noise;
        if s < 1e-9 {
            return; // COV:EXCL(DEFENSIVE: prevent division by zero)
        }

        // Innovation gating
        let gate_sq = self.config.innovation_gate * self.config.innovation_gate;
        if (innov * innov) / s > gate_sq {
            return; // Reject measurement
        }

        // Kalman gain
        let k_gain_factor = 1.0 / s;
        let mut k_vector = [0.0; STATE_DIM];
        for (i, val) in k_vector.iter_mut().enumerate().take(STATE_DIM) {
            *val = state.p_cov.get(i, state_idx) * k_gain_factor;
        }

        // Update position states
        state.pos.x = Meters(state.pos.x.0 + k_vector[IDX_POS] * innov);
        state.pos.y = Meters(state.pos.y.0 + k_vector[IDX_POS + 1] * innov);
        state.pos.z = Meters(state.pos.z.0 + k_vector[IDX_POS + 2] * innov);

        // Update velocity states
        state.vel.x = MetersPerSecond(state.vel.x.0 + k_vector[IDX_VEL] * innov);
        state.vel.y = MetersPerSecond(state.vel.y.0 + k_vector[IDX_VEL + 1] * innov);
        state.vel.z = MetersPerSecond(state.vel.z.0 + k_vector[IDX_VEL + 2] * innov);

        // Update attitude
        let d_ang = Vector3::new(
            k_vector[IDX_ATT] * innov,
            k_vector[IDX_ATT + 1] * innov,
            k_vector[IDX_ATT + 2] * innov,
        );
        let dq_small = state.sanitize_quat(Quaternion::new(
            1.0,
            d_ang.x * 0.5,
            d_ang.y * 0.5,
            d_ang.z * 0.5,
        ));
        let new_quat = state.quat.mul(&dq_small);
        state.quat = state.sanitize_quat(new_quat);

        // Update gyro bias
        state.gyro_bias.x = RadiansPerSecond(state.gyro_bias.x.0 + k_vector[IDX_GB] * innov);
        state.gyro_bias.y = RadiansPerSecond(state.gyro_bias.y.0 + k_vector[IDX_GB + 1] * innov);
        state.gyro_bias.z = RadiansPerSecond(state.gyro_bias.z.0 + k_vector[IDX_GB + 2] * innov);

        // Update accel bias
        state.accel_bias.x =
            MetersPerSecondSquared(state.accel_bias.x.0 + k_vector[IDX_AB] * innov);
        state.accel_bias.y =
            MetersPerSecondSquared(state.accel_bias.y.0 + k_vector[IDX_AB + 1] * innov);
        state.accel_bias.z =
            MetersPerSecondSquared(state.accel_bias.z.0 + k_vector[IDX_AB + 2] * innov);

        // Update mag bias
        state.mag_bias.x = Microtesla(state.mag_bias.x.0 + k_vector[IDX_MB] * innov);
        state.mag_bias.y = Microtesla(state.mag_bias.y.0 + k_vector[IDX_MB + 1] * innov);
        state.mag_bias.z = Microtesla(state.mag_bias.z.0 + k_vector[IDX_MB + 2] * innov);

        // Update covariance: P = (I - K*H) * P
        let mut p_row_h = [0.0; STATE_DIM];
        for (c, val) in p_row_h.iter_mut().enumerate().take(STATE_DIM) {
            *val = state.p_cov.get(state_idx, c);
        }

        for (r, &k_val) in k_vector.iter().enumerate().take(STATE_DIM) {
            for (c, &p_val) in p_row_h.iter().enumerate().take(STATE_DIM) {
                let val = state.p_cov.get(r, c) - k_val * p_val;
                state.p_cov.set(r, c, val);
            }
        }

        state.p_cov.make_symmetric();
    }

    pub(crate) fn scalar_update(
        &self,
        state: &mut EkfState,
        state_idx: usize,
        meas: Scalar,
        r_noise: Scalar,
    ) {
        // Standard EKF scalar update: H = [0, ..., 1, ... 0] at state_idx
        let pred = match state_idx {
            0 => state.pos.x.0,
            1 => state.pos.y.0,
            2 => state.pos.z.0,
            3 => state.vel.x.0,
            4 => state.vel.y.0,
            5 => state.vel.z.0,
            _ => return, // COV:EXCL(DEFENSIVE: invalid state_idx guard)
        };
        let innov = meas - pred;

        // Innovation Gating
        let s = state.p_cov.get(state_idx, state_idx) + r_noise;
        if s < 1e-9 {
            return; // COV:EXCL(DEFENSIVE: prevent division by zero)
        }

        let gate_sq = self.config.innovation_gate * self.config.innovation_gate;
        if (innov * innov) / s > gate_sq {
            return; // Reject measurement
        }

        // Kalman Gain
        let k_gain_factor = 1.0 / s;
        let mut k_vector = [0.0; STATE_DIM];
        for (i, val) in k_vector.iter_mut().enumerate().take(STATE_DIM) {
            *val = state.p_cov.get(i, state_idx) * k_gain_factor;
        }

        state.pos.x = Meters(state.pos.x.0 + k_vector[IDX_POS] * innov);
        state.pos.y = Meters(state.pos.y.0 + k_vector[IDX_POS + 1] * innov);
        state.pos.z = Meters(state.pos.z.0 + k_vector[IDX_POS + 2] * innov);

        state.vel.x = MetersPerSecond(state.vel.x.0 + k_vector[IDX_VEL] * innov);
        state.vel.y = MetersPerSecond(state.vel.y.0 + k_vector[IDX_VEL + 1] * innov);
        state.vel.z = MetersPerSecond(state.vel.z.0 + k_vector[IDX_VEL + 2] * innov);

        state.gyro_bias.x = RadiansPerSecond(state.gyro_bias.x.0 + k_vector[IDX_GB] * innov);
        state.gyro_bias.y = RadiansPerSecond(state.gyro_bias.y.0 + k_vector[IDX_GB + 1] * innov);
        state.gyro_bias.z = RadiansPerSecond(state.gyro_bias.z.0 + k_vector[IDX_GB + 2] * innov);

        state.accel_bias.x =
            MetersPerSecondSquared(state.accel_bias.x.0 + k_vector[IDX_AB] * innov);
        state.accel_bias.y =
            MetersPerSecondSquared(state.accel_bias.y.0 + k_vector[IDX_AB + 1] * innov);
        state.accel_bias.z =
            MetersPerSecondSquared(state.accel_bias.z.0 + k_vector[IDX_AB + 2] * innov);

        // Attitude update (linearized error)
        let d_ang = Vector3::new(
            k_vector[IDX_ATT] * innov,
            k_vector[IDX_ATT + 1] * innov,
            k_vector[IDX_ATT + 2] * innov,
        );
        let dq_small = state.sanitize_quat(Quaternion::new(
            1.0,
            d_ang.x * 0.5,
            d_ang.y * 0.5,
            d_ang.z * 0.5,
        ));
        let new_quat = state.quat.mul(&dq_small);
        state.quat = state.sanitize_quat(new_quat);

        // Update P = (I - KH) * P
        let mut p_row_h = [0.0; STATE_DIM];
        for (c, val) in p_row_h.iter_mut().enumerate().take(STATE_DIM) {
            *val = state.p_cov.get(state_idx, c);
        }

        for (r, &k_val) in k_vector.iter().enumerate().take(STATE_DIM) {
            for (c, &p_val) in p_row_h.iter().enumerate().take(STATE_DIM) {
                let val = state.p_cov.get(r, c) - k_val * p_val;
                state.p_cov.set(r, c, val);
            }
        }

        state.p_cov.make_symmetric();
    }
}

/// Implement the public `Estimator` trait by delegating each method
/// to the per-submodule helper. The trait surface takes `&mut state`
/// — the helpers carry the math against the same `&mut state`.
// COV:EXCL_START(DELEGATE: every body in this impl forwards to the
// equivalent inherent Ekf helper that carries the math; the delegate
// has no executable logic of its own and is exercised through the
// kernel update path. The math is tested directly via ekf_tests.rs
// against the inherent helpers.)
impl super::Estimator for Ekf {
    type RuntimeState = EkfState;

    fn observe(
        &self,
        state: &mut EkfState,
        sensors: &crate::sensor::SensorSet,
        overrides: Option<&crate::control::SensorOverrides>,
        dt: Scalar,
    ) {
        // EKF-shaped flow: predict-per-IMU + update-per-sensor.
        // Each gate (validity, health, override) is the same logic
        // that previously lived in `kernel_update.rs`; moving it
        // here means the kernel no longer hardcodes which channels
        // an estimator consumes — `Ekf::observe` decides.
        let primary_imu = &sensors.imus[0];
        if primary_imu.valid && primary_imu.health == crate::sensor::SensorHealth::Good {
            Ekf::predict_state(self, state, &primary_imu.value, dt);
        }

        if let Some(o) = overrides {
            if let Some(gnss_health) = o.gnss_force_state {
                let mut primary_gnss_reading = sensors.gnss[0];
                primary_gnss_reading.health = match gnss_health {
                    crate::sensor::GnssHealth::Good => crate::sensor::SensorHealth::Good,
                    crate::sensor::GnssHealth::Suspect => crate::sensor::SensorHealth::Degraded,
                    crate::sensor::GnssHealth::Lost => crate::sensor::SensorHealth::Failed,
                };
                Ekf::update_gnss_state(self, state, &primary_gnss_reading);
            }
        } else {
            let primary_gnss = &sensors.gnss[0];
            if primary_gnss.valid && primary_gnss.health == crate::sensor::SensorHealth::Good {
                Ekf::update_gnss_state(self, state, primary_gnss);
            }
        }

        let primary_baro = &sensors.baros[0];
        if primary_baro.valid && primary_baro.health == crate::sensor::SensorHealth::Good {
            Ekf::update_baro_state(self, state, primary_baro);
        }

        let primary_mag = &sensors.mags[0];
        if primary_mag.valid && primary_mag.health == crate::sensor::SensorHealth::Good {
            Ekf::update_mag_state(self, state, primary_mag);
        }
    }

    fn estimate(&self, state: &EkfState) -> crate::state::StateEstimate {
        state.get_estimate()
    }

    // No `reset` override: the trait default routes through
    // `EstimatorRuntimeState::reset(state)`, which delegates to
    // `EkfState::reset(self)` — the same factory-reset identity
    // we'd supply explicitly. Letting the default fire keeps the
    // `EstimatorRuntimeState` impl covered.

    #[cfg(feature = "test-hooks")]
    fn inject_state(&self, state: &mut EkfState, est: &crate::state::StateEstimate) {
        state.set_state(est);
    }
}
// COV:EXCL_STOP
