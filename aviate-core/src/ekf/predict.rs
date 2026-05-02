//! EKF prediction step: IMU-driven state and covariance propagation.
//!
//! Phase 4: this is now part of `impl Estimator for Ekf` — the
//! algorithm reads its tuning from `&self.config` and mutates
//! filter state through `&mut state: &mut EstimatorState`. There is
//! exactly one owner of the persistent filter state (KernelState).

use super::{EstimatorState, IDX_AB, IDX_ATT, IDX_GB, IDX_MB, IDX_POS, IDX_VEL, STATE_DIM};
use crate::ekf::Ekf;
use crate::math::{Matrix, Quaternion, Vector3};
use crate::sensor::ImuData;
#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::{FloatExt, Meters, MetersPerSecond, Scalar, Validated};

impl Ekf {
    pub(crate) fn predict_state(&self, state: &mut EstimatorState, imu: &ImuData, dt: Scalar) {
        if !state.initialized {
            return;
        }
        if !dt.is_finite() || dt <= 0.0 {
            return;
        }

        // Optional: early bail if IMU clearly bad
        for c in 0..3 {
            if !imu.gyro[c].is_valid() || !imu.accel[c].is_valid() {
                // TODO: set fault flag / quality = Degraded
                return;
            }
        }

        // 1. Extract IMU data (corrected with current bias)
        let gyro_corr = Vector3 {
            x: imu.gyro[0] - state.gyro_bias.x,
            y: imu.gyro[1] - state.gyro_bias.y,
            z: imu.gyro[2] - state.gyro_bias.z,
        };

        state.last_gyro_body = gyro_corr;

        let accel_corr = Vector3 {
            x: imu.accel[0] - state.accel_bias.x,
            y: imu.accel[1] - state.accel_bias.y,
            z: imu.accel[2] - state.accel_bias.z,
        };

        // 2. Integrate State

        // Rotate accel from Body to Earth (NED)
        let accel_corr_scalar = Vector3 {
            x: accel_corr.x.0,
            y: accel_corr.y.0,
            z: accel_corr.z.0,
        };
        let accel_earth_scalar = state.quat.rotate_vector(accel_corr_scalar);

        // Gravity in NED frame (positive z is down)
        let g = Vector3::new(0.0, 0.0, 9.81);
        let accel_net_scalar = Vector3 {
            x: accel_earth_scalar.x,
            y: accel_earth_scalar.y,
            z: accel_earth_scalar.z + g.z,
        };

        // Integrate Position & Velocity
        // pos = pos + vel * dt + 0.5 * acc * dt * dt
        state.pos.x =
            Meters(state.pos.x.0 + state.vel.x.0 * dt + 0.5 * accel_net_scalar.x * dt * dt);
        state.pos.y =
            Meters(state.pos.y.0 + state.vel.y.0 * dt + 0.5 * accel_net_scalar.y * dt * dt);
        state.pos.z =
            Meters(state.pos.z.0 + state.vel.z.0 * dt + 0.5 * accel_net_scalar.z * dt * dt);

        state.vel.x = MetersPerSecond(state.vel.x.0 + accel_net_scalar.x * dt);
        state.vel.y = MetersPerSecond(state.vel.y.0 + accel_net_scalar.y * dt);
        state.vel.z = MetersPerSecond(state.vel.z.0 + accel_net_scalar.z * dt);

        // Attitude (Quaternion integration)
        let delta_angle = Vector3 {
            x: gyro_corr.x.0,
            y: gyro_corr.y.0,
            z: gyro_corr.z.0,
        };
        let angle_mag = (delta_angle.x * delta_angle.x
            + delta_angle.y * delta_angle.y
            + delta_angle.z * delta_angle.z)
            .sqrt();
        let dq = if angle_mag > 1e-6 {
            Quaternion::from_axis_angle(
                Vector3::new(
                    delta_angle.x / angle_mag,
                    delta_angle.y / angle_mag,
                    delta_angle.z / angle_mag,
                ),
                angle_mag * dt,
            )
        } else {
            Quaternion::IDENTITY
        };
        let new_quat = state.quat.mul(&dq);
        state.quat = state.sanitize_quat(new_quat);

        // 3. Propagate Covariance (P = F*P*F' + Q)
        let mut f_mat = Matrix::<STATE_DIM, STATE_DIM>::identity();

        // dPos/dVel = I * dt
        f_mat.set(IDX_POS, IDX_VEL, dt);
        f_mat.set(IDX_POS + 1, IDX_VEL + 1, dt);
        f_mat.set(IDX_POS + 2, IDX_VEL + 2, dt);

        // dVel/dAtt = -[R * a]x * dt
        let rot_accel_skew = accel_earth_scalar.skew_symmetric();

        for r in 0..3 {
            for c in 0..3 {
                let val = -rot_accel_skew.get(r, c) * dt;
                f_mat.set(IDX_VEL + r, IDX_ATT + c, val);
            }
        }

        // dVel/dAccelBias = -R * dt
        let r_mat = state.quat.to_rotation_matrix();
        for r in 0..3 {
            for c in 0..3 {
                let val = -r_mat.get(r, c) * dt;
                f_mat.set(IDX_VEL + r, IDX_AB + c, val);
            }
        }

        // dAtt/dGyroBias = -R * dt
        for r in 0..3 {
            for c in 0..3 {
                let val = -r_mat.get(r, c) * dt;
                f_mat.set(IDX_ATT + r, IDX_GB + c, val);
            }
        }

        // Q (Process Noise)
        let mut q_noise = Matrix::<STATE_DIM, STATE_DIM>::zero();
        for i in 0..3 {
            q_noise.set(IDX_POS + i, IDX_POS + i, 0.001);
        }
        for i in 0..3 {
            q_noise.set(
                IDX_VEL + i,
                IDX_VEL + i,
                self.config.process_noise_accel * dt * dt,
            );
        }
        for i in 0..3 {
            q_noise.set(
                IDX_ATT + i,
                IDX_ATT + i,
                self.config.process_noise_gyro * dt * dt,
            );
        }
        for i in 0..3 {
            q_noise.set(
                IDX_GB + i,
                IDX_GB + i,
                self.config.process_noise_gyro_bias * dt,
            );
        }
        for i in 0..3 {
            q_noise.set(
                IDX_AB + i,
                IDX_AB + i,
                self.config.process_noise_accel_bias * dt,
            );
        }
        for i in 0..3 {
            q_noise.set(
                IDX_MB + i,
                IDX_MB + i,
                self.config.process_noise_mag_bias * dt,
            );
        }

        // P = F * P * F' + Q
        let fp = f_mat.mat_mul(&state.p_cov);
        let fpft = fp.mat_mul(&f_mat.t());
        state.p_cov = fpft.add(&q_noise);
        state.p_cov.make_symmetric();
    }
}
