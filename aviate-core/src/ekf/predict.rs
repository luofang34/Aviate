//! EKF prediction step: IMU-driven state and covariance propagation.
//!
//! Extracted from `ekf.rs` to keep that file under the 500-line cap. No
//! re-exports — this is just an `impl Ekf` block split across files so
//! rustc's coverage phantom-DA issue never triggers.

use super::{Ekf, IDX_AB, IDX_ATT, IDX_GB, IDX_MB, IDX_POS, IDX_VEL, STATE_DIM};
use crate::math::{Matrix, Quaternion, Vector3};
use crate::sensor::ImuData;
#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::{FloatExt, Meters, MetersPerSecond, Scalar, Validated};

impl Ekf {
    pub fn predict(&mut self, imu: &ImuData, dt: Scalar) {
        if !self.initialized {
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
            x: imu.gyro[0] - self.gyro_bias.x,
            y: imu.gyro[1] - self.gyro_bias.y,
            z: imu.gyro[2] - self.gyro_bias.z,
        };

        self.last_gyro_body = gyro_corr;

        let accel_corr = Vector3 {
            x: imu.accel[0] - self.accel_bias.x,
            y: imu.accel[1] - self.accel_bias.y,
            z: imu.accel[2] - self.accel_bias.z,
        };

        // 2. Integrate State

        // Rotate accel from Body to Earth (NED)
        // accel_corr is in body frame.
        // quat represents Body -> Earth rotation.
        // Rotate vector logic currently takes Vector3<Scalar>. Need to adapt or unwrap.
        let accel_corr_scalar = Vector3 {
            x: accel_corr.x.0,
            y: accel_corr.y.0,
            z: accel_corr.z.0,
        };
        let accel_earth_scalar = self.quat.rotate_vector(accel_corr_scalar);

        // Gravity in NED frame (positive z is down)
        let g = Vector3::new(0.0, 0.0, 9.81);
        // In NED, IMU measures proper acceleration. When at rest, IMU measures approx [0, 0, -9.81]
        // Adding gravity to this (kinematic_accel = proper_accel + gravity) results in zero kinematic acceleration.
        let accel_net_scalar = Vector3 {
            x: accel_earth_scalar.x,
            y: accel_earth_scalar.y,
            z: accel_earth_scalar.z + g.z,
        };

        // Integrate Position & Velocity
        // pos = pos + vel * dt + 0.5 * acc * dt * dt
        self.pos.x = Meters(self.pos.x.0 + self.vel.x.0 * dt + 0.5 * accel_net_scalar.x * dt * dt);
        self.pos.y = Meters(self.pos.y.0 + self.vel.y.0 * dt + 0.5 * accel_net_scalar.y * dt * dt);
        self.pos.z = Meters(self.pos.z.0 + self.vel.z.0 * dt + 0.5 * accel_net_scalar.z * dt * dt);

        self.vel.x = MetersPerSecond(self.vel.x.0 + accel_net_scalar.x * dt);
        self.vel.y = MetersPerSecond(self.vel.y.0 + accel_net_scalar.y * dt);
        self.vel.z = MetersPerSecond(self.vel.z.0 + accel_net_scalar.z * dt);

        // Attitude (Quaternion integration)
        // dq = 0.5 * q * omega * dt
        // approximate rotation vector
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
        self.quat = self.sanitize_quat(self.quat.mul(&dq));

        // 3. Propagate Covariance (P = F*P*F' + Q)
        // We need Jacobian F (15x15).
        // It's sparse. We can construct it or do sparse multiplication.
        // For simplicity/robustness in this initial version, we can use a simplified P propagation or full matrix if we can afford it.
        // Given strict limits, sparse logic is better but complex to write.
        // I'll implement a simplified F computation (Identity + small terms).

        // F blocks:
        // Pos dot = Vel -> F_pos_vel = I * dt
        // Vel dot = R * Accel -> F_vel_att (skew symmetric term), F_vel_ab = -R
        // Att dot = -Omega * Att -> F_att_att, F_att_gb = -I
        // Bias dot = 0 -> I

        // Let's build F explicitly as Matrix<STATE_DIM, STATE_DIM>.
        let mut f_mat = Matrix::<STATE_DIM, STATE_DIM>::identity();

        // dPos/dVel = I * dt
        f_mat.set(IDX_POS, IDX_VEL, dt);
        f_mat.set(IDX_POS + 1, IDX_VEL + 1, dt);
        f_mat.set(IDX_POS + 2, IDX_VEL + 2, dt);

        // dVel/dAtt = -[R * a]x * dt
        // accel_earth_scalar is R * a (approximately, assuming a is accel_corr)
        // Note: accel_net_scalar includes gravity, but gravity is not affected by attitude error (it's in nav frame).
        // However, the linearization is of R * a_body.
        let rot_accel_skew = accel_earth_scalar.skew_symmetric();

        // F[IDX_VEL][IDX_ATT] = -rot_accel_skew * dt
        // We need to copy this 3x3 block into F
        for r in 0..3 {
            for c in 0..3 {
                let val = -rot_accel_skew.get(r, c) * dt;
                f_mat.set(IDX_VEL + r, IDX_ATT + c, val);
            }
        }

        // dVel/dAccelBias = -R * dt
        let r_mat = self.quat.to_rotation_matrix();
        for r in 0..3 {
            for c in 0..3 {
                let val = -r_mat.get(r, c) * dt;
                f_mat.set(IDX_VEL + r, IDX_AB + c, val);
            }
        }

        // dAtt/dGyroBias = -R * dt
        // Assuming earth frame attitude error state
        for r in 0..3 {
            for c in 0..3 {
                let val = -r_mat.get(r, c) * dt;
                f_mat.set(IDX_ATT + r, IDX_GB + c, val);
            }
        }

        // Q (Process Noise)
        let mut q_noise = Matrix::<STATE_DIM, STATE_DIM>::zero();
        // Populate Q diagonal
        for i in 0..3 {
            q_noise.set(IDX_POS + i, IDX_POS + i, 0.001);
        } // Small pos noise
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
        let fp = f_mat.mat_mul(&self.p_cov);
        let fpft = fp.mat_mul(&f_mat.t());
        self.p_cov = fpft.add(&q_noise);
        self.p_cov.make_symmetric();
    }
}
