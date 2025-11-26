use crate::math::{Matrix, Quaternion, Vector3};
use crate::types::{Scalar, Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond, Validated, FloatExt};
use crate::state::{StateEstimate, EstimateQuality, StateValidFlags};
use crate::sensor::{ImuData, GnssData, SensorReading, SensorHealth, GnssFix, BaroData, MagData};

// State dimension: 3 pos, 3 vel, 3 att_err, 3 gyro_bias, 3 accel_bias = 15
pub const STATE_DIM: usize = 15;

// State indices
const IDX_POS: usize = 0;
const IDX_VEL: usize = 3;
const IDX_ATT: usize = 6;
const IDX_GB: usize = 9;
const IDX_AB: usize = 12;

#[derive(Clone, Copy, Debug)]
pub struct EkfConfig {
    pub process_noise_gyro: Scalar,
    pub process_noise_accel: Scalar,
    pub process_noise_gyro_bias: Scalar,
    pub process_noise_accel_bias: Scalar,
    pub meas_noise_gnss_pos: Scalar,
    pub meas_noise_gnss_vel: Scalar,
    pub meas_noise_baro: Scalar,
    pub meas_noise_mag: Scalar,
    // Innovation gate threshold (sigma)
    pub innovation_gate: Scalar,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            process_noise_gyro: 1e-3,
            process_noise_accel: 1e-2,
            process_noise_gyro_bias: 1e-4,
            process_noise_accel_bias: 1e-4,
            meas_noise_gnss_pos: 0.5, // m^2
            meas_noise_gnss_vel: 0.1, // (m/s)^2
            meas_noise_baro: 2.0,     // m^2
            meas_noise_mag: 0.01,     // uT^2 (very rough guess)
            innovation_gate: 5.0,     // 5-sigma gate
        }
    }
}

pub struct Ekf {
    // Core state
    quat: Quaternion,
    pos: Vector3<Meters>,
    vel: Vector3<MetersPerSecond>,
    gyro_bias: Vector3<RadiansPerSecond>,
    accel_bias: Vector3<MetersPerSecondSquared>,
    
    last_gyro_body: Vector3<RadiansPerSecond>,

    // Covariance P (15x15)
    p_cov: Matrix<STATE_DIM, STATE_DIM>,

    // Configuration
    config: EkfConfig,
    
    initialized: bool,
}

impl Default for Ekf {
    fn default() -> Self {
        Self::new(EkfConfig::default())
    }
}

impl Ekf {
    pub fn new(config: EkfConfig) -> Self {
        Self {
            quat: Quaternion::IDENTITY,
            pos: Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            vel: Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            gyro_bias: Vector3::new(RadiansPerSecond(0.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)),
            accel_bias: Vector3::new(MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0)),
            last_gyro_body: Vector3::new(RadiansPerSecond(0.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)),
            p_cov: Matrix::identity().mul_scalar(0.1), // Initial uncertainty
            config,
            initialized: false,
        }
    }
    
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn init(&mut self, pos: Vector3<Meters>, vel: Vector3<MetersPerSecond>, quat: Quaternion) {
        self.pos = pos;
        self.vel = vel;
        self.quat = quat;
        self.gyro_bias = Vector3::new(RadiansPerSecond(0.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0));
        self.accel_bias = Vector3::new(MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0));
        self.p_cov = Matrix::identity().mul_scalar(0.1);
        self.initialized = true;
    }

    pub fn predict(&mut self, imu: &ImuData, dt: Scalar) {
        if !self.initialized { return; }
        if !dt.is_finite() || dt <= 0.0 { return; }

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
        let accel_corr_scalar = Vector3 { x: accel_corr.x.0, y: accel_corr.y.0, z: accel_corr.z.0 };
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
        let delta_angle = Vector3 { x: gyro_corr.x.0, y: gyro_corr.y.0, z: gyro_corr.z.0 };
        let angle_mag = (delta_angle.x*delta_angle.x + delta_angle.y*delta_angle.y + delta_angle.z*delta_angle.z).sqrt();
        let dq = if angle_mag > 1e-6 {
             Quaternion::from_axis_angle(Vector3::new(delta_angle.x/angle_mag, delta_angle.y/angle_mag, delta_angle.z/angle_mag), angle_mag * dt)
        } else {
             Quaternion::IDENTITY
        };
        self.quat = self.quat.mul(&dq).normalize();


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
        
        // Let's build F explicitly as Matrix<15,15>.
        let mut f_mat = Matrix::<15, 15>::identity();
        
        // dPos/dVel = I * dt
        f_mat.set(IDX_POS, IDX_VEL, dt);
        f_mat.set(IDX_POS+1, IDX_VEL+1, dt);
        f_mat.set(IDX_POS+2, IDX_VEL+2, dt);
        
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
        let mut q_noise = Matrix::<15, 15>::zero();
        // Populate Q diagonal
        for i in 0..3 { q_noise.set(IDX_POS+i, IDX_POS+i, 0.001); } // Small pos noise
        for i in 0..3 { q_noise.set(IDX_VEL+i, IDX_VEL+i, self.config.process_noise_accel * dt * dt); }
        for i in 0..3 { q_noise.set(IDX_ATT+i, IDX_ATT+i, self.config.process_noise_gyro * dt * dt); }
        for i in 0..3 { q_noise.set(IDX_GB+i, IDX_GB+i, self.config.process_noise_gyro_bias * dt); }
        for i in 0..3 { q_noise.set(IDX_AB+i, IDX_AB+i, self.config.process_noise_accel_bias * dt); }

        // P = F * P * F' + Q
        let fp = f_mat.mat_mul(&self.p_cov);
        let fpft = fp.mat_mul(&f_mat.t());
        self.p_cov = fpft.add(&q_noise);
    }

    pub fn update_gnss(&mut self, gnss_reading: &SensorReading<GnssData>) {
        // 0. Health gate
        match gnss_reading.health {
            SensorHealth::Good => { /* continue */ }
            _ => {
                // For now: do nothing. Spec says Suspect is diagnostics-only.
                // In a real implementation, we might log this or switch modes.
                return;
            }
        }
        
        // Extra check for fix type if needed
        if gnss_reading.value.fix == GnssFix::None { return; }

        let gnss = &gnss_reading.value;

        // Update Position NED
        let r_pos = self.config.meas_noise_gnss_pos;

        self.update_scalar(IDX_POS, gnss.position_ned[0].0, r_pos);
        self.update_scalar(IDX_POS + 1, gnss.position_ned[1].0, r_pos);
        self.update_scalar(IDX_POS + 2, gnss.position_ned[2].0, r_pos);
        
        // Update Velocity NED
        let r_vel = self.config.meas_noise_gnss_vel;
        self.update_scalar(IDX_VEL, gnss.velocity_ned[0].0, r_vel);
        self.update_scalar(IDX_VEL + 1, gnss.velocity_ned[1].0, r_vel);
        self.update_scalar(IDX_VEL + 2, gnss.velocity_ned[2].0, r_vel);
    }

    pub fn update_baro(&mut self, baro_reading: &SensorReading<BaroData>) {
        match baro_reading.health {
            SensorHealth::Good => { /* continue */ }
            _ => { return; }
        }
        
        if let Some(_pressure) = baro_reading.value.air.static_pressure {
             // TODO: Convert pressure to altitude using standard atmosphere or reference
             if let Some(alt) = baro_reading.value.altitude {
                 // NED Z is negative altitude (down).
                 let z_meas = -alt.0;
                 let r_baro = self.config.meas_noise_baro;
                 self.update_scalar(IDX_POS + 2, z_meas, r_baro);
             }
        }
    }
    
    pub fn update_mag(&mut self, mag_reading: &SensorReading<MagData>) {
        match mag_reading.health {
            SensorHealth::Good => { /* continue */ }
            _ => { return; }
        }
        
        // Placeholder for mag update
        let _mag = &mag_reading.value;
        // Real implementation would involve estimating heading/yaw from 3D mag + tilt
    }

    fn update_scalar(&mut self, state_idx: usize, meas: Scalar, r_noise: Scalar) {
        // Standard EKF scalar update
        // H = [0, ..., 1, ... 0] at state_idx
        
        // 1. Innovation
        let pred = match state_idx {
            0 => self.pos.x.0,
            1 => self.pos.y.0,
            2 => self.pos.z.0,
            3 => self.vel.x.0,
            4 => self.vel.y.0,
            5 => self.vel.z.0,
            _ => return, // Should not happen for this simple usage
        };
        let innov = meas - pred;
        
        // Innovation Gating
        let s = self.p_cov.get(state_idx, state_idx) + r_noise;
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
        self.pos.y = Meters(self.pos.y.0 + k_vector[IDX_POS+1] * innov);
        self.pos.z = Meters(self.pos.z.0 + k_vector[IDX_POS+2] * innov);
        
        self.vel.x = MetersPerSecond(self.vel.x.0 + k_vector[IDX_VEL] * innov);
        self.vel.y = MetersPerSecond(self.vel.y.0 + k_vector[IDX_VEL+1] * innov);
        self.vel.z = MetersPerSecond(self.vel.z.0 + k_vector[IDX_VEL+2] * innov);
        
        // Update other states (biases, etc.)
        self.gyro_bias.x = RadiansPerSecond(self.gyro_bias.x.0 + k_vector[IDX_GB] * innov);
        self.gyro_bias.y = RadiansPerSecond(self.gyro_bias.y.0 + k_vector[IDX_GB+1] * innov);
        self.gyro_bias.z = RadiansPerSecond(self.gyro_bias.z.0 + k_vector[IDX_GB+2] * innov);
        
        self.accel_bias.x = MetersPerSecondSquared(self.accel_bias.x.0 + k_vector[IDX_AB] * innov);
        self.accel_bias.y = MetersPerSecondSquared(self.accel_bias.y.0 + k_vector[IDX_AB+1] * innov);
        self.accel_bias.z = MetersPerSecondSquared(self.accel_bias.z.0 + k_vector[IDX_AB+2] * innov);
        
        // Attitude update (linearized error)
        let d_ang = Vector3::new(
            k_vector[IDX_ATT] * innov,
            k_vector[IDX_ATT+1] * innov,
            k_vector[IDX_ATT+2] * innov
        );
        // Apply small angle rotation to quaternion
        // Better: use small angle approx dq = [1, dx/2, dy/2, dz/2]
        let dq_small = Quaternion::new(1.0, d_ang.x * 0.5, d_ang.y * 0.5, d_ang.z * 0.5).normalize();
        self.quat = self.quat.mul(&dq_small).normalize();

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
    }

    pub fn get_estimate(&self) -> StateEstimate {
        StateEstimate {
            attitude: self.quat,
            angular_velocity: [
                self.last_gyro_body.x, 
                self.last_gyro_body.y, 
                self.last_gyro_body.z
            ],
            position_ned: [
                self.pos.x,
                self.pos.y,
                self.pos.z,
            ],
            velocity_ned: [
                self.vel.x,
                self.vel.y,
                self.vel.z,
            ],
            quality: if self.initialized { EstimateQuality::Good } else { EstimateQuality::Unusable },
            valid_flags: if self.initialized { StateValidFlags::all() } else { StateValidFlags::empty() },
        }
    }
}
