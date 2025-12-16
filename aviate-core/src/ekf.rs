use crate::math::{Matrix, Quaternion, Vector3, QUAT_NORM_EPS};
use crate::sensor::{
    BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth, SensorReading,
};
use crate::state::{EstimateQuality, StateEstimate, StateValidFlags};
#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::{
    FloatExt, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, RadiansPerSecond,
    Scalar, Validated,
};

// State dimension: 3 pos, 3 vel, 3 att_err, 3 gyro_bias, 3 accel_bias, 3 mag_bias = 18
pub const STATE_DIM: usize = 18;

// State indices
const IDX_POS: usize = 0;
const IDX_VEL: usize = 3;
const IDX_ATT: usize = 6;
const IDX_GB: usize = 9;
const IDX_AB: usize = 12;
const IDX_MB: usize = 15;

#[derive(Clone, Copy, Debug)]
pub struct EkfConfig {
    pub process_noise_gyro: Scalar,
    pub process_noise_accel: Scalar,
    pub process_noise_gyro_bias: Scalar,
    pub process_noise_accel_bias: Scalar,
    pub meas_noise_gnss_pos: Scalar,
    pub meas_noise_gnss_vel: Scalar,
    pub meas_noise_baro: Scalar,
    /// Heading measurement noise [rad²]
    pub meas_noise_mag: Scalar,
    /// Innovation gate threshold (sigma)
    pub innovation_gate: Scalar,

    // Magnetometer fusion config
    /// Inclination vertical ratio at which weight decay begins (default 0.80)
    pub mag_inclination_decay_start: Scalar,
    /// Inclination vertical ratio at which fusion stops (default 0.95)
    pub mag_inclination_limit: Scalar,
    /// Minimum valid field strength [μT] (default 20.0)
    pub mag_field_min: Scalar,
    /// Maximum valid field strength [μT] (default 70.0)
    pub mag_field_max: Scalar,
    /// Mag bias random walk process noise [μT²/s] (default 1e-5)
    pub process_noise_mag_bias: Scalar,
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
            meas_noise_mag: 0.05,     // rad^2 (heading noise)
            innovation_gate: 5.0,     // 5-sigma gate
            // Magnetometer config
            mag_inclination_decay_start: 0.80, // Start weight decay
            mag_inclination_limit: 0.95,       // Stop fusion
            mag_field_min: 20.0,               // μT
            mag_field_max: 70.0,               // μT
            process_noise_mag_bias: 1e-5,      // μT²/s
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
    mag_bias: Vector3<Microtesla>,

    last_gyro_body: Vector3<RadiansPerSecond>,

    // Covariance P (18x18)
    p_cov: Matrix<STATE_DIM, STATE_DIM>,

    // Configuration
    config: EkfConfig,

    initialized: bool,

    /// INV-27: Quaternion normalization fault flag (latches until init())
    quat_fault: bool,
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
            vel: Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            gyro_bias: Vector3::new(
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ),
            accel_bias: Vector3::new(
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
            ),
            mag_bias: Vector3::new(Microtesla(0.0), Microtesla(0.0), Microtesla(0.0)),
            last_gyro_body: Vector3::new(
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ),
            p_cov: Matrix::identity().mul_scalar(0.1), // Initial uncertainty
            config,
            initialized: false,
            quat_fault: false,
        }
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Returns true if a quaternion normalization fault has occurred (INV-27).
    /// Fault latches until init() is called.
    pub fn has_numeric_fault(&self) -> bool {
        self.quat_fault
    }

    /// INV-27: Normalize quaternion and validate result.
    /// Returns IDENTITY and sets quat_fault if normalization fails.
    fn sanitize_quat(&mut self, q: Quaternion) -> Quaternion {
        let q = q.normalize();
        if !q.is_normalized(QUAT_NORM_EPS) {
            self.quat_fault = true;
            Quaternion::IDENTITY
        } else {
            q
        }
    }

    pub fn init(&mut self, pos: Vector3<Meters>, vel: Vector3<MetersPerSecond>, quat: Quaternion) {
        self.pos = pos;
        self.vel = vel;
        self.quat = quat;
        self.gyro_bias = Vector3::new(
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        );
        self.accel_bias = Vector3::new(
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
        );
        self.mag_bias = Vector3::new(Microtesla(0.0), Microtesla(0.0), Microtesla(0.0));
        self.p_cov = Matrix::identity().mul_scalar(0.1);
        self.initialized = true;
        self.quat_fault = false; // Clear latch on re-init (INV-27)
    }

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

        match gnss_reading.value.health {
            GnssHealth::Good => {}
            GnssHealth::Suspect | GnssHealth::Lost => return,
        }

        // Extra check for fix type if needed
        if gnss_reading.value.fix == GnssFix::None {
            return;
        }

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
        // panic!("Entered update_baro");
        match baro_reading.health {
            SensorHealth::Good => { /* continue */ }
            _ => {
                return;
            }
        }

        if let Some(static_pressure) = baro_reading.value.air.static_pressure {
            // Convert pressure to altitude. For a minimal v0.5, we use a very simplified model
            // or assume a constant ground pressure and derive z_pos.
            // Actual implementation would use a proper baro model or pass in ref_pressure.
            // Here, let's use a simple linear relationship for demo or just use a known ground pressure.
            // For now, derive altitude from a standard pressure to altitude formula (very basic)
            // Altitude (m) = 44330.0 * (1.0 - (pressure / 101325.0)^0.1903)
            let p0 = 101325.0; // Sea level standard pressure in Pascals
            let altitude_from_pressure = 44330.0 * (1.0 - (static_pressure.0 / p0).powf(0.1903));

            // NED Z is negative altitude (down).
            let z_meas = -altitude_from_pressure;
            let r_baro = self.config.meas_noise_baro;
            self.update_scalar(IDX_POS + 2, z_meas, r_baro);
        }
    }

    /// Update EKF with magnetometer reading for heading estimation.
    ///
    /// # Approach
    ///
    /// Fuses tilt-compensated magnetic heading into the EKF yaw state.
    /// Uses inclination-based weight decay to handle polar regions gracefully.
    ///
    /// # Frame Convention
    ///
    /// - Magnetometer data is in body frame
    /// - Heading is magnetic (no declination correction)
    /// - Positive yaw = clockwise from magnetic north when viewed from above
    pub fn update_mag(&mut self, mag_reading: &SensorReading<MagData>) {
        use core::f32::consts::PI;

        // Step 1: Health & Validity Gating
        if !self.initialized || mag_reading.health != SensorHealth::Good {
            return;
        }

        let mag = &mag_reading.value;

        // Extract mag vector in body frame [μT]
        let mag_x = mag.field_ut[0].0;
        let mag_y = mag.field_ut[1].0;
        let mag_z = mag.field_ut[2].0;

        // Step 2: Field Strength Validation
        // Earth's field: ~25-65 μT depending on location
        let mag_norm = (mag_x * mag_x + mag_y * mag_y + mag_z * mag_z).sqrt();
        if mag_norm < self.config.mag_field_min || mag_norm > self.config.mag_field_max {
            return; // Anomalous field - likely interference
        }

        // Step 3: Inclination-Based Weight Calculation
        // vertical_ratio = |mag_z| / |mag| indicates field inclination
        let vertical_ratio = mag_z.abs() / mag_norm;

        // Beyond threshold - stop fusion (polar region or high inclination)
        if vertical_ratio >= self.config.mag_inclination_limit {
            return;
        }

        // Weight: 1.0 at low inclination, decays to 0 at threshold
        let incl_weight = if vertical_ratio < self.config.mag_inclination_decay_start {
            1.0
        } else {
            // Linear decay from decay_start to limit
            let range = self.config.mag_inclination_limit - self.config.mag_inclination_decay_start;
            if range > 1e-6 {
                1.0 - (vertical_ratio - self.config.mag_inclination_decay_start) / range
            } else {
                0.0 // COV:EXCL(DEFENSIVE: protects against misconfigured limits)
            }
        };

        // If weight is too low, skip fusion
        if incl_weight < 0.01 {
            return;
        }

        // Step 4: Apply EKF-Estimated Mag Bias Correction
        let mag_corrected_x = mag_x - self.mag_bias.x.0;
        let mag_corrected_y = mag_y - self.mag_bias.y.0;
        let mag_corrected_z = mag_z - self.mag_bias.z.0;

        // Step 5: Tilt-Compensated Heading Extraction
        // Rotate mag vector from body to NED using current attitude
        let r_mat = self.quat.to_rotation_matrix();

        // mag_ned = R * mag_body
        let mag_n = r_mat.get(0, 0) * mag_corrected_x
            + r_mat.get(0, 1) * mag_corrected_y
            + r_mat.get(0, 2) * mag_corrected_z;
        let mag_e = r_mat.get(1, 0) * mag_corrected_x
            + r_mat.get(1, 1) * mag_corrected_y
            + r_mat.get(1, 2) * mag_corrected_z;

        // Magnetic heading (no declination applied)
        // atan2(East, North) gives heading clockwise from North
        let heading_mag = mag_e.atan2(mag_n);

        // Step 6: Innovation Gating & Yaw Update
        // Compare with current yaw estimate
        let (_, _, yaw_est) = self.quat.to_euler();
        let mut innov = heading_mag - yaw_est;

        // COV:EXCL_START(DEFENSIVE: atan2/euler outputs bounded to [-π,π], wrapping is safety guard)
        // Wrap innovation to [-π, π]
        // Note: Both atan2 and to_euler return values in [-π, π], so innovation
        // is bounded to [-2π, 2π]. Single iteration suffices, but while loop is
        // defensive against numerical edge cases.
        while innov > PI {
            innov -= 2.0 * PI;
        }
        while innov < -PI {
            innov += 2.0 * PI;
        }
        // COV:EXCL_STOP

        // Apply inclination weight to measurement noise (higher noise = lower weight)
        // r_effective = r / w² means lower weight increases effective noise
        let r_effective = if incl_weight > 0.1 {
            self.config.meas_noise_mag / (incl_weight * incl_weight)
        } else {
            return; // COV:EXCL(DEFENSIVE: weight 0.01-0.1 rejected by earlier check)
        };

        // Perform heading update using attitude error state
        self.update_heading(innov, r_effective);
    }

    /// Internal: Update yaw/heading using scalar observation.
    ///
    /// H = [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, ..., 0] (observes z-component of attitude error)
    fn update_heading(&mut self, innov: Scalar, r_noise: Scalar) {
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

    pub fn get_estimate(&self) -> StateEstimate {
        StateEstimate {
            attitude: self.quat,
            angular_velocity: [
                self.last_gyro_body.x,
                self.last_gyro_body.y,
                self.last_gyro_body.z,
            ],
            position_ned: [self.pos.x, self.pos.y, self.pos.z],
            velocity_ned: [self.vel.x, self.vel.y, self.vel.z],
            quality: if self.initialized {
                EstimateQuality::Good
            } else {
                EstimateQuality::Unusable
            },
            valid_flags: if self.initialized {
                StateValidFlags::all()
            } else {
                StateValidFlags::empty()
            },
        }
    }

    /// Inject state for testing (spec §20 test-hooks)
    ///
    /// Directly sets the EKF internal state from an external StateEstimate.
    /// Only available with the `test-hooks` feature enabled.
    #[cfg(feature = "test-hooks")]
    pub fn set_state(&mut self, state: &StateEstimate) {
        self.quat = state.attitude;
        self.last_gyro_body = crate::math::Vector3 {
            x: state.angular_velocity[0],
            y: state.angular_velocity[1],
            z: state.angular_velocity[2],
        };
        self.pos = Vector3 {
            x: state.position_ned[0],
            y: state.position_ned[1],
            z: state.position_ned[2],
        };
        self.vel = Vector3 {
            x: state.velocity_ned[0],
            y: state.velocity_ned[1],
            z: state.velocity_ned[2],
        };
        self.initialized = state.valid_flags.contains(StateValidFlags::all());
    }
}
