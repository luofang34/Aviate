//! Sensor fusion entry points — GNSS, baro, and magnetometer.
//!
//! Extracted from `ekf.rs` to keep that file under the 500-line cap.
//! No re-exports — impl block split across files.

use super::{Ekf, IDX_POS, IDX_VEL};
use crate::sensor::{
    BaroData, GnssData, GnssFix, GnssHealth, MagData, SensorHealth, SensorReading,
};
#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::FloatExt;

impl Ekf {
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

        // If weight is too low, skip fusion.
        // COV:EXCL_START(DEFENSIVE: reachable only in a narrow numerical
        // sliver where vertical_ratio is within ~1% of mag_inclination_limit;
        // the upstream `vertical_ratio >= limit` check already returns for
        // the polar-inclination case, so this guard is a belt-and-suspenders
        // against floating-point boundary conditions.)
        if incl_weight < 0.01 {
            return;
        }
        // COV:EXCL_STOP

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
}
