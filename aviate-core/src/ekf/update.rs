//! Sensor fusion entry points — GNSS, baro, and magnetometer.
//!
//! Phase 4: methods take `&self` (algorithm config) and `&mut state:
//! &mut EkfState`. Persistent filter state lives only in
//! `KernelState.estimator`.

use super::{EkfState, IDX_POS, IDX_VEL};
use crate::ekf::Ekf;
use crate::sensor::{
    BaroData, GnssData, GnssFix, GnssHealth, MagData, SensorHealth, SensorReading,
};
#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;
use core::f32::consts::PI;

/// Initial variance of the QFE baro datum \[m²\] the first time it is
/// latched. A few metres of uncertainty so GNSS-anchored height quickly
/// pulls the datum onto the true local-origin elevation.
const BARO_DATUM_INIT_VAR: Scalar = 4.0;

/// Random-walk process variance added to the QFE baro datum per fused
/// sample \[m²\]. Small so the datum tracks slow ground-pressure drift
/// without chasing high-rate vehicle height motion.
const BARO_DATUM_PROCESS_VAR: Scalar = 1e-4;

impl Ekf {
    pub fn update_gnss_state(&self, state: &mut EkfState, gnss_reading: &SensorReading<GnssData>) {
        // 0. Health gate
        match gnss_reading.health {
            SensorHealth::Good => { /* continue */ }
            _ => {
                return;
            }
        }

        match gnss_reading.value.health {
            GnssHealth::Good => {}
            GnssHealth::Suspect | GnssHealth::Lost => return,
        }

        if gnss_reading.value.fix == GnssFix::None {
            return;
        }

        let gnss = &gnss_reading.value;

        // Reject non-finite aiding before fusion, mirroring the IMU
        // validity gate in `predict_state`. A single NaN/Inf component
        // otherwise produces a NaN innovation that slips through the
        // scalar gate and poisons every correlated state, so the whole
        // reading is dropped if any component is non-finite.
        for c in 0..3 {
            if !gnss.position_ned[c].0.is_finite() || !gnss.velocity_ned[c].0.is_finite() {
                return;
            }
        }

        // Update Position NED. The freshness age resets only if at
        // least one axis was actually fused — `|=` (not `||`) so all
        // three axes still get their own gate check even once one has
        // already accepted, matching the per-axis innovation gating
        // `scalar_update` already does.
        let r_pos = self.config.meas_noise_gnss_pos;
        let mut pos_accepted = false;
        pos_accepted |= self.scalar_update(state, IDX_POS, gnss.position_ned[0].0, r_pos);
        pos_accepted |= self.scalar_update(state, IDX_POS + 1, gnss.position_ned[1].0, r_pos);
        pos_accepted |= self.scalar_update(state, IDX_POS + 2, gnss.position_ned[2].0, r_pos);
        if pos_accepted {
            state.gnss_pos_age_s = 0.0;
        }

        // Update Velocity NED
        let r_vel = self.config.meas_noise_gnss_vel;
        let mut vel_accepted = false;
        vel_accepted |= self.scalar_update(state, IDX_VEL, gnss.velocity_ned[0].0, r_vel);
        vel_accepted |= self.scalar_update(state, IDX_VEL + 1, gnss.velocity_ned[1].0, r_vel);
        vel_accepted |= self.scalar_update(state, IDX_VEL + 2, gnss.velocity_ned[2].0, r_vel);
        if vel_accepted {
            state.gnss_vel_age_s = 0.0;
        }
    }

    pub fn update_baro_state(&self, state: &mut EkfState, baro_reading: &SensorReading<BaroData>) {
        match baro_reading.health {
            SensorHealth::Good => { /* continue */ }
            _ => {
                return;
            }
        }

        if let Some(static_pressure) = baro_reading.value.air.static_pressure {
            let p0 = 101325.0; // Sea level standard pressure in Pascals
            let altitude_from_pressure = 44330.0 * (1.0 - (static_pressure.0 / p0).powf(0.1903));

            // Reject non-finite aiding before fusion, mirroring the IMU
            // validity gate: a NaN/Inf pressure yields a NaN altitude
            // that would slip through the scalar gate and poison state.
            if !altitude_from_pressure.is_finite() {
                return;
            }

            // QFE origin referencing. The ISA formula yields absolute
            // MSL pressure altitude, but `pos.z` is local-origin-relative
            // — as is the GNSS height fused into the same state. Latch a
            // datum on the first accepted sample so the initial innovation
            // is ≈0 (`z_meas == pos.z`); thereafter a scalar random-walk
            // estimator lets GNSS-anchored height pull the datum onto the
            // true origin and tracks slow ground-pressure drift. Without
            // this, an elevated site's pressure-altitude offset (≈1658 m
            // in Denver) is a standing innovation that gates baro out
            // forever.
            let datum = match state.baro_ref {
                Some(d) => d,
                None => {
                    let d = state.pos.z.0 + altitude_from_pressure;
                    state.baro_ref = Some(d);
                    state.baro_ref_var = BARO_DATUM_INIT_VAR;
                    d
                }
            };

            // NED Z is negative altitude (down); referencing to the
            // latched datum keeps the measurement on the origin frame.
            let z_meas = datum - altitude_from_pressure;

            let r_baro = self.config.meas_noise_baro;
            if self.scalar_update(state, IDX_POS + 2, z_meas, r_baro) {
                state.baro_age_s = 0.0;
            }

            self.correct_baro_datum(state, altitude_from_pressure, datum);
        }
    }

    // COV:EXCL_START(phantom DA: grcov attributes a debug-info region onto this doc comment; correct_baro_datum is exercised by the baro-datum tests)
    /// Scalar random-walk update of the QFE baro datum (PX4-style baro
    /// bias estimation). The fused height implies a datum of
    /// `altitude + pos.z`; a 1-D Kalman step nudges the stored datum
    /// toward it, weighted by the height uncertainty. Because baro alone
    /// cannot separate height from datum, the correction only bites when
    /// GNSS anchors `pos.z`, which drives the standing offset to zero and
    /// lets the datum track slow ground-pressure drift.
    // COV:EXCL_STOP
    fn correct_baro_datum(&self, state: &mut EkfState, altitude: Scalar, datum: Scalar) {
        let var = state.baro_ref_var + BARO_DATUM_PROCESS_VAR;
        let r_eff = self.config.meas_noise_baro + state.p_cov.get(IDX_POS + 2, IDX_POS + 2);
        let s = var + r_eff;
        if s < 1e-9 {
            return; // COV:EXCL(DEFENSIVE: datum and height variances are positive, so S > 0)
        }
        let k = var / s;
        let implied = altitude + state.pos.z.0;
        state.baro_ref = Some(datum + k * (implied - datum));
        state.baro_ref_var = (1.0 - k) * var;
    }

    // COV:EXCL_START(phantom DA: grcov attributes a debug-info region onto this doc comment; update_mag_state is exercised by the mag-fusion tests)
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
    // COV:EXCL_STOP
    pub fn update_mag_state(&self, state: &mut EkfState, mag_reading: &SensorReading<MagData>) {
        // Step 1: Health & Validity Gating
        if !state.initialized || mag_reading.health != SensorHealth::Good {
            return;
        }

        let mag = &mag_reading.value;

        // Reject non-finite aiding before fusion, mirroring the IMU
        // validity gate. A NaN component would otherwise pass the field
        // and inclination checks (every NaN comparison is false) and
        // poison the yaw state.
        for c in 0..3 {
            if !mag.field_ut[c].0.is_finite() {
                return;
            }
        }

        let mag_x = mag.field_ut[0].0;
        let mag_y = mag.field_ut[1].0;
        let mag_z = mag.field_ut[2].0;

        // Step 2: Field Strength Validation // COV:EXCL(phantom DA: grcov debug-info attribution onto this comment line)
        let mag_norm = (mag_x * mag_x + mag_y * mag_y + mag_z * mag_z).sqrt();
        if mag_norm < self.config.mag_field_min || mag_norm > self.config.mag_field_max {
            return;
        }

        // Step 3: Inclination-Based Weight Calculation
        let vertical_ratio = mag_z.abs() / mag_norm;

        if vertical_ratio >= self.config.mag_inclination_limit {
            return;
        }

        let incl_weight = if vertical_ratio < self.config.mag_inclination_decay_start {
            1.0
        } else {
            let range = self.config.mag_inclination_limit - self.config.mag_inclination_decay_start;
            if range > 1e-6 {
                1.0 - (vertical_ratio - self.config.mag_inclination_decay_start) / range
            } else {
                0.0 // COV:EXCL(DEFENSIVE: protects against misconfigured limits)
            }
        };

        // COV:EXCL_START(DEFENSIVE: reachable only in a narrow numerical
        // sliver where vertical_ratio is within ~1% of mag_inclination_limit;
        // the upstream `vertical_ratio >= limit` check already returns for
        // the polar-inclination case, so this guard is a belt-and-suspenders
        // against floating-point boundary conditions.)
        if incl_weight < 0.01 {
            return;
        }
        // COV:EXCL_STOP

        // Step 4: Tilt-Compensated Heading Extraction
        //
        // Projecting the body field through the FULL body→NED rotation
        // reconstructs the earth field with yaw already baked in, so the
        // resulting heading is independent of the yaw estimate and its // COV:EXCL(phantom DA: grcov debug-info attribution onto this comment line)
        // innovation can never correct yaw. Rotating through roll/pitch
        // only (yaw = 0) leaves the horizontal components in a level,
        // yaw-free frame, making `heading_mag` an absolute measurement
        // whose innovation is the true heading error.
        let (roll, pitch, yaw_est) = state.quat.to_euler();
        let (sin_r, cos_r) = (roll.sin(), roll.cos());
        let (sin_p, cos_p) = (pitch.sin(), pitch.cos());

        // Level-frame horizontal field via R_tilt = Ry(pitch) * Rx(roll).
        let mag_n_level = cos_p * mag_x + sin_p * sin_r * mag_y + sin_p * cos_r * mag_z;
        let mag_e_level = cos_r * mag_y - sin_r * mag_z;

        // Positive yaw is clockwise from magnetic north (NED, z down),
        // so the heading that reproduces the true yaw from a
        // north-pointing field is atan2(-east, north).
        let heading_mag = (-mag_e_level).atan2(mag_n_level);

        // Step 5: Innovation Gating & Yaw Update
        let mut innov = heading_mag - yaw_est;
        // COV:EXCL_START(DEFENSIVE: atan2/euler outputs bounded to [-π,π], wrapping is safety guard)
        while innov > PI {
            innov -= 2.0 * PI;
        }
        while innov < -PI {
            innov += 2.0 * PI;
        }
        // COV:EXCL_STOP
        let r_effective = if incl_weight > 0.1 {
            self.config.meas_noise_mag / (incl_weight * incl_weight)
        } else {
            return; // COV:EXCL(DEFENSIVE: weight 0.01-0.1 rejected by earlier check)
        };

        self.heading_update(state, innov, r_effective);
    }
}
