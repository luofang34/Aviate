//! Air-data aiding: true airspeed, zero sideslip and two wind states.
//!
//! A fixed wing moves through the air along its nose. The filter uses
//! that as two measurements of the velocity relative to the air mass:
//! its magnitude is the true airspeed, and its component along the body
//! lateral axis is zero. The air mass moves over the ground with a wind
//! that the filter carries as two horizontal states.
//!
//! These measurements bound the velocity states without a position fix,
//! and through the coupling of velocity and attitude they keep roll and
//! pitch observable: a tilt error turns gravity into a velocity error
//! that grows with time, and the air measurements see it.
//!
//! The wind states live beside the 15-state covariance, with their own
//! block and a cross block. A filter without this aiding never touches
//! them, so its arithmetic is that of the 15-state filter.

use super::scalar::COV_VAR_FLOOR;
use super::validity::AGE_SATURATION_S;
use super::{Ekf, EkfState, IDX_AB, IDX_ATT, IDX_GB, IDX_POS, IDX_VEL, STATE_DIM};
use crate::math::{Matrix, Quaternion, Vector3};
use crate::sensor::SensorSet;
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond, Scalar};

/// Variance floor of each wind state \[(m/s)²\].
const WIND_VAR_FLOOR: Scalar = 1e-6;

/// Tuning of the air-data aiding.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirAidingConfig {
    /// Variance of the true-airspeed measurement \[(m/s)²\].
    pub meas_noise_tas: Scalar,
    /// Standard deviation of the sideslip that the vehicle flies with
    /// \[rad\]. The lateral air velocity is zero to within this angle
    /// times the airspeed.
    pub sideslip_sigma: Scalar,
    /// Variance of each wind state before any fix has shown the wind
    /// \[(m/s)²\].
    pub wind_prior_var: Scalar,
    /// Growth of each wind variance \[(m/s)² per second\].
    pub process_noise_wind: Scalar,
    /// True airspeed \[m/s\] below which the vehicle is not flying on its
    /// wing and nothing is fused.
    pub min_true_airspeed: Scalar,
}

/// Wind states and their covariance blocks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirAidingState {
    /// Wind over the ground, north and east \[m/s\].
    pub wind_ne: [Scalar; 2],
    /// Covariance of the wind states.
    pub p_ww: [[Scalar; 2]; 2],
    /// Covariance of each filter state with each wind state.
    pub p_xw: [[Scalar; 2]; STATE_DIM],
    /// Whether the wind block takes part in the filter.
    pub active: bool,
    /// Time since the last accepted airspeed fusion \[s\].
    pub age_s: Scalar,
}

impl Default for AirAidingState {
    fn default() -> Self {
        Self {
            wind_ne: [0.0; 2],
            p_ww: [[0.0; 2]; 2],
            p_xw: [[0.0; 2]; STATE_DIM],
            active: false,
            age_s: AGE_SATURATION_S,
        }
    }
}

/// One scalar measurement of the joint state: its Jacobian on the
/// filter states and on the wind states, its innovation and its noise.
pub(super) struct JointMeasurement {
    pub(super) h_x: [Scalar; STATE_DIM],
    pub(super) h_w: [Scalar; 2],
    pub(super) innovation: Scalar,
    pub(super) noise: Scalar,
}

impl AirAidingState {
    /// Carry the cross block through one covariance prediction.
    ///
    /// The wind is a random walk, so its transition is the identity and
    /// the cross block takes the transition `f` of the filter states.
    pub(super) fn propagate(
        &mut self,
        f: &Matrix<STATE_DIM, STATE_DIM>,
        wind_variance_growth: Scalar,
    ) {
        if !self.active {
            return;
        }
        let before = self.p_xw;
        for (row, cross) in self.p_xw.iter_mut().enumerate() {
            for (wind, value) in cross.iter_mut().enumerate() {
                *value = before
                    .iter()
                    .enumerate()
                    .map(|(column, source)| f.get(row, column) * source[wind])
                    .sum();
            }
        }
        for (axis, row) in self.p_ww.iter_mut().enumerate() {
            row[axis] += wind_variance_growth;
        }
    }

    /// Follow a scalar update that measured filter state `state_idx`
    /// with gain `k_x` and innovation variance `s`.
    ///
    /// The measurement has no wind component, but the wind is correlated
    /// with the measured state, so the wind and both wind blocks move.
    pub(super) fn follow_state_update(
        &mut self,
        state_idx: usize,
        k_x: &[Scalar; STATE_DIM],
        s: Scalar,
        innovation: Scalar,
    ) {
        if !self.active {
            return;
        }
        let measured = self.p_xw[state_idx];
        for (wind, covariance) in self.wind_ne.iter_mut().zip(measured) {
            *wind += covariance / s * innovation;
        }
        for (cross, gain) in self.p_xw.iter_mut().zip(k_x) {
            for (value, covariance) in cross.iter_mut().zip(measured) {
                *value -= gain * covariance;
            }
        }
        for (row, covariance_row) in self.p_ww.iter_mut().zip(measured) {
            for (value, covariance_column) in row.iter_mut().zip(measured) {
                *value -= covariance_row * covariance_column / s;
            }
        }
        self.repair_wind_block();
    }

    /// Keep the wind block symmetric and its variances positive.
    fn repair_wind_block(&mut self) {
        let off_diagonal = 0.5 * (self.p_ww[0][1] + self.p_ww[1][0]);
        self.p_ww[0][1] = off_diagonal;
        self.p_ww[1][0] = off_diagonal;
        for axis in 0..2 {
            self.p_ww[axis][axis] = self.p_ww[axis][axis].max(WIND_VAR_FLOOR);
        }
    }
}

impl Ekf {
    /// Whether this estimator is configured with air-data aiding.
    #[must_use]
    pub const fn has_air_aiding(&self) -> bool {
        self.config.air_aiding.is_some()
    }

    /// Fuse the airspeed sample of this cycle, when the configuration
    /// has air-data aiding and the sample is usable.
    pub(super) fn fuse_air_data(&self, state: &mut EkfState, sensors: &SensorSet, dt: Scalar) {
        let Some(config) = self.config.air_aiding else {
            return;
        };
        if !state.initialized {
            return;
        }
        if !state.air.active {
            state.air = AirAidingState {
                p_ww: [[config.wind_prior_var, 0.0], [0.0, config.wind_prior_var]],
                active: true,
                ..AirAidingState::default()
            };
        }
        state.air.age_s = (state.air.age_s + dt).min(AGE_SATURATION_S);

        let Some(true_airspeed) = sensors
            .primary_air_data()
            .and_then(|air| air.pitot_static())
            .map(|probe| probe.true_mps)
            .filter(|speed| speed.is_finite() && *speed >= config.min_true_airspeed)
        else {
            return;
        };
        if self.fuse_joint(state, &airspeed_measurement(state, true_airspeed, &config)) {
            state.air.age_s = 0.0;
        }
        self.fuse_joint(state, &sideslip_measurement(state, true_airspeed, &config));
    }

    /// One scalar update of the filter states and the wind states.
    /// Returns whether the innovation gate accepted the measurement.
    pub(super) fn fuse_joint(&self, state: &mut EkfState, measurement: &JointMeasurement) -> bool {
        let (h_x, h_w) = (&measurement.h_x, &measurement.h_w);
        let air = &state.air;
        // Covariance of every state with the measurement.
        let mut ph_x = [0.0; STATE_DIM];
        for (row, value) in ph_x.iter_mut().enumerate() {
            let from_states: Scalar = (0..STATE_DIM)
                .map(|column| state.p_cov.get(row, column) * h_x[column])
                .sum();
            *value = from_states + air.p_xw[row][0] * h_w[0] + air.p_xw[row][1] * h_w[1];
        }
        let mut ph_w = [0.0; 2];
        for (wind, value) in ph_w.iter_mut().enumerate() {
            let from_states: Scalar = (0..STATE_DIM)
                .map(|row| air.p_xw[row][wind] * h_x[row])
                .sum();
            *value = from_states + air.p_ww[wind][0] * h_w[0] + air.p_ww[wind][1] * h_w[1];
        }
        let from_states: Scalar = h_x.iter().zip(&ph_x).map(|(h, ph)| h * ph).sum();
        let s = from_states + h_w[0] * ph_w[0] + h_w[1] * ph_w[1] + measurement.noise;
        let innovation = measurement.innovation;
        let gate = self.config.innovation_gate;
        if !s.is_finite() || s < 1e-9 || innovation * innovation > gate * gate * s {
            return false;
        }

        let mut correction = [0.0; STATE_DIM];
        for (value, ph) in correction.iter_mut().zip(&ph_x) {
            *value = ph / s * innovation;
        }
        self.apply_correction(state, &correction);

        for row in 0..STATE_DIM {
            let gain = ph_x[row] / s;
            for column in 0..STATE_DIM {
                let value = state.p_cov.get(row, column) - gain * ph_x[column];
                state.p_cov.set(row, column, value);
            }
            state.air.p_xw[row][0] -= gain * ph_w[0];
            state.air.p_xw[row][1] -= gain * ph_w[1];
        }
        state.p_cov.make_symmetric();
        state.p_cov.floor_diagonal(COV_VAR_FLOOR);
        for wind in 0..2 {
            let gain = ph_w[wind] / s;
            state.air.wind_ne[wind] += gain * innovation;
            state.air.p_ww[wind][0] -= gain * ph_w[0];
            state.air.p_ww[wind][1] -= gain * ph_w[1];
        }
        state.air.repair_wind_block();
        true
    }

    /// Add `correction` to the filter states. The attitude part is an
    /// error in the navigation frame, so it multiplies from the left.
    fn apply_correction(&self, state: &mut EkfState, correction: &[Scalar; STATE_DIM]) {
        let part = |base: usize| (correction[base], correction[base + 1], correction[base + 2]);
        let (x, y, z) = part(IDX_POS);
        state.pos = Vector3::new(
            Meters(state.pos.x.0 + x),
            Meters(state.pos.y.0 + y),
            Meters(state.pos.z.0 + z),
        );
        let (x, y, z) = part(IDX_VEL);
        state.vel = Vector3::new(
            MetersPerSecond(state.vel.x.0 + x),
            MetersPerSecond(state.vel.y.0 + y),
            MetersPerSecond(state.vel.z.0 + z),
        );
        let (x, y, z) = part(IDX_GB);
        state.gyro_bias = Vector3::new(
            RadiansPerSecond(state.gyro_bias.x.0 + x),
            RadiansPerSecond(state.gyro_bias.y.0 + y),
            RadiansPerSecond(state.gyro_bias.z.0 + z),
        );
        let (x, y, z) = part(IDX_AB);
        state.accel_bias = Vector3::new(
            MetersPerSecondSquared(state.accel_bias.x.0 + x),
            MetersPerSecondSquared(state.accel_bias.y.0 + y),
            MetersPerSecondSquared(state.accel_bias.z.0 + z),
        );
        self.clamp_biases(state);
        let (x, y, z) = part(IDX_ATT);
        let small = state.sanitize_quat(Quaternion::new(1.0, 0.5 * x, 0.5 * y, 0.5 * z));
        let rotated = small.mul(&state.quat);
        state.quat = state.sanitize_quat(rotated);
    }
}

/// Velocity of the vehicle relative to the air mass, in the navigation frame.
fn air_velocity(state: &EkfState) -> Vector3<Scalar> {
    Vector3::new(
        state.vel.x.0 - state.air.wind_ne[0],
        state.vel.y.0 - state.air.wind_ne[1],
        state.vel.z.0,
    )
}

/// True airspeed: the magnitude of the air-relative velocity.
fn airspeed_measurement(
    state: &EkfState,
    true_airspeed: Scalar,
    config: &AirAidingConfig,
) -> JointMeasurement {
    let air = air_velocity(state);
    let predicted = (air.x * air.x + air.y * air.y + air.z * air.z).sqrt();
    // A predicted airspeed near zero has no direction of its own: the
    // nose gives it, because that is where the air comes from.
    let direction = if predicted >= config.min_true_airspeed {
        Vector3::new(air.x / predicted, air.y / predicted, air.z / predicted)
    } else {
        state.quat.rotate_vector(Vector3::new(1.0, 0.0, 0.0))
    };
    let mut h_x = [0.0; STATE_DIM];
    h_x[IDX_VEL] = direction.x;
    h_x[IDX_VEL + 1] = direction.y;
    h_x[IDX_VEL + 2] = direction.z;
    JointMeasurement {
        h_x,
        h_w: [-direction.x, -direction.y],
        innovation: true_airspeed - predicted,
        noise: config.meas_noise_tas,
    }
}

/// Zero sideslip: no air-relative velocity along the body lateral axis.
fn sideslip_measurement(
    state: &EkfState,
    true_airspeed: Scalar,
    config: &AirAidingConfig,
) -> JointMeasurement {
    let air = air_velocity(state);
    let lateral = state.quat.rotate_vector(Vector3::new(0.0, 1.0, 0.0));
    // An attitude error d rotates the lateral axis to lateral + d x lateral,
    // so the measurement changes by d . (lateral x air).
    let lever = Vector3::new(
        lateral.y * air.z - lateral.z * air.y,
        lateral.z * air.x - lateral.x * air.z,
        lateral.x * air.y - lateral.y * air.x,
    );
    let mut h_x = [0.0; STATE_DIM];
    h_x[IDX_VEL] = lateral.x;
    h_x[IDX_VEL + 1] = lateral.y;
    h_x[IDX_VEL + 2] = lateral.z;
    h_x[IDX_ATT] = lever.x;
    h_x[IDX_ATT + 1] = lever.y;
    h_x[IDX_ATT + 2] = lever.z;
    let sigma = config.sideslip_sigma * true_airspeed;
    JointMeasurement {
        h_x,
        h_w: [-lateral.x, -lateral.y],
        innovation: -(lateral.x * air.x + lateral.y * air.y + lateral.z * air.z),
        noise: sigma * sigma,
    }
}
