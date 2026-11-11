//! Air-data aiding: true airspeed, zero sideslip and two wind states.
//!
//! A fixed wing moves through the air along its nose. The filter uses
//! that as two measurements of the velocity relative to the air mass:
//! its magnitude is the true airspeed, and its component along the body
//! lateral axis is zero. The air mass moves over the ground with a wind
//! that the filter carries as two horizontal states.
//!
//! These measurements bound the velocity states without a position fix.
//! Through the coupling of velocity and attitude they also keep roll and
//! pitch observable: a tilt error turns gravity into a velocity error
//! that grows with time, and the air measurements see it.
//!
//! The wind states live beside the 15-state covariance, with their own
//! block and a cross block. The estimator configuration decides at
//! construction whether the block exists. A filter without it never
//! touches the block, so its arithmetic is that of the 15-state filter.

use super::joint::{state_row, JointMeasurement};
use super::validity::AGE_SATURATION_S;
use super::{Ekf, EkfState, STATE_DIM};
use crate::math::{Matrix, Vector3};
use crate::sensor::SensorSet;
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

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

impl AirAidingState {
    /// A block that takes part in the filter, with no knowledge of the
    /// wind beyond `wind_var` on each axis.
    pub(crate) fn activated(wind_var: Scalar) -> Self {
        Self {
            p_ww: [[wind_var, 0.0], [0.0, wind_var]],
            active: true,
            ..Self::default()
        }
    }

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
        for (cross, f_row) in self.p_xw.iter_mut().zip(&f.data) {
            *cross = f_row
                .iter()
                .zip(&before)
                .fold([0.0; 2], |sum, (f, source)| {
                    [sum[0] + f * source[0], sum[1] + f * source[1]]
                });
        }
        self.p_ww[0][0] += wind_variance_growth;
        self.p_ww[1][1] += wind_variance_growth;
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
        let Some(&measured) = self.p_xw.get(state_idx) else {
            return;
        };
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
    pub(super) fn repair_wind_block(&mut self) {
        let off_diagonal = 0.5 * (self.p_ww[0][1] + self.p_ww[1][0]);
        self.p_ww[0][1] = off_diagonal;
        self.p_ww[1][0] = off_diagonal;
        let floored = |variance: Scalar| {
            if variance.is_finite() && variance >= WIND_VAR_FLOOR {
                variance
            } else {
                WIND_VAR_FLOOR
            }
        };
        self.p_ww[0][0] = floored(self.p_ww[0][0]);
        self.p_ww[1][1] = floored(self.p_ww[1][1]);
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
            state.air = AirAidingState::activated(config.wind_prior_var);
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
pub(crate) fn airspeed_measurement(
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
    JointMeasurement {
        h_x: state_row([direction.x, direction.y, direction.z], [0.0; 3]),
        h_w: [-direction.x, -direction.y],
        innovation: true_airspeed - predicted,
        noise: config.meas_noise_tas,
    }
}

/// Zero sideslip: no air-relative velocity along the body lateral axis.
pub(crate) fn sideslip_measurement(
    state: &EkfState,
    true_airspeed: Scalar,
    config: &AirAidingConfig,
) -> JointMeasurement {
    let air = air_velocity(state);
    let lateral = state.quat.rotate_vector(Vector3::new(0.0, 1.0, 0.0));
    // An attitude error d rotates the lateral axis to lateral + d x lateral,
    // so the measurement changes by d . (lateral x air).
    let lever = [
        lateral.y * air.z - lateral.z * air.y,
        lateral.z * air.x - lateral.x * air.z,
        lateral.x * air.y - lateral.y * air.x,
    ];
    let sigma = config.sideslip_sigma * true_airspeed;
    JointMeasurement {
        h_x: state_row([lateral.x, lateral.y, lateral.z], lever),
        h_w: [-lateral.x, -lateral.y],
        innovation: -(lateral.x * air.x + lateral.y * air.y + lateral.z * air.z),
        noise: sigma * sigma,
    }
}
