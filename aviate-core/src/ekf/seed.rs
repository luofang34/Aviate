//! Covariance of a freshly seeded filter.

use super::air_aiding::AirAidingState;
use super::{EkfState, IDX_AB, IDX_ATT, IDX_GB, IDX_POS, IDX_VEL, STATE_DIM};
use crate::math::{Matrix, Quaternion, Vector3};
use crate::types::{Meters, MetersPerSecond, Scalar};

/// Variance of roll and pitch of a seed from the accelerometer at rest
/// \[rad²\]: 3 degrees at one standard deviation, the tilt that an
/// accelerometer bias of 0.2 m/s² and a small motion give. The heading
/// update sees roll and pitch through the levelled field; with a loose
/// tilt prior it would explain a heading error as tilt.
const SEED_TILT_VAR_RAD2: Scalar = 0.0027;

/// Initial covariance: position, velocity and heading start loose
/// (0.1), roll and pitch at the accelerometer tilt
/// (`SEED_TILT_VAR_RAD2`), bias blocks at realistic MEMS scales — gyro
/// (0.02 rad/s)², accel (0.2 m/s²)². Seeding the gyro-bias variance
/// at 0.1 (σ ≈ 0.32 rad/s, ~30× a real bias) invites the filter to
/// explain early attitude motion as bias: with oscillating flight
/// the GNSS cross-terms walk the bias state to whole rad/s and
/// predict integrates the phantom rate (the #123 pump).
pub(super) fn initial_covariance() -> Matrix<STATE_DIM, STATE_DIM> {
    let mut p = Matrix::identity().mul_scalar(0.1);
    p.set(IDX_ATT, IDX_ATT, SEED_TILT_VAR_RAD2);
    p.set(IDX_ATT + 1, IDX_ATT + 1, SEED_TILT_VAR_RAD2);
    for i in 0..3 {
        p.set(IDX_GB + i, IDX_GB + i, 4e-4);
        p.set(IDX_AB + i, IDX_AB + i, 0.04);
    }
    p
}

/// Variances of a seed that no position fix backs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SeedUncertainty {
    /// Variance of each horizontal position component \[m²\].
    pub horizontal_position: Scalar,
    /// Variance of each velocity component about the seed \[(m/s)²\],
    /// without the wind.
    pub velocity: Scalar,
    /// Variance of roll and pitch \[rad²\].
    pub tilt: Scalar,
    /// Variance of the heading \[rad²\].
    pub heading: Scalar,
    /// Variance of each wind component \[(m/s)²\] when the velocity seed
    /// is an air velocity, `None` when it is a velocity over the ground.
    pub wind: Option<Scalar>,
}

impl EkfState {
    /// Seed the filter from a state that no position fix backs.
    ///
    /// The variances state how far the seed can be from the truth. A
    /// seed with an honest variance lets the first fix through the
    /// innovation gate; a seed that claims the default variance would
    /// have it rejected.
    ///
    /// A velocity seed that is an air velocity misses the wind. The
    /// velocity over the ground is then uncertain by the wind, and the
    /// error of the two is the same error: the seed states that with the
    /// covariance of velocity and wind, so that the airspeed still
    /// agrees with the seed however large the wind variance is.
    pub fn init_uncertain(
        &mut self,
        pos: Vector3<Meters>,
        vel: Vector3<MetersPerSecond>,
        quat: Quaternion,
        uncertainty: &SeedUncertainty,
    ) {
        self.init(pos, vel, quat);
        let wind = uncertainty.wind.unwrap_or(0.0);
        let seeded = [
            (IDX_POS, uncertainty.horizontal_position),
            (IDX_POS + 1, uncertainty.horizontal_position),
            (IDX_VEL, uncertainty.velocity + wind),
            (IDX_VEL + 1, uncertainty.velocity + wind),
            (IDX_VEL + 2, uncertainty.velocity),
            (IDX_ATT, uncertainty.tilt),
            (IDX_ATT + 1, uncertainty.tilt),
            (IDX_ATT + 2, uncertainty.heading),
        ];
        for (idx, variance) in seeded {
            self.p_cov.set(idx, idx, variance);
        }
        if let Some(wind) = uncertainty.wind {
            self.air = AirAidingState::activated(wind);
            self.air.p_xw[IDX_VEL] = [wind, 0.0];
            self.air.p_xw[IDX_VEL + 1] = [0.0, wind];
        }
    }
}
