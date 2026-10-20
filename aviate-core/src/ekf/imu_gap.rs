//! Covariance widening for time the inertial model did not cover.
//!
//! A cycle whose IMU sample is non-finite or not `Good` runs no
//! prediction: LLR-EST-204 forbids it from touching the state or the
//! covariance. The vehicle keeps moving and turning through such a
//! gap, so a covariance left at its pre-gap size claims a confidence
//! nothing measured. Once the IMU returns, the GNSS evidence of where
//! the vehicle went then lies outside the innovation gate on every
//! sample, and the estimate never re-converges. The first propagation
//! after the gap therefore adds the uncertainty the missing samples
//! would have bounded.
//!
//! This is not the age-triggered reset #135 forbids. What widens is
//! the IMU's absence, never an aiding source's staleness: with a
//! healthy IMU nothing here runs, however long GNSS has been rejected,
//! and GNSS still enters only through the unchanged innovation gate.

use super::{EkfState, IDX_ATT, IDX_POS, IDX_VEL};
use crate::ekf::Ekf;
use crate::types::Scalar;

/// Rotation rate \[rad/s\] the vehicle may have held, unmeasured,
/// through an IMU gap: a multirotor's attitude loop running on a frozen
/// estimate can tilt it by tens of degrees per second.
pub(crate) const IMU_GAP_RATE_SIGMA: Scalar = 0.5;

/// Acceleration \[m/s²\] the vehicle may have held, unmeasured, through
/// an IMU gap: hover thrust on a 30° tilt is already g·tan(30°) ≈ 5.7.
pub(crate) const IMU_GAP_ACCEL_SIGMA: Scalar = 5.0;

/// Gap length \[s\] past which widening stops growing. At the sigmas
/// above it already spans every attitude, 50 m/s and 250 m, so a longer
/// gap adds no information, only f32 range.
pub(crate) const IMU_GAP_CEILING_S: Scalar = 10.0;

impl Ekf {
    /// Widen the attitude, velocity and position variances for the time
    /// since the last propagated IMU sample, beyond the `dt` the caller
    /// is about to propagate, then restart that clock. Adds only to the
    /// diagonal, so the covariance stays positive semi-definite.
    pub(crate) fn widen_for_imu_gap(&self, state: &mut EkfState, dt: Scalar) {
        let gap_s = (state.imu_age_s - dt).min(IMU_GAP_CEILING_S);
        state.imu_age_s = 0.0;
        if gap_s <= 0.0 {
            return;
        }
        let att_var = (IMU_GAP_RATE_SIGMA * gap_s) * (IMU_GAP_RATE_SIGMA * gap_s);
        let vel_var = (IMU_GAP_ACCEL_SIGMA * gap_s) * (IMU_GAP_ACCEL_SIGMA * gap_s);
        let pos_sigma = 0.5 * IMU_GAP_ACCEL_SIGMA * gap_s * gap_s;
        for (base, added) in [
            (IDX_ATT, att_var),
            (IDX_VEL, vel_var),
            (IDX_POS, pos_sigma * pos_sigma),
        ] {
            for i in base..base + 3 {
                let widened = state.p_cov.get(i, i) + added;
                state.p_cov.set(i, i, widened);
            }
        }
    }
}
