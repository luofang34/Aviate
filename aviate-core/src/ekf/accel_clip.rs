//! Covariance widening for accelerometer samples at the sensor's range.
//!
//! A hard touchdown or a crash decelerates the vehicle at hundreds of g
//! for a few milliseconds, far past any accelerometer's range. A sample
//! at the range states only that the specific force was at least that
//! large, so integrating it leaves most of the impulse out of the
//! velocity estimate, while the covariance keeps claiming the confidence
//! of an in-range measurement. GNSS then reports the vehicle at rest on
//! the ground, outside the innovation gate on every sample, and the
//! velocity never returns. Each clipped sample is still integrated, as
//! the best lower bound there is, and the covariance widens by what the
//! unmeasured excess could have done over the run of clipped samples.
//!
//! This is not the age-triggered reset #135 forbids. What widens is the
//! accelerometer declaring itself saturated, never an aiding source's
//! staleness: with in-range samples nothing here runs, however long GNSS
//! has been rejected, and GNSS still enters only through the unchanged
//! innovation gate. Gyro saturation is the matching case for attitude
//! and is not handled here.

use super::{EkfState, IDX_POS, IDX_VEL};
use crate::ekf::Ekf;
use crate::types::{MetersPerSecondSquared, Scalar};

/// Specific force \[m/s²\] at which a sample counts as clipped: 16 g, the
/// smallest accelerometer range any supported board configures. A sample
/// at or past it is clipped on that sensor; on a wider-range sensor it is
/// merely treated with more caution than it needed.
pub(crate) const ACCEL_CLIP_MPS2: Scalar = 16.0 * 9.80665;

/// Excess specific force \[m/s²\] a clipped sample may have hidden: a
/// touchdown at 17 m/s on stiff gear averages about 300 g over its first
/// 4 ms.
pub(crate) const ACCEL_CLIP_EXCESS_SIGMA: Scalar = 500.0;

/// Clipping-run length \[s\] past which widening stops growing. At the
/// sigma above it already spans 500 m/s, so a longer run adds no
/// information, only f32 range.
pub(crate) const ACCEL_CLIP_CEILING_S: Scalar = 1.0;

impl Ekf {
    /// Widen the velocity and position variances for one clipped sample,
    /// as one more `dt` of a run whose hidden excess is a single unknown
    /// force: the run's standard deviation grows with its length, so the
    /// total does not depend on the sample rate. An in-range sample ends
    /// the run. Adds only to the diagonal, so the covariance stays
    /// positive semi-definite.
    pub(crate) fn widen_for_accel_clip(
        &self,
        state: &mut EkfState,
        accel: &[MetersPerSecondSquared; 3],
        dt: Scalar,
    ) {
        if !accel.iter().any(|a| a.0.abs() >= ACCEL_CLIP_MPS2) {
            state.accel_clip_s = 0.0;
            return;
        }
        let before_s = state.accel_clip_s;
        let after_s = (before_s + dt).min(ACCEL_CLIP_CEILING_S);
        state.accel_clip_s = after_s;
        let vel_var = |t: Scalar| (ACCEL_CLIP_EXCESS_SIGMA * t) * (ACCEL_CLIP_EXCESS_SIGMA * t);
        let pos_var = |t: Scalar| vel_var(t) * (0.5 * t) * (0.5 * t);
        let vel_added = vel_var(after_s) - vel_var(before_s);
        let pos_added = pos_var(after_s) - pos_var(before_s);
        for (base, added) in [(IDX_VEL, vel_added), (IDX_POS, pos_added)] {
            for i in base..base + 3 {
                let widened = state.p_cov.get(i, i) + added;
                state.p_cov.set(i, i, widened);
            }
        }
    }
}
