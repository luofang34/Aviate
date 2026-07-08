//! Aiding-freshness bookkeeping and honest state-validity derivation.
//!
//! `get_estimate()` used to report `StateValidFlags::all()` and
//! `EstimateQuality::Good` for the entire time the filter was
//! `initialized`, regardless of whether GNSS/baro aiding was still
//! being fused. A filter that has silently stopped being corrected —
//! GNSS lost, every innovation gated out — kept claiming full
//! validity forever, which made any downstream estimator-validity
//! gate (mode manager, failsafe) ineffective.
//!
//! This module ties POSITION/VELOCITY validity to two independent,
//! per-source signals tracked on `EkfState`:
//!   - freshness — seconds since the last ACCEPTED (not merely
//!     attempted) fusion of the matching GNSS channel, ticked once per
//!     `observe()` cycle by `tick_aiding_age` and reset to zero by
//!     `ekf/update.rs` only when `scalar_update` returns `true`;
//!   - boundedness — the corresponding covariance diagonal, which
//!     catches numerical divergence that freshness alone would miss
//!     (aiding still "recent" by the clock, covariance already
//!     unbounded).
//!
//! ATTITUDE stays gyro-driven (IMU-only, no GNSS/baro dependency) per
//! HLR-EST-205. POSITION and VELOCITY additionally require ATTITUDE:
//! both are expressed in the same attitude frame, so a faulted
//! attitude poisons them regardless of how fresh their own aiding is —
//! this also keeps INV-005 ("POSITION_VALID implies ATTITUDE_VALID")
//! true by construction rather than by coincidence.

use super::{EkfState, IDX_POS, IDX_VEL};
use crate::state::{EstimateQuality, StateEstimate, StateValidFlags};
use crate::types::Scalar;

/// Ceiling every aiding-age counter saturates at, and the value `new()`
/// / `init()` / `reset()` seed them with to mean "never fused".
/// Comfortably above every timeout below, so neither "never aided"
/// nor "aided a very long time ago" can be mistaken for fresh, and a
/// permanently-unaided filter's counters never grow toward f32
/// infinity.
pub(crate) const AGE_SATURATION_S: Scalar = 1.0e4;

/// GNSS-position aiding timeout \[s\]. Matches the bounded
/// dead-reckoning window HLR-EST-205 already certifies (≤2 m
/// horizontal drift over 5 s of GNSS loss): inside that window the
/// dead-reckoned estimate is still the one HLR-EST-205 promises is
/// trustworthy, so POSITION should not drop before the window closes.
pub(crate) const GNSS_POS_AIDING_TIMEOUT_S: Scalar = 5.0;

/// GNSS-velocity aiding timeout \[s\]. One GNSS reading carries both
/// position and velocity at the same cadence, so the same bound
/// applies.
pub(crate) const GNSS_VEL_AIDING_TIMEOUT_S: Scalar = 5.0;

/// Barometer aiding timeout \[s\]. Baro anchors the vertical QFE datum
/// (`correct_baro_datum`); once it has gone stale this long, `Good`
/// backs off to `Degraded` even while GNSS keeps POSITION/VELOCITY set,
/// since the vertical-redundancy source has gone quiet.
pub(crate) const BARO_AIDING_TIMEOUT_S: Scalar = 5.0;

/// Position covariance-diagonal bound \[m²\] per axis. Wide margin over
/// the converged steady-state diagonal (`meas_noise_gnss_pos` = 0.5 m²
/// settles the filter well under 1 m²), but tight enough to catch
/// divergence that freshness alone would miss.
pub(crate) const POSITION_COV_BOUND_M2: Scalar = 50.0;

/// Velocity covariance-diagonal bound \[(m/s)²\]; same rationale as
/// `POSITION_COV_BOUND_M2`.
pub(crate) const VELOCITY_COV_BOUND_M2S2: Scalar = 25.0;

impl EkfState {
    /// Advance every aiding-age counter by one cycle's elapsed time.
    /// Mirrors `predict_state`'s dt gate: a non-finite or non-positive
    /// `dt`, or an un-initialized filter, leaves the counters untouched
    /// rather than corrupting them. Saturates at `AGE_SATURATION_S` so
    /// a filter that free-runs for a long time without any aiding
    /// never wraps back toward "fresh".
    pub(crate) fn tick_aiding_age(&mut self, dt: Scalar) {
        if !self.initialized || !dt.is_finite() || dt <= 0.0 {
            return;
        }
        self.gnss_pos_age_s = (self.gnss_pos_age_s + dt).min(AGE_SATURATION_S);
        self.gnss_vel_age_s = (self.gnss_vel_age_s + dt).min(AGE_SATURATION_S);
        self.baro_age_s = (self.baro_age_s + dt).min(AGE_SATURATION_S);
    }

    /// Largest covariance diagonal across the 3-axis block starting at
    /// `base_idx` (`IDX_POS` or `IDX_VEL`) — the worst-case axis is
    /// what should gate validity, not an average that could hide one
    /// divergent axis behind two converged ones.
    fn max_block_variance(&self, base_idx: usize) -> Scalar {
        (0..3)
            .map(|i| self.p_cov.get(base_idx + i, base_idx + i))
            .fold(Scalar::MIN, Scalar::max)
    }

    /// Snapshot the current state estimate for downstream consumers.
    ///
    /// POSITION and VELOCITY are graded independently: each requires
    /// its own components finite, its own GNSS channel fused within
    /// its timeout, and its own covariance diagonal bounded. A fault
    /// isolated to one (e.g. a poisoned `pos.x`) drops only that
    /// flag — it does not blank out the other, healthy state. A
    /// non-finite pos or vel component anywhere is still a stronger
    /// signal than ordinary staleness, so it forces `Unusable`
    /// quality outright regardless of which specific flags survive.
    pub fn get_estimate(&self) -> StateEstimate {
        let position_ned = [self.pos.x, self.pos.y, self.pos.z];
        let velocity_ned = [self.vel.x, self.vel.y, self.vel.z];

        let pos_finite = [self.pos.x.0, self.pos.y.0, self.pos.z.0]
            .iter()
            .all(|v| v.is_finite());
        let vel_finite = [self.vel.x.0, self.vel.y.0, self.vel.z.0]
            .iter()
            .all(|v| v.is_finite());

        let attitude_valid = self.initialized && !self.quat_fault;
        let position_aided = self.gnss_pos_age_s <= GNSS_POS_AIDING_TIMEOUT_S;
        let velocity_aided = self.gnss_vel_age_s <= GNSS_VEL_AIDING_TIMEOUT_S;
        let baro_aided = self.baro_age_s <= BARO_AIDING_TIMEOUT_S;
        let position_bounded = self.max_block_variance(IDX_POS) < POSITION_COV_BOUND_M2;
        let velocity_bounded = self.max_block_variance(IDX_VEL) < VELOCITY_COV_BOUND_M2S2;

        let position_valid = attitude_valid && pos_finite && position_aided && position_bounded;
        let velocity_valid = attitude_valid && vel_finite && velocity_aided && velocity_bounded;

        let mut valid_flags = StateValidFlags::empty();
        if attitude_valid {
            valid_flags |= StateValidFlags::ATTITUDE | StateValidFlags::ANGULAR_RATE;
        }
        if position_valid {
            valid_flags |= StateValidFlags::POSITION;
        }
        if velocity_valid {
            valid_flags |= StateValidFlags::VELOCITY;
        }

        let quality = if !attitude_valid || !pos_finite || !vel_finite {
            EstimateQuality::Unusable
        } else if position_valid && velocity_valid && baro_aided {
            EstimateQuality::Good
        } else {
            EstimateQuality::Degraded
        };

        StateEstimate {
            attitude: self.quat,
            angular_velocity: [
                self.last_gyro_body.x,
                self.last_gyro_body.y,
                self.last_gyro_body.z,
            ],
            position_ned,
            velocity_ned,
            quality,
            valid_flags,
        }
    }
}
