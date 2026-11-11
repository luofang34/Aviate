//! Canonical byte encoding of `EkfState` for cross-channel replication.
//!
//! Lockstep witnesses compare byte-identical `encode_canonical` output,
//! so this block pins a fixed-width, deterministic layout for every
//! field of the persistent filter state.

use super::{EkfState, STATE_DIM};

impl crate::replicable::Replicable for EkfState {
    // 4 (quat) + 3 (pos) + 3 (vel) + 3 (gyro_bias) + 3 (accel_bias)
    // + 3 (last_gyro_body) = 19 f32s for vector data,
    // + STATE_DIM*STATE_DIM = 225 f32s for the covariance matrix,
    // + 2 bytes for the boolean latches (initialized, quat_fault),
    // + 1 presence byte and 4 f32 bytes for the QFE baro_ref datum,
    // + 4 f32 bytes for its estimator variance (baro_ref_var),
    // + 4 f32 bytes for the uncovered IMU time (imu_age_s),
    // + 4 f32 bytes for the current clipping run (accel_clip_s).
    // Total = (19 + 225) * 4 + 2 + 5 + 4 + 4 + 4 = 995 bytes.
    // The wind block adds 2 wind states, 4 wind covariances, 2 cross
    // covariances for each filter state, one age and one activity tag.
    // The navigation frame adds a presence byte, the origin latitude and
    // longitude (2 f64) and the rebase count (u32).
    const ENCODED_LEN: usize = (19 + STATE_DIM * STATE_DIM) * 4
        + 2
        + 5
        + 4
        + 4
        + 4
        + (2 + 4 + 2 * STATE_DIM + 1) * 4
        + 1
        + 1
        + 2 * 8
        + 4;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        use crate::replicable::copy_into;
        let mut w = 0usize;
        // Vector helper: writes a Vector3<inner-f32> to buf.
        macro_rules! v3 {
            ($v:expr) => {{
                w += copy_into(buf, w, &$v.x.0.to_le_bytes());
                w += copy_into(buf, w, &$v.y.0.to_le_bytes());
                w += copy_into(buf, w, &$v.z.0.to_le_bytes());
            }};
        }
        // Quaternion: w, x, y, z in declaration order (no newtype wrap).
        w += copy_into(buf, w, &self.quat.w.to_le_bytes());
        w += copy_into(buf, w, &self.quat.x.to_le_bytes());
        w += copy_into(buf, w, &self.quat.y.to_le_bytes());
        w += copy_into(buf, w, &self.quat.z.to_le_bytes());
        v3!(self.pos);
        v3!(self.vel);
        v3!(self.gyro_bias);
        v3!(self.accel_bias);
        v3!(self.last_gyro_body);
        // Covariance matrix: row-major, then column-major within each row.
        for row in &self.p_cov.data {
            for v in row {
                w += copy_into(buf, w, &v.to_le_bytes());
            }
        }
        // Boolean latches.
        w += copy_into(buf, w, &[if self.initialized { 1 } else { 0 }]);
        w += copy_into(buf, w, &[if self.quat_fault { 1 } else { 0 }]);
        // QFE baro datum: presence byte then the reference f32 (0.0
        // when un-latched) so both channels encode a fixed-width,
        // deterministic block regardless of latch state.
        let (present, baro_ref) = match self.baro_ref {
            Some(r) => (1u8, r),
            None => (0u8, 0.0),
        };
        w += copy_into(buf, w, &[present]);
        w += copy_into(buf, w, &baro_ref.to_le_bytes());
        w += copy_into(buf, w, &self.baro_ref_var.to_le_bytes());
        w += copy_into(buf, w, &self.imu_age_s.to_le_bytes());
        w += copy_into(buf, w, &self.accel_clip_s.to_le_bytes());
        let air = &self.air;
        let wind_block = air.p_ww.iter().flatten();
        let cross_block = air.p_xw.iter().flatten();
        for v in air.wind_ne.iter().chain(wind_block).chain(cross_block) {
            w += copy_into(buf, w, &v.to_le_bytes());
        }
        w += copy_into(buf, w, &air.age_s.to_le_bytes());
        w += copy_into(buf, w, &[u8::from(air.active)]);
        let origin = self.frame.origin;
        w += copy_into(buf, w, &[u8::from(origin.is_some())]);
        let (latitude, longitude) =
            origin.map_or((0.0, 0.0), |o| (o.latitude_deg, o.longitude_deg));
        w += copy_into(buf, w, &latitude.to_le_bytes());
        w += copy_into(buf, w, &longitude.to_le_bytes());
        w += copy_into(buf, w, &self.frame.rebases.to_le_bytes());
        w
    }
}
