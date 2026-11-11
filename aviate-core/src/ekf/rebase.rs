//! Rigid change of the navigation frame.
//!
//! The navigation frame is a local tangent plane. Far from its origin
//! the plane leaves the ellipsoid and its axes leave the local level, so
//! the board moves the origin to the vehicle before that matters. The
//! move is a rigid transform `new = R (old - o)`: the same physical
//! state, the same uncertainty, expressed in the new axes. No estimate
//! changes, so nothing the vehicle flies on steps.

use super::{EkfState, IDX_ATT, IDX_POS, IDX_VEL, STATE_DIM};
use crate::math::{Matrix, Quaternion, Vector3};
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::{Meters, MetersPerSecond, Scalar};

/// Geodetic point at the origin of a navigation frame, on the datum.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavOrigin {
    /// Geodetic latitude \[deg\].
    pub latitude_deg: f64,
    /// Longitude \[deg\].
    pub longitude_deg: f64,
}

/// The navigation frame that the estimate is expressed in.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NavFrame {
    /// Origin after the last rebase; `None` is the configured origin.
    pub origin: Option<NavOrigin>,
    /// Number of rebases since the filter was seeded.
    pub rebases: u32,
}

/// A rigid change of the navigation frame: `new = rotation (old - offset)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameShift {
    /// Rotation from the old axes to the new axes.
    pub rotation: [[f64; 3]; 3],
    /// The new origin in the old frame \[m\].
    pub offset_m: [f64; 3],
    /// The new origin.
    pub origin: NavOrigin,
}

impl FrameShift {
    /// Apply the shift to a position in the old frame.
    #[must_use]
    pub fn position(&self, old: [f64; 3]) -> [f64; 3] {
        let [x, y, z] = old;
        let [ox, oy, oz] = self.offset_m;
        self.direction([x - ox, y - oy, z - oz])
    }

    /// Apply the rotation to a direction or a velocity in the old frame.
    #[must_use]
    pub fn direction(&self, old: [f64; 3]) -> [f64; 3] {
        self.rotation
            .map(|row| row[0] * old[0] + row[1] * old[1] + row[2] * old[2])
    }

    /// Change of a heading: the bearing of the old north in the new
    /// frame, added to an old heading to give the new one.
    #[must_use]
    pub fn heading_change(&self) -> f64 {
        let [north, east, _] = self.direction([1.0, 0.0, 0.0]);
        libm::atan2(east, north)
    }

    fn rotation_f32(&self) -> Matrix<3, 3> {
        Matrix {
            data: self.rotation.map(|row| row.map(|value| value as Scalar)),
        }
    }

    fn rotation_quaternion(&self) -> Quaternion {
        let r = &self.rotation;
        // Shepperd's method on the largest of the four squared terms.
        let trace = r[0][0] + r[1][1] + r[2][2];
        let (w, x, y, z) = if trace > 0.0 {
            let s = 2.0 * libm::sqrt(1.0 + trace);
            (
                0.25 * s,
                (r[2][1] - r[1][2]) / s,
                (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s,
            )
        } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
            let s = 2.0 * libm::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]);
            (
                (r[2][1] - r[1][2]) / s,
                0.25 * s,
                (r[0][1] + r[1][0]) / s,
                (r[0][2] + r[2][0]) / s,
            )
        } else if r[1][1] > r[2][2] {
            let s = 2.0 * libm::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]);
            (
                (r[0][2] - r[2][0]) / s,
                (r[0][1] + r[1][0]) / s,
                0.25 * s,
                (r[1][2] + r[2][1]) / s,
            )
        } else {
            let s = 2.0 * libm::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]);
            (
                (r[1][0] - r[0][1]) / s,
                (r[0][2] + r[2][0]) / s,
                (r[1][2] + r[2][1]) / s,
                0.25 * s,
            )
        };
        Quaternion::new(w as Scalar, x as Scalar, y as Scalar, z as Scalar)
    }
}

impl EkfState {
    /// Express the filter in the frame that `shift` leads to.
    ///
    /// Position, velocity and attitude move with the frame; the biases
    /// are in body axes and stay. The covariance takes the same rotation
    /// on its position, velocity and attitude blocks, and the wind block
    /// on its horizontal axes.
    ///
    /// The barometer datum stays. The barometer measures a height, and
    /// at the new origin a height is exactly a plane coordinate, so the
    /// datum is right in the new frame as it is. Moving it with the
    /// vehicle's coordinate would carry the drop of the old plane below
    /// the ellipsoid (`d² / 2R`) into every later height, one drop per
    /// rebase.
    pub fn rebase(&mut self, shift: &FrameShift) {
        let [n, e, d] = shift.position([
            f64::from(self.pos.x.0),
            f64::from(self.pos.y.0),
            f64::from(self.pos.z.0),
        ]);
        self.pos = Vector3::new(
            Meters(n as Scalar),
            Meters(e as Scalar),
            Meters(d as Scalar),
        );
        let [vn, ve, vd] = shift.direction([
            f64::from(self.vel.x.0),
            f64::from(self.vel.y.0),
            f64::from(self.vel.z.0),
        ]);
        self.vel = Vector3::new(
            MetersPerSecond(vn as Scalar),
            MetersPerSecond(ve as Scalar),
            MetersPerSecond(vd as Scalar),
        );
        let turned = shift.rotation_quaternion().mul(&self.quat);
        self.quat = self.sanitize_quat(turned);

        let rotation = shift.rotation_f32();
        let transform = block_rotation(&rotation);
        self.p_cov = transform.mat_mul(&self.p_cov).mat_mul(&transform.t());
        self.p_cov.make_symmetric();
        self.rotate_wind(&rotation, &transform);

        self.frame = NavFrame {
            origin: Some(shift.origin),
            rebases: self.frame.rebases.wrapping_add(1),
        };
    }

    /// Rotate the wind and its blocks by the horizontal part of the shift.
    fn rotate_wind(&mut self, rotation: &Matrix<3, 3>, transform: &Matrix<STATE_DIM, STATE_DIM>) {
        if !self.air.active {
            return;
        }
        let r = |row: usize, column: usize| rotation.get(row, column);
        let horizontal = [[r(0, 0), r(0, 1)], [r(1, 0), r(1, 1)]];
        let turn = |v: [Scalar; 2]| {
            [
                horizontal[0][0] * v[0] + horizontal[0][1] * v[1],
                horizontal[1][0] * v[0] + horizontal[1][1] * v[1],
            ]
        };
        self.air.wind_ne = turn(self.air.wind_ne);
        // P_xw' = T P_xw H^T and P_ww' = H P_ww H^T.
        let crossed = self.air.p_xw.map(turn);
        for (cross, t_row) in self.air.p_xw.iter_mut().zip(&transform.data) {
            *cross = t_row.iter().zip(&crossed).fold([0.0; 2], |sum, (t, c)| {
                [sum[0] + t * c[0], sum[1] + t * c[1]]
            });
        }
        let [row0, row1] = self.air.p_ww.map(turn);
        let turned = turn([row0[0], row1[0]]);
        let turned_1 = turn([row0[1], row1[1]]);
        self.air.p_ww = [[turned[0], turned_1[0]], [turned[1], turned_1[1]]];
        self.air.repair_wind_block();
    }
}

/// The 15-state transform of a frame rotation: the rotation on the
/// position, velocity and attitude blocks, the identity on the biases.
fn block_rotation(rotation: &Matrix<3, 3>) -> Matrix<STATE_DIM, STATE_DIM> {
    let mut transform = Matrix::identity();
    for base in [IDX_POS, IDX_VEL, IDX_ATT] {
        for row in 0..3 {
            for column in 0..3 {
                transform.set(base + row, base + column, rotation.get(row, column));
            }
        }
    }
    transform
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests;
