use crate::types::FloatExt;
use crate::types::Scalar;

/// Tolerance for quaternion unit-length validation (INV-27)
pub const QUAT_NORM_EPS: Scalar = 1e-4;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vector3<T> {
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

impl Vector3<Scalar> {
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn skew_symmetric(&self) -> Matrix<3, 3> {
        let mut m = Matrix::<3, 3>::zero();
        m.data[0][1] = -self.z;
        m.data[0][2] = self.y;
        m.data[1][0] = self.z;
        m.data[1][2] = -self.x;
        m.data[2][0] = -self.y;
        m.data[2][1] = self.x;
        m
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Matrix<const R: usize, const C: usize> {
    pub data: [[Scalar; C]; R],
}

impl<const R: usize, const C: usize> Matrix<R, C> {
    pub fn zero() -> Self {
        Self {
            data: [[0.0; C]; R],
        }
    }

    pub fn get(&self, r: usize, c: usize) -> Scalar {
        self.data[r][c]
    }

    pub fn set(&mut self, r: usize, c: usize, val: Scalar) {
        self.data[r][c] = val;
    }
}

impl<const N: usize> Matrix<N, N> {
    pub fn identity() -> Self {
        let mut m = Self::zero();
        for i in 0..N {
            m.data[i][i] = 1.0;
        }
        m
    }

    pub fn make_symmetric(&mut self) {
        for r in 0..N {
            for c in 0..r {
                let avg = (self.data[r][c] + self.data[c][r]) * 0.5;
                self.data[r][c] = avg;
                self.data[c][r] = avg;
            }
        }
    }
}

impl<const R: usize, const C: usize> Matrix<R, C> {
    pub fn t(&self) -> Matrix<C, R> {
        let mut res = Matrix::<C, R>::zero();
        for r in 0..R {
            for c in 0..C {
                res.data[c][r] = self.data[r][c];
            }
        }
        res
    }

    pub fn add(&self, other: &Self) -> Self {
        let mut res = Self::zero();
        for r in 0..R {
            for c in 0..C {
                res.data[r][c] = self.data[r][c] + other.data[r][c];
            }
        }
        res
    }

    pub fn sub(&self, other: &Self) -> Self {
        let mut res = Self::zero();
        for r in 0..R {
            for c in 0..C {
                res.data[r][c] = self.data[r][c] - other.data[r][c];
            }
        }
        res
    }

    pub fn mul_scalar(&self, s: Scalar) -> Self {
        let mut res = Self::zero();
        for r in 0..R {
            for c in 0..C {
                res.data[r][c] = self.data[r][c] * s;
            }
        }
        res
    }
}

// Mat * Mat
impl<const R1: usize, const C1: usize> Matrix<R1, C1> {
    pub fn mat_mul<const C2: usize>(&self, other: &Matrix<C1, C2>) -> Matrix<R1, C2> {
        let mut res = Matrix::<R1, C2>::zero();
        for r in 0..R1 {
            for c in 0..C2 {
                let mut sum = 0.0;
                for k in 0..C1 {
                    sum += self.data[r][k] * other.data[k][c];
                }
                res.data[r][c] = sum;
            }
        }
        res
    }
}

// Vector ops
impl<const N: usize> Matrix<N, 1> {
    pub fn to_vec3(&self) -> Vector3<Scalar> {
        // Assumes N >= 3
        Vector3 {
            x: self.data[0][0],
            y: self.data[1][0],
            z: self.data[2][0],
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Quaternion {
    pub w: Scalar,
    pub x: Scalar,
    pub y: Scalar,
    pub z: Scalar,
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(w: Scalar, x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self { w, x, y, z }
    }

    pub fn norm_sq(&self) -> Scalar {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Check if quaternion is unit-length within tolerance (INV-27)
    pub fn is_normalized(&self, epsilon: Scalar) -> bool {
        let norm_sq = self.norm_sq();
        // Explicit NaN/Inf handling: any non-finite value is not normalized
        if !norm_sq.is_finite() {
            return false;
        }
        (norm_sq - 1.0).abs() < epsilon
    }

    /// Convenience method using default tolerance
    pub fn is_normalized_default(&self) -> bool {
        self.is_normalized(QUAT_NORM_EPS)
    }

    pub fn normalize(&self) -> Self {
        let n = self.norm_sq().sqrt();
        if n > 1e-6 && n.is_finite() {
            Self {
                w: self.w / n,
                x: self.x / n,
                y: self.y / n,
                z: self.z / n,
            }
        } else {
            Self::IDENTITY
        }
    }

    /// Rotate a vector using this quaternion.
    ///
    /// # Frame Convention
    ///
    /// In Aviate, the quaternion represents the **Body → NED** transformation.
    /// This method computes `v' = q ⊗ v ⊗ q*` (Hamilton product convention).
    ///
    /// - Input: `v` in Body frame
    /// - Output: `v'` in NED (Earth) frame
    ///
    /// # Example
    /// ```ignore
    /// // Accelerometer reads [0, 0, -9.81] in body frame when level
    /// let accel_body = Vector3::new(0.0, 0.0, -9.81);
    /// let accel_ned = quat.rotate_vector(accel_body);
    /// // accel_ned ≈ [0, 0, -9.81] when attitude is level
    /// ```
    pub fn rotate_vector(&self, v: Vector3<Scalar>) -> Vector3<Scalar> {
        // Hamilton product: v' = q ⊗ v ⊗ q*
        let qx = self.x;
        let qy = self.y;
        let qz = self.z;
        let qw = self.w;

        let ix = qw * v.x + qy * v.z - qz * v.y;
        let iy = qw * v.y + qz * v.x - qx * v.z;
        let iz = qw * v.z + qx * v.y - qy * v.x;
        let iw = -qx * v.x - qy * v.y - qz * v.z;

        Vector3 {
            x: ix * qw + iw * -qx + iy * -qz - iz * -qy,
            y: iy * qw + iw * -qy + iz * -qx - ix * -qz,
            z: iz * qw + iw * -qz + ix * -qy - iy * -qx,
        }
    }

    // Quaternion multiplication
    pub fn mul(&self, other: &Self) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }

    // Create from axis-angle
    pub fn from_axis_angle(axis: Vector3<Scalar>, angle: Scalar) -> Self {
        let half_angle = angle * 0.5;
        let s = half_angle.sin();
        Self {
            w: half_angle.cos(),
            x: axis.x * s,
            y: axis.y * s,
            z: axis.z * s,
        }
    }

    /// Extract Euler angles (roll, pitch, yaw) from this quaternion.
    ///
    /// # Frame Convention
    ///
    /// Uses **ZYX (yaw-pitch-roll)** Euler sequence in NED frame:
    /// - Roll (φ): rotation about X-axis (North), positive = right wing down
    /// - Pitch (θ): rotation about Y-axis (East), positive = nose up
    /// - Yaw (ψ): rotation about Z-axis (Down), positive = clockwise from above
    ///
    /// # Returns
    ///
    /// `(roll, pitch, yaw)` in radians, range:
    /// - roll: [-π, π]
    /// - pitch: [-π/2, π/2] (gimbal lock protected)
    /// - yaw: [-π, π]
    ///
    /// # Note
    ///
    /// Gimbal lock occurs at pitch = ±90°. This implementation clamps pitch
    /// to avoid numerical issues near the singularity.
    pub fn to_euler(&self) -> (Scalar, Scalar, Scalar) {
        // ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 {
            // use 90 degrees if out of range
            if sinp > 0.0 {
                core::f32::consts::FRAC_PI_2
            } else {
                -core::f32::consts::FRAC_PI_2
            }
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    /// Convert this quaternion to a 3×3 rotation matrix.
    ///
    /// # Frame Convention
    ///
    /// The returned matrix R transforms vectors from **Body frame to NED frame**:
    /// ```ignore
    /// v_ned = R * v_body
    /// ```
    ///
    /// This is consistent with `rotate_vector()`:
    /// ```ignore
    /// let v_ned_a = quat.rotate_vector(v_body);
    /// let v_ned_b = quat.to_rotation_matrix() * v_body;
    /// // v_ned_a ≈ v_ned_b
    /// ```
    ///
    /// # Matrix Layout
    ///
    /// ```text
    /// R = | R[0][0]  R[0][1]  R[0][2] |   | r_n·b_x  r_n·b_y  r_n·b_z |
    ///     | R[1][0]  R[1][1]  R[1][2] | = | r_e·b_x  r_e·b_y  r_e·b_z |
    ///     | R[2][0]  R[2][1]  R[2][2] |   | r_d·b_x  r_d·b_y  r_d·b_z |
    /// ```
    ///
    /// Where `r_n`, `r_e`, `r_d` are NED basis vectors expressed in body frame.
    pub fn to_rotation_matrix(&self) -> Matrix<3, 3> {
        let x2 = self.x + self.x;
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;
        let xx = self.x * x2;
        let xy = self.x * y2;
        let xz = self.x * z2;
        let yy = self.y * y2;
        let yz = self.y * z2;
        let zz = self.z * z2;
        let wx = self.w * x2;
        let wy = self.w * y2;
        let wz = self.w * z2;

        let mut m = Matrix::<3, 3>::zero();

        m.data[0][0] = 1.0 - (yy + zz);
        m.data[0][1] = xy - wz;
        m.data[0][2] = xz + wy;

        m.data[1][0] = xy + wz;
        m.data[1][1] = 1.0 - (xx + zz);
        m.data[1][2] = yz - wx;

        m.data[2][0] = xz - wy;
        m.data[2][1] = yz + wx;
        m.data[2][2] = 1.0 - (xx + yy);

        m
    }
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}
