use crate::math::Quaternion;
use crate::types::{RadiansPerSecond, Scalar};

#[derive(Clone, Debug)]
pub struct AttitudeController {
    pub gains: [Scalar; 3], // P gains for Roll, Pitch, Yaw error -> Rate
    /// Per-axis cap on the commanded body rate (rad/s). Mirrors
    /// `CascadeGains::att_max_rate_cmd` — airframe tuning covered
    /// by the canonical config hash; the saturation rationale
    /// lives on that field's doc.
    pub max_rate_cmd: Scalar,
}

impl AttitudeController {
    pub fn new(gains: [Scalar; 3], max_rate_cmd: Scalar) -> Self {
        Self {
            gains,
            max_rate_cmd,
        }
    }

    pub fn step(&self, setpoint: &Quaternion, current: &Quaternion) -> [RadiansPerSecond; 3] {
        // Body-frame attitude error: `Δq_body = q_cur⁻¹ · q_des`.
        // The vector part is the rotation axis expressed in
        // body frame, which is the frame the rate loop interprets
        // its setpoint in.
        let q_cur_inv = Quaternion::new(current.w, -current.x, -current.y, -current.z);
        let q_err = q_cur_inv.mul(setpoint).normalize();

        // Extract rotation vector (axis-angle) from q_err.
        // q = [cos(theta/2), v*sin(theta/2)]
        // if w < 0, negate entire quaternion to take shortest path (q and -q are same rotation)
        let (_w, x, y, z) = if q_err.w < 0.0 {
            (-q_err.w, -q_err.x, -q_err.y, -q_err.z)
        } else {
            (q_err.w, q_err.x, q_err.y, q_err.z)
        };

        // 2 * acos(w) is angle. But for small angles, 2 * x, 2 * y, 2 * z is approx angle * axis.
        // Standard P controller uses the vector part directly for feedback.
        // rate_cmd = gain * sign(w) * [x, y, z] * 2?
        // Or just gain * [x, y, z] * 2.

        // Using small angle approximation for control:
        let roll_err = 2.0 * x;
        let pitch_err = 2.0 * y;
        let yaw_err = 2.0 * z;

        let roll_cmd = (roll_err * self.gains[0]).clamp(-self.max_rate_cmd, self.max_rate_cmd);
        let pitch_cmd = (pitch_err * self.gains[1]).clamp(-self.max_rate_cmd, self.max_rate_cmd);
        let yaw_cmd = (yaw_err * self.gains[2]).clamp(-self.max_rate_cmd, self.max_rate_cmd);

        [
            RadiansPerSecond(roll_cmd),
            RadiansPerSecond(pitch_cmd),
            RadiansPerSecond(yaw_cmd),
        ]
    }
}
