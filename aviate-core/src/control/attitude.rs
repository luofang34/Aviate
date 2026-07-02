use crate::math::Quaternion;
use crate::types::{RadiansPerSecond, Scalar};

/// Maximum body-frame angular rate the attitude controller is
/// allowed to demand from the rate loop, per axis (rad/s).
/// Without this cap the P-controller demands gain·error rad/s
/// for arbitrarily large attitude errors — at a 90° error that
/// is `gain · π/2` rad/s, which then saturates the rate loop
/// against gyro and induces a recovery overshoot far larger
/// than the original error. Physically, a multirotor cannot
/// servo angular rates beyond a few rad/s without losing
/// thrust authority anyway (the rotors spend their thrust on
/// torque rather than lift).
const MAX_ATTITUDE_RATE_CMD: Scalar = 3.0;

#[derive(Clone, Debug)]
pub struct AttitudeController {
    pub gains: [Scalar; 3], // P gains for Roll, Pitch, Yaw error -> Rate
}

impl AttitudeController {
    pub fn new(gains: [Scalar; 3]) -> Self {
        Self { gains }
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

        let roll_cmd =
            (roll_err * self.gains[0]).clamp(-MAX_ATTITUDE_RATE_CMD, MAX_ATTITUDE_RATE_CMD);
        let pitch_cmd =
            (pitch_err * self.gains[1]).clamp(-MAX_ATTITUDE_RATE_CMD, MAX_ATTITUDE_RATE_CMD);
        let yaw_cmd =
            (yaw_err * self.gains[2]).clamp(-MAX_ATTITUDE_RATE_CMD, MAX_ATTITUDE_RATE_CMD);

        [
            RadiansPerSecond(roll_cmd),
            RadiansPerSecond(pitch_cmd),
            RadiansPerSecond(yaw_cmd),
        ]
    }
}
