use crate::types::{Scalar, RadiansPerSecond};
use crate::math::{Quaternion, Vector3};

#[derive(Clone, Debug)]
pub struct AttitudeController {
    pub gains: [Scalar; 3], // P gains for Roll, Pitch, Yaw error -> Rate
}

impl AttitudeController {
    pub fn new(gains: [Scalar; 3]) -> Self {
        Self { gains }
    }

    pub fn step(
        &self,
        setpoint: &Quaternion,
        current: &Quaternion,
    ) -> [RadiansPerSecond; 3] {
        // q_err = q_setpoint * q_current^-1 (local frame error)
        // q_current^-1 is conjugate for unit quaternions
        
        // If q_current rotates Earth->Body, then q_err should represent rotation from Current Body to Desired Body?
        // Standard: q_e = q_des * q_est.inv()
        // If q is Body->Earth? Usually q is Earth->Body in some conventions, or Body->Earth.
        // Aviate EKF: "accel_earth = self.quat.rotate_vector(accel_corr)".
        // `rotate_vector` uses standard q v q*.
        // So `quat` rotates Body -> Earth (NED).
        
        // We want angular velocity in Body frame to correct the error.
        // Error Quaternion q_err: Rotation from Body_Current to Body_Desired?
        // No, we want rotation vector that moves current to desired.
        // q_err = q_est.inv() * q_des
        
        // Let's check math.rs Quaternion implementation.
        // It has `w, x, y, z`.
        // `inv` would be `w, -x, -y, -z`.
        
        let q_est_inv = Quaternion::new(current.w, -current.x, -current.y, -current.z);
        // Error Quaternion q_err: Rotation from Current to Setpoint
        let q_err = setpoint.mul(&q_est_inv).normalize();
        
        // Extract rotation vector (axis-angle) from q_err.
        // q = [cos(theta/2), v*sin(theta/2)]
        // if w < 0, negate entire quaternion to take shortest path (q and -q are same rotation)
        let (w, x, y, z) = if q_err.w < 0.0 {
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
        
        [
            RadiansPerSecond(roll_err * self.gains[0]),
            RadiansPerSecond(pitch_err * self.gains[1]),
            RadiansPerSecond(yaw_err * self.gains[2]),
        ]
    }
}
