use crate::math::{Quaternion, Vector3};
#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::{FloatExt, MetersPerSecond, Normalized, Scalar};

#[derive(Clone, Debug)]
pub struct VelocityController {
    pub gains: [Scalar; 3],     // P gains for X, Y, Z velocity
    pub max_roll_pitch: Scalar, // Max roll/pitch angle in radians
    /// Hover thrust trim (Normalized [0..1]) — the collective-thrust
    /// command at which motor lift equals airframe weight. The
    /// vertical loop commands corrections AROUND this value. Wrong
    /// trim makes the loop saturate; airframe-specific.
    /// See `ResolvedKernelConfig.hover_thrust_norm`.
    pub hover_thrust_norm: Scalar,
}

impl VelocityController {
    pub fn new(gains: [Scalar; 3], max_roll_pitch: Scalar, hover_thrust_norm: Scalar) -> Self {
        Self {
            gains,
            max_roll_pitch,
            hover_thrust_norm,
        }
    }

    pub fn step(
        &self,
        setpoint: Vector3<MetersPerSecond>,
        current: Vector3<MetersPerSecond>,
        current_att: &Quaternion, // Need current attitude to calculate roll/pitch commands
    ) -> (Normalized, Quaternion) {
        // Returns collective thrust and attitude setpoint (roll/pitch)
        let error = Vector3 {
            x: setpoint.x.0 - current.x.0,
            y: setpoint.y.0 - current.y.0,
            z: setpoint.z.0 - current.z.0,
        };

        // Collective thrust is derived from Z velocity error (vertical)
        // In NED: +Z is down. To arrest descent (current.z > setpoint.z), need more thrust.
        // error.z = setpoint.z - current.z (negative when descending too fast)
        // We invert because negative error (descending too fast) needs positive thrust correction
        let trim = self.hover_thrust_norm;
        // Allow corrections to span the full range below trim (to drop
        // thrust to zero) and the remaining headroom above (to
        // saturate at 1.0). Without this asymmetric clamp, an
        // airframe with trim>0.5 can never command full thrust.
        let max_up = 1.0 - trim;
        let max_dn = trim;
        let collective_thrust_cmd = trim + (-error.z * self.gains[2]).clamp(-max_dn, max_up);
        let collective = Normalized(collective_thrust_cmd.clamp(0.0, 1.0));

        // Roll/Pitch commands are derived from X/Y velocity errors (horizontal)
        // Output is a desired acceleration in NED frame.
        // We need to convert this to desired roll/pitch angles.
        // Simplified: acc_x = g * tan(pitch); acc_y = -g * tan(roll)
        // pitch = atan(acc_x / g)
        // roll = -atan(acc_y / g)

        let g = 9.81;
        let acc_x_cmd = (error.x * self.gains[0]).clamp(-g, g); // Clamp to max accel
        let acc_y_cmd = (error.y * self.gains[1]).clamp(-g, g);

        let pitch_sp = acc_x_cmd.atan2(g); // atan2(acc_x, g) gives pitch
        let roll_sp = -acc_y_cmd.atan2(g); // roll

        // Clamp roll/pitch setpoints to max_roll_pitch
        let roll_sp_clamped = roll_sp.clamp(-self.max_roll_pitch, self.max_roll_pitch);
        let pitch_sp_clamped = pitch_sp.clamp(-self.max_roll_pitch, self.max_roll_pitch);

        // Convert roll/pitch to quaternion setpoint
        // Simplified for small angles: yaw=0 (or keep current yaw)
        // To accurately get a setpoint quaternion, we should rotate the current attitude by the desired roll/pitch correction.
        // For simplicity for now, let's create a new quaternion from desired roll/pitch/current yaw.
        // This is complex, will simplify to direct roll/pitch for now assuming current_att is aligned or represents some target.
        // If current_att is our reference (e.g. current yaw), then we only want to change roll/pitch.

        // This is a common pattern:
        // desired_quat = current_yaw_quat * delta_roll_pitch_quat

        // For now, let's just make a quaternion from desired roll/pitch (assuming 0 yaw).
        // This will result in an absolute attitude setpoint assuming the body X axis aligns with velocity.
        // This is an oversimplification for a real velocity controller but sufficient for a stub.

        let _half_roll = roll_sp_clamped * 0.5;
        let _half_pitch = pitch_sp_clamped * 0.5;
        // For small angles, this is okay. Else, need proper rotation math.

        // Let's create a quaternion from desired roll, pitch, and current yaw (approx).
        // Roll: q_roll = [cos(r/2), sin(r/2), 0, 0]
        // Pitch: q_pitch = [cos(p/2), 0, sin(p/2), 0]
        // Yaw from current_att (extract yaw from current_att)

        // This becomes too complex for a minimal stub.
        // I will return the target roll and pitch rates for the attitude controller.
        // The VelocityController's output should be an attitude setpoint.
        // Let's create a setpoint for a fixed body frame, and then allow the attitude controller to manage it.
        // This is effectively returning the desired body frame based on velocity errors.

        // For now, return a simplified attitude setpoint assuming current yaw from EKF.
        let current_yaw_quat = Quaternion::new(current_att.w, 0.0, 0.0, current_att.z).normalize(); // Extract yaw from current attitude
        let roll_pitch_quat =
            Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), roll_sp_clamped) // Roll rotation
                .mul(&Quaternion::from_axis_angle(
                    Vector3::new(0.0, 1.0, 0.0),
                    pitch_sp_clamped,
                )); // Pitch rotation

        let att_sp_quat = current_yaw_quat.mul(&roll_pitch_quat).normalize();

        (collective, att_sp_quat)
    }
}
