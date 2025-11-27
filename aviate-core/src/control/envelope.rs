use crate::types::{Scalar, Meters, MetersPerSecond, Radians, RadiansPerSecond, FloatExt};
use crate::state::StateEstimate;
use crate::control::{Setpoint, Limits, AuthorityProfile};
use crate::math::{Quaternion, Vector3};

bitflags::bitflags! {
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub struct AxisLimitFlags: u8 {
        const ROLL = 0x01;
        const PITCH = 0x02;
        const YAW = 0x04;
        const ALTITUDE = 0x08;
        const SPEED = 0x10;
        const LOAD_FACTOR = 0x20;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct EnvelopeMargin {
    pub roll_rad: Radians,
    pub pitch_rad: Radians,
    pub yaw_rate_rad_s: RadiansPerSecond,
    pub altitude_m: Meters,
    pub airspeed_mps: MetersPerSecond,
    pub load_factor: Scalar,
}

#[derive(Copy, Clone, Debug)]
pub struct ProtectionStatus {
    pub limited_axes: AxisLimitFlags,
    pub saturated: bool,
}

pub trait EnvelopeProtector {
    fn constrain(
        &self,
        raw_sp: &Setpoint,
        state: &StateEstimate,
        limits: &Limits,
        authority: AuthorityProfile,
    ) -> (Setpoint, ProtectionStatus);
}

pub struct SimpleEnvelopeProtector;

impl EnvelopeProtector for SimpleEnvelopeProtector {
    fn constrain(
        &self,
        raw_sp: &Setpoint,
        _state: &StateEstimate, // State can be used for dynamic limits or alpha protection
        limits: &Limits,
        _authority: AuthorityProfile, // Could relax limits if SoftEnvelope
    ) -> (Setpoint, ProtectionStatus) {
        let mut sp = raw_sp.clone();
        let mut flags = AxisLimitFlags::empty();
        
        // 1. Roll/Pitch Limits (Attitude)
        if let Some(att) = &mut sp.attitude {
            let (r, p, y) = att.to_euler();
            let mut modified = false;
            
            let mut r_clamped = r;
            if r > limits.max_roll.0 {
                r_clamped = limits.max_roll.0;
                flags.insert(AxisLimitFlags::ROLL);
                modified = true;
            } else if r < -limits.max_roll.0 {
                r_clamped = -limits.max_roll.0;
                flags.insert(AxisLimitFlags::ROLL);
                modified = true;
            }
            
            let mut p_clamped = p;
            if p > limits.max_pitch.0 {
                p_clamped = limits.max_pitch.0;
                flags.insert(AxisLimitFlags::PITCH);
                modified = true;
            } else if p < -limits.max_pitch.0 {
                p_clamped = -limits.max_pitch.0;
                flags.insert(AxisLimitFlags::PITCH);
                modified = true;
            }
            
            if modified {
                 // Reconstruct quaternion from clamped euler (Z-Y-X sequence)
                 let qx = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), r_clamped);
                 let qy = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), p_clamped);
                 let qz = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), y); 
                 
                 *att = qz.mul(&qy).mul(&qx);
            }
        }

        // 1b. Angular Rate Limits
        if let Some(rates) = &mut sp.angular_rate {
            // rates: [RadiansPerSecond; 3] -> Roll, Pitch, Yaw rates
            // limits: max_roll_rate, max_pitch_rate, max_yaw_rate
            
            // Roll rate
            if rates[0].0 > limits.max_roll_rate.0 {
                rates[0].0 = limits.max_roll_rate.0;
                flags.insert(AxisLimitFlags::ROLL); // Reusing ROLL flag for rate too? Spec says AxisLimitFlags::ROLL.
            } else if rates[0].0 < -limits.max_roll_rate.0 {
                rates[0].0 = -limits.max_roll_rate.0;
                flags.insert(AxisLimitFlags::ROLL);
            }

            // Pitch rate
            if rates[1].0 > limits.max_pitch_rate.0 {
                rates[1].0 = limits.max_pitch_rate.0;
                flags.insert(AxisLimitFlags::PITCH);
            } else if rates[1].0 < -limits.max_pitch_rate.0 {
                rates[1].0 = -limits.max_pitch_rate.0;
                flags.insert(AxisLimitFlags::PITCH);
            }

            // Yaw rate
            if rates[2].0 > limits.max_yaw_rate.0 {
                rates[2].0 = limits.max_yaw_rate.0;
                flags.insert(AxisLimitFlags::YAW);
            } else if rates[2].0 < -limits.max_yaw_rate.0 {
                rates[2].0 = -limits.max_yaw_rate.0;
                flags.insert(AxisLimitFlags::YAW);
            }
        }
        
        // 2. Altitude Limits
        if let Some(alt) = &mut sp.altitude {
             if alt.0 > limits.max_altitude.0 {
                 alt.0 = limits.max_altitude.0;
                 flags.insert(AxisLimitFlags::ALTITUDE);
             }
             if alt.0 < limits.min_altitude.0 {
                 alt.0 = limits.min_altitude.0;
                 flags.insert(AxisLimitFlags::ALTITUDE);
             }
        }
        
        // 3. Vertical Speed Limits
        if let Some(vz) = &mut sp.vertical_speed {
            // NED: +vz is down (descent), -vz is up (climb)
            // max_climb_rate (positive value) -> limit vz to -max_climb_rate
            // max_descent_rate (positive value) -> limit vz to max_descent_rate
            
            let min_vz = -limits.max_climb_rate.0;
            let max_vz = limits.max_descent_rate.0;
            
            if vz.0 < min_vz {
                vz.0 = min_vz;
                flags.insert(AxisLimitFlags::SPEED);
            } else if vz.0 > max_vz {
                vz.0 = max_vz;
                flags.insert(AxisLimitFlags::SPEED);
            }
        }
        
        // 4. Horizontal Speed Limits
        // This usually requires checking velocity setpoint magnitude
        if let Some(vel) = &mut sp.velocity {
             // vel is [MetersPerSecond; 3] (NED)
             let vx = vel[0].0;
             let vy = vel[1].0;
             let h_speed_sq = vx*vx + vy*vy;
             let max_sq = limits.max_horizontal_speed.0 * limits.max_horizontal_speed.0;
             
             if h_speed_sq > max_sq {
                 let scale = limits.max_horizontal_speed.0 / h_speed_sq.sqrt();
                 vel[0].0 *= scale;
                 vel[1].0 *= scale;
                 flags.insert(AxisLimitFlags::SPEED);
             }
        }

        (sp, ProtectionStatus { limited_axes: flags, saturated: !flags.is_empty() })
    }
}
