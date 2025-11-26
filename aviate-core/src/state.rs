use crate::math::Quaternion;
use crate::types::{Meters, MetersPerSecond, RadiansPerSecond};

bitflags::bitflags! {
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub struct StateValidFlags: u8 {
        const ATTITUDE = 0x01;
        const ANGULAR_RATE = 0x02;
        const POSITION = 0x04;
        const VELOCITY = 0x08;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EstimateQuality {
    Good,
    Degraded,
    Unusable,
}

#[derive(Clone, Debug)]
pub struct StateEstimate {
    pub attitude: Quaternion,
    pub angular_velocity: [RadiansPerSecond; 3],
    pub position_ned: [Meters; 3],
    pub velocity_ned: [MetersPerSecond; 3],
    pub quality: EstimateQuality,
    pub valid_flags: StateValidFlags,
}

impl Default for StateEstimate {
    fn default() -> Self {
        Self {
            attitude: Quaternion::IDENTITY,
            angular_velocity: [RadiansPerSecond(0.0); 3],
            position_ned: [Meters(0.0); 3],
            velocity_ned: [MetersPerSecond(0.0); 3],
            quality: EstimateQuality::Unusable,
            valid_flags: StateValidFlags::empty(),
        }
    }
}
