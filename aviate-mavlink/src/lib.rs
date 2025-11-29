//! MAVLink message parsing/serialization for HIL simulation
//!
//! Supports the subset of MAVLink 2.0 messages needed for SITL:
//! - HIL_SENSOR (#107) - IMU/baro/mag from simulator
//! - HIL_GPS (#113) - GNSS from simulator
//! - HIL_ACTUATOR_CONTROLS (#93) - Motor outputs to simulator
//! - HEARTBEAT (#0) - Keep-alive
//!
//! This crate is `no_std` compatible with no allocations.

#![no_std]
#![forbid(unsafe_code)]

pub mod messages;
pub mod parser;
pub mod serialize;

pub use messages::*;
pub use parser::{parse_mavlink, ParseError};
pub use serialize::serialize_mavlink;

/// MAVLink 2.0 start byte
pub const MAVLINK_STX_V2: u8 = 0xFD;

/// MAVLink 1.0 start byte (for compatibility detection)
pub const MAVLINK_STX_V1: u8 = 0xFE;

/// Maximum MAVLink message payload size
pub const MAX_PAYLOAD_LEN: usize = 255;

/// System ID for Aviate autopilot
pub const AVIATE_SYSTEM_ID: u8 = 1;

/// Component ID for Aviate autopilot
pub const AVIATE_COMPONENT_ID: u8 = 1;

/// MAVLink component types
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavComponent {
    Autopilot = 1,
    Camera = 100,
    Gimbal = 154,
    Gcs = 190,
}

/// MAVLink autopilot types
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavAutopilot {
    Generic = 0,
    Px4 = 12,
    Ardupilot = 3,
    Aviate = 18, // Custom ID for Aviate
}

/// MAVLink system type
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavType {
    Generic = 0,
    FixedWing = 1,
    Quadrotor = 2,
    Coaxial = 3,
    Helicopter = 4,
    Hexarotor = 13,
    Octorotor = 14,
    Vtol = 19,
    Gcs = 6,
}

/// MAVLink system state
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavState {
    Uninit = 0,
    Boot = 1,
    Calibrating = 2,
    Standby = 3,
    Active = 4,
    Critical = 5,
    Emergency = 6,
    Poweroff = 7,
    FlightTermination = 8,
}

/// MAVLink mode flags
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MavModeFlag(pub u8);

impl MavModeFlag {
    pub const CUSTOM_MODE_ENABLED: Self = Self(1);
    pub const TEST_ENABLED: Self = Self(2);
    pub const AUTO_ENABLED: Self = Self(4);
    pub const GUIDED_ENABLED: Self = Self(8);
    pub const STABILIZE_ENABLED: Self = Self(16);
    pub const HIL_ENABLED: Self = Self(32);
    pub const MANUAL_INPUT_ENABLED: Self = Self(64);
    pub const SAFETY_ARMED: Self = Self(128);
}

/// MAV_CMD constants
pub mod mav_cmd {
    pub const NAV_LAND: u16 = 21;
    pub const NAV_TAKEOFF: u16 = 22;
    pub const DO_SET_MODE: u16 = 176;
    pub const COMPONENT_ARM_DISARM: u16 = 400;
}

/// MAV_RESULT constants
pub mod mav_result {
    pub const ACCEPTED: u8 = 0;
    pub const TEMPORARILY_REJECTED: u8 = 1;
    pub const DENIED: u8 = 2;
    pub const UNSUPPORTED: u8 = 3;
    pub const FAILED: u8 = 4;
    pub const IN_PROGRESS: u8 = 5;
    pub const CANCELLED: u8 = 6;
}
