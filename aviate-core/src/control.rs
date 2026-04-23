//! Flight-control surface: enums, setpoints, commands, limits, and the
//! `VehicleController` trait the kernel drives every cycle.
//!
//! Center-coded enums (ControlMode / ControlLawV1 / SafetyLevelV1 /
//! CommandSource / ConfigMode) live in the [`enums`] submodule to keep
//! this file under the 500-line cap; everything else (Setpoint, Command,
//! Limits, AxisCommand, VehicleController, …) stays here.

use crate::math::Quaternion;
use crate::sensor::GnssHealth;
use crate::state::StateEstimate;
use crate::types::{
    Meters, MetersPerSecond, Normalized, NormalizedSigned, Radians, RadiansPerSecond,
};

// Re-exporting for submodules
pub use crate::types::Scalar;

pub mod enums;
pub use enums::{CommandSource, ConfigMode, ControlLawV1, ControlMode, SafetyLevelV1};

#[derive(Clone, Debug)]
pub struct Setpoint {
    pub attitude: Option<Quaternion>,
    pub angular_rate: Option<[RadiansPerSecond; 3]>,
    pub altitude: Option<Meters>,
    pub vertical_speed: Option<MetersPerSecond>,
    pub heading: Option<Radians>,
    pub position: Option<[Meters; 3]>,
    pub velocity: Option<[MetersPerSecond; 3]>,
    pub lateral_deviation: Option<Meters>,
    pub vertical_deviation: Option<Meters>,
    pub collective_thrust: Normalized,
}

impl Default for Setpoint {
    fn default() -> Self {
        Self {
            attitude: None,
            angular_rate: None,
            altitude: None,
            vertical_speed: None,
            heading: None,
            position: None,
            velocity: None,
            lateral_deviation: None,
            vertical_deviation: None,
            collective_thrust: Normalized(0.0),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Timestamp {
    pub ticks: u64,
    pub source: TimeSource,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeSource {
    Internal,
    Gps,
    Ptp,
}

#[derive(Copy, Clone, Debug)]
pub struct SensorOverrides {
    pub gnss_force_state: Option<GnssHealth>, // None = no override
}

#[derive(Clone, Debug)]
pub struct Command {
    pub mode: ControlMode,
    pub setpoint: Setpoint,
    pub config_mode_request: Option<ConfigMode>,
    pub sensor_overrides: Option<SensorOverrides>,
    // pub timestamp: Timestamp, // Removed for now as Timestamp is not fully defined in this file context
    pub sequence: u32,
    pub source: CommandSource,
}

impl Command {
    /// Validate all enum fields for SEU resilience (Spec §15.3)
    ///
    /// Checks that enum discriminants are within valid ranges.
    /// Returns true if all fields are valid, false if any corruption detected.
    /// This is a fast O(1) operation that checks discriminant values.
    #[inline]
    pub fn validate_enums(&self) -> bool {
        // COV:EXCL_START(DEFENSIVE: SEU/memory corruption detection - cannot trigger in unit tests)
        // Check ControlMode discriminant (0-5)
        if !self.mode.is_valid_discriminant() {
            return false;
        }

        // Check CommandSource discriminant (0-3)
        if !self.source.is_valid_discriminant() {
            return false;
        }

        // Check ConfigMode if present (0-3)
        if let Some(config_mode) = self.config_mode_request {
            if !config_mode.is_valid_discriminant() {
                return false;
            }
        }
        // COV:EXCL_STOP

        true
    }
}

#[derive(Clone, Debug)]
pub struct Limits {
    pub max_roll: Radians,
    pub max_pitch: Radians,
    pub max_roll_rate: RadiansPerSecond,
    pub max_pitch_rate: RadiansPerSecond,
    pub max_yaw_rate: RadiansPerSecond,
    pub max_horizontal_speed: MetersPerSecond,
    pub max_climb_rate: MetersPerSecond,
    pub max_descent_rate: MetersPerSecond,
    pub max_altitude: Meters,
    pub min_altitude: Meters,
    pub min_airspeed: Option<MetersPerSecond>,
    pub max_airspeed: Option<MetersPerSecond>,
    pub max_load_factor: Scalar,
    pub min_load_factor: Scalar,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AuthorityProfile {
    HardEnvelope,
    SoftEnvelope,
}

#[derive(Clone, Debug)]
pub struct LawProfile {
    pub authority: AuthorityProfile,
    pub chain: &'static [ControlLawV1],
    // capabilities...
}

#[derive(Clone, Debug)]
pub struct AxisCommand {
    pub roll: NormalizedSigned,
    pub pitch: NormalizedSigned,
    pub yaw: NormalizedSigned,
    pub collective: Normalized,
}

pub trait VehicleController {
    fn step(
        &mut self,
        state: &StateEstimate,
        command: &Command, // Note: This now refers to the new Command struct
        mode: ConfigMode,
        limits: &Limits,
    ) -> AxisCommand;
}

pub mod attitude;
pub mod envelope;
pub mod position;
pub mod rate;
pub mod velocity;

#[cfg(feature = "mc")]
pub mod multirotor;

#[cfg(feature = "fw")]
pub mod fixed_wing;

#[cfg(feature = "vtol")]
pub mod vtol;
