use crate::state::StateEstimate;
use crate::types::{Normalized, NormalizedSigned, Radians, RadiansPerSecond, Meters, MetersPerSecond, Seconds};
use crate::math::{Quaternion, Vector3};

// Re-exporting for submodules
pub use crate::types::Scalar;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Rate,
    Attitude,
    AltitudeHold,
    PositionHold,
    VelocityControl,
    DeviationTracking,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ControlLaw {
    Normal = 0,
    Alternate1 = 1,
    Alternate2 = 2,
    Direct = 3,
    Frozen = 4,
}

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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CommandSource { Pilot, Autopilot, Gcs, Failsafe }

#[derive(Copy, Clone, Debug)]
pub struct Timestamp {
    pub ticks: u64,
    pub source: TimeSource,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeSource { Internal, Gps, Ptp }

#[derive(Copy, Clone, Debug)]
pub struct SensorOverrides {
    pub gnss_force_state: Option<u8>, // Using u8 as GnssHealth placeholder to avoid circular deps
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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigMode {
    Hover,
    Cruise,
    Transition,
    Degraded,
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
    pub chain: &'static [ControlLaw],
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

pub mod rate;
pub mod attitude;
pub mod position;
pub mod velocity;
pub mod envelope;

#[cfg(feature = "mc")]
pub mod mc;

#[cfg(feature = "fw")]
pub mod fw;

#[cfg(feature = "vtol")]
pub mod vtol;