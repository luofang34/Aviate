use crate::math::Quaternion;
use crate::sensor::GnssHealth;
use crate::state::StateEstimate;
use crate::types::{
    Meters, MetersPerSecond, Normalized, NormalizedSigned, Radians, RadiansPerSecond,
};

// Re-exporting for submodules
pub use crate::types::Scalar;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Rate = 0,
    Attitude = 1,
    AltitudeHold = 2,
    PositionHold = 3,
    VelocityControl = 4,
    DeviationTracking = 5,
}

impl ControlMode {
    /// Center-codes for 6 variants (spaced across 16-bit range)
    const CODES: &'static [(Self, u16)] = &[
        (ControlMode::Rate, 0x0000),
        (ControlMode::Attitude, 0x2222),
        (ControlMode::AltitudeHold, 0x4444),
        (ControlMode::PositionHold, 0x6666),
        (ControlMode::VelocityControl, 0x8888),
        (ControlMode::DeviationTracking, 0xAAAA),
    ];

    pub fn decode_center(value: u16) -> (Self, u8, bool) {
        let mut best = (ControlMode::Rate, u8::MAX, false);
        for &(mode, code) in Self::CODES {
            let d = (value ^ code).count_ones() as u8;
            if d < best.1 {
                best = (mode, d, value == code);
            }
        }
        best
    }

    pub const fn to_code(self) -> u16 {
        match self {
            ControlMode::Rate => 0x0000,
            ControlMode::Attitude => 0x2222,
            ControlMode::AltitudeHold => 0x4444,
            ControlMode::PositionHold => 0x6666,
            ControlMode::VelocityControl => 0x8888,
            ControlMode::DeviationTracking => 0xAAAA,
        }
    }
}

impl TryFrom<u16> for ControlMode {
    type Error = crate::EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (mode, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(mode)
        } else {
            Err(crate::EnumValidationError)
        }
    }
}

impl TryFrom<u8> for ControlMode {
    type Error = crate::EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ControlMode::Rate),
            1 => Ok(ControlMode::Attitude),
            2 => Ok(ControlMode::AltitudeHold),
            3 => Ok(ControlMode::PositionHold),
            4 => Ok(ControlMode::VelocityControl),
            5 => Ok(ControlMode::DeviationTracking),
            _ => Err(crate::EnumValidationError),
        }
    }
}

/// Control law capability: what control strategies are available.
/// NOTE: ControlLawV1 describes flight control capability, NOT safety/risk level.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ControlLawV1 {
    /// Full envelope protection, all loops active
    Primary = 0,
    /// Reduced protections, degraded but flyable
    Alternate = 1,
    /// Manual with minimal augmentation
    Direct = 2,
    /// Last-ditch stability only
    Backup = 3,
}

impl ControlLawV1 {
    /// Center-codes with maximum Hamming distance (≥8 bits between any pair)
    const CODES: &'static [(Self, u16)] = &[
        (ControlLawV1::Primary, 0x0000),
        (ControlLawV1::Alternate, 0x5555),
        (ControlLawV1::Direct, 0xAAAA),
        (ControlLawV1::Backup, 0xFFFF),
    ];

    /// Get the severity level (higher = more degraded)
    ///
    /// Used to determine if a transition is a degradation.
    pub fn severity(&self) -> u8 {
        *self as u8
    }

    /// Decode with Hamming distance calculation (for wire/cross-channel decode)
    ///
    /// Returns (nearest_enum, hamming_distance, is_exact_center)
    pub fn decode_center(value: u16) -> (Self, u8, bool) {
        let mut best = (ControlLawV1::Primary, u8::MAX, false);
        for &(law, code) in Self::CODES {
            let d = (value ^ code).count_ones() as u8;
            if d < best.1 {
                best = (law, d, value == code);
            }
        }
        best
    }

    /// Returns the 16-bit center-code for this variant
    pub const fn to_code(self) -> u16 {
        match self {
            ControlLawV1::Primary => 0x0000,
            ControlLawV1::Alternate => 0x5555,
            ControlLawV1::Direct => 0xAAAA,
            ControlLawV1::Backup => 0xFFFF,
        }
    }

    /// Future: ECC decode allowing 1-2 bit correction
    #[allow(dead_code)]
    pub fn try_from_with_ecc(value: u16) -> Result<(Self, u8), crate::EnumValidationError> {
        let (law, d, _) = Self::decode_center(value);
        if d <= 2 {
            Ok((law, d))
        } else {
            Err(crate::EnumValidationError)
        }
    }
}

/// v0.5.1: Strict center-only decode - all non-center codes → EnumInvalid
impl TryFrom<u16> for ControlLawV1 {
    type Error = crate::EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (law, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(law)
        } else {
            Err(crate::EnumValidationError)
        }
    }
}

impl TryFrom<u8> for ControlLawV1 {
    type Error = crate::EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ControlLawV1::Primary),
            1 => Ok(ControlLawV1::Alternate),
            2 => Ok(ControlLawV1::Direct),
            3 => Ok(ControlLawV1::Backup),
            _ => Err(crate::EnumValidationError),
        }
    }
}

/// Safety level: whole-aircraft situational risk assessment.
/// Orthogonal to control law capability and channel health.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SafetyLevelV1 {
    /// Normal flight with adequate margins (altitude, fuel, divert options)
    FlightNormal = 0,
    /// Margins noticeably reduced (takeoff/landing, config change, oceanic, low fuel)
    FlightMarginal = 1,
    /// Urgent but controllable, analogous to "PAN-PAN"
    FlightUrgent = 2,
    /// Life/platform threatening, analogous to "MAYDAY"
    FlightEmergency = 3,
}

impl SafetyLevelV1 {
    const CODES: &'static [(Self, u16)] = &[
        (SafetyLevelV1::FlightNormal, 0x0000),
        (SafetyLevelV1::FlightMarginal, 0x5555),
        (SafetyLevelV1::FlightUrgent, 0xAAAA),
        (SafetyLevelV1::FlightEmergency, 0xFFFF),
    ];

    pub fn decode_center(value: u16) -> (Self, u8, bool) {
        let mut best = (SafetyLevelV1::FlightNormal, u8::MAX, false);
        for &(level, code) in Self::CODES {
            let d = (value ^ code).count_ones() as u8;
            if d < best.1 {
                best = (level, d, value == code);
            }
        }
        best
    }

    pub const fn to_code(self) -> u16 {
        match self {
            SafetyLevelV1::FlightNormal => 0x0000,
            SafetyLevelV1::FlightMarginal => 0x5555,
            SafetyLevelV1::FlightUrgent => 0xAAAA,
            SafetyLevelV1::FlightEmergency => 0xFFFF,
        }
    }
}

impl TryFrom<u16> for SafetyLevelV1 {
    type Error = crate::EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (level, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(level)
        } else {
            Err(crate::EnumValidationError)
        }
    }
}

impl TryFrom<u8> for SafetyLevelV1 {
    type Error = crate::EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(SafetyLevelV1::FlightNormal),
            1 => Ok(SafetyLevelV1::FlightMarginal),
            2 => Ok(SafetyLevelV1::FlightUrgent),
            3 => Ok(SafetyLevelV1::FlightEmergency),
            _ => Err(crate::EnumValidationError),
        }
    }
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
pub enum CommandSource {
    Pilot = 0,
    Autopilot = 1,
    Gcs = 2,
    Failsafe = 3,
}

impl CommandSource {
    const CODES: &'static [(Self, u16)] = &[
        (CommandSource::Pilot, 0x0000),
        (CommandSource::Autopilot, 0x5555),
        (CommandSource::Gcs, 0xAAAA),
        (CommandSource::Failsafe, 0xFFFF),
    ];

    pub fn decode_center(value: u16) -> (Self, u8, bool) {
        let mut best = (CommandSource::Pilot, u8::MAX, false);
        for &(src, code) in Self::CODES {
            let d = (value ^ code).count_ones() as u8;
            if d < best.1 {
                best = (src, d, value == code);
            }
        }
        best
    }

    pub const fn to_code(self) -> u16 {
        match self {
            CommandSource::Pilot => 0x0000,
            CommandSource::Autopilot => 0x5555,
            CommandSource::Gcs => 0xAAAA,
            CommandSource::Failsafe => 0xFFFF,
        }
    }
}

impl TryFrom<u16> for CommandSource {
    type Error = crate::EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (src, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(src)
        } else {
            Err(crate::EnumValidationError)
        }
    }
}

impl TryFrom<u8> for CommandSource {
    type Error = crate::EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(CommandSource::Pilot),
            1 => Ok(CommandSource::Autopilot),
            2 => Ok(CommandSource::Gcs),
            3 => Ok(CommandSource::Failsafe),
            _ => Err(crate::EnumValidationError),
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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigMode {
    Hover = 0,
    Cruise = 1,
    Transition = 2,
    Degraded = 3,
}

impl ConfigMode {
    const CODES: &'static [(Self, u16)] = &[
        (ConfigMode::Hover, 0x0000),
        (ConfigMode::Cruise, 0x5555),
        (ConfigMode::Transition, 0xAAAA),
        (ConfigMode::Degraded, 0xFFFF),
    ];

    pub fn decode_center(value: u16) -> (Self, u8, bool) {
        let mut best = (ConfigMode::Hover, u8::MAX, false);
        for &(mode, code) in Self::CODES {
            let d = (value ^ code).count_ones() as u8;
            if d < best.1 {
                best = (mode, d, value == code);
            }
        }
        best
    }

    pub const fn to_code(self) -> u16 {
        match self {
            ConfigMode::Hover => 0x0000,
            ConfigMode::Cruise => 0x5555,
            ConfigMode::Transition => 0xAAAA,
            ConfigMode::Degraded => 0xFFFF,
        }
    }
}

impl TryFrom<u16> for ConfigMode {
    type Error = crate::EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (mode, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(mode)
        } else {
            Err(crate::EnumValidationError)
        }
    }
}

impl TryFrom<u8> for ConfigMode {
    type Error = crate::EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ConfigMode::Hover),
            1 => Ok(ConfigMode::Cruise),
            2 => Ok(ConfigMode::Transition),
            3 => Ok(ConfigMode::Degraded),
            _ => Err(crate::EnumValidationError),
        }
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
pub mod mc;

#[cfg(feature = "fw")]
pub mod fw;

#[cfg(feature = "vtol")]
pub mod vtol;
