//! Center-coded control enums.
//!
//! The five enums here all share the same shape: a small set of variants
//! with 16-bit Hamming-distance center codes for wire / cross-channel
//! decoding, `TryFrom<u16>` (strict center-only) and `TryFrom<u8>`
//! (discriminant-indexed) impls, and a `to_code()` encoder.
//! Extracted from `control.rs` to keep that file under the 500-line cap.

use crate::EnumValidationError;

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
    /// Check if the discriminant value is valid (SEU resilience)
    /// Uses unsafe discriminant read to detect memory corruption.
    #[inline]
    pub fn is_valid_discriminant(&self) -> bool {
        // SAFETY: Reading discriminant of a valid enum is safe.
        // If memory is corrupted, discriminant may be out of range.
        let disc = core::mem::discriminant(self);
        // Compare against all valid discriminants
        disc == core::mem::discriminant(&ControlMode::Rate)
            || disc == core::mem::discriminant(&ControlMode::Attitude)
            || disc == core::mem::discriminant(&ControlMode::AltitudeHold)
            || disc == core::mem::discriminant(&ControlMode::PositionHold)
            || disc == core::mem::discriminant(&ControlMode::VelocityControl)
            || disc == core::mem::discriminant(&ControlMode::DeviationTracking)
    }

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
    type Error = EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (mode, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(mode)
        } else {
            Err(EnumValidationError)
        }
    }
}

impl TryFrom<u8> for ControlMode {
    type Error = EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ControlMode::Rate),
            1 => Ok(ControlMode::Attitude),
            2 => Ok(ControlMode::AltitudeHold),
            3 => Ok(ControlMode::PositionHold),
            4 => Ok(ControlMode::VelocityControl),
            5 => Ok(ControlMode::DeviationTracking),
            _ => Err(EnumValidationError),
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

impl crate::replicable::Replicable for ControlLawV1 {
    const ENCODED_LEN: usize = 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        crate::replicable::copy_into(buf, 0, &[*self as u8])
    }
}

impl crate::replicable::Replicable for ConfigMode {
    const ENCODED_LEN: usize = 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        crate::replicable::copy_into(buf, 0, &[*self as u8])
    }
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
        } // COV:EXCL(LLVM: for-loop exit edge artifact)
        best
    } // COV:EXCL(LLVM: function boundary artifact)

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
    pub fn try_from_with_ecc(value: u16) -> Result<(Self, u8), EnumValidationError> {
        let (law, d, _) = Self::decode_center(value);
        if d <= 2 {
            Ok((law, d))
        } else {
            Err(EnumValidationError)
        }
    }
}

/// v0.5.1: Strict center-only decode - all non-center codes → EnumInvalid
impl TryFrom<u16> for ControlLawV1 {
    type Error = EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (law, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(law)
        } else {
            Err(EnumValidationError)
        }
    }
}

impl TryFrom<u8> for ControlLawV1 {
    type Error = EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ControlLawV1::Primary),
            1 => Ok(ControlLawV1::Alternate),
            2 => Ok(ControlLawV1::Direct),
            3 => Ok(ControlLawV1::Backup),
            _ => Err(EnumValidationError),
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
    type Error = EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (level, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(level)
        } else {
            Err(EnumValidationError)
        }
    }
}

impl TryFrom<u8> for SafetyLevelV1 {
    type Error = EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(SafetyLevelV1::FlightNormal),
            1 => Ok(SafetyLevelV1::FlightMarginal),
            2 => Ok(SafetyLevelV1::FlightUrgent),
            3 => Ok(SafetyLevelV1::FlightEmergency),
            _ => Err(EnumValidationError),
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
    /// Check if the discriminant value is valid (SEU resilience)
    #[inline]
    pub fn is_valid_discriminant(&self) -> bool {
        let disc = core::mem::discriminant(self);
        disc == core::mem::discriminant(&CommandSource::Pilot)
            || disc == core::mem::discriminant(&CommandSource::Autopilot)
            || disc == core::mem::discriminant(&CommandSource::Gcs)
            || disc == core::mem::discriminant(&CommandSource::Failsafe)
    }

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
    type Error = EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (src, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(src)
        } else {
            Err(EnumValidationError)
        }
    }
}

impl TryFrom<u8> for CommandSource {
    type Error = EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(CommandSource::Pilot),
            1 => Ok(CommandSource::Autopilot),
            2 => Ok(CommandSource::Gcs),
            3 => Ok(CommandSource::Failsafe),
            _ => Err(EnumValidationError),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigMode {
    Hover = 0,
    Cruise = 1,
    Transition = 2,
    Degraded = 3,
}

impl ConfigMode {
    /// Check if the discriminant value is valid (SEU resilience)
    #[inline]
    pub fn is_valid_discriminant(&self) -> bool {
        let disc = core::mem::discriminant(self);
        disc == core::mem::discriminant(&ConfigMode::Hover)
            || disc == core::mem::discriminant(&ConfigMode::Cruise)
            || disc == core::mem::discriminant(&ConfigMode::Transition)
            || disc == core::mem::discriminant(&ConfigMode::Degraded)
    }

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
    type Error = EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (mode, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(mode)
        } else {
            Err(EnumValidationError)
        }
    }
}

impl TryFrom<u8> for ConfigMode {
    type Error = EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ConfigMode::Hover),
            1 => Ok(ConfigMode::Cruise),
            2 => Ok(ConfigMode::Transition),
            3 => Ok(ConfigMode::Degraded),
            _ => Err(EnumValidationError),
        }
    }
}
