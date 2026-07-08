//! Kernel init/lifecycle state machine (spec §17).
//!
//! Split out of `kernel_types.rs` to keep that file under the
//! 500-line cap; behavior that consumes `InitState` still lives in
//! `kernel_logic.rs`.

use crate::control::ControlLawV1;
use crate::kernel_types::EnumValidationError;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InitState {
    PowerOn = 0,
    ConfigLoading = 1,
    SensorInit = 2,
    EstimatorConverging = 3,
    PreArm = 4,
    Ready = 5,
    Armed = 6,
    Disarmed = 7,
    Fault = 8,
}

impl InitState {
    /// Center-codes for 9 variants (spread across 16-bit space)
    const CODES: &'static [(Self, u16)] = &[
        (InitState::PowerOn, 0x0000),
        (InitState::ConfigLoading, 0x1C71),
        (InitState::SensorInit, 0x38E2),
        (InitState::EstimatorConverging, 0x5553),
        (InitState::PreArm, 0x71C4),
        (InitState::Ready, 0x8E35),
        (InitState::Armed, 0xAAA6),
        (InitState::Disarmed, 0xC717),
        (InitState::Fault, 0xE388),
    ];

    pub fn allows_active_control(&self) -> bool {
        matches!(self, InitState::Armed)
    }

    pub fn forced_control_law(&self) -> Option<ControlLawV1> {
        if self.allows_active_control() {
            None
        } else {
            Some(ControlLawV1::Backup)
        }
    }

    pub fn decode_center(value: u16) -> (Self, u8, bool) {
        let mut best = (InitState::PowerOn, u8::MAX, false);
        for &(state, code) in Self::CODES {
            let d = (value ^ code).count_ones() as u8;
            if d < best.1 {
                best = (state, d, value == code);
            }
        }
        best
    }

    pub const fn to_code(self) -> u16 {
        match self {
            InitState::PowerOn => 0x0000,
            InitState::ConfigLoading => 0x1C71,
            InitState::SensorInit => 0x38E2,
            InitState::EstimatorConverging => 0x5553,
            InitState::PreArm => 0x71C4,
            InitState::Ready => 0x8E35,
            InitState::Armed => 0xAAA6,
            InitState::Disarmed => 0xC717,
            InitState::Fault => 0xE388,
        }
    }
}

impl TryFrom<u16> for InitState {
    type Error = EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (state, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(state)
        } else {
            Err(EnumValidationError)
        }
    }
}

impl TryFrom<u8> for InitState {
    type Error = EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(InitState::PowerOn),
            1 => Ok(InitState::ConfigLoading),
            2 => Ok(InitState::SensorInit),
            3 => Ok(InitState::EstimatorConverging),
            4 => Ok(InitState::PreArm),
            5 => Ok(InitState::Ready),
            6 => Ok(InitState::Armed),
            7 => Ok(InitState::Disarmed),
            8 => Ok(InitState::Fault),
            _ => Err(EnumValidationError),
        }
    }
}

impl crate::replicable::Replicable for InitState {
    const ENCODED_LEN: usize = 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        // Discriminants are explicit on the enum decl (PowerOn=0..Fault=8).
        // Cast through u8 so this is target-endian-independent.
        crate::replicable::copy_into(buf, 0, &[*self as u8])
    }
}
