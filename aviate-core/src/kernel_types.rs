//! Kernel type definitions and constants.
//!
//! Every type here is a pure data shape used by `AviateKernelImpl` and
//! its public `AviateKernelTrait`. Behavior lives in `kernel.rs`,
//! `kernel_logic.rs`, and `kernel_trait.rs`.

use crate::checks::{DegradationReason, TransitionFailure};
use crate::control::envelope::ProtectionStatus;
use crate::control::{ConfigMode, ControlLawV1, ControlMode, ModeEntryDecision};
use crate::fault::FaultFlags;
use crate::mixer::{ActuatorCmd, SanitizeReport};
use crate::state::{EstimateQuality, StateEstimate};
use crate::time::Timestamp;
use crate::types::{Meters, MetersPerSecond, Radians, RadiansPerSecond};

// --- Critical faults ---

/// Critical faults that trigger immediate fault state entry
///
/// These are faults that require the aircraft to enter a safe state immediately.
/// The kernel will transition to InitState::Fault when any of these are detected.
pub const CRITICAL_FAULTS: FaultFlags = FaultFlags::ALL_IMU_FAILED
    .union(FaultFlags::NUMERIC_ERROR)
    .union(FaultFlags::ESTIMATOR_DIVERGED);

/// Default command timeout in milliseconds
pub const DEFAULT_COMMAND_TIMEOUT_MS: u32 = 500;

// --- Spec §18: Timing Constants ---

/// Control loop period in microseconds (1 kHz)
pub const CONTROL_LOOP_PERIOD_US: u64 = 1000;

/// Control loop deadline in microseconds (80% utilization)
pub const CONTROL_LOOP_DEADLINE_US: u64 = 800;

/// Number of consecutive timing violations before degradation
pub const TIMING_VIOLATION_THRESHOLD: u32 = 3;

// --- Spec §15.3: Enum Validation Error ---

/// Error returned when enum validation fails (spec §15.3)
///
/// Used by TryFrom implementations for control-plane enums
/// to detect SEU (Single Event Upset) corruption.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EnumValidationError;

// --- Spec §19: Configuration Types ---

/// Configuration block for loading from storage (spec §19 - stub for now)
///
/// NOTE: Using &'static per spec. For test injection from RAM, use helper wrapper.
#[derive(Clone, Debug)]
pub struct ConfigBlock {
    pub data: &'static [u8],
    pub version: u16,
    pub checksum: u32,
}

/// Error returned when configuration loading fails
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// Configuration format is invalid
    InvalidFormat,
    /// Configuration version not supported
    UnsupportedVersion,
    /// Parameter value out of valid range
    OutOfRange,
    /// Checksum verification failed
    ChecksumMismatch,
}

// `Config` (a {limits, command_timeout_ms} placeholder) was removed in
// the Phase 1 ResolvedKernelConfig consolidation. The flight-period
// configuration surface now lives at `crate::kernel::config::ResolvedKernelConfig`,
// which is the single source of truth for limits, mode_config,
// fault_table, command_timeout_ms, and safe_output.
//
// `AviateKernelTrait::get_config()` correspondingly returns
// `&ResolvedKernelConfig` — see `kernel_trait.rs`.

// --- Init state machine ---

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

#[derive(Clone, Debug)]
pub struct InitResult {
    pub state: InitState,
    pub faults: FaultFlags,
    pub ready: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ArmError {
    NotReady,
    Faulted,
    AlreadyArmed,
    ConfigInvalid,
    InFaultState,
}

/// Error returned when attempting a configuration mode transition
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TransitionError {
    /// Not armed (transitions only allowed while armed)
    NotArmed,
    /// Already transitioning to another mode
    AlreadyTransitioning,
    /// Transition checks failed
    ChecksFailed(TransitionFailure),
    /// Target mode same as current mode
    AlreadyInMode,
    /// System in fault state
    InFaultState,
}

// --- Spec §16: Channel & Redundancy Model ---

/// Channel identifier for redundant flight controllers
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ChannelId(pub u8);

impl ChannelId {
    pub const PRIMARY: Self = Self(0);
    pub const SECONDARY: Self = Self(1);
    pub const TERTIARY: Self = Self(2);
    pub const MAX_CHANNELS: usize = 3;
}

/// Multi-channel container for **derived** cross-channel signals
/// — estimates, health, commands, sequences. Used WITHIN
/// redundant mode for consensus / voting / FDIR; agreement model
/// is value-domain (a 0.001-m difference between two channels'
/// position estimates is normal). Distinct from
/// [`crate::kernel::snapshot::ChannelSnapshot`], which is the
/// byte-domain lockstep-ENTRY witness; see that module for the
/// full role boundary. Spec §16.
#[derive(Clone, Debug)]
pub struct CrossChannelData {
    pub estimates: [Option<StateEstimate>; ChannelId::MAX_CHANNELS],
    pub health: [Option<ChannelHealthV1>; ChannelId::MAX_CHANNELS],
    pub commands: [Option<ActuatorCmd>; ChannelId::MAX_CHANNELS],
    pub sequences: [Option<u32>; ChannelId::MAX_CHANNELS],
}

/// Channel health status (spec §16)
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum ChannelHealthV1 {
    /// Channel fully operational
    #[default]
    Operative,
    /// Channel functional but with reduced capability
    Degraded,
    /// Channel has failed
    Failed,
    /// Channel in test/maintenance mode
    Offline,
}

impl ChannelHealthV1 {
    /// Center-codes with maximum Hamming distance (≥8 bits between any pair)
    const CODES: &'static [(Self, u16)] = &[
        (ChannelHealthV1::Operative, 0x0000),
        (ChannelHealthV1::Degraded, 0x5555),
        (ChannelHealthV1::Failed, 0xAAAA),
        (ChannelHealthV1::Offline, 0xFFFF),
    ];

    /// Decode with Hamming distance calculation (for wire/cross-channel decode)
    ///
    /// Returns (nearest_enum, hamming_distance, is_exact_center)
    pub fn decode_center(value: u16) -> (Self, u8, bool) {
        let mut best = (ChannelHealthV1::Operative, u8::MAX, false);
        for &(health, code) in Self::CODES {
            let d = (value ^ code).count_ones() as u8;
            if d < best.1 {
                best = (health, d, value == code);
            }
        }
        best
    }

    /// Returns the 16-bit center-code for this variant
    pub const fn to_code(self) -> u16 {
        match self {
            ChannelHealthV1::Operative => 0x0000,
            ChannelHealthV1::Degraded => 0x5555,
            ChannelHealthV1::Failed => 0xAAAA,
            ChannelHealthV1::Offline => 0xFFFF,
        }
    }
}

/// v0.5.1: Strict center-only decode - all non-center codes → EnumInvalid
impl TryFrom<u16> for ChannelHealthV1 {
    type Error = EnumValidationError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let (health, _d, is_center) = Self::decode_center(value);
        if is_center {
            Ok(health)
        } else {
            Err(EnumValidationError)
        }
    }
}

impl TryFrom<u8> for ChannelHealthV1 {
    type Error = EnumValidationError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ChannelHealthV1::Operative),
            1 => Ok(ChannelHealthV1::Degraded),
            2 => Ok(ChannelHealthV1::Failed),
            3 => Ok(ChannelHealthV1::Offline),
            _ => Err(EnumValidationError),
        }
    }
}

// --- Spec §18: Timing ---

/// Timing statistics for control loop
#[derive(Copy, Clone, Debug, Default)]
pub struct TimingStats {
    pub last_cycle_us: u32,
    pub max_cycle_us: u32,
    pub min_cycle_us: u32,
    pub deadline_violations: u32,
    pub consecutive_violations: u32,
    pub total_cycles: u64,
}

impl crate::replicable::Replicable for TimingStats {
    // 5 × u32 + 1 × u64 = 28 bytes.
    const ENCODED_LEN: usize = 5 * 4 + 8;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = 0usize;
        w += crate::replicable::copy_into(buf, w, &self.last_cycle_us.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.max_cycle_us.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.min_cycle_us.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.deadline_violations.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.consecutive_violations.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.total_cycles.to_le_bytes());
        w
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

/// Per-cycle timing information
#[derive(Copy, Clone, Debug)]
pub struct CycleTiming {
    pub cycle_start_us: u32,
    pub cycle_end_us: u32,
    pub duration_us: u32,
    pub deadline_met: bool,
}

impl Default for CycleTiming {
    fn default() -> Self {
        Self {
            cycle_start_us: 0,
            cycle_end_us: 0,
            duration_us: 0,
            deadline_met: true,
        }
    }
}

// --- Spec §13: Envelope Margin ---

/// Remaining margin before limit breach (positive = margin remaining)
#[derive(Copy, Clone, Debug, Default)]
pub struct EnvelopeMargin {
    pub roll_rad: Radians,
    pub pitch_rad: Radians,
    pub yaw_rate_rad_s: RadiansPerSecond,
    pub altitude_m: Meters,
    pub airspeed_mps: MetersPerSecond,
    pub load_factor: f32,
}

// --- Spec §14: Degradation ---

#[derive(Copy, Clone, Debug)]
pub struct DegradationEvent {
    pub from: ControlLawV1,
    pub to: ControlLawV1,
    pub reason: DegradationReason,
    pub timestamp: Timestamp,
}

// --- Spec §4.4: Configuration Transition ---

/// Configuration transition state for morphing aircraft
#[derive(Clone, Debug)]
pub enum ConfigTransitionState {
    /// Stable in a configuration mode
    Stable(ConfigMode),
    /// Actively transitioning between modes
    Switching {
        from: ConfigMode,
        to: ConfigMode,
        progress: f32,
    },
    /// Transition failed
    Failed {
        intended: ConfigMode,
        actual: ConfigMode,
        reason: TransitionFailure,
    },
}

impl Default for ConfigTransitionState {
    fn default() -> Self {
        Self::Stable(ConfigMode::Hover)
    }
}

// --- Spec §16: Channel Status ---

/// Full per-cycle status from kernel
#[derive(Clone, Debug)]
pub struct ChannelStatus {
    /// Mode actually driving the cascade this cycle — the
    /// estimator-validity-gated mode (see `mode_entry`), not
    /// necessarily what was requested. Reporting the raw request here
    /// even when the gate refused it would be a silent lie about what
    /// the vehicle is actually doing.
    pub mode: ControlMode,
    pub config_mode: ConfigMode,
    pub transition_state: ConfigTransitionState,
    pub law: ControlLawV1,
    pub health: ChannelHealthV1,
    pub faults: FaultFlags,
    pub confidence: EstimateQuality,
    pub envelope_margin: EnvelopeMargin,
    pub sequence: u32,
    pub protection: ProtectionStatus,
    pub sanitize_report: SanitizeReport,
    /// Estimator-validity mode-entry gate outcome for this cycle:
    /// the requested mode, the effective mode (`mode` above), and —
    /// when they differ — the validity bits the requested mode was
    /// short of. Lets an OEM mode manager see why a mode was refused
    /// instead of only observing the substituted behavior.
    pub mode_entry: ModeEntryDecision,
}

impl Default for ChannelStatus {
    fn default() -> Self {
        Self {
            mode: ControlMode::Rate,
            config_mode: ConfigMode::Hover,
            transition_state: ConfigTransitionState::default(),
            law: ControlLawV1::Primary,
            health: ChannelHealthV1::Operative,
            faults: FaultFlags::empty(),
            confidence: EstimateQuality::Good,
            envelope_margin: EnvelopeMargin::default(),
            sequence: 0,
            protection: ProtectionStatus::default(),
            sanitize_report: SanitizeReport::default(),
            mode_entry: ModeEntryDecision::Granted(ControlMode::Rate),
        }
    }
}

// --- Spec §20: Core Interface ---

/// Full result from kernel update() - spec §20
#[derive(Clone, Debug)]
pub struct UpdateResult {
    pub actuator: ActuatorCmd,
    pub status: ChannelStatus,
    pub estimate: StateEstimate,
    pub timing: CycleTiming,
    pub degradation: Option<DegradationEvent>,
}

/// Lightweight health snapshot - spec §20
#[derive(Clone, Debug)]
pub struct HealthReport {
    pub init_state: InitState,
    pub control_law: ControlLawV1,
    pub config_mode: ConfigMode,
    pub transition_state: ConfigTransitionState,
    pub faults: FaultFlags,
    pub channel_health: ChannelHealthV1,
}
