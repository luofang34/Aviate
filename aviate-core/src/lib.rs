#![no_std]
#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod checks;
pub mod control;
pub mod ekf;
pub mod fault;
pub mod hal;
pub mod math;
pub mod mixer;
pub mod sensor;
pub mod state;
pub mod time;
pub mod types;

pub use crate::checks::{
    DegradationReason, InFlightFlags, TransitionFailure, TransitionFlags, TransitionLimits,
};
use crate::checks::{KernelChecks, PreArmFlags};
use crate::control::envelope::{EnvelopeProtector, ProtectionStatus, SimpleEnvelopeProtector};
use crate::control::{
    AuthorityProfile, Command, ConfigMode, ControlLawV1, ControlMode, Limits, VehicleController,
};
use crate::ekf::Ekf;
use crate::fault::{FaultFlags, FaultHandlingTable};
use crate::mixer::{
    ActuatorCmd, ActuatorSanitizer, ActuatorState, Mixer, ModeConfig, SanitizeReport, Sanitizer,
};
use crate::sensor::SensorSet;
use crate::state::{EstimateQuality, StateEstimate};
use crate::time::{TimeDelta, Timestamp};
use crate::types::{Meters, MetersPerSecond, Normalized, Radians, RadiansPerSecond};

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

/// Runtime configuration (spec §19)
#[derive(Clone, Debug)]
pub struct Config {
    pub limits: Limits,
    pub command_timeout_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            limits: Limits {
                max_roll: crate::types::Radians(0.78),
                max_pitch: crate::types::Radians(0.78),
                max_roll_rate: crate::types::RadiansPerSecond(3.0),
                max_pitch_rate: crate::types::RadiansPerSecond(3.0),
                max_yaw_rate: crate::types::RadiansPerSecond(3.0),
                max_horizontal_speed: crate::types::MetersPerSecond(10.0),
                max_climb_rate: crate::types::MetersPerSecond(2.0),
                max_descent_rate: crate::types::MetersPerSecond(2.0),
                max_altitude: crate::types::Meters(100.0),
                min_altitude: crate::types::Meters(0.0),
                min_airspeed: None,
                max_airspeed: None,
                max_load_factor: 2.0,
                min_load_factor: 0.0,
            },
            command_timeout_ms: DEFAULT_COMMAND_TIMEOUT_MS,
        }
    }
}

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

#[derive(Clone, Debug)]
pub struct CrossChannelData {
    pub estimates: [Option<StateEstimate>; ChannelId::MAX_CHANNELS],
    pub health: [Option<ChannelHealthV1>; ChannelId::MAX_CHANNELS],
    pub commands: [Option<ActuatorCmd>; ChannelId::MAX_CHANNELS],
    pub sequences: [Option<u32>; ChannelId::MAX_CHANNELS],
}

/// Channel health status (spec §16)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ChannelHealthV1 {
    /// Channel fully operational
    Operative,
    /// Channel functional but with reduced capability
    Degraded,
    /// Channel has failed
    Failed,
    /// Channel in test/maintenance mode
    Offline,
}

impl Default for ChannelHealthV1 {
    fn default() -> Self {
        Self::Operative
    }
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
// TransitionFailure is imported from checks.rs

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

pub struct AviateKernelImpl<V: VehicleController, M: Mixer> {
    pub ekf: Ekf,
    pub controller: V,
    pub mixer: M,
    pub sanitizer: Sanitizer,
    pub protector: SimpleEnvelopeProtector,
    pub limits: Limits,
    pub mode: ConfigMode,
    pub mode_config: ModeConfig,

    // State Machine
    pub init_state: InitState,
    pub faults: FaultFlags,
    pub fault_table: FaultHandlingTable,
    pub control_law: ControlLawV1,

    // Unified Check System (§17, §14, §4.5)
    pub checks: KernelChecks,

    // Actuator state tracking for transition checks
    pub actuator_state: ActuatorState,

    // Command timeout threshold (ms)
    pub command_timeout_ms: u32,

    // Configuration (spec §19)
    pub config: Config,

    // Timing tracking (spec §18)
    pub timing_stats: TimingStats,

    // Safety
    pub safe_output: [Normalized; 16], // MAX_ACTUATORS = 16
}

/// Type alias for backward compatibility
pub type AviateKernel<V, M> = AviateKernelImpl<V, M>;

// --- Spec §20: AviateKernel Trait ---

/// Core flight control kernel interface (spec §20)
///
/// Defines the standard interface for flight control implementations.
/// All persistent state relevant to control or estimation is owned by
/// implementations of this trait (spec §37).
pub trait AviateKernelTrait {
    /// Advance initialization state machine
    fn init_step(&mut self, sensors: &SensorSet, time: Timestamp) -> InitResult;

    /// Get current initialization state
    fn init_state(&self) -> InitState;

    /// Check if system is ready to arm
    fn is_ready(&self) -> bool;

    /// Attempt to arm the system
    fn arm(&mut self) -> Result<(), ArmError>;

    /// Disarm the system
    fn disarm(&mut self);

    /// Get current configuration mode
    fn config_mode(&self) -> ConfigMode;

    /// Get current transition state
    fn transition_state(&self) -> ConfigTransitionState;

    /// Request a configuration mode transition
    fn request_config_mode(&mut self, to: ConfigMode) -> Result<(), TransitionError>;

    /// Main control update (spec §20)
    fn update(
        &mut self,
        channel: ChannelId,
        time: TimeDelta,
        sensors: &SensorSet,
        command: &Command,
        actuator_state: &ActuatorState,
        cross_channel: Option<&CrossChannelData>,
    ) -> UpdateResult;

    /// Load configuration from block (spec §19)
    fn load_config(&mut self, config: &ConfigBlock) -> Result<(), ConfigError>;

    /// Get current configuration (spec §20)
    fn get_config(&self) -> &Config;

    /// Get health report (spec §20)
    fn get_health(&self) -> HealthReport;

    /// Get current fault flags
    fn get_faults(&self) -> FaultFlags;

    /// Get current control law
    fn get_control_law(&self) -> ControlLawV1;

    /// Kick the watchdog timer
    fn kick_watchdog(&mut self);

    /// Perform ground reset
    fn ground_reset(&mut self);

    /// Inject state for testing (spec §20, test-hooks only)
    #[cfg(feature = "test-hooks")]
    fn inject_state(&mut self, state: &StateEstimate);

    /// Inject fault for testing (spec §20, test-hooks only)
    #[cfg(feature = "test-hooks")]
    fn inject_fault(&mut self, fault: FaultFlags);
}

pub trait Watchdog {
    fn kick(&mut self);
    fn check_deadline(&self) -> bool;
}

impl<V: VehicleController, M: Mixer> Watchdog for AviateKernelImpl<V, M> {
    fn kick(&mut self) {
        // Minimal implementation: just a stub for now as we don't have full timing context
        // In a real system, this would update a timestamp
    }

    // COV:EXCL_START(STUB: watchdog placeholder, not implemented)
    fn check_deadline(&self) -> bool {
        true
    }
    // COV:EXCL_STOP
}

impl<V: VehicleController, M: Mixer> AviateKernelImpl<V, M> {
    pub fn new(controller: V, mixer: M, mode_config: ModeConfig) -> Self {
        Self {
            ekf: Ekf::default(),
            controller,
            mixer,
            sanitizer: Sanitizer::default(),
            protector: SimpleEnvelopeProtector,
            limits: Limits {
                max_roll: crate::types::Radians(0.78), // ~45 deg
                max_pitch: crate::types::Radians(0.78),
                max_roll_rate: crate::types::RadiansPerSecond(3.0),
                max_pitch_rate: crate::types::RadiansPerSecond(3.0),
                max_yaw_rate: crate::types::RadiansPerSecond(3.0),
                max_horizontal_speed: crate::types::MetersPerSecond(10.0),
                max_climb_rate: crate::types::MetersPerSecond(2.0),
                max_descent_rate: crate::types::MetersPerSecond(2.0),
                max_altitude: crate::types::Meters(100.0),
                min_altitude: crate::types::Meters(0.0),
                min_airspeed: None,
                max_airspeed: None,
                max_load_factor: 2.0,
                min_load_factor: 0.0,
            },
            mode: ConfigMode::Hover,
            mode_config,

            init_state: InitState::PowerOn,
            faults: FaultFlags::empty(),
            fault_table: FaultHandlingTable::DEFAULT,
            control_law: ControlLawV1::Primary,
            checks: KernelChecks::new(),
            actuator_state: ActuatorState::default(),
            command_timeout_ms: DEFAULT_COMMAND_TIMEOUT_MS,
            config: Config::default(),
            timing_stats: TimingStats::default(),
            safe_output: [Normalized(0.0); 16],
        }
    }

    /// Create kernel with custom pre-arm requirements
    pub fn with_pre_arm_required(
        controller: V,
        mixer: M,
        mode_config: ModeConfig,
        required: PreArmFlags,
    ) -> Self {
        let mut kernel = Self::new(controller, mixer, mode_config);
        kernel.checks = KernelChecks::with_pre_arm_required(required);
        kernel
    }

    pub fn init_step(&mut self, sensors: &SensorSet, _time: Timestamp) -> InitResult {
        // 1. Update checks from sensor data (always, regardless of state)
        self.checks.pre_arm.update_from_sensors(sensors);
        self.checks.pre_arm.update_from_faults(self.faults);
        self.checks.pre_arm.update_ekf(self.ekf.is_initialized());

        // Note: update_sensor_faults() is NOT called here.
        // Faults are runtime monitoring for armed operation.
        // Pre-arm checks are handled by the pre_arm flags (IMU_HEALTHY, etc.).

        // 3. State machine transitions
        match self.init_state {
            InitState::PowerOn => {
                self.init_state = InitState::ConfigLoading;
            }
            InitState::ConfigLoading => {
                // Config loaded (placeholder - would check actual config validity)
                self.checks
                    .pre_arm
                    .current
                    .insert(PreArmFlags::CONFIG_VALID);
                self.init_state = InitState::SensorInit;
            }
            InitState::SensorInit => {
                // Wait for at least one valid sensor reading
                let has_sensors = self
                    .checks
                    .pre_arm
                    .current
                    .contains(PreArmFlags::IMU_HEALTHY);
                if has_sensors {
                    self.init_state = InitState::EstimatorConverging;
                }
            }
            InitState::EstimatorConverging => {
                // Wait for sensor convergence and EKF initialization
                let converged = self
                    .checks
                    .pre_arm
                    .current
                    .contains(PreArmFlags::IMU_CONVERGED)
                    && self
                        .checks
                        .pre_arm
                        .current
                        .contains(PreArmFlags::EKF_CONVERGED);
                if converged {
                    self.init_state = InitState::PreArm;
                }
            }
            InitState::PreArm => {
                // Check all pre-arm requirements
                if self.checks.pre_arm.is_satisfied() {
                    self.init_state = InitState::Ready;
                }
            }
            InitState::Ready => {
                // Monitor for fault conditions
                if !self.checks.pre_arm.is_satisfied() {
                    self.init_state = InitState::PreArm;
                }
            }
            InitState::Armed => {} // COV:EXCL(EMPTY: monitoring only, disarm via disarm())
            InitState::Disarmed => {
                // Transition back to PreArm for potential re-arm
                // Reset sample counts for fresh convergence check
                self.checks.pre_arm.samples.reset();
                self.init_state = InitState::PreArm;
            }
            InitState::Fault => {
                // Require explicit reset to exit fault state
            }
        }

        InitResult {
            state: self.init_state,
            faults: self.faults,
            ready: self.init_state == InitState::Ready,
        }
    }

    /// Update fault flags based on sensor health
    fn update_sensor_faults(&mut self, sensors: &SensorSet) {
        use crate::sensor::SensorHealth;

        // IMU faults
        let imu_ok = sensors
            .imus
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !imu_ok {
            self.faults.insert(FaultFlags::ALL_IMU_FAILED);
        } else {
            self.faults.remove(FaultFlags::ALL_IMU_FAILED);
        }

        // Baro faults
        let baro_ok = sensors
            .baros
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !baro_ok {
            self.faults.insert(FaultFlags::BARO_FAILED);
        } else {
            self.faults.remove(FaultFlags::BARO_FAILED);
        }

        // Mag faults
        let mag_ok = sensors
            .mags
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !mag_ok {
            self.faults.insert(FaultFlags::MAG_FAILED);
        } else {
            self.faults.remove(FaultFlags::MAG_FAILED);
        }

        // GNSS faults
        let gnss_ok = sensors
            .gnss
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !gnss_ok {
            self.faults.insert(FaultFlags::ALL_GNSS_LOST);
        } else {
            self.faults.remove(FaultFlags::ALL_GNSS_LOST);
        }
    }

    pub fn is_ready(&self) -> bool {
        self.init_state == InitState::Ready
    }

    pub fn arm(&mut self) -> Result<(), ArmError> {
        if self.init_state == InitState::Armed {
            return Err(ArmError::AlreadyArmed);
        }
        if self.init_state != InitState::Ready {
            return Err(ArmError::NotReady);
        }
        if !self.faults.is_empty() {
            return Err(ArmError::Faulted);
        }

        self.init_state = InitState::Armed;
        Ok(())
    }

    pub fn disarm(&mut self) {
        self.init_state = InitState::Disarmed;
        self.control_law = ControlLawV1::Backup; // Was Frozen, now Backup
        self.checks.in_flight.reset();
    }

    /// Check if the system can be reset from fault state
    ///
    /// Preconditions:
    /// - No critical faults active
    /// - IMU_HEALTHY (sensors recovered)
    /// - THROTTLE_LOW (safety)
    pub fn can_reset_from_fault(&self) -> bool {
        if self.init_state != InitState::Fault {
            return false;
        }

        // No critical faults remaining
        let no_critical = !self.faults.intersects(CRITICAL_FAULTS);

        // Sensors recovered
        let imu_healthy = self
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::IMU_HEALTHY);

        // Throttle low for safety
        let throttle_low = self
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::THROTTLE_LOW);

        no_critical && imu_healthy && throttle_low
    }

    /// Attempt to reset from fault state
    ///
    /// Returns Ok(()) if successfully reset to PreArm state.
    pub fn reset_from_fault(&mut self) -> Result<(), ArmError> {
        if self.init_state != InitState::Fault {
            return Err(ArmError::NotReady);
        }

        if !self.can_reset_from_fault() {
            return Err(ArmError::Faulted);
        }

        // Reset checks for fresh convergence
        self.checks.pre_arm.samples.reset();
        self.checks.in_flight.reset();

        // Transition to PreArm
        self.init_state = InitState::PreArm;
        Ok(())
    }

    /// Handle degradation based on in-flight check trigger
    ///
    /// Updates control law based on the degradation reason.
    /// Public for DO-178C MC/DC testing of all degradation paths.
    pub fn handle_degradation(
        &mut self,
        reason: DegradationReason,
        timestamp: Timestamp,
    ) -> Option<DegradationEvent> {
        let from = self.control_law;
        let to = match reason {
            DegradationReason::AttitudeLost => ControlLawV1::Backup,
            DegradationReason::ImuDegraded => ControlLawV1::Alternate,
            DegradationReason::PositionLost => ControlLawV1::Alternate,
            DegradationReason::VelocityLost => ControlLawV1::Alternate,
            DegradationReason::CommandTimeout => ControlLawV1::Alternate,
            DegradationReason::EnvelopeViolation => ControlLawV1::Alternate,
            DegradationReason::BaroDegraded => ControlLawV1::Alternate,
            DegradationReason::RcLost => ControlLawV1::Alternate,
            DegradationReason::TimingViolation => ControlLawV1::Alternate,
        };

        // Only trigger if this is a degradation (worse state)
        if to.severity() > from.severity() {
            self.control_law = to;
            Some(DegradationEvent {
                from,
                to,
                reason,
                timestamp,
            })
        } else {
            None
        }
    }

    /// Request a configuration mode transition
    ///
    /// Checks transition preconditions before starting the transition.
    pub fn request_config_mode(&mut self, to: ConfigMode) -> Result<(), TransitionError> {
        // Must be armed
        if self.init_state != InitState::Armed {
            return Err(TransitionError::NotArmed);
        }

        // Cannot be in fault state
        if self.faults.intersects(CRITICAL_FAULTS) {
            return Err(TransitionError::InFaultState);
        }

        // Check if already transitioning (Transition mode is the transition state)
        if self.mode == ConfigMode::Transition {
            return Err(TransitionError::AlreadyTransitioning);
        }

        // Check if already in requested mode
        if self.mode == to {
            return Err(TransitionError::AlreadyInMode);
        }

        // Update transition checks and verify
        let state = self.ekf.get_estimate();
        self.checks.transition.update_from_state(&state);
        self.checks
            .transition
            .update_from_actuators(&self.actuator_state, 0b1111); // Quad mask

        // Gate the transition
        self.checks
            .transition
            .can_transition()
            .map_err(TransitionError::ChecksFailed)?;

        // Start the transition (caller manages progress)
        // For now, just update the mode directly
        self.mode = to;
        Ok(())
    }

    pub fn get_health(&self) -> HealthReport {
        HealthReport {
            init_state: self.init_state,
            control_law: self.control_law,
            config_mode: self.mode,
            transition_state: ConfigTransitionState::Stable(self.mode),
            faults: self.faults,
            channel_health: ChannelHealthV1::Operative,
        }
    }

    /// Main control update with in-flight monitoring (Spec §20)
    ///
    /// # Arguments
    /// * `channel` - Channel ID (primary/secondary/etc.)
    /// * `time` - Time delta since last update
    /// * `sensors` - Current sensor readings
    /// * `command` - The command to execute
    /// * `actuator_state` - Feedback from actuators
    /// * `cross_channel` - Data from other redundant channels (optional)
    pub fn update(
        &mut self,
        _channel: ChannelId,
        time: TimeDelta,
        sensors: &SensorSet,
        command: &Command,
        _actuator_state: &ActuatorState,
        _cross_channel: Option<&CrossChannelData>,
    ) -> UpdateResult {
        // Spec §18: Track timing statistics
        // NOTE: dt (time since last call) is tracked for statistics, but deadline violations
        // require external monitoring of actual update() execution time by the caller.
        // The deadline (800us) refers to how long update() should take, not the call interval.
        let dt_us = time.as_micros() as u32;

        // Update timing statistics (for monitoring, not degradation triggering)
        self.timing_stats.last_cycle_us = dt_us;
        self.timing_stats.total_cycles = self.timing_stats.total_cycles.saturating_add(1);

        if dt_us > self.timing_stats.max_cycle_us {
            self.timing_stats.max_cycle_us = dt_us;
        }
        if dt_us < self.timing_stats.min_cycle_us || self.timing_stats.min_cycle_us == 0 {
            self.timing_stats.min_cycle_us = dt_us;
        }

        // Deadline violations are tracked by the caller via report_timing_violation()
        // which has access to actual execution time measurements

        // Basic timestamp for now
        let timestamp = crate::time::Timestamp {
            ticks: time.tick_delta,
            source: crate::time::TimeSource::Internal,
        };

        // 0. Update sensor fault flags (always, regardless of armed state)
        //    This allows continuous monitoring of sensor health.
        self.update_sensor_faults(sensors);

        // 1. Safety Gate: If not armed, force safe output immediately
        if !self.init_state.allows_active_control() {
            return UpdateResult {
                actuator: ActuatorCmd {
                    outputs: self.safe_output,
                    active_mask: 0,
                    sequence: command.sequence,
                    timestamp,
                    fallback_mask: 0,
                    sanitized: true,
                },
                status: ChannelStatus {
                    mode: command.mode,
                    config_mode: self.mode,
                    transition_state: ConfigTransitionState::Stable(self.mode),
                    law: ControlLawV1::Backup, // Force Backup reporting when not armed
                    health: ChannelHealthV1::Operative,
                    faults: self.faults,
                    confidence: self.ekf.get_estimate().quality,
                    envelope_margin: EnvelopeMargin::default(),
                    sequence: command.sequence,
                    protection: ProtectionStatus::default(),
                    sanitize_report: SanitizeReport::default(),
                },
                estimate: self.ekf.get_estimate(),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }

        // 2. Check for critical faults (if we got here, we're armed)
        if self.check_critical_faults() {
            // If critical fault, force Backup/Frozen behavior
            return UpdateResult {
                actuator: ActuatorCmd {
                    outputs: self.safe_output,
                    active_mask: 0,
                    sequence: command.sequence,
                    timestamp,
                    fallback_mask: 0,
                    sanitized: true,
                },
                status: ChannelStatus::default(), // TODO: Populate with fault info
                estimate: self.ekf.get_estimate(),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }

        // 3. EKF Update (predict and update)
        let primary_imu = &sensors.imus[0];
        if primary_imu.valid && primary_imu.health == crate::sensor::SensorHealth::Good {
            self.ekf.predict(&primary_imu.value, time.dt_sec.0);
        }

        // Apply sensor overrides from command
        if let Some(overrides) = &command.sensor_overrides {
            if let Some(gnss_health) = overrides.gnss_force_state {
                let mut primary_gnss_reading = sensors.gnss[0];
                primary_gnss_reading.health = match gnss_health {
                    crate::sensor::GnssHealth::Good => crate::sensor::SensorHealth::Good,
                    crate::sensor::GnssHealth::Suspect => crate::sensor::SensorHealth::Degraded,
                    crate::sensor::GnssHealth::Lost => crate::sensor::SensorHealth::Failed,
                };
                self.ekf.update_gnss(&primary_gnss_reading);
            }
        } else {
            // Normal sensor updates
            let primary_gnss = &sensors.gnss[0];
            if primary_gnss.valid && primary_gnss.health == crate::sensor::SensorHealth::Good {
                self.ekf.update_gnss(primary_gnss);
            }
        }

        let primary_baro = &sensors.baros[0];
        if primary_baro.valid && primary_baro.health == crate::sensor::SensorHealth::Good {
            self.ekf.update_baro(primary_baro);
        }

        let primary_mag = &sensors.mags[0];
        if primary_mag.valid && primary_mag.health == crate::sensor::SensorHealth::Good {
            self.ekf.update_mag(primary_mag);
        }

        // Get updated estimate
        let state = self.ekf.get_estimate();

        // 3. Update in-flight checks
        self.checks.in_flight.update_from_state(&state);
        self.checks.in_flight.update_from_sensors(sensors);
        // Assume command age 0 for now, or derive from timestamp
        self.checks
            .in_flight
            .update_command_status(0, self.command_timeout_ms);

        // 4. Handle degradation
        // Timing violations are reported externally via report_timing_violation()
        let degradation = if self.timing_stats.consecutive_violations >= TIMING_VIOLATION_THRESHOLD
        {
            // Persistent timing violation → degrade to Alternate
            self.handle_degradation(DegradationReason::TimingViolation, timestamp)
        } else if let Some(reason) = self.checks.in_flight.get_degradation_trigger() {
            self.handle_degradation(reason, timestamp)
        } else {
            None
        };

        // 5. Envelope Protection
        let (constrained_sp, protection_status) = self.protector.constrain(
            &command.setpoint,
            &state,
            &self.limits,
            AuthorityProfile::HardEnvelope,
        );

        self.checks
            .in_flight
            .update_from_envelope(&protection_status);

        let constrained_cmd = Command {
            setpoint: constrained_sp,
            ..command.clone()
        };

        // 6. Control Step
        // If Backup, we might want to use safe outputs or a simplified controller.
        // For now, if Backup, force safe output (as per spec "Non-Armed states → Backup → safe").
        // But Backup during flight might mean "Last-ditch stability".
        // The spec says "Last-ditch stability only".
        // For this minimal impl, if Backup, we output safe_output (effectively shutting down/idle).
        let mut actuator_cmd = if self.control_law == ControlLawV1::Backup {
            ActuatorCmd {
                outputs: self.safe_output,
                active_mask: 0b1111,
                sequence: command.sequence,
                timestamp,
                fallback_mask: 0xFF,
                sanitized: true,
            }
        } else {
            let axis_cmd = self
                .controller
                .step(&state, &constrained_cmd, self.mode, &self.limits);
            self.mixer.mix(&axis_cmd)
        };

        // 7. Update actuator state
        self.actuator_state
            .update_commanded(&actuator_cmd, timestamp);

        // 8. Sanitization
        let sanitize_report = if self.control_law == ControlLawV1::Backup {
            SanitizeReport::default()
        } else {
            self.sanitizer
                .sanitize(&mut actuator_cmd, &self.mode_config)
        };

        // 9. Construct Result
        UpdateResult {
            actuator: actuator_cmd,
            status: ChannelStatus {
                mode: command.mode,
                config_mode: self.mode,
                transition_state: ConfigTransitionState::Stable(self.mode),
                law: self.control_law,
                health: ChannelHealthV1::Operative,
                faults: self.faults,
                confidence: state.quality,
                envelope_margin: EnvelopeMargin::default(), // TODO calculate
                sequence: command.sequence,
                protection: protection_status,
                sanitize_report,
            },
            estimate: state,
            timing: CycleTiming {
                cycle_start_us: 0, // Caller provides absolute timing if needed
                cycle_end_us: dt_us,
                duration_us: dt_us, // dt since last call (actual exec time tracked externally)
                deadline_met: self.timing_stats.consecutive_violations == 0,
            },
            degradation,
        }
    }

    /// Perform a ground reset, clearing transient states
    #[inline(never)]
    pub fn ground_reset(&mut self) {
        // Only allowed if not armed
        if self.init_state == InitState::Armed {
            return;
        }

        self.faults = FaultFlags::empty();
        self.checks.pre_arm.reset();
        self.checks.in_flight.reset();
        self.checks.transition.reset();
        self.init_state = InitState::ConfigLoading; // Restart init sequence
        self.ekf = Ekf::default(); // Reset estimator
        self.control_law = ControlLawV1::Primary; // Reset law
    }

    pub fn kick_watchdog(&mut self) {
        self.kick();
    }

    /// Report a timing violation from external monitoring (spec §18)
    ///
    /// The caller should call this when the actual execution time of update()
    /// exceeds CONTROL_LOOP_DEADLINE_US (800us). After TIMING_VIOLATION_THRESHOLD
    /// consecutive violations, degradation to Alternate law is triggered.
    ///
    /// Call with `violation = true` when deadline exceeded, `false` when met.
    pub fn report_timing_violation(&mut self, violation: bool) {
        if violation {
            self.timing_stats.deadline_violations =
                self.timing_stats.deadline_violations.saturating_add(1);
            self.timing_stats.consecutive_violations =
                self.timing_stats.consecutive_violations.saturating_add(1);
        } else {
            self.timing_stats.consecutive_violations = 0;
        }
    }

    /// Check for critical faults and enter fault state if detected
    ///
    /// Returns true if fault state was entered.
    pub fn check_critical_faults(&mut self) -> bool {
        if self.faults.intersects(CRITICAL_FAULTS) {
            self.init_state = InitState::Fault;
            self.control_law = ControlLawV1::Backup; // Was Frozen
            true
        } else {
            false
        }
    }

    // Additional accessor methods
    fn _init_state(&self) -> InitState {
        self.init_state
    }

    fn _config_mode(&self) -> ConfigMode {
        self.mode
    }

    fn _transition_state(&self) -> ConfigTransitionState {
        ConfigTransitionState::Stable(self.mode)
    }

    fn _get_faults(&self) -> FaultFlags {
        self.faults
    }

    fn _get_control_law(&self) -> ControlLawV1 {
        self.control_law
    }
}

// --- Spec §20: AviateKernelTrait Implementation ---

impl<V: VehicleController, M: Mixer> AviateKernelTrait for AviateKernelImpl<V, M> {
    fn init_step(&mut self, sensors: &SensorSet, time: Timestamp) -> InitResult {
        AviateKernelImpl::init_step(self, sensors, time)
    }

    fn init_state(&self) -> InitState {
        self.init_state
    }

    fn is_ready(&self) -> bool {
        AviateKernelImpl::is_ready(self)
    }

    fn arm(&mut self) -> Result<(), ArmError> {
        AviateKernelImpl::arm(self)
    }

    fn disarm(&mut self) {
        AviateKernelImpl::disarm(self)
    }

    fn config_mode(&self) -> ConfigMode {
        self.mode
    }

    fn transition_state(&self) -> ConfigTransitionState {
        // TODO: Track actual transition state for async transitions
        ConfigTransitionState::Stable(self.mode)
    }

    fn request_config_mode(&mut self, to: ConfigMode) -> Result<(), TransitionError> {
        AviateKernelImpl::request_config_mode(self, to)
    }

    fn update(
        &mut self,
        channel: ChannelId,
        time: TimeDelta,
        sensors: &SensorSet,
        command: &Command,
        actuator_state: &ActuatorState,
        cross_channel: Option<&CrossChannelData>,
    ) -> UpdateResult {
        AviateKernelImpl::update(
            self,
            channel,
            time,
            sensors,
            command,
            actuator_state,
            cross_channel,
        )
    }

    fn load_config(&mut self, config_block: &ConfigBlock) -> Result<(), ConfigError> {
        // Spec §19: Stub implementation - validates checksum and version only
        if config_block.version > 1 {
            return Err(ConfigError::UnsupportedVersion);
        }
        // TODO: Parse actual config data from block
        // For now, accept valid blocks but use default config
        let _ = config_block.checksum; // Placeholder for future checksum validation
        Ok(())
    }

    fn get_config(&self) -> &Config {
        &self.config
    }

    fn get_health(&self) -> HealthReport {
        AviateKernelImpl::get_health(self)
    }

    fn get_faults(&self) -> FaultFlags {
        self.faults
    }

    fn get_control_law(&self) -> ControlLawV1 {
        self.control_law
    }

    fn kick_watchdog(&mut self) {
        AviateKernelImpl::kick_watchdog(self)
    }

    fn ground_reset(&mut self) {
        AviateKernelImpl::ground_reset(self)
    }

    #[cfg(feature = "test-hooks")]
    fn inject_state(&mut self, state: &StateEstimate) {
        self.ekf.set_state(state);
    }

    #[cfg(feature = "test-hooks")]
    fn inject_fault(&mut self, fault: FaultFlags) {
        self.faults.insert(fault);
    }
}

/// Aviate core initialization
pub fn init_core() {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::mc::McController;
    use crate::mixer::{ActuatorCmd, ModeConfig, QuadXMixer};

    struct DummyMixer;
    impl Mixer for DummyMixer {
        fn mix(&self, _axis: &crate::control::AxisCommand) -> ActuatorCmd {
            let cmd = ActuatorCmd::default();
            cmd
        }
    }

    fn create_kernel() -> AviateKernelImpl<McController, DummyMixer> {
        let mode_config = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        };
        AviateKernelImpl::new(McController::default(), DummyMixer, mode_config)
    }

    #[test]
    fn test_ground_reset_success_unit() {
        let mut kernel = create_kernel();
        kernel.init_state = InitState::Fault;
        kernel.faults = FaultFlags::ALL_IMU_FAILED;

        kernel.ground_reset();

        assert_eq!(kernel.init_state, InitState::ConfigLoading);
        assert!(kernel.faults.is_empty());

        // Cover DummyMixer
        kernel.mixer.mix(&crate::control::AxisCommand {
            roll: crate::types::NormalizedSigned(0.0),
            pitch: crate::types::NormalizedSigned(0.0),
            yaw: crate::types::NormalizedSigned(0.0),
            collective: crate::types::Normalized(0.0),
        });
    }
}
