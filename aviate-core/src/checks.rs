//! Unified Check System
//!
//! Provides a consistent pattern for all safety checks in aviate-core.
//! Each check flag is traceable to spec requirements for formal validation.
//!
//! ## Check Categories
//!
//! - **PreArmFlags**: Conditions required before arming (§17 InitState)
//! - **InFlightFlags**: Continuous monitoring during flight (§14, §15)
//! - **TransitionFlags**: Safety checks for config mode changes (§4.5)
//!
//! ## Design Philosophy
//!
//! - Proactive checks (pre-conditions) vs Reactive faults (FaultFlags)
//! - Each bit traceable to spec section
//! - Configurable `required` flags per vehicle type
//! - `missing()` reports exactly what failed for diagnostics

use crate::control::envelope::{EnvelopeMargin, ProtectionStatus};
use crate::fault::FaultFlags;
use crate::mixer::{ActuatorHealth, ActuatorState};
use crate::sensor::{SensorHealth, SensorSet};
use crate::state::{StateEstimate, StateValidFlags};
use crate::types::MetersPerSecond;

// ============================================================================
// PRE-ARM CHECKS (§17 InitState transitions)
// ============================================================================

bitflags::bitflags! {
    /// Pre-arm checks required before InitState::PreArm → Ready → Armed
    ///
    /// Each bit is traceable to a spec requirement.
    /// Vehicle configs specify which flags are required via `CheckStatus.required`.
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
    pub struct PreArmFlags: u32 {
        // === Sensor Health (from SensorReading.health) ===
        // Ref: §15.1 FaultCategory::ImuFailed, BaroFailed, MagFailed, GnssLost

        /// At least one IMU reporting Good health
        const IMU_HEALTHY       = 1 << 0;
        /// At least one barometer reporting Good health
        const BARO_HEALTHY      = 1 << 1;
        /// At least one magnetometer reporting Good health
        const MAG_HEALTHY       = 1 << 2;
        /// GNSS available with valid fix (optional for some vehicles)
        const GNSS_AVAILABLE    = 1 << 3;

        // === Sensor Convergence (sample counts for EKF) ===
        // Ref: §17 InitState::EstimatorConverging

        /// Sufficient IMU samples received for EKF initialization
        const IMU_CONVERGED     = 1 << 4;
        /// Sufficient baro samples received
        const BARO_CONVERGED    = 1 << 5;
        /// Sufficient mag samples received
        const MAG_CONVERGED     = 1 << 6;
        /// EKF state estimate has converged
        const EKF_CONVERGED     = 1 << 7;

        // === Safety Conditions ===

        /// Throttle/collective at low position (stick safety)
        const THROTTLE_LOW      = 1 << 8;
        /// Configuration loaded and valid
        /// Ref: §15.1 FaultCategory::ConfigInvalid
        const CONFIG_VALID      = 1 << 9;
        /// No active faults (FaultFlags::is_empty())
        const NO_FAULTS         = 1 << 10;
        /// Physical hardware safety switch armed (if equipped)
        const HARDWARE_ARMED    = 1 << 11;
        /// Gyro bias calibration complete
        const GYRO_CALIBRATED   = 1 << 12;
        /// Accelerometer calibration valid
        const ACCEL_CALIBRATED  = 1 << 13;

        // === Composite Requirements ===

        /// Minimum checks for basic quadrotor (no GPS required)
        const QUAD_MINIMUM = Self::IMU_HEALTHY.bits()
                           | Self::BARO_HEALTHY.bits()
                           | Self::IMU_CONVERGED.bits()
                           | Self::BARO_CONVERGED.bits()
                           | Self::EKF_CONVERGED.bits()
                           | Self::THROTTLE_LOW.bits()
                           | Self::CONFIG_VALID.bits()
                           | Self::NO_FAULTS.bits();

        /// Full checks for GPS-enabled vehicle
        const QUAD_WITH_GPS = Self::QUAD_MINIMUM.bits()
                            | Self::MAG_HEALTHY.bits()
                            | Self::MAG_CONVERGED.bits()
                            | Self::GNSS_AVAILABLE.bits();
    }
}

// ============================================================================
// IN-FLIGHT CHECKS (continuous monitoring)
// ============================================================================

/// Reason for triggering degraded mode or failsafe
///
/// Maps from InFlightFlags violations to specific degradation responses.
/// Used by the kernel to decide control law changes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DegradationReason {
    /// Attitude estimate lost - most critical
    AttitudeLost,
    /// Position estimate lost - drop to attitude mode
    PositionLost,
    /// Velocity estimate lost - affects position hold
    VelocityLost,
    /// No commands received within timeout
    CommandTimeout,
    /// IMU sensor degraded or failed
    ImuDegraded,
    /// Barometer failed - affects altitude hold
    BaroDegraded,
    /// Outside safe envelope limits
    EnvelopeViolation,
    /// RC link lost
    RcLost,
}

bitflags::bitflags! {
    /// In-flight safety checks for continuous monitoring
    ///
    /// These flags are updated every control cycle and used for:
    /// - Degraded mode decisions
    /// - Failsafe triggering
    /// - Telemetry reporting
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
    pub struct InFlightFlags: u32 {
        // === State Validity (§14 StateValidFlags) ===

        /// Attitude estimate valid and confident
        const ATTITUDE_VALID    = 1 << 0;
        /// Velocity estimate valid (may require GPS)
        const VELOCITY_VALID    = 1 << 1;
        /// Position estimate valid (requires GPS or other source)
        const POSITION_VALID    = 1 << 2;
        /// Heading reference valid
        const HEADING_VALID     = 1 << 3;

        // === Envelope (§13 EnvelopeMargin) ===

        /// Within all envelope limits (attitude, altitude, speed)
        const WITHIN_ENVELOPE   = 1 << 4;
        /// Altitude within geofence
        const ALTITUDE_OK       = 1 << 5;

        // === Communications ===

        /// Recent valid command received (no timeout)
        /// Ref: §15.1 FaultCategory::CommandTimeout
        const COMMAND_RECENT    = 1 << 8;
        /// RC link available (if equipped)
        const RC_AVAILABLE      = 1 << 9;
        /// Telemetry link healthy
        const TELEMETRY_OK      = 1 << 10;

        // === Sensors (in-flight health) ===

        /// IMU still healthy in flight
        const IMU_OK            = 1 << 12;
        /// Baro still healthy in flight
        const BARO_OK           = 1 << 13;

        // === Composite ===

        /// Minimum for safe attitude-mode flight
        const ATTITUDE_FLIGHT = Self::ATTITUDE_VALID.bits()
                              | Self::IMU_OK.bits()
                              | Self::COMMAND_RECENT.bits();

        /// Required for position hold
        const POSITION_FLIGHT = Self::ATTITUDE_FLIGHT.bits()
                              | Self::POSITION_VALID.bits()
                              | Self::VELOCITY_VALID.bits();
    }
}

// ============================================================================
// TRANSITION CHECKS (§4.5 Config Mode transitions)
// ============================================================================

bitflags::bitflags! {
    /// Transition safety checks for ConfigMode changes
    ///
    /// Ref: §4.5 Transition Safety Rules
    /// RULE 1: Switching allowed only if actuators and envelope permit.
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
    pub struct TransitionFlags: u32 {
        /// Aircraft in stable, non-aggressive flight
        const STABLE_FLIGHT     = 1 << 0;
        /// All actuators responsive (no stuck)
        /// Ref: §4.4 TransitionFailure::ActuatorStuck
        const ACTUATORS_OK      = 1 << 1;
        /// Within safe envelope for transition
        /// Ref: §4.4 TransitionFailure::UnsafeConditions
        const WITHIN_ENVELOPE   = 1 << 2;
        /// Actuator symmetry OK
        /// Ref: §4.4 TransitionFailure::Asymmetry
        const SYMMETRIC         = 1 << 3;
        /// Sufficient altitude for transition
        const ALTITUDE_OK       = 1 << 4;
        /// Sufficient airspeed (for VTOL transitions)
        const AIRSPEED_OK       = 1 << 5;

        // === Composite ===

        /// Minimum for hover ↔ forward transition
        const VTOL_TRANSITION = Self::STABLE_FLIGHT.bits()
                              | Self::ACTUATORS_OK.bits()
                              | Self::WITHIN_ENVELOPE.bits()
                              | Self::SYMMETRIC.bits();
    }
}

// ============================================================================
// SAMPLE COUNTS (for convergence tracking)
// ============================================================================

/// Sample counts for sensor convergence checks
#[derive(Copy, Clone, Debug, Default)]
pub struct SampleCounts {
    /// Valid IMU samples received
    pub imu: u32,
    /// Valid baro samples received
    pub baro: u32,
    /// Valid mag samples received
    pub mag: u32,
    /// Valid GNSS samples received
    pub gnss: u32,
    /// Minimum samples required for convergence (default: 100 @ 1kHz = 100ms)
    pub min_required: u32,
}

impl SampleCounts {
    /// Default minimum samples for EKF convergence (~100ms at 1kHz)
    pub const DEFAULT_MIN_SAMPLES: u32 = 100;

    pub fn new() -> Self {
        Self {
            min_required: Self::DEFAULT_MIN_SAMPLES,
            ..Default::default()
        }
    }

    /// Reset all counts (e.g., after disarm or fault)
    pub fn reset(&mut self) {
        self.imu = 0;
        self.baro = 0;
        self.mag = 0;
        self.gnss = 0;
    }

    /// Check if IMU has converged
    pub fn imu_converged(&self) -> bool {
        self.imu >= self.min_required
    }

    /// Check if baro has converged
    pub fn baro_converged(&self) -> bool {
        self.baro >= self.min_required
    }

    /// Check if mag has converged
    pub fn mag_converged(&self) -> bool {
        self.mag >= self.min_required
    }
}

// ============================================================================
// CHECK STATUS (generic tracker)
// ============================================================================

/// Check status for PreArmFlags
#[derive(Copy, Clone, Debug)]
pub struct PreArmStatus {
    /// Checks required to pass (configurable per vehicle)
    pub required: PreArmFlags,
    /// Checks currently passing
    pub current: PreArmFlags,
    /// Sample counts for convergence
    pub samples: SampleCounts,
}

impl Default for PreArmStatus {
    fn default() -> Self {
        Self {
            required: PreArmFlags::QUAD_MINIMUM,
            current: PreArmFlags::empty(),
            samples: SampleCounts::new(),
        }
    }
}

impl PreArmStatus {
    /// Create with custom required flags
    pub fn with_required(required: PreArmFlags) -> Self {
        Self {
            required,
            current: PreArmFlags::empty(),
            samples: SampleCounts::new(),
        }
    }

    /// Check if all required checks pass
    pub fn is_satisfied(&self) -> bool {
        self.current.contains(self.required)
    }

    /// Get flags that are required but not passing
    pub fn missing(&self) -> PreArmFlags {
        self.required - self.current
    }

    /// Update checks from sensor data
    pub fn update_from_sensors(&mut self, sensors: &SensorSet) {
        // Update sensor health flags
        let imu_healthy = sensors
            .imus
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        let baro_healthy = sensors
            .baros
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        let mag_healthy = sensors
            .mags
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        let gnss_available = sensors
            .gnss
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);

        self.current.set(PreArmFlags::IMU_HEALTHY, imu_healthy);
        self.current.set(PreArmFlags::BARO_HEALTHY, baro_healthy);
        self.current.set(PreArmFlags::MAG_HEALTHY, mag_healthy);
        self.current
            .set(PreArmFlags::GNSS_AVAILABLE, gnss_available);

        // Update sample counts
        if imu_healthy {
            self.samples.imu = self.samples.imu.saturating_add(1);
        }
        if baro_healthy {
            self.samples.baro = self.samples.baro.saturating_add(1);
        }
        if mag_healthy {
            self.samples.mag = self.samples.mag.saturating_add(1);
        }
        if gnss_available {
            self.samples.gnss = self.samples.gnss.saturating_add(1);
        }

        // Update convergence flags
        self.current
            .set(PreArmFlags::IMU_CONVERGED, self.samples.imu_converged());
        self.current
            .set(PreArmFlags::BARO_CONVERGED, self.samples.baro_converged());
        self.current
            .set(PreArmFlags::MAG_CONVERGED, self.samples.mag_converged());
    }

    /// Update from fault flags
    pub fn update_from_faults(&mut self, faults: FaultFlags) {
        self.current.set(PreArmFlags::NO_FAULTS, faults.is_empty());
        self.current.set(
            PreArmFlags::CONFIG_VALID,
            !faults.contains(FaultFlags::CONFIG_INVALID),
        );
    }

    /// Update throttle check
    pub fn update_throttle(&mut self, throttle_low: bool) {
        self.current.set(PreArmFlags::THROTTLE_LOW, throttle_low);
    }

    /// Update EKF convergence
    pub fn update_ekf(&mut self, converged: bool) {
        self.current.set(PreArmFlags::EKF_CONVERGED, converged);
    }

    /// Reset for re-arming attempt
    pub fn reset(&mut self) {
        self.current = PreArmFlags::empty();
        self.samples.reset();
    }
}

/// Check status for InFlightFlags
#[derive(Copy, Clone, Debug)]
pub struct InFlightStatus {
    /// Checks required for current flight mode
    pub required: InFlightFlags,
    /// Checks currently passing
    pub current: InFlightFlags,
}

impl Default for InFlightStatus {
    fn default() -> Self {
        Self {
            required: InFlightFlags::ATTITUDE_FLIGHT,
            current: InFlightFlags::empty(),
        }
    }
}

impl InFlightStatus {
    /// Create with specific required flags
    pub fn with_required(required: InFlightFlags) -> Self {
        Self {
            required,
            current: InFlightFlags::empty(),
        }
    }

    /// Check if all required flags are satisfied
    pub fn is_satisfied(&self) -> bool {
        self.current.contains(self.required)
    }

    /// Get flags that are required but not passing
    pub fn missing(&self) -> InFlightFlags {
        self.required - self.current
    }

    /// Update state validity flags from EKF output
    pub fn update_from_state(&mut self, state: &StateEstimate) {
        self.current.set(
            InFlightFlags::ATTITUDE_VALID,
            state.valid_flags.contains(StateValidFlags::ATTITUDE),
        );
        self.current.set(
            InFlightFlags::VELOCITY_VALID,
            state.valid_flags.contains(StateValidFlags::VELOCITY),
        );
        self.current.set(
            InFlightFlags::POSITION_VALID,
            state.valid_flags.contains(StateValidFlags::POSITION),
        );
        // Heading is valid if attitude is valid
        self.current.set(
            InFlightFlags::HEADING_VALID,
            state.valid_flags.contains(StateValidFlags::ATTITUDE),
        );
    }

    /// Update sensor health flags
    pub fn update_from_sensors(&mut self, sensors: &SensorSet) {
        let imu_ok = sensors
            .imus
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        let baro_ok = sensors
            .baros
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);

        self.current.set(InFlightFlags::IMU_OK, imu_ok);
        self.current.set(InFlightFlags::BARO_OK, baro_ok);
    }

    /// Update envelope protection status
    pub fn update_from_envelope(&mut self, protection: &ProtectionStatus) {
        // Within envelope if no limiting is happening
        let within_envelope = protection.limited_axes.is_empty() && !protection.saturated;
        self.current
            .set(InFlightFlags::WITHIN_ENVELOPE, within_envelope);
    }

    /// Update command timeout status
    ///
    /// # Arguments
    /// * `age_ms` - Age of last command in milliseconds
    /// * `timeout_ms` - Timeout threshold in milliseconds
    pub fn update_command_status(&mut self, age_ms: u32, timeout_ms: u32) {
        self.current
            .set(InFlightFlags::COMMAND_RECENT, age_ms < timeout_ms);
    }

    /// Update RC link status
    pub fn update_rc_status(&mut self, available: bool) {
        self.current.set(InFlightFlags::RC_AVAILABLE, available);
    }

    /// Update altitude OK flag
    pub fn update_altitude(&mut self, within_limits: bool) {
        self.current.set(InFlightFlags::ALTITUDE_OK, within_limits);
    }

    /// Get the highest-priority degradation trigger, if any
    ///
    /// Returns the most critical missing flag that requires immediate response.
    /// Priority order: Attitude > IMU > Position > Velocity > Command > Envelope
    pub fn get_degradation_trigger(&self) -> Option<DegradationReason> {
        let missing = self.missing();

        // Priority 1: Attitude lost is most critical
        if missing.contains(InFlightFlags::ATTITUDE_VALID) {
            return Some(DegradationReason::AttitudeLost);
        }

        // Priority 2: IMU degradation
        if missing.contains(InFlightFlags::IMU_OK) {
            return Some(DegradationReason::ImuDegraded);
        }

        // Priority 3: Position lost - affects position modes
        if missing.contains(InFlightFlags::POSITION_VALID) {
            return Some(DegradationReason::PositionLost);
        }

        // Priority 4: Velocity lost - affects velocity modes
        if missing.contains(InFlightFlags::VELOCITY_VALID) {
            return Some(DegradationReason::VelocityLost);
        }

        // Priority 5: Command timeout
        if missing.contains(InFlightFlags::COMMAND_RECENT) {
            return Some(DegradationReason::CommandTimeout);
        }

        // Priority 6: Envelope violation
        if missing.contains(InFlightFlags::WITHIN_ENVELOPE) {
            return Some(DegradationReason::EnvelopeViolation);
        }

        // Priority 7: Baro degradation
        if missing.contains(InFlightFlags::BARO_OK) {
            return Some(DegradationReason::BaroDegraded);
        }

        // Priority 8: RC link lost
        if missing.contains(InFlightFlags::RC_AVAILABLE) {
            return Some(DegradationReason::RcLost);
        }

        None
    }

    /// Reset all flags (typically on disarm)
    pub fn reset(&mut self) {
        self.current = InFlightFlags::empty();
    }
}

/// Error returned when a transition is not allowed
///
/// Maps to §4.4 TransitionFailure reasons in the spec.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TransitionFailure {
    /// Aircraft not in stable flight
    UnstableFlight,
    /// One or more actuators stuck or failed
    ActuatorStuck,
    /// Outside safe envelope for transition
    UnsafeConditions,
    /// Actuator output asymmetry detected
    Asymmetry,
    /// Insufficient altitude for transition
    AltitudeTooLow,
    /// Insufficient airspeed for forward flight
    AirspeedTooLow,
    /// Multiple checks failed
    MultipleFailures,
}

/// Limits for transition checks
#[derive(Copy, Clone, Debug)]
pub struct TransitionLimits {
    /// Minimum altitude for transition (meters)
    pub min_altitude: f32,
    /// Maximum attitude rate for stable flight (rad/s)
    pub max_attitude_rate: f32,
    /// Minimum airspeed for forward transition (m/s)
    pub min_airspeed: f32,
    /// Maximum actuator asymmetry tolerance (0-1)
    pub max_asymmetry: f32,
}

impl Default for TransitionLimits {
    fn default() -> Self {
        Self {
            min_altitude: 10.0,     // 10m AGL minimum
            max_attitude_rate: 0.5, // ~30 deg/s
            min_airspeed: 15.0,     // 15 m/s for forward flight
            max_asymmetry: 0.1,     // 10% max asymmetry
        }
    }
}

/// Check status for TransitionFlags
#[derive(Copy, Clone, Debug)]
pub struct TransitionStatus {
    /// Checks required for pending transition
    pub required: TransitionFlags,
    /// Checks currently passing
    pub current: TransitionFlags,
    /// Limits for transition checks
    pub limits: TransitionLimits,
}

impl Default for TransitionStatus {
    fn default() -> Self {
        Self {
            required: TransitionFlags::VTOL_TRANSITION,
            current: TransitionFlags::empty(),
            limits: TransitionLimits::default(),
        }
    }
}

impl TransitionStatus {
    /// Create with specific required flags
    pub fn with_required(required: TransitionFlags) -> Self {
        Self {
            required,
            current: TransitionFlags::empty(),
            limits: TransitionLimits::default(),
        }
    }

    /// Create with custom limits
    pub fn with_limits(required: TransitionFlags, limits: TransitionLimits) -> Self {
        Self {
            required,
            current: TransitionFlags::empty(),
            limits,
        }
    }

    /// Check if all required flags are satisfied
    pub fn is_satisfied(&self) -> bool {
        self.current.contains(self.required)
    }

    /// Get flags that are required but not passing
    pub fn missing(&self) -> TransitionFlags {
        self.required - self.current
    }

    /// Update actuator-related flags from ActuatorState
    pub fn update_from_actuators(&mut self, actuators: &ActuatorState, active_mask: u16) {
        // Check if all actuators are healthy (Good or Unknown)
        let actuators_ok = actuators.all_healthy(active_mask);
        self.current
            .set(TransitionFlags::ACTUATORS_OK, actuators_ok);

        // Check for stuck actuators
        let none_stuck = actuators.count_by_health(ActuatorHealth::Stuck, active_mask) == 0;
        // ACTUATORS_OK already includes stuck check, but we can be explicit

        // Check symmetry for quadrotor (pairs: front-left/front-right, rear-left/rear-right)
        // Default pairs for quad X config
        let symmetric = actuators.check_symmetric(&[(0, 1), (2, 3)], self.limits.max_asymmetry);
        self.current
            .set(TransitionFlags::SYMMETRIC, symmetric && none_stuck);
    }

    /// Update state-related flags from StateEstimate
    pub fn update_from_state(&mut self, state: &StateEstimate) {
        // Check altitude (NED frame: z is down, so altitude = -z)
        let altitude = -state.position_ned[2].0;
        self.current.set(
            TransitionFlags::ALTITUDE_OK,
            altitude >= self.limits.min_altitude,
        );

        // Check for stable flight (low angular rates)
        let wx = state.angular_velocity[0].0;
        let wy = state.angular_velocity[1].0;
        let wz = state.angular_velocity[2].0;
        let rate_magnitude = libm::sqrtf(wx * wx + wy * wy + wz * wz);
        self.current.set(
            TransitionFlags::STABLE_FLIGHT,
            rate_magnitude < self.limits.max_attitude_rate,
        );
    }

    /// Update envelope margin flag
    pub fn update_from_envelope(&mut self, margin: &EnvelopeMargin) {
        // Within envelope if all margins are positive
        let within = margin.roll_rad.0 > 0.0
            && margin.pitch_rad.0 > 0.0
            && margin.altitude_m.0 > 0.0
            && margin.load_factor > 0.0;
        self.current.set(TransitionFlags::WITHIN_ENVELOPE, within);
    }

    /// Update airspeed flag
    pub fn update_airspeed(&mut self, airspeed: Option<MetersPerSecond>) {
        let ok = airspeed.is_some_and(|ias| ias.0 >= self.limits.min_airspeed);
        self.current.set(TransitionFlags::AIRSPEED_OK, ok);
    }

    /// Gate function: can transition proceed?
    ///
    /// Returns Ok(()) if all required checks pass, or Err with the primary failure reason.
    #[inline(never)]
    pub fn can_transition(&self) -> Result<(), TransitionFailure> {
        if self.is_satisfied() {
            return Ok(());
        }

        let missing = self.missing();

        // Count how many are missing to detect multiple failures
        let missing_count = missing.bits().count_ones();
        if missing_count > 1 {
            return Err(TransitionFailure::MultipleFailures);
        }

        // Map specific failure
        if missing.contains(TransitionFlags::STABLE_FLIGHT) {
            return Err(TransitionFailure::UnstableFlight);
        }
        if missing.contains(TransitionFlags::ACTUATORS_OK) {
            return Err(TransitionFailure::ActuatorStuck);
        }
        if missing.contains(TransitionFlags::WITHIN_ENVELOPE) {
            return Err(TransitionFailure::UnsafeConditions);
        }
        if missing.contains(TransitionFlags::SYMMETRIC) {
            return Err(TransitionFailure::Asymmetry);
        }
        if missing.contains(TransitionFlags::ALTITUDE_OK) {
            return Err(TransitionFailure::AltitudeTooLow);
        }
        if missing.contains(TransitionFlags::AIRSPEED_OK) {
            return Err(TransitionFailure::AirspeedTooLow);
        }

        // Fallback for any unhandled flags
        Err(TransitionFailure::MultipleFailures)
    }

    /// Reset all flags (typically after transition completes or is aborted)
    pub fn reset(&mut self) {
        self.current = TransitionFlags::empty();
    }
}

// ============================================================================
// KERNEL CHECKS (aggregate)
// ============================================================================

/// All checks managed by the kernel
#[derive(Clone, Debug, Default)]
pub struct KernelChecks {
    /// Pre-arm checks (InitState transitions)
    pub pre_arm: PreArmStatus,
    /// In-flight checks (continuous monitoring)
    pub in_flight: InFlightStatus,
    /// Transition checks (ConfigMode changes)
    pub transition: TransitionStatus,
}

impl KernelChecks {
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with custom pre-arm requirements
    pub fn with_pre_arm_required(required: PreArmFlags) -> Self {
        Self {
            pre_arm: PreArmStatus::with_required(required),
            ..Default::default()
        }
    }
}

// ============================================================================
// INVARIANT CHECKS (DO-178C verification)
// ============================================================================

/// Invariant verification for DO-178C compliance
///
/// These checks verify that the system state is internally consistent.
/// They are run in debug builds and can be used for formal verification.
pub struct CheckInvariants;

impl CheckInvariants {
    /// INV-001: ALL_IMU_FAILED fault implies !IMU_HEALTHY pre-arm flag
    ///
    /// If all IMUs have failed, we cannot have IMU_HEALTHY set.
    pub fn check_imu_consistency(faults: FaultFlags, pre_arm: &PreArmStatus) -> bool {
        if faults.contains(FaultFlags::ALL_IMU_FAILED) {
            !pre_arm.current.contains(PreArmFlags::IMU_HEALTHY)
        } else {
            true // Consistent by default if fault not present
        }
    }

    /// INV-002: ALL_GNSS_LOST fault implies !GNSS_AVAILABLE pre-arm flag
    ///
    /// If GNSS is completely lost, GNSS_AVAILABLE should not be set.
    pub fn check_gnss_consistency(faults: FaultFlags, pre_arm: &PreArmStatus) -> bool {
        if faults.contains(FaultFlags::ALL_GNSS_LOST) {
            !pre_arm.current.contains(PreArmFlags::GNSS_AVAILABLE)
        } else {
            true
        }
    }

    /// INV-003: NO_FAULTS pre-arm flag iff faults.is_empty()
    ///
    /// The NO_FAULTS flag must be consistent with the actual fault state.
    pub fn check_no_faults_consistency(faults: FaultFlags, pre_arm: &PreArmStatus) -> bool {
        let has_no_faults_flag = pre_arm.current.contains(PreArmFlags::NO_FAULTS);
        let actually_no_faults = faults.is_empty();
        has_no_faults_flag == actually_no_faults
    }

    /// INV-004: EKF_CONVERGED implies IMU_CONVERGED
    ///
    /// The EKF cannot converge without IMU data converging first.
    pub fn check_ekf_convergence_consistency(pre_arm: &PreArmStatus) -> bool {
        if pre_arm.current.contains(PreArmFlags::EKF_CONVERGED) {
            pre_arm.current.contains(PreArmFlags::IMU_CONVERGED)
        } else {
            true
        }
    }

    /// INV-005: POSITION_VALID in-flight implies ATTITUDE_VALID
    ///
    /// Position estimate requires a valid attitude estimate.
    pub fn check_position_attitude_consistency(in_flight: &InFlightStatus) -> bool {
        if in_flight.current.contains(InFlightFlags::POSITION_VALID) {
            in_flight.current.contains(InFlightFlags::ATTITUDE_VALID)
        } else {
            true
        }
    }

    /// INV-006: Sample counts must be monotonically increasing (except on reset)
    ///
    /// This invariant is checked by comparing with previous sample counts.
    pub fn check_sample_count_monotonic(prev: &SampleCounts, curr: &SampleCounts) -> bool {
        // Counts should be >= previous unless they were reset to 0
        let imu_ok = curr.imu >= prev.imu || curr.imu == 0;
        let baro_ok = curr.baro >= prev.baro || curr.baro == 0;
        let mag_ok = curr.mag >= prev.mag || curr.mag == 0;
        let gnss_ok = curr.gnss >= prev.gnss || curr.gnss == 0;
        imu_ok && baro_ok && mag_ok && gnss_ok
    }

    /// Run all state consistency checks
    ///
    /// Returns true if all invariants hold, false if any is violated.
    pub fn verify_all(
        faults: FaultFlags,
        pre_arm: &PreArmStatus,
        in_flight: &InFlightStatus,
    ) -> bool {
        Self::check_imu_consistency(faults, pre_arm)
            && Self::check_gnss_consistency(faults, pre_arm)
            && Self::check_no_faults_consistency(faults, pre_arm)
            && Self::check_ekf_convergence_consistency(pre_arm)
            && Self::check_position_attitude_consistency(in_flight)
    }

    /// Get a bitmask of which invariants are violated
    ///
    /// Each bit corresponds to an invariant (bit 0 = INV-001, etc.)
    pub fn get_violations(
        faults: FaultFlags,
        pre_arm: &PreArmStatus,
        in_flight: &InFlightStatus,
    ) -> u8 {
        let mut violations = 0u8;
        if !Self::check_imu_consistency(faults, pre_arm) {
            violations |= 1 << 0; // INV-001
        }
        if !Self::check_gnss_consistency(faults, pre_arm) {
            violations |= 1 << 1; // INV-002
        }
        if !Self::check_no_faults_consistency(faults, pre_arm) {
            violations |= 1 << 2; // INV-003
        }
        if !Self::check_ekf_convergence_consistency(pre_arm) {
            violations |= 1 << 3; // INV-004
        }
        if !Self::check_position_attitude_consistency(in_flight) {
            violations |= 1 << 4; // INV-005
        }
        violations
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pre_arm_flags_quad_minimum() {
        let required = PreArmFlags::QUAD_MINIMUM;
        assert!(required.contains(PreArmFlags::IMU_HEALTHY));
        assert!(required.contains(PreArmFlags::BARO_HEALTHY));
        assert!(required.contains(PreArmFlags::THROTTLE_LOW));
        assert!(!required.contains(PreArmFlags::GNSS_AVAILABLE)); // Not required for basic quad
    }

    #[test]
    fn test_pre_arm_status_missing() {
        let mut status = PreArmStatus::default();
        status.current = PreArmFlags::IMU_HEALTHY | PreArmFlags::BARO_HEALTHY;

        let missing = status.missing();
        assert!(missing.contains(PreArmFlags::THROTTLE_LOW));
        assert!(missing.contains(PreArmFlags::EKF_CONVERGED));
        assert!(!missing.contains(PreArmFlags::IMU_HEALTHY)); // Already passing
    }

    #[test]
    fn test_sample_counts_convergence() {
        let mut counts = SampleCounts::new();
        assert!(!counts.imu_converged());

        for _ in 0..100 {
            counts.imu = counts.imu.saturating_add(1);
        }
        assert!(counts.imu_converged());
    }

    #[test]
    fn test_pre_arm_satisfied() {
        let mut status =
            PreArmStatus::with_required(PreArmFlags::IMU_HEALTHY | PreArmFlags::THROTTLE_LOW);

        assert!(!status.is_satisfied());

        status.current = PreArmFlags::IMU_HEALTHY | PreArmFlags::THROTTLE_LOW;
        assert!(status.is_satisfied());
    }

    #[test]
    fn test_transition_unhandled_flag_unit() {
        let mut status = TransitionStatus::default();
        // Inject a flag that isn't handled by specific checks (bit 30)
        let unknown_flag = TransitionFlags::from_bits_retain(1 << 30);
        status.required = unknown_flag;

        let res = status.can_transition();
        assert_eq!(res, Err(TransitionFailure::MultipleFailures));
    }
}
