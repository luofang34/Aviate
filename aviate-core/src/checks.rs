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

use crate::sensor::{SensorSet, SensorHealth};
use crate::fault::FaultFlags;

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
        let imu_healthy = sensors.imus.iter().any(|s| s.valid && s.health == SensorHealth::Good);
        let baro_healthy = sensors.baros.iter().any(|s| s.valid && s.health == SensorHealth::Good);
        let mag_healthy = sensors.mags.iter().any(|s| s.valid && s.health == SensorHealth::Good);
        let gnss_available = sensors.gnss.iter().any(|s| s.valid && s.health == SensorHealth::Good);

        self.current.set(PreArmFlags::IMU_HEALTHY, imu_healthy);
        self.current.set(PreArmFlags::BARO_HEALTHY, baro_healthy);
        self.current.set(PreArmFlags::MAG_HEALTHY, mag_healthy);
        self.current.set(PreArmFlags::GNSS_AVAILABLE, gnss_available);

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
        self.current.set(PreArmFlags::IMU_CONVERGED, self.samples.imu_converged());
        self.current.set(PreArmFlags::BARO_CONVERGED, self.samples.baro_converged());
        self.current.set(PreArmFlags::MAG_CONVERGED, self.samples.mag_converged());
    }

    /// Update from fault flags
    pub fn update_from_faults(&mut self, faults: FaultFlags) {
        self.current.set(PreArmFlags::NO_FAULTS, faults.is_empty());
        self.current.set(PreArmFlags::CONFIG_VALID, !faults.contains(FaultFlags::CONFIG_INVALID));
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
    pub fn is_satisfied(&self) -> bool {
        self.current.contains(self.required)
    }

    pub fn missing(&self) -> InFlightFlags {
        self.required - self.current
    }
}

/// Check status for TransitionFlags
#[derive(Copy, Clone, Debug)]
pub struct TransitionStatus {
    /// Checks required for pending transition
    pub required: TransitionFlags,
    /// Checks currently passing
    pub current: TransitionFlags,
}

impl Default for TransitionStatus {
    fn default() -> Self {
        Self {
            required: TransitionFlags::VTOL_TRANSITION,
            current: TransitionFlags::empty(),
        }
    }
}

impl TransitionStatus {
    pub fn is_satisfied(&self) -> bool {
        self.current.contains(self.required)
    }

    pub fn missing(&self) -> TransitionFlags {
        self.required - self.current
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
        let mut status = PreArmStatus::with_required(
            PreArmFlags::IMU_HEALTHY | PreArmFlags::THROTTLE_LOW
        );

        assert!(!status.is_satisfied());

        status.current = PreArmFlags::IMU_HEALTHY | PreArmFlags::THROTTLE_LOW;
        assert!(status.is_satisfied());
    }
}
