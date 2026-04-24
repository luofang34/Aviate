//! In-flight checks (continuous monitoring, spec §14 / §15).
//!
//! `DegradationReason` is the kernel's decision signal: given a set of
//! failed in-flight checks, `InFlightStatus::get_degradation_trigger`
//! returns the highest-priority reason to drop to a less capable
//! control law.

use crate::control::envelope::ProtectionStatus;
use crate::sensor::{SensorHealth, SensorSet};
use crate::state::{StateEstimate, StateValidFlags};

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
    /// Persistent timing violation (spec §18)
    TimingViolation,
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
