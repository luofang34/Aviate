//! Transition checks (spec §4.5 Config Mode transitions).
//!
//! Safety gate for `ConfigMode` changes (e.g. VTOL hover → forward flight).
//! `TransitionStatus::can_transition` returns the primary reason to reject
//! a transition attempt so the kernel can surface a single, specific failure.

use crate::control::envelope::EnvelopeMargin;
use crate::mixer::{ActuatorHealth, ActuatorState};
use crate::state::StateEstimate;
use crate::types::MetersPerSecond;

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

impl crate::replicable::Replicable for TransitionFlags {
    const ENCODED_LEN: usize = 4;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = crate::replicable::ByteWriter::new(buf);
        w.write_u32(self.bits());
        w.bytes_written()
    }
}

impl crate::replicable::Replicable for TransitionLimits {
    // 4 × f32 = 16 bytes.
    const ENCODED_LEN: usize = 4 * 4;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = crate::replicable::ByteWriter::new(buf);
        w.write_f32(self.min_altitude);
        w.write_f32(self.max_attitude_rate);
        w.write_f32(self.min_airspeed);
        w.write_f32(self.max_asymmetry);
        w.bytes_written()
    }
}

impl crate::replicable::Replicable for TransitionStatus {
    const ENCODED_LEN: usize =
        TransitionFlags::ENCODED_LEN + TransitionFlags::ENCODED_LEN + TransitionLimits::ENCODED_LEN;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut written = self.required.encode_canonical(buf);
        if written < buf.len() {
            written += self.current.encode_canonical(&mut buf[written..]);
        }
        if written < buf.len() {
            written += self.limits.encode_canonical(&mut buf[written..]);
        }
        written
    }
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

#[cfg(test)]
mod tests {
    use super::*;

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
