//! Airborne determination (spec §17 lifecycle support).
//!
//! The kernel needs one question answered before it can decide whether an
//! ordinary disarm is safe: *is the vehicle currently holding itself up?*
//! Nothing else in the kernel answers it. [`crate::checks::InFlightFlags`]
//! reports whether estimates and sensors are *usable*, not whether the
//! aircraft has left the ground — a vehicle sitting on the pad with a
//! healthy estimator passes every in-flight check.
//!
//! ## Why a latch rather than an instantaneous test
//!
//! Height alone is not a decision: it dips through the takeoff threshold
//! on every bounce, and a vehicle descending through it is still flying.
//! [`FlightPhaseState`] therefore latches:
//!
//! * `OnGround → Airborne` on positive evidence of having climbed away
//!   from the datum captured at arm;
//! * `Airborne → OnGround` only after the landed condition (low height
//!   *and* low vertical speed) holds continuously for a debounce period.
//!
//! ## Which signal the height comes from
//!
//! Height is read from the estimate's vertical channel, gated on the
//! estimator's own `EstimateQuality` rather than on
//! `StateValidFlags::POSITION`. The POSITION flag means a full 3-D fix,
//! which a multirotor flying on baro and inertial aiding alone never
//! raises — gating on it would leave the latch permanently `OnGround`
//! through an entire GNSS-denied flight, which is precisely the flight
//! where an unguarded disarm is most likely to be reached for. Quality
//! is the same self-assessment the pre-arm estimator gate already
//! trusts to decide whether the vehicle may arm at all.
//!
//! ## Behaviour when the estimate is unusable
//!
//! An `Unusable` estimate freezes the phase where it is: the latch
//! neither sets nor clears. Both directions are the conservative choice
//! for the decision this feeds. A vehicle believed airborne keeps
//! refusing an ordinary disarm; a vehicle that never demonstrably left
//! the ground keeps accepting one, so a degraded estimator can never
//! strand a pilot with spinning motors and no ordinary way to stop them.
//!
//! The residual case — an estimator so broken from the moment of arm
//! that a real climb is never observed — leaves the phase at `OnGround`
//! and an in-flight disarm reachable. That case is what the pre-arm
//! estimator gates (HLR-FLT-202) exist to prevent, and the emergency
//! terminate path is deliberately independent of this determination.

use crate::replicable::{copy_into, Replicable};
use crate::state::{EstimateQuality, StateEstimate, StateValidFlags};
use crate::types::{Meters, MetersPerSecond, Seconds};

/// Whether the vehicle is holding itself up.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum FlightPhase {
    /// Not observed to have left the ground since the last arm.
    #[default]
    OnGround = 0,
    /// Observed to have climbed away from the arm datum and not yet
    /// observed to have landed again.
    Airborne = 1,
}

/// Thresholds governing the [`FlightPhase`] latch.
///
/// Separate takeoff and landed heights give the latch hysteresis, so a
/// vehicle hovering near the threshold cannot chatter between phases.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FlightPhaseLimits {
    /// Height above the arm datum that latches `Airborne`.
    pub takeoff_height: Meters,
    /// Height above the arm datum below which the vehicle may be
    /// considered landed. Must be below `takeoff_height`.
    pub landed_height: Meters,
    /// Vertical speed below which the vehicle may be considered landed.
    pub landed_speed: MetersPerSecond,
    /// How long the landed condition must hold continuously before the
    /// latch clears.
    ///
    /// Expressed as a duration rather than a cycle count because the
    /// kernel does not run at one fixed rate — the Gazebo and hardware
    /// runners step at 1 kHz, the jMAVSim runner at 400 Hz. A cycle
    /// count would silently mean different amounts of real time on
    /// different vehicles, and the question this answers ("has it been
    /// still long enough to call it down?") is about seconds.
    pub landed_debounce: Seconds,
}

impl Default for FlightPhaseLimits {
    fn default() -> Self {
        Self {
            // Clear of ground effect and of the noise floor of a
            // baro/GNSS-aided vertical estimate, while still low enough
            // that the latch is set long before the vehicle is high
            // enough for a motor cut to break anything.
            takeoff_height: Meters(0.5),
            landed_height: Meters(0.3),
            landed_speed: MetersPerSecond(0.3),
            landed_debounce: Seconds(0.5),
        }
    }
}

impl FlightPhaseLimits {
    /// Whether the thresholds are ordered such that the latch has
    /// hysteresis. Equal heights would make the latch chatter.
    pub fn is_hysteretic(&self) -> bool {
        self.landed_height.0 < self.takeoff_height.0
    }
}

/// Latching airborne determination.
///
/// Owned by the kernel; updated once per cycle from the estimate, and
/// read by the lifecycle transitions that must not drop a flying
/// aircraft.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct FlightPhaseState {
    phase: FlightPhase,
    /// Altitude (up-positive) captured at the arm that began this
    /// flight period. Meaningless unless `datum_valid`.
    ground_datum: Meters,
    datum_valid: bool,
    /// Seconds the landed condition has held continuously.
    landed_elapsed: Seconds,
}

impl FlightPhaseState {
    /// Current phase.
    pub fn phase(&self) -> FlightPhase {
        self.phase
    }

    /// Whether the vehicle is believed to be holding itself up.
    pub fn is_airborne(&self) -> bool {
        self.phase == FlightPhase::Airborne
    }

    /// Capture the ground datum for a new flight period.
    ///
    /// Called on a successful arm. A datum captured from an invalid
    /// position estimate is marked unusable rather than trusted, which
    /// leaves the latch unable to set — see the module note on the
    /// residual case.
    pub fn begin_flight_period(&mut self, estimate: &StateEstimate) {
        self.phase = FlightPhase::OnGround;
        self.landed_elapsed = Seconds(0.0);
        self.datum_valid = height_is_usable(estimate);
        self.ground_datum = Meters(altitude_of(estimate).0);
    }

    /// Clear the determination back to its power-on shape.
    ///
    /// Called wherever the flight period ends — disarm, terminate, and
    /// ground reset — so the next arm starts from a fresh datum.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Fold one cycle's estimate into the latch.
    ///
    /// `armed` is the kernel's own lifecycle view: a disarmed vehicle
    /// cannot be climbing under its own power, so the latch only
    /// advances while armed.
    pub fn update(
        &mut self,
        estimate: &StateEstimate,
        armed: bool,
        dt: Seconds,
        limits: &FlightPhaseLimits,
    ) {
        if !armed || !self.datum_valid {
            return;
        }
        if !height_is_usable(estimate) {
            // Freeze: an unusable estimate is not evidence of anything.
            // The debounce restarts so a landing must be re-demonstrated
            // from a usable estimate rather than completed across a gap.
            self.landed_elapsed = Seconds(0.0);
            return;
        }

        let height = altitude_of(estimate).0 - self.ground_datum.0;
        match self.phase {
            FlightPhase::OnGround => {
                if height > limits.takeoff_height.0 {
                    self.phase = FlightPhase::Airborne;
                    self.landed_elapsed = Seconds(0.0);
                }
            }
            FlightPhase::Airborne => self.update_landed_debounce(height, estimate, dt, limits),
        }
    }

    /// Advance (or restart) the landed debounce while airborne.
    ///
    /// Vertical speed is required as well as height because a vehicle
    /// descending fast through the landed height is mid-approach, not
    /// down. Velocity validity is required for the same reason the
    /// position gate exists: an unusable vertical speed is not evidence
    /// that the vehicle is at rest.
    fn update_landed_debounce(
        &mut self,
        height: f32,
        estimate: &StateEstimate,
        dt: Seconds,
        limits: &FlightPhaseLimits,
    ) {
        let velocity_usable = estimate.valid_flags.contains(StateValidFlags::VELOCITY);
        let descending_slowly =
            velocity_usable && vertical_speed_of(estimate).0.abs() < limits.landed_speed.0;

        if !(height < limits.landed_height.0 && descending_slowly) {
            self.landed_elapsed = Seconds(0.0);
            return;
        }

        // A landing is a thing observed over time, so no single sample
        // may complete it. A step is capped at half the window, which
        // means at least two observations however the clock behaves;
        // a non-finite or non-positive step contributes nothing at all.
        // Without the cap, one cycle after a stall — where there was no
        // continuous observation to accumulate — would land the
        // vehicle on a single post-stall sample.
        let step = if dt.0.is_finite() && dt.0 > 0.0 {
            dt.0.min(limits.landed_debounce.0 * 0.5)
        } else {
            0.0
        };
        self.landed_elapsed = Seconds(self.landed_elapsed.0 + step);
        if self.landed_elapsed.0 >= limits.landed_debounce.0 {
            self.phase = FlightPhase::OnGround;
            self.landed_elapsed = Seconds(0.0);
        }
    }
}

/// Whether the estimate's vertical channel can be believed.
///
/// `Degraded` still counts: a baro/inertial vertical solution is exactly
/// what a GNSS-denied multirotor flies on, and it is far more accurate
/// than the half-metre threshold this feeds. Only `Unusable` — a filter
/// that never initialized or has latched a numeric fault — is refused.
fn height_is_usable(estimate: &StateEstimate) -> bool {
    estimate.quality != EstimateQuality::Unusable
}

/// Altitude (up-positive) from an NED estimate, whose z axis is
/// down-positive.
fn altitude_of(estimate: &StateEstimate) -> Meters {
    Meters(-estimate.position_ned[2].0)
}

/// Vertical speed (up-positive) from an NED estimate.
fn vertical_speed_of(estimate: &StateEstimate) -> MetersPerSecond {
    MetersPerSecond(-estimate.velocity_ned[2].0)
}

impl Replicable for FlightPhase {
    const ENCODED_LEN: usize = 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        copy_into(buf, 0, &[*self as u8])
    }
}

impl Replicable for FlightPhaseState {
    const ENCODED_LEN: usize = FlightPhase::ENCODED_LEN + 4 + 1 + 4;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut written = self.phase.encode_canonical(buf);
        written += copy_into(buf, written, &self.ground_datum.0.to_le_bytes());
        written += copy_into(buf, written, &[self.datum_valid as u8]);
        written += copy_into(buf, written, &self.landed_elapsed.0.to_le_bytes());
        written
    }
}

#[cfg(test)]
mod tests;
