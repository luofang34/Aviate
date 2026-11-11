//! Actuator mixer for a conventional fixed-wing vehicle.
//!
//! Maps [`AxisCommand`] onto four lanes (`active_mask = 0b1111`):
//!
//! - Lane 0: throttle, `0.0` (idle) to `1.0` (full)
//! - Lane 1: aileron, `0.5` neutral, more than `0.5` rolls right
//! - Lane 2: elevator, `0.5` neutral, more than `0.5` pitches the nose up
//! - Lane 3: rudder, `0.5` neutral, more than `0.5` yaws the nose right
//!
//! The lanes carry the effect on the vehicle, as a pilot's controls do,
//! not the deflection sign of one surface. The board maps a lane onto
//! its servos.

use super::{ActuatorCmd, Mixer, MAX_ACTUATORS};
use crate::control::AxisCommand;
use crate::time::Timestamp;
use crate::types::{Normalized, NormalizedSigned};

/// Bitmask for the four active fixed-wing lanes.
pub const FIXED_WING_ACTIVE_MASK: u16 = 0b1111;

/// Lane of the throttle.
pub const THROTTLE_LANE: usize = 0;
/// Lane of the aileron.
pub const AILERON_LANE: usize = 1;
/// Lane of the elevator.
pub const ELEVATOR_LANE: usize = 2;
/// Lane of the rudder.
pub const RUDDER_LANE: usize = 3;

/// Four-lane fixed-wing mixer: throttle, aileron, elevator, rudder.
pub struct FixedWingMixer {
    /// Timestamp provider for emitted [`ActuatorCmd`] packets.
    pub timestamp_source: fn() -> Timestamp,
}

/// Actuator vector with zero throttle and neutral surfaces.
///
/// A fixed-wing configuration uses it as `safe_output`: a vector of all
/// zeros would hold each surface at its stop.
#[must_use]
pub fn neutral_outputs() -> [Normalized; MAX_ACTUATORS] {
    lanes(Normalized(0.0), [NormalizedSigned(0.0); 3])
}

fn surface(value: NormalizedSigned) -> Normalized {
    Normalized((0.5 + 0.5 * value.0).clamp(0.0, 1.0))
}

fn lanes(throttle: Normalized, surfaces: [NormalizedSigned; 3]) -> [Normalized; MAX_ACTUATORS] {
    let mut outputs = [Normalized(0.0); MAX_ACTUATORS];
    let values = core::iter::once(throttle).chain(surfaces.into_iter().map(surface));
    for (lane, value) in outputs.iter_mut().zip(values) {
        *lane = value;
    }
    outputs
}

impl Mixer for FixedWingMixer {
    // Registered in cert/algorithm_id_registry.toml as
    // "mixer.fixed_wing.v1".
    const ALGORITHM_ID: u64 = 0x4D49_5846_5747_5631; // "MIXFWGV1"

    const GEOMETRY: crate::kernel::config::MixerGeometry =
        crate::kernel::config::MixerGeometry::FixedWing;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        ActuatorCmd {
            outputs: lanes(
                Normalized(axis.collective.0.clamp(0.0, 1.0)),
                [axis.roll, axis.pitch, axis.yaw],
            ),
            active_mask: FIXED_WING_ACTIVE_MASK,
            sequence: 0,
            timestamp: (self.timestamp_source)(),
            fallback_mask: 0,
            sanitized: false,
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::time::TimeSource;
    use crate::types::NormalizedThrust;

    fn ts() -> Timestamp {
        Timestamp {
            ticks: 77,
            source: TimeSource::Internal,
        }
    }

    fn mix(roll: f32, pitch: f32, yaw: f32, throttle: f32) -> ActuatorCmd {
        FixedWingMixer {
            timestamp_source: ts,
        }
        .mix(&AxisCommand {
            roll: NormalizedSigned(roll),
            pitch: NormalizedSigned(pitch),
            yaw: NormalizedSigned(yaw),
            collective: NormalizedThrust(throttle),
        })
    }

    #[test]
    fn lanes_carry_pilot_sense_commands_around_a_neutral_half() {
        let cmd = mix(0.5, -1.0, 1.0, 0.25);
        assert_eq!(cmd.active_mask, FIXED_WING_ACTIVE_MASK);
        assert_eq!(cmd.timestamp.ticks, 77);
        assert_eq!(cmd.outputs[THROTTLE_LANE].0, 0.25);
        // Roll right is above neutral, nose down is below neutral, nose
        // right is above neutral.
        assert_eq!(cmd.outputs[AILERON_LANE].0, 0.75);
        assert_eq!(cmd.outputs[ELEVATOR_LANE].0, 0.0);
        assert_eq!(cmd.outputs[RUDDER_LANE].0, 1.0);
        assert!(cmd.outputs[4..].iter().all(|lane| lane.0 == 0.0));
    }

    #[test]
    fn commands_outside_the_range_stop_at_the_lane_limits() {
        let cmd = mix(3.0, -3.0, 0.0, 1.7);
        assert_eq!(cmd.outputs[THROTTLE_LANE].0, 1.0);
        assert_eq!(cmd.outputs[AILERON_LANE].0, 1.0);
        assert_eq!(cmd.outputs[ELEVATOR_LANE].0, 0.0);
        assert_eq!(cmd.outputs[RUDDER_LANE].0, 0.5);
        assert_eq!(mix(0.0, 0.0, 0.0, -0.4).outputs[THROTTLE_LANE].0, 0.0);
    }

    #[test]
    fn neutral_vector_is_idle_throttle_and_centred_surfaces() {
        let neutral = neutral_outputs();
        assert_eq!(neutral[THROTTLE_LANE].0, 0.0);
        assert_eq!(neutral[AILERON_LANE].0, 0.5);
        assert_eq!(neutral[ELEVATOR_LANE].0, 0.5);
        assert_eq!(neutral[RUDDER_LANE].0, 0.5);
        assert!(neutral[4..].iter().all(|lane| lane.0 == 0.0));
        assert_eq!(mix(0.0, 0.0, 0.0, 0.0).outputs, neutral);
    }
}
