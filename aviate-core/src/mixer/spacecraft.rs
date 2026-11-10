//! Actuator mixer for spacecraft and launch/reentry vehicles.
//!
//! Maps [`AxisCommand`] (`collective` main-engine throttle and
//! `roll`/`pitch`/`yaw` normalized torque demands), reaction-wheel momentum
//! state, and body-frame velocity error into 15 bounded `[0.0, 1.0]`
//! actuator channels (`active_mask = 0x7FFF`):
//!
//! - Channel 0: main-engine throttle `[0.0, 1.0]`
//! - Channels 1..=2: 2-axis TVC gimbal (`pitch`, `yaw`) centered at `0.5`
//! - Channels 3..=5: 3-axis aerodynamic fins (`roll`, `pitch`, `yaw`) centered at `0.5`
//! - Channels 6..=8: 3-axis reaction wheels (`roll`, `pitch`, `yaw`) centered at `0.5`
//! - Channels 9..=11: 3-axis rotational RCS (`roll`, `pitch`, `yaw`) centered at `0.5`
//! - Channels 12..=14: 3-axis CG translational RCS (`X`, `Y`, `Z`) centered at `0.5`

use super::{ActuatorCmd, Mixer, MAX_ACTUATORS};
use crate::control::spacecraft::{allocate_rw_and_rotational_rcs, allocate_translational_rcs};
use crate::control::AxisCommand;
use crate::time::Timestamp;
use crate::types::{MetersPerSecond, Normalized, NormalizedSigned};

/// Bitmask for the 15 active spacecraft actuator channels (`0..=14`).
pub const SPACECRAFT_ACTIVE_MASK: u16 = 0x7FFF;

/// Fifteen-channel spacecraft mixer allocating main throttle, 2-axis TVC,
/// 3-axis aerodynamic fins, 3-axis reaction wheels, 3-axis rotational RCS,
/// and 3-axis translational RCS.
pub struct SpacecraftMixer {
    /// Timestamp provider for emitted [`ActuatorCmd`] packets.
    pub timestamp_source: fn() -> Timestamp,
    /// Normalized stored angular momentum `[0.0, 1.0]` per reaction-wheel axis.
    pub wheel_momentum_norm: [Normalized; 3],
    /// Body-frame translational velocity error `[m/s]` for 6-DOF CG RCS allocation.
    pub velocity_error_body_mps: [MetersPerSecond; 3],
}

impl SpacecraftMixer {
    /// Construct a [`SpacecraftMixer`] with zero initial wheel momentum and
    /// zero translational velocity error.
    #[must_use]
    pub const fn new(timestamp_source: fn() -> Timestamp) -> Self {
        Self {
            timestamp_source,
            wheel_momentum_norm: [Normalized(0.0); 3],
            velocity_error_body_mps: [MetersPerSecond(0.0); 3],
        }
    }
}

fn bipolar_to_unit(value: NormalizedSigned) -> Normalized {
    Normalized((0.5 + 0.5 * value.0).clamp(0.0, 1.0))
}

impl Mixer for SpacecraftMixer {
    // Registered in cert/algorithm_id_registry.toml as
    // "mixer.spacecraft.v1".
    const ALGORITHM_ID: u64 = 0x4D49_5853_5043_5631; // "MIXSPCV1"

    const GEOMETRY: crate::kernel::config::MixerGeometry =
        crate::kernel::config::MixerGeometry::QuadX;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        let rw_rcs = allocate_rw_and_rotational_rcs(
            [axis.roll, axis.pitch, axis.yaw],
            self.wheel_momentum_norm,
        );
        let trans_rcs = allocate_translational_rcs(self.velocity_error_body_mps);

        let mut outputs = [Normalized(0.0); MAX_ACTUATORS];
        // Ch 0: main engine throttle
        outputs[0] = Normalized(axis.collective.0.clamp(0.0, 1.0));
        // Ch 1..=2: 2-axis TVC gimbal (pitch, yaw)
        outputs[1] = bipolar_to_unit(axis.pitch);
        outputs[2] = bipolar_to_unit(axis.yaw);
        // Ch 3..=5: 3-axis aerodynamic fins (roll, pitch, yaw)
        outputs[3] = bipolar_to_unit(axis.roll);
        outputs[4] = bipolar_to_unit(axis.pitch);
        outputs[5] = bipolar_to_unit(axis.yaw);
        // Ch 6..=8: 3-axis reaction wheels (roll, pitch, yaw)
        outputs[6] = bipolar_to_unit(rw_rcs.reaction_wheel_torque[0]);
        outputs[7] = bipolar_to_unit(rw_rcs.reaction_wheel_torque[1]);
        outputs[8] = bipolar_to_unit(rw_rcs.reaction_wheel_torque[2]);
        // Ch 9..=11: 3-axis rotational RCS (roll, pitch, yaw)
        outputs[9] = bipolar_to_unit(rw_rcs.rcs_attitude_torque[0]);
        outputs[10] = bipolar_to_unit(rw_rcs.rcs_attitude_torque[1]);
        outputs[11] = bipolar_to_unit(rw_rcs.rcs_attitude_torque[2]);
        // Ch 12..=14: 3-axis CG translational RCS (X, Y, Z)
        outputs[12] = bipolar_to_unit(trans_rcs[0]);
        outputs[13] = bipolar_to_unit(trans_rcs[1]);
        outputs[14] = bipolar_to_unit(trans_rcs[2]);

        ActuatorCmd {
            outputs,
            active_mask: SPACECRAFT_ACTIVE_MASK,
            sequence: 0,
            timestamp: (self.timestamp_source)(),
            fallback_mask: 0,
            sanitized: false,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::time::TimeSource;
    use crate::types::{NormalizedSigned, NormalizedThrust};

    fn ts() -> Timestamp {
        Timestamp {
            ticks: 42,
            source: TimeSource::Internal,
        }
    }

    #[test]
    fn mixes_throttle_tvc_fins_rw_and_rcs_channels() {
        let mut mixer = SpacecraftMixer::new(ts);
        mixer.wheel_momentum_norm = [Normalized(0.2), Normalized(0.9), Normalized(0.1)];
        mixer.velocity_error_body_mps = [
            MetersPerSecond(0.01),
            MetersPerSecond(0.22),
            MetersPerSecond(-0.62),
        ];
        let cmd = mixer.mix(&AxisCommand {
            roll: NormalizedSigned(0.5),
            pitch: NormalizedSigned(-0.5),
            yaw: NormalizedSigned(0.0),
            collective: NormalizedThrust(0.8),
        });
        assert_eq!(cmd.active_mask, SPACECRAFT_ACTIVE_MASK);
        assert_eq!(cmd.timestamp.ticks, 42);
        // Main throttle
        assert!((cmd.outputs[0].0 - 0.8).abs() < 1e-6);
        // TVC pitch & yaw
        assert!((cmd.outputs[1].0 - 0.25).abs() < 1e-6);
        assert!((cmd.outputs[2].0 - 0.5).abs() < 1e-6);
        // Aero fins roll, pitch, yaw
        assert!((cmd.outputs[3].0 - 0.75).abs() < 1e-6);
        assert!((cmd.outputs[4].0 - 0.25).abs() < 1e-6);
        assert!((cmd.outputs[5].0 - 0.5).abs() < 1e-6);
        // RW roll unsaturated (0.75), RW pitch saturated -> neutral (0.5)
        assert!((cmd.outputs[6].0 - 0.75).abs() < 1e-6);
        assert!((cmd.outputs[7].0 - 0.5).abs() < 1e-6);
        assert!((cmd.outputs[8].0 - 0.5).abs() < 1e-6);
        // Rotational RCS roll inactive (0.5), pitch active (0.25)
        assert!((cmd.outputs[9].0 - 0.5).abs() < 1e-6);
        assert!((cmd.outputs[10].0 - 0.25).abs() < 1e-6);
        assert!((cmd.outputs[11].0 - 0.5).abs() < 1e-6);
        // Translational RCS: X inside deadband (0.5), Y +0.5 -> 0.75, Z -1.0 -> 0.0
        assert!((cmd.outputs[12].0 - 0.5).abs() < 1e-6);
        assert!((cmd.outputs[13].0 - 0.75).abs() < 1e-5);
        assert!((cmd.outputs[14].0 - 0.0).abs() < 1e-5);
    }
}
