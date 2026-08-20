use crate::time::Timestamp;
use crate::types::Normalized;

use crate::control::AxisCommand;

/// Mixer trait - converts axis commands to actuator outputs
pub trait Mixer {
    /// 64-bit algorithm-identity constant, fixed at the impl site.
    /// See `Estimator::ALGORITHM_ID` for the contract — same scope
    /// (mixer-class identity) and same lockstep gating role.
    const ALGORITHM_ID: u64;

    /// Compile-time geometry identity. The builder refuses a kernel
    /// whose resolved configuration declares a different
    /// `mixer_geometry` than the compiled mixer carries, so a preset
    /// resolved for one geometry cannot silently drive another
    /// (#140): the declaration in the canonical hash and the code
    /// that flies are bound at construction.
    const GEOMETRY: crate::kernel::config::MixerGeometry;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd;
}

/// Quadrotor X-configuration mixer
/// Motor layout:
///   0(CW)   1(CCW)
///      \   /
///       \[X\]
///      /   \
///   2(CCW)  3(CW)
///
/// PX4 X500 airframes (and many other off-the-shelf quad-X drones)
/// use the opposite spin pattern: CW on the FL+RR diagonal,
/// CCW on the FR+RL diagonal. See [`QuadXMixerX500`] for that
/// variant. Picking the wrong mixer for the airframe makes the
/// yaw loop close in the wrong direction — the controller
/// commands +yaw and the body yaws -yaw, runaway-style.
pub struct QuadXMixer {
    pub timestamp_source: fn() -> Timestamp,
}

/// Per-motor axis signs for [`QuadXMixer`]:
///
/// ```text
///   m0 = t − r + p − y      m1 = t + r + p + y
///   m2 = t + r − p + y      m3 = t − r − p − y
/// ```
///
/// Roll (+right) lowers the right side (M0/M3), pitch (+nose-up)
/// raises the front (M0/M1), and +yaw (CW) raises the CCW motors
/// (M1/M2) whose reaction torque is CW.
const QUAD_X_SIGNS: desaturate::QuadSigns = desaturate::QuadSigns {
    roll: [-1.0, 1.0, 1.0, -1.0],
    pitch: [1.0, 1.0, -1.0, -1.0],
    yaw: [-1.0, 1.0, 1.0, -1.0],
};

impl Mixer for QuadXMixer {
    // Registered in cert/algorithm_id_registry.toml as
    // "mixer.quad_x.v2" — v2 replaced per-motor clamping with
    // priority desaturation (see `desaturate`), which changes the
    // saturated-regime outputs, so lockstep must not match a v1
    // image.
    const ALGORITHM_ID: u64 = 0x4D49_5851_5541_4432; // "MIXQUAD2"

    const GEOMETRY: crate::kernel::config::MixerGeometry =
        crate::kernel::config::MixerGeometry::QuadX;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        quad_actuator_cmd(
            desaturate::mix_desaturated(
                axis.collective.0,
                axis.roll.0,
                axis.pitch.0,
                axis.yaw.0,
                &QUAD_X_SIGNS,
            ),
            (self.timestamp_source)(),
        )
    }
}

/// Packs four desaturated motor outputs into an [`ActuatorCmd`].
fn quad_actuator_cmd(motors: [crate::types::Scalar; 4], timestamp: Timestamp) -> ActuatorCmd {
    let mut outputs = [Normalized(0.0); MAX_ACTUATORS];
    for (out, m) in outputs.iter_mut().zip(motors) {
        *out = Normalized(m);
    }
    ActuatorCmd {
        outputs,
        active_mask: 0b1111,
        sequence: 0,
        timestamp,
        fallback_mask: 0,
        sanitized: false,
    }
}

/// Quadrotor X-configuration mixer matching the PX4-gazebo-models
/// X500 motor layout (and the PX4 "Quad X" airframe class).
///
/// Motor indices match the gz model's `rotor_N` link names:
/// ```text
///    rotor_2(CW,FL)   rotor_0(CCW,FR)
///                \   /
///                 [X]
///                /   \
///    rotor_1(CCW,RL)  rotor_3(CW,RR)
/// ```
///
/// Yaw signs flip on the CCW corners vs [`QuadXMixer`]; the
/// pitch / roll equations match physical position. Picking this
/// mixer for the X500 closes the yaw loop in the correct
/// direction; picking the wrong mixer makes the yaw command
/// produce body rotation in the opposite direction (positive
/// feedback → tumble).
pub struct QuadXMixerX500 {
    pub timestamp_source: fn() -> Timestamp,
}

/// Per-motor axis signs for [`QuadXMixerX500`]. Roll and pitch
/// follow physical position; yaw sign follows the spin direction's
/// reaction torque on the body — CCW motors produce +CW body torque
/// (so +yaw means more thrust on CCW motors):
///
/// ```text
///   rotor_0: FR, CCW → −r +p +y      rotor_1: RL, CCW → +r −p +y
///   rotor_2: FL, CW  → +r +p −y      rotor_3: RR, CW  → −r −p −y
/// ```
const QUAD_X500_SIGNS: desaturate::QuadSigns = desaturate::QuadSigns {
    roll: [-1.0, 1.0, 1.0, -1.0],
    pitch: [1.0, -1.0, 1.0, -1.0],
    yaw: [1.0, 1.0, -1.0, -1.0],
};

impl Mixer for QuadXMixerX500 {
    // Registered in cert/algorithm_id_registry.toml as
    // "mixer.quad_x_x500.v2" — v2 for the same desaturation change
    // as "mixer.quad_x.v2".
    const ALGORITHM_ID: u64 = 0x4D49_5851_5835_5632; // "MIXQX5V2"

    const GEOMETRY: crate::kernel::config::MixerGeometry =
        crate::kernel::config::MixerGeometry::QuadXX500;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        quad_actuator_cmd(
            desaturate::mix_desaturated(
                axis.collective.0,
                axis.roll.0,
                axis.pitch.0,
                axis.yaw.0,
                &QUAD_X500_SIGNS,
            ),
            (self.timestamp_source)(),
        )
    }
}

/// Quadrotor X-configuration mixer for the X500 lane layout with
/// every rotor's spin direction REVERSED (X-Plane's Alia-250 lift
/// rotors: CW on the FR+RL diagonal, CCW on FL+RR).
///
/// Roll and pitch follow physical position and match
/// [`QuadXMixerX500`]; only the yaw column flips, because yaw sign
/// follows spin direction. Flying the reversed-spin airframe on the
/// X500 mixer closes the yaw loop as positive feedback: the flight
/// that found this wound up from 0.05 to 1.0 rad/s — the attitude
/// loop's rate command limit — with the controller pushing INTO the
/// spin, and strengthening the yaw integrator made it wind up faster.
pub struct QuadXMixerReversedSpin {
    pub timestamp_source: fn() -> Timestamp,
}

/// Per-motor axis signs for [`QuadXMixerReversedSpin`]:
///
/// ```text
///   rotor_0: FR, CW  → −r +p −y      rotor_1: RL, CW  → +r −p −y
///   rotor_2: FL, CCW → +r +p +y      rotor_3: RR, CCW → −r −p +y
/// ```
const QUAD_X500_REVERSED_SIGNS: desaturate::QuadSigns = desaturate::QuadSigns {
    roll: [-1.0, 1.0, 1.0, -1.0],
    pitch: [1.0, -1.0, 1.0, -1.0],
    yaw: [-1.0, -1.0, 1.0, 1.0],
};

impl Mixer for QuadXMixerReversedSpin {
    // Registered in cert/algorithm_id_registry.toml as
    // "mixer.quad_x_x500_reversed_spin.v1".
    const ALGORITHM_ID: u64 = 0x4D49_5851_5852_5631; // "MIXQXRV1"

    const GEOMETRY: crate::kernel::config::MixerGeometry =
        crate::kernel::config::MixerGeometry::QuadXX500ReversedSpin;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        quad_actuator_cmd(
            desaturate::mix_desaturated(
                axis.collective.0,
                axis.roll.0,
                axis.pitch.0,
                axis.yaw.0,
                &QUAD_X500_REVERSED_SIGNS,
            ),
            (self.timestamp_source)(),
        )
    }
}

mod actuators;
mod desaturate;
mod replicable;
mod sanitizer_impl;

pub use actuators::*;
