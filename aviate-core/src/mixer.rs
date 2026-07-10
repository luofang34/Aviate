use crate::control::ConfigMode;
use crate::time::Timestamp;
use crate::types::Normalized;

use crate::control::AxisCommand;

/// Mixer trait - converts axis commands to actuator outputs
pub trait Mixer {
    /// 64-bit algorithm-identity constant, fixed at the impl site.
    /// See `Estimator::ALGORITHM_ID` for the contract — same scope
    /// (mixer-class identity) and same lockstep gating role.
    const ALGORITHM_ID: u64;

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

pub const MAX_ACTUATORS: usize = 16;
pub const MAX_GROUPS: usize = 8;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ActuatorKind {
    Motor,
    MotorBidirectional,
    Servo,
    TiltServo,
    Wheel,
    Flap,
    Spoiler,
    MorphingJoint,
    Custom(u8),
}

#[derive(Copy, Clone, Debug)]
pub struct ActuatorChannelConfig {
    pub kind: ActuatorKind,
    pub output_min: u16,
    pub output_max: u16,
    pub safe_output: Normalized,
    pub enabled: bool,
}

/// Group kind - semantic role of this actuator group
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GroupKind {
    Multirotor,
    DistributedThrust,
    ControlSurfaces,
    Morphing,
    Auxiliary,
    Custom(u8),
}

/// Coupling semantics within a group
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CouplingKind {
    Strong,
    Weak,
}

/// Group-level actuator vector (shared by config and runtime fallback)
#[derive(Clone, Debug, Copy)] // Copy needed for array init if not using Default
pub struct GroupVector {
    pub outputs: [Normalized; MAX_ACTUATORS],
    pub mask: u16,
    pub valid: bool,
}

impl Default for GroupVector {
    fn default() -> Self {
        Self {
            outputs: [Normalized(0.0); MAX_ACTUATORS],
            mask: 0,
            valid: false,
        }
    }
}

/// Fallback strategy when fault detected
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FallbackPolicy {
    HoldLastGood,
    DecayToSafe { tau_ms: u16 },
    SafePattern,
}

/// Configuration for one actuator group (defined per ConfigMode)
#[derive(Clone, Debug)]
pub struct ActuatorGroupConfig {
    pub kind: GroupKind,
    pub coupling: CouplingKind,
    pub fallback: FallbackPolicy,
    /// Member channel indices
    pub members: &'static [u8],
    /// Safe pattern for this group
    pub safe_pattern: GroupVector,
}

#[derive(Clone, Debug)]
pub struct ActuatorCmd {
    pub outputs: [Normalized; MAX_ACTUATORS],
    pub active_mask: u16,
    pub sequence: u32,
    pub timestamp: Timestamp,
    pub fallback_mask: u8,
    pub sanitized: bool,
}

impl Default for ActuatorCmd {
    fn default() -> Self {
        Self {
            outputs: [Normalized(0.0); MAX_ACTUATORS],
            active_mask: 0,
            sequence: 0,
            timestamp: Timestamp::default(),
            fallback_mask: 0,
            sanitized: false,
        }
    }
}

/// Health status for individual actuators (DO-178C traceability)
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum ActuatorHealth {
    /// Actuator operating normally
    Good,
    /// Actuator degraded but functional (e.g., reduced performance)
    Degraded,
    /// Actuator has failed completely
    Failed,
    /// Actuator is stuck at a fixed position
    Stuck,
    /// Health status cannot be determined
    #[default]
    Unknown,
}

/// Runtime actuator state with health monitoring
///
/// Used by TransitionStatus to gate configuration mode changes,
/// ensuring actuator health before critical transitions.
#[derive(Clone, Debug)]
pub struct ActuatorState {
    /// Health status per actuator channel
    pub health: [ActuatorHealth; MAX_ACTUATORS],
    /// Last commanded output values
    pub commanded: [Normalized; MAX_ACTUATORS],
    /// Actual position/speed from feedback sensors (if available)
    pub actual: Option<[Normalized; MAX_ACTUATORS]>,
    /// Timestamp of last update
    pub timestamp: Timestamp,
}

impl Default for ActuatorState {
    fn default() -> Self {
        Self {
            health: [ActuatorHealth::Unknown; MAX_ACTUATORS],
            commanded: [Normalized(0.0); MAX_ACTUATORS],
            actual: None,
            timestamp: Timestamp::default(),
        }
    }
}

impl ActuatorState {
    /// Create a new ActuatorState with all actuators in Unknown health
    pub fn new() -> Self {
        Self::default()
    }

    /// Update commanded outputs from an ActuatorCmd
    pub fn update_commanded(&mut self, cmd: &ActuatorCmd, timestamp: Timestamp) {
        self.commanded = cmd.outputs;
        self.timestamp = timestamp;
    }

    /// Update health status for a specific channel
    pub fn set_health(&mut self, channel: usize, health: ActuatorHealth) {
        if channel < MAX_ACTUATORS {
            self.health[channel] = health;
        }
    }

    /// Update actual feedback for a specific channel
    pub fn set_actual(&mut self, channel: usize, value: Normalized) {
        let actual = self.actual.get_or_insert([Normalized(0.0); MAX_ACTUATORS]);
        if channel < MAX_ACTUATORS {
            actual[channel] = value;
        }
    }

    /// Check if all active actuators (by mask) are healthy
    pub fn all_healthy(&self, active_mask: u16) -> bool {
        for i in 0..MAX_ACTUATORS {
            if (active_mask & (1 << i)) != 0 {
                match self.health[i] {
                    ActuatorHealth::Good | ActuatorHealth::Unknown => {}
                    ActuatorHealth::Degraded | ActuatorHealth::Failed | ActuatorHealth::Stuck => {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Check if actuators are symmetric (for transition safety)
    /// Returns true if paired actuators have similar commanded/actual values
    pub fn check_symmetric(&self, pairs: &[(usize, usize)], tolerance: f32) -> bool {
        for &(a, b) in pairs {
            if a >= MAX_ACTUATORS || b >= MAX_ACTUATORS {
                continue;
            }
            let diff = (self.commanded[a].0 - self.commanded[b].0).abs();
            if diff > tolerance {
                return false;
            }
        }
        true
    }

    /// Count actuators with a specific health status
    pub fn count_by_health(&self, health: ActuatorHealth, active_mask: u16) -> usize {
        let mut count = 0;
        for i in 0..MAX_ACTUATORS {
            if (active_mask & (1 << i)) != 0 && self.health[i] == health {
                count += 1;
            }
        }
        count
    }
}

/// Tracks last-known-good actuator vectors per group
#[derive(Clone, Debug)]
pub struct ActuatorFallbackState {
    pub last_good: [GroupVector; MAX_GROUPS],
    pub age: [u16; MAX_GROUPS],
    /// Per-group consecutive fallback counter (for degradation triggering)
    pub consecutive_fallback: [u16; MAX_GROUPS],
}

impl Default for ActuatorFallbackState {
    fn default() -> Self {
        Self {
            last_good: [GroupVector::default(); MAX_GROUPS],
            age: [0; MAX_GROUPS],
            consecutive_fallback: [0; MAX_GROUPS],
        }
    }
}

/// Airframe configuration-mode descriptor for the actuator sanitizer:
/// the configuration mode (VTOL Hover/Cruise/Transition) and the
/// actuator groups it validates. Not a flight-mode/loop contract —
/// cascade loop selection is owned by
/// [`crate::control::VehicleControlMode`].
#[derive(Clone, Debug)]
pub struct ModeConfig {
    pub mode: ConfigMode,
    pub groups: &'static [ActuatorGroupConfig],
}

// Spec §10.3 Sanitization

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GroupSanitizeResult {
    AllValid,
    Clamped,
    FallbackLastGood,
    FallbackSafe,
    FallbackUnavailable,
}

#[derive(Clone, Debug)]
pub struct SanitizeReport {
    pub group_results: [GroupSanitizeResult; MAX_GROUPS],
    pub any_fallback: bool,
    pub critical_failure: bool,
    /// True when any group has exceeded MAX_CONSECUTIVE_FALLBACK frames
    pub consecutive_fallback_limit_exceeded: bool,
}

impl Default for SanitizeReport {
    fn default() -> Self {
        Self {
            group_results: [GroupSanitizeResult::AllValid; MAX_GROUPS],
            any_fallback: false,
            critical_failure: false,
            consecutive_fallback_limit_exceeded: false,
        }
    }
}

pub const MAX_FALLBACK_AGE_CYCLES: u16 = 100;

/// Maximum consecutive fallback cycles before triggering degradation.
/// Degradation triggers on frame (MAX_CONSECUTIVE_FALLBACK + 1).
pub const MAX_CONSECUTIVE_FALLBACK: u16 = 10;

/// Sanitizer trait — algorithm identity + per-cycle decision logic.
///
/// Phase 4: takes `&self` (algorithm config / unit) plus
/// `&mut fallback: &mut ActuatorFallbackState` for the
/// last-good / age / consecutive-fallback counters that persist
/// across cycles. The fallback state lives only in
/// `KernelState.fallback` — the sanitizer carries no per-cycle
/// state of its own.
// COV:EXCL_START(phantom DA: trait declaration + method signature
// param lines carry coverage attribution from rustc but have no
// executable code. Same artifact class as VehicleController.)
pub trait ActuatorSanitizer {
    /// 64-bit algorithm-identity constant, fixed at the impl site.
    /// See `Estimator::ALGORITHM_ID` for the contract — same scope
    /// (sanitizer-class identity) and same lockstep gating role.
    const ALGORITHM_ID: u64;

    fn sanitize(
        &self,
        cmd: &mut ActuatorCmd,
        mode: &ModeConfig,
        fallback: &mut ActuatorFallbackState,
    ) -> SanitizeReport;
}
// COV:EXCL_STOP

// COV:EXCL_START(phantom DA: rustc's coverage attribution places
// phantom DA entries on `Sanitizer`'s declaration / surrounding doc
// + module comments after Phase 4 made it a unit struct — same
// artifact class as the kernel_trait.rs DELEGATE block. No
// executable code on these lines.)
/// Group-aware actuator sanitizer (spec §10). Phase 4 stripped its
/// internal state field — fallback memory now lives in
/// `KernelState.fallback`. The sanitizer itself is a unit struct,
/// preserved as a name for the trait impl + future tuning fields.
#[derive(Default)]
pub struct Sanitizer;

// Impl block lives in mixer/sanitizer_impl.rs to keep this file under the
// 500-line per-.rs cap. No re-export here — rustc's coverage phantom-DA
// issue triggers on `pub use submodule::X`, not on method-carrying impl
// blocks split across files.
mod desaturate;
mod replicable;
mod sanitizer_impl;
// COV:EXCL_STOP
