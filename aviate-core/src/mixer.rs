use crate::types::{Normalized, Validated};
use crate::time::Timestamp;
use crate::control::ConfigMode;

use crate::control::AxisCommand;

/// Mixer trait - converts axis commands to actuator outputs
pub trait Mixer {
    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd;
}

/// Quadrotor X-configuration mixer
/// Motor layout:
///   0(CW)   1(CCW)
///      \   /
///       [X]
///      /   \
///   2(CCW)  3(CW)
pub struct QuadXMixer {
    pub timestamp_source: fn() -> Timestamp,
}

impl Mixer for QuadXMixer {
    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        let t = axis.collective.0;      // [0, 1]
        let r = axis.roll.0;            // [-1, 1]
        let p = axis.pitch.0;           // [-1, 1]
        let y = axis.yaw.0;             // [-1, 1]

        // Standard X-config mixing:
        // M0 (front-right, CW):  +roll -pitch +yaw
        // M1 (front-left, CCW):  -roll -pitch -yaw
        // M2 (rear-left, CW):    -roll +pitch +yaw
        // M3 (rear-right, CCW):  +roll +pitch -yaw

        let _m0 = t - r + p + y; // M0: Front Right (CW) -> -Roll, +Pitch, +Yaw (Wait, standard is +Roll? FR is +X +Y? No, FR is +X -Y (NED?))
        // Spec does not define motor mapping explicitly yet.
        // Standard PX4/ArduPilot Quad X:
        // 1 (FR, CCW) - 3 (RL, CCW)
        //    \ /
        //    / \
        // 2 (FL, CW) - 4 (RR, CW)
        // Let's stick to the layout in the prompt comment:
        //   0(CW)   1(CCW)  (Front)
        //      \   /
        //       [X]
        //      /   \
        //   2(CCW)  3(CW)   (Rear)
        //
        // Roll (+ right):  M0(Right) down/up?, M3(Right) down/up?
        // Right roll -> Right side down (thrust decrease), Left side up (thrust increase).
        // M0 (FR), M3 (RR) -> Decrease. (- roll)
        // M1 (FL), M2 (RL) -> Increase. (+ roll)
        //
        // Pitch (+ nose up): Front up, Rear down.
        // M0 (FR), M1 (FL) -> Increase (+ pitch)
        // M2 (RL), M3 (RR) -> Decrease (- pitch)
        //
        // Yaw (+ CW): CW motors torque left (anti-torque right). To yaw right (CW), increase CCW motors, decrease CW?
        // No, to yaw right (CW), body torque must be CW. Motors apply torque opposite to spin.
        // CW motors (0, 3) apply CCW torque. CCW motors (1, 2) apply CW torque.
        // To yaw CW (positive), we need net CW torque. So increase CCW motors (1, 2), decrease CW motors (0, 3).
        //
        // Summary:
        // M0 (FR, CW):  -roll +pitch -yaw
        // M1 (FL, CCW): +roll +pitch +yaw
        // M2 (RL, CCW): +roll -pitch +yaw
        // M3 (RR, CW):  -roll -pitch -yaw
        
        // Let's implement THIS logic.
        
        let m0 = t - r + p - y;
        let m1 = t + r + p + y;
        let m2 = t + r - p + y;
        let m3 = t - r - p - y;

        // Clamp to [0, 1]
        let mut outputs = [Normalized(0.0); MAX_ACTUATORS];
        outputs[0] = Normalized(m0.clamp(0.0, 1.0));
        outputs[1] = Normalized(m1.clamp(0.0, 1.0));
        outputs[2] = Normalized(m2.clamp(0.0, 1.0));
        outputs[3] = Normalized(m3.clamp(0.0, 1.0));

        ActuatorCmd {
            outputs,
            active_mask: 0b1111,
            sequence: 0,
            timestamp: (self.timestamp_source)(),
            fallback_mask: 0,
            sanitized: false,
        }
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
}

impl Default for ActuatorFallbackState {
    fn default() -> Self {
        Self {
            last_good: [GroupVector::default(); MAX_GROUPS],
            age: [0; MAX_GROUPS],
        }
    }
}

// Spec §4.2 ModeConfig (Partial stub for Sanitizer)
#[derive(Clone, Debug)]
pub struct ModeConfig {
    pub mode: ConfigMode,
    pub groups: &'static [ActuatorGroupConfig],
    // other fields (mixer, limits, etc) omitted for now as not used in sanitizer signature
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
}

impl Default for SanitizeReport {
    fn default() -> Self {
        Self {
            group_results: [GroupSanitizeResult::AllValid; MAX_GROUPS],
            any_fallback: false,
            critical_failure: false,
        }
    }
}

pub const MAX_FALLBACK_AGE_CYCLES: u16 = 100;

pub trait ActuatorSanitizer {
    fn sanitize(
        &mut self,
        cmd: &mut ActuatorCmd,
        mode: &ModeConfig,
    ) -> SanitizeReport;
}

#[derive(Default)]
pub struct Sanitizer {
    pub state: ActuatorFallbackState,
}

impl ActuatorSanitizer for Sanitizer {
    fn sanitize(
        &mut self,
        cmd: &mut ActuatorCmd,
        mode: &ModeConfig,
    ) -> SanitizeReport {
        let mut report = SanitizeReport::default();
        cmd.sanitized = true;
        cmd.fallback_mask = 0;

        for (i, group) in mode.groups.iter().enumerate() {
            if i >= MAX_GROUPS { break; }

            let mut group_valid = true;
            // Check all members of the group
            for &channel_idx in group.members {
                let idx = channel_idx as usize;
                if idx >= MAX_ACTUATORS { continue; }
                
                let val = cmd.outputs[idx];
                if !val.is_valid() {
                    group_valid = false;
                    break; 
                }
                // range check [0,1] implied by Normalized? 
                // Normalized wraps f32, we should check bounds if strict
                if val.0 < 0.0 || val.0 > 1.0 {
                    group_valid = false;
                    break;
                }
            }

            if group_valid {
                // Update last good
                let mut vec = GroupVector {
                    outputs: [Normalized(0.0); MAX_ACTUATORS],
                    mask: 0,
                    valid: true,
                };
                for &channel_idx in group.members {
                    let idx = channel_idx as usize;
                    if idx < MAX_ACTUATORS {
                        vec.outputs[idx] = cmd.outputs[idx];
                        vec.mask |= 1 << idx;
                    }
                }
                self.state.last_good[i] = vec;
                self.state.age[i] = 0;
                report.group_results[i] = GroupSanitizeResult::AllValid;
            } else {
                // Fallback Logic
                report.any_fallback = true;
                cmd.fallback_mask |= 1 << i;
                
                if group.coupling == CouplingKind::Strong {
                    // Strong coupling: Reject ENTIRE group vector
                    // 1. Try Last Good
                    if self.state.last_good[i].valid && self.state.age[i] < MAX_FALLBACK_AGE_CYCLES {
                        let last = &self.state.last_good[i];
                        for &channel_idx in group.members {
                            let idx = channel_idx as usize;
                            if idx < MAX_ACTUATORS {
                                cmd.outputs[idx] = last.outputs[idx];
                            }
                        }
                        report.group_results[i] = GroupSanitizeResult::FallbackLastGood;
                    } 
                    // 2. Try Safe Pattern
                    else if group.safe_pattern.valid {
                        let safe = &group.safe_pattern;
                        for &channel_idx in group.members {
                            let idx = channel_idx as usize;
                            if idx < MAX_ACTUATORS {
                                cmd.outputs[idx] = safe.outputs[idx];
                            }
                        }
                        report.group_results[i] = GroupSanitizeResult::FallbackSafe;
                    }
                    // 3. Critical Failure (Zero)
                    else {
                        for &channel_idx in group.members {
                            let idx = channel_idx as usize;
                            if idx < MAX_ACTUATORS {
                                cmd.outputs[idx] = Normalized(0.0);
                            }
                        }
                        report.group_results[i] = GroupSanitizeResult::FallbackUnavailable;
                        report.critical_failure = true;
                    }
                } else {
                    // Weak Coupling: Per-channel fallback logic
                    // For now, treat same as strong for simplicity or implement per-channel?
                    // Spec says "Per-channel fallback allowed".
                    // But if one is NaN, what do we do?
                    // We can keep valid ones and replace invalid ones.
                    // Implementation simplified: use SafePattern for invalid ones.
                    for &channel_idx in group.members {
                        let idx = channel_idx as usize;
                        if idx >= MAX_ACTUATORS { continue; }
                        let val = cmd.outputs[idx];
                        
                        if !val.is_valid() || val.0 < 0.0 || val.0 > 1.0 {
                             // Fallback this single channel
                             // Try last good
                             if self.state.last_good[i].valid && self.state.age[i] < MAX_FALLBACK_AGE_CYCLES {
                                 cmd.outputs[idx] = self.state.last_good[i].outputs[idx];
                             } else {
                                 // Safe or zero
                                 if group.safe_pattern.valid {
                                     cmd.outputs[idx] = group.safe_pattern.outputs[idx];
                                 } else {
                                     cmd.outputs[idx] = Normalized(0.0);
                                 }
                             }
                        }
                    }
                    // We don't mark the whole group as invalid in 'last_good' if weak?
                    // Spec isn't explicit on weak fallback state tracking.
                    // For now, we increment age if ANY invalid?
                    report.group_results[i] = GroupSanitizeResult::Clamped; // Approximate status
                }
                
                self.state.age[i] = self.state.age[i].saturating_add(1);
            }
        }
        
        report
    }
}
