use crate::types::{Normalized, Validated};
use crate::time::Timestamp;
use crate::control::ConfigMode;

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

#[derive(Clone, Debug)]
pub struct ActuatorState {
    pub feedback: [Normalized; MAX_ACTUATORS],
    pub timestamp: Timestamp,
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

pub struct Sanitizer {
    pub state: ActuatorFallbackState,
}

impl Default for Sanitizer {
    fn default() -> Self {
        Self {
            state: ActuatorFallbackState::default(),
        }
    }
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
