//! `ActuatorSanitizer` implementation for `Sanitizer`.
//!
//! The sanitize method is the bulk of `mixer.rs` — pulling it into its
//! own file keeps the parent module under the 500-line cap. Only an
//! impl block lives here, with no re-exports, so rustc's coverage
//! instrumentation doesn't emit phantom DA entries against `mixer.rs`
//! line numbers (see the control.rs split for the background).

use super::{
    ActuatorCmd, ActuatorSanitizer, CouplingKind, GroupSanitizeResult, GroupVector, ModeConfig,
    SanitizeReport, Sanitizer, MAX_ACTUATORS, MAX_CONSECUTIVE_FALLBACK, MAX_FALLBACK_AGE_CYCLES,
    MAX_GROUPS,
};
use crate::types::{Normalized, Validated};

impl ActuatorSanitizer for Sanitizer {
    fn sanitize(&mut self, cmd: &mut ActuatorCmd, mode: &ModeConfig) -> SanitizeReport {
        let mut report = SanitizeReport::default();
        cmd.sanitized = true;
        cmd.fallback_mask = 0;

        for (i, group) in mode.groups.iter().enumerate() {
            if i >= MAX_GROUPS {
                break; // COV:EXCL(DEFENSIVE: bounds check)
            }

            let mut group_valid = true;
            // Check all members of the group
            for &channel_idx in group.members {
                let idx = channel_idx as usize;
                if idx >= MAX_ACTUATORS {
                    continue; // COV:EXCL(DEFENSIVE: bounds check)
                }

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
                    if self.state.last_good[i].valid && self.state.age[i] < MAX_FALLBACK_AGE_CYCLES
                    {
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
                        if idx >= MAX_ACTUATORS {
                            continue; // COV:EXCL(DEFENSIVE: bounds check)
                        }
                        let val = cmd.outputs[idx];

                        if !val.is_valid() || val.0 < 0.0 || val.0 > 1.0 {
                            // Fallback this single channel
                            // Try last good
                            if self.state.last_good[i].valid
                                && self.state.age[i] < MAX_FALLBACK_AGE_CYCLES
                            {
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

            // Consecutive fallback tracking (per-group)
            match report.group_results[i] {
                GroupSanitizeResult::AllValid | GroupSanitizeResult::Clamped => {
                    // Reset counter: we still have control authority
                    self.state.consecutive_fallback[i] = 0;
                }
                GroupSanitizeResult::FallbackLastGood
                | GroupSanitizeResult::FallbackSafe
                | GroupSanitizeResult::FallbackUnavailable => {
                    // Increment consecutive counter: lost authority
                    self.state.consecutive_fallback[i] =
                        self.state.consecutive_fallback[i].saturating_add(1);

                    // Check limit: trigger on frame (MAX + 1)
                    if self.state.consecutive_fallback[i] > MAX_CONSECUTIVE_FALLBACK {
                        report.consecutive_fallback_limit_exceeded = true;
                    }
                }
            }
        }

        report
    }
}
