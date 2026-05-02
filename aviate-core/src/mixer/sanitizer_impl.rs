//! `ActuatorSanitizer` implementation for `Sanitizer`.
//!
//! Phase 4: the sanitize method takes `&self` (algorithm) plus
//! `&mut fallback` (`KernelState.fallback`'s persistent counters).
//! The sanitizer holds no state of its own.
//!
//! Pulling the body into its own file keeps the parent `mixer.rs`
//! under the 500-line cap. Only an impl block lives here, with no
//! re-exports, so rustc's coverage instrumentation doesn't emit
//! phantom DA entries against `mixer.rs` line numbers.

use super::{
    ActuatorCmd, ActuatorFallbackState, ActuatorSanitizer, CouplingKind, GroupSanitizeResult,
    GroupVector, ModeConfig, SanitizeReport, Sanitizer, MAX_ACTUATORS, MAX_CONSECUTIVE_FALLBACK,
    MAX_FALLBACK_AGE_CYCLES, MAX_GROUPS,
};
use crate::types::{Normalized, Validated};

impl ActuatorSanitizer for Sanitizer {
    fn sanitize(
        &self,
        cmd: &mut ActuatorCmd,
        mode: &ModeConfig,
        fallback: &mut ActuatorFallbackState,
    ) -> SanitizeReport {
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
                fallback.last_good[i] = vec;
                fallback.age[i] = 0;
                report.group_results[i] = GroupSanitizeResult::AllValid;
            } else {
                // Fallback Logic
                report.any_fallback = true;
                cmd.fallback_mask |= 1 << i;

                if group.coupling == CouplingKind::Strong {
                    // Strong coupling: Reject ENTIRE group vector
                    if fallback.last_good[i].valid && fallback.age[i] < MAX_FALLBACK_AGE_CYCLES {
                        let last = &fallback.last_good[i];
                        for &channel_idx in group.members {
                            let idx = channel_idx as usize;
                            if idx < MAX_ACTUATORS {
                                cmd.outputs[idx] = last.outputs[idx];
                            }
                        }
                        report.group_results[i] = GroupSanitizeResult::FallbackLastGood;
                    } else if group.safe_pattern.valid {
                        let safe = &group.safe_pattern;
                        for &channel_idx in group.members {
                            let idx = channel_idx as usize;
                            if idx < MAX_ACTUATORS {
                                cmd.outputs[idx] = safe.outputs[idx];
                            }
                        }
                        report.group_results[i] = GroupSanitizeResult::FallbackSafe;
                    } else {
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
                    for &channel_idx in group.members {
                        let idx = channel_idx as usize;
                        if idx >= MAX_ACTUATORS {
                            continue; // COV:EXCL(DEFENSIVE: bounds check)
                        }
                        let val = cmd.outputs[idx];

                        if !val.is_valid() || val.0 < 0.0 || val.0 > 1.0 {
                            if fallback.last_good[i].valid
                                && fallback.age[i] < MAX_FALLBACK_AGE_CYCLES
                            {
                                cmd.outputs[idx] = fallback.last_good[i].outputs[idx];
                            } else if group.safe_pattern.valid {
                                cmd.outputs[idx] = group.safe_pattern.outputs[idx];
                            } else {
                                cmd.outputs[idx] = Normalized(0.0);
                            }
                        }
                    }
                    report.group_results[i] = GroupSanitizeResult::Clamped;
                }

                fallback.age[i] = fallback.age[i].saturating_add(1);
            }

            // Consecutive fallback tracking (per-group)
            match report.group_results[i] {
                GroupSanitizeResult::AllValid | GroupSanitizeResult::Clamped => {
                    fallback.consecutive_fallback[i] = 0;
                }
                GroupSanitizeResult::FallbackLastGood
                | GroupSanitizeResult::FallbackSafe
                | GroupSanitizeResult::FallbackUnavailable => {
                    fallback.consecutive_fallback[i] =
                        fallback.consecutive_fallback[i].saturating_add(1);

                    if fallback.consecutive_fallback[i] > MAX_CONSECUTIVE_FALLBACK {
                        report.consecutive_fallback_limit_exceeded = true;
                    }
                }
            }
        }

        report
    }
}
