//! Replicable impls for mixer types (ActuatorHealth /
//! GroupVector / ActuatorState / ActuatorFallbackState).
//!
//! Extracted from mixer.rs to keep that file under the
//! 500-line cap. The encoding contracts are documented
//! by LLR-REPL-101 / LLR-REPL-102; see the parent module
//! and `aviate-core/src/replicable.rs` for the trait shape.

use super::{
    ActuatorFallbackState, ActuatorHealth, ActuatorState, GroupVector, MAX_ACTUATORS, MAX_GROUPS,
};
use crate::replicable::{copy_into, Replicable};

impl Replicable for GroupVector {
    // 16 × f32 outputs + u16 mask + bool valid = 64 + 2 + 1 = 67.
    const ENCODED_LEN: usize = MAX_ACTUATORS * 4 + 2 + 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = 0usize;
        for n in &self.outputs {
            w += copy_into(buf, w, &n.0.to_le_bytes());
        }
        w += copy_into(buf, w, &self.mask.to_le_bytes());
        w += copy_into(buf, w, &[if self.valid { 1 } else { 0 }]);
        w
    }
}

impl Replicable for ActuatorHealth {
    const ENCODED_LEN: usize = 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        // Tag byte assigned in declaration order; adding a variant
        // SHALL extend this match (the exhaustiveness check fails
        // CI when this lags).
        let tag: u8 = match self {
            ActuatorHealth::Good => 0,
            ActuatorHealth::Degraded => 1,
            ActuatorHealth::Failed => 2,
            ActuatorHealth::Stuck => 3,
            ActuatorHealth::Unknown => 4,
        };
        copy_into(buf, 0, &[tag])
    }
}

impl Replicable for ActuatorState {
    // health (16 × 1 byte) + commanded (16 × 4 bytes) +
    // actual discriminant + payload (1 + 16*4) + timestamp (8 + 1)
    const ENCODED_LEN: usize = MAX_ACTUATORS + MAX_ACTUATORS * 4 + 1 + MAX_ACTUATORS * 4 + 9;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = 0usize;
        for h in &self.health {
            let off = w.min(buf.len());
            w += h.encode_canonical(&mut buf[off..]);
        }
        for n in &self.commanded {
            w += copy_into(buf, w, &n.0.to_le_bytes());
        }
        // Option<[Normalized; 16]>: discriminant byte + payload (when
        // Some) or empty (when None). Fixed-width invariant requires
        // payload bytes to be present even on None — write zeros.
        match &self.actual {
            Some(arr) => {
                w += copy_into(buf, w, &[1]);
                for n in arr {
                    w += copy_into(buf, w, &n.0.to_le_bytes());
                }
            }
            None => {
                w += copy_into(buf, w, &[0]);
                for _ in 0..MAX_ACTUATORS {
                    w += copy_into(buf, w, &0.0_f32.to_le_bytes());
                }
            }
        }
        // Timestamp: ticks (u64) + source (TimeSource enum tag).
        // Tag byte via match — adding a variant requires extending
        // this, which fails CI via the exhaustiveness check.
        w += copy_into(buf, w, &self.timestamp.ticks.to_le_bytes());
        let source_tag: u8 = match self.timestamp.source {
            crate::time::TimeSource::Internal => 0,
            crate::time::TimeSource::Gps => 1,
            crate::time::TimeSource::Ptp => 2,
        };
        w += copy_into(buf, w, &[source_tag]);
        w
    }
}

impl Replicable for ActuatorFallbackState {
    // last_good (8 × GroupVector) + age (8 × u16) +
    // consecutive_fallback (8 × u16).
    const ENCODED_LEN: usize =
        MAX_GROUPS * GroupVector::ENCODED_LEN + MAX_GROUPS * 2 + MAX_GROUPS * 2;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = 0usize;
        for v in &self.last_good {
            let off = w.min(buf.len());
            w += v.encode_canonical(&mut buf[off..]);
        }
        for &a in &self.age {
            w += copy_into(buf, w, &a.to_le_bytes());
        }
        for &c in &self.consecutive_fallback {
            w += copy_into(buf, w, &c.to_le_bytes());
        }
        w
    }
}
