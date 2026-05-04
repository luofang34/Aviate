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
use crate::replicable::{ByteWriter, Replicable};

impl Replicable for GroupVector {
    // 16 × f32 outputs + u16 mask + bool valid = 64 + 2 + 1 = 67.
    const ENCODED_LEN: usize = MAX_ACTUATORS * 4 + 2 + 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = ByteWriter::new(buf);
        for n in &self.outputs {
            w.write_f32(n.0);
        }
        w.write_u16(self.mask);
        w.write_bool(self.valid);
        w.bytes_written()
    }
}

impl Replicable for ActuatorHealth {
    const ENCODED_LEN: usize = 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = ByteWriter::new(buf);
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
        w.write_u8(tag);
        w.bytes_written()
    }
}

impl Replicable for ActuatorState {
    // health (16 × 1 byte) + commanded (16 × 4 bytes) +
    // actual discriminant + payload (1 + 16*4) + timestamp (8 + 1)
    const ENCODED_LEN: usize = MAX_ACTUATORS + MAX_ACTUATORS * 4 + 1 + MAX_ACTUATORS * 4 + 9;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut written = 0usize;
        for h in &self.health {
            if written >= buf.len() {
                break;
            }
            written += h.encode_canonical(&mut buf[written..]);
        }
        let mut w = ByteWriter::new(&mut buf[written..]);
        for n in &self.commanded {
            w.write_f32(n.0);
        }
        // Option<[Normalized; 16]>: discriminant byte + payload (when
        // Some) or empty (when None). Fixed-width invariant requires
        // payload bytes to be present even on None — write zeros.
        match &self.actual {
            Some(arr) => {
                w.write_u8(1);
                for n in arr {
                    w.write_f32(n.0);
                }
            }
            None => {
                w.write_u8(0);
                for _ in 0..MAX_ACTUATORS {
                    w.write_f32(0.0);
                }
            }
        }
        // Timestamp: ticks (u64) + source (TimeSource enum tag).
        // Tag byte via match — adding a variant requires extending
        // this, which fails CI via the exhaustiveness check.
        w.write_u64(self.timestamp.ticks);
        let source_tag: u8 = match self.timestamp.source {
            crate::time::TimeSource::Internal => 0,
            crate::time::TimeSource::Gps => 1,
            crate::time::TimeSource::Ptp => 2,
        };
        w.write_u8(source_tag);
        written + w.bytes_written()
    }
}

impl Replicable for ActuatorFallbackState {
    // last_good (8 × GroupVector) + age (8 × u16) +
    // consecutive_fallback (8 × u16).
    const ENCODED_LEN: usize =
        MAX_GROUPS * GroupVector::ENCODED_LEN + MAX_GROUPS * 2 + MAX_GROUPS * 2;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut written = 0usize;
        for v in &self.last_good {
            if written >= buf.len() {
                break;
            }
            written += v.encode_canonical(&mut buf[written..]);
        }
        let mut w = ByteWriter::new(&mut buf[written..]);
        for &a in &self.age {
            w.write_u16(a);
        }
        for &c in &self.consecutive_fallback {
            w.write_u16(c);
        }
        written + w.bytes_written()
    }
}
