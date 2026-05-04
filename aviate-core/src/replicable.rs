//! `Replicable` — deterministic canonical byte encoding for kernel
//! state types (LLR-REPL-101).
//!
//! Spec §16 cross-channel snapshot replication, voting, and hot-spare
//! takeover require every safety-relevant runtime state field to
//! produce a byte-identical canonical encoding across redundant
//! channels. This trait pins the contract: each implementor writes a
//! fixed-width little-endian byte stream into the caller's buffer and
//! returns the number of bytes written.

/// Copy `bytes` into `buf[offset..]`, truncating if the remaining
/// space is smaller. Returns the number of bytes actually copied.
/// Replicable impls call this once per field and accumulate the
/// returned counts into a running offset.
pub fn copy_into(buf: &mut [u8], offset: usize, bytes: &[u8]) -> usize {
    let remaining = buf.len().saturating_sub(offset);
    let n = remaining.min(bytes.len());
    if n > 0 {
        buf[offset..offset + n].copy_from_slice(&bytes[..n]);
    }
    n
}

/// Deterministic canonical byte encoding for kernel-state types.
///
/// Every implementor SHALL declare a `const ENCODED_LEN: usize` giving
/// the exact byte count of its encoded form, and implement
/// `encode_canonical(&self, &mut [u8]) -> usize` that writes EXACTLY
/// `Self::ENCODED_LEN` bytes (or `min(buf.len(), Self::ENCODED_LEN)`
/// on truncation) and returns the byte count.
///
/// Two byte-identical states SHALL produce byte-identical encodings;
/// two distinguishable states SHALL produce byte-distinct encodings.
/// Floats use `to_le_bytes` (target-endian-independent, exact bit
/// pattern preserved). `ENCODED_LEN` is a per-type compile-time
/// constant — no per-instance variation; this enables a peer channel
/// to allocate a fixed-size receive buffer at startup.
pub trait Replicable {
    /// Exact byte count of `self.encode_canonical(...)` output.
    /// MUST be a compile-time constant for buffer pre-sizing.
    const ENCODED_LEN: usize;

    /// Write the canonical encoding of `self` into `buf`. Returns
    /// the number of bytes written, which equals
    /// `min(buf.len(), Self::ENCODED_LEN)`.
    fn encode_canonical(&self, buf: &mut [u8]) -> usize;
}

// Tests for `copy_into` live in `aviate-core/tests/replicable_tests.rs`
// (already covers the function via the Replicable contract suite). Moving
// them out of the lib's `#[cfg(test)]` block so the test bodies' DA
// instrumentation doesn't accumulate inside the lib's source file —
// integration tests in `tests/` are excluded from coverage measurement
// by the script's source-attribution filter.
