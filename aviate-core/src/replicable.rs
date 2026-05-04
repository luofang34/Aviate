//! `Replicable` — deterministic canonical byte encoding for kernel
//! state types (LLR-REPL-101).
//!
//! Spec §16 cross-channel snapshot replication, voting, and hot-spare
//! takeover require every safety-relevant runtime state field to
//! produce a byte-identical canonical encoding across redundant
//! channels. This trait pins the contract: each implementor writes a
//! fixed-width little-endian byte stream into the caller's buffer and
//! returns the number of bytes written.

mod byte_writer;
pub use byte_writer::ByteWriter;

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
