//! `Replicable` — deterministic canonical byte encoding for kernel
//! state types (LLR-REPL-101).
//!
//! Spec §16 cross-channel snapshot replication, voting, and hot-spare
//! takeover require every safety-relevant runtime state field to
//! produce a byte-identical canonical encoding across redundant
//! channels. This trait pins the contract: each implementor writes a
//! fixed-width little-endian byte stream into the caller's buffer and
//! returns the number of bytes written.
//!
//! Why "byte-identical, not Hash":
//! - `core::hash::Hasher` is a one-way digest; the bytes are not
//!   recoverable for a remote channel to compare. Lockstep needs the
//!   actual state image, not just a fingerprint.
//! - `core::hash::Hasher` permits implementations to differ in
//!   per-call ordering; deterministic encoding bans that.
//! - Cross-channel transmission needs serialized bytes; the hash is
//!   computed over them downstream (via the same FNV-1a fold the
//!   `algorithm_identity_hash` and `canonical_hash` functions use).
//!
//! Encoding rules:
//!
//!   - **Floats**: `f32::to_le_bytes` / `f64::to_le_bytes`. Target-
//!     endian-independent, exact bits preserved (NaN bit patterns
//!     too — relevant for fault-latch states).
//!   - **Integers**: little-endian via `to_le_bytes`. `usize` is
//!     widened to `u64` to hash identically on 32-bit and 64-bit
//!     targets.
//!   - **Bools**: a single byte, `0` or `1`.
//!   - **Enums**: a tag byte assigned in declaration order; payload
//!     follows for variants that carry data.
//!   - **Options**: a discriminant byte (0 = None, 1 = Some) plus
//!     payload on Some.
//!   - **Arrays / slices**: each element in order, no length prefix
//!     (length is part of the type at compile time for arrays; slice
//!     fields fold a length prefix as their owner's responsibility).
//!   - **Structs**: each field in declaration order, no separator
//!     bytes between fields (fixed-width per type means concatenation
//!     aliasing is prevented by the type-level shape, not by sentinels).
//!
//! Fixed-width invariant: every implementation writes EXACTLY
//! `ENCODED_LEN` bytes, regardless of state value. A variable-length
//! encoding (e.g. shrinking when the EKF is uninitialized) would
//! defeat byte-equality comparison.
//!
//! Phantom-DA note: this module avoids `pub use submodule::Trait`
//! re-exports — see `aviate-core/src/lib.rs` for the rationale.

/// Helper for `Replicable` impls: writes primitive fields into a
/// byte buffer with truncation tracking. Saturating: writes stop
/// silently when the buffer is exhausted, so callers can detect
/// undersized buffers via the returned byte count.
pub struct ByteWriter<'a> {
    buf: &'a mut [u8],
    written: usize,
}

impl<'a> ByteWriter<'a> {
    /// Wrap a destination buffer.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, written: 0 }
    }

    /// Append `bytes`, truncating if the buffer is too small.
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        let remaining = self.buf.len().saturating_sub(self.written);
        let n = remaining.min(bytes.len());
        if n == 0 {
            return;
        }
        self.buf[self.written..self.written + n].copy_from_slice(&bytes[..n]);
        self.written += n;
    }

    /// Append one byte.
    pub fn write_u8(&mut self, x: u8) {
        self.write_bytes(&[x]);
    }

    /// Append a `bool` as one byte (1 = true, 0 = false).
    pub fn write_bool(&mut self, b: bool) {
        self.write_u8(if b { 1 } else { 0 });
    }

    /// Append a `u16` in little-endian order.
    pub fn write_u16(&mut self, x: u16) {
        self.write_bytes(&x.to_le_bytes());
    }

    /// Append a `u32` in little-endian order.
    pub fn write_u32(&mut self, x: u32) {
        self.write_bytes(&x.to_le_bytes());
    }

    /// Append a `u64` in little-endian order.
    pub fn write_u64(&mut self, x: u64) {
        self.write_bytes(&x.to_le_bytes());
    }

    /// Append a `usize` widened to `u64` for cross-target stability.
    pub fn write_usize(&mut self, x: usize) {
        self.write_u64(x as u64);
    }

    /// Append an `f32` in little-endian order, exact bit pattern
    /// preserved.
    pub fn write_f32(&mut self, x: f32) {
        self.write_bytes(&x.to_le_bytes());
    }

    /// Number of bytes written so far.
    pub fn bytes_written(&self) -> usize {
        self.written
    }
}

/// Deterministic canonical byte encoding for kernel-state types.
///
/// Every implementor SHALL:
///
///   1. Declare a `const ENCODED_LEN: usize` giving the exact byte
///      count of its encoded form.
///   2. Implement `fn encode_canonical(&self, buf: &mut [u8]) -> usize`
///      that writes EXACTLY `Self::ENCODED_LEN` bytes starting at
///      offset 0 of `buf` and returns `Self::ENCODED_LEN`. The
///      caller is responsible for providing a buffer of at least
///      `ENCODED_LEN` bytes; impls SHALL panic-free truncate (write
///      `min(buf.len(), ENCODED_LEN)`) and return the actual count
///      so a too-small buffer fails the byte-equality check at the
///      lockstep boundary rather than corrupting memory.
///
/// Two byte-identical states SHALL produce byte-identical
/// encodings; two distinguishable states SHALL produce
/// byte-distinct encodings. Floating-point fields preserve exact
/// bit patterns (no canonicalization of NaN payloads or signed
/// zero), so a divergent fault latch in one channel becomes
/// observable to its peer.
///
/// `ENCODED_LEN` is a per-type compile-time constant — no
/// per-instance variation. This enables a peer channel to allocate
/// a fixed-size receive buffer at startup.
pub trait Replicable {
    /// Exact byte count of `self.encode_canonical(...)` output.
    /// MUST be a compile-time constant for buffer pre-sizing.
    const ENCODED_LEN: usize;

    /// Write the canonical encoding of `self` into `buf`. Returns
    /// the number of bytes written, which equals
    /// `min(buf.len(), Self::ENCODED_LEN)`.
    fn encode_canonical(&self, buf: &mut [u8]) -> usize;
}
