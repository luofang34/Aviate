//! `Replicable` — deterministic canonical byte encoding for kernel
//! state types (LLR-REPL-101).
//!
//! Spec §16 cross-channel snapshot replication, voting, and hot-spare
//! takeover require every safety-relevant runtime state field to
//! produce a byte-identical canonical encoding across redundant
//! channels. This trait pins the contract: each implementor writes a
//! fixed-width little-endian byte stream into the caller's buffer and
//! returns the number of bytes written. See the `Replicable` trait
//! and `ByteWriter` types in this module for the concrete encoding
//! rules and helpers; the trait doc enumerates per-shape encoding
//! conventions (floats, integers, bools, enums, options, arrays,
//! structs).
//!
//! Why byte-identical, not Hash: `core::hash::Hasher` is a one-way
//! digest, the bytes are not recoverable for a remote channel to
//! compare; lockstep needs the actual state image, not just a
//! fingerprint. Hashes are computed downstream over these bytes via
//! the same FNV-1a fold the `algorithm_identity_hash` and
//! `canonical_hash` functions use.
//!
//! Phantom-DA note: this module avoids `pub use submodule::Trait`
//! re-exports — see `aviate-core/src/lib.rs` for the rationale.

// COV:EXCL_START(phantom DA: doc-comment + struct-decl lines for
// ByteWriter pick up coverage attributions from grcov even though
// they have no executable code; same artifact class documented in
// `aviate-core/src/ekf.rs` and `aviate-core/src/kernel/config.rs`.)
/// Helper for `Replicable` impls: writes primitive fields into a
/// byte buffer with truncation tracking. Saturating: writes stop
/// silently when the buffer is exhausted, so callers can detect
/// undersized buffers via the returned byte count.
pub struct ByteWriter<'a> {
    buf: &'a mut [u8],
    written: usize,
}
// COV:EXCL_STOP

// COV:EXCL_START(grcov phantom DA: doc-comment lines and trivial
// one-line wrapper helpers in this impl block accumulate spurious
// DA entries that the byte-level unit tests in `byte_writer_tests`
// already exercise behaviorally. Excluding the impl block from
// line/branch coverage suppresses the false positives without
// hiding any real branch — every helper is a `write_bytes`
// thunk over a `to_le_bytes` projection, no decision logic.
// `write_bytes` itself contains the only real branch in this
// module and is covered by `write_bytes_after_full_buffer_no_ops`
// (n == 0 path) and `write_bytes_copies_full_slice_when_buffer_fits`
// (copy path) inside `byte_writer_tests`.)
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
// COV:EXCL_STOP

#[cfg(test)]
mod byte_writer_tests {
    // The public `Replicable` impls use a subset of the helpers
    // (write_f32 / write_bool / write_bytes for EkfState; nothing
    // for NoControllerState). The remaining helpers (write_u16 /
    // write_u32 / write_u64 / write_usize) are scaffold for future
    // KernelState fields that aren't replicable yet (FaultFlags
    // bitflags, TimingStats counters, etc.). Exercise each helper
    // directly so coverage tracks them.
    use super::ByteWriter;

    #[test]
    fn helpers_emit_correct_bytes() {
        let mut buf = [0u8; 32];
        let mut w = ByteWriter::new(&mut buf);
        w.write_u8(0xAB);
        w.write_bool(true);
        w.write_bool(false);
        w.write_u16(0x1234);
        w.write_u32(0xDEAD_BEEF);
        w.write_u64(0xCAFE_BABE_F00D_BAAD);
        w.write_usize(0x1122_3344);
        w.write_f32(1.5_f32);
        let n = w.bytes_written();
        assert_eq!(n, 1 + 1 + 1 + 2 + 4 + 8 + 8 + 4);
        // Spot-check little-endian layout:
        assert_eq!(buf[0], 0xAB);
        assert_eq!(buf[1], 1);
        assert_eq!(buf[2], 0);
        assert_eq!(&buf[3..5], &[0x34, 0x12]);
        assert_eq!(&buf[5..9], &[0xEF, 0xBE, 0xAD, 0xDE]);
        assert_eq!(
            &buf[9..17],
            &[0xAD, 0xBA, 0x0D, 0xF0, 0xBE, 0xBA, 0xFE, 0xCA]
        );
    }

    #[test]
    fn truncates_when_buffer_runs_out() {
        let mut buf = [0u8; 3];
        let mut w = ByteWriter::new(&mut buf);
        w.write_u32(0x1234_5678);
        // Only 3 bytes accepted; 4th byte dropped.
        assert_eq!(w.bytes_written(), 3);
        assert_eq!(buf, [0x78, 0x56, 0x34]);
    }

    #[test]
    fn empty_input_is_a_no_op() {
        let mut buf = [0u8; 4];
        let mut w = ByteWriter::new(&mut buf);
        w.write_bytes(&[]);
        assert_eq!(w.bytes_written(), 0);
    }

    #[test]
    fn write_bytes_copies_full_slice_when_buffer_fits() {
        // Direct exercise of the copy path inside `write_bytes`. The
        // integration `replicable_tests` reach `write_bytes` via the
        // typed helpers (write_f32 etc.); duplicating the call here
        // means grcov tracks the copy path within the lib's own
        // compilation unit even if cross-crate inlining would
        // otherwise hide it.
        let mut buf = [0u8; 8];
        let mut w = ByteWriter::new(&mut buf);
        w.write_bytes(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(w.bytes_written(), 4);
        assert_eq!(buf, [0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0]);
    }

    #[test]
    fn write_bytes_after_full_buffer_no_ops() {
        // First fill the buffer, then write more — exercises the
        // `n == 0` branch when `remaining == 0`.
        let mut buf = [0u8; 2];
        let mut w = ByteWriter::new(&mut buf);
        w.write_bytes(&[1, 2]);
        assert_eq!(w.bytes_written(), 2);
        w.write_bytes(&[3, 4]);
        assert_eq!(w.bytes_written(), 2, "no more bytes accepted past capacity");
        assert_eq!(buf, [1, 2]);
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
