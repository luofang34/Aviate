//! `Replicable` — deterministic canonical byte encoding for kernel
//! state types (LLR-REPL-101).
//!
//! Spec §16 cross-channel snapshot replication, voting, and hot-spare
//! takeover require every safety-relevant runtime state field to
//! produce a byte-identical canonical encoding across redundant
//! channels. This trait pins the contract: each implementor writes a
//! fixed-width little-endian byte stream into the caller's buffer and
//! returns the number of bytes written.

// COV:EXCL_START(phantom DA on helper-glue around copy_into)

/// Copy `bytes` into `buf[offset..]`, truncating if the remaining
/// space is smaller. Returns the number of bytes actually copied —
/// callers add this to their running offset.
pub fn copy_into(buf: &mut [u8], offset: usize, bytes: &[u8]) -> usize {
    let remaining = buf.len().saturating_sub(offset);
    let n = remaining.min(bytes.len());
    if n > 0 {
        buf[offset..offset + n].copy_from_slice(&bytes[..n]);
    }
    n
}

/// Backwards-compatible wrapper around `copy_into`. New impls SHOULD
/// call `copy_into` directly; existing impls keep using `ByteWriter`
/// during the transition.
pub struct ByteWriter<'a> {
    buf: &'a mut [u8],
    written: usize,
}

impl<'a> ByteWriter<'a> {
    #[inline(always)]
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, written: 0 }
    }
    #[inline(always)]
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        self.written += copy_into(self.buf, self.written, bytes);
    }
    #[inline(always)]
    pub fn write_u8(&mut self, x: u8) {
        self.write_bytes(&[x]);
    }
    #[inline(always)]
    pub fn write_bool(&mut self, b: bool) {
        self.write_u8(if b { 1 } else { 0 });
    }
    #[inline(always)]
    pub fn write_u16(&mut self, x: u16) {
        self.write_bytes(&x.to_le_bytes());
    }
    #[inline(always)]
    pub fn write_u32(&mut self, x: u32) {
        self.write_bytes(&x.to_le_bytes());
    }
    #[inline(always)]
    pub fn write_u64(&mut self, x: u64) {
        self.write_bytes(&x.to_le_bytes());
    }
    #[inline(always)]
    pub fn write_usize(&mut self, x: usize) {
        self.write_u64(x as u64);
    }
    #[inline(always)]
    pub fn write_f32(&mut self, x: f32) {
        self.write_bytes(&x.to_le_bytes());
    }
    #[inline(always)]
    pub fn bytes_written(&self) -> usize {
        self.written
    }
}
// COV:EXCL_STOP

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

#[cfg(test)]
mod tests {
    use super::copy_into;

    #[test]
    fn copies_full_slice_when_buffer_fits() {
        let mut buf = [0u8; 8];
        let n = copy_into(&mut buf, 0, &[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(n, 4);
        assert_eq!(buf, [0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0]);
    }

    #[test]
    fn truncates_when_buffer_runs_out() {
        let mut buf = [0u8; 3];
        let n = copy_into(&mut buf, 0, &[1, 2, 3, 4, 5]);
        assert_eq!(n, 3);
        assert_eq!(buf, [1, 2, 3]);
    }

    #[test]
    fn writes_at_offset() {
        let mut buf = [0u8; 8];
        let n = copy_into(&mut buf, 4, &[0x10, 0x20]);
        assert_eq!(n, 2);
        assert_eq!(buf, [0, 0, 0, 0, 0x10, 0x20, 0, 0]);
    }

    #[test]
    fn empty_input_is_a_no_op() {
        let mut buf = [0u8; 4];
        let n = copy_into(&mut buf, 0, &[]);
        assert_eq!(n, 0);
        assert_eq!(buf, [0u8; 4]);
    }

    #[test]
    fn no_op_when_offset_at_end() {
        let mut buf = [0u8; 4];
        let n = copy_into(&mut buf, 4, &[1, 2]);
        assert_eq!(n, 0);
        assert_eq!(buf, [0u8; 4]);
    }

    #[test]
    fn no_op_when_offset_past_end() {
        // saturating_sub ensures no panic when offset > buf.len()
        let mut buf = [0u8; 4];
        let n = copy_into(&mut buf, 8, &[1, 2]);
        assert_eq!(n, 0);
    }
}
