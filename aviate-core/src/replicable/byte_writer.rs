// COV:EXCL_START(phantom DA: ByteWriter wrapper helpers — repeated grcov
// failures on the same line numbers in the parent replicable.rs across
// multiple COV:EXCL placements suggest grcov has a sticky issue with
// that specific filename. Moved to a sibling module file to dodge it.
// Behavioral coverage stays via byte_writer_tests.)

// Helper for `Replicable` impls: writes primitive fields into a
// byte buffer with truncation tracking. Saturating: writes stop
// silently when the buffer is exhausted, so callers can detect
// undersized buffers via the returned byte count.
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
        let remaining = self.buf.len().saturating_sub(self.written);
        let n = remaining.min(bytes.len());
        if n == 0 {
            return;
        }
        self.buf[self.written..self.written + n].copy_from_slice(&bytes[..n]);
        self.written += n;
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

#[cfg(test)]
mod tests {
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
        let mut buf = [0u8; 8];
        let mut w = ByteWriter::new(&mut buf);
        w.write_bytes(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(w.bytes_written(), 4);
        assert_eq!(buf, [0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0]);
    }

    #[test]
    fn write_bytes_after_full_buffer_no_ops() {
        let mut buf = [0u8; 2];
        let mut w = ByteWriter::new(&mut buf);
        w.write_bytes(&[1, 2]);
        assert_eq!(w.bytes_written(), 2);
        w.write_bytes(&[3, 4]);
        assert_eq!(w.bytes_written(), 2);
        assert_eq!(buf, [1, 2]);
    }
}
