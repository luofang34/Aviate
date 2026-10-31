//! Fixed-capacity stack byte ring buffer for `no_std` MAVLink streams.

/// Bounded FIFO byte ring stored inline without heap allocation.
#[derive(Debug, Clone)]
pub struct DirectByteRing<const N: usize> {
    buf: [u8; N],
    head: usize,
    len: usize,
}

impl<const N: usize> Default for DirectByteRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> DirectByteRing<N> {
    /// Construct an empty [`DirectByteRing`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            head: 0,
            len: 0,
        }
    }

    /// Return the total byte capacity `N`.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Return the number of readable bytes currently stored.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Return `true` when the ring holds zero bytes.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Return the number of free bytes available for [`Self::push`].
    #[must_use]
    pub const fn remaining_capacity(&self) -> usize {
        N.saturating_sub(self.len)
    }

    /// Discard all buffered bytes.
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    /// Append as many prefix bytes of `data` as fit and return the byte count written.
    pub fn push(&mut self, data: &[u8]) -> usize {
        if N == 0 {
            return 0;
        }
        let writable = data.len().min(self.remaining_capacity());
        for &byte in data.iter().take(writable) {
            let tail = (self.head.wrapping_add(self.len)) % N;
            if let Some(slot) = self.buf.get_mut(tail) {
                *slot = byte;
            }
            self.len = self.len.wrapping_add(1);
        }
        writable
    }

    /// Append `data` atomically only when the entire slice fits in free space.
    pub fn push_all(&mut self, data: &[u8]) -> bool {
        if data.len() > self.remaining_capacity() {
            return false;
        }
        let _ = self.push(data);
        true
    }

    /// Copy up to `out.len()` bytes without removing them from the ring.
    pub fn peek(&self, out: &mut [u8]) -> usize {
        if N == 0 {
            return 0;
        }
        let readable = out.len().min(self.len);
        for (offset, dst) in out.iter_mut().take(readable).enumerate() {
            let idx = (self.head.wrapping_add(offset)) % N;
            if let Some(&byte) = self.buf.get(idx) {
                *dst = byte;
            }
        }
        readable
    }

    /// Advance the read cursor by up to `count` bytes and return the count removed.
    pub fn consume(&mut self, count: usize) -> usize {
        if N == 0 {
            return 0;
        }
        let removed = count.min(self.len);
        self.head = (self.head.wrapping_add(removed)) % N;
        self.len = self.len.saturating_sub(removed);
        if self.len == 0 {
            self.head = 0;
        }
        removed
    }

    /// Read and remove up to `out.len()` bytes from the front of the ring.
    pub fn pop(&mut self, out: &mut [u8]) -> usize {
        let count = self.peek(out);
        let _ = self.consume(count);
        count
    }
}
