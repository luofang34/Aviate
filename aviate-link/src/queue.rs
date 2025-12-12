//! Time-deterministic telemetry frame queue for DO-178C compliance
//!
//! This module provides a fixed-size, no-allocation ring buffer for telemetry frames.
//! It's designed for high-DAL control code that requires provable WCET.
//!
//! ## DO-178C Properties
//!
//! - **push()**: O(1), bounded cycles (~50 CPU cycles @ 480 MHz), non-blocking
//! - **pop_with()**: O(1), bounded cycles (~30 CPU cycles @ 480 MHz), non-blocking
//! - **Memory**: Statically allocated, no fragmentation, no dynamic allocation
//! - **Failure mode**: Explicit `QueueError::Full`, high-DAL code can count drops
//!
//! ## WCET Analysis (Engineering Targets)
//!
//! These are engineering targets, subject to validation:
//!
//! - `push()`: ~50 CPU cycles @ 480 MHz (memcpy + index arithmetic)
//! - `pop_with()`: ~30 CPU cycles @ 480 MHz (index arithmetic + callback)
//!
//! Actual WCET depends on:
//! - Compiler optimization level (release vs debug)
//! - CPU cache behavior (data cache hit/miss)
//! - Frame size (memcpy scales with frame length)
//!
//! ## Usage Example
//!
//! ```ignore
//! // In application context:
//! static mut TELEM_QUEUE: TelemetryQueue<16, 280> = TelemetryQueue::new();
//!
//! // High-DAL control task (provable WCET, no I/O):
//! pub fn control_task(ctx: &mut AppContext) {
//!     let mut buf = [0u8; 256];
//!     if let Ok(len) = format_attitude(&state, time_ms, sys_id, comp_id, &mut seq, &mut buf) {
//!         let _ = ctx.telemetry_queue.push(&buf[..len]);  // O(1), non-blocking
//!     }
//! }
//!
//! // Low-DAL telemetry task (can fail, doesn't affect control):
//! pub fn telemetry_task(ctx: &mut AppContext, transport: &mut impl FrameTx) {
//!     while ctx.telemetry_queue.pop_with(|frame| {
//!         let _ = transport.try_send(frame);  // Failure OK
//!     }) {}
//! }
//! ```

/// Queue operation errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueError {
    /// Queue is full, cannot push more frames
    ///
    /// High-DAL code should count drops and continue.
    /// This is an expected failure mode, not a critical error.
    Full,
}

/// Fixed-size, no-allocation telemetry frame queue
///
/// ## Type Parameters
///
/// - `N`: Queue capacity (number of frames)
/// - `FRAME_SIZE`: Maximum frame size in bytes (e.g., 280 for MAVLink v2)
///
/// ## Memory Footprint
///
/// Total memory: `N * FRAME_SIZE + N * 2 + 5` bytes
///
/// Example: TelemetryQueue<16, 280> = 16*280 + 16*2 + 5 = 4517 bytes (~4.4 KB)
///
/// ## Const Constructible
///
/// Can be used in static/const contexts for zero-cost initialization:
///
/// ```ignore
/// static mut TELEM_QUEUE: TelemetryQueue<16, 280> = TelemetryQueue::new();
/// ```
pub struct TelemetryQueue<const N: usize, const FRAME_SIZE: usize> {
    /// Frame storage (ring buffer of fixed-size frames)
    frames: [[u8; FRAME_SIZE]; N],
    /// Actual frame lengths (valid range: 0..=FRAME_SIZE)
    lens: [u16; N],
    /// Write pointer (next slot to write)
    head: usize,
    /// Read pointer (next slot to read)
    tail: usize,
    /// Full flag (distinguishes full from empty when head == tail)
    full: bool,
}

impl<const N: usize, const FRAME_SIZE: usize> TelemetryQueue<N, FRAME_SIZE> {
    /// Create a new empty queue
    ///
    /// This is a const fn, can be used in static/const contexts.
    ///
    /// ## DO-178C Contract
    ///
    /// - Non-blocking: YES (pure const initialization)
    /// - WCET: O(1), compile-time constant
    /// - Errors: None (infallible)
    pub const fn new() -> Self {
        Self {
            frames: [[0u8; FRAME_SIZE]; N],
            lens: [0; N],
            head: 0,
            tail: 0,
            full: false,
        }
    }

    /// Check if queue is empty
    ///
    /// ## DO-178C Contract
    ///
    /// - Non-blocking: YES
    /// - WCET: O(1), ~5 CPU cycles
    #[inline]
    pub fn is_empty(&self) -> bool {
        (!self.full) && (self.head == self.tail)
    }

    /// Check if queue is full
    ///
    /// ## DO-178C Contract
    ///
    /// - Non-blocking: YES
    /// - WCET: O(1), ~5 CPU cycles
    #[inline]
    pub fn is_full(&self) -> bool {
        self.full
    }

    /// Push a frame into the queue (non-blocking)
    ///
    /// ## Parameters
    ///
    /// - `frame`: Frame bytes to enqueue (length must be ≤ FRAME_SIZE)
    ///
    /// ## Returns
    ///
    /// - `Ok(())`: Frame successfully enqueued
    /// - `Err(QueueError::Full)`: Queue is full, frame dropped
    ///   - Also returned if frame.len() > FRAME_SIZE
    ///
    /// ## DO-178C Contract
    ///
    /// - Non-blocking: YES (no busy-wait, no interrupt wait)
    /// - Time complexity: O(frame.len()), bounded by memcpy
    /// - WCET (engineering target): ~50 CPU cycles + memcpy time @ 480 MHz
    ///   - For 280-byte frame: ~50 + 280 = ~330 cycles = ~0.7 μs @ 480 MHz
    /// - Memory: No heap allocation, uses statically allocated buffer
    ///
    /// ## Error Handling (DO-178C)
    ///
    /// High-DAL code MUST handle `QueueError::Full` gracefully:
    /// - Increment drop counter
    /// - Continue control loop (do NOT panic or halt)
    /// - Alert operator if drop rate exceeds threshold
    pub fn push(&mut self, frame: &[u8]) -> Result<(), QueueError> {
        // Check capacity and frame size
        if self.full || frame.len() > FRAME_SIZE {
            return Err(QueueError::Full);
        }

        let idx = self.head;

        // Copy frame into ring buffer
        self.frames[idx][..frame.len()].copy_from_slice(frame);
        self.lens[idx] = frame.len() as u16;

        // Advance head pointer (wrap around)
        self.head = (self.head + 1) % N;

        // Update full flag
        if self.head == self.tail {
            self.full = true;
        }

        Ok(())
    }

    /// Pop a frame from the queue and call callback with frame bytes
    ///
    /// ## Parameters
    ///
    /// - `f`: Callback function that receives frame bytes
    ///
    /// ## Returns
    ///
    /// - `true`: A frame was popped and callback was called
    /// - `false`: Queue is empty, callback was not called
    ///
    /// ## DO-178C Contract
    ///
    /// - Non-blocking: YES (callback must also be non-blocking!)
    /// - Time complexity: O(1) + callback time
    /// - WCET (engineering target): ~30 CPU cycles + callback time @ 480 MHz
    /// - Memory: No heap allocation
    ///
    /// ## Callback Requirements (DO-178C)
    ///
    /// The callback `f` MUST:
    /// - Be non-blocking (no busy-wait, no interrupt wait)
    /// - Have bounded execution time
    /// - Not panic or cause undefined behavior
    ///
    /// Example callback: `|frame| { let _ = transport.try_send(frame); }`
    pub fn pop_with<F: FnOnce(&[u8])>(&mut self, f: F) -> bool {
        if self.is_empty() {
            return false;
        }

        let idx = self.tail;
        let len = self.lens[idx] as usize;

        // Call callback with frame bytes
        f(&self.frames[idx][..len]);

        // Advance tail pointer (wrap around)
        self.tail = (self.tail + 1) % N;

        // Clear full flag
        self.full = false;

        true
    }

    /// Get current number of frames in queue
    ///
    /// ## DO-178C Contract
    ///
    /// - Non-blocking: YES
    /// - WCET: O(1), ~10 CPU cycles
    #[inline]
    pub fn len(&self) -> usize {
        if self.full {
            N
        } else if self.head >= self.tail {
            self.head - self.tail
        } else {
            N - (self.tail - self.head)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_queue_basic_operations() {
        let mut queue = TelemetryQueue::<4, 16>::new();

        assert!(queue.is_empty());
        assert!(!queue.is_full());
        assert_eq!(queue.len(), 0);

        // Push one frame
        let frame1 = b"test_frame_1";
        assert!(queue.push(frame1).is_ok());
        assert_eq!(queue.len(), 1);
        assert!(!queue.is_empty());

        // Pop one frame
        let mut popped = [0u8; 16];
        let mut popped_len = 0;
        assert!(queue.pop_with(|f| {
            popped[..f.len()].copy_from_slice(f);
            popped_len = f.len();
        }));
        assert_eq!(&popped[..popped_len], frame1);
        assert!(queue.is_empty());
    }

    #[test]
    fn test_queue_full() {
        let mut queue = TelemetryQueue::<2, 16>::new();

        // Fill queue
        assert!(queue.push(b"frame1").is_ok());
        assert!(queue.push(b"frame2").is_ok());
        assert!(queue.is_full());

        // Try to push when full
        assert!(matches!(queue.push(b"frame3"), Err(QueueError::Full)));
    }

    #[test]
    fn test_queue_wraparound() {
        let mut queue = TelemetryQueue::<2, 16>::new();

        // Push, pop, push, pop (wraparound)
        assert!(queue.push(b"A").is_ok());
        let mut buf = [0u8; 16];
        let mut len = 0;
        assert!(queue.pop_with(|f| {
            buf[..f.len()].copy_from_slice(f);
            len = f.len();
        }));
        assert_eq!(&buf[..len], b"A");

        assert!(queue.push(b"B").is_ok());
        assert!(queue.pop_with(|f| {
            buf[..f.len()].copy_from_slice(f);
            len = f.len();
        }));
        assert_eq!(&buf[..len], b"B");
    }

    #[test]
    fn test_frame_too_large() {
        let mut queue = TelemetryQueue::<4, 8>::new();

        // Try to push frame larger than FRAME_SIZE
        let large_frame = [0u8; 16];
        assert!(matches!(queue.push(&large_frame), Err(QueueError::Full)));
    }
}
