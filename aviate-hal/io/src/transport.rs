//! Transport layer abstractions for frame-based communication
//!
//! Provides non-blocking, time-deterministic transport traits for:
//! - Telemetry output (FrameTx)
//! - Command input (FrameRx)
//!
//! Used by protocol layers (aviate-link) to send/receive complete frames
//! over various physical transports (USB CDC, UART, CAN, Ethernet, etc.)
//!
//! ## DO-178C Compliance Requirements
//!
//! **NON-BLOCKING GUARANTEE**: All operations MUST complete within a bounded,
//! provable time limit. This is critical for high-DAL (Design Assurance Level)
//! tasks where WCET (worst-case execution time) must be analyzable.
//!
//! **ERROR SEMANTICS**: All errors are recoverable and documented. No panics,
//! no unwraps, no hidden failures. Errors are classified by impact:
//! - `BufferFull`: Expected during overload, handled by dropping frames
//! - `Disconnected`: Link health issue, may trigger system alerts
//! - `InvalidFrame`: Protocol error, logged for debugging
//!
//! ## Architecture
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────┐
//! │  High-DAL Control Loop                                   │
//! │  - Deterministic timing                                  │
//! │  - Uses TelemetryFormatter (format only, no I/O)         │
//! │  - Pushes to bounded queue                               │
//! └──────────────────┬───────────────────────────────────────┘
//!                    │
//!                    ▼
//! ┌──────────────────────────────────────────────────────────┐
//! │  Low-DAL Telemetry Task                                  │
//! │  - Pops from queue                                       │
//! │  - Uses FrameTx::try_send (may fail, doesn't affect ctl) │
//! └──────────────────┬───────────────────────────────────────┘
//!                    │
//!                    ▼
//! ┌──────────────────────────────────────────────────────────┐
//! │  Board Transport Layer                                   │
//! │  - UsbCdcTx/Rx, UartTx/Rx, CanTx/Rx, EthernetTx/Rx       │
//! │  - Hardware buffers, DMA, interrupts                     │
//! └──────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage Example
//!
//! ```ignore
//! // High-DAL control loop: format only, no I/O
//! let mut buf = [0u8; 256];
//! let len = formatter.format_state(&state, time_ms, &mut buf)?;
//! telemetry_queue.push(&buf[..len])?;  // Bounded, non-blocking
//!
//! // Low-DAL telemetry task: send with I/O
//! if let Some(frame) = telemetry_queue.pop() {
//!     match transport.try_send(frame) {
//!         Ok(()) => { /* Success */ },
//!         Err(TransportError::BufferFull) => {
//!             // Expected during overload, drop frame and increment counter
//!             dropped_frames += 1;
//!         },
//!         Err(TransportError::Disconnected) => {
//!             // Link health issue, may trigger alert
//!             link_status = LinkStatus::Disconnected;
//!         },
//!         Err(e) => { /* Log other errors */ },
//!     }
//! }
//! ```

#![forbid(unsafe_code)]

use core::fmt;

/// Transport layer error
///
/// All errors are recoverable and have documented handling strategies.
/// DO NOT panic on transport errors - they are expected during normal operation!
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    /// Output buffer full, frame must be dropped
    ///
    /// **Handling**: Increment dropped frame counter, continue operation.
    /// This is EXPECTED during telemetry bursts or link congestion.
    ///
    /// **DO-178C Note**: This error does NOT affect control loop correctness.
    /// Telemetry is non-safety-critical (low DAL), drops are acceptable.
    BufferFull,

    /// Transport link disconnected or hardware failure
    ///
    /// **Handling**: Update link health status, may trigger system alert.
    /// Continue attempting to send (link may recover).
    ///
    /// **Examples**: USB cable unplugged, UART RX overflow, CAN bus-off
    Disconnected,

    /// Invalid frame format (malformed data, CRC error, etc.)
    ///
    /// **Handling**: Log error for debugging, discard frame.
    /// May indicate protocol mismatch or bit errors.
    ///
    /// **Only for FrameRx**: This error does not occur on FrameTx
    InvalidFrame,
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferFull => write!(f, "Transport buffer full (frame dropped)"),
            Self::Disconnected => write!(f, "Transport link disconnected"),
            Self::InvalidFrame => write!(f, "Invalid frame format or CRC error"),
        }
    }
}

/// Frame transmit transport (USB CDC, UART, CAN, Ethernet, etc.)
///
/// Used by telemetry and command acknowledgment layers to send protocol frames.
///
/// ## Contract (DO-178C Critical)
///
/// 1. **NON-BLOCKING**: `try_send()` MUST return immediately (< 100 CPU cycles typical).
///    NO waiting for hardware, NO busy loops, NO unbounded operations.
///
/// 2. **BOUNDED FAILURE**: If buffer is full, return `Err(BufferFull)` immediately.
///    Do NOT block waiting for space. Caller handles dropped frames.
///
/// 3. **NO PANIC**: All error conditions return `Result<_, TransportError>`.
///    Implementations MUST NOT panic, unwrap, or expect.
///
/// 4. **WCET ANALYZABLE**: Worst-case execution time must be provable and documented.
///    Typically: bounds check + memcpy + index increment.
///
/// ## Typical Implementation
///
/// ```ignore
/// pub struct UsbCdcTx {
///     ring_buf: [u8; 4096],
///     head: usize,
///     tail: usize,
/// }
///
/// impl FrameTx for UsbCdcTx {
///     fn try_send(&mut self, frame: &[u8]) -> Result<(), TransportError> {
///         let avail = self.ring_buf.len() - (self.head - self.tail);
///         if frame.len() > avail {
///             return Err(TransportError::BufferFull);  // Immediate return!
///         }
///
///         // Fast memcpy to ring buffer (bounded by frame.len())
///         // ...
///
///         Ok(())  // Hardware ISR will drain buffer asynchronously
///     }
/// }
/// ```
pub trait FrameTx {
    /// Attempt to send a complete protocol frame (non-blocking, bounded time)
    ///
    /// # Parameters
    ///
    /// - `frame`: Complete protocol frame (MAVLink, CCSDS, custom, etc.)
    ///   Maximum frame size is protocol-dependent (e.g., 280 bytes for MAVLink 2.0)
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Frame queued in hardware buffer (will be sent asynchronously)
    /// - `Err(BufferFull)`: No buffer space, frame DROPPED (increment counter)
    /// - `Err(Disconnected)`: Transport not ready (may recover later)
    ///
    /// # Timing Guarantee
    ///
    /// WCET: O(frame.len()) for memcpy, typically < 1 microsecond for 280-byte frame.
    /// NO system calls, NO hardware waits, NO unbounded loops.
    ///
    /// # Usage in Low-DAL Telemetry Task
    ///
    /// ```ignore
    /// let mut dropped = 0;
    /// match tx.try_send(frame) {
    ///     Ok(()) => {},  // Success
    ///     Err(TransportError::BufferFull) => dropped += 1,  // Expected
    ///     Err(e) => log::warn!("Transport error: {}", e),
    /// }
    /// ```
    fn try_send(&mut self, frame: &[u8]) -> Result<(), TransportError>;
}

/// Frame receive transport (USB CDC, UART, CAN, Ethernet, etc.)
///
/// Used by command and telemetry-acknowledgment layers to receive protocol frames.
///
/// ## Contract (DO-178C Critical)
///
/// 1. **NON-BLOCKING**: `try_recv()` MUST return immediately if no frame available.
///    Returns `Ok(0)` for "no data", NOT an error!
///
/// 2. **BOUNDED TIME**: Complete frame parsing in bounded time.
///    If frame incomplete, return `Ok(0)` and wait for more data.
///
/// 3. **NO PANIC**: All error conditions return `Result<usize, TransportError>`.
///    Implementations MUST NOT panic, unwrap, or expect.
///
/// 4. **WCET ANALYZABLE**: Worst-case execution time must be provable.
///    Typically: ring buffer read + CRC check (bounded by max frame size).
///
/// ## Typical Implementation
///
/// ```ignore
/// pub struct UsbCdcRx {
///     ring_buf: [u8; 4096],
///     parser_state: ParserState,
/// }
///
/// impl FrameRx for UsbCdcRx {
///     fn try_recv(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
///         // Parse ring buffer for complete frame
///         match self.parser_state.parse(&self.ring_buf) {
///             Some((frame, len)) if frame.crc_valid() => {
///                 buf[..len].copy_from_slice(frame);
///                 Ok(len)
///             },
///             Some((_, _)) => Err(TransportError::InvalidFrame),  // CRC bad
///             None => Ok(0),  // No complete frame yet (NOT an error!)
///         }
///     }
/// }
/// ```
pub trait FrameRx {
    /// Attempt to receive a complete protocol frame (non-blocking, bounded time)
    ///
    /// # Parameters
    ///
    /// - `buf`: Buffer to receive frame data (must be ≥ max frame size)
    ///
    /// # Returns
    ///
    /// - `Ok(len > 0)`: Complete frame received, `len` bytes written to `buf`
    /// - `Ok(0)`: No complete frame available yet (NOT an error, try again later)
    /// - `Err(InvalidFrame)`: Frame received but malformed (CRC error, length error, etc.)
    /// - `Err(Disconnected)`: Transport not ready
    ///
    /// # Timing Guarantee
    ///
    /// WCET: O(max_frame_size) for parse + CRC check, typically < 10 microseconds.
    /// NO system calls, NO hardware waits, NO unbounded loops.
    ///
    /// # Usage in Command Task
    ///
    /// ```ignore
    /// let mut buf = [0u8; 512];
    /// match rx.try_recv(&mut buf) {
    ///     Ok(0) => {},  // No frame yet (normal!)
    ///     Ok(len) => {
    ///         // Process complete frame: &buf[..len]
    ///         let cmd = parse_command(&buf[..len])?;
    ///         gateway.verify_and_execute(cmd)?;
    ///     },
    ///     Err(TransportError::InvalidFrame) => {
    ///         invalid_frames += 1;  // Log for debugging
    ///     },
    ///     Err(e) => log::warn!("Transport error: {}", e),
    /// }
    /// ```
    fn try_recv(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;
}
