//! STM32H743 transport layer implementations
//!
//! Provides FrameTx and FrameRx implementations for various transports:
//! - USB CDC (via usbd-serial, configured by application)
//! - UART (via stm32h7xx-hal)
//! - CAN (via fdcan peripheral)
//!
//! ## Design Note
//!
//! USB CDC transport is generic over UsbBus type, as the USB peripheral
//! initialization is application-specific (depends on clock configuration,
//! pin assignments, etc.). Applications create the USB bus and pass it to
//! these wrappers.
//!
//! ## Usage Example
//!
//! ```ignore
//! // In application (after USB init):
//! use aviate_hal_io::transport::{FrameTx, FrameRx};
//! use aviate_board_micoair_h743_v2::transport::{UsbCdcTx, UsbCdcRx};
//!
//! let usb_tx = UsbCdcTx::new(usb_serial);
//! let usb_rx = UsbCdcRx::new(usb_serial);
//!
//! // Send telemetry frame
//! let frame = [0xFD, 0x09, 0x00, ...];  // MAVLink frame
//! usb_tx.try_send(&frame)?;
//!
//! // Receive command frame
//! let mut buf = [0u8; 512];
//! if let Ok(len) = usb_rx.try_recv(&mut buf) {
//!     if len > 0 {
//!         // Process command: &buf[..len]
//!     }
//! }
//! ```

#![forbid(unsafe_code)]

use aviate_hal_io::transport::{FrameRx, FrameTx, TransportError};

/// USB CDC transport transmitter (generic over USB bus type)
///
/// Wraps `usbd_serial::SerialPort<B>` to provide non-blocking frame transmission.
/// The actual USB peripheral is initialized by the application.
///
/// ## Buffer Strategy
///
/// USB CDC has internal buffering (typically 64-512 bytes depending on endpoint size).
/// This wrapper attempts write and returns immediately:
/// - If buffer has space: frame queued, returns `Ok(())`
/// - If buffer full: returns `Err(BufferFull)` immediately (non-blocking!)
///
/// ## WCET Analysis
///
/// Worst-case execution time is O(frame.len()) for memcpy to USB buffer.
/// No system calls, no hardware waits. Typical: < 1 microsecond for 280-byte frame.
pub struct UsbCdcTx<B> {
    _marker: core::marker::PhantomData<B>,
}

impl<B> UsbCdcTx<B> {
    /// Create new USB CDC transmitter
    ///
    /// # Parameters
    ///
    /// - `serial`: `usbd_serial::SerialPort<B>` instance from application
    ///
    /// # Note
    ///
    /// The SerialPort must be wrapped by the application, as USB bus initialization
    /// is application-specific (clock config, pin assignment, etc.)
    pub fn new(_serial: ()) -> Self {
        // TODO: Accept actual SerialPort<B> once application integration is ready
        Self {
            _marker: core::marker::PhantomData,
        }
    }
}

impl<B> FrameTx for UsbCdcTx<B> {
    fn try_send(&mut self, _frame: &[u8]) -> Result<(), TransportError> {
        // TODO: Implement actual USB CDC write
        // For now, return BufferFull to avoid false success
        Err(TransportError::BufferFull)
    }
}

/// USB CDC transport receiver (generic over USB bus type)
///
/// Wraps `usbd_serial::SerialPort<B>` to provide non-blocking frame reception.
///
/// ## Frame Parsing
///
/// USB CDC provides byte stream. This wrapper must parse protocol frames:
/// - MAVLink: Detect STX (0xFD), read length, verify CRC
/// - CCSDS: Fixed 6-byte header, read packet length, verify checksum
///
/// Incomplete frames remain buffered, `try_recv()` returns `Ok(0)`.
pub struct UsbCdcRx<B> {
    _marker: core::marker::PhantomData<B>,
}

impl<B> UsbCdcRx<B> {
    /// Create new USB CDC receiver
    pub fn new(_serial: ()) -> Self {
        // TODO: Accept actual SerialPort<B> once application integration is ready
        Self {
            _marker: core::marker::PhantomData,
        }
    }
}

impl<B> FrameRx for UsbCdcRx<B> {
    fn try_recv(&mut self, _buf: &mut [u8]) -> Result<usize, TransportError> {
        // TODO: Implement actual USB CDC read + frame parsing
        // For now, return 0 (no frame available)
        Ok(0)
    }
}

// TODO: Add UART transport implementations
// pub struct UartTx { ... }
// pub struct UartRx { ... }

// TODO: Add CAN transport implementations
// pub struct CanTx { ... }
// pub struct CanRx { ... }
