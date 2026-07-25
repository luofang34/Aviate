//! Error types for link-layer telemetry and command operations
//!
//! This module defines domain-level errors that wrap lower-level HAL errors
//! and protocol-specific errors.

use aviate_hal_io::transport::TransportError;

/// Errors originating from telemetry formatting / sending.
///
/// This is a domain-level error type that applications see.
/// Lower-level errors from HAL and protocol layers are wrapped here.
#[derive(Debug)]
pub enum TelemetryError {
    /// Output buffer too small for formatted message
    BufferTooSmall,
    /// Transport layer error (from HAL FrameTx)
    Transport(TransportError),
    /// Protocol-level formatting error
    Protocol,
    /// A configured rate is zero; carries the offending config field or
    /// runtime parameter name
    ZeroRate(&'static str),
}

impl core::fmt::Display for TelemetryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TelemetryError::BufferTooSmall => write!(f, "output buffer too small"),
            TelemetryError::Transport(e) => write!(f, "transport error: {:?}", e),
            TelemetryError::Protocol => write!(f, "protocol formatting error"),
            TelemetryError::ZeroRate(field) => {
                write!(f, "{} is zero; rates must be 1-255 Hz", field)
            }
        }
    }
}

/// Result type for telemetry operations
pub type TelemetryResult<T> = Result<T, TelemetryError>;

/// Errors in the command reception pipeline (before security).
///
/// This is a domain-level error type for the link layer only.
/// Security errors (authentication, replay) are separate.
#[derive(Debug)]
pub enum LinkError {
    /// Transport layer error (from HAL FrameRx)
    Transport(TransportError),
    /// Message parsing failed; carries the protocol-level cause
    /// (CRC mismatch, invalid format, unsupported incompat flags, ...)
    Parse(crate::mavlink::protocol::ParseError),
    /// Message parsed successfully but not mapped to Command
    UnsupportedMsg,
    /// Command is addressed to a different vehicle or component.
    ///
    /// An addressing filter, not an authorization decision: a verified
    /// credential still says who may command us, and this says only that
    /// this particular frame was not aimed here.
    WrongAddressee {
        /// The frame's target system id.
        target_system: u8,
        /// The frame's target component id.
        target_component: u8,
    },

    /// Frame bytes and parse result are inconsistent (trailing bytes
    /// beyond the parsed frame, or a frame too large for the fixed
    /// signature buffer); carries the frame and consumed lengths
    FrameLengthMismatch {
        /// Total bytes handed to the parser.
        frame_len: usize,
        /// Bytes the parser actually consumed.
        consumed: usize,
    },
}

/// Result type for command link operations
pub type LinkResult<T> = Result<T, LinkError>;
