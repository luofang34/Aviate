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
    /// A configured rate is zero; carries the offending config field name
    ZeroRate(&'static str),
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
    /// Message parsing failed (CRC mismatch, invalid format, etc.)
    ParseError,
    /// Message parsed successfully but not mapped to Command
    UnsupportedMsg,
}

/// Result type for command link operations
pub type LinkResult<T> = Result<T, LinkError>;
