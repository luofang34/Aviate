//! Error types for security layer
//!
//! This module defines high-level security errors that map to underlying
//! HAL and link layer errors while adding security-specific semantics.
//!
//! ## Error Hierarchy (DO-178C Traceability)
//!
//! ```text
//! GatewayError (Application-visible)
//!   ├─ Link(LinkError)       ← From aviate-link
//!   │   ├─ Transport(...)    ← From HAL transport
//!   │   ├─ ParseError
//!   │   └─ UnsupportedMsg
//!   └─ Auth(AuthError)       ← Security policy errors
//!       ├─ Crypto(...)       ← From HAL crypto
//!       ├─ InvalidSignature
//!       ├─ MissingSignature
//!       └─ ReplayAttack
//! ```

use aviate_hal_io::security::CryptoError;
use aviate_link::errors::LinkError;

/// Authentication and signature verification errors
///
/// These errors represent security policy violations or cryptographic failures.
#[derive(Debug)]
pub enum AuthError {
    /// Cryptographic operation failed (HAL-level error)
    ///
    /// Examples:
    /// - Key not found in OTP/flash
    /// - HMAC computation failed
    /// - Hardware crypto accelerator error
    Crypto(CryptoError),

    /// Signature verification failed
    ///
    /// The HMAC-SHA256 signature does not match the expected value.
    /// This indicates either:
    /// - Wrong key used by sender
    /// - Message tampered in transit
    /// - Implementation bug (different HMAC computation)
    InvalidSignature,

    /// Command requires signature but none provided
    ///
    /// Security policy requires signed commands, but the received
    /// command has no signature metadata.
    MissingSignature,

    /// Anti-replay check failed
    ///
    /// The command's timestamp is not strictly greater than the last
    /// accepted timestamp for its `(system_id, component_id, link_id)`
    /// identity. This indicates either:
    /// - Replay attack (old message retransmitted)
    /// - Out-of-order delivery (not expected in MAVLink over USB/UART)
    /// - Sender timestamp rollover (should not happen in practice)
    ReplayAttack,

    /// Anti-replay table is full of already-authenticated identities
    ///
    /// A frame from a new, authenticated signing identity arrived but every
    /// tracking slot is occupied. Because identities are only committed
    /// after signature verification, this reflects genuinely more
    /// concurrent peers than the bounded table supports, not an attack.
    ReplayCapacityExhausted,

    /// Authenticated identity maps to no authorized command source
    ///
    /// The frame verified, but its `(system_id, component_id, link_id)`
    /// identity is not bound to any [`CommandSource`](crate::CommandSource)
    /// by the gateway's credential policy. The command's authority comes
    /// from this binding, never from a payload claim, so an unbound
    /// identity is rejected.
    UnauthorizedSource,
}

/// Result of an authentication or anti-replay operation.
pub type AuthResult<T> = Result<T, AuthError>;

/// High-level gateway errors (what applications see)
///
/// This combines errors from the link layer (transport, parsing) and
/// the security layer (authentication, anti-replay).
#[derive(Debug)]
pub enum GatewayError {
    /// Link layer error (transport or protocol parsing)
    ///
    /// Examples:
    /// - USB disconnected
    /// - MAVLink CRC mismatch
    /// - Unsupported message type
    Link(LinkError),

    /// Authentication or security policy error
    ///
    /// Examples:
    /// - Signature verification failed
    /// - Replay attack detected
    /// - Missing required signature
    Auth(AuthError),

    /// No command available (not an error, just no data)
    ///
    /// Used internally to distinguish "no command ready" from "error occurred".
    /// Applications should treat this as Ok(None) in poll semantics.
    NoCommand,
}

/// Result surfaced to applications combining link- and security-layer errors.
pub type GatewayResult<T> = Result<T, GatewayError>;

/// Convert LinkError to GatewayError
impl From<LinkError> for GatewayError {
    fn from(err: LinkError) -> Self {
        GatewayError::Link(err)
    }
}

/// Convert AuthError to GatewayError
impl From<AuthError> for GatewayError {
    fn from(err: AuthError) -> Self {
        GatewayError::Auth(err)
    }
}

/// Convert CryptoError to AuthError
impl From<CryptoError> for AuthError {
    fn from(err: CryptoError) -> Self {
        AuthError::Crypto(err)
    }
}
