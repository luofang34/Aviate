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
    /// The MAVLink 2 `sha256_48` signature (SHA-256 over
    /// `secret_key ‖ signed bytes`, truncated to 48 bits) does not match
    /// the expected value. This indicates either:
    /// - Wrong key used by sender
    /// - Message tampered in transit
    /// - Implementation bug (different signature computation)
    InvalidSignature,

    /// Command requires signature but none provided
    ///
    /// Security policy requires signed commands, but the received
    /// command has no signature metadata.
    MissingSignature,

    /// Anti-replay check failed
    ///
    /// The command's freshness counter is not strictly greater than the
    /// last accepted counter for its [`Principal`](crate::Principal). This
    /// indicates either:
    /// - Replay attack (old message retransmitted)
    /// - Out-of-order delivery (not expected on a strictly ordered link)
    /// - Sender counter rollover (should not happen in practice)
    ReplayAttack,

    /// First frame of a new principal's stream is too old
    ///
    /// The [`Principal`](crate::Principal) has never been seen, and its
    /// freshness counter is more than the profile's age bound behind the
    /// trusted local counter (MAVLink:
    /// [`NEW_STREAM_MAX_AGE_10US`](crate::anti_replay::NEW_STREAM_MAX_AGE_10US)).
    /// This is the reboot-replay defense: without it, an attacker could
    /// replay any captured command after the receiver restarts with an
    /// empty replay window.
    StaleNewStream {
        /// The rejected command's freshness counter.
        counter: u64,
        /// The trusted local counter it was measured against.
        local_counter: u64,
    },

    /// Anti-replay table is full of already-authorized principals
    ///
    /// A command from a new, authorized principal arrived but every
    /// tracking slot is occupied. Because principals are only committed
    /// after verification and authorization, this reflects genuinely more
    /// concurrent peers than the bounded table supports, not an attack.
    ReplayCapacityExhausted,

    /// Authenticated principal maps to no authorized command source
    ///
    /// The command verified, but its [`Principal`](crate::Principal) is not
    /// bound to any [`CommandSource`](crate::CommandSource) by the gateway's
    /// authorization policy. Authority comes from this binding, never from a
    /// payload claim, so an unbound principal is rejected.
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
