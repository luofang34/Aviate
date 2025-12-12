//! Security and cryptographic hardware abstraction
//!
//! Provides traits for command authentication using hardware capabilities.
//! Implementations may use hardware accelerators (HASH/CRYP peripherals)
//! or software fallbacks depending on board capabilities.
//!
//! ## Design Principles
//!
//! These traits expose **hardware cryptographic capabilities** (what this board can do),
//! NOT security policy (what should be verified, when, etc).
//!
//! Security policy lives in `aviate-security` crate, which uses these traits.
//!
//! ## Separation of Concerns
//!
//! - **This module**: KeyStore + CryptoEngine (cryptographic hardware)
//! - **transport.rs**: FrameTx + FrameRx (link layer I/O)
//! - **aviate-security**: SecurityProfile, CommandAuth, CommandGateway (policy)
//! - **aviate-link**: TelemetryBackend, CommandLink (protocol abstraction)
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  aviate-security (Policy Layer)                             │
//! │  - SecurityProfile enum                                     │
//! │  - CommandAuth trait (PlainAuth, SecureAuth<K, C>)          │
//! │  - CommandGateway (unified command entry point)             │
//! │  - Anti-replay logic                                        │
//! └──────────────────┬──────────────────────────────────────────┘
//!                    │ uses KeyStore + CryptoEngine traits
//!                    ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │  aviate-hal/io/security (Hardware Capability)               │
//! │  - KeyStore trait (OTP/flash key access)                    │
//! │  - CryptoEngine trait (HMAC/AES/Ed25519)                    │
//! └──────────────────┬──────────────────────────────────────────┘
//!                    │ implemented by
//!                    ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │  aviate-board-* (Board-Specific)                            │
//! │  - H743KeyStore (OTP access)                                │
//! │  - H743CryptoEngine (may use HASH/CRYP HW or SW fallback)   │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage Example
//!
//! ```ignore
//! use aviate_hal_io::security::{KeyStore, CryptoEngine, CryptoAlgo};
//!
//! // Board provides cryptographic hardware capabilities
//! let keystore = H743KeyStore::new();
//! let crypto = H743CryptoEngine::new();
//!
//! // Security policy layer uses these traits to build command authentication
//! let auth = SecureAuth::new(keystore, crypto, SecurityProfile::AuthOnly);
//! let gateway = CommandGateway::new(auth);
//!
//! // App just calls gateway - all verification happens inside
//! if let Ok(Some(cmd)) = gateway.poll_command(now_ms) {
//!     kernel.execute(cmd);  // Safe: command verified by gateway
//! }
//! ```

#![forbid(unsafe_code)]

use core::fmt;

/// Cryptographic algorithm supported by hardware or software
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CryptoAlgo {
    /// HMAC-SHA256 (symmetric authentication)
    HmacSha256,
    /// AES-128-GCM (symmetric encryption + authentication)
    AesGcm128,
    /// Ed25519 (asymmetric signing)
    Ed25519,
}

/// Low-level cryptographic operation error (HAL layer)
///
/// This is the HAL layer error type. Higher layers (e.g., `aviate-security`)
/// wrap this into domain-specific errors like `AuthError`.
///
/// ## Error Semantics (DO-178C)
///
/// - `InvalidKey`: Key material is malformed or inaccessible
/// - `VerificationFailed`: Computed tag/signature does not match expected value
/// - `UnsupportedAlgo`: Requested algorithm not available on this hardware
/// - `HardwareFailure`: Crypto peripheral fault (with `hw-crypto` feature)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CryptoError {
    /// Key not found or invalid length/format
    InvalidKey,
    /// Verification failed (signature/MAC mismatch)
    VerificationFailed,
    /// Algorithm not supported by this hardware
    UnsupportedAlgo,
    /// Hardware crypto peripheral failure
    HardwareFailure,
}

impl fmt::Display for CryptoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidKey => write!(f, "Invalid or missing key"),
            Self::VerificationFailed => write!(f, "Signature/MAC verification failed"),
            Self::UnsupportedAlgo => write!(f, "Unsupported cryptographic algorithm"),
            Self::HardwareFailure => write!(f, "Hardware crypto peripheral failure"),
        }
    }
}

/// Key purpose identifier for multi-key systems
///
/// Different use cases may require different keys.
/// This enum allows the KeyStore to segregate keys by purpose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyPurpose {
    /// Command authentication/signing key (HMAC or Ed25519 private key)
    Command,
    /// Firmware signature verification key (Ed25519 public key)
    Firmware,
}

/// Key selector for per-link_id key management
///
/// MAVLink message signing requires per-link keys to support multiple
/// ground stations or command sources with independent credentials.
///
/// ## Design Rationale
///
/// - `link_id`: Identifies the command source (GCS instance, operator station, etc.)
///   - 0: Single shared secret (development/testing)
///   - 1-255: Per-station keys (production multi-GCS setups)
/// - `purpose`: Segregates keys by use case (Command vs Firmware)
///
/// ## Security Model
///
/// Each (link_id, purpose) pair maps to a unique key.
/// This allows:
/// - Independent key rotation per ground station
/// - Revocation without affecting other links
/// - Different key strengths per purpose
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeySelector {
    /// Link identifier (0-255)
    ///
    /// For MAVLink: maps to the `link_id` field in signed messages.
    /// 0 = single shared secret, 1-255 = per-station keys.
    pub link_id: u8,

    /// Purpose of this key (Command or Firmware)
    pub purpose: KeyPurpose,
}

/// Board-provided key storage (OTP, flash, secure element, etc.)
///
/// Keys are typically provisioned during manufacturing or initial setup.
/// This trait provides read-only access to stored keys.
///
/// ## Per-Link_ID Key Management
///
/// Production systems should support multiple keys indexed by `link_id`:
/// - `link_id=0`: Single shared secret (fallback for development)
/// - `link_id=1..255`: Per-station keys (production multi-GCS)
///
/// Simple implementations may map all `link_id` values to a single key.
///
/// ## Security Note
///
/// Keys should NEVER be logged or transmitted in plaintext!
/// Implementers must ensure keys are protected per DO-178C security requirements.
///
/// ## Error Semantics (DO-178C)
///
/// - `Ok(&[u8])`: Key successfully loaded
/// - `Err(CryptoError::InvalidKey)`: Key selector not provisioned or corrupted
/// - `Err(CryptoError::HardwareFailure)`: OTP/secure element read failure
pub trait KeyStore {
    /// Load key for specified link and purpose
    ///
    /// # Parameters
    ///
    /// - `selector`: Which key to load (link_id + purpose)
    ///
    /// # Returns
    ///
    /// - `Ok(&[u8])`: Key material (length depends on algorithm)
    ///   - HMAC-SHA256: 32 bytes
    ///   - Ed25519: 32 bytes (private) or 32 bytes (public)
    /// - `Err(CryptoError::InvalidKey)`: Key not provisioned or inaccessible
    /// - `Err(CryptoError::HardwareFailure)`: Hardware fault during read
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Load command key for ground station #1
    /// let key = keystore.load_key(KeySelector {
    ///     link_id: 1,
    ///     purpose: KeyPurpose::Command,
    /// })?;
    /// ```
    fn load_key(&self, selector: KeySelector) -> Result<&[u8], CryptoError>;
}

/// Board-provided cryptographic engine (hardware or software)
///
/// May use hardware accelerators (STM32 HASH/CRYP, NXP CAAM, TI SA2UL, etc.)
/// or pure software implementations (RustCrypto crates).
///
/// ## Performance
///
/// Hardware implementations should provide constant-time operations and low latency.
/// Software fallbacks are acceptable for development but may impact WCET analysis.
pub trait CryptoEngine {
    /// Get the preferred algorithm for this engine
    fn algo(&self) -> CryptoAlgo;

    /// Verify message authentication code or signature
    ///
    /// # Parameters
    ///
    /// - `algo`: Cryptographic algorithm to use
    /// - `key`: Key material (secret key for HMAC, public key for Ed25519)
    /// - `msg`: Message to verify
    /// - `tag`: Expected MAC/signature
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Verification successful
    /// - `Err(CryptoError::VerificationFailed)`: MAC/signature mismatch
    /// - `Err(CryptoError::UnsupportedAlgo)`: Algorithm not supported
    /// - `Err(CryptoError::HardwareFailure)`: Hardware fault
    fn verify(
        &mut self,
        algo: CryptoAlgo,
        key: &[u8],
        msg: &[u8],
        tag: &[u8],
    ) -> Result<(), CryptoError>;

    /// Generate message authentication code or signature
    ///
    /// # Parameters
    ///
    /// - `algo`: Cryptographic algorithm to use
    /// - `key`: Key material (secret key for HMAC, private key for Ed25519)
    /// - `msg`: Message to sign
    /// - `out`: Output buffer for MAC/signature
    ///
    /// # Returns
    ///
    /// - `Ok(len)`: Number of bytes written to `out`
    /// - `Err(CryptoError::InvalidKey)`: Key material invalid
    /// - `Err(CryptoError::UnsupportedAlgo)`: Algorithm not supported
    /// - `Err(CryptoError::HardwareFailure)`: Hardware fault
    ///
    /// # Buffer Size
    ///
    /// Caller must ensure `out` is large enough:
    /// - HMAC-SHA256: 32 bytes
    /// - AES-GCM: 16 bytes (tag)
    /// - Ed25519: 64 bytes
    fn sign(
        &mut self,
        algo: CryptoAlgo,
        key: &[u8],
        msg: &[u8],
        out: &mut [u8],
    ) -> Result<usize, CryptoError>;
}
