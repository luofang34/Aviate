//! Command authentication implementations
//!
//! This module defines the `CommandAuth` trait and provides two implementations:
//! - `PlainAuth`: No verification (development/testing only)
//! - `SignedAuth`: MAVLink signature verification with HMAC-SHA256
//!
//! ## Security Model
//!
//! Commands can be authenticated in two modes:
//!
//! ### PlainAuth (Insecure)
//! - Accepts ALL commands without verification
//! - For development, SITL simulation, and testing only
//! - MUST NOT be used in production flight systems
//!
//! ### SignedAuth (Secure)
//! - Requires MAVLink message signing (13-byte signature extension)
//! - HMAC-SHA256 verification per MAVLink spec
//! - Per-link_id key lookup
//! - Anti-replay protection (strict monotonic counter)
//!
//! ## DO-178C Criticality
//!
//! - **DAL A/B**: Flight-critical security policy
//! - ALL external commands MUST go through CommandAuth
//! - Bypass paths are prohibited

use aviate_hal_io::security::{CryptoEngine, KeyPurpose, KeySelector, KeyStore};
use aviate_link::command::Command;

use crate::anti_replay::AntiReplayWindow;
use crate::errors::{AuthError, AuthResult};

/// Command authentication trait
///
/// Implementations verify that a command is authentic and authorized.
/// This includes signature verification and anti-replay checks.
///
/// ## Contract
///
/// - **verify()**: Check if command is authentic
///   - Ok(()): Command verified, safe to execute
///   - Err(AuthError): Command rejected, do NOT execute
///
/// ## DO-178C Requirements
///
/// - Deterministic: Same input → same output
/// - Non-blocking: Returns immediately, no waiting
/// - Time-bounded: WCET provable (no unbounded loops)
pub trait CommandAuth {
    /// Verify command authenticity
    ///
    /// ## Parameters
    ///
    /// - `cmd`: Command to verify (includes optional signature metadata)
    ///
    /// ## Returns
    ///
    /// - `Ok(())`: Command is authentic and authorized
    /// - `Err(AuthError)`: Command is rejected (invalid signature, replay, etc.)
    fn verify(&mut self, cmd: &Command) -> AuthResult<()>;
}

/// Plain authentication (no verification)
///
/// ## Security Warning
///
/// This implementation accepts ALL commands without verification!
/// It is ONLY safe for:
/// - Development on isolated test benches
/// - SITL simulation (software-in-the-loop)
/// - Unit testing
///
/// **NEVER use this in production flight systems!**
///
/// ## Usage Example
///
/// ```ignore
/// let auth = PlainAuth::new();
/// let mut gateway = CommandGateway::new(link, auth);
/// ```
pub struct PlainAuth;

impl PlainAuth {
    /// Create new plain authentication (no verification)
    pub const fn new() -> Self {
        Self
    }
}

impl CommandAuth for PlainAuth {
    fn verify(&mut self, _cmd: &Command) -> AuthResult<()> {
        // Accept all commands without verification
        Ok(())
    }
}

impl Default for PlainAuth {
    fn default() -> Self {
        Self::new()
    }
}

/// Signed authentication using HMAC-SHA256
///
/// This implementation verifies MAVLink message signatures according to
/// the MAVLink signing specification:
///
/// ## Verification Steps
///
/// 1. Check signature is present (reject if missing)
/// 2. Anti-replay check (timestamp must be strictly greater than last)
/// 3. Load key for link_id from KeyStore
/// 4. Recompute HMAC-SHA256 over raw frame bytes
/// 5. Compare computed signature with provided signature (constant-time)
///
/// ## Type Parameters
///
/// - `K`: KeyStore implementation (OTP, flash, TPM, etc.)
/// - `C`: CryptoEngine implementation (hardware or software HMAC)
///
/// ## Usage Example
///
/// ```ignore
/// use aviate_hal_stm32h7::{Stm32h7KeyStore, Stm32h7CryptoEngine};
///
/// let keystore = Stm32h7KeyStore::new();
/// let crypto = Stm32h7CryptoEngine::new();
/// let auth = SignedAuth::new(keystore, crypto);
/// let mut gateway = CommandGateway::new(link, auth);
/// ```
pub struct SignedAuth<K: KeyStore, C: CryptoEngine> {
    /// Key storage (OTP, flash, TPM, etc.)
    keystore: K,

    /// Cryptographic operations (HMAC-SHA256)
    crypto: C,

    /// Anti-replay window (per-link_id monotonic counters)
    anti_replay: AntiReplayWindow,
}

impl<K: KeyStore, C: CryptoEngine> SignedAuth<K, C> {
    /// Create new signed authentication
    ///
    /// ## Parameters
    ///
    /// - `keystore`: Key storage implementation
    /// - `crypto`: Cryptographic operations implementation
    pub fn new(keystore: K, crypto: C) -> Self {
        Self {
            keystore,
            crypto,
            anti_replay: AntiReplayWindow::new(),
        }
    }

    /// Verify HMAC-SHA256 signature
    ///
    /// ## Parameters
    ///
    /// - `link_id`: Sender identifier (for key lookup)
    /// - `raw_frame`: Original frame bytes (for HMAC computation)
    /// - `expected_sig`: Signature from command (6 bytes, truncated HMAC)
    ///
    /// ## Returns
    ///
    /// - `Ok(())`: Signature valid
    /// - `Err(AuthError)`: Signature invalid or crypto error
    fn verify_signature(
        &mut self,
        link_id: u8,
        raw_frame: &[u8],
        expected_sig: &[u8; 6],
    ) -> AuthResult<()> {
        // Load key for this link_id
        let selector = KeySelector {
            link_id,
            purpose: KeyPurpose::Command,
        };
        let key = self.keystore.load_key(selector)?;

        // Compute HMAC-SHA256 over raw frame
        // Per MAVLink spec: HMAC includes everything up to (but not including) signature
        let mut computed_sig = [0u8; 32]; // Full HMAC-SHA256 output
        use aviate_hal_io::security::CryptoAlgo;
        self.crypto.sign(CryptoAlgo::HmacSha256, key, raw_frame, &mut computed_sig)?;

        // Compare first 6 bytes (MAVLink uses truncated HMAC)
        // Use constant-time comparison to prevent timing attacks
        let matches = computed_sig[..6]
            .iter()
            .zip(expected_sig.iter())
            .fold(0u8, |acc, (a, b)| acc | (a ^ b));

        if matches == 0 {
            Ok(())
        } else {
            Err(AuthError::InvalidSignature)
        }
    }
}

impl<K: KeyStore, C: CryptoEngine> CommandAuth for SignedAuth<K, C> {
    fn verify(&mut self, cmd: &Command) -> AuthResult<()> {
        // Extract signature metadata (reject if missing)
        let sig_meta = cmd.signature.as_ref().ok_or(AuthError::MissingSignature)?;

        // Anti-replay check (strict monotonic)
        self.anti_replay
            .check_and_update(sig_meta.link_id, sig_meta.timestamp)?;

        // Verify HMAC-SHA256 signature
        self.verify_signature(sig_meta.link_id, &sig_meta.raw_frame, &sig_meta.sig)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use alloc::vec;
    use super::*;
    use aviate_link::command::{CommandKind, SignatureMeta};

    #[test]
    fn test_plain_auth_accepts_all() {
        let mut auth = PlainAuth::new();

        // Unsigned command
        let cmd = Command {
            kind: CommandKind::Arm,
            params: [0.0; 7],
            timestamp_ms: 1000,
            signature: None,
        };
        assert!(auth.verify(&cmd).is_ok());

        // Signed command (PlainAuth doesn't care)
        let cmd_signed = Command {
            kind: CommandKind::Arm,
            params: [0.0; 7],
            timestamp_ms: 1000,
            signature: Some(SignatureMeta {
                link_id: 5,
                timestamp: 1000,
                sig: [0xAA; 6],
                raw_frame: vec![0u8; 32],
            }),
        };
        assert!(auth.verify(&cmd_signed).is_ok());
    }

    // Note: Full SignedAuth tests require mock KeyStore/CryptoEngine
    // or integration tests with real implementations.
    // We'll add those when testing the full security layer.
}
