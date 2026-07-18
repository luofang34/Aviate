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
use aviate_link::command::{Command, SignatureMeta};

use crate::anti_replay::AntiReplayWindow;
use crate::errors::{AuthError, AuthResult};

/// Command authentication trait
///
/// Implementations decide whether a frame's bytes are authentic and fresh.
/// This includes signature verification and anti-replay checks.
///
/// ## DO-178C Requirements
///
/// - Deterministic: Same input → same output
/// - Non-blocking: Returns immediately, no waiting
/// - Time-bounded: WCET provable (no unbounded loops)
pub trait CommandAuth {
    /// Authenticate a frame's (optional) signature metadata.
    ///
    /// This is the primitive the command gateway calls on the bytes it is
    /// about to trust. For a signed frame it verifies the signature over the
    /// exact [`SignatureMeta::signed_message`] coverage and, only if that
    /// succeeds, commits the frame's anti-replay counter under its full
    /// `(system_id, component_id, link_id)` identity.
    ///
    /// `sig` is `None` for an unsigned frame: a signing policy MUST reject
    /// it ([`AuthError::MissingSignature`]); only an explicitly insecure
    /// development policy accepts it.
    ///
    /// ## Ordering guarantee
    ///
    /// Signature verification happens *before* any replay state is mutated,
    /// so a forged frame carrying a high timestamp but an invalid signature
    /// cannot advance — and thereby poison — a legitimate sender's counter.
    fn authenticate(&mut self, sig: Option<&SignatureMeta>) -> AuthResult<()>;

    /// Authenticate a parsed link command by its embedded signature.
    ///
    /// Convenience over [`Self::authenticate`] for the [`Command`] type; the
    /// default delegates to it, so a signing policy rejects an unsigned
    /// command and an insecure policy accepts it.
    fn verify(&mut self, cmd: &Command) -> AuthResult<()> {
        self.authenticate(cmd.signature.as_ref())
    }
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
    fn authenticate(&mut self, _sig: Option<&SignatureMeta>) -> AuthResult<()> {
        // Development/SITL only: accept signed or unsigned without
        // verification. Flight builds must not compile this in (gated by an
        // explicit non-flight assembly at the consumer).
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
///
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
        self.crypto
            .sign(CryptoAlgo::HmacSha256, key, raw_frame, &mut computed_sig)?;

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

    /// Verify a signed frame, then commit its anti-replay counter.
    ///
    /// The ordering is load-bearing: the HMAC is checked over the canonical
    /// [`SignatureMeta::signed_message`] coverage FIRST, and only a valid
    /// signature is allowed to advance the sender's replay counter. A frame
    /// whose signature does not verify never mutates any state, so it cannot
    /// poison a legitimate identity's high-water mark with a forged
    /// timestamp.
    fn authenticate_signed(&mut self, sig_meta: &SignatureMeta) -> AuthResult<()> {
        // Canonical coverage: the frame minus its trailing signature bytes.
        // A malformed length yields `None` — treated as a bad signature, not
        // a panic.
        let message = sig_meta
            .signed_message()
            .ok_or(AuthError::InvalidSignature)?;

        // 1. Verify HMAC-SHA256 BEFORE touching replay state.
        self.verify_signature(sig_meta.link_id, message, &sig_meta.sig)?;

        // 2. Only an authenticated frame commits its anti-replay counter,
        //    keyed on the full (system, component, link) identity.
        self.anti_replay.check_and_update(
            sig_meta.system_id,
            sig_meta.component_id,
            sig_meta.link_id,
            sig_meta.timestamp,
        )
    }
}

impl<K: KeyStore, C: CryptoEngine> CommandAuth for SignedAuth<K, C> {
    fn authenticate(&mut self, sig: Option<&SignatureMeta>) -> AuthResult<()> {
        // A signing policy rejects any unsigned frame outright.
        let sig_meta = sig.ok_or(AuthError::MissingSignature)?;
        self.authenticate_signed(sig_meta)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::test_support::{correct_sig, signed_auth, signed_meta};
    use aviate_link::command::{CommandKind, SignatureMeta, MAX_SIGNED_FRAME_SIZE};

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
            signature: Some(signed_meta(1, 1, 5, 1000, &[0u8; 26], [0xAA; 6])),
        };
        assert!(auth.verify(&cmd_signed).is_ok());
    }

    #[test]
    fn valid_signature_and_monotonic_timestamp_accepted() {
        let mut auth = signed_auth();
        let msg = [0x10u8, 0x11, 0x12, 0x13];
        let meta = signed_meta(1, 1, 5, 1000, &msg, correct_sig(&msg));
        assert!(auth.authenticate(Some(&meta)).is_ok());
    }

    #[test]
    fn bad_signature_rejected() {
        let mut auth = signed_auth();
        let msg = [0x10u8, 0x11, 0x12, 0x13];
        let meta = signed_meta(1, 1, 5, 1000, &msg, [0x00; 6]);
        assert!(matches!(
            auth.authenticate(Some(&meta)),
            Err(AuthError::InvalidSignature)
        ));
    }

    /// The load-bearing guardrail: a forged frame with a huge timestamp but
    /// an INVALID signature must not advance the sender's replay counter.
    /// If replay state were committed before verification (the poisoning
    /// bug), the later legitimate low-timestamp frame would be rejected.
    #[test]
    fn invalid_signature_does_not_poison_replay_window() {
        let mut auth = signed_auth();
        let msg = [0xAAu8, 0xBB, 0xCC];

        // Attacker: identity (1,1,5), timestamp far in the future, bad sig.
        let forged = signed_meta(1, 1, 5, 9_000_000, &msg, [0xFF; 6]);
        assert!(matches!(
            auth.authenticate(Some(&forged)),
            Err(AuthError::InvalidSignature)
        ));

        // Legitimate sender, same identity, a normal (low) timestamp, valid
        // sig. This MUST still be accepted — the forged frame left the
        // window untouched.
        let legit = signed_meta(1, 1, 5, 1000, &msg, correct_sig(&msg));
        assert!(
            auth.authenticate(Some(&legit)).is_ok(),
            "forged bad-signature frame poisoned the replay window"
        );
    }

    #[test]
    fn replayed_timestamp_rejected_after_valid_frame() {
        let mut auth = signed_auth();
        let msg = [0x01u8, 0x02];
        let meta = signed_meta(1, 1, 7, 5000, &msg, correct_sig(&msg));
        assert!(auth.authenticate(Some(&meta)).is_ok());
        // Same identity, same timestamp → replay.
        let replay = signed_meta(1, 1, 7, 5000, &msg, correct_sig(&msg));
        assert!(matches!(
            auth.authenticate(Some(&replay)),
            Err(AuthError::ReplayAttack)
        ));
    }

    #[test]
    fn malformed_frame_length_is_rejected_not_panicked() {
        let mut auth = signed_auth();
        // raw_frame_len shorter than the signature trailer → signed_message
        // returns None → InvalidSignature, never an out-of-bounds slice.
        let meta = SignatureMeta {
            system_id: 1,
            component_id: 1,
            link_id: 5,
            timestamp: 1000,
            sig: [0u8; 6],
            raw_frame: [0u8; MAX_SIGNED_FRAME_SIZE],
            raw_frame_len: 3,
        };
        assert!(matches!(
            auth.authenticate(Some(&meta)),
            Err(AuthError::InvalidSignature)
        ));

        // raw_frame_len beyond the backing buffer → also None, no panic.
        let meta = SignatureMeta {
            system_id: 1,
            component_id: 1,
            link_id: 5,
            timestamp: 1000,
            sig: [0u8; 6],
            raw_frame: [0u8; MAX_SIGNED_FRAME_SIZE],
            raw_frame_len: MAX_SIGNED_FRAME_SIZE + 100,
        };
        assert!(matches!(
            auth.authenticate(Some(&meta)),
            Err(AuthError::InvalidSignature)
        ));
    }
}
