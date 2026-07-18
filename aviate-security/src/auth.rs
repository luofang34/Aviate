//! MAVLink 2 signature verification (`sha256_48`).
//!
//! [`SignedAuth`] is the cryptographic core of the MAVLink signing
//! profile: it verifies a signed frame's signature and nothing else. It
//! holds no replay state and makes no authorization decision — those are
//! scheme-neutral gateway concerns ([`crate::CommandGateway`]). The MAVLink
//! admission adapter ([`crate::admission::MavlinkAdmission`]) owns a
//! `SignedAuth` and turns a verified frame into a scheme-neutral
//! [`AuthenticatedCommand`](crate::AuthenticatedCommand).
//!
//! ## Signature construction
//!
//! MAVLink 2 signing is `sha256_48`: the first 48 bits of
//! `SHA-256(secret_key ‖ header ‖ payload ‖ CRC ‖ link_id ‖ timestamp)`.
//! This is a keyed-prefix digest, NOT HMAC — an HMAC-SHA256 tag over the
//! same bytes is a different value and does not interoperate with MAVLink
//! peers (QGC, pymavlink, ArduPilot).
//!
//! ## DO-178C Criticality
//!
//! - **DAL A/B**: Flight-critical security policy
//! - Deterministic, non-blocking, WCET-bounded (no unbounded loops)

use aviate_hal_io::security::{CryptoAlgo, CryptoEngine, KeyPurpose, KeySelector, KeyStore};
use aviate_link::command::SignatureMeta;

use crate::errors::{AuthError, AuthResult};

/// MAVLink 2 signature verifier (`sha256_48`).
///
/// ## Verification Steps
///
/// 1. Recover the canonical signed coverage
///    ([`SignatureMeta::signed_message`]).
/// 2. Load the key for the frame's `link_id` from the `KeyStore`.
/// 3. Recompute `sha256_48` = SHA-256(`key ‖ signed bytes`), truncated.
/// 4. Compare with the frame's 6 signature bytes (constant-time).
///
/// Anti-replay and authorization are performed by the gateway, after the
/// adapter has produced a principal — so an unauthorized-but-signed frame
/// never advances any counter.
///
/// ## Type Parameters
///
/// - `K`: KeyStore implementation (OTP, flash, TPM, etc.)
/// - `C`: CryptoEngine implementation (hardware or software SHA-256)
pub struct SignedAuth<K: KeyStore, C: CryptoEngine> {
    keystore: K,
    crypto: C,
}

impl<K: KeyStore, C: CryptoEngine> SignedAuth<K, C> {
    /// Create a new signature verifier.
    pub fn new(keystore: K, crypto: C) -> Self {
        Self { keystore, crypto }
    }

    /// Verify a signed frame's signature over its canonical coverage.
    ///
    /// Returns `Ok(())` only when the recomputed `sha256_48` matches the
    /// frame's signature bytes. A malformed `raw_frame_len` yields
    /// [`AuthError::InvalidSignature`] (via `signed_message` returning
    /// `None`), never a panic. This mutates no state.
    pub fn verify_frame(&mut self, sig: &SignatureMeta) -> AuthResult<()> {
        // Canonical coverage: the frame minus its trailing signature bytes.
        let message = sig.signed_message().ok_or(AuthError::InvalidSignature)?;
        self.verify_signature(sig.link_id, message, &sig.sig)
    }

    /// Verify a MAVLink 2 `sha256_48` signature over `signed_bytes`.
    fn verify_signature(
        &mut self,
        link_id: u8,
        signed_bytes: &[u8],
        expected_sig: &[u8; 6],
    ) -> AuthResult<()> {
        let selector = KeySelector {
            link_id,
            purpose: KeyPurpose::Command,
        };
        let key = self.keystore.load_key(selector)?;

        // MAVLink 2 signing: SHA-256 over `key ‖ signed bytes` (the
        // keyed-prefix construction, NOT HMAC), truncated to 48 bits.
        let mut computed = [0u8; 32];
        self.crypto.sign(
            CryptoAlgo::Sha256KeyedPrefix,
            key,
            signed_bytes,
            &mut computed,
        )?;

        // Constant-time comparison of the 48-bit truncation.
        let diff = computed[..6]
            .iter()
            .zip(expected_sig.iter())
            .fold(0u8, |acc, (a, b)| acc | (a ^ b));

        if diff == 0 {
            Ok(())
        } else {
            Err(AuthError::InvalidSignature)
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::test_support::{correct_sig, signed_auth, signed_meta};
    use aviate_link::command::MAX_SIGNED_FRAME_SIZE;

    #[test]
    fn valid_signature_accepted() {
        let mut auth = signed_auth();
        let msg = [0x10u8, 0x11, 0x12, 0x13];
        let meta = signed_meta(1, 1, 5, 1000, &msg, correct_sig(&msg));
        assert!(auth.verify_frame(&meta).is_ok());
    }

    #[test]
    fn bad_signature_rejected() {
        let mut auth = signed_auth();
        let msg = [0x10u8, 0x11, 0x12, 0x13];
        let meta = signed_meta(1, 1, 5, 1000, &msg, [0x00; 6]);
        assert!(matches!(
            auth.verify_frame(&meta),
            Err(AuthError::InvalidSignature)
        ));
    }

    /// verify_frame must be pure: no replay/authorization state exists here
    /// to mutate, so verifying the same frame twice both succeed.
    #[test]
    fn verify_frame_is_stateless() {
        let mut auth = signed_auth();
        let msg = [0x42u8; 4];
        let meta = signed_meta(1, 1, 5, 5000, &msg, correct_sig(&msg));
        assert!(auth.verify_frame(&meta).is_ok());
        assert!(auth.verify_frame(&meta).is_ok());
    }

    #[test]
    fn malformed_frame_length_is_rejected_not_panicked() {
        let mut auth = signed_auth();
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
            auth.verify_frame(&meta),
            Err(AuthError::InvalidSignature)
        ));

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
            auth.verify_frame(&meta),
            Err(AuthError::InvalidSignature)
        ));
    }
}
