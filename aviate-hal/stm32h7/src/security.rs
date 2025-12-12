//! STM32H7 security primitives implementation
//!
//! Provides KeyStore and CryptoEngine implementations for STM32H7 family.
//! Implements traits from `aviate-hal-io::security`.
//!
//! ## Key Storage Strategy
//!
//! - **Development** (default): Flash const keys (easy to update, INSECURE)
//! - **Production** (`secure-keys` feature): OTP reads (write-once, tamper-resistant)
//!
//! ## Crypto Implementation
//!
//! - **Software** (default): `sha2` + `hmac` crates (~50 μs for 64-byte message)
//! - **Hardware** (`hw-crypto` feature): HASH/CRYP peripherals (TODO, ~2 μs target)
//!
//! ## DO-178C Contract
//!
//! All functions are non-blocking and have bounded WCET:
//! - `load_command_key()`: O(1), ~10 CPU cycles (pointer return)
//! - `verify()`: O(msg_len), ~50 μs for 64-byte message (software HMAC)
//! - `sign()`: O(msg_len), ~50 μs for 64-byte message (software HMAC)

use aviate_hal_io::security::{
    CryptoAlgo, CryptoEngine, CryptoError, KeyPurpose, KeySelector, KeyStore,
};
use hmac::{Hmac, Mac};
use sha2::Sha256;

type HmacSha256 = Hmac<Sha256>;

/// STM32H7 key storage implementation
///
/// ## Key Management Model
///
/// Simple implementation: **Single shared secret** for all `link_id` values.
/// Maps all (link_id, purpose) combinations to the same key per purpose.
///
/// Production multi-GCS setups should implement per-link_id key storage
/// using a more sophisticated KeyStore (e.g., external secure element).
///
/// ## Production Mode (feature = "secure-keys")
///
/// Keys stored in OTP memory at fixed offsets:
/// - Command key (HMAC): OTP bytes 0-31 (32 bytes)
/// - Firmware pubkey (Ed25519): OTP bytes 32-63 (32 bytes)
///
/// OTP is write-once during manufacturing, cannot be changed in field.
///
/// ## Development Mode (default)
///
/// Keys hardcoded in flash for testing. **DO NOT USE IN PRODUCTION!**
/// These keys are visible in firmware binary and easily extracted.
pub struct Stm32h7KeyStore {
    _marker: (),
}

impl Stm32h7KeyStore {
    /// Create new keystore instance
    ///
    /// # DO-178C Contract
    ///
    /// - Non-blocking: YES (no I/O, no initialization)
    /// - WCET: O(1), ~5 CPU cycles
    /// - Errors: None (infallible)
    pub fn new() -> Self {
        Self { _marker: () }
    }
}

impl Default for Stm32h7KeyStore {
    fn default() -> Self {
        Self::new()
    }
}

impl KeyStore for Stm32h7KeyStore {
    fn load_key(&self, selector: KeySelector) -> Result<&[u8], CryptoError> {
        // Simple implementation: Single shared secret (ignore link_id)
        // Production multi-GCS setups should implement per-link_id keys

        match selector.purpose {
            KeyPurpose::Command => {
                #[cfg(feature = "secure-keys")]
                {
                    // Production: Read from OTP
                    // TODO: Implement OTP access at 0x1FF0_F000 + offset
                    // Safety: OTP reads are safe, memory-mapped peripheral
                    //
                    // For now, return error to avoid false security claims
                    Err(CryptoError::InvalidKey)
                }

                #[cfg(not(feature = "secure-keys"))]
                {
                    // Development fallback: Hardcoded test key (INSECURE!)
                    // DO NOT USE IN PRODUCTION - visible in firmware binary!
                    //
                    // This key is intentionally weak for development/testing only.
                    // 32 bytes = 256 bits for HMAC-SHA256
                    //
                    // Note: Same key for all link_id values (single shared secret)
                    const DEV_KEY: &[u8] = b"aviate_dev_key_do_not_use_prod!!";
                    Ok(DEV_KEY)
                }
            }

            KeyPurpose::Firmware => {
                #[cfg(feature = "secure-keys")]
                {
                    // Production: Read from OTP
                    // TODO: Implement OTP access at 0x1FF0_F000 + 32
                    Err(CryptoError::InvalidKey)
                }

                #[cfg(not(feature = "secure-keys"))]
                {
                    // Development fallback: Hardcoded test pubkey (INSECURE!)
                    // Placeholder Ed25519 public key (all zeros, invalid for real use)
                    const DEV_PUBKEY: &[u8] = &[
                        0x00; 32 // 32-byte Ed25519 public key placeholder
                    ];
                    Ok(DEV_PUBKEY)
                }
            }
        }
    }
}

/// STM32H7 cryptographic engine implementation
///
/// ## Hardware Acceleration (feature = "hw-crypto")
///
/// Uses HASH and CRYP peripherals for:
/// - HMAC-SHA256: ~2 μs for 64-byte message (HW accelerated, target)
/// - AES-128-GCM: ~10 μs for 256-byte message (HW accelerated, TODO)
///
/// ## Software Fallback (default)
///
/// Uses RustCrypto crates:
/// - `hmac` + `sha2` for HMAC-SHA256 (~50 μs for 64-byte message)
/// - `aes-gcm` for AES-GCM (TODO, ~200 μs for 256-byte message)
/// - `ed25519-dalek` for Ed25519 signatures (TODO, ~2 ms per verify)
///
/// Software fallback is acceptable for development but impacts WCET analysis.
///
/// ## DO-178C Contract
///
/// All operations are:
/// - Non-blocking (no busy-wait, no interrupt wait)
/// - Bounded time complexity: O(msg_len + key_len)
/// - Deterministic (no data-dependent branches in crypto primitives)
pub struct Stm32h7CryptoEngine {
    _marker: (),
}

impl Stm32h7CryptoEngine {
    /// Create new crypto engine instance
    ///
    /// # DO-178C Contract
    ///
    /// - Non-blocking: YES
    /// - WCET: O(1), ~5 CPU cycles
    /// - Errors: None (infallible)
    pub fn new() -> Self {
        Self { _marker: () }
    }
}

impl Default for Stm32h7CryptoEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl CryptoEngine for Stm32h7CryptoEngine {
    fn algo(&self) -> CryptoAlgo {
        // Prefer HMAC-SHA256 for simplicity and performance
        CryptoAlgo::HmacSha256
    }

    /// Verify message authentication code or signature
    ///
    /// # DO-178C Contract
    ///
    /// - Non-blocking: YES (pure computation, no I/O)
    /// - Time complexity: O(msg_len + key_len), bounded by HMAC computation
    /// - WCET (engineering target, to be validated):
    ///   - Software HMAC-SHA256: ~50 μs for 64-byte message @ 480 MHz
    ///   - Hardware HMAC-SHA256: ~2 μs for 64-byte message (with `hw-crypto`)
    /// - Constant-time: YES (HMAC implementation is constant-time w.r.t. key)
    ///
    /// # Error Semantics
    ///
    /// - `Ok(())`: Verification successful, tag matches
    /// - `Err(CryptoError::VerificationFailed)`: Tag mismatch (invalid signature)
    /// - `Err(CryptoError::UnsupportedAlgo)`: Algorithm not supported
    /// - `Err(CryptoError::HardwareFailure)`: Hardware fault (with `hw-crypto`)
    fn verify(
        &mut self,
        algo: CryptoAlgo,
        key: &[u8],
        msg: &[u8],
        tag: &[u8],
    ) -> Result<(), CryptoError> {
        match algo {
            CryptoAlgo::HmacSha256 => {
                #[cfg(feature = "hw-crypto")]
                {
                    // TODO: Use STM32H7 HASH peripheral for HW acceleration
                    // For now, fall back to software
                    self.verify_hmac_sw(key, msg, tag)
                }

                #[cfg(not(feature = "hw-crypto"))]
                {
                    self.verify_hmac_sw(key, msg, tag)
                }
            }

            CryptoAlgo::AesGcm128 => {
                // TODO: Implement AES-GCM using CRYP peripheral or RustCrypto
                Err(CryptoError::UnsupportedAlgo)
            }

            CryptoAlgo::Ed25519 => {
                // TODO: Implement Ed25519 using ed25519-dalek
                Err(CryptoError::UnsupportedAlgo)
            }
        }
    }

    /// Generate message authentication code or signature
    ///
    /// # DO-178C Contract
    ///
    /// - Non-blocking: YES (pure computation, no I/O)
    /// - Time complexity: O(msg_len + key_len), bounded by HMAC computation
    /// - WCET (engineering target, to be validated):
    ///   - Software HMAC-SHA256: ~50 μs for 64-byte message @ 480 MHz
    ///   - Hardware HMAC-SHA256: ~2 μs for 64-byte message (with `hw-crypto`)
    ///
    /// # Error Semantics
    ///
    /// - `Ok(len)`: Success, `len` bytes written to `out`
    /// - `Err(CryptoError::InvalidKey)`: Key length invalid
    /// - `Err(CryptoError::UnsupportedAlgo)`: Algorithm not supported
    /// - `Err(CryptoError::HardwareFailure)`: Hardware fault (with `hw-crypto`)
    ///
    /// # Buffer Size Requirements
    ///
    /// Caller must ensure `out` is large enough:
    /// - HMAC-SHA256: 32 bytes minimum
    /// - AES-GCM: 16 bytes (tag only)
    /// - Ed25519: 64 bytes
    fn sign(
        &mut self,
        algo: CryptoAlgo,
        key: &[u8],
        msg: &[u8],
        out: &mut [u8],
    ) -> Result<usize, CryptoError> {
        match algo {
            CryptoAlgo::HmacSha256 => {
                if out.len() < 32 {
                    return Err(CryptoError::InvalidKey);
                }

                #[cfg(feature = "hw-crypto")]
                {
                    // TODO: Use STM32H7 HASH peripheral for HW acceleration
                    self.sign_hmac_sw(key, msg, out)
                }

                #[cfg(not(feature = "hw-crypto"))]
                {
                    self.sign_hmac_sw(key, msg, out)
                }
            }

            CryptoAlgo::AesGcm128 => {
                // TODO: Implement AES-GCM tag generation
                Err(CryptoError::UnsupportedAlgo)
            }

            CryptoAlgo::Ed25519 => {
                // TODO: Implement Ed25519 signing
                Err(CryptoError::UnsupportedAlgo)
            }
        }
    }
}

impl Stm32h7CryptoEngine {
    /// Software HMAC-SHA256 verification (fallback)
    ///
    /// Uses RustCrypto `hmac` and `sha2` crates for constant-time verification.
    ///
    /// # DO-178C Contract
    ///
    /// - Time complexity: O(msg_len + key_len)
    /// - WCET (engineering target): ~50 μs for 64-byte message @ 480 MHz
    /// - Constant-time: YES (w.r.t. key and tag comparison)
    fn verify_hmac_sw(&self, key: &[u8], msg: &[u8], tag: &[u8]) -> Result<(), CryptoError> {
        // Create HMAC instance with key
        let mut mac = HmacSha256::new_from_slice(key).map_err(|_| CryptoError::InvalidKey)?;

        // Update with message
        mac.update(msg);

        // Verify tag (constant-time comparison)
        mac.verify_slice(tag)
            .map_err(|_| CryptoError::VerificationFailed)
    }

    /// Software HMAC-SHA256 signing (fallback)
    ///
    /// Uses RustCrypto `hmac` and `sha2` crates.
    ///
    /// # DO-178C Contract
    ///
    /// - Time complexity: O(msg_len + key_len)
    /// - WCET (engineering target): ~50 μs for 64-byte message @ 480 MHz
    fn sign_hmac_sw(&self, key: &[u8], msg: &[u8], out: &mut [u8]) -> Result<usize, CryptoError> {
        // Create HMAC instance with key
        let mut mac = HmacSha256::new_from_slice(key).map_err(|_| CryptoError::InvalidKey)?;

        // Update with message
        mac.update(msg);

        // Finalize and write to output buffer
        let result = mac.finalize();
        let tag_bytes = result.into_bytes();
        out[..32].copy_from_slice(&tag_bytes);

        Ok(32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_keystore_dev_mode() {
        let keystore = Stm32h7KeyStore::new();

        // In dev mode (without secure-keys), should return test key
        #[cfg(not(feature = "secure-keys"))]
        {
            let selector = KeySelector {
                link_id: 0,
                purpose: KeyPurpose::Command,
            };
            let key = keystore.load_key(selector);
            assert!(key.is_ok());
            assert_eq!(key.unwrap().len(), 32);

            // Same key for different link_id values (single shared secret)
            let selector2 = KeySelector {
                link_id: 1,
                purpose: KeyPurpose::Command,
            };
            let key2 = keystore.load_key(selector2);
            assert!(key2.is_ok());
            assert_eq!(key.unwrap(), key2.unwrap());
        }

        // In production mode (with secure-keys), should return error (OTP not implemented)
        #[cfg(feature = "secure-keys")]
        {
            let selector = KeySelector {
                link_id: 0,
                purpose: KeyPurpose::Command,
            };
            let key = keystore.load_key(selector);
            assert!(matches!(key, Err(CryptoError::InvalidKey)));
        }
    }

    #[test]
    fn test_hmac_sign_and_verify() {
        let mut crypto = Stm32h7CryptoEngine::new();
        let key = b"test_key_32_bytes_long_padding!!";
        let msg = b"Hello, Aviate!";
        let mut tag = [0u8; 32];

        // Sign message
        let len = crypto
            .sign(CryptoAlgo::HmacSha256, key, msg, &mut tag)
            .expect("sign should succeed");
        assert_eq!(len, 32);

        // Verify signature
        crypto
            .verify(CryptoAlgo::HmacSha256, key, msg, &tag)
            .expect("verify should succeed");

        // Tamper with tag - verification should fail
        tag[0] ^= 0xFF;
        let result = crypto.verify(CryptoAlgo::HmacSha256, key, msg, &tag);
        assert!(matches!(result, Err(CryptoError::VerificationFailed)));
    }

    #[test]
    fn test_unsupported_algos() {
        let mut crypto = Stm32h7CryptoEngine::new();
        let key = b"test_key";
        let msg = b"test_msg";
        let tag = [0u8; 32];

        // AES-GCM not implemented yet
        let result = crypto.verify(CryptoAlgo::AesGcm128, key, msg, &tag);
        assert!(matches!(result, Err(CryptoError::UnsupportedAlgo)));

        // Ed25519 not implemented yet
        let result = crypto.verify(CryptoAlgo::Ed25519, key, msg, &tag);
        assert!(matches!(result, Err(CryptoError::UnsupportedAlgo)));
    }

    #[test]
    fn test_algo_preference() {
        let crypto = Stm32h7CryptoEngine::new();
        assert_eq!(crypto.algo(), CryptoAlgo::HmacSha256);
    }
}
