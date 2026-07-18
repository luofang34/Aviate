//! Deterministic test doubles for the security layer.
//!
//! A real HMAC engine is a board concern; these unit tests only need a
//! keyed function that is sensitive to the key and to every message byte,
//! so verification ORDERING, canonical coverage, and identity binding can
//! be exercised without a crypto dependency.

// Shared scaffolding: not every helper is exercised by every test module.
#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used, dead_code)]

use aviate_hal_io::security::{CryptoAlgo, CryptoEngine, CryptoError, KeySelector, KeyStore};
use aviate_link::command::{SignatureMeta, MAVLINK_SIGNATURE_TRAILER_LEN, MAX_SIGNED_FRAME_SIZE};

use crate::auth::SignedAuth;

/// The single shared secret every `link_id` resolves to in tests.
pub const TEST_KEY: [u8; 32] = [0x42; 32];

/// Deterministic stand-in for a hardware HMAC engine.
pub struct MockCrypto;

impl MockCrypto {
    /// Keyed FNV-style fold over key then message, splattered to 32 bytes.
    pub fn tag(key: &[u8], msg: &[u8]) -> [u8; 32] {
        let mut acc: u64 = 0xcbf2_9ce4_8422_2325;
        for &b in key.iter().chain(msg.iter()) {
            acc ^= b as u64;
            acc = acc.wrapping_mul(0x0000_0100_0000_01b3);
        }
        let mut out = [0u8; 32];
        for (i, slot) in out.iter_mut().enumerate() {
            *slot = (acc.rotate_left((i as u32) * 7) & 0xff) as u8;
        }
        out
    }
}

impl CryptoEngine for MockCrypto {
    fn algo(&self) -> CryptoAlgo {
        CryptoAlgo::HmacSha256
    }
    fn verify(
        &mut self,
        _algo: CryptoAlgo,
        _key: &[u8],
        _msg: &[u8],
        _tag: &[u8],
    ) -> Result<(), CryptoError> {
        Err(CryptoError::UnsupportedAlgo)
    }
    fn sign(
        &mut self,
        _algo: CryptoAlgo,
        key: &[u8],
        msg: &[u8],
        out: &mut [u8],
    ) -> Result<usize, CryptoError> {
        let tag = Self::tag(key, msg);
        let n = out.len().min(tag.len());
        out[..n].copy_from_slice(&tag[..n]);
        Ok(n)
    }
}

/// Key store that hands `TEST_KEY` to every selector.
pub struct MockKeyStore;

impl KeyStore for MockKeyStore {
    fn load_key(&self, _selector: KeySelector) -> Result<&'static [u8], CryptoError> {
        Ok(&TEST_KEY)
    }
}

/// A `SignedAuth` wired to the deterministic test doubles.
pub fn signed_auth() -> SignedAuth<MockKeyStore, MockCrypto> {
    SignedAuth::new(MockKeyStore, MockCrypto)
}

/// The correct 6-byte signature for `message` under `TEST_KEY`.
pub fn correct_sig(message: &[u8]) -> [u8; 6] {
    let tag = MockCrypto::tag(&TEST_KEY, message);
    let mut sig = [0u8; 6];
    sig.copy_from_slice(&tag[..6]);
    sig
}

/// Build a `SignatureMeta` whose `signed_message()` recovers exactly
/// `message` (with `sig` appended as the trailing signature bytes).
pub fn signed_meta(
    system_id: u8,
    component_id: u8,
    link_id: u8,
    timestamp: u64,
    message: &[u8],
    sig: [u8; 6],
) -> SignatureMeta {
    let mut raw_frame = [0u8; MAX_SIGNED_FRAME_SIZE];
    raw_frame[..message.len()].copy_from_slice(message);
    SignatureMeta {
        system_id,
        component_id,
        link_id,
        timestamp,
        sig,
        raw_frame,
        raw_frame_len: message.len() + MAVLINK_SIGNATURE_TRAILER_LEN,
    }
}

/// A `signed_meta` carrying the correct signature for `message`.
pub fn valid_meta(
    system_id: u8,
    component_id: u8,
    link_id: u8,
    timestamp: u64,
    message: &[u8],
) -> SignatureMeta {
    signed_meta(
        system_id,
        component_id,
        link_id,
        timestamp,
        message,
        correct_sig(message),
    )
}
