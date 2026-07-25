//! `sha256_48` interoperability against pymavlink-signed golden frames.
//!
//! pymavlink is an independent MAVLink implementation, so these tests pin
//! [`SignedAuth`]'s signature computation against a foreign signer rather
//! than against this crate's own code. An HMAC-based implementation fails
//! every one of them. Frames come from `gen_golden_vectors.py` (checked in
//! next to this file).
//!
//! Anti-replay, authorization, and reboot freshness are gateway concerns
//! and are covered end-to-end in `mavlink_interop.rs`.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_hal_io::security::{CryptoAlgo, CryptoEngine, CryptoError, KeySelector, KeyStore};
use aviate_link::command::{SignatureMeta, MAX_SIGNED_FRAME_SIZE};
use aviate_security::{AuthError, SignedAuth};
use sha2::{Digest, Sha256};

#[path = "golden/vectors.rs"]
mod vectors;
use vectors::{SIGNED_ARM_SYS1_COMP1, T0};

/// The signing secret the golden frames were produced with: 0x00..0x1f.
const TEST_SIGNING_KEY: [u8; 32] = {
    let mut key = [0u8; 32];
    let mut i = 0;
    while i < 32 {
        key[i] = i as u8;
        i += 1;
    }
    key
};

struct FixedKeyStore;

impl KeyStore for FixedKeyStore {
    fn load_key(&self, _selector: KeySelector) -> Result<&[u8], CryptoError> {
        Ok(&TEST_SIGNING_KEY)
    }
}

/// Real SHA-256 engine implementing the MAVLink keyed-prefix construction.
struct Sha256PrefixEngine;

impl CryptoEngine for Sha256PrefixEngine {
    fn algo(&self) -> CryptoAlgo {
        CryptoAlgo::Sha256KeyedPrefix
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
        algo: CryptoAlgo,
        key: &[u8],
        msg: &[u8],
        out: &mut [u8],
    ) -> Result<usize, CryptoError> {
        if algo != CryptoAlgo::Sha256KeyedPrefix {
            return Err(CryptoError::UnsupportedAlgo);
        }
        let mut hasher = Sha256::new();
        hasher.update(key);
        hasher.update(msg);
        let digest = hasher.finalize();
        let n = out.len().min(digest.len());
        out[..n].copy_from_slice(&digest[..n]);
        Ok(n)
    }
}

fn signed_auth() -> SignedAuth<FixedKeyStore, Sha256PrefixEngine> {
    SignedAuth::new(FixedKeyStore, Sha256PrefixEngine)
}

/// Build `SignatureMeta` from a captured signed frame, as a transport
/// would: identity from the header, link_id/timestamp/signature from the
/// trailing 13-byte signing block.
fn meta_from_frame(frame: &[u8]) -> SignatureMeta {
    let n = frame.len();
    let mut ts_bytes = [0u8; 8];
    ts_bytes[..6].copy_from_slice(&frame[n - 12..n - 6]);
    let mut sig = [0u8; 6];
    sig.copy_from_slice(&frame[n - 6..]);
    let mut raw_frame = [0u8; MAX_SIGNED_FRAME_SIZE];
    raw_frame[..n].copy_from_slice(frame);
    SignatureMeta {
        system_id: frame[5],
        component_id: frame[6],
        link_id: frame[n - 13],
        timestamp: u64::from_le_bytes(ts_bytes),
        sig,
        raw_frame,
        raw_frame_len: n,
    }
}

#[test]
fn pymavlink_signed_frame_verifies() {
    let meta = meta_from_frame(SIGNED_ARM_SYS1_COMP1);
    // Sanity: the golden block decodes to the generator's parameters.
    assert_eq!(meta.link_id, 5);
    assert_eq!(meta.timestamp, T0);
    assert_eq!((meta.system_id, meta.component_id), (1, 1));

    assert!(
        signed_auth().verify_frame(&meta).is_ok(),
        "pymavlink-signed frame must verify — sha256_48, not HMAC"
    );
}

#[test]
fn tampered_signature_rejected() {
    let mut meta = meta_from_frame(SIGNED_ARM_SYS1_COMP1);
    meta.sig[0] ^= 0xFF;
    assert!(matches!(
        signed_auth().verify_frame(&meta),
        Err(AuthError::InvalidSignature)
    ));
}

#[test]
fn tampered_covered_byte_rejected() {
    let mut meta = meta_from_frame(SIGNED_ARM_SYS1_COMP1);
    // param1 lives inside the signed coverage; flip one byte of it.
    meta.raw_frame[12] ^= 0x01;
    assert!(matches!(
        signed_auth().verify_frame(&meta),
        Err(AuthError::InvalidSignature)
    ));
}
