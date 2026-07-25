//! End-to-end admission against pymavlink-signed golden frames.
//!
//! Drives the full MAVLink profile — [`MavlinkAdmission`] (parse +
//! `sha256_48` verify) into [`CommandGateway`] (authorize + anti-replay +
//! stamp) — against frames produced by pymavlink, an independent MAVLink
//! implementation. Frames come from `gen_golden_vectors.py`.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_hal_io::security::{CryptoAlgo, CryptoEngine, CryptoError, KeySelector, KeyStore};
use aviate_hal_io::SystemCommand;
use aviate_link::errors::LinkError;
use aviate_link::mavlink::protocol::ParseError;
use aviate_link::mavlink::LocalAddress;
use aviate_security::admission::MavlinkAdmission;
use aviate_security::{
    AuthError, CommandGateway, CommandSource, FreshnessConfig, GatewayError, Principal,
    SourcePolicy, TrustedCounter, NEW_STREAM_MAX_AGE_10US,
};
use sha2::{Digest, Sha256};

#[path = "golden/vectors.rs"]
mod vectors;
use vectors::{SIGNED_ARM_SYS1_COMP1, SIGNED_ARM_SYS2_COMP1, SIGNED_DISARM_SYS1_COMP1, T0};

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

fn admission() -> MavlinkAdmission<FixedKeyStore, Sha256PrefixEngine> {
    MavlinkAdmission::new(
        FixedKeyStore,
        Sha256PrefixEngine,
        LocalAddress {
            system_id: 1,
            component_id: 1,
        },
    )
}

/// A gateway authorizing the golden credential (slot 5, identity (1,1) →
/// GCS), seeded with `trusted_ts` for first-frame freshness.
fn gateway(trusted_ts: u64) -> CommandGateway {
    let mut policy = SourcePolicy::new();
    policy
        .bind(Principal::mavlink(1, 1, 5), CommandSource::GcsDatalink)
        .expect("bind credential");
    CommandGateway::new(
        policy,
        FreshnessConfig {
            initial_trusted_counter: TrustedCounter::Rtc(trusted_ts),
            new_stream_max_age: NEW_STREAM_MAX_AGE_10US,
            counter_tick_us: 10,
            max_skew: NEW_STREAM_MAX_AGE_10US,
        },
    )
}

/// X.25 CRC over `data`, extended with `crc_extra` — a local copy so a
/// tampered frame can be re-sealed without touching the crate under test.
fn mavlink_crc(data: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data.iter().chain(core::iter::once(&crc_extra)) {
        let tmp = b ^ (crc as u8);
        let tmp = tmp ^ (tmp << 4);
        crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    }
    crc
}

const COMMAND_LONG_CRC_EXTRA: u8 = 152;

#[test]
fn pymavlink_signed_arm_admits_end_to_end() {
    let mut adm = admission();
    let mut gw = gateway(T0);
    let claim = adm
        .authenticate(SIGNED_ARM_SYS1_COMP1)
        .expect("pymavlink-signed ARM must authenticate — sha256_48 interop");
    let verified = gw.admit(claim, 42_000).expect("bound principal admits");
    assert!(matches!(verified.command(), SystemCommand::Arm));
    let receipt = verified.receipt();
    assert_eq!(receipt.source(), CommandSource::GcsDatalink);
    assert_eq!(receipt.sequence(), T0);
    assert_eq!(receipt.received_at_us(), 42_000);
}

#[test]
fn replayed_golden_frame_rejected_but_stream_continues() {
    let mut adm = admission();
    let mut gw = gateway(T0);

    let first = adm.authenticate(SIGNED_ARM_SYS1_COMP1).unwrap();
    assert!(gw.admit(first, 1_000).is_ok());

    // Byte-exact replay of the captured ARM frame.
    let replay = adm.authenticate(SIGNED_ARM_SYS1_COMP1).unwrap();
    assert!(matches!(
        gw.admit(replay, 2_000),
        Err(GatewayError::Auth(AuthError::ReplayAttack))
    ));

    // The stream's next frame (timestamp T0+1) still admits.
    let disarm = adm.authenticate(SIGNED_DISARM_SYS1_COMP1).unwrap();
    let verified = gw
        .admit(disarm, 3_000)
        .expect("next frame in stream admits after a replay attempt");
    assert!(matches!(verified.command(), SystemCommand::Disarm));
}

/// Payload/signature substitution at the byte level: flip ARM's param1 to
/// zero (turning it into a DISARM) and re-seal the CRC so the frame
/// parses. The signature was computed over the original bytes, so
/// admission MUST fail — the decoded command cannot ride another frame's
/// signature.
#[test]
fn payload_substitution_with_valid_crc_rejected_by_signature() {
    let mut tampered = SIGNED_ARM_SYS1_COMP1.to_vec();
    // param1 (1.0f32 LE = 00 00 80 3F) is payload offset 0 → frame bytes
    // 10..14. Zero it: the frame now claims param1 = 0 (disarm).
    tampered[12] = 0x00;
    tampered[13] = 0x00;

    // Re-seal the CRC (over payload_len..payload, plus crc_extra) so the
    // tamper is invisible to the parser.
    let payload_end = 10 + 30;
    let crc = mavlink_crc(&tampered[1..payload_end], COMMAND_LONG_CRC_EXTRA);
    tampered[payload_end] = (crc & 0xFF) as u8;
    tampered[payload_end + 1] = (crc >> 8) as u8;

    // It decodes as Disarm, but the signature (over the original bytes)
    // no longer matches.
    assert!(matches!(
        admission().authenticate(&tampered),
        Err(GatewayError::Auth(AuthError::InvalidSignature))
    ));
}

#[test]
fn tampered_signature_bytes_rejected() {
    let mut tampered = SIGNED_ARM_SYS1_COMP1.to_vec();
    let last = tampered.len() - 1;
    tampered[last] ^= 0xFF;
    assert!(matches!(
        admission().authenticate(&tampered),
        Err(GatewayError::Auth(AuthError::InvalidSignature))
    ));
}

#[test]
fn tampered_payload_without_crc_fix_fails_parse() {
    let mut tampered = SIGNED_ARM_SYS1_COMP1.to_vec();
    tampered[12] = 0x00;
    assert!(matches!(
        admission().authenticate(&tampered),
        Err(GatewayError::Link(LinkError::Parse(
            ParseError::CrcMismatch
        )))
    ));
}

/// Reboot-replay defense: a receiver restarted with a trusted counter
/// ahead of the captured traffic rejects the old frame's attempt to start
/// a "new" stream.
#[test]
fn captured_frame_rejected_after_reboot_with_trusted_counter() {
    let mut adm = admission();
    let mut gw = gateway(T0 + 2 * NEW_STREAM_MAX_AGE_10US);
    let claim = adm.authenticate(SIGNED_ARM_SYS1_COMP1).unwrap();
    assert!(matches!(
        gw.admit(claim, 1_000),
        Err(GatewayError::Auth(AuthError::StaleNewStream { .. }))
    ));
}

/// The sys=2 frame is authentically signed with the shared link-5 key, but
/// the credential names identity (1, 1): possession of the key does not
/// let the holder present another identity.
#[test]
fn shared_key_cannot_impersonate_unbound_identity() {
    let mut adm = admission();
    let claim = adm.authenticate(SIGNED_ARM_SYS2_COMP1).unwrap();
    assert!(matches!(
        gateway(T0).admit(claim, 1_000),
        Err(GatewayError::Auth(AuthError::UnauthorizedSource))
    ));

    // Proof the rejection was authorization, not the signature: a policy
    // whose credential names (2, 1) admits the same bytes.
    let mut adm = admission();
    let claim = adm.authenticate(SIGNED_ARM_SYS2_COMP1).unwrap();
    let mut policy = SourcePolicy::new();
    policy
        .bind(Principal::mavlink(2, 1, 5), CommandSource::Offboard)
        .expect("bind");
    let mut gw = CommandGateway::new(
        policy,
        FreshnessConfig {
            initial_trusted_counter: TrustedCounter::Rtc(T0),
            new_stream_max_age: NEW_STREAM_MAX_AGE_10US,
            counter_tick_us: 10,
            max_skew: NEW_STREAM_MAX_AGE_10US,
        },
    );
    let verified = gw
        .admit(claim, 1_000)
        .expect("same bytes admit under the credential that names sys=2");
    assert_eq!(verified.receipt().source(), CommandSource::Offboard);
}
