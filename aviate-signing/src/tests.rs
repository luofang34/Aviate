//! Pins the signing primitive against a frame signed by pymavlink.
//!
//! The frame and its expected tag come from an independent MAVLink
//! implementation, so these assert interoperability rather than
//! self-consistency: a wrong construction (HMAC instead of keyed prefix)
//! or a transposed key/message order fails every case here, on the host,
//! for the same function the flight build calls.

use super::*;

/// Key 0x00..0x1f, matching the generator in
/// `aviate-security/tests/gen_golden_vectors.py`.
const KEY: [u8; 32] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
];

/// A pymavlink-signed COMPONENT_ARM_DISARM frame: everything the
/// signature covers, i.e. the frame minus its trailing 6 signature bytes
/// (link_id and the 48-bit timestamp are inside the coverage).
const COVERED: &[u8] = &[
    0xFD, 0x1E, 0x01, 0x00, 0x00, 0x01, 0x01, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x01, 0x8A, 0x9F, 0x05, 0xD2, 0x02, 0x96, 0x49, 0x00,
    0x00,
];

/// The 48 bits pymavlink put on the wire for `COVERED`.
const EXPECTED_TAG: [u8; MAVLINK_SIGNATURE_LEN] = [0x26, 0x33, 0xF7, 0x37, 0x16, 0x14];

#[test]
fn matches_a_pymavlink_signature() {
    let digest = sha256_keyed_prefix(&KEY, COVERED);
    assert_eq!(
        &digest[..MAVLINK_SIGNATURE_LEN],
        &EXPECTED_TAG,
        "the shipped primitive must reproduce pymavlink's tag"
    );
}

#[test]
fn key_and_message_order_is_load_bearing() {
    // The transposition that would otherwise reach the aircraft: it
    // compiles, it is still SHA-256, and it verifies nothing a real
    // ground station sends.
    let transposed = sha256_keyed_prefix(COVERED, &KEY);
    assert_ne!(&transposed[..MAVLINK_SIGNATURE_LEN], &EXPECTED_TAG);
}

#[test]
fn a_correct_tag_verifies() {
    assert!(verify_sha256_keyed_prefix(
        &KEY,
        COVERED,
        &EXPECTED_TAG,
        MAVLINK_SIGNATURE_LEN
    ));
}

#[test]
fn a_tampered_tag_is_refused() {
    let mut tag = EXPECTED_TAG;
    tag[0] ^= 0x01;
    assert!(!verify_sha256_keyed_prefix(
        &KEY,
        COVERED,
        &tag,
        MAVLINK_SIGNATURE_LEN
    ));
}

#[test]
fn a_tampered_covered_byte_is_refused() {
    let mut covered = [0u8; 49];
    covered.copy_from_slice(COVERED);
    covered[10] ^= 0x01;
    assert!(!verify_sha256_keyed_prefix(
        &KEY,
        &covered,
        &EXPECTED_TAG,
        MAVLINK_SIGNATURE_LEN
    ));
}

#[test]
fn a_short_tag_is_refused_rather_than_folding_to_a_match() {
    // An empty tag zips against nothing, so the fold yields 0 and a naive
    // comparison reports success — verification that accepts anything.
    for len in 0..MAVLINK_SIGNATURE_LEN {
        assert!(
            !verify_sha256_keyed_prefix(&KEY, COVERED, &EXPECTED_TAG[..len], MAVLINK_SIGNATURE_LEN),
            "a {len}-byte tag must not verify"
        );
    }
}

#[test]
fn a_wrong_key_is_refused() {
    let mut key = KEY;
    key[0] ^= 0x01;
    assert!(!verify_sha256_keyed_prefix(
        &key,
        COVERED,
        &EXPECTED_TAG,
        MAVLINK_SIGNATURE_LEN
    ));
}
