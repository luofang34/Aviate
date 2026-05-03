//! TST-REPL-101: structural witness for the `Replicable` contract.
//!
//! Verifies that the trait's deterministic-encoding invariant holds
//! for the in-tree implementations (`EkfState`, `NoControllerState`)
//! that are baked into spec §16 cross-channel snapshot replication.
//! The contract:
//!
//!   1. `ENCODED_LEN` is a per-type compile-time constant; each
//!      instance writes exactly that many bytes when given a
//!      sufficient buffer.
//!   2. Two byte-equal states produce byte-equal encodings.
//!   3. Mutating any field changes at least one byte of the encoding
//!      (a regression where a field is silently dropped from the
//!      encoding would defeat byte-equality replication).
//!   4. A too-small buffer truncates without panic; the returned
//!      byte count tells the caller how much actually landed.

use aviate_core::ekf::EkfState;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::replicable::Replicable;
use aviate_core::types::{Meters, MetersPerSecond, RadiansPerSecond};

#[test]
fn ekf_state_encoded_len_matches_documented_size() {
    // 22 vector f32s + 18*18 = 324 covariance f32s = 346 floats =
    // 1384 bytes; plus 2 boolean latches = 1386 bytes. Mismatched
    // const ENCODED_LEN would surface a manual edit drift between
    // the doc-comment and the impl.
    assert_eq!(EkfState::ENCODED_LEN, 1386);
}

#[test]
fn ekf_state_default_encodes_full_length() {
    let state = EkfState::default();
    let mut buf = [0u8; EkfState::ENCODED_LEN];
    let n = state.encode_canonical(&mut buf);
    assert_eq!(
        n,
        EkfState::ENCODED_LEN,
        "encode_canonical must write exactly ENCODED_LEN bytes when buffer is sufficient"
    );
}

#[test]
fn ekf_state_two_default_clones_encode_byte_equal() {
    let a = EkfState::default();
    let b = a.clone();
    let mut buf_a = [0u8; EkfState::ENCODED_LEN];
    let mut buf_b = [0u8; EkfState::ENCODED_LEN];
    let na = a.encode_canonical(&mut buf_a);
    let nb = b.encode_canonical(&mut buf_b);
    assert_eq!(na, nb);
    assert_eq!(
        &buf_a[..na],
        &buf_b[..nb],
        "byte-equal states must produce byte-equal encodings"
    );
}

#[test]
fn ekf_state_mutating_position_changes_encoding() {
    let baseline = EkfState::default();
    let mut mutated = EkfState::default();
    mutated.pos = Vector3::new(Meters(1.0), Meters(2.0), Meters(3.0));

    let mut buf_a = [0u8; EkfState::ENCODED_LEN];
    let mut buf_b = [0u8; EkfState::ENCODED_LEN];
    let na = baseline.encode_canonical(&mut buf_a);
    let nb = mutated.encode_canonical(&mut buf_b);
    assert_eq!(na, nb);
    assert_ne!(
        &buf_a[..na],
        &buf_b[..nb],
        "changing position field must change the encoding (catches a regression where pos is dropped from encode_canonical)"
    );
}

#[test]
fn ekf_state_mutating_quat_changes_encoding() {
    let baseline = EkfState::default();
    let mut mutated = EkfState::default();
    mutated.quat = Quaternion::new(0.5, 0.5, 0.5, 0.5);

    let mut buf_a = [0u8; EkfState::ENCODED_LEN];
    let mut buf_b = [0u8; EkfState::ENCODED_LEN];
    baseline.encode_canonical(&mut buf_a);
    mutated.encode_canonical(&mut buf_b);
    assert_ne!(buf_a, buf_b);
}

#[test]
fn ekf_state_mutating_initialized_flag_changes_encoding() {
    let baseline = EkfState::default();
    let mut mutated = EkfState::default();
    mutated.initialized = true;

    let mut buf_a = [0u8; EkfState::ENCODED_LEN];
    let mut buf_b = [0u8; EkfState::ENCODED_LEN];
    baseline.encode_canonical(&mut buf_a);
    mutated.encode_canonical(&mut buf_b);
    assert_ne!(
        buf_a, buf_b,
        "boolean init latch must contribute to the encoding"
    );
}

#[test]
fn ekf_state_mutating_quat_fault_flag_changes_encoding() {
    let baseline = EkfState::default();
    let mut mutated = EkfState::default();
    mutated.quat_fault = true;

    let mut buf_a = [0u8; EkfState::ENCODED_LEN];
    let mut buf_b = [0u8; EkfState::ENCODED_LEN];
    baseline.encode_canonical(&mut buf_a);
    mutated.encode_canonical(&mut buf_b);
    assert_ne!(
        buf_a, buf_b,
        "quat_fault latch must contribute to the encoding"
    );
}

#[test]
fn ekf_state_mutating_velocity_changes_encoding() {
    let baseline = EkfState::default();
    let mut mutated = EkfState::default();
    mutated.vel = Vector3::new(
        MetersPerSecond(0.0),
        MetersPerSecond(0.0),
        MetersPerSecond(7.5),
    );

    let mut buf_a = [0u8; EkfState::ENCODED_LEN];
    let mut buf_b = [0u8; EkfState::ENCODED_LEN];
    baseline.encode_canonical(&mut buf_a);
    mutated.encode_canonical(&mut buf_b);
    assert_ne!(buf_a, buf_b);
}

#[test]
fn ekf_state_mutating_gyro_bias_changes_encoding() {
    let baseline = EkfState::default();
    let mut mutated = EkfState::default();
    mutated.gyro_bias = Vector3::new(
        RadiansPerSecond(0.01),
        RadiansPerSecond(-0.01),
        RadiansPerSecond(0.0),
    );

    let mut buf_a = [0u8; EkfState::ENCODED_LEN];
    let mut buf_b = [0u8; EkfState::ENCODED_LEN];
    baseline.encode_canonical(&mut buf_a);
    mutated.encode_canonical(&mut buf_b);
    assert_ne!(
        buf_a, buf_b,
        "gyro_bias must contribute to the encoding (snapshot replication needs to carry trim)"
    );
}

#[test]
fn ekf_state_truncates_too_small_buffer_without_panic() {
    let state = EkfState::default();
    let mut tiny = [0u8; 16];
    let n = state.encode_canonical(&mut tiny);
    assert_eq!(
        n, 16,
        "truncated buffer must be written full and return its actual length"
    );
}

#[test]
fn no_controller_state_zero_byte_encoding() {
    use aviate_core::control::runtime::NoControllerState;
    assert_eq!(NoControllerState::ENCODED_LEN, 0);
    let mut buf = [0u8; 8];
    let n = NoControllerState.encode_canonical(&mut buf);
    assert_eq!(n, 0, "unit-struct runtime state writes zero bytes");
    assert_eq!(buf, [0u8; 8], "untouched buffer stays at its initial value");
}
