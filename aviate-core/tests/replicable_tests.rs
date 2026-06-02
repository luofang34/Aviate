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
use aviate_core::replicable::{copy_into, Replicable};
use aviate_core::types::{Meters, MetersPerSecond, RadiansPerSecond};

#[test]
fn copy_into_full_slice_when_buffer_fits() {
    let mut buf = [0u8; 8];
    let n = copy_into(&mut buf, 0, &[0xAA, 0xBB, 0xCC, 0xDD]);
    assert_eq!(n, 4);
    assert_eq!(buf, [0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0]);
}

#[test]
fn multirotor_runtime_state_encodes_full_length_and_is_deterministic() {
    // The controller runtime state participates in cross-channel
    // replication too; witness its `Replicable::encode_canonical`
    // writes exactly ENCODED_LEN bytes and is byte-stable for two
    // default clones (same contract as EkfState / KernelState).
    use aviate_core::control::multirotor::MultirotorRuntimeState;
    let len = <MultirotorRuntimeState as Replicable>::ENCODED_LEN;
    let mut buf_a = [0u8; 64];
    let mut buf_b = [0u8; 64];
    let na = MultirotorRuntimeState::default().encode_canonical(&mut buf_a);
    let nb = MultirotorRuntimeState::default().encode_canonical(&mut buf_b);
    assert_eq!(
        na, len,
        "encode_canonical must write exactly ENCODED_LEN bytes"
    );
    assert_eq!(na, nb);
    assert_eq!(
        buf_a[..na],
        buf_b[..nb],
        "two default clones must encode byte-equal"
    );
}

#[test]
fn copy_into_truncates_when_buffer_runs_out() {
    let mut buf = [0u8; 3];
    let n = copy_into(&mut buf, 0, &[1, 2, 3, 4, 5]);
    assert_eq!(n, 3);
    assert_eq!(buf, [1, 2, 3]);
}

#[test]
fn copy_into_writes_at_offset() {
    let mut buf = [0u8; 8];
    let n = copy_into(&mut buf, 4, &[0x10, 0x20]);
    assert_eq!(n, 2);
    assert_eq!(buf, [0, 0, 0, 0, 0x10, 0x20, 0, 0]);
}

#[test]
fn copy_into_empty_input_is_a_no_op() {
    let mut buf = [0u8; 4];
    let n = copy_into(&mut buf, 0, &[]);
    assert_eq!(n, 0);
    assert_eq!(buf, [0u8; 4]);
}

#[test]
fn copy_into_no_op_when_offset_at_end() {
    let mut buf = [0u8; 4];
    let n = copy_into(&mut buf, 4, &[1, 2]);
    assert_eq!(n, 0);
    assert_eq!(buf, [0u8; 4]);
}

#[test]
fn copy_into_no_op_when_offset_past_end() {
    // saturating_sub ensures no panic when offset > buf.len()
    let mut buf = [0u8; 4];
    let n = copy_into(&mut buf, 8, &[1, 2]);
    assert_eq!(n, 0);
}

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

#[test]
fn kernel_state_default_encodes_full_length() {
    // Default kernel state with EkfState + NoControllerState as the
    // generic parameters. ENCODED_LEN sums every leaf field.
    use aviate_core::kernel::state::KernelState;
    let s: KernelState = KernelState::default();
    let mut buf = [0u8; <KernelState as Replicable>::ENCODED_LEN];
    let n = s.encode_canonical(&mut buf);
    assert_eq!(
        n,
        <KernelState as Replicable>::ENCODED_LEN,
        "encode_canonical writes exactly ENCODED_LEN bytes"
    );
}

#[test]
fn kernel_state_encoded_len_is_sum_of_field_lens() {
    // Cross-check: KernelState's ENCODED_LEN must equal the sum of
    // each leaf field's ENCODED_LEN. A regression where a field
    // gets dropped from the const expression would make this fail
    // (assuming a corresponding mismatch between encoded byte count
    // and ENCODED_LEN).
    use aviate_core::checks::KernelChecks;
    use aviate_core::control::runtime::NoControllerState;
    use aviate_core::control::{ConfigMode, ControlLawV1};
    use aviate_core::ekf::EkfState;
    use aviate_core::fault::FaultFlags;
    use aviate_core::kernel::state::KernelState;
    use aviate_core::kernel_types::{InitState, TimingStats};
    use aviate_core::mixer::{ActuatorFallbackState, ActuatorState};

    let expected = <InitState as Replicable>::ENCODED_LEN
        + <ConfigMode as Replicable>::ENCODED_LEN
        + <FaultFlags as Replicable>::ENCODED_LEN
        + <ControlLawV1 as Replicable>::ENCODED_LEN
        + <KernelChecks as Replicable>::ENCODED_LEN
        + <ActuatorState as Replicable>::ENCODED_LEN
        + <TimingStats as Replicable>::ENCODED_LEN
        + <EkfState as Replicable>::ENCODED_LEN
        + <ActuatorFallbackState as Replicable>::ENCODED_LEN
        + <NoControllerState as Replicable>::ENCODED_LEN;

    assert_eq!(<KernelState as Replicable>::ENCODED_LEN, expected);
}

#[test]
fn kernel_state_two_default_clones_encode_byte_equal() {
    use aviate_core::kernel::state::KernelState;
    let a: KernelState = KernelState::default();
    let b = a.clone();
    let mut buf_a = [0u8; <KernelState as Replicable>::ENCODED_LEN];
    let mut buf_b = [0u8; <KernelState as Replicable>::ENCODED_LEN];
    a.encode_canonical(&mut buf_a);
    b.encode_canonical(&mut buf_b);
    assert_eq!(
        buf_a, buf_b,
        "byte-equal kernel states must produce byte-equal encodings"
    );
}

#[test]
fn kernel_state_mutating_init_state_changes_encoding() {
    use aviate_core::kernel::state::KernelState;
    use aviate_core::kernel_types::InitState;

    let baseline: KernelState = KernelState::default();
    let mut mutated: KernelState = KernelState::default();
    mutated.init_state = InitState::Armed;

    let mut buf_a = [0u8; <KernelState as Replicable>::ENCODED_LEN];
    let mut buf_b = [0u8; <KernelState as Replicable>::ENCODED_LEN];
    baseline.encode_canonical(&mut buf_a);
    mutated.encode_canonical(&mut buf_b);
    assert_ne!(
        buf_a, buf_b,
        "init_state field must contribute to the kernel-state encoding"
    );
}

#[test]
fn kernel_state_mutating_faults_changes_encoding() {
    use aviate_core::fault::FaultFlags;
    use aviate_core::kernel::state::KernelState;

    let baseline: KernelState = KernelState::default();
    let mut mutated: KernelState = KernelState::default();
    mutated.faults |= FaultFlags::ALL_IMU_FAILED;

    let mut buf_a = [0u8; <KernelState as Replicable>::ENCODED_LEN];
    let mut buf_b = [0u8; <KernelState as Replicable>::ENCODED_LEN];
    baseline.encode_canonical(&mut buf_a);
    mutated.encode_canonical(&mut buf_b);
    assert_ne!(
        buf_a, buf_b,
        "faults field must contribute to the kernel-state encoding"
    );
}

#[test]
fn actuator_health_each_variant_encodes_distinct_tag() {
    // Encode all five ActuatorHealth variants and verify each gets a
    // distinct single-byte tag. This exercises every match arm in
    // mixer/replicable.rs's ActuatorHealth::encode_canonical (Good=0,
    // Degraded=1, Failed=2, Stuck=3, Unknown=4) — a regression that
    // collapses two variants to the same tag would defeat snapshot
    // discrimination across channels.
    use aviate_core::mixer::ActuatorHealth;
    let mut tags = [0u8; 5];
    for (i, h) in [
        ActuatorHealth::Good,
        ActuatorHealth::Degraded,
        ActuatorHealth::Failed,
        ActuatorHealth::Stuck,
        ActuatorHealth::Unknown,
    ]
    .iter()
    .enumerate()
    {
        let mut buf = [0u8; 1];
        let n = h.encode_canonical(&mut buf);
        assert_eq!(n, 1, "ActuatorHealth must encode exactly 1 byte");
        tags[i] = buf[0];
    }
    assert_eq!(tags, [0, 1, 2, 3, 4]);
}

#[test]
fn actuator_state_some_actual_changes_encoding_vs_none() {
    // Drive the Some(arr) branch in ActuatorState::encode_canonical:
    // when actual is Some, the discriminant is 1 and the payload
    // f32 array follows; when None, discriminant is 0 and zero
    // payload bytes are written. Replication needs to distinguish
    // "we have feedback" from "we don't" byte-distinctly.
    use aviate_core::mixer::ActuatorState;
    use aviate_core::types::Normalized;

    let baseline = ActuatorState::default(); // actual = None
    let mut with_actual = ActuatorState::default();
    with_actual.actual = Some([Normalized(0.5); aviate_core::mixer::MAX_ACTUATORS]);

    let mut buf_a = [0u8; <ActuatorState as Replicable>::ENCODED_LEN];
    let mut buf_b = [0u8; <ActuatorState as Replicable>::ENCODED_LEN];
    let na = baseline.encode_canonical(&mut buf_a);
    let nb = with_actual.encode_canonical(&mut buf_b);
    assert_eq!(na, nb, "fixed-width invariant: same length for Some/None");
    assert_ne!(
        buf_a, buf_b,
        "Some(actual) must produce a byte-distinct encoding from None"
    );
}

#[test]
fn actuator_state_timestamp_source_each_variant_changes_encoding() {
    // Drive the TimeSource match in ActuatorState::encode_canonical
    // through its Gps and Ptp arms. Internal is the default and is
    // covered by the all-defaults baseline above.
    use aviate_core::mixer::ActuatorState;
    use aviate_core::time::{TimeSource, Timestamp};

    let mut internal = ActuatorState::default();
    internal.timestamp = Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    };
    let mut gps = ActuatorState::default();
    gps.timestamp = Timestamp {
        ticks: 0,
        source: TimeSource::Gps,
    };
    let mut ptp = ActuatorState::default();
    ptp.timestamp = Timestamp {
        ticks: 0,
        source: TimeSource::Ptp,
    };

    let mut bi = [0u8; <ActuatorState as Replicable>::ENCODED_LEN];
    let mut bg = [0u8; <ActuatorState as Replicable>::ENCODED_LEN];
    let mut bp = [0u8; <ActuatorState as Replicable>::ENCODED_LEN];
    internal.encode_canonical(&mut bi);
    gps.encode_canonical(&mut bg);
    ptp.encode_canonical(&mut bp);

    assert_ne!(bi, bg, "Internal vs Gps must produce distinct encodings");
    assert_ne!(bg, bp, "Gps vs Ptp must produce distinct encodings");
    assert_ne!(bi, bp, "Internal vs Ptp must produce distinct encodings");
}
