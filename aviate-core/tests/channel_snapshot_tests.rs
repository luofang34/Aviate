//! TST-CCS-101 / 102 / 103: end-to-end witness for
//! [`AviateKernelImpl::project_for_cross_channel`] against a
//! complete kernel built from production traits.
//!
//! Unit tests for the comparison primitive itself live in
//! `aviate-core/src/kernel/snapshot.rs::tests` — they exercise
//! `ChannelSnapshot::agrees_with` in isolation. This file integrates
//! the projection method against `AviateKernelImpl<Ekf,
//! MultirotorController, QuadXMixer, Sanitizer>`, the production
//! algorithm bundle.

use aviate_core::checks::{KernelChecks, PreArmFlags};
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::ekf::Ekf;
use aviate_core::fault::FaultFlags;
use aviate_core::kernel::config::ResolvedKernelConfig;
use aviate_core::kernel::pipeline::KernelPipeline;
use aviate_core::kernel::state::KernelState;
use aviate_core::kernel::AviateKernelImpl;
use aviate_core::mixer::{ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::replicable::Replicable;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::ChannelId;

fn fake_ts() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

type ProdKernel = AviateKernelImpl<Ekf, MultirotorController, QuadXMixer, Sanitizer>;
type ProdState =
    KernelState<aviate_core::ekf::EkfState, aviate_core::control::runtime::NoControllerState>;

fn make_kernel() -> ProdKernel {
    AviateKernelImpl {
        pipeline: KernelPipeline::new(
            Ekf::default(),
            MultirotorController::default(),
            QuadXMixer {
                timestamp_source: fake_ts,
            },
            Sanitizer,
        ),
        state: KernelState::new(KernelChecks::with_pre_arm_required(PreArmFlags::empty())),
        cfg: ResolvedKernelConfig {
            mode_config: ModeConfig {
                mode: aviate_core::control::ConfigMode::Hover,
                groups: &[],
            },
            ..Default::default()
        },
    }
}

#[test]
fn project_byte_stable_across_two_default_kernels() {
    // TST-CCS-101: two pipelines built identically with identical
    // KernelState produce snapshots that `agrees_with`.
    let k1 = make_kernel();
    let k2 = make_kernel();
    let mut buf1 = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let mut buf2 = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let s1 = k1.project_for_cross_channel(42, ChannelId::PRIMARY, &mut buf1);
    let s2 = k2.project_for_cross_channel(43, ChannelId::SECONDARY, &mut buf2);
    assert!(
        s1.agrees_with(&s2),
        "byte-equal kernels with byte-equal state ⇒ agreeing snapshots"
    );
}

#[test]
fn project_changes_when_state_mutates() {
    // TST-CCS-102: mutating any kernel-state field changes the
    // snapshot bytes. Catches a regression where a leaf field is
    // silently dropped from `KernelState::encode_canonical`.
    let k1 = make_kernel();
    let mut k2 = make_kernel();
    k2.state.faults |= FaultFlags::ALL_IMU_FAILED;

    let mut buf1 = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let mut buf2 = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let s1 = k1.project_for_cross_channel(0, ChannelId::PRIMARY, &mut buf1);
    let s2 = k2.project_for_cross_channel(0, ChannelId::SECONDARY, &mut buf2);
    assert!(
        !s1.agrees_with(&s2),
        "mutating faults must change the snapshot bytes (peer detects state divergence)"
    );
    assert_eq!(
        s1.algorithm_identity_hash, s2.algorithm_identity_hash,
        "algorithm-identity hash unchanged — divergence is in state bytes only"
    );
}

#[test]
fn algorithm_hash_matches_pipeline() {
    // TST-CCS-103: snapshot's `algorithm_identity_hash` equals the
    // direct pipeline call. Witnesses the "what code is running"
    // half of the agreement.
    let k = make_kernel();
    let mut buf = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let snap = k.project_for_cross_channel(7, ChannelId::PRIMARY, &mut buf);
    assert_eq!(
        snap.algorithm_identity_hash,
        k.pipeline.algorithm_identity_hash(),
        "ChannelSnapshot's identity hash must match the pipeline's"
    );
}

#[test]
fn project_truncates_safely_with_short_buffer() {
    // Defensive: a too-small buffer truncates without panic. The
    // truncated snapshot will fail `agrees_with` against any
    // full-size peer because of the length check — exactly the
    // failure mode we want (caller bug surfaces as cross-channel
    // disagreement, not as silent corruption).
    let k = make_kernel();
    let mut tiny = [0u8; 16];
    let snap = k.project_for_cross_channel(0, ChannelId::PRIMARY, &mut tiny);
    assert_eq!(
        snap.state_bytes.len(),
        16,
        "short buffer truncates to its capacity"
    );

    let mut full = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let full_snap = k.project_for_cross_channel(0, ChannelId::SECONDARY, &mut full);
    assert!(
        !snap.agrees_with(&full_snap),
        "truncated snapshot SHALL NOT agree with full snapshot — \
         length mismatch is the safe-fail signal"
    );
}

#[test]
fn check_lockstep_agreement_enters_when_three_kernels_agree() {
    // TST-CCS-105: three independent kernels at the same default
    // state run the one-call gate API and all decide Enter.
    use aviate_core::kernel::snapshot::LockstepDecision;
    let k_local = make_kernel();
    let k_p1 = make_kernel();
    let k_p2 = make_kernel();

    let mut buf_p1 = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let mut buf_p2 = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let snap_p1 = k_p1.project_for_cross_channel(100, ChannelId::SECONDARY, &mut buf_p1);
    let snap_p2 = k_p2.project_for_cross_channel(101, ChannelId::TERTIARY, &mut buf_p2);

    let mut buf_local = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let decision = k_local.check_lockstep_agreement(
        99,
        ChannelId::PRIMARY,
        &mut buf_local,
        &[Some(snap_p1), Some(snap_p2)],
        2, // both peers required
    );
    assert_eq!(decision, LockstepDecision::Enter);
}

#[test]
fn check_lockstep_agreement_refuses_when_peer_config_diverges() {
    // Two kernels with byte-equal state but divergent ResolvedKernelConfig
    // (different command_timeout_ms) — gate surfaces RefuseConfigMismatch.
    use aviate_core::kernel::snapshot::LockstepDecision;
    let k_local = make_kernel();
    let mut k_peer = make_kernel();
    k_peer.cfg.command_timeout_ms = k_local.cfg.command_timeout_ms.wrapping_add(1);

    let mut buf_peer = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let snap_peer = k_peer.project_for_cross_channel(0, ChannelId::SECONDARY, &mut buf_peer);

    let mut buf_local = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let decision = k_local.check_lockstep_agreement(
        0,
        ChannelId::PRIMARY,
        &mut buf_local,
        &[Some(snap_peer)],
        1,
    );
    assert_eq!(
        decision,
        LockstepDecision::RefuseConfigMismatch {
            peer: ChannelId::SECONDARY,
        },
        "diverged config must surface as RefuseConfigMismatch with peer's ChannelId"
    );
}

#[test]
fn check_lockstep_agreement_refuses_when_peer_state_diverges() {
    // TST-CCS-106: one peer mutates its state; the gate surfaces
    // RefuseStateMismatch with the offending peer's id.
    use aviate_core::fault::FaultFlags;
    use aviate_core::kernel::snapshot::LockstepDecision;
    let k_local = make_kernel();
    let k_p_good = make_kernel();
    let mut k_p_bad = make_kernel();
    k_p_bad.state.faults |= FaultFlags::ALL_IMU_FAILED;

    let mut buf_good = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let mut buf_bad = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let snap_good = k_p_good.project_for_cross_channel(0, ChannelId::SECONDARY, &mut buf_good);
    let snap_bad = k_p_bad.project_for_cross_channel(0, ChannelId::TERTIARY, &mut buf_bad);

    let mut buf_local = [0u8; <ProdState as Replicable>::ENCODED_LEN];
    let decision = k_local.check_lockstep_agreement(
        0,
        ChannelId::PRIMARY,
        &mut buf_local,
        &[Some(snap_good), Some(snap_bad)],
        2,
    );
    assert_eq!(
        decision,
        LockstepDecision::RefuseStateMismatch {
            peer: ChannelId::TERTIARY,
        },
        "diverged peer must surface as RefuseStateMismatch with its own ChannelId"
    );
}
