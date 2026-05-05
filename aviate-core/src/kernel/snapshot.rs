//! `ChannelSnapshot` — single-channel projection of `KernelState`
//! for spec §16 cross-channel firmware verification and state
//! replication.
//!
//! A snapshot bundles two things into one byte-comparable witness:
//!
//!   - `algorithm_identity_hash` (LLR-PIPE-103) — what code is
//!     running.
//!   - `state_bytes` (HLR-REPL-001) — what state that code holds.
//!
//! Cross-channel agreement is byte equality of those two together.
//! `cycle_seq` and `channel_id` are carried for staleness / origin
//! tracking but are NOT part of the agreement check — peers run
//! cycles on their own clocks and have distinct IDs by definition.
//!
//! This module deliberately does NOT define a wire format. The
//! caller chooses how to serialize the snapshot for transport
//! (UART, CAN, Ethernet) — `state_bytes` is already canonical, so
//! framing only needs length-prefixing on top.

use crate::ChannelId;

/// Single-channel projection of the kernel state for cross-channel
/// exchange and comparison (spec §16).
///
/// Borrows the canonical state bytes from a caller-owned buffer
/// (no allocation, no_std-friendly). The lifetime ties the
/// snapshot to that buffer — if the buffer is rewritten, the
/// snapshot is invalidated by Rust's borrow checker.
#[derive(Clone, Debug)]
pub struct ChannelSnapshot<'a> {
    /// Origin channel for this snapshot.
    pub channel_id: ChannelId,
    /// Cycle sequence number from the channel that produced this.
    /// Peers compare seq monotonicity to gate stale data; the
    /// agreement check itself ignores it.
    pub cycle_seq: u64,
    /// `KernelPipeline::algorithm_identity_hash()` (LLR-PIPE-103) at
    /// the producing channel. Mismatch here means the channels are
    /// running structurally different firmware bundles — peer
    /// lockstep SHALL NOT be entered.
    pub algorithm_identity_hash: u64,
    /// Canonical encoding of the channel's `KernelState`
    /// (HLR-REPL-001). Length is `KernelState::ENCODED_LEN` for the
    /// chosen `(E, R)` parameterization.
    pub state_bytes: &'a [u8],
}

impl<'a> ChannelSnapshot<'a> {
    /// Cross-channel agreement: byte-equality of the
    /// algorithm-identity hash AND the canonical state bytes.
    ///
    /// `cycle_seq` and `channel_id` are NOT compared — peers run
    /// cycles on their own clocks and have distinct IDs by
    /// definition. The caller is responsible for any staleness
    /// gating (e.g. "reject snapshots whose cycle_seq is more than
    /// N cycles behind the local channel").
    ///
    /// Returns true iff the two snapshots witness byte-identical
    /// firmware-and-state at their respective channels.
    pub fn agrees_with(&self, other: &Self) -> bool {
        self.algorithm_identity_hash == other.algorithm_identity_hash
            && self.state_bytes.len() == other.state_bytes.len()
            && self.state_bytes == other.state_bytes
    }
}

/// Decision returned by [`decide_lockstep`].
///
/// `Enter` is the only outcome that authorizes peer-lockstep entry.
/// All `Refuse*` variants block entry; the variant carries the
/// failure mode so the caller can route the higher-level
/// redundancy response (downgrade to channel-isolated, retry next
/// cycle, declare hot-spare takeover).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LockstepDecision {
    /// Quorum of peers present, all agree with local. Lockstep
    /// entry SHALL proceed.
    Enter,
    /// At least one peer's `algorithm_identity_hash` diverges from
    /// local. Indicates structurally different firmware bundles.
    /// Carries the offending peer's `ChannelId` for the redundancy
    /// policy to consult.
    RefuseHashMismatch { peer: crate::ChannelId },
    /// At least one peer's algorithm hash agrees with local but
    /// state bytes diverge. Indicates a state-divergence event
    /// (one channel ran a cycle the others didn't, sensor input
    /// fan-out is asymmetric, etc.). Lockstep entry SHALL be
    /// refused.
    RefuseStateMismatch { peer: crate::ChannelId },
    /// Fewer peer snapshots are present than the caller-specified
    /// quorum requires. Default policy is to refuse — lockstep
    /// requires confirmed agreement, not absence of disagreement.
    RefuseQuorum { present: usize, required: usize },
}

/// Cross-channel agreement gate. Given the local channel's
/// snapshot and a slice of optional peer snapshots, decide whether
/// to enter lockstep.
///
/// `quorum` is the minimum number of peer snapshots required to be
/// present (i.e. `Some`). Below quorum, the function returns
/// [`LockstepDecision::RefuseQuorum`] without inspecting any peer
/// hashes or bytes — refuse on absence of evidence, not just
/// presence of disagreement.
///
/// At quorum or above, the function inspects each present peer and
/// returns the FIRST disagreement found (hash mismatch takes
/// precedence over state mismatch within a single peer). Returning
/// on first disagreement is intentional: cross-channel
/// disagreement is a fail-stop event for lockstep — there is no
/// "majority overrules" semantics here.
///
/// The function SHALL NOT panic, SHALL NOT allocate, SHALL NOT
/// consult any external state, and SHALL be `#[inline]`-eligible.
///
/// Lifetime parameters are independent: the local snapshot and the
/// peer snapshots typically come from different caller-owned
/// buffers (local cycle's projection vs. transport-deserialized
/// peer frames). The function compares values, not borrows.
pub fn decide_lockstep<'a, 'b>(
    local: &ChannelSnapshot<'a>,
    peers: &[Option<ChannelSnapshot<'b>>],
    quorum: usize,
) -> LockstepDecision {
    let present = peers.iter().filter(|p| p.is_some()).count();
    if present < quorum {
        return LockstepDecision::RefuseQuorum {
            present,
            required: quorum,
        };
    }
    for peer_opt in peers {
        let Some(peer) = peer_opt.as_ref() else {
            continue;
        };
        if peer.algorithm_identity_hash != local.algorithm_identity_hash {
            return LockstepDecision::RefuseHashMismatch {
                peer: peer.channel_id,
            };
        }
        if peer.state_bytes.len() != local.state_bytes.len()
            || peer.state_bytes != local.state_bytes
        {
            return LockstepDecision::RefuseStateMismatch {
                peer: peer.channel_id,
            };
        }
    }
    LockstepDecision::Enter
}

#[cfg(test)]
mod tests {
    use super::ChannelSnapshot;
    use crate::ChannelId;

    fn make(channel: ChannelId, seq: u64, hash: u64, bytes: &[u8]) -> ChannelSnapshot<'_> {
        ChannelSnapshot {
            channel_id: channel,
            cycle_seq: seq,
            algorithm_identity_hash: hash,
            state_bytes: bytes,
        }
    }

    #[test]
    fn agrees_with_identical_snapshots() {
        let bytes = [1u8, 2, 3, 4];
        let a = make(ChannelId::PRIMARY, 42, 0xABCD, &bytes);
        let b = make(ChannelId::SECONDARY, 42, 0xABCD, &bytes);
        assert!(
            a.agrees_with(&b),
            "byte-equal hash and state bytes ⇒ agreement"
        );
    }

    #[test]
    fn disagrees_when_state_bytes_differ() {
        let bytes_a = [1u8, 2, 3, 4];
        let bytes_b = [1u8, 2, 3, 5];
        let a = make(ChannelId::PRIMARY, 42, 0xABCD, &bytes_a);
        let b = make(ChannelId::SECONDARY, 42, 0xABCD, &bytes_b);
        assert!(
            !a.agrees_with(&b),
            "differing state bytes ⇒ disagreement (Hamming-1 catches single-byte drift)"
        );
    }

    #[test]
    fn disagrees_when_hash_differs() {
        let bytes = [1u8, 2, 3, 4];
        let a = make(ChannelId::PRIMARY, 42, 0xABCD, &bytes);
        let b = make(ChannelId::SECONDARY, 42, 0x1234, &bytes);
        assert!(
            !a.agrees_with(&b),
            "differing algorithm hash ⇒ disagreement (firmware bundle mismatch)"
        );
    }

    #[test]
    fn agrees_ignores_channel_and_seq() {
        let bytes = [1u8, 2, 3, 4];
        let a = make(ChannelId::PRIMARY, 1, 0xABCD, &bytes);
        let b = make(ChannelId::TERTIARY, 999, 0xABCD, &bytes);
        assert!(
            a.agrees_with(&b),
            "channel_id and cycle_seq must NOT block agreement \
             (peers run on their own clocks and have distinct IDs)"
        );
    }

    #[test]
    fn disagrees_on_length_mismatch() {
        let bytes_a = [1u8, 2, 3, 4];
        let bytes_b = [1u8, 2, 3];
        let a = make(ChannelId::PRIMARY, 42, 0xABCD, &bytes_a);
        let b = make(ChannelId::SECONDARY, 42, 0xABCD, &bytes_b);
        assert!(
            !a.agrees_with(&b),
            "length mismatch ⇒ disagreement (firmware-version skew detection)"
        );
    }

    use super::{decide_lockstep, LockstepDecision};

    #[test]
    fn decide_enter_when_all_peers_agree() {
        let bytes = [1u8, 2, 3, 4];
        let local = make(ChannelId::PRIMARY, 10, 0xABCD, &bytes);
        let p1 = make(ChannelId::SECONDARY, 11, 0xABCD, &bytes);
        let p2 = make(ChannelId::TERTIARY, 12, 0xABCD, &bytes);
        let peers = [Some(p1), Some(p2)];
        assert_eq!(decide_lockstep(&local, &peers, 2), LockstepDecision::Enter);
    }

    #[test]
    fn decide_refuse_hash_mismatch_takes_priority_over_state() {
        // A peer with both hash AND state mismatch surfaces as
        // RefuseHashMismatch — hash divergence is structurally more
        // serious (different firmware bundles) than state
        // divergence (same firmware, transient state drift).
        let local_bytes = [1u8, 2, 3, 4];
        let peer_bytes = [9u8, 9, 9, 9];
        let local = make(ChannelId::PRIMARY, 0, 0xABCD, &local_bytes);
        let peer = make(ChannelId::SECONDARY, 0, 0xDEAD, &peer_bytes);
        let peers = [Some(peer)];
        assert_eq!(
            decide_lockstep(&local, &peers, 1),
            LockstepDecision::RefuseHashMismatch {
                peer: ChannelId::SECONDARY,
            }
        );
    }

    #[test]
    fn decide_refuse_state_mismatch_when_hash_agrees() {
        let local_bytes = [1u8, 2, 3, 4];
        let peer_bytes = [1u8, 2, 3, 5]; // single bit flip
        let local = make(ChannelId::PRIMARY, 0, 0xABCD, &local_bytes);
        let peer = make(ChannelId::TERTIARY, 0, 0xABCD, &peer_bytes);
        let peers = [Some(peer)];
        assert_eq!(
            decide_lockstep(&local, &peers, 1),
            LockstepDecision::RefuseStateMismatch {
                peer: ChannelId::TERTIARY,
            }
        );
    }

    #[test]
    fn decide_refuse_quorum_when_below_threshold() {
        let bytes = [1u8, 2, 3, 4];
        let local = make(ChannelId::PRIMARY, 0, 0xABCD, &bytes);
        let peers: [Option<ChannelSnapshot>; 2] = [None, None];
        // Quorum 2 required, 0 present
        assert_eq!(
            decide_lockstep(&local, &peers, 2),
            LockstepDecision::RefuseQuorum {
                present: 0,
                required: 2,
            }
        );
    }

    #[test]
    fn decide_refuse_quorum_does_not_inspect_peers() {
        // Even if the one present peer would disagree, RefuseQuorum
        // surfaces first because absence-of-evidence is the
        // outcome — the gate refuses to GUESS at agreement from a
        // partial peer set.
        let bytes_local = [1u8, 2, 3, 4];
        let bytes_peer = [9u8, 9, 9, 9]; // would mismatch
        let local = make(ChannelId::PRIMARY, 0, 0xABCD, &bytes_local);
        let peer = make(ChannelId::SECONDARY, 0, 0xDEAD, &bytes_peer);
        let peers = [Some(peer), None];
        // 1 present, quorum 2 required ⇒ RefuseQuorum, not RefuseHashMismatch
        assert_eq!(
            decide_lockstep(&local, &peers, 2),
            LockstepDecision::RefuseQuorum {
                present: 1,
                required: 2,
            }
        );
    }

    #[test]
    fn decide_enter_when_zero_quorum_required_and_no_peers() {
        // Edge case: zero-quorum is degenerate but well-defined —
        // an empty peer set with quorum 0 is "trivially in
        // agreement". Useful for single-channel fault scenarios
        // where the policy degrades to non-lockstep operation.
        let bytes = [1u8, 2, 3, 4];
        let local = make(ChannelId::PRIMARY, 0, 0xABCD, &bytes);
        let peers: [Option<ChannelSnapshot>; 0] = [];
        assert_eq!(decide_lockstep(&local, &peers, 0), LockstepDecision::Enter);
    }
}
