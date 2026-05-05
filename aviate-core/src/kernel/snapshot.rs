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
}
