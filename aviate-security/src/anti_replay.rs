//! Anti-replay protection using per-identity monotonic counters
//!
//! This module implements replay attack prevention for signed commands.
//! Each signing identity maintains an independent monotonic counter, and
//! the window as a whole maintains a trusted local signing timestamp that
//! bounds how old a *first* frame from a new identity may be.
//!
//! ## Security Model
//!
//! - **Per-identity tracking**: the replay identity is the full MAVLink
//!   signing tuple `(system_id, component_id, link_id)`, not `link_id`
//!   alone. Two senders that share a `link_id` but differ in system or
//!   component id are distinct peers with independent counters.
//!
//! - **Strict monotonic**: a new timestamp MUST be strictly greater than
//!   the last accepted timestamp for its identity (`new > last`). No
//!   equality, no backwards movement.
//!
//! - **First-frame freshness**: an identity never seen before is accepted
//!   only if its timestamp is no more than [`NEW_STREAM_MAX_AGE_10US`]
//!   (one minute) behind the window's trusted local timestamp, per the
//!   MAVLink signing spec. Without this bound, rebooting the receiver
//!   would let an attacker replay any command ever captured: the reboot
//!   empties the per-identity slots, and a bare `> 0` check accepts the
//!   old frame as a "new" stream. The local timestamp MUST therefore be
//!   initialized from a trusted source (persisted high-water mark or RTC)
//!   at construction, and it advances to the highest accepted timestamp
//!   so it can be persisted back.
//!
//! - **No skew window**: unlike some protocols (e.g. IPsec) we do NOT allow
//!   a replay window for out-of-order packets. MAVLink over USB/UART is
//!   strictly ordered, so any non-monotonic timestamp is suspicious.
//!
//! - **Bounded, authenticated-only**: the table holds a fixed number of
//!   identities. Callers MUST verify a frame's signature *before*
//!   committing its identity here, so only cryptographically authenticated
//!   peers ever occupy a slot — an attacker cannot flood the table with
//!   forged identities. A new identity is rejected only when every slot is
//!   held by an already-authenticated peer.
//!
//! ## DO-178C Properties
//!
//! - **Time complexity**: O(MAX_SIGNING_PEERS) scan — a small fixed bound
//! - **Memory**: `MAX_SIGNING_PEERS` fixed-size entries, no allocation
//! - **WCET**: bounded linear scan of a small array
//! - **Determinism**: no allocation, no unbounded loops

use crate::errors::{AuthError, AuthResult};

/// Maximum number of distinct signing identities tracked concurrently.
///
/// Sized for an inner-loop flight controller: a handful of authenticated
/// peers (e.g. an RC bridge, a GCS/datalink, an offboard companion). A slot
/// is only ever occupied by a peer whose signature already verified.
pub const MAX_SIGNING_PEERS: usize = 16;

/// Maximum age of the first frame from a previously unseen identity,
/// relative to the trusted local signing timestamp.
///
/// MAVLink signing timestamps count 10 µs units, so 6,000,000 ticks is
/// one minute — the bound the MAVLink signing spec prescribes for
/// accepting a new logical stream.
pub const NEW_STREAM_MAX_AGE_10US: u64 = 6_000_000;

/// The MAVLink signing identity a replay counter is tracked against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SigningIdentity {
    system_id: u8,
    component_id: u8,
    link_id: u8,
}

/// One tracked identity and its last accepted timestamp.
#[derive(Debug, Clone, Copy)]
struct Slot {
    identity: SigningIdentity,
    last_timestamp: u64,
}

/// Anti-replay window tracking per-identity timestamps plus a trusted
/// local signing timestamp for first-frame freshness.
///
/// ## Usage Example
///
/// ```ignore
/// // `persisted_ts` restored from storage (or derived from an RTC as
/// // 10 µs ticks since the MAVLink signing epoch, 2015-01-01).
/// let mut window = AntiReplayWindow::new(persisted_ts);
///
/// // First command from (sys=1, comp=1, link=5): accepted only when no
/// // more than one minute behind `persisted_ts`.
/// window.check_and_update(1, 1, 5, ts)?;
///
/// // Second command from same identity: must be strictly newer.
/// window.check_and_update(1, 1, 5, ts + 1)?;
///
/// // Periodically persist `window.local_timestamp()` so the next boot
/// // starts from a trusted high-water mark.
/// ```
pub struct AntiReplayWindow {
    /// Occupied identity slots. `None` slots are free.
    slots: [Option<Slot>; MAX_SIGNING_PEERS],
    /// Trusted local signing timestamp (10 µs ticks). Seeded from a
    /// persisted/trusted source at construction; advances to the highest
    /// accepted frame timestamp.
    local_timestamp: u64,
}

impl AntiReplayWindow {
    /// Create an anti-replay window with no tracked identities, seeded
    /// with a trusted local signing timestamp.
    ///
    /// `initial_trusted_timestamp` MUST come from a source an attacker
    /// cannot rewind: a persisted high-water mark from the previous boot,
    /// or an RTC converted to 10 µs ticks since the MAVLink signing epoch
    /// (2015-01-01). Passing `0` disables first-frame freshness — every
    /// captured frame ever signed becomes replayable after a reboot — and
    /// is acceptable only on an isolated bench.
    pub const fn new(initial_trusted_timestamp: u64) -> Self {
        Self {
            slots: [None; MAX_SIGNING_PEERS],
            local_timestamp: initial_trusted_timestamp,
        }
    }

    /// The trusted local signing timestamp: the maximum of the seed value
    /// and every accepted frame timestamp. Persist this periodically and
    /// feed it back into [`Self::new`] on the next boot.
    pub fn local_timestamp(&self) -> u64 {
        self.local_timestamp
    }

    /// Locate the occupied slot for `identity`, if tracked.
    fn find(&self, identity: SigningIdentity) -> Option<usize> {
        self.slots.iter().position(|slot| match slot {
            Some(s) => s.identity == identity,
            None => false,
        })
    }

    /// Check whether `timestamp` is valid for its identity and update the
    /// window.
    ///
    /// ## Parameters
    ///
    /// - `system_id` / `component_id` / `link_id`: the signing identity
    /// - `timestamp`: remote monotonic counter from the command signature
    ///
    /// ## Returns
    ///
    /// - `Ok(())`: accepted; the identity's high-water mark (and the local
    ///   timestamp) advance to `timestamp`
    /// - `Err(AuthError::ReplayAttack)`: timestamp is not strictly greater
    ///   than the identity's last accepted timestamp (or is zero)
    /// - `Err(AuthError::StaleNewStream { .. })`: the identity is new and
    ///   its first timestamp is more than [`NEW_STREAM_MAX_AGE_10US`]
    ///   behind the trusted local timestamp
    /// - `Err(AuthError::ReplayCapacityExhausted)`: the identity is new and
    ///   every slot is already held by an authenticated peer
    ///
    /// ## Security Invariant
    ///
    /// Callers MUST have verified the frame's signature AND authorized its
    /// identity before calling this; on success the identity's high-water
    /// mark advances to `timestamp`.
    ///
    /// ## DO-178C Contract
    ///
    /// - **Time complexity**: O(MAX_SIGNING_PEERS)
    /// - **Side effects**: updates internal state on success, no change on
    ///   failure
    /// - **Thread safety**: NOT thread-safe (requires external
    ///   synchronization)
    pub fn check_and_update(
        &mut self,
        system_id: u8,
        component_id: u8,
        link_id: u8,
        timestamp: u64,
    ) -> AuthResult<()> {
        let identity = SigningIdentity {
            system_id,
            component_id,
            link_id,
        };

        if let Some(idx) = self.find(identity) {
            let slot = match self.slots.get_mut(idx).and_then(Option::as_mut) {
                Some(slot) => slot,
                None => return Err(AuthError::ReplayAttack),
            };
            if timestamp <= slot.last_timestamp {
                return Err(AuthError::ReplayAttack);
            }
            slot.last_timestamp = timestamp;
            self.local_timestamp = self.local_timestamp.max(timestamp);
            return Ok(());
        }

        // New identity: implicit last = 0, so the first timestamp must be
        // strictly positive...
        if timestamp == 0 {
            return Err(AuthError::ReplayAttack);
        }
        // ...and no more than one minute behind the trusted local
        // timestamp — otherwise a reboot (which empties the slots) would
        // accept any previously captured frame as a "new" stream.
        if timestamp < self.local_timestamp.saturating_sub(NEW_STREAM_MAX_AGE_10US) {
            return Err(AuthError::StaleNewStream {
                timestamp,
                local_timestamp: self.local_timestamp,
            });
        }

        match self.slots.iter_mut().find(|slot| slot.is_none()) {
            Some(free) => {
                *free = Some(Slot {
                    identity,
                    last_timestamp: timestamp,
                });
                self.local_timestamp = self.local_timestamp.max(timestamp);
                Ok(())
            }
            None => Err(AuthError::ReplayCapacityExhausted),
        }
    }

    /// Last accepted timestamp for an identity (debugging/telemetry).
    ///
    /// Returns `0` when the identity has never been accepted.
    pub fn last_timestamp(&self, system_id: u8, component_id: u8, link_id: u8) -> u64 {
        let identity = SigningIdentity {
            system_id,
            component_id,
            link_id,
        };
        match self.find(identity) {
            Some(idx) => match self.slots.get(idx).and_then(Option::as_ref) {
                Some(slot) => slot.last_timestamp,
                None => 0,
            },
            None => 0,
        }
    }

    /// Forget a specific identity (testing/recovery).
    ///
    /// ## Security Warning
    ///
    /// Resetting allows previously-seen timestamps for that identity to be
    /// replayed (bounded by the first-frame freshness window). Only use in
    /// controlled scenarios (testing, operator command).
    pub fn reset_identity(&mut self, system_id: u8, component_id: u8, link_id: u8) {
        let identity = SigningIdentity {
            system_id,
            component_id,
            link_id,
        };
        if let Some(idx) = self.find(identity) {
            if let Some(slot) = self.slots.get_mut(idx) {
                *slot = None;
            }
        }
    }

    /// Forget all identities (testing only). The trusted local timestamp
    /// is retained — clearing it would re-open the reboot-replay hole.
    ///
    /// ## Security Warning
    ///
    /// This clears all per-identity anti-replay state! Only use in test
    /// code.
    pub fn reset_all(&mut self) {
        self.slots = [None; MAX_SIGNING_PEERS];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_command_accepted() {
        let mut window = AntiReplayWindow::new(0);
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        assert_eq!(window.last_timestamp(1, 1, 5), 1000);
    }

    #[test]
    fn test_monotonic_increase_accepted() {
        let mut window = AntiReplayWindow::new(0);
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        assert!(window.check_and_update(1, 1, 5, 1001).is_ok());
        assert!(window.check_and_update(1, 1, 5, 1002).is_ok());
        assert_eq!(window.last_timestamp(1, 1, 5), 1002);
    }

    #[test]
    fn test_replay_same_timestamp_rejected() {
        let mut window = AntiReplayWindow::new(0);
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        match window.check_and_update(1, 1, 5, 1000) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_replay_older_timestamp_rejected() {
        let mut window = AntiReplayWindow::new(0);
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        match window.check_and_update(1, 1, 5, 999) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_link_id_alone_is_not_identity() {
        let now = 10_000_000;
        let mut window = AntiReplayWindow::new(now);
        // Same link_id, different component_id → independent counters.
        assert!(window.check_and_update(1, 1, 5, now + 1000).is_ok());
        assert!(window.check_and_update(1, 2, 5, now + 500).is_ok());
        // And different system_id is independent too.
        assert!(window.check_and_update(2, 1, 5, now + 300).is_ok());
        assert_eq!(window.last_timestamp(1, 1, 5), now + 1000);
        assert_eq!(window.last_timestamp(1, 2, 5), now + 500);
        assert_eq!(window.last_timestamp(2, 1, 5), now + 300);
    }

    #[test]
    fn test_reset_identity() {
        let mut window = AntiReplayWindow::new(0);
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        window.reset_identity(1, 1, 5);
        assert_eq!(window.last_timestamp(1, 1, 5), 0);
        assert!(window.check_and_update(1, 1, 5, 500).is_ok());
    }

    #[test]
    fn test_zero_timestamp_rejected_for_new_identity() {
        let mut window = AntiReplayWindow::new(0);
        match window.check_and_update(1, 1, 5, 0) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack for timestamp=0"),
        }
    }

    /// The reboot-replay defense: with the local timestamp seeded from a
    /// trusted source, a captured command older than one minute is NOT
    /// accepted as the first frame of a "new" stream.
    #[test]
    fn stale_first_frame_rejected_against_trusted_timestamp() {
        let boot_ts = 100_000_000;
        let mut window = AntiReplayWindow::new(boot_ts);

        // Captured long before boot_ts → rejected.
        let old = boot_ts - NEW_STREAM_MAX_AGE_10US - 1;
        match window.check_and_update(1, 1, 5, old) {
            Err(AuthError::StaleNewStream {
                timestamp,
                local_timestamp,
            }) => {
                assert_eq!(timestamp, old);
                assert_eq!(local_timestamp, boot_ts);
            }
            other => panic!("Expected StaleNewStream, got {other:?}"),
        }

        // Exactly at the one-minute bound → accepted.
        let edge = boot_ts - NEW_STREAM_MAX_AGE_10US;
        assert!(window.check_and_update(1, 1, 5, edge).is_ok());
    }

    /// The local timestamp follows the highest accepted frame, so a later
    /// new stream is measured against live traffic, not just the boot
    /// seed — and it is exposed for persistence across reboots.
    #[test]
    fn local_timestamp_advances_with_accepted_frames() {
        let mut window = AntiReplayWindow::new(1000);
        assert_eq!(window.local_timestamp(), 1000);

        let far_ahead = 1000 + NEW_STREAM_MAX_AGE_10US * 10;
        assert!(window.check_and_update(1, 1, 5, far_ahead).is_ok());
        assert_eq!(window.local_timestamp(), far_ahead);

        // A second identity must now be fresh relative to `far_ahead`.
        match window.check_and_update(1, 2, 5, 2000) {
            Err(AuthError::StaleNewStream { .. }) => {}
            other => panic!("Expected StaleNewStream, got {other:?}"),
        }
        assert!(window
            .check_and_update(1, 2, 5, far_ahead - NEW_STREAM_MAX_AGE_10US)
            .is_ok());
    }

    /// A rejected frame must not advance the trusted local timestamp.
    #[test]
    fn rejected_frames_do_not_advance_local_timestamp() {
        let mut window = AntiReplayWindow::new(1000);
        assert!(window.check_and_update(1, 1, 5, 5000).is_ok());
        // Replay: rejected, local timestamp unchanged.
        assert!(window.check_and_update(1, 1, 5, 5000).is_err());
        assert_eq!(window.local_timestamp(), 5000);
    }

    #[test]
    fn test_capacity_exhausted_rejects_new_identity() {
        let mut window = AntiReplayWindow::new(0);
        // Fill every slot with a distinct authenticated identity.
        for i in 0..MAX_SIGNING_PEERS as u8 {
            assert!(window.check_and_update(1, 1, i, 1000).is_ok());
        }
        // A further NEW identity has nowhere to go.
        match window.check_and_update(9, 9, 200, 1000) {
            Err(AuthError::ReplayCapacityExhausted) => {}
            _ => panic!("Expected ReplayCapacityExhausted"),
        }
        // But an already-tracked identity still advances fine.
        assert!(window.check_and_update(1, 1, 0, 1001).is_ok());
    }
}
