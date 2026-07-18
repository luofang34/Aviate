//! Anti-replay protection using per-identity monotonic counters
//!
//! This module implements replay attack prevention for signed commands.
//! Each signing identity maintains an independent monotonic counter.
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
//!   equality, no backwards movement. An identity never seen before has an
//!   implicit `last = 0`, so its first timestamp must be `> 0`.
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

/// Anti-replay window tracking per-identity timestamps.
///
/// ## Usage Example
///
/// ```ignore
/// let mut window = AntiReplayWindow::new();
///
/// // First command from (sys=1, comp=1, link=5)
/// window.check_and_update(1, 1, 5, 1000)?;  // OK
///
/// // Second command from same identity
/// window.check_and_update(1, 1, 5, 1001)?;  // OK (1001 > 1000)
///
/// // Replay attack (same or older timestamp)
/// window.check_and_update(1, 1, 5, 1000)?;  // Error: ReplayAttack
///
/// // Same link_id but different component → independent identity
/// window.check_and_update(1, 2, 5, 500)?;   // OK
/// ```
pub struct AntiReplayWindow {
    /// Occupied identity slots. `None` slots are free.
    slots: [Option<Slot>; MAX_SIGNING_PEERS],
}

impl AntiReplayWindow {
    /// Create a new anti-replay window with no tracked identities.
    ///
    /// ## Post-condition
    ///
    /// Every slot is free. The first command from each identity is accepted
    /// as long as its timestamp is `> 0` (an unseen identity has an implicit
    /// `last = 0`).
    pub const fn new() -> Self {
        Self {
            slots: [None; MAX_SIGNING_PEERS],
        }
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
    /// - `Ok(())`: timestamp is strictly greater than the last for this
    ///   identity (or the identity is new with `timestamp > 0`); the window
    ///   is updated
    /// - `Err(AuthError::ReplayAttack)`: timestamp is not strictly greater
    /// - `Err(AuthError::ReplayCapacityExhausted)`: the identity is new and
    ///   every slot is already held by an authenticated peer
    ///
    /// ## Security Invariant
    ///
    /// Callers MUST have verified the frame's signature before calling this;
    /// on success the identity's high-water mark advances to `timestamp`.
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
            return Ok(());
        }

        // New identity: implicit last = 0, so the first timestamp must be
        // strictly positive.
        if timestamp == 0 {
            return Err(AuthError::ReplayAttack);
        }

        match self.slots.iter_mut().find(|slot| slot.is_none()) {
            Some(free) => {
                *free = Some(Slot {
                    identity,
                    last_timestamp: timestamp,
                });
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
    /// replayed. Only use in controlled scenarios (testing, operator
    /// command).
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

    /// Forget all identities (testing only).
    ///
    /// ## Security Warning
    ///
    /// This clears all anti-replay state! Only use in test code.
    pub fn reset_all(&mut self) {
        self.slots = [None; MAX_SIGNING_PEERS];
    }
}

impl Default for AntiReplayWindow {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_command_accepted() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        assert_eq!(window.last_timestamp(1, 1, 5), 1000);
    }

    #[test]
    fn test_monotonic_increase_accepted() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        assert!(window.check_and_update(1, 1, 5, 1001).is_ok());
        assert!(window.check_and_update(1, 1, 5, 1002).is_ok());
        assert_eq!(window.last_timestamp(1, 1, 5), 1002);
    }

    #[test]
    fn test_replay_same_timestamp_rejected() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        match window.check_and_update(1, 1, 5, 1000) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_replay_older_timestamp_rejected() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        match window.check_and_update(1, 1, 5, 999) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_link_id_alone_is_not_identity() {
        let mut window = AntiReplayWindow::new();
        // Same link_id, different component_id → independent counters.
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        assert!(window.check_and_update(1, 2, 5, 500).is_ok());
        // And different system_id is independent too.
        assert!(window.check_and_update(2, 1, 5, 300).is_ok());
        assert_eq!(window.last_timestamp(1, 1, 5), 1000);
        assert_eq!(window.last_timestamp(1, 2, 5), 500);
        assert_eq!(window.last_timestamp(2, 1, 5), 300);
    }

    #[test]
    fn test_reset_identity() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(1, 1, 5, 1000).is_ok());
        window.reset_identity(1, 1, 5);
        assert_eq!(window.last_timestamp(1, 1, 5), 0);
        assert!(window.check_and_update(1, 1, 5, 500).is_ok());
    }

    #[test]
    fn test_zero_timestamp_rejected_for_new_identity() {
        let mut window = AntiReplayWindow::new();
        match window.check_and_update(1, 1, 5, 0) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack for timestamp=0"),
        }
    }

    #[test]
    fn test_capacity_exhausted_rejects_new_identity() {
        let mut window = AntiReplayWindow::new();
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
