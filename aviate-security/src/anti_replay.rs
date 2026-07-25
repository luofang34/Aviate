//! Anti-replay protection using per-principal monotonic counters
//!
//! This module implements replay attack prevention for authenticated
//! commands. It is scheme-neutral: the replay identity is a
//! [`Principal`], and freshness is a monotonic `counter`, regardless of
//! which admission adapter produced them. For MAVLink signing the counter
//! is the signing timestamp; for a future AEAD envelope it is the session
//! message counter.
//!
//! ## Security Model
//!
//! - **Per-principal tracking**: each [`Principal`] maintains an
//!   independent monotonic counter. Two senders that differ in scheme,
//!   credential, or asserted identity are distinct peers.
//!
//! - **Strict monotonic**: a new counter MUST be strictly greater than the
//!   last accepted counter for its principal (`new > last`). No equality,
//!   no backwards movement.
//!
//! - **First-frame freshness**: a principal never seen before is accepted
//!   only if its counter is no more than `new_stream_max_age` behind the
//!   window's trusted local counter. Without this bound, rebooting the
//!   receiver would let an attacker replay any command ever captured: the
//!   reboot empties the per-principal slots, and a bare `> 0` check accepts
//!   the old frame as a "new" stream. The local counter MUST therefore be
//!   initialized from a trusted source (persisted high-water mark or RTC)
//!   at construction, and it advances to the highest accepted counter so
//!   it can be persisted back. The age bound is scheme-specific and
//!   supplied by the profile (MAVLink: [`NEW_STREAM_MAX_AGE_10US`]).
//!
//! - **No skew window**: we do NOT allow a replay window for out-of-order
//!   packets. Command links are strictly ordered, so any non-monotonic
//!   counter is suspicious.
//!
//! - **Bounded, authenticated-only**: the table holds a fixed number of
//!   principals. Callers MUST verify a frame's cryptography *and* authorize
//!   its principal before committing here, so only authorized peers ever
//!   occupy a slot — an attacker cannot flood the table with forged or
//!   unauthorized principals.
//!
//! ## DO-178C Properties
//!
//! - **Time complexity**: O(MAX_SIGNING_PEERS) scan — a small fixed bound
//! - **Memory**: `MAX_SIGNING_PEERS` fixed-size entries, no allocation
//! - **WCET**: bounded linear scan of a small array
//! - **Determinism**: no allocation, no unbounded loops

use crate::errors::{AuthError, AuthResult};
use crate::principal::Principal;

/// Maximum number of distinct principals tracked concurrently.
///
/// Sized for an inner-loop flight controller: a handful of authenticated
/// peers (e.g. an RC bridge, a GCS/datalink, an offboard companion). A slot
/// is only ever occupied by a peer whose command already verified and was
/// authorized.
pub const MAX_SIGNING_PEERS: usize = 16;

/// Maximum age of the first frame from a previously unseen principal,
/// relative to the trusted local counter, **for the MAVLink signing
/// profile**.
///
/// MAVLink signing timestamps count 10 µs units, so 6,000,000 ticks is
/// one minute — the bound the MAVLink signing spec prescribes for
/// accepting a new logical stream. Other schemes supply their own bound.
pub const NEW_STREAM_MAX_AGE_10US: u64 = 6_000_000;

/// One tracked principal and its last accepted counter.
#[derive(Debug, Clone, Copy)]
struct Slot {
    principal: Principal,
    last_counter: u64,
}

/// Anti-replay window tracking per-principal counters plus a trusted local
/// counter for first-frame freshness.
pub struct AntiReplayWindow {
    /// Occupied principal slots. `None` slots are free.
    slots: [Option<Slot>; MAX_SIGNING_PEERS],
    /// Floor for first-frame freshness. Seeded from a trusted source at
    /// construction and NEVER moved by a peer-supplied counter: letting
    /// one sender raise it made that sender's clock the threshold every
    /// other principal had to clear, and the value is persisted, so the
    /// lockout outlived the reboot.
    freshness_floor: u64,
    /// Highest counter ever accepted, for persisting across a reboot.
    /// Read by the assembly, never consulted as a threshold here.
    ///
    /// Its growth is NOT bounded when no forward ceiling applies, which
    /// is the case for a `PersistedHighWater` seed. An authorized peer
    /// with a wrong clock therefore raises this, and since it is what the
    /// next boot seeds the floor from, the cross-principal lockout
    /// returns one reboot later. Only fitting a real-time clock — which
    /// restores the forward bound — closes that; a lower bound with
    /// unknown lag structurally cannot also produce a safe upper one.
    high_water: u64,
    /// How far behind `local_counter` the first frame of a new stream may
    /// be. Scheme-specific; supplied by the profile.
    new_stream_max_age: u64,
}

impl AntiReplayWindow {
    /// Create an anti-replay window with no tracked principals, seeded with
    /// a trusted local counter and a scheme-specific new-stream age bound.
    ///
    /// `initial_trusted_counter` MUST come from a source an attacker cannot
    /// rewind: a persisted high-water mark from the previous boot, or an
    /// RTC converted to the scheme's counter units. Passing `0` disables
    /// first-frame freshness and is acceptable only on an isolated bench.
    pub const fn new(initial_trusted_counter: u64, new_stream_max_age: u64) -> Self {
        Self {
            slots: [None; MAX_SIGNING_PEERS],
            freshness_floor: initial_trusted_counter,
            high_water: initial_trusted_counter,
            new_stream_max_age,
        }
    }

    /// The trusted local counter: the maximum of the seed value and every
    /// accepted counter. Persist this periodically and feed it back into
    /// [`Self::new`] on the next boot.
    pub fn local_counter(&self) -> u64 {
        self.high_water
    }

    /// Locate the occupied slot for `principal`, if tracked.
    fn find(&self, principal: Principal) -> Option<usize> {
        self.slots.iter().position(|slot| match slot {
            Some(s) => s.principal == principal,
            None => false,
        })
    }

    /// Check whether `counter` is valid for `principal` and update the
    /// window.
    ///
    /// ## Returns
    ///
    /// - `Ok(())`: accepted; the principal's high-water mark (and the local
    ///   counter) advance to `counter`
    /// - `Err(AuthError::ReplayAttack)`: not strictly greater than the
    ///   principal's last accepted counter (or is zero)
    /// - `Err(AuthError::StaleNewStream { .. })`: the principal is new and
    ///   its first counter is more than `new_stream_max_age` behind the
    ///   trusted local counter
    /// - `Err(AuthError::ReplayCapacityExhausted)`: the principal is new
    ///   and every slot is already held
    ///
    /// ## Security Invariant
    ///
    /// Callers MUST have verified the command's cryptography AND authorized
    /// its principal before calling this.
    pub fn check_and_update(
        &mut self,
        principal: Principal,
        counter: u64,
        plausible_ceiling: Option<u64>,
    ) -> AuthResult<()> {
        // A forward bound, when the caller can derive one. It is `None`
        // unless the seed came from a real-time clock: a persisted
        // high-water mark is a lower bound on "now" with unknown lag —
        // however long power was off — so measuring "too far ahead"
        // against it reads an ordinary battery swap as a wrong clock and
        // refuses the operator forever. Cross-principal poisoning is
        // prevented by `freshness_floor` never moving, not by this.
        if let Some(ceiling) = plausible_ceiling {
            if counter > ceiling {
                return Err(AuthError::CounterImplausiblyAhead { counter, ceiling });
            }
        }

        if let Some(idx) = self.find(principal) {
            let slot = match self.slots.get_mut(idx).and_then(Option::as_mut) {
                Some(slot) => slot,
                None => return Err(AuthError::ReplayAttack),
            };
            if counter <= slot.last_counter {
                return Err(AuthError::ReplayAttack);
            }
            slot.last_counter = counter;
            self.high_water = self.high_water.max(counter);
            return Ok(());
        }

        // New principal: implicit last = 0, so the first counter must be
        // strictly positive...
        if counter == 0 {
            return Err(AuthError::ReplayAttack);
        }
        // ...and no more than `new_stream_max_age` behind the trusted local
        // counter — otherwise a reboot (which empties the slots) would
        // accept any previously captured frame as a "new" stream.
        if counter < self.freshness_floor.saturating_sub(self.new_stream_max_age) {
            return Err(AuthError::StaleNewStream {
                counter,
                local_counter: self.freshness_floor,
            });
        }

        match self.slots.iter_mut().find(|slot| slot.is_none()) {
            Some(free) => {
                *free = Some(Slot {
                    principal,
                    last_counter: counter,
                });
                self.high_water = self.high_water.max(counter);
                Ok(())
            }
            None => Err(AuthError::ReplayCapacityExhausted),
        }
    }

    /// Last accepted counter for a principal (debugging/telemetry).
    ///
    /// Returns `0` when the principal has never been accepted.
    pub fn last_counter(&self, principal: Principal) -> u64 {
        match self.find(principal) {
            Some(idx) => match self.slots.get(idx).and_then(Option::as_ref) {
                Some(slot) => slot.last_counter,
                None => 0,
            },
            None => 0,
        }
    }

    /// Forget a specific principal (testing/recovery).
    ///
    /// ## Security Warning
    ///
    /// Resetting allows previously-seen counters for that principal to be
    /// replayed (bounded by the first-frame freshness window). Only use in
    /// controlled scenarios (testing, operator command).
    pub fn reset_principal(&mut self, principal: Principal) {
        if let Some(idx) = self.find(principal) {
            if let Some(slot) = self.slots.get_mut(idx) {
                *slot = None;
            }
        }
    }

    /// Forget all principals (testing only). The trusted local counter is
    /// retained — clearing it would re-open the reboot-replay hole.
    pub fn reset_all(&mut self) {
        self.slots = [None; MAX_SIGNING_PEERS];
    }
}

#[cfg(test)]
mod tests {
    /// No forward bound, so a case about monotonicity or freshness is not
    /// silently also testing the plausibility bound. Cases that ARE about
    /// that bound pass one explicitly.
    const NO_CEILING: Option<u64> = None;

    use super::*;

    fn p(system_id: u8, component_id: u8, link_id: u8) -> Principal {
        Principal::mavlink(system_id, component_id, link_id)
    }

    #[test]
    fn test_first_command_accepted() {
        let mut window = AntiReplayWindow::new(0, NEW_STREAM_MAX_AGE_10US);
        assert!(window
            .check_and_update(p(1, 1, 5), 1000, NO_CEILING)
            .is_ok());
        assert_eq!(window.last_counter(p(1, 1, 5)), 1000);
    }

    #[test]
    fn test_monotonic_increase_accepted() {
        let mut window = AntiReplayWindow::new(0, NEW_STREAM_MAX_AGE_10US);
        assert!(window
            .check_and_update(p(1, 1, 5), 1000, NO_CEILING)
            .is_ok());
        assert!(window
            .check_and_update(p(1, 1, 5), 1001, NO_CEILING)
            .is_ok());
        assert!(window
            .check_and_update(p(1, 1, 5), 1002, NO_CEILING)
            .is_ok());
        assert_eq!(window.last_counter(p(1, 1, 5)), 1002);
    }

    #[test]
    fn test_replay_same_counter_rejected() {
        let mut window = AntiReplayWindow::new(0, NEW_STREAM_MAX_AGE_10US);
        assert!(window
            .check_and_update(p(1, 1, 5), 1000, NO_CEILING)
            .is_ok());
        match window.check_and_update(p(1, 1, 5), 1000, NO_CEILING) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_replay_older_counter_rejected() {
        let mut window = AntiReplayWindow::new(0, NEW_STREAM_MAX_AGE_10US);
        assert!(window
            .check_and_update(p(1, 1, 5), 1000, NO_CEILING)
            .is_ok());
        match window.check_and_update(p(1, 1, 5), 999, NO_CEILING) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_distinct_principals_are_independent() {
        let now = 10_000_000;
        let mut window = AntiReplayWindow::new(now, NEW_STREAM_MAX_AGE_10US);
        assert!(window
            .check_and_update(p(1, 1, 5), now + 1000, NO_CEILING)
            .is_ok());
        assert!(window
            .check_and_update(p(1, 2, 5), now + 500, NO_CEILING)
            .is_ok());
        assert!(window
            .check_and_update(p(2, 1, 5), now + 300, NO_CEILING)
            .is_ok());
        assert_eq!(window.last_counter(p(1, 1, 5)), now + 1000);
        assert_eq!(window.last_counter(p(1, 2, 5)), now + 500);
        assert_eq!(window.last_counter(p(2, 1, 5)), now + 300);
    }

    #[test]
    fn test_reset_principal() {
        let mut window = AntiReplayWindow::new(0, NEW_STREAM_MAX_AGE_10US);
        assert!(window
            .check_and_update(p(1, 1, 5), 1000, NO_CEILING)
            .is_ok());
        window.reset_principal(p(1, 1, 5));
        assert_eq!(window.last_counter(p(1, 1, 5)), 0);
        assert!(window.check_and_update(p(1, 1, 5), 500, NO_CEILING).is_ok());
    }

    #[test]
    fn test_zero_counter_rejected_for_new_principal() {
        let mut window = AntiReplayWindow::new(0, NEW_STREAM_MAX_AGE_10US);
        match window.check_and_update(p(1, 1, 5), 0, NO_CEILING) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack for counter=0"),
        }
    }

    /// The reboot-replay defense: with the local counter seeded from a
    /// trusted source, a captured command older than the age bound is NOT
    /// accepted as the first frame of a "new" stream.
    #[test]
    fn stale_first_frame_rejected_against_trusted_counter() {
        let boot = 100_000_000;
        let mut window = AntiReplayWindow::new(boot, NEW_STREAM_MAX_AGE_10US);

        let old = boot - NEW_STREAM_MAX_AGE_10US - 1;
        match window.check_and_update(p(1, 1, 5), old, NO_CEILING) {
            Err(AuthError::StaleNewStream {
                counter,
                local_counter,
            }) => {
                assert_eq!(counter, old);
                assert_eq!(local_counter, boot);
            }
            other => panic!("Expected StaleNewStream, got {other:?}"),
        }

        let edge = boot - NEW_STREAM_MAX_AGE_10US;
        assert!(window
            .check_and_update(p(1, 1, 5), edge, NO_CEILING)
            .is_ok());
    }

    #[test]
    fn the_high_water_mark_advances_with_accepted_frames() {
        // What gets persisted is the highest counter ever accepted, so a
        // reboot resumes from the newest thing the receiver has seen.
        let mut window = AntiReplayWindow::new(1000, NEW_STREAM_MAX_AGE_10US);
        assert_eq!(window.local_counter(), 1000);

        let far_ahead = 1000 + NEW_STREAM_MAX_AGE_10US * 10;
        assert!(window
            .check_and_update(p(1, 1, 5), far_ahead, NO_CEILING)
            .is_ok());
        assert_eq!(window.local_counter(), far_ahead);
    }

    #[test]
    fn one_principals_counter_does_not_raise_the_floor_for_another() {
        // The freshness floor stays where it was seeded. Advancing it with
        // peer-supplied counters made whichever sender had the fastest
        // clock the threshold every other principal had to clear — and the
        // value is persisted, so the lockout outlived the reboot.
        let mut window = AntiReplayWindow::new(1000, NEW_STREAM_MAX_AGE_10US);

        let far_ahead = 1000 + NEW_STREAM_MAX_AGE_10US * 10;
        assert!(window
            .check_and_update(p(1, 1, 5), far_ahead, NO_CEILING)
            .is_ok());

        // A second principal presenting a counter near the seed is still a
        // fresh first frame, and must be admitted.
        assert!(
            window
                .check_and_update(p(1, 2, 5), 2000, NO_CEILING)
                .is_ok(),
            "a peer's counter must not become another peer's floor"
        );
    }

    #[test]
    fn rejected_frames_do_not_advance_local_counter() {
        let mut window = AntiReplayWindow::new(1000, NEW_STREAM_MAX_AGE_10US);
        assert!(window
            .check_and_update(p(1, 1, 5), 5000, NO_CEILING)
            .is_ok());
        assert!(window
            .check_and_update(p(1, 1, 5), 5000, NO_CEILING)
            .is_err());
        assert_eq!(window.local_counter(), 5000);
    }

    #[test]
    fn test_capacity_exhausted_rejects_new_principal() {
        let mut window = AntiReplayWindow::new(0, NEW_STREAM_MAX_AGE_10US);
        for i in 0..MAX_SIGNING_PEERS as u8 {
            assert!(window
                .check_and_update(p(1, 1, i), 1000, NO_CEILING)
                .is_ok());
        }
        match window.check_and_update(p(9, 9, 200), 1000, NO_CEILING) {
            Err(AuthError::ReplayCapacityExhausted) => {}
            _ => panic!("Expected ReplayCapacityExhausted"),
        }
        assert!(window
            .check_and_update(p(1, 1, 0), 1001, NO_CEILING)
            .is_ok());
    }
}
