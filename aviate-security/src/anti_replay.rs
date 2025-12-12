//! Anti-replay protection using per-link_id monotonic counters
//!
//! This module implements replay attack prevention for signed commands.
//! Each link_id maintains an independent monotonic counter.
//!
//! ## Security Model
//!
//! - **Per-link_id tracking**: Each ground station (identified by link_id) has
//!   an independent counter. This allows multiple independent operators.
//!
//! - **Strict monotonic**: New timestamp MUST be strictly greater than last
//!   accepted timestamp (`new > last`). No equality, no backwards movement.
//!
//! - **No skew window**: Unlike some protocols (e.g., IPsec), we do NOT allow
//!   a "replay window" for out-of-order packets. MAVLink over USB/UART is
//!   strictly ordered, so any non-monotonic timestamp is suspicious.
//!
//! ## DO-178C Properties
//!
//! - **Time complexity**: O(1) lookup and update (fixed-size array)
//! - **Memory**: 256 * 8 bytes = 2 KB (one u64 per link_id)
//! - **WCET**: ~10 CPU cycles (array index + comparison + update)
//! - **No allocation**: Stack-allocated, deterministic

use crate::errors::{AuthError, AuthResult};

/// Number of possible link IDs (0-255)
const MAX_LINK_IDS: usize = 256;

/// Anti-replay window tracking per-link_id timestamps
///
/// ## Usage Example
///
/// ```ignore
/// let mut window = AntiReplayWindow::new();
///
/// // First command from link_id=5
/// window.check_and_update(5, 1000)?;  // OK
///
/// // Second command from same link
/// window.check_and_update(5, 1001)?;  // OK (1001 > 1000)
///
/// // Replay attack (same or older timestamp)
/// window.check_and_update(5, 1000)?;  // Error: ReplayAttack
/// window.check_and_update(5, 999)?;   // Error: ReplayAttack
///
/// // Independent link_id
/// window.check_and_update(7, 500)?;   // OK (different link_id)
/// ```
pub struct AntiReplayWindow {
    /// Last accepted timestamp for each link_id
    ///
    /// - Index: link_id (0-255)
    /// - Value: Last accepted timestamp (0 = never seen)
    ///
    /// **Initialization**: All zeros means no commands accepted yet.
    /// The first command from any link_id will be accepted (timestamp > 0).
    last_timestamp: [u64; MAX_LINK_IDS],
}

impl AntiReplayWindow {
    /// Create new anti-replay window with all timestamps at zero
    ///
    /// ## Post-condition
    ///
    /// All link_ids start with `last_timestamp[i] = 0`, meaning no commands
    /// have been accepted yet. The first command from each link_id must have
    /// timestamp > 0 to be accepted.
    pub const fn new() -> Self {
        Self {
            last_timestamp: [0; MAX_LINK_IDS],
        }
    }

    /// Check if timestamp is valid and update window
    ///
    /// ## Parameters
    ///
    /// - `link_id`: Command sender identifier (0-255)
    /// - `timestamp`: Remote monotonic counter from command signature
    ///
    /// ## Returns
    ///
    /// - `Ok(())`: Timestamp is valid (strictly greater than last), window updated
    /// - `Err(AuthError::ReplayAttack)`: Timestamp is not strictly greater
    ///
    /// ## Security Invariant
    ///
    /// After successful check, `last_timestamp[link_id]` is updated to `timestamp`.
    /// Subsequent calls with same or older timestamp will fail.
    ///
    /// ## DO-178C Contract
    ///
    /// - **Time complexity**: O(1)
    /// - **WCET**: ~10 CPU cycles @ 480 MHz (array index + compare + store)
    /// - **Side effects**: Updates internal state on success, no change on failure
    /// - **Thread safety**: NOT thread-safe (requires external synchronization)
    pub fn check_and_update(&mut self, link_id: u8, timestamp: u64) -> AuthResult<()> {
        let link_id_idx = link_id as usize;
        let last = self.last_timestamp[link_id_idx];

        // Strict monotonic check: new timestamp MUST be strictly greater
        if timestamp <= last {
            return Err(AuthError::ReplayAttack);
        }

        // Accept: update window
        self.last_timestamp[link_id_idx] = timestamp;
        Ok(())
    }

    /// Get last accepted timestamp for a link_id (for debugging/telemetry)
    ///
    /// ## Returns
    ///
    /// - Last accepted timestamp for this link_id
    /// - 0 if no commands from this link_id have been accepted yet
    pub fn last_timestamp(&self, link_id: u8) -> u64 {
        self.last_timestamp[link_id as usize]
    }

    /// Reset window for a specific link_id (for testing/recovery)
    ///
    /// ## Use Cases
    ///
    /// - Testing: Reset state between test cases
    /// - Recovery: Clear stuck link_id after ground station restart
    ///
    /// ## Security Warning
    ///
    /// Resetting allows previously-seen timestamps to be replayed!
    /// Only use this in controlled scenarios (testing, operator command).
    pub fn reset_link(&mut self, link_id: u8) {
        self.last_timestamp[link_id as usize] = 0;
    }

    /// Reset all link_ids to zero (for testing only)
    ///
    /// ## Security Warning
    ///
    /// This clears all anti-replay state! Only use in test code.
    pub fn reset_all(&mut self) {
        self.last_timestamp = [0; MAX_LINK_IDS];
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
        // First command from link_id=5 with timestamp=1000
        assert!(window.check_and_update(5, 1000).is_ok());
        assert_eq!(window.last_timestamp(5), 1000);
    }

    #[test]
    fn test_monotonic_increase_accepted() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(5, 1000).is_ok());
        assert!(window.check_and_update(5, 1001).is_ok());
        assert!(window.check_and_update(5, 1002).is_ok());
        assert_eq!(window.last_timestamp(5), 1002);
    }

    #[test]
    fn test_replay_same_timestamp_rejected() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(5, 1000).is_ok());
        // Replay with same timestamp
        match window.check_and_update(5, 1000) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_replay_older_timestamp_rejected() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(5, 1000).is_ok());
        // Replay with older timestamp
        match window.check_and_update(5, 999) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack error"),
        }
    }

    #[test]
    fn test_independent_link_ids() {
        let mut window = AntiReplayWindow::new();
        // link_id=5
        assert!(window.check_and_update(5, 1000).is_ok());
        // link_id=7 (independent)
        assert!(window.check_and_update(7, 500).is_ok());
        // Each maintains own state
        assert_eq!(window.last_timestamp(5), 1000);
        assert_eq!(window.last_timestamp(7), 500);
    }

    #[test]
    fn test_reset_link() {
        let mut window = AntiReplayWindow::new();
        assert!(window.check_and_update(5, 1000).is_ok());
        // Reset link_id=5
        window.reset_link(5);
        assert_eq!(window.last_timestamp(5), 0);
        // Can now accept timestamp=500 (previously would be rejected)
        assert!(window.check_and_update(5, 500).is_ok());
    }

    #[test]
    fn test_zero_timestamp_rejected_after_init() {
        let mut window = AntiReplayWindow::new();
        // Timestamp=0 rejected if it's not the first (all start at 0)
        match window.check_and_update(5, 0) {
            Err(AuthError::ReplayAttack) => {}
            _ => panic!("Expected ReplayAttack for timestamp=0"),
        }
    }
}
