//! Watchdog Hardware Abstraction Layer
//!
//! Provides a minimal watchdog trait for system liveness monitoring.
//! The watchdog is separate from transport because:
//! - System liveness is NOT a transport concern
//! - Watchdog semantics (IWDG, WWDG) differ from communication
//! - Allows independent evolution of watchdog and transport
//!
//! ## Hardware Semantics (IWDG/WWDG)
//!
//! Hardware watchdogs (e.g., STM32 IWDG) have specific behavior:
//! - **Cannot be stopped** once started (fuse-like behavior)
//! - **Timeout is one-shot** - configured at construction, not runtime
//! - **kick() must be periodic** - missing kicks causes hardware reset
//!
//! The trait reflects these hardware realities:
//! - No `start()` method (IWDG auto-starts at construction or first kick)
//! - No `stop()` method (IWDG cannot be stopped)
//! - No `set_timeout()` method (timeout is constructor config)
//!
//! ## Usage Pattern
//!
//! ```ignore
//! // Chip HAL (e.g., aviate-hal-stm32h7)
//! impl WatchdogHal for Stm32h7Watchdog {
//!     fn kick(&mut self) {
//!         // Write to IWDG reload register
//!     }
//! }
//!
//! // Runner (aviate-runtime)
//! loop {
//!     while time.tick_ready() {
//!         board.board_step(...);
//!         watchdog.kick();  // Must kick after each control step
//!     }
//!     // ... USB servicing ...
//! }
//! ```
//!
//! ## DO-178C Compliance
//!
//! - `kick()` MUST be called within timeout period or system resets
//! - Never "gate" kicks based on system state - that causes reset
//! - Timeout should be chosen carefully:
//!   - Development: 500-1000ms (allows debug breakpoints)
//!   - Production: 50-100ms (catches real hangs quickly)

/// Watchdog trait for hardware watchdog timers
///
/// This trait provides a minimal interface for kicking hardware watchdogs.
/// Configuration (timeout) happens at construction, not through the trait.
///
/// ## Contract
///
/// - `kick()` reloads the watchdog counter, preventing reset
/// - `kick()` must be called within the configured timeout period
/// - Missing `kick()` calls cause hardware reset
/// - Never gate kicks on "system alive" flags - that causes reset!
pub trait WatchdogHal {
    /// Kick the watchdog to prevent system reset
    ///
    /// Must be called within the configured timeout period.
    /// Typically called once per control tick (1kHz = 1ms).
    ///
    /// # Timing
    ///
    /// WCET: O(1), typically a single register write (~10 cycles).
    ///
    /// # Safety
    ///
    /// Safe for windowed watchdogs when called once per tick.
    /// Do NOT call more frequently than necessary (e.g., in spin loops).
    fn kick(&mut self);
}

/// Fake watchdog for SITL and testing
///
/// Tracks kick count for testing but does nothing on hardware.
/// Useful for unit tests and SITL where no real watchdog exists.
#[derive(Debug, Clone, Default)]
pub struct FakeWatchdog {
    /// Number of times kick() has been called
    pub kick_count: u64,
    /// Simulated timeout (for testing timeout scenarios)
    pub timeout_ms: u32,
}

impl FakeWatchdog {
    /// Create a new fake watchdog
    pub fn new() -> Self {
        Self {
            kick_count: 0,
            timeout_ms: 1000, // Default 1 second
        }
    }

    /// Create with custom timeout (for testing)
    pub fn with_timeout_ms(timeout_ms: u32) -> Self {
        Self {
            kick_count: 0,
            timeout_ms,
        }
    }

    /// Reset kick counter (for testing)
    pub fn reset_count(&mut self) {
        self.kick_count = 0;
    }
}

impl WatchdogHal for FakeWatchdog {
    fn kick(&mut self) {
        self.kick_count = self.kick_count.saturating_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fake_watchdog_new() {
        let wdg = FakeWatchdog::new();
        assert_eq!(wdg.kick_count, 0);
        assert_eq!(wdg.timeout_ms, 1000);
    }

    #[test]
    fn test_fake_watchdog_kick() {
        let mut wdg = FakeWatchdog::new();
        assert_eq!(wdg.kick_count, 0);

        wdg.kick();
        assert_eq!(wdg.kick_count, 1);

        wdg.kick();
        wdg.kick();
        assert_eq!(wdg.kick_count, 3);
    }

    #[test]
    fn test_fake_watchdog_with_timeout() {
        let wdg = FakeWatchdog::with_timeout_ms(500);
        assert_eq!(wdg.timeout_ms, 500);
    }

    #[test]
    fn test_fake_watchdog_reset() {
        let mut wdg = FakeWatchdog::new();
        wdg.kick();
        wdg.kick();
        assert_eq!(wdg.kick_count, 2);

        wdg.reset_count();
        assert_eq!(wdg.kick_count, 0);
    }
}
