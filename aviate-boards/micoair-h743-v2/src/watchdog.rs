//! Independent Watchdog (IWDG) Implementation
//!
//! Real hardware watchdog for MicoAir H743-V2 using STM32H7 IWDG peripheral.
//!
//! ## IWDG Characteristics
//!
//! - **Cannot be stopped**: Once started, IWDG runs until reset
//! - **Separate clock**: Runs on LSI (~32kHz), independent of system clock
//! - **Timeout range**: ~125µs to ~32.7s (at 32kHz LSI)
//! - **Window mode**: Optional - can reject early kicks (not used here)
//!
//! ## Usage
//!
//! ```ignore
//! // Create and start watchdog with 500ms timeout
//! let mut watchdog = BoardWatchdog::new(dp.IWDG, 500);
//!
//! // In main loop
//! loop {
//!     do_work();
//!     watchdog.kick();
//! }
//! ```

use aviate_hal_io::WatchdogHal;
use fugit::ExtU32;
use stm32h7xx_hal::independent_watchdog::IndependentWatchdog;
use stm32h7xx_hal::pac::IWDG;

/// Board-level watchdog wrapper
///
/// Wraps the stm32h7xx-hal IndependentWatchdog with the WatchdogHal trait.
/// Starts immediately upon construction - be sure clocks are stable first!
pub struct BoardWatchdog {
    iwdg: IndependentWatchdog,
    /// Configured timeout in milliseconds (for debugging)
    timeout_ms: u32,
    /// Kick count for metrics
    kick_count: u64,
}

impl BoardWatchdog {
    /// Create and start a new watchdog with the specified timeout
    ///
    /// # Arguments
    ///
    /// * `iwdg` - IWDG peripheral from PAC
    /// * `timeout_ms` - Timeout in milliseconds (valid: 1-32000ms)
    ///
    /// # Input Validation
    ///
    /// - If timeout_ms is 0, defaults to 1ms
    /// - If timeout_ms > 32000, clamps to 32000ms
    ///
    /// # Warning
    ///
    /// **IWDG cannot be stopped once started!** Only call this after
    /// clocks are stable and you're ready to kick regularly.
    pub fn new(iwdg: IWDG, timeout_ms: u32) -> Self {
        // Clamp to valid range (defensive, no panic)
        let timeout_ms = timeout_ms.clamp(1, 32_000);

        // Create HAL watchdog wrapper
        let mut watchdog = IndependentWatchdog::new(iwdg);

        // Start with specified timeout
        // Note: stm32h7xx-hal uses fugit duration types
        watchdog.start(timeout_ms.millis());

        Self {
            iwdg: watchdog,
            timeout_ms,
            kick_count: 0,
        }
    }

    /// Get configured timeout in milliseconds
    pub fn timeout_ms(&self) -> u32 {
        self.timeout_ms
    }

    /// Get total kick count
    pub fn kick_count(&self) -> u64 {
        self.kick_count
    }
}

impl WatchdogHal for BoardWatchdog {
    /// Kick/feed the watchdog to prevent reset
    ///
    /// Must be called within the timeout period. Failing to call
    /// this method will result in a hardware reset.
    fn kick(&mut self) {
        self.iwdg.feed();
        self.kick_count = self.kick_count.saturating_add(1);
    }
}

#[cfg(test)]
mod tests {
    // Note: Real IWDG tests require hardware
    // Unit tests would need a mock IWDG peripheral
}
