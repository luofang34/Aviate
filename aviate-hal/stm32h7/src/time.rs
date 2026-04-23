//! STM32H7 Time Implementation
//!
//! DWT (Data Watchpoint and Trace) based time source for STM32H7 family.
//! All Cortex-M7 cores have DWT CYCCNT which provides cycle-accurate timing.
//!
//! ## Architecture
//!
//! - `now_us()`: Uses DWT CYCCNT with rollover tracking (32-bit → 64-bit)
//! - `sleep_until_us()`: Delegates to board-provided `SleepTimer` trait
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_hal_stm32h7::time::{Stm32h7Time, SleepTimer};
//!
//! // Create with a concrete SleepTimer implementation
//! let sleep_timer = MyBoardSleepTimer::new(tim6);
//! let mut time = Stm32h7Time::new(480, sleep_timer);  // 480 MHz CPU
//!
//! // Use via TimeHal trait
//! let now = time.now_us();
//! time.sleep_until_us(now + 1000);  // Sleep 1ms
//! ```
//!
//! ## DWT CYCCNT Rollover
//!
//! At 480 MHz, 32-bit CYCCNT rolls over every ~8.9 seconds.
//! This implementation tracks rollover to provide 64-bit monotonic time.
//!
//! **Constraint**: `now_us()` must be called at least once every 8 seconds
//! to detect rollover. The control loop at 1kHz satisfies this easily.

use aviate_hal_io::TimeHal;

/// Sleep timer trait for board-specific timer implementation
///
/// Boards implement this trait to provide efficient sleep via timer compare.
/// The timer should:
/// 1. Set a compare value for the target time delta
/// 2. Enable compare interrupt
/// 3. WFI (wait for interrupt)
/// 4. Clear interrupt and return
///
/// ## Contract
///
/// - `sleep_for_us(0)` should return immediately (no sleep)
/// - `sleep_for_us(delta)` should sleep for approximately `delta` microseconds
/// - Precision depends on timer resolution (typically 1us at 1MHz timer clock)
/// - Must be non-blocking internally (no polling loops)
pub trait SleepTimer {
    /// Sleep for the specified duration in microseconds
    ///
    /// Uses timer compare + WFI for efficient sleep.
    /// Returns immediately if `delta_us` is 0.
    fn sleep_for_us(&mut self, delta_us: u32);
}

/// No-op sleep timer for testing or busy-wait fallback
///
/// This implementation does nothing - the caller will busy-wait in the
/// control loop's catch-up mechanism. NOT recommended for production as
/// it wastes CPU cycles.
pub struct NoSleep;

impl SleepTimer for NoSleep {
    fn sleep_for_us(&mut self, _delta_us: u32) {
        // No-op: caller will busy-wait
        // COV:EXCL(STUB) - Hardware-only code
    }
}

/// STM32H7 DWT-based time source
///
/// Uses DWT CYCCNT for microsecond-resolution timing.
/// Requires a `SleepTimer` implementation for `sleep_until_us()`.
///
/// ## Type Parameters
///
/// - `S`: Sleep timer implementation (board-specific)
///
/// ## Fields
///
/// - `last_cyccnt`: Last CYCCNT value for rollover detection
/// - `time_us`: Accumulated time in microseconds (64-bit, no rollover)
/// - `cpu_freq_mhz`: CPU frequency in MHz (e.g., 480 for STM32H743)
/// - `sleep`: Sleep timer implementation
pub struct Stm32h7Time<S: SleepTimer> {
    last_cyccnt: u32,
    time_us: u64,
    cpu_freq_mhz: u32,
    sleep: S,
}

impl<S: SleepTimer> Stm32h7Time<S> {
    /// Create a new DWT time source
    ///
    /// # Arguments
    ///
    /// - `cpu_freq_mhz`: CPU frequency in MHz (e.g., 480 for STM32H743 @ max speed)
    /// - `sleep`: Sleep timer implementation for `sleep_until_us()`
    ///
    /// # Panics
    ///
    /// Panics if `cpu_freq_mhz` is 0.
    ///
    /// # DWT Initialization
    ///
    /// The caller must ensure DWT CYCCNT is enabled before calling this.
    /// Typically done during board init:
    ///
    /// ```ignore
    /// // Enable DWT CYCCNT
    /// let mut core = cortex_m::Peripherals::take().unwrap();
    /// core.DCB.enable_trace();
    /// core.DWT.enable_cycle_counter();
    /// ```
    pub fn new(cpu_freq_mhz: u32, sleep: S) -> Self {
        // COV:EXCL_START(STUB) - Hardware-only code
        assert!(cpu_freq_mhz > 0, "CPU frequency must be > 0");
        Self {
            last_cyccnt: 0,
            time_us: 0,
            cpu_freq_mhz,
            sleep,
        }
        // COV:EXCL_STOP
    }

    /// Read DWT CYCCNT register
    ///
    /// # Safety
    ///
    /// This reads from a memory-mapped register. Safe because:
    /// - DWT CYCCNT is read-only from software
    /// - Single-instruction read is atomic
    #[inline]
    fn read_cyccnt() -> u32 {
        // COV:EXCL_START(STUB) - Hardware-only code
        // SAFETY: DWT CYCCNT is a read-only register at 0xE0001004
        // Reading it has no side effects.
        #[cfg(target_arch = "arm")]
        {
            const DWT_CYCCNT: *const u32 = 0xE000_1004 as *const u32;
            // SAFETY: Memory-mapped register read, no side effects
            unsafe { core::ptr::read_volatile(DWT_CYCCNT) }
        }

        #[cfg(not(target_arch = "arm"))]
        {
            // For testing on host - return incrementing value
            static mut MOCK_CYCCNT: u32 = 0;
            // SAFETY: Single-threaded test environment
            unsafe {
                MOCK_CYCCNT = MOCK_CYCCNT.wrapping_add(480); // ~1us at 480MHz
                MOCK_CYCCNT
            }
        }
        // COV:EXCL_STOP
    }
}

impl<S: SleepTimer> TimeHal for Stm32h7Time<S> {
    /// Get current time in microseconds (monotonic, handles rollover)
    ///
    /// Uses DWT CYCCNT with 32-bit to 64-bit extension via rollover tracking.
    ///
    /// # Constraint
    ///
    /// Must be called at least once every ~8 seconds (at 480 MHz) to detect
    /// rollover. The 1kHz control loop satisfies this.
    fn now_us(&mut self) -> u64 {
        // COV:EXCL_START(STUB) - Hardware-only code
        let cyccnt = Self::read_cyccnt();

        // Calculate elapsed cycles since last read, handling rollover
        let elapsed_cycles = if cyccnt >= self.last_cyccnt {
            cyccnt - self.last_cyccnt
        } else {
            // Rollover: cycles from last to MAX + cycles from 0 to current
            (u32::MAX - self.last_cyccnt)
                .wrapping_add(cyccnt)
                .wrapping_add(1)
        };

        // Convert cycles to microseconds and accumulate
        // Division by MHz gives microseconds directly
        let elapsed_us = elapsed_cycles / self.cpu_freq_mhz;
        self.time_us = self.time_us.wrapping_add(elapsed_us as u64);
        self.last_cyccnt = cyccnt;

        self.time_us
        // COV:EXCL_STOP
    }

    /// Sleep until the specified time
    ///
    /// If `target_us` is in the past, returns immediately.
    /// Otherwise, calculates delta and delegates to the sleep timer.
    ///
    /// # Implementation
    ///
    /// 1. Read current time
    /// 2. Calculate delta to target
    /// 3. If delta > 0, call `sleep.sleep_for_us(delta)`
    fn sleep_until_us(&mut self, target_us: u64) {
        // COV:EXCL_START(STUB) - Hardware-only code
        let now = self.now_us();

        // Check if target is in the past (wrapping-safe comparison)
        // If (target - now) as i64 < 0, target is in the past
        let delta = target_us.wrapping_sub(now);
        if (delta as i64) <= 0 {
            return; // Already past target
        }

        // Clamp delta to u32 max (timer can't sleep longer anyway)
        let delta_us = if delta > u32::MAX as u64 {
            u32::MAX
        } else {
            delta as u32
        };

        self.sleep.sleep_for_us(delta_us);
        // COV:EXCL_STOP
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock sleep timer that records the last sleep call
    struct MockSleep {
        last_delta_us: u32,
        call_count: u32,
    }

    impl MockSleep {
        fn new() -> Self {
            Self {
                last_delta_us: 0,
                call_count: 0,
            }
        }
    }

    impl SleepTimer for MockSleep {
        fn sleep_for_us(&mut self, delta_us: u32) {
            self.last_delta_us = delta_us;
            self.call_count += 1;
        }
    }

    #[test]
    fn test_no_sleep() {
        let mut sleep = NoSleep;
        sleep.sleep_for_us(1000); // Should do nothing
    }

    #[test]
    #[should_panic(expected = "CPU frequency must be > 0")]
    fn test_zero_cpu_freq_panics() {
        let _time = Stm32h7Time::new(0, NoSleep);
    }

    #[test]
    fn test_time_creation() {
        let time = Stm32h7Time::new(480, NoSleep);
        assert_eq!(time.cpu_freq_mhz, 480);
        assert_eq!(time.time_us, 0);
    }

    #[test]
    fn test_mock_sleep_records_calls() {
        let mut sleep = MockSleep::new();
        sleep.sleep_for_us(100);
        assert_eq!(sleep.last_delta_us, 100);
        assert_eq!(sleep.call_count, 1);

        sleep.sleep_for_us(200);
        assert_eq!(sleep.last_delta_us, 200);
        assert_eq!(sleep.call_count, 2);
    }
}
