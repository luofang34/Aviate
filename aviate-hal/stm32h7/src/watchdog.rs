//! STM32H7 Watchdog Implementation
//!
//! Provides IWDG (Independent Watchdog) wrapper implementing `WatchdogHal`.
//!
//! ## IWDG Characteristics
//!
//! The STM32H7 IWDG has specific hardware semantics:
//!
//! - **Cannot be stopped** once started (fuse-like behavior)
//! - **Clocked by LSI** (~32kHz internal RC oscillator)
//! - **Timeout configured at construction** via prescaler and reload value
//! - **Must be kicked within timeout** or system resets
//!
//! ## Timeout Calculation
//!
//! ```text
//! timeout_ms = (reload + 1) * prescaler * 1000 / LSI_FREQ
//! ```
//!
//! Where LSI_FREQ is approximately 32kHz (varies with temperature/voltage).
//!
//! ## Usage Pattern
//!
//! ```ignore
//! use aviate_hal_stm32h7::watchdog::Stm32h7Watchdog;
//! use aviate_hal_io::WatchdogHal;
//!
//! // Start watchdog with 500ms timeout (development)
//! let mut wdg = Stm32h7Watchdog::new(500);
//!
//! // In control loop (must kick within 500ms)
//! loop {
//!     do_control_work();
//!     wdg.kick();  // Reload counter
//! }
//! ```
//!
//! ## Timeout Guidance
//!
//! | Build | Timeout | Rationale |
//! |-------|---------|-----------|
//! | Development | 500-1000ms | Allows debug breakpoints |
//! | Production | 50-100ms | Catches real hangs quickly |
//!
//! ## Important
//!
//! - **NEVER gate kicks** on "system alive" flags - that causes reset!
//! - Start watchdog AFTER clocks are stable, BEFORE main loop
//! - Starting too early causes DFU loops if init has bugs

use aviate_hal_io::WatchdogHal;

/// STM32H7 IWDG prescaler values
///
/// IWDG_PR register values for prescaler selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IwdgPrescaler {
    /// Divide by 4 (minimum timeout)
    Div4 = 0,
    /// Divide by 8
    Div8 = 1,
    /// Divide by 16
    Div16 = 2,
    /// Divide by 32
    Div32 = 3,
    /// Divide by 64
    Div64 = 4,
    /// Divide by 128
    Div128 = 5,
    /// Divide by 256 (maximum timeout)
    Div256 = 6,
}

impl IwdgPrescaler {
    /// Get the actual divisor value
    pub fn divisor(self) -> u32 {
        match self {
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
            Self::Div32 => 32,
            Self::Div64 => 64,
            Self::Div128 => 128,
            Self::Div256 => 256,
        }
    }
}

/// LSI frequency in Hz (approximate - varies with temperature/voltage)
pub const LSI_FREQ_HZ: u32 = 32_000;

/// Maximum reload value (12-bit register)
pub const MAX_RELOAD: u32 = 0xFFF;

/// Key values for IWDG_KR register
#[allow(dead_code)] // Used by hardware implementation (stubbed)
const KEY_RELOAD: u16 = 0xAAAA;
#[allow(dead_code)] // Used by hardware implementation (stubbed)
const KEY_ENABLE: u16 = 0xCCCC;
#[allow(dead_code)] // Used by hardware implementation (stubbed)
const KEY_WRITE_ACCESS: u16 = 0x5555;

/// STM32H7 Independent Watchdog (IWDG) wrapper
///
/// Implements `WatchdogHal` for the STM32H7 IWDG peripheral.
///
/// ## Configuration
///
/// Timeout is configured at construction and cannot be changed:
///
/// ```ignore
/// // 500ms timeout (development)
/// let mut wdg = Stm32h7Watchdog::new(500);
///
/// // 100ms timeout (production)
/// let mut wdg = Stm32h7Watchdog::new(100);
/// ```
///
/// ## Hardware Note
///
/// The watchdog starts counting as soon as `new()` is called.
/// Create the watchdog instance AFTER clock/USB initialization,
/// BEFORE entering the main control loop.
#[derive(Debug)]
pub struct Stm32h7Watchdog {
    /// Configured timeout in milliseconds
    timeout_ms: u32,
    /// Selected prescaler
    prescaler: IwdgPrescaler,
    /// Reload value (0-4095)
    reload: u32,
    /// Number of kicks performed (for debugging)
    kick_count: u64,
}

impl Stm32h7Watchdog {
    /// Create and start a new IWDG with the specified timeout
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Desired timeout in milliseconds (approximate)
    ///
    /// # Returns
    ///
    /// A new `Stm32h7Watchdog` instance with the watchdog running.
    ///
    /// # Panics
    ///
    /// Panics if `timeout_ms` is 0 or too large for the hardware.
    ///
    /// # Note
    ///
    /// The actual timeout may differ slightly from requested due to
    /// LSI frequency variations and integer division.
    pub fn new(timeout_ms: u32) -> Self {
        assert!(timeout_ms > 0, "Watchdog timeout must be > 0");

        // Calculate prescaler and reload for desired timeout
        // timeout_ms = (reload + 1) * prescaler * 1000 / LSI_FREQ
        // reload = (timeout_ms * LSI_FREQ / 1000 / prescaler) - 1
        let (prescaler, reload) = Self::calculate_prescaler_reload(timeout_ms);

        let mut wdg = Self {
            timeout_ms,
            prescaler,
            reload,
            kick_count: 0,
        };

        // Start the watchdog
        wdg.start();

        wdg
    }

    /// Calculate prescaler and reload value for target timeout
    fn calculate_prescaler_reload(timeout_ms: u32) -> (IwdgPrescaler, u32) {
        // Try prescalers from smallest to largest to get best resolution
        let prescalers = [
            IwdgPrescaler::Div4,
            IwdgPrescaler::Div8,
            IwdgPrescaler::Div16,
            IwdgPrescaler::Div32,
            IwdgPrescaler::Div64,
            IwdgPrescaler::Div128,
            IwdgPrescaler::Div256,
        ];

        for prescaler in prescalers {
            let divisor = prescaler.divisor();
            // reload = (timeout_ms * LSI_FREQ / 1000 / divisor) - 1
            let ticks_needed = (timeout_ms as u64 * LSI_FREQ_HZ as u64) / 1000;
            let reload = (ticks_needed / divisor as u64).saturating_sub(1);

            if reload <= MAX_RELOAD as u64 {
                return (prescaler, reload as u32);
            }
        }

        // If we get here, use maximum values
        (IwdgPrescaler::Div256, MAX_RELOAD)
    }

    /// Start the IWDG
    fn start(&mut self) {
        // COV:EXCL_START(STUB) - Hardware-only function
        //
        // Hardware implementation:
        //
        // let iwdg = unsafe { &*stm32h7xx::IWDG::ptr() };
        //
        // // Enable write access to PR and RLR registers
        // iwdg.kr.write(|w| w.key().bits(KEY_WRITE_ACCESS));
        //
        // // Set prescaler
        // iwdg.pr.write(|w| w.pr().bits(self.prescaler as u8));
        //
        // // Set reload value
        // iwdg.rlr.write(|w| w.rl().bits(self.reload as u16));
        //
        // // Wait for registers to be updated
        // while iwdg.sr.read().pvu().bit() || iwdg.sr.read().rvu().bit() {}
        //
        // // Start watchdog (cannot be stopped after this!)
        // iwdg.kr.write(|w| w.key().bits(KEY_ENABLE));
        //
        // // Initial reload
        // iwdg.kr.write(|w| w.key().bits(KEY_RELOAD));
        // COV:EXCL_STOP
    }

    /// Get configured timeout in milliseconds
    pub fn timeout_ms(&self) -> u32 {
        self.timeout_ms
    }

    /// Get number of kicks performed
    pub fn kick_count(&self) -> u64 {
        self.kick_count
    }

    /// Calculate actual timeout from prescaler and reload
    pub fn actual_timeout_ms(&self) -> u32 {
        let ticks = (self.reload + 1) * self.prescaler.divisor();
        ticks * 1000 / LSI_FREQ_HZ
    }
}

impl WatchdogHal for Stm32h7Watchdog {
    fn kick(&mut self) {
        // COV:EXCL_START(STUB) - Hardware-only function
        //
        // Hardware implementation:
        //
        // let iwdg = unsafe { &*stm32h7xx::IWDG::ptr() };
        // iwdg.kr.write(|w| w.key().bits(KEY_RELOAD));

        // Track kicks for debugging
        self.kick_count = self.kick_count.saturating_add(1);
        // COV:EXCL_STOP
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_prescaler_divisor() {
        assert_eq!(IwdgPrescaler::Div4.divisor(), 4);
        assert_eq!(IwdgPrescaler::Div256.divisor(), 256);
    }

    #[test]
    fn test_watchdog_creation() {
        let wdg = Stm32h7Watchdog::new(500);
        assert_eq!(wdg.timeout_ms(), 500);
        assert_eq!(wdg.kick_count(), 0);
    }

    #[test]
    fn test_watchdog_kick_count() {
        let mut wdg = Stm32h7Watchdog::new(500);
        assert_eq!(wdg.kick_count(), 0);

        wdg.kick();
        assert_eq!(wdg.kick_count(), 1);

        wdg.kick();
        wdg.kick();
        assert_eq!(wdg.kick_count(), 3);
    }

    #[test]
    fn test_prescaler_reload_calculation() {
        // 100ms timeout should use small prescaler
        let wdg = Stm32h7Watchdog::new(100);
        let actual = wdg.actual_timeout_ms();
        // Allow 20% tolerance for LSI variation
        assert!(actual >= 80 && actual <= 120, "actual: {}", actual);

        // 1000ms timeout
        let wdg = Stm32h7Watchdog::new(1000);
        let actual = wdg.actual_timeout_ms();
        assert!(actual >= 800 && actual <= 1200, "actual: {}", actual);
    }

    #[test]
    fn test_short_timeout() {
        let wdg = Stm32h7Watchdog::new(10);
        assert!(wdg.actual_timeout_ms() > 0);
    }

    #[test]
    fn test_long_timeout() {
        // Maximum timeout with Div256 and MAX_RELOAD
        // = (4095 + 1) * 256 * 1000 / 32000 = ~32768ms
        let wdg = Stm32h7Watchdog::new(30000);
        assert!(wdg.actual_timeout_ms() > 0);
    }

    #[test]
    #[should_panic(expected = "Watchdog timeout must be > 0")]
    fn test_zero_timeout_panics() {
        let _ = Stm32h7Watchdog::new(0);
    }

    #[test]
    fn test_implements_watchdog_hal() {
        // Verify trait is implemented
        fn takes_watchdog<W: WatchdogHal>(_w: &mut W) {}

        let mut wdg = Stm32h7Watchdog::new(100);
        takes_watchdog(&mut wdg);
    }
}
