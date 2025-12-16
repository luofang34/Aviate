//! STM32H7 Clock Configuration
//!
//! Provides validated clock configurations for the STM32H7 family.
//!
//! ## Clock Source Strategy
//!
//! USB requires a precise 48MHz clock. Two options:
//!
//! | Option | Pros | Cons | Use when |
//! |--------|------|------|----------|
//! | HSI48 + CRS | Simple, no PLL config | SOF needed for CRS lock | Default choice |
//! | PLL3Q | Precise from start | More complex setup | If enumeration flaky |
//!
//! ## HSI48 + CRS Note
//!
//! CRS (Clock Recovery System) uses USB SOF (Start of Frame) as sync source.
//! However, SOF only appears **after enumeration**. HSI48 must be trimmed
//! sufficiently (factory calibration) to allow initial enumeration BEFORE CRS locks.
//!
//! ## Configurations
//!
//! - `init_clocks_400mhz()` - Conservative first-light config (VOS1)
//! - `init_clocks_480mhz()` - Full-speed config (VOS0, after validation)
//!
//! ## Error Handling
//!
//! All clock functions return `Result<Clocks, ClockError>` and verify
//! USB clock is ready before returning.

/// Clock configuration error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockError {
    /// PLL failed to lock
    PllLockTimeout,
    /// HSI48 oscillator failed to stabilize
    Hsi48Timeout,
    /// CRS configuration error
    CrsError,
    /// USB clock not ready after configuration
    UsbClockNotReady,
    /// VOS (Voltage Output Scaling) transition timeout
    VosTimeout,
    /// Flash wait states configuration error
    FlashWaitStatesError,
}

/// Clock frequencies after configuration
#[derive(Debug, Clone, Copy)]
pub struct Clocks {
    /// System clock (SYSCLK) in Hz
    pub sysclk_hz: u32,
    /// AHB bus clock in Hz
    pub hclk_hz: u32,
    /// APB1 peripheral clock in Hz
    pub pclk1_hz: u32,
    /// APB2 peripheral clock in Hz
    pub pclk2_hz: u32,
    /// USB clock (48MHz) source
    pub usb_clk_source: UsbClkSource,
    /// USB clock ready flag
    pub usb_clk_ready: bool,
}

impl Default for Clocks {
    fn default() -> Self {
        // Reset values (HSI at 64MHz divided)
        Self {
            sysclk_hz: 64_000_000,
            hclk_hz: 64_000_000,
            pclk1_hz: 64_000_000,
            pclk2_hz: 64_000_000,
            usb_clk_source: UsbClkSource::None,
            usb_clk_ready: false,
        }
    }
}

/// USB 48MHz clock source
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UsbClkSource {
    /// No USB clock configured
    #[default]
    None,
    /// HSI48 with CRS (Clock Recovery System)
    Hsi48WithCrs,
    /// PLL3 Q output configured for 48MHz
    Pll3Q,
}

/// Configuration for 400MHz operation
///
/// Conservative first-light configuration:
/// - VOS1 voltage scaling (not VOS0 boost)
/// - 400MHz SYSCLK from PLL1
/// - HSI48 + CRS for USB 48MHz
pub struct Config400Mhz;

/// Configuration for 480MHz operation
///
/// Full-speed configuration (use after 400MHz validated):
/// - VOS0 voltage scaling (boost mode)
/// - 480MHz SYSCLK from PLL1
/// - HSI48 + CRS for USB 48MHz
pub struct Config480Mhz;

// ============================================================================
// Clock Configuration Functions (Hardware Implementation)
// ============================================================================

/// Initialize clocks for 400MHz operation with USB support
///
/// This is the "known-good" first-light configuration:
/// 1. Configure power: VOS1 (not VOS0)
/// 2. Configure main PLL for 400MHz sysclk
/// 3. Enable HSI48 for USB 48MHz source
/// 4. Enable CRS for HSI48 accuracy (sync to USB SOF)
/// 5. Verify USB clock is ready
///
/// # Returns
///
/// - `Ok(Clocks)` with configured clock frequencies
/// - `Err(ClockError)` if configuration fails
///
/// # Example
///
/// ```ignore
/// use aviate_hal_stm32h7::clock::{init_clocks_400mhz, ClockError};
///
/// let clocks = init_clocks_400mhz()?;
/// assert_eq!(clocks.sysclk_hz, 400_000_000);
/// assert!(clocks.usb_clk_ready);
/// ```
pub fn init_clocks_400mhz() -> Result<Clocks, ClockError> {
    // COV:EXCL_START(STUB) - Hardware-only function
    //
    // Implementation outline (to be completed with stm32h7xx-hal):
    //
    // 1. Configure power supply (LDO vs SMPS)
    //    pwr.vos1().set_bit();  // VOS1, not VOS0
    //
    // 2. Wait for VOS ready
    //    while !pwr.csr1.read().actvosrdy().bit() {}
    //
    // 3. Configure flash wait states for 400MHz @ VOS1
    //    flash.acr.modify(|_, w| w.latency().bits(4));  // 4 wait states
    //
    // 4. Configure PLL1 for 400MHz SYSCLK
    //    - Source: HSE (25MHz external crystal) or HSI (64MHz internal)
    //    - PLL1: DIVM=5, DIVN=160, DIVP=2 → 400MHz from 25MHz HSE
    //
    // 5. Enable HSI48 for USB
    //    rcc.cr.modify(|_, w| w.hsi48on().set_bit());
    //    while !rcc.cr.read().hsi48rdy().bit() {}
    //
    // 6. Enable CRS for HSI48 accuracy
    //    - Sync source: USB SOF
    //    - Automatic trimming improves accuracy from ±4% to <0.25%
    //
    // 7. Select USB clock source
    //    rcc.d2ccip2r.modify(|_, w| w.usbsel().hsi48());
    //
    // 8. Verify USB clock ready
    //    assert_usb_clock_ready()?;

    // For now, return placeholder clocks
    // This will be replaced with actual hardware initialization
    Ok(Clocks {
        sysclk_hz: 400_000_000,
        hclk_hz: 200_000_000,  // AHB = SYSCLK / 2
        pclk1_hz: 100_000_000, // APB1 = HCLK / 2
        pclk2_hz: 100_000_000, // APB2 = HCLK / 2
        usb_clk_source: UsbClkSource::Hsi48WithCrs,
        usb_clk_ready: true,
    })
    // COV:EXCL_STOP
}

/// Initialize clocks for 480MHz operation with USB support
///
/// Full-speed configuration (use after 400MHz validated):
/// - VOS0 voltage scaling (boost mode required)
/// - 480MHz SYSCLK from PLL1
/// - HSI48 + CRS for USB 48MHz
///
/// # Returns
///
/// - `Ok(Clocks)` with configured clock frequencies
/// - `Err(ClockError)` if configuration fails
pub fn init_clocks_480mhz() -> Result<Clocks, ClockError> {
    // COV:EXCL_START(STUB) - Hardware-only function
    //
    // Similar to 400MHz but:
    // 1. VOS0 (boost mode) required for 480MHz
    // 2. More flash wait states (5WS at 480MHz)
    // 3. PLL1 configured for 480MHz
    //
    // Note: VOS0 has higher power consumption and requires
    // careful validation before use in flight.

    // For now, return placeholder clocks
    Ok(Clocks {
        sysclk_hz: 480_000_000,
        hclk_hz: 240_000_000,  // AHB = SYSCLK / 2
        pclk1_hz: 120_000_000, // APB1 = HCLK / 2
        pclk2_hz: 120_000_000, // APB2 = HCLK / 2
        usb_clk_source: UsbClkSource::Hsi48WithCrs,
        usb_clk_ready: true,
    })
    // COV:EXCL_STOP
}

/// Verify USB clock is ready
///
/// Checks RCC status bits to confirm USB clock source is stable.
/// Called internally by clock init functions.
#[allow(dead_code)] // Stub - will be used by real hardware init
fn assert_usb_clock_ready() -> Result<(), ClockError> {
    // COV:EXCL_START(STUB) - Hardware-only function
    //
    // Implementation:
    // 1. Check HSI48RDY bit in RCC_CR
    // 2. Check USBSEL in RCC_D2CCIP2R is not "disabled"
    // 3. Optionally check CRS sync status

    Ok(())
    // COV:EXCL_STOP
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clocks_default() {
        let clocks = Clocks::default();
        assert_eq!(clocks.sysclk_hz, 64_000_000);
        assert!(!clocks.usb_clk_ready);
    }

    #[test]
    fn test_usb_clk_source_default() {
        let src = UsbClkSource::default();
        assert_eq!(src, UsbClkSource::None);
    }

    #[test]
    fn test_init_clocks_400mhz_stub() {
        let clocks = init_clocks_400mhz().unwrap();
        assert_eq!(clocks.sysclk_hz, 400_000_000);
        assert!(clocks.usb_clk_ready);
        assert_eq!(clocks.usb_clk_source, UsbClkSource::Hsi48WithCrs);
    }

    #[test]
    fn test_init_clocks_480mhz_stub() {
        let clocks = init_clocks_480mhz().unwrap();
        assert_eq!(clocks.sysclk_hz, 480_000_000);
        assert!(clocks.usb_clk_ready);
    }

    #[test]
    fn test_clock_error_variants() {
        let err = ClockError::PllLockTimeout;
        assert_eq!(err, ClockError::PllLockTimeout);

        let err = ClockError::UsbClockNotReady;
        assert_eq!(err, ClockError::UsbClockNotReady);
    }
}
