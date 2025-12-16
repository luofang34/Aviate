//! USB Runtime Static Allocation
//!
//! This module contains the static allocation for USB device and
//! the minimal ISR handler. **All unsafe code** for USB is confined here.
//!
//! ## Safety Contract
//!
//! The ISR does ONLY:
//! 1. Mask OTG_FS IRQ at NVIC level
//! 2. Set pending flag + increment counter
//! 3. Return immediately (no USB stack calls, no byte copying)
//!
//! This ensures trivial ISR WCET - no stack-dependent work in ISR.
//!
//! ## Static Resources
//!
//! USB requires static buffers because the USB device and endpoints
//! must have stable addresses throughout device lifetime.
//!
//! ## HAL Contract
//!
//! - `disable_usb_irq()`: Masks OTG_FS IRQ at NVIC level (called from ISR)
//! - `enable_usb_irq()`: Unmasks OTG_FS IRQ (called from main loop)
//!
//! Main loop does bounded service, then re-enables IRQ.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};

// ============================================================================
// Static State (ISR Communication)
// ============================================================================

/// Flag indicating USB IRQ has fired and needs servicing
///
/// ISR sets this; main loop clears after servicing.
/// Uses AcqRel ordering for proper synchronization.
pub static USB_IRQ_PENDING: AtomicBool = AtomicBool::new(false);

/// Counter of USB IRQs for metrics/flood detection
///
/// Monotonically increasing. Used for:
/// - High-water mark calculation
/// - Flood detection (too many IRQs per frame)
pub static USB_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

// ============================================================================
// IRQ Control Functions (HAL Wrapper)
// ============================================================================

/// Disable USB IRQ from ISR context
///
/// Called by OTG_FS ISR to mask itself before setting pending flag.
/// This ensures no re-entry while main loop services USB.
///
/// # Safety
///
/// This function accesses NVIC registers to mask the IRQ.
/// Safe to call from ISR context.
#[inline]
pub fn disable_usb_irq_from_isr() {
    stm32h7xx_hal::pac::NVIC::mask(stm32h7xx_hal::pac::Interrupt::OTG_FS);
}

/// Enable USB IRQ from main loop
///
/// Called after main loop has finished servicing USB.
/// Re-enables OTG_FS IRQ at NVIC level.
///
/// # Safety
///
/// This function accesses NVIC registers to unmask the IRQ.
/// Must be called from main loop (not ISR) after servicing.
#[inline]
pub fn enable_usb_irq() {
    unsafe { stm32h7xx_hal::pac::NVIC::unmask(stm32h7xx_hal::pac::Interrupt::OTG_FS) };
}

// ============================================================================
// ISR Handler (Commented Out - Hardware Implementation)
// ============================================================================
//
// OTG_FS Interrupt Handler
//
// This ISR does ONLY:
// 1. Mask OTG_FS IRQ at NVIC level
// 2. Set pending flag
// 3. Increment counter
// 4. Return immediately
//
// NO USB stack calls. NO byte copying. NO peripheral register access
// beyond NVIC mask.
//
// WCET is trivial and does not depend on USB stack state.
//
// Safety:
//
// This function is called by hardware interrupt. It only touches:
// - NVIC (to mask OTG_FS IRQ)
// - Atomic statics (USB_IRQ_PENDING, USB_IRQ_COUNT)
//
// All USB protocol work happens in main loop via `service()`.
//
// COV:EXCL_START(STUB) - Hardware-only function
// OTG_FS Interrupt Handler
use stm32h7xx_hal::pac::interrupt;

#[interrupt]
fn OTG_FS() {
    // 1. Disable/mask OTG_FS IRQ via HAL wrapper
    disable_usb_irq_from_isr();

    // 2. Signal main loop
    USB_IRQ_PENDING.store(true, Ordering::Release);

    // 3. Increment counter for metrics/flood detection
    USB_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
}

// ============================================================================
// Static USB Resources
// ============================================================================

/// USB endpoint buffer size (Full-Speed = 64 bytes)
pub const USB_EP_BUF_SIZE: usize = 64;

/// USB RX buffer size (multiple MAVLink frames)
pub const USB_RX_BUF_SIZE: usize = 512;

/// USB TX buffer size
pub const USB_TX_BUF_SIZE: usize = 512;

// USB_BUS_STORAGE
/// Static USB bus allocator - must live for 'static
/// Uses USB2 (OTG_FS) since board uses PA11/PA12
pub static mut USB_BUS_STORAGE: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB2>>> = None;

// ============================================================================
// Bounded Service Constants
// ============================================================================

/// Maximum bytes processed per service() call
///
/// Bounds the work done in a single service iteration.
/// Set to 2x CDC packet size for reasonable throughput.
pub const SERVICE_MAX_BYTES: usize = 128;

/// Maximum USB packets processed per service() call
pub const SERVICE_MAX_PACKETS: usize = 4;

/// Maximum service() iterations per frame
///
/// Bounds CPU usage even under IRQ flood.
/// Main loop calls service() up to this many times while IRQ pending.
pub const SERVICE_MAX_ITERS: u32 = 8;

/// Maximum time USB IRQ may remain masked (microseconds)
///
/// Derived from: SERVICE_MAX_ITERS × service_wcet_us (with margin)
///
/// This is a conservative assumption for production builds.
/// Development builds measure WCET via DWT CYCCNT.
pub const USB_IRQ_MASK_MAX_US: u32 = 200;

/// Threshold for detecting IRQ immediate re-fire (HAL contract validation)
///
/// If IRQ re-fires immediately after unmask too many times,
/// indicates HAL contract violation (service() not clearing IRQ sources).
pub const IRQ_REFIRE_THRESHOLD: u32 = 10;

// ============================================================================
// Helper Functions
// ============================================================================

/// Check if USB IRQ is pending
///
/// Used by main loop to determine if extra service iterations needed.
#[inline]
pub fn is_usb_irq_pending() -> bool {
    USB_IRQ_PENDING.load(Ordering::Acquire)
}

/// Clear USB IRQ pending flag
///
/// Called by main loop after servicing.
/// Returns previous value (useful for conditional service loops).
#[inline]
pub fn clear_usb_irq_pending() -> bool {
    USB_IRQ_PENDING.swap(false, Ordering::AcqRel)
}

/// Get USB IRQ count (for metrics)
#[inline]
pub fn usb_irq_count() -> u32 {
    USB_IRQ_COUNT.load(Ordering::Relaxed)
}

/// Reset USB IRQ count (for per-frame metrics)
#[inline]
pub fn reset_usb_irq_count() -> u32 {
    USB_IRQ_COUNT.swap(0, Ordering::Relaxed)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_irq_pending_flag() {
        // Start cleared
        USB_IRQ_PENDING.store(false, Ordering::SeqCst);
        assert!(!is_usb_irq_pending());

        // Set pending
        USB_IRQ_PENDING.store(true, Ordering::SeqCst);
        assert!(is_usb_irq_pending());

        // Clear and get previous
        let was_pending = clear_usb_irq_pending();
        assert!(was_pending);
        assert!(!is_usb_irq_pending());

        // Clear again (already cleared)
        let was_pending = clear_usb_irq_pending();
        assert!(!was_pending);
    }

    #[test]
    fn test_irq_count() {
        // Reset to 0
        USB_IRQ_COUNT.store(0, Ordering::SeqCst);
        assert_eq!(usb_irq_count(), 0);

        // Increment
        USB_IRQ_COUNT.fetch_add(1, Ordering::SeqCst);
        USB_IRQ_COUNT.fetch_add(1, Ordering::SeqCst);
        assert_eq!(usb_irq_count(), 2);

        // Reset and get count
        let count = reset_usb_irq_count();
        assert_eq!(count, 2);
        assert_eq!(usb_irq_count(), 0);
    }

    #[test]
    fn test_constants() {
        // Verify constants are reasonable
        assert_eq!(USB_EP_BUF_SIZE, 64);
        assert_eq!(USB_RX_BUF_SIZE, 512);
        assert_eq!(USB_TX_BUF_SIZE, 512);
        assert_eq!(SERVICE_MAX_BYTES, 128);
        assert_eq!(SERVICE_MAX_ITERS, 8);
        assert!(USB_IRQ_MASK_MAX_US > 0);
    }
}
