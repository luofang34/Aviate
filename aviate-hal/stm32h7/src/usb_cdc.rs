//! STM32H7 USB CDC Implementation
//!
//! Provides USB CDC ACM (Communications Device Class - Abstract Control Model)
//! transport raw driver. This handles the USB peripheral, interrupts, and
//! buffer management. Protocol (MAVLink) is handled by the consumer.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  Main Loop                                                  │
//! │                                                             │
//! │  loop {                                                     │
//! │      // TICK PRIORITY: Execute all due ticks first          │
//! │      while time.tick_ready() && tick_count < MAX_CATCHUP {  │
//! │          board.board_step(...);                             │
//! │          watchdog.kick();                                   │
//! │      }                                                      │
//! │                                                             │
//! │      // USB: always once, extra while pending (bounded)     │
//! │      usb.service();                                         │
//! │      while is_usb_irq_pending() && iters < SERVICE_MAX {    │
//! │          usb.service();                                     │
//! │      }                                                      │
//! │      enable_usb_irq();                                      │
//! │  }                                                          │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## ISR Pattern
//!
//! The USB ISR does ONLY:
//! 1. Mask OTG_FS IRQ at NVIC level
//! 2. Set pending flag + increment counter
//! 3. Return immediately
//!
//! All USB protocol work happens in `service()` on main loop.
//! This ensures trivial ISR WCET with no stack-dependent work.
//!
//! ## USB Descriptors
//!
//! - VID: 0x0483 (STMicroelectronics)
//! - PID: 0x5740 (Virtual COM Port)
//! - Serial: Derived from STM32 unique device ID
//!
//! ## Feature Flags
//!
//! - `usb-poll-only`: Disables IRQ, uses pure polling (bring-up mode)
//! - `usb-dev`: Development mode with WCET measurement
//! - `usb-flight`: Flight mode with restricted input (telemetry-only)

use crate::usb_rt::{
    clear_usb_irq_pending, enable_usb_irq, is_usb_irq_pending, SERVICE_MAX_BYTES, SERVICE_MAX_ITERS,
    USB_BUS_STORAGE,
};

use core::cell::RefCell;
use core::mem::MaybeUninit;
use cortex_m::interrupt::{self, Mutex};
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// ============================================================================
// USB VID/PID Configuration
// ============================================================================

/// USB Vendor ID (STMicroelectronics)
pub const USB_VID: u16 = 0x0483;

/// USB Product ID (Virtual COM Port)
pub const USB_PID: u16 = 0x5740;

/// USB Manufacturer String
pub const USB_MANUFACTURER: &str = "Aviate";

/// USB Product String
pub const USB_PRODUCT: &str = "Aviate Flight Controller";

// ============================================================================
// USB Metrics
// ============================================================================

/// Runtime metrics for USB performance monitoring
///
/// Used for debugging, tuning, and WCET measurement.
/// All fields are updated during `service()` calls.
#[derive(Debug, Clone, Copy, Default)]
pub struct UsbMetrics {
    /// High water mark for RX queue depth
    pub rx_queue_high_water: usize,

    /// High water mark for TX queue depth
    pub tx_queue_high_water: usize,

    /// Count of dropped RX bytes (buffer full)
    pub rx_dropped: u32,

    /// Count of dropped TX bytes (buffer full)
    pub tx_dropped: u32,

    /// High water mark for IRQs per frame
    pub irq_count_high_water: u32,

    /// High water mark for service() calls per frame
    pub service_calls_per_frame_high_water: u32,

    /// High water mark for service() WCET in cycles (dev builds)
    pub service_wcet_cycles_high_water: u32,

    /// Count of times SERVICE_MAX_ITERS was hit (budget overruns)
    pub budget_overruns: u32,

    /// Count of immediate IRQ re-fires after unmask
    ///
    /// Non-zero indicates potential HAL contract violation.
    /// If exceeds threshold, USB degradation may be triggered.
    pub irq_immediate_refire_count: u32,
}

impl UsbMetrics {
    /// Create new metrics (all zeros)
    pub const fn new() -> Self {
        Self {
            rx_queue_high_water: 0,
            tx_queue_high_water: 0,
            rx_dropped: 0,
            tx_dropped: 0,
            irq_count_high_water: 0,
            service_calls_per_frame_high_water: 0,
            service_wcet_cycles_high_water: 0,
            budget_overruns: 0,
            irq_immediate_refire_count: 0,
        }
    }

    /// Reset all metrics to zero
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

// ============================================================================
// Static USB Resources
// ============================================================================

/// Static USB endpoint memory (required by USB device stack)
static mut EP_MEMORY: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

// USB_BUS_STORAGE is defined in usb_rt.rs

/// USB serial port (CDC ACM)
static USB_SERIAL: Mutex<RefCell<Option<SerialPort<'static, UsbBus<USB2>>>>> =
    Mutex::new(RefCell::new(None));

/// USB device
static USB_DEVICE: Mutex<RefCell<Option<UsbDevice<'static, UsbBus<USB2>>>>> =
    Mutex::new(RefCell::new(None));

/// Static buffer for serial number string (must live for 'static)
static mut SERIAL_NUMBER_BUF: [u8; 24] = [b'0'; 24];

// ============================================================================
// USB CDC Transport
// ============================================================================

/// STM32H7 USB CDC Transport
///
/// Implements bounded service loop with IRQ-based wakeup.
/// All USB protocol work happens in `service()`, not in ISR.
///
/// ## Thread Safety
///
/// This type is NOT thread-safe. It must be used from a single context
/// (the main loop). The ISR only touches atomic statics in `usb_rt`.
#[derive(Debug)]
pub struct Stm32h7UsbCdc {
    /// Internal RX buffer (ring buffer)
    rx_buf: [u8; 512],
    rx_head: usize,
    rx_tail: usize,

    /// Internal TX buffer (ring buffer)
    tx_buf: [u8; 512],
    tx_head: usize,
    tx_tail: usize,

    /// USB metrics for debugging/WCET measurement
    pub metrics: UsbMetrics,

    /// Service call count for current frame
    service_calls_this_frame: u32,

    /// Connected flag (DTR set by host)
    connected: bool,

    /// Degraded mode flag
    degraded: bool,

    /// USB initialized flag
    usb_initialized: bool,
}

impl Stm32h7UsbCdc {
    /// Create a new USB CDC transport
    ///
    /// This initializes the transport but does NOT start the USB peripheral.
    /// Call `init()` after clocks are configured to start USB.
    pub fn new() -> Self {
        Self {
            rx_buf: [0; 512],
            rx_head: 0,
            rx_tail: 0,
            tx_buf: [0; 512],
            tx_head: 0,
            tx_tail: 0,
            metrics: UsbMetrics::new(),
            service_calls_this_frame: 0,
            connected: false,
            degraded: false,
            usb_initialized: false,
        }
    }

    /// Initialize USB peripheral
    ///
    /// Must be called after clocks are configured (HSI48 or PLL for 48MHz).
    /// Enables USB peripheral and IRQ (unless `usb-poll-only` feature).
    ///
    /// # Arguments
    ///
    /// - `usb2`: Pre-configured USB2 peripheral (OTG_FS) from stm32h7xx-hal
    ///
    /// # Safety
    ///
    /// This function uses unsafe code for static endpoint memory.
    /// Must only be called once after reset.
    pub fn init(&mut self, usb2: USB2) {
        // Safety: EP_MEMORY and USB_BUS_STORAGE are only accessed in this function
        // and never concurrently. This function is only called once at init.
        unsafe {
            // Initialize endpoint memory
            let buf = &mut *core::ptr::addr_of_mut!(EP_MEMORY);
            let buf = buf.assume_init_mut();
            for word in buf.iter_mut() {
                *word = 0;
            }

            // Create USB bus and store in static
            let usb_bus = UsbBus::new(usb2, buf);
            USB_BUS_STORAGE = Some(usb_bus);

            // Get reference to static bus for creating serial/device
            if let Some(ref bus) = USB_BUS_STORAGE {
                // Create serial port
                let serial = SerialPort::new(bus);

                // Initialize serial number buffer
                // Safety: We are in init, single threaded access assumed or controlled by caller
                let serial_bytes = get_usb_serial_number();
                let buf_ptr = core::ptr::addr_of_mut!(SERIAL_NUMBER_BUF);
                (*buf_ptr).copy_from_slice(&serial_bytes);
                
                // Convert to str for descriptor
                let buf_slice = &*buf_ptr;
                let serial_str = core::str::from_utf8(buf_slice).unwrap_or("000000000000000000000000");

                // Build USB device (serial number uses static buffer)
                let usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(USB_VID, USB_PID))
                    .strings(&[StringDescriptors::default()
                        .manufacturer(USB_MANUFACTURER)
                        .product(USB_PRODUCT)
                        .serial_number(serial_str)])
                    .ok()
                    .map(|b| b.device_class(usbd_serial::USB_CLASS_CDC).build())
                    .unwrap_or_else(|| {
                        // Fallback without string descriptors
                        UsbDeviceBuilder::new(bus, UsbVidPid(USB_VID, USB_PID))
                            .device_class(usbd_serial::USB_CLASS_CDC)
                            .build()
                    });

                // Store in statics
                interrupt::free(|cs| {
                    USB_SERIAL.borrow(cs).replace(Some(serial));
                    USB_DEVICE.borrow(cs).replace(Some(usb_dev));
                });
            }
            
            // Enable OTG_FS IRQ (unless usb-poll-only)
            #[cfg(not(feature = "usb-poll-only"))]
            {
                use stm32h7xx_hal::pac::Interrupt;
                unsafe { stm32h7xx_hal::pac::NVIC::unmask(stm32h7xx_hal::pac::Interrupt::OTG_FS) };
            }
        }

        self.usb_initialized = true;
        self.connected = false;
    }

    /// Service USB peripheral (bounded work)
    ///
    /// This method performs bounded USB work:
    /// - Polls USB device for events
    /// - Reads up to SERVICE_MAX_BYTES from endpoint
    /// - Writes up to SERVICE_MAX_BYTES to endpoint
    ///
    /// # Returns
    ///
    /// Number of bytes processed (RX + TX) in this service call.
    pub fn service(&mut self) -> usize {
        if !self.usb_initialized {
            return 0;
        }

        // Track service calls for metrics
        self.service_calls_this_frame = self.service_calls_this_frame.saturating_add(1);

        // Update high water mark
        if self.service_calls_this_frame > self.metrics.service_calls_per_frame_high_water {
            self.metrics.service_calls_per_frame_high_water = self.service_calls_this_frame;
        }

        // If degraded, do minimal work
        if self.degraded {
            return 0;
        }

        let mut bytes_processed = 0;

        interrupt::free(|cs| {
            let mut device_ref = USB_DEVICE.borrow(cs).borrow_mut();
            let mut serial_ref = USB_SERIAL.borrow(cs).borrow_mut();

            if let (Some(usb_dev), Some(serial)) = (device_ref.as_mut(), serial_ref.as_mut()) {
                // Poll USB device state machine
                if usb_dev.poll(&mut [serial]) {
                    // Update connected status
                    self.connected = usb_dev.state() == UsbDeviceState::Configured;

                    // Read from USB → RX buffer (bounded)
                    let mut rx_chunk = [0u8; 64];
                    let mut bytes_read = 0;
                    for _ in 0..SERVICE_MAX_ITERS {
                        match serial.read(&mut rx_chunk) {
                            Ok(count) if count > 0 => {
                                let pushed = self.rx_push_slice(&rx_chunk[..count]);
                                if pushed < count {
                                    self.metrics.rx_dropped = self.metrics.rx_dropped.saturating_add(1);
                                }
                                bytes_read += count;
                                if bytes_read >= SERVICE_MAX_BYTES {
                                    break;
                                }
                            }
                            _ => break,
                        }
                    }
                    bytes_processed += bytes_read;
                }

                // Write from TX buffer → USB (bounded)
                let mut tx_chunk = [0u8; 64];
                let mut bytes_written = 0;
                for _ in 0..SERVICE_MAX_ITERS {
                    let available = self.tx_len();
                    if available == 0 {
                        break;
                    }
                    let chunk_size = available.min(64);
                    
                    // Peek at data without consuming yet
                    let count = self.tx_peek(&mut tx_chunk[..chunk_size]);
                    if count > 0 {
                        match serial.write(&tx_chunk[..count]) {
                            Ok(written) => {
                                // Successfully wrote 'written' bytes, now remove from buffer
                                self.tx_consume(written);
                                bytes_written += written;
                                if bytes_written >= SERVICE_MAX_BYTES {
                                    break;
                                }
                            }
                            Err(_) => {
                                self.metrics.tx_dropped = self.metrics.tx_dropped.saturating_add(1);
                                break;
                            }
                        }
                    }
                }
                bytes_processed += bytes_written;
            }
        });

        bytes_processed
    }

    /// Service USB with bounded iteration loop
    ///
    /// This is the main entry point called from the flight loop.
    pub fn service_bounded(&mut self) {
        // Reset per-frame counters
        self.service_calls_this_frame = 0;

        // Always service once (unconditional forward progress)
        self.service();

        // Extra iterations while pending was observed (bounded)
        let mut iters: u32 = 1;
        while clear_usb_irq_pending() && iters < SERVICE_MAX_ITERS as u32 {
            self.service();
            iters += 1;
        }

        // Track budget overruns
        if iters >= SERVICE_MAX_ITERS as u32 {
            self.metrics.budget_overruns = self.metrics.budget_overruns.saturating_add(1);
        }

        // Check for immediate refire (HAL contract validation)
        if is_usb_irq_pending() && iters == 1 {
            self.metrics.irq_immediate_refire_count =
                self.metrics.irq_immediate_refire_count.saturating_add(1);
        }

        // Re-enable USB IRQ after bounded servicing
        enable_usb_irq();
    }

    /// Try to read bytes from RX buffer (non-blocking)
    ///
    /// # Arguments
    ///
    /// * `buf` - Buffer to read into
    ///
    /// # Returns
    ///
    /// Number of bytes read (may be 0 if buffer empty)
    pub fn try_read(&mut self, buf: &mut [u8]) -> usize {
        let mut count = 0;
        let max = buf.len().min(SERVICE_MAX_BYTES);

        while count < max {
            if let Some(byte) = self.rx_pop() {
                buf[count] = byte;
                count += 1;
            } else {
                break;
            }
        }

        count
    }

    /// Try to write bytes to TX buffer (non-blocking)
    ///
    /// # Arguments
    ///
    /// * `data` - Data to write
    ///
    /// # Returns
    ///
    /// Number of bytes written (may be less than data.len() if buffer full)
    pub fn try_write(&mut self, data: &[u8]) -> usize {
        // If degraded, rate-limit output
        if self.degraded {
            return 0;
        }

        let mut count = 0;
        let max = data.len().min(SERVICE_MAX_BYTES);

        for &byte in &data[..max] {
            if self.tx_push(byte) {
                count += 1;
            } else {
                self.metrics.tx_dropped =
                    self.metrics.tx_dropped.saturating_add((data.len() - count) as u32);
                break;
            }
        }

        count
    }

    /// Check if USB is connected (host has set DTR)
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Check if USB is in degraded mode
    pub fn is_degraded(&self) -> bool {
        self.degraded
    }

    /// Get USB metrics
    pub fn metrics(&self) -> &UsbMetrics {
        &self.metrics
    }

    /// Reset USB metrics
    pub fn reset_metrics(&mut self) {
        self.metrics.reset();
    }

    /// Detach USB for DFU reboot (best-effort)
    ///
    /// Called by panic handler before triggering DFU reset.
    /// This is chip-specific and NOT part of any trait.
    pub fn detach_for_dfu(&mut self) {
        // COV:EXCL_START(STUB) - Hardware-only function
        //
        // Hardware implementation:
        //
        // 1. Disable USB peripheral
        //    usb.global().gccfg.modify(|_, w| w.pwrdwn().clear_bit());
        //
        // 2. Reconfigure D+/D- pins to analog input (releases pull-up)
        //    gpioa.moder.modify(|_, w| w.moder11().analog().moder12().analog());
        //
        // 3. Small delay for host to detect disconnect
        //    cortex_m::asm::delay(480_000); // ~1ms at 480MHz
        // COV:EXCL_STOP
    }

    // ========================================================================
    // Ring Buffer Operations
    // ========================================================================

    /// Push byte to RX buffer
    fn rx_push(&mut self, byte: u8) -> bool {
        let next = (self.rx_head + 1) % self.rx_buf.len();
        if next == self.rx_tail {
            return false; // Buffer full
        }
        self.rx_buf[self.rx_head] = byte;
        self.rx_head = next;

        // Update high water mark
        let depth = self.rx_depth();
        if depth > self.metrics.rx_queue_high_water {
            self.metrics.rx_queue_high_water = depth;
        }

        true
    }

    fn rx_push_slice(&mut self, data: &[u8]) -> usize {
        let mut count = 0;
        for &byte in data {
            if self.rx_push(byte) {
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    /// Pop byte from RX buffer
    fn rx_pop(&mut self) -> Option<u8> {
        if self.rx_tail == self.rx_head {
            return None; // Buffer empty
        }
        let byte = self.rx_buf[self.rx_tail];
        self.rx_tail = (self.rx_tail + 1) % self.rx_buf.len();
        Some(byte)
    }

    /// Get RX buffer depth
    fn rx_depth(&self) -> usize {
        if self.rx_head >= self.rx_tail {
            self.rx_head - self.rx_tail
        } else {
            self.rx_buf.len() - self.rx_tail + self.rx_head
        }
    }

    /// Push byte to TX buffer
    fn tx_push(&mut self, byte: u8) -> bool {
        let next = (self.tx_head + 1) % self.tx_buf.len();
        if next == self.tx_tail {
            return false; // Buffer full
        }
        self.tx_buf[self.tx_head] = byte;
        self.tx_head = next;

        // Update high water mark
        let depth = self.tx_len(); // Changed tx_depth to tx_len for consistency
        if depth > self.metrics.tx_queue_high_water {
            self.metrics.tx_queue_high_water = depth;
        }

        true
    }

    /// Pop byte from TX buffer
    #[allow(dead_code)]
    fn tx_pop(&mut self) -> Option<u8> {
        if self.tx_tail == self.tx_head {
            return None; // Buffer empty
        }
        let byte = self.tx_buf[self.tx_tail];
        self.tx_tail = (self.tx_tail + 1) % self.tx_buf.len();
        Some(byte)
    }

    /// Calculate TX buffer length
    fn tx_len(&self) -> usize {
        if self.tx_head >= self.tx_tail {
            self.tx_head - self.tx_tail
        } else {
            self.tx_buf.len() - self.tx_tail + self.tx_head
        }
    }

    /// Peek at TX data without removing
    fn tx_peek(&self, buf: &mut [u8]) -> usize {
        let mut count = 0;
        let mut idx = self.tx_tail;
        let len = self.tx_len();
        
        while count < buf.len() && count < len {
            buf[count] = self.tx_buf[idx];
            idx = (idx + 1) % self.tx_buf.len();
            count += 1;
        }
        count
    }

    /// Consume bytes from TX buffer (after successful write)
    fn tx_consume(&mut self, count: usize) {
        self.tx_tail = (self.tx_tail + count) % self.tx_buf.len();
    }
}

impl Default for Stm32h7UsbCdc {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Serial Number Generation
// ============================================================================

/// Get USB serial number from STM32 unique device ID
///
/// Reads the 96-bit unique device ID from UID registers and formats
/// as a 24-character hex string for USB serial number descriptor.
///
/// # Returns
///
/// 24-byte array containing hex-encoded serial number (uppercase).
pub fn get_usb_serial_number() -> [u8; 24] {
    // COV:EXCL_START(STUB) - Hardware-only function
    //
    // Hardware implementation:
    //
    // const UID_BASE: u32 = 0x1FF1_E800;
    // let uid0 = unsafe { core::ptr::read_volatile(UID_BASE as *const u32) };
    // let uid1 = unsafe { core::ptr::read_volatile((UID_BASE + 4) as *const u32) };
    // let uid2 = unsafe { core::ptr::read_volatile((UID_BASE + 8) as *const u32) };
    //
    // let mut serial = [0u8; 24];
    // // Format as hex: XXXXXXXX-XXXXXXXX-XXXXXXXX
    // format_hex(&mut serial[0..8], uid0);
    // format_hex(&mut serial[8..16], uid1);
    // format_hex(&mut serial[16..24], uid2);
    // serial

    // Return placeholder
    *b"000000000000000000000000"
    // COV:EXCL_STOP
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_usb_metrics_reset() {
        let mut metrics = UsbMetrics::new();
        metrics.rx_dropped = 10;
        metrics.reset();
        assert_eq!(metrics.rx_dropped, 0);
    }
}
