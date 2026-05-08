//! STM32H7 Family HAL Primitives for Aviate
//!
//! This crate provides STM32H7 family implementations of Aviate HAL traits:
//! - Security primitives (KeyStore, CryptoEngine)
//! - Transport wrappers (USB CDC, UART, CAN)
//!
//! ## Architecture Position
//!
//! This is a **Tier 2: Chip Runtime HAL** crate in the Aviate HAL layering:
//!
//! ```text
//! Tier 1: aviate-hal-io        → Platform-agnostic traits
//! Tier 2: aviate-hal-stm32h7   → STM32H7 family primitives (THIS CRATE)
//! Tier 3: aviate-boards/*      → Board-specific pin mappings
//! ```
//!
//! ## Shared Across STM32H7 Boards
//!
//! This crate is reused by all STM32H7-based boards:
//! - MicoAir H743-V2
//! - Nucleo H743ZI
//! - Custom H750/H7A3 designs
//!
//! Only pin assignments and routing vary per board, not the chip capabilities.
//!
//! ## Feature Flags
//!
//! - **`secure-keys`** (default OFF): Use OTP for production key storage
//!   - OFF: Flash const keys (development, insecure, easy to update)
//!   - ON: OTP reads (production, write-once, tamper-resistant)
//!
//! - **`hw-crypto`** (default OFF): Use hardware crypto acceleration
//!   - OFF: Software crypto using `sha2`/`hmac` crates
//!   - ON: HASH/CRYP peripheral acceleration (TODO)
//!
//! ## Security Primitives
//!
//! - `Stm32h7KeyStore`: Reads keys from OTP or flash const
//! - `Stm32h7CryptoEngine`: HMAC-SHA256 (software or hardware)
//!
//! ## Transport Wrappers
//!
//! - `Stm32h7UsbCdcTx/Rx`: USB CDC with non-blocking frame I/O (TODO)
//! - `Stm32h7UartTx/Rx`: UART transport wrappers (TODO)
//! - `Stm32h7CanTx/Rx`: CAN transport wrappers (TODO)
//!
//! ## Example Usage
//!
//! ```ignore
//! use aviate_hal_stm32h7::{Stm32h7KeyStore, Stm32h7CryptoEngine};
//! use aviate_hal_io::security::{KeyStore, CryptoEngine, CryptoAlgo};
//!
//! // Create chip-level security primitives
//! let keystore = Stm32h7KeyStore::new();
//! let mut crypto = Stm32h7CryptoEngine::new();
//!
//! // Use HAL traits
//! if let Some(key) = keystore.load_command_key() {
//!     let msg = b"ARM_MOTORS";
//!     let tag = [0u8; 32];
//!     crypto.verify(CryptoAlgo::HmacSha256, key, msg, &tag)?;
//! }
//! ```

#![no_std]
// Note: time.rs uses unsafe for DWT register access
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]

pub mod clock;
pub mod pwm;
pub mod security;
pub mod time;
pub mod transport;
pub mod usb_cdc;
pub mod usb_rt;
pub mod watchdog;

// Re-export for convenience
pub use clock::{init_clocks_400mhz, init_clocks_480mhz, ClockError, Clocks, UsbClkSource};
pub use pwm::{PwmConfig, PwmMotors};
pub use security::{Stm32h7CryptoEngine, Stm32h7KeyStore};
pub use time::{NoSleep, SleepTimer, Stm32h7Time};
pub use transport::{CanTransport, Stm32h7Transport, UartTransport, UsbCdcTransport};
pub use usb_cdc::{get_usb_serial_number, Stm32h7UsbCdc, UsbMetrics, USB_PID, USB_VID};
pub use usb_rt::{
    clear_usb_irq_pending, enable_usb_irq, is_usb_irq_pending, usb_irq_count, SERVICE_MAX_BYTES,
    SERVICE_MAX_ITERS, USB_IRQ_MASK_MAX_US,
};
pub use watchdog::{IwdgPrescaler, Stm32h7Watchdog};
