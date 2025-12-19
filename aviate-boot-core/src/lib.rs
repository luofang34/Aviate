//! Multi-MCU bootloader protocol
//!
//! This crate defines the boot protocol logic without any MCU-specific code.
//! All MCU-specific implementations are done via traits.

#![no_std]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

/// Boot magic constants (used internally by chip backends for encoding)
pub mod magic {
    pub const BOOT_TO_BOOTLOADER: u32 = 0xb0_07_b0_07;
    pub const CRASH_DETECTED: u32 = 0xde_ad_be_ef;
    pub const FIRMWARE_OK: u32 = 0xb0_09_3a_26;
}

/// Boot/reset reason (MCU-specific)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootReason {
    PowerOn,
    Watchdog,
    Software,
    ExternalReset,
    Unknown,
}

/// Logical boot flags (boolean view, not raw magic values)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootFlags {
    pub want_bootloader: bool,
    pub crash_detected: bool,
    pub firmware_ok: bool,
}

/// Crash/boot flag backend (implemented by each MCU)
pub trait CrashBackend {
    /// Load logical flags from MCU-specific storage
    /// (STM32: RTC backup regs, RP2040: SRAM region, RP2350: SRAM/RTC, ESP32: RTC slow mem)
    fn load_flags(&self) -> BootFlags;

    /// Store logical flags to MCU-specific storage
    fn store_flags(&mut self, flags: BootFlags);

    /// Get reset/boot reason from MCU-specific registers
    fn boot_reason(&self) -> BootReason;

    // ---- Default helpers (implemented in terms of load/store) ----

    fn set_crash_detected(&mut self) {
        let mut f = self.load_flags();
        f.crash_detected = true;
        self.store_flags(f);
    }

    fn clear_crash_detected(&mut self) {
        let mut f = self.load_flags();
        f.crash_detected = false;
        self.store_flags(f);
    }

    fn request_bootloader(&mut self) {
        let mut f = self.load_flags();
        f.want_bootloader = true;
        self.store_flags(f);
    }

    fn clear_bootloader_request(&mut self) {
        let mut f = self.load_flags();
        f.want_bootloader = false;
        self.store_flags(f);
    }

    fn set_firmware_ok(&mut self) {
        let mut f = self.load_flags();
        f.firmware_ok = true;
        self.store_flags(f);
    }

    fn clear_firmware_ok(&mut self) {
        let mut f = self.load_flags();
        f.firmware_ok = false;
        self.store_flags(f);
    }
}

/// Status LED control (board-specific pins, chip-specific HAL)
pub trait StatusLeds {
    fn set_red(&mut self, on: bool);
    fn set_green(&mut self, on: bool);
    fn set_blue(&mut self, on: bool);
}

/// Delay provider (MCU-specific timing)
pub trait Delay {
    fn delay_ms(&mut self, ms: u32);
}

/// Firmware update mode backend (implemented by each MCU)
pub trait UpdateBackend {
    /// Enter firmware update mode (never returns)
    /// (STM32: USB DFU, RP2040: ROM USB MSC, ESP32: ROM bootloader)
    fn enter_update_mode(&mut self) -> !;
}

/// Application validation and jump (implemented by each MCU)
pub trait AppBackend {
    /// Validate application at given address
    /// Returns true if valid (stack pointer and reset vector look reasonable)
    fn validate_app(&self, app_start: u32) -> bool;

    /// Jump to application at given address (never returns on success)
    ///
    /// # Safety
    /// Caller must ensure app_start points to valid application with:
    /// - Valid stack pointer at app_start + 0
    /// - Valid reset vector at app_start + 4
    unsafe fn jump_to_app(&self, app_start: u32) -> !;
}

/// Complete backend (combines all traits)
pub trait BootBackend: CrashBackend + StatusLeds + Delay + UpdateBackend + AppBackend {}

// Blanket implementation
impl<T> BootBackend for T where T: CrashBackend + StatusLeds + Delay + UpdateBackend + AppBackend {}

/// Generic combined backend (reusable across all chips)
pub struct CombinedBackend<C, L, Dly, Upd, App> {
    pub crash: C,
    pub leds: L,
    pub delay: Dly,
    pub update: Upd,
    pub app: App,
}

impl<C, L, Dly, Upd, App> CombinedBackend<C, L, Dly, Upd, App> {
    pub fn new(crash: C, leds: L, delay: Dly, update: Upd, app: App) -> Self {
        Self {
            crash,
            leds,
            delay,
            update,
            app,
        }
    }
}

// Trait delegations
impl<C: CrashBackend, L, Dly, Upd, App> CrashBackend for CombinedBackend<C, L, Dly, Upd, App> {
    fn load_flags(&self) -> BootFlags {
        self.crash.load_flags()
    }
    fn store_flags(&mut self, flags: BootFlags) {
        self.crash.store_flags(flags)
    }
    fn boot_reason(&self) -> BootReason {
        self.crash.boot_reason()
    }
    // Default helpers use load/store, so no more methods needed
}

impl<C, L: StatusLeds, Dly, Upd, App> StatusLeds for CombinedBackend<C, L, Dly, Upd, App> {
    fn set_red(&mut self, on: bool) {
        self.leds.set_red(on)
    }
    fn set_green(&mut self, on: bool) {
        self.leds.set_green(on)
    }
    fn set_blue(&mut self, on: bool) {
        self.leds.set_blue(on)
    }
}

impl<C, L, Dly: Delay, Upd, App> Delay for CombinedBackend<C, L, Dly, Upd, App> {
    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms)
    }
}

impl<C, L, Dly, Upd: UpdateBackend, App> UpdateBackend for CombinedBackend<C, L, Dly, Upd, App> {
    fn enter_update_mode(&mut self) -> ! {
        self.update.enter_update_mode()
    }
}

impl<C, L, Dly, Upd, App: AppBackend> AppBackend for CombinedBackend<C, L, Dly, Upd, App> {
    fn validate_app(&self, app_start: u32) -> bool {
        self.app.validate_app(app_start)
    }
    unsafe fn jump_to_app(&self, app_start: u32) -> ! {
        self.app.jump_to_app(app_start)
    }
}

/// Bootloader state machine (MCU-agnostic)
/// Priority: crash > explicit bootloader request > app validation > jump or DFU
///
/// # Parameters
/// - `backend`: Platform-specific boot backend implementation
/// - `app_start`: Address where the application is located (chip-specific)
pub fn boot_sequence<B: BootBackend>(mut backend: B, app_start: u32) -> ! {
    let _flags = backend.load_flags();
    let _reason = backend.boot_reason(); // For future use


    // 1. Crash detected → indicate + enter update mode (only with software-dfu feature)
    #[cfg(feature = "software-dfu")]
    if _flags.crash_detected {
        // 3 quick purple blinks (red+blue) to indicate crash recovery
        for _ in 0..3 {
            backend.set_red(true);
            backend.set_blue(true);
            backend.delay_ms(200);
            backend.set_red(false);
            backend.set_blue(false);
            backend.delay_ms(200);
        }
        backend.clear_crash_detected();
        backend.enter_update_mode(); // Never returns
    }

    // 2. Software-requested bootloader (only with software-dfu feature)
    #[cfg(feature = "software-dfu")]
    if _flags.want_bootloader {
        backend.clear_bootloader_request();
        backend.enter_update_mode(); // Never returns
    }

    // 3. Validate application
    if backend.validate_app(app_start) {
        // App is valid - jump to it
        // Safety: validate_app confirmed the app has valid stack pointer and reset vector
        unsafe {
            backend.jump_to_app(app_start);
        }
    }

    // 4. No valid app - enter update mode
    backend.enter_update_mode();
}
