//! RP2350 chip backend for Aviate bootloader

#![no_std]
// Production-grade lints (when software-dfu is disabled)
#![cfg_attr(not(feature = "software-dfu"), forbid(clippy::panic))]
#![cfg_attr(not(feature = "software-dfu"), forbid(clippy::unwrap_used))]
#![cfg_attr(not(feature = "software-dfu"), forbid(clippy::expect_used))]

use aviate_boot_core::{
    AppBackend, BootFlags, BootReason, CombinedBackend, CrashBackend, Delay, StatusLeds,
    UpdateBackend,
};
use rp235x_hal as hal;
use rp235x_hal::pac;

/// GPIO pin identifier
#[derive(Debug, Clone, Copy)]
pub struct GpioPin(pub u8);

/// Board metadata for RP2350 LED initialization
#[derive(Debug, Clone, Copy)]
pub struct Rp2350LedMetadata {
    pub red: Option<GpioPin>,
    pub green: Option<GpioPin>,
    pub blue: Option<GpioPin>,
}

impl Default for Rp2350LedMetadata {
    fn default() -> Self {
        Self {
            red: None,
            green: None,
            blue: None,
        }
    }
}

/// Crash/boot flag backend using watchdog scratch registers
pub struct Rp2350CrashBackend {
    watchdog: pac::WATCHDOG,
}

impl Rp2350CrashBackend {
    pub fn new(watchdog: pac::WATCHDOG) -> Self {
        Self { watchdog }
    }
}

impl CrashBackend for Rp2350CrashBackend {
    fn load_flags(&self) -> BootFlags {
        // Use watchdog scratch registers to store boot flags
        // SCRATCH0: want_bootloader magic
        // SCRATCH1: crash_detected magic
        // SCRATCH2: firmware_ok magic
        let scratch0 = self.watchdog.scratch0().read().bits();
        let scratch1 = self.watchdog.scratch1().read().bits();
        let scratch2 = self.watchdog.scratch2().read().bits();

        BootFlags {
            want_bootloader: scratch0 == aviate_boot_core::magic::BOOT_TO_BOOTLOADER,
            crash_detected: scratch1 == aviate_boot_core::magic::CRASH_DETECTED,
            firmware_ok: scratch2 == aviate_boot_core::magic::FIRMWARE_OK,
        }
    }

    fn store_flags(&mut self, flags: BootFlags) {
        let want_bl = if flags.want_bootloader {
            aviate_boot_core::magic::BOOT_TO_BOOTLOADER
        } else {
            0
        };
        let crash = if flags.crash_detected {
            aviate_boot_core::magic::CRASH_DETECTED
        } else {
            0
        };
        let fw_ok = if flags.firmware_ok {
            aviate_boot_core::magic::FIRMWARE_OK
        } else {
            0
        };

        self.watchdog
            .scratch0()
            .write(|w| unsafe { w.bits(want_bl) });
        self.watchdog.scratch1().write(|w| unsafe { w.bits(crash) });
        self.watchdog.scratch2().write(|w| unsafe { w.bits(fw_ok) });
    }

    fn boot_reason(&self) -> BootReason {
        // Check chip reset reason from CHIP_RESET register
        let chip_reset = unsafe { &*pac::POWMAN::ptr() };
        let reason = chip_reset.chip_reset().read().bits();

        // RP2350 reset reasons in CHIP_RESET register
        if reason & (1 << 20) != 0 {
            // HAD_POR - power on reset
            BootReason::PowerOn
        } else if reason & (1 << 24) != 0 {
            // HAD_WATCHDOG_RESET_RSM
            BootReason::Watchdog
        } else if reason & (1 << 25) != 0 {
            // HAD_WATCHDOG_RESET_SWCORE
            BootReason::Software
        } else if reason & (1 << 16) != 0 {
            // HAD_RUN_LOW - external reset
            BootReason::ExternalReset
        } else {
            BootReason::Unknown
        }
    }
}

/// Status LED backend for RP2350
pub struct Rp2350LedBackend {
    sio: pac::SIO,
    _metadata: Rp2350LedMetadata,
}

impl Rp2350LedBackend {
    pub fn new(
        io_bank: pac::IO_BANK0,
        pads: pac::PADS_BANK0,
        sio: pac::SIO,
        metadata: Rp2350LedMetadata,
    ) -> Self {
        // Configure LED pins as outputs if specified
        // Enable GPIO function (function 5 = SIO) for each LED pin
        for pin in [metadata.red, metadata.green, metadata.blue]
            .iter()
            .flatten()
        {
            let pin_num = pin.0 as usize;

            // Set pad to output enable (using indexed gpio accessor)
            pads.gpio(pin_num)
                .modify(|_, w| w.ie().set_bit().od().clear_bit());

            // Set function to SIO (function 5)
            io_bank
                .gpio(pin_num)
                .gpio_ctrl()
                .write(|w| w.funcsel().sio());

            // Enable output
            sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << pin_num) });
        }

        Self {
            sio,
            _metadata: metadata,
        }
    }

    fn set_pin(&mut self, pin: Option<GpioPin>, on: bool) {
        if let Some(p) = pin {
            if on {
                self.sio
                    .gpio_out_set()
                    .write(|w| unsafe { w.bits(1 << p.0) });
            } else {
                self.sio
                    .gpio_out_clr()
                    .write(|w| unsafe { w.bits(1 << p.0) });
            }
        }
    }
}

impl StatusLeds for Rp2350LedBackend {
    fn set_red(&mut self, on: bool) {
        self.set_pin(Some(GpioPin(25)), on); // Default to GPIO25 (onboard LED)
    }

    fn set_green(&mut self, on: bool) {
        // No green LED on default Pico 2
        let _ = on;
    }

    fn set_blue(&mut self, on: bool) {
        // No blue LED on default Pico 2
        let _ = on;
    }
}

/// Delay backend using busy-wait
pub struct Rp2350DelayBackend;

impl Rp2350DelayBackend {
    pub fn new() -> Self {
        Self
    }
}

impl Default for Rp2350DelayBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl Delay for Rp2350DelayBackend {
    fn delay_ms(&mut self, ms: u32) {
        // RP2350 default clock is ~150MHz, but after reset it runs at ~12MHz (XOSC)
        // Use a conservative estimate for busy-wait
        // 12MHz = 12000 cycles per ms
        for _ in 0..ms {
            cortex_m::asm::delay(12_000);
        }
    }
}

/// Update backend - enters BOOTSEL mode
pub struct Rp2350UpdateBackend;

impl Rp2350UpdateBackend {
    pub fn new() -> Self {
        Self
    }
}

impl Default for Rp2350UpdateBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl UpdateBackend for Rp2350UpdateBackend {
    fn enter_update_mode(&mut self) -> ! {
        // Reboot into BOOTSEL mode (USB mass storage)
        hal::reboot::reboot(
            hal::reboot::RebootKind::BootSel {
                msd_disabled: false,
                picoboot_disabled: false,
            },
            hal::reboot::RebootArch::Normal,
        );
    }
}

/// Application backend for RP2350
pub struct Rp2350AppBackend;

impl Rp2350AppBackend {
    pub fn new() -> Self {
        Self
    }
}

impl Default for Rp2350AppBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl AppBackend for Rp2350AppBackend {
    fn validate_app(&self, app_start: u32) -> bool {
        // Read stack pointer and reset vector from app's vector table
        let sp = unsafe { core::ptr::read_volatile(app_start as *const u32) };
        let reset = unsafe { core::ptr::read_volatile((app_start + 4) as *const u32) };

        // RP2350 has 520KB SRAM at 0x2000_0000 (ends at 0x2008_2000)
        // Stack pointer typically points to END of RAM and grows downward
        // Use inclusive range to allow SP at exact end of RAM
        let sp_valid = (0x2000_0000..=0x2008_2000).contains(&sp);

        // Reset vector should point to flash (0x1000_0000-0x1100_0000 for 16MB max)
        // or be within a reasonable range after app_start
        let reset_valid = reset >= app_start && reset < (app_start + 0x0100_0000);

        sp_valid && reset_valid
    }

    unsafe fn jump_to_app(&self, app_start: u32) -> ! {
        // Disable interrupts
        cortex_m::interrupt::disable();

        // Read vector table
        let sp = core::ptr::read_volatile(app_start as *const u32);
        let reset = core::ptr::read_volatile((app_start + 4) as *const u32);

        // Set VTOR to app's vector table
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.vtor.write(app_start);

        // Memory barriers
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // Set stack pointer and jump
        cortex_m::asm::bootstrap(sp as *const u32, reset as *const u32);
    }
}

/// Type alias for RP2350 backend
pub type Rp2350Backend = CombinedBackend<
    Rp2350CrashBackend,
    Rp2350LedBackend,
    Rp2350DelayBackend,
    Rp2350UpdateBackend,
    Rp2350AppBackend,
>;

/// App start address for RP2350
/// Flash starts at 0x1000_0000, bootloader uses first 64KB
pub const APP_START: u32 = 0x1001_0000;

/// Chip-specific main function
pub fn chip_main(led_metadata: Rp2350LedMetadata) -> ! {
    // Take peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Create backends
    let crash = Rp2350CrashBackend::new(dp.WATCHDOG);
    let leds = Rp2350LedBackend::new(dp.IO_BANK0, dp.PADS_BANK0, dp.SIO, led_metadata);
    let delay = Rp2350DelayBackend::new();
    let update = Rp2350UpdateBackend::new();
    let app = Rp2350AppBackend::new();

    // Combine backends
    let backend = Rp2350Backend::new(crash, leds, delay, update, app);

    // Run boot sequence with chip-specific app start address
    aviate_boot_core::boot_sequence(backend, APP_START)
}
