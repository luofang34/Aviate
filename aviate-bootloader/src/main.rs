//! Aviate Custom DFU Bootloader for STM32H743
//!
//! Memory Layout:
//! - 0x0800_0000 - 0x0801_FFFF: Bootloader (128KB)
//! - 0x0802_0000 - 0x081F_FFFF: Application (1920KB)
//!
//! Boot Flow:
//! 1. Check RTC_BK0R for magic value (0xB007_B007)
//! 2. If magic present: clear magic, enter USB DFU mode
//! 3. If no magic: validate app, jump to app

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32h7xx_hal::pac;

mod dfu;

// Magic value to trigger bootloader mode
const BOOT_MAGIC: u32 = 0xB007_B007;

// Application start address (after 128KB bootloader)
const APP_ADDRESS: u32 = 0x0802_0000;

// STM32H743 register addresses
const RCC_BASE: u32 = 0x5802_4400;
const RCC_APB4ENR: *mut u32 = (RCC_BASE + 0x0F4) as *mut u32;
const RCC_APB4ENR_PWREN: u32 = 1 << 4;
const RCC_AHB4ENR: *mut u32 = (RCC_BASE + 0x0E0) as *mut u32;
const RCC_AHB4ENR_GPIOEEN: u32 = 1 << 4;

const PWR_BASE: u32 = 0x5802_4800;
const PWR_CR1: *mut u32 = PWR_BASE as *mut u32;
const PWR_CR1_DBP: u32 = 1 << 8;

const RTC_BASE: u32 = 0x5800_4000;
const RTC_BK0R: *mut u32 = (RTC_BASE + 0x50) as *mut u32;

// LED pins (active low) - MicoAir H743-V2
const GPIOE_BASE: u32 = 0x5802_1000;
const GPIOE_MODER: *mut u32 = GPIOE_BASE as *mut u32;
const GPIOE_BSRR: *mut u32 = (GPIOE_BASE + 0x18) as *mut u32;

// PE2 = Green (bootloader indicator), PE3 = Red, PE4 = Blue (activity)
const LED_GREEN: u32 = 2;
const LED_RED: u32 = 3;
const LED_BLUE: u32 = 4;

/// Read the boot magic from RTC backup register
fn read_boot_magic() -> u32 {
    unsafe {
        // Enable PWR peripheral clock
        let apb4enr = core::ptr::read_volatile(RCC_APB4ENR);
        core::ptr::write_volatile(RCC_APB4ENR, apb4enr | RCC_APB4ENR_PWREN);
        cortex_m::asm::dsb();

        // Enable backup domain access
        let cr1 = core::ptr::read_volatile(PWR_CR1);
        core::ptr::write_volatile(PWR_CR1, cr1 | PWR_CR1_DBP);

        // Wait for DBP bit to be set
        while (core::ptr::read_volatile(PWR_CR1) & PWR_CR1_DBP) == 0 {}

        // Read magic value
        core::ptr::read_volatile(RTC_BK0R)
    }
}

/// Clear the boot magic
fn clear_boot_magic() {
    unsafe {
        core::ptr::write_volatile(RTC_BK0R, 0);
        cortex_m::asm::dsb();
    }
}

/// Initialize LEDs
fn init_leds() {
    unsafe {
        // Enable GPIOE clock
        let ahb4enr = core::ptr::read_volatile(RCC_AHB4ENR);
        core::ptr::write_volatile(RCC_AHB4ENR, ahb4enr | RCC_AHB4ENR_GPIOEEN);
        cortex_m::asm::dsb();

        // Configure PE2, PE3, PE4 as outputs (MODER bits)
        let moder = core::ptr::read_volatile(GPIOE_MODER);
        let moder = moder & !(0b11 << (LED_GREEN * 2)) & !(0b11 << (LED_RED * 2)) & !(0b11 << (LED_BLUE * 2));
        let moder = moder | (0b01 << (LED_GREEN * 2)) | (0b01 << (LED_RED * 2)) | (0b01 << (LED_BLUE * 2));
        core::ptr::write_volatile(GPIOE_MODER, moder);
    }
}

/// Set LED state (active low)
fn set_led(led: u32, on: bool) {
    unsafe {
        if on {
            // Reset bit (turn on, active low)
            core::ptr::write_volatile(GPIOE_BSRR, 1 << (led + 16));
        } else {
            // Set bit (turn off)
            core::ptr::write_volatile(GPIOE_BSRR, 1 << led);
        }
    }
}

/// Check if application is valid by examining the vector table
fn is_app_valid() -> bool {
    unsafe {
        let app_stack = core::ptr::read_volatile(APP_ADDRESS as *const u32);
        let app_reset = core::ptr::read_volatile((APP_ADDRESS + 4) as *const u32);

        // Stack pointer should point to valid RAM (D1 RAM: 0x2400_0000 - 0x2408_0000)
        let stack_valid = (0x2400_0000..=0x2408_0000).contains(&app_stack);

        // Reset vector should point to flash (app region)
        let reset_valid = (APP_ADDRESS..=0x081F_FFFF).contains(&app_reset);

        stack_valid && reset_valid
    }
}

/// Jump to application
#[inline(never)]
fn jump_to_app() -> ! {
    unsafe {
        let app_stack = core::ptr::read_volatile(APP_ADDRESS as *const u32);
        let app_reset = core::ptr::read_volatile((APP_ADDRESS + 4) as *const u32);

        // Disable interrupts
        cortex_m::interrupt::disable();

        // Set vector table offset
        let scb = &*pac::SCB::PTR;
        scb.vtor.write(APP_ADDRESS);

        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // Set MSP and jump to app reset handler
        // Use inline assembly to set stack and branch
        core::arch::asm!(
            "msr msp, {0}",
            "bx {1}",
            in(reg) app_stack,
            in(reg) app_reset,
            options(noreturn)
        );
    }
}

#[entry]
fn main() -> ! {
    // Initialize LEDs first for status indication
    init_leds();
    // Turn off all LEDs immediately (GPIOs default to ON for active-low LEDs)
    set_led(LED_GREEN, false);
    set_led(LED_RED, false);
    set_led(LED_BLUE, false);

    // Check for boot magic
    let magic = read_boot_magic();
    let enter_dfu = magic == BOOT_MAGIC;

    if enter_dfu {
        // Clear the magic so we don't loop forever
        clear_boot_magic();

        // Turn on GREEN LED to indicate DFU mode
        set_led(LED_GREEN, true);
        set_led(LED_BLUE, false);

        // Enter DFU mode
        dfu::run_dfu();
    }

    // No magic - try to boot app
    if is_app_valid() {
        // Brief blue flash to indicate jumping to app
        set_led(LED_BLUE, true);
        for _ in 0..100_000 {
            cortex_m::asm::nop();
        }
        set_led(LED_BLUE, false);

        jump_to_app();
    }

    // App invalid - fall through to DFU mode
    // Turn OFF green LED so we can see blue blinks clearly
    set_led(LED_GREEN, false);
    set_led(LED_BLUE, false);

    dfu::run_dfu();
}
