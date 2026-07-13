//! RP2350 test app for software DFU
//!
//! This app:
//! 1. Blinks LED 5 times (shows app is running)
//! 2. Sets want_bootloader flag
//! 3. Triggers software reset -> bootloader enters BOOTSEL mode

#![no_std]
#![no_main]

use aviate_boot_core::magic::BOOT_TO_BOOTLOADER;
use embedded_hal::digital::OutputPin;
use panic_halt as _;
use rp235x_hal as hal;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up GPIO for LED on GPIO25 (Pico 2 onboard LED)
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio25.into_push_pull_output();

    // Blink LED 5 times to show app is running
    for _ in 0..5 {
        led.set_high().ok();
        cortex_m::asm::delay(3_000_000); // ~250ms at 12MHz
        led.set_low().ok();
        cortex_m::asm::delay(3_000_000);
    }

    // Short pause
    cortex_m::asm::delay(6_000_000);

    // Set want_bootloader flag in watchdog scratch register
    // This tells the bootloader to enter update mode on next boot
    pac.WATCHDOG
        .scratch0()
        .write(|w| unsafe { w.bits(BOOT_TO_BOOTLOADER) });

    // LED on to indicate we're about to reset
    led.set_high().ok();
    cortex_m::asm::delay(1_500_000);

    // Trigger software reset - bootloader will see the flag and enter BOOTSEL
    hal::reboot::reboot(
        hal::reboot::RebootKind::Normal,
        hal::reboot::RebootArch::Normal,
    );
}
