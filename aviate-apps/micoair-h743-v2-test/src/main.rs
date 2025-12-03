//! MicoAir H743-V2 Test Application with USB CDC Serial
//!
//! Features:
//! - USB CDC serial interface for debug output
//! - Protected reboot-to-bootloader command
//! - LED status indication
//! - I2C bus scanning and sensor probing
//!
//! ## Commands
//!
//! | Command | Description |
//! |---------|-------------|
//! | `help`  | Show available commands |
//! | `info`  | Show board information |
//! | `dfu`   | Start protected reboot sequence |
//! | `i2c1`  | Scan I2C1 bus (external: mag) |
//! | `i2c2`  | Scan I2C2 bus (internal: baro) |
//! | `spi`   | Probe SPI sensors (BMI088, BMI270) |
//! | `baro`  | Read SPL06 barometer |
//!
//! ## Reboot Protocol
//!
//! 1. Send `dfu` → Device generates random 4-digit code, responds `CONFIRM:xxxx`
//! 2. Send `xxxx` within 5 seconds → Device reboots to bootloader
//! 3. Timeout or wrong code → Returns to normal operation

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*, usb_hs};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// Type aliases for I2C buses
type I2c1Type = stm32h7xx_hal::i2c::I2c<pac::I2C1>;
type I2c2Type = stm32h7xx_hal::i2c::I2c<pac::I2C2>;

#[cfg(feature = "software-bootloader")]
use aviate_board_micoair_h743_v2::bootloader;

// LED pins (active low) - PE2=Green, PE3=Red, PE4=Blue
const GPIOE_BASE: u32 = 0x5802_1000;
const GPIOE_MODER: *mut u32 = GPIOE_BASE as *mut u32;
const GPIOE_BSRR: *mut u32 = (GPIOE_BASE + 0x18) as *mut u32;

const LED_GREEN: u32 = 2;
const LED_RED: u32 = 3;
const LED_BLUE: u32 = 4;

// RCC registers
const RCC_BASE: u32 = 0x5802_4400;
const RCC_AHB4ENR: *mut u32 = (RCC_BASE + 0x0E0) as *mut u32;
const RCC_AHB4ENR_GPIOEEN: u32 = 1 << 4;

// State machine for reboot confirmation
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Normal,
    AwaitingConfirmation { code: u16, timeout_ticks: u32 },
}

/// Simple LCG random number generator
struct SimpleRng {
    state: u32,
}

impl SimpleRng {
    fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    fn next(&mut self) -> u32 {
        // LCG parameters from Numerical Recipes
        self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
        self.state
    }

    fn next_range(&mut self, min: u16, max: u16) -> u16 {
        let range = (max - min) as u32;
        (min as u32 + (self.next() % (range + 1))) as u16
    }
}

/// Initialize LEDs
fn init_leds() {
    unsafe {
        // Enable GPIOE clock
        let ahb4enr = core::ptr::read_volatile(RCC_AHB4ENR);
        core::ptr::write_volatile(RCC_AHB4ENR, ahb4enr | RCC_AHB4ENR_GPIOEEN);
        cortex_m::asm::dsb();

        // Configure PE2, PE3, PE4 as outputs (MODER = 01)
        let moder = core::ptr::read_volatile(GPIOE_MODER);
        let moder = moder
            & !(0b11 << (LED_GREEN * 2))
            & !(0b11 << (LED_RED * 2))
            & !(0b11 << (LED_BLUE * 2));
        let moder = moder
            | (0b01 << (LED_GREEN * 2))
            | (0b01 << (LED_RED * 2))
            | (0b01 << (LED_BLUE * 2));
        core::ptr::write_volatile(GPIOE_MODER, moder);

        // Turn off all LEDs (set high for active low)
        core::ptr::write_volatile(GPIOE_BSRR, (1 << LED_GREEN) | (1 << LED_RED) | (1 << LED_BLUE));
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

/// Command buffer
struct CommandBuffer {
    buf: [u8; 64],
    len: usize,
}

impl CommandBuffer {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            len: 0,
        }
    }

    fn push(&mut self, byte: u8) -> bool {
        if self.len < self.buf.len() {
            self.buf[self.len] = byte;
            self.len += 1;
            true
        } else {
            false
        }
    }

    fn clear(&mut self) {
        self.len = 0;
    }

    fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(&self.buf[..self.len]).ok()
    }
}

/// Format a 4-digit code
fn format_code(code: u16) -> [u8; 4] {
    let mut result = [b'0'; 4];
    let mut n = code;
    for i in (0..4).rev() {
        result[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    result
}

/// Parse a 4-digit code from string
fn parse_code(s: &str) -> Option<u16> {
    let s = s.trim();
    if s.len() != 4 {
        return None;
    }
    let mut result: u16 = 0;
    for c in s.bytes() {
        if !c.is_ascii_digit() {
            return None;
        }
        result = result * 10 + (c - b'0') as u16;
    }
    Some(result)
}

/// Format a byte as hex string
fn format_hex(byte: u8) -> [u8; 2] {
    const HEX: &[u8] = b"0123456789ABCDEF";
    [HEX[(byte >> 4) as usize], HEX[(byte & 0x0F) as usize]]
}

/// Probe I2C1 for QMC5883L magnetometer (0x0D)
fn probe_i2c1(i2c: &mut I2c1Type, serial: &mut SerialPort<usb_hs::UsbBus<usb_hs::USB2>>) {
    let _ = serial.write(b"\r\nProbing I2C1 for QMC5883L (0x0D)...\r\n");

    // Read chip ID register (0x0D) - should return 0xFF for QMC5883L
    let mut buf = [0u8; 1];
    match i2c.write_read(0x0D, &[0x0D], &mut buf) {
        Ok(()) => {
            let _ = serial.write(b"  QMC5883L found! ChipID=0x");
            let hex = format_hex(buf[0]);
            let _ = serial.write(&hex);
            let _ = serial.write(b"\r\n");
        }
        Err(_) => {
            let _ = serial.write(b"  QMC5883L not found on I2C1\r\n");
        }
    }
}

/// Also probe I2C2 for QMC5883L (PX4 uses -I which is internal I2C)
fn probe_mag_i2c2(i2c: &mut I2c2Type, serial: &mut SerialPort<usb_hs::UsbBus<usb_hs::USB2>>) {
    let _ = serial.write(b"\r\nProbing I2C2 for QMC5883L (0x0D)...\r\n");

    let mut buf = [0u8; 1];
    match i2c.write_read(0x0D, &[0x0D], &mut buf) {
        Ok(()) => {
            let _ = serial.write(b"  QMC5883L found on I2C2! ChipID=0x");
            let hex = format_hex(buf[0]);
            let _ = serial.write(&hex);
            let _ = serial.write(b"\r\n");
        }
        Err(_) => {
            let _ = serial.write(b"  QMC5883L not found on I2C2\r\n");
        }
    }
}

/// Probe I2C2 for SPL06 barometer (0x77 per PX4 config)
fn probe_i2c2(i2c: &mut I2c2Type, serial: &mut SerialPort<usb_hs::UsbBus<usb_hs::USB2>>) {
    let _ = serial.write(b"\r\nProbing I2C2 for SPL06...\r\n");

    // Try both addresses 0x76 and 0x77
    for addr in [0x77u8, 0x76u8] {
        let mut buf = [0u8; 1];
        // Read product ID register (0x0D) - should return 0x10 for SPL06
        match i2c.write_read(addr, &[0x0D], &mut buf) {
            Ok(()) => {
                let _ = serial.write(b"  SPL06 found at 0x");
                let hex = format_hex(addr);
                let _ = serial.write(&hex);
                let _ = serial.write(b" ProductID=0x");
                let hex = format_hex(buf[0]);
                let _ = serial.write(&hex);
                let _ = serial.write(b"\r\n");
                return;
            }
            Err(_) => {}
        }
    }
    let _ = serial.write(b"  SPL06 not found at 0x76 or 0x77\r\n");
}

#[entry]
fn main() -> ! {
    // Initialize LEDs first
    init_leds();

    let dp = unsafe { pac::Peripherals::steal() };
    let cp = unsafe { cortex_m::Peripherals::steal() };

    // Configure clocks
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(120.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // Setup SysTick for timing (1ms ticks at 120MHz)
    let mut syst = cp.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(120_000 - 1); // 1ms at 120MHz
    syst.clear_current();
    syst.enable_counter();

    // Initialize RNG with SysTick counter as seed
    let mut rng = SimpleRng::new(syst.cvr.read());

    // Check if HSI48 is running for USB
    if ccdr.clocks.hsi48_ck().is_none() {
        // HSI48 not running - error state: rapid red blink
        loop {
            set_led(LED_RED, true);
            for _ in 0..500_000 {
                cortex_m::asm::nop();
            }
            set_led(LED_RED, false);
            for _ in 0..500_000 {
                cortex_m::asm::nop();
            }
        }
    }

    // Configure USB clock source
    use stm32h7xx_hal::rcc::rec::UsbClkSel;
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

    // Configure USB
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    let usb = usb_hs::USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        gpioa.pa11.into_alternate(),
        gpioa.pa12.into_alternate(),
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    #[allow(static_mut_refs)]
    let usb_bus = usb_hs::UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x0483, 0x5740))
        .strings(&[StringDescriptors::default()
            .manufacturer("Aviate")
            .product("MicoAir H743-V2 Test")
            .serial_number("AVT002")])
        .expect("string descriptor")
        .device_class(USB_CLASS_CDC)
        .build();

    // Signal successful USB init with green LED flash
    set_led(LED_GREEN, true);
    for _ in 0..500_000 {
        cortex_m::asm::nop();
    }
    set_led(LED_GREEN, false);

    // I2C initialization - try it now, should be safe
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // I2C1: External magnetometer (QMC5883L) - PB8 SCL, PB9 SDA
    let i2c1_scl = gpiob.pb8.into_alternate_open_drain();
    let i2c1_sda = gpiob.pb9.into_alternate_open_drain();
    let mut i2c1 = dp.I2C1.i2c(
        (i2c1_scl, i2c1_sda),
        100.kHz(),
        ccdr.peripheral.I2C1,
        &ccdr.clocks,
    );

    // I2C2: Internal barometer (SPL06) - PB10 SCL, PB11 SDA
    let i2c2_scl = gpiob.pb10.into_alternate_open_drain();
    let i2c2_sda = gpiob.pb11.into_alternate_open_drain();
    let mut i2c2 = dp.I2C2.i2c(
        (i2c2_scl, i2c2_sda),
        100.kHz(),
        ccdr.peripheral.I2C2,
        &ccdr.clocks,
    );

    // Double-flash green to signal I2C init complete
    set_led(LED_GREEN, true);
    for _ in 0..200_000 {
        cortex_m::asm::nop();
    }
    set_led(LED_GREEN, false);
    for _ in 0..200_000 {
        cortex_m::asm::nop();
    }
    set_led(LED_GREEN, true);
    for _ in 0..200_000 {
        cortex_m::asm::nop();
    }
    set_led(LED_GREEN, false);

    let mut cmd_buf = CommandBuffer::new();
    let mut state = State::Normal;
    let mut tick_count: u32 = 0;
    let mut last_blink: u32 = 0;
    let mut led_state = false;

    // Confirmation timeout: 5 seconds (5000 ticks at 1ms)
    const CONFIRM_TIMEOUT_TICKS: u32 = 5000;

    loop {
        // Update tick count from SysTick
        if syst.has_wrapped() {
            tick_count = tick_count.wrapping_add(1);
        }

        // Poll USB
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                for &byte in &buf[..count] {
                    match byte {
                        b'\r' | b'\n' => {
                            if cmd_buf.len > 0 {
                                if let Some(cmd) = cmd_buf.as_str() {
                                    let cmd = cmd.trim();
                                    match state {
                                        State::Normal => {
                                            if cmd.eq_ignore_ascii_case("help") {
                                                let _ = serial.write(b"\r\nCommands:\r\n");
                                                let _ = serial.write(b"  help  - Show this help\r\n");
                                                let _ = serial.write(b"  info  - Board information\r\n");
                                                let _ = serial.write(b"  i2c1  - Scan I2C1 (expect 0x0D: mag)\r\n");
                                                let _ = serial.write(b"  i2c2  - Scan I2C2 (expect 0x76: baro)\r\n");
                                                let _ = serial.write(b"  dfu   - Reboot to bootloader\r\n");
                                            } else if cmd.eq_ignore_ascii_case("i2c1") {
                                                probe_i2c1(&mut i2c1, &mut serial);
                                            } else if cmd.eq_ignore_ascii_case("i2c2") {
                                                probe_i2c2(&mut i2c2, &mut serial);
                                            } else if cmd.eq_ignore_ascii_case("mag") {
                                                // Probe both buses for QMC5883L
                                                probe_i2c1(&mut i2c1, &mut serial);
                                                probe_mag_i2c2(&mut i2c2, &mut serial);
                                            } else if cmd.eq_ignore_ascii_case("all") {
                                                // Probe all sensors
                                                let _ = serial.write(b"\r\n=== All Sensors ===");
                                                probe_i2c1(&mut i2c1, &mut serial);
                                                probe_mag_i2c2(&mut i2c2, &mut serial);
                                                probe_i2c2(&mut i2c2, &mut serial);
                                            } else if cmd.eq_ignore_ascii_case("info") {
                                                let _ = serial.write(b"\r\nBoard: MicoAir H743-V2\r\n");
                                                let _ = serial.write(b"MCU: STM32H743VIT6\r\n");
                                                let _ = serial.write(b"App: Test v0.1\r\n");
                                                #[cfg(feature = "software-bootloader")]
                                                let _ = serial.write(b"DFU: software-bootloader enabled\r\n");
                                                #[cfg(not(feature = "software-bootloader"))]
                                                let _ = serial.write(b"DFU: disabled (use BOOT button)\r\n");
                                            } else if cmd.eq_ignore_ascii_case("dfu") {
                                                #[cfg(feature = "software-bootloader")]
                                                {
                                                    // Generate random 4-digit code (1000-9999)
                                                    let code = rng.next_range(1000, 9999);
                                                    let code_str = format_code(code);

                                                    let _ = serial.write(b"\r\nCONFIRM:");
                                                    let _ = serial.write(&code_str);
                                                    let _ = serial.write(b"\r\n");

                                                    state = State::AwaitingConfirmation {
                                                        code,
                                                        timeout_ticks: tick_count.wrapping_add(CONFIRM_TIMEOUT_TICKS),
                                                    };
                                                }
                                                #[cfg(not(feature = "software-bootloader"))]
                                                {
                                                    let _ = serial.write(b"\r\nDFU disabled. Use BOOT+RESET button.\r\n");
                                                }
                                            } else if !cmd.is_empty() {
                                                let _ = serial.write(b"\r\nUnknown command: ");
                                                let _ = serial.write(cmd.as_bytes());
                                                let _ = serial.write(b"\r\nType 'help' for commands.\r\n");
                                            }
                                        }
                                        State::AwaitingConfirmation { code, .. } => {
                                            if let Some(entered) = parse_code(cmd) {
                                                if entered == code {
                                                    let _ = serial.write(b"\r\nRebooting to bootloader...\r\n");
                                                    // Wait for USB to flush
                                                    for _ in 0..100_000 {
                                                        cortex_m::asm::nop();
                                                    }
                                                    #[cfg(feature = "software-bootloader")]
                                                    bootloader::reboot_to_bootloader();
                                                    #[cfg(not(feature = "software-bootloader"))]
                                                    {
                                                        let _ = serial.write(b"ERROR: DFU disabled\r\n");
                                                        state = State::Normal;
                                                    }
                                                } else {
                                                    let _ = serial.write(b"\r\nWrong code. Cancelled.\r\n");
                                                    state = State::Normal;
                                                }
                                            } else {
                                                let _ = serial.write(b"\r\nInvalid code format. Cancelled.\r\n");
                                                state = State::Normal;
                                            }
                                        }
                                    }
                                }
                                cmd_buf.clear();
                            }
                        }
                        0x7F | 0x08 => {
                            // Backspace
                            if cmd_buf.len > 0 {
                                cmd_buf.len -= 1;
                                let _ = serial.write(b"\x08 \x08");
                            }
                        }
                        _ => {
                            if cmd_buf.push(byte) {
                                let _ = serial.write(&[byte]);
                            }
                        }
                    }
                }
            }
        }

        // Check confirmation timeout
        if let State::AwaitingConfirmation { timeout_ticks, .. } = state {
            if tick_count.wrapping_sub(timeout_ticks) < 0x8000_0000 {
                // Timeout has passed
                let _ = serial.write(b"\r\nTimeout. Cancelled.\r\n");
                state = State::Normal;
            }
        }

        // LED blinking
        let blink_interval = match state {
            State::Normal => 500,                    // Blue slow blink (500ms)
            State::AwaitingConfirmation { .. } => 100, // Red fast blink (100ms)
        };

        if tick_count.wrapping_sub(last_blink) >= blink_interval {
            last_blink = tick_count;
            led_state = !led_state;

            match state {
                State::Normal => {
                    set_led(LED_RED, false);
                    set_led(LED_BLUE, led_state);
                }
                State::AwaitingConfirmation { .. } => {
                    set_led(LED_BLUE, false);
                    set_led(LED_RED, led_state);
                }
            }
        }
    }
}
