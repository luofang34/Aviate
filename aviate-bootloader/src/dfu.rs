//! USB DFU implementation for STM32H743
//!
//! Uses usbd-dfu crate for DFU protocol handling

use stm32h7xx_hal::{pac, prelude::*, usb_hs};
use usb_device::prelude::*;
use usbd_dfu::*;

// Flash constants for STM32H743
const FLASH_BASE: u32 = 0x0800_0000;
const APP_START: u32 = 0x0802_0000;
const FLASH_END: u32 = 0x0820_0000; // 2MB total
const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

// STM32H743 Flash register addresses (direct access)
const FLASH_KEYR1: *mut u32 = 0x5200_2004 as *mut u32;
const FLASH_CR1: *mut u32 = 0x5200_200C as *mut u32;
const FLASH_SR1: *mut u32 = 0x5200_2010 as *mut u32;
const FLASH_CCR1: *mut u32 = 0x5200_2014 as *mut u32;

// CR1 bits
const CR1_LOCK: u32 = 1 << 0;
const CR1_PG: u32 = 1 << 1;
const CR1_SER: u32 = 1 << 2;
const CR1_START: u32 = 1 << 7;

// SR1 bits
const SR1_BSY: u32 = 1 << 0;

// LED control (from main.rs)
const GPIOE_BSRR: *mut u32 = (0x5802_1000 + 0x18) as *mut u32;
const LED_GREEN: u32 = 2;
const LED_RED: u32 = 3;
const LED_BLUE: u32 = 4;

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

fn set_activity_led(on: bool) {
    set_led(LED_BLUE, on);
}

fn set_bootloader_led(on: bool) {
    set_led(LED_GREEN, on);
}

// Transfer buffer size (must match TRANSFER_SIZE)
const BUFFER_SIZE: usize = 128;

/// Flash memory interface for DFU
pub struct FlashMemory {
    buffer: [u8; BUFFER_SIZE],
    buffer_len: usize,
    /// Track if an erase operation is in progress
    erase_pending: bool,
}

impl FlashMemory {
    pub fn new() -> Self {
        Self {
            buffer: [0xFF; BUFFER_SIZE],
            buffer_len: 0,
            erase_pending: false,
        }
    }

    /// Unlock flash for writing
    fn unlock(&mut self) {
        unsafe {
            // Check if already unlocked
            if (core::ptr::read_volatile(FLASH_CR1) & CR1_LOCK) != 0 {
                core::ptr::write_volatile(FLASH_KEYR1, FLASH_KEY1);
                core::ptr::write_volatile(FLASH_KEYR1, FLASH_KEY2);
            }
        }
    }

    /// Lock flash
    fn lock(&mut self) {
        unsafe {
            let cr1 = core::ptr::read_volatile(FLASH_CR1);
            core::ptr::write_volatile(FLASH_CR1, cr1 | CR1_LOCK);
        }
    }

    /// Check if flash is busy
    fn is_busy(&self) -> bool {
        unsafe { (core::ptr::read_volatile(FLASH_SR1) & SR1_BSY) != 0 }
    }

    /// Wait for flash operation to complete
    fn wait_ready(&self) {
        while self.is_busy() {}
    }

    /// Complete any pending erase operation
    fn complete_pending_erase(&mut self) {
        if self.erase_pending {
            self.wait_ready();
            unsafe {
                // Clear SER bit
                let cr1 = core::ptr::read_volatile(FLASH_CR1);
                core::ptr::write_volatile(FLASH_CR1, cr1 & !CR1_SER);
            }
            self.lock();
            self.erase_pending = false;
            set_activity_led(false);
        }
    }

    /// Start erasing a sector (non-blocking, returns immediately after starting)
    fn start_erase_sector(&mut self, sector: u8) {
        // Complete any previous operation
        self.complete_pending_erase();

        set_activity_led(true);
        self.unlock();

        // Wait for any previous operation (should be instant since we just completed)
        self.wait_ready();

        unsafe {
            // Clear any previous errors
            core::ptr::write_volatile(FLASH_CCR1, 0x0FEF_0000);

            // Set sector and start erase (SNB is bits 10:8)
            let cr1 = CR1_SER | ((sector as u32 & 0x7) << 8) | CR1_START;
            core::ptr::write_volatile(FLASH_CR1, cr1);
        }

        // Mark erase as pending - we'll wait for completion later
        self.erase_pending = true;
    }

    /// Program a 256-bit (32-byte) flash word
    fn program_word(&mut self, address: u32, data: &[u8]) {
        self.wait_ready();

        // Build complete 32-byte buffer with padding
        let mut buffer = [0xFFu8; 32];
        let copy_len = core::cmp::min(data.len(), 32);
        buffer[..copy_len].copy_from_slice(&data[..copy_len]);

        unsafe {
            // Clear any previous error flags
            core::ptr::write_volatile(FLASH_CCR1, 0x0FEF_0000);

            // Enable programming
            let cr1 = core::ptr::read_volatile(FLASH_CR1);
            core::ptr::write_volatile(FLASH_CR1, cr1 | CR1_PG);

            // Write 32 bytes as 8 consecutive 32-bit words
            // CRITICAL: Must write in ascending address order for H7
            let dest = address as *mut u32;
            for i in 0..8 {
                let offset = i * 4;
                let word = u32::from_le_bytes([
                    buffer[offset],
                    buffer[offset + 1],
                    buffer[offset + 2],
                    buffer[offset + 3],
                ]);
                core::ptr::write_volatile(dest.add(i), word);
            }

            // Memory barriers
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }

        self.wait_ready();

        unsafe {
            // Check and clear any programming errors
            let sr1 = core::ptr::read_volatile(FLASH_SR1);
            if (sr1 & 0x0FEF_0000) != 0 {
                core::ptr::write_volatile(FLASH_CCR1, 0x0FEF_0000);
            }

            // Disable programming
            let cr1 = core::ptr::read_volatile(FLASH_CR1);
            core::ptr::write_volatile(FLASH_CR1, cr1 & !CR1_PG);
        }
    }
}

impl DFUMemIO for FlashMemory {
    const INITIAL_ADDRESS_POINTER: u32 = APP_START;
    const PROGRAM_TIME_MS: u32 = 10;
    // STM32H743 128KB sector erase can take up to 2s (typ 1.1s per RM0433)
    const ERASE_TIME_MS: u32 = 2500;
    // Full erase: 8 sectors * 2.5s = 20s
    const FULL_ERASE_TIME_MS: u32 = 20000;
    // DfuSe memory info: 'e' = erasable+writeable (not read-back)
    // Format: @<name>/<start>/<count>*<size><mult><flags>
    const MEM_INFO_STRING: &'static str = "@Flash/0x08020000/15*128Ke";
    const HAS_DOWNLOAD: bool = true;
    const HAS_UPLOAD: bool = true;
    const MANIFESTATION_TOLERANT: bool = false;
    const DETACH_TIMEOUT: u16 = 5000;
    // Note: Must be <= control endpoint buffer size (usually 128 for usb-device)
    // synopsys-usb-otg HS mode may support larger, but 128 is safer
    const TRANSFER_SIZE: u16 = 128;

    fn read(&mut self, address: u32, length: usize) -> Result<&[u8], DFUMemError> {
        // Complete any pending erase before reading
        self.complete_pending_erase();

        // Only allow reading app region
        if address < APP_START || address + length as u32 > FLASH_END {
            return Err(DFUMemError::Address);
        }

        let slice = unsafe {
            core::slice::from_raw_parts(address as *const u8, length)
        };
        Ok(slice)
    }

    fn erase(&mut self, address: u32) -> Result<(), DFUMemError> {
        // Flash red briefly to show erase called
        set_led(LED_RED, true);

        // Only allow erasing app region (sectors 1-15 for bank 1)
        if address < APP_START || address >= FLASH_END {
            set_led(LED_RED, false);
            return Err(DFUMemError::Address);
        }

        // Calculate sector number (128KB sectors)
        // Sector 0 = bootloader, sectors 1-7 = bank 1 app area
        let sector = ((address - FLASH_BASE) / (128 * 1024)) as u8;

        // Don't erase bootloader (sector 0)
        if sector == 0 {
            set_led(LED_RED, false);
            return Err(DFUMemError::Address);
        }

        // Start erase (non-blocking) - returns immediately
        self.start_erase_sector(sector);
        set_led(LED_RED, false);

        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), DFUMemError> {
        // Erase all app sectors (1-7 for bank 1, skip bootloader sector 0)
        for sector in 1..8 {
            self.start_erase_sector(sector);
            // Wait for this sector before starting next
            self.complete_pending_erase();
        }
        Ok(())
    }

    fn store_write_buffer(&mut self, src: &[u8]) -> Result<(), ()> {
        // Store incoming data in buffer
        if src.len() > BUFFER_SIZE {
            return Err(());
        }
        self.buffer[..src.len()].copy_from_slice(src);
        self.buffer_len = src.len();
        Ok(())
    }

    fn program(&mut self, address: u32, length: usize) -> Result<(), DFUMemError> {
        // Complete any pending erase before programming
        self.complete_pending_erase();

        // Only allow programming app region
        if address < APP_START || address + length as u32 > FLASH_END {
            return Err(DFUMemError::Address);
        }

        // Address must be 32-byte aligned for H7
        if address % 32 != 0 {
            return Err(DFUMemError::Address);
        }

        set_activity_led(true);
        self.unlock();

        // Program in 32-byte chunks from buffer
        let mut offset = 0;
        let buffer_len = self.buffer_len;
        while offset < length && offset < buffer_len {
            let end = core::cmp::min(offset + 32, buffer_len);
            // Copy chunk to local buffer to avoid borrow conflict
            let mut chunk = [0xFFu8; 32];
            let chunk_len = end - offset;
            chunk[..chunk_len].copy_from_slice(&self.buffer[offset..end]);
            self.program_word(address + offset as u32, &chunk[..chunk_len]);
            offset += 32;
        }

        self.lock();
        set_activity_led(false);

        // Clear buffer
        self.buffer_len = 0;

        Ok(())
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        // Reset to run new firmware
        cortex_m::peripheral::SCB::sys_reset();
    }
}

// PA8 VBUS sensing register addresses
const GPIOA_BASE: u32 = 0x5802_0000;
const GPIOA_MODER: *mut u32 = GPIOA_BASE as *mut u32;
const GPIOA_PUPDR: *mut u32 = (GPIOA_BASE + 0x0C) as *mut u32;

/// Configure PA8 as input with pulldown for USB VBUS sensing
fn configure_vbus_sensing() {
    unsafe {
        // PA8: MODER = 00 (input)
        let moder = core::ptr::read_volatile(GPIOA_MODER);
        core::ptr::write_volatile(GPIOA_MODER, moder & !(0b11 << 16));

        // PA8: PUPDR = 10 (pulldown)
        let pupdr = core::ptr::read_volatile(GPIOA_PUPDR);
        core::ptr::write_volatile(GPIOA_PUPDR, (pupdr & !(0b11 << 16)) | (0b10 << 16));
    }
}

/// Run the DFU bootloader
pub fn run_dfu() -> ! {
    use core::mem::MaybeUninit;
    use stm32h7xx_hal::rcc::rec::UsbClkSel;

    // Turn off all LEDs at start
    set_led(LED_GREEN, false);
    set_led(LED_RED, false);
    set_led(LED_BLUE, false);

    let dp = unsafe { pac::Peripherals::steal() };

    // Configure clocks
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(80.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Use HSI48 for USB clock
    if ccdr.clocks.hsi48_ck().is_none() {
        // HSI48 not running - error state: rapid red blink
        loop {
            set_led(LED_RED, true);
            for _ in 0..500_000 { cortex_m::asm::nop(); }
            set_led(LED_RED, false);
            for _ in 0..500_000 { cortex_m::asm::nop(); }
        }
    }
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

    // Configure GPIOA clock (needed for USB pins and VBUS)
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    // Configure PA8 for VBUS sensing (required for USB enumeration)
    configure_vbus_sensing();

    let usb = usb_hs::USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        gpioa.pa11.into_alternate(),
        gpioa.pa12.into_alternate(),
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    // USB endpoint memory - must be zeroed before use
    static mut EP_MEMORY: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

    let usb_bus = unsafe {
        // Zero the buffer
        let buf = EP_MEMORY.assume_init_mut();
        for word in buf.iter_mut() {
            *word = 0;
        }
        usb_hs::UsbBus::new(usb, buf)
    };

    let flash = FlashMemory::new();
    let mut dfu = DFUClass::new(&usb_bus, flash);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x0483, 0xDF11))
        .strings(&[StringDescriptors::default()
            .manufacturer("Aviate")
            .product("Aviate Bootloader")
            .serial_number("AVT001")])
        .expect("string descriptor")
        .device_class(0x00)
        .build();

    // Solid green = DFU mode ready
    set_bootloader_led(true);

    // Main loop - poll USB as fast as possible
    // Activity LED is controlled by erase/program operations
    loop {
        usb_dev.poll(&mut [&mut dfu]);
    }
}
