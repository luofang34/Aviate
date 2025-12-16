//! UpdateBackend implementation for STM32H743
//!
//! Implements USB DFU (Device Firmware Update) mode

use aviate_boot_core::UpdateBackend;
use core::mem::MaybeUninit;
use stm32h7xx_hal::{pac, prelude::*, usb_hs};
use usb_device::prelude::*;
use usbd_dfu::*;

// Import memory layout constants from chip configuration
use crate::memory::{
    APP_START, DFU_MEM_INFO, FLASH_BASE, FLASH_END, FLASH_KEY1, FLASH_KEY2, SECTOR_SIZE,
};

// Transfer buffer size (must match TRANSFER_SIZE)
const BUFFER_SIZE: usize = 128;

/// Flash memory interface for DFU
struct FlashMemory {
    flash: pac::FLASH,
    buffer: [u8; BUFFER_SIZE],
    buffer_len: usize,
    erase_pending: bool,
}

impl FlashMemory {
    fn new(flash: pac::FLASH) -> Self {
        Self {
            flash,
            buffer: [0xFF; BUFFER_SIZE],
            buffer_len: 0,
            erase_pending: false,
        }
    }

    fn unlock(&mut self) {
        if self.flash.bank1().cr.read().lock().bit_is_set() {
            self.flash
                .bank1()
                .keyr
                .write(|w| unsafe { w.bits(FLASH_KEY1) });
            self.flash
                .bank1()
                .keyr
                .write(|w| unsafe { w.bits(FLASH_KEY2) });
        }
    }

    fn lock(&mut self) {
        self.flash.bank1().cr.modify(|_, w| w.lock().set_bit());
    }

    fn is_busy(&self) -> bool {
        self.flash.bank1().sr.read().bsy().bit_is_set()
    }

    fn wait_ready(&self) {
        while self.is_busy() {}
    }

    fn complete_pending_erase(&mut self) {
        if self.erase_pending {
            self.wait_ready();
            self.flash.bank1().cr.modify(|_, w| w.ser().clear_bit());
            self.lock();
            self.erase_pending = false;
        }
    }

    fn start_erase_sector(&mut self, sector: u8) {
        self.complete_pending_erase();
        self.unlock();
        self.wait_ready();

        // Clear error flags
        self.flash
            .bank1()
            .ccr
            .write(|w| unsafe { w.bits(0x0FEF_0000) });

        // Start sector erase
        self.flash
            .bank1()
            .cr
            .modify(|_, w| unsafe { w.ser().set_bit().snb().bits(sector & 0x7).start().set_bit() });

        self.erase_pending = true;
    }

    fn program_word(&mut self, address: u32, data: &[u8]) {
        self.wait_ready();

        let mut buffer = [0xFFu8; 32];
        let copy_len = core::cmp::min(data.len(), 32);
        buffer[..copy_len].copy_from_slice(&data[..copy_len]);

        // Clear error flags
        self.flash
            .bank1()
            .ccr
            .write(|w| unsafe { w.bits(0x0FEF_0000) });

        // Enable programming
        self.flash.bank1().cr.modify(|_, w| w.pg().set_bit());

        // Write 256 bits (8 words)
        unsafe {
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

            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }

        self.wait_ready();

        // Clear error flags if any
        if self.flash.bank1().sr.read().bits() & 0x0FEF_0000 != 0 {
            self.flash
                .bank1()
                .ccr
                .write(|w| unsafe { w.bits(0x0FEF_0000) });
        }

        // Disable programming
        self.flash.bank1().cr.modify(|_, w| w.pg().clear_bit());
    }
}

impl DFUMemIO for FlashMemory {
    const INITIAL_ADDRESS_POINTER: u32 = APP_START;
    const PROGRAM_TIME_MS: u32 = 10;
    const ERASE_TIME_MS: u32 = 2500;
    const FULL_ERASE_TIME_MS: u32 = 20000;
    const MEM_INFO_STRING: &'static str = DFU_MEM_INFO;
    const HAS_DOWNLOAD: bool = true;
    const HAS_UPLOAD: bool = true;
    const MANIFESTATION_TOLERANT: bool = false;
    const DETACH_TIMEOUT: u16 = 5000;
    const TRANSFER_SIZE: u16 = 128;

    fn read(&mut self, address: u32, length: usize) -> Result<&[u8], DFUMemError> {
        self.complete_pending_erase();

        if address < APP_START || address + length as u32 > FLASH_END {
            return Err(DFUMemError::Address);
        }

        let slice = unsafe { core::slice::from_raw_parts(address as *const u8, length) };
        Ok(slice)
    }

    fn erase(&mut self, address: u32) -> Result<(), DFUMemError> {
        if address < APP_START || address >= FLASH_END {
            return Err(DFUMemError::Address);
        }

        let sector = ((address - FLASH_BASE) / SECTOR_SIZE) as u8;

        if sector == 0 {
            return Err(DFUMemError::Address);
        }

        self.start_erase_sector(sector);

        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), DFUMemError> {
        for sector in 1..8 {
            self.start_erase_sector(sector);
            self.complete_pending_erase();
        }
        Ok(())
    }

    fn store_write_buffer(&mut self, src: &[u8]) -> Result<(), ()> {
        if src.len() > BUFFER_SIZE {
            return Err(());
        }
        self.buffer[..src.len()].copy_from_slice(src);
        self.buffer_len = src.len();
        Ok(())
    }

    fn program(&mut self, address: u32, length: usize) -> Result<(), DFUMemError> {
        self.complete_pending_erase();

        if address < APP_START || address + length as u32 > FLASH_END {
            return Err(DFUMemError::Address);
        }

        if address % 32 != 0 {
            return Err(DFUMemError::Address);
        }

        self.unlock();

        let mut offset = 0;
        let buffer_len = self.buffer_len;
        while offset < length && offset < buffer_len {
            let end = core::cmp::min(offset + 32, buffer_len);
            let mut chunk = [0xFFu8; 32];
            let chunk_len = end - offset;
            chunk[..chunk_len].copy_from_slice(&self.buffer[offset..end]);
            self.program_word(address + offset as u32, &chunk[..chunk_len]);
            offset += 32;
        }

        self.lock();
        self.buffer_len = 0;

        Ok(())
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        cortex_m::peripheral::SCB::sys_reset();
    }
}

/// PA8 VBUS sensing configuration using PAC
fn configure_vbus_sensing(gpioa: &pac::GPIOA) {
    // Set PA8 to input mode (clear bits 16-17 in MODER)
    gpioa
        .moder
        .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 16)) });

    // Set PA8 pull-down (10 in bits 16-17 of PUPDR)
    gpioa
        .pupdr
        .modify(|r, w| unsafe { w.bits((r.bits() & !(0b11 << 16)) | (0b10 << 16)) });
}

pub struct Stm32h743UpdateBackend {
    _usb_otg: pac::OTG2_HS_GLOBAL,
}

impl Stm32h743UpdateBackend {
    pub fn new(_usb_otg: pac::OTG2_HS_GLOBAL) -> Self {
        Self { _usb_otg }
    }
}

impl UpdateBackend for Stm32h743UpdateBackend {
    fn enter_update_mode(&mut self) -> ! {
        use stm32h7xx_hal::rcc::rec::UsbClkSel;

        // Safety: steal() is used because peripherals were taken in chip_main()
        // This is the only place that accesses these peripherals after chip_main
        let dp = unsafe { pac::Peripherals::steal() };

        // Enable HSI48 oscillator before HAL takes ownership of RCC
        // Required for USB 48MHz clock source (RM0433 Section 8.5.3)
        dp.RCC.cr.modify(|_, w| w.hsi48on().set_bit());
        while !dp.RCC.cr.read().hsi48rdy().bit_is_set() {
            cortex_m::asm::nop();
        }

        // Configure clocks with HSI48 for USB
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = dp.RCC.constrain();
        let mut ccdr = rcc.sys_ck(80.MHz()).freeze(pwrcfg, &dp.SYSCFG);
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        // Enable USB voltage regulator (USB33DEN in PWR_CR3)
        // Required for USB transceivers to work on STM32H7
        // Safety: PWR peripheral was consumed by constrain(), use PAC directly
        let pwr = unsafe { &*pac::PWR::ptr() };
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());

        // Wait for USB supply ready (USB33RDY) - may not be set on all packages
        // For LQFP100 package, VDD33USB is supplied via VDD, so USB33RDY won't be set
        // Wait briefly for voltage stabilization
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }

        // Enable GPIOA clock FIRST, then configure GPIO
        // (VBUS sensing must happen before gpioa.split() consumes the PAC reference)
        let rcc_ahb4 = unsafe { &*pac::RCC::ptr() };
        rcc_ahb4.ahb4enr.modify(|_, w| w.gpioaen().set_bit());
        cortex_m::asm::dsb(); // Ensure clock is enabled before GPIO access

        let gpioa_pac = dp.GPIOA; // Keep PAC reference for VBUS
        configure_vbus_sensing(&gpioa_pac);

        // Split GPIOA for USB pins (consumes gpioa_pac, clock already enabled)
        let gpioa = gpioa_pac.split(ccdr.peripheral.GPIOA);

        // USB2 (OTG2_HS in FS mode) uses PA11/PA12 internal PHY
        let usb = usb_hs::USB2::new(
            dp.OTG2_HS_GLOBAL,
            dp.OTG2_HS_DEVICE,
            dp.OTG2_HS_PWRCLK,
            gpioa.pa11.into_alternate(),
            gpioa.pa12.into_alternate(),
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        static mut EP_MEMORY: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

        let usb_bus = unsafe {
            let buf = &mut *core::ptr::addr_of_mut!(EP_MEMORY);
            let buf = buf.assume_init_mut();
            for word in buf.iter_mut() {
                *word = 0;
            }
            usb_hs::UsbBus::new(usb, buf)
        };

        // Pass FLASH peripheral to FlashMemory
        let flash = FlashMemory::new(dp.FLASH);
        let mut dfu = DFUClass::new(&usb_bus, flash);

        // Build USB device - use match to handle string descriptor errors without panic
        let usb_dev_builder = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x0483, 0xDF11))
            .strings(&[StringDescriptors::default()
                .manufacturer("Aviate")
                .product("Aviate Bootloader")
                .serial_number("AVT001")]);

        let mut usb_dev = match usb_dev_builder {
            Ok(builder) => builder.device_class(0x00).build(),
            Err(_) => {
                // Fallback: build without string descriptors
                UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x0483, 0xDF11))
                    .device_class(0x00)
                    .build()
            }
        };

        // Turn on GREEN LED to indicate DFU mode is ready (using PAC)
        // PE2, active low - reset bit 2
        // Enable GPIOE clock first
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.ahb4enr.modify(|_, w| w.gpioeen().set_bit());
        // Set PE2 as output (bits 4-5 = 01 for output mode)
        dp.GPIOE
            .moder
            .modify(|r, w| unsafe { w.bits((r.bits() & !(0b11 << 4)) | (0b01 << 4)) });
        // Drive PE2 low (active low LED)
        dp.GPIOE.bsrr.write(|w| unsafe { w.bits(1 << (2 + 16)) });

        // Main USB polling loop
        loop {
            usb_dev.poll(&mut [&mut dfu]);
        }
    }
}
