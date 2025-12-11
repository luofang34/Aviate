//! Delay backend implementation for STM32H743
//!
//! Uses DWT cycle counter with runtime clock detection.
//! Falls back to NOP loop if DWT unavailable.

use aviate_boot_core::Delay;
use stm32h7xx_hal::pac;

pub struct Stm32h743DelayBackend {
    clock_hz: u32,
    use_dwt: bool,
}

impl Stm32h743DelayBackend {
    pub fn new() -> Self {
        let clock_hz = Self::detect_boot_clock_hz();
        let use_dwt = Self::try_enable_dwt();

        Self { clock_hz, use_dwt }
    }

    /// Detect boot clock frequency from RCC registers
    fn detect_boot_clock_hz() -> u32 {
        let rcc = unsafe { &*pac::RCC::ptr() };

        // Read system clock switch status (SWS field)
        let sws = rcc.cfgr.read().sws().bits();

        // Get base clock frequency
        let base_freq = match sws {
            0 => 64_000_000,  // HSI (64 MHz) - DEFAULT after reset
            1 => 4_000_000,   // CSI (4 MHz)
            2 => 32_768,      // HSE (varies)
            3 => 64_000_000,  // PLL (assume HSI source in bootloader)
            _ => 64_000_000,  // Default to HSI
        };

        // Read AHB prescaler (HPRE field in D1CFGR) to get actual CPU frequency
        let hpre = rcc.d1cfgr.read().hpre().bits();

        // HPRE encoding: 0-7 = no division, 8 = /2, 9 = /4, 10 = /8, etc.
        let divisor = match hpre {
            0..=7 => 1,
            8 => 2,
            9 => 4,
            10 => 8,
            11 => 16,
            12 => 64,
            13 => 128,
            14 => 256,
            15 => 512,
            _ => 1,
        };

        base_freq / divisor
    }

    /// Try to enable DWT cycle counter
    fn try_enable_dwt() -> bool {
        let mut cp = unsafe { cortex_m::Peripherals::steal() };

        // Enable DWT
        unsafe {
            cp.DCB.demcr.modify(|r| r | (1 << 24)); // TRCENA bit
            cp.DWT.cyccnt.write(0);
            cp.DWT.ctrl.modify(|r| r | 1); // Enable CYCCNT
        }

        // Verify CYCCNT is actually incrementing
        let start = cp.DWT.cyccnt.read();
        for _ in 0..100 {
            cortex_m::asm::nop();
        }
        let end = cp.DWT.cyccnt.read();

        end > start  // Return true only if counter is working
    }

    /// Delay using DWT cycle counter (accurate)
    fn delay_dwt(&self, ms: u32) {
        let cycles = (self.clock_hz / 1000) * ms;
        let cp = unsafe { cortex_m::Peripherals::steal() };
        let start = cp.DWT.cyccnt.read();

        loop {
            let now = cp.DWT.cyccnt.read();
            if now.wrapping_sub(start) >= cycles {
                break;
            }
        }
    }

    /// Delay using NOP loop (fallback)
    /// Use volatile operations to prevent compiler optimization
    #[inline(never)]
    fn delay_nop(&self, ms: u32) {
        let iterations = if ms == 500 {
            25_000  // Exactly 1/20th of 500_000 (which gave 10s)
        } else {
            ms * 50
        };

        // Use volatile counter to prevent optimization
        let mut counter: u32 = 0;
        for _ in 0..iterations {
            unsafe {
                core::ptr::write_volatile(&mut counter,
                    core::ptr::read_volatile(&counter).wrapping_add(1));
            }
            cortex_m::asm::nop();
        }

        // Prevent compiler from optimizing away the entire loop
        core::hint::black_box(counter);
    }
}

impl Delay for Stm32h743DelayBackend {
    fn delay_ms(&mut self, ms: u32) {
        if self.use_dwt {
            self.delay_dwt(ms);
        } else {
            self.delay_nop(ms);
        }
    }
}
