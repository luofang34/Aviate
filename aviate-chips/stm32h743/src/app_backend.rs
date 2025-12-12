//! Application validation and jump for STM32H743
//!
//! Validates application firmware and performs vector table relocation + jump

use aviate_boot_core::AppBackend;

// Import memory layout constants from chip configuration
use crate::memory::{APP_END, APP_START, RAM_END, RAM_START};

pub struct Stm32h743AppBackend;

impl Stm32h743AppBackend {
    pub fn new() -> Self {
        Self
    }
}

impl AppBackend for Stm32h743AppBackend {
    fn validate_app(&self, app_start: u32) -> bool {
        // Read stack pointer and reset vector from app start
        let app_stack = unsafe { core::ptr::read_volatile(app_start as *const u32) };
        let app_reset = unsafe { core::ptr::read_volatile((app_start + 4) as *const u32) };

        // Stack pointer should point to valid RAM (AXI SRAM where HAL places stack)
        let stack_valid = (RAM_START..=RAM_END).contains(&app_stack);

        // Reset vector should point to application flash region
        let reset_valid = (APP_START..=APP_END).contains(&app_reset);

        stack_valid && reset_valid
    }

    unsafe fn jump_to_app(&self, app_start: u32) -> ! {
        // Read stack pointer and reset vector
        let app_stack = core::ptr::read_volatile(app_start as *const u32);
        let app_reset = core::ptr::read_volatile((app_start + 4) as *const u32);

        // Disable interrupts
        cortex_m::interrupt::disable();

        // Set vector table offset register (VTOR) to application
        let scb = &*stm32h7xx_hal::pac::SCB::PTR;
        scb.vtor.write(app_start);

        // Memory barriers
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // Set MSP and jump to app reset handler
        core::arch::asm!(
            "msr msp, {0}",
            "bx {1}",
            in(reg) app_stack,
            in(reg) app_reset,
            options(noreturn)
        );
    }
}
