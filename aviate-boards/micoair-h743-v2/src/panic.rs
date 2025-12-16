use core::panic::PanicInfo;
use aviate_boot_core::magic;
use stm32h7xx_hal::pac;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Disable interrupts to ensure atomic operation
    cortex_m::interrupt::disable();

    // Enable backup access to write RTC registers
    // Safety: direct register access for crash handling
    let pwr = unsafe { &*pac::PWR::ptr() };
    pwr.cr1.modify(|_, w| w.dbp().set_bit());
    
    // Wait for write access? DBP is immediate.

    // Write CRASH_DETECTED magic to RTC_BKPR1
    // This tells the bootloader to stay in DFU mode after reset
    let rtc = unsafe { &*pac::RTC::ptr() };
    
    // Check if RTC is accessible/clocked? 
    // Usually RTC domain is always on if Vbat is present, but APB clock needed?
    // D3/D1 logic...
    // Assuming RTC APB clock enabled in init. 
    // If not, we might fault here. But we are already panicking.
    // Try to enable RTC clock just in case?
    // RCC.apb4enr.rtcapen? No, rtc en is in bdcr.
    
    // Write magic
    rtc.bkpr[1].write(|w| w.bits(magic::CRASH_DETECTED));
    
    // Ensure memory operations complete
    cortex_m::asm::dsb();

    // Loop forever - Watchdog will reset the system
    loop {
        cortex_m::asm::nop();
    }
}
