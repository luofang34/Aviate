#![no_std]
#![no_main]

use aviate_core::AviateKernel;
use aviate_core::control::mc::McController;
use aviate_core::control::Command;
use aviate_core::types::Normalized;
use cortex_m_rt::entry;
use panic_halt as _;

// Force a symbol to be kept
#[used]
static mut SINK: u32 = 0;

#[entry]
fn main() -> ! {
    let mut kernel = AviateKernel::new(McController);
    let cmd = Command { collective_thrust: Normalized(0.5) };
    
    loop {
        let output = kernel.step(&cmd);
        
        // Force side effect
        unsafe {
            core::ptr::write_volatile(&mut SINK, output.collective.0.to_bits());
        }
    }
}