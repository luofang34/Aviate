#![no_std]
#![no_main]

use aviate_core::AviateKernel;

use aviate_core::control::mc::McController;

use aviate_core::control::Command;

use aviate_core::types::Normalized;

use cortex_m_rt::entry;

use panic_halt as _;

use core::hint::black_box;



#[entry]

fn main() -> ! {

    // Initialize Kernel

    let mut kernel = AviateKernel::new(McController);

    

    // Dummy command to prevent optimization stripping everything

    let cmd = Command { collective_thrust: Normalized(0.5) };

    

    loop {

        // Run one step

        let output = kernel.step(black_box(&cmd));

        

        // Prevent optimization of the loop

        black_box(output);

    }

}
