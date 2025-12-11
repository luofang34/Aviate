use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Determine which chip is selected and copy its memory.x
    let chip_path = if cfg!(feature = "chip-stm32h743") {
        "../aviate-chips/stm32h743/memory.x"
    } else {
        panic!("No chip selected! Enable exactly one chip-* feature.");
    };

    // Put memory.x in our output directory
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let memory_x_path = PathBuf::from(chip_path);

    fs::copy(&memory_x_path, out.join("memory.x"))
        .unwrap_or_else(|e| panic!("Failed to copy memory.x from {:?}: {}", memory_x_path, e));

    println!("cargo:rustc-link-search={}", out.display());

    // Re-run if the chip's memory.x changes
    println!("cargo:rerun-if-changed={}", chip_path);
}
