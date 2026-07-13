use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // rp235x-pac provides device.x, while the application owns this partition
    // map. The bootloader reads the application vectors at APP_START during
    // handoff; only the bootloader image carries the Boot ROM .start_block.
    let memory_x = r#"MEMORY
{
    FLASH : ORIGIN = 0x10010000, LENGTH = 1008K
    RAM   : ORIGIN = 0x20000000, LENGTH = 520K
}
"#;

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memory_x.as_bytes())
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    // Own the Cortex-M runtime linker argument at the package boundary so
    // workspace-level Cargo configuration cannot omit it.
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=build.rs");
}
