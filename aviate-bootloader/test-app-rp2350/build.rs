use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Provide memory.x - rp235x-pac only provides device.x (interrupt vectors).
    // The app is linked into the application partition (0x1001_0000), above
    // the bootloader's 64 KiB region. It carries no .start_block: the Boot
    // ROM validates the bootloader image, and the bootloader jumps directly
    // to this application's vector table.
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
    // Link the Cortex-M runtime script. A search path alone does not apply
    // link.x, which is why the standalone build produced an unlinked shell
    // (entry 0x0, no vector table). The package owns this so a workspace
    // cargo config cannot silently drop it.
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=build.rs");
}
