use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Provide memory.x - rp235x-pac only provides device.x (interrupt vectors)
    // App starts at 0x10010000 (after 64KB bootloader region)
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
    println!("cargo:rerun-if-changed=build.rs");
}
