use std::env;
use std::path::PathBuf;

fn main() {
    // Only link if the gz-plugin feature is enabled
    if env::var("CARGO_FEATURE_GZ_PLUGIN").is_ok() {
        // Locate the C++ build directory relative to this crate
        // crate: aviate-hal/xil/backends/gz
        // build: aviate-hal/xil/backends/gz/plugin/build
        let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
        let crate_dir = PathBuf::from(manifest_dir);

        // Plugin is now a subdirectory of this crate
        let build_dir = crate_dir.join("plugin/build");

        if !build_dir.exists() {
            println!("cargo:warning=AviateGzPlugin build directory not found at {:?}. FFI linking may fail.", build_dir);
            println!("cargo:warning=Please build the plugin first: cd aviate-hal/xil/backends/gz/plugin/build && cmake .. && make");
        }

        // Link against the bridge library
        println!("cargo:rustc-link-search=native={}", build_dir.display());
        println!("cargo:rustc-link-lib=dylib=aviate_gz_bridge");

        // Re-run if library changes
        println!(
            "cargo:rerun-if-changed={}/libaviate_gz_bridge.so",
            build_dir.display()
        );
    }
}
