use std::env;
use std::path::PathBuf;

fn main() {
    // Only link if the gz-plugin feature is enabled
    if env::var("CARGO_FEATURE_GZ_PLUGIN").is_ok() {
        // Locate the C++ build directory relative to this crate
        // crate: aviate-platform/backends/gz
        // build: aviate-platform/aviate_gz_plugin/build
        let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
        let crate_dir = PathBuf::from(manifest_dir);

        // Try new location first
        let build_dir_new = crate_dir
            .parent() // backends
            .and_then(|p| p.parent()) // aviate-platform
            .map(|p| p.join("aviate_gz_plugin/build"));

        // Try legacy location
        let build_dir_legacy = crate_dir
            .parent() // backends
            .and_then(|p| p.parent()) // aviate-platform
            .map(|p| p.join("sitl/aviate_gz_plugin/build"));

        let build_dir = if let Some(p) = build_dir_new {
            if p.exists() {
                p
            } else {
                build_dir_legacy.unwrap_or_default()
            }
        } else {
            PathBuf::from(".")
        };

        if !build_dir.exists() {
            println!("cargo:warning=AviateGzPlugin build directory not found at {:?}. FFI linking may fail.", build_dir);
            println!("cargo:warning=Please build the plugin first: cd aviate-platform/aviate_gz_plugin/build && cmake .. && make");
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
