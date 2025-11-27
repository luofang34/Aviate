//! Build script for aviate-platform-sitl
//!
//! When the `gz-plugin` feature is enabled, this links against libaviate_gz_bridge.

fn main() {
    // Only process gz-plugin feature
    #[cfg(feature = "gz-plugin")]
    {
        // Path to the built aviate_gz_plugin libraries
        let plugin_build_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/aviate_gz_plugin/build");

        // Add library search path
        println!("cargo:rustc-link-search=native={}", plugin_build_dir);

        // Link against the bridge library
        println!("cargo:rustc-link-lib=dylib=aviate_gz_bridge");

        // Also need to link rt for shm_open
        println!("cargo:rustc-link-lib=rt");

        // Re-run if the library changes
        println!("cargo:rerun-if-changed={}/libaviate_gz_bridge.so", plugin_build_dir);
    }
}
