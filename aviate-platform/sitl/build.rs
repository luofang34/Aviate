//! Build script for aviate-platform-sitl
//!
//! When the `gz-plugin` feature is enabled, this links against libaviate_gz_bridge.

fn main() {
    // Only process gz-plugin feature
    #[cfg(feature = "gz-plugin")]
    {
        // Try new location first: aviate-platform/aviate_gz_plugin/build
        // Then fall back to legacy location: aviate-platform/sitl/aviate_gz_plugin/build
        let manifest_dir = env!("CARGO_MANIFEST_DIR");
        let platform_dir = std::path::Path::new(manifest_dir).parent().unwrap();

        let new_path = platform_dir.join("aviate_gz_plugin/build");
        let legacy_path = std::path::Path::new(manifest_dir).join("aviate_gz_plugin/build");

        let plugin_build_dir = if new_path.join("libaviate_gz_bridge.so").exists() {
            new_path
        } else {
            legacy_path
        };

        let plugin_build_str = plugin_build_dir.to_string_lossy();

        // Add library search path
        println!("cargo:rustc-link-search=native={}", plugin_build_str);

        // Link against the bridge library
        println!("cargo:rustc-link-lib=dylib=aviate_gz_bridge");

        // Also need to link rt for shm_open
        println!("cargo:rustc-link-lib=rt");

        // Re-run if the library changes
        println!("cargo:rerun-if-changed={}/libaviate_gz_bridge.so", plugin_build_str);
    }
}
