use std::env;
use std::path::PathBuf;

fn main() {
    // Only add RPATH if gz-plugin feature is enabled
    if env::var("CARGO_FEATURE_GZ_PLUGIN").is_ok() {
        // Locate the plugin build directory
        // This crate: aviate-apps/sitl-gazebo-x500
        // Plugin:     aviate-hal/xil/backends/gz/plugin/build
        let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
        let crate_dir = PathBuf::from(&manifest_dir);

        let build_dir = crate_dir
            .join("../../aviate-hal/xil/backends/gz/plugin/build")
            .canonicalize()
            .unwrap_or_else(|_| crate_dir.join("../../aviate-hal/xil/backends/gz/plugin/build"));

        // Embed RPATH for runtime library loading using $ORIGIN (portable)
        // Binary: target/{debug,release}/sitl-gazebo-x500
        // Library: aviate-hal/xil/backends/gz/plugin/build/libaviate_gz_bridge.so
        //
        // OUT_DIR is something like: target/debug/build/aviate-app-sitl-gazebo-x500-xxx/out
        // We need to go up to target/{debug,release}/ which is 3 levels up
        let out_dir = env::var("OUT_DIR").unwrap();
        let target_dir = PathBuf::from(&out_dir)
            .ancestors()
            .nth(3)
            .map(|p| p.to_path_buf())
            .unwrap_or_else(|| PathBuf::from(&out_dir));

        if let Some(rel_path) = pathdiff::diff_paths(&build_dir, &target_dir) {
            // Use $ORIGIN-relative RPATH for portability
            println!(
                "cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/{}",
                rel_path.display()
            );
        } else {
            // Fallback to absolute path if relative path calculation fails
            println!("cargo:rustc-link-arg=-Wl,-rpath,{}", build_dir.display());
        }
    }
}
