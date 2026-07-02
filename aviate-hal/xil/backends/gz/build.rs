use std::env;
use std::path::PathBuf;

fn main() {
    // The shared-memory FFI bridge is only needed when `gz-plugin` is on.
    if env::var("CARGO_FEATURE_GZ_PLUGIN").is_err() {
        return;
    }

    // `plugin/aviate_gz_bridge.cc` is a self-contained POSIX shared-
    // memory shim (shm_open / mmap, no Gazebo headers), so compile it
    // from source here instead of requiring a prebuilt
    // `libaviate_gz_bridge_static.a`. This keeps `cargo build` and
    // `cargo test` self-contained on hosts without the Gazebo/CMake
    // toolchain (CI). The full gz-sim system plugin (the `.so` Gazebo
    // dlopen's at runtime) is still produced separately via CMake — it
    // is a runtime artifact, not a link-time dependency of this crate.
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR set by cargo");
    let plugin_dir = PathBuf::from(&manifest_dir).join("plugin");
    let bridge_src = plugin_dir.join("aviate_gz_bridge.cc");

    println!("cargo:rerun-if-changed={}", bridge_src.display());
    println!(
        "cargo:rerun-if-changed={}",
        plugin_dir.join("aviate_gz_bridge.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        plugin_dir.join("shared_state.h").display()
    );

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .file(&bridge_src)
        .include(&plugin_dir)
        .compile("aviate_gz_bridge_static");

    // shm_open / mmap resolve against librt on older Linux glibc; macOS
    // folds POSIX RT into libSystem so no extra link is needed there.
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux") {
        println!("cargo:rustc-link-lib=rt");
    }
}
