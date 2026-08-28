//! Build identity values supplied by Cargo.

fn main() {
    let target = std::env::var("TARGET").unwrap_or_else(|_| "unknown-target".to_owned());
    let profile = std::env::var("PROFILE").unwrap_or_else(|_| "unknown-profile".to_owned());
    let compiler = compiler_identity();
    let mut features = std::env::vars()
        .filter_map(|(name, _)| name.strip_prefix("CARGO_FEATURE_").map(str::to_owned))
        .collect::<Vec<_>>();
    features.sort();
    let crate_features = if features.is_empty() {
        "none".to_owned()
    } else {
        features.join(",").to_ascii_lowercase().replace('_', "-")
    };
    println!("cargo:rustc-env=AVIATE_BUILD_TARGET={target}");
    println!("cargo:rustc-env=AVIATE_BUILD_PROFILE={profile}");
    println!("cargo:rustc-env=AVIATE_BUILD_COMPILER={compiler}");
    println!(
        "cargo:rustc-env=AVIATE_BUILD_FEATURES={crate_features};dependencies=aviate-runtime/env-sitl"
    );
    println!("cargo:rerun-if-changed=build.rs");
}

fn compiler_identity() -> String {
    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".to_owned());
    std::process::Command::new(rustc)
        .arg("--version")
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .map_or_else(
            || "unknown-rustc".to_owned(),
            |value| value.trim().to_owned(),
        )
}
