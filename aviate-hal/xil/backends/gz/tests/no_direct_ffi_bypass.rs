//! Guardrail for the retired direct-FFI gz-bridge bypass.
//!
//! `gazebo_bridge.rs` was a `#[no_mangle] extern "C"` channel that
//! fed sensors and pulled motor commands straight into the flight
//! controller, "bypassing MAVLink entirely" — a sim-only path into
//! the real application that outlived its purpose once the
//! shared-memory contract landed, and that forced this crate to
//! build as a `cdylib` (whose unhashed artifact collides under
//! `cargo test --all-targets`; cargo#6313).
//!
//! It is gone. These checks fail if it — or the artifact shape that
//! enabled it — comes back: the plugin talks to the FC only over the
//! `aviate-xil-contract` shared block, and this backend must expose
//! no C ABI of its own.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::PathBuf;

fn read(rel: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

#[test]
fn crate_exposes_no_c_abi() {
    // A cdylib/staticlib crate-type is the artifact shape a C
    // consumer links; this backend has no such consumer. Inspect the
    // `crate-type = [...]` value itself, not the whole file — the
    // surrounding comment deliberately names cdylib to explain why
    // it is banned.
    let cargo = read("Cargo.toml");
    let crate_type = cargo
        .lines()
        .find(|l| l.trim_start().starts_with("crate-type"))
        .expect("Cargo.toml declares an explicit [lib] crate-type");
    assert!(
        !crate_type.contains("cdylib") && !crate_type.contains("staticlib"),
        "aviate-backend-gz must stay rlib-only — a C-linkable artifact \
         reintroduces the direct-FFI bypass and the cargo#6313 collision \
         (crate-type = {crate_type:?})"
    );
}

#[test]
fn no_extern_c_bypass_surface_in_sources() {
    // Walk the crate's own sources; any `#[no_mangle]` extern "C"
    // entry point here is a resurrected bypass into the FC.
    let src = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut offenders = Vec::new();
    let mut stack = vec![src];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(&dir).expect("read src dir") {
            let path = entry.expect("dir entry").path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().is_some_and(|e| e == "rs") {
                let text = std::fs::read_to_string(&path).expect("read source");
                if text.contains("no_mangle") || text.contains("extern \"C\"") {
                    offenders.push(path);
                }
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "direct-FFI bypass surface is back in: {offenders:?} — the gz \
         plugin must reach the FC only over the shared-memory contract"
    );
}
