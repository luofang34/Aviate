//! Executable cross-language agreement on shm names: the
//! `aviate_shm_instance_name` constructor shipped inside the
//! checked-in C header must produce byte-identical names to the Rust
//! [`aviate_xil_contract::shm_name`] authority. Compiles the probe
//! against `include/aviate_xil_contract.h` — the exact header the
//! gz plugin consumes — runs it, and diffs the output.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::PathBuf;
use std::process::Command;

/// Both edges (0 and the maximum supported instance) plus interior
/// points; the same list is baked into the C++ probe below.
const INSTANCES: [u32; 5] = [0, 1, 7, 4096, u32::MAX];

/// Same resolution order as scripts/check_xil_contract_header.sh.
fn find_cxx() -> Option<String> {
    if let Ok(cxx) = std::env::var("CXX") {
        if !cxx.is_empty() {
            return Some(cxx);
        }
    }
    ["c++", "g++", "clang++"].iter().find_map(|c| {
        Command::new(c)
            .arg("--version")
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|_| (*c).to_string())
    })
}

#[test]
fn c_and_rust_construct_identical_names() {
    let cxx = find_cxx().expect(
        "a C++ compiler is required (set CXX, or install c++/g++/clang++) — \
         the same requirement as scripts/check_xil_contract_header.sh",
    );

    let include_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("include");
    let tmp = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    let src = tmp.join("name_agreement_probe.cc");
    let bin = tmp.join("name_agreement_probe");

    let instance_list = INSTANCES
        .iter()
        .map(|i| format!("{i}u"))
        .collect::<Vec<_>>()
        .join(", ");
    let probe = format!(
        r#"#include "aviate_xil_contract.h"
#include <cstdio>
int main()
{{
    const uint32_t instances[] = {{{instance_list}}};
    for (uint32_t instance : instances) {{
        char buf[AviateSHM_NAME_MAX + 1];
        if (aviate_shm_instance_name(instance, buf, sizeof buf) != 0) {{
            return 1;
        }}
        std::printf("%s\n", buf);
    }}
    // A too-small buffer must be rejected, never truncated.
    char small[4];
    if (aviate_shm_instance_name(1u, small, sizeof small) == 0) {{
        return 2;
    }}
    return 0;
}}
"#
    );
    std::fs::write(&src, probe).expect("write probe source");

    let compile = Command::new(&cxx)
        .args(["-std=c++17", "-I"])
        .arg(&include_dir)
        .arg(&src)
        .arg("-o")
        .arg(&bin)
        .output()
        .expect("run C++ compiler");
    assert!(
        compile.status.success(),
        "probe failed to compile against the checked-in header:\n{}",
        String::from_utf8_lossy(&compile.stderr)
    );

    let run = Command::new(&bin).output().expect("run probe");
    assert!(
        run.status.success(),
        "probe exited {:?} — the header's constructor rejected a valid \
         instance or accepted a too-small buffer",
        run.status.code()
    );

    let cxx_names: Vec<&str> = std::str::from_utf8(&run.stdout)
        .expect("probe output is UTF-8")
        .lines()
        .collect();
    let rust_names: Vec<String> = INSTANCES
        .iter()
        .map(|&i| aviate_xil_contract::shm_name(i).as_str().to_string())
        .collect();
    assert_eq!(
        cxx_names, rust_names,
        "the C header's aviate_shm_instance_name and Rust shm_name disagree"
    );
}
