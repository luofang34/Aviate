#!/bin/bash
set -e

# Aviate Static Analysis Suite
# Enforces strict lints and successful compilation for all targets.

echo "========================================"
echo "Aviate Static Analysis"
echo "========================================"

# 1. Prerequisites
if ! rustup target list | grep -q "thumbv7em-none-eabihf.*(installed)"; then
    echo "Installing ARM target..."
    rustup target add thumbv7em-none-eabihf
fi

# Hardware-only crates are excluded from the parent workspace, so
# --workspace commands skip them here and they're checked separately
# below against the thumbv7em target.
EMBEDDED_HAL_MANIFEST=aviate-hal/stm32h7/Cargo.toml

# 2. Host Analysis (x86_64)
echo ""
echo "--- Host Analysis (x86_64) ---"
echo "Running clippy on workspace..."
cargo clippy --workspace -- -D warnings

echo "Running tests compilation check..."
cargo test --workspace --no-run

# 3. Target Analysis (ARM Cortex-M7)
echo ""
echo "--- Target Analysis (ARM Cortex-M7) ---"
echo "Checking embedded HAL..."
cargo check --manifest-path "$EMBEDDED_HAL_MANIFEST" --target thumbv7em-none-eabihf

echo "Running clippy on target..."
cargo clippy --manifest-path "$EMBEDDED_HAL_MANIFEST" --target thumbv7em-none-eabihf -- -D warnings

# 4. Doc Check
echo ""
echo "--- Documentation Check ---"
cargo doc --workspace --no-deps --document-private-items
env RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps

echo ""
echo "PASSED: All static analysis checks passed."

# Position-controller limits stay hash-covered: the only constructor
# takes explicit limits, so an implicit-default constructor cannot
# reappear unreviewed (one production tuning source; see
# cert/trace DRQ-CTL-001).
if grep -En "pub fn (new|default)\b" aviate-core/src/control/position.rs > /dev/null; then
    echo "FAIL: implicit-default constructor reintroduced in control/position.rs" >&2
    echo "Position controllers must take explicit limits from the hash-covered config." >&2
    exit 1
fi
