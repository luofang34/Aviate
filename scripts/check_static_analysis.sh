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

# The bare-metal app lives in its own standalone workspace
# (aviate-apps/micoair-h743-v2-test/Cargo.toml has a [workspace] table),
# so the parent --workspace flag does not descend into it.
EMBEDDED_MANIFEST=aviate-apps/micoair-h743-v2-test/Cargo.toml

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
echo "Checking embedded app..."
cargo check --manifest-path "$EMBEDDED_MANIFEST" --target thumbv7em-none-eabihf

echo "Running clippy on target..."
cargo clippy --manifest-path "$EMBEDDED_MANIFEST" --target thumbv7em-none-eabihf -- -D warnings

# 4. Doc Check
echo ""
echo "--- Documentation Check ---"
cargo doc --workspace --no-deps --document-private-items
env RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps

echo ""
echo "PASSED: All static analysis checks passed."
