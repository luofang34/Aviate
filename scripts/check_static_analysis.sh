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

# 2. Host Analysis (x86_64)
echo ""
echo "--- Host Analysis (x86_64) ---"
echo "Running clippy on workspace..."
# Exclude bare-metal app from host build
cargo clippy --workspace --exclude aviate-app-quadcopter-stm32h7 -- -D warnings

echo "Running tests compilation check..."
cargo test --workspace --exclude aviate-app-quadcopter-stm32h7 --no-run

# 3. Target Analysis (ARM Cortex-M7)
echo ""
echo "--- Target Analysis (ARM Cortex-M7) ---"
echo "Checking quadcopter-stm32h7..."
cargo check -p aviate-app-quadcopter-stm32h7 --target thumbv7em-none-eabihf

echo "Running clippy on target..."
cargo clippy -p aviate-app-quadcopter-stm32h7 --target thumbv7em-none-eabihf -- -D warnings

# 4. Doc Check
echo ""
echo "--- Documentation Check ---"
cargo doc --workspace --exclude aviate-app-quadcopter-stm32h7 --no-deps --document-private-items
env RUSTDOCFLAGS="-D warnings" cargo doc --workspace --exclude aviate-app-quadcopter-stm32h7 --no-deps

echo ""
echo "PASSED: All static analysis checks passed."
