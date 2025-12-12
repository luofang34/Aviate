#!/bin/bash
# Verify hardware template generation and build

set -e

cd /home/fang/Aviate

echo "=== Generating hardware app from template ==="
rm -rf aviate-apps/test-hw-app
cargo generate --path aviate-app-template --name test-hw-app --destination aviate-apps \
    -d board=micoair-h743-v2 -d model=x500 -d airframe=x500 -d env=flight

echo ""
echo "=== Generated Cargo.toml ==="
cat aviate-apps/test-hw-app/Cargo.toml

echo ""
echo "=== Setting up .cargo/config.toml ==="
cd aviate-apps/test-hw-app
mkdir -p .cargo
cp .cargo-flight/config.toml .cargo/config.toml

echo ""
echo "=== Building for thumbv7em-none-eabihf ==="
cargo build --release 2>&1 | tail -20

echo ""
echo "=== SUCCESS ==="
