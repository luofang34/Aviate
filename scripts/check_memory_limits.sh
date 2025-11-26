#!/bin/bash
set -e

# Config
MAX_FLASH_BYTES=$((512 * 1024))
MAX_RAM_BYTES=$((256 * 1024))
TARGET="thumbv8m.main-none-eabihf"

echo "=== Memory Limit Verification ==="
echo "Target: $TARGET"

# Build library (we check the core library footprint primarily)
echo "Building aviate-core..."
cargo build -p aviate-core --target $TARGET --release

# Find the rlib
RLIB=$(ls target/$TARGET/release/deps/libaviate_core-*.rlib | head -n 1)

if [ ! -f "$RLIB" ]; then
    echo "Error: Library not found"
    exit 1
fi

echo "Analyzing footprint of: $RLIB"

# Extract size of all object files inside the rlib
# 'size' on archive lists members.
# Output: text data bss ... filename
# We sum the columns.

SIZE_OUTPUT=$(size "$RLIB" | tail -n +2) # skip header

TOTAL_TEXT=0
TOTAL_DATA=0
TOTAL_BSS=0

while read -r text data bss _rest; do
    # Skip if it's lib.rmeta (which has 0 usually but just in case)
    # size output lines are: <size> <size> <size> <dec> <hex> <filename>
    # We handle potential archive member lines
    if [[ "$text" =~ ^[0-9]+$ ]]; then
        TOTAL_TEXT=$((TOTAL_TEXT + text))
        TOTAL_DATA=$((TOTAL_DATA + data))
        TOTAL_BSS=$((TOTAL_BSS + bss))
    fi
done <<< "$SIZE_OUTPUT"

FLASH_USAGE=$((TOTAL_TEXT + TOTAL_DATA))
RAM_USAGE=$((TOTAL_DATA + TOTAL_BSS))

echo "--------------------------------"
echo "Core Library Footprint:"
echo "  Code (.text): $TOTAL_TEXT bytes"
echo "  Data (.data): $TOTAL_DATA bytes"
echo "  BSS  (.bss):  $TOTAL_BSS bytes"
echo "--------------------------------"
echo "Total Flash Contrib: $FLASH_USAGE bytes"
echo "Total RAM Contrib:   $RAM_USAGE bytes"
echo "--------------------------------"
echo "Target Limit: Flash < ${MAX_FLASH_BYTES}, RAM < ${MAX_RAM_BYTES}"

if [ $FLASH_USAGE -gt $MAX_FLASH_BYTES ]; then
    echo "❌ FAILURE: Core library exceeds Flash limit!"
    exit 1
fi

if [ $RAM_USAGE -gt $MAX_RAM_BYTES ]; then
    echo "❌ FAILURE: Core library exceeds RAM limit!"
    exit 1
fi

echo "✅ SUCCESS: Core footprint is within limits."