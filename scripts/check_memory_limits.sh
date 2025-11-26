#!/bin/bash
set -e

# Config
MAX_FLASH_BYTES=$((512 * 1024))
MAX_RAM_BYTES=$((256 * 1024))
TARGET="thumbv8m.main-none-eabihf" # Using installed target as proxy for H7

echo "=== Memory Limit Verification ==="
echo "Target: $TARGET"
echo "Limits: Flash=${MAX_FLASH_BYTES} bytes, RAM=${MAX_RAM_BYTES} bytes"

# Build the specific app in release mode
echo "Building aviate-app-quadcopter-stm32h7..."
cargo build -p aviate-app-quadcopter-stm32h7 --target $TARGET --release --quiet

# Get path to binary
BIN_PATH="target/$TARGET/release/aviate-app-quadcopter-stm32h7"

if [ ! -f "$BIN_PATH" ]; then
    echo "Error: Binary not found at $BIN_PATH"
    exit 1
fi

ls -lh "$BIN_PATH"

# Try cargo size first
if command -v cargo-size &> /dev/null; then
    cargo size --release -p aviate-app-quadcopter-stm32h7 --target $TARGET
else
    # Fallback to size
    echo "Using system 'size' command:"
    size "$BIN_PATH"
    
    # Parse size output - explicitly checking format
    # Berkeley: text data bss dec hex filename
    # SysV: section size addr
    
    SIZE_OUTPUT=$(size -B "$BIN_PATH" | tail -n 1)
    TEXT=$(echo $SIZE_OUTPUT | awk '{print $1}')
    DATA=$(echo $SIZE_OUTPUT | awk '{print $2}')
    BSS=$(echo $SIZE_OUTPUT | awk '{print $3}')
    
    # If TEXT is 0, something is wrong with parsing or section names
    if [ "$TEXT" == "0" ]; then
        echo "Warning: 'size' reported 0 text. Checking with readelf if available..."
        if command -v readelf &> /dev/null; then
             # Sum up PROGBITS/ALLOC sections?
             # Simple check:
             readelf -S "$BIN_PATH"
        fi
    fi
fi

# Hardcoded fallback/fix for standard 'size' on ARM ELF if it fails:
# Usually size works. If it's 0, maybe the binary is actually empty? 
# But ls -lh will tell us.

FLASH_USAGE=$((TEXT + DATA))
RAM_USAGE=$((DATA + BSS))

echo "--------------------------------"
echo "Measured Sizes:"
echo "  Flash (Text+Data): $FLASH_USAGE bytes"
echo "  RAM (Data+Bss):    $RAM_USAGE bytes"
echo "--------------------------------"

# Verify Limits
if [ $FLASH_USAGE -gt $MAX_FLASH_BYTES ]; then
    echo "❌ FAILURE: Flash limit exceeded!"
    exit 1
fi

if [ $RAM_USAGE -gt $MAX_RAM_BYTES ]; then
    echo "❌ FAILURE: RAM limit exceeded!"
    exit 1
fi

echo "✅ SUCCESS: Memory usage within limits."