#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if command -v arm-none-eabi-readelf >/dev/null 2>&1; then
    READELF=arm-none-eabi-readelf
elif command -v readelf >/dev/null 2>&1; then
    READELF=readelf
else
    echo "FAIL: readelf is required to verify bootloader images" >&2
    exit 1
fi

section_fields() {
    local elf=$1
    local section=$2

    "$READELF" --sections --wide "$elf" | awk -v target="$section" '
        $1 == "[" {
            name = $3
            address = $5
            size = $7
        }
        $1 ~ /^\[[0-9]+\]$/ {
            name = $2
            address = $4
            size = $6
        }
        name == target {
            print address, size
            exit
        }
    '
}

require_section() {
    local elf=$1
    local section=$2
    local expected_address=$3
    local fields
    local address
    local size

    fields="$(section_fields "$elf" "$section")"
    if [[ -z "$fields" ]]; then
        echo "FAIL: $elf is missing $section" >&2
        return 1
    fi

    read -r address size <<<"$fields"
    if (( 16#$address != expected_address )); then
        printf 'FAIL: %s has %s at 0x%s, expected 0x%08x\n' \
            "$elf" "$section" "$address" "$expected_address" >&2
        return 1
    fi

    if (( 16#$size == 0 )); then
        echo "FAIL: $elf has an empty $section" >&2
        return 1
    fi
}

require_nonempty_section() {
    local elf=$1
    local section=$2
    local fields
    local size

    fields="$(section_fields "$elf" "$section")"
    if [[ -z "$fields" ]]; then
        echo "FAIL: $elf is missing $section" >&2
        return 1
    fi

    read -r _ size <<<"$fields"
    if (( 16#$size == 0 )); then
        echo "FAIL: $elf has an empty $section" >&2
        return 1
    fi
}

require_entry_in_range() {
    local elf=$1
    local range_start=$2
    local range_end=$3
    local entry
    local entry_hex

    entry="$($READELF --file-header "$elf" | awk '/Entry point address:/ { print $4 }')"
    if [[ -z "$entry" ]]; then
        echo "FAIL: could not read the entry point from $elf" >&2
        return 1
    fi

    entry_hex=${entry#0x}
    if (( 16#$entry_hex < range_start || 16#$entry_hex >= range_end )); then
        printf 'FAIL: %s entry point %s is outside [0x%08x, 0x%08x)\n' \
            "$elf" "$entry" "$range_start" "$range_end" >&2
        return 1
    fi
}

verify_stm32h743() {
    local elf="$REPO_ROOT/aviate-bootloader/target/thumbv7em-none-eabihf/release/aviate-bootloader"

    [[ -f "$elf" ]] || {
        echo "FAIL: STM32H743 bootloader ELF not found: $elf" >&2
        return 1
    }

    require_entry_in_range "$elf" 0x08000000 0x08020000
    require_section "$elf" .vector_table 0x08000000
    require_nonempty_section "$elf" .text
    echo "STM32H743 bootloader image: OK"
}

verify_rp2350() {
    local elf="$REPO_ROOT/aviate-bootloader/target/thumbv8m.main-none-eabihf/release/aviate-bootloader"

    [[ -f "$elf" ]] || {
        echo "FAIL: RP2350 bootloader ELF not found: $elf" >&2
        return 1
    }

    require_entry_in_range "$elf" 0x10000000 0x10040000
    require_section "$elf" .start_block 0x10000000
    require_section "$elf" .vector_table 0x10000100
    require_nonempty_section "$elf" .text
    echo "RP2350 bootloader image: OK"
}

verify_stm32h743
verify_rp2350
