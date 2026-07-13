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

# Parse a single hex constant (e.g. 0x1001_0000) out of a source file so
# the partition boundary has one authoritative definition instead of a
# copy per consumer. Underscores are stripped; failure to parse is fatal.
parse_hex_constant() {
    local file=$1
    local regex=$2
    local line
    local value

    line="$(grep -E "$regex" "$file" 2>/dev/null | head -1)"
    if [[ -z "$line" ]]; then
        echo "FAIL: no line matching /$regex/ in $file" >&2
        return 1
    fi

    value="$(printf '%s' "$line" | sed -E 's/.*(0x[0-9A-Fa-f_]+).*/\1/' | tr -d '_')"
    if [[ ! "$value" =~ ^0x[0-9A-Fa-f]+$ ]]; then
        echo "FAIL: could not parse a hex constant from '$line' in $file" >&2
        return 1
    fi
    printf '%d\n' "$((value))"
}

# Assert every flash-resident LOAD segment stays within
# [flash_base, boundary). A segment's flash footprint is its physical
# (load) address range [PhysAddr, PhysAddr + FileSiz); using the load
# address rather than the section VMA catches copy-to-RAM data whose
# runtime address is in RAM but whose initializer occupies flash below
# the application region. Segments loaded into RAM (PhysAddr >= ram_base)
# consume no flash and are skipped.
require_load_footprint_below() {
    local elf=$1
    local flash_base=$2
    local boundary=$3
    local ram_base=$4
    local phys filesz p end
    local violations=0

    while read -r phys filesz; do
        [[ -z "$phys" ]] && continue
        p=$((phys))
        (( p >= ram_base )) && continue
        end=$((p + filesz))
        if (( p < flash_base )); then
            printf 'FAIL: %s LOAD segment at 0x%08x is below flash base 0x%08x\n' \
                "$elf" "$p" "$flash_base" >&2
            violations=1
        fi
        if (( end > boundary )); then
            printf 'FAIL: %s LOAD segment [0x%08x, 0x%08x) crosses application boundary 0x%08x\n' \
                "$elf" "$p" "$end" "$boundary" >&2
            violations=1
        fi
    done < <("$READELF" --segments --wide "$elf" | awk '$1 == "LOAD" { print $4, $5 }')

    return $violations
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

# Fixed RP2350 flash geometry (Boot ROM contract — not tunable).
readonly RP2350_FLASH_BASE=0x10000000
readonly RP2350_RAM_BASE=0x20000000
readonly RP2350_START_BLOCK=0x10000000
readonly RP2350_VECTOR_TABLE=0x10000100

# The bootloader/application partition boundary, read from its single
# source of truth (the backend handoff constant). The test-app origin,
# and therefore the linker geometry the ELF is built with, must agree.
rp2350_boundary() {
    parse_hex_constant \
        "$REPO_ROOT/aviate-chips/rp2350/src/lib.rs" \
        'pub const APP_START: *u32 *= *0x'
}

require_rp2350_boundary_agreement() {
    local boundary=$1
    local testapp_origin

    testapp_origin="$(parse_hex_constant \
        "$REPO_ROOT/aviate-bootloader/test-app-rp2350/build.rs" \
        'FLASH *: *ORIGIN *= *0x')" || return 1

    if (( testapp_origin != boundary )); then
        printf 'FAIL: test-app origin 0x%08x disagrees with APP_START 0x%08x\n' \
            "$testapp_origin" "$boundary" >&2
        return 1
    fi
}

# Negative proof (mutation): re-run the footprint guard with the boundary
# lowered to just past the vector table. The real bootloader's .text and
# .rodata sit above that, so a correct guard MUST reject it. If this
# lowered check passes, the guard is broken and cannot be trusted to
# catch a real crossing — fail loudly.
assert_guard_rejects_crossing() {
    local elf=$1
    local decoy_boundary=$RP2350_VECTOR_TABLE

    if require_load_footprint_below "$elf" "$RP2350_FLASH_BASE" \
        "$decoy_boundary" "$RP2350_RAM_BASE" 2>/dev/null; then
        printf 'FAIL: partition guard accepted an image crossing 0x%08x — guard is not effective\n' \
            "$decoy_boundary" >&2
        return 1
    fi
    printf 'RP2350 partition guard self-test: OK (rejects a crossing at 0x%08x)\n' \
        "$decoy_boundary"
}

verify_rp2350() {
    local elf="$REPO_ROOT/aviate-bootloader/target/thumbv8m.main-none-eabihf/release/aviate-bootloader"
    local boundary

    [[ -f "$elf" ]] || {
        echo "FAIL: RP2350 bootloader ELF not found: $elf" >&2
        return 1
    }

    boundary="$(rp2350_boundary)" || return 1
    require_rp2350_boundary_agreement "$boundary" || return 1

    require_entry_in_range "$elf" "$RP2350_FLASH_BASE" "$boundary"
    require_section "$elf" .start_block "$RP2350_START_BLOCK"
    require_section "$elf" .vector_table "$RP2350_VECTOR_TABLE"
    require_nonempty_section "$elf" .text
    require_load_footprint_below "$elf" "$RP2350_FLASH_BASE" "$boundary" "$RP2350_RAM_BASE"
    assert_guard_rejects_crossing "$elf"

    printf 'RP2350 bootloader image: OK (all flash below 0x%08x)\n' "$boundary"
}

verify_stm32h743
verify_rp2350
