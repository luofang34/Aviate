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

require_absent_section() {
    local elf=$1
    local section=$2

    if [[ -n "$(section_fields "$elf" "$section")" ]]; then
        printf 'FAIL: %s must not contain %s\n' "$elf" "$section" >&2
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

# Check [start, start + size) without computing the potentially overflowing
# end address. The caller supplies decimal values parsed from trusted ELF
# metadata or repository constants.
require_range_within() {
    local label=$1
    local start=$2
    local size=$3
    local range_start=$4
    local range_end=$5

    if (( start < range_start )); then
        printf 'FAIL: %s starts at 0x%08x, below 0x%08x\n' \
            "$label" "$start" "$range_start" >&2
        return 1
    fi
    if (( start >= range_end )); then
        printf 'FAIL: %s starts at 0x%08x, outside [0x%08x, 0x%08x)\n' \
            "$label" "$start" "$range_start" "$range_end" >&2
        return 1
    fi
    if (( size > range_end - start )); then
        printf 'FAIL: %s at 0x%08x with size 0x%x exceeds 0x%08x\n' \
            "$label" "$start" "$size" "$range_end" >&2
        return 1
    fi
}

# Assert every nonempty flash-resident LOAD segment stays within the given
# range. A segment's flash footprint is its physical (load) address range;
# using the load address rather than the section VMA catches copy-to-RAM data
# whose runtime address is in RAM but whose initializer occupies flash.
require_flash_loads_in_range() {
    local elf=$1
    local range_start=$2
    local range_end=$3
    local ram_base=$4
    local segments
    local phys filesz p size
    local flash_loads=0
    local violations=0

    if ! segments="$("$READELF" --segments --wide "$elf" | awk \
        '$1 == "LOAD" { print $4, $5 }')"; then
        echo "FAIL: could not read LOAD segments from $elf" >&2
        return 1
    fi

    while read -r phys filesz; do
        [[ -z "$phys" ]] && continue
        p=$((phys))
        size=$((filesz))
        (( size == 0 )) && continue
        (( p >= ram_base )) && continue
        flash_loads=$((flash_loads + 1))
        require_range_within "$elf LOAD segment" "$p" "$size" \
            "$range_start" "$range_end" || violations=1
    done <<<"$segments"

    if (( flash_loads == 0 )); then
        echo "FAIL: $elf has no nonempty flash-resident LOAD segments" >&2
        return 1
    fi

    return $violations
}

# Require all nonempty allocated executable sections to lie in the application
# partition. This binds the proof to the code mapping rather than merely to the
# presence of a section named .text.
require_executable_sections_in_range() {
    local elf=$1
    local range_start=$2
    local range_end=$3
    local sections
    local name address size
    local address_value size_value
    local executable_sections=0
    local violations=0

    if ! sections="$("$READELF" --sections --wide "$elf" | awk '
        $1 == "[" {
            name = $3
            address = $5
            size = $7
            flags = $9
            if (flags ~ /A/ && flags ~ /X/) {
                print name, address, size
            }
            next
        }
        $1 ~ /^\[[0-9]+\]$/ {
            name = $2
            address = $4
            size = $6
            flags = $8
            if (flags ~ /A/ && flags ~ /X/) {
                print name, address, size
            }
        }
    ')"; then
        echo "FAIL: could not read executable sections from $elf" >&2
        return 1
    fi

    while read -r name address size; do
        [[ -z "$name" ]] && continue
        address_value=$((16#$address))
        size_value=$((16#$size))
        (( size_value == 0 )) && continue
        executable_sections=$((executable_sections + 1))
        require_range_within "$elf executable section $name" \
            "$address_value" "$size_value" "$range_start" "$range_end" || violations=1
    done <<<"$sections"

    if (( executable_sections == 0 )); then
        echo "FAIL: $elf has no nonempty allocated executable sections" >&2
        return 1
    fi

    return $violations
}

# Read a linker/global symbol's value (hex, no 0x prefix) from the ELF
# symbol table.
symbol_value() {
    local elf=$1
    local sym=$2

    "$READELF" --syms --wide "$elf" | awk -v s="$sym" '$8 == s { print $2; exit }'
}

# Assert the linker's FLASH region end stays within the boundary. This
# checks the actual linker geometry, not the current image: build.rs
# exports `__aviate_bootloader_flash_region_end = ORIGIN(FLASH) +
# LENGTH(FLASH)`, so a region sized past the application start fails here
# even when the (small) image footprint still fits below the boundary.
require_region_end_within() {
    local elf=$1
    local boundary=$2
    local sym=__aviate_bootloader_flash_region_end
    local value

    value="$(symbol_value "$elf" "$sym")"
    if [[ -z "$value" ]]; then
        echo "FAIL: $elf is missing linker symbol $sym (build.rs must export the FLASH region end)" >&2
        return 1
    fi
    if (( 16#$value > boundary )); then
        printf 'FAIL: %s FLASH region ends at 0x%08x, past application boundary 0x%08x\n' \
            "$elf" "$((16#$value))" "$boundary" >&2
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

# Fixed RP2350 flash geometry (Boot ROM contract — not tunable).
readonly RP2350_FLASH_BASE=0x10000000
readonly RP2350_RAM_BASE=0x20000000
readonly RP2350_START_BLOCK=0x10000000
readonly RP2350_VECTOR_TABLE=0x10000100
# Top of the XIP flash window (16 MiB max) — upper bound for the
# application entry-point range.
readonly RP2350_FLASH_TOP=0x11000000

# The application boundary comes from the backend handoff constant. The
# test-app origin must equal it, while the bootloader linker region is
# independently required to end at or below it.
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
# lowered to the vector-table start. The real bootloader's .text and
# .rodata load above the vector table, so a correct guard MUST reject it.
# If this lowered check passes, the guard is broken and cannot be trusted
# to catch a real crossing — fail loudly.
assert_guard_rejects_crossing() {
    local elf=$1
    local decoy_boundary=$RP2350_VECTOR_TABLE

    if require_flash_loads_in_range "$elf" "$RP2350_FLASH_BASE" \
        "$decoy_boundary" "$RP2350_RAM_BASE" 2>/dev/null; then
        printf 'FAIL: partition guard accepted an image crossing 0x%08x — guard is not effective\n' \
            "$decoy_boundary" >&2
        return 1
    fi
    printf 'RP2350 image-footprint self-test: OK (rejects a crossing at 0x%08x)\n' \
        "$decoy_boundary"
}

# The real ELF proves the parser rejects a LOAD below the lower bound. An
# isolated two-byte range binds the upper self-test specifically to the
# overflow-safe size predicate, so another LOAD cannot make it pass by accident.
assert_test_app_range_guard_rejects_mutations() {
    local elf=$1
    local boundary=$2
    local decoy_start=$((boundary + 1))
    local decoy_end=$((boundary + 1))

    if require_flash_loads_in_range "$elf" "$decoy_start" \
        "$RP2350_FLASH_TOP" "$RP2350_RAM_BASE" 2>/dev/null; then
        printf 'FAIL: test-app guard accepted a LOAD below 0x%08x\n' \
            "$decoy_start" >&2
        return 1
    fi
    printf 'RP2350 test-app lower-bound self-test: OK (rejects a LOAD below 0x%08x)\n' \
        "$decoy_start"

    if require_range_within "$elf oversize LOAD mutation" \
        "$boundary" 2 "$boundary" "$decoy_end" 2>/dev/null; then
        printf 'FAIL: range guard accepted a LOAD exceeding 0x%08x\n' \
            "$decoy_end" >&2
        return 1
    fi
    printf 'RP2350 range upper-bound self-test: OK (rejects a LOAD exceeding 0x%08x)\n' \
        "$decoy_end"
}

# Independent negative proof for the region-end guard: re-run it with the
# boundary lowered below the real FLASH region end. The linker region
# genuinely extends past that decoy, so a correct guard MUST reject it.
# This proves the region-capacity check binds to the boundary, distinct
# from the image-footprint proof above.
assert_region_guard_rejects_oversize() {
    local elf=$1
    local decoy_boundary=$RP2350_VECTOR_TABLE

    if require_region_end_within "$elf" "$decoy_boundary" 2>/dev/null; then
        printf 'FAIL: region guard accepted a FLASH region past 0x%08x — guard is not effective\n' \
            "$decoy_boundary" >&2
        return 1
    fi
    printf 'RP2350 region-capacity self-test: OK (rejects a region past 0x%08x)\n' \
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
    require_flash_loads_in_range "$elf" "$RP2350_FLASH_BASE" "$boundary" "$RP2350_RAM_BASE"
    require_region_end_within "$elf" "$boundary"
    assert_guard_rejects_crossing "$elf"
    assert_region_guard_rejects_oversize "$elf"

    printf 'RP2350 bootloader image: OK (image footprint and linker region end at or below 0x%08x)\n' \
        "$boundary"
}

verify_test_app_rp2350() {
    local elf="$REPO_ROOT/aviate-bootloader/target/thumbv8m.main-none-eabihf/release/test-app-rp2350"
    local boundary

    [[ -f "$elf" ]] || {
        echo "FAIL: RP2350 test-app ELF not found: $elf" >&2
        return 1
    }

    boundary="$(rp2350_boundary)" || return 1

    # The application vector table sits exactly at the partition boundary,
    # where the bootloader hands off. The entry must be nonzero and inside
    # the application region, .text must be present, and there must be no
    # .start_block — the Boot ROM validates only the bootloader image.
    require_section "$elf" .vector_table "$boundary"
    require_nonempty_section "$elf" .text
    require_entry_in_range "$elf" "$boundary" "$RP2350_FLASH_TOP"
    require_absent_section "$elf" .start_block
    require_flash_loads_in_range "$elf" "$boundary" "$RP2350_FLASH_TOP" "$RP2350_RAM_BASE"
    require_executable_sections_in_range "$elf" "$boundary" "$RP2350_FLASH_TOP"
    assert_test_app_range_guard_rejects_mutations "$elf" "$boundary"

    printf 'RP2350 test-app image: OK (vector exactly at 0x%08x; entry in [0x%08x, 0x%08x))\n' \
        "$boundary" "$boundary" "$RP2350_FLASH_TOP"
}

verify_stm32h743
verify_rp2350
verify_test_app_rp2350
