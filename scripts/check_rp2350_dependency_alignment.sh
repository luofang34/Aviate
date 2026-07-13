#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BOOTLOADER_MANIFEST="$REPO_ROOT/aviate-bootloader/Cargo.toml"
TARGET=thumbv8m.main-none-eabihf
BOOTLOADER_FEATURE=aviate-bootloader/pico2
EXPECTED_HAL_VERSION=0.4.0
EXPECTED_PAC_VERSION=0.2.0

dependency_tree() {
    local package=$1

    cargo tree \
        --manifest-path "$BOOTLOADER_MANIFEST" \
        --workspace \
        --target "$TARGET" \
        --locked \
        --features "$BOOTLOADER_FEATURE" \
        --edges normal \
        --prefix none \
        --invert "$package"
}

require_package_version() {
    local tree=$1
    local package=$2
    local expected_version=$3
    local count

    count="$(awk -v package="$package" '$1 == package { count += 1 } END { print count + 0 }' \
        <<<"$tree")"
    if [[ "$count" -ne 1 ]]; then
        printf 'FAIL: expected one %s version in the RP2350 graph, found %s\n' \
            "$package" "$count" >&2
        return 1
    fi

    if ! awk -v package="$package" -v version="v$expected_version" \
        '$1 == package && $2 == version { found = 1 } END { exit !found }' <<<"$tree"; then
        printf 'FAIL: expected %s v%s in the RP2350 graph\n' \
            "$package" "$expected_version" >&2
        return 1
    fi
}

require_owner() {
    local tree=$1
    local owner=$2

    if ! awk -v owner="$owner" '$1 == owner { found = 1 } END { exit !found }' \
        <<<"$tree"; then
        printf 'FAIL: RP2350 dependency graph is missing owner %s\n' "$owner" >&2
        return 1
    fi
}

hal_tree="$(dependency_tree rp235x-hal)"
pac_tree="$(dependency_tree rp235x-pac)"

require_package_version "$hal_tree" rp235x-hal "$EXPECTED_HAL_VERSION"
require_package_version "$pac_tree" rp235x-pac "$EXPECTED_PAC_VERSION"

for owner in aviate-bootloader aviate-chip-rp2350 test-app-rp2350; do
    require_owner "$hal_tree" "$owner"
done

require_package_version "$pac_tree" rp235x-hal "$EXPECTED_HAL_VERSION"

printf 'RP2350 dependency alignment: OK (rp235x-hal %s, rp235x-pac %s)\n' \
    "$EXPECTED_HAL_VERSION" "$EXPECTED_PAC_VERSION"
