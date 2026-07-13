#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

tree_output="$({
    cd "$REPO_ROOT/aviate-bootloader"
    cargo tree \
        --target thumbv7em-none-eabihf \
        --no-default-features \
        --features micoair-h743-v2 \
        --locked \
        --edges normal \
        --prefix none
})"

local_packages="$(
    awk -v repo_path="($REPO_ROOT/" \
        'index($0, repo_path) { print $1 }' <<<"$tree_output" | sort -u
)"

required_local=(
    aviate-bootloader
    aviate-boot-core
    aviate-chip-stm32h743
    aviate-board-micoair-h743-v2-metadata
)

for package in "${required_local[@]}"; do
    if ! awk -v package="$package" '$1 == package { found = 1 } END { exit !found }' \
        <<<"$local_packages"; then
        printf 'FAIL: bootloader dependency tree is missing %s\n' "$package" >&2
        exit 1
    fi
done

while IFS= read -r package; do
    case "$package" in
        aviate-bootloader | aviate-boot-core | aviate-chip-stm32h743 | \
            aviate-board-micoair-h743-v2-metadata) ;;
        *)
            printf 'FAIL: disallowed local crate in bootloader dependency tree: %s\n' \
                "$package" >&2
            exit 1
            ;;
    esac
done <<<"$local_packages"

flight_only=(
    embedded-alloc
    embedded-hal-bus
    bmi088
    bmi2
    spl06-007
    qmc5883l
)

for package in "${flight_only[@]}"; do
    if awk -v package="$package" '$1 == package { found = 1 } END { exit !found }' \
        <<<"$tree_output"; then
        printf 'FAIL: flight-only crate in bootloader dependency tree: %s\n' \
            "$package" >&2
        exit 1
    fi
done

metadata_tree="$({
    cd "$REPO_ROOT"
    cargo tree \
        --package aviate-board-micoair-h743-v2-metadata \
        --locked \
        --all-features \
        --target all \
        --edges normal,build \
        --prefix none
})"

if ! awk '
    NF { count += 1; unexpected = unexpected || $1 != "aviate-board-micoair-h743-v2-metadata" }
    END { exit count != 1 || unexpected }
' <<<"$metadata_tree"; then
    printf 'FAIL: MicoAir bootloader metadata crate must have no normal or build dependencies\n' >&2
    exit 1
fi

printf 'MicoAir bootloader dependency boundary: OK\n'
