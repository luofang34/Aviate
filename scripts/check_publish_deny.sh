#!/usr/bin/env bash
#
# Fail if any first-party crate other than `aviate` or `aviate-core` is
# publishable to crates.io. Inventories EVERY first-party manifest, not just
# the root workspace members, so an excluded/standalone crate (bootloader,
# board, chip, HAL, test app) cannot silently become publishable.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# The only crates permitted to publish to crates.io.
ALLOWED=" aviate aviate-core "

violations=0
checked=0

while IFS= read -r manifest; do
    # A crate manifest has a [package] table; a bare [workspace] root does not.
    grep -q '^\[package\]' "$manifest" || continue

    name="$(sed -n 's/^name[[:space:]]*=[[:space:]]*"\([^"]*\)".*/\1/p' "$manifest" | head -1)"
    [ -n "$name" ] || {
        echo "FAIL: $manifest has [package] but no name" >&2
        violations=1
        continue
    }
    checked=$((checked + 1))

    # `publish = false` denies crates.io; anything else is publishable.
    if grep -Eq '^[[:space:]]*publish[[:space:]]*=[[:space:]]*false' "$manifest"; then
        continue
    fi

    case "$ALLOWED" in
        *" $name "*) ;; # explicitly allowed
        *)
            printf 'FAIL: %s (%s) is publishable but not in the allowed set {aviate, aviate-core}; add publish = false\n' \
                "$name" "$manifest" >&2
            violations=1
            ;;
    esac
done < <(find . -name Cargo.toml -not -path './external/*' -not -path '*/target/*')

if [ "$violations" -ne 0 ]; then
    exit 1
fi
printf 'publish-deny: OK (%d first-party manifests inventoried; only aviate + aviate-core publishable)\n' "$checked"
