#!/usr/bin/env bash
#
# Each published crate's package must contain the dual-license texts and a
# README with the corrected safety wording (and not the blanket real-hardware
# ban). Run before OIDC auth and also in CI so a regression cannot surface
# only at tag time.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

for crate in aviate-core aviate; do
    list="$(cargo package --list -p "$crate")"
    for f in LICENSE-MIT LICENSE-APACHE README.md; do
        grep -qx "$f" <<<"$list" \
            || { echo "::error::${crate} package is missing ${f}" >&2; exit 1; }
    done
    crate_dir="$(cargo metadata --no-deps --locked --format-version 1 \
        | jq -r --arg n "$crate" '.packages[] | select(.name==$n) | .manifest_path' \
        | xargs dirname)"
    grep -q "control a real vehicle" "$crate_dir/README.md" \
        || { echo "::error::${crate} README missing corrected safety wording" >&2; exit 1; }
    ! grep -q "Do not deploy it on real hardware" "$crate_dir/README.md" \
        || { echo "::error::${crate} README still carries the blanket real-hardware ban" >&2; exit 1; }
done
echo "package contents OK (license texts + corrected wording present)"
