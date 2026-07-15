#!/usr/bin/env bash
#
# Every committed Cargo.lock must be up to date under --locked. Standalone
# crates (e.g. the MicoAir board) path-depend on the external/ submodules,
# so the CALLER must check out submodules (submodules: recursive) or these
# checks fail with "No such file or directory".

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

fails=0
while IFS= read -r lock; do
    dir="$(dirname "$lock")"
    echo "checking $dir"
    cargo tree --locked --manifest-path "$dir/Cargo.toml" >/dev/null \
        || { echo "::error::${lock} is not up to date (--locked failed)"; fails=1; }
done < <(git ls-files 'Cargo.lock' '**/Cargo.lock')

[ "$fails" -eq 0 ] && echo "all committed lockfiles are up to date (--locked)"
exit "$fails"
