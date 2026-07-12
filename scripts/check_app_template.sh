#!/usr/bin/env bash
#
# Template-generation compile guard.
#
# Generates the SITL variant of aviate-app-template and type-checks it
# against the current tree, so a board/kernel API change that breaks
# the template fails CI instead of the next user's first build. The
# generated crate inherits workspace fields, so it is temporarily added
# to the workspace members for the check and removed afterwards.
#
# Requires cargo-generate. Exit codes: 0 template compiles, 1 it does
# not, 2 tooling missing.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

APP=template-check-app
APP_DIR="aviate-apps/$APP"

command -v cargo-generate > /dev/null 2>&1 || {
    echo "cargo-generate is required: cargo install cargo-generate --locked" >&2
    exit 2
}

cleanup() {
    git checkout --quiet -- Cargo.toml 2>/dev/null || true
    rm -rf "$APP_DIR"
}
trap cleanup EXIT

rm -rf "$APP_DIR"
cargo generate --path aviate-app-template --name "$APP" --destination aviate-apps \
    -d board=sitl-gazebo -d model=x500 -d airframe=x500 -d env=sitl

python3 - <<PY
s = open('Cargo.toml').read()
marker = '    "aviate-apps/sitl-gazebo-x500",'
assert marker in s
s = s.replace(marker, marker + '\n    "$APP_DIR",', 1)
open('Cargo.toml', 'w').write(s)
PY

cargo check -p "aviate-app-$APP"
echo "App-template compile guard: OK"
