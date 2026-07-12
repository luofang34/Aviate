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

# Never destroy state this run did not create: refuse a pre-existing
# generation directory, and restore the workspace manifest from a
# byte-precise backup instead of `git checkout`, which would discard a
# user's uncommitted Cargo.toml edits.
if [ -e "$APP_DIR" ]; then
    echo "refusing to overwrite existing $APP_DIR; remove it first" >&2
    exit 2
fi

MANIFEST_BACKUP="$(mktemp)"
cp Cargo.toml "$MANIFEST_BACKUP"

cleanup() {
    cp "$MANIFEST_BACKUP" Cargo.toml
    rm -f "$MANIFEST_BACKUP"
    rm -rf "$APP_DIR"
}
trap cleanup EXIT
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
