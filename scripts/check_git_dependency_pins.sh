#!/usr/bin/env bash
#
# Supply-chain gate for git dependencies.
#
# A git dependency without a rev follows whatever its branch points at:
# `cargo update` (or a fresh lockfile) silently links different code into
# the firmware. A full commit rev cannot move.
#
# Rule enforced here:
#
#   Every `git = "..."` dependency in a tracked Cargo.toml states
#   `rev = "<40-hex commit>"` on the same line, and no `branch` or `tag`.
#
# Usage:
#   scripts/check_git_dependency_pins.sh              # run from repo root
#   scripts/check_git_dependency_pins.sh --self-test  # prove the gate still bites

set -euo pipefail

MODE="${1:-live}"
if [[ "$MODE" != "live" && "$MODE" != "--self-test" ]]; then
    echo "usage: $0 [--self-test]" >&2
    exit 2
fi

# Prints one `file:line: reason` per offending dependency.
scan() {
    local file line_no line
    for file in "$@"; do
        line_no=0
        while IFS= read -r line || [[ -n "$line" ]]; do
            line_no=$((line_no + 1))
            [[ "$line" =~ ^[[:space:]]*# ]] && continue
            [[ "$line" =~ git[[:space:]]*=[[:space:]]*\" ]] || continue
            if [[ "$line" =~ (branch|tag)[[:space:]]*= ]]; then
                echo "$file:$line_no: git dependency follows a branch or tag"
            elif [[ ! "$line" =~ rev[[:space:]]*=[[:space:]]*\"[0-9a-f]{40}\" ]]; then
                echo "$file:$line_no: git dependency without a full commit rev"
            fi
        done < "$file"
    done
}

if [[ "$MODE" == "--self-test" ]]; then
    scratch="$(mktemp -d)"
    trap 'rm -rf "$scratch"' EXIT
    cat > "$scratch/Cargo.toml" <<'TOML'
a = { git = "https://example.com/a", rev = "0123456789abcdef0123456789abcdef01234567" }
b = { git = "https://example.com/b" }
c = { git = "https://example.com/c", branch = "main" }
d = { git = "https://example.com/d", rev = "0123abc" }
# e = { git = "https://example.com/e" }
TOML
    found="$(scan "$scratch/Cargo.toml")"
    expected="$scratch/Cargo.toml:2: git dependency without a full commit rev
$scratch/Cargo.toml:3: git dependency follows a branch or tag
$scratch/Cargo.toml:4: git dependency without a full commit rev"
    if [[ "$found" != "$expected" ]]; then
        echo "self-test: the gate did not flag exactly the unpinned fixtures:"
        echo "$found"
        exit 1
    fi
    echo "self-test: unpinned git dependencies are refused"
    exit 0
fi

mapfile -t manifests < <(git ls-files '*Cargo.toml' ':!:external/**')
found="$(scan "${manifests[@]}")"
if [[ -n "$found" ]]; then
    echo "$found"
    exit 1
fi
echo "git dependencies: every one is pinned to a commit rev"
