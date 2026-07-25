#!/usr/bin/env bash
#
# Supply-chain gate for GitHub Actions references.
#
# Every third-party action a workflow runs executes with the workflow's
# permissions — in release.yml that includes the OIDC identity used to
# publish to crates.io. A tag is a mutable pointer: whoever controls the
# action's repository can move `v7`, or `v7.0.0`, to different code
# without the reference in this repo changing. A commit SHA cannot be
# moved, so it is the only reference that pins what actually runs.
#
# Rule enforced here:
#
#   Every `uses:` in .github/workflows/*.yml names either a 40-character
#   commit SHA carrying a `# <version>` comment, or an action on the
#   ALLOWLIST below.
#
# The version comment is required, not decorative: it is what makes a
# bare SHA reviewable, and it is what Dependabot rewrites when it bumps
# the pin. A SHA with no comment is rejected.
#
# Local composite actions (`uses: ./...`) are in-repo and reviewed as
# part of this repository, so they are out of scope.
#
# Exit codes:
#   0 — every reference is pinned (or allowlisted).
#   1 — at least one reference is a mutable ref, an unlabelled SHA, or
#       has no ref at all.
#
# Usage:
#   scripts/check_action_pins.sh              # run from repo root
#   scripts/check_action_pins.sh --self-test  # prove the gate still bites

set -euo pipefail

MODE="${1:-live}"
if [[ "$MODE" != "live" && "$MODE" != "--self-test" ]]; then
    echo "usage: $0 [--self-test]" >&2
    exit 2
fi

# Actions that intentionally use a mutable ref. Pairs: `action|reason`.
#
# Adding an entry weakens the gate, so each one has to earn its place by
# naming a property that a SHA pin would actually break.
ALLOWLIST=(
    "dtolnay/rust-toolchain|The ref names the Rust toolchain to install (a channel such as stable, or an exact version) — the action reads it from the ref itself, so a SHA pin silently changes which compiler CI uses. Instances that do pin a SHA must pass the channel through the 'toolchain:' input instead."
)

is_allowlisted() {
    local action="$1" entry
    for entry in "${ALLOWLIST[@]}"; do
        [[ "$action" == "${entry%%|*}" ]] && return 0
    done
    return 1
}

# Prints one `file:line: reason` per offending reference; prints nothing
# when the tree is clean. Both the live run and the self-test go through
# this same function, so a fixture can only pass by satisfying the rule
# the live tree is held to.
scan_workflows() {
    local dir="$1"
    local file lineno text token action ref comment

    while IFS= read -r file; do
        lineno=0
        while IFS= read -r text; do
            lineno=$((lineno + 1))

            [[ "$text" =~ ^[[:space:]]*(-[[:space:]]+)?uses:[[:space:]]*(.*)$ ]] || continue
            token="${BASH_REMATCH[2]}"

            # Split the reference from any trailing comment.
            comment=""
            if [[ "$token" == *"#"* ]]; then
                comment="${token#*#}"
                token="${token%%#*}"
            fi
            token="${token%"${token##*[![:space:]]}"}"
            token="${token#[\"\']}"
            token="${token%[\"\']}"

            [[ -z "$token" || "$token" == ./* ]] && continue

            if [[ "$token" != *"@"* ]]; then
                echo "$file:$lineno: '$token' names no ref; pin a commit SHA"
                continue
            fi

            action="${token%@*}"
            ref="${token##*@}"

            is_allowlisted "$action" && continue

            if [[ ! "$ref" =~ ^[0-9a-f]{40}$ ]]; then
                echo "$file:$lineno: '$action@$ref' is a mutable ref; pin the commit SHA it resolves to and label it '# $ref'"
            elif [[ -z "${comment//[[:space:]]/}" ]]; then
                echo "$file:$lineno: '$action' is pinned but unlabelled; append '# <version>' so the pin is reviewable"
            fi
        done <"$file"
    done < <(find "$dir" -maxdepth 1 \( -name '*.yml' -o -name '*.yaml' \) -type f | sort)
}

# --- Self-test -----------------------------------------------------------
#
# Feeds one fixture per rejection reason, plus the shapes that must be
# accepted, through scan_workflows.
#
# Each rejection assertion matches the *reason* the fixture was refused,
# not merely that some line was printed. Asserting "output is non-empty"
# would let a mutable-ref fixture pass this test while being caught for
# an unrelated reason — the rule under test could then be deleted with
# the self-test still reporting success.

self_test() {
    local tmp
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' RETURN

    local sha="9c091bb21b7c1c1d1991bb908d89e4e9dddfe3e0"
    local failures=0

    # `emit <uses-value> [extension]` — the extension defaults to yml, and
    # is a parameter so a fixture can prove `.yaml` is scanned too.
    emit() {
        printf 'jobs:\n  j:\n    steps:\n      - uses: %s\n' "$1" >"$tmp/fixture.${2:-yml}"
    }

    # `expect_caught <expected-reason> <description>`
    expect_caught() {
        local out
        out="$(scan_workflows "$tmp")"
        if [[ -z "$out" ]]; then
            echo "SELF-TEST FAILED: $2 was accepted but must be rejected" >&2
            failures=$((failures + 1))
        elif [[ "$out" != *"$1"* ]]; then
            echo "SELF-TEST FAILED: $2 was rejected for the wrong reason;" >&2
            echo "  expected to contain: $1" >&2
            echo "  got:                 $out" >&2
            failures=$((failures + 1))
        fi
        rm -f "$tmp"/*.yml "$tmp"/*.yaml
    }

    # `expect_clean <description>`
    expect_clean() {
        local out
        out="$(scan_workflows "$tmp")"
        if [[ -n "$out" ]]; then
            echo "SELF-TEST FAILED: $1 was rejected but must be accepted:" >&2
            echo "$out" >&2
            failures=$((failures + 1))
        fi
        rm -f "$tmp"/*.yml "$tmp"/*.yaml
    }

    emit "actions/checkout@v7"
    expect_caught "is a mutable ref" "a floating major tag (actions/checkout@v7)"

    emit "actions/checkout@v7.0.0"
    expect_caught "is a mutable ref" "an exact but still-mutable version tag (actions/checkout@v7.0.0)"

    emit "actions/checkout@main"
    expect_caught "is a mutable ref" "a branch ref (actions/checkout@main)"

    emit "actions/checkout@$sha"
    expect_caught "pinned but unlabelled" "a SHA pin with no version comment"

    emit "actions/checkout"
    expect_caught "names no ref" "a reference with no ref at all"

    emit "actions/checkout@${sha:0:7} # v7.0.0"
    expect_caught "is a mutable ref" "an abbreviated SHA"

    # `.yaml` is as valid a workflow extension as `.yml`. A scan that
    # covered only one would leave any workflow named the other way
    # entirely unenforced, while still reporting a clean tree.
    emit "actions/checkout@v7" yaml
    expect_caught "is a mutable ref" "a floating tag in a .yaml workflow"

    emit "actions/checkout@$sha # v7.0.0"
    expect_clean "a labelled SHA pin"

    emit "actions/checkout@$sha # v7.0.0" yaml
    expect_clean "a labelled SHA pin in a .yaml workflow"

    emit "\"actions/checkout@$sha\" # v7.0.0"
    expect_clean "a quoted but correctly pinned ref"

    emit "dtolnay/rust-toolchain@stable"
    expect_clean "an allowlisted toolchain-channel ref"

    printf '# local composite action\njobs:\n  j:\n    steps:\n      - uses: ./.github/actions/thing\n' >"$tmp/fixture.yml"
    expect_clean "a local composite action"

    if ((failures > 0)); then
        echo "check_action_pins.sh self-test: $failures assertion(s) failed" >&2
        exit 1
    fi
    echo "check_action_pins.sh self-test: all assertions held"
}

if [[ "$MODE" == "--self-test" ]]; then
    self_test
    exit 0
fi

WORKFLOWS=".github/workflows"
if [[ ! -d "$WORKFLOWS" ]]; then
    echo "$WORKFLOWS not found; run from the repository root" >&2
    exit 2
fi

violations="$(scan_workflows "$WORKFLOWS")"
if [[ -n "$violations" ]]; then
    echo "Unpinned GitHub Actions references:" >&2
    echo "$violations" >&2
    cat >&2 <<'EOF'

Resolve the tag to its commit and pin it, keeping the version as a comment:

    - uses: actions/checkout@9c091bb21b7c1c1d1991bb908d89e4e9dddfe3e0 # v7.0.0

The SHA for a tag is:

    curl -s https://api.github.com/repos/<owner>/<repo>/git/ref/tags/<tag>
EOF
    exit 1
fi

echo "All GitHub Actions references are pinned to a commit SHA."
