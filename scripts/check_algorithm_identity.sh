#!/usr/bin/env bash
#
# Identity adjudication for production algorithm implementations.
#
# A change to a production estimator / controller / mixer / sanitizer
# implementation path must be adjudicated by a human, never inferred:
# either the change rotates an identity (the diff touches
# cert/algorithm_id_registry.toml), or the author states that behavior
# is unchanged with an explicit commit trailer carrying a rationale:
#
#   Algorithm-Identity-Unchanged: <why this cannot change observable behavior>
#
# Usage:
#   scripts/check_algorithm_identity.sh <base-ref> <head-ref>
#   scripts/check_algorithm_identity.sh --self-test
#
# Exit codes: 0 adjudicated (or no production path touched), 1 a
# production path changed without registry change or rationale, 2 bad
# invocation.

set -euo pipefail

# Production implementation roots. Test modules under these roots do
# not change flight behavior and are excluded.
PROD_PATTERNS=(
    'aviate-core/src/ekf.rs'
    'aviate-core/src/ekf/'
    'aviate-core/src/control/multirotor.rs'
    'aviate-core/src/control/multirotor/'
    'aviate-core/src/mixer.rs'
    'aviate-core/src/mixer/'
)
REGISTRY='cert/algorithm_id_registry.toml'
TRAILER='Algorithm-Identity-Unchanged:'

# adjudicate <changed-files (newline list)> <trailer-lines (newline list)>
# Prints the verdict; returns 0/1.
adjudicate() {
    local changed="$1" trailers="$2"
    local touched=()

    while IFS= read -r file; do
        [[ -z "$file" ]] && continue
        case "$file" in
            *tests.rs | */tests/* ) continue ;;
        esac
        for pattern in "${PROD_PATTERNS[@]}"; do
            if [[ "$file" == "$pattern"* ]]; then
                touched+=("$file")
                break
            fi
        done
    done <<< "$changed"

    if [[ ${#touched[@]} -eq 0 ]]; then
        echo "No production algorithm path changed; no adjudication needed."
        return 0
    fi

    if grep -qxF "$REGISTRY" <<< "$changed"; then
        echo "Identity adjudicated: registry changed alongside production paths."
        return 0
    fi

    local rationale
    rationale="$(grep -F "$TRAILER" <<< "$trailers" | sed "s/.*$TRAILER//" | tr -d ' \t' | head -1 || true)"
    if [[ -n "$rationale" ]]; then
        echo "Identity adjudicated: explicit $TRAILER rationale present."
        return 0
    fi

    echo "FAIL: production algorithm paths changed without adjudication:" >&2
    printf '  %s\n' "${touched[@]}" >&2
    echo "Either rotate the identity ($REGISTRY) or add a commit trailer:" >&2
    echo "  $TRAILER <why this cannot change observable behavior>" >&2
    return 1
}

self_test() {
    local failures=0

    # A production change with neither registry nor rationale fails.
    if adjudicate 'aviate-core/src/ekf/update.rs' '' > /dev/null 2>&1; then
        echo "SELF-TEST FAIL: unadjudicated production change passed" >&2
        failures=1
    fi

    # The same change with a registry rotation passes.
    adjudicate $'aviate-core/src/ekf/update.rs\ncert/algorithm_id_registry.toml' '' > /dev/null || {
        echo "SELF-TEST FAIL: registry rotation rejected" >&2
        failures=1
    }

    # The same change with an explicit rationale trailer passes.
    adjudicate 'aviate-core/src/ekf/update.rs' \
        "$TRAILER comment-only reordering, no executable change" > /dev/null || {
        echo "SELF-TEST FAIL: explicit rationale rejected" >&2
        failures=1
    }

    # A bare trailer with no rationale text does not count.
    if adjudicate 'aviate-core/src/ekf/update.rs' "$TRAILER" > /dev/null 2>&1; then
        echo "SELF-TEST FAIL: empty rationale accepted" >&2
        failures=1
    fi

    # Test-module and non-production changes need no adjudication.
    adjudicate $'aviate-core/src/ekf/tests.rs\naviate-link/src/queue.rs' '' > /dev/null || {
        echo "SELF-TEST FAIL: non-production change rejected" >&2
        failures=1
    }

    if [[ $failures -ne 0 ]]; then
        return 1
    fi
    echo "Identity-adjudication self-test: OK"
}

case "${1:-}" in
    --self-test)
        self_test
        ;;
    "")
        echo "usage: $0 <base-ref> <head-ref> | --self-test" >&2
        exit 2
        ;;
    *)
        base="$1"
        head="${2:?usage: $0 <base-ref> <head-ref>}"
        changed="$(git diff --name-only "$base".."$head")"
        trailers="$(git log --format=%B "$base".."$head" | grep -F "$TRAILER" || true)"
        adjudicate "$changed" "$trailers"
        ;;
esac
