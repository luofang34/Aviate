#!/usr/bin/env bash
#
# Checksum/yank-aware idempotent publish, in dependency order.
#
# For each crate, in order (kernel before facade):
#   - index lookup is FAIL-CLOSED: a network/HTTP/JSON error is an error,
#     never mistaken for "not yet published";
#   - if the version is present it is skipped ONLY when it is not yanked and
#     its recorded checksum matches the archive this commit built; a yanked
#     or mismatched entry fails the run (bump the version);
#   - if absent it is published, and after the index shows it, the entry is
#     re-verified (not yanked, checksum matches the local archive).
#
# Requires CARGO_REGISTRY_TOKEN in the environment (from OIDC auth).
# Usage: release_publish.sh <version>
#        release_publish.sh --self-test   # decision-logic unit test, no network

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

index_path() {
    local c=$1
    case ${#c} in
        1) printf '1/%s' "$c" ;;
        2) printf '2/%s' "$c" ;;
        3) printf '3/%s/%s' "${c:0:1}" "$c" ;;
        *) printf '%s/%s/%s' "${c:0:2}" "${c:2:2}" "$c" ;;
    esac
}

# Fetch the sparse-index body for a crate. Prints the body on HTTP 200,
# prints nothing on 404 (crate absent), and returns 2 on any other outcome
# (network failure, 5xx, ...). Overridable in the self-test.
index_fetch() {
    local c=$1 url resp http body
    url="https://index.crates.io/$(index_path "$c")"
    resp="$(curl -sS -m 30 -w $'\n%{http_code}' "$url" 2>/dev/null)" || {
        echo "::error::index request for ${c} failed (network/curl)" >&2
        return 2
    }
    http="${resp##*$'\n'}"
    body="${resp%$'\n'*}"
    case "$http" in
        200) printf '%s' "$body" ;;
        404) : ;;
        *) echo "::error::index request for ${c} returned HTTP ${http}" >&2; return 2 ;;
    esac
}

# Compact JSON index entry for an exact version, or empty if absent.
# Propagates index_fetch failures (return 2).
index_entry() {
    local c=$1 v=$2 body out
    body="$(index_fetch "$c")" || return 2
    [ -n "$body" ] || return 0
    out="$(jq -c --arg v "$v" 'select(.vers == $v)' <<<"$body")" || {
        echo "::error::failed to parse index JSON for ${c}" >&2
        return 2
    }
    printf '%s' "$out"
}

# sha256 of the packaged .crate. Overridable in the self-test.
local_checksum() {
    local c=$1 v=$2
    local f="target/package/${c}-${v}.crate"
    [ -f "$f" ] || { echo "::error::package archive not found: ${f}" >&2; return 2; }
    sha256sum "$f" | awk '{print $1}'
}

# Print "publish" or "skip", or fail (1 = policy violation, 2 = lookup error).
classify_version() {
    local c=$1 v=$2 entry yanked cksum local_ck
    entry="$(index_entry "$c" "$v")" || {
        echo "::error::index lookup failed for ${c} ${v}; refusing to proceed" >&2
        return 2
    }
    if [ -z "$entry" ]; then
        echo publish
        return 0
    fi
    yanked="$(jq -r '.yanked' <<<"$entry")"
    cksum="$(jq -r '.cksum' <<<"$entry")"
    local_ck="$(local_checksum "$c" "$v")" || return 2
    if [ "$yanked" = "true" ]; then
        echo "::error::${c} ${v} is yanked on crates.io; bump the version, do not re-release" >&2
        return 1
    fi
    if [ "$cksum" != "$local_ck" ]; then
        echo "::error::${c} ${v} already published with checksum ${cksum}, but the archive built from this commit is ${local_ck}; refusing to skip a mismatched artifact" >&2
        return 1
    fi
    echo skip
}

wait_visible() {
    local c=$1 v=$2
    for _ in $(seq 1 60); do
        [ -n "$(index_entry "$c" "$v")" ] && return 0
        sleep 5
    done
    echo "::error::${c} ${v} did not become visible on the index" >&2
    return 1
}

# Post-publish: the freshly-published version must be present, not yanked,
# and match the archive we uploaded.
verify_published() {
    local c=$1 v=$2 entry yanked cksum
    entry="$(index_entry "$c" "$v")" || return 2
    [ -n "$entry" ] || { echo "::error::${c} ${v} absent from index after publish" >&2; return 1; }
    yanked="$(jq -r '.yanked' <<<"$entry")"
    cksum="$(jq -r '.cksum' <<<"$entry")"
    [ "$yanked" = "false" ] || { echo "::error::${c} ${v} is yanked immediately after publish" >&2; return 1; }
    [ "$cksum" = "$(local_checksum "$c" "$v")" ] \
        || { echo "::error::${c} ${v} published checksum differs from the local archive" >&2; return 1; }
    echo "${c} ${v} verified on index (not yanked, checksum matches)"
}

publish_or_skip() {
    local c=$1 v=$2 decision
    # Build the archive now, in dependency order: aviate-core is a leaf, and
    # aviate is reached only after aviate-core is on the registry, so its
    # `=<version>` pin resolves without a patch. This is the archive the
    # checksum comparison and (for a new version) `cargo publish` will use.
    cargo package -p "$c" --locked --no-verify
    decision="$(classify_version "$c" "$v")"
    case "$decision" in
        skip)
            echo "${c} ${v} already published with matching checksum — skipping"
            ;;
        publish)
            echo "Publishing ${c} ${v}"
            cargo publish -p "$c" --locked
            wait_visible "$c" "$v"
            verify_published "$c" "$v"
            ;;
    esac
}

# --- self-test: exercise classify_version against mocked index/checksum ----
# Mocks are driven by dedicated globals to avoid colliding with the locals
# named `body`/`out` inside index_entry.
MOCK_LOCAL_CK="aaaa"
MOCK_INDEX_BODY=""
MOCK_INDEX_ERR="0"

# shellcheck disable=SC2317  # called indirectly via classify_version
mock_local_checksum() { printf '%s' "$MOCK_LOCAL_CK"; }
# shellcheck disable=SC2317
mock_index_fetch() {
    [ "$MOCK_INDEX_ERR" = "1" ] && return 2
    printf '%s' "$MOCK_INDEX_BODY"
}

self_test() {
    local fails=0
    local_checksum() { mock_local_checksum; }
    index_fetch() { mock_index_fetch; }

    _assert() { # <label> <expected: publish|skip|fail1|fail2>
        local label=$1 expected=$2 out rc got
        set +e
        out="$(classify_version testcrate 1.2.3)"; rc=$?
        set -e
        case "$rc" in
            0) got="$out" ;;
            1) got="fail1" ;;
            *) got="fail2" ;;
        esac
        if [ "$got" = "$expected" ]; then
            echo "  ok: ${label} -> ${got}"
        else
            echo "  FAIL: ${label} expected ${expected}, got ${got}" >&2
            fails=1
        fi
    }

    MOCK_INDEX_ERR=0
    MOCK_INDEX_BODY=""
    _assert "absent" publish

    MOCK_INDEX_BODY="$(jq -cn --arg ck "$MOCK_LOCAL_CK" '{vers:"1.2.3",cksum:$ck,yanked:false}')"
    _assert "present-match" skip

    MOCK_INDEX_BODY="$(jq -cn '{vers:"1.2.3",cksum:"different",yanked:false}')"
    _assert "present-mismatch" fail1

    MOCK_INDEX_BODY="$(jq -cn --arg ck "$MOCK_LOCAL_CK" '{vers:"1.2.3",cksum:$ck,yanked:true}')"
    _assert "present-yanked" fail1

    MOCK_INDEX_BODY=""
    MOCK_INDEX_ERR=1
    _assert "index-error" fail2

    if [ "$fails" -ne 0 ]; then
        echo "release_publish self-test: FAILED" >&2
        return 1
    fi
    echo "release_publish self-test: OK"
}

if [ "${1:-}" = "--self-test" ]; then
    self_test
    exit $?
fi

VERSION="${1:?usage: release_publish.sh <version> | --self-test}"
publish_or_skip aviate-core "$VERSION"
publish_or_skip aviate "$VERSION"
echo "Release v${VERSION} publish step complete."
