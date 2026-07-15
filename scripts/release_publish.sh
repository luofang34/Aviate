#!/usr/bin/env bash
#
# Checksum/yank-aware idempotent publish, in dependency order (kernel first).
#
#   - index lookup is FAIL-CLOSED: a network/HTTP/JSON error is an error,
#     never mistaken for "not yet published";
#   - a present version is skipped ONLY when it is not yanked and its recorded
#     checksum matches the archive this commit built; a yanked/mismatched
#     entry fails the run (bump the version);
#   - after publishing, the index entry is re-verified (not yanked, checksum
#     matches the local archive);
#   - for the facade, the archive published after aviate-core is on the
#     registry is checked to be source-equivalent to the preflight archive
#     (excluding Cargo.lock) and to pin aviate-core to the published core's
#     checksum in its Cargo.lock.
#
# Requires CARGO_REGISTRY_TOKEN in the environment (from OIDC auth).
# Usage: release_publish.sh <version>
#        release_publish.sh --self-test   # decision/verify logic, no network

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# wait_visible pacing (overridden fast in the self-test).
WAIT_ATTEMPTS="${WAIT_ATTEMPTS:-60}"
WAIT_SLEEP="${WAIT_SLEEP:-5}"

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
# nothing on 404 (absent), returns 2 on any other outcome. Overridable.
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

# sha256 of the packaged .crate. Overridable.
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
    local c=$1 v=$2 i
    for ((i = 0; i < WAIT_ATTEMPTS; i++)); do
        [ -n "$(index_entry "$c" "$v")" ] && return 0
        sleep "$WAIT_SLEEP"
    done
    echo "::error::${c} ${v} did not become visible on the index" >&2
    return 1
}

# Post-publish: present, not yanked, checksum matches the local archive.
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

# Extract the checksum the facade's embedded Cargo.lock records for aviate-core.
# The [[package]] reset confines the match to aviate-core's own block: a path
# crate (no checksum) yields nothing rather than the next package's checksum.
facade_lock_core_checksum() {
    local archive=$1 version=$2
    tar -xzOf "$archive" "aviate-${version}/Cargo.lock" \
        | awk '/^\[\[package\]\]/ { f = 0 }
               /^name = "aviate-core"$/ { f = 1 }
               f && /^checksum = / { gsub(/"/, "", $3); print $3; exit }'
}

# The facade archive that is actually published (repackaged unpatched after
# aviate-core is on the registry) must be source-equivalent to the preflight
# archive (everything but Cargo.lock) and must pin aviate-core to the exact
# checksum of the core archive we published.
verify_facade_provenance() {
    local v=$1
    local real="target/package/aviate-${v}.crate"
    local pre="target/package/aviate-${v}.preflight.crate"
    local core="target/package/aviate-core-${v}.crate"

    [ -f "$pre" ] || { echo "::error::preflight facade archive missing: ${pre}" >&2; return 1; }
    [ -f "$core" ] || { echo "::error::published core archive missing: ${core}" >&2; return 1; }

    local da db
    da="$(mktemp -d)"; db="$(mktemp -d)"
    tar -xzf "$real" -C "$da"; tar -xzf "$pre" -C "$db"
    rm -f "$da/aviate-${v}/Cargo.lock" "$db/aviate-${v}/Cargo.lock"
    if ! diff -r "$da" "$db" >/dev/null 2>&1; then
        echo "::error::published facade source differs from the preflight archive (excluding Cargo.lock)" >&2
        rm -rf "$da" "$db"; return 1
    fi
    rm -rf "$da" "$db"

    local lock_ck core_ck
    lock_ck="$(facade_lock_core_checksum "$real" "$v")"
    core_ck="$(sha256sum "$core" | awk '{print $1}')"
    [ -n "$lock_ck" ] || { echo "::error::facade Cargo.lock has no registry checksum for aviate-core" >&2; return 1; }
    [ "$lock_ck" = "$core_ck" ] \
        || { echo "::error::facade locks aviate-core checksum ${lock_ck} != published core archive ${core_ck}" >&2; return 1; }
    echo "facade provenance OK: source matches preflight; Cargo.lock pins aviate-core to the published checksum"
}

# Overridable cargo actions (mocked in the self-test).
do_cargo_package() { cargo package -p "$1" --locked --no-verify; }
do_cargo_publish() { cargo publish -p "$1" --locked; }

publish_or_skip() {
    local c=$1 v=$2 decision
    do_cargo_package "$c"
    [ "$c" = "aviate" ] && verify_facade_provenance "$v"
    decision="$(classify_version "$c" "$v")"
    case "$decision" in
        skip)
            echo "${c} ${v} already published with matching checksum — skipping"
            ;;
        publish)
            echo "Publishing ${c} ${v}"
            do_cargo_publish "$c"
            wait_visible "$c" "$v"
            verify_published "$c" "$v"
            ;;
    esac
}

# --- self-test -------------------------------------------------------------
# Mocks driven by dedicated globals to avoid colliding with index_entry's
# locals. Covers classify decisions AND the fresh-publish path (publish ->
# wait_visible -> verify_published), including post-publish failures.
MOCK_LOCAL_CK="aaaa"
MOCK_PUBLISHED="0"
MOCK_INDEX_ERR="0"
MOCK_BEFORE=""      # index body before publish
MOCK_AFTER=""       # index body after publish

# shellcheck disable=SC2317  # called indirectly after redefinition
mock_local_checksum() { printf '%s' "$MOCK_LOCAL_CK"; }
# shellcheck disable=SC2317
mock_index_fetch() {
    [ "$MOCK_INDEX_ERR" = "1" ] && return 2
    if [ "$MOCK_PUBLISHED" = "1" ]; then printf '%s' "$MOCK_AFTER"; else printf '%s' "$MOCK_BEFORE"; fi
}

self_test() {
    local fails=0
    WAIT_ATTEMPTS=2
    WAIT_SLEEP=0
    # These overrides are invoked indirectly via classify_version/publish_or_skip.
    # shellcheck disable=SC2317,SC2329
    local_checksum() { mock_local_checksum; }
    # shellcheck disable=SC2317,SC2329
    index_fetch() { mock_index_fetch; }
    # shellcheck disable=SC2317,SC2329
    do_cargo_package() { :; }
    # shellcheck disable=SC2317,SC2329
    do_cargo_publish() { MOCK_PUBLISHED=1; }

    local present_match present_mismatch present_yanked
    present_match="$(jq -cn --arg ck "$MOCK_LOCAL_CK" '{vers:"1.2.3",cksum:$ck,yanked:false}')"
    present_mismatch="$(jq -cn '{vers:"1.2.3",cksum:"different",yanked:false}')"
    present_yanked="$(jq -cn --arg ck "$MOCK_LOCAL_CK" '{vers:"1.2.3",cksum:$ck,yanked:true}')"

    _reset() { MOCK_PUBLISHED=0; MOCK_INDEX_ERR=0; MOCK_BEFORE=""; MOCK_AFTER=""; }

    _expect() { # <label> <expected pass|fail> <fn...>
        local label=$1 expected=$2; shift 2
        local rc
        set +e; "$@" >/dev/null 2>&1; rc=$?; set -e
        local got="pass"; [ "$rc" -ne 0 ] && got="fail"
        if [ "$got" = "$expected" ]; then echo "  ok: ${label} -> ${got}"
        else echo "  FAIL: ${label} expected ${expected}, got ${got} (rc=${rc})" >&2; fails=1; fi
    }

    # classify decisions
    _reset;                                _expect "classify:absent"           pass classify_version testcrate 1.2.3
    _reset; MOCK_BEFORE="$present_match";   _expect "classify:match"            pass classify_version testcrate 1.2.3
    _reset; MOCK_BEFORE="$present_mismatch";_expect "classify:mismatch"         fail classify_version testcrate 1.2.3
    _reset; MOCK_BEFORE="$present_yanked";  _expect "classify:yanked"           fail classify_version testcrate 1.2.3
    _reset; MOCK_INDEX_ERR=1;               _expect "classify:index-error"      fail classify_version testcrate 1.2.3

    # fresh-publish path: absent -> publish -> visible -> verify
    _reset; MOCK_AFTER="$present_match";    _expect "publish:fresh-ok"          pass publish_or_skip testcrate 1.2.3
    _reset; MOCK_AFTER="$present_yanked";   _expect "publish:yanked-after"      fail publish_or_skip testcrate 1.2.3
    _reset; MOCK_AFTER="$present_mismatch"; _expect "publish:mismatch-after"    fail publish_or_skip testcrate 1.2.3
    _reset; MOCK_AFTER="";                  _expect "publish:never-visible"     fail publish_or_skip testcrate 1.2.3
    # already-published, matching -> skip (no publish)
    _reset; MOCK_BEFORE="$present_match"; MOCK_AFTER="$present_match"
    _expect "publish:already-skip" pass publish_or_skip testcrate 1.2.3

    if [ "$fails" -ne 0 ]; then echo "release_publish self-test: FAILED" >&2; return 1; fi
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
