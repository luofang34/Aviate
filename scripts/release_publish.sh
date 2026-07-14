#!/usr/bin/env bash
#
# Checksum/yank-aware idempotent publish, in dependency order.
#
# For each crate: if the version is already on the index, refuse to "skip"
# unless it is not yanked AND its recorded checksum matches the archive we
# just built — otherwise the published archive differs from this commit and
# the release must fail (bump the version). If absent, publish and wait for
# index visibility.
#
# Requires CARGO_REGISTRY_TOKEN in the environment (from OIDC auth).
# Usage: release_publish.sh <version>

set -euo pipefail

VERSION="${1:?usage: release_publish.sh <version>}"

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

# Print the compact JSON index entry for a version, or nothing if absent.
index_entry() {
    local c=$1 v=$2
    curl -sfL "https://index.crates.io/$(index_path "$c")" 2>/dev/null \
        | jq -c --arg v "$v" 'select(.vers == $v)' 2>/dev/null || true
}

local_checksum() {
    local c=$1 v=$2
    sha256sum "target/package/${c}-${v}.crate" | awk '{print $1}'
}

wait_visible() {
    local c=$1 v=$2
    for _ in $(seq 1 60); do
        [ -n "$(index_entry "$c" "$v")" ] && { echo "${c} ${v} visible on index"; return 0; }
        sleep 5
    done
    echo "::error::${c} ${v} did not become visible on the index" >&2
    return 1
}

publish_or_skip() {
    local c=$1 v=$2
    local entry
    entry="$(index_entry "$c" "$v")"

    if [ -n "$entry" ]; then
        local yanked cksum local_ck
        yanked="$(jq -r '.yanked' <<<"$entry")"
        cksum="$(jq -r '.cksum' <<<"$entry")"
        local_ck="$(local_checksum "$c" "$v")"
        if [ "$yanked" = "true" ]; then
            echo "::error::${c} ${v} is yanked on crates.io; bump the version, do not re-release" >&2
            return 1
        fi
        if [ "$cksum" != "$local_ck" ]; then
            echo "::error::${c} ${v} already published with checksum ${cksum}, but the archive built from this commit is ${local_ck}; refusing to skip a mismatched artifact" >&2
            return 1
        fi
        echo "${c} ${v} already published with matching checksum — skipping"
        return 0
    fi

    echo "Publishing ${c} ${v}"
    cargo publish -p "$c" --locked
    wait_visible "$c" "$v"
}

# Dependency order: kernel before facade.
publish_or_skip aviate-core "$VERSION"
publish_or_skip aviate "$VERSION"
echo "Release v${VERSION} publish step complete."
