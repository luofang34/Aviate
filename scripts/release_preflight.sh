#!/usr/bin/env bash
#
# Release preflight: package BOTH crates, prove each archive's recorded VCS
# commit equals the tagged commit, and build an external consumer against the
# PACKAGED sources (not the workspace path deps). Runs before OIDC auth; no
# token or network publish here.
#
# Usage: release_preflight.sh <version> <expected-git-sha>

set -euo pipefail

VERSION="${1:?usage: release_preflight.sh <version> <git-sha>}"
EXPECTED_SHA="${2:?usage: release_preflight.sh <version> <git-sha>}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# Package both crates. aviate-core is a leaf and is fully build-verified.
# aviate exact-pins `aviate-core =<version>`, which is not on the registry
# until the release publishes it, so packaging aviate here would fail the
# index lookup. A resolution-only patch to the local aviate-core lets it
# package now.
#
# The patched archive is SOURCE/PROVENANCE-equivalent to what publish
# uploads (same source, manifest still `=<version>`, same .cargo_vcs_info),
# but NOT byte-identical: its embedded Cargo.lock records aviate-core as a
# path crate (no registry source/checksum), whereas the archive published
# after aviate-core is on the registry records the registry source and the
# core's checksum. release_publish.sh compares the real archive to this one
# excluding Cargo.lock, and separately checks the lock's core checksum.
cargo package -p aviate-core --locked
cargo package -p aviate --locked --no-verify \
    --config 'patch.crates-io.aviate-core.path="aviate-core"'
# Preserve the patched facade archive for the publish-time source comparison
# (publish repackages aviate unpatched, overwriting the default path).
cp "target/package/aviate-${VERSION}.crate" "target/package/aviate-${VERSION}.preflight.crate"

for crate in aviate-core aviate; do
    archive="target/package/${crate}-${VERSION}.crate"
    [ -f "$archive" ] || {
        echo "::error::expected package archive not found: $archive" >&2
        exit 1
    }
    vcs_sha="$(tar -xzOf "$archive" "${crate}-${VERSION}/.cargo_vcs_info.json" \
        | jq -r '.git.sha1')"
    if [ "$vcs_sha" != "$EXPECTED_SHA" ]; then
        echo "::error::${crate} .cargo_vcs_info.json git sha1 is ${vcs_sha}, expected ${EXPECTED_SHA} (the tagged commit)" >&2
        exit 1
    fi
    echo "${crate}: archive VCS commit matches ${EXPECTED_SHA}"
done

# External-consumer smoke: extract both packaged crates and build a throwaway
# binary that depends on the packaged `aviate`, patching `aviate-core` to the
# packaged copy. This exercises the published sources as a downstream user
# would, not the in-tree workspace.
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
for crate in aviate-core aviate; do
    tar -xzf "target/package/${crate}-${VERSION}.crate" -C "$work"
done

mkdir -p "$work/consumer/src"
cat > "$work/consumer/Cargo.toml" <<EOF
[package]
name = "aviate-consumer-smoke"
version = "0.0.0"
edition = "2021"
publish = false

[dependencies]
aviate = { path = "../aviate-${VERSION}" }

[patch.crates-io]
aviate-core = { path = "../aviate-core-${VERSION}" }
EOF

cat > "$work/consumer/src/main.rs" <<'EOF'
fn main() {
    // Touch the re-exported surface so the packaged facade must compile.
    let _ = aviate::math::Vector3::new(
        aviate::types::Meters(0.0),
        aviate::types::Meters(0.0),
        aviate::types::Meters(0.0),
    );
}
EOF

( cd "$work/consumer" && cargo build )
echo "external-consumer smoke build: OK (packaged aviate ${VERSION} consumed)"
