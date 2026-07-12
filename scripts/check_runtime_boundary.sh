#!/usr/bin/env bash
# Verifies aviate-runtime's env-flight build is dependency-closed within
# the cert boundary (LLR-RTM-101 / LLR-RTM-102, TST-RTM-101).
#
# CI gate. The mutually-exclusive feature guards in
# aviate-runtime/src/lib.rs prevent SITL/HITL transports from
# COMPILING into a flight image. This script prevents a new sandbox-
# class crate from being LINKED in via a non-feature-gated path.
#
# Allowlist = `in_scope` crates from cert/boundary.toml plus a small
# fixed set of DAL-D base externals. Any new transitive dependency not
# on the allowlist fails the gate; lifting it requires either adding
# to the allowlist (with cert review) or feature-gating the dep.

set -euo pipefail

ALLOWED=(
  # in_scope per cert/boundary.toml
  aviate-core aviate-config aviate-hal-io aviate-drivers
  aviate-airframe-multirotor aviate-airframe-fixed-wing aviate-link
  aviate-runtime aviate-security
  # External DAL-D base crates approved for the flight build
  log bitflags embedded-hal libm
)

DEPS=$(cargo tree -p aviate-runtime --no-default-features --features env-flight \
  --prefix none --edges normal 2>/dev/null \
  | awk '{print $1}' | sed '/^$/d' | sort -u)

violations=0
while IFS= read -r dep; do
  [[ -z "$dep" ]] && continue
  found=0
  for ok in "${ALLOWED[@]}"; do
    if [[ "$dep" == "$ok" ]]; then found=1; break; fi
  done
  if [[ $found -eq 0 ]]; then
    echo "BOUNDARY VIOLATION: aviate-runtime --features env-flight pulls $dep" >&2
    violations=$((violations + 1))
  fi
done <<< "$DEPS"

if [[ $violations -gt 0 ]]; then
  echo "" >&2
  echo "Add the dep to scripts/check_runtime_boundary.sh's ALLOWED list" >&2
  echo "(cert review required) or feature-gate it behind env-sitl/env-hitl." >&2
  exit 1
fi

echo "Runtime boundary: OK ($(echo "$DEPS" | wc -l | tr -d ' ') deps under env-flight, all in allowlist)"

# Airframe selection belongs to the app layer: the runtime provides
# the generic runner and must not name a concrete controller/mixer or
# airframe tuning set.
# Manifest-level: a generic board/runtime crate must not even depend
# on an airframe crate — src-level greps cannot see Cargo.toml.
for manifest in aviate-runtime/Cargo.toml \
    aviate-boards/sitl-gazebo/Cargo.toml aviate-boards/sitl-jmavsim/Cargo.toml; do
  if grep -En "aviate-airframe" "$manifest" > /dev/null; then
    echo "FAIL: airframe crate dependency in $manifest" >&2
    grep -En "aviate-airframe" "$manifest" >&2
    exit 1
  fi
done

for tree in aviate-runtime/src aviate-boards/sitl-gazebo/src aviate-boards/sitl-jmavsim/src; do
  if grep -rEn "MultirotorController|QuadXMixerX500|x500_defaults" "$tree" > /dev/null; then
    echo "FAIL: concrete airframe types in $tree" >&2
    grep -rEn "MultirotorController|QuadXMixerX500|x500_defaults" "$tree" >&2
    exit 1
  fi
done
echo "Runtime/board airframe boundary: OK (apps own airframe selection)"
