#!/usr/bin/env bash
#
# Reproducible MAVLink dialect generation for the Aviate dialect.
#
# aviate.xml includes common.xml; resolving that include against whatever
# XML happens to be installed on the machine makes generation
# irreproducible. This script stages the vendored, pinned upstream
# definitions (aviate-link/message_definitions/upstream/) next to
# aviate.xml in target/mavlink-dialect/ and generates the Python dialect
# there with a pinned pymavlink generator version. The definitions and
# the generator are pinned separately: the codec tracks upstream
# message_definitions (which carry extension fields the released
# pymavlink snapshots lack), while the generator pin fixes the tool.
#
# Modes:
#   scripts/generate_mavlink_dialect.sh          # generate only
#   scripts/generate_mavlink_dialect.sh --check  # generate, then verify
#       the generated wire facts (message id, crc_extra, payload length)
#       against aviate-link/message_definitions/expected_wire.json, which
#       pins the same numbers the hand-written Rust codec uses.
#
# Exit codes: 0 on success, 1 on any failure.

set -euo pipefail

PYMAVLINK_PIN="2.4.41"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEFS="$REPO_ROOT/aviate-link/message_definitions"
OUT="$REPO_ROOT/target/mavlink-dialect"

CHECK=0
for arg in "$@"; do
    case "$arg" in
        --check) CHECK=1 ;;
        -h|--help)
            awk 'NR >= 2 && !/^#/ { exit } NR >= 2 { sub(/^# ?/, ""); print }' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            exit 1
            ;;
    esac
done

version="$(python3 -c 'import pymavlink; print(pymavlink.__version__)' 2>/dev/null)" || {
    echo "pymavlink $PYMAVLINK_PIN is not importable; run:" >&2
    echo "  python3 -m pip install --require-hashes -r scripts/mavlink-requirements.txt" >&2
    exit 1
}
if [[ "$version" != "$PYMAVLINK_PIN" ]]; then
    echo "pymavlink $version does not match the pin $PYMAVLINK_PIN;" >&2
    echo "generation is only reproducible against the pinned version." >&2
    exit 1
fi

mkdir -p "$OUT"
cp "$DEFS/upstream/common.xml" "$DEFS/upstream/standard.xml" "$DEFS/upstream/minimal.xml" \
    "$DEFS/aviate.xml" "$OUT/"

python3 - "$OUT" <<'PY'
import os
import sys

from pymavlink.generator import mavgen

out = sys.argv[1]
# validate=False: the XSD bundled with released pymavlink lags the
# upstream message definitions (e.g. minValue and superseded elements).
# The wire-fact check below is the gate that matters.
opts = mavgen.Opts(
    output=os.path.join(out, "aviate_dialect.py"),
    language="Python",
    wire_protocol="2.0",
    validate=False,
)
mavgen.mavgen(opts, [os.path.join(out, "aviate.xml")])
PY

if [[ "$CHECK" -eq 1 ]]; then
    python3 - "$OUT" "$DEFS/expected_wire.json" <<'PY'
import importlib.util
import json
import os
import sys

out, expected_path = sys.argv[1], sys.argv[2]
spec = importlib.util.spec_from_file_location(
    "aviate_dialect", os.path.join(out, "aviate_dialect.py")
)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

with open(expected_path) as fh:
    expected = json.load(fh)

failures = []
for name, want in expected.items():
    msg_type = mod.mavlink_map.get(want["id"])
    if msg_type is None:
        failures.append(f"{name}: id {want['id']} missing from generated dialect")
        continue
    if msg_type.msgname != name:
        failures.append(f"id {want['id']}: name {msg_type.msgname} != {name}")
    if msg_type.crc_extra != want["crc_extra"]:
        failures.append(
            f"{name}: crc_extra {msg_type.crc_extra} != expected {want['crc_extra']}"
        )
    payload_len = msg_type.unpacker.size
    if payload_len != want["payload_len"]:
        failures.append(
            f"{name}: payload_len {payload_len} != expected {want['payload_len']}"
        )

# Reverse direction: a private-range message added to aviate.xml without
# a pinned entry must fail, or its wire facts land unpinned.
expected_ids = {want["id"] for want in expected.values()}
for msg_id, msg_type in mod.mavlink_map.items():
    if msg_id >= 20000 and msg_id not in expected_ids:
        failures.append(
            f"{msg_type.msgname}: private-range id {msg_id} missing from expected_wire.json"
        )

for failure in failures:
    print(f"FAIL: {failure}", file=sys.stderr)
sys.exit(1 if failures else 0)
PY
    echo "Dialect wire-fact check: OK"
fi

echo "Generated: $OUT/aviate_dialect.py"
