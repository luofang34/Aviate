# Aviate MAVLink dialect

`aviate.xml` is the canonical wire contract for every Aviate-specific
MAVLink message. The hand-written Rust codec in
`aviate-link/src/mavlink/` implements this contract; it is pinned to it
by byte-exact golden vectors generated with pymavlink, and by
`expected_wire.json`, which records the wire facts (message id,
CRC_EXTRA, payload length) both sides must agree on.

## Reproducible generation

`aviate.xml` includes `common.xml`. Resolving that include against an
unpinned local installation makes generation irreproducible, so the
pinned upstream definitions are vendored in `upstream/` (see its README
for provenance) and generation goes through one entry point:

```sh
pip install pymavlink==2.4.41
scripts/generate_mavlink_dialect.sh          # writes target/mavlink-dialect/aviate_dialect.py
scripts/generate_mavlink_dialect.sh --check  # also verifies expected_wire.json
```

The script refuses to run against any other pymavlink version.
`expected_wire.json` pins the message id, CRC_EXTRA, and full payload
length of every message the codec implements — standard and Aviate —
so `--check` fails when the vendored definitions and the codec disagree
about any of them, and when a private-range message exists in aviate.xml
without a pinned entry.

## Changing the wire contract

The dialect is WIP and private: message id 20000 is self-assigned and no
upstream id range has been requested. Until the dialect is declared
stable, layout changes are allowed, and each one must land in a single
commit that updates together:

1. `aviate.xml` (the contract),
2. `expected_wire.json` (the pinned wire facts),
3. the Rust codec constants and field layout,
4. the pymavlink golden vectors in the codec tests.

`--check` fails when the XML and the pinned wire facts drift apart;
the golden-vector tests fail when the Rust codec drifts from pymavlink
output. Publishing the dialect requires requesting a dedicated
message-id range upstream first.
