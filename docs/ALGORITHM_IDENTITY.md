# Algorithm identity

Three different hashes answer three different questions. Confusing them
defeats the cross-channel lockstep gate.

| Hash | Question it answers | Changes when |
|---|---|---|
| Algorithm identity (`algorithm_identity_hash`, FNV fold of the four `ALGORITHM_ID` constants) | Do two channels run behaviorally compatible estimator / controller / mixer / sanitizer implementations? | An implementation's observable behavior changes and its registry identity rotates |
| Firmware image hash | Are two channels running the same build artifact? | Any recompilation — compiler version, codegen, unrelated code |
| Resolved-config hash (`ResolvedKernelConfig`) | Are two channels flying the same tuning and limits? | Any configuration value changes |

Identity is deliberately coarser than the image hash: two differently
compiled images of the same algorithm version must agree on identity,
and two images of behaviorally different algorithm versions must not —
even when their state shapes are identical.

## Registry and rotation

`cert/algorithm_id_registry.toml` is the identity ledger. Rotating an
identity means: allocate a fresh, never-reused 64-bit ID for the new
version, retire the old ID in a registry comment with a
behavior-based rationale (retired IDs stay quoted in comments
forever — that comment ledger is what the reuse check reads), update
the implementation's `ALGORITHM_ID` constant, and — when the rotated
implementation belongs to a pinned production bundle — move the
pinned aggregates in `identity_hash_is_stable_across_builds` /
`identity_hash_is_stable_across_builds_x500` and in
`scripts/check_algorithm_identity.sh` — all in one commit. Never
quote an active ID in a registry comment: the checker treats every
commented hex literal as retired.

Test-only identities are registered under `[testing]`. They stay
outside the production allocation (a production implementation may
never take a `[testing]` ID and vice versa) but participate in the
global collision and reuse checks.

## Adjudication

`scripts/check_algorithm_identity.sh` runs in CI for pull requests
and for pushes to protected branches (`before..sha`), so a direct or
admin-bypass push to `main` is adjudicated exactly like a PR. The
script maps every changed production file to the registry entries
that own it; the gate is satisfied only by:

- a rotation of an owning entry — editing an unrelated registry entry
  (or a comment) does not count, or
- an exact `Algorithm-Identity-Unchanged` git trailer on **every
  commit** in the range that touches the non-rotated file:

  ```
  Algorithm-Identity-Unchanged: <why this cannot change observable behavior>
  ```

The trailer is parsed with `git interpret-trailers` from the final
trailer block: look-alike keys, copies embedded mid-body, duplicated
trailers, and bare trailers without rationale text all fail. When a
trailer-adjudicated PR is squash-merged, keep the trailer as a proper
trailer in the squash commit message — the push-range adjudication of
`main` re-checks it.

Independently of what changed, the script proves at the head
revision: active IDs (including `[testing]`) are globally unique; no
active ID reuses a retired one; every implementation's compiled
constant equals its registry entry; every `ALGORITHM_ID` literal in
the repo is registered in the right section; the generic quad-X and
X500 production aggregates match their pins; and no ID active at base
vanished from the ledger without a retirement record.

Test modules are outside the path guard: they cannot change flight
behavior.
