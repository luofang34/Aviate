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
version, retire the old name with a behavior-based rationale, update
the implementation's `ALGORITHM_ID` constant, and move the pinned
aggregate in `identity_hash_is_stable_across_builds` — all in one
commit.

## Adjudication

`scripts/check_algorithm_identity.sh` runs in CI on every pull request.
Whenever a production estimator / controller / mixer / sanitizer
implementation path changes, the change must be adjudicated by its
author — neutrality is never inferred from the shape of the diff:

- a behavior change rotates the identity (the diff touches the
  registry), or
- a behavior-neutral change carries an explicit commit trailer with a
  rationale:

  ```
  Algorithm-Identity-Unchanged: <why this cannot change observable behavior>
  ```

A bare trailer without rationale text does not count. Test modules are
outside the guard: they cannot change flight behavior.
