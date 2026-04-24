# Certification overrides

Aviate uses [`cargo-evidence`](https://crates.io/crates/cargo-evidence)
to track DO-178C evidence and enforce reproducibility across builds.
This document records the two override protocols the tool recognizes
and the review expectations that come with each.

## `Override-Deterministic-Baseline:` — toolchain drift

`cargo evidence` projects every build's toolchain fingerprint
(`rustc`, `cargo`, `llvm_version`, `cargo_lock_hash`,
`rust_toolchain_toml`, `rustflags`). When a PR changes any of those
inputs, the cross-time determinism check requires an explicit
acknowledgment in the form of a single line in the PR body or the
head commit message:

```
Override-Deterministic-Baseline: <one-sentence reason>
```

### When to use it

Only when the drift is intentional. Examples:

```
Override-Deterministic-Baseline: bumped serde_json to 1.0.130 for CVE-2025-NNNNN
Override-Deterministic-Baseline: pinned rust-toolchain to 1.96 for std::simd stabilization
Override-Deterministic-Baseline: added -C opt-level=3 to RUSTFLAGS for flight-path hot loops
```

### What not to do

- Don't paper over unexpected drift — investigate the root cause first
  (stale `Cargo.lock`, rebase picking up a surprise bump).
- Don't use the override line as a general "I know what I'm doing"
  bypass — it is specifically for toolchain/reproducibility inputs.
- **Squash-merge caveat**: the squash button drops the PR body unless
  the committer hand-copies it into the commit message. If your PR
  carries an `Override-Deterministic-Baseline:` line, paste it into
  the squash commit's extended description before merging, or a
  post-merge dogfood run on `main` will fail.

## `Lower-Floor:` — ratcheting evidence floors

`cert/floors.toml` declares lower bounds on evidence dimensions
(trace counts, `#[test]` counts, library panic counts, etc.). The
ratchet only moves up; to intentionally drop a floor you must add

```
Lower-Floor: <dimension> <reason>
```

to the PR body or squash commit. Example:

```
Lower-Floor: trace_llr_count removed LLR-074 (merged into LLR-071)
```

Without the line, `scripts/floors-lower-lint.sh` fires in CI with
`FLOORS_LOWERED_WITHOUT_JUSTIFICATION`.

## Reviewer checklist

If you see either override line in a PR:

- [ ] The reason is specific (CVE number, API reference, LLR ID) —
      not "upgrade deps" or "cleanup".
- [ ] The diff actually matches the claim (e.g. if it's a toolchain
      bump, `rust-toolchain.toml` is the only in-scope change).
- [ ] For squash-merge, the override line has been copied into the
      commit message body.
