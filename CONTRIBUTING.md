# Contributing

## Running the local gates before pushing

```bash
cargo fmt --all -- --check
cargo clippy --workspace -- -D warnings
cargo test --workspace
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
cargo evidence doctor
cargo evidence check
cargo evidence floors
```

See `scripts/run_pre_release_checks.sh` for the pre-release superset.

## The `Override-Deterministic-Baseline:` protocol

Aviate tracks a toolchain projection (rustc / cargo / llvm_version /
cargo_lock_hash / rust_toolchain_toml / rustflags) across builds. A
PR whose projection differs from the last green `main` build is
assumed to be drift unless the author explicitly acknowledges the
change.

When the projection differs, include a line of this exact shape in
either the PR body or the head commit message:

    Override-Deterministic-Baseline: <one-sentence reason>

Examples of legitimate reasons:

    Override-Deterministic-Baseline: bumped serde_json to 1.0.130 for CVE-2026-NNNN
    Override-Deterministic-Baseline: added -C opt-level=3 to RUSTFLAGS for size target
    Override-Deterministic-Baseline: upgraded rust-toolchain pin to 1.96

Without that line, `cargo evidence` will reject the build with the
full projection diff. The friction is intentional — reproducibility
inputs don't change by accident.

### Squash-merge caveat

GitHub's default "Squash and merge" button drops the PR body unless
the committer hand-copies it into the squash commit message. If your
PR carries an `Override-Deterministic-Baseline:` line, paste the line
into the squash commit's extended description before merging —
otherwise a post-merge dogfood run of the lint on `main` would fail
against the squashed commit's body. Projects that use merge-commits
or rebase-and-merge preserve the PR body and are unaffected.

## The `Lower-Floor:` protocol

Ratcheting floors in `cert/floors.toml` only move up. If a PR
legitimately needs to reduce a floor (measurement methodology
changed, a suite was retired, etc.), include a line of this shape:

    Lower-Floor: <dimension> <one-sentence reason>

Without it, `cargo evidence floors` fires
`FLOORS_LOWERED_WITHOUT_JUSTIFICATION` and CI blocks the merge.

## Commit-message hygiene

Commit messages are part of the traceability chain. Keep the subject
line short (≤ 70 chars) and use the body to describe the *why*.
Reference LLR/HLR UIDs from `cert/trace/` when a change implements
or modifies a tracked requirement.
