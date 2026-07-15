# Repository governance

`main` is protected by an active GitHub ruleset. A green workflow is a
release gate, not advisory: GitHub refuses the merge unless the policy
below holds.

## Ruleset policy for `main`

Exactly one active branch ruleset, `main-protection`, reaches `main`
(via `~DEFAULT_BRANCH`, with no exclusions). Its policy:

- Changes land through pull requests only; direct pushes are refused.
- Review approval is not required while the project has a single
  developer, because self-approval would be theater. This is the
  explicit solo-developer exception: `required_approving_review_count`
  is 0, `dismiss_stale_reviews_on_push` and
  `required_review_thread_resolution` are false. When a second
  qualified reviewer exists, flip all three (to 1/true/true) in the
  live ruleset, in the expected values inside
  `scripts/check_branch_protection.sh`, and in this document — in the
  same PR.
- The required status check is the single aggregate `CI Success` job,
  which fails unless every blocking gate in `.github/workflows/ci.yml`
  succeeds. Individual matrix names (for example the SITL missions)
  may change without touching the ruleset; the aggregate absorbs them.
- `CI Success` is source-bound to the GitHub Actions App
  (`integration_id` 15368, slug `github-actions`, observed on the app
  field of a live `CI Success` check run). A commit status or a check
  produced by any other integration cannot satisfy the gate.
- The branch must be up to date with `main` before merge
  (`strict_required_status_checks_policy`), and required checks also
  apply on branch creation.
- Force-push and branch deletion are blocked.
- `bypass_actors` is empty: no actor — including the repository
  admin role — can bypass the rules while they are active. There is
  no standing break-glass entry. The only override path is editing
  the ruleset itself, which requires repository administration, is
  recorded in the repository audit log, and is caught as drift by the
  next audit run; treat any such edit as an event to explain.

## Required-workflow trust anchor (open platform gap)

The policy above pins who may produce `CI Success` (the GitHub
Actions App) but not what `.github/workflows/ci.yml` contains: a pull
request can weaken the workflow that defines the aggregate gate and
then merge under the weakened gate.

GitHub's protection for this — the "require workflows to pass before
merging" ruleset rule — cannot be expressed on this repository:

- The GitHub docs ("Available rules for rulesets", Enterprise Cloud
  variant) state that ruleset workflows "can be configured at the
  organization or enterprise level"; the rule is absent from the
  repository-ruleset variant of the same page.
- A live probe confirmed it: a `PUT` adding a `workflows` rule (with
  a valid `repository_id` and `ref`) to ruleset `18824861` on this
  user-owned repository returns HTTP 422 `Invalid rule 'workflows'`.

The gap stays explicitly open (tracked in issue #254) until the
repository moves under an organization or GitHub ships the rule for
user-owned repositories. Partial mitigations that do apply here:

- The `integration_id` binding above stops any non-Actions producer
  from faking the `CI Success` context.
- Workflow-file diffs under `.github/workflows/` get explicit human
  attention in review. When a second reviewer exists, a `CODEOWNERS`
  entry for `.github/workflows/**` combined with
  `require_code_owner_review: true` and a nonzero approval count
  turns this into a GitHub-enforced anchor.

## Other active rulesets

`release-tag-immutability` (tag ruleset, `refs/tags/v*`) blocks tag
deletion, update, and non-fast-forward moves, with no bypass actors.
It does not reach `main`; the audit recognizes it and excludes it
from the branch verdict.

## Verification

`scripts/check_branch_protection.sh` audits the live rulesets against
this document: it enumerates every ruleset, computes which active
branch rulesets reach `main`, fails on any undocumented one, and
verifies every field of `main-protection` listed above — targeting,
enforcement, deletion/non-fast-forward, review parameters, required
check identity, strictness, producing app, and the empty bypass list.
A token that cannot see `bypass_actors` fails the audit rather than
passing it.

`scripts/check_branch_protection.sh --self-test` replays adversarial
single-field mutations of a policy fixture through the same assertion
logic and fails if any drift case escapes; it needs no network access
or credentials.

The `governance-audit` workflow runs both on a weekly schedule and on
manual dispatch with `contents: read` only, and uploads the live JSON
the verdict was computed from as a run artifact. It never runs on
pull-request code. If the default workflow token cannot read
`bypass_actors`, add a `GOVERNANCE_AUDIT_TOKEN` repository secret
containing a fine-grained PAT with read-only Administration
permission scoped to this repository.
