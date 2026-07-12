# Repository governance

`main` is protected by an active GitHub ruleset. A green workflow is a
release gate, not advisory: GitHub refuses the merge unless the policy
below holds.

## Ruleset policy for `main`

- Changes land through pull requests only; direct pushes are refused.
- At least one approving review; approvals are dismissed when new
  commits materially change the PR, and every review conversation must
  be resolved before merge.
- The required status check is the single aggregate `CI Success` job,
  which fails unless every blocking gate in `.github/workflows/ci.yml`
  succeeds. Individual matrix names (for example the SITL missions)
  may change without touching the ruleset; the aggregate absorbs them.
- The branch must be up to date with `main` before merge.
- Force-push and branch deletion are blocked.
- Bypass is limited to the repository admin role. Every bypass use is
  recorded in the repository audit log; treat any bypass as an event to
  explain in the next review.

## Verification

Read-only check of the active ruleset:

```sh
gh api repos/{owner}/{repo}/rulesets --jq '.[] | {id, name, enforcement, target}'
gh api repos/{owner}/{repo}/rulesets/<id> --jq '{conditions, rules: [.rules[].type], bypass: .bypass_actors}'
```

`scripts/check_branch_protection.sh` performs this audit and exits
non-zero when the active ruleset no longer matches the policy above.
It needs repository-administration read permission, so it runs
manually or from a scheduled job with a dedicated token — never from
pull-request CI.
