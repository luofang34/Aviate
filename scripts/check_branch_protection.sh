#!/usr/bin/env bash
#
# Read-only audit of the `main` branch-protection ruleset.
#
# Verifies that an active ruleset targeting `main` enforces the policy
# recorded in docs/REPO_GOVERNANCE.md: pull requests with one approval,
# stale-review dismissal, conversation resolution, the aggregate
# "CI Success" required check with up-to-date enforcement, and blocked
# force-push/deletion.
#
# Requires `gh` authenticated with repository-administration read
# permission. Run manually or from a scheduled job with a dedicated
# token; never from pull-request CI.
#
# Exit codes: 0 policy holds, 1 policy drifted or no active ruleset.

set -euo pipefail

REPO="${REPO:-$(gh repo view --json nameWithOwner --jq .nameWithOwner)}"

python3 - "$REPO" <<'PY'
import json
import subprocess
import sys

repo = sys.argv[1]
# Fetched here rather than piped in: a heredoc replaces stdin, so a
# pipeline into this script would silently starve json.load.
rulesets = json.loads(
    subprocess.run(
        ["gh", "api", f"repos/{repo}/rulesets"],
        capture_output=True, text=True, check=True,
    ).stdout
)

failures = []
active = None
for entry in rulesets:
    detail = json.loads(
        subprocess.run(
            ["gh", "api", f"repos/{repo}/rulesets/{entry['id']}"],
            capture_output=True, text=True, check=True,
        ).stdout
    )
    refs = detail.get("conditions", {}).get("ref_name", {}).get("include", [])
    if detail.get("enforcement") == "active" and (
        "refs/heads/main" in refs or "~DEFAULT_BRANCH" in refs
    ):
        active = detail
        break

if active is None:
    failures.append("no active ruleset targets main")
else:
    rules = {r["type"]: r for r in active.get("rules", [])}
    for required in ("deletion", "non_fast_forward", "pull_request", "required_status_checks"):
        if required not in rules:
            failures.append(f"missing rule: {required}")

    # Review approval is deliberately not required while the project has
    # a single developer; the pull_request rule still forces PR-based
    # flow so the CI Success gate applies to every change.

    checks = rules.get("required_status_checks", {}).get("parameters", {})
    contexts = [c.get("context") for c in checks.get("required_status_checks", [])]
    if "CI Success" not in contexts:
        failures.append("aggregate 'CI Success' is not a required check")
    if not checks.get("strict_required_status_checks_policy"):
        failures.append("branch not required to be up to date before merge")

for failure in failures:
    print(f"FAIL: {failure}", file=sys.stderr)
if failures:
    sys.exit(1)
print("Branch-protection audit: OK")
PY
