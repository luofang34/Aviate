#!/usr/bin/env bash
#
# Read-only audit of the effective ruleset policy protecting `main`.
#
# Enumerates every repository ruleset, identifies all active branch
# rulesets whose ref conditions reach `main` (fnmatch include/exclude
# evaluation, plus the `~ALL` / `~DEFAULT_BRANCH` tokens), and
# verifies the aggregate against the policy recorded in
# docs/REPO_GOVERNANCE.md. Any undocumented or duplicate ruleset
# reaching `main`, and any single-field drift inside the documented
# one — ref targeting, enforcement, exact rule-type inventory, review
# parameters, required-check identity and producing app, bypass
# actors — is a failure.
#
# Requires `gh` authenticated with permission to read ruleset
# `bypass_actors` (the field is omitted for low-privilege callers, and
# an invisible field is treated as a failure, never as "empty"). Run
# manually or from the governance-audit workflow with a least-privilege
# token; never from pull-request CI, which must not hold a token that
# can read repository administration data.
#
# `--self-test` feeds mutated fixtures through the same assertion
# logic to prove every drift case fails; it performs no network access
# and needs no credentials.
#
# When EVIDENCE_DIR is set, the live JSON the verdict was computed
# from is written there before any assertion runs, so a failing audit
# still records what it saw.
#
# Exit codes: 0 policy holds (or every drift fixture was detected),
# 1 policy drifted or a drift fixture escaped detection.

set -euo pipefail

MODE="${1:-live}"
if [[ "$MODE" != "live" && "$MODE" != "--self-test" ]]; then
    echo "usage: $0 [--self-test]" >&2
    exit 2
fi

if [[ "$MODE" == "live" ]]; then
    REPO="${REPO:-$(gh repo view --json nameWithOwner --jq .nameWithOwner)}"
else
    REPO=""
fi

python3 - "$MODE" "$REPO" <<'PY'
import copy
import json
import os
import re
import subprocess
import sys
from collections import Counter

MODE = sys.argv[1]
REPO = sys.argv[2]

# The constants below mirror docs/REPO_GOVERNANCE.md. A deliberate
# policy change edits the live ruleset, these constants, and the docs
# in the same PR; disagreement in any direction is drift.
EXPECTED_RULESET_NAME = "main-protection"

# GitHub Actions App id, observed on the app of a live `CI Success`
# check run. Binding the required check to this integration means a
# commit status or a check from any other app cannot satisfy the gate.
EXPECTED_CI_APP_ID = 15368

EXPECTED_CONTEXTS = {"CI Success"}

EXPECTED_CONDITIONS = {"ref_name": {"exclude": [], "include": ["~DEFAULT_BRANCH"]}}

# Solo-developer exception: approvals stay off while the project has a
# single qualified reviewer, because self-approval would be theater.
# When a second reviewer exists, flip required_approving_review_count
# to 1, dismiss_stale_reviews_on_push and
# required_review_thread_resolution to True — live ruleset, these
# constants, and docs together.
EXPECTED_REVIEW = {
    "required_approving_review_count": 0,
    "dismiss_stale_reviews_on_push": False,
    "required_reviewers": [],
    "require_code_owner_review": False,
    "require_last_push_approval": False,
    "required_review_thread_resolution": False,
}
EXPECTED_MERGE_METHODS = {"merge", "squash", "rebase"}
EXPECTED_RULE_TYPES = (
    "deletion",
    "non_fast_forward",
    "pull_request",
    "required_status_checks",
)

MAIN_REF = "refs/heads/main"


def ref_pattern_regex(pattern):
    # Ruleset ref conditions use fnmatch: `*` stops at `/`, `**`
    # crosses it, `?` is one non-slash character, brackets are
    # character classes. An unterminated bracket falls back to a
    # literal so a malformed pattern cannot crash the audit.
    out = []
    i = 0
    while i < len(pattern):
        char = pattern[i]
        if char == "*":
            if pattern[i + 1:i + 2] == "*":
                out.append(".*")
                i += 1
            else:
                out.append("[^/]*")
        elif char == "?":
            out.append("[^/]")
        elif char == "[":
            end = pattern.find("]", i + 2)
            if end == -1:
                out.append(re.escape(char))
            else:
                body = pattern[i + 1:end]
                negate = body[:1] in ("!", "^")
                if negate:
                    body = body[1:]
                out.append("[" + ("^" if negate else "") + body + "]")
                i = end
        else:
            out.append(re.escape(char))
        i += 1
    return "".join(out)


def pattern_matches_main(pattern):
    if pattern in ("~ALL", "~DEFAULT_BRANCH"):
        return True
    return re.fullmatch(ref_pattern_regex(pattern), MAIN_REF) is not None


def reaches_main(detail):
    if detail.get("target") != "branch":
        return False
    if detail.get("enforcement") != "active":
        return False
    ref_name = detail.get("conditions", {}).get("ref_name", {})
    if not any(pattern_matches_main(p) for p in ref_name.get("include", [])):
        return False
    if any(pattern_matches_main(p) for p in ref_name.get("exclude", [])):
        return False
    return True


def audit_pull_request(rule, failures):
    params = rule.get("parameters", {})
    for key, expected in EXPECTED_REVIEW.items():
        got = params.get(key)
        if got != expected:
            failures.append(
                f"pull_request.{key} = {json.dumps(got)}, "
                f"documented policy says {json.dumps(expected)}"
            )
    methods = set(params.get("allowed_merge_methods", []))
    if methods != EXPECTED_MERGE_METHODS:
        failures.append(
            f"allowed_merge_methods drifted: {sorted(methods)} "
            f"(policy: {sorted(EXPECTED_MERGE_METHODS)})"
        )


def audit_required_checks(rule, failures):
    params = rule.get("parameters", {})
    checks = params.get("required_status_checks", [])
    contexts = {c.get("context") for c in checks}
    if contexts != EXPECTED_CONTEXTS:
        failures.append(
            f"required check contexts drifted: {sorted(contexts)} "
            f"(policy: {sorted(EXPECTED_CONTEXTS)})"
        )
    for check in checks:
        if (
            check.get("context") == "CI Success"
            and check.get("integration_id") != EXPECTED_CI_APP_ID
        ):
            failures.append(
                f"'CI Success' is not bound to GitHub Actions app "
                f"{EXPECTED_CI_APP_ID}: integration_id = "
                f"{json.dumps(check.get('integration_id'))}"
            )
    if not params.get("strict_required_status_checks_policy"):
        failures.append("branch not required to be up to date before merge")
    if params.get("do_not_enforce_on_create"):
        failures.append("required checks are skipped on branch creation")


def audit(rulesets):
    failures = []
    matching = [d for d in rulesets if reaches_main(d)]
    if not matching:
        failures.append("no active branch ruleset reaches refs/heads/main")
        return failures

    for detail in matching:
        if detail.get("name") != EXPECTED_RULESET_NAME:
            failures.append(
                f"undocumented ruleset {detail.get('name')!r} "
                f"(id {detail.get('id')}) reaches main; the documented "
                f"policy has exactly one ruleset, {EXPECTED_RULESET_NAME!r}"
            )
    documented = [d for d in matching if d.get("name") == EXPECTED_RULESET_NAME]
    if not documented:
        failures.append(
            f"documented ruleset {EXPECTED_RULESET_NAME!r} does not reach main"
        )
        return failures
    if len(documented) > 1:
        failures.append(
            f"{len(documented)} rulesets named {EXPECTED_RULESET_NAME!r} "
            "reach main; the documented policy has exactly one"
        )
    ruleset = documented[0]

    if ruleset.get("conditions") != EXPECTED_CONDITIONS:
        failures.append(
            f"ref targeting drifted: {json.dumps(ruleset.get('conditions'))} "
            f"(policy: {json.dumps(EXPECTED_CONDITIONS)})"
        )

    # An absent bypass_actors field means the token was not allowed to
    # see it; passing on that would let a low-privilege audit certify a
    # bypass hole, so invisibility is itself a failure.
    bypass = ruleset.get("bypass_actors")
    if bypass is None:
        failures.append(
            "bypass_actors not visible to this token; grant read-only "
            "repository Administration permission so the audit can "
            "verify the bypass list"
        )
    elif bypass != []:
        failures.append(
            f"bypass_actors must be empty, found {json.dumps(bypass)}"
        )

    # Each expected rule type must appear exactly once and nothing
    # else may: a duplicate rule of a checked type could carry weaker
    # parameters than the instance the audit happens to inspect, and
    # an unexpected type is policy the docs do not describe.
    type_counts = Counter(rule["type"] for rule in ruleset.get("rules", []))
    for rule_type, count in sorted(type_counts.items()):
        if rule_type not in EXPECTED_RULE_TYPES:
            failures.append(f"unexpected rule type: {rule_type}")
        elif count > 1:
            failures.append(
                f"duplicate rule type: {rule_type} appears {count} times"
            )

    rules = {rule["type"]: rule for rule in ruleset.get("rules", [])}
    for required in ("deletion", "non_fast_forward"):
        if required not in rules:
            failures.append(f"missing rule: {required}")

    if "pull_request" not in rules:
        failures.append("missing rule: pull_request")
    else:
        audit_pull_request(rules["pull_request"], failures)

    if "required_status_checks" not in rules:
        failures.append("missing rule: required_status_checks")
    else:
        audit_required_checks(rules["required_status_checks"], failures)

    return failures


def fetch_live(repo):
    def api(path):
        # Fetched here rather than piped in: a heredoc replaces stdin,
        # so a pipeline into this script would silently starve
        # json.load.
        return json.loads(
            subprocess.run(
                ["gh", "api", path],
                capture_output=True, text=True, check=True,
            ).stdout
        )

    index = api(f"repos/{repo}/rulesets")
    details = [api(f"repos/{repo}/rulesets/{entry['id']}") for entry in index]

    evidence_dir = os.environ.get("EVIDENCE_DIR")
    if evidence_dir:
        os.makedirs(evidence_dir, exist_ok=True)
        with open(os.path.join(evidence_dir, "rulesets-index.json"), "w") as f:
            json.dump(index, f, indent=2)
        for detail in details:
            path = os.path.join(evidence_dir, f"ruleset-{detail['id']}.json")
            with open(path, "w") as f:
                json.dump(detail, f, indent=2)
    return details


def baseline():
    # Mirror of the live policy, plus the tag ruleset to prove that
    # rulesets not reaching main are recognized and left out of the
    # branch verdict.
    return [
        {
            "id": 18824861,
            "name": "main-protection",
            "target": "branch",
            "enforcement": "active",
            "conditions": {
                "ref_name": {"exclude": [], "include": ["~DEFAULT_BRANCH"]}
            },
            "bypass_actors": [],
            "rules": [
                {"type": "deletion"},
                {"type": "non_fast_forward"},
                {
                    "type": "pull_request",
                    "parameters": {
                        "required_approving_review_count": 0,
                        "dismiss_stale_reviews_on_push": False,
                        "required_reviewers": [],
                        "require_code_owner_review": False,
                        "require_last_push_approval": False,
                        "required_review_thread_resolution": False,
                        "allowed_merge_methods": ["merge", "squash", "rebase"],
                    },
                },
                {
                    "type": "required_status_checks",
                    "parameters": {
                        "strict_required_status_checks_policy": True,
                        "do_not_enforce_on_create": False,
                        "required_status_checks": [
                            {"context": "CI Success", "integration_id": 15368}
                        ],
                    },
                },
            ],
        },
        {
            "id": 18950901,
            "name": "release-tag-immutability",
            "target": "tag",
            "enforcement": "active",
            "conditions": {
                "ref_name": {"exclude": [], "include": ["refs/tags/v*"]}
            },
            "bypass_actors": [],
            "rules": [
                {"type": "deletion"},
                {"type": "non_fast_forward"},
                {"type": "update"},
            ],
        },
    ]


def drop_rule(rs, rule_type):
    rs[0]["rules"] = [r for r in rs[0]["rules"] if r["type"] != rule_type]


def rule_params(rs, rule_type):
    return next(r for r in rs[0]["rules"] if r["type"] == rule_type)["parameters"]


def add_shadow_ruleset(rs, include=None, exclude=None,
                       name="shadow-main-ruleset"):
    shadow = copy.deepcopy(rs[0])
    shadow["id"] = 99999999
    shadow["name"] = name
    if include is not None:
        shadow["conditions"]["ref_name"]["include"] = include
    if exclude is not None:
        shadow["conditions"]["ref_name"]["exclude"] = exclude
    rs.append(shadow)


def duplicate_rule(rs, rule_type):
    original = next(r for r in rs[0]["rules"] if r["type"] == rule_type)
    rs[0]["rules"].append(copy.deepcopy(original))


# Every property the audit asserts gets an adversarial single-field
# mutation here; the self-test fails if any fixture passes the audit
# or fails it for the wrong reason.
DRIFT_CASES = [
    ("ruleset list empty",
     lambda rs: rs.clear(),
     "no active branch ruleset"),
    ("enforcement flipped to evaluate",
     lambda rs: rs[0].update(enforcement="evaluate"),
     "no active branch ruleset"),
    ("target flipped from branch to push",
     lambda rs: rs[0].update(target="push"),
     "no active branch ruleset"),
    ("include retargeted away from main",
     lambda rs: rs[0]["conditions"]["ref_name"].update(
         include=["refs/heads/dev"]),
     "no active branch ruleset"),
    ("main excluded by ref condition",
     lambda rs: rs[0]["conditions"]["ref_name"].update(
         exclude=["refs/heads/main"]),
     "no active branch ruleset"),
    ("include widened to ~ALL",
     lambda rs: rs[0]["conditions"]["ref_name"].update(include=["~ALL"]),
     "ref targeting drifted"),
    ("documented ruleset renamed",
     lambda rs: rs[0].update(name="main-protection-legacy"),
     "undocumented ruleset"),
    ("second undocumented ruleset reaches main",
     add_shadow_ruleset,
     "undocumented ruleset"),
    ("shadow ruleset included via ~ALL",
     lambda rs: add_shadow_ruleset(rs, include=["~ALL"]),
     "undocumented ruleset"),
    ("shadow ruleset included via ~DEFAULT_BRANCH",
     lambda rs: add_shadow_ruleset(rs, include=["~DEFAULT_BRANCH"]),
     "undocumented ruleset"),
    ("shadow ruleset included via refs/heads/* fnmatch",
     lambda rs: add_shadow_ruleset(rs, include=["refs/heads/*"]),
     "undocumented ruleset"),
    ("shadow ruleset included via refs/heads/** fnmatch",
     lambda rs: add_shadow_ruleset(rs, include=["refs/heads/**"]),
     "undocumented ruleset"),
    ("shadow ruleset included via refs/heads/ma* fnmatch",
     lambda rs: add_shadow_ruleset(rs, include=["refs/heads/ma*"]),
     "undocumented ruleset"),
    ("duplicate ruleset named main-protection",
     lambda rs: add_shadow_ruleset(rs, name="main-protection"),
     "rulesets named"),
    ("always-on bypass actor restored",
     lambda rs: rs[0].update(bypass_actors=[
         {"actor_id": 5, "actor_type": "RepositoryRole",
          "bypass_mode": "always"}]),
     "bypass_actors must be empty"),
    ("bypass_actors invisible to token",
     lambda rs: rs[0].pop("bypass_actors"),
     "not visible to this token"),
    ("deletion rule removed",
     lambda rs: drop_rule(rs, "deletion"),
     "missing rule: deletion"),
    ("non_fast_forward rule removed",
     lambda rs: drop_rule(rs, "non_fast_forward"),
     "missing rule: non_fast_forward"),
    ("pull_request rule removed",
     lambda rs: drop_rule(rs, "pull_request"),
     "missing rule: pull_request"),
    ("approval count drifted from documented value",
     lambda rs: rule_params(rs, "pull_request").update(
         required_approving_review_count=1),
     "pull_request.required_approving_review_count"),
    ("stale-review dismissal drifted from documented value",
     lambda rs: rule_params(rs, "pull_request").update(
         dismiss_stale_reviews_on_push=True),
     "pull_request.dismiss_stale_reviews_on_push"),
    ("thread resolution drifted from documented value",
     lambda rs: rule_params(rs, "pull_request").update(
         required_review_thread_resolution=True),
     "pull_request.required_review_thread_resolution"),
    ("code-owner review drifted from documented value",
     lambda rs: rule_params(rs, "pull_request").update(
         require_code_owner_review=True),
     "pull_request.require_code_owner_review"),
    ("last-push approval drifted from documented value",
     lambda rs: rule_params(rs, "pull_request").update(
         require_last_push_approval=True),
     "pull_request.require_last_push_approval"),
    ("required reviewers drifted from documented value",
     lambda rs: rule_params(rs, "pull_request").update(
         required_reviewers=[{"id": 1}]),
     "pull_request.required_reviewers"),
    ("merge methods narrowed",
     lambda rs: rule_params(rs, "pull_request").update(
         allowed_merge_methods=["merge"]),
     "allowed_merge_methods"),
    ("required_status_checks rule removed",
     lambda rs: drop_rule(rs, "required_status_checks"),
     "missing rule: required_status_checks"),
    ("required check context renamed",
     lambda rs: rule_params(rs, "required_status_checks")
     ["required_status_checks"][0].update(context="CI Green"),
     "required check contexts drifted"),
    ("extra required check context added",
     lambda rs: rule_params(rs, "required_status_checks")
     ["required_status_checks"].append({"context": "Extra Check"}),
     "required check contexts drifted"),
    ("strict up-to-date policy disabled",
     lambda rs: rule_params(rs, "required_status_checks").update(
         strict_required_status_checks_policy=False),
     "up to date"),
    ("checks skipped on branch creation",
     lambda rs: rule_params(rs, "required_status_checks").update(
         do_not_enforce_on_create=True),
     "branch creation"),
    ("check bound to wrong integration",
     lambda rs: rule_params(rs, "required_status_checks")
     ["required_status_checks"][0].update(integration_id=99999),
     "not bound to GitHub Actions app"),
    ("check source binding removed",
     lambda rs: rule_params(rs, "required_status_checks")
     ["required_status_checks"][0].pop("integration_id"),
     "not bound to GitHub Actions app"),
    ("duplicate required_status_checks rule",
     lambda rs: duplicate_rule(rs, "required_status_checks"),
     "duplicate rule type"),
    ("duplicate pull_request rule",
     lambda rs: duplicate_rule(rs, "pull_request"),
     "duplicate rule type"),
    ("unexpected rule type present",
     lambda rs: rs[0]["rules"].append({"type": "required_signatures"}),
     "unexpected rule type"),
]

# Fixtures the audit must NOT flag: rulesets whose ref conditions do
# not reach main must stay out of the branch verdict, or the audit
# would cry wolf and train readers to ignore it.
CLEAN_CASES = [
    ("shadow ruleset with non-matching include pattern",
     lambda rs: add_shadow_ruleset(rs, include=["refs/heads/release/*"])),
    ("shadow ruleset excluding main by literal ref",
     lambda rs: add_shadow_ruleset(rs, include=["refs/heads/**"],
                                   exclude=["refs/heads/main"])),
    ("shadow ruleset excluding main by fnmatch pattern",
     lambda rs: add_shadow_ruleset(rs, include=["~ALL"],
                                   exclude=["refs/heads/ma*"])),
]


def self_test():
    escaped = 0
    clean = audit(baseline())
    if clean:
        print(f"SELF-TEST FAIL: baseline fixture rejected: {clean}",
              file=sys.stderr)
        escaped += 1
    for name, mutate in CLEAN_CASES:
        rulesets = baseline()
        mutate(rulesets)
        failures = audit(rulesets)
        if failures:
            print(
                f"SELF-TEST FAIL: {name}: expected a clean audit, "
                f"got {failures}",
                file=sys.stderr,
            )
            escaped += 1
        else:
            print(f"non-matching fixture ignored: {name}")
    for name, mutate, fragment in DRIFT_CASES:
        rulesets = baseline()
        mutate(rulesets)
        failures = audit(rulesets)
        if failures and any(fragment in f for f in failures):
            print(f"drift detected: {name}")
        else:
            print(
                f"SELF-TEST FAIL: {name}: failures={failures} "
                f"(expected a failure containing {fragment!r})",
                file=sys.stderr,
            )
            escaped += 1
    if escaped:
        sys.exit(1)
    print(
        f"Self-test: all {len(DRIFT_CASES)} drift cases detected, "
        f"{len(CLEAN_CASES)} non-matching fixtures ignored"
    )


if MODE == "--self-test":
    self_test()
else:
    live_failures = audit(fetch_live(REPO))
    for failure in live_failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if live_failures:
        sys.exit(1)
    print("Branch-protection audit: OK")
PY
