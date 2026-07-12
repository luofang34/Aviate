#!/usr/bin/env python3
"""Validate and query the SITL mission manifest.

tests/missions/manifest.json is the single source of truth for mission
name, gate status, shard assignment, run count, and pass threshold.
Validation enforces:

* bijection: every manifest mission names a real tests/missions TOML
  and every TOML has a manifest entry — a mission cannot vanish from
  orchestration silently;
* schema: blocking missions carry a shard and a reliability bar;
  quarantined missions additionally link a tracking issue (quarantine
  never masquerades as a passing gate — it runs in a visible
  non-blocking lane); manual missions state a reason;
* each mission is assigned exactly once by construction (one entry,
  one shard).

Query modes emit orchestration inputs so CI YAML and shell scripts
carry no hand-written duplicate mission lists:

    --emit-shard-matrix         GitHub Actions matrix for blocking shards
    --emit-quarantine-missions  space-separated quarantined mission names
    --emit-default-missions     blocking + quarantined names (local runs)
    --mission-plan NAME         "<runs> <pass_threshold>" for one mission

Exit codes: 0 valid, 1 validation failure, 2 bad invocation.
"""

import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
MANIFEST = REPO / "tests" / "missions" / "manifest.json"
MISSIONS_DIR = REPO / "tests" / "missions"

GATES = ("blocking", "quarantined", "manual")


def load():
    with open(MANIFEST) as fh:
        return json.load(fh)["missions"]


def validate(missions):
    failures = []

    toml_files = {p.stem for p in MISSIONS_DIR.glob("*.toml")}
    declared = set(missions)
    for missing in sorted(declared - toml_files):
        failures.append(f"{missing}: declared in manifest but tests/missions/{missing}.toml does not exist")
    for orphan in sorted(toml_files - declared):
        failures.append(f"{orphan}: tests/missions/{orphan}.toml has no manifest entry (silent omission)")

    for name, spec in sorted(missions.items()):
        gate = spec.get("gate")
        if gate not in GATES:
            failures.append(f"{name}: gate must be one of {GATES}, got {gate!r}")
            continue
        if gate in ("blocking", "quarantined"):
            if not isinstance(spec.get("shard"), str) or not spec["shard"]:
                failures.append(f"{name}: {gate} mission needs a shard")
            runs = spec.get("runs")
            threshold = spec.get("pass_threshold")
            if not (isinstance(runs, int) and isinstance(threshold, int) and runs >= threshold >= 1):
                failures.append(f"{name}: needs runs >= pass_threshold >= 1, got runs={runs!r} pass_threshold={threshold!r}")
        if gate == "quarantined":
            if not isinstance(spec.get("tracking_issue"), int):
                failures.append(f"{name}: quarantined mission must link a tracking_issue")
            if spec.get("shard") != "quarantine":
                failures.append(f"{name}: quarantined mission must sit in the quarantine shard, not a blocking one")
        if gate == "blocking" and spec.get("shard") == "quarantine":
            failures.append(f"{name}: blocking mission cannot sit in the quarantine shard")
        if gate == "manual" and not spec.get("reason"):
            failures.append(f"{name}: manual mission must state a reason")

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return not failures


def shard_matrix(missions):
    shards = {}
    for name, spec in sorted(missions.items()):
        if spec["gate"] == "blocking":
            shards.setdefault(spec["shard"], []).append(name)
    return {
        "include": [
            {"name": shard, "missions": " ".join(names)}
            for shard, names in sorted(shards.items())
        ]
    }


def main():
    missions = load()
    if not validate(missions):
        return 1

    args = sys.argv[1:]
    if not args:
        print("Mission manifest: OK")
        return 0
    if args == ["--emit-shard-matrix"]:
        print(json.dumps(shard_matrix(missions), separators=(",", ":")))
        return 0
    if args == ["--emit-quarantine-missions"]:
        print(" ".join(sorted(n for n, s in missions.items() if s["gate"] == "quarantined")))
        return 0
    if args == ["--emit-default-missions"]:
        print(" ".join(sorted(n for n, s in missions.items() if s["gate"] in ("blocking", "quarantined"))))
        return 0
    if len(args) == 2 and args[0] == "--mission-plan":
        spec = missions.get(args[1])
        if spec is None or spec["gate"] == "manual":
            print(f"FAIL: {args[1]} is not an orchestrated mission", file=sys.stderr)
            return 1
        print(f"{spec['runs']} {spec['pass_threshold']}")
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
