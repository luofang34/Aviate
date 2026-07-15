#!/usr/bin/env python3
"""Verify docs/AVIATE_CORE_TEST_PLAN.md against the cert trace tree.

The test plan is a human navigation view over cert/trace/*.toml. It is
hand-written Markdown, so nothing structural stops its verification
matrix or DRQ table from drifting away from the trace it claims to
mirror — a row can list a requirement ID that does not exist, omit one
of an HLR's decomposing LLRs, or claim a DRQ lifecycle status that the
authoritative derived.toml does not carry. This checker makes that
drift a CI failure:

  * Every 200-series HLR in cert/trace/hlr.toml appears exactly once
    in the plan's verification matrix, and no matrix row names an HLR
    absent from the trace.
  * Each matrix row's "Decomposed by" set equals the set of LLRs whose
    traces_to includes that HLR.
  * Each matrix row's "Witnessed by" TST set equals the set of TST
    rows tracing to those LLRs.
  * The plan's DRQ table lists exactly the DRQ IDs in derived.toml.
  * Every derived.toml requirement carries a machine-readable
    status ("open" | "closed" | "standing"), and the plan's DRQ table
    states the same status. A DRQ cannot be closed in the navigation
    view while the authoritative file says otherwise.

Exit codes: 0 — in sync; 1 — at least one drift defect (all printed).

Usage: scripts/check_test_plan_sync.py   # from repo root
"""

import re
import sys
import tomllib
from pathlib import Path

TRACE_DIR = Path("cert/trace")
PLAN = Path("docs/AVIATE_CORE_TEST_PLAN.md")

HLR_RE = re.compile(r"HLR-[A-Z]+-\d+[A-Z]?")
LLR_RE = re.compile(r"LLR-[A-Z]+-\d+[A-Z]?")
TST_RE = re.compile(r"TST-[A-Z]+-\d+[A-Z]?")
DRQ_RE = re.compile(r"DRQ-[A-Z]+-\d+[A-Z]?")
BEHAVIORAL_HLR_RE = re.compile(r"^HLR-[A-Z]+-2\d\d$")
VALID_STATUSES = {"open", "closed", "standing"}


def load_trace():
    """Return (behavioral_hlrs, decomp, witnesses, drq_status)."""
    tomls = {
        name: tomllib.loads((TRACE_DIR / f"{name}.toml").read_text())
        for name in ("hlr", "llr", "tests", "derived")
    }
    hlr_rows = tomls["hlr"]["requirements"]
    llr_rows = tomls["llr"]["requirements"]
    tst_rows = tomls["tests"]["tests"]
    drq_rows = tomls["derived"]["requirements"]

    behavioral = {r["id"]: r["uid"] for r in hlr_rows if BEHAVIORAL_HLR_RE.match(r["id"])}

    decomp = {hlr_id: set() for hlr_id in behavioral}
    llr_uid_to_id = {}
    for llr in llr_rows:
        llr_uid_to_id[llr["uid"]] = llr["id"]
        for hlr_id, hlr_uid in behavioral.items():
            if hlr_uid in llr.get("traces_to", []):
                decomp[hlr_id].add(llr["id"])

    witnesses = {hlr_id: set() for hlr_id in behavioral}
    for tst in tst_rows:
        traced_llrs = {
            llr_uid_to_id[u] for u in tst.get("traces_to", []) if u in llr_uid_to_id
        }
        for hlr_id in behavioral:
            if traced_llrs & decomp[hlr_id]:
                witnesses[hlr_id].add(tst["id"])

    drq_status = {r["id"]: r.get("status") for r in drq_rows}
    return behavioral, decomp, witnesses, drq_status


def plan_sections(text):
    """Split the plan into (heading, body-lines) sections."""
    sections = {}
    heading = ""
    for line in text.splitlines():
        if line.startswith("## "):
            heading = line[3:].strip()
            sections[heading] = []
        elif heading:
            sections[heading].append(line)
    return sections


def parse_matrix(lines):
    """Return {hlr_id: (llr_set, tst_set)} from the matrix table."""
    rows = {}
    defects = []
    for line in lines:
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 4 or not HLR_RE.search(cells[0]):
            continue
        hlr_ids = HLR_RE.findall(cells[0])
        if len(hlr_ids) != 1:
            defects.append(f"matrix row names {len(hlr_ids)} HLR ids: {line.strip()}")
            continue
        hlr_id = hlr_ids[0]
        if hlr_id in rows:
            defects.append(f"matrix lists {hlr_id} more than once")
            continue
        rows[hlr_id] = (set(LLR_RE.findall(cells[2])), set(TST_RE.findall(cells[3])))
    return rows, defects


def parse_drq_table(lines):
    """Return {drq_id: status_word} from the DRQ table."""
    statuses = {}
    for line in lines:
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 2 or not DRQ_RE.search(cells[0]):
            continue
        drq_ids = DRQ_RE.findall(cells[0])
        word = re.match(r"[A-Za-z]+", cells[1])
        statuses[drq_ids[0]] = word.group(0).lower() if word else ""
    return statuses


def diff_sets(kind, hlr_id, plan_set, trace_set, defects):
    missing = trace_set - plan_set
    extra = plan_set - trace_set
    if missing:
        defects.append(f"{hlr_id}: matrix omits {kind} {sorted(missing)} (trace has them)")
    if extra:
        defects.append(f"{hlr_id}: matrix lists {kind} {sorted(extra)} not traced to it")


def main():
    behavioral, decomp, witnesses, drq_status = load_trace()
    sections = plan_sections(PLAN.read_text())

    matrix_lines = sections.get("Verification matrix", [])
    drq_lines = next(
        (body for head, body in sections.items() if head.startswith("Tier DRQ")), []
    )

    defects = []
    matrix, row_defects = parse_matrix(matrix_lines)
    defects += row_defects

    for hlr_id in sorted(set(behavioral) - set(matrix)):
        defects.append(f"matrix is missing behavioral HLR {hlr_id}")
    for hlr_id in sorted(set(matrix) - set(behavioral)):
        defects.append(f"matrix lists {hlr_id}, which is not a behavioral HLR in the trace")
    for hlr_id in sorted(set(matrix) & set(behavioral)):
        plan_llrs, plan_tsts = matrix[hlr_id]
        diff_sets("LLRs", hlr_id, plan_llrs, decomp[hlr_id], defects)
        diff_sets("TSTs", hlr_id, plan_tsts, witnesses[hlr_id], defects)

    plan_drqs = parse_drq_table(drq_lines)
    for drq_id in sorted(set(drq_status) - set(plan_drqs)):
        defects.append(f"DRQ table is missing {drq_id}")
    for drq_id in sorted(set(plan_drqs) - set(drq_status)):
        defects.append(f"DRQ table lists {drq_id}, which is not in derived.toml")
    for drq_id in sorted(set(plan_drqs) & set(drq_status)):
        authoritative = drq_status[drq_id]
        if authoritative not in VALID_STATUSES:
            defects.append(
                f"{drq_id}: derived.toml status is {authoritative!r}; "
                f"expected one of {sorted(VALID_STATUSES)}"
            )
        elif plan_drqs[drq_id] != authoritative:
            defects.append(
                f"{drq_id}: plan says {plan_drqs[drq_id]!r} but derived.toml "
                f"says {authoritative!r}"
            )

    if defects:
        print(f"TEST_PLAN_SYNC: {len(defects)} defect(s) between {PLAN} and {TRACE_DIR}/")
        for d in defects:
            print(f"  [x] {d}")
        return 1
    print(f"TEST_PLAN_SYNC_OK: {PLAN} matches {TRACE_DIR}/ "
          f"({len(matrix)} HLR rows, {len(plan_drqs)} DRQ rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
