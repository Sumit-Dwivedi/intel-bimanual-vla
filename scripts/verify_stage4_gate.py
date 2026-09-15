"""Redesign Stage 4, Step 3: the SIX-ITEM gate (ADR-061) -- Stage 2's own
five items (`scripts/verify_stage2_gate.py`, imported and re-run here
UNCHANGED) plus a new item 6: every one of the eight skills' FULL approach
sequence is swept-path clear from home.

Item 6 reuses `scripts/diagnose_swept_path.py`'s own capture-and-check
machinery (the SAME monkeypatch technique, so the waypoint chain checked
here is identical to what Step 2's diagnosis already validated) rather than
re-deriving waypoint targets a third time.

A skill counts as PASSING item 6 only if EVERY waypoint captured for it is
individually `swept_path_clear`. The headline number this script reports
("N of 8 skills pass the six-item gate") is item-6-per-skill AND items 1-5
(which are global, not per-skill) all passing together.

Run on bm-ptl:
    python scripts/verify_stage4_gate.py
"""
from __future__ import annotations

import json
import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import importlib  # noqa: E402

OUT_PATH = REPO_ROOT / "out" / "stage4_six_item_gate.json"


def main() -> int:
    # ---- Items 1-5: re-run Stage 2's own gate script as a subprocess so
    # its own main()/return-code semantics are reused verbatim, not
    # reimplemented. ----
    import subprocess
    stage2 = subprocess.run(
        [sys.executable, str(REPO_ROOT / "scripts" / "verify_stage2_gate.py")],
        capture_output=True, text=True,
    )
    print(stage2.stdout)
    if stage2.returncode != 0:
        print(stage2.stderr)
    items_1_5_pass = stage2.returncode == 0
    stage2_report_path = REPO_ROOT / "out" / "stage2_gate_report.json"
    stage2_report = json.loads(stage2_report_path.read_text()) if stage2_report_path.exists() else {}

    # ---- Item 6: full approach-sequence swept-path clearance, per skill --
    # reuse diagnose_swept_path.py's own capture machinery directly (import,
    # not subprocess, so we get the in-memory report without re-parsing
    # stdout). ----
    diag = importlib.import_module("diagnose_swept_path")
    # diag.main() prints a full report and writes stage4_swept_path_diagnosis.json;
    # re-run it fresh here so item 6 always reflects the CURRENT geometry,
    # not a stale file from an earlier stage of this same session.
    diag.main()
    diag_report = json.loads(diag.OUT_PATH.read_text())

    item6_per_skill = {}
    for skill_report in diag_report["per_skill"]:
        label = skill_report["label"]
        all_clear = all(wp["clear"] for wp in skill_report["waypoints"])
        item6_per_skill[label] = all_clear

    print("\n" + "=" * 70)
    print("SIX-ITEM GATE SUMMARY")
    print(f"Items 1-5 (Stage 2 static gate): {'PASS' if items_1_5_pass else 'FAIL'}")
    print("Item 6 (full approach sequence swept-path clear from home), per skill:")
    n_six_item_pass = 0
    for label, ok in item6_per_skill.items():
        six_item_pass = items_1_5_pass and ok
        n_six_item_pass += 1 if six_item_pass else 0
        print(f"    {label:32s} item6={'PASS' if ok else 'FAIL'}  six-item-gate={'PASS' if six_item_pass else 'FAIL'}")

    print(f"\n{n_six_item_pass}/{len(item6_per_skill)} skills pass the full SIX-ITEM gate.")

    out = {
        "items_1_5_pass": items_1_5_pass,
        "stage2_report": stage2_report,
        "item6_per_skill": item6_per_skill,
        "n_six_item_pass": n_six_item_pass,
        "n_skills": len(item6_per_skill),
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, default=str), newline="\n", encoding="utf-8")
    print(f"\nFull report: {OUT_PATH}")
    return 0 if n_six_item_pass == len(item6_per_skill) else 1


if __name__ == "__main__":
    raise SystemExit(main())
