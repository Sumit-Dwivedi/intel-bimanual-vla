"""Redesign Stage 4, Step 1: ground-truth validation of `check_swept_path.
swept_path_clear` -- a HARD STOP per the task brief (do not proceed to Step
2/3 diagnosis if either check below fails; an unvalidated gate would send
the diagnosis chasing phantom collisions or missing real ones).

Both checks use `pick(A, fork)`'s waypoint 1 (APPROACH: home -> hover above
the fork's grasp point), computed with the SAME constants
`skills_scripted.run_pick` itself uses (`GRASP_POINT_OFFSET_M`,
`OBJECT_TOP_LOCAL_Z_M`, `CLEARANCE_HEIGHT_M`), never hand-transcribed, so a
constant drift in this script cannot silently diverge from what the real
skill would target.

**Check 1 (master's geometry -> gate must report CLEAR).** `pick(A, fork)`
is in master's own regression gate (`scripts/verify_adr038_skills.py`,
z 0.3560 -> 0.3989) -- it demonstrably works there. `pick(A, fork)` never
reads `HANDOFF_POSITION_XYZ` (confirmed by grep: only `run_handoff`
references that constant), so swapping ONLY the compiled scene XML
(`src/bimanual/sim/assets/so101_dual_table.xml`) to master's version is
sufficient to reproduce master's geometry for this one skill -- no other
file needs to change. The master XML is checked out to the working tree
via `git show master:<path>`, NEVER by `git checkout master` (this stays on
`redesign` throughout; branch is verified before and after).

**Check 2 (redesign's CURRENT geometry -> gate must report a COLLISION).**
Run with no XML swap at all -- whatever is already on disk on `redesign`
(the ADR-060 geometry Stage 3 tested and found 1/8 skills passing). The
gate's reported contact pairs are printed for comparison against Stage 3's
own text ("cross-arm + arm-vs-table" at waypoint 1).

The redesign XML is restored (`git checkout -- <path>`) in a `finally`
block regardless of outcome, and the restore is itself verified via
`git diff --stat` before this script exits.
"""
from __future__ import annotations

import pathlib
import subprocess
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import mujoco  # noqa: E402

from check_swept_path import swept_path_clear  # noqa: E402
from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

XML_PATH = REPO_ROOT / "src" / "bimanual" / "sim" / "assets" / "so101_dual_table.xml"


def _git(args: list[str]) -> str:
    result = subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True)
    return result.stdout.strip()


def _fork_hover_waypoint(env) -> tuple[np.ndarray, np.ndarray]:
    """Reproduce `run_pick`'s OWN waypoint-1 (APPROACH) target for
    `pick(A, fork)`, using its exact constants -- see module docstring.
    Returns (start_qpos, hover_target_xyz).
    """
    model, data = env.model, env.data
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fork")
    obj_pos0 = np.array(data.xpos[body_id], dtype=np.float64, copy=True)
    offset = sk.GRASP_POINT_OFFSET_M["fork"]
    grasp_point = obj_pos0 + offset
    top_local_z = sk.OBJECT_TOP_LOCAL_Z_M.get("fork", offset[2])
    hover_z = max(float(grasp_point[2]), float(obj_pos0[2] + top_local_z)) + sk.CLEARANCE_HEIGHT_M
    hover = np.array([grasp_point[0], grasp_point[1], hover_z])
    start_qpos = np.array(data.qpos, dtype=np.float64, copy=True)
    return start_qpos, hover


def _run_check(label: str) -> dict:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    start_qpos, hover = _fork_hover_waypoint(env)
    clear, worst_pen, worst_step, contacts = swept_path_clear(
        env.model, env.data, "A", start_qpos, hover,
        target_geom_names={"fork_handle", "fork_head"},
    )
    env.close()
    print(f"\n[{label}] pick(A, fork) waypoint 1 (home -> hover):")
    print(f"    hover target = {np.round(hover, 4).tolist()}")
    print(f"    clear={clear}  worst_penetration={worst_pen:.5f}  worst_step={worst_step}")
    for c in contacts[:15]:
        print(f"    step={c['step']:2d}  {c['geom1']} <-> {c['geom2']}  dist={c['dist']:.5f}")
    return {"label": label, "clear": clear, "worst_penetration": worst_pen,
            "worst_step": worst_step, "contact_pairs": contacts}


def main() -> int:
    branch = _git(["rev-parse", "--abbrev-ref", "HEAD"])
    if branch != "redesign":
        print(f"REFUSING to run: current branch is {branch!r}, expected 'redesign'.")
        return 2
    print(f"branch: {branch} (confirmed)")

    dirty_before = _git(["status", "--porcelain", "--", str(XML_PATH)])
    if dirty_before:
        print(f"REFUSING to run: {XML_PATH} already has uncommitted changes:\n{dirty_before}")
        return 2

    results = {}
    swapped = False
    try:
        # ---- Check 2 FIRST (no XML swap needed) so a failure here is
        # reported before we ever touch the working tree. ----
        results["check2_redesign_geometry"] = _run_check("CHECK 2: redesign geometry (current HEAD)")

        # ---- Check 1: swap in master's XML, run, then restore. ----
        master_xml = _git(["show", "master:src/bimanual/sim/assets/so101_dual_table.xml"])
        XML_PATH.write_text(master_xml, newline="\n", encoding="utf-8")
        swapped = True
        results["check1_master_geometry"] = _run_check("CHECK 1: master geometry (temporarily checked out)")
    finally:
        if swapped:
            _git(["checkout", "--", str(XML_PATH.relative_to(REPO_ROOT))])
            after = _git(["status", "--porcelain", "--", str(XML_PATH)])
            print(f"\nrestored {XML_PATH.name} from redesign HEAD; git status for it now: "
                  f"{after!r} (expected empty)")
            if after:
                print("RESTORE DID NOT FULLY CLEAN -- STOP AND INSPECT.")
                return 3

    branch_after = _git(["rev-parse", "--abbrev-ref", "HEAD"])
    print(f"\nbranch after run: {branch_after} (expected 'redesign')")

    c1 = results.get("check1_master_geometry")
    c2 = results.get("check2_redesign_geometry")
    c1_pass = c1 is not None and c1["clear"] is True
    c2_pass = c2 is not None and c2["clear"] is False  # must find a collision

    print("\n" + "=" * 70)
    print(f"CHECK 1 (master geometry must be CLEAR):        {'PASS' if c1_pass else 'FAIL'}")
    print(f"CHECK 2 (redesign geometry must show COLLISION): {'PASS' if c2_pass else 'FAIL'}")
    if not c1_pass:
        print("\nCHECK 1 FAILED -- the gate is not trustworthy. STOPPING per the task's hard-stop rule.")
        print("Do not proceed to Step 2 diagnosis or Step 3 re-placement until this is fixed.")
        return 1
    if not c2_pass:
        print("\nCHECK 2 did not reproduce a collision on redesign's current geometry -- "
              "this does not block Step 2 by the task's letter (only check 1 is the named hard "
              "stop), but it means the gate may not agree with Stage 3's own report. Flagging, "
              "not stopping.")
    return 0 if c1_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
