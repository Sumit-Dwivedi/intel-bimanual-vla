"""Redesign Stage 3, Step 2 multi-seed diagnostic (ADR-056/057, ADR-060).

For each of the FOUR distinct root-cause failures found by
`scripts/stage3_eight_skill_retest.py` at the new 0.40 m geometry (two of
the eight skill failures are downstream of `pick(A, fork)`'s failure and two
downstream of `pick(A, mug)`'s -- see the retest JSON -- so there are 4
independent targets to diagnose, not 8), this script runs the SAME 32-seed
perturbation `scripts/probe_multiseed_diagnostic.py` (ADR-057) already
defines, reused by import (never reimplemented), and reports whether the
residual PLATEAUS (still unreachable) or VARIES/DROPS (a local minimum /
skill-logic issue at otherwise-reachable geometry).

Read-only with respect to every frozen module named in this stage's rules
(`skills_scripted.py` beyond Step 1's single constant, `grasp.py`, `ik.py`,
`executor.py`, `env.py`) -- all imported, never edited. Also imports
`probe_multiseed_diagnostic.py`'s own `perturbed_residuals`/`summarize`
functions rather than re-deriving the perturbation formula a third time.

Run on bm-ptl:
    python scripts/stage3_multiseed_diagnostic.py
"""
from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.control.executor import ScriptedSkillExecutor  # noqa: E402
from bimanual.language.skills import SkillCall  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

from probe_multiseed_diagnostic import perturbed_residuals, summarize  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_PATH = REPO_ROOT / "out" / "stage3_multiseed_diagnostic.json"


def hover_target(env, target_object: str, arm: str):
    body_name = sk.OBJECT_BODY_NAME[target_object]
    body_id = sk._body_id(env.model, body_name)
    obj_pos0 = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
    offset = sk.GRASP_POINT_OFFSET_M.get(target_object, np.zeros(3))
    grasp_point = obj_pos0 + offset
    top_local_z = sk.OBJECT_TOP_LOCAL_Z_M.get(target_object, offset[2])
    hover_z = max(float(grasp_point[2]), float(obj_pos0[2] + top_local_z)) + sk.CLEARANCE_HEIGHT_M
    return np.array([grasp_point[0], grasp_point[1], hover_z])


def diag_pick_fork_waypoint1() -> dict:
    """Root cause of skill #1 (pick(A,fork)), and shared by #2 (place(A,fork,
    table), aborts because its nested pick fails identically) and #7
    (handoff(A->B,fork) phase 1, which IS run_pick(A,fork) verbatim).
    Documented failure type: COLLISION (cross_arm + armA-vs-table_top), not
    a reported convergence failure -- so this diagnostic's real question is
    whether the IK itself even struggles here (probably not) as distinct
    from the physical collision, which multi-seed IK restarts cannot
    resolve on their own (the collision check runs on whichever
    configuration _run_waypoint actually reaches, not on alternate IK
    solutions).
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    try:
        target = hover_target(env, "fork", "A")
        residuals = perturbed_residuals(env.model, env.data, "A", target)
        return {
            "target_label": "pick(A, fork) waypoint-1 approach (root cause of #1, #2, #7)",
            "arm": "A",
            "target": target.tolist(),
            "documented_failure_type": "collision (cross_arm contacts=1; armA-vs-table_top contacts=1) at waypoint 1, NOT a reported convergence failure",
            "residuals_all_32": [round(r, 5) for r in residuals],
            "summary": summarize(residuals),
        }
    finally:
        env.close()


def diag_pick_mug_waypoint1() -> dict:
    """Root cause of skill #5 (pick(A,mug)) and #6 (place(A,mug,table),
    aborts identically). Documented failure type: weld_attach_failed_after_
    300_frames -- a GRIP/weld-mechanism failure, i.e. waypoint 1 (approach)
    and the descend/grip waypoints already converged (frames_used=1300, the
    same shape probe_multiseed_diagnostic.py's target_e found for a
    different prop) BEFORE the grasp-attach mechanism itself gave up. Run
    for completeness per this stage's own instruction, with the same
    caveat probe_multiseed_diagnostic.py's target_e states explicitly:
    multi-seed IK restarts cannot rescue a failure that was never an IK
    failure.
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    try:
        target = hover_target(env, "mug", "A")
        residuals = perturbed_residuals(env.model, env.data, "A", target)
        return {
            "target_label": "pick(A, mug) waypoint-1 approach (root cause of #5, #6)",
            "arm": "A",
            "target": target.tolist(),
            "documented_failure_type": "weld_attach_failed_after_300_frames (GRIP/weld-mechanism failure downstream of waypoint 1, NOT an IK convergence failure)",
            "residuals_all_32": [round(r, 5) for r in residuals],
            "summary": summarize(residuals),
        }
    finally:
        env.close()


def diag_place_water_bottle_destination() -> dict:
    """Root cause of skill #4 (place(A, water_bottle, table)). Documented
    failure type: genuine convergence failure (waypoint 1 (approach
    destination) IK residual=0.0150 m >= 0.01 m tol). Entry state: pick(A,
    bottle) run for real first (matches run_place's own internal nested
    pick), exactly mirroring probe_multiseed_diagnostic.py's target_a.
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    try:
        executor = ScriptedSkillExecutor()
        body_id = sk._body_id(env.model, "water_bottle")
        pick_call = SkillCall(skill="pick", arm="A", target_object="bottle", params={})
        pick_result = executor.execute(pick_call, env)

        obj_xy = np.array(env.data.xpos[body_id][:2], dtype=np.float64, copy=True)
        dest_xy = obj_xy + np.array(sk.PLACE_OFFSET_XY_M)
        dest_xy[0] = float(np.clip(dest_xy[0], -0.30, 0.30))
        dest_xy[1] = float(np.clip(dest_xy[1], -0.18, 0.18))
        target = np.array([dest_xy[0], dest_xy[1], sk.TABLE_SURFACE_Z + sk.CLEARANCE_HEIGHT_M])

        residuals = perturbed_residuals(env.model, env.data, "A", target)
        return {
            "target_label": "place(A, water_bottle, table) destination approach (root cause of #4)",
            "arm": "A",
            "pick_success": bool(pick_result.success),
            "target": target.tolist(),
            "documented_failure_type": "convergence (IK residual=0.0150 m >= 0.01 m) at waypoint 1 (approach destination)",
            "residuals_all_32": [round(r, 5) for r in residuals],
            "summary": summarize(residuals),
        }
    finally:
        env.close()


def diag_handoff_b_to_a_fork_phase1() -> dict:
    """Root cause of skill #8 (handoff(B->A, fork)). handoff(B->A, ...) is
    run_handoff(env, to_arm='A', from_arm='B', 'fork') -- Phase 1 is
    run_pick(env, 'B', 'fork', ...), i.e. arm B (NOT fork's usual arm A)
    reaching across for the fork. Documented failure type: near-miss
    COLLISION (arm-vs-prop: plate, dist=-0.0053 m; threshold=-0.005 m) at
    waypoint 1 -- only 0.3 mm past the gate.
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    try:
        target = hover_target(env, "fork", "B")
        residuals = perturbed_residuals(env.model, env.data, "B", target)
        return {
            "target_label": "handoff(B->A, fork) phase 1 (arm B picks fork) waypoint-1 approach (root cause of #8)",
            "arm": "B",
            "target": target.tolist(),
            "documented_failure_type": "near-miss collision (arm-vs-prop: plate, dist=-0.0053 m; threshold=-0.005 m) at waypoint 1",
            "residuals_all_32": [round(r, 5) for r in residuals],
            "summary": summarize(residuals),
        }
    finally:
        env.close()


def main() -> None:
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    results = {}
    for name, fn in [
        ("pick_A_fork_waypoint1", diag_pick_fork_waypoint1),
        ("pick_A_mug_waypoint1", diag_pick_mug_waypoint1),
        ("place_A_water_bottle_destination", diag_place_water_bottle_destination),
        ("handoff_B_to_A_fork_phase1", diag_handoff_b_to_a_fork_phase1),
    ]:
        print("=" * 70, flush=True)
        print(f"RUNNING: {name}", flush=True)
        r = fn()
        results[name] = r
        OUT_PATH.write_text(json.dumps(results, indent=2, default=str), newline="\n", encoding="utf-8")
        print(json.dumps(r, indent=2, default=str), flush=True)

    print(f"\nFull results: {OUT_PATH}")


if __name__ == "__main__":
    main()
