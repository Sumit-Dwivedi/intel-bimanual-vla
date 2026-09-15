"""Redesign Stage 2, Step 4: the verification gate (ADR-059).

Runs all five gate checks the task requires against the CURRENTLY generated
`so101_dual_table.xml`, from the actual "home" keyframe applied via
`TableSettingEnv.reset()` (not a hand-built qpos vector), and prints a clear
PASS/FAIL per item plus the underlying numbers -- no filter that has been
distrusted elsewhere in this repo (ADR-058) is used: contacts are read
directly from `data.contact`.

    1. Model compiles (already proven by the caller importing TableSettingEnv
       successfully -- re-confirmed here too).
    2. Home keyframe: zero contacts >1 mm, by enumerating `data.contact`.
    3. Every prop's grasp point: IK residual < 0.005 m from home, for its
       assigned grasping arm (PROP_ARM, same assignment as
       `search_home_keyframe.py`).
    4. Handoff point: IK residual < 0.005 m for BOTH arms, each seeded from
       its OWN approach pose (hover above the handoff point at clearance
       height, itself solved from home) -- NOT solved directly from home.
    5. Both arms' handoff-converged configurations applied SIMULTANEOUSLY via
       mj_forward: zero cross-arm contact with depth below -0.005 m.
"""
from __future__ import annotations

import json
import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control.skills_scripted import (  # noqa: E402
    GRASP_POINT_OFFSET_M, HANDOFF_POSITION_XYZ, CLEARANCE_HEIGHT_M,
)
from bimanual.sim.env import TableSettingEnv  # noqa: E402

IK_TOL_M = 0.005
CROSS_ARM_DEPTH_TOL_M = 0.005
TABLE_PEN_TOL_M = 0.001

PROP_ARM = {"plate": "A", "mug": "B", "fork": "A", "spoon": "B", "bottle": "A"}
PROP_BODY = {"plate": "plate", "mug": "mug", "fork": "fork", "spoon": "spoon", "bottle": "water_bottle"}


def _body_id(model, name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found")
    return bid


def _arm_geom_ids(model, arm):
    prefix = f"arm{arm}_"
    body_ids = {b for b in range(model.nbody)
                if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(prefix)}
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}


def main() -> int:
    scene_path = REPO_ROOT / "src" / "bimanual" / "sim" / "assets" / "so101_dual_table.xml"
    report = {}

    # ---- Item 1: compiles ----
    env = TableSettingEnv(scene_path=scene_path, cameras=None)
    env.reset(seed=0)
    model, data = env.model, env.data
    report["item1_compiles"] = True
    print("[1] model compiles: PASS")

    arm_a_geoms = _arm_geom_ids(model, "A")
    arm_b_geoms = _arm_geom_ids(model, "B")
    table_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")

    # ---- Item 2: home keyframe contacts >1mm ----
    violations = []
    for i in range(data.ncon):
        c = data.contact[i]
        if float(c.dist) < -TABLE_PEN_TOL_M:
            violations.append((int(c.geom1), int(c.geom2), float(c.dist)))
    item2_pass = len(violations) == 0
    report["item2_home_contacts_gt_1mm"] = violations
    print(f"[2] home keyframe contacts >1mm: {'PASS' if item2_pass else 'FAIL'} "
          f"({len(violations)} found)")
    for g1, g2, d in violations[:10]:
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1)
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2)
        print(f"    {n1} <-> {n2}  dist={d:.5f}")

    # ---- Item 3: every prop grasp point, IK residual <0.005 from home ----
    item3_results = {}
    for prop, arm in PROP_ARM.items():
        bid = _body_id(model, PROP_BODY[prop])
        obj_pos0 = np.array(data.xpos[bid], dtype=np.float64, copy=True)
        offset = GRASP_POINT_OFFSET_M.get(prop, np.zeros(3))
        target = obj_pos0 + offset
        sol = ik.solve_position_ik(model, data, arm, target, tol=IK_TOL_M)
        item3_results[prop] = {"arm": arm, "residual_m": sol.position_error_m,
                                "pass": bool(sol.position_error_m < IK_TOL_M)}
    item3_pass = all(r["pass"] for r in item3_results.values())
    report["item3_prop_grasp_ik"] = item3_results
    print(f"[3] prop grasp-point IK <0.005 from home: {'PASS' if item3_pass else 'FAIL'}")
    for prop, r in item3_results.items():
        print(f"    {prop:8s} arm {r['arm']}: residual={r['residual_m']:.5f} {'OK' if r['pass'] else 'FAIL'}")

    # ---- Item 4: handoff point, both arms, seeded from EACH ARM'S OWN
    # approach pose (hover above the handoff point, itself solved from home)
    # -- not solved directly from home. ----
    handoff_target = np.array(HANDOFF_POSITION_XYZ, dtype=np.float64)
    hover_target = handoff_target + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    item4_results = {}
    converged_qpos = {}
    for arm in ("A", "B"):
        # Stage A: home -> hover (the approach waypoint every staged skill
        # in skills_scripted.py uses before descending onto a target).
        hover_sol = ik.solve_position_ik(model, data, arm, hover_target, tol=IK_TOL_M)
        approach_data = mujoco.MjData(model)
        approach_data.qpos[:] = data.qpos
        jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ik.arm_joint_names(arm)]
        qpos_adrs = [int(model.jnt_qposadr[j]) for j in jids]
        for k, qadr in enumerate(qpos_adrs):
            approach_data.qpos[qadr] = hover_sol.joint_angles[k]
        mujoco.mj_forward(model, approach_data)

        # Stage B: FROM the approach pose (not home), solve to the actual
        # handoff point.
        final_sol = ik.solve_position_ik(model, approach_data, arm, handoff_target, tol=IK_TOL_M)
        item4_results[arm] = {
            "hover_residual_m": hover_sol.position_error_m,
            "handoff_residual_m": final_sol.position_error_m,
            "pass": bool(final_sol.position_error_m < IK_TOL_M),
        }
        final_data = mujoco.MjData(model)
        final_data.qpos[:] = approach_data.qpos
        for k, qadr in enumerate(qpos_adrs):
            final_data.qpos[qadr] = final_sol.joint_angles[k]
        converged_qpos[arm] = (qpos_adrs, final_sol.joint_angles)
    item4_pass = all(r["pass"] for r in item4_results.values())
    report["item4_handoff_ik"] = item4_results
    print(f"[4] handoff-point IK <0.005, seeded from own approach pose: "
          f"{'PASS' if item4_pass else 'FAIL'}")
    for arm, r in item4_results.items():
        print(f"    arm {arm}: hover_residual={r['hover_residual_m']:.5f} "
              f"handoff_residual={r['handoff_residual_m']:.5f} {'OK' if r['pass'] else 'FAIL'}")

    # ---- Item 5: both arms' handoff-converged configs applied
    # SIMULTANEOUSLY -> zero cross-arm contact below -0.005 m. ----
    combined = mujoco.MjData(model)
    combined.qpos[:] = data.qpos
    for arm in ("A", "B"):
        qpos_adrs, angles = converged_qpos[arm]
        for k, qadr in enumerate(qpos_adrs):
            combined.qpos[qadr] = angles[k]
    combined.qvel[:] = 0.0
    mujoco.mj_forward(model, combined)
    cross_arm_violations = []
    for i in range(combined.ncon):
        c = combined.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        d = float(c.dist)
        if ((g1 in arm_a_geoms and g2 in arm_b_geoms) or (g1 in arm_b_geoms and g2 in arm_a_geoms)) and d < -CROSS_ARM_DEPTH_TOL_M:
            n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1)
            n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2)
            cross_arm_violations.append({"geom1": n1, "geom2": n2, "dist": d})
    item5_pass = len(cross_arm_violations) == 0
    report["item5_cross_arm_at_handoff"] = cross_arm_violations
    print(f"[5] cross-arm contact at simultaneous handoff config, below -0.005m: "
          f"{'PASS' if item5_pass else 'FAIL'} ({len(cross_arm_violations)} violations)")
    for v in cross_arm_violations[:10]:
        print(f"    {v['geom1']} <-> {v['geom2']}  dist={v['dist']:.5f}")

    env.close()

    all_pass = item2_pass and item3_pass and item4_pass and item5_pass
    report["all_pass"] = all_pass
    out_path = REPO_ROOT / "out" / "stage2_gate_report.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2, default=str), newline="\n", encoding="utf-8")
    print("\n" + "=" * 70)
    print(f"OVERALL: {'ALL GATE ITEMS PASS' if all_pass else 'AT LEAST ONE GATE ITEM FAILED'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
