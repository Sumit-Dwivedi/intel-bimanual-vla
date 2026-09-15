"""Redesign Stage 2, Step 3: find a new "home" fold by SEARCH, not by hand
(ADR-059, superseding ADR-025/ADR-026's hand-derived fold).

Why a search, not another hand-picked sign combination (ADR-026's own
method): base separation changed in Step 1/2, so ADR-026's
(shoulder_lift=-1.2, elbow_flex=-1.6) fold -- measured zero-collision only
for the OLD 0.50 m separation -- has no reason to still be the best fold
(or even a safe one) at the new separation. This script samples candidate
folded configurations (5 joint angles, applied identically to both arms,
matching ADR-026's own finding that identical-not-mirrored signs are the
symmetric, collision-free family) and scores each against every criterion
the task requires:

  1. Zero self-collision (contacts between two geoms of the SAME arm).
  2. Zero cross-arm contact (contacts between an armA_* geom and an armB_*
     geom).
  3. Zero table penetration beyond 1 mm, measured by DIRECTLY enumerating
     `data.contact` (ADR-058's own lesson: the mesh-convex-hull filter used
     elsewhere in this repo is not trusted -- this script never imports or
     reuses it, it reads `data.contact[i].dist` itself).
  4. Arms folded clear of the prop region: zero contact between any arm
     geom and any prop body's geoms.
  5. IK from this candidate home pose to every prop's grasp point, and to
     the handoff point (both arms, each seeded from ITS OWN candidate home
     state -- `solve_position_ik` always starts from `data`'s current
     qpos), converges with position error < 0.005 m.

A candidate PASSES only if all five hold simultaneously. Candidates are
tried in a fixed, reproducible order (numpy `default_rng` with a fixed
seed) up to `--n-candidates`. The first fully-passing candidate is
returned; if none passes, the candidate with the fewest total violations is
reported as the best-effort result, with every violation named -- not
silently accepted as a pass.

Run on bm-ptl (imports mujoco via TableSettingEnv):
    python scripts/search_home_keyframe.py --scene <path> --n-candidates 800
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control.skills_scripted import GRASP_POINT_OFFSET_M, HANDOFF_POSITION_XYZ  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

IK_TOL_M = 0.005

#: Which arm's grasp is scored for each prop, per the brief's own worked
#: example command (PLAN.md M05: "pick up the plate with arm A ... pick up
#: the mug with arm B") plus the two props already exercised by the live
#: regression gate (`scripts/verify_adr038_skills.py`: fork and the water
#: bottle are both picked by arm A; fork is also the handoff object).
#: spoon has no precedent in either source, so it is assigned to arm B by
#: symmetry with mug (both "arm B" props sit on the same side of the
#: worked example). This assignment is a documented assumption of THIS
#: script, not something read from the (non-existent, in this cut-down
#: submission) TaskPlan grounder output for these specific props.
PROP_ARM = {
    "plate": "A",
    "mug": "B",
    "fork": "A",
    "spoon": "B",
    "bottle": "A",
}
PROP_BODY = {
    "plate": "plate",
    "mug": "mug",
    "fork": "fork",
    "spoon": "spoon",
    "bottle": "water_bottle",
}


def _body_id(model, name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found")
    return bid


def _arm_geom_ids(model, arm):
    prefix = f"arm{arm}_"
    body_ids = set()
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name and name.startswith(prefix):
            body_ids.add(b)
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}


def _prop_geom_ids(model):
    out = set()
    for body_name in PROP_BODY.values():
        bid = _body_id(model, body_name)
        out |= {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == bid}
    return out


def _table_geom_id(model):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    if gid == -1:
        raise ValueError("geom 'table_top' not found")
    return gid


def evaluate_candidate(model, base_qpos, arm_a_geoms, arm_b_geoms, prop_geoms, table_gid,
                        joint_qpos_adrs, angles, prop_targets):
    """Apply `angles` (5 values: shoulder_pan, shoulder_lift, elbow_flex,
    wrist_flex, wrist_roll) identically to both arms on top of
    `base_qpos` (drawer + props already placed, gripper open), run
    mj_forward, and score every criterion. Returns a dict with booleans per
    criterion, the raw violation list, and per-target IK residuals.
    """
    data = mujoco.MjData(model)
    data.qpos[:] = base_qpos
    for arm in ("A", "B"):
        for k, qadr in enumerate(joint_qpos_adrs[arm]):
            data.qpos[qadr] = angles[k]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    self_collisions = []
    cross_arm = []
    table_pen = []
    prop_contact = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        d = float(c.dist)
        in_a1, in_a2 = g1 in arm_a_geoms, g2 in arm_a_geoms
        in_b1, in_b2 = g1 in arm_b_geoms, g2 in arm_b_geoms
        if (in_a1 and in_a2) or (in_b1 and in_b2):
            self_collisions.append((g1, g2, d))
        if (in_a1 and in_b2) or (in_a2 and in_b1):
            cross_arm.append((g1, g2, d))
        if g1 == table_gid or g2 == table_gid:
            other = g2 if g1 == table_gid else g1
            if (other in arm_a_geoms or other in arm_b_geoms) and d < -0.001:
                table_pen.append((g1, g2, d))
        if (g1 in prop_geoms and (g2 in arm_a_geoms or g2 in arm_b_geoms)) or \
           (g2 in prop_geoms and (g1 in arm_a_geoms or g1 in arm_b_geoms)):
            prop_contact.append((g1, g2, d))

    ik_results = {}
    for name, (arm, target) in prop_targets.items():
        sol = ik.solve_position_ik(model, data, arm, target, tol=IK_TOL_M)
        ik_results[name] = {
            "arm": arm,
            "residual_m": sol.position_error_m,
            "converged_lt_0p005": bool(sol.position_error_m < IK_TOL_M),
        }

    n_violations = (
        len(self_collisions) + len(cross_arm) + len(table_pen) + len(prop_contact)
        + sum(1 for r in ik_results.values() if not r["converged_lt_0p005"])
    )
    return {
        "angles": [float(a) for a in angles],
        "n_self_collisions": len(self_collisions),
        "n_cross_arm": len(cross_arm),
        "n_table_pen": len(table_pen),
        "n_prop_contact": len(prop_contact),
        "ik_results": ik_results,
        "n_violations": n_violations,
        "passes": n_violations == 0,
        "contact_detail": {
            "self_collisions": self_collisions[:5],
            "cross_arm": cross_arm[:5],
            "table_pen": table_pen[:5],
            "prop_contact": prop_contact[:5],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=str, required=True)
    parser.add_argument("--n-candidates", type=int, default=800)
    parser.add_argument("--seed", type=int, default=20260915)
    parser.add_argument("--out", type=str, default=str(REPO_ROOT / "out" / "home_keyframe_search.json"))
    args = parser.parse_args()

    env = TableSettingEnv(scene_path=pathlib.Path(args.scene), cameras=None)
    env.reset(seed=0)  # applies whatever "home" keyframe currently exists (a placeholder we override below)
    model = env.model
    base_qpos = np.array(env.data.qpos, dtype=np.float64, copy=True)

    arm_a_geoms = _arm_geom_ids(model, "A")
    arm_b_geoms = _arm_geom_ids(model, "B")
    prop_geoms = _prop_geom_ids(model)
    table_gid = _table_geom_id(model)

    joint_qpos_adrs = {}
    joint_ranges = {}
    for arm in ("A", "B"):
        jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ik.arm_joint_names(arm)]
        joint_qpos_adrs[arm] = [int(model.jnt_qposadr[j]) for j in jids]
        joint_ranges[arm] = [tuple(model.jnt_range[j]) for j in jids]

    # Grasp-point targets: obj_pos (current, from base_qpos's prop layout) +
    # GRASP_POINT_OFFSET_M, exactly the formula skills_scripted.run_pick
    # uses (module docstring above). Handoff point checked for BOTH arms,
    # matching the current HANDOFF_POSITION_XYZ (this script does not
    # modify skills_scripted.py -- see the Stage 2 report's file-freeze
    # note for why that constant could not be moved to the new region
    # centroid in this pass).
    prop_targets = {}
    for prop, arm in PROP_ARM.items():
        body_name = PROP_BODY[prop]
        bid = _body_id(model, body_name)
        obj_pos0 = np.array(env.data.xpos[bid], dtype=np.float64, copy=True)
        offset = GRASP_POINT_OFFSET_M.get(prop, np.zeros(3))
        prop_targets[prop] = (arm, obj_pos0 + offset)
    prop_targets["handoff_A"] = ("A", np.array(HANDOFF_POSITION_XYZ, dtype=np.float64))
    prop_targets["handoff_B"] = ("B", np.array(HANDOFF_POSITION_XYZ, dtype=np.float64))

    # Candidate joint ranges to sample from: the arm's OWN compiled limits
    # for shoulder_pan/wrist_flex/wrist_roll, but biased toward the FOLDED
    # half of shoulder_lift/elbow_flex's range (ADR-026 measured that the
    # folded-back direction, not the extended one, is the collision-free
    # family) -- sampled from [range_lo, 0] for both, i.e. still a real
    # search over the whole folded half, not a single hand-picked point.
    lo_sp, hi_sp = joint_ranges["A"][0]
    lo_sl, hi_sl = joint_ranges["A"][1]
    lo_ef, hi_ef = joint_ranges["A"][2]
    lo_wf, hi_wf = joint_ranges["A"][3]
    lo_wr, hi_wr = joint_ranges["A"][4]

    rng = np.random.default_rng(args.seed)
    n = args.n_candidates
    cand_sp = rng.uniform(-0.3, 0.3, size=n)  # near-zero pan: face the shared band, don't twist away
    cand_sl = rng.uniform(lo_sl, 0.0, size=n)
    cand_ef = rng.uniform(lo_ef, 0.0, size=n)
    cand_wf = rng.uniform(-0.3, 0.3, size=n)
    cand_wr = rng.uniform(-0.3, 0.3, size=n)

    # ADR-026's own winning point is included as candidate 0 -- if the new
    # separation happens to leave it valid, the search should find that
    # immediately rather than by luck.
    cand_sp[0], cand_sl[0], cand_ef[0], cand_wf[0], cand_wr[0] = 0.0, -1.2, -1.6, 0.0, 0.0

    best = None
    winner = None
    tried = 0
    for i in range(n):
        angles = np.array([cand_sp[i], cand_sl[i], cand_ef[i], cand_wf[i], cand_wr[i]])
        result = evaluate_candidate(
            model, base_qpos, arm_a_geoms, arm_b_geoms, prop_geoms, table_gid,
            joint_qpos_adrs, angles, prop_targets,
        )
        tried += 1
        if best is None or result["n_violations"] < best["n_violations"]:
            best = result
        if result["passes"]:
            winner = result
            break
        if tried % 100 == 0:
            print(f"  tried {tried}/{n}, best violations so far: {best['n_violations']}", flush=True)

    env.close()

    out = {
        "n_candidates_tried": tried,
        "winner": winner,
        "best_effort": best,
        "prop_arm_assignment": PROP_ARM,
    }
    pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    pathlib.Path(args.out).write_text(json.dumps(out, indent=2), newline="\n", encoding="utf-8")

    print("=" * 70)
    if winner is not None:
        print(f"WINNER found after {tried} candidates: angles={winner['angles']}")
    else:
        print(f"NO fully-passing candidate found in {tried} tries. Best effort "
              f"({best['n_violations']} violations): angles={best['angles']}")
    print(json.dumps(out["winner"] or out["best_effort"], indent=2))
    return 0 if winner is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
