"""Redesign Stage 4, Step 3 (remedy for categories c/d): re-run the home-pose
search with SWEPT-PATH clearance added to the scoring, not just the static
contact/IK checks `search_home_keyframe.py::evaluate_candidate` already
does (ADR-061).

Why: Step 2's diagnosis found `c_arm_vs_arm` (7 colliding waypoints) and
`d_self_collision` (6) concentrated at the FIRST hop of a skill's approach
(home -> hover), for MULTIPLE null-space branches (`num_endpoint_seeds`) --
i.e. the home fold itself leaves too little margin for the redundant
solver's drift, even though ADR-060's own static-endpoint check
(`evaluate_candidate`) already passed it. This script imports
`evaluate_candidate` UNCHANGED (ADR-060's own precedent: "reusing Stage 2's
own evaluate_candidate, not reimplemented") and adds one MORE criterion per
candidate: `swept_path_clear` from that candidate's home pose to every
prop's grasp point and both arms' handoff point, at a REDUCED
(search-speed) fidelity (`num_endpoint_seeds=4`, `n_steps=15`) -- the
winning candidate is re-verified at full fidelity
(`num_endpoint_seeds=8`, `n_steps=25`, matching Step 1's validated
settings) before being reported.

Run on bm-ptl:
    python scripts/search_home_keyframe_stage4.py --scene <path> --n-candidates 400
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import mujoco  # noqa: E402

from check_swept_path import swept_path_clear, PROP_GEOMS_BY_BODY  # noqa: E402
from search_home_keyframe import (  # noqa: E402
    PROP_ARM, PROP_BODY, evaluate_candidate, _arm_geom_ids, _prop_geom_ids, _table_geom_id, _body_id,
)
from bimanual.control import ik  # noqa: E402
from bimanual.control.skills_scripted import GRASP_POINT_OFFSET_M, HANDOFF_POSITION_XYZ  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

IK_TOL_M = 0.005


def _swept_score(model, data_qpos, joint_qpos_adrs, angles, prop_targets, num_endpoint_seeds, n_steps):
    """Apply `angles` to both arms on top of `data_qpos` (the candidate home
    pose), then run `swept_path_clear` from THAT pose to every named target
    (home -> first hover of every skill's approach, the concentration point
    Step 2 found). Returns (n_swept_violations, detail).

    `target_geom_names` (per-prop, from `PROP_GEOMS_BY_BODY`) is passed so
    the gate applies `skills_scripted.CRUSH_THRESHOLD_M` to the swept arm's
    OWN declared target exactly as the real controller does, instead of the
    tight bystander bar -- see `check_swept_path._effective_tol`'s
    docstring. The two `handoff_*` targets are a point in space, not a
    prop, so they get `None` (no target-prop exemption -- a handoff transfer
    point has no "object" to legitimately touch).
    """
    base = np.array(data_qpos, dtype=np.float64, copy=True)
    for arm in ("A", "B"):
        for k, qadr in enumerate(joint_qpos_adrs[arm]):
            base[qadr] = angles[k]

    n_violations = 0
    detail = {}
    for name, (arm, target) in prop_targets.items():
        body_name = PROP_BODY.get(name)
        target_geom_names = PROP_GEOMS_BY_BODY.get(body_name)
        clear, worst_pen, worst_step, contacts = swept_path_clear(
            model, None, arm, base, target,
            n_steps=n_steps, num_endpoint_seeds=num_endpoint_seeds,
            target_geom_names=target_geom_names,
        )
        detail[name] = {"clear": clear, "worst_penetration_m": worst_pen}
        if not clear:
            n_violations += 1
    return n_violations, detail


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=str, required=True)
    parser.add_argument("--n-candidates", type=int, default=400)
    parser.add_argument("--seed", type=int, default=20260915)
    parser.add_argument("--out", type=str, default=str(REPO_ROOT / "out" / "stage4_home_keyframe_search.json"))
    args = parser.parse_args()

    env = TableSettingEnv(scene_path=pathlib.Path(args.scene), cameras=None)
    env.reset(seed=0)
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

    prop_targets = {}
    for prop, arm in PROP_ARM.items():
        body_name = PROP_BODY[prop]
        bid = _body_id(model, body_name)
        obj_pos0 = np.array(env.data.xpos[bid], dtype=np.float64, copy=True)
        offset = GRASP_POINT_OFFSET_M.get(prop, np.zeros(3))
        prop_targets[prop] = (arm, obj_pos0 + offset)
    prop_targets["handoff_A"] = ("A", np.array(HANDOFF_POSITION_XYZ, dtype=np.float64))
    prop_targets["handoff_B"] = ("B", np.array(HANDOFF_POSITION_XYZ, dtype=np.float64))

    lo_sp, hi_sp = joint_ranges["A"][0]
    lo_sl, hi_sl = joint_ranges["A"][1]
    lo_ef, hi_ef = joint_ranges["A"][2]
    lo_wf, hi_wf = joint_ranges["A"][3]
    lo_wr, hi_wr = joint_ranges["A"][4]

    rng = np.random.default_rng(args.seed)
    n = args.n_candidates
    cand_sp = rng.uniform(-0.3, 0.3, size=n)
    cand_sl = rng.uniform(lo_sl, 0.0, size=n)
    cand_ef = rng.uniform(lo_ef, 0.0, size=n)
    cand_wf = rng.uniform(-0.3, 0.3, size=n)
    cand_wr = rng.uniform(-0.3, 0.3, size=n)
    # ADR-060's own current winner included as candidate 0 -- if it already
    # clears the NEW swept-path criterion too, the search should say so
    # immediately rather than by luck.
    cand_sp[0], cand_sl[0], cand_ef[0], cand_wf[0], cand_wr[0] = (
        0.15752951354904826, -0.9930387740421327, -0.9041594728051238,
        0.057312372361992936, -0.24665855564066247,
    )

    scored = []
    for i in range(n):
        angles = np.array([cand_sp[i], cand_sl[i], cand_ef[i], cand_wf[i], cand_wr[i]])
        static_result = evaluate_candidate(
            model, base_qpos, arm_a_geoms, arm_b_geoms, prop_geoms, table_gid,
            joint_qpos_adrs, angles, prop_targets,
        )
        if not static_result["passes"]:
            continue  # must still pass every ADR-059/060 static criterion
        n_swept_violations, swept_detail = _swept_score(
            model, base_qpos, joint_qpos_adrs, angles, prop_targets,
            num_endpoint_seeds=4, n_steps=15,
        )
        scored.append({
            "candidate_index": i,
            "angles": [float(a) for a in angles],
            "static_result": static_result,
            "n_swept_violations": n_swept_violations,
            "swept_detail": swept_detail,
        })
        if (i + 1) % 50 == 0:
            best_so_far = min((s["n_swept_violations"] for s in scored), default=None)
            print(f"  tried {i+1}/{n}, static-pass candidates so far: {len(scored)}, "
                  f"best swept-violations so far: {best_so_far}", flush=True)

    env.close()

    if not scored:
        print("NO candidate passed even the STATIC (ADR-060) criteria -- this would be a genuine "
              "regression from Stage 3's own home pose, which itself passed those. Not expected; "
              "reports empty result.")
        out = {"n_candidates_tried": n, "static_pass_count": 0, "winner": None}
        pathlib.Path(args.out).write_text(json.dumps(out, indent=2), newline="\n", encoding="utf-8")
        return 1

    scored.sort(key=lambda s: s["n_swept_violations"])
    score_distribution = {}
    for s in scored:
        score_distribution[s["n_swept_violations"]] = score_distribution.get(s["n_swept_violations"], 0) + 1

    # The reduced (search-fidelity) score often ties several candidates --
    # re-verify the TOP-K at FULL fidelity (Step 1's validated settings,
    # num_endpoint_seeds=8/n_steps=25) and keep whichever of THOSE actually
    # scores best there, rather than trusting the reduced-fidelity ranking's
    # arbitrary tie-break order.
    top_k = scored[:10]
    full_scored = []
    for s in top_k:
        angles = np.array(s["angles"])
        n_full_violations, full_detail = _swept_score(
            model, base_qpos, joint_qpos_adrs, angles, prop_targets,
            num_endpoint_seeds=8, n_steps=25,
        )
        full_scored.append({**s, "n_full_violations": n_full_violations, "full_detail": full_detail})
        print(f"  full-fidelity re-verify candidate_index={s['candidate_index']}: "
              f"{n_full_violations} violations", flush=True)
    full_scored.sort(key=lambda s: s["n_full_violations"])
    best = full_scored[0]
    n_full_violations = best["n_full_violations"]
    full_detail = best["full_detail"]

    out = {
        "n_candidates_tried": n,
        "static_pass_count": len(scored),
        "score_distribution_by_n_swept_violations": score_distribution,
        "winner": best,
        "winner_full_fidelity_violations": n_full_violations,
        "winner_full_fidelity_detail": full_detail,
    }
    pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    pathlib.Path(args.out).write_text(json.dumps(out, indent=2, default=str), newline="\n", encoding="utf-8")

    print("=" * 70)
    print(f"{len(scored)}/{n} candidates passed the STATIC (ADR-059/060) criteria.")
    print(f"Score distribution (n_swept_violations -> count of candidates): {score_distribution}")
    print(f"BEST candidate index={best['candidate_index']}, angles={best['angles']}, "
          f"search-fidelity swept_violations={best['n_swept_violations']}")
    print(f"Full-fidelity re-verification: {n_full_violations} swept violations (out of "
          f"{len(prop_targets)} targets)")
    print(json.dumps(full_detail, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
