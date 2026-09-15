"""Redesign Stage 3, Step 0: home-pose robustness validation (ADR-060).

Stage 2's search (`scripts/search_home_keyframe.py`, ADR-059) stopped at the
FIRST fully-passing candidate out of a possible 800 -- candidate 1 -- which
proves the pose passes every criterion, but says nothing about whether it is
a typical passing configuration or a lucky draw near a boundary (only 2
candidates were ever scored). This script re-uses `search_home_keyframe.py`'s
OWN `evaluate_candidate` function (imported, never reimplemented -- so the
scoring criteria are byte-identical to Stage 2's) but does NOT stop at the
first pass: it scores ALL `--n-candidates` candidates from the SAME
reproducible random stream (same default seed, same distributions) and
reports the pass rate, the violation-count distribution, and where Stage 2's
adopted pose lands relative to the other passing candidates.

**Ranking metric for passing candidates.** Every passing candidate has
`n_violations == 0` by definition, so violation count alone cannot rank them
against each other. This script additionally sums the 7 per-target IK
residuals (5 props + handoff_A + handoff_B) as a secondary "margin" score --
lower total residual means the pose converges more comfortably away from the
0.005 m gate on every target, which is a reasonable, stated (not hidden)
proxy for "how comfortably passing" a configuration is. This is a NEW
composite score computed by this script; `evaluate_candidate` itself is
unmodified.

**Stage 2's pose is evaluated twice for robustness of the "rank" claim:**
(1) implicitly, wherever it naturally falls in this run's candidate index 1
(same seed, same per-call `rng.uniform(low, high, size=n)` semantics as
`search_home_keyframe.py`, so index 1 reproduces Stage 2's own candidate 1
byte-for-byte as long as n>=2); (2) explicitly, by re-evaluating the exact
angles ADR-059 recorded as the winner, independent of any assumption about
where they land in this run's random stream. Both are reported so the
"where Stage 2's pose ranks" answer does not depend on a fragile assumption
about numpy's RNG internals.

Read-only with respect to every frozen module (`skills_scripted.py`,
`grasp.py`, `ik.py`, `executor.py`, `env.py`) and with respect to
`search_home_keyframe.py` itself -- imported, never edited.

Run on bm-ptl:
    python scripts/validate_home_pose_stage3.py --scene <path> --n-candidates 200
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
from bimanual.sim.env import TableSettingEnv  # noqa: E402

# Import Stage 2's own scoring function and helpers verbatim -- not
# reimplemented -- so this script's criteria are provably identical.
sys.path.insert(0, str(REPO_ROOT / "scripts"))
from search_home_keyframe import (  # noqa: E402
    evaluate_candidate,
    _arm_geom_ids,
    _prop_geom_ids,
    _table_geom_id,
    _body_id,
    PROP_ARM,
    PROP_BODY,
)
from bimanual.control.skills_scripted import GRASP_POINT_OFFSET_M, HANDOFF_POSITION_XYZ  # noqa: E402

# ADR-059's recorded winner (search_home_keyframe.py candidate 1 of 2 tried).
STAGE2_POSE = {
    "shoulder_pan": 0.05402,
    "shoulder_lift": -1.48130,
    "elbow_flex": -0.46647,
    "wrist_flex": 0.09897,
    "wrist_roll": 0.17687,
}
# Higher-precision values, straight from scripts/gen_dual_scene.py's own
# HOME_* constants (what is ACTUALLY baked into the compiled "home"
# keyframe) -- used for the explicit re-evaluation so it matches the
# compiled scene exactly, not the report's rounded display values.
STAGE2_POSE_PRECISE = [
    0.054022898001139796,
    -1.4813027413443594,
    -0.46647495231644354,
    0.09897040413436309,
    0.17687383568459125,
]


def total_ik_residual(ik_results: dict) -> float:
    return float(sum(r["residual_m"] for r in ik_results.values()))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=str, required=True)
    parser.add_argument("--n-candidates", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260915)  # same default as search_home_keyframe.py
    parser.add_argument("--out", type=str,
                         default=str(REPO_ROOT / "out" / "home_pose_stage3_validation.json"))
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

    # ---- Candidate generation: IDENTICAL to search_home_keyframe.py's main() ----
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
    cand_sp[0], cand_sl[0], cand_ef[0], cand_wf[0], cand_wr[0] = 0.0, -1.2, -1.6, 0.0, 0.0

    all_results = []
    for i in range(n):
        angles = np.array([cand_sp[i], cand_sl[i], cand_ef[i], cand_wf[i], cand_wr[i]])
        result = evaluate_candidate(
            model, base_qpos, arm_a_geoms, arm_b_geoms, prop_geoms, table_gid,
            joint_qpos_adrs, angles, prop_targets,
        )
        result["index"] = i
        result["total_ik_residual_m"] = total_ik_residual(result["ik_results"])
        all_results.append(result)
        if (i + 1) % 50 == 0:
            print(f"  scored {i + 1}/{n}", flush=True)

    # ---- Explicit Stage 2 pose re-evaluation (independent of RNG-stream position) ----
    stage2_angles = np.array(STAGE2_POSE_PRECISE)
    stage2_result = evaluate_candidate(
        model, base_qpos, arm_a_geoms, arm_b_geoms, prop_geoms, table_gid,
        joint_qpos_adrs, stage2_angles, prop_targets,
    )
    stage2_result["total_ik_residual_m"] = total_ik_residual(stage2_result["ik_results"])

    # Does candidate index 1 (this run's first random draw) match Stage 2's
    # pose, as expected if the RNG-stream-position argument in the module
    # docstring holds?
    idx1_matches_stage2 = bool(np.allclose(
        [cand_sp[1], cand_sl[1], cand_ef[1], cand_wf[1], cand_wr[1]],
        stage2_angles, atol=1e-4,
    ))

    env.close()

    # ---- Summary statistics ----
    n_violations_arr = np.array([r["n_violations"] for r in all_results])
    n_pass = int(np.sum(n_violations_arr == 0))
    pass_rate = n_pass / n

    passing = [r for r in all_results if r["passes"]]
    passing_residuals = sorted(r["total_ik_residual_m"] for r in passing)

    # Rank of Stage 2's pose among the PASSING candidates by total_ik_residual_m
    # (rank 1 = lowest residual = most comfortably passing). Only meaningful
    # if stage2_result itself passes.
    rank_info = {"stage2_passes": bool(stage2_result["passes"])}
    if stage2_result["passes"] and passing_residuals:
        # rank = 1 + count of passing candidates with a strictly better (lower) score
        better = sum(1 for v in passing_residuals if v < stage2_result["total_ik_residual_m"])
        rank_info["rank_among_passing"] = better + 1
        rank_info["n_passing_total"] = len(passing_residuals)
        # top-quartile check: is stage2 residual <= the 25th-percentile
        # threshold of the passing-candidate residual distribution?
        q1_threshold = float(np.percentile(passing_residuals, 25))
        rank_info["top_quartile_threshold_m"] = q1_threshold
        rank_info["in_top_quartile"] = bool(stage2_result["total_ik_residual_m"] <= q1_threshold)
        rank_info["best_passing_residual_m"] = passing_residuals[0]
        rank_info["stage2_residual_m"] = stage2_result["total_ik_residual_m"]
        rank_info["materially_better_exists"] = bool(
            passing_residuals[0] < 0.5 * stage2_result["total_ik_residual_m"]
        )

    violation_histogram = {}
    for v in n_violations_arr:
        violation_histogram[str(int(v))] = violation_histogram.get(str(int(v)), 0) + 1

    summary = {
        "n_candidates_tried": n,
        "n_pass": n_pass,
        "pass_rate": pass_rate,
        "near_degenerate_lt_5pct": bool(pass_rate < 0.05),
        "n_violations_histogram": violation_histogram,
        "n_violations_min": int(n_violations_arr.min()),
        "n_violations_max": int(n_violations_arr.max()),
        "n_violations_mean": float(n_violations_arr.mean()),
        "n_violations_median": float(np.median(n_violations_arr)),
        "passing_total_ik_residual_stats_m": {
            "min": float(min(passing_residuals)) if passing_residuals else None,
            "max": float(max(passing_residuals)) if passing_residuals else None,
            "mean": float(np.mean(passing_residuals)) if passing_residuals else None,
            "median": float(np.median(passing_residuals)) if passing_residuals else None,
        },
        "idx1_matches_stage2_pose": idx1_matches_stage2,
        "stage2_pose_explicit_reeval": {
            "angles": stage2_result["angles"],
            "n_violations": stage2_result["n_violations"],
            "passes": stage2_result["passes"],
            "total_ik_residual_m": stage2_result["total_ik_residual_m"],
            "ik_results": stage2_result["ik_results"],
        },
        "stage2_pose_rank": rank_info,
    }

    out = {"summary": summary, "all_results": all_results}
    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2, default=str), newline="\n", encoding="utf-8")

    print("=" * 70)
    print(json.dumps(summary, indent=2, default=str))
    print(f"\nFull results: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
