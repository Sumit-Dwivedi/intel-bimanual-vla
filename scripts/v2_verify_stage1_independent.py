"""Orchestrator's INDEPENDENT verification of Stage 1's IK.

Deliberately does NOT reuse scripts/v2_validate_ik.py. Different RNG seed,
own target generation, own pinch-point read, and one check the builder's
script does not make:

  The builder defined the tool approach axis as (pinch - wrist_flex_anchor)
  and reported min dot 0.998258 with (0,0,-1). That vector includes the
  ADR-025 lateral pinch offset, so it CANNOT be exactly vertical even for a
  perfect solve. The question that actually matters for grasping is whether
  the GRIPPER BODY's own frame points down. This script reports both, so the
  0.998 can be attributed to either a definitional artifact or a real tilt.
"""
import pathlib
import sys

REPO = pathlib.Path.cwd()
assert (REPO / "src").is_dir(), "run from the repo root"

sys.path.insert(0, str(REPO / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik_geometric as ikg  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

PINCH = ("arm{a}_gripper", "arm{a}_moving_jaw_so101_v1")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def bid(model, name):
    i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert i >= 0, name
    return i


def main():
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model = env.model
    scratch = mujoco.MjData(model)

    rng = np.random.default_rng(777)  # different from the builder's 20260915
    results = {}

    for arm in ("A", "B"):
        jids = []
        for j in JOINTS:
            k = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "arm%s_%s" % (arm, j))
            assert k >= 0
            jids.append(k)
        adrs = [model.jnt_qposadr[k] for k in jids]
        lims = np.array([model.jnt_range[k] for k in jids])

        g_id = bid(model, PINCH[0].format(a=arm))
        j_id = bid(model, PINCH[1].format(a=arm))
        wf_id = jids[3]

        errs, dots_wp, dots_body, lim_viol = [], [], [], 0
        n = 0
        tried = 0
        while n < 400 and tried < 200000:
            tried += 1
            tgt = np.array([rng.uniform(-0.25, 0.25),
                            rng.uniform(-0.45, 0.45),
                            rng.uniform(0.34, 0.62)])
            sol = ikg.solve_topdown_ik(model, env.data, arm, tgt, yaw=0.0)
            if sol is None:
                continue
            sol = np.asarray(sol, dtype=float)
            if sol.shape != (5,) or not np.all(np.isfinite(sol)):
                continue

            # limits, checked by me not by the solver
            if np.any(sol < lims[:, 0] - 1e-9) or np.any(sol > lims[:, 1] + 1e-9):
                lim_viol += 1

            mujoco.mj_resetData(model, scratch)
            for a, q in zip(adrs, sol):
                scratch.qpos[a] = q
            mujoco.mj_forward(model, scratch)

            pinch = 0.5 * (scratch.xpos[g_id] + scratch.xpos[j_id])
            errs.append(float(np.linalg.norm(pinch - tgt)))

            wf = np.array(scratch.xanchor[wf_id])
            v = pinch - wf
            dots_wp.append(float(v @ np.array([0, 0, -1.0]) / np.linalg.norm(v)))

            # the gripper BODY's own z-axis in world frame
            R = scratch.xmat[g_id].reshape(3, 3)
            zax = R[:, 2]
            dots_body.append(float(zax @ np.array([0, 0, -1.0])))
            n += 1

        errs = np.array(errs)
        results[arm] = dict(n=n, tried=tried, mean=errs.mean(), max=errs.max(),
                            dot_wp_min=min(dots_wp), dot_body_min=min(dots_body),
                            dot_body_mean=float(np.mean(dots_body)),
                            dot_body_max=max(dots_body), lim_viol=lim_viol)

    print("=" * 72)
    print("INDEPENDENT STAGE 1 VERIFICATION (rng seed 777, own readback)")
    print("=" * 72)
    for arm, r in results.items():
        print("arm %s: solved %d of %d sampled" % (arm, r["n"], r["tried"]))
        print("   round-trip mean = %.3e m   max = %.3e m" % (r["mean"], r["max"]))
        print("   GATE mean<2mm max<5mm : %s"
              % ("PASS" if r["mean"] < 2e-3 and r["max"] < 5e-3 else "FAIL"))
        print("   dot(wrist->pinch, down) min  = %.6f   [builder's definition]"
              % r["dot_wp_min"])
        print("   dot(gripper body z, down)    min=%.6f mean=%.6f max=%.6f"
              % (r["dot_body_min"], r["dot_body_mean"], r["dot_body_max"]))
        print("   joint-limit violations in returned solutions: %d" % r["lim_viol"])
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
