"""v2 Stage 1 pre-work: measure every kinematic constant the geometric
top-down IK needs, FROM THE MJCF, plus settle three claims in the v2 brief.

The brief is explicit that link lengths, the tool offset and the wrist_flex
constant must be measured rather than taken from a drawing. This script is
that measurement. It is read-only: it never writes to the scene and never
calls a skill.

It also answers three questions the brief assumes answers to:

  1. What is the physics timestep? This decides the render stride that
     preserves real motion speed. The brief says "every 6th physics step at
     30 fps so real motion speed is preserved" -- that is only true for one
     particular timestep, and this reports the actual one.

  2. Does the all-zeros pose (the brief's Stage 3 starting point, from Lab
     8) actually collide at 0.5 m separation? ADR-026 says it does, badly
     (34 contacts, 29 of them armA<->armB, deepest -0.0597 m) and that is
     WHY the current folded home keyframe exists. Re-measured here.

  3. What is the current home pose, numerically? The brief calls it an
     "extended pose"; the MJCF comment calls it folded back.

Run on bm-ptl only (ADR-020).
"""
from __future__ import annotations

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.sim.env import TableSettingEnv  # noqa: E402

# ADR-025: the pinch point is the midpoint of these two bodies. It is what
# IK must target -- NOT the armX_gripperframe site, which sits 0.0888 m away.
PINCH_BODIES = ("{a}_gripper", "{a}_moving_jaw_so101_v1")

POSITIONING_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                      "wrist_flex", "wrist_roll")


def body_xpos(model, data, name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError("no body named %r" % name)
    return np.array(data.xbody_xpos[bid] if hasattr(data, "xbody_xpos")
                    else data.xpos[bid])


def pinch_point(model, data, arm):
    ps = [body_xpos(model, data, b.format(a="arm" + arm)) for b in PINCH_BODIES]
    return np.mean(ps, axis=0)


def main() -> int:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model, data = env.model, env.data

    print("=" * 68)
    print("TIMESTEP AND RENDER STRIDE")
    print("=" * 68)
    dt = float(model.opt.timestep)
    print("model.opt.timestep = %.6f s" % dt)
    for stride in (6, 7, 17):
        sim_per_frame = stride * dt
        speed_at_30fps = sim_per_frame / (1.0 / 30.0)
        print("  stride %2d -> %.4f s sim/frame -> %.3fx real speed at 30 fps"
              % (stride, sim_per_frame, speed_at_30fps))
    print("  stride for TRUE real-time at 30 fps = %.2f" % ((1.0 / 30.0) / dt))

    print()
    print("=" * 68)
    print("CURRENT HOME KEYFRAME (what the brief calls the 'extended pose')")
    print("=" * 68)
    for arm in ("A", "B"):
        vals = []
        for j in POSITIONING_JOINTS:
            jname = "arm%s_%s" % (arm, j)
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                vals.append((j, None, None))
                continue
            adr = model.jnt_qposadr[jid]
            lo, hi = model.jnt_range[jid]
            vals.append((j, float(data.qpos[adr]), (float(lo), float(hi))))
        print("arm%s:" % arm)
        for j, q, rng in vals:
            if q is None:
                print("   %-14s MISSING" % j)
            else:
                print("   %-14s q=%+.4f rad  range=[%+.4f, %+.4f]"
                      % (j, q, rng[0], rng[1]))
        print("   pinch point at home: %s" % np.round(pinch_point(model, data, arm), 4))

    print()
    print("=" * 68)
    print("ALL-ZEROS POSE -- the brief's Stage 3 starting point (Lab 8 home)")
    print("=" * 68)
    saved = data.qpos.copy()
    for arm in ("A", "B"):
        for j in POSITIONING_JOINTS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    "arm%s_%s" % (arm, j))
            if jid >= 0:
                data.qpos[model.jnt_qposadr[jid]] = 0.0
    mujoco.mj_forward(model, data)

    cross = 0
    deepest = 0.0
    total = data.ncon
    for i in range(data.ncon):
        c = data.contact[i]
        b1 = model.geom_bodyid[c.geom1]
        b2 = model.geom_bodyid[c.geom2]
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b1) or ""
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b2) or ""
        if ("armA" in n1 and "armB" in n2) or ("armB" in n1 and "armA" in n2):
            cross += 1
            deepest = min(deepest, float(c.dist))
    print("total contacts at all-zeros : %d" % total)
    print("armA<->armB contacts        : %d" % cross)
    print("deepest cross-arm penetration: %.4f m" % deepest)
    print("ADR-026 recorded 34 total / 29 cross-arm / -0.0597 m deepest.")
    print("VERDICT: all-zeros is %s as a dual-arm home at this separation."
          % ("NOT usable" if cross > 0 else "usable"))

    data.qpos[:] = saved
    mujoco.mj_forward(model, data)

    print()
    print("=" * 68)
    print("LINK GEOMETRY for the geometric IK (measured, not from a drawing)")
    print("=" * 68)
    # Zero the arm so the chain lies in a known configuration, then read the
    # joint anchor positions (xanchor) -- these are the true rotation centres.
    for arm in ("A", "B"):
        for j in POSITIONING_JOINTS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    "arm%s_%s" % (arm, j))
            if jid >= 0:
                data.qpos[model.jnt_qposadr[jid]] = 0.0
    mujoco.mj_forward(model, data)

    for arm in ("A", "B"):
        print("arm%s (all positioning joints = 0):" % arm)
        anchors = {}
        for j in POSITIONING_JOINTS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    "arm%s_%s" % (arm, j))
            if jid < 0:
                continue
            anchors[j] = np.array(data.xanchor[jid])
            print("   anchor %-14s %s" % (j, np.round(anchors[j], 5)))
        pp = pinch_point(model, data, arm)
        print("   pinch point           %s" % np.round(pp, 5))
        if "shoulder_lift" in anchors and "elbow_flex" in anchors:
            L1 = np.linalg.norm(anchors["elbow_flex"] - anchors["shoulder_lift"])
            print("   L1 shoulder_lift->elbow_flex = %.5f m" % L1)
        if "elbow_flex" in anchors and "wrist_flex" in anchors:
            L2 = np.linalg.norm(anchors["wrist_flex"] - anchors["elbow_flex"])
            print("   L2 elbow_flex->wrist_flex    = %.5f m" % L2)
        if "wrist_flex" in anchors:
            off = pp - anchors["wrist_flex"]
            print("   tool offset wrist_flex->pinch (vec) = %s" % np.round(off, 5))
            print("   tool offset magnitude               = %.5f m"
                  % np.linalg.norm(off))
        base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm%s_base" % arm)
        if base >= 0:
            print("   base body xpos        %s" % np.round(data.xpos[base], 5))

    data.qpos[:] = saved
    mujoco.mj_forward(model, data)

    print()
    print("=" * 68)
    print("JOINT AXIS SIGN CHECK (perturb each joint, watch the pinch point)")
    print("=" * 68)
    for arm in ("A",):
        base_pp = pinch_point(model, data, arm)
        for j in POSITIONING_JOINTS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    "arm%s_%s" % (arm, j))
            if jid < 0:
                continue
            adr = model.jnt_qposadr[jid]
            keep = float(data.qpos[adr])
            data.qpos[adr] = keep + 0.10
            mujoco.mj_forward(model, data)
            moved = pinch_point(model, data, arm) - base_pp
            data.qpos[adr] = keep
            mujoco.mj_forward(model, data)
            print("   +0.10 rad on %-14s -> pinch moves %s (|d|=%.4f m)"
                  % (j, np.round(moved, 4), np.linalg.norm(moved)))
            print("      axis in world = %s" % np.round(data.xaxis[jid], 3))

    print()
    print("PROBE COMPLETE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
