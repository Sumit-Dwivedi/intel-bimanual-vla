"""ADR-028 Step 3 gate: verify the NEW finger-pad geoms' separation varies
materially with the gripper joint angle, on arm A, using the real compiled
scene (src/bimanual/sim/assets/so101_dual_table.xml).

This is the specific check the task's Step 3 requires: measure distance
between `armA_static_finger_pad` and `armA_moving_finger_pad` (the pads
themselves, NOT the jaw mesh collision geoms) at three gripper joint angles
-- fully closed (-0.1745), midway (~0.785), fully open (+1.7453) -- and
confirm the distance changes materially across them. This is exactly the
gate Fix D failed silently: its replacement spheres sat on the moving jaw's
own rotation axis and read an identical gap (+0.00618 m) at both extremes.

Not part of the shipped skill code -- a diagnostic script, like
scripts/probe_jaw_opening.py and friends (DECISIONS.md's M06a retest
ladder).
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE = REPO_ROOT / "src" / "bimanual" / "sim" / "assets" / "so101_dual_table.xml"

ANGLES = {
    "fully closed": -0.1745,
    "midway": 0.785,
    "fully open": 1.7453,
}


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    static_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "armA_static_finger_pad")
    moving_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "armA_moving_finger_pad")
    assert static_geom_id >= 0 and moving_geom_id >= 0, "pad geoms not found -- generator did not run"

    gripper_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "armA_gripper")
    assert gripper_joint_id >= 0
    qpos_adr = model.jnt_qposadr[gripper_joint_id]

    # Reset to the scene's own "home" keyframe first (arms folded back,
    # zero self/cross-arm collision, ADR-026) so every other joint is at a
    # realistic pose, then override ONLY the gripper joint per angle.
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    assert home_key >= 0

    results = {}
    for label, angle in ANGLES.items():
        mujoco.mj_resetDataKeyframe(model, data, home_key)
        data.qpos[qpos_adr] = angle
        mujoco.mj_forward(model, data)
        static_pos = data.geom_xpos[static_geom_id].copy()
        moving_pos = data.geom_xpos[moving_geom_id].copy()
        dist = float(np.linalg.norm(static_pos - moving_pos))
        results[label] = dist
        print(f"{label:>12s} (qpos={angle:+.4f}): pad separation = {dist:.5f} m "
              f"(static world pos={static_pos}, moving world pos={moving_pos})")

    values = list(results.values())
    spread = max(values) - min(values)
    print(f"\nspread across three angles: {spread:.5f} m")
    if spread < 1e-4:
        print("GATE FAILED: separation is materially identical across angles "
              "(same failure mode as Fix D) -- a pad is likely parented to the "
              "wrong body or sitting on a rotation axis.")
    else:
        print("GATE PASSED: separation varies materially with joint angle.")


if __name__ == "__main__":
    main()
