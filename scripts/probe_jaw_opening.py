"""Probe: measure the SO-101 gripper's actual jaw opening in metres (M06a
grasp fix ladder, Step 4 pre-measurement).

Runs only on bm-ptl (ADR-020): imports `mujoco` via `bimanual.sim.env`.

**Why body-ORIGIN distance is the wrong measurement (found empirically, not
assumed -- see `scripts/probe_jaw_kinematics_debug.py`).** The moving jaw's
hinge joint (`armX_gripper`, type=hinge, axis=(0,0,1)) has `jnt_pos=(0,0,0)`
in the moving jaw body's own local frame -- i.e. the rotation axis passes
exactly through that body's origin. `data.xpos[moving_jaw_body_id]` is
therefore IDENTICAL at every joint angle (confirmed: byte-identical at both
`jnt_range` extremes). The jaw's own MESH geometry is what actually sweeps:
`scripts/probe_jaw_geoms_debug.py` found the moving jaw's mesh geom sits at
a local offset of about 3.1 cm from that axis (`local_pos=(-0.0014,
-0.0247, 0.0189)`), and it is THAT point -- not the body origin -- whose
world position changes as the jaw opens and closes.

**Also found empirically, and directly relevant to why M06a grasp fix D
(fine jaw-tip collision sphere) measured zero effect (DECISIONS.md, Fix D
retest, Sept 12, 2026): fix D placed its new collision sphere at LOCAL
`pos="0 0 0"` on the moving jaw body -- exactly ON the rotation axis found
above. That sphere therefore never moves as the jaw opens or closes,
regardless of commanded ctrl. This is reported here as a finding, not
silently worked around (out of this probe's scope to fix).**

Method: read the moving jaw's own mesh geom (the disabled bulky collision
geom is left in place with its original `pos`, per fix D) and command the
REAL joint (via `mj_forward`, not an actuator) to each extreme of
`model.jnt_range` (the same range `skills_scripted._gripper_ctrl` reads via
`actuator_ctrlrange`), then measure the world-frame distance between that
point and the corresponding point on the FIXED jaw side. The difference
between the CLOSED and OPEN distances is the gripper's actual opening
range in metres.
"""

from __future__ import annotations

import numpy as np
import mujoco

from bimanual.control import ik
from bimanual.sim.env import TableSettingEnv


def main() -> None:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model, data = env.model, env.data

    arm = "A"
    gripper_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ik.gripper_joint_name(arm))
    qadr = int(model.jnt_qposadr[gripper_jid])
    lo, hi = model.jnt_range[gripper_jid]
    print(f"gripper joint {ik.gripper_joint_name(arm)!r} range (rad): [{lo:.5f}, {hi:.5f}]")

    fixed_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.fixed_jaw_body_name(arm))
    moving_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.moving_jaw_body_name(arm))

    # The two bulky MESH geoms (still present, just contype=0 after fix D --
    # see JAW_COLLISION_MESHES in scripts/gen_dual_scene.py), one per body,
    # identified by geom_type==7 (mjGEOM_MESH) and NOT the fix-D tip sphere
    # (type==2, mjGEOM_SPHERE, local_pos (0,0,0)).
    def _mesh_geom_local_pos(body_id: int) -> np.ndarray:
        for gid in range(model.ngeom):
            if int(model.geom_bodyid[gid]) == body_id and int(model.geom_type[gid]) == mujoco.mjtGeom.mjGEOM_MESH:
                return np.array(model.geom_pos[gid], dtype=np.float64, copy=True)
        raise RuntimeError(f"no mesh geom found on body {body_id}")

    fixed_local = _mesh_geom_local_pos(fixed_body)
    moving_local = _mesh_geom_local_pos(moving_body)
    print(f"fixed jaw mesh geom local_pos={fixed_local}")
    print(f"moving jaw mesh geom local_pos={moving_local}")

    results = {}
    for label, angle in (("CLOSED", lo), ("OPEN", hi)):
        data.qpos[qadr] = angle
        mujoco.mj_forward(model, data)
        fixed_world = data.xpos[fixed_body] + data.xmat[fixed_body].reshape(3, 3) @ fixed_local
        moving_world = data.xpos[moving_body] + data.xmat[moving_body].reshape(3, 3) @ moving_local
        dist = float(np.linalg.norm(fixed_world - moving_world))
        results[label] = dist
        print(f"{label}: gripper_qpos={angle:.5f} fixed_world={fixed_world} "
              f"moving_world={moving_world} |fixed-moving|={dist:.4f} m")

    opening_range = abs(results["OPEN"] - results["CLOSED"])
    print(f"\nMEASURED jaw opening range (OPEN - CLOSED mesh-point distance): {opening_range:.4f} m")

    env.close()


if __name__ == "__main__":
    main()
