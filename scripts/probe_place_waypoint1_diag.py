"""Diagnostic only (no source change): after pick(A, water_bottle), replay
place's own waypoint-1 (approach-above-destination) computation directly via
`_drive_to_target`, with a MUCH larger step cap than the normal
APPROACH_DESCENT_STEPS=500, to see whether the 0.0138 m residual observed at
500 steps is still shrinking (slow convergence) or has plateaued
(a real reachability/servo-stall limit)."""

from __future__ import annotations

import numpy as np
import mujoco

from bimanual.control.executor import ScriptedSkillExecutor
from bimanual.control import ik
from bimanual.control.skills_scripted import (
    TABLE_SURFACE_Z, CLEARANCE_HEIGHT_M, PLACE_OFFSET_XY_M,
    _drive_to_target, _body_id, GRIPPER_CLOSE_FRACTION,
)
from bimanual.language.skills import SkillCall
from bimanual.sim.env import TableSettingEnv

SEED = 0


def main() -> None:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)
    try:
        executor = ScriptedSkillExecutor()
        body_id = _body_id(env.model, "water_bottle")

        pick_call = SkillCall(skill="pick", arm="A", target_object="bottle", params={})
        pick_result = executor.execute(pick_call, env)
        print(f"pick success={pick_result.success}")

        obj_xy = np.array(env.data.xpos[body_id][:2], dtype=np.float64, copy=True)
        dest_xy = obj_xy + np.array(PLACE_OFFSET_XY_M)
        dest_xy[0] = float(np.clip(dest_xy[0], -0.30, 0.30))
        dest_xy[1] = float(np.clip(dest_xy[1], -0.18, 0.18))
        approach_above_dest = np.array([dest_xy[0], dest_xy[1], TABLE_SURFACE_Z + CLEARANCE_HEIGHT_M])
        print(f"approach_above_dest target = {approach_above_dest}")

        site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "armA_gripperframe")
        print(f"current gripperframe site pos = {env.data.site_xpos[site_id]}")

        # Drive toward it with a generous step cap (2000) to see if it
        # eventually converges or plateaus.
        converged, steps, violations = _drive_to_target(
            env, "A", approach_above_dest, GRIPPER_CLOSE_FRACTION, max_steps=2000,
            target_object="bottle",
        )
        final_site_pos = env.data.site_xpos[site_id].copy()
        residual = float(np.linalg.norm(approach_above_dest - final_site_pos))
        print(f"after {steps} extra steps: converged={converged} violations={violations} "
              f"final_site_pos={final_site_pos} residual={residual:.4f}")

        solution = ik.solve_position_ik(env.model, env.data, "A", approach_above_dest)
        print(f"fresh IK solve residual at this pose: {solution.position_error_m:.4f}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
