"""M06 verification probe (Day 3, ADR-033 follow-up): does place(A,
water_bottle, table) work once the bottle is already held by a prior
pick(A, water_bottle) in the same episode?

Runs pick(A, water_bottle) then place(A, water_bottle, "table") through the
real ScriptedSkillExecutor + SkillCall path (same path scripts/run_skill.py
and tests/test_skills.py use), on a freshly reset TableSettingEnv, seed 0.

Prints, in order:
  - pick's SkillResult (success, reason, frames_used, weld_attach_frame,
    weld_active_at_end)
  - weld.is_holding('A') immediately after pick
  - place's SkillResult (success, reason, frames_used, weld_attach_frame,
    weld_active_at_end)
  - weld.is_holding('A') immediately after place (must be None if released)
  - the bottle's final body-origin xyz and quaternion (xquat, wxyz)
  - TABLE_SURFACE_Z for reference, and the expected resting body-origin z
    (TABLE_SURFACE_Z + bottle cylinder half-height 0.09, from
    scripts/gen_dual_scene.py's water_bottle_body geom size="0.03 0.09")
  - an upright check: the angle between the body's local +z axis (rotated
    into world frame via xquat) and world +z, in degrees -- 0 deg is
    perfectly upright, 90 deg is on its side.

No source file is modified by this script. It only reads/executes existing
code.
"""

from __future__ import annotations

import numpy as np
import mujoco

from bimanual.control.executor import ScriptedSkillExecutor
from bimanual.control.skills_scripted import TABLE_SURFACE_Z
from bimanual.language.skills import SkillCall
from bimanual.sim.env import TableSettingEnv

SEED = 0
BOTTLE_HALF_HEIGHT_M = 0.09  # scripts/gen_dual_scene.py:1039, water_bottle_body geom size="0.03 0.09"


def quat_to_up_angle_deg(xquat: np.ndarray) -> float:
    """Angle (degrees) between the body's local +z axis, rotated into world
    frame by `xquat` (w, x, y, z), and world +z. 0 = upright, 90 = on its
    side, 180 = upside down."""
    local_z = np.array([0.0, 0.0, 1.0])
    world_z = np.zeros(3)
    mujoco.mju_rotVecQuat(world_z, local_z, xquat)
    cos_angle = float(np.clip(np.dot(world_z, np.array([0.0, 0.0, 1.0])), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_angle)))


def main() -> None:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)
    try:
        executor = ScriptedSkillExecutor()
        body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "water_bottle")

        print("=== pick(A, water_bottle) ===")
        pick_call = SkillCall(skill="pick", arm="A", target_object="bottle", params={})
        pick_result = executor.execute(pick_call, env)
        print(f"pick result: success={pick_result.success} reason={pick_result.reason!r} "
              f"frames_used={pick_result.frames_used} weld_attach_frame={pick_result.weld_attach_frame} "
              f"weld_active_at_end={pick_result.weld_active_at_end}")
        print(f"weld.is_holding('A') after pick: {executor.weld.is_holding('A')!r}")

        print("=== place(A, water_bottle, table) ===")
        place_call = SkillCall(
            skill="place", arm="A", target_object="bottle", params={"destination": "table"}
        )
        place_result = executor.execute(place_call, env)
        print(f"place result: success={place_result.success} reason={place_result.reason!r} "
              f"frames_used={place_result.frames_used} weld_attach_frame={place_result.weld_attach_frame} "
              f"weld_active_at_end={place_result.weld_active_at_end}")
        holding_after_place = executor.weld.is_holding("A")
        print(f"weld.is_holding('A') after place: {holding_after_place!r}")

        final_xpos = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
        final_xquat = np.array(env.data.xquat[body_id], dtype=np.float64, copy=True)
        up_angle_deg = quat_to_up_angle_deg(final_xquat)
        expected_resting_z = TABLE_SURFACE_Z + BOTTLE_HALF_HEIGHT_M

        print("=== final bottle state ===")
        print(f"final body-origin xyz: x={final_xpos[0]:.4f} y={final_xpos[1]:.4f} z={final_xpos[2]:.4f}")
        print(f"final xquat (w,x,y,z): {final_xquat}")
        print(f"TABLE_SURFACE_Z = {TABLE_SURFACE_Z}")
        print(f"expected resting body-origin z (TABLE_SURFACE_Z + half-height 0.09) = {expected_resting_z:.4f}")
        print(f"upright angle (0=upright, 90=on its side) = {up_angle_deg:.2f} deg")

        print("=== summary checks ===")
        print(f"SkillResult.success == True: {place_result.success is True}")
        print(f"is_holding('A') is None: {holding_after_place is None}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
