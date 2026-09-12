"""One-off debug: why did probe_jaw_opening.py show identical fixed/moving
jaw xpos at both gripper joint extremes? Inspect the kinematic chain between
the fixed jaw body and the moving jaw body directly.
"""
from __future__ import annotations

import mujoco

from bimanual.control import ik
from bimanual.sim.env import TableSettingEnv


def main() -> None:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model, data = env.model, env.data

    arm = "A"
    fixed_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.fixed_jaw_body_name(arm))
    moving_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.moving_jaw_body_name(arm))
    print(f"fixed body id={fixed_id} name={mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, fixed_id)}")
    print(f"moving body id={moving_id} name={mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, moving_id)}")
    print(f"moving body's parent id={model.body_parentid[moving_id]} "
          f"name={mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.body_parentid[moving_id]))}")

    # Every joint attached to the moving jaw body.
    print("joints on moving jaw body:")
    for jid in range(model.njnt):
        if model.jnt_bodyid[jid] == moving_id:
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            jtype = model.jnt_type[jid]
            qadr = int(model.jnt_qposadr[jid])
            print(f"  joint id={jid} name={jname!r} type={jtype} qposadr={qadr} "
                  f"range={model.jnt_range[jid]} axis={model.jnt_axis[jid]} pos={model.jnt_pos[jid]}")

    gripper_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ik.gripper_joint_name(arm))
    print(f"gripper_joint_name({arm}) = {ik.gripper_joint_name(arm)!r} -> jid={gripper_jid} "
          f"bodyid={model.jnt_bodyid[gripper_jid]} "
          f"body_name={mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.jnt_bodyid[gripper_jid]))}")

    env.close()


if __name__ == "__main__":
    main()
