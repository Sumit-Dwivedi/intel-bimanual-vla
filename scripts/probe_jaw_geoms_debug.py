"""One-off debug: list every geom on the fixed and moving jaw bodies with
their LOCAL pos (geom_pos is body-local, not world), so the actual swept
"jaw tip" distance from the rotation axis can be computed (the body ORIGIN
itself sits ON the hinge axis -- confirmed by probe_jaw_kinematics_debug.py
-- so body xpos alone never moves; the mesh/collision geoms are offset from
that origin and are what actually sweeps through the arc)."""
from __future__ import annotations

import mujoco

from bimanual.control import ik
from bimanual.sim.env import TableSettingEnv


def main() -> None:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model = env.model

    arm = "A"
    for label, body_suffix in (("FIXED", "gripper"), ("MOVING", "moving_jaw_so101_v1")):
        body_name = f"arm{arm}_{body_suffix}"
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        print(f"--- {label} jaw body {body_name!r} (id={body_id}) ---")
        for gid in range(model.ngeom):
            if int(model.geom_bodyid[gid]) == body_id:
                gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "(unnamed)"
                gtype = int(model.geom_type[gid])
                gpos = model.geom_pos[gid]
                gsize = model.geom_size[gid]
                rbound = model.geom_rbound[gid]
                contype = int(model.geom_contype[gid])
                print(f"  geom id={gid} name={gname!r} type={gtype} local_pos={gpos} "
                      f"size={gsize} rbound={rbound:.4f} contype={contype}")

    env.close()


if __name__ == "__main__":
    main()
