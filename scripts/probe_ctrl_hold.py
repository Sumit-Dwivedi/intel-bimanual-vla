"""scripts/probe_ctrl_hold.py -- M06 Phase 2 Commit 1 verification (step 2).

Confirms the ctrl-hold mechanism this repo keeps (`_hold_ctrl`,
`src/bimanual/control/skills_scripted.py:419-430`) actually protects an idle
arm across many `env.step()` calls, exercised the way a NEW caller (anything
outside `skills_scripted.py`'s own `_drive_to_target`/`_dwell` loops) would
have to use it -- per `docs/hardware/m06-phase2-prerequisites.md` Q2's
finding that `env.step()` itself does no holding at all (a bare, unconditional
12-vector overwrite, `env.py:260`) and the "home" keyframe declares qpos only
(no `ctrl`), so an unheld idle arm's ctrl target defaults to 0.0 -- the
pre-ADR-026 cross-arm interpenetration pose -- the instant a caller forgets
to reproduce `_hold_ctrl`'s pattern.

Method: reset to "home", record arm B's home qpos, then run 100 `env.step()`
calls that cycle ONLY arm A between two distinct joint poses (never arm B),
building each step's full 12-vector ctrl exactly the way `_drive_to_target`/
`_dwell` do: `_hold_ctrl(env)` first (reads every actuator's CURRENT qpos,
including arm B's, into the full vector), then `_write_arm_ctrl` overwrites
only arm A's 6-slot slice on top of that. Arm B's ctrl slice is therefore
supplied entirely by `_hold_ctrl`, every step, never by this script directly
-- exactly the "reproduce `_hold_ctrl`'s pattern" case the audit's
implication #1 describes. If arm B's qpos stays within 0.01 rad of its home
value throughout, the mechanism holds under direct `env.step()` use, not just
inside `skills_scripted.py`'s own existing call sites.

Runs only on bm-ptl (ADR-020) -- imports `bimanual.sim.env`, which imports
`mujoco`.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control.skills_scripted import _hold_ctrl, _write_arm_ctrl  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

#: Number of env.step() calls to run (task instruction: "run 100 env.step()
#: calls").
N_STEPS = 100

#: Switch arm A's target pose every this many steps, so it visibly moves
#: (not just sits at one pose), while never touching arm B's slice directly.
SWITCH_EVERY = 25

#: Max allowed arm B qpos drift from home, per the task's own pass bar.
DRIFT_TOL_RAD = 0.01


def _arm_a_pose(env, which: int) -> np.ndarray:
    """Two distinct, in-range joint-angle targets for arm A's 5 positioning
    joints (`ik.ARM_JOINT_SUFFIXES` order), chosen well inside each joint's
    `jnt_range` (see `inspect_home.py`-style measurement: arm A's home is
    shoulder_pan=0, shoulder_lift=-1.2, elbow_flex=-1.6, wrist_flex=0,
    wrist_roll=0) so this probe actually commands motion, not a no-op hold.
    """
    if which == 0:
        return np.array([0.0, -1.2, -1.6, 0.0, 0.0], dtype=np.float64)
    return np.array([0.5, -0.8, -1.2, 0.3, 0.5], dtype=np.float64)


def main() -> int:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)

    # Record arm B's home qpos (every joint address `_hold_ctrl` would also
    # read) BEFORE any step, so drift is measured against the actual starting
    # state, not an assumption about what "home" is.
    b_joint_names = ik.arm_joint_names("B") + [ik.gripper_joint_name("B")]
    b_qadr = [
        int(env.model.jnt_qposadr[mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, name)])
        for name in b_joint_names
    ]
    home_b_qpos = np.array([env.data.qpos[q] for q in b_qadr], dtype=np.float64)

    # Arm A's own gripper actuator needs a ctrl value too (`_write_arm_ctrl`
    # sets it as part of arm A's slice); hold it at its current (home) value
    # since this probe is about idle-arm protection, not grasping.
    a_gripper_jid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, ik.gripper_joint_name("A"))
    a_gripper_qadr = int(env.model.jnt_qposadr[a_gripper_jid])
    a_gripper_home = float(env.data.qpos[a_gripper_qadr])

    max_drift = 0.0
    max_drift_step = -1
    max_drift_joint = ""

    for step in range(N_STEPS):
        # Exactly the pattern `_drive_to_target`/`_dwell` use
        # (skills_scripted.py:540-544, 587-591): build the FULL ctrl vector
        # from `_hold_ctrl` first (every actuator, including arm B's, held
        # at its CURRENT qpos), THEN overwrite only the actively-driven
        # arm's slice on top.
        ctrl = _hold_ctrl(env)
        pose = _arm_a_pose(env, (step // SWITCH_EVERY) % 2)
        _write_arm_ctrl(ctrl, env.model, "A", pose, a_gripper_home)
        env.step(ctrl)

        cur_b_qpos = np.array([env.data.qpos[q] for q in b_qadr], dtype=np.float64)
        drift = np.abs(cur_b_qpos - home_b_qpos)
        step_max = float(np.max(drift))
        if step_max > max_drift:
            max_drift = step_max
            max_drift_step = step
            max_drift_joint = b_joint_names[int(np.argmax(drift))]

    print(f"steps_run={N_STEPS} switch_every={SWITCH_EVERY}")
    print(f"home_arm_B_qpos={home_b_qpos.tolist()}")
    print(f"max_arm_B_drift_rad={max_drift:.6f} at step={max_drift_step} joint={max_drift_joint}")
    print(f"within_tol({DRIFT_TOL_RAD} rad)={max_drift <= DRIFT_TOL_RAD}")

    return 0 if max_drift <= DRIFT_TOL_RAD else 1


if __name__ == "__main__":
    raise SystemExit(main())
