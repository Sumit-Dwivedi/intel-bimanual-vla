"""Position-only damped-least-squares IK for one SO-101 arm (M06a, ADR-024).

**The DoF problem (see docs/learn/06-ik-and-skills.md and ADR-024).** Each arm
has six `<position>` actuators, but the sixth (`armX_gripper`) only opens and
closes the jaw -- it contributes nothing to end-effector pose. That leaves
five *positioning* joints (`shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`) against a 6-DoF task space (3 position + 3
orientation). A 5-DoF arm cannot in general hit an arbitrary position AND
orientation simultaneously.

**ADR-024's decision, implemented here: relax orientation entirely, solve
position exactly.** `solve_position_ik` drives only a 3D position error
(damped least squares on the 3xN position Jacobian of one site) using all
five positioning joints. No orientation target is ever specified -- whatever
orientation the redundant (5 joints, 3 constraints) solve converges to is
accepted as-is ("roll falls out"). This is deliberately the simpler of
ADR-024's two options; see that ADR for the alternative (a 5-DoF xyz+2-angle
target) and for the stated consequence to `handoff`, where the two arms'
grippers must meet in a shared pose with no coordinated control over either
arm's approach orientation.

**Which body this targets (the naming trap, ADR-024 / GLOSSARY note 01).**
`<body name="armX_gripper">` is the body driven by the `armX_wrist_roll`
joint -- it holds the fixed half of the gripper mechanism and the
`armX_gripperframe` site. `armX_moving_jaw_so101_v1` is a CHILD of that body,
driven by the separate `armX_gripper` joint/actuator (the jaw open/close
hinge), so its position also shifts as the jaw opens or closes. This solver
targets the `armX_gripperframe` SITE (defined on the `armX_gripper` body, not
on the moving jaw), which is exactly the case the trap warns about: aiming at
`armX_moving_jaw_so101_v1` instead would make the IK target silently drift
every time a skill opens or closes the jaw.

**Why a fresh scratch `MjData` per solve.** The solver is called once per
control step from a closed loop (see `skills_scripted.py`): read
`get_state()`-equivalent info, solve IK from the CURRENT joint angles toward
the current subgoal, write the result as one step of actuator targets, let
`env.step()` advance real physics, then repeat next step with the arm's new
(physically-moved) position as the new starting point. The solve itself must
never mutate `env.data` directly -- only `env.step()` may advance the real
simulation -- so every call works on a throwaway `mujoco.MjData` copy that
starts from the real data's current `qpos` and is discarded afterward.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

# ---------------------------------------------------------------------------
# Module-level tuning constants (task instructions: "anyone tuning these must
# find them in one place", not hardcoded inside functions).
# ---------------------------------------------------------------------------

#: Convergence threshold for the position-only DLS solve, in metres. 1 cm is
#: comfortably tighter than the ~1.5 cm phase-convergence tolerance skills
#: use to decide a physical move is "close enough" (skills_scripted.py's
#: POS_CONVERGENCE_TOL_M), so the IK solve itself is not the bottleneck.
IK_POSITION_TOLERANCE_M = 0.01

#: Orientation tolerance, in radians. NOT enforced by `solve_position_ik`
#: (ADR-024 relaxes orientation entirely -- there is no orientation target to
#: check against). Kept as a named constant, not deleted, so (a) a future
#: orientation-aware solve has one place to read a threshold from, and (b) a
#: diagnostic that wants to *report* how far off-axis the falls-out
#: orientation ended up has a documented bar to compare against.
IK_ORIENTATION_TOLERANCE_RAD = 0.35  # ~20 degrees

#: Default step BUDGET for a skill's closed control loop -- a count of
#: `env.step()` calls, not a wall-clock duration. See skills_scripted.py's
#: module docstring for why a step budget (not a timer) is the right unit:
#: it is comparable across runs and machines, per PLAN.md M06 done-when 3.
#:
#: 3000 was set empirically (not guessed): tracing a single ~0.29 m
#: closed-loop move under this scene's position-actuator gains
#: (`kp=998.22`, `kv=2.731` -- so101_dual_table.xml's `sts3215` default
#: class) showed convergence to within `IK_POSITION_TOLERANCE_M` at
#: roughly step ~200-240, not instantly -- these are real servos settling
#: under contact/damping, not a teleport. A compound skill (pick = 4
#: phases; place/handoff = pick + several more) needs several such moves
#: back to back, so 3000 leaves comfortable headroom without being an
#: unbounded loop.
DEFAULT_STEP_BUDGET = 3000

# Internal IK solver tuning (not part of the "anyone tuning these" contract
# above since they govern the solver's internals, not skill behaviour, but
# still centralized here rather than buried in a function body).
_IK_MAX_ITERS = 60
_IK_DAMPING = 0.05  # Levenberg-Marquardt damping lambda for the DLS solve

# The five positioning joints, in the order the SO-101 kinematic chain
# defines them (shoulder to wrist). Suffixes only -- `arm_joint_names()`
# prefixes them with "armA_"/"armB_" per scripts/gen_dual_scene.py's naming
# convention. Deliberately EXCLUDES "gripper" (the jaw hinge): that actuator
# never participates in IK, it is driven directly by each skill's open/close
# commands (skills_scripted.py).
ARM_JOINT_SUFFIXES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

GRIPPER_JOINT_SUFFIX = "gripper"
GRIPPERFRAME_SITE_SUFFIX = "gripperframe"


def arm_joint_names(arm: str) -> list[str]:
    """The five IK-controlled joint/actuator names for one arm ("A" or "B").

    Actuator names equal joint names in the generated scene
    (`src/bimanual/sim/assets/so101_dual_table.xml`'s `<actuator>` block), so
    this same list doubles as the actuator name list.
    """
    return [f"arm{arm}_{suffix}" for suffix in ARM_JOINT_SUFFIXES]


def gripper_joint_name(arm: str) -> str:
    """The jaw joint/actuator name for one arm, e.g. 'armA_gripper'.

    This is NOT one of `arm_joint_names()` -- it drives the jaw hinge on
    `armX_moving_jaw_so101_v1`, not arm positioning (see module docstring).
    """
    return f"arm{arm}_{GRIPPER_JOINT_SUFFIX}"


def gripperframe_site_name(arm: str) -> str:
    """The IK end-effector reference site name for one arm.

    Targets `armX_gripperframe`, a site defined on body `armX_gripper` (the
    body driven by `armX_wrist_roll`) -- see module docstring's naming-trap
    paragraph for why this, and not the moving-jaw body, is correct.
    """
    return f"arm{arm}_{GRIPPERFRAME_SITE_SUFFIX}"


@dataclass
class IKSolution:
    """One `solve_position_ik` result.

    Attributes:
        joint_angles: The 5 solved joint angles, in `ARM_JOINT_SUFFIXES`
            order, already clipped to each joint's `jnt_range`.
        position_error_m: Euclidean distance between the solved
            configuration's site position and the requested target, in
            metres, after the solve terminated (by convergence or by
            exhausting `_IK_MAX_ITERS`).
        converged: True if `position_error_m < tol` when the solve stopped.
        iterations: Number of Newton/DLS iterations actually run.
    """

    joint_angles: np.ndarray
    position_error_m: float
    converged: bool
    iterations: int


def _name2id_or_raise(model, obj_type, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id == -1:
        raise ValueError(f"{name!r} not found in the compiled model (obj_type={obj_type})")
    return obj_id


def solve_position_ik(
    model,
    data,
    arm: str,
    target_pos,
    *,
    max_iters: int = _IK_MAX_ITERS,
    tol: float = IK_POSITION_TOLERANCE_M,
    damping: float = _IK_DAMPING,
    step_scale: float = 1.0,
) -> IKSolution:
    """Solve for the 5 joint angles of `arm` that place its gripperframe site
    at `target_pos` (world xyz, metres), ignoring orientation (ADR-024).

    Damped least squares (Levenberg-Marquardt) on the site's 3xN position
    Jacobian (`mujoco.mj_jacSite`), restricted to the 5 columns for this
    arm's positioning joints. Starts from `data`'s CURRENT qpos (read-only --
    see module docstring on why a scratch `MjData` is used) and iterates
    Newton-style updates, re-running forward kinematics each iteration so
    the Jacobian and site position stay consistent with the updated qpos.

    Args:
        model: Compiled `mujoco.MjModel` for the dual-arm scene.
        data: The live `mujoco.MjData` to read the CURRENT joint angles from
            (never written to -- a scratch copy is solved instead).
        arm: "A" or "B".
        target_pos: Length-3 array-like, world-frame target position.
        max_iters, tol, damping, step_scale: Solver tuning; default to the
            module-level constants above.

    Returns:
        An `IKSolution`. Even when `converged` is False (max_iters
        exhausted), `joint_angles` holds the best configuration found -- the
        caller (a skill's closed control loop) commands it anyway and lets
        the NEXT control step's fresh solve correct further, exactly like a
        real visual/proprioceptive servo loop.
    """
    site_id = _name2id_or_raise(model, mujoco.mjtObj.mjOBJ_SITE, gripperframe_site_name(arm))
    joint_ids = [
        _name2id_or_raise(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in arm_joint_names(arm)
    ]
    dof_ids = [int(model.jnt_dofadr[j]) for j in joint_ids]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    joint_ranges = [tuple(model.jnt_range[j]) for j in joint_ids]

    # Scratch copy: mirrors `data`'s current qpos so the solve starts from
    # wherever the real arm physically is right now, but every update below
    # touches only this throwaway copy. `env.step()` is the only thing
    # allowed to advance the real simulation.
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    scratch.qvel[:] = 0.0
    mujoco.mj_forward(model, scratch)

    target = np.asarray(target_pos, dtype=np.float64).reshape(3)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))  # unused (position-only IK), but mj_jacSite wants both arrays

    iterations = 0
    err = target - scratch.site_xpos[site_id]
    for iterations in range(1, max_iters + 1):
        err = target - scratch.site_xpos[site_id]
        if np.linalg.norm(err) < tol:
            break

        mujoco.mj_jacSite(model, scratch, jacp, jacr, site_id)
        jac = jacp[:, dof_ids]  # (3, 5): this arm's columns only

        # Damped least squares: delta_q = J^T (J J^T + lambda^2 I)^-1 err.
        # The damping term keeps the solve well-conditioned near
        # singularities (e.g. a fully extended elbow) instead of producing
        # huge, unstable joint steps there.
        lam2 = damping * damping
        delta = jac.T @ np.linalg.solve(jac @ jac.T + lam2 * np.eye(3), err)

        for k, qadr in enumerate(qpos_adrs):
            lo, hi = joint_ranges[k]
            new_q = scratch.qpos[qadr] + step_scale * delta[k]
            scratch.qpos[qadr] = float(np.clip(new_q, lo, hi))
        mujoco.mj_forward(model, scratch)

    final_err = float(np.linalg.norm(target - scratch.site_xpos[site_id]))
    joint_angles = np.array([scratch.qpos[a] for a in qpos_adrs], dtype=np.float64)
    return IKSolution(
        joint_angles=joint_angles,
        position_error_m=final_err,
        converged=final_err < tol,
        iterations=iterations,
    )
