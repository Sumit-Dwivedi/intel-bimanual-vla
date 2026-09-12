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
hinge), so its position also shifts as the jaw opens or closes.

**Retargeting to the pinch point (ADR-025, M06a follow-up).** M06a's own
tests found that `armX_gripperframe` is NOT where the jaws actually pinch:
measured at rest on arm A, the site sits at (y=0.1414, z=0.5765) while the
fixed jaw body `armA_gripper` is at (y=0.0432, z=0.5844) and the moving jaw
body `armA_moving_jaw_so101_v1` is at (y=0.0666, z=0.6055) -- the site is
roughly `PINCH_POINT_OFFSET_M` (~8 cm) away from the midpoint of the two jaw
bodies, which is where the object is actually pinched. `solve_position_ik`
therefore targets that computed midpoint directly, evaluated fresh every
solve from the CURRENT `qpos` (via `mujoco.mj_jacBody` on both jaw bodies),
so the target tracks the moving jaw as a skill opens or closes it -- exactly
the tracking problem the naming trap above warns about, solved by computing
the point instead of aiming at a fixed site or a fixed offset from one.
`armX_gripperframe` itself is left in the XML and in
`gripperframe_site_name()` below, unused by the solver, for reference and
for any future diagnostic that wants to compare the two.

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

#: Measured distance, in metres, between the upstream `armX_gripperframe`
#: site and the computed pinch-point midpoint this solver targets instead
#: (ADR-025). Measured at rest on arm A: `armA_gripperframe` at
#: (y=0.1414, z=0.5765) vs. the fixed jaw body `armA_gripper` at
#: (y=0.0432, z=0.5844) and the moving jaw body
#: `armA_moving_jaw_so101_v1` at (y=0.0666, z=0.6055) -- roughly 8 cm from
#: the site to the true pinch zone. This constant is documentation, NOT a
#: correction applied anywhere in this file: the whole reason to retarget
#: the solve (rather than just adding a fixed offset to the old site
#: target) is that this offset is not constant in the world frame -- it is
#: a fixed LOCAL-frame vector (rigid for a given jaw open/close angle) that
#: rotates with whatever orientation the redundant 5-joint solve falls
#: into (ADR-024), so its world-frame direction changes with the target
#: even though its magnitude does not. See the module docstring's
#: "Retargeting to the pinch point" section.
PINCH_POINT_OFFSET_M = 0.08

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

#: The fixed jaw BODY name suffix. NOTE: this is the same string as
#: `GRIPPER_JOINT_SUFFIX` -- `armX_gripper` names both a JOINT (driving
#: wrist_roll) and a BODY (the fixed half of the gripper mechanism) in the
#: compiled model; MuJoCo namespaces body/joint/site/actuator names
#: separately, so this is not a collision (see ADR-024's naming-trap
#: paragraph, which already documents this same body under a different
#: name pun). Kept as its own constant, not merely reusing
#: `gripper_joint_name()`'s string, so a reader of `fixed_jaw_body_name()`
#: does not have to go verify that reuse is safe.
FIXED_JAW_BODY_SUFFIX = "gripper"

#: The moving jaw BODY name suffix -- a CHILD of the fixed jaw body above,
#: driven by the `armX_gripper` joint/actuator (the jaw open/close hinge).
MOVING_JAW_BODY_SUFFIX = "moving_jaw_so101_v1"


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
    """The upstream gripper-frame reference site name for one arm.

    `armX_gripperframe` is a site defined on body `armX_gripper` (the body
    driven by `armX_wrist_roll`). NOT the IK target as of ADR-025:
    `solve_position_ik` targets the computed pinch-point midpoint instead
    (see module docstring's "Retargeting to the pinch point" section and
    `PINCH_POINT_OFFSET_M`). Left in place, unused by the solver, so the
    site stays available in the XML for reference and for any future
    diagnostic that wants to compare the two.
    """
    return f"arm{arm}_{GRIPPERFRAME_SITE_SUFFIX}"


def fixed_jaw_body_name(arm: str) -> str:
    """The fixed jaw BODY name for one arm, e.g. 'armA_gripper'.

    Same string as `gripper_joint_name()` returns, but names the BODY
    (holds `armX_gripperframe` and is the parent of the moving jaw), not
    the wrist_roll joint -- see `FIXED_JAW_BODY_SUFFIX`'s docstring.
    """
    return f"arm{arm}_{FIXED_JAW_BODY_SUFFIX}"


def moving_jaw_body_name(arm: str) -> str:
    """The moving jaw BODY name for one arm, e.g. 'armA_moving_jaw_so101_v1'.

    A CHILD of `fixed_jaw_body_name(arm)`, driven by the `armX_gripper`
    joint/actuator (the jaw open/close hinge) -- its position shifts as a
    skill opens or closes the jaw, which is exactly why the pinch point
    (the midpoint between this body and the fixed jaw body) must be
    recomputed every solve rather than cached (ADR-025).
    """
    return f"arm{arm}_{MOVING_JAW_BODY_SUFFIX}"


@dataclass
class IKSolution:
    """One `solve_position_ik` result.

    Attributes:
        joint_angles: The 5 solved joint angles, in `ARM_JOINT_SUFFIXES`
            order, already clipped to each joint's `jnt_range`.
        position_error_m: Euclidean distance between the solved
            configuration's PINCH-POINT position (ADR-025: the midpoint
            between the fixed and moving jaw bodies, not the
            `armX_gripperframe` site) and the requested target, in
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
    """Solve for the 5 joint angles of `arm` that place its PINCH POINT
    (ADR-025: the midpoint between the fixed and moving jaw bodies, computed
    fresh from the current qpos every solve -- NOT the `armX_gripperframe`
    site) at `target_pos` (world xyz, metres), ignoring orientation
    (ADR-024).

    Damped least squares (Levenberg-Marquardt) on the pinch point's 3xN
    position Jacobian, restricted to the 5 columns for this arm's
    positioning joints. The pinch point's Jacobian is the average of the two
    jaw bodies' own Jacobians (`mujoco.mj_jacBody`), since the pinch point
    itself is defined as their average position -- differentiating an
    average gives the average of the derivatives. Starts from `data`'s
    CURRENT qpos (read-only -- see module docstring on why a scratch
    `MjData` is used) and iterates Newton-style updates, re-running forward
    kinematics each iteration so the Jacobians and jaw-body positions stay
    consistent with the updated qpos -- this is what makes the target track
    the moving jaw's ACTUAL current position (e.g. mid-close) rather than
    wherever it was when the solve began.

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
    fixed_jaw_id = _name2id_or_raise(model, mujoco.mjtObj.mjOBJ_BODY, fixed_jaw_body_name(arm))
    moving_jaw_id = _name2id_or_raise(model, mujoco.mjtObj.mjOBJ_BODY, moving_jaw_body_name(arm))
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
    # Separate Jacobian buffers per body: mj_jacBody writes into whatever
    # array it is given, and we need both bodies' translational Jacobians
    # simultaneously to average them below. jacr_* (rotational) is computed
    # by the same call but unused here (position-only IK).
    jacp_fixed = np.zeros((3, model.nv))
    jacr_fixed = np.zeros((3, model.nv))
    jacp_moving = np.zeros((3, model.nv))
    jacr_moving = np.zeros((3, model.nv))

    def _pinch_point() -> np.ndarray:
        # The pinch point is defined as the midpoint of the two jaw bodies'
        # world-frame origins (mj_jacBody / xpos both refer to a body's own
        # frame origin, not its center of mass, so this is consistent).
        return 0.5 * (scratch.xpos[fixed_jaw_id] + scratch.xpos[moving_jaw_id])

    iterations = 0
    err = target - _pinch_point()
    for iterations in range(1, max_iters + 1):
        err = target - _pinch_point()
        if np.linalg.norm(err) < tol:
            break

        mujoco.mj_jacBody(model, scratch, jacp_fixed, jacr_fixed, fixed_jaw_id)
        mujoco.mj_jacBody(model, scratch, jacp_moving, jacr_moving, moving_jaw_id)
        jacp = 0.5 * (jacp_fixed + jacp_moving)  # Jacobian of the midpoint
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

    final_err = float(np.linalg.norm(target - _pinch_point()))
    joint_angles = np.array([scratch.qpos[a] for a in qpos_adrs], dtype=np.float64)
    return IKSolution(
        joint_angles=joint_angles,
        position_error_m=final_err,
        converged=final_err < tol,
        iterations=iterations,
    )
