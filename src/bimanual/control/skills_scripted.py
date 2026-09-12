"""Scripted, IK-driven skill primitives (M06a): `open_drawer`, `pick`, `place`,
`handoff`. `pour` is M06b and is intentionally NOT implemented in this module
-- see PLAN.md's POUR GATE and the M06a task scope.

Every skill is a closed control loop, per docs/learn/06-ik-and-skills.md:
read the current state (`env.model`/`env.data`, privileged per ADR-005 --
the scripted controller has no perception step), pick the current subgoal,
solve IK (`bimanual.control.ik.solve_position_ik`), write joint targets via
`env.step()`, test whether the subgoal is met, advance or time out. Every
function here returns a `SkillResult(success, reason, frames_used)`, where
`frames_used` counts `env.step()` calls (the step-budget unit, not wall
clock -- PLAN.md M06 done-when 3).

**Why grasping is done by geometry read from the scene, not by editing the
scene.** Per the task scope, `src/bimanual/sim/` (including the MJCF asset)
is off-limits to this module. The SO-101 gripper is a single rotating jaw
with no handle/lip geometry provided anywhere on the drawer or the props, so
two different physical techniques are used, chosen per object:
  - `pick`/`place`/`handoff` on the five props: a PINCH grasp, approaching a
    small graspable feature of each object (a rim edge, a handle) read from
    the object's own geom size in the compiled model (see
    `GRASP_POINT_OFFSET_M`), never a hardcoded world coordinate.
  - `open_drawer`: the drawer body (`drawer_box`) is a solid box far larger
    than the gripper's span and has no handle to pinch, so it is opened by
    FRICTION DRAG instead: press down on its top surface and walk the
    contact point toward the drawer's open direction in small waypoints
    (ADR-021's `drawer_slide` axis, `(0, -1, 0)`), relying on downward
    contact force and friction to drag the drawer along, the same way one
    might slide a book across a table by pressing and pushing rather than
    pinching it.

**Orientation is never controlled (ADR-024).** `ik.solve_position_ik` only
targets end-effector *position*; the jaw's approach angle is whatever falls
out of the redundant 5-joint solve. This is why every grasp point below is
chosen to be forgiving of approach angle (a small, roughly axisymmetric
feature) rather than requiring a precise pinch orientation.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import mujoco
import numpy as np

from bimanual.control import ik

logger = logging.getLogger(__name__)


@dataclass
class SkillResult:
    """The typed result every scripted skill returns (PLAN.md M06 done-when 2).

    Attributes:
        success: Whether the skill's measurable success condition was met
            (e.g. the object rose above the table by the stated margin).
        reason: A short, human-readable explanation carrying the MEASURED
            number(s) that justify `success` -- e.g. "lifted to z=0.391
            (initial z=0.356, margin=0.03)" -- so a Tester or a log reader
            never has to re-derive what happened.
        frames_used: Total `env.step()` calls made while running this
            skill. This is the step-BUDGET unit (PLAN.md M06 done-when 3),
            not a wall-clock duration -- comparable across runs and
            machines regardless of host speed.
    """

    success: bool
    reason: str
    frames_used: int


# ---------------------------------------------------------------------------
# Object vocabulary. Canonical names match `SkillCall.target_object`
# (bimanual/language/rule_grounder.py's `_NOUN_CANON`), which is why "bottle"
# (not "water_bottle") is the key even though the MJCF body is named
# "water_bottle".
# ---------------------------------------------------------------------------
OBJECT_BODY_NAME = {
    "plate": "plate",
    "mug": "mug",
    "fork": "fork",
    "spoon": "spoon",
    "bottle": "water_bottle",
    "drawer": "drawer",
}

#: Body-local grasp-point offset for each pickable prop, added to the
#: object's current world position (read from `data.xpos`, never
#: hardcoded) to get the IK target for a pinch grasp. Chosen from each
#: object's own geometry in `so101_dual_table.xml` (verified scene
#: geometry): the plate's rim (radius 0.09 m), the mug's handle midpoint
#: (a ~1.6 cm diameter capsule -- far more graspable by a small parallel
#: jaw than the mug's own 7 cm body), the fork/spoon handles, and near the
#: water bottle's neck. These offsets assume the object is still close to
#: its as-placed (near-identity) orientation, which holds at the start of
#: every skill this module implements (none of them tumble props before
#: grasping) -- a documented simplification, not a claim of full generality.
GRASP_POINT_OFFSET_M = {
    "plate": np.array([0.09, 0.0, 0.0]),
    "mug": np.array([0.0475, 0.0, 0.0]),
    "fork": np.array([-0.015, 0.0, 0.0]),
    "spoon": np.array([-0.01, 0.0, 0.0]),
    "bottle": np.array([0.0, 0.0, 0.02]),
}

# ---------------------------------------------------------------------------
# Module-level tuning constants -- one place to find and adjust every
# skill-behaviour number (task instructions: "anyone tuning these must find
# them in one place").
# ---------------------------------------------------------------------------

#: Tabletop surface height, metres (verified scene geometry: table_top box
#: centred at z=0.34, half-thickness 0.01 -> top surface at 0.35).
TABLE_SURFACE_Z = 0.35

#: How far above a grasp point to hover before descending, metres.
APPROACH_HEIGHT_M = 0.10

#: How far above the grasp point to lift once grasped, metres.
LIFT_HEIGHT_M = 0.12

#: Success bar for "lifted": the object's world z must rise by at least
#: this much above its OWN measured starting height this run (PLAN.md M06
#: done-when 1: "lifted above a stated height threshold, reported as a
#: measured number").
PICK_LIFT_MARGIN_M = 0.03

#: Gripper ctrl targets, expressed as a fraction of `armX_gripper`'s
#: [low, high] `jnt_range` (so they scale automatically if the range ever
#: changes). Which end is physically "open" vs. "closed" was inferred, not
#: measured against the jaw mesh (no mesh-geometry inspection is available
#: without touching src/bimanual/sim/): 1.0 (the joint's positive/high end,
#: a ~100 degree sweep away from rest) is treated as OPEN and 0.0 (the
#: slightly-negative low end) as CLOSED. If a bm-ptl run shows the jaw
#: behaving backwards, swap these two constants -- nothing else changes.
GRIPPER_OPEN_FRACTION = 1.0
GRIPPER_CLOSE_FRACTION = 0.0

#: How many control steps to dwell while closing/opening the jaw, letting
#: the position servo and contact solver settle before the arm moves again.
#: Measured empirically, not guessed: tracing `armA_gripper`'s qpos during a
#: commanded close showed it takes roughly 180 steps to travel the jaw's
#: full ~1.9 rad range under this scene's actuator gains (kp=998.22) -- a
#: dwell much shorter than that (an earlier version used 40) commands the
#: close but abandons it barely started, well before the jaw has travelled
#: far enough to contact (or fail to contact) anything.
GRASP_DWELL_STEPS = 200
RELEASE_DWELL_STEPS = 200

#: How close the gripperframe site must get to a phase's target position,
#: metres, before a `_drive_to_target` phase is considered converged.
#: Looser than `ik.IK_POSITION_TOLERANCE_M` because this measures the REAL,
#: physically-servoed arm (subject to PD overshoot/settling), not the
#: kinematic IK solve.
POS_CONVERGENCE_TOL_M = 0.015

#: `place`'s destination offset (x, y) in metres, relative to the object's
#: pick-up location -- "somewhere else on the table", not back where it
#: started.
PLACE_OFFSET_XY_M = (0.10, -0.05)

#: Verified scene geometry: the ~0.10 m wide band at the table centre both
#: arms can reach (arm bases at y=-0.25/+0.25, ~0.30 m reach each) -- the
#: only place a handoff can physically happen (ADR-010, ADR-024).
HANDOFF_TRANSFER_XY = (0.0, 0.0)
HANDOFF_TRANSFER_HEIGHT_M = 0.10
#: How far off the transfer centre each arm's gripper sits while both are
#: present, so the two gripper mechanisms do not try to occupy the same
#: point (ADR-024's "handoff" consequence: with no orientation control,
#: this offset is our only concurrency-safety margin between the two
#: jaws).
HANDOFF_SIDE_OFFSET_M = 0.03

#: `open_drawer` tuning. See module docstring: friction-drag, not a pinch.
DRAWER_WAYPOINT_STEP_M = 0.01
DRAWER_MAX_WAYPOINTS = 20
DRAWER_STEPS_PER_WAYPOINT = 30
DRAWER_PRESS_PENETRATION_M = 0.01
#: Success bar: within 0.01 of the `drawer_slide` joint's 0.15 range limit
#: (PLAN.md M06 tests/test_skills.py spec).
DRAWER_SUCCESS_QPOS = 0.14


# ---------------------------------------------------------------------------
# Low-level MuJoCo lookups and control-vector helpers, shared by every skill.
# ---------------------------------------------------------------------------


def _actuator_id(model, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid == -1:
        raise ValueError(f"actuator {name!r} not found in the compiled model")
    return aid


def _joint_id(model, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid == -1:
        raise ValueError(f"joint {name!r} not found in the compiled model")
    return jid


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _site_id(model, name: str) -> int:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if sid == -1:
        raise ValueError(f"site {name!r} not found in the compiled model")
    return sid


def _gripper_ctrl(model, arm: str, fraction: float) -> float:
    """Map a 0..1 fraction to an absolute ctrl value within `armX_gripper`'s
    joint range (see `GRIPPER_OPEN_FRACTION`/`GRIPPER_CLOSE_FRACTION`)."""
    jid = _joint_id(model, ik.gripper_joint_name(arm))
    lo, hi = model.jnt_range[jid]
    return float(lo + fraction * (hi - lo))


def _hold_ctrl(env) -> np.ndarray:
    """A full-length ctrl vector that commands every actuator to hold its
    CURRENT joint position -- the safe default for whichever arm is not
    being actively driven this step (ADR-010: "the idle arm [is held] at a
    safe pose" while the other arm works the shared workspace).
    """
    ctrl = np.zeros(env.model.nu, dtype=np.float64)
    for aid in range(env.model.nu):
        jid = int(env.model.actuator_trnid[aid, 0])
        qadr = int(env.model.jnt_qposadr[jid])
        ctrl[aid] = env.data.qpos[qadr]
    return ctrl


def _write_arm_ctrl(ctrl: np.ndarray, model, arm: str, joint_angles, gripper_ctrl: float) -> None:
    """Overwrite `ctrl`'s slice for one arm's 5 positioning actuators plus
    its jaw actuator, in place. `ctrl` is expected to already hold sane
    values (e.g. from `_hold_ctrl`) for every OTHER actuator."""
    for name, angle in zip(ik.arm_joint_names(arm), joint_angles):
        ctrl[_actuator_id(model, name)] = float(angle)
    ctrl[_actuator_id(model, ik.gripper_joint_name(arm))] = gripper_ctrl


def _drive_to_target(
    env,
    arm: str,
    target_pos,
    gripper_fraction: float,
    max_steps: int,
    pos_tol: float = POS_CONVERGENCE_TOL_M,
) -> tuple[bool, int]:
    """One closed-loop phase: repeatedly solve IK toward `target_pos` and
    step physics (holding the other arm still via `_hold_ctrl`) until the
    arm's gripperframe site is within `pos_tol` of the target, or
    `max_steps` `env.step()` calls have been spent.

    Returns (converged, steps_used). `converged=False` at `max_steps` is a
    normal outcome for a hard subgoal, not necessarily a bug -- the caller
    decides what that means for overall skill success/timeout.
    """
    if max_steps <= 0:
        return False, 0

    target = np.asarray(target_pos, dtype=np.float64).reshape(3)
    site_id = _site_id(env.model, ik.gripperframe_site_name(arm))
    gripper_ctrl = _gripper_ctrl(env.model, arm, gripper_fraction)

    steps = 0
    while steps < max_steps:
        solution = ik.solve_position_ik(env.model, env.data, arm, target)
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
        env.step(ctrl)
        steps += 1

        actual = env.data.site_xpos[site_id]
        if np.linalg.norm(target - actual) < pos_tol:
            return True, steps
    return False, steps


def _dwell(env, arm: str, hold_pos, gripper_fraction: float, n_steps: int) -> int:
    """Hold `arm`'s gripperframe near `hold_pos` for `n_steps`, commanding
    `gripper_fraction` on the jaw throughout. Used to let a grasp/release
    settle (contact + PD) without asking the arm to travel anywhere.
    Returns the number of steps actually taken (may be less than
    `n_steps` if the caller is out of step budget -- callers pass
    `min(N, remaining)`).
    """
    gripper_ctrl = _gripper_ctrl(env.model, arm, gripper_fraction)
    target = np.asarray(hold_pos, dtype=np.float64).reshape(3)
    for i in range(n_steps):
        solution = ik.solve_position_ik(env.model, env.data, arm, target)
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
        env.step(ctrl)
    return n_steps


# ---------------------------------------------------------------------------
# Skills
# ---------------------------------------------------------------------------


def run_pick(env, arm: str, target_object: str, step_budget: int = ik.DEFAULT_STEP_BUDGET) -> SkillResult:
    """pick(object, arm): approach, descend, close, lift.

    Targets the object body named in `OBJECT_BODY_NAME`, offset by
    `GRASP_POINT_OFFSET_M` to a small graspable feature (see module
    docstring). Success: the object's world z rises at least
    `PICK_LIFT_MARGIN_M` above its own height measured at the start of
    this call.
    """
    frames = 0
    body_name = OBJECT_BODY_NAME.get(target_object)
    if body_name is None:
        return SkillResult(False, f"unknown target_object {target_object!r}", frames)

    body_id = _body_id(env.model, body_name)
    obj_pos0 = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
    initial_z = float(obj_pos0[2])
    offset = GRASP_POINT_OFFSET_M.get(target_object, np.zeros(3))
    grasp_point = obj_pos0 + offset

    open_frac, close_frac = GRIPPER_OPEN_FRACTION, GRIPPER_CLOSE_FRACTION
    remaining = step_budget

    # Phase 1: hover above the grasp point, gripper open.
    hover = grasp_point + np.array([0.0, 0.0, APPROACH_HEIGHT_M])
    _, used = _drive_to_target(env, arm, hover, open_frac, remaining)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout during approach; frames_used={frames}", frames)

    # Phase 2: descend onto the grasp point, gripper still open.
    _, used = _drive_to_target(env, arm, grasp_point, open_frac, remaining)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout during descent; frames_used={frames}", frames)

    # Phase 3: close the gripper and dwell so contact/friction settles
    # before we ask the arm to move again.
    used = _dwell(env, arm, grasp_point, close_frac, min(GRASP_DWELL_STEPS, remaining))
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout during grasp dwell; frames_used={frames}", frames)

    # Phase 4: lift straight up, gripper held closed.
    lift_target = grasp_point + np.array([0.0, 0.0, LIFT_HEIGHT_M])
    _, used = _drive_to_target(env, arm, lift_target, close_frac, remaining)
    frames += used
    remaining -= used

    final_z = float(env.data.xpos[body_id][2])
    lifted = final_z >= initial_z + PICK_LIFT_MARGIN_M
    reason = (
        f"{'lifted' if lifted else 'did not lift'} {target_object}: "
        f"initial_z={initial_z:.4f} final_z={final_z:.4f} "
        f"margin_required={PICK_LIFT_MARGIN_M}"
    )
    if lifted:
        return SkillResult(True, reason, frames)
    if remaining <= 0:
        return SkillResult(False, f"timeout; {reason}", frames)
    return SkillResult(False, reason, frames)


def run_place(
    env,
    arm: str,
    target_object: str,
    destination: str = "table",
    step_budget: int = ik.DEFAULT_STEP_BUDGET,
) -> SkillResult:
    """place(object, target=table, arm): pick the object up (if not already
    held), transport it to a new tabletop location, release, and retreat.

    Success: the object ends resting on the table (z within a plausible
    resting band) and within the tabletop's xy bounds -- not fallen through
    and not flown off the edge.
    """
    frames = 0
    body_name = OBJECT_BODY_NAME.get(target_object)
    if body_name is None:
        return SkillResult(False, f"unknown target_object {target_object!r}", frames)
    if destination != "table":
        return SkillResult(False, f"unsupported destination {destination!r} (only 'table' is implemented)", frames)

    body_id = _body_id(env.model, body_name)

    # `place` is self-contained: it performs the grasp itself rather than
    # assuming a prior `pick` already ran, so it is independently testable
    # (tests/test_skills.py calls `place` directly at seed=0, object still on
    # the table).
    pick_budget = max(1, step_budget * 2 // 3)
    pick_result = run_pick(env, arm, target_object, step_budget=pick_budget)
    frames += pick_result.frames_used
    if not pick_result.success:
        return SkillResult(False, f"place aborted: pick failed ({pick_result.reason})", frames)

    remaining = step_budget - frames
    if remaining <= 0:
        return SkillResult(False, f"timeout after pick; frames_used={frames}", frames)

    obj_xy = np.array(env.data.xpos[body_id][:2], dtype=np.float64, copy=True)
    dest_xy = obj_xy + np.array(PLACE_OFFSET_XY_M)
    # Keep the destination safely inside the tabletop (verified scene
    # geometry: x in [-0.40, 0.40], y in [-0.25, 0.25]) with margin so the
    # object cannot be released right at the edge.
    dest_xy[0] = float(np.clip(dest_xy[0], -0.30, 0.30))
    dest_xy[1] = float(np.clip(dest_xy[1], -0.18, 0.18))

    close_frac, open_frac = GRIPPER_CLOSE_FRACTION, GRIPPER_OPEN_FRACTION

    # Transport: carry the held object, still closed, to a point above the
    # destination.
    transport_target = np.array([dest_xy[0], dest_xy[1], TABLE_SURFACE_Z + LIFT_HEIGHT_M])
    _, used = _drive_to_target(env, arm, transport_target, close_frac, remaining)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout during transport; frames_used={frames}", frames)

    # Lower to the destination, gripper still closed.
    offset = GRASP_POINT_OFFSET_M.get(target_object, np.zeros(3))
    lower_target = np.array([dest_xy[0], dest_xy[1], TABLE_SURFACE_Z]) + offset
    _, used = _drive_to_target(env, arm, lower_target, close_frac, remaining)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout during lowering; frames_used={frames}", frames)

    # Release and dwell so the object settles before the arm retreats.
    used = _dwell(env, arm, lower_target, open_frac, min(RELEASE_DWELL_STEPS, remaining))
    frames += used
    remaining -= used

    # Retreat straight up so the arm does not drag the object off the table
    # as it withdraws.
    if remaining > 0:
        retreat_target = lower_target + np.array([0.0, 0.0, APPROACH_HEIGHT_M])
        _, used = _drive_to_target(env, arm, retreat_target, open_frac, remaining)
        frames += used
        remaining -= used

    final_pos = env.data.xpos[body_id]
    final_z = float(final_pos[2])
    on_table_height = (TABLE_SURFACE_Z - 0.05) <= final_z <= (TABLE_SURFACE_Z + 0.10)
    on_table_xy = -0.40 <= float(final_pos[0]) <= 0.40 and -0.25 <= float(final_pos[1]) <= 0.25
    reason = (
        f"final position of {target_object}: x={final_pos[0]:.4f} y={final_pos[1]:.4f} "
        f"z={final_z:.4f} (table surface z={TABLE_SURFACE_Z})"
    )
    if on_table_height and on_table_xy:
        return SkillResult(True, reason, frames)
    return SkillResult(False, f"object not resting on the table after place; {reason}", frames)


def run_handoff(
    env,
    to_arm: str,
    from_arm: str,
    target_object: str,
    step_budget: int = ik.DEFAULT_STEP_BUDGET,
) -> SkillResult:
    """handoff(object, from_arm, to_arm): `from_arm` picks the object up and
    presents it in the shared overlap band; `to_arm` approaches, closes;
    `from_arm` opens and retreats (ADR-010's scripted grip-state sequence).

    Success: the object ends measurably closer to `to_arm`'s gripperframe
    site than to `from_arm`'s, and has been lifted since the handoff began
    (the same lift-margin check `run_pick` uses), confirming `to_arm` is now
    the one actually holding it.
    """
    frames = 0
    if to_arm == from_arm:
        return SkillResult(False, f"handoff requires two different arms, got to_arm=from_arm={to_arm!r}", frames)
    body_name = OBJECT_BODY_NAME.get(target_object)
    if body_name is None:
        return SkillResult(False, f"unknown target_object {target_object!r}", frames)

    body_id = _body_id(env.model, body_name)
    initial_z = float(env.data.xpos[body_id][2])

    pick_budget = max(1, step_budget // 2)
    pick_result = run_pick(env, from_arm, target_object, step_budget=pick_budget)
    frames += pick_result.frames_used
    if not pick_result.success:
        return SkillResult(False, f"handoff aborted: pick by arm {from_arm} failed ({pick_result.reason})", frames)

    remaining = step_budget - frames
    if remaining <= 0:
        return SkillResult(False, f"timeout after pick; frames_used={frames}", frames)

    close_frac, open_frac = GRIPPER_CLOSE_FRACTION, GRIPPER_OPEN_FRACTION
    tx, ty = HANDOFF_TRANSFER_XY
    transfer_z = TABLE_SURFACE_Z + HANDOFF_TRANSFER_HEIGHT_M

    # `from_arm` presents the object at the shared-workspace transfer point.
    presenting_target = np.array([tx, ty, transfer_z])
    _, used = _drive_to_target(env, from_arm, presenting_target, close_frac, remaining)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout while presenting; frames_used={frames}", frames)

    # `to_arm` approaches from its own side, offset so the two gripper
    # mechanisms are not asked to occupy the same point (ADR-024).
    side = -1.0 if to_arm == "A" else 1.0  # arm A base at y=-0.25, arm B at y=+0.25
    receiving_target = np.array([tx, ty + side * HANDOFF_SIDE_OFFSET_M, transfer_z])
    _, used = _drive_to_target(env, to_arm, receiving_target, open_frac, remaining)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout while receiving; frames_used={frames}", frames)

    # Explicit grip-state sequence (ADR-010): close the receiver first...
    used = _dwell(env, to_arm, receiving_target, close_frac, min(GRASP_DWELL_STEPS, remaining))
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout during receive dwell; frames_used={frames}", frames)

    # ...then open and retreat the presenter.
    used = _dwell(env, from_arm, presenting_target, open_frac, min(RELEASE_DWELL_STEPS, remaining))
    frames += used
    remaining -= used
    if remaining > 0:
        retreat_target = presenting_target + np.array([0.0, side * -1.0 * 0.05, 0.05])
        _, used = _drive_to_target(env, from_arm, retreat_target, open_frac, remaining)
        frames += used
        remaining -= used

    if remaining > 0:
        lift_target = receiving_target + np.array([0.0, 0.0, LIFT_HEIGHT_M])
        _, used = _drive_to_target(env, to_arm, lift_target, close_frac, remaining)
        frames += used
        remaining -= used

    final_pos = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
    final_z = float(final_pos[2])
    site_a = _site_id(env.model, ik.gripperframe_site_name("A"))
    site_b = _site_id(env.model, ik.gripperframe_site_name("B"))
    dist_a = float(np.linalg.norm(final_pos - env.data.site_xpos[site_a]))
    dist_b = float(np.linalg.norm(final_pos - env.data.site_xpos[site_b]))
    closer_to_to_arm = dist_b < dist_a if to_arm == "B" else dist_a < dist_b
    lifted = final_z >= initial_z + PICK_LIFT_MARGIN_M

    reason = (
        f"z={final_z:.4f} (initial {initial_z:.4f}) dist_to_armA={dist_a:.4f} "
        f"dist_to_armB={dist_b:.4f} (to_arm={to_arm})"
    )
    if lifted and closer_to_to_arm:
        return SkillResult(True, f"held by arm {to_arm}: {reason}", frames)
    return SkillResult(False, f"object not confirmed held by arm {to_arm}: {reason}", frames)


def run_open_drawer(env, arm: str = "A", step_budget: int = ik.DEFAULT_STEP_BUDGET) -> SkillResult:
    """open_drawer(arm): friction-drag the drawer open (see module docstring
    for why -- no handle geometry exists to pinch).

    Presses down on the drawer box's top surface near its front edge, then
    walks the contact target toward -y (the drawer's open direction,
    `drawer_slide`'s axis) in small waypoints. Success: `drawer_slide`'s
    qpos reaches at least `DRAWER_SUCCESS_QPOS` (within 0.01 of its 0.15
    limit).
    """
    frames = 0
    jid = _joint_id(env.model, "drawer_slide")
    qadr = int(env.model.jnt_qposadr[jid])
    body_id = _body_id(env.model, "drawer")
    geom_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "drawer_box")
    if geom_id == -1:
        raise ValueError("geom 'drawer_box' not found in the compiled model")
    _hx, hy, hz = env.model.geom_size[geom_id]

    initial_qpos = float(env.data.qpos[qadr])
    drawer_pos0 = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
    drawer_x = float(drawer_pos0[0])
    box_top_z = float(drawer_pos0[2] + hz)
    # ~3 cm inside the box's front (most-negative-y) edge, read from the
    # geom's own half-extent `hy`, not a hardcoded coordinate.
    contact_y0 = float(drawer_pos0[1] - hy + 0.03)
    press_z = box_top_z - DRAWER_PRESS_PENETRATION_M

    close_frac = GRIPPER_CLOSE_FRACTION  # jaw state is irrelevant for a press-drag; kept closed/compact
    remaining = step_budget

    # Phase 1: hover above the initial contact point.
    hover = np.array([drawer_x, contact_y0, box_top_z + APPROACH_HEIGHT_M])
    _, used = _drive_to_target(env, arm, hover, close_frac, remaining)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(False, f"timeout during approach; drawer_slide qpos={float(env.data.qpos[qadr]):.4f}", frames)

    # Phase 2: press down onto the drawer's top surface.
    press_point = np.array([drawer_x, contact_y0, press_z])
    _, used = _drive_to_target(env, arm, press_point, close_frac, remaining, pos_tol=0.02)
    frames += used
    remaining -= used
    if remaining <= 0:
        return SkillResult(
            False, f"timeout before contact; drawer_slide qpos={float(env.data.qpos[qadr]):.4f}", frames
        )

    # Phase 3: drag. Advance the press point toward -y in waypoints; each
    # waypoint gets a few steps to let the arm catch up and friction pull
    # the drawer along.
    current_y = contact_y0
    for _ in range(DRAWER_MAX_WAYPOINTS):
        if remaining <= 0:
            break
        current_y -= DRAWER_WAYPOINT_STEP_M
        waypoint = np.array([drawer_x, current_y, press_z])
        steps_this_waypoint = min(DRAWER_STEPS_PER_WAYPOINT, remaining)
        _, used = _drive_to_target(env, arm, waypoint, close_frac, steps_this_waypoint, pos_tol=0.02)
        frames += used
        remaining -= used
        if float(env.data.qpos[qadr]) >= DRAWER_SUCCESS_QPOS:
            break

    final_qpos = float(env.data.qpos[qadr])
    success = final_qpos >= DRAWER_SUCCESS_QPOS
    reason = f"drawer_slide qpos: initial={initial_qpos:.4f} final={final_qpos:.4f} (limit 0.15, success>={DRAWER_SUCCESS_QPOS})"
    if success:
        return SkillResult(True, reason, frames)
    if remaining <= 0:
        return SkillResult(False, f"timeout; {reason}", frames)
    return SkillResult(False, f"drag stalled; {reason}", frames)
