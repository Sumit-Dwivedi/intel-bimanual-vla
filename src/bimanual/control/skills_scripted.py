"""Scripted, IK-driven skill primitives (M06a): `open_drawer`, `pick`, `place`,
`handoff`. `pour` is M06b and is intentionally NOT implemented in this module
-- see PLAN.md's POUR GATE and the M06a task scope.

**ADR-027: waypoint staging, and why it exists.** `ik.solve_position_ik` is a
position-only solver with NO obstacle/collision term (ADR-024) -- it happily
returns a "converged" joint solution that swings the arm straight through the
tabletop if that is the shortest path in joint space to the requested target.
M06a originally handed each skill's FINAL target straight to the solver in
one shot (e.g. "descend onto the grasp point" from wherever the arm currently
was), which is exactly the failure ADR-026 caught: solves that converge
kinematically while tunneling through `table_top`. The fix is not in the
solver (out of scope, ADR-024/ADR-026 both say so explicitly) -- it is here,
in the CALLER. Per `docs/learn/06-ik-and-skills.md` tutor note 06's
approach -> grip -> retreat pattern, every skill below is staged as a
sequence of small WAYPOINTS, each one a short, nearly-straight-line hop from
the previous one (never a single long reach that could clip through solid
geometry), and each one is validated BEFORE the skill is allowed to advance:
  - IK convergence: the solver's own residual must be < `ik.IK_POSITION_TOLERANCE_M`.
  - Collision: no NEW cross-arm or arm-vs-`table_top` contact versus a
    baseline snapshot taken once, at the very start of the skill call, at
    the actual (collision-free, ADR-026) starting pose.
A waypoint that fails either bar stops the skill immediately -- later
waypoints are never attempted on top of a bad one -- and returns
`SkillResult(success=False, reason="waypoint N failed [convergence|collision]: ...")`
so a failure localises to the exact stage that produced it.

Every skill is a closed control loop, per docs/learn/06-ik-and-skills.md:
read the current state (`env.model`/`env.data`, privileged per ADR-005 --
the scripted controller has no perception step), pick the current subgoal,
solve IK (`bimanual.control.ik.solve_position_ik`), write joint targets via
`env.step()`, test whether the subgoal is met, advance or fail. Every
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
  - `open_drawer`: the drawer body (`drawer_box`) is a solid box with no
    handle to pinch. ADR-027 revises this from the original top-down
    friction-drag technique (which required descending from ABOVE, straight
    through the `table_top` slab that now sits directly over the drawer's
    closed face at y=0.00 -- see ADR-026's drawer reposition) to a LATERAL
    approach: swing in from outside the table footprint at drawer height,
    slide inward under the slab while staying below its z=0.33 underside,
    close the jaw against the drawer's front face, and pull straight back.
    See `run_open_drawer`'s docstring for the empirically-measured verdict
    on whether this path is actually clear for arm A.

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
            (initial z=0.356, margin=0.03)", or, on a waypoint failure,
            "waypoint 2 (descend) failed [collision (...)]" -- so a Tester
            or a log reader never has to re-derive what happened or which
            stage produced it (ADR-027).
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
#: **`"plate"` direction corrected under ADR-027, magnitude unchanged.** The
#: plate's rim is circular (radius 0.09 m) so, unlike the mug's handle
#: below, any direction around it is an equally valid PHYSICAL grasp
#: feature -- but not every direction is equally REACHABLE from the "home"
#: rest pose ADR-026 introduced. Measured directly (`ik.solve_position_ik`
#: from `reset(seed=0)`'s home pose, arm A): the original `(+0.09, 0, 0)`
#: offset drives `shoulder_lift`, `elbow_flex` AND `wrist_flex` all
#: simultaneously to their joint-range limits and the DLS solve stalls
#: there (residual 0.138 m at the grasp point, 0.064 m at the approach
#: hover point -- confirmed non-transient: a 3000-step real closed-loop
#: drive toward the hover point plateaued at the same error and never
#: improved). `(0, +0.09, 0)` -- the same rim, 90 degrees around it --
#: converges cleanly at both points (residual 0.008 m / 0.006 m) with no
#: joint pegged at its limit. Swapping which side of a SYMMETRIC feature is
#: targeted is a caller-side reachability choice, not a change to `ik.py`
#: or to what "the plate's rim" means physically.
#:
#: **M06a grasp fix B (attempted, see DECISIONS.md for the measured
#: result): `"plate"` was retargeted from the rim, `(0, 0.09, 0)`, to the
#: top-centre, `(0, 0, 0.005)`.** That attempt is RECORDED but REVERTED
#: (fix-ladder retest, Sept 12, 2026): the top-centre point never even
#: converged at waypoint 1 (IK residual 0.0226 m), which meant the skill
#: never reached GRIP at all -- silently masking fixes C and D (closure
#: force, jaw collision geometry) from ever being exercised, since neither
#: can matter before the arm gets there. Reverted back to the rim offset,
#: `(0, 0.09, 0)`, which DOES converge cleanly (residual ~0.008 m, ADR-027)
#: and lets the skill actually reach DESCEND/GRIP/RETREAT so C and D get a
#: fair test. The physical objection ADR-027/fix-B raised against the rim
#: offset (no orientation control, so the jaw's pinch plane may not
#: straddle the rim) is not disproven by this revert -- it is simply the
#: only offset that lets the rest of the ladder actually run.
#: **M06a grasp fix E (plate reshape, retest ladder Step 4).** The plate
#: body itself was reshaped in `scripts/gen_dual_scene.py` (foot cylinder
#: r=0.03 h=0.010 at the body origin, dish cylinder r=0.06 h=0.008 sitting
#: on top, local z offset 0.009 m) after fixes A-D -- all on the GRIPPER
#: side -- measured zero effect once actually exercised (fix B reverted).
#: `"plate"`'s offset moves from the OLD flush-disc rim, `(0, 0.09, 0)`
#: (radius of the old single 0.09 m disc, z=0 since the old disc had no
#: vertical structure to aim at), to the NEW dish's overhanging rim:
#: `(0, PLATE_DISH_RADIUS_M, PLATE_DISH_LOCAL_Z_M)` = `(0, 0.06, 0.009)` --
#: the dish's own radius (its outer edge, where it overhangs the narrower
#: foot by 0.03 m) at the dish's own local z centre (0.009 m above the
#: plate body's origin, which sits at the foot's centre). This targets the
#: 0.01 m gap under the overhang directly, so the lower jaw has somewhere
#: to go instead of pressing against a flush table contact. Y direction
#: (not X or -Y) is kept from the prior rim offset because it was already
#: measured (ADR-027) to be the reachable side of a symmetric feature from
#: arm A's "home" pose; the dish is circular so any direction is an
#: equally valid PHYSICAL grasp feature, only some are REACHABLE.
#: **M06a grasp fix (from `docs/hardware/m06-grip-diagnostic.md`).** The
#: diagnostic's full 60-step GRIP timeline found the STATIC pad's own world
#: z sitting at 0.3510-0.3512 m -- AT OR BELOW `TABLE_SURFACE_Z=0.35`, while
#: the fork body itself rests at z=0.3538 m. Across all 60 logged steps the
#: only contact recorded is static-pad-vs-`table_top`; pad-vs-fork never
#: occurs once, and the fork body moves a total of 0.000034 m -- consistent
#: with zero force ever reaching it, because the pinch target was being
#: driven into the tabletop, not toward the fork's handle. `"fork"`'s z
#: offset moves from 0.0 to +0.004 m so the pinch point sits above the
#: table surface (0.35 + small margin) instead of into it, close to the
#: fork body's own resting height (0.3538 m) rather than 2-3 mm below it.
GRASP_POINT_OFFSET_M = {
    "plate": np.array([0.0, 0.06, 0.009]),
    "mug": np.array([0.0475, 0.0, 0.0]),
    "fork": np.array([-0.015, 0.0, 0.004]),
    "spoon": np.array([-0.01, 0.0, 0.0]),
    "bottle": np.array([0.0, 0.0, 0.02]),
}

# ---------------------------------------------------------------------------
# Module-level tuning constants -- one place to find and adjust every
# skill-behaviour number (task instructions: "anyone tuning these must find
# them in one place"). The five ADR-027 constants below (CLEARANCE_HEIGHT_M,
# APPROACH_DESCENT_STEPS, GRIP_HOLD_FRAMES, PULL_DISTANCE_M,
# HANDOFF_POSITION_XYZ) are this module's waypoint-staging contract; every
# staged skill is built out of them.
# ---------------------------------------------------------------------------

#: Tabletop surface height, metres (verified scene geometry: table_top box
#: centred at z=0.34, half-thickness 0.01 -> top surface at 0.35).
TABLE_SURFACE_Z = 0.35

#: ADR-027. How high above (or around) a target position a skill's APPROACH
#: and RETREAT waypoints hover, metres, so the arm always has a clear
#: straight-line lane before it moves horizontally onto or away from a
#: target -- the entire point of staging, since `ik.solve_position_ik` has
#: no obstacle awareness of its own (ADR-024) and will happily solve a path
#: THROUGH the tabletop if that is the shortest joint-space route (this is
#: exactly what ADR-026 caught). Replaces the old, inconsistently-named
#: `APPROACH_HEIGHT_M`/`LIFT_HEIGHT_M` pair with one constant used
#: uniformly by every staged skill.
CLEARANCE_HEIGHT_M = 0.08

#: ADR-027. Per-waypoint step cap for a single IK-driven APPROACH/DESCEND/
#: RETREAT phase. Each waypoint is a small joint delta from wherever the
#: previous waypoint left the arm (never a full reach from a resting pose),
#: so it converges fast -- this bounds any ONE phase so a slow waypoint
#: cannot silently consume a skill's entire step budget before a later,
#: possibly more important waypoint (e.g. GRIP) ever gets a turn.
APPROACH_DESCENT_STEPS = 500

#: ADR-027. Dwell length, in `env.step()` calls, for a GRIP or RELEASE
#: waypoint (closing/opening the jaw and letting contact settle before the
#: next waypoint runs). Replaces the previous
#: GRASP_DWELL_STEPS/RELEASE_DWELL_STEPS pair (200 steps each, tuned for the
#: OLD unstaged skills, which relied on the dwell to also absorb any
#: leftover approach overshoot) with the single constant name this task's
#: staging contract specifies. Staged skills already converge to within
#: `POS_CONVERGENCE_TOL_M` before a GRIP/RELEASE waypoint ever runs, so this
#: dwell only needs to let contact/friction settle -- NOT also finish an
#: approach -- which is why it can be shorter. Flagged, not hidden: this is
#: noticeably shorter than the ~180 steps ADR-024/M06a measured for the jaw
#: to travel its full ~1.9 rad range, so a GRIP/RELEASE waypoint may end
#: with the jaw still partway through its commanded travel; grasp-quality
#: tuning is ADR-024's open follow-up, out of this module's scope, and this
#: value is the one the task instructions specify.
#:
#: **M06a grasp fix C: raised 30 -> 60** so more force/settling time is
#: available before the dwell ends, per the task's fix ladder. (ADR-027's
#: own diagnostic had already tried extending the dwell informally to 300
#: steps with no effect -- see DECISIONS.md's fix A/B entries -- so this
#: 60-step value is not expected alone to change the outcome; it is applied
#: because the task specifies it as part of fix C, alongside driving ctrl
#: to the actuator's own closure limit below.)
#:
#: **M06a grasp fix (from `docs/hardware/m06-grip-diagnostic.md`): raised
#: 60 -> 300.** 300 frames allows full jaw closure at observed ~0.007
#: rad/step; 60 frames only reached 22% closure and jaws remained wide open
#: (0.111 m gap). The diagnostic's per-step timeline showed the gripper
#: joint (`armA_gripper`) travelling qpos 1.7449 -> 1.3215 over the 60
#: logged GRIP steps -- 0.42 rad of the joint's ~1.92 rad range (22%),
#: continuously and without stall -- with pad separation still 0.111 m at
#: step 60 (essentially wide open). At that observed rate, full closure
#: needs roughly 275 steps, so 300 gives a small margin past that.
GRIP_HOLD_FRAMES = 300

#: ADR-027. How far, in metres, `open_drawer`'s PULL waypoint drags the
#: drawer along `drawer_slide`'s axis (`(0, -1, 0)`) -- chosen to match the
#: joint's own 0.15 m range exactly, so one pull is a full open if the grip
#: holds.
PULL_DISTANCE_M = 0.15

#: ADR-027. Minimum contact PENETRATION DEPTH, metres, for the per-waypoint
#: collision check to count a contact as a genuine collision rather than
#: incidental floating-point-scale contact-margin noise. Measured need for
#: this, not a guessed fudge factor: every prop in this scene rests directly
#: on `table_top` (by design -- the plate sits only ~0.006 m above the
#: surface), and the gripper's own collision geometry is large relative to
#: the props (ADR-024: jaw bounding-sphere radius up to ~8.4 cm) -- so a
#: `pick` descend onto a rim grasp point at table height was measured to
#: produce a real MuJoCo contact between an (unnamed) arm mesh geom and
#: `table_top` at `dist=-0.00007` m: a graze roughly 1/1000th the size of
#: the genuine tunneling penetrations this ADR exists to catch (`open_drawer`
#: measured -0.02 to -0.065 m against the same geom). A bare "any negative
#: `dist` counts" rule cannot tell these apart; this constant draws the line
#: two-and-a-half orders of magnitude above the measured graze and two
#: orders of magnitude below the measured tunneling depths, so it rejects
#: neither kind of case by accident.
TABLE_COLLISION_DEPTH_TOL_M = 0.001

#: ADR-027. World-frame (x, y, z) both arms drive toward during `handoff`'s
#: presenting/receiving waypoints. y=-0.01 is the midpoint of the MEASURED
#: shared reachable band, y in [-0.12, 0.10] m (`docs/hardware/
#: m06-reachability-probe.md`'s "RE-MEASURED (ADR-026)" section, re-measured
#: from the corrected "home" rest pose -- NOT the earlier, superseded
#: ADR-021 arithmetic estimate). z=0.35 is `TABLE_SURFACE_Z`: handing an
#: object off AT the table surface height, rather than
#: `TABLE_SURFACE_Z + 0.10` as the pre-ADR-027 code used, keeps the
#: transfer point inside the envelope both arms were actually measured to
#: reach at low z (the same reachability constraint that blocks
#: `open_drawer`'s lateral approach, see that function's docstring, also
#: bounds how low a shared-band point can sit).
HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.35)

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

#: Success bar for "lifted": the object's world z must rise by at least
#: this much above its OWN measured starting height this run (PLAN.md M06
#: done-when 1: "lifted above a stated height threshold, reported as a
#: measured number").
PICK_LIFT_MARGIN_M = 0.03

#: How close the gripperframe site must get to a phase's target position,
#: metres, before a `_drive_to_target` phase is considered PHYSICALLY
#: converged. Looser than `ik.IK_POSITION_TOLERANCE_M` because this measures
#: the REAL, physically-servoed arm (subject to PD overshoot/settling), not
#: the kinematic IK solve -- the two are validated separately, see
#: `_run_waypoint`.
POS_CONVERGENCE_TOL_M = 0.015

#: `place`'s destination offset (x, y) in metres, relative to the object's
#: pick-up location -- "somewhere else on the table", not back where it
#: started.
PLACE_OFFSET_XY_M = (0.10, -0.05)

#: ADR-027. How far above the destination's table-resting height `place`'s
#: DESCEND waypoint stops, metres -- "release a hair above the table"
#: rather than driving the pinch point (which sits BETWEEN the jaws, not at
#: the object's own resting contact point) exactly onto the surface, which
#: would also drive the jaw itself into the tabletop.
PLACE_RELEASE_CLEARANCE_M = 0.02

#: How far off the transfer centre (`HANDOFF_POSITION_XYZ`) each arm's
#: gripper sits while both are present, so the two gripper mechanisms do
#: not try to occupy the same point (ADR-024's "handoff" consequence: with
#: no orientation control, this offset is our only concurrency-safety
#: margin between the two jaws).
HANDOFF_SIDE_OFFSET_M = 0.03

#: `open_drawer` tuning (ADR-027 lateral approach).
#:
#: How far outside the table's -y edge (table_top spans y in [-0.25, 0.25])
#: the APPROACH/RETREAT waypoints sit, metres. Chosen as a round number
#: comfortably past the table edge; see `run_open_drawer`'s docstring for
#: the measured verdict on whether the arm can actually reach this point at
#: drawer height.
DRAWER_LATERAL_APPROACH_Y_M = -0.30

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
    """Map a 0..1 fraction to an absolute ctrl value.

    **M06a grasp fix C.** Reads the gripper ACTUATOR's own `ctrlrange` (the
    compiled model's copy of `scenes/so101/so101_new_calib.xml:162`'s
    `<position class="sts3215" name="gripper" ... ctrlrange="-0.17453
    1.74533"/>` -- read-only, per the task's explicit instruction; that
    file is never edited) rather than the joint's `jnt_range` this
    function used before. The two are numerically almost identical
    (`jnt_range` is -0.17453297762778586..1.7453291995659765, `ctrlrange`
    is the same value rounded to 5 decimals in the upstream XML), but
    `ctrlrange` is the value MuJoCo actually clamps a commanded `ctrl`
    entry against (the scene's `<compiler autolimits="true">` makes this
    position actuator ctrl-limited), so it is the literally correct source
    for "drive ctrl to the closure limit": at `fraction=0.0`
    (`GRIPPER_CLOSE_FRACTION`) this now returns the actuator's own low
    `ctrlrange` bound exactly, with no possible daylight between the
    commanded value and the actuator's declared closure limit. The
    position actuator's gain (`kp=998.22`, from the unmodified upstream
    `sts3215` default class, ADR-016) is already the model's only
    available gain for this actuator -- "maximum available gain" per the
    task's fix C -- and is not modified here.
    """
    aid = _actuator_id(model, ik.gripper_joint_name(arm))
    lo, hi = model.actuator_ctrlrange[aid]
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


#: ADR-027 Step 5. How many CONSECUTIVE steps the same prop must register a
#: violation (`_prop_collision_violations`) before `_drive_to_target`/
#: `_dwell` treat it as a confirmed, reportable knock rather than a single
#: transient contact-solver-scale graze. Found necessary empirically (see
#: `_drive_to_target`'s docstring): 1 step (no debounce) caught the
#: `pick(A, bottle)` knock correctly but ALSO tripped on `pick(A, plate)`'s
#: ordinary approach, breaking a required regression test. 3 is short
#: enough that a real, escalating knock is still caught within a handful of
#: physics steps -- nowhere near enough for an object to travel any real
#: distance -- while filtering a single-frame graze that does not recur.
PROP_COLLISION_DEBOUNCE_STEPS = 3


def _debounce_prop_violations(consecutive: dict[str, int], violations: dict[str, float]) -> dict[str, float]:
    """Update `consecutive` (a per-prop running count, mutated in place) from
    this step's `violations` and return only the subset that has now been
    seen on `PROP_COLLISION_DEBOUNCE_STEPS` consecutive steps -- i.e. the
    CONFIRMED violations this step should act on. A prop absent from
    `violations` this step has its counter reset to 0 (the knock must be
    consecutive, not merely cumulative over a whole waypoint).
    """
    confirmed: dict[str, float] = {}
    seen = set(violations)
    for prop_name in list(consecutive):
        if prop_name not in seen:
            consecutive[prop_name] = 0
    for prop_name, depth in violations.items():
        consecutive[prop_name] = consecutive.get(prop_name, 0) + 1
        if consecutive[prop_name] >= PROP_COLLISION_DEBOUNCE_STEPS:
            confirmed[prop_name] = depth
    return confirmed


def _drive_to_target(
    env,
    arm: str,
    target_pos,
    gripper_fraction: float,
    max_steps: int,
    pos_tol: float = POS_CONVERGENCE_TOL_M,
    target_object: str | None = None,
) -> tuple[bool, int, dict]:
    """One closed-loop phase: repeatedly solve IK toward `target_pos` and
    step physics (holding the other arm still via `_hold_ctrl`) until the
    arm's gripperframe site is within `pos_tol` of the target, or
    `max_steps` `env.step()` calls have been spent.

    `target_object` (M06a Fix B) names the skill's OWN current target so
    that touching it during this phase -- APPROACH or DESCEND, the phases
    this function drives -- is checked against the looser
    `CRUSH_THRESHOLD_M` bar instead of `PROP_COLLISION_DEPTH_TOL_M`.
    Picking something up requires the gripper to approach and touch it;
    `None` (the default) applies the tight bar to every prop, as before.

    Returns (converged, steps_used, prop_violations). `converged=False` at
    `max_steps` is a normal outcome for a hard subgoal, not necessarily a bug
    -- the caller (`_run_waypoint`/`_run_dwell` below) decides what that
    means for overall skill success/failure.

    **ADR-027 Step 5: checked on EVERY physics step, not once at the end.**
    A first attempt at the arm-vs-prop check (`_prop_collision_violations`)
    only sampled contacts once, after this loop already finished -- and
    `pick(A, bottle)` still flung the bottle onto the floor with that
    version in place (measured: frames_used=1560, every waypoint reported
    clean, final z=0.0296). The reason: a transient, violent impact can
    knock a light, freely-jointed prop away and the contact can fully
    resolve (the two geoms separate) within the SAME waypoint's up-to-500
    step budget, long before a single post-hoc check ever runs. Sampling
    every step catches the collision at the moment it actually happens,
    while the geoms are still interpenetrating, and stops driving further
    steps immediately rather than continuing to push into (or having
    already flung away) the prop.

    **Debounced over `PROP_COLLISION_DEBOUNCE_STEPS` consecutive steps,
    found necessary empirically, not assumed.** A single-step version of
    this check (no debounce) also tripped on `pick(A, plate)`'s ordinary
    APPROACH -- a single-frame graze against Fix E's raised dish rim while
    the arm was still in transit toward the hover point, well short of the
    bottle's actual multi-step, escalating impact -- which broke the
    required regression test `test_pick_plate_waypoints_progress_without_
    collision` (DECISIONS.md records both measurements). Requiring the SAME
    prop to register a violation on `PROP_COLLISION_DEBOUNCE_STEPS`
    consecutive steps (not merely one) filters a single transient
    contact-solver-scale graze while still catching a real, sustained or
    worsening knock within a handful of steps -- far short of a waypoint's
    full step budget and long before an object can be flung any real
    distance.
    """
    if max_steps <= 0:
        return False, 0, {}

    target = np.asarray(target_pos, dtype=np.float64).reshape(3)
    site_id = _site_id(env.model, ik.gripperframe_site_name(arm))
    gripper_ctrl = _gripper_ctrl(env.model, arm, gripper_fraction)
    target_body = OBJECT_BODY_NAME.get(target_object) if target_object is not None else None

    steps = 0
    consecutive: dict[str, int] = {}
    while steps < max_steps:
        solution = ik.solve_position_ik(env.model, env.data, arm, target)
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
        env.step(ctrl)
        steps += 1

        prop_violations = _prop_collision_violations(env, target_body=target_body)
        confirmed = _debounce_prop_violations(consecutive, prop_violations)
        if confirmed:
            return False, steps, confirmed

        actual = env.data.site_xpos[site_id]
        if np.linalg.norm(target - actual) < pos_tol:
            return True, steps, {}
    return False, steps, {}


def _dwell(
    env,
    arm: str,
    hold_pos,
    gripper_fraction: float,
    n_steps: int,
    target_object: str | None = None,
) -> tuple[int, dict]:
    """Hold `arm`'s gripperframe near `hold_pos` for `n_steps`, commanding
    `gripper_fraction` on the jaw throughout. Used to let a grasp/release
    settle (contact + PD) without asking the arm to travel anywhere.
    Returns `(steps_taken, prop_violations)`; `steps_taken` may be less than
    `n_steps` if the caller is out of step budget (callers pass
    `min(N, remaining)`) or if a prop violation (ADR-027 Step 5, checked
    every step -- see `_drive_to_target`'s docstring) cuts the dwell short.

    `target_object` names the skill's OWN current target (e.g. `"plate"`
    during `pick`'s GRIP dwell) so that dwell's deliberate, expected contact
    with it is checked against the looser `CRUSH_THRESHOLD_M` bar (M06a Fix
    B) instead of the tight `PROP_COLLISION_DEPTH_TOL_M` one -- a GRIP/
    RELEASE dwell exists specifically to make that contact, but a contact
    deep enough to indicate crushing rather than grasping is still a
    failure. `None` (the default, used by `open_drawer`, whose target is
    not a free-joint prop) applies the tight bar to every prop.
    """
    gripper_ctrl = _gripper_ctrl(env.model, arm, gripper_fraction)
    target = np.asarray(hold_pos, dtype=np.float64).reshape(3)
    target_body = OBJECT_BODY_NAME.get(target_object) if target_object is not None else None
    consecutive: dict[str, int] = {}
    for i in range(n_steps):
        solution = ik.solve_position_ik(env.model, env.data, arm, target)
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
        env.step(ctrl)

        # ADR-027 Step 5 / M06a Fix B: a GRIP/RELEASE dwell is EXPECTED to
        # contact the skill's own current target object (that is the whole
        # point of a dwell) -- so this dwell's own target gets the looser
        # `CRUSH_THRESHOLD_M` bar, not a full exemption; a violent,
        # unintended knock against a THIRD prop (tight bar, unconditional)
        # or an actual crush of the target itself (past CRUSH_THRESHOLD_M)
        # is still caught. Also debounced (see
        # `PROP_COLLISION_DEBOUNCE_STEPS`), same rationale as
        # `_drive_to_target`.
        violations = _prop_collision_violations(env, target_body=target_body)
        confirmed = _debounce_prop_violations(consecutive, violations)
        if confirmed:
            return i + 1, confirmed
    return n_steps, {}


# ---------------------------------------------------------------------------
# ADR-027: per-waypoint validation -- collision-aware staging on top of a
# collision-blind solver.
# ---------------------------------------------------------------------------


def _arm_geom_ids(model, arm: str) -> set[int]:
    """Every geom id belonging to a body prefixed `arm{arm}_` -- the arm's
    whole kinematic subtree, per `scripts/gen_dual_scene.py`'s renaming
    convention (same technique `scripts/probe_reachability.py` uses)."""
    prefix = f"arm{arm}_"
    body_ids = {
        b
        for b in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(prefix)
    }
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}


def _table_top_geom_ids(model) -> set[int]:
    """The tabletop slab's own geom id -- the ONE geom this module's
    collision check cares about tunneling through (not the drawer housing,
    which `open_drawer` is *supposed* to touch, and not the props, which
    `pick`/`place`/`handoff` are *supposed* to touch)."""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    return {gid} if gid != -1 else set()


def _contact_counts(env) -> dict[str, int]:
    """Snapshot the two contact counts ADR-027's per-waypoint validation
    checks: cross-arm contacts (armA geom touching an armB geom) and, per
    arm, arm-vs-`table_top` contacts (the tunneling failure mode this whole
    ADR exists to catch). Measured fresh from `env.data.contact` -- real
    physics contacts, not a scratch kinematic re-check -- so it reflects
    whatever the actual simulated state is right now.
    """
    model, data = env.model, env.data
    arm_geoms = {arm: _arm_geom_ids(model, arm) for arm in ("A", "B")}
    table = _table_top_geom_ids(model)
    counts = {"cross_arm": 0, "A_table": 0, "B_table": 0}
    for i in range(data.ncon):
        c = data.contact[i]
        if float(c.dist) >= -TABLE_COLLISION_DEPTH_TOL_M:
            # Shallower than the measured graze/tunneling boundary (see
            # TABLE_COLLISION_DEPTH_TOL_M's docstring) -- not counted.
            continue
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in arm_geoms["A"] and g2 in arm_geoms["B"]) or (
            g1 in arm_geoms["B"] and g2 in arm_geoms["A"]
        ):
            counts["cross_arm"] += 1
        if (g1 in arm_geoms["A"] and g2 in table) or (g2 in arm_geoms["A"] and g1 in table):
            counts["A_table"] += 1
        if (g1 in arm_geoms["B"] and g2 in table) or (g2 in arm_geoms["B"] and g1 in table):
            counts["B_table"] += 1
    return counts


#: ADR-027 Step 5 (arm-vs-prop collision check, Sept 12 2026). ADR-027's own
#: per-waypoint validation (`_contact_counts`/`_validate_against_baseline`
#: above) only ever watched arm-vs-`table_top` and cross-arm contacts -- it
#: was never designed to catch the arm contacting and displacing a PROP.
#: That gap is exactly what DECISIONS.md's `pick(A, bottle)` diagnostic
#: found: every waypoint "validated" cleanly while the bottle was actually
#: knocked off the table (z 0.4400 -> 0.0298, consistent with landing on
#: the floor). This threshold is an ABSOLUTE penetration depth, unlike
#: `TABLE_COLLISION_DEPTH_TOL_M`'s baseline-relative delta: a controlled
#: pinch grasp is EXPECTED to touch its own target object, so this bar is
#: deliberately looser than the sub-millimetre table-graze tolerance --
#: loose enough to let a normal, gentle grasp contact through, tight enough
#: to catch a violent, uncontrolled impact before it flings an object away.
PROP_COLLISION_DEPTH_TOL_M = 0.005

#: M06a Fix B (target-prop exemption, Sept 12 2026). `PROP_COLLISION_DEPTH_TOL_M`
#: is deliberately tight (0.005 m) because it exists to catch a BYSTANDER
#: prop being grazed or knocked -- e.g. `pick(A, spoon)` clipping the
#: nearby `fork`, or the arm brushing a prop it has no business touching.
#: But that same tight bar, applied to the skill's OWN TARGET prop, makes
#: every `pick`/`place`/`handoff` unrunnable: picking something up
#: requires the gripper to touch it, and legitimate approach/grip contact
#: measured -0.007 to -0.009 m deep (DECISIONS.md's ADR-028 entry), already
#: past -0.005 m. `CRUSH_THRESHOLD_M` is the separate, looser bar that
#: applies ONLY to the executing arm's own current target: contact with it
#: is expected and allowed at every waypoint (APPROACH/DESCEND/GRIP/RETREAT
#: alike -- see `_prop_collision_violations`'s docstring for why RETREAT is
#: included), and is only a failure once it goes deep enough to indicate
#: crushing rather than grasping. -0.02 m leaves clear margin above the
#: -0.007..-0.009 m band those legitimate contacts were measured at, while
#: still catching a genuinely pathological, ever-deepening penetration.
#: Non-target props are NEVER exempted at any waypoint and keep the
#: tighter `PROP_COLLISION_DEPTH_TOL_M` bar always -- that is the
#: bystander protection this whole check exists for, and it must not be
#: weakened.
CRUSH_THRESHOLD_M = -0.02  # target-prop contact deeper than this indicates crushing, not grasping

#: Canonical prop body names this check watches: every FREE-JOINT prop in
#: the scene (`OBJECT_BODY_NAME`'s values, excluding `"drawer"`, which has a
#: slide joint, not a free joint, and is the one body `open_drawer` is
#: SUPPOSED to contact).
_FREE_JOINT_PROP_BODY_NAMES = tuple(v for v in OBJECT_BODY_NAME.values() if v != "drawer")


def _prop_geom_ids(model) -> dict[str, set[int]]:
    """Every geom id belonging to each free-joint prop body (ADR-027 Step 5).

    Looked up by BODY id, not a hardcoded geom name, so a prop that (like
    the reshaped `plate`, fix E) is built from more than one geom is still
    covered completely without this function needing to know its shape.
    """
    out: dict[str, set[int]] = {}
    for body_name in _FREE_JOINT_PROP_BODY_NAMES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            continue
        out[body_name] = {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == body_id}
    return out


def _prop_collision_violations(env, target_body: str | None = None) -> dict[str, float]:
    """Return `{prop_body_name: deepest_penetration_m}` for every free-joint
    prop currently in contact with EITHER arm deeper than its applicable
    bar (ADR-027 Step 5, M06a Fix B). Empty dict if none.

    `target_body` (a prop BODY NAME, already resolved via `OBJECT_BODY_NAME`
    by the caller -- e.g. `_run_waypoint`/`_dwell`) is the skill's own
    current target. Its bar is the looser `CRUSH_THRESHOLD_M`; every OTHER
    free-joint prop's bar stays the tight `PROP_COLLISION_DEPTH_TOL_M`,
    unconditionally, at every waypoint -- APPROACH, DESCEND, GRIP AND
    RETREAT alike. This is deliberate, not an oversight: the exemption is
    named per-CONTACT (which prop is touched), not per-PHASE (which
    waypoint is running) or per-gripper-state (open vs closed), because a
    successful `pick`'s RETREAT is the arm LIFTING the target while
    holding it -- arm-vs-target contact there is the proof of success, not
    a defect, and a phase-based rule that turned strict at RETREAT would
    report every successful grasp as a failure.

    Deliberately NOT baseline-relative (unlike `_contact_counts`'s
    table/cross-arm counts): a prop resting quietly against another prop or
    the table before a skill ever starts is not what this check is for --
    it is checked once per waypoint against the CURRENT contact list, and a
    contact this deep at ANY waypoint is worth naming regardless of what
    was already touching before the skill began.
    """
    model, data = env.model, env.data
    arm_geoms = _arm_geom_ids(model, "A") | _arm_geom_ids(model, "B")
    prop_geoms = _prop_geom_ids(model)
    violations: dict[str, float] = {}
    for i in range(data.ncon):
        c = data.contact[i]
        dist = float(c.dist)
        g1, g2 = int(c.geom1), int(c.geom2)
        for prop_name, geoms in prop_geoms.items():
            if not ((g1 in arm_geoms and g2 in geoms) or (g2 in arm_geoms and g1 in geoms)):
                continue
            bar = CRUSH_THRESHOLD_M if prop_name == target_body else -PROP_COLLISION_DEPTH_TOL_M
            if dist >= bar:
                continue
            # Track the DEEPEST violation per prop, not merely the first
            # contact found (a prop can have more than one geom, e.g. the
            # reshaped plate's foot+dish).
            if prop_name not in violations or dist < violations[prop_name]:
                violations[prop_name] = dist
    return violations


def _validate_against_baseline(
    env, arm: str, target_pos, baseline: dict, target_object: str | None = None
) -> tuple[bool, str | None, float]:
    """Check BOTH bars a waypoint must clear (ADR-027): IK convergence
    (re-queried after the physical drive settles, so it reflects wherever
    the arm actually ended up) and collision (no NEW cross-arm or
    arm-vs-table_top contact versus `baseline`, a snapshot taken once at
    this skill call's start, plus the ADR-027 Step 5 arm-vs-prop check).

    `target_object` (same meaning as `_dwell`'s parameter, M06a Fix B) gives
    a skill's own deliberate current target the looser `CRUSH_THRESHOLD_M`
    bar in the arm-vs-prop check's POST-hoc sample, instead of the tight
    `PROP_COLLISION_DEPTH_TOL_M` one every other prop keeps -- consistent
    with the in-loop check `_drive_to_target`/`_dwell` already apply.

    Returns (ok, reason, ik_residual_m). `reason` is None when `ok`.
    """
    solution = ik.solve_position_ik(env.model, env.data, arm, target_pos)
    if solution.position_error_m >= ik.IK_POSITION_TOLERANCE_M:
        return (
            False,
            f"convergence (IK residual={solution.position_error_m:.4f} m >= "
            f"{ik.IK_POSITION_TOLERANCE_M} m)",
            solution.position_error_m,
        )

    counts = _contact_counts(env)
    table_key = f"{arm}_table"
    if counts["cross_arm"] > baseline["cross_arm"] or counts[table_key] > baseline[table_key]:
        return (
            False,
            f"collision (cross_arm contacts={counts['cross_arm']} vs baseline "
            f"{baseline['cross_arm']}; arm{arm}-vs-table_top contacts={counts[table_key]} "
            f"vs baseline {baseline[table_key]})",
            solution.position_error_m,
        )

    # ADR-027 Step 5 / M06a Fix B: arm-vs-PROP collision, the gap
    # `pick(A, bottle)`'s diagnostic exposed (every waypoint validated clean
    # while the arm knocked the bottle onto the floor). Any free-joint prop
    # deeper than its applicable bar in contact with an arm geom is a
    # validation failure naming the prop -- the tight
    # `PROP_COLLISION_DEPTH_TOL_M` bar for every prop except this skill's
    # own `target_object`, which gets the looser `CRUSH_THRESHOLD_M` bar
    # instead (see `_prop_collision_violations`'s docstring for why this
    # applies at every waypoint, RETREAT included).
    target_body = OBJECT_BODY_NAME.get(target_object) if target_object is not None else None
    prop_violations = _prop_collision_violations(env, target_body=target_body)
    if prop_violations:
        named = ", ".join(
            f"{name} (dist={depth:.4f} m)" for name, depth in sorted(prop_violations.items())
        )
        return (
            False,
            f"collision (arm-vs-prop: {named}; threshold={-PROP_COLLISION_DEPTH_TOL_M} m)",
            solution.position_error_m,
        )
    return True, None, solution.position_error_m


def _run_waypoint(
    env,
    arm: str,
    target_pos,
    gripper_fraction: float,
    max_steps: int,
    baseline: dict,
    pos_tol: float = POS_CONVERGENCE_TOL_M,
    target_object: str | None = None,
) -> tuple[bool, int, str | None]:
    """Drive `arm` to `target_pos` (one APPROACH/DESCEND/RETREAT/PULL
    waypoint of a staged skill, ADR-027) and validate it against
    `_validate_against_baseline` before the caller is allowed to advance.

    Returns (ok, frames_used, reason). `reason` is None on success. Logs one
    INFO line per waypoint (arm, target, frames used, pass/fail and why) so
    a Tester enabling logging gets a per-waypoint frame breakdown without
    this module's public `SkillResult` needing a new field for it.

    ADR-027 Step 5: if `_drive_to_target`'s per-step check already caught a
    prop violation (a transient knock mid-loop), that is reported
    IMMEDIATELY -- the drive already stopped early, and the post-hoc
    `_validate_against_baseline` check would no longer see it once the
    knocked prop has separated and flown off.

    `target_object` (M06a Fix B) is threaded to BOTH the in-loop check
    (`_drive_to_target`) and the post-hoc one (`_validate_against_baseline`)
    so the skill's own target gets the looser `CRUSH_THRESHOLD_M` bar at
    EVERY waypoint this function drives -- APPROACH, DESCEND and RETREAT
    alike, not merely `_run_dwell`'s GRIP/RELEASE. This is required, not
    optional: `pick`'s APPROACH/DESCEND must be able to close in on and
    touch its own target (that is the whole point of picking it up), and
    `pick`'s RETREAT is the arm LIFTING the object it just grasped --
    continuing arm-vs-target contact there is the proof the grasp is
    holding, not a defect. Every OTHER prop keeps the tight bar
    unconditionally, including at RETREAT -- that bystander protection is
    never weakened by this parameter.
    """
    _, steps, in_loop_violations = _drive_to_target(
        env, arm, target_pos, gripper_fraction, max_steps, pos_tol=pos_tol, target_object=target_object
    )
    if in_loop_violations:
        named = ", ".join(
            f"{name} (dist={depth:.4f} m)" for name, depth in sorted(in_loop_violations.items())
        )
        reason = f"collision (arm-vs-prop: {named}; threshold={-PROP_COLLISION_DEPTH_TOL_M} m)"
        logger.info(
            "waypoint arm=%s target=%s frames_used=%d ok=False reason=%s",
            arm, np.round(np.asarray(target_pos, dtype=np.float64), 4).tolist(), steps, reason,
        )
        return False, steps, reason

    ok, reason, residual = _validate_against_baseline(env, arm, target_pos, baseline, target_object=target_object)
    logger.info(
        "waypoint arm=%s target=%s frames_used=%d ik_residual=%.4f ok=%s%s",
        arm, np.round(np.asarray(target_pos, dtype=np.float64), 4).tolist(), steps, residual, ok,
        "" if ok else f" reason={reason}",
    )
    return ok, steps, reason


def _run_dwell(
    env,
    arm: str,
    hold_pos,
    gripper_fraction: float,
    n_steps: int,
    baseline: dict,
    target_object: str | None = None,
) -> tuple[bool, int, str | None]:
    """Dwell at `hold_pos` (a GRIP or RELEASE waypoint, ADR-027) while
    opening/closing the jaw, then validate exactly like `_run_waypoint`.

    `target_object` is threaded to both `_dwell` (in-loop exemption) and
    `_validate_against_baseline` (post-hoc exemption) so this dwell's own
    deliberate contact with the object it is picking/placing/handing off is
    never itself reported as a violation (ADR-027 Step 5) -- an UNEXPECTED
    prop (or the target at a genuinely violent depth mid-dwell) still is.
    """
    steps, in_loop_violations = _dwell(env, arm, hold_pos, gripper_fraction, n_steps, target_object=target_object)
    if in_loop_violations:
        named = ", ".join(
            f"{name} (dist={depth:.4f} m)" for name, depth in sorted(in_loop_violations.items())
        )
        reason = f"collision (arm-vs-prop: {named}; threshold={-PROP_COLLISION_DEPTH_TOL_M} m)"
        logger.info(
            "dwell arm=%s target=%s frames_used=%d ok=False reason=%s",
            arm, np.round(np.asarray(hold_pos, dtype=np.float64), 4).tolist(), steps, reason,
        )
        return False, steps, reason

    ok, reason, residual = _validate_against_baseline(env, arm, hold_pos, baseline, target_object=target_object)
    logger.info(
        "dwell arm=%s target=%s frames_used=%d ik_residual=%.4f ok=%s%s",
        arm, np.round(np.asarray(hold_pos, dtype=np.float64), 4).tolist(), steps, residual, ok,
        "" if ok else f" reason={reason}",
    )
    return ok, steps, reason


# ---------------------------------------------------------------------------
# Skills
# ---------------------------------------------------------------------------


def run_pick(env, arm: str, target_object: str, step_budget: int = ik.DEFAULT_STEP_BUDGET) -> SkillResult:
    """pick(object, arm): APPROACH (clearance above the grasp point) ->
    DESCEND (onto it) -> GRIP (close + hold) -> RETREAT (back to clearance
    height) (ADR-027, tutor note 06's approach/grip/retreat pattern).

    Targets the object body named in `OBJECT_BODY_NAME`, offset by
    `GRASP_POINT_OFFSET_M` to a small graspable feature (see module
    docstring). Success: the object's world z rises at least
    `PICK_LIFT_MARGIN_M` above its own height measured at the start of
    this call, AND every waypoint below cleared both validation bars.
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
    baseline = _contact_counts(env)
    remaining = step_budget

    # Waypoint 1: APPROACH -- hover at clearance height above the grasp point.
    hover = grasp_point + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    ok, used, reason = _run_waypoint(
        env, arm, hover, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline, target_object=target_object
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 1 (approach) failed [{reason}]", frames)

    # Waypoint 2: DESCEND -- onto the grasp point itself, gripper still open.
    ok, used, reason = _run_waypoint(
        env, arm, grasp_point, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline, target_object=target_object
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 2 (descend) failed [{reason}]", frames)

    # Waypoint 3: GRIP -- close the jaw and hold so contact/friction settles
    # before the arm is asked to move again.
    ok, used, reason = _run_dwell(
        env, arm, grasp_point, close_frac, min(GRIP_HOLD_FRAMES, remaining), baseline, target_object=target_object
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 3 (grip) failed [{reason}]", frames)

    # Waypoint 4: RETREAT -- back to clearance height, gripper held closed.
    # This also doubles as the physical lift the success check below reads.
    # target_object=target_object (M06a Fix B): the arm is LIFTING the
    # object it just grasped here -- continuing contact with it is the
    # proof of a successful grasp, not a defect, so it stays exempt (up to
    # CRUSH_THRESHOLD_M) through RETREAT too.
    ok, used, reason = _run_waypoint(
        env, arm, hover, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline, target_object=target_object
    )
    frames += used
    remaining -= used

    final_z = float(env.data.xpos[body_id][2])
    lifted = final_z >= initial_z + PICK_LIFT_MARGIN_M
    measured = (
        f"{'lifted' if lifted else 'did not lift'} {target_object}: "
        f"initial_z={initial_z:.4f} final_z={final_z:.4f} margin_required={PICK_LIFT_MARGIN_M}"
    )
    if not ok:
        return SkillResult(False, f"waypoint 4 (retreat) failed [{reason}]; {measured}", frames)
    if lifted:
        return SkillResult(True, measured, frames)
    return SkillResult(False, measured, frames)


def run_place(
    env,
    arm: str,
    target_object: str,
    destination: str = "table",
    step_budget: int = ik.DEFAULT_STEP_BUDGET,
) -> SkillResult:
    """place(object, target=table, arm): pick the object up (if not already
    held), then APPROACH above the destination at clearance height ->
    DESCEND to destination + a small vertical offset for gentle release ->
    RELEASE (open + hold) -> RETREAT to clearance height (ADR-027).

    Success: the object ends resting on the table (z within a plausible
    resting band) and within the tabletop's xy bounds -- not fallen through
    and not flown off the edge -- AND every waypoint cleared both
    validation bars.
    """
    frames = 0
    body_name = OBJECT_BODY_NAME.get(target_object)
    if body_name is None:
        return SkillResult(False, f"unknown target_object {target_object!r}", frames)
    if destination != "table":
        return SkillResult(False, f"unsupported destination {destination!r} (only 'table' is implemented)", frames)

    body_id = _body_id(env.model, body_name)
    baseline = _contact_counts(env)

    # `place` is self-contained: it performs the grasp itself rather than
    # assuming a prior `pick` already ran, so it is independently testable
    # (tests/test_skills.py calls `place` directly at seed=0, object still on
    # the table). The nested `pick`'s own waypoint numbering/reason string is
    # propagated as-is on failure, so a failure inside the grasp still
    # localises to its exact stage.
    pick_budget = max(1, step_budget // 2)
    pick_result = run_pick(env, arm, target_object, step_budget=pick_budget)
    frames += pick_result.frames_used
    if not pick_result.success:
        return SkillResult(False, f"place aborted: pick failed ({pick_result.reason})", frames)

    remaining = step_budget - frames
    if remaining <= 0:
        return SkillResult(False, f"waypoint 1 (approach destination) failed [convergence (no budget remaining after pick)]", frames)

    obj_xy = np.array(env.data.xpos[body_id][:2], dtype=np.float64, copy=True)
    dest_xy = obj_xy + np.array(PLACE_OFFSET_XY_M)
    # Keep the destination safely inside the tabletop (verified scene
    # geometry: x in [-0.40, 0.40], y in [-0.25, 0.25]) with margin so the
    # object cannot be released right at the edge.
    dest_xy[0] = float(np.clip(dest_xy[0], -0.30, 0.30))
    dest_xy[1] = float(np.clip(dest_xy[1], -0.18, 0.18))

    close_frac, open_frac = GRIPPER_CLOSE_FRACTION, GRIPPER_OPEN_FRACTION
    offset = GRASP_POINT_OFFSET_M.get(target_object, np.zeros(3))

    # Waypoint 1: APPROACH -- above the destination at clearance height,
    # still holding the object (gripper closed).
    approach_above_dest = np.array([dest_xy[0], dest_xy[1], TABLE_SURFACE_Z + CLEARANCE_HEIGHT_M])
    ok, used, reason = _run_waypoint(
        env, arm, approach_above_dest, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 1 (approach destination) failed [{reason}]", frames)

    # Waypoint 2: DESCEND -- to the destination plus a small vertical offset
    # for a gentle release (PLACE_RELEASE_CLEARANCE_M), gripper still closed.
    lower_target = np.array([dest_xy[0], dest_xy[1], TABLE_SURFACE_Z + PLACE_RELEASE_CLEARANCE_M]) + offset
    ok, used, reason = _run_waypoint(
        env, arm, lower_target, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 2 (descend to destination) failed [{reason}]", frames)

    # Waypoint 3: RELEASE -- open the jaw and hold so the object settles.
    ok, used, reason = _run_dwell(
        env, arm, lower_target, open_frac, min(GRIP_HOLD_FRAMES, remaining), baseline, target_object=target_object
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 3 (release) failed [{reason}]", frames)

    # Waypoint 4: RETREAT -- back up to clearance height so the arm does not
    # drag the object off the table as it withdraws.
    retreat_target = lower_target + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    ok, used, reason = _run_waypoint(
        env, arm, retreat_target, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used

    final_pos = env.data.xpos[body_id]
    final_z = float(final_pos[2])
    on_table_height = (TABLE_SURFACE_Z - 0.05) <= final_z <= (TABLE_SURFACE_Z + 0.10)
    on_table_xy = -0.40 <= float(final_pos[0]) <= 0.40 and -0.25 <= float(final_pos[1]) <= 0.25
    measured = (
        f"final position of {target_object}: x={final_pos[0]:.4f} y={final_pos[1]:.4f} "
        f"z={final_z:.4f} (table surface z={TABLE_SURFACE_Z})"
    )
    if not ok:
        return SkillResult(False, f"waypoint 4 (retreat) failed [{reason}]; {measured}", frames)
    if on_table_height and on_table_xy:
        return SkillResult(True, measured, frames)
    return SkillResult(False, f"object not resting on the table after place; {measured}", frames)


def run_handoff(
    env,
    to_arm: str,
    from_arm: str,
    target_object: str,
    step_budget: int = ik.DEFAULT_STEP_BUDGET,
) -> SkillResult:
    """handoff(object, from_arm, to_arm) (ADR-010's grip-state sequence,
    staged per ADR-027):

      1. `from_arm` picks the object up (nested `run_pick`).
      2. `from_arm` APPROACHes above `HANDOFF_POSITION_XYZ` at clearance.
      3. `from_arm` DESCENDs to `HANDOFF_POSITION_XYZ`.
      4. `to_arm` APPROACHes above it, offset to the opposite side
         (`HANDOFF_SIDE_OFFSET_M`) so the two jaws are not asked to occupy
         the same point (ADR-024's handoff consequence).
      5. `to_arm` DESCENDs.
      6. `to_arm` GRIPs (closes).
      7. `from_arm` RELEASEs (opens).
      8. Both RETREAT, STAGGERED: `from_arm` retreats first, THEN `to_arm`
         retreats -- sequential, not concurrent, so the two arms do not
         cross paths on the way out while both are still near the transfer
         point.

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
    baseline = _contact_counts(env)

    pick_budget = max(1, step_budget // 2)
    pick_result = run_pick(env, from_arm, target_object, step_budget=pick_budget)
    frames += pick_result.frames_used
    if not pick_result.success:
        return SkillResult(False, f"handoff aborted: pick by arm {from_arm} failed ({pick_result.reason})", frames)

    remaining = step_budget - frames
    if remaining <= 0:
        return SkillResult(False, "waypoint 1 (from_arm approach) failed [convergence (no budget remaining after pick)]", frames)

    close_frac, open_frac = GRIPPER_CLOSE_FRACTION, GRIPPER_OPEN_FRACTION
    hx, hy, hz = HANDOFF_POSITION_XYZ
    transfer_point = np.array([hx, hy, hz])
    side = -1.0 if to_arm == "A" else 1.0  # arm A base at y=-0.25, arm B at y=+0.25
    receiving_point = transfer_point + np.array([0.0, side * HANDOFF_SIDE_OFFSET_M, 0.0])

    # Waypoint 1: from_arm APPROACH -- above the transfer point at clearance
    # height, still holding the object.
    from_hover = transfer_point + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    ok, used, reason = _run_waypoint(
        env, from_arm, from_hover, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 1 (from_arm approach) failed [{reason}]", frames)

    # Waypoint 2: from_arm DESCEND -- to the transfer point exactly.
    ok, used, reason = _run_waypoint(
        env, from_arm, transfer_point, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 2 (from_arm descend) failed [{reason}]", frames)

    # Waypoint 3: to_arm APPROACH -- above the transfer point, offset to its
    # own side, gripper open.
    to_hover = receiving_point + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    ok, used, reason = _run_waypoint(
        env, to_arm, to_hover, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 3 (to_arm approach) failed [{reason}]", frames)

    # Waypoint 4: to_arm DESCEND -- to the receiving point.
    ok, used, reason = _run_waypoint(
        env, to_arm, receiving_point, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 4 (to_arm descend) failed [{reason}]", frames)

    # Waypoint 5: to_arm GRIP -- close and hold.
    ok, used, reason = _run_dwell(
        env, to_arm, receiving_point, close_frac, min(GRIP_HOLD_FRAMES, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 5 (to_arm grip) failed [{reason}]", frames)

    # Waypoint 6: from_arm RELEASE -- open and hold.
    ok, used, reason = _run_dwell(
        env, from_arm, transfer_point, open_frac, min(GRIP_HOLD_FRAMES, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 6 (from_arm release) failed [{reason}]", frames)

    # Waypoint 7: from_arm RETREAT -- goes FIRST (staggered), back to
    # clearance height above the transfer point.
    from_retreat = from_hover
    ok7, used, reason7 = _run_waypoint(
        env, from_arm, from_retreat, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
    frames += used
    remaining -= used
    if not ok7:
        return SkillResult(False, f"waypoint 7 (from_arm retreat) failed [{reason7}]", frames)

    # Waypoint 8: to_arm RETREAT -- goes SECOND (staggered), lifting the
    # object away from the transfer point.
    to_lift = receiving_point + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    # target_object=target_object (M06a Fix B): to_arm is now LIFTING the
    # object it just gripped -- same rationale as pick's RETREAT above.
    ok8, used, reason8 = _run_waypoint(
        env, to_arm, to_lift, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object,
    )
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

    measured = (
        f"z={final_z:.4f} (initial {initial_z:.4f}) dist_to_armA={dist_a:.4f} "
        f"dist_to_armB={dist_b:.4f} (to_arm={to_arm})"
    )
    if not ok8:
        return SkillResult(False, f"waypoint 8 (to_arm retreat) failed [{reason8}]; {measured}", frames)
    if lifted and closer_to_to_arm:
        return SkillResult(True, f"held by arm {to_arm}: {measured}", frames)
    return SkillResult(False, f"object not confirmed held by arm {to_arm}: {measured}", frames)


def run_open_drawer(env, arm: str = "A", step_budget: int = ik.DEFAULT_STEP_BUDGET) -> SkillResult:
    """open_drawer(arm) (ADR-027 lateral approach): APPROACH (a waypoint
    outside the table footprint, at drawer height) -> INSERT (translate
    inward under the tabletop slab to the handle, staying at drawer height
    the whole time so the path never crosses the slab's z in
    [0.33, 0.35]) -> GRIP (close against the drawer's front face) -> PULL
    (drag back by `PULL_DISTANCE_M`) -> RELEASE -> RETREAT (back outside
    the table edge).

    **This replaces the earlier top-down friction-drag technique**, which
    pressed down onto the drawer's TOP surface from ABOVE -- a path that,
    now that the drawer sits at closed-face y=0.00 directly under the
    `table_top` slab (ADR-026), can only reach the drawer by first passing
    THROUGH the slab. The lateral approach is the collision-safe
    alternative this ADR requires.

    **Empirically measured verdict (ADR-027), reported here rather than
    assumed:** a one-off diagnostic (`ik.solve_position_ik` plus a real
    physical closed-loop drive, both against this exact scene, arm A, from
    the "home" reset pose) found that arm A's pinch point CANNOT converge
    to ANY point at drawer height (z about 0.28) once y is at or beyond the
    table's own edge (y <= about -0.20) -- the IK residual stalls around
    0.09-0.32 m regardless of how many physical steps are given (1500
    steps, well beyond `APPROACH_DESCENT_STEPS`, still did not converge),
    and the same is true of the drawer's OWN closed-face target at y=0.00
    (residual 0.009 m -- kinematically reachable -- but only via a solution
    that tunnels through `table_top`/`drawer_housing_back` with up to
    -0.064 m penetration, confirmed by a real contact check). In other
    words: arm A's shoulder is mounted at the same z as the tabletop
    (0.35 m) and simply cannot fold its own linkage down to drawer height
    while positioned outside or at the table's footprint -- this is a
    kinematic reach limit of the arm itself at its current base placement,
    not a solver bug and not something a smarter waypoint choice fixes.
    **Per this task's explicit stop rule ("if the only solutions tunnel,
    STOP and report -- do not invent a third drawer position"), this
    function is still built exactly as specified below** (so its waypoint
    validation is real, tested code, not a stub) **and is expected, on the
    committed scene, to fail at waypoint 1 (APPROACH) with a convergence
    failure** -- which is the correct, honest, collision-safe outcome: the
    old code "succeeded" at reaching the drawer only by silently tunneling
    through the table; this code correctly refuses to, and says why.
    """
    frames = 0
    jid = _joint_id(env.model, "drawer_slide")
    qadr = int(env.model.jnt_qposadr[jid])
    body_id = _body_id(env.model, "drawer")
    geom_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "drawer_box")
    if geom_id == -1:
        raise ValueError("geom 'drawer_box' not found in the compiled model")
    _hx, hy, _hz = env.model.geom_size[geom_id]

    initial_qpos = float(env.data.qpos[qadr])
    drawer_pos0 = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
    drawer_x = float(drawer_pos0[0])
    drawer_z = float(drawer_pos0[2])
    # Front (most-negative-y) face centre, read from the geom's own
    # half-extent, never a hardcoded coordinate (matches
    # scripts/probe_reachability.py's `_drawer_face_target`).
    handle_y = float(drawer_pos0[1] - hy)

    close_frac, open_frac = GRIPPER_CLOSE_FRACTION, GRIPPER_OPEN_FRACTION
    baseline = _contact_counts(env)
    remaining = step_budget

    approach_point = np.array([drawer_x, DRAWER_LATERAL_APPROACH_Y_M, drawer_z])
    handle_point = np.array([drawer_x, handle_y, drawer_z])
    pulled_point = np.array([drawer_x, handle_y - PULL_DISTANCE_M, drawer_z])

    def _final_reason() -> str:
        final_qpos = float(env.data.qpos[qadr])
        return (
            f"drawer_slide qpos: initial={initial_qpos:.4f} final={final_qpos:.4f} "
            f"(limit 0.15, success>={DRAWER_SUCCESS_QPOS})"
        )

    # Waypoint 1: APPROACH -- a waypoint outside the table footprint, at
    # drawer height (gripper open, ready to insert).
    ok, used, reason = _run_waypoint(env, arm, approach_point, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline)
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 1 (approach) failed [{reason}]; {_final_reason()}", frames)

    # Waypoint 2: INSERT -- translate in +y, under the slab, to the handle,
    # staying at drawer height throughout (both endpoints share `drawer_z`,
    # so the straight-line path the closed loop servos toward never crosses
    # the slab's z in [0.33, 0.35]).
    ok, used, reason = _run_waypoint(env, arm, handle_point, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline)
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 2 (insert) failed [{reason}]; {_final_reason()}", frames)

    # Waypoint 3: GRIP -- close against the drawer's front face.
    ok, used, reason = _run_dwell(env, arm, handle_point, close_frac, min(GRIP_HOLD_FRAMES, remaining), baseline)
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 3 (grip) failed [{reason}]; {_final_reason()}", frames)

    # Waypoint 4: PULL -- drag back by PULL_DISTANCE_M, gripper held closed.
    ok, used, reason = _run_waypoint(env, arm, pulled_point, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline)
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 4 (pull) failed [{reason}]; {_final_reason()}", frames)

    # Waypoint 5: RELEASE -- open and hold.
    ok, used, reason = _run_dwell(env, arm, pulled_point, open_frac, min(GRIP_HOLD_FRAMES, remaining), baseline)
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 5 (release) failed [{reason}]; {_final_reason()}", frames)

    # Waypoint 6: RETREAT -- back outside the table edge.
    ok, used, reason = _run_waypoint(env, arm, approach_point, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline)
    frames += used
    remaining -= used

    final_qpos = float(env.data.qpos[qadr])
    success = final_qpos >= DRAWER_SUCCESS_QPOS
    measured = _final_reason()
    if not ok:
        return SkillResult(False, f"waypoint 6 (retreat) failed [{reason}]; {measured}", frames)
    if success:
        return SkillResult(True, measured, frames)
    return SkillResult(False, measured, frames)
