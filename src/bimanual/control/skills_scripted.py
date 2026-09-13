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

**Ctrl hold pattern per `docs/hardware/m06-phase2-prerequisites.md` Q2 --
`TableSettingEnv.step()` overwrites all 12 actuator targets unconditionally
(`env.py:260`), and the `home` keyframe declares qpos only, so an unheld idle
arm drifts toward qpos=0, the pre-ADR-026 cross-arm interpenetration pose.**
`env.step()` does no holding of its own -- it is a bare, unconditional
12-vector overwrite with no merge against the previous `data.ctrl` and no
reference to `qpos`. Holding the idle arm is entirely the CALLER's
responsibility, and every `env.step()` call in this module already
discharges it via `_hold_ctrl` (below), called immediately before
`_write_arm_ctrl` overwrites the actively-driven arm's own slice
(`_drive_to_target`/`_dwell`, ADR-010's "hold the idle arm at a safe pose").

**M06 Phase 2 Commit 1 decision: keep `_hold_ctrl` as the ONE ctrl-hold
mechanism; do not add a second, competing one.** The alternative considered
was pinning idle joints to a `skill_start_ctrl` snapshot taken once at
`ScriptedSkillExecutor.execute()`'s entry and holding it as a fixed setpoint
for the whole skill call, versus `_hold_ctrl`'s existing behaviour of
re-anchoring to wherever the idle joints CURRENTLY sit, fresh, every step.
Two reasons, both checked directly rather than assumed:
  1. **`skill_start_ctrl` pinning is semantically WRONG for `run_handoff`,
     not merely harder to wire.** `handoff`'s own staging
     (`run_handoff`'s docstring, steps 2-7) has `from_arm` drive to
     `HANDOFF_POSITION_XYZ` and then sit there HOLDING THE OBJECT while
     `to_arm` approaches, descends, and grips -- i.e. partway through a
     single `handoff` call, `from_arm` is "idle" (not the arm
     `_run_waypoint`/`_run_dwell` is actively driving) at a pose far from
     where it started the skill. Pinning idle joints to a snapshot taken at
     `execute()`'s entry would command `from_arm` back toward its FOLDED
     HOME pose while it is still holding the object at the transfer point --
     fighting the very coordination `handoff` depends on, not protecting it.
     `_hold_ctrl`'s "wherever it currently is" is the only one of the two
     designs that is correct for a skill whose active/idle role swaps
     mid-call.
  2. Relatedly, `ScriptedSkillExecutor.execute()` (`executor.py`) never
     calls `env.step()` itself -- every step happens deep inside this
     module's own nested call chain (`run_pick`/`run_place`/`run_handoff`/
     `run_open_drawer` -> `_run_waypoint`/`_run_dwell` -> `_drive_to_target`/
     `_dwell` -> `env.step()`), so implementing (1)'s wrong design would
     also require threading a snapshot through every one of those layers --
     a wide, invasive change with a wrong answer at the end of it.

**Measured limitation of the kept mechanism, found by
`scripts/probe_ctrl_hold.py` and NOT to be understated: `_hold_ctrl` does
not actually hold the idle arm still over a long dwell.** Because it
re-reads CURRENT qpos every step and commands exactly that (zero position
error at the instant of the read), it supplies no restoring force against
gravity between reads -- each step's small gravity-induced sag becomes the
next step's new "hold" target, so the arm's true reference ratchets away
from its original pose monotonically. Measured directly (100-2000
`env.step()` calls, only arm A driven, arm B held only by `_hold_ctrl`):
arm B's `shoulder_lift` drifted 0.0402 rad by step 100 (already over this
task's 0.01 rad bar), 0.461 rad by step 1000, and saturated at 0.546 rad by
roughly step 1500 -- which lands within noise of arm B's OWN
`shoulder_lift` joint's lower `jnt_range` bound (home -1.2, range floor
-1.7453; -1.2 - 0.546 = -1.746), i.e. the idle arm sags under gravity until
a hard mechanical stop catches it, not until any controller-side limit
does. This is a real, previously-unmeasured gap -- the Q2 audit was
explicitly read-only ("No probes run") and only established that
`_hold_ctrl` prevents the ADR-026 zero-pose case, not that it holds the arm
motionless. It is reported here rather than fixed: a correct fix (cache
the idle arm's own LAST ACTIVELY-COMMANDED ctrl and hold that fixed value,
refreshed only when that arm is next driven -- neither of this task's two
offered designs, since "current qpos" sags and "skill-start" is wrong per
point 1 above) is a different, larger change than either option on the
table for this scaffolding commit, and is flagged here as a Phase 2
follow-up rather than attempted under this commit's time-box.

The real gap the audit identified is orthogonal to which idle-hold POLICY is
used: it is that the protection lives HERE, in `skills_scripted.py`'s own
`_hold_ctrl`, not inside `env.py` or `executor.py`, so any FUTURE code path
that calls `env.step()` directly (e.g. Phase 2 Commit 2's `WeldGrasp` wiring)
must route through `_hold_ctrl` or reproduce its exact pattern, or it will
silently reintroduce the ADR-026 34-contact interpenetration the first time
it drives one arm while leaving the other's ctrl slice at 0.
`scripts/probe_ctrl_hold.py` exercises exactly this reproduce-the-pattern
case directly against `env.step()` (not through this module's own skill
loops) and reports the measured idle-arm drift.

**ADR-031: the ACTIVE arm is frozen during `_dwell` (GRIP/RELEASE), a
different mechanism from `_hold_ctrl` above (which only ever governed the
IDLE arm).** `b9e9cb4`'s own instrumentation found that re-solving
`ik.solve_position_ik` every dwell step against a fixed target chases the
PINCH POINT (ADR-025's midpoint of the fixed and moving jaw bodies), which
physically shifts as the jaw opens or closes -- so the old dwell loop
commanded the arm to retreat as the jaws closed, and `WeldGrasp`'s
proximity gate and closure gate never held true on the same frame. `_dwell`
now captures the driven arm's own 5 positioning-joint ctrl targets once,
before the dwell starts, and holds that fixed value for every step of the
dwell instead of re-solving IK -- see `_dwell`'s own docstring for the
measured before/after distances.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import mujoco
import numpy as np

from bimanual.control import ik
from bimanual.sim.grasp import WeldGrasp

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
        weld_attach_frame: M06 Phase 2 Commit 2 (ADR-030). The GLOBAL
            (whole-skill-call, not phase-local) `frames_used` count at the
            instant `WeldGrasp.attempt_grasp` first returned True for this
            skill's own grasp, or `None` if it never attached (either
            because the skill never reached a GRIP waypoint, or it reached
            one and exhausted `GRIP_HOLD_FRAMES` without attaching). This is
            what makes "attached late" and "never attached" distinguishable
            from a plain `success=False` -- see this module's `_dwell`/
            `_run_dwell` docstrings for how it is measured.
        weld_active_at_end: M06 Phase 2 Commit 2. Whether the relevant arm
            (the acting arm for `pick`, the RECEIVING arm for `handoff`)
            still holds the object via an active weld at the moment this
            `SkillResult` is returned -- i.e. `weld.is_holding(arm) ==
            object_name`, re-checked at return time rather than inferred
            from `weld_attach_frame` alone (a later RELEASE, or a transfer
            failure, can make the two disagree).
    """

    success: bool
    reason: str
    frames_used: int
    weld_attach_frame: int | None = None
    weld_active_at_end: bool = False


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

#: **ADR-033 (per-prop APPROACH/RETREAT hover height for tall props).**
#: `docs/hardware/m06-water-bottle-diagnostic.md` measured `pick(A,
#: water_bottle)` displacing the bottle ~0.157 m in x / -0.061 m in z from
#: its reset position DURING (non-convergent) APPROACH/DESCEND, before GRIP
#: even starts -- i.e. the bottle is knocked, not merely reached for late.
#: The geometry explains why: `hover`'s old formula (`grasp_point.z +
#: CLEARANCE_HEIGHT_M` = 0.46 + 0.08 = 0.54 m) sits BELOW the bottle's own
#: physical top. `scripts/gen_dual_scene.py`'s `water_bottle_cap` geom is
#: `pos="0 0 0.10" size="0.012 0.01"` (a cylinder, half-length 0.01) on top
#: of `water_bottle_body`'s `size="0.03 0.09"` cylinder -- so the cap's own
#: top surface sits at local z = 0.10 + 0.01 = 0.11 m above the body's
#: origin (BOTTLE_POS z=0.44 at reset), i.e. world z = 0.55 m -- 0.01 m
#: ABOVE the old "clearance" hover point (0.54 m). Every OTHER pickable
#: prop's `GRASP_POINT_OFFSET_M` already sits at or near its own physical
#: top (the fork/spoon/plate/mug are all short relative to their grasp
#: offset), so `CLEARANCE_HEIGHT_M` above the grasp point already clears
#: the whole object for them; only the bottle's grasp point (near its neck,
#: partway down a 0.20 m-tall body+cap) leaves its own upper structure
#: uncleared. This is consistent with the diagnostic's other finding (the
#: pinch point is frozen and stationary throughout GRIP, per ADR-031 -- the
#: knock is not an arm-chasing-the-jaw artifact, it happens earlier, while
#: the arm is swinging toward/through a "hover" point that was never
#: actually above the object).
#:
#: `OBJECT_TOP_LOCAL_Z_M` gives, for props whose physical top exceeds their
#: own grasp point (only the bottle, so far), that top height as a local z
#: offset above the object's own body origin (`obj_pos0`, read fresh from
#: `env.data.xpos` every `run_pick` call -- never hardcoded world Z). Used
#: only to compute `hover`'s z (below): `hover.z = max(grasp_point.z,
#: obj_pos0.z + OBJECT_TOP_LOCAL_Z_M.get(target_object, offset.z)) +
#: CLEARANCE_HEIGHT_M`. For every prop NOT listed here, `.get(...,
#: offset.z)` falls back to the SAME z used for `grasp_point`, making the
#: `max(...)` a no-op -- this deliberately leaves every other prop's hover
#: height, and therefore every existing passing/failing test for them,
#: completely unchanged. `CLEARANCE_HEIGHT_M` (the existing, measured-safe
#: margin every other prop already clears by) is still the margin ABOVE
#: whichever of the two candidate heights is larger, not a new number of
#: its own.
OBJECT_TOP_LOCAL_Z_M = {
    "bottle": 0.11,
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
#:
#: **M06 Phase 2 Commit 1: 0.08 -> 0.05 tried and REVERTED; left at 0.08.**
#: NOT justified by `m06-ik-lift-diagnostic.md`'s 68%-at-10cm/61%-at-15cm
#: single-shot-IK figures in any case -- per `m06-phase2-prerequisites.md`
#: Q1, every waypoint in this module uses the INCREMENTAL regime
#: (`_drive_to_target`/`_dwell` re-solve `ik.solve_position_ik` fresh
#: against `env.data`'s current state on every `env.step()`), never the
#: single-shot solve-once-and-hold regime those figures describe, so citing
#: them here would describe a control loop the skills never run. The
#: candidate reasons for 0.05 (smaller vertical excursion -> less workspace
#: swept per waypoint, smaller shared-band sweep during `handoff`, fewer
#: incremental per-step IK re-solves to converge) are geometrically
#: plausible, but checked empirically here rather than assumed, and the
#: check failed: at 0.05, `pick(A, plate)`'s APPROACH waypoint hovers close
#: enough over the plate's own raised rim (`GRASP_POINT_OFFSET_M["plate"]`)
#: that the jaw's collision geometry now grazes it during the approach --
#: `pytest tests/test_skills.py::test_pick_plate_waypoints_progress_
#: without_collision` (an ADR-027 regression test, previously passing)
#: newly fails with `waypoint 1 (approach) failed [collision (arm-vs-prop:
#: plate (dist=-0.0301 m); threshold=-0.005 m)]` -- an order of magnitude
#: past `TABLE_COLLISION_DEPTH_TOL_M`'s graze/tunnel boundary, not a
#: transient debounce artefact. Per this task's own instruction ("if you
#: find neither reason holds up, leave the constant at 0.08 rather than
#: making an unjustified change"), this is left at 0.08: the shorter
#: excursion's theoretical benefits do not outweigh a real, newly-introduced
#: collision against a prop this module already has to clear at 0.08.
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

#: **ADR-036: z raised 0.35 -> 0.43.** ADR-035 traced `handoff(A, B, fork)`'s
#: remaining failure to this exact number: 0.35 is simultaneously
#: `HANDOFF_POSITION_XYZ`'s z AND `TABLE_SURFACE_Z`, so the transfer point
#: sat exactly at tabletop height, and `to_arm`'s DESCEND waypoint down onto
#: it produced a real `armB`-vs-`table_top` contact past
#: `TABLE_COLLISION_DEPTH_TOL_M` (waypoint 4, step 2/3, target z=0.351 --
#: one millimetre above the slab). Two arms exchanging an object in mid-air
#: never need to touch the table at all, so the fix moves the exchange
#: itself off the tabletop plane: z=0.43 is `TABLE_SURFACE_Z +
#: CLEARANCE_HEIGHT_M`, the SAME height ADR-035's own fresh sweep this
#: session (`HANDOFF_STAGING_Y_M`'s docstring) already measured as a
#: converged, joint-limit-clean staging cell for BOTH arms, and the height
#: waypoints 1 and 3 (APPROACH) were already driving to before this change
#: (as `transfer_point + CLEARANCE_HEIGHT_M`). Raising the transfer point to
#: this already-verified height, rather than to some new number, means
#: `run_handoff`'s APPROACH waypoints do not change AT ALL numerically --
#: only the now-redundant DESCEND-to-table-height step (which is what
#: collided) is removed, see `run_handoff`'s docstring and body. y=-0.01
#: (the midpoint of the MEASURED shared reachable band, y in [-0.12, 0.10]
#: m, `docs/hardware/m06-reachability-probe.md`'s "RE-MEASURED (ADR-026)"
#: section) is unchanged -- only z moved.
HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.43)

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

#: **ADR-035 (target interpolation in the handoff traverse).**
#: `docs/hardware/m06-handoff-warmstart-diagnostic.md` found that a single
#: direct IK shot at `HANDOFF_POSITION_XYZ`'s z=0.35 plane fails for BOTH
#: arms' home-seeded solves in the middle of the shared workspace (residual
#: >14x tolerance), while a solve WARM-STARTED from a nearby already-
#: converged pose, then walked 0.02 m at a time toward the target, converges
#: at every single step, for both arms, across the full disjoint gap. This
#: constant is that 0.02 m step, applied by `_run_interpolated_waypoint`
#: below to the waypoint(s) that drive each arm from its APPROACH hover down
#: to its transfer/receiving point, and by `_run_approach_with_staging` to
#: an arm's very first move of a `handoff` call if that move does not
#: converge directly.
HANDOFF_INTERP_STEP_M = 0.02

#: ADR-035. Per-arm home-seeded, joint-limit-and-residual-converged staging
#: cell at hover height (`TABLE_SURFACE_Z + CLEARANCE_HEIGHT_M` = 0.43 m),
#: MEASURED this session (not assumed from the diagnostic's z=0.35 table,
#: which never checked hover height): a fresh sweep at z=0.43, home-seeded,
#: found arm A's own reachable band starts at y=+0.06 (residual 0.00999;
#: y=+0.04 fails at 0.04175) and arm B's mirror starts at y=-0.06 (residual
#: 0.00999; y=-0.04 fails at 0.04175) -- the same disjoint-band shape the
#: diagnostic found at z=0.35, just re-verified at the actual height
#: `run_handoff`'s APPROACH waypoint uses. Used ONLY as a first-hop staging
#: point when an arm's real APPROACH target does not converge in one shot
#: from wherever that arm currently is (this only actually happens for
#: whichever arm has not moved yet this `handoff` call, i.e. is still at
#: HOME) -- never as a demanded handoff position, and never applied when the
#: direct shot already succeeds.
HANDOFF_STAGING_Y_M = {"A": 0.06, "B": -0.06}

#: ADR-035. Minimum joint-limit margin (radians, despite the diagnostic
#: script's "_M" suffix on the equivalent constant -- it bounds joint ANGLE,
#: not position) an interpolation step's PHYSICALLY-REALIZED joint angles
#: must keep from either `jnt_range` bound. Mirrors
#: `scripts/probe_handoff_reachability_home.py`'s `JOINT_LIMIT_MARGIN_TOL_M`
#: gate exactly, closing the caveat the warm-start diagnostic explicitly
#: left open ("did not check joint-limit margins on the chained configs").
HANDOFF_JOINT_LIMIT_MARGIN_TOL = 1e-4

#: ADR-037 (sequential choreography, Phase 5). How far above
#: `HANDOFF_POSITION_XYZ`'s z `from_arm`'s FIRST retreat waypoint lifts to,
#: metres. Chosen so the resulting Euclidean distance from `from_arm`'s
#: gripperframe to the transfer point clears `HANDOFF_RETREAT_GATE_M`
#: (below) -- `CLEARANCE_HEIGHT_M` alone (0.08 m) is NOT enough (a pure
#: vertical lift of 0.08 m is only 0.08 m of total displacement, short of
#: the required > 0.10 m gate), so this is a SEPARATE, slightly taller
#: constant used only for `from_arm`'s own first retreat hop, verified
#: empirically (not assumed) -- see `run_handoff`'s Phase 5 and
#: DECISIONS.md's ADR-037 entry for the measured reachability/distance at
#: this height.
HANDOFF_FROM_ARM_RETREAT_CLEARANCE_M = 0.12

#: ADR-037 (sequential choreography, Phase 5). `from_arm` must retreat this
#: far (metres, Euclidean distance from its gripperframe site to the
#: transfer point) before `to_arm` is allowed to begin its OWN retreat --
#: the task's own Phase 5 spec ("once `from_arm`'s gripper is > 0.10 m from
#: the transfer point, `to_arm` retreats"), checked directly against the
#: PHYSICALLY-REALIZED site position, not assumed from the waypoint target
#: alone (PD settling can leave a small residual, ADR-027's own
#: `POS_CONVERGENCE_TOL_M` rationale).
HANDOFF_RETREAT_GATE_M = 0.10

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


def _hold_ctrl(env, frozen_base: np.ndarray | None = None) -> np.ndarray:
    """A full-length ctrl vector that commands every actuator to hold a
    position -- the safe default for whichever arm is not being actively
    driven this step (ADR-010: "the idle arm [is held] at a safe pose"
    while the other arm works the shared workspace).

    **ADR-037 fix: `frozen_base`, if given, is returned as a COPY directly
    -- nothing is re-derived from `env.data.qpos` in that case.** Before
    ADR-037, this function ALWAYS recomputed the returned vector fresh from
    CURRENT qpos every single call (see the measured drift this caused,
    below). `docs/hardware/m06-handoff-choreography.md`'s own re-measurement
    confirmed that gap directly: because re-reading qpos and commanding
    zero position error at that instant supplies no restoring force against
    gravity between reads, the idle arm's true setpoint ratchets away from
    its original pose monotonically -- 0.040 rad by 100 steps (already past
    a 0.01 rad bar), 0.546 rad (a joint hard-limit stop, not a controller
    limit) by roughly 1500 steps. A single `pick`'s ~655-frame trace alone
    implies ~0.26 rad of drift, and `run_handoff`'s sequential choreography
    (ADR-037) holds one arm idle across an entire multi-waypoint PHASE --
    several thousand frames -- so this is not a cosmetic gap for that
    caller. The fix follows the same insight as ADR-031's GRIP-dwell
    freeze: a caller doing a genuine long-duration idle hold captures a
    snapshot ONCE, at the instant the arm becomes idle (via a plain
    `_hold_ctrl(env)` call, no `frozen_base` -- which still reads live qpos
    for that ONE snapshot), and then passes that SAME array back in as
    `frozen_base` on every subsequent step of the idle span, however many
    waypoint/dwell calls that span covers. Because the idle arm's own ctrl
    entries are never touched by `_write_arm_ctrl` (only the ACTIVELY DRIVEN
    arm's slice is overwritten each step), holding the frozen array fixed
    means the idle arm's actuator setpoint is exactly what it was at the
    moment it stopped moving -- a real PD position error develops as soon as
    gravity sag begins, and the actuator's own gain (`kp=998.22`, ADR-016)
    corrects it, instead of the setpoint chasing the sag.

    `frozen_base=None` (every caller that existed before ADR-037: `run_pick`,
    `run_place`, `run_open_drawer`, and any `run_handoff` call site that
    does not opt in) reproduces this function's ORIGINAL qpos-chasing
    behaviour exactly, unchanged -- this is a purely additive, backward-
    compatible parameter; nothing about those callers' numerics changes
    (verified: see DECISIONS.md's ADR-037 entry for the pick/place/handoff
    re-run after this change).
    """
    if frozen_base is not None:
        return np.array(frozen_base, dtype=np.float64, copy=True)
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
    hold_ctrl_base: np.ndarray | None = None,
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

    `hold_ctrl_base` (ADR-037): passed straight through to `_hold_ctrl` as
    its `frozen_base` -- a caller doing a genuine long-duration idle hold
    (`run_handoff`'s sequential choreography) supplies the SAME captured
    array across many calls so the idle arm's setpoint never re-derives
    from (sagging) live qpos. `None` (the default) reproduces this
    function's pre-ADR-037 per-step qpos-chasing hold, unchanged.

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
        ctrl = _hold_ctrl(env, frozen_base=hold_ctrl_base)
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
    weld: WeldGrasp | None = None,
    weld_object_name: str | None = None,
    hold_ctrl_base: np.ndarray | None = None,
) -> tuple[int, dict, int | None]:
    """Hold `arm`'s gripperframe near `hold_pos` for `n_steps`, commanding
    `gripper_fraction` on the jaw throughout. Used to let a grasp/release
    settle (contact + PD) without asking the arm to travel anywhere.
    Returns `(steps_taken, prop_violations, attach_frame)`; `steps_taken`
    may be less than `n_steps` if the caller is out of step budget (callers
    pass `min(N, remaining)`), if a prop violation (ADR-027 Step 5, checked
    every step -- see `_drive_to_target`'s docstring) cuts the dwell short,
    or (M06 Phase 2 Commit 2) if `weld.attempt_grasp` attaches.

    `target_object` names the skill's OWN current target (e.g. `"plate"`
    during `pick`'s GRIP dwell) so that dwell's deliberate, expected contact
    with it is checked against the looser `CRUSH_THRESHOLD_M` bar (M06a Fix
    B) instead of the tight `PROP_COLLISION_DEPTH_TOL_M` one -- a GRIP/
    RELEASE dwell exists specifically to make that contact, but a contact
    deep enough to indicate crushing rather than grasping is still a
    failure. `None` (the default, used by `open_drawer`, whose target is
    not a free-joint prop) applies the tight bar to every prop.

    `weld`/`weld_object_name` (M06 Phase 2 Commit 2, ADR-030): if both are
    given, every step (after `env.step()`, so it sees the jaw's
    just-commanded closure) calls `weld.attempt_grasp(arm, weld_object_name)`.
    The FIRST step this returns True, the dwell stops immediately (a
    successful grasp does not need to keep dwelling) and the 1-based,
    dwell-LOCAL step index is returned as `attach_frame` (the caller adds
    its own frame offset to make this a whole-skill-call count, per
    `SkillResult.weld_attach_frame`'s docstring). If `n_steps` is exhausted
    without attaching, `attach_frame` is `None` -- the caller reports this
    as a `weld_attach_failed_after_N_frames` failure, not a generic one.
    `weld=None` (the default -- every RELEASE dwell, and `open_drawer`'s
    GRIP/RELEASE dwells, which have no weld concept) leaves this function's
    behavior identical to before this commit.

    `hold_ctrl_base` (ADR-037): same meaning as `_drive_to_target`'s
    parameter of the same name -- threaded to `_hold_ctrl` so a long idle
    hold spanning this dwell (and possibly other calls before/after it)
    uses one frozen snapshot instead of re-deriving from live qpos.
    `None` (the default) is unchanged pre-ADR-037 behaviour.

    **ADR-031: arm frozen for the whole dwell -- `hold_pos` is no longer
    re-solved against every step.** `ik.solve_position_ik` targets the
    PINCH POINT (ADR-025: the midpoint of the fixed and moving jaw BODIES,
    recomputed fresh from whatever the jaw's CURRENT open/closed angle is),
    not a fixed point on the arm -- so as a GRIP dwell closes the jaw (or a
    RELEASE dwell opens it), that midpoint physically shifts even though
    `hold_pos` itself does not. The old per-step loop re-solved IK toward
    the now-shifted-relative-to-`hold_pos` pinch point every step, which
    means it kept commanding the ARM to move so the midpoint would track
    the jaw's own motion -- i.e. the arm chased its own gripper closing,
    walking backward as the jaws closed. Measured directly (b9e9cb4's
    instrumentation, `pick(A, fork)`'s GRIP dwell): pinch-point distance to
    the target grew 0.0351 -> 0.0793 m and gripper-body distance grew
    0.0299 -> 0.0795 m over the dwell -- both starting BELOW `WeldGrasp`'s
    0.05 m proximity gate and ending ABOVE it, while the jaw closure gate
    (`qpos < 0.3`) needs about 150 steps to close -- so the two gates never
    held true on the same frame and the weld never attached. Fix: capture
    the arm's 5 positioning-joint ctrl targets ONCE, before this loop
    starts (wherever the preceding APPROACH/DESCEND waypoint already
    converged the arm to), and reapply that SAME frozen ctrl every step for
    the rest of the dwell -- no IK solve at all during the loop. Only the
    gripper joint's ctrl actually changes step to step (toward
    `gripper_fraction`). This removes the feedback loop entirely: the arm
    holds still (modulo ordinary PD droop under gravity, bounded by a fixed
    setpoint rather than a moving one) while the jaw closes or opens freely
    underneath it. Applies to EVERY call of this function -- GRIP dwells
    (`run_pick`'s/`run_handoff`'s own GRIP, `open_drawer`'s GRIP) and
    RELEASE dwells (`run_place`'s/`run_handoff`'s own RELEASE,
    `open_drawer`'s RELEASE) alike, since `_dwell` is the one shared
    implementation for both and the shifting-pinch-point problem is
    symmetric: an opening jaw shifts the same midpoint the same way, and
    the arm has no more business chasing it during RELEASE than during
    GRIP.
    """
    gripper_ctrl = _gripper_ctrl(env.model, arm, gripper_fraction)
    target_body = OBJECT_BODY_NAME.get(target_object) if target_object is not None else None
    consecutive: dict[str, int] = {}

    # ADR-031: freeze the arm's own 5 positioning-actuator ctrl targets for
    # the whole dwell, read once here from `env.data.ctrl` (i.e. whatever
    # the preceding waypoint already converged and left commanded) rather
    # than re-solving IK toward `hold_pos` every step below.
    arm_actuator_ids = [_actuator_id(env.model, name) for name in ik.arm_joint_names(arm)]
    frozen_arm_ctrl = np.array([env.data.ctrl[aid] for aid in arm_actuator_ids], dtype=np.float64)

    for i in range(n_steps):
        ctrl = _hold_ctrl(env, frozen_base=hold_ctrl_base)
        _write_arm_ctrl(ctrl, env.model, arm, frozen_arm_ctrl, gripper_ctrl)
        env.step(ctrl)

        if weld is not None and weld_object_name is not None:
            if weld.attempt_grasp(arm, weld_object_name):
                return i + 1, {}, i + 1

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
            return i + 1, confirmed, None
    return n_steps, {}, None


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
    hold_ctrl_base: np.ndarray | None = None,
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

    `hold_ctrl_base` (ADR-037): threaded straight through to
    `_drive_to_target`/`_hold_ctrl`. `None` (the default) is unchanged
    pre-ADR-037 behaviour.
    """
    _, steps, in_loop_violations = _drive_to_target(
        env, arm, target_pos, gripper_fraction, max_steps, pos_tol=pos_tol, target_object=target_object,
        hold_ctrl_base=hold_ctrl_base,
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
    weld: WeldGrasp | None = None,
    weld_object_name: str | None = None,
    hold_ctrl_base: np.ndarray | None = None,
) -> tuple[bool, int, str | None, int | None]:
    """Dwell at `hold_pos` (a GRIP or RELEASE waypoint, ADR-027) while
    opening/closing the jaw, then validate exactly like `_run_waypoint`.
    Returns `(ok, frames_used, reason, attach_frame)` -- the extra
    `attach_frame` element (M06 Phase 2 Commit 2, ADR-030) is `_dwell`'s own
    dwell-LOCAL attach index, unchanged here (the caller adds its own
    running frame offset to report a whole-skill-call frame number).

    **ADR-031: the arm no longer travels during this dwell.** `_dwell`
    itself now freezes the arm's 5 positioning-joint ctrl targets for the
    whole dwell instead of re-solving IK toward `hold_pos` every step (see
    `_dwell`'s own docstring for the measured before/after numbers and why
    -- the pinch point `ik.solve_position_ik` targets shifts as the jaw
    opens/closes, so re-solving toward a fixed `hold_pos` was commanding the
    arm to retreat). The post-hoc `_validate_against_baseline` call below is
    unaffected: it still re-solves IK once, after the dwell, to report a
    convergence residual for the waypoint's final resting pose.

    `target_object` is threaded to both `_dwell` (in-loop exemption) and
    `_validate_against_baseline` (post-hoc exemption) so this dwell's own
    deliberate contact with the object it is picking/placing/handing off is
    never itself reported as a violation (ADR-027 Step 5) -- an UNEXPECTED
    prop (or the target at a genuinely violent depth mid-dwell) still is.

    `weld`/`weld_object_name` (M06 Phase 2 Commit 2): passed straight
    through to `_dwell`; `None` (the default) leaves this function's
    behavior identical to before this commit.

    `hold_ctrl_base` (ADR-037): threaded straight through to `_dwell`/
    `_hold_ctrl`. `None` (the default) is unchanged pre-ADR-037 behaviour.
    """
    steps, in_loop_violations, attach_frame = _dwell(
        env, arm, hold_pos, gripper_fraction, n_steps, target_object=target_object,
        weld=weld, weld_object_name=weld_object_name, hold_ctrl_base=hold_ctrl_base,
    )
    if in_loop_violations:
        named = ", ".join(
            f"{name} (dist={depth:.4f} m)" for name, depth in sorted(in_loop_violations.items())
        )
        reason = f"collision (arm-vs-prop: {named}; threshold={-PROP_COLLISION_DEPTH_TOL_M} m)"
        logger.info(
            "dwell arm=%s target=%s frames_used=%d ok=False reason=%s",
            arm, np.round(np.asarray(hold_pos, dtype=np.float64), 4).tolist(), steps, reason,
        )
        return False, steps, reason, attach_frame

    ok, reason, residual = _validate_against_baseline(env, arm, hold_pos, baseline, target_object=target_object)
    logger.info(
        "dwell arm=%s target=%s frames_used=%d ik_residual=%.4f ok=%s%s attach_frame=%s",
        arm, np.round(np.asarray(hold_pos, dtype=np.float64), 4).tolist(), steps, residual, ok,
        "" if ok else f" reason={reason}", attach_frame,
    )
    return ok, steps, reason, attach_frame


# ---------------------------------------------------------------------------
# Skills
# ---------------------------------------------------------------------------


#: M06 Phase 2 Commit 2 (ADR-030). How far above `TABLE_SURFACE_Z` the
#: object's final z must sit for `pick` to count as a physical lift, once a
#: weld is wired in. Deliberately a SEPARATE constant from the older,
#: initial-z-relative `PICK_LIFT_MARGIN_M`: this commit's task instructions
#: specify the pick success bar as an ABSOLUTE height above the table
#: surface ("object z > table_surface + 0.02"), not a margin above whatever
#: height this particular prop happened to rest at. The two are close in
#: practice (every prop rests within about 4 mm of the table surface) but
#: are not the same formula, so both constants are kept, each doing the job
#: it was specified for. See `m06-ik-lift-diagnostic.md`/
#: `m06-weld-verification.md` (referenced in DECISIONS.md's ADR-029/Phase 2
#: entries) for why even this 0.02 m bar is expected to be hard to clear
#: under the incremental IK regime `_drive_to_target`/`_dwell` both use.
WELD_PICK_SUCCESS_MARGIN_M = 0.02


def run_pick(
    env,
    arm: str,
    target_object: str,
    step_budget: int = ik.DEFAULT_STEP_BUDGET,
    weld: WeldGrasp | None = None,
    hold_ctrl_base: np.ndarray | None = None,
) -> SkillResult:
    """pick(object, arm): APPROACH (clearance above the grasp point) ->
    DESCEND (onto it) -> GRIP (close + attempt weld attach) -> RETREAT (back
    to clearance height) (ADR-027, tutor note 06's approach/grip/retreat
    pattern; weld attach is M06 Phase 2 Commit 2, ADR-030).

    Targets the object body named in `OBJECT_BODY_NAME`, offset by
    `GRASP_POINT_OFFSET_M` to a small graspable feature (see module
    docstring).

    `weld` (ADR-030): if given, GRIP repeatedly calls
    `weld.attempt_grasp(arm, body_name)` (every step, after commanding jaw
    closure) until it attaches or `GRIP_HOLD_FRAMES` is exhausted. Failing
    to attach is reported as its own distinguishable reason,
    `weld_attach_failed_after_N_frames`, rather than folded into a generic
    "grip failed" -- a Tester needs to tell "the weld mechanism never
    engaged" apart from "an arm-motion/collision failure happened first".
    `weld=None` (the default, e.g. a caller with no `WeldGrasp` instance)
    reproduces this module's pre-Commit-2 behavior: GRIP is a plain
    close-and-dwell with no attach attempt, and `success` falls back to a
    lift-only check.

    Success (with `weld` given): the object's world z ends above
    `TABLE_SURFACE_Z + WELD_PICK_SUCCESS_MARGIN_M` AND
    `weld.is_holding(arm) == body_name` -- i.e. it is both physically lifted
    and still confirmed held, not merely "was welded at some earlier frame".
    Success (with `weld=None`): the older initial-z-relative
    `PICK_LIFT_MARGIN_M` check, unchanged from before this commit.

    `hold_ctrl_base` (ADR-037): threaded to every waypoint/dwell this
    function runs, for a caller doing a genuine long-duration idle hold of
    the OTHER arm across this ENTIRE `run_pick` call (`run_handoff`'s
    Phase 1, where `to_arm` must stay frozen at HOME for the whole nested
    pick, not just one waypoint of it). `None` (the default -- every
    standalone `pick(...)` call, e.g. `tests/test_skills.py`) reproduces
    this function's pre-ADR-037 behaviour exactly: the OTHER arm is still
    held (ADR-010), just via `_hold_ctrl`'s original per-step qpos-chasing
    default rather than a caller-supplied frozen snapshot.
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

    # Waypoint 1: APPROACH -- hover at clearance height above the grasp
    # point OR above the object's own physical top, whichever is higher
    # (ADR-033). For every prop except the bottle, `OBJECT_TOP_LOCAL_Z_M`'s
    # `.get(..., offset[2])` fallback makes this identical to the old
    # `grasp_point + CLEARANCE_HEIGHT_M` formula -- see that constant's
    # docstring for the measured reason the bottle alone needs the taller
    # of the two.
    top_local_z = OBJECT_TOP_LOCAL_Z_M.get(target_object, offset[2])
    hover_z = max(float(grasp_point[2]), float(obj_pos0[2] + top_local_z)) + CLEARANCE_HEIGHT_M
    hover = np.array([grasp_point[0], grasp_point[1], hover_z])
    ok, used, reason = _run_waypoint(
        env, arm, hover, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline, target_object=target_object,
        hold_ctrl_base=hold_ctrl_base,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 1 (approach) failed [{reason}]", frames)

    # Waypoint 2: DESCEND -- onto the grasp point itself, gripper still open.
    ok, used, reason = _run_waypoint(
        env, arm, grasp_point, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline, target_object=target_object,
        hold_ctrl_base=hold_ctrl_base,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"waypoint 2 (descend) failed [{reason}]", frames)

    # Waypoint 3: GRIP -- close the jaw and, if a WeldGrasp was supplied,
    # attempt attach every step until it engages or GRIP_HOLD_FRAMES runs
    # out (ADR-030). `grip_start_frames` lets the local dwell-index
    # `_run_dwell` returns be turned into a whole-skill-call frame number,
    # matching `SkillResult.weld_attach_frame`'s documented meaning.
    grip_start_frames = frames
    ok, used, reason, attach_frame_local = _run_dwell(
        env, arm, grasp_point, close_frac, min(GRIP_HOLD_FRAMES, remaining), baseline,
        target_object=target_object, weld=weld, weld_object_name=body_name if weld is not None else None,
        hold_ctrl_base=hold_ctrl_base,
    )
    frames += used
    remaining -= used
    attach_frame = grip_start_frames + attach_frame_local if attach_frame_local is not None else None
    if weld is not None and attach_frame is None:
        # Distinguishable from a plain waypoint/collision failure per the
        # task instructions -- the weld mechanism itself never engaged.
        return SkillResult(
            False, f"weld_attach_failed_after_{used}_frames", frames, weld_attach_frame=None, weld_active_at_end=False
        )
    if not ok:
        return SkillResult(
            False, f"waypoint 3 (grip) failed [{reason}]", frames,
            weld_attach_frame=attach_frame, weld_active_at_end=False,
        )

    # Waypoint 4: RETREAT -- back to clearance height, gripper held closed.
    # This also doubles as the physical lift the success check below reads.
    # target_object=target_object (M06a Fix B): the arm is LIFTING the
    # object it just grasped here -- continuing contact with it is the
    # proof of a successful grasp, not a defect, so it stays exempt (up to
    # CRUSH_THRESHOLD_M) through RETREAT too.
    ok, used, reason = _run_waypoint(
        env, arm, hover, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline, target_object=target_object,
        hold_ctrl_base=hold_ctrl_base,
    )
    frames += used
    remaining -= used

    final_z = float(env.data.xpos[body_id][2])
    weld_holding = weld.is_holding(arm) == body_name if weld is not None else False

    if weld is not None:
        lifted = final_z > TABLE_SURFACE_Z + WELD_PICK_SUCCESS_MARGIN_M
        measured = (
            f"{'lifted' if lifted else 'did not lift'} {target_object}: initial_z={initial_z:.4f} "
            f"final_z={final_z:.4f} table_surface_z={TABLE_SURFACE_Z} "
            f"success_threshold_z={TABLE_SURFACE_Z + WELD_PICK_SUCCESS_MARGIN_M:.4f} "
            f"weld_attach_frame={attach_frame} weld_active_at_end={weld_holding}"
        )
        success = lifted and weld_holding
    else:
        lifted = final_z >= initial_z + PICK_LIFT_MARGIN_M
        measured = (
            f"{'lifted' if lifted else 'did not lift'} {target_object}: "
            f"initial_z={initial_z:.4f} final_z={final_z:.4f} margin_required={PICK_LIFT_MARGIN_M}"
        )
        success = lifted

    if not ok:
        return SkillResult(
            False, f"waypoint 4 (retreat) failed [{reason}]; {measured}", frames,
            weld_attach_frame=attach_frame, weld_active_at_end=weld_holding,
        )
    return SkillResult(success, measured, frames, weld_attach_frame=attach_frame, weld_active_at_end=weld_holding)


def run_place(
    env,
    arm: str,
    target_object: str,
    destination: str = "table",
    step_budget: int = ik.DEFAULT_STEP_BUDGET,
    weld: WeldGrasp | None = None,
) -> SkillResult:
    """place(object, target=table, arm): pick the object up (if not already
    held), then APPROACH above the destination at clearance height ->
    DESCEND to destination + a small vertical offset for gentle release ->
    RELEASE (weld.release(arm) BEFORE opening the jaw, then open + hold) ->
    RETREAT to clearance height (ADR-027; the weld-release-before-jaws-open
    ordering is M06 Phase 2 Commit 2, ADR-030 -- releasing first means the
    object is not kicked by the opening jaw's own collision geometry while
    still welded rigidly to it).

    `weld` (ADR-030) is threaded straight into the nested `run_pick` call
    below, so `place`'s own internal grasp attaches exactly like a
    standalone `pick` would. `weld=None` (the default) reproduces this
    module's pre-Commit-2 behavior throughout.

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
    # ASSUMING a prior `pick` already ran, so it is independently testable
    # (tests/test_skills.py calls `place` directly at seed=0, object still on
    # the table). The nested `pick`'s own waypoint numbering/reason string is
    # propagated as-is on failure, so a failure inside the grasp still
    # localises to its exact stage.
    #
    # **Verification fix, Sept 13 2026 (M06 place-with-bottle verification
    # probe): only call the nested `pick` if `arm` is NOT already holding
    # `body_name`.** The docstring above already promised "pick the object
    # up (if not already held)" -- this branch was the only place that
    # promise was not actually implemented; previously this call ran
    # UNCONDITIONALLY even when `weld.is_holding(arm) == body_name` already.
    # Measured failure this fix addresses (`scripts/probe_place_bottle.py`,
    # bm-ptl, seed 0): running `pick(A, water_bottle)` then
    # `place(A, water_bottle, table)` in the same episode -- i.e. exactly
    # the sequence a `TaskPlan` produces, and exactly what this docstring's
    # parenthetical anticipates -- had the nested `run_pick` recompute a
    # fresh `hover` from the bottle's CURRENT (already-lifted, final
    # pick z=0.6192) position using ADR-033's tall-prop top offset
    # (`OBJECT_TOP_LOCAL_Z_M["bottle"]=0.11`), driving APPROACH toward
    # roughly 0.17 m higher than where the arm already was holding the
    # bottle -- a target the arm could not kinematically reach (IK
    # residual=0.1639 m against `ik.IK_POSITION_TOLERANCE_M`=0.01 m after
    # the full 500-step waypoint budget), so `place` failed at "waypoint 1
    # (approach)" before ever reaching its OWN destination waypoints, and
    # the weld was never released (`is_holding('A')` stayed `'water_bottle'`
    # at the end). This is NOT the ADR-033 failure mode (a hover point
    # sitting below a STATIONARY object's own top) and ADR-033's
    # `OBJECT_TOP_LOCAL_Z_M` pattern does not apply here -- the bug is that
    # `place` re-picks an object it is already holding at all, not that its
    # hover height formula is wrong once it does. The fix is therefore a
    # caller-side branch, not a new per-prop constant: skip the nested pick
    # entirely when the weld already confirms this arm holds this object,
    # go straight to the destination waypoints below with the object
    # already in hand. When `weld is None` (no `WeldGrasp` supplied) or the
    # object is not already held, this reproduces the exact prior
    # behaviour -- the nested `run_pick` call is unchanged in that case.
    already_held = weld is not None and weld.is_holding(arm) == body_name
    if already_held:
        pick_result = None
    else:
        pick_budget = max(1, step_budget // 2)
        pick_result = run_pick(env, arm, target_object, step_budget=pick_budget, weld=weld)
        frames += pick_result.frames_used
        if not pick_result.success:
            return SkillResult(
                False, f"place aborted: pick failed ({pick_result.reason})", frames,
                weld_attach_frame=pick_result.weld_attach_frame, weld_active_at_end=pick_result.weld_active_at_end,
            )

    # `pick_result` is `None` when the object was already held (skipped the
    # nested pick above) -- in that case no NEW attach happened during THIS
    # `place` call, so `weld_attach_frame` reports `None` per its own
    # docstring ("the instant `attempt_grasp` first returned True for THIS
    # skill's own grasp"), not the earlier call's frame count.
    pick_attach_frame = pick_result.weld_attach_frame if pick_result is not None else None

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

    # Waypoint 3: RELEASE. ADR-030: release the weld BEFORE commanding the
    # jaw open, so the object is set free of the rigid attach first and is
    # not kicked by the opening jaw's own moving collision geometry -- then
    # open + hold so the object settles under gravity/contact as before.
    if weld is not None:
        weld.release(arm)
    ok, used, reason, _ = _run_dwell(
        env, arm, lower_target, open_frac, min(GRIP_HOLD_FRAMES, remaining), baseline, target_object=target_object
    )
    frames += used
    remaining -= used
    weld_holding = weld.is_holding(arm) == body_name if weld is not None else False
    if not ok:
        return SkillResult(
            False, f"waypoint 3 (release) failed [{reason}]", frames,
            weld_attach_frame=pick_attach_frame, weld_active_at_end=weld_holding,
        )

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
        f"z={final_z:.4f} (table surface z={TABLE_SURFACE_Z}) weld_active_at_end={weld_holding}"
    )
    if not ok:
        return SkillResult(
            False, f"waypoint 4 (retreat) failed [{reason}]; {measured}", frames,
            weld_attach_frame=pick_attach_frame, weld_active_at_end=weld_holding,
        )
    if on_table_height and on_table_xy:
        return SkillResult(
            True, measured, frames,
            weld_attach_frame=pick_attach_frame, weld_active_at_end=weld_holding,
        )
    return SkillResult(
        False, f"object not resting on the table after place; {measured}", frames,
        weld_attach_frame=pick_attach_frame, weld_active_at_end=weld_holding,
    )


def _joint_limit_margin_now(env, arm: str) -> float:
    """ADR-035. Joint-limit margin (radians) of `arm`'s 5 positioning
    joints at their CURRENT, PHYSICALLY-REALIZED qpos (not a fresh
    scratch IK solve) -- this checks the config the interpolation loop
    actually left the real, simulated arm in, since a static IK chain
    (what the diagnostic checked) is not the same thing as an executed
    trajectory (the diagnostic's own explicitly-left-open caveat)."""
    margin = float("inf")
    for name in ik.arm_joint_names(arm):
        jid = _joint_id(env.model, name)
        qadr = int(env.model.jnt_qposadr[jid])
        angle = float(env.data.qpos[qadr])
        lo, hi = env.model.jnt_range[jid]
        margin = min(margin, angle - float(lo), float(hi) - angle)
    return margin


def _run_interpolated_waypoint(
    env,
    arm: str,
    start_pos,
    end_pos,
    gripper_fraction: float,
    remaining_budget: int,
    baseline: dict,
    target_object: str | None = None,
    step_size: float = HANDOFF_INTERP_STEP_M,
    hold_ctrl_base: np.ndarray | None = None,
) -> tuple[bool, int, str | None]:
    """ADR-035: drive `arm` from `start_pos` to `end_pos` in `step_size`
    increments, re-converging (IK residual + collision, via the existing
    `_run_waypoint`) at EACH intermediate target before advancing to the
    next, instead of handing the solver the full-distance target in one
    shot.

    **`start_pos` is captured ONCE by the caller, before this function is
    entered, and never re-read from live state inside this loop.** This is
    the Zeno-bug fix the corrected brief called out: if the "current
    position" used to build each intermediate target were re-read from the
    sim every iteration, the arm would cover only a shrinking fraction of
    the remaining distance each step and asymptotically approach `end_pos`
    without ever arriving. Fixing `start_pos` once and computing
    `start_pos + (end_pos - start_pos) * (i / n_steps)` for a fixed `i`
    guarantees step `n_steps` lands exactly on `end_pos`.

    Per-step budget is `APPROACH_DESCENT_STEPS` (unchanged) at every
    intermediate hop -- the TOTAL budget this call can spend scales with
    `n_steps` (up to `n_steps * APPROACH_DESCENT_STEPS`, capped only by
    `remaining_budget`), so adding waypoints does not starve the traverse
    by dividing one fixed budget across more of them.

    **Joint-limit margin is checked after every step converges**
    (`HANDOFF_JOINT_LIMIT_MARGIN_TOL`), closing the diagnostic's own
    explicitly-left-open caveat that its chain only ever checked IK
    residual. A step that converges kinematically but leaves a joint
    pegged at (or past) its `jnt_range` bound still fails this function,
    with the specific step and joint-margin number reported.

    Returns `(ok, frames_used, reason)`, `reason` naming the specific
    intermediate step (and its target) that failed, so a caller-side
    failure localises exactly like every other waypoint in this module.

    `hold_ctrl_base` (ADR-037): threaded to every `_run_waypoint` call this
    function makes. `None` (the default) is unchanged pre-ADR-037 behaviour.
    """
    start = np.asarray(start_pos, dtype=np.float64).reshape(3)
    end = np.asarray(end_pos, dtype=np.float64).reshape(3)
    distance = float(np.linalg.norm(end - start))
    n_steps = max(1, int(np.ceil(distance / step_size)))

    frames = 0
    remaining = remaining_budget
    for i in range(1, n_steps + 1):
        target_i = start + (end - start) * (i / n_steps)
        max_steps = min(APPROACH_DESCENT_STEPS, remaining)
        if max_steps <= 0:
            return False, frames, f"interpolation step {i}/{n_steps} failed [no budget remaining]"
        ok, used, reason = _run_waypoint(
            env, arm, target_i, gripper_fraction, max_steps, baseline, target_object=target_object,
            hold_ctrl_base=hold_ctrl_base,
        )
        frames += used
        remaining -= used
        if not ok:
            return (
                False, frames,
                f"interpolation step {i}/{n_steps} (target={np.round(target_i, 4).tolist()}) failed [{reason}]",
            )
        margin = _joint_limit_margin_now(env, arm)
        if margin <= HANDOFF_JOINT_LIMIT_MARGIN_TOL:
            return (
                False, frames,
                f"interpolation step {i}/{n_steps} (target={np.round(target_i, 4).tolist()}) failed "
                f"[joint_limit_margin={margin:.6f} rad <= {HANDOFF_JOINT_LIMIT_MARGIN_TOL} rad]",
            )
    return True, frames, None


def _run_approach_with_staging(
    env,
    arm: str,
    hover_target,
    gripper_fraction: float,
    remaining_budget: int,
    baseline: dict,
    target_object: str | None = None,
    hold_ctrl_base: np.ndarray | None = None,
) -> tuple[bool, int, str | None]:
    """ADR-035 correction 2: an arm's very first move of a `handoff` call
    (this arm has not been driven anywhere yet this call, so its live
    qpos is still whatever it started the call at -- HOME, in the ordinary
    case) can fail to converge in ONE SHOT if HOME sits on the wrong side
    of the disjoint home-seeded reachable band this task's diagnostic
    measured (`HANDOFF_STAGING_Y_M`'s docstring) -- and interpolating a
    target that starts inside that bad basin cannot help, per the
    corrected brief: "the traverse must START from a converged pose."

    Tries the direct shot first (cheap, and the common case for whichever
    arm's HOME already happens to sit on the correct side). Only if that
    fails does it back up to `arm`'s own known-converged staging cell
    (same y, x and z as `hover_target` except for `y`, which is replaced
    by `HANDOFF_STAGING_Y_M[arm]`), confirm THAT converges, and then
    interpolate onward from there to the real `hover_target` via
    `_run_interpolated_waypoint` -- i.e. staging is a real, physically
    driven waypoint of its own, not merely a re-seeded solve.

    Returns `(ok, frames_used, reason)`.

    `hold_ctrl_base` (ADR-037): threaded to every waypoint/interpolation
    call this function makes. `None` (the default) is unchanged pre-ADR-037
    behaviour.
    """
    hover = np.asarray(hover_target, dtype=np.float64).reshape(3)
    max_steps = min(APPROACH_DESCENT_STEPS, remaining_budget)
    ok, used, reason = _run_waypoint(
        env, arm, hover, gripper_fraction, max_steps, baseline, target_object=target_object,
        hold_ctrl_base=hold_ctrl_base,
    )
    if ok:
        return True, used, None

    frames = used
    remaining = remaining_budget - used
    staging_y = HANDOFF_STAGING_Y_M.get(arm)
    if staging_y is None or remaining <= 0:
        return False, frames, f"direct approach failed [{reason}] and no staging cell available for arm {arm!r}"

    staging_target = np.array([hover[0], staging_y, hover[2]])
    max_steps = min(APPROACH_DESCENT_STEPS, remaining)
    ok2, used2, reason2 = _run_waypoint(
        env, arm, staging_target, gripper_fraction, max_steps, baseline, target_object=target_object,
        hold_ctrl_base=hold_ctrl_base,
    )
    frames += used2
    remaining -= used2
    if not ok2:
        return (
            False, frames,
            f"direct approach failed [{reason}]; staging to y={staging_y} also failed [{reason2}]",
        )

    ok3, used3, reason3 = _run_interpolated_waypoint(
        env, arm, staging_target, hover, gripper_fraction, remaining, baseline, target_object=target_object,
        hold_ctrl_base=hold_ctrl_base,
    )
    frames += used3
    if not ok3:
        return (
            False, frames,
            f"direct approach failed [{reason}]; staged at y={staging_y}, then interpolation to hover failed [{reason3}]",
        )
    return True, frames, None


def run_handoff(
    env,
    to_arm: str,
    from_arm: str,
    target_object: str,
    step_budget: int = ik.DEFAULT_STEP_BUDGET,
    weld: WeldGrasp | None = None,
) -> SkillResult:
    """handoff(object, from_arm, to_arm): ADR-037's SEQUENTIAL CHOREOGRAPHY
    -- one arm moves at a time, the other is genuinely frozen (not merely
    "not explicitly driven"), replacing the earlier same-point-targeting
    design (ADR-010/027/030) that this ADR's own investigation found still
    fails on a cross-arm collision (`f92806e`) even though nothing in the
    OLD code ever moved both arms in the SAME physics step either -- the
    problem `f92806e` found was never simultaneity, it was that `from_arm`
    sits STATIC in a spot `to_arm`'s own approach sweeps through, and
    sequencing alone does not move `from_arm` out of the way (see Phase 3's
    own docstring paragraph below, and DECISIONS.md's ADR-037 entry for the
    full measurement trail, including the staging-geometry alternatives
    that were tried and NOT adopted).

    **Phase 1** -- `from_arm` picks the object up (nested `run_pick`, `weld`
    threaded through so `from_arm`'s own initial grasp attaches exactly like
    a standalone `pick`). `to_arm` is held stationary at HOME for this
    entire phase via a FROZEN ctrl snapshot (`to_arm_hold`, ADR-037's
    `_hold_ctrl(frozen_base=...)` fix -- see that function's docstring for
    the measured drift this replaces: 0.04 rad by 100 steps, ~0.26 rad
    implied over a nested pick's own ~655-frame trace, under the OLD
    per-step qpos-chasing hold).

    **Phase 2** -- `from_arm` APPROACHes (and, since ADR-036, directly
    arrives at -- no separate DESCEND) `HANDOFF_POSITION_XYZ`
    (`transfer_point`), staged via `_run_approach_with_staging` if it does
    not converge in one shot (ADR-035). `to_arm` is STILL held at the SAME
    `to_arm_hold` snapshot from Phase 1 -- it has not moved, so the
    snapshot taken before Phase 1 is still exactly correct; no arm is EVER
    idle-held via a stale or re-derived-from-a-different-instant vector.

    **Phase 3** -- `from_arm` is now held stationary at `transfer_point`,
    STILL HOLDING THE OBJECT, via a NEW frozen snapshot (`from_arm_hold`,
    captured the instant Phase 2 finishes) -- while `to_arm` APPROACHes the
    receiving point (`HANDOFF_SIDE_OFFSET_M` off `transfer_point`, ADR-024),
    staged the same way Phase 2 was.

    **This is the phase `f92806e` found collides, and sequential
    choreography by itself does not fix it.** `from_arm` is genuinely
    STATIC here (Phase 2 already finished; it is not "not yet driven", it
    has arrived and stopped) -- yet its own body occupies exactly the
    region `to_arm`'s home-seeded staging cell (`HANDOFF_STAGING_Y_M["B"]
    = -0.06`) must cross through to reach the receiving point on the
    OPPOSITE side (`+HANDOFF_SIDE_OFFSET_M`). This ADR's own measurement
    (DECISIONS.md) tried two routing alternatives instead of the plain
    lateral crossing this phase still uses:
      - **Elevated ("dodge") crossing**: stage `to_arm` at a taller z,
        sweep laterally clear of `from_arm`'s operating height, then
        descend. Measured: the LATERAL sweep at z=0.53 converges
        collision-free in some runs, but is reproducibly ON A JOINT-LIMIT
        KNIFE-EDGE (margin as small as -0.000001 rad -- i.e. flips pass/fail
        on essentially no perturbation) at every elevated height tried
        (0.48-0.70), and the DESCEND back down to the real receiving height
        reliably hits a genuine joint-limit wall around z~0.49-0.50
        regardless of the height dodged to. Rejected as NOT robust enough
        to ship in place of the one corridor that IS kinematically solid.
      - **Horizontal ("dodge-in-x") crossing**: sweep `to_arm` out to
        x=+-0.15/+-0.20 before crossing y, then back. Measured:
        inconclusive -- ran out of step budget mid-interpolation in every
        variant tried (500-1000 steps per hop, well past ordinary
        convergence time), suggesting this corridor is not meaningfully
        easier, not that it is fine given more budget.
    Neither alternative is adopted. This phase therefore uses the SAME
    `HANDOFF_STAGING_Y_M`/`_run_approach_with_staging` lateral corridor
    ADR-035 already verified kinematically -- the one corridor proven to
    converge -- and is EXPECTED, per this measurement, to still fail here
    with a cross-arm collision. That is reported as this ADR's actual,
    honest verification result (see DECISIONS.md), not patched over by
    routing `to_arm` through a shakier alternative.

    **Phase 4** -- `to_arm` GRIPs (closes the jaw, attempts weld attach
    every step until it engages or `GRIP_HOLD_FRAMES` runs out -- same
    mechanism as `run_pick`'s GRIP, ADR-030), `from_arm` still frozen at
    `from_arm_hold`. **4a**: `weld.is_holding(to_arm) == body_name` is
    checked EXPLICITLY here, BEFORE `from_arm` is ever asked to release
    (ADR-030's transfer-safety gate, unchanged). **This task's Part 3
    concern -- two simultaneous welds on one body (`from_arm`-vs-fork AND
    `to_arm`-vs-fork both active at once, a closed kinematic chain through
    the fork) -- is tested explicitly**, not assumed safe: see
    DECISIONS.md's ADR-037 entry for the measured constraint forces/qpos
    delta during the exact frame window both welds are simultaneously
    active. **4b**: `weld.release(from_arm)` is called BEFORE the jaw is
    commanded open (same ordering rationale as `place`'s RELEASE), only
    once 4a has confirmed the transfer. `to_arm` is then frozen at a NEW
    snapshot (`to_arm_grip_hold`, captured the instant it finishes
    gripping) while `from_arm` runs its own RELEASE dwell (open the jaw,
    ADR-031's per-dwell arm freeze governs `from_arm`'s OWN ctrl during
    this; `to_arm_grip_hold` governs the now-idle `to_arm`'s).

    **Phase 5** -- `from_arm` retreats FIRST (to a taller clearance,
    `HANDOFF_FROM_ARM_RETREAT_CLEARANCE_M`, chosen so the total Euclidean
    distance from its gripperframe to `transfer_point` clears
    `HANDOFF_RETREAT_GATE_M` -- a plain `CLEARANCE_HEIGHT_M` vertical lift
    is only 0.08 m of total displacement, short of the required > 0.10 m),
    while `to_arm` is held at `to_arm_grip_hold`. Only once that gate is
    measured (not assumed) to be cleared does `to_arm` retreat, held frozen
    by a FINAL snapshot of `from_arm`'s now-retreated pose
    (`from_arm_retreated_hold`).

    `weld=None` (the default) reproduces this module's pre-ADR-037
    attach/release/gate behavior throughout (no attach attempts, no
    transfer gate, no release calls) -- the older lift-margin/distance-only
    success check below is unchanged in that case.

    Success (with `weld` given): the object ends measurably closer to
    `to_arm`'s gripperframe site than `from_arm`'s, has been lifted since
    the handoff began, `weld.is_holding(to_arm) == body_name` while
    `weld.is_holding(from_arm)` is not, AND `from_arm` is clear of the
    shared workspace at the end (`HANDOFF_RETREAT_GATE_M`, checked in
    Phase 5). Success (with `weld=None`): the older distance+lift-only
    check.
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

    close_frac, open_frac = GRIPPER_CLOSE_FRACTION, GRIPPER_OPEN_FRACTION
    hx, hy, hz = HANDOFF_POSITION_XYZ
    transfer_point = np.array([hx, hy, hz])
    side = -1.0 if to_arm == "A" else 1.0  # arm A base at y=-0.25, arm B at y=+0.25
    receiving_point = transfer_point + np.array([0.0, side * HANDOFF_SIDE_OFFSET_M, 0.0])

    # === Phase 1: from_arm picks the object; to_arm held stationary at
    # HOME for the WHOLE phase (ADR-037 frozen snapshot, taken before
    # anything in this call has moved). ===
    to_arm_hold = _hold_ctrl(env)
    pick_budget = max(1, step_budget // 2)
    pick_result = run_pick(
        env, from_arm, target_object, step_budget=pick_budget, weld=weld, hold_ctrl_base=to_arm_hold,
    )
    frames += pick_result.frames_used
    if not pick_result.success:
        return SkillResult(
            False, f"phase 1 (from_arm pick) failed ({pick_result.reason})", frames,
            weld_attach_frame=pick_result.weld_attach_frame, weld_active_at_end=False,
        )

    remaining = step_budget - frames
    if remaining <= 0:
        return SkillResult(False, "phase 2 (from_arm approach) failed [convergence (no budget remaining after pick)]", frames)

    # === Phase 2: from_arm approaches/arrives at the transfer point
    # (ADR-036: no separate DESCEND, transfer_point already sits at hover
    # height). to_arm is STILL held at the SAME to_arm_hold snapshot -- it
    # has not moved since Phase 1, so nothing needs re-snapshotting. ===
    ok, used, reason = _run_approach_with_staging(
        env, from_arm, transfer_point, close_frac, remaining, baseline, target_object=target_object,
        hold_ctrl_base=to_arm_hold,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"phase 2 (from_arm approach) failed [{reason}]", frames)

    # === Phase 3: from_arm now held stationary AT the transfer point,
    # STILL HOLDING THE OBJECT (a NEW frozen snapshot, taken the instant it
    # arrives) -- while to_arm approaches the receiving point. See this
    # function's own docstring, Phase 3, for the measured routing
    # alternatives that were tried and NOT adopted here. ===
    from_arm_hold = _hold_ctrl(env)
    ok, used, reason = _run_approach_with_staging(
        env, to_arm, receiving_point, open_frac, remaining, baseline, target_object=target_object,
        hold_ctrl_base=from_arm_hold,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(False, f"phase 3 (to_arm approach) failed [{reason}]", frames)

    # === Phase 4: to_arm GRIPs -- close and, if a WeldGrasp was supplied,
    # attempt attach every step until it engages or GRIP_HOLD_FRAMES runs
    # out (ADR-030, same mechanism as run_pick's GRIP). from_arm remains
    # frozen at from_arm_hold throughout. ===
    grip_start_frames = frames
    ok, used, reason, attach_frame_local = _run_dwell(
        env, to_arm, receiving_point, close_frac, min(GRIP_HOLD_FRAMES, remaining), baseline,
        target_object=target_object, weld=weld, weld_object_name=body_name if weld is not None else None,
        hold_ctrl_base=from_arm_hold,
    )
    frames += used
    remaining -= used
    attach_frame = grip_start_frames + attach_frame_local if attach_frame_local is not None else None
    if weld is not None and attach_frame is None:
        return SkillResult(
            False, f"phase 4 (to_arm grip) weld_attach_failed_after_{used}_frames", frames,
            weld_attach_frame=None, weld_active_at_end=False,
        )
    if not ok:
        return SkillResult(
            False, f"phase 4 (to_arm grip) failed [{reason}]", frames,
            weld_attach_frame=attach_frame, weld_active_at_end=False,
        )

    # Phase 4a (ADR-030's transfer-safety gate): verify to_arm actually
    # holds the object, via the weld state itself, BEFORE from_arm is ever
    # asked to release. If this does not hold, fail immediately and leave
    # from_arm's weld untouched -- the object stays with from_arm rather
    # than risking a state where neither arm holds it.
    if weld is not None and weld.is_holding(to_arm) != body_name:
        return SkillResult(
            False, "phase 4 (handoff_transfer_failed)", frames,
            weld_attach_frame=attach_frame, weld_active_at_end=False,
        )

    # Phase 4b: from_arm RELEASEs -- weld released BEFORE the jaw opens
    # (same ordering rationale as `place`'s RELEASE), only reached once 4a
    # above has confirmed the transfer. to_arm is now the idle one (it has
    # just gripped and must hold still while from_arm opens its own jaw),
    # so a NEW frozen snapshot is taken here, the instant to_arm stops.
    if weld is not None:
        weld.release(from_arm)
    to_arm_grip_hold = _hold_ctrl(env)
    ok, used, reason, _ = _run_dwell(
        env, from_arm, transfer_point, open_frac, min(GRIP_HOLD_FRAMES, remaining), baseline,
        target_object=target_object, hold_ctrl_base=to_arm_grip_hold,
    )
    frames += used
    remaining -= used
    if not ok:
        return SkillResult(
            False, f"phase 4 (from_arm release) failed [{reason}]", frames,
            weld_attach_frame=attach_frame,
            weld_active_at_end=(weld.is_holding(to_arm) == body_name if weld is not None else False),
        )

    # === Phase 5a: from_arm RETREATs FIRST (staggered), to a taller
    # clearance than the usual CLEARANCE_HEIGHT_M so the total Euclidean
    # distance from its gripperframe to transfer_point clears
    # HANDOFF_RETREAT_GATE_M (measured requirement, see this function's
    # docstring). to_arm is held at the SAME to_arm_grip_hold snapshot --
    # it has not moved since Phase 4b. ===
    from_retreat = transfer_point + np.array([0.0, 0.0, HANDOFF_FROM_ARM_RETREAT_CLEARANCE_M])
    ok7, used, reason7 = _run_waypoint(
        env, from_arm, from_retreat, open_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object, hold_ctrl_base=to_arm_grip_hold,
    )
    frames += used
    remaining -= used
    if not ok7:
        return SkillResult(
            False, f"phase 5 (from_arm retreat) failed [{reason7}]", frames,
            weld_attach_frame=attach_frame,
            weld_active_at_end=(weld.is_holding(to_arm) == body_name if weld is not None else False),
        )

    # Phase 5 gate: from_arm's gripper must measurably clear
    # HANDOFF_RETREAT_GATE_M from transfer_point (Euclidean distance, the
    # PHYSICALLY-REALIZED site position -- not assumed from the waypoint
    # target) before to_arm is allowed to retreat.
    site_from = _site_id(env.model, ik.gripperframe_site_name(from_arm))
    from_arm_retreat_dist = float(np.linalg.norm(env.data.site_xpos[site_from] - transfer_point))
    if from_arm_retreat_dist <= HANDOFF_RETREAT_GATE_M:
        return SkillResult(
            False,
            f"phase 5 (from_arm retreat) did not clear the {HANDOFF_RETREAT_GATE_M} m gate before "
            f"to_arm's retreat: from_arm_retreat_dist={from_arm_retreat_dist:.4f} m",
            frames, weld_attach_frame=attach_frame,
            weld_active_at_end=(weld.is_holding(to_arm) == body_name if weld is not None else False),
        )

    # === Phase 5b: to_arm RETREATs SECOND, now that from_arm has
    # measurably cleared the shared workspace. from_arm is held at a FINAL
    # frozen snapshot, taken the instant it finishes its own retreat. ===
    from_arm_retreated_hold = _hold_ctrl(env)
    to_lift = receiving_point + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    # target_object=target_object (M06a Fix B): to_arm is now LIFTING the
    # object it just gripped -- same rationale as pick's RETREAT above.
    ok8, used, reason8 = _run_waypoint(
        env, to_arm, to_lift, close_frac, min(APPROACH_DESCENT_STEPS, remaining), baseline,
        target_object=target_object, hold_ctrl_base=from_arm_retreated_hold,
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

    # ADR-030: fold the weld's own held-object bookkeeping into the success
    # check, not just distance/lift -- "the object ends held by to_arm and
    # NOT by from_arm" is exactly what `is_holding` on both arms answers
    # directly, rather than inferring it from gripperframe proximity alone.
    weld_to_holding = weld.is_holding(to_arm) == body_name if weld is not None else False
    weld_from_holding = weld.is_holding(from_arm) == body_name if weld is not None else False
    # ADR-037: "from_arm clear of the shared workspace at the end" is the
    # SAME measured distance Phase 5's own gate already checked -- from_arm
    # does not move again after Phase 5a (it is held frozen through Phase
    # 5b), so this is still the correct end-of-call value, not stale.
    from_arm_clear = from_arm_retreat_dist > HANDOFF_RETREAT_GATE_M

    measured = (
        f"z={final_z:.4f} (initial {initial_z:.4f}) dist_to_armA={dist_a:.4f} "
        f"dist_to_armB={dist_b:.4f} (to_arm={to_arm}) weld_holding_to_arm={weld_to_holding} "
        f"weld_holding_from_arm={weld_from_holding} from_arm_retreat_dist={from_arm_retreat_dist:.4f} "
        f"from_arm_clear={from_arm_clear}"
    )
    if not ok8:
        return SkillResult(
            False, f"phase 5b (to_arm retreat) failed [{reason8}]; {measured}", frames,
            weld_attach_frame=attach_frame, weld_active_at_end=weld_to_holding,
        )
    if weld is not None:
        success = lifted and closer_to_to_arm and weld_to_holding and not weld_from_holding and from_arm_clear
    else:
        success = lifted and closer_to_to_arm and from_arm_clear
    if success:
        return SkillResult(
            True, f"held by arm {to_arm}: {measured}", frames,
            weld_attach_frame=attach_frame, weld_active_at_end=weld_to_holding,
        )
    return SkillResult(
        False, f"object not confirmed held by arm {to_arm}: {measured}", frames,
        weld_attach_frame=attach_frame, weld_active_at_end=weld_to_holding,
    )


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

    # Waypoint 3: GRIP -- close against the drawer's front face. No `weld`
    # concept here (the drawer is not one of `WeldGrasp`'s `GRASPABLE_OBJECTS`
    # -- it has a slide joint, not a free joint, ADR-029), so this call
    # never passes `weld`/`weld_object_name` and behaves exactly as before.
    ok, used, reason, _ = _run_dwell(env, arm, handle_point, close_frac, min(GRIP_HOLD_FRAMES, remaining), baseline)
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
    ok, used, reason, _ = _run_dwell(env, arm, pulled_point, open_frac, min(GRIP_HOLD_FRAMES, remaining), baseline)
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
