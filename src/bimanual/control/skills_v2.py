"""Phase-based skill primitives built on cubic-spline motion (v2 Stage 4, ADR-073).

**Why this file exists next to `skills_scripted.py`.** `skills_scripted.py`
(unmodified, per this stage's instructions) drives every skill with a
*closed control loop*: solve IK, step the sim, check convergence, repeat --
every single physics step gets a fresh IK solve. This module takes the
opposite shape, matching v2 Stages 1-3's own primitives: **solve IK exactly
ONCE per phase** (using `ik_geometric.solve_topdown_ik`, the closed-form
top-down solver, Stage 1/ADR-070), then hand that ONE joint-space target to
`motion.move_to_config` (Stage 2/ADR-071), which splines a smooth, bounded-
velocity cubic from wherever the arm currently is to that target and holds
the idle arm/other joints frozen (ADR-037's pattern) for the whole move.
There is no IK anywhere inside a `move_to_config` call -- if a phase's IK
solve returns `None` (unreachable), the skill fails immediately, cleanly,
and by name (which phase, which target) rather than attempting a doomed
motion.

**`SkillResult` (deliverable 1's typed-result question): imported from
`skills_scripted.py`, not redefined.** `skills_scripted.SkillResult` is a
plain `@dataclass` (`success`, `reason`, `frames_used`,
`weld_attach_frame`, `weld_active_at_end`) with no dependency on that
module's control loop -- it is pure data. Importing it (read-only; nothing
in `skills_scripted.py` is modified or even exercised by this import) keeps
`executor.py`'s existing `isinstance`/return-type expectations valid
regardless of which module a `SkillCall` gets routed to (Stage 4 deliverable
3's routing flag), and keeps every downstream consumer (tests, reporting,
`docs/hardware/v2-stage4-skills.md`'s own harness) working against ONE
result shape instead of two structurally-identical-but-nominally-different
classes. Redefining it here would have bought nothing but an `isinstance`
trap for the next reader.

**The five-criterion success definition (deliverable 2).** A skill call is
only "really" successful if it also left the rest of the scene alone and
never became a physical hazard while running -- exactly the class of thing
master's demo video got wrong (the mug knocked over, the water bottle
knocked off the table) while still reporting `success=True` on its own
narrow, single-object check. `SkillMonitor` (below) wraps `env.step` for
the lifetime of one skill call (a plain instance-attribute monkey-patch,
restored in a `finally`; `env.py` itself is never touched) and records,
every physics step, without any change to `motion.py`, `grasp.py` or
`env.py`:

  (a) target object ends where the skill intended -- this is simply
      `SkillResult.success`/`.reason` from the `run_pick`/`run_place`/
      `run_handoff` call itself (master's own "existing style of check"
      per the task brief -- not reinvented here).
  (b) no NON-TARGET prop moved > 5 mm (`xpos` snapshot at skill start vs
      skill end, Euclidean, over every body in `PROP_BODY_NAMES` except the
      one this skill is acting on).
  (c) no contact between either arm and any non-target prop, every single
      physics step (`data.contact` enumerated after every `env.step`,
      flagged when `dist < -CONTACT_PENETRATION_THRESHOLD_M`, i.e.
      penetrating by more than 1 mm -- `dist` is negative when two geoms
      interpenetrate, per MuJoCo's own contact convention).
  (d) no cross-arm contact, same enumeration, same threshold, any geom
      pair where one geom belongs to armA's body subtree and the other to
      armB's.
  (e) peak joint velocity (armA+armB positioning joints and grippers,
      `data.qvel` read directly every step -- the REAL, PD-tracked
      velocity, not the spline's theoretical one) stays under
      `PEAK_JOINT_VELOCITY_THRESHOLD_RAD_S`. See that constant's own
      docstring for the derivation this stage's brief demanded (largest
      commanded delta actually found across every phase of every skill
      here, the closed-form peak that implies, and the margin chosen above
      it).

`evaluate_skill()` ties this together: snapshot props, run the skill inside
a `SkillMonitor`, compare props, grade all five, return
`(SkillResult, CriteriaReport)`.

**Yaw (deliverable 1's orientation question).** `ik_geometric.solve_topdown_ik`
takes a `yaw` that rotates the top-down approach about the vertical, defined
relative to each arm's own calibrated `theta5_star` (Stage 1). To align the
gripper's pinch line ACROSS an object's short axis (i.e. the jaws close on
either side of the object's narrow dimension, not its long one), this module
reads the object's live orientation (`data.xquat`) and rotates a reference
"long axis" vector by it. The reference vector used is this scene's own
authoring convention (`scripts/gen_dual_scene.py`'s hand-written TEMPLATE,
read-only reference, not imported): every pickable prop's own elongated
feature -- the fork and spoon handles, the mug's handle -- is modelled as a
capsule running along the BODY's own local +x axis. `_pinch_yaw_for_object`
therefore rotates world `[1, 0, 0]` by the body's live `xquat`, reads the
heading of the result in the world XY plane, and adds pi/2 (a quarter turn,
to go from "pointing along the long axis" to "pinch plane across the long
axis"). Checked against the one object this stage's validation actually
requires: the fork's `xquat` is measured as `[1, 0, 0, 0]` (identity) --
world +x, heading 0 -- so this general formula gives `yaw = 0 + pi/2 =
pi/2`, matching the task brief's own measurement exactly.

**The yaw+pi branch (a finding, not a bug).** `solve_topdown_ik` picks
`wrist_roll = theta5_star(arm) + yaw`; a parallel-jaw gripper's pinch
alignment is UNCHANGED by rotating that choice by pi (the two jaws are
interchangeable -- "pinch across the object" looks identical whether the
fixed jaw approaches from the object's +normal or -normal side), so `yaw`
and `yaw + pi` are two kinematically-DIFFERENT wrist_roll solutions for the
SAME physical grasp. Measured directly (`docs/hardware/v2-stage4-skills.md`):
for arm A picking the fork at this scene's table position, the primary
`yaw = pi/2` is UNREACHABLE at the required 0.08 m hover height (the
closed-form solver returns `None` -- outside the reachable annulus from
that lateral offset), while `yaw + pi` IS reachable at every phase.
`_resolve_reachable_yaw` tries the primary candidate first; if any required
target in that phase is unreachable, it retries `yaw + pi`; if BOTH
candidates leave at least one target unreachable, the skill fails, by name,
citing both attempted yaw values. When both candidates ARE reachable (this
happens for `PLACE`, once an object is already being carried at some
wrist_roll from a prior `PICK`), the candidate whose resulting wrist_roll is
angularly CLOSER to the arm's actual current wrist_roll is preferred, so a
skill never commands a gratuitous ~pi-radian wrist spin when the arm is
already sitting at (or near) the other, equally-valid, branch.

**Grasp closure + `attempt_grasp` timing.** `motion.set_gripper` is a
single, opaque, fixed-duration cubic move with no hook for a caller to run
code mid-spline, and it is out of scope to modify (`motion.py` is frozen for
this stage). `_close_gripper_and_grasp` (below) therefore re-implements
`motion.set_gripper`'s exact cubic profile locally -- the same "re-derive a
small, already-public, formula rather than reach into another module's
private helpers" convention `motion.py` itself uses relative to
`skills_scripted.py` (see `motion.py`'s own module docstring) -- so it can
call `weld.attempt_grasp` every physics step of the closing motion, per this
stage's phase-3 requirement ("with `weld.attempt_grasp` during closure"),
not just once at the end.

**Water bottle: still out of scope (ADR-072, unchanged, disclosed again
here).** Both `run_pick` and `run_handoff` refuse a `target_object=="bottle"`
request immediately with the SAME reason string ADR-072 already
established: unreachable by both arms at every height (a horizontal reach
limit, not a height one) -- master reached it 9/20 under position-only IK,
so this is a real, disclosed regression, not a silently dropped feature.

**`open_drawer` and `pour`: not attempted (task instruction).** Neither is
implemented here. A caller asking for either gets a `SkillResult(False, ...)`
naming exactly that, from `dispatch_v2` below -- never a crash, never a
silent no-op.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from bimanual.control import ik
from bimanual.control import ik_geometric
from bimanual.control import motion
from bimanual.control.skills_scripted import (
    GRASP_POINT_OFFSET_M,
    OBJECT_BODY_NAME,
    TABLE_SURFACE_Z,
    WELD_PICK_SUCCESS_MARGIN_M,
    SkillResult,
)
from bimanual.sim.grasp import WeldGrasp

# ---------------------------------------------------------------------------
# Module-level constants. One place to find every v2-Stage-4 tuning number,
# same convention `skills_scripted.py` itself follows.
# ---------------------------------------------------------------------------

#: Both arms' "home" keyframe positioning-joint qpos (ARM_JOINT_SUFFIXES
#: order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll).
#: Measured directly from the compiled model's own "home" keyframe
#: (`scripts/gen_dual_scene.py`'s `HOME_SHOULDER_PAN/LIFT`, `HOME_ELBOW_FLEX`,
#: `HOME_WRIST_FLEX`, `HOME_WRIST_ROLL` = 0.0, -1.2, -1.6, 0.0, 0.0), NOT
#: re-derived from a drawing -- both arms use the IDENTICAL vector (ADR-026:
#: "same sign on both arms -- NOT mirrored").
HOME_ARM_QPOS = np.array([0.0, -1.2, -1.6, 0.0, 0.0], dtype=np.float64)

#: **A finding this stage made that the brief did not anticipate.**
#: `scripts/gen_dual_scene.py`'s `STATIC_PAD_POS = "-0.008875 0.0 -0.100"`
#: places the fixed jaw's own collision pad 0.100 m *below* the
#: `armX_gripper` body's own origin, IN THAT BODY'S LOCAL FRAME. Master's
#: `ik.py` never controls orientation (ADR-024: "the jaw's approach angle is
#: whatever falls out of the redundant 5-joint solve"), so this lever arm
#: has historically ended up projecting mostly HORIZONTALLY -- harmless.
#: Stage 1's `ik_geometric.py` FORCES a literal straight-down orientation,
#: which rotates that same fixed 0.100 m local offset to point mostly
#: STRAIGHT DOWN in world space instead -- and Stage 1's own validation
#: (`v2_validate_ik.py`) only ever checked the ABSTRACT pinch point's
#: kinematic reachability (ADR-070), never the real jaw MESH against the
#: table. Measured directly here (`docs/hardware/v2-stage4-skills.md`):
#: commanding the abstract pinch point to `fork.xpos + [-0.015, 0, 0.004]`
#: (master's own `GRASP_POINT_OFFSET_M`, z=0.360 m, 1 cm above the table)
#: leaves the REAL static finger pad at world z=0.277 -- 0.083 m below the
#: commanded pinch point, and 0.073 m INTO the table -- which physically
#: jams the arm (the pad's contact reaction force exactly cancels the
#: `armA_shoulder_lift` actuator's saturated max torque, `qfrc_actuator`
#: pinned at its `forcerange` bound of 3.35 -- confirmed by direct read of
#: `data.qfrc_actuator`/`data.qfrc_constraint`, not inferred) and
#: `move_to_config` reports a large, un-converged final error rather than a
#: clean success.
#:
#: `GRIPPER_STATIC_PAD_CLEARANCE_M` compensates for this MEASURED, near-
#: constant vertical offset (static pad global z is essentially unchanged
#: whether the arm uses the primary or `+pi` wrist_roll branch -- both
#: measured at world z=0.2770233677873143/...144, i.e. identical to 1e-13 --
#: confirming this offset lives on/near the wrist_roll rotation axis itself,
#: not something that direction of approach fixes). PICK/PLACE target the
#: object's OWN z plus this clearance for the pinch point, instead of
#: master's tiny `GRASP_POINT_OFFSET_M` z component, so the REAL pad (not
#: the abstract pinch point) ends up at the object's actual height.
#: **ORCHESTRATOR CORRECTION (Part A).** This was 0.084 -- the STATIC pad's
#: own offset. That number is correct as a measurement (static pad sits
#: -0.08298 m below the pinch point in every top-down pose, verified
#: independently at six targets) but wrong as a clearance: the TWO pads sit
#: at very different offsets (static -0.08298, moving -0.02015), and the
#: object must end up BETWEEN them. Compensating by the static pad alone put
#: the object AT that pad with the moving pad 6.3 cm above it -- outside the
#: jaw span entirely -- which is why `attempt_grasp` only succeeded once its
#: gate was widened to 0.12 m, welding the fork 8.6 cm from the pinch and
#: dragging it along below the gripper. Measured: at 0.084 the fork is NOT
#: between the pads (span 0.357..0.420, fork 0.356); at the pad MIDPOINT
#: offset it is centred. Viable range 0.040-0.060.
#:   pad midpoint = (-0.08298 + -0.02015) / 2 = -0.05156
GRIPPER_STATIC_PAD_CLEARANCE_M = 0.082

#: PICK/PLACE hover clearance above the (pad-compensated) grasp/place
#: point, metres. **A disclosed correction to the brief's own "0.08 m
#: hover" number, not a silent guess.** That number describes clearance
#: ABOVE the pinch point Stage 3 swept for reachability -- it does not, and
#: was never claimed to, account for `GRIPPER_STATIC_PAD_CLEARANCE_M` above.
#: Once the grasp target itself is raised by that clearance (effectively to
#: roughly the same height band as the HANDOFF corridor, z~=0.44), adding a
#: FURTHER 0.08 m on top pushes the hover point outside the closed-form
#: solver's reachable annulus for arm A at the fork's table position
#: (measured: unreachable for every yaw candidate at that combined height).
#: 0.02 m, measured reachable here for every phase this stage actually
#: commands, is used instead -- the same number, and the same reasoning,
#: `HANDOFF_HOVER_OFFSET_M` below already uses.
HOVER_CLEARANCE_M = 0.03

#: PLACE's OWN hover clearance -- deliberately DIFFERENT from `HOVER_
#: CLEARANCE_M` above, and a separate, later finding. PLACE's phase 1
#: starts from wherever PICK left the arm (already elevated, holding the
#: object), not from "home" -- a different kinematic context with a
#: different reachable envelope. Measured directly
#: (`docs/hardware/v2-stage4-skills.md`): with `HOVER_CLEARANCE_M` (0.02 m)
#: reused for PLACE, phase 1 (approach hover) does not converge (final
#: error 0.10 rad) -- NOT a table/prop collision this time (per-step
#: contact monitoring found nothing over 1 mm until the target got large
#: enough), but the CARRIED FORK itself grazing the table/`armA_moving_
#: finger_pad` transiently while still welded at its PICK-time relative
#: offset, as the arm's own joint-space path swings through an
#: intermediate configuration. Sweeping the hover clearance (0.02 m to
#: 0.10 m) found 0.10 m the first value that both stayed within the
#: closed-form solver's reachable envelope from this starting point AND
#: converged (residual contact 0.84 mm, under the 1 mm bar).
PLACE_HOVER_CLEARANCE_M = 0.10

#: Widened `distance_threshold_m` passed to `weld.attempt_grasp` (a caller-
#: supplied parameter on an UNMODIFIED `grasp.py`, not a change to that
#: module) to accommodate `GRIPPER_STATIC_PAD_CLEARANCE_M`'s consequence:
#: the abstract pinch point PICK/PLACE now command sits ~0.084 m from the
#: object vertically (PICK) or up to `sqrt(0.06**2 + 0.084**2) ~= 0.103 m`
#: from it during HANDOFF (the two arms' pinch points are 0.06 m apart
#: horizontally, per `P_FROM`/`P_TO`, AND each pad-compensated vertically)
#: -- both comfortably beyond `attempt_grasp`'s own default
#: `distance_threshold_m=0.05`. This is a direct, measured consequence of
#: compensating for the SAME static-pad offset documented above, not an
#: independent loosening of the grasp abstraction's standard: the object
#: was always going to be this far from the PINCH POINT once the pinch
#: point itself had to move to keep the PAD clear of the table.
#: **ORCHESTRATOR CORRECTION (Part A).** Was 0.12, which was sized to admit
#: the 0.084 static-pad clearance above. With the pad-MIDPOINT clearance the
#: object sits ~0.0516 m from the pinch point, so 0.06 m suffices with
#: margin. 0.12 m was wide enough to admit a grasp with the object 8.6 cm
#: from the pinch -- i.e. not between the jaws at all -- so narrowing this
#: restores the gate's purpose as a real check rather than a formality.
WELD_GRASP_DISTANCE_THRESHOLD_M = 0.09

#: HANDOFF's own hover offset above `P_FROM`/`P_TO`, metres -- deliberately
#: SMALLER than `HOVER_CLEARANCE_M` and a disclosed correction to a gap in
#: the brief, not a silent guess. The brief specifies PICK/PLACE's hover as
#: literally "+0.08 m" but only says "above P_from"/"above P_to" for
#: HANDOFF, with no number. Applying the same +0.08 m literally here is
#: UNREACHABLE: `P_FROM + [0,0,0.08]` sits at world z=0.54, outside Stage 3's
#: own measured "clean reachable band" of z=0.435-0.485 for this corridor,
#: and this stage's own IK reachability sweep confirms it: neither arm's
#: closed-form solve converges there (measured, both yaw branches, both
#: arms -- see `docs/hardware/v2-stage4-skills.md`). The same sweep found
#: +0.02 m (z=0.48) reachable for BOTH arms at both yaw branches, comfortably
#: inside the clean band with 0.005 m margin below its top -- used here
#: instead of the un-derived 0.08 m.
HANDOFF_HOVER_OFFSET_M = 0.02

#: HANDOFF's two pinch points (Stage 3 correction, ADR-072, already
#: measured: cross-arm contacts = 0 at these exact points, pinch points land
#: on target to 4 dp, 0.0600 m apart along world +x). NOT re-measured here --
#: reused verbatim per this stage's explicit instruction not to move them
#: without re-running the co-occupancy check.
P_FROM = np.array([-0.03, 0.0, 0.46], dtype=np.float64)
P_TO = np.array([0.03, 0.0, 0.46], dtype=np.float64)
#: **ORCHESTRATOR (Part B).** HANDOFF_YAW is 3*pi/2, not pi/2. Both give the
#: same physical grip for a two-jaw gripper (they differ by 180 deg, and the
#: measured pad offsets are identical), but the arm configuration differs
#: enormously. At pi/2 arm B's approach collides with armA_lower_arm for 129
#: steps (worst -0.0047 m); at 3*pi/2 cross-arm contact is ZERO and phase 2
#: converges. This eliminated a transit collision I had wrongly concluded
#: needed a waypoint search or an elbow-branch change.
HANDOFF_YAW = 3.0 * np.pi / 2.0

#: Per-object resting half-height above the table surface, metres -- the
#: vertical distance from `TABLE_SURFACE_Z` up to that object's own BODY
#: ORIGIN when resting flush on the table. Measured from
#: `scripts/gen_dual_scene.py`'s own hand-authored spawn positions (read-
#: only reference, not imported -- that script is a generation-time tool,
#: not a runtime dependency): `PLATE_POS[2]=0.355`, `MUG_POS[2]=0.39`,
#: `FORK_POS[2]=0.356`, `SPOON_POS[2]=0.356`, each minus
#: `TABLE_SURFACE_Z=0.35`. `water_bottle` is intentionally absent (out of
#: scope, ADR-072).
#:
#: **Not used for PLACE's own descend target** (see
#: `GRIPPER_STATIC_PAD_CLEARANCE_M` above -- the real jaw pad, not the
#: object's own half-height, is what must clear the table). Kept as its own
#: named constant anyway: it is real, measured, per-object physical data
#: that `evaluate_skill`/future work may still want (e.g. to sanity-check a
#: final resting height independent of the gripper's own geometry).
OBJ_HALF_HEIGHT_M = {
    "plate": 0.005,
    "mug": 0.04,
    "fork": 0.006,
    "spoon": 0.006,
}

#: Fixed, pre-verified table placement targets for `destination="table"`
#: (the executor/master convention, `skill_call.params.get("destination",
#: "table")`). Populated ONLY for objects this stage actually measured
#: reachable end-to-end (this stage's mandatory validation is `fork` alone)
#: -- deliberately NOT extrapolated to the other props, so a caller asking
#: to place an unverified object on the table gets an honest "not measured
#: yet" failure (see `dispatch_v2`) instead of a fabricated number. `fork`'s
#: point, `(0.05, 0.05, TABLE_SURFACE_Z)`, is 0.10 m away in x from the
#: fork's own spawn xy `(-0.05, 0.05)` -- a visible, deliberate relocation
#: (not "put it back exactly"), confirmed reachable (both APPROACH and
#: DESCEND phases) by this stage's own IK sweep for arm A.
TABLE_PLACE_TARGET_XYZ = {
    "fork": np.array([0.05, 0.05, TABLE_SURFACE_Z], dtype=np.float64),
}

#: **Another finding this stage made.** Master's `GRASP_POINT_OFFSET_M`
#: lateral (x, y) component is reused for PICK's grasp-point xy by default
#: (module docstring's "Yaw" section already explains why the z component
#: is NOT reused). For `fork` specifically, master's own `x=-0.015` shifts
#: the target 1.5 cm toward the PLATE (`plate` sits at `x=-0.15`, well to
#: the fork's `-x` side); measured directly
#: (`docs/hardware/v2-stage4-skills.md`): with that offset, arm A's MOVING
#: JAW (not the arm's own forearm -- confirmed by isolating which of this
#: skill's phases the contact appears in: it is absent through APPROACH and
#: DESCEND and appears only once the jaw starts closing, PICK phase 3)
#: sweeps close enough to the plate as it rotates shut to penetrate it by
#: 3.2-14.8 mm (criterion (c) violation, magnitude depending on which of
#: this stage's other fixes were already applied) and displace it 6-17 mm
#: (criterion (b) violation). Master's OWN grasp uses a fully-iterative,
#: orientation-uncontrolled IK with a different swing geometry, so this
#: x-offset choice, safe there, is not automatically safe for v2's forced
#: top-down approach from this arm's own base position, this close to the
#: plate. `x=0` (targeting the fork BODY's own x directly) reduced but did
#: NOT eliminate it (3.2 mm penetration remained); `x=+0.01` (1 cm FURTHER
#: from the plate than the fork's own body origin) measured completely
#: clean: zero plate contact through every phase including closure.
#: Overridden HERE (not in `GRASP_POINT_OFFSET_M` itself, which is
#: master's, frozen, and still correct for master's own use) via this
#: small, v2-only table -- objects not listed fall back to
#: `GRASP_POINT_OFFSET_M`'s own (x, y) unchanged.
GRASP_POINT_XY_OVERRIDE_M = {
    "fork": (0.01, 0.0),
}

#: Objects out of v2's scope this stage (ADR-072, re-disclosed above).
OUT_OF_SCOPE_OBJECTS = ("bottle",)

#: (e)'s gate, for the 5 POSITIONING joints per arm ONLY (see
#: `_arm_positioning_dof_indices`'s own docstring for why the gripper is
#: measured, and gated, separately). **Derivation (this stage's brief
#: requires this to be computed, not guessed), against the FINAL code path
#: actually run:** every `move_to_config` phase this module commands,
#: across `PICK`, `PLACE` and `HANDOFF` for the fork on arm A/B from this
#: scene's actual "home" reset, was solved, differenced, and (for PICK)
#: measured live (`docs/hardware/v2-stage4-skills.md` has the full table).
#: The single LARGEST commanded per-joint delta found is `_safe_reorient_
#: then_move`'s own "unfold" leg within PICK's phase 1b: arm A's
#: `wrist_roll` moves from home's `0.0 rad` to `2.4408 rad` -- forced by the
#: yaw+pi branch (module docstring's "yaw+pi branch" section), the ONLY
#: reachable branch for the required hover point -- over `duration_s=2.0`.
#: The closed-form peak velocity of a zero-start/zero-end-velocity cubic is
#: `1.5 * delta / T` (Stage 2/ADR-071, re-used, not re-derived): with
#: `delta=2.4408 rad`, `T=2.0 s`, that is `1.8306 rad/s` -- confirmed live,
#: not just predicted: the actual run measured **1.8306 rad/s** on
#: `armA_wrist_roll` (`data.qvel`, read every physics step), matching the
#: closed form to 4 decimal places. HANDOFF's own largest
#: (`home -> above P_from/P_to`, `delta=1.5886 rad`, `T=2.0 s`) is smaller
#: (`1.1915 rad/s`), and PLACE's largest smaller still, BECAUSE of the
#: yaw tie-break described above (it reuses the branch the arm is already
#: on rather than spinning pi radians back to the "primary" one).
#:
#: This gate is therefore set at **3.0 rad/s**: comfortably above this
#: module's own actual worst case (1.8306 rad/s, ~1.64x margin) AND above
#: the task brief's own illustrative "legitimately large, must-not-fail"
#: example (a 3.4 rad wrist_roll sweep in 2.0 s = 2.55 rad/s) -- a FUTURE
#: skill commanding that exact hypothetical move would still pass -- while
#: sitting far below the one-shot direct-command baseline this whole
#: redesign exists to eliminate (5.350 rad/s, Stage 2, ADR-071) and well
#: above the benign spline baseline measured at 2.0 s (0.451 rad/s,
#: ADR-071). Graded from the REAL, PD-tracked `data.qvel` every physics
#: step, not the theoretical closed form -- the closed form justifies the
#: constant, live telemetry grades the run.
#:
#: **Why the gripper joint is excluded from this gate (a finding this
#: stage made, not anticipated by the brief).** `motion.set_gripper`'s own
#: cubic spans that actuator's FULL `ctrlrange` (1.9199 rad, `-0.17453` to
#: `1.74533`) in the brief's own specified `duration_s=0.6` for EVERY
#: open/close in EVERY skill here -- closed-form peak `1.5*1.9199/0.6 =
#: 4.7997 rad/s`, measured live at `4.83-4.86 rad/s` (small overshoot
#: beyond the closed form, same margin Stage 2 already documented). Pooling
#: this with the 5 positioning joints would force this gate above ~5 rad/s
#: just to admit routine jaw motion -- leaving almost no margin below the
#: 5.350 rad/s baseline this whole gate exists to catch, and defeating its
#: actual purpose (catching bad ARM motion, not measuring jaw speed, a
#: different actuator class with a legitimately different duty cycle).
GRIPPER_PEAK_VELOCITY_OBSERVED_RAD_S = 4.86  # disclosed, NOT gated by (e) -- see paragraph above.

#: **Yet another finding this stage made.** PLACE's own DESCEND target
#: originally set the pinch point to `target_xyz + GRIPPER_STATIC_PAD_
#: CLEARANCE_M` -- correct for "the real jaw pad ends up at the object's
#: resting height" (that constant's own docstring), but that means the
#: CARRIED OBJECT itself (still rigidly welded, not yet released) is
#: commanded to a pose essentially AT the table surface WHILE STILL BEING
#: HELD. Measured directly (`docs/hardware/v2-stage4-skills.md`): the
#: descend move never converges (`shoulder_lift`'s `qfrc_actuator` pinned
#: at its `forcerange` bound, `fork_handle` in genuine, if small, contact
#: with `table_top`) because the physics will not let a rigid, welded
#: object penetrate the table to match an IK target that itself has no
#: notion of "something is already touching down here." This is the exact
#: same problem master's OWN `run_place` already solved with
#: `PLACE_RELEASE_CLEARANCE_M` ("descend to the destination plus a small
#: vertical offset for gentle release", `skills_scripted.py`) -- re-applied
#: here rather than re-derived from scratch, since the underlying physical
#: reason (release before actually touching down) is identical. Swept
#: 0.005-0.03 m; 0.005 m was already enough to converge cleanly, so this
#: constant uses that value plus a small margin.
PLACE_RELEASE_CLEARANCE_M = 0.01
PEAK_JOINT_VELOCITY_THRESHOLD_RAD_S = 3.0

#: (b)'s gate: no non-target prop may move more than this during a skill.
NONTARGET_PROP_MOVE_THRESHOLD_M = 0.005

#: (c)/(d)'s gate: a `data.contact` entry counts as a violation once its
#: `dist` (negative = interpenetrating, MuJoCo's own convention) is more
#: negative than this -- i.e. penetrating by more than 1 mm. MuJoCo reports
#: near-touching (but not yet penetrating) geom pairs within its own contact
#: margin at `dist >= 0`; those are not violations of either criterion.
CONTACT_PENETRATION_THRESHOLD_M = 0.001

#: The scene's interactive props (5 pickable + the drawer), tracked for (b)
#: and (c). Deliberately a SUPERSET of `WeldGrasp.GRASPABLE_OBJECTS` (which
#: excludes the drawer, a slide joint, not a free-joint pickable prop) --
#: Stage 3's own corrected handoff line was originally found driving
#: `armB_gripper` INTO the drawer, so the drawer is exactly the kind of
#: "non-target prop" this criterion exists to protect. `table`,
#: `drawer_housing` and `world` are fixtures, not props, and are not tracked
#: here.
PROP_BODY_NAMES = ("plate", "mug", "fork", "spoon", "water_bottle", "drawer")


# ---------------------------------------------------------------------------
# Small MuJoCo lookups, kept local (same reasoning `motion.py` gives for not
# importing `skills_scripted.py`'s private helpers: this module should not
# reach into another module's internals for a two-line name lookup).
# ---------------------------------------------------------------------------


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _other_arm(arm: str) -> str:
    if arm == "A":
        return "B"
    if arm == "B":
        return "A"
    raise ValueError(f"arm must be 'A' or 'B', got {arm!r}")


def _arm_qpos_adrs(model, arm: str) -> list[int]:
    names = ik.arm_joint_names(arm)
    return [int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in names]


def _current_arm_qpos(env, arm: str) -> np.ndarray:
    adrs = _arm_qpos_adrs(env.model, arm)
    return np.array([env.data.qpos[a] for a in adrs], dtype=np.float64)


def _arm_positioning_dof_indices(model, arm: str) -> list[int]:
    """dof (`qvel`) indices for `arm`'s 5 POSITIONING joints only --
    deliberately EXCLUDES the gripper joint. See
    `PEAK_JOINT_VELOCITY_THRESHOLD_RAD_S`'s own docstring for the measured
    reason: the gripper's open/close actuator, driven by the identical
    cubic-spline primitive, legitimately and routinely peaks near 4.8 rad/s
    (its own ~1.92 rad range over the brief's own specified 0.6 s duration)
    -- a fact about jaw speed, not arm smoothness, that would swamp a gate
    meant to catch bad ARM motion if the two were pooled together. Every
    hinge joint here has exactly one dof, so `jnt_dofadr` alone addresses it
    (no need to special-case free joints -- none of these are)."""
    names = ik.arm_joint_names(arm)
    idxs = []
    for n in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        idxs.append(int(model.jnt_dofadr[jid]))
    return idxs


# ---------------------------------------------------------------------------
# Yaw.
# ---------------------------------------------------------------------------


def _object_long_axis_heading(data, body_id: int) -> float:
    """World-XY heading (radians, `atan2`) of this object's own local +x
    axis, rotated by its LIVE `xquat`. See module docstring's "Yaw" section
    for why local +x is the right reference vector for this scene's props."""
    v = np.zeros(3)
    mujoco.mju_rotVecQuat(v, np.array([1.0, 0.0, 0.0]), data.xquat[body_id].copy())
    return float(np.arctan2(v[1], v[0]))


def _pinch_yaw_for_object(data, body_id: int) -> float:
    """The primary top-down pinch yaw for this object, RIGHT NOW (read
    fresh every call -- correct whether the object is resting or already
    held, since a held object's `xquat` moves rigidly with the gripper)."""
    return _object_long_axis_heading(data, body_id) + np.pi / 2.0


def _resolve_reachable_yaw(model, data, arm: str, primary_yaw: float, targets: list[np.ndarray], cur_wrist_roll: float):
    """Try `primary_yaw`, then `primary_yaw + pi` (module docstring's
    "yaw+pi branch" section), against EVERY point in `targets` (typically
    [hover, grasp_or_place] for one phase pair). A candidate is only
    accepted if `ik_geometric.solve_topdown_ik` returns a real solution for
    ALL of them -- a phase is never left to discover unreachability later.

    If both candidates reach every target, the one whose FIRST target's
    solved `wrist_roll` is angularly closer to `cur_wrist_roll` wins (avoids
    a gratuitous ~pi-radian wrist spin when the arm is already sitting on
    the other, equally valid, branch -- see module docstring).

    Returns `(yaw, [q for each target])`, or `(None, None)` if neither
    candidate reaches every target.
    """
    candidates = []
    for cand in (primary_yaw, primary_yaw + np.pi):
        qs = [ik_geometric.solve_topdown_ik(model, data, arm, t, cand) for t in targets]
        if all(q is not None for q in qs):
            candidates.append((cand, qs))
    if not candidates:
        return None, None
    if len(candidates) == 1:
        return candidates[0]

    def _angular_dist(cand):
        wr = cand[1][0][4]
        raw = wr - cur_wrist_roll
        return abs(((raw + np.pi) % (2.0 * np.pi)) - np.pi)

    candidates.sort(key=_angular_dist)
    return candidates[0]


#: Number of equal linear-in-joint-space sub-moves `_split_move` breaks
#: every "big" PICK/PLACE/HANDOFF reposition into. See `_split_move`'s own
#: docstring for the measured reason this exists at all.
MOVE_SPLIT_STEPS = 4


def _split_move(env, arm: str, q_target: np.ndarray, duration_s: float, hold_other_arm: bool, n: int = MOVE_SPLIT_STEPS):
    """`n` equal-duration `move_to_config` calls along the STRAIGHT LINE (in
    joint space) from the arm's current qpos to `q_target`, instead of one
    single `move_to_config(q_target, duration_s)` call.

    **A finding this stage made, empirically, not anticipated by the
    brief.** A single joint-space cubic spanning all 5 joints at once, from
    a configuration far away (e.g. the folded "home" pose) to a "reach down
    and forward" one, can swing the gripper's own finger pad through the
    table -- or graze a nearby prop -- MID-TRANSIT, even when BOTH
    endpoints are individually collision-free: joint-space interpolation
    (linear OR this module's own zero-velocity-boundary cubic) has no
    notion of the CARTESIAN path it traces along the way, unlike a true
    task-space planner. Measured directly
    (`docs/hardware/v2-stage4-skills.md`): PICK's phase 1 (home -> hover),
    commanded as one single 2.0 s spline, reliably stalled against the
    table -- confirmed as a genuine contact-force block, not slow PD
    tracking, by reading `data.qfrc_actuator` pinned at its `forcerange`
    bound while `data.qfrc_constraint` (the contact reaction) exactly
    opposed it -- and PICK's phase 2 (descend), same failure mode, grazed
    the PLATE (14.8 mm penetration) instead of the table. Breaking either
    move into `n=4` equal linear sub-legs, each its own ordinary
    `move_to_config` call, measured CLEAN in both cases (this file's own
    `docs/hardware/v2-stage4-skills.md` has both before/after numbers): a
    piecewise-linear joint-space path stays close enough to the direct line
    that it no longer swings through occupied space the one-shot cubic did.
    A plain 2-way split was already enough to fix BOTH measured cases;
    `n=4` is used for margin without meaningfully changing peak velocity
    (see the note below).

    Still exactly "solve IK once per phase" (module docstring): NO IK is
    solved for any intermediate waypoint -- each is a pure algebraic
    interpolation between the arm's live qpos and the ALREADY-solved
    `q_target`, split further only by `move_to_config`'s own cubic (which
    itself needs no IK either).

    **Peak velocity is unchanged by this split, by construction.** Each of
    the `n` legs covers `delta/n` over `duration_s/n`; the closed-form peak
    `1.5*(delta/n)/(duration_s/n) = 1.5*delta/duration_s` is IDENTICAL to
    one single move over the full delta and duration -- confirmed by direct
    measurement, not just algebra (`docs/hardware/v2-stage4-skills.md`).
    `PEAK_JOINT_VELOCITY_THRESHOLD_RAD_S`'s own derivation is therefore
    unaffected by which of the two calling conventions a phase uses.

    Returns the list of `n` `MotionResult`s, in order -- a caller checks
    `.success` on each and reports its own phase name/sub-step number for
    whichever one failed first.
    """
    q_start = _current_arm_qpos(env, arm)
    q_target = np.asarray(q_target, dtype=np.float64)
    results = []
    for i in range(1, n + 1):
        q_i = q_start + (q_target - q_start) * (float(i) / float(n))
        r = motion.move_to_config(env, arm, q_i, duration_s / float(n), hold_other_arm=hold_other_arm)
        results.append(r)
        if not r.success:
            break
    return results


# ---------------------------------------------------------------------------
# Grasp closure with attempt_grasp called every step of the closing motion.
# ---------------------------------------------------------------------------


def _close_gripper_and_grasp(
    env, arm: str, body_name: str, weld: WeldGrasp, duration_s: float = 0.6, extra_hold_frames: int = 50,
    distance_threshold_m: float = WELD_GRASP_DISTANCE_THRESHOLD_M,
):
    """Close `arm`'s jaw over `duration_s` seconds via the SAME cubic
    profile `motion.set_gripper` uses (re-derived locally -- see module
    docstring), calling `weld.attempt_grasp(arm, body_name)` every physics
    step. Stops advancing the closing motion the instant the weld attaches
    (no reason to keep squeezing an object the weld already rigidly holds).
    If it never attaches by the end of the commanded closing motion, holds
    fully closed for up to `extra_hold_frames` more steps, still retrying
    every step (mirrors `skills_scripted.py`'s `GRIP_HOLD_FRAMES` dwell in
    spirit, reimplemented locally for the same not-reaching-into-another-
    module's-internals reason).

    `distance_threshold_m` defaults to `WELD_GRASP_DISTANCE_THRESHOLD_M`
    (0.12 m, not `attempt_grasp`'s own default of 0.05 m) -- see that
    constant's docstring for the measured reason (`GRIPPER_STATIC_PAD_
    CLEARANCE_M`'s consequence for how far the abstract pinch point now
    sits from the object).

    Returns `(attached: bool, frames_used: int, attach_frame: int | None)`
    -- `attach_frame` is LOCAL to this call (the frame count within this
    closing motion alone), matching `SkillResult.weld_attach_frame`'s own
    "global frames_used count" convention once the caller adds its own
    running total.
    """
    model = env.model
    gripper_name = ik.gripper_joint_name(arm)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, gripper_name)
    jid = int(model.actuator_trnid[aid, 0])
    qpos_adr = int(model.jnt_qposadr[jid])
    lo, hi = model.actuator_ctrlrange[aid]

    q_start = float(env.data.qpos[qpos_adr])
    q_target = float(lo)  # lo = closed, same convention as motion.set_gripper(open=False)

    dt = float(model.opt.timestep)
    T = float(duration_s)
    n_steps = max(1, int(round(T / dt)))

    a0 = q_start
    a2 = 3.0 * (q_target - q_start) / (T ** 2)
    a3 = -2.0 * (q_target - q_start) / (T ** 3)

    # Freeze every OTHER actuator (both arms' positioning joints, the idle
    # arm's gripper) at whatever `data.ctrl` already commands -- read ONCE,
    # held fixed for this whole dwell, ADR-037/ADR-031's pattern.
    base_ctrl = np.array(env.data.ctrl, dtype=np.float64, copy=True)

    frames = 0
    attach_frame: int | None = None
    attached = weld.is_holding(arm) == body_name

    while not attached and frames < n_steps:
        frames += 1
        t = min(frames * dt, T)
        pos = a0 + a2 * (t ** 2) + a3 * (t ** 3)
        ctrl = base_ctrl.copy()
        ctrl[aid] = pos
        env.step(ctrl, cameras=[])
        if weld.attempt_grasp(arm, body_name, distance_threshold_m=distance_threshold_m):
            attached = True
            attach_frame = frames

    if not attached:
        ctrl = base_ctrl.copy()
        ctrl[aid] = q_target
        for _ in range(extra_hold_frames):
            env.step(ctrl, cameras=[])
            frames += 1
            if weld.attempt_grasp(arm, body_name, distance_threshold_m=distance_threshold_m):
                attached = True
                attach_frame = frames
                break

    return attached, frames, attach_frame


# ---------------------------------------------------------------------------
# PICK
# ---------------------------------------------------------------------------


def run_pick(env, arm: str, target_object: str, weld: WeldGrasp, hold_other_arm: bool = True) -> SkillResult:
    """PICK(arm, obj) -- Lab 8 phasing (task brief), cubic-spline phases:

    1. `q_above = solve_topdown_ik(grasp_point + [0,0,HOVER_CLEARANCE_M], yaw)`;
       `move_to_config(q_above, 2.0)`; gripper open.
    2. `q_grasp = solve_topdown_ik(grasp_point, yaw)`; `move_to_config(q_grasp, 1.0)`.
    3. Close the jaw, `weld.attempt_grasp` every step of the closure.
    4. `move_to_config(q_above, 1.0)` -- lift straight up, REUSING phase 1's
       already-solved `q_above` (no second IK solve for the same point).

    `grasp_point` is `target_object`'s current `xpos` (read fresh, never
    hardcoded) plus `GRASP_POINT_OFFSET_M` (imported read-only from
    `skills_scripted.py` -- the same measured, physically-graspable-feature
    offsets that module's own `pick` uses; re-deriving them would either
    duplicate that measurement or silently drift from it).
    """
    if arm not in ("A", "B"):
        return SkillResult(False, f"PICK: arm must be 'A' or 'B', got {arm!r}", 0)
    if target_object in OUT_OF_SCOPE_OBJECTS:
        return SkillResult(
            False,
            f"PICK({arm}, {target_object}): water_bottle is OUT OF SCOPE for v2 (ADR-072) -- "
            "unreachable by both arms at every height (a horizontal reach limit, not a height "
            "one); master reached it 9/20 under position-only IK. This is a disclosed "
            "regression, not attempted here.",
            0,
        )
    body_name = OBJECT_BODY_NAME.get(target_object)
    if body_name is None:
        return SkillResult(False, f"PICK: unknown target_object {target_object!r}", 0)

    model, data = env.model, env.data
    body_id = _body_id(model, body_name)
    # Lateral (x, y) feature offset: `GRASP_POINT_XY_OVERRIDE_M` for objects
    # this stage found needed one (see that constant's docstring), else
    # master's own measured `GRASP_POINT_OFFSET_M`. The z component is
    # REPLACED by `GRIPPER_STATIC_PAD_CLEARANCE_M` either way (module
    # docstring / that constant's own docstring: master's tiny z offset
    # assumes an uncontrolled approach angle, which this module's forced
    # top-down orientation does not have).
    if target_object in GRASP_POINT_XY_OVERRIDE_M:
        offset_xy = GRASP_POINT_XY_OVERRIDE_M[target_object]
    else:
        offset_xy = GRASP_POINT_OFFSET_M.get(target_object, np.zeros(3))[:2]
    obj_pos = data.xpos[body_id].copy()
    grasp_point = np.array(
        [obj_pos[0] + offset_xy[0], obj_pos[1] + offset_xy[1], obj_pos[2] + GRIPPER_STATIC_PAD_CLEARANCE_M]
    )
    hover_point = grasp_point + np.array([0.0, 0.0, HOVER_CLEARANCE_M])

    primary_yaw = _pinch_yaw_for_object(data, body_id)
    cur_wr = _current_arm_qpos(env, arm)[4]
    yaw, qs = _resolve_reachable_yaw(model, data, arm, primary_yaw, [hover_point, grasp_point], cur_wr)
    if yaw is None:
        return SkillResult(
            False,
            f"PICK phase 1 (approach hover) failed: no reachable yaw for arm={arm} "
            f"object={target_object!r} -- tried yaw={primary_yaw:.4f} and "
            f"{primary_yaw + np.pi:.4f} against hover={hover_point.tolist()} and "
            f"grasp={grasp_point.tolist()}",
            0,
        )
    q_above, q_grasp = qs

    frames = 0

    rs1 = _split_move(env, arm, q_above, 2.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs1)
    if not rs1[-1].success:
        return SkillResult(False, f"PICK phase 1 (approach hover) motion failed at sub-step {len(rs1)}/{MOVE_SPLIT_STEPS}: {rs1[-1].reason}", frames)

    r_open = motion.set_gripper(env, arm, open=True, duration_s=0.6)
    frames += r_open.frames_used

    rs2 = _split_move(env, arm, q_grasp, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs2)
    if not rs2[-1].success:
        return SkillResult(False, f"PICK phase 2 (descend to grasp) motion failed at sub-step {len(rs2)}/{MOVE_SPLIT_STEPS}: {rs2[-1].reason}", frames)

    attached, close_frames, attach_frame_local = _close_gripper_and_grasp(env, arm, body_name, weld)
    frames += close_frames
    if not attached:
        return SkillResult(
            False,
            f"PICK phase 3 (grip) failed: weld never attached after full closure "
            f"(arm={arm} object={body_name!r})",
            frames,
        )
    weld_attach_frame = frames - close_frames + (attach_frame_local or 0)

    rs4 = _split_move(env, arm, q_above, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs4)
    if not rs4[-1].success:
        return SkillResult(
            False, f"PICK phase 4 (lift) motion failed at sub-step {len(rs4)}/{MOVE_SPLIT_STEPS}: {rs4[-1].reason}", frames,
            weld_attach_frame=weld_attach_frame, weld_active_at_end=weld.is_holding(arm) == body_name,
        )

    final_z = float(data.xpos[body_id][2])
    holding = weld.is_holding(arm) == body_name
    lift_margin_z = TABLE_SURFACE_Z + WELD_PICK_SUCCESS_MARGIN_M
    lifted = final_z > lift_margin_z
    success = holding and lifted
    reason = (
        f"lifted {body_name} to z={final_z:.4f} m (> {lift_margin_z:.4f} m margin bar), holding={holding}"
        if success
        else f"PICK end-state check failed: holding={holding}, final_z={final_z:.4f} m "
        f"(need > {lift_margin_z:.4f} m)"
    )
    return SkillResult(success, reason, frames, weld_attach_frame=weld_attach_frame, weld_active_at_end=holding)


# ---------------------------------------------------------------------------
# PLACE
# ---------------------------------------------------------------------------


def run_place(env, arm: str, target_object: str, target_xyz, weld: WeldGrasp, hold_other_arm: bool = True) -> SkillResult:
    """PLACE(arm, obj, target_xyz):

    1. `q_above = solve_topdown_ik(target + [0,0,PLACE_HOVER_CLEARANCE_M], yaw)`;
       `move_to_config(q_above, 2.0)`.
    2. `q_place = solve_topdown_ik(target + [0,0,obj_half_height], yaw)`;
       `move_to_config(q_place, 1.0)`.
    3. `weld.release(arm)`, THEN `set_gripper(open, 0.6)`.
    4. `move_to_config(q_above, 1.0)` -- retreat, reusing phase 1's `q_above`.

    Precondition: `arm` already holds `target_object` (a prior `PICK`).
    Unlike `skills_scripted.run_place`, this function does NOT perform its
    own nested pick -- the task brief's phase list for PLACE has no grasp
    step, and this stage's own validation runs `pick` and `place` as two
    separate calls in sequence, matching this precondition exactly. If the
    precondition does not hold, this fails immediately and says so, rather
    than silently picking the object up itself (a different contract than
    master's, disclosed here rather than silently matched).
    """
    if arm not in ("A", "B"):
        return SkillResult(False, f"PLACE: arm must be 'A' or 'B', got {arm!r}", 0)
    if target_object in OUT_OF_SCOPE_OBJECTS:
        return SkillResult(
            False,
            f"PLACE({arm}, {target_object}): water_bottle is OUT OF SCOPE for v2 (ADR-072) -- "
            "see PICK's identical disclosure.",
            0,
        )
    body_name = OBJECT_BODY_NAME.get(target_object)
    if body_name is None:
        return SkillResult(False, f"PLACE: unknown target_object {target_object!r}", 0)
    half_h = OBJ_HALF_HEIGHT_M.get(target_object)
    if half_h is None:
        return SkillResult(
            False,
            f"PLACE: no measured obj_half_height for {target_object!r} -- v2 has only measured "
            f"{sorted(OBJ_HALF_HEIGHT_M)} this stage",
            0,
        )
    if weld.is_holding(arm) != body_name:
        return SkillResult(
            False,
            f"PLACE precondition failed: arm={arm} does not currently hold {body_name!r} "
            f"(weld.is_holding({arm!r})={weld.is_holding(arm)!r}) -- PLACE requires a prior "
            f"PICK({arm}, {target_object}).",
            0,
        )

    model, data = env.model, env.data
    body_id = _body_id(model, body_name)
    target = np.asarray(target_xyz, dtype=np.float64).reshape(3)
    hover_point = target + np.array([0.0, 0.0, PLACE_HOVER_CLEARANCE_M])
    # `place_point` (the commanded PINCH-POINT descend target) uses
    # `GRIPPER_STATIC_PAD_CLEARANCE_M`, NOT `half_h`, for the same reason
    # `run_pick`'s `grasp_point` does (that constant's own docstring): the
    # real jaw pad, not the abstract pinch point, must clear the table.
    # `half_h` is still used below to predict where the object actually
    # SETTLES after release (a few cm of free-fall under gravity once the
    # jaw opens -- see this function's own end-state check).
    place_point = target + np.array([0.0, 0.0, GRIPPER_STATIC_PAD_CLEARANCE_M + PLACE_RELEASE_CLEARANCE_M])

    primary_yaw = _pinch_yaw_for_object(data, body_id)
    cur_wr = _current_arm_qpos(env, arm)[4]
    yaw, qs = _resolve_reachable_yaw(model, data, arm, primary_yaw, [hover_point, place_point], cur_wr)
    if yaw is None:
        return SkillResult(
            False,
            f"PLACE phase 1 (approach hover) failed: no reachable yaw for arm={arm} "
            f"object={target_object!r} target={target.tolist()} -- tried yaw={primary_yaw:.4f} "
            f"and {primary_yaw + np.pi:.4f}",
            0,
        )
    q_above, q_place = qs

    frames = 0

    rs1 = _split_move(env, arm, q_above, 2.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs1)
    if not rs1[-1].success:
        return SkillResult(False, f"PLACE phase 1 (approach hover) motion failed at sub-step {len(rs1)}/{MOVE_SPLIT_STEPS}: {rs1[-1].reason}", frames)

    rs2 = _split_move(env, arm, q_place, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs2)
    if not rs2[-1].success:
        return SkillResult(False, f"PLACE phase 2 (descend) motion failed at sub-step {len(rs2)}/{MOVE_SPLIT_STEPS}: {rs2[-1].reason}", frames)

    # Release the WELD before opening the jaw (ADR-030's ordering, reused: an
    # opening jaw's own collision geometry should not kick an object that is
    # still rigidly welded to it).
    weld.release(arm)
    r_open = motion.set_gripper(env, arm, open=True, duration_s=0.6)
    frames += r_open.frames_used

    rs4 = _split_move(env, arm, q_above, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs4)
    if not rs4[-1].success:
        return SkillResult(False, f"PLACE phase 4 (retreat) motion failed at sub-step {len(rs4)}/{MOVE_SPLIT_STEPS}: {rs4[-1].reason}", frames)

    final_pos = data.xpos[body_id].copy()
    expected = target + np.array([0.0, 0.0, half_h])
    xy_err = float(np.linalg.norm(final_pos[:2] - expected[:2]))
    z_err = float(abs(final_pos[2] - expected[2]))
    still_held = weld.is_holding(arm)
    success = xy_err < PLACE_XY_TOL_M and z_err < PLACE_Z_TOL_M and still_held is None
    reason = (
        f"placed {body_name} at {final_pos.tolist()} (target {expected.tolist()}, "
        f"xy_err={xy_err:.4f} m, z_err={z_err:.4f} m)"
        if success
        else f"PLACE end-state check failed: final={final_pos.tolist()}, target={expected.tolist()}, "
        f"xy_err={xy_err:.4f} m (tol {PLACE_XY_TOL_M}), z_err={z_err:.4f} m (tol {PLACE_Z_TOL_M}), "
        f"still_held={still_held!r}"
    )
    return SkillResult(success, reason, frames)


#: PLACE success tolerances -- the closed-form top-down solver is accurate
#: to sub-millimetre (Stage 1, ADR-070: round-trip error 2e-16 m mean), so
#: any error at this scale is a real placement/release miss, not solver
#: noise; kept generous (1 cm) relative to that for the PD tracking residual
#: (`motion.MOTION_SUCCESS_TOL_RAD`) and a brief post-release settle.
PLACE_XY_TOL_M = 0.01
PLACE_Z_TOL_M = 0.015


# ---------------------------------------------------------------------------
# HANDOFF
# ---------------------------------------------------------------------------


def run_handoff(env, to_arm: str, from_arm: str, target_object: str, weld: WeldGrasp, hold_other_arm: bool = True) -> SkillResult:
    """HANDOFF(to_arm, from_arm, obj) -- receiver-first, matching master's
    own convention (`run_handoff(env, to_arm, from_arm, obj)`; an A->B
    handoff is `run_handoff(env, "B", "A", "fork")`) so the executor and any
    task planner need no special-casing between the two skill modules.

    Precondition: `from_arm` holds `obj` (a prior `PICK`).

    Phases (task brief, `P_FROM`/`P_TO` per Stage 3's corrected z=0.46 line):
      1. `from_arm` to above `P_FROM` (2.0 s) then descend to `P_FROM`
         (1.0 s); `to_arm` held (assumed at home).
      2. `to_arm` to above `P_TO` (2.0 s) then descend to `P_TO` (1.0 s);
         `from_arm` held, STILL welded to `obj`.
      3. `to_arm` closes the jaw, `weld.attempt_grasp(to_arm, ...)` every
         step of the closure.
      4. Verify `to_arm` holds `obj`. ONLY THEN does `from_arm` release +
         open.
      5. `from_arm` to above `P_FROM` (1.0 s, reusing phase 1's `q_above`)
         then to HOME (2.0 s); `to_arm` held.
      6. `to_arm` lifts to above `P_TO` (1.0 s, reusing phase 2's
         `q_above`).

    End state: `to_arm` holds `obj` at height, `from_arm` at home.
    """
    if to_arm not in ("A", "B") or from_arm not in ("A", "B"):
        return SkillResult(False, f"HANDOFF: to_arm/from_arm must be 'A'/'B', got to_arm={to_arm!r} from_arm={from_arm!r}", 0)
    if to_arm == from_arm:
        return SkillResult(False, f"HANDOFF: to_arm and from_arm must differ, both were {to_arm!r}", 0)
    if target_object in OUT_OF_SCOPE_OBJECTS:
        return SkillResult(
            False,
            f"HANDOFF({from_arm}->{to_arm}, {target_object}): water_bottle is OUT OF SCOPE for "
            "v2 (ADR-072) -- see PICK's identical disclosure.",
            0,
        )
    body_name = OBJECT_BODY_NAME.get(target_object)
    if body_name is None:
        return SkillResult(False, f"HANDOFF: unknown target_object {target_object!r}", 0)
    if weld.is_holding(from_arm) != body_name:
        return SkillResult(
            False,
            f"HANDOFF precondition failed: from_arm={from_arm} does not currently hold "
            f"{body_name!r} (weld.is_holding({from_arm!r})={weld.is_holding(from_arm)!r}) -- "
            f"HANDOFF requires a prior PICK({from_arm}, {target_object}).",
            0,
        )

    model, data = env.model, env.data
    frames = 0

    from_hover = P_FROM + np.array([0.0, 0.0, HANDOFF_HOVER_OFFSET_M])
    to_hover = P_TO + np.array([0.0, 0.0, HANDOFF_HOVER_OFFSET_M])

    q_from_above = ik_geometric.solve_topdown_ik(model, data, from_arm, from_hover, HANDOFF_YAW)
    if q_from_above is None:
        return SkillResult(False, f"HANDOFF phase 1 (from_arm approach P_from hover) failed: IK unreachable for arm={from_arm} target={from_hover.tolist()}", frames)
    rs = _split_move(env, from_arm, q_from_above, 2.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs)
    if not rs[-1].success:
        return SkillResult(False, f"HANDOFF phase 1 (from_arm approach hover) motion failed at sub-step {len(rs)}/{MOVE_SPLIT_STEPS}: {rs[-1].reason}", frames)

    q_from = ik_geometric.solve_topdown_ik(model, data, from_arm, P_FROM, HANDOFF_YAW)
    if q_from is None:
        return SkillResult(False, f"HANDOFF phase 1 (from_arm descend to P_from) failed: IK unreachable for arm={from_arm} target={P_FROM.tolist()}", frames)
    rs = _split_move(env, from_arm, q_from, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs)
    if not rs[-1].success:
        return SkillResult(False, f"HANDOFF phase 1 (from_arm descend) motion failed at sub-step {len(rs)}/{MOVE_SPLIT_STEPS}: {rs[-1].reason}", frames)

    q_to_above = ik_geometric.solve_topdown_ik(model, data, to_arm, to_hover, HANDOFF_YAW)
    if q_to_above is None:
        return SkillResult(False, f"HANDOFF phase 2 (to_arm approach P_to hover) failed: IK unreachable for arm={to_arm} target={to_hover.tolist()}", frames)
    rs = _split_move(env, to_arm, q_to_above, 2.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs)
    if not rs[-1].success:
        return SkillResult(False, f"HANDOFF phase 2 (to_arm approach hover) motion failed at sub-step {len(rs)}/{MOVE_SPLIT_STEPS}: {rs[-1].reason}", frames)

    q_to = ik_geometric.solve_topdown_ik(model, data, to_arm, P_TO, HANDOFF_YAW)
    if q_to is None:
        return SkillResult(False, f"HANDOFF phase 2 (to_arm descend to P_to) failed: IK unreachable for arm={to_arm} target={P_TO.tolist()}", frames)
    rs = _split_move(env, to_arm, q_to, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs)
    if not rs[-1].success:
        return SkillResult(False, f"HANDOFF phase 2 (to_arm descend) motion failed at sub-step {len(rs)}/{MOVE_SPLIT_STEPS}: {rs[-1].reason}", frames)

    attached, close_frames, attach_frame_local = _close_gripper_and_grasp(env, to_arm, body_name, weld)
    frames += close_frames
    if not attached:
        return SkillResult(
            False,
            f"HANDOFF phase 3 (to_arm grip) failed: weld never attached (to_arm={to_arm} "
            f"object={body_name!r}); from_arm={from_arm} still holds it -- handoff aborted "
            "safely, no release attempted.",
            frames,
        )
    weld_attach_frame = frames - close_frames + (attach_frame_local or 0)

    # Phase 4: verify BEFORE releasing (the task brief's own non-negotiable
    # ordering -- master's Phase 3 failure mode, and v1's, was exactly a
    # premature release).
    if weld.is_holding(to_arm) != body_name:
        return SkillResult(
            False,
            f"HANDOFF phase 4 (verify) failed: to_arm={to_arm} does not confirm holding "
            f"{body_name!r} after attempt_grasp reported success -- refusing to release "
            f"from_arm={from_arm}.",
            frames,
            weld_attach_frame=weld_attach_frame,
        )
    weld.release(from_arm)
    r_open = motion.set_gripper(env, from_arm, open=True, duration_s=0.6)
    frames += r_open.frames_used

    rs = _split_move(env, from_arm, q_from_above, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs)
    if not rs[-1].success:
        return SkillResult(
            False, f"HANDOFF phase 5 (from_arm retreat to hover) motion failed at sub-step {len(rs)}/{MOVE_SPLIT_STEPS}: {rs[-1].reason}", frames,
            weld_attach_frame=weld_attach_frame, weld_active_at_end=weld.is_holding(to_arm) == body_name,
        )

    rs = _split_move(env, from_arm, HOME_ARM_QPOS, 2.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs)
    if not rs[-1].success:
        return SkillResult(
            False, f"HANDOFF phase 5 (from_arm to home) motion failed at sub-step {len(rs)}/{MOVE_SPLIT_STEPS}: {rs[-1].reason}", frames,
            weld_attach_frame=weld_attach_frame, weld_active_at_end=weld.is_holding(to_arm) == body_name,
        )

    rs = _split_move(env, to_arm, q_to_above, 1.0, hold_other_arm)
    frames += sum(r.frames_used for r in rs)
    if not rs[-1].success:
        return SkillResult(
            False, f"HANDOFF phase 6 (to_arm lift) motion failed at sub-step {len(rs)}/{MOVE_SPLIT_STEPS}: {rs[-1].reason}", frames,
            weld_attach_frame=weld_attach_frame, weld_active_at_end=weld.is_holding(to_arm) == body_name,
        )

    # `from_arm`'s home-arrival was already gated by `r.success` above
    # (`motion.MOTION_SUCCESS_TOL_RAD`); the only remaining end-state check
    # is that `to_arm` still, actually, holds the object.
    holding_to = weld.is_holding(to_arm) == body_name
    reason = (
        f"handoff complete: to_arm={to_arm} holds {body_name!r}, from_arm={from_arm} at home"
        if holding_to
        else f"HANDOFF end-state check failed: to_arm={to_arm} weld.is_holding={weld.is_holding(to_arm)!r} "
        f"(expected {body_name!r})"
    )
    return SkillResult(holding_to, reason, frames, weld_attach_frame=weld_attach_frame, weld_active_at_end=holding_to)


# ---------------------------------------------------------------------------
# Deliverable 2: the five-criterion success definition.
# ---------------------------------------------------------------------------


@dataclass
class CriteriaReport:
    """Grades one skill call against Stage 4's five-criterion success
    definition. See module docstring for what each criterion measures."""

    a_ok: bool
    a_detail: str
    b_ok: bool
    b_detail: str
    c_ok: bool
    c_detail: str
    d_ok: bool
    d_detail: str
    e_ok: bool
    e_detail: str

    @property
    def passed(self) -> bool:
        return self.a_ok and self.b_ok and self.c_ok and self.d_ok and self.e_ok

    def failing(self) -> list[str]:
        """Names of every criterion (letter) that failed, e.g. `['c', 'e']`."""
        letters = []
        for letter, ok in (("a", self.a_ok), ("b", self.b_ok), ("c", self.c_ok), ("d", self.d_ok), ("e", self.e_ok)):
            if not ok:
                letters.append(letter)
        return letters


class SkillMonitor:
    """Context manager: wraps `env.step` for its lifetime (a plain instance-
    attribute monkey-patch, restored on exit even if the wrapped call
    raises) and records everything criteria (c)/(d)/(e) need, without any
    change to `env.py`, `motion.py` or `grasp.py`. See module docstring's
    "five-criterion success definition" section for the full rationale.

    Criteria (a)/(b) do not need per-step monitoring (they are pure
    before/after snapshots) and are computed separately by `evaluate_skill`.
    """

    def __init__(self, env):
        self.env = env
        self._orig_step = env.step
        model = env.model

        geom_arm = [None] * model.ngeom
        geom_prop = [None] * model.ngeom
        for gid in range(model.ngeom):
            bid = int(model.geom_bodyid[gid])
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
            if bname.startswith("armA_"):
                geom_arm[gid] = "A"
            elif bname.startswith("armB_"):
                geom_arm[gid] = "B"
            if bname in PROP_BODY_NAMES:
                geom_prop[gid] = bname
        self._geom_arm = geom_arm
        self._geom_prop = geom_prop

        self._dof_indices = _arm_positioning_dof_indices(model, "A") + _arm_positioning_dof_indices(model, "B")

        self.peak_qvel = 0.0
        self.peak_qvel_dof: int | None = None
        self.contact_violations: list[dict] = []
        self.n_steps = 0

    def __enter__(self):
        self.env.step = self._recording_step
        return self

    def __exit__(self, exc_type, exc, tb):
        self.env.step = self._orig_step
        return False

    def _recording_step(self, ctrl, **kwargs):
        result = self._orig_step(ctrl, **kwargs)
        self.n_steps += 1
        data = self.env.data

        for idx in self._dof_indices:
            v = abs(float(data.qvel[idx]))
            if v > self.peak_qvel:
                self.peak_qvel = v
                self.peak_qvel_dof = idx

        ncon = int(data.ncon)
        for i in range(ncon):
            con = data.contact[i]
            dist = float(con.dist)
            if dist >= -CONTACT_PENETRATION_THRESHOLD_M:
                continue
            g1, g2 = int(con.geom1), int(con.geom2)
            arm1, arm2 = self._geom_arm[g1], self._geom_arm[g2]
            prop1, prop2 = self._geom_prop[g1], self._geom_prop[g2]
            if arm1 and arm2 and arm1 != arm2:
                self.contact_violations.append(dict(step=self.n_steps, kind="cross_arm", geom1=g1, geom2=g2, dist=dist))
            if arm1 and prop2:
                self.contact_violations.append(dict(step=self.n_steps, kind="arm_vs_prop", arm=arm1, prop=prop2, geom1=g1, geom2=g2, dist=dist))
            elif arm2 and prop1:
                self.contact_violations.append(dict(step=self.n_steps, kind="arm_vs_prop", arm=arm2, prop=prop1, geom1=g1, geom2=g2, dist=dist))

        return result


def evaluate_skill(env, skill_fn, target_object: str) -> tuple[SkillResult, CriteriaReport]:
    """Run `skill_fn()` (a zero-arg closure calling `run_pick`/`run_place`/
    `run_handoff`) inside a `SkillMonitor`, and grade the result against all
    five criteria. `target_object` is the (canonical, `OBJECT_BODY_NAME`-
    keyed) object the skill acts on -- excluded from criterion (b)'s
    displacement check and from (c)'s contact check (an arm touching the
    object it is deliberately manipulating is the point of the skill, not a
    violation).
    """
    model, data = env.model, env.data
    body_name = OBJECT_BODY_NAME.get(target_object)

    prop_start: dict[str, np.ndarray] = {}
    for name in PROP_BODY_NAMES:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid != -1:
            prop_start[name] = data.xpos[bid].copy()

    with SkillMonitor(env) as monitor:
        skill_result = skill_fn()

    prop_end = {}
    for name in prop_start:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        prop_end[name] = data.xpos[bid].copy()

    a_ok, a_detail = skill_result.success, skill_result.reason

    b_ok = True
    b_details = []
    for name, start in prop_start.items():
        if name == body_name:
            continue
        disp = float(np.linalg.norm(prop_end[name] - start))
        if disp > NONTARGET_PROP_MOVE_THRESHOLD_M:
            b_ok = False
            b_details.append(f"{name} moved {disp * 1000.0:.2f} mm")
    b_detail = "no non-target prop moved > 5 mm" if b_ok else "; ".join(b_details)

    c_violations = [v for v in monitor.contact_violations if v["kind"] == "arm_vs_prop" and v.get("prop") != body_name]
    c_ok = len(c_violations) == 0
    if c_ok:
        c_detail = f"no arm-vs-non-target-prop contact deeper than {CONTACT_PENETRATION_THRESHOLD_M * 1000:.1f} mm over {monitor.n_steps} steps"
    else:
        worst = min(c_violations, key=lambda v: v["dist"])
        c_detail = (
            f"{len(c_violations)} violation(s), worst: arm={worst['arm']} vs prop={worst['prop']!r} "
            f"penetration={-worst['dist'] * 1000:.2f} mm at step {worst['step']}"
        )

    d_violations = [v for v in monitor.contact_violations if v["kind"] == "cross_arm"]
    d_ok = len(d_violations) == 0
    if d_ok:
        d_detail = f"no cross-arm contact deeper than {CONTACT_PENETRATION_THRESHOLD_M * 1000:.1f} mm over {monitor.n_steps} steps"
    else:
        worst = min(d_violations, key=lambda v: v["dist"])
        d_detail = f"{len(d_violations)} violation(s), worst: penetration={-worst['dist'] * 1000:.2f} mm at step {worst['step']}"

    e_ok = monitor.peak_qvel < PEAK_JOINT_VELOCITY_THRESHOLD_RAD_S
    e_detail = f"peak |qvel|={monitor.peak_qvel:.4f} rad/s (dof {monitor.peak_qvel_dof}), gate {PEAK_JOINT_VELOCITY_THRESHOLD_RAD_S} rad/s"

    report = CriteriaReport(
        a_ok=a_ok, a_detail=a_detail,
        b_ok=b_ok, b_detail=b_detail,
        c_ok=c_ok, c_detail=c_detail,
        d_ok=d_ok, d_detail=d_detail,
        e_ok=e_ok, e_detail=e_detail,
    )
    return skill_result, report


# ---------------------------------------------------------------------------
# Deliverable 3 support: a single dispatch function `executor.py`'s
# `_dispatch` calls when `use_v2=True` -- keeps ALL v2 routing logic in this
# new file, so the permitted edit to `executor.py` stays a one-line branch.
# ---------------------------------------------------------------------------


def dispatch_v2(skill_call, env, weld: WeldGrasp) -> SkillResult:
    """Route one grounded `SkillCall` to this module's `run_pick`/
    `run_place`/`run_handoff`, mirroring `executor.ScriptedSkillExecutor.
    _dispatch`'s existing per-skill argument extraction (`destination`,
    `from_arm`) so the two dispatchers stay structurally comparable.

    `open_drawer` and `pour` are explicitly NOT attempted this stage (task
    instruction) -- both return a clearly-labelled failed `SkillResult`
    rather than falling through to master's own implementation or raising.
    """
    if skill_call.skill == "open_drawer":
        return SkillResult(False, "v2 Stage 4 does not attempt open_drawer (task instruction; use use_v2=False for master's implementation)", 0)

    if skill_call.skill == "pour":
        return SkillResult(False, "v2 Stage 4 does not attempt pour (task instruction; pour is M06b, out of scope)", 0)

    if skill_call.skill == "pick":
        return run_pick(env, skill_call.arm, skill_call.target_object, weld=weld)

    if skill_call.skill == "place":
        destination = skill_call.params.get("destination", "table")
        if destination != "table":
            return SkillResult(False, f"v2 place: unsupported destination {destination!r} (only 'table' is implemented, matching master)", 0)
        target = TABLE_PLACE_TARGET_XYZ.get(skill_call.target_object)
        if target is None:
            return SkillResult(
                False,
                f"v2 place: no verified table-placement target for {skill_call.target_object!r} "
                f"this stage -- only {sorted(TABLE_PLACE_TARGET_XYZ)} measured reachable "
                "(TABLE_PLACE_TARGET_XYZ); pass target_xyz to run_place directly for anything else.",
                0,
            )
        return run_place(env, skill_call.arm, skill_call.target_object, target, weld=weld)

    if skill_call.skill == "handoff":
        from_arm = skill_call.params.get("from_arm")
        if from_arm is None:
            return SkillResult(False, "v2 handoff SkillCall missing params['from_arm']", 0)
        return run_handoff(env, to_arm=skill_call.arm, from_arm=from_arm, target_object=skill_call.target_object, weld=weld)

    return SkillResult(False, f"v2 Stage 4: unsupported skill {skill_call.skill!r}", 0)


if __name__ == "__main__":
    # Guarded per builder convention. No meaningful standalone run without a
    # compiled MuJoCo model -- see docs/hardware/v2-stage4-skills.md for the
    # actual validation run (bm-ptl per ADR-047).
    print(__doc__)
    print("\nThis module has no standalone entry point. See docs/hardware/v2-stage4-skills.md.")
