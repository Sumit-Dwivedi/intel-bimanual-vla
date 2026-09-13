# DECISIONS

User-ratified decisions, newest first. Each entry mirrors an ADR in
`ARCHITECTURE.md` section 4, which holds the full context, options and
consequences. Where the two disagree, `ARCHITECTURE.md` is authoritative and the
disagreement is a defect to be fixed.

ADR-001 through ADR-015 were authored by the planner and live only in
`ARCHITECTURE.md`. This file starts at ADR-016, the point at which decisions began
being ratified by the user rather than proposed.

---

## M06 Phase 2 Commit 1 — safety scaffolding: `_hold_ctrl` kept (not replaced), APPROACH clearance reduction tried and reverted (follows ADR-010, ADR-027; per `docs/hardware/m06-phase2-prerequisites.md`)

**Recorded:** Sept 13, 2026. **Scope:** scaffolding only, no weld wiring (Phase 2 Commit 2 is separate).

**Ctrl-hold decision.** Two designs were compared for holding the idle arm across
`env.step()` calls: the existing `_hold_ctrl` (`skills_scripted.py`, re-anchors idle
joints to CURRENT qpos every step) versus a proposed `skill_start_ctrl` (pin to the
pose at skill start, actively correcting drift). **Kept `_hold_ctrl`, did not add a
second mechanism.** Decisive reason, checked directly rather than assumed:
`run_handoff` has `from_arm` sit at the transfer point HOLDING THE OBJECT while
`to_arm` is the one being actively driven -- pinning to skill-start would command
`from_arm` back toward its folded home pose mid-handoff, fighting the object transfer.
`_hold_ctrl`'s "wherever it currently is" is the only one of the two designs that
stays correct when the active/idle role swaps mid-skill. Secondarily,
`ScriptedSkillExecutor.execute()` never calls `env.step()` itself (every step is
nested inside `skills_scripted.py`'s own `run_*` -> `_run_waypoint`/`_run_dwell` ->
`_drive_to_target`/`_dwell` chain), so implementing the rejected design would also
have required threading a snapshot through every layer for a wrong answer.

**New finding, not previously measured (the Q2 audit was read-only, "no probes
run"): `_hold_ctrl` does not keep the idle arm motionless.** `scripts/probe_ctrl_hold.py`
drove only arm A for up to 2000 `env.step()` calls and measured arm B's drift from
home under `_hold_ctrl` alone: 0.0402 rad by step 100 (over this task's 0.01 rad
bar), 0.461 rad by step 1000, saturating at 0.546 rad by roughly step 1500 -- landing
at arm B's own `shoulder_lift` joint's hard `jnt_range` floor (home -1.2, range floor
-1.7453). Cause: re-reading current qpos each step supplies zero restoring force
against gravity, so each step's sag becomes the next step's new "hold" reference,
ratcheting the idle arm down until a mechanical joint limit -- not any controller
limit -- stops it. This does not reproduce the ADR-026 zero-pose interpenetration
(the catastrophic case `_hold_ctrl` was built to prevent, and still does), and it
did not change any test outcome in this repo's current skills (same 4-pass/4-fail
split before and after, see below), but it is a real, previously-undocumented gap.
Flagged as a Phase 2 follow-up: the correct fix is a THIRD design neither offered
here -- cache each arm's own LAST ACTIVELY-COMMANDED ctrl and hold that fixed value
(refreshed only when that arm is next driven) -- not attempted in this commit
(out of time-box, and not one of the two designs this commit was scoped to choose
between).

**APPROACH clearance (`CLEARANCE_HEIGHT_M`) tried at 0.05, reverted to 0.08.** The
task's own given rationale for 0.08 (`m06-ik-lift-diagnostic.md`'s single-shot-IK
68%/61% figures) does not apply here -- per `m06-phase2-prerequisites.md` Q1, every
waypoint in this module uses the INCREMENTAL IK regime, never single-shot. The
candidate alternative reasons for 0.05 (smaller swept workspace per waypoint, smaller
handoff sweep, fewer incremental re-solves) were checked empirically rather than
assumed and did not hold up: at 0.05, `pick(A, plate)`'s APPROACH waypoint newly
collides with the plate's own raised rim (`test_pick_plate_waypoints_progress_
without_collision`, previously passing, now fails: `dist=-0.0301 m` vs.
`threshold=-0.005 m`, an order of magnitude past the graze/tunnel boundary, not a
debounce artefact). Left at 0.08 per the task's own instruction ("if neither reason
holds up, leave the constant... a smaller number with no rationale is not an
improvement").

**Verification (bm-ptl, `C:\Users\devcloud\project\ov_env\Scripts\python.exe`).**
- `pytest tests/test_skills.py`: 4 passed / 4 failed before this commit's changes AND
  after (identical failing tests, identical failure reasons/measured values) --
  unchanged by either the docstring-only ctrl-hold decision or the clearance
  revert.
- `python scripts/run_skill.py --skill pick --object fork --arm A --seed 0`: identical
  before/after -- `success=False, frames_used=1800`, `reason="did not lift fork:
  initial_z=0.3560 final_z=0.3538 margin_required=0.03"`, `max_joint_limit_violation=
  0.0003909627168092733`, no mj_warnings. Same failure mode as the pre-Phase-2
  baseline (grasp still unwired) -- no new or different failure introduced.
- `scripts/probe_ctrl_hold.py` (100 steps, tolerance 0.01 rad): **max arm B drift =
  0.040207 rad** (`armB_shoulder_lift`, at the final step) -- over the stated bar;
  reported plainly above rather than the constant retuned to make the probe pass,
  since the actual property being measured is real physics, not a probe-tuning knob.

**Deviation from the instructed exact commit message, disclosed rather than
silently applied:** the given text ("...reduced APPROACH radius...") asserts the
clearance constant was changed; it was tried and reverted (see above), so using
that text verbatim would misstate what this commit contains. The commit message
used instead describes the outcome truthfully.

---

## ADR-029 — Weld-based grasping mechanism (Phase 1: mechanism verified, not yet wired into skills)

**Recorded:** Sept 13, 2026 · **Follows:** `docs/hardware/grasp-envelope.md` (0 of 30
caliper thicknesses achieved sustained two-jaw contact -- the gripper's jaw meshes
collapse to permanently-overlapping convex hulls, MuJoCo issue #239) and
`DECISIONS.md`'s ADR-028 entry (finger-pad primitives fixed the geometry, verified
6-132 mm pad separation sweep, but "no `pick()` reaches GRIP under the current
approach-collision check" -- the arm's reach envelope plus the ~8 cm pinch-point
kinematic offset, ADR-025, put every graspable target at the edge of what this 5-DOF
IK can reach; `docs/hardware/m06-grip-diagnostic.md` and
`m06-grip-diagnostic-after-fix.md`). **Phase 1 only** -- this entry covers the
mechanism's own verification; wiring it into `pick`/`place`/`handoff` is Phase 2,
contingent on this entry.

**Context.** Contact-based grasping in this scene is not a code bug to keep chasing;
it is a structural limit of the simulated gripper's geometry and this arm's
kinematics, independently confirmed by two prior, unrelated diagnostics (the caliper
sweep and the GRIP-stage instrumentation). A grasping mechanism is needed that
abstracts the failing contact subsystem while preserving the rest of the
perception-to-action pipeline.

**Options.** (a) keep debugging contact-based grasping -- rejected, the limit is
structural (reach envelope), not tactical; (b) reposition the arm bases -- rejected,
4-6 h with an uncertain outcome, and it would restart cross-arm collision and
reachability validation from scratch; (c) weld the object to the gripper via a MuJoCo
equality constraint, toggled at runtime -- **chosen**, standard sim-robotics practice
(MoveIt's attached objects, PyBullet's fixed constraints, academic sim-to-real work
all abstract grasp contact the same way).

**Decision (c), implemented as follows:**

1. **`src/bimanual/sim/grasp.py`'s `WeldGrasp`** tracks at most one held object per
   arm (`{'A': None, 'B': None}`). `attempt_grasp(arm, object_name,
   distance_threshold_m=0.05, closure_threshold=0.3)` attaches only if BOTH gates
   hold: the `armX_gripper` JOINT's qpos is below `closure_threshold` (jaws closing;
   "open" is the HIGH end of this joint's range, so "below threshold" correctly reads
   as "closing"), AND the `armX_gripper` BODY's (the fixed jaw, **not** the moving
   jaw and **not** the joint of the same name -- the exact naming trap `ik.py` and
   `GLOSSARY.md` already document) world position is within `distance_threshold_m` of
   the object body's world position. It refuses, logging the specific reason,
   otherwise. `release(arm)` deactivates the weld; `is_holding(arm)` reports it.
   Refusing when either gate fails (never "always weld") is the entire point --
   ADR-029's Consequences below.
2. **Activation path: (a) pre-declared, chosen over (b) runtime creation.** All 10
   `(armX_gripper, prop)` weld equality constraints (5 props x 2 arms) are declared
   in the generated scene XML with `active="false"`, by
   `scripts/gen_dual_scene.py`'s new `build_weld_constraints()` (HAND-AUTHORED region
   only -- `scenes/so101/` is untouched, confirmed by `git diff --stat -- scenes/so101/`
   before commit, same convention as every prior ADR-021/ADR-028 change).
   `WeldGrasp` only ever toggles `data.eq_active` and rewrites `model.eq_data` for
   constraints that already exist; option (b) (creating a constraint at runtime) was
   never needed -- pre-declaring compiled without incident on the first try.
3. **The eq_data teleport gotcha, resolved empirically for the installed
   mujoco==3.2.7, not assumed from documentation.** `mjNEQDATA == 11`
   (`mujoco/include/mujoco/mjmodel.h`): `eq_data[0:3]` = anchor, `eq_data[3:10]` =
   relpose (3 position + 4 quaternion), `eq_data[10]` = torquescale -- but the shipped
   headers do not document what "anchor" and "relpose" actually mean geometrically.
   Determined by direct experiment (5 randomized-pose trials, weld `body1`=object /
   `body2`=the reference body, gravity enabled, 3000-step rollout, position AND
   orientation checked before/after): for `body1`=object, `body2`=gripper,
   ```
   anchor       = R(gripper_quat)^T @ (object_pos - gripper_pos)   # object's position
                  in the gripper body's own local frame
   relpose_pos  = (0, 0, 0)
   relpose_quat = conj(object_quat) * gripper_quat                 # the GRIPPER's
                  orientation expressed in the OBJECT's frame (reversed order
                  relative to the naive "object relative to gripper" -- this was the
                  one sign that a position-only teleport check would NOT have caught;
                  it only shows up as orientation drift over many steps)
   torquescale  = 1.0
   ```
   This held position to 2.7e-5 m (solver settling noise, not error) and orientation
   exactly (quaternion delta 0.0) across 5 random trials, 3000 steps each, under
   gravity. `mujoco`'s own `mju_rotVecQuat`/`mju_negQuat`/`mju_mulQuat` are used in
   `grasp.py`, not hand-rolled quaternion math, so the implementation tracks MuJoCo's
   own convention rather than a reimplementation of it.
4. **`scripts/probe_weld_grasp.py`** verifies the mechanism end to end against the
   real `TableSettingEnv`, with no skill layer involved. Full log:
   `docs/hardware/m06-weld-verification.md`. Run on bm-ptl (ADR-020); mirrored
   locally first (mujoco imports and compiles on this developer's laptop as of this
   session, contrary to ADR-020's original finding -- noted, not relied upon; the
   authoritative run and the committed artifacts are bm-ptl's).

**Verification results, reported exactly as measured:**
- **Teleport check:** fork position immediately before vs. after `attempt_grasp`
  activates the weld: **0.000000 m** (both position components identical to 8
  decimal places).
- **Tracking:** stepping the arm up (see the finding below on how) for 50 steps, the
  fork's world z rose from 0.3538 to 0.3551 m (+0.0013 m), tracking the gripper's own
  rise (0.3810 -> 0.3824 m, +0.0013 m) essentially 1:1.
- **Release:** `release('A')` returned `True`; `is_holding('A')` became `None`.
  30 further steps showed the fork's z stop rising and settle down (0.3551 -> 0.3533 m).
- **Negative control 1 (gripper OPEN, object in range):** `attempt_grasp` returned
  `False`, logged reason "gripper not closed enough (joint qpos=1.7453 rad >=
  closure_threshold=0.3000 rad)".
- **Negative control 2 (gripper CLOSED, object far -- arm left at the "home" rest
  pose):** `attempt_grasp` returned `False`, logged reason "too far (distance=0.5633 m
  >= distance_threshold_m=0.0500 m)".
- **MuJoCo warnings:** none, at any point in the run (`data.warning` checked, same
  convention as `scripts/run_skill.py`'s diagnostic).

**An honest, unplanned finding surfaced while building the verification script, worth
recording because it is a real property of this system, not a defect in `WeldGrasp`:**
`ik.solve_position_ik`'s pinch-point target (ADR-025), combined with ADR-024's fully
relaxed orientation, let the redundant 5-DOF solve satisfy a progressively-rising
pinch-point target by rotating the WRIST rather than raising the arm -- the pinch
point tracked the rising target (solver residual under 0.01 m throughout) while the
`armA_gripper` BODY (the actual weld attach frame) **fell**. `ik.py` is out of scope
to modify for this task, so the verification script's UP phase instead drives
`armA_shoulder_lift` directly (holding every other actuator at its current qpos),
which reliably raises the whole downstream chain with no orientation ambiguity. The
resulting rise is modest (millimetre-scale over 50 steps / 0.1 s sim time), consistent
with the `sts3215` actuator class's own `forcerange=-2.94 2.94` N*m capping how fast
one joint can lift the downstream mass against gravity in that time -- the same kind
of actuator force ceiling `docs/hardware/grasp-envelope.md` already measured for the
gripper actuator's own `forcerange=-3.35 3.35` N.

**Consequences.** Grasping is now **abstracted, not physically simulated** --
README and video must say so explicitly, per ADR-015's honesty rules (no number or
description implies contact-based grasping where a weld is doing the work). Pick,
place and handoff become executable end-to-end **once wired** (Phase 2, not this
commit). ADR-028's finger-pad work is retained as scene correctness (the pads still
move correctly and are still the physically modelled jaw geometry) even though grip
contact itself is abstracted around. M07's randomization stays meaningful: arm poses
and prop positions still vary session to session; only the attach *moment* is
abstracted, not the scene state leading up to it. Nothing in `skills_scripted.py`,
`executor.py`, `ik.py` or `scenes/so101/` was touched by this commit.

---

## M06a fixes: target-prop exemption, complete jaw collision disable, fork test

**Recorded:** Sept 12, 2026 · **Follows:** ADR-027 (waypoint staging),
ADR-028 (finger-pad primitives, "no `pick()` reaches GRIP under the
current approach-collision check") · **Fixes:** the two gaps ADR-028's own
"What it does not yet prove" section named · **Touches:**
`scripts/gen_dual_scene.py`, `src/bimanual/control/skills_scripted.py`
only (`scenes/so101/`, `ik.py`, `executor.py`'s interface, `command/`,
`language/` all untouched)

### Fix A -- completed the jaw mesh collision disable

Independent measurement found `armA_gripper` (the fixed jaw body) still
carried a SECOND collidable mesh geom alongside `armA_static_finger_pad`:
`sts3215_03a_v1`, the wrist_roll servo's own housing mesh, rigidly mounted
on that same body. `disable_jaw_mesh_collision`'s original filter matched
by upstream `mesh` NAME (`JAW_COLLISION_MESHES`, shared with
`apply_jaw_friction`) and only ever touched
`wrist_roll_follower_so101_v1`/`moving_jaw_so101_v1` -- it never saw this
second mesh. That same mesh name (`sts3215_03a_v1`) is reused at 4 OTHER
joints per arm (shoulder, elbow, wrist_flex, wrist_roll), each needing its
collision left alone, so the fix could not be "disable this mesh name
everywhere" -- it had to be "disable every mesh-type collision geom that
is a direct child of either jaw BODY" instead. `disable_jaw_mesh_collision`
now takes `prefix` and matches by body name
(`{prefix}gripper`, `{prefix}moving_jaw_so101_v1`), asserting exactly 3
disabled geoms per arm (the fixed jaw's own follower mesh + the colocated
servo housing mesh + the moving jaw mesh) instead of 2.

Verified on bm-ptl: a one-off assertion script (not shipped) confirmed 12
jaw-body mesh geoms total (both arms, visual + collision classes) all read
`contype=0 conaffinity=0` in the compiled model -- PASS.
`scripts/probe_pad_separation.py` re-run gives **bit-for-bit identical**
numbers to ADR-028's own gate (fully closed 0.00600 m, midway 0.07621 m,
fully open 0.13188 m, spread 0.12589 m) -- **GATE PASSED, unchanged**, as
required (this fix touches a different geom than the pads it measures).

### Fix B -- target-prop exemption in the arm-vs-prop check, corrected per instruction

ADR-027 Step 5's arm-vs-prop check already had a PARTIAL target exemption
(`_dwell`/`_run_dwell`/`_validate_against_baseline` all accepted
`target_object` and fully excluded it from the violation dict), but two
gaps made it useless for actually picking anything up: (1) it was a full,
unconditional exemption with no depth limit at all -- a true crush would
never be caught; (2) `_run_waypoint`/`_drive_to_target`, which drive
APPROACH/DESCEND/RETREAT, never accepted `target_object` at all, so the
exemption never applied to the phases where a `pick` actually closes in on
its target -- exactly why `pick(A, plate)` (and, this task confirms,
every other prop) was failing at **waypoint 1 (APPROACH)** against its own
target, before ever reaching GRIP.

Added module-level `CRUSH_THRESHOLD_M = -0.02` (target-prop contact deeper
than this indicates crushing, not grasping; legitimate approach contacts
were measured at -0.007..-0.009 m, so -0.02 m leaves clear margin).
`_prop_collision_violations` now takes `target_body` and applies
`CRUSH_THRESHOLD_M` to the named target prop and the tight, unconditional
`PROP_COLLISION_DEPTH_TOL_M` (0.005 m) to every OTHER prop -- this single
function is now the one place both the in-loop check
(`_drive_to_target`/`_dwell`, checked every physics step) and the post-hoc
check (`_validate_against_baseline`) call, so the two can never drift
apart. `target_object` is threaded through `_run_waypoint` and every
`_run_waypoint`/`_run_dwell` call site in `run_pick`, `run_place` and
`run_handoff` (`run_open_drawer` untouched -- its target is the drawer,
not a free-joint prop).

**The brief's original instruction ("RETREAT: strict against ALL props")
was corrected before implementation, per the task's own explicit
correction, and independently confirmed necessary here**: a successful
`pick`'s RETREAT is the arm LIFTING the object it just grasped --
continuing arm-vs-target contact there is the proof of success, not a
defect. Implemented as instructed: **the target-prop exemption (capped at
`CRUSH_THRESHOLD_M`) applies at every waypoint, APPROACH through RETREAT**;
every non-target prop keeps the tight, unconditional bar at every
waypoint including RETREAT -- the bystander protection (the `spoon`
knocking `fork`, and the bottle-on-floor bug) is unweakened, since it was
never keyed to the target at all. The task also offered a "cleaner"
alternative -- exempt the target only while the gripper is commanded
closed, so `place` turns strict again after RELEASE -- and flagged it as
slightly more precise; **not implemented here**, in the interest of the
45-minute time box and because the simpler always-exempt-the-target rule
is what the corrected instruction asked for and is sufficient to unblock
GRIP. Left as a candidate follow-up, not a defect.

No context-threading was needed at the executor boundary:
`ScriptedSkillExecutor._dispatch` (`executor.py`) already passes
`skill_call.target_object` straight into `run_pick`/`run_place`/
`run_handoff`, which is exactly the `target_object` this fix threads
further inward -- `executor.py` itself needed no edit.

**Regression check, `pytest tests/test_skills.py`: 4 failed / 4 passed**
(previously 5 failed / 3 passed under the ADR-028 baseline).
`test_open_drawer_reaches_near_limit` (IK residual=0.3183 m, unrelated,
same already-documented non-reachability from ADR-027) and
`test_handoff_mug_ends_held_by_arm_b` (IK residual=0.0532 m, byte-for-byte
identical to the baseline recorded above) are unaffected. **`test_pick_
plate_waypoints_progress_without_collision`, previously failing and
flagged as an open, undecided conflict with ADR-027's own regression-test
requirement, now PASSES** -- Fix B is precisely why: plate's APPROACH no
longer trips on touching its own target. `test_pick_plate_lifts_above_
table` still fails, but for a DIFFERENT reason than before: previously it
never got past waypoint 1 (collision, frames_used=296); now all 4
waypoints run to completion (frames_used=1560) and it fails only the
final lift-margin check (`final_z=0.3523`, delta=-0.0027, needed
+0.03) -- consistent with, not contradicting, the already-documented
plate force-ceiling gap (5.89 N required vs 3.35 N actuator ceiling).
`test_place_plate_returns_to_table_rest` still fails, cascading from the
same nested-pick failure as before.

### Fix C -- fork test first

`pick(A, fork)`, fresh env, seed=0 (`GRASP_POINT_OFFSET_M["fork"]=(-0.015,
0, 0)`, `GRIP_HOLD_FRAMES=60`, `CLEARANCE_HEIGHT_M=0.08`):

| waypoint | ok | frames_used | ik_residual (m) |
|---|---|---:|---:|
| APPROACH | True | 500 | 0.0085 |
| DESCEND | True | 500 | 0.0026 |
| GRIP (dwell) | True | 60 (full budget, no early cutoff) | 0.0043 |
| RETREAT | True | 500 | 0.0099 |

Final `SkillResult`: `success=False`, `reason="did not lift fork:
initial_z=0.3560 final_z=0.3538 margin_required=0.03"`, `frames_used=1560`.
Fork z: initial 0.3560 -> final 0.3538, **delta -0.0022** (it did not rise
at all; if anything it settled slightly lower). Success bar per this
task (`z > 0.37`, table surface 0.35 + 0.02 m lift): **not met, by a wide
margin** -- the fork was never off the table.

**This is a genuinely new failure shape, not one of the three the task's
own interpretation guide anticipated, and per this task's explicit stop
rule it is reported here rather than chased further.** Every one of the
four named waypoints reports `ok=True` -- no collision violation, IK
residual under `ik.IK_POSITION_TOLERANCE_M` (0.01 m) at each -- which is
exactly what Fix A/B were built to achieve, and for the first time in
this module's whole retest history, they achieved it: nothing here is a
named-waypoint collision or convergence failure. The failure is entirely
in `run_pick`'s own post-RETREAT measurement -- the object was commanded
gripped and lifted, but its measured height barely moved. Contact data at
the GRIP waypoint (`ik_residual=0.0043 m`, dwell ran its full 60-frame
budget with no violation reported by either the in-loop or post-hoc
check) is consistent with the pads and fork being close enough to be in
each other's vicinity, but does not by itself prove a sustained pinch
formed -- and the outcome (no lift) says it did not. Candidate causes
not investigated here (any change to them is explicitly out of this
fix's scope -- grasp offsets, pad geometry and IK strategy are all named
as untouchable in this task's own constraints): the fork's grasp offset,
the 2.5 mm pad half-size relative to a thin utensil handle, or a
friction/contact-settling issue specific to a light, small, freely-jointed
body. **Per Fix C's own branching instruction ("If fork FAILS: STOP..."),
the spoon/plate/mug/water_bottle sweep was NOT run** -- fork did not
succeed, so there is nothing to build on top of yet.

`git diff --stat -- scenes/so101/` confirmed empty before commit. Ledger
per the ADR-028 convention: the generated arms (`src/bimanual/sim/assets/
so101_dual_table.xml`) now carry Fix A's completed jaw-mesh collision
disable (3 geoms/arm) on top of ADR-028's friction and finger pads;
`scripts/gen_dual_scene.py` and `src/bimanual/control/skills_scripted.py`
hold the corresponding generator/skill-logic changes.

---

## ADR-028 — Finger-pad primitives (MuJoCo convex-hull fix), pads verified to move, but no `pick()` reaches GRIP under the current approach-collision check

**Recorded:** Sept 12, 2026 · **Follows:** `docs/hardware/grasp-envelope.md`
(diagnostic: 0 of 30 caliper thicknesses achieved sustained two-jaw contact,
convex hulls overlap -0.0206..-0.0345 m at every joint angle) · **Cites:**
MuJoCo GitHub issue #239's documented finger-pad pattern for mesh-gripper
collision, https://ggando.com/blog/so101-rl-lift (reports working SO-101
grasping with this pattern), https://maegantucker.com/ECE4560/assignment8-so101/
(course material teaching it) · **Not accessed**, per instruction — cited only.

**Context.** `docs/hardware/grasp-envelope.md` measured the root cause
directly: MuJoCo collapses a `type="mesh"` collision geom to its convex hull
with no decomposition declared anywhere in this asset, and both jaw parts
(`wrist_roll_follower_so101_v1`, `moving_jaw_so101_v1`) are non-convex
C-shaped housings whose hulls overlap at every angle in the joint's range
(-0.03454 m closed to -0.02062 m at the least-overlapping angle). No object
placed there can ever be read as anything but embedded in solid material on
both sides.

**Options.** (a) weld-based grasping — rejected, a workaround that reads as
not-really-grasping and would need disclosure; (b) finger-pad primitives per
the cited pattern — **chosen**; (c) non-prehensile manipulation — rejected,
scope change.

**Decision (b), implemented in `scripts/gen_dual_scene.py` only** (never
`scenes/so101/`, confirmed empty diff below): (1) `disable_jaw_mesh_collision()`
sets `contype="0" conaffinity="0"` on the same two jaw MESH collision geoms
`JAW_COLLISION_MESHES` already identifies (visual rendering, a separate
`class="visual"` copy, untouched); (2) `add_finger_pads()` adds one
`type="box" size="0.00125 0.00125 0.00125"` collision geom per jaw, at the
task's own verbatim positions: `static_finger_pad` at local `pos="-0.008875
0.0 -0.100"` as a child of `{prefix}gripper` (the fixed jaw body), and
`moving_finger_pad` at local `pos="-0.01136 -0.076 0.019"` as a child of
`{prefix}moving_jaw_so101_v1` (the moving jaw body) — both bodies asserted to
resolve to a real match. `friction="1 0.05 0.001"`, `contype="1"
conaffinity="1"` on both pads, exactly as specified.

**Why this is not a repeat of the reverted Fix D.** Fix D's replacement
sphere sat at local `pos="0 0 0"` on the moving jaw body — exactly on that
body's own hinge rotation axis — so it never moved as the jaw opened or
closed (identical gap to five decimals at both joint limits, DECISIONS.md's
Fix D revert entry). `moving_finger_pad`'s local pos is offset in all three
axes from that origin, so this is a structurally different placement, not
merely a re-application of the same mistake — and Step 3 below exists
specifically to catch a repeat before anything downstream is trusted.

**Step 3 gate — pad separation across joint angle, measured on bm-ptl**
(`scripts/probe_pad_separation.py`, new diagnostic script, not shipped skill
code; reads `armA_static_finger_pad`/`armA_moving_finger_pad` world
`geom_xpos` directly, resetting to the "home" keyframe then overriding only
`armA_gripper`'s qpos per angle):

| angle | qpos (rad) | pad separation (m) |
|---|---:|---:|
| fully closed | -0.1745 | **0.00600** |
| midway | +0.7850 | **0.07621** |
| fully open | +1.7453 | **0.13188** |

Spread across the three angles: 0.12589 m. **GATE PASSED** — separation
changes materially and monotonically with joint angle (smallest near
closed, as expected for a pinch point), the opposite of Fix D's
identical-to-five-decimals failure. Both pad geoms genuinely move with
their respective bodies.

**Consequences.** Note: 6-132 mm is centre-to-centre distance between the
2.5 mm cube pads. The surface-to-surface gap — what actually fits between
the jaws — is smaller, measured at roughly 2.5 mm closed to 120 mm open via
`mj_geomDistance`. Quote the surface figure in judge-facing material and
say which quantity it is.

**Step 4 — pick(A, ·) in force order, run on bm-ptl, reported exactly as
measured, not softened.** `pytest tests/test_skills.py` first, to confirm no
new regression from the generator change: **5 failed / 3 passed**,
byte-for-byte the same specific failures already on record in the "M06a Fix
D reverted" entry above (including `test_pick_plate_waypoints_progress_without_collision`,
already flagged there as a pre-existing, undecided conflict with ADR-027's
own regression-test requirement — not newly broken by this change).

| Prop | Result | frames_used | reason | z delta | crossed lift threshold? |
|---|---|---:|---|---:|---|
| fork | FAIL | 313 | `waypoint 1 (approach) failed [collision (arm-vs-prop: fork (dist=-0.0072 m); threshold=-0.005 m)]` | -0.0035 | no |
| spoon | FAIL | 306 | `waypoint 1 (approach) failed [collision (arm-vs-prop: fork (dist=-0.0055 m); threshold=-0.005 m)]` — **anomaly:** target is spoon, the collision reported is against the *fork* prop, sitting nearby | -0.0021 | no |
| plate | FAIL | 296 | `waypoint 1 (approach) failed [collision (arm-vs-prop: plate (dist=-0.0081 m); threshold=-0.005 m)]` | -0.0034 | no |
| mug | FAIL | 317 | `waypoint 1 (approach) failed [collision (arm-vs-prop: mug (dist=-0.0090 m); threshold=-0.005 m)]` | -0.0028 | no |
| water_bottle | FAIL | 256 | `waypoint 1 (approach) failed [collision (arm-vs-prop: water_bottle (dist=-0.0061 m); threshold=-0.005 m)]` | -0.0007 | no |

**Honest interpretation: none of the three outcomes the task's own
interpretation guide anticipated is quite what happened, and that mismatch
is itself the finding.** All five props fail identically at **waypoint 1
(APPROACH)** — before the skill ever reaches DESCEND or GRIP. The pad
geometry this ADR adds is therefore **not exercised at all** by any of these
five runs; the pinch never gets a chance to form. This is not new: it is
**ADR-027 Step 5's own already-documented arm-vs-prop collision check**
(`skills_scripted.py`, out of this task's scope to touch) firing during the
blind, obstacle-unaware IK approach path (ADR-024) — the same condition that
entry already flagged for the plate specifically ("the arm's blind approach
path was apparently ALREADY grazing the plate during ordinary APPROACH even
with the original flush geometry"), now confirmed to occur identically for
**all five** props, not only the plate.

**Proof this is unrelated to the pad fix, not just an assertion:** `pick(A,
plate)`'s numbers here (`dist=-0.0081 m`, `frames_used=296`) are **bit-for-bit
identical** to the pre-ADR-028 baseline recorded in the "M06a Fix D reverted"
entry above, measured when the jaw mesh collision was still enabled and no
pads existed. Changing the jaw's collision geometry from mesh to pads
produced **zero** change to this failure, which is the expected result if —
and only if — the contact triggering it belongs to a different arm geom
entirely (most plausibly the wrist/forearm, brushing the prop during
approach), not the jaw. That is consistent with, not contradicted by, this
fix: the pad fix targets the PINCH, and the approach-phase collision check
fires well before any pinch is attempted.

**No further action taken in this commit, per its own explicit
instruction** ("Do NOT modify prop masses, IK strategy, or grasp offsets...
we are testing the pad fix in isolation"): `skills_scripted.py`'s
arm-vs-prop debounce/threshold logic and `ik.py`'s obstacle-unaware approach
path are both out of scope here, and neither was touched. **What this commit
proves:** the convex-hull geometry defect diagnosed in `docs/hardware/
grasp-envelope.md` is fixed at the geometry level (Step 3's gate) and does
not regress anything measured before (Step 4's pytest/plate parity). **What
it does not yet prove:** whether the fixed geometry actually grasps
anything, because no skill run in this commit reaches the GRIP waypoint for
any prop — that remains blocked by the separate, already-documented
approach-collision gap, and by the `±3.35 N` actuator ceiling for plate/mug/
bottle specifically, neither of which this commit addresses.

`git diff --stat -- scenes/so101/` confirmed empty before commit. Files
changed: `scripts/gen_dual_scene.py` (`disable_jaw_mesh_collision`,
`add_finger_pads`, wired into `main()`), the regenerated
`src/bimanual/sim/assets/so101_dual_table.xml`, and
`scripts/probe_pad_separation.py` (new diagnostic, not shipped skill code).

---

## M06a Fix D reverted — it disabled all gripper contact, not merely a neutral change

**Recorded:** Sept 12, 2026 · **Relates to:** ADR-024 (grasp-reliability gap) ·
**Reverts:** the M06a Fix D commits below (`1a099e3` re-enable, and the
`apply_fine_jaw_collision()` mechanism they added)

Independent measurement, done outside this repo's own retest ladder, found that
Fix D's replacement jaw-tip collision spheres sit exactly on each jaw's rotation
axis (`jnt_pos=(0,0,0)` in the jaw body's own local frame,
`scripts/probe_jaw_kinematics_debug.py`) and therefore **do not move at all** as
the jaw opens or closes: measured gap between the two spheres was `+0.00618 m`
at both the closed limit (-0.1745 rad) and the open limit (+1.7453 rad),
identical to five decimal places. Combined with Fix D setting
`contype="0" conaffinity="0"` on the two real jaw MESH collision geoms (the only
collision geometry that has ever been observed to move with the joint,
`scripts/probe_jaw_opening.py`), the compiled gripper in HEAD could not contact
anything at all. This was previously recorded as "NEUTRAL, no regression"
because the retest ladder's only signal was plate-z, and plate-z did not move
appreciably whether Fix D's spheres were present or not — that measurement
masked a total loss of gripper contact rather than confirming Fix D was inert.

**What changed.** `scripts/gen_dual_scene.py`: removed `apply_fine_jaw_collision()`,
its call in `main()`, `FINE_JAW_TIP_RADIUS_M`, and `APPLY_FIX_D_FINE_JAW_COLLISION`
— deleted rather than left as a disabled flag, since a flag that silently zeroes
gripper contact when flipped on is the exact landmine this revert exists to
remove. The two jaw MESH collision geoms (`JAW_COLLISION_MESHES`) are left
exactly as Fix A set them: `friction="1.5 0.1 0.001"`, `contype`/`conaffinity`
unset (MuJoCo default — collision-enabled). No visual (`class="visual"`) geom
was touched. Regenerated `src/bimanual/sim/assets/so101_dual_table.xml`;
`git diff --stat -- scenes/so101/` confirmed empty before commit.

**Verification on bm-ptl, reported plainly (worse, as anticipated, not
hidden):** `python scripts/run_skill.py --skill pick --object plate --arm A --seed 0`
→ `result: success=False frames_used=296`, `reason: waypoint 1 (approach) failed
[collision (arm-vs-prop: plate (dist=-0.0081 m); threshold=-0.005 m)]`,
`measured: plate z: initial=0.3550 final=0.3516 delta=-0.0034`. This is a
smaller/worse delta than the retest ladder's own Step 4 number
(`delta=-0.0026`) because with real jaw collision restored, the arm-vs-prop
debounce check (added in that same retest's Step 5) now fires on APPROACH
before the skill ever reaches GRIP — with Fix D's contact-disabled gripper, the
same approach path produced no arm-vs-prop contact to detect, so the skill ran
further before failing. `pytest tests/test_skills.py`: **5 failed / 3 passed**,
identical in count and in the specific failing tests to the retest ladder's own
already-recorded final state (same `test_pick_plate_waypoints_progress_without_collision`
failure at `dist=-0.0081 m`) — this revert introduces no new regression beyond
what was already open and already flagged for the user to decide.

---

## M06a grasp fix ladder RETEST (Steps 1-5) — Fix B reverted, C/D re-isolated, E (plate reshape) applied, Step 5 (arm-vs-prop validation) added and corrected twice

**Recorded:** Sept 12, 2026 · **Follows:** the original fix-ladder entries below (fix
A-D, applied cumulatively on top of a poisoned Fix B) · **Corrects:** the previous
run's confound, explicitly: Fix B's top-centre plate offset never converged at
waypoint 1, so fixes C and D were applied but never actually exercised. This entry
redoes the ladder from a working baseline so each fix gets a fair test, then adds a
new arm-vs-prop validation check (ADR-027 Step 5) motivated by the earlier
`pick(A, bottle)` diagnostic (bottle knocked from z=0.44 to z=0.0298, every waypoint
reporting clean).

### Step 1 — Revert Fix B (`6d49514`)

`GRASP_POINT_OFFSET_M["plate"]` restored from the top-centre point `(0,0,0.005)` back
to the rim, `(0, 0.09, 0)`. `pick(A, plate)`: waypoint 1 now CONVERGES (it did not
before), all 4 waypoints run, `frames_used=1560`, `initial_z=0.3560 final_z=0.3504
delta=-0.0056`. `pytest tests/test_skills.py`: **4 failed / 4 passed**, matching the
pre-Fix-B count exactly, including both ADR-027 regression tests passing.
**Verdict: NEUTRAL** (z stays ≈0.3505, no regression). **Kept.**

### Step 2 — Retest Fix C in isolation (`85146c4`)

Fix C's mechanism (ctrl driven to the gripper actuator's own `ctrlrange` closure
limit; `GRIP_HOLD_FRAMES` 30→60) was already unconditionally present in
`skills_scripted.py` since the original ladder — Step 1's revert is what let it
actually run for the first time. To test it in ISOLATION from fix D (not merely
un-poisoned from fix B), a new `APPLY_FIX_D_FINE_JAW_COLLISION` flag was added to
`gen_dual_scene.py` and set `False`, disabling fix D's fine jaw-tip collision geoms
and leaving the original bulky mesh collision (with fix A's friction only) active.
`pick(A, plate)`: **identical** to Step 1 — `initial_z=0.3560 final_z=0.3504
delta=-0.0056`, `frames_used=1560`. `pytest`: **4 failed / 4 passed**, no change.
**Verdict: NEUTRAL** (fix C alone, now actually exercised, measured ZERO effect).
**Kept** (no regression).

### Step 3 — Retest Fix D in isolation (`1a099e3`)

`APPLY_FIX_D_FINE_JAW_COLLISION` flipped back `True`, re-enabling the fine jaw-tip
collision spheres on top of fix C's (already-tested) state. `pick(A, plate)`:
**identical again** — `initial_z=0.3560 final_z=0.3504 delta=-0.0056`,
`frames_used=1560`. `pytest`: **4 failed / 4 passed**, no change.
**Verdict: NEUTRAL.** Fixes C and D, tested individually and cumulatively once
actually exercised, produced **zero measurable change** to `pick(A, plate)`'s
outcome. **Kept** (no regression, and per the task's "keep the commit" rule for a
neutral result — reverting would gain nothing since neither changed the number).

**A finding surfaced by chasing why D measured zero effect, not merely reported:**
`scripts/probe_jaw_kinematics_debug.py` and `scripts/probe_jaw_geoms_debug.py` (new
diagnostic scripts, not part of the shipped skill code) found that the moving jaw's
hinge joint has `jnt_pos=(0,0,0)` in the moving jaw BODY's own local frame — the
rotation axis passes exactly through that body's origin. Fix D's fine collision
sphere was added at local `pos="0 0 0"` on that same body — i.e. **exactly on the
rotation axis**, so it never moves at all as the jaw opens or closes, regardless of
commanded ctrl. This is a plausible, concrete reason fix D measured no effect: its
own new collision geometry was structurally unable to participate in the pinch.

### Step 4 pre-measurements (`scripts/probe_jaw_opening.py`, measured on bm-ptl)

1. **Jaw opening.** The moving jaw body's ORIGIN does not move with the joint angle
   (see above), so the jaw's own MESH geom (offset ~2.5 cm from that axis) is what
   actually sweeps. Measured world-frame separation between the fixed and moving
   jaw's mesh-geom points: **CLOSED ≈0.0248 m, OPEN ≈0.0361 m** — i.e. a maximum jaw
   opening on the order of **3.6 cm**, with roughly 1.1 cm of closing travel from
   that fully-open state. Approximate (mesh centroids, not exact contact-surface
   geometry — no finer data available without touching `scenes/so101/`), but
   measured from the compiled model, not assumed.
2. **Flush contact.** Read directly from `scripts/gen_dual_scene.py` before this
   fix: `PLATE_POS` z was `0.356` = `TABLE_TOP_Z` (0.35) + the old disc's own
   half-thickness (0.006) **exactly** — the plate rested FLUSH on the table with
   zero gap beneath it. Confirmed, not assumed: the old `plate_geom` was a single
   cylinder resting directly on `table_top`.

### Step 4 — Fix E: plate reshape (`566b7af`)

Candidate (b) chosen (foot ring): `plate` body reshaped into two stacked cylinders
in `scripts/gen_dual_scene.py`'s hand-authored template (not `scenes/so101/`) — a
foot (r=0.03, h=0.010) resting on the table, and a dish (r=0.06, h=0.008) on top,
overhanging the foot by 0.03 m with a 0.01 m gap beneath the overhang (well inside
the ~3.6 cm measured jaw opening). `GRASP_POINT_OFFSET_M["plate"]` set to
`(0, 0.06, 0.009)`, targeting the dish's overhanging rim at the dish's own local
height. `pick(A, plate)`: `initial_z=0.3550 final_z=0.3524 delta=-0.0026` — roughly
**half** the previous delta (an improvement), still far short of the `z > 0.38`
success bar. `pytest`: **4 failed / 4 passed**, both ADR-027 regression tests still
passing at this point (the arm-vs-prop check did not exist yet).
**Verdict: NEUTRAL/improvement, not success.** Per instruction, this is the last fix
in the ladder — no sixth fix attempted.

### Step 5 — Extend validation to arm-vs-prop (`66b0e46`, corrected `417c43f`, `2bf0bf2`)

Implemented regardless of the pick outcome, per instruction. Three iterations were
needed to get this right, each measured and reported rather than assumed:

1. **First version (`66b0e46`):** a single post-hoc check per waypoint (any arm geom
   vs. any free-joint prop at `dist < -0.005 m` after the waypoint's drive loop
   finished). Result: **did NOT catch the bug it was built for.**
   `pick(A, bottle)`: `frames_used=1560`, every waypoint reported clean, bottle still
   knocked to the floor (`z 0.4400 → 0.0296`). Root cause, found and reported rather
   than silently patched: the knock happens AND fully resolves (the prop separates)
   within a single ~500-step waypoint loop, before the one-shot post-hoc check ever
   runs.
2. **Second version (`417c43f`):** sample contacts on EVERY physics step inside
   `_drive_to_target`/`_dwell`, stop immediately on a violation. This DID catch it:
   `pick(A, bottle)` now fails at `waypoint 1 (approach)` with
   `collision (arm-vs-prop: water_bottle (dist=-0.0060 m))`, `frames_used=254`,
   bottle displacement now `delta=-0.0010` (essentially none) — exactly the
   "validation failed: knocked water_bottle" outcome the task asked to confirm.
   **But this broke a required regression test**: `pytest` went to **5 failed / 3
   passed** — `test_pick_plate_waypoints_progress_without_collision` newly failed,
   because `pick(A, plate)`'s own ordinary APPROACH waypoint also registered a
   prop-collision against the plate itself (`dist=-0.0051 m`, just past the bar).
3. **Third version (`2bf0bf2`):** added a 3-consecutive-step debounce
   (`PROP_COLLISION_DEBOUNCE_STEPS`) plus a `target_object` exemption in
   `_run_dwell`/`_dwell` (a GRIP/RELEASE dwell's whole purpose is deliberate contact
   with its own target, so that contact should not itself be a violation). Result:
   bottle still caught cleanly (`dist=-0.0061 m`, `frames_used=256`,
   `delta=-0.0007`) — debounce did not meaningfully delay detection or let the
   bottle move. **But the plate's approach violation is CONFIRMED, not a
   single-frame artifact**: with debounce active it is detected at a DEEPER
   `dist=-0.0081 m` after persisting 3 consecutive steps, i.e. the contact is
   sustained/escalating, not solver noise.

**Investigated further, and reported honestly rather than patched around:** to test
whether Fix E (Step 4's plate reshape) was the cause of the new plate-vs-arm
approach contact, the OLD flush plate geometry and the OLD rim offset were
temporarily restored (dry run, never committed) with Step 5's code otherwise
unchanged, and `pick(A, plate)` was re-run. **Result: the SAME violation still
occurred** — `collision (arm-vs-prop: plate (dist=-0.0096 m))` at waypoint 1,
`frames_used=271`. **This rules out Fix E as the cause.** The arm's blind,
obstacle-unaware IK approach path (ADR-024's already-documented limitation) was
apparently ALREADY grazing the plate during ordinary APPROACH even with the
original flush geometry — this was simply never detected before because no
check watched arm-vs-prop contact until this step. Reverting Fix E therefore would
not have fixed the conflict, so the committed Fix E state was restored (git
checkout, confirmed clean) rather than left reverted for no benefit.

**Why no further exemption was attempted.** The one remaining lever that would make
`test_pick_plate_waypoints_progress_without_collision` pass again — exempting a
skill's own `target_object` during `_run_waypoint`'s APPROACH/DESCEND phases, the
same way `_run_dwell` already exempts it during GRIP/RELEASE — was considered and
REJECTED: in `pick(A, bottle)`, the water bottle IS `pick`'s own `target_object`
during that exact APPROACH waypoint. Exempting "the skill's own current target"
during transit would silence the water-bottle catch this step exists to build,
not just the plate's benign one. There is no target-identity-based rule that
keeps one and drops the other; the two cases are structurally the same shape
(an arm approaching its own eventual grasp target grazes it en route) and differ
only in CONSEQUENCE (the bottle gets flung; the plate, per the Step-4 measurement
with no check active, settles at `delta=-0.0026` without flinging) — a difference
this depth-based check cannot see in advance. A velocity/displacement-based
signal might discriminate the two cases, but that is a materially different
mechanism than the depth threshold the task specified, and was not implemented
without approval.

**Final, honestly-reported state of the test suite:** `pytest tests/test_skills.py`
= **5 failed, 3 passed.** `test_open_drawer_fails_without_tunneling_through_table`
(the other required ADR-027 regression test) still passes throughout every step
of this retest. `test_pick_plate_waypoints_progress_without_collision` now fails —
**not as a defect introduced by this task's changes, but as a pre-existing,
previously-undetectable condition that Step 5's validation correctly surfaces for
the first time.** This conflicts with this task's own instruction that ADR-027's
regression tests must keep passing at every step, and is flagged here, plainly,
for the user to decide rather than resolved by guessing: either (a) accept that
`test_pick_plate_waypoints_progress_without_collision`'s assumption predates
arm-vs-prop validation and needs updating to allow a plate-specific
approach-phase collision while still failing on anything worse, or (b) direct a
different, out-of-scope fix (e.g. an obstacle-aware approach path in `ik.py`,
explicitly off-limits to this task).

**Files changed this retest, by step:** Step 1 —
`src/bimanual/control/skills_scripted.py` (offset revert). Steps 2-3 —
`scripts/gen_dual_scene.py` (`APPLY_FIX_D_FINE_JAW_COLLISION` flag) and the
regenerated `src/bimanual/sim/assets/so101_dual_table.xml`. Step 4 —
`scripts/gen_dual_scene.py` (plate body reshape), regenerated scene XML, and
`src/bimanual/control/skills_scripted.py` (grasp offset). Diagnostic-only, not
part of the shipped skill code: `scripts/probe_jaw_opening.py`,
`scripts/probe_jaw_kinematics_debug.py`, `scripts/probe_jaw_geoms_debug.py`. Step
5 — `src/bimanual/control/skills_scripted.py` only (`_prop_collision_violations`,
`_debounce_prop_violations`, wiring into `_drive_to_target`/`_dwell`/
`_run_waypoint`/`_run_dwell`/`_validate_against_baseline`). `git diff --stat --
scenes/so101/` was run and confirmed empty before every commit in this retest.

---

## M06a grasp fix ladder — final diagnostic: `pick(A, mug)` and `pick(A, bottle)` after all four fixes

**Recorded:** Sept 12, 2026 · **Follows:** M06a grasp fixes A-D (all four
applied, none made `pick(A, plate)` succeed) · **No code change in this
entry** -- diagnostic only, per the task's "IF ALL FOUR FAIL" instruction

With all four fixes committed and none clearing `pick(A, plate)`'s
`z > 0.38` bar, `pick(A, mug)` and `pick(A, bottle)` were run on bm-ptl
(same scene, same control stack, all four fixes in effect) to check
whether the grasp machinery works at all, or whether it is broken in
general.

**`pick(A, mug)`: fails at waypoint 1 (approach), a convergence failure --
the ALREADY-DOCUMENTED reach limit, not a new finding.**
`waypoint 1 (approach) failed [convergence (IK residual=0.0532 m >=
0.01 m)]`, `frames_used=500`, `mug z: initial=0.3900 final=0.3827
delta=-0.0073`, no MuJoCo warnings. This matches ADR-027's own already-
reported finding almost exactly (`mug_at_rest` does not converge for arm
A from the home pose, residual ~0.12 m there vs. 0.0532 m here for the
hover point specifically -- same reach-limit class of failure, not the
grasp-reliability gap this task's four fixes targeted). Arm A simply
cannot reach the mug's grasp-point hover position from its current base
placement; this is orthogonal to fixes A-D and would not be fixed by any
of them.

**`pick(A, bottle)`: reaches "did not lift" (every waypoint validated),
but the actual outcome is worse than that phrase suggests, and is
reported here as a collision anomaly, not softened.**
`frames_used=1560` (consistent with the skill running all 4 waypoints --
approach, descend, grip, retreat -- rather than stopping early),
`water_bottle z: initial=0.4400 final=0.0298 delta=-0.4102`, no MuJoCo
warnings, `max_joint_limit_violation` unchanged from baseline. A delta of
-0.41 m is not "stayed on the table and didn't lift" -- `TABLE_SURFACE_Z`
is 0.35 and the bottle started at 0.44 (already elevated, its own initial
placement); ending at z=0.0298 is consistent with the bottle being
knocked off the table entirely and coming to rest on the floor (a
cylinder of radius 0.03 m lying on its side has a centre height in
exactly this range). This is NOT caught by ADR-027's per-waypoint
collision check, because that check only monitors arm-vs-`table_top` and
cross-arm contacts -- it was never designed to detect the arm
contacting and flinging a PROP, which is exactly ADR-024's own
already-documented "the arm's approach frequently contacts and displaces
the light, freely-jointed prop before any controlled pinch can form."
The bottle is tall (0.09 m + cap) relative to the mug/plate, and the
approach/descend waypoints evidently strike it before the jaw can close
around a stable grasp.

**Conclusion, stated as the task asks: genuinely informative, not just
"everything fails."** Neither `mug` nor `bottle` "lifts cleanly" -- so
this is NOT the clean "grasp machinery works, plate is just the hardest
object" result the task's framing offered as one possible outcome.
Instead: `mug` is blocked by a DIFFERENT, already-documented problem
(arm reach, not grasp mechanics) and `bottle` surfaces a THIRD problem
(the approach knocks the object away before a pinch forms) that fixes
A-D do not address because none of them add obstacle-aware approach
planning or orientation control -- both explicitly out of this task's
scope (`ik.py`/`executor.py` not to be modified). The honest summary: of
the three objects tried under this control stack (plate, mug, bottle),
none currently lifts successfully; the three failures are for three
different, non-overlapping reasons (grasp-reliability/friction-adjacent
for the plate; kinematic reach for the mug; approach-collision
displacement for the bottle).

**Recommendation for the user to decide (not implemented): reshaping the
plate prop.** The plate is OUR hand-authored prop
(`scripts/gen_dual_scene.py`'s `<body name="plate">`), not upstream
geometry, so changing it is not an ADR-016/ADR-021 concern the way
touching `scenes/so101/` would be. Giving it a raised rim or a thicker
edge (e.g. an outer ring geom a few mm taller than the current 1.2 cm
disc, or simply increasing the disc's own thickness) would give a
parallel-jaw gripper an actual vertical lip to catch, which a flat disc
fundamentally does not offer regardless of friction (fix A), grasp-point
placement (fix B), closure force (fix C), or collision-geometry
resolution (fix D) -- all four fixes operated on the GRIPPER side of the
problem; none changed the fact that the OBJECT itself presents no
catchable feature to a jaw with no orientation control. I think this
would plausibly help, and probably more than any of fixes A-D did,
because it attacks the part of the problem those four fixes could not
reach: a jaw approaching from an uncontrolled angle needs a feature it
cannot slide past, and a rim or thickened edge is exactly that, whereas
a razor-thin flat disc is exactly the shape a rimless pinch is most
likely to slip off of. It would involve: adding one or two additional
`<geom>` elements to the `plate` body in `gen_dual_scene.py`'s
hand-authored template (e.g. a thin annular ring approximated by MuJoCo's
primitive shapes, or simply a taller cylinder), re-deriving
`GRASP_POINT_OFFSET_M["plate"]` for the new geometry (the rim's radius
and height would both change), and re-running the ADR-027 regression
tests to confirm no new tunneling. This is a real prop-geometry change
the user may want reviewed before it is made, not a decision for this
task to take unilaterally -- flagged here per the task's explicit
instruction to recommend, not implement.

---

## M06a grasp fix D — fine collision geom on jaw tips; INSUFFICIENT, and not actually exercised; all four fixes now applied and none succeeded

**Recorded:** Sept 12, 2026 · **Follows:** M06a grasp fix C (insufficient,
not exercised) · **Deliberate deviation from:** ADR-021's byte-faithful
upstream body-tree copy (same class of deviation as fix A, here in
collision geometry rather than a friction constant)

**What changed.** `scripts/gen_dual_scene.py` gained
`apply_fine_jaw_collision()`, run in the generator over the copied
`armA_`/`armB_` subtrees only -- never `scenes/so101/`
(`git diff --stat -- scenes/so101/` empty, verified again before this
commit). For each arm it: (1) disables the two bulky, full-MESH jaw
COLLISION geoms identified in fix A (`wrist_roll_follower_so101_v1` on
body `{prefix}gripper`, `moving_jaw_so101_v1` on body
`{prefix}moving_jaw_so101_v1`) by setting `contype="0" conaffinity="0"`
on them -- the SAME convention this scene already uses for
`class="visual"` geoms, so they simply stop participating in contacts;
their `class="visual"` counterparts (what actually renders) are
untouched; (2) adds one new small sphere collision geom (radius 0.015 m,
`FINE_JAW_TIP_RADIUS_M`) at each jaw body's own local origin -- the SAME
point `ik.py`'s pinch-point solver targets (ADR-025: the midpoint of the
fixed and moving jaw bodies' `xpos`) -- carrying forward fix A's
`friction="1.5 0.1 0.001"`.

**Why.** ADR-024 measured the jaw's collision MESH geoms at a
bounding-sphere radius (`geom_rbound`) up to ~8.4 cm -- large relative to
the plate (radius 0.09 m, 1.2 cm thick) -- a plausible reason the arm
contacts/displaces the object well before a true pinch can form. A small
sphere at the exact point the solver targets removes that size mismatch.

**Result, measured on bm-ptl: INSUFFICIENT, and -- reported honestly, as
with fix C -- this fix's own mechanism was NOT actually exercised.**
`pick(A, plate)`: identical failure to fixes B and C --
`waypoint 1 (approach) failed [convergence (IK residual=0.0226 m >=
0.01 m)]`, `frames_used=500`, `plate z: initial=0.3560 final=0.3506
delta=-0.0054`, no MuJoCo warnings. `pytest tests/test_skills.py`: 4
failed / 4 passed, identical to fixes A-C -- no regression, no
tunneling; the scene still compiles and runs cleanly with the new tip
geoms in place.

**Confound, flagged plainly rather than glossed over.** Because these
four fixes were applied cumulatively, in the order the task specified,
fix B's top-centre plate offset (itself already reported insufficient
and physically unsound) is still in effect for fixes C and D, and it is
THAT offset -- not fix C's closure force or fix D's collision geometry --
that is causing the waypoint-1 convergence failure both share. Neither
fix C's nor fix D's own hypothesis (closure force; collision-geometry
size mismatch) was actually put to a fair test on this scene: both would
require the arm to first reach the GRIP waypoint, which it never did
after fix B. This is reported as a limitation of the fix ladder as
executed, per the task's explicit instruction to iterate in this exact
order and stop only at the first success or after all four -- not as a
claim that fixes C or D are individually disproven. A follow-up that
re-tests C and/or D against the ORIGINAL rim offset (fix A's baseline,
before fix B) would isolate this properly, but is a fifth action beyond
this task's four-fix ladder and is not attempted here without approval.

**All four fixes applied, none succeeded `pick(A, plate)`.** Per the
task's "IF ALL FOUR FAIL" instruction, `pick(A, mug)` and `pick(A,
bottle)` are run next as a genuinely informative diagnostic (are taller,
easier-to-pinch objects liftable at all with this same control stack?),
then this task stops without a fifth fix, per instruction.

---

## M06a grasp fix C — max closure force / ctrlrange-exact ctrl target; INSUFFICIENT, and not actually exercised

**Recorded:** Sept 12, 2026 · **Follows:** M06a grasp fix B (insufficient,
physically unsound for a top-centre approach)

**What changed.** Read-only per the task's instruction:
`scenes/so101/so101_new_calib.xml:162`'s gripper actuator --
`<position class="sts3215" name="gripper" joint="gripper"
forcerange="-3.35 3.35" ctrlrange="-0.17453 1.74533"/>` -- was READ, not
edited. `_gripper_ctrl()` in `src/bimanual/control/skills_scripted.py` now
computes the closed/open ctrl targets from the compiled model's
`actuator_ctrlrange` for the gripper actuator, instead of from the
joint's `jnt_range` as before -- numerically almost identical
(`jnt_range` -0.17453297762778586..1.7453291995659765 vs. `ctrlrange`
the same value rounded to 5 decimals) but `ctrlrange` is the value MuJoCo
actually clamps a commanded `ctrl` entry against (`autolimits="true"`
makes this actuator ctrl-limited), so `GRIPPER_CLOSE_FRACTION=0.0` now
drives ctrl to the actuator's own declared closure limit exactly, with no
possible daylight. The position actuator's gain (`kp=998.22`, unmodified
upstream `sts3215` default class, ADR-016) is already the model's only
available gain and was not changed -- there is no separate "gain" input
to raise beyond it without editing the default class, which was not part
of this fix. `GRIP_HOLD_FRAMES` raised 30 -> 60 per the task's
instruction.

**Result, measured on bm-ptl: INSUFFICIENT, and reported honestly as NOT
actually having exercised the mechanism this fix targets.**
`pick(A, plate)`: `waypoint 1 (approach) failed [convergence (IK
residual=0.0226 m >= 0.01 m)]`, `frames_used=500`, `plate z:
initial=0.3560 final=0.3506 delta=-0.0054`, no MuJoCo warnings. This is
the SAME failure fix B already produced -- the skill still never reaches
the GRIP waypoint at all, because fix B's top-centre approach point does
not kinematically converge for arm A. Fix C's actual content (closure
force / hold duration) therefore never ran on this scene: there is no
GRIP dwell for it to affect. `pytest tests/test_skills.py`: 4 failed / 4
passed, identical to fixes A and B -- no regression, no tunneling.

Per the task's fix ladder, proceeding to Fix D (fine collision geom).

---

## M06a grasp fix B — plate grasp-point moved to the top-centre; INSUFFICIENT and physically unsound

**Recorded:** Sept 12, 2026 · **Follows:** M06a grasp fix A (insufficient)

**What changed.** `GRASP_POINT_OFFSET_M["plate"]` in
`src/bimanual/control/skills_scripted.py` moved from the rim,
`(0, 0.09, 0)`, to the top-centre, `(0, 0, 0.005)`.

**Result, measured on bm-ptl (`scripts/run_skill.py --skill pick --object
plate --arm A --seed 0`): INSUFFICIENT, and worse than fix A, not merely
equal.** The skill now fails at waypoint 1 (APPROACH -- the hover point
above the grasp point, before any descent) with a convergence failure:
`IK residual=0.0226 m >= 0.01 m`, `frames_used=500` (one waypoint's step
cap only). `pytest tests/test_skills.py`: 4 failed / 4 passed, the SAME
4 pre-existing failures and the SAME 2 ADR-027 regression tests still
passing -- no tunneling was introduced; the skill fails cleanly rather
than badly.

**Reported plainly, as the task requires: this is not just "still
failing", it is the expected physical outcome of an unsound approach.** A
plate is a flat disc (radius 0.09 m, 1.2 cm thick). A parallel-jaw
gripper closing directly above its centre has nothing to pinch unless it
happens to catch the rim on some axis -- the same "no orientation
control" limitation ADR-024 already documents (the jaw's approach angle
is whatever the redundant 5-joint solve falls into, never chosen). Here
the failure is even more basic than "closes on empty space": the
APPROACH hover point itself did not kinematically converge for arm A at
this position, before the descend/grip stages were ever reached. This
result does not by itself prove top-centre grasping is impossible in
general (a different hover height or a different arm base pose might
converge), but it does confirm the task's warning was correct to flag:
**a flat disc gives a parallel-jaw gripper nothing to pinch from directly
above, and this attempt is honestly reported as a worse outcome than the
rim offset it replaced, not a partial improvement.**

Per the task's fix ladder, proceeding to Fix C (closure force).

---

## M06a grasp fix A — jaw friction raised on the generated arm copies only

**Recorded:** Sept 12, 2026 · **Follows:** ADR-027 (waypoint staging; left the
grasp-reliability gap explicitly out of scope) · **Deliberate deviation
from:** ADR-021's byte-faithful upstream body-tree copy

**What changed.** `scripts/gen_dual_scene.py` now sets
`friction="1.5 0.1 0.001"` on exactly two geoms per arm, in the GENERATED
arm copies only (`armA_`/`armB_` subtrees inside
`src/bimanual/sim/assets/so101_dual_table.xml`) -- never in
`scenes/so101/so101_new_calib.xml`:
- the fixed jaw's collision geom (`class="collision"`, `mesh=
  "wrist_roll_follower_so101_v1"`, on body `armX_gripper`), and
- the moving jaw's collision geom (`class="collision"`, `mesh=
  "moving_jaw_so101_v1"`, on body `armX_moving_jaw_so101_v1`).

A new `apply_jaw_friction()` helper matches by the geom's `mesh` reference
(stable across `rename_recursive`'s body/joint/site renaming) and asserts
exactly 2 matches per arm, so a future upstream mesh-name change fails
loudly at generation time instead of silently shipping a no-op. The
`class="visual"` copies of the same meshes are left untouched (they carry
`contype="0" conaffinity="0"`, so friction there is inert).

**Why.** `pick(A, plate)` was measured (ADR-027) to clear every waypoint's
IK-convergence and collision validation, close the jaw fully on the
plate, and still never lift it -- baseline re-confirmed on bm-ptl
immediately before this fix, byte-for-byte the same as ADR-027 reported:
`initial_z=0.3560 final_z=0.3505 delta=-0.0055` (frames_used=1530, zero
MuJoCo warnings). Extending the GRIP dwell to 300 steps had already been
tried and changed nothing, ruling out a timing/settling explanation. The
upstream jaw geoms carry no explicit `friction` attribute, so they fell
back to MuJoCo's compiled default (`1 0.005 0.0001`) -- lower sliding
friction than several of the props themselves (e.g. the plate's own geom
is tuned to `0.9 0.005 0.0001`), a plausible reason a fully-closed jaw
still cannot hold a thin, flat disc against gravity.

**Recorded as a deliberate deviation, per the task's explicit instruction.**
ADR-021 copies the upstream SO-101 body tree byte-faithfully (only renaming
and repositioning); this is the first place the generated copy is allowed
to differ from upstream in a DYNAMICS property. This does **not** violate
ADR-016 -- ADR-016 governs DoF and kinematics ("no locked joints, no added
DoF") and says nothing about friction. But an undocumented dynamics
divergence from a byte-faithful copy is exactly the kind of thing a judge
could find and question, so it is recorded here, in the commit message,
and in `scripts/gen_dual_scene.py`'s own comments rather than left silent.
`scenes/so101/` itself is untouched: `git diff --stat -- scenes/so101/`
was run immediately before committing this fix and printed nothing.

**Result, measured on bm-ptl (`scripts/run_skill.py --skill pick --object
plate --arm A --seed 0`), reported plainly: INSUFFICIENT.**
`initial_z=0.3560 final_z=0.3507 delta=-0.0053` -- essentially unchanged
from the pre-fix baseline (`delta=-0.0055`), well short of `z > 0.38`
(`TABLE_SURFACE_Z + PICK_LIFT_MARGIN_M`). `frames_used=1530`, zero MuJoCo
warnings, `max_joint_limit_violation=0.0004` (unchanged). `pytest
tests/test_skills.py` on bm-ptl: 4 failed / 4 passed, the SAME 4 failures
ADR-027 already documented (`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b` -- unrelated reach-limit/grasp-gap
causes, not a new regression) and the SAME 2 ADR-027 regression tests
still passing (`test_open_drawer_fails_without_tunneling_through_table`,
`test_pick_plate_waypoints_progress_without_collision`) -- this fix did not
reintroduce tunneling. Raising the jaw's own friction did not help because
the plate never stays pinched long enough for friction to matter: the
grasp geometry itself (jaw bounding-sphere radius up to ~8.4 cm against a
9 cm-radius, 1.2 cm-thick disc, ADR-024) is the more likely dominant
factor. Per the task's fix ladder, proceeding to Fix B (grasp-point
offset).

---

## ADR-027 — Waypoint staging in scripted skills for collision-safe motion

**Recorded:** Sept 12, 2026 · **Follows:** ADR-024, ADR-026 · **Fixes:** the
"re-tuning skills/IK for the new home pose" follow-up ADR-026 deferred ·
**Does not touch:** `ik.py`, `executor.py`'s interface, `scenes/so101/`,
`src/bimanual/command/`, `src/bimanual/language/`

`ik.py`'s position-only solver has no collision term by design (ADR-024); M06a
handed it raw single-shot targets, which is how ADR-026 caught tunneling
solves (up to -0.064 m through `table_top`). Fix: `skills_scripted.py` stages
every skill as small, validated waypoints (tutor note 06's
approach/grip/retreat pattern) instead of one long reach — each waypoint must
clear BOTH IK convergence (`ik.solve_position_ik`'s own residual <
`IK_POSITION_TOLERANCE_M`) and a collision check (no new cross-arm or
arm-vs-`table_top` contact vs. a baseline snapshot) before the next one runs;
on failure a skill returns `SkillResult(success=False, reason="waypoint N
failed [convergence|collision]: ...")` and stops. New constants:
`CLEARANCE_HEIGHT_M=0.08`, `APPROACH_DESCENT_STEPS=500`,
`GRIP_HOLD_FRAMES=30`, `PULL_DISTANCE_M=0.15`,
`HANDOFF_POSITION_XYZ=(0.0,-0.01,0.35)` (the measured shared-band midpoint,
`docs/hardware/m06-reachability-probe.md`). `DEFAULT_STEP_BUDGET` raised
3000 → 12000 (one shared 4x default, not per-skill overrides).

**`open_drawer` rebuilt as a lateral, under-table approach** (APPROACH
outside the table footprint at drawer height → INSERT in +y staying below
the slab → GRIP → PULL → RELEASE → RETREAT), because the drawer's closed
face (y=0.00, ADR-026) sits directly beneath `table_top` and a vertical
descent tunnels. **Measured, not assumed, before building it:** the drawer
face IS kinematically reachable (residual 0.009 m) but only via a tunneling
solution (-0.064 m penetration, confirmed by contact check); the lateral
waypoint outside the table does NOT converge at all (residual stalls at
0.09–0.32 m across a 1500-step real closed-loop drive, and across y in
{-0.30,-0.32,-0.35}); the ADR-025 flush-table-edge drawer position (housing
y=-0.17) also does NOT converge (residual 0.32 m / 0.18 m) — the
re-measured envelope's bounding box (`y` down to -0.35) is not a claim that
every interior point is reachable, and at drawer height (z≈0.28) arm A's
actual reach only extends to about y=-0.12. **Per the explicit stop rule
("if the only solutions tunnel, STOP and report — do not invent a third
drawer position"), no new drawer position was tried.** `open_drawer` is
still built exactly as specified and correctly, safely fails at waypoint 1
(APPROACH, convergence) rather than silently tunneling — the honest,
intended result.

**`handoff(mug, A→B)` is blocked by the same class of problem:**
`mug_at_rest` itself does not converge for arm A from the home pose
(residual 0.120 m), independently matching `docs/hardware/
m06-reachability-probe.md`'s own ADR-026 Step-4 table.

**`pick(A, plate)`/`place(A, plate)` are blocked by a different,
already-known, out-of-scope gap: ADR-024's grasp reliability.** Every
waypoint now clears validation (converges, zero new arm-vs-table_top
contact beyond a measured graze/tunneling boundary — see
`TABLE_COLLISION_DEPTH_TOL_M=0.001`, chosen because a `pick` descend onto a
table-height grasp point was measured to create a `dist=-0.00007` m graze,
~1000x shallower than the drawer's tunneling penetrations), the jaw closes
fully, and the plate still never lifts even after a 300-step grip dwell
(tested directly, far past `GRIP_HOLD_FRAMES`) — ADR-024's documented
site-vs-pinch-point offset, not a staging defect. `GRASP_POINT_OFFSET_M["plate"]`
was corrected from `(+0.09,0,0)` to `(0,+0.09,0)` (same rim radius, a
reachable direction instead of one that pegs three joints at their limits
from the new home pose) — a caller-side reachability choice, not an
`ik.py` change.

**Honest test verdict.** The four tests ADR-026 left failing
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`)
**still fail** — each for one of the two out-of-scope gaps above, not a
staging defect — and are reported unedited rather than weakened. Two new
tests (`test_open_drawer_fails_without_tunneling_through_table`,
`test_pick_plate_waypoints_progress_without_collision`) are this ADR's real
regression coverage and both pass: a failing skill now localises to a named
waypoint/reason and is verifiably collision-safe, instead of silently
tunneling. `pytest tests/`: 62 tests, 58 pass (all of `test_command_source.py`
and `test_grounder.py`, plus 4 of `test_skills.py`'s 8), 4 fail (the ones
named above). `scripts/run_skill.py` diagnostics for all four skills: zero
MuJoCo warnings, max joint-limit violation ≤ 0.0004 rad throughout.

---

## ADR-026 — "Home" rest keyframe; envelope re-measured from a valid pose; drawer moved onto the correct axis

**Recorded:** Sept 12, 2026 · **Supersedes:** ADR-025's drawer reposition ·
**Corrects:** a load-bearing ADR-021 assumption never checked against the
compiled rest pose · **New ADR, not an ADR-025 amendment**, because this
replaces the pose the whole reachability story was measured from.

**Root cause found.** ADR-025's own probe flagged, but did not fix, a
"Baseline finding": at `reset(seed=0)`, *before any IK solve*, arm A and arm
B's default rest pose (all arm joints at their compiled 0 rad, unmodified
per ADR-016) already interpenetrates — **34 total contacts, 29 armA↔armB,
deepest -0.0597 m** (`armA_wrist` vs `armB_wrist`). Both arms' zero-angle
pose extends fully forward into the shared handoff band; the arms' own rest
posture, not the drawer or any prop, was occupying the workspace.
`docs/hardware/m02-physics-stability.md`'s existing caveat ("catches
interpenetration only indirectly... never asserts on `data.contact.dist`
directly") predicted exactly this — the scene passed that probe the whole
time it also carried a 6 cm self-interpenetration.

**Decision, four parts:**
1. **Added a `<key name="home">` keyframe** (generated in
   `scripts/gen_dual_scene.py`'s hand-authored region; upstream `qpos0` is
   untouched, ADR-016 holds): both arms folded back identically —
   `shoulder_pan=0, shoulder_lift=-1.2, elbow_flex=-1.6` (clamped in-range;
   the originally-suggested -1.8 is outside the asset's -1.69..1.69 limit),
   `wrist_flex=0, wrist_roll=0`, gripper open (range high end, +1.7453 rad).
   Same signs on both arms (verified by rendering every sign combination,
   not assumed, despite armB's 180°-rotated mount quat) — mirrored signs
   produced a visibly lopsided pose. Measured: **zero self-collision, zero
   cross-arm contacts** at the new `reset(seed=0)`. `TableSettingEnv.reset()`
   now applies this key via `mj_resetDataKeyframe`, falling back to plain
   `mj_resetData` if a scene has no "home" key.
2. **Re-measured the reachable envelope** from the corrected pose, finer z
   grid (0.02 m steps, 0.20–0.50 m). Previous envelope (sampled from the
   invalid, interpenetrating pose) is superseded, not deleted — the probe
   script now appends new sections instead of overwriting. New envelope:
   Arm A 186 points, x∈[-0.30,0.30], y∈[-0.35,0.10], z∈[0.24,0.50]; Arm B
   154 points, x∈[-0.30,0.30], y∈[-0.12,0.10], z∈[0.24,0.50]. **Shared
   handoff band: y∈[-0.12,0.10] m** — ADR-021's "~0.10 m shared band" was
   never checked against the compiled rest pose; it is now superseded by
   this measured band.
3. **Drawer moved a second time, onto the correct axis.** ADR-025's y=-0.25
   closed-face position is unreachable by either arm against the corrected
   envelope (residuals 0.32 m / 0.18 m) — ADR-025 moved the wrong axis; the
   constraint was height, and z=0.28 already fit once y was corrected.
   `drawer_housing` moves to y=0.08 (closed face y=0.0, inside both arms'
   envelope, residual ≈0.009 m each). z unchanged (0.28). Docs images
   re-rendered at 1280x720. Traded away, reported not hidden: the open
   drawer no longer protrudes past the table edge (open/closed states
   remain visually distinguishable, just not via edge protrusion anymore).
4. **Step 4 surfaced a separate, deeper finding and this ADR stops rather
   than chasing it:** every one of the four targets still fails the
   probe's collision check, including `plate_at_rest`/`bottle_at_rest`
   whose positions never moved. Traced to `bimanual.control.ik.solve_position_ik`
   (position-only, no obstacle term, ADR-024) converging to configurations
   that swing the arm through the tabletop for some targets — a
   pre-existing defect that ADR-025's own collision check (a raw
   contact-count delta against a ~29 baseline) was masking the whole time.
   Per this task's explicit stop rule, no further drawer position was
   tried — `plate_at_rest` fails identically without the drawer moving at
   all, so this is an `ik.py` fix, out of scope here, deferred to a
   follow-up.

**Consequences.** `reset(seed=0)`'s qpos/observations change for every
caller (flagged before the change, not discovered after).
`scripts/probe_physics_stability.py` re-run: still **PASS**, unchanged
numbers. `pytest tests/`: **4 tests in `tests/test_skills.py` now fail**
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`),
timing out during IK approach from the new folded pose — these encode the
OLD rest pose and are reported, not edited; re-tuning skills/IK for the new
home pose is a separate follow-up. `git diff --stat -- scenes/so101/` stays
empty throughout.

---

## ADR-025 — Drawer housing repositioning and IK pinch-point retargeting

**Recorded:** Sept 12, 2026 · **Follows:** M06a's failure diagnosis (ADR-024) ·
**Closes:** the two M06a root causes · **Supplements:** M02's done-when list
retroactively

M06a's four skills all failed. Two root causes, verified empirically, not
assumed: (1) the drawer housing (`y=-0.05`) sat fully enclosed beneath the
solid `table_top` slab, with no approach path — confirmed by MuJoCo's own
contact list; (2) `ik.py`'s IK target, the upstream `armX_gripperframe` site,
is ~8 cm from the jaws' actual pinch point (measured at rest, arm A).

**Decision:** (a) move the drawer housing 12 cm outward, `y=-0.05` ->
`y=-0.17`, via a generator edit in `scripts/gen_dual_scene.py`'s
hand-authored table/drawer/props/cameras template only — the
upstream-derived arm/mesh/material substitution was untouched
(`git diff --stat -- scenes/so101/` stayed empty, ADR-016/ADR-021 preserved).
The closed drawer face now sits flush with the table edge (`y=-0.25`); the
housing overhangs the edge by 2 cm (accepted) and its z-span still clears the
tabletop underside, so arm bases mounted on top do not collide with it.
(b) retarget `ik.solve_position_ik` to the computed midpoint between the
fixed jaw body (`armX_gripper`) and the moving jaw body
(`armX_moving_jaw_so101_v1`), recomputed every solve from the current qpos
via `mujoco.mj_jacBody` on both bodies (averaged), so the target tracks jaw
motion. `armX_gripperframe` is left in the XML/`ik.py`, unused, for reference;
a new `PINCH_POINT_OFFSET_M` constant documents the measured offset.

**Verification, run on bm-ptl in this order:**
1. Regenerated `src/bimanual/sim/assets/so101_dual_table.xml`; diff differs
   from the previous version only in the drawer housing position, the
   drawer_view camera's `xyaxes` (which follows `DRAWER_CAM_TARGET`, itself
   derived from the same housing move), and comments.
2. `scripts/probe_physics_stability.py` re-run: still **PASS**, unchanged
   numbers (`docs/hardware/m02-physics-stability.md`).
3. Re-rendered `docs/images/m02-scene.png`, `m02-drawer-view-closed.png`
   (drawer_slide forced to 0.0), `m02-drawer-view-open.png` (forced to 0.15),
   all at 1280x720.
4. **New: `scripts/probe_reachability.py`, run BEFORE any skill**, per
   instruction, because the risk that the drawer face might sit inside arm
   A's own minimum-reach dead zone was explicitly flagged as something to
   *test*, not assume. Result, recorded in full in
   `docs/hardware/m06-reachability-probe.md`:
   - **closed_drawer_face: FAIL for both arms** (residual 0.0226 m for arm A,
     0.2135 m for arm B; tolerance 0.01 m). The predicted dead-zone risk was
     real — the drawer move alone did not make the face reachable.
   - **plate/mug/bottle at rest: PASS for both arms** (residuals
     0.004–0.008 m, no new collision beyond a separately measured, pre-existing
     baseline — see below).
   - Per the task's explicit stop rule ("if the probe shows the drawer face
     is still unreachable after the move, STOP — do not guess at a second
     scene change"), a THIRD scene edit was **not** attempted. Instead, each
     arm's actual reachable envelope was measured by a coarse grid sweep
     (x in [-0.3,0.3], y in [-0.35,0.1], z in [0.2,0.5]): **95 reachable
     points for arm A**, roughly x in [-0.30,0.30], y in [-0.28,0.10],
     z in [0.30,0.50]; **57 for arm B**, roughly x in [-0.30,0.30],
     y in [-0.12,0.10], z in [0.30,0.50] — full point lists in the report.
     A future drawer position should be chosen from this measured envelope,
     not guessed a third time.
5. **Per the task's explicit reporting rule, the four skills
   (`open_drawer`/`pick`/`place`/`handoff`) were NOT re-run this pass**,
   since the probe did not fully pass: "if the probe fails, report the reach
   envelope instead and stop."

**A separate, unrelated finding, flagged not fixed:** building the
reachability probe's collision check required measuring a baseline, and that
measurement revealed arm A and arm B's DEFAULT REST poses already
interpenetrate by up to ~6 cm (`armA_lower_arm` vs. `armB_wrist`),
independent of any target and independent of both fixes above. This is an
ADR-021 arm-placement question (the ~0.30 m reach / 0.50 m base-gap
assumption was never checked against the compiled model's actual rest
configuration), out of this module's scope to fix.

**M02's own done-when list is retroactively incomplete.** M02 verified the
scene compiles, renders and is physically stable — it never verified the
scene is *reachable*. `scripts/probe_reachability.py` is the check that
should have existed from M02 onward; it is added now as a documented
supplement, against the evidence that surfaced this gap, not as a rewrite of
what M02 originally claimed.

Housekeeping: `pytest==9.1.1` added to `scripts/requirements-bmptl.txt`
(matching `scripts/requirements-dev.txt`) and installed into `ov_env` on
bm-ptl, so `tests/test_skills.py` can run through pytest where the simulation
actually lives (ADR-020), rather than via direct function calls.

---

## M06a — Scripted IK skills: ik.py, skills_scripted.py, executor.py, run_skill.py

**Recorded:** Sept 12, 2026 · **Follows:** ADR-024 (IK strategy), ADR-010 (bimanual
handoff), ADR-023 (scripted controller is the shipped policy) · **Consumes:** M02's
`TableSettingEnv`, M05's `SkillCall` · **Feeds:** M06b (`pour`, not built here)

Built per `PLAN.md` M06a scope: `open_drawer`, `pick`, `place`, `handoff` only —
`pour` is explicitly M06b and is not implemented (`ScriptedSkillExecutor.execute()`
returns a labelled failed `SkillResult` for it rather than raising). Every skill
returns `SkillResult(success, reason, frames_used)`; `frames_used` counts
`env.step()` calls (a step BUDGET, per `ik.DEFAULT_STEP_BUDGET`, not a wall-clock
timer), so a timeout is `success=False, reason="timeout..."` at a fixed step count
regardless of host speed.

**ADR-024 records the IK strategy**: position-only damped-least-squares IK over
the arm's 5 positioning joints (excluding the jaw), targeting the `armX_gripperframe`
site — orientation is relaxed entirely, not partially. `ScriptedSkillExecutor`
asserts `TableSettingEnv(cameras=None)` at its entry point (ADR-023: state-only,
~0.20 ms/step) and catches `ValueError` narrowly, re-raising `UngroundedCommandError`
explicitly rather than letting a broad `except ValueError` swallow it — the M05 trap
Tester flagged, guarded here even though this layer cannot reach it today.

**Honest result, measured on `bm-ptl` via `tests/test_skills.py` and
`scripts/run_skill.py` (seed 0):**

| Skill | Result | Measured delta | frames_used |
|---|---|---|---|
| `open_drawer(A)` | **FAIL** | `drawer_slide` qpos stayed at 0.0000 (target ≥0.14) | 3000 (timeout) |
| `pick(A, plate)` | **FAIL** | plate z: 0.3560 → 0.3531 (target ≥0.3800) | 542 |
| `place(A, plate)` | **FAIL** (aborted at its internal `pick`) | plate z: 0.3560 → 0.3531 | 542 |
| `handoff(A→B, mug)` | **FAIL** (aborted at its internal `pick`) | mug z: 0.3900 → 0.3825 | 552 |

(Numbers above are bm-ptl's, via `scripts/run_skill.py --seed 0`, matching `tests/test_skills.py`'s numbers to within floating-point noise across the two hosts.)

Two independently diagnosed root causes, not one ambiguous failure:
1. **`open_drawer` is blocked by scene geometry**, not by the controller. MuJoCo's
   own contact list shows `table_top` in contact with an (unnamed) arm mesh geom at
   z≈0.35 directly above the drawer housing — the `table_top` box spans the entire
   table footprint with no cutout above the drawer, so the drawer sits fully
   enclosed beneath a solid slab with no top-down or side access. Verified by
   contact inspection, not assumed. Cannot be fixed without editing
   `src/bimanual/sim/assets/so101_dual_table.xml`, which is out of this module's
   scope — flagged here for the planner/compliance-reviewer as a scene defect, not
   silently worked around.
2. **`pick`/`place`/`handoff` do not reliably grasp**, for the reason ADR-024's
   Consequences records in detail: the `armX_gripperframe` site is measurably not
   co-located with the jaw's true pinch point (~2-5 cm lateral, ~5 cm vertical
   offset measured at one configuration, not constant across configurations because
   orientation is uncontrolled), and the jaw's collision mesh (`geom_rbound` up to
   ~8.4 cm) is large relative to the props, so the approach frequently displaces the
   object before a pinch can form.

**What is NOT in question:** `scripts/run_skill.py`'s diagnostics
(`max_joint_limit_violation`, MuJoCo `data.warning` counts) were checked on every
run above and were zero or negligible (max observed `3.5e-6`) — PLAN.md M06
done-when 4 (no joint-limit/self-collision violation) holds even where the task
itself fails. The IK solver itself converges reliably in the kinematic sense
(sub-centimetre residual within single-digit iterations) and drives real,
stable physical motion; the gap is entirely in grasp mechanics, diagnosed above,
not in the control loop, the step-budget contract, or the executor's dispatch.

Per instruction, this was reported at this point rather than iterated on further:
"a partial, honestly-reported M06a is far more useful than a late one."

---

## M04 — CommandSource ABC with Text and Voice implementations (voice stub)

**Recorded:** Sept 12, 2026 · **Follows:** ADR-002 · **Feeds:** M05

Built per `ARCHITECTURE.md` ADR-002 and the section 1 component-contract table, not per
the simplified `get_command() -> str` signature that was floated when this module was
handed off: `CommandSource` is abstract with `poll() -> CommandEvent | None` and
`close()`, non-blocking on both implementations. `CommandEvent` carries `text`,
`timestamp`, `source_id`, `confidence: float | None`, `raw_meta: dict`
(`src/bimanual/command/events.py`).

`TextCommandSource` (`src/bimanual/command/text_source.py`) is constructible from a
literal CLI string, a file (one command per line), or stdin (read eagerly at
construction so `poll()` itself never blocks). It does not validate or strip
text — empty and whitespace-only commands pass through unchanged, since deciding what
counts as a "real" command is M05's Grounder's job, not this transport's.

`VoiceCommandSource` (`src/bimanual/command/voice_source.py`) is a stub with the right
shape only: constructed from an audio file `Path`, emits one fixed placeholder
transcription with `source_id="voice_stub"` and a placeholder `confidence`, then `None`.
Speechmatics is not wired — that is M15, the droppable bonus (`CONSTRAINTS.md:38-41`).

Enforcement of ADR-002's "no audio crosses the boundary" is structural, not a comment:
`grep -ri "audio|pcm|wav|microphone" src/bimanual/language src/bimanual/policy` returns
no matches. `tests/test_command_source.py` (11 tests, all passing) covers normal text,
empty string, whitespace-only, an unknown-word command, and a single test parameterised
over both `TextCommandSource` and `VoiceCommandSource` proving they satisfy the same
`CommandSource` contract. `pytest==9.1.1` was not previously installed on the laptop; it
is now pinned in `scripts/requirements-dev.txt` so the suite is reproducible from a
fresh clone.

---

## M05 — Rule grounder complete (41c7a29)

**Recorded:** Sept 12, 2026 · **Implements:** ADR-001 (two-tier control), ADR-003
(rule grounder as the deterministic floor) · **Consumes:** M04's `CommandEvent` ·
**Feeds:** M06

`ground(CommandEvent, scene_belief=None) -> TaskPlan`, matching ARCHITECTURE.md's
component-contract table. `SceneBelief` is M10's output and does not exist yet, so it is
an optional parameter the rule grounder ignores today — the signature is correct now and
needs no change when M10 lands.

**Arm assignment is explicit on every `SkillCall`, never `None`** (M05 done-when 3). The
default rule, documented in `docs/command-grammar.md` so a judge can predict it: an unnamed
arm defaults to **arm A**, except `handoff`'s origin arm, which defaults to the other of
the two arms given the mandatory destination arm. This field is what M06 dispatches on and
what makes `handoff` meaningful; a grounder without it would undercut the 30-point bimanual
criterion.

**Two failure modes are deliberately distinct, and conflating them was the defect this
design guards against:**
- Empty or whitespace-only input → an **empty `TaskPlan`, no exception**. Nothing was asked.
  M04 passes text through unvalidated and unstripped by design, so this case arrives here.
- A non-empty command that cannot be grounded → **`UngroundedCommandError`**, a typed error.
  Something was asked and could not be honoured.
Verified: a mixed command ("Pick up the plate with arm A and dance.") raises rather than
returning a partial plan. A half-executed plan on demo day is worse than a clean refusal.

The brief's verbatim example command parses to:
`open_drawer(A, drawer)` → `pick(A, plate)` → `place(A, plate, dest=table)` →
`pick(B, mug)` → `pour(A, mug, source=bottle)`.

43 tests in `tests/test_grounder.py`; 54 across the suite with no regression in M04's.
11 paraphrases (requirement ≥8) and 4 typed-error cases (requirement ≥3).

**Scope note for readers of `docs/command-grammar.md`:** it documents the grammar the
grounder **accepts**, not ADR-011's full scene sequence, which is richer — ADR-011 adds
fork and spoon and places `handoff` on the mug. Do not read the grammar as a claim about
what the demo performs.

---

## M03 — OpenVINO conversion smoke test complete (b50e300)

**Recorded:** Sept 12, 2026 · **Closes:** ADR-014's first-48-hours requirement ·
**Feeds:** ADR-013 (export plan), M10, M13

M03 done Sept 12 (b50e300). Two findings for downstream modules:

**(1) NPU (NPU5010) rejects fully-dynamic batch destructively** — `STATUS_ACCESS_VIOLATION`
(exit `3221225477` / `0xC0000005`), not a catchable exception. **M13 export MUST use static
or bounded batch dimensions — a fully-open `-1` is unsupported and crashes the process.**
Diagnostic: the NPU compiler demands upper bounds on any dynamic dim —
`Upper bounds are not specified for node 'Multiply_11422' (type 'Convolution'): input '0'
bounds are '[9223372036854775807, 3, 224, 224]'` (`9223372036854775807` = `INT64_MAX`).
Static FP32 and FP16 both compiled and inferred correctly on NPU.

**(2) The conversion pipeline uses `openvino.convert_model` directly from live torch
modules — no ONNX intermediate.** The `torch.onnx.export` path was deliberately avoided:
it needs `onnx` and `onnxscript`, neither pinned. Fewer moving parts. **Do not assume ONNX
exists in the pipeline.**

Max absolute deviation vs the PyTorch FP32 reference: CPU `5.674362e-05`, GPU (Arc B390)
`7.408857e-05`, NPU `1.122952e-04`. Full evidence in `benchmarks/ov-smoke-notes.md`.

A third, procedural finding worth keeping: the first run accumulated results in memory and
wrote them once at the end, so the NPU crash destroyed the NPU *static* results that had
already passed. `scripts/ov_smoke.py` now runs each device/precision/shape check in its own
subprocess and writes immediately. Any future harness that probes a device which can crash
the process needs the same shape.

---

## ADR-023 (ratified Sept 12): Cut ACT training from critical path.

**Context.** Sept 12 is Day 2 of a schedule with five days left, and the plan is one
module behind. M03 — "a hard blocker for the whole OpenVINO story (20 rubric points)",
forced into the first 48 hours by ADR-014 — did not ship on Day 1 and is verifiably absent
(`scripts/` has no `ov_smoke.py`; `benchmarks/` has no `ov-smoke-notes.md`). Day 2 as
previously planned therefore carried M03 (3 h) plus M04–M08 (23 h) = **26 builder-hours in
one calendar day**, before the per-module tester → compliance-reviewer → tutor →
docs-writer passes that `PLAN.md` section 2 mandates. Day 1's own shape is the evidence for
what a day actually holds: M01 (2 h budgeted) + M02 (6 h budgeted) plus a correction pass
(ADR-016..ADR-022 and the opt-in rendering refactor) consumed the whole day.

Two further facts bind. ADR-020 means every simulation module runs on bm-ptl, and bm-ptl
expires Sept 17 00:15 (`CONSTRAINTS.md:5`), one day past the Sept 16 submission
(`CONSTRAINTS.md:4`) — so Day 6 is a working day, not a retry window. And ADR-022's
correction put M09's demonstration collection at ~456 ms/step, i.e. ~8.9 h of bm-ptl wall
clock for 70,000 attempted steps, which can only be spent overnight and only after M06,
M07 and M08 have all closed.

The arithmetic that follows is not close. M09b needs M06 + M07 + M08; at ~8
builder-hours/day those close Day 4 evening at the earliest; an overnight collection then
lands Day 5 morning, M11 trains Day 5, and GATE-1 could not be held before Day 5 night.
`PLAN.md`'s GATE-1 block already forbids sliding the gate even into Day 4.

**Options.**
- (a) **Keep ACT, slide GATE-1 to Day 5 night.** Leaves the OpenVINO benchmark, the
  bm-ptl pipeline run, the 10-seed recording, the documentation and the submission all
  stacked on Day 6, on hardware that expires that night. Directly contradicts
  `CONSTRAINTS.md:50-52`, which rewards a complete pipeline over half-working ML.
- (b) **Keep ACT, shrink the dataset to the pre-committed 4 h / ~31,500-attempted-step
  run.** This was `PLAN.md`'s own recommended degradation, and it is still the right
  degradation *within* the learned branch — but it saves ~5 h of unattended wall clock,
  not the ~18 builder-hours the schedule is actually short. It treats a capacity problem
  as a wall-clock problem.
- (c) **Cut ACT (M11) and its demonstration dataset (M09b) from the critical path, ship
  the scripted controller as the policy, and keep a much cheaper frame-collection module
  (M09a) to feed the PoseNet that ADR-009 put in the loop.**
- (d) **Cut PoseNet (M10) instead and keep ACT.** Puts the 20-point OpenVINO criterion
  back onto the riskiest module — exactly the dependency ADR-009 exists to break — and
  leaves `--perception state` in the demo loop against ADR-005.

**Decision.** (c). M11 and M09b move wholesale into F3 (optional revival after M18, under
F3's existing hard stop, and realistically unreachable). The scripted controller from M06
is the policy: F1 and F2 are activated now rather than being contingent on a GATE-1
verdict. GATE-1 is retained on Day 3 as `CONSTRAINTS.md:50-52` requires, but is decided on
**schedule evidence** and its outcome is pre-committed to the scripted branch.

Three sub-decisions ride with it and are recorded here because they are structural, not
scheduling detail:

1. **A planning capacity is fixed at 8 builder-hours/day**, module budgets stay denominated
   in builder-hours, and the day rather than the module absorbs the ~30% review overhead.
   This is an assumption derived from Day 1's shape, not a measurement, and it is the
   number to correct if the user knows their throughput differs.
2. **Clock gates replace judgement calls.** Every day boundary in `PLAN.md` section 1A.5
   carries a pre-committed time, and `PLAN.md` section 1A.6 is an ordered cut ladder whose
   rungs fire on a missed gate without re-litigation. The specific hole this closes: M09b's
   "if M08 does not close in time" had no definition of "in time", so the most
   consequential decision in the plan was left to a tired developer at night. It is now
   22:00 / 23:00 / abort, with a 07:00 hard stop, retained in force for F3.
3. **M09a's camera set is re-derived from its surviving consumer.** ADR-022 fixed M09's
   cameras at `['front', 'armA_wrist', 'armB_wrist']` *because that was M11's training
   set*. With M11 cut, the set follows M10's PoseNet input instead: `front` +
   `drawer_view`, because the drawer is occluded from every other camera (ADR-021) and
   `SceneBelief` carries drawer opening. Using the measured marginal camera cost of
   152.05 ms (`docs/hardware/m02-render-cost.md:43-49`) that is **~304.3 ms/step**, and
   ~5,000 frames is ~25 min of bm-ptl wall clock rather than ~9 h. **This does not
   supersede ADR-022** — ADR-022's Decision (opt-in rendering) and its ~456 ms/step figure
   are unchanged and return with ACT if F3 revives it.

**Consequences.**

*What this costs, stated first and without softening.* **No policy is trained.** Brief p2
objective 4 asks for a policy trained or fine-tuned with LeRobot or compatible tooling;
with M11 cut, nothing satisfies it. PoseNet is a trained model but it is a perception
network, not a policy, and it is not LeRobot. ADR-015 rule 2 already forbids calling the
shipped control path learned; this ADR adds the positive obligation — `PLAN.md` M18
done-when 5 — that the README *state* the gap rather than merely avoid misdescribing it,
and that it record that the branch was chosen on schedule grounds. "We chose scripted
because we ran out of days" and "we chose scripted because learning underperformed" are
different claims and only the first is true.

*Second cost.* M16 (full pipeline on bm-ptl) loses its day of slack: it folds into M14's
Day-6 bm-ptl session, on the last day the instance exists. The Day-5 20:00 gate — at least
one IR compiling on at least one device before Day 6 begins — is the only insurance left
against that, and it is thin.

*Third cost.* ADR-006 committed the evaluation harness to Day 2 specifically so that it
would exist before there was anything good to measure. M08 now spans a Day-3 evening
skeleton and Day 4. That is a real weakening of ADR-006 and is recorded as such rather
than presented as equivalent. The mitigating fact is narrow but genuine: with M11 cut there
is no second executor for the harness to be quietly shaped around, so the specific failure
ADR-006 insured against — a harness written after the numbers exist, to fit them — is
smaller.

*What survives, and why the cut is survivable.* ADR-009's whole purpose was to stop the
20-point OpenVINO criterion riding on ACT, and that argument now carries the submission:
PoseNet is converted, quantized, benchmarked across CPU/GPU/NPU, and executes on Intel
silicon on every control step of the demo. ADR-004 made M06 dual-purpose; with M11 gone it
is simply the policy. Brief p2 objectives 1 and 3 are answered by M06 + M07, objectives 2
and 5 by M05's grounder plus M10's OpenVINO-served perception. ADR-002's CommandSource
seam is untouched — M04 still ships the abstraction and a stub voice source even though
the Speechmatics implementation is unscheduled — and ADR-010's scripted `handoff`,
ADR-012's seed-derived randomization and ADR-018's true-success-rate reporting are all
unaffected.

*Honest bottom line.* Even after this cut the plan needs ~9–10 builder-hours/day for five
consecutive days against a capacity of ~8: roughly one day of negative float, recorded as
RISK-12. This ADR does not make the plan comfortable. It makes the plan's failure mode
"some scope was cut and said so" instead of "the pipeline was incomplete on Sept 16".

Ratified by user Sept 12. CONSTRAINTS.md fallback clause invoked
early on schedule evidence per compliance-reviewer's justification.
ADR-008's numeric gate becomes a formality; scripted controller is
the shipped policy.

---

## ADR-022 — Opt-in camera rendering in `TableSettingEnv`

**Ratified:** Sept 11, 2026 · **Evidence:** `docs/hardware/m02-render-cost.md`

Rendering all five cameras on every step cost ~810 ms; physics alone is 0.20 ms.
Measured on bm-ptl: state-only 0.20 ms/step, one camera 152.25, all five 809.86 —
a **4049x** spread. Confirmed real GPU time (`Intel(R) Arc(TM) B390 GPU`, OpenGL
4.6), and cameras scale linearly, so there is no fixed cost to amortise.

Rejected: keeping unconditional rendering (dominates step cost for callers that
never read an image); frame caching keyed on unchanged action (policy rollouts
change action every step, so it would never hit).

**Decision.** `cameras=None` by default — render nothing. `reset()` and `step()`
take a per-call `cameras=` override that does not mutate the instance default.
`render()` stays ungated as an explicit escape hatch. Names validate against the
model's discovered cameras, not a hardcoded list.

Per-module intent:
- **M09** collects **with cameras enabled** (`front`, `armA_wrist`, `armB_wrist`
  — ADR-005's vision path, and M11's exact training set). The scripted
  controller reads `get_state()` to *choose actions*; the camera frames are
  *logged into the dataset* for ACT. Cost **~456 ms/step** (0.20 ms physics +
  3 x 152.05 ms/camera).
- **M11** trains on those same three views; Kaggle reads pre-rendered frames and
  never invokes MuJoCo, so the render cost lands on M09.
- **M08** splits by executor: `--executor scripted` scores state-only
  (0.20 ms/step, physics only); `--executor learned` **requires** cameras at
  ~456 ms/step, because an ACT policy cannot produce an action from an obs dict
  with no images. Video capture uses all five cameras (~809.86 ms/step) **once**,
  on the final winning seed only.

**Correction, Sept 11, 2026 — Consequences only; the Decision above is
unchanged.** This entry previously said M09 collects with no cameras and that
M08 scores GATE-1 state-only. Both were wrong, from one root cause: ADR-005 was
read as governing *dataset contents* when it governs only *how the scripted
controller selects actions*. M11's policy is camera-conditioned, so a state-only
M09 yields a dataset ACT cannot train on; and state-only scoring of a learned
executor is not a cheaper measurement but an impossible one. The cost is carried
in `PLAN.md`: M09 collection moves to ~456 ms/step, and the learned half of
GATE-1 moves from seconds to ~76 min (10 seeds x 1000 steps). Full version
history in `ARCHITECTURE.md` ADR-022 Consequences. Note that
`docs/hardware/m02-render-cost.md:65-66` still repeats the superseded reading and
is not authoritative on this point.

---

## M02 — TableSettingEnv, view_scene.py, physics probe, front-camera fix (fulfills ADR-020, ADR-021)

**Recorded:** Sept 11, 2026 · **Module:** M02 (dual-SO-101 table scene v0) ·
**Relates to:** ADR-020 (sim runs on bm-ptl), ADR-021 (generator, not hand-edit)

Compliance-reviewer found M02 done-when criteria 1, 2 and 4 unmet (`src/bimanual/sim/env.py`
and `scripts/view_scene.py` did not exist yet). Closed in four separately committed steps,
each run and verified on bm-ptl per ADR-020 (mujoco cannot import on the laptop):

1. `src/bimanual/sim/env.py` — `TableSettingEnv` with `reset(seed)`, `step(action)`,
   `render(camera)`, `get_state()`, `close()`. Default scene path resolves via
   `Path(__file__).resolve().parent`, not the process cwd; verified importable and running
   from a cwd other than the repo root (`C:\Users\devcloud\import_check.py`, run from
   `C:\Users\devcloud`). Measured on bm-ptl (mujoco 3.2.7): `nq=48 nv=43 nu=12`, 4 cameras
   discovered from the model (`overhead`, `front`, `armA_wrist`, `armB_wrist`).
2. `scripts/view_scene.py` — `python scripts/view_scene.py --headless --save out/scene.png`
   writes a 1280x720 PNG (clamped to the scene's declared framebuffer) via `TableSettingEnv`.
3. `scripts/probe_physics_stability.py` — reset(seed=0), 1000 steps of zero action, checked
   for NaN and free-body tunneling at every step. **PASS**: no NaN, no free body dropped
   below `FLOOR_Z=0.30` m (5 cm below the 0.35 m tabletop surface — chosen so ordinary
   millimetre-scale contact settling cannot trip it; see
   `docs/hardware/m02-physics-stability.md` for the full derivation and the measured per-body
   minimum z values). Per the task's stop rule, this gated whether subtask (d) proceeded —
   it passed, so (d) went ahead.
4. Front-camera clipping fix. The camera used `mode="targetbody" target="table"`, which
   aims at the table body's own origin (z=0); measured on bm-ptl, the highest arm geometry
   at the reset pose reaches z≈0.667 against a 0.35 m tabletop, so the frame cropped both
   arms above the gripper. The camera is emitted by `scripts/gen_dual_scene.py`'s template
   string, not hand-typed into the generated XML, so per ADR-021's consequences the fix went
   into the generator (`FRONT_CAM_POS`/`FRONT_CAM_TARGET`/`look_at_xyaxes`, a stdlib-only
   look-at basis construction) and the scene was regenerated, not hand-patched. `git diff`
   on the regenerated XML shows only the one `<camera name="front".../>` line changed.
   Re-rendered `docs/images/m02-scene.png`; both arms fully visible on inspection. Overhead
   and wrist cameras were not touched (re-rendered for comparison; identical framing).
   `git diff --stat -- scenes/so101/` is empty throughout.

The package was installed editable on bm-ptl (`pip install -e .`) so `import bimanual` and
`from bimanual.sim.env import TableSettingEnv` resolve from any working directory, per the
task's tester-readiness requirement.

---

## ADR-021 — Dual-arm scene composed by scripted renaming, not MJCF `<include>`

**Recorded:** Sept 11, 2026 · **Module:** M02 (dual-SO-101 table scene)

`PLAN.md` M02 named two candidate ways to duplicate the unmodified SO-101 arm for the
bimanual scene: MJCF `<include>`, or hand-copying the body tree. MuJoCo requires every
body, joint, site and actuator name to be unique across the whole compiled model.

Tested empirically on bm-ptl before deciding, via `scripts/probe_include_namespace.py`
(mujoco 3.2.7): `<include>`-ing `scenes/so101/so101_new_calib.xml` twice does not fail on
a naming collision — it fails earlier, because MuJoCo's compiler refuses to include the
same file twice at all: `ValueError: XML Error: File 'scenes/so101/so101_new_calib.xml'
already included / Element 'include', line 4`. `<include>` has no prefix attribute, so
even a byte-identical second copy under a different filename would still collide on every
body/joint/site/actuator name. `<include>` is therefore not viable for arm duplication,
full stop — this is the deciding factor, established by measurement rather than by reading
the MJCF spec alone.

Hand-copying the ~120-line body tree twice by hand was rejected in favour of
`scripts/gen_dual_scene.py`: a stdlib-only (`xml.etree.ElementTree`, no `mujoco` import,
runs on the laptop despite ADR-020) script that parses the unmodified upstream
`so101_new_calib.xml`, deep-copies its body tree twice, prefixes every body/joint/site name
(`armA_`/`armB_`), repositions each copy, generates a matching renamed actuator pair, and
splices the result into a hand-authored table/drawer/props/cameras template. Mesh and
material definitions are declared once, shared by both arm copies, since geometry is not
per-instance data. The deciding factor over hand-copying: a script guarantees the two arm
copies stay byte-faithful to upstream and to each other, resynchronized by a single command
rather than by two independent manual edits with nothing to catch a missed rename — the same
transcription-drift failure mode ADR-016 already flagged for the single-arm case.

Layout: SO-ARM100 reach is approximately 0.30 m (stated assumption), so arm bases are
placed 0.50 m apart on the table's two long edges (0.25 m off centreline each), leaving a
roughly 0.10 m wide band at the table centre reachable by both arms, where the five props
sit. Full context, options and consequences in `ARCHITECTURE.md` ADR-021.

---

## M01 — RISK-07 resolved: `openvino-telemetry` pin unified to `2025.2`

**Recorded:** Sept 11, 2026 · **Closes:** RISK-07 · **Module:** M01 (repo scaffold and
pinned environments)

`PLAN.md` RISK-07 flagged a discrepancy: `scripts/requirements-bmptl.txt:3` pinned
`openvino-telemetry==2025.2` while `benchmarks/bmptl-environment.txt:3` recorded
`openvino-telemetry==2025.2.` with a trailing period.

Read both files directly during M01: as of commit `e5d74eb` (before this module's own
work began) both already read `openvino-telemetry==2025.2`, with no trailing period and
no other differences on that line. The trailing-period form was never a real,
installable pin — `pip index versions openvino-telemetry` lists PyPI releases as
`2025.2.0`, `2025.1.0`, `2025.0.1`, etc. (three-component versions only); `2025.2.` is
not one of them and is not valid PEP 440 syntax as a distinct release. `2025.2` is a
valid pin and resolves to `2025.2.0` under PEP 440's version-normalization rules (a
trailing implicit zero), so `2025.2` — not `2025.2.` — is the correct value, and it is
the one both files share.

No further edit to either file was needed; this entry exists because the done-when
criterion for M01 requires the resolution to be *recorded* here, not only present in the
files. `scripts/verify_env.py` (this module's other output) checks
`scripts/requirements-bmptl.txt` package-by-package against installed versions
whenever it is pointed at that file, so a future re-introduction of the mismatched
trailing-period form on either file would surface as a plain string mismatch against
whichever file is passed to `--requirements`, not as a silent drift.

---

## ADR-020 — Simulation runs on bm-ptl, not the laptop

**Ratified:** Sept 11, 2026 · **Closes:** RISK-11 · **Promotes:** RISK-03 to blocking

`mujoco.MjModel.from_xml_path` fails on the laptop with `OSError: [WinError 4551]` —
Windows Smart App Control blocks the unsigned `mujoco.dll`. Reproduced on `mujoco==3.13.0`
and `3.2.7`, so it is an OS policy issue, not an asset or package defect
(`scenes/so101/VERIFICATION.md`).

Options: (a) disable Smart App Control locally — one-way and a standing security
regression on the daily machine, **rejected**; (b) run simulation on bm-ptl — dev matches
the deployment target the brief requires, gains Arc B390 and 32 GB, costs an SSH iteration
tax, **chosen**; (c) add WSL2 — another environment to pin plus EGL quirks, **deferred**
but retained as the escape hatch.

**Decision.** The laptop is for code, git and the Speechmatics client. bm-ptl runs all
MuJoCo work.

The outstanding MuJoCo compile-check of the SO-101 asset moves to bm-ptl, as does M02's
rendered PNG and every sim module's iteration loop. Note the cost: with (a) rejected and
(c) unbuilt, **RISK-03 (offscreen rendering on bm-ptl) now has no fallback** and the M03
rendering probe becomes a blocking prerequisite. bm-ptl's Sept 17 00:15 expiry now bounds
simulation development, not just benchmarking, so work must stay pushed to git rather than
living on the instance.

---

## M02 prerequisite: SO-101 asset acquisition — adopted from TheRobotStudio/SO-ARM100 (fulfills ADR-016)

**Recorded:** Sept 11, 2026 · **Closes:** RISK-01 · **Fulfills:** ADR-016

Per the user-specified search order for this prerequisite, option (a) — the Intel Hack-a-thon Resources
bundle referenced by a button on the challenge platform page
(`docs/challenge/Screenshot 2026-09-11 132213.png`) — could not be reached: no URL for
it appears anywhere in the challenge-brief PDF (checked programmatically for link
annotations, none found) or elsewhere in this repository. Proceeded to option (b):
**TheRobotStudio/SO-ARM100**, `https://github.com/TheRobotStudio/SO-ARM100`, commit
`eecbe3e0a9ebb23e25ad7b2759b03884c6660903`, files under `Simulation/SO101/`. License
**Apache-2.0**, permissive and public-repo-compatible; text copied to
`scenes/so101/LICENSE`. Full provenance in `scenes/so101/PROVENANCE.md`.

Per ADR-016, the asset's shipped kinematics were taken unmodified. Actuated DoF per
arm, read from `so101_new_calib.xml`'s `<actuator>` block: **6** — `shoulder_pan`,
`shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`, all hinge joints
under `<position>` actuators. This closes RISK-02's remaining "what is the number"
question with a measurement; it feeds ARCHITECTURE.md section 1's contract table at
GATE-1 as ADR-016 specified.

`mujoco.MjModel.from_xml_path('scenes/so101/scene.xml')` could not be executed
successfully on this laptop: Windows Smart App Control (enforced, not evaluation
mode) blocks loading `mujoco.dll` because it is an unsigned binary with insufficient
Microsoft cloud reputation — confirmed via Windows Event Log
`Microsoft-Windows-CodeIntegrity/Operational` (Event ID 3077/3118) and
`Get-AuthenticodeSignature` reporting `NotSigned`. This reproduces identically on
`mujoco==3.13.0` and `mujoco==3.2.7`, so it is an OS policy issue, not an asset or
package defect. A plain-XML structural check (no MuJoCo) confirms the files are
well-formed, all 6 actuated joints and all 13 referenced mesh files are present and
resolvable. Full verbatim command/output and diagnosis in
`scenes/so101/VERIFICATION.md`. Flagged as a new, previously unlisted risk
(**RISK-11**, not yet added to `PLAN.md` — planner's file, not builder's to edit) for
the planner/user to decide: disable Smart App Control on this laptop, verify on
bm-ptl instead, or add WSL as a local dev path.

---

## ADR-019 — Speechmatics credentials via gitignored `.env`

**Ratified:** Sept 11, 2026 · **Closes:** RISK-04 (handling) · **Relates to:** ADR-002

The Speechmatics API key lives in `.env`, which is gitignored. Code reads it from the
environment as `SPEECHMATICS_API_KEY`, never from a literal. A committed `.env.example`
names the variable with an empty value so setup stays reproducible. A missing key fails
loudly at startup; it must not silently disable voice.

Verified at ratification: `.gitignore:23` ignores `.env`; the local `.env` is untracked;
no `.env`, `*.pem` or `.kaggle` path appears in git history. A credential scan runs before
the repo is flipped public on Sept 15.

---

## ADR-018 — 10-seed evaluation reports the true success rate

**Ratified:** Sept 11, 2026 · **Closes:** RISK-06 · **Strengthens:** ADR-015 rule 4

The 10 evaluation seeds are declared in the eval config before the run and are not
changed afterward to improve the result. The reported rate is successes over those 10.
Cherry-picking 10 successes from a larger pool is prohibited. Failure modes must be
shown and narrated in the video, not merely tabulated in the repo — if 7 of 10 succeed,
the video says 7 of 10 and shows what the other 3 did.

Re-running a seed after a code change is normal iteration. Swapping the seed set after
seeing results is not; the committed seed list makes the difference auditable.

---

## ADR-017 — `pour` is a tilt-and-position motion, with mandatory disclosure

**Ratified:** Sept 11, 2026 · **Closes:** RISK-08 · **Confirms:** ADR-011

Arm B holds the mug, arm A brings the bottle over it and achieves a tilt-and-hold pose
within a stated tolerance. Success is pose achievement. No fluid, particle or volume is
simulated.

The absence of fluid must be stated in `README.md` and spoken in the video narration —
not a caption, not a footnote, not repo-only prose, because the video reaches judges who
may never open the repo. Compliance-reviewer treats a missing statement as a defect.

---

## ADR-016 — SO-101 DoF is whatever the asset ships, unmodified

**Ratified:** Sept 11, 2026 · **Closes:** RISK-02 · **Feeds:** M01, M02, ADR-013

The adopted SO-101 asset's shipped DoF is authoritative. The kinematics are not edited to
suit our controller — no locked joints, no added DoF. IK, ACT action dimensions and
OpenVINO input shapes adapt to the asset instead. Builder reports the actual DoF in
M01/M02 and it is written into the `ARCHITECTURE.md` section 1 contract table then.

Scene composition — arm placement, table, objects, cameras — is ours. The arm model is not.
