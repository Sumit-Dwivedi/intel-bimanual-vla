# DECISIONS

User-ratified decisions, newest first. Each entry mirrors an ADR in
`ARCHITECTURE.md` section 4, which holds the full context, options and
consequences. Where the two disagree, `ARCHITECTURE.md` is authoritative and the
disagreement is a defect to be fixed.

ADR-001 through ADR-015 were authored by the planner and live only in
`ARCHITECTURE.md`. This file starts at ADR-016, the point at which decisions began
being ratified by the user rather than proposed.

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
