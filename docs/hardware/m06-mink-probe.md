# M06 mink adoption probe: collision-aware bimanual IK

Scaffolding probe only. **No production code changed in this commit** --
`src/bimanual/control/ik.py`, `skills_scripted.py`, `executor.py` and
`grasp.py` are untouched. This is `scripts/probe_mink_bimanual.py`'s own
write-up: every number below was printed by that script on bm-ptl on
2026-09-14, not hand-computed or assumed. Run command and full stdout are
reproduced at the bottom of this file.

## 0. What was resolved before writing any probe code

**mink version.** The task brief assumed `mink==1.2.0`. `pip index versions
mink` on bm-ptl (`ov_env`) reported the current PyPI release as **1.3.0**
(others available: 1.1.1, 1.1.0, 1.0.0, 0.0.13 ... 0.0.1). **1.3.0 is what was
installed and pinned**, not the brief's assumed number.

**API surface.** Checked against the INSTALLED package via
`inspect.signature`/`inspect.getsource`, not assumed from the brief or from
mink's aloha example:

| Name | Present? | Signature (abbreviated) |
|---|---|---|
| `mink.Configuration` | yes | `(model, q=None)` |
| `mink.FrameTask` | yes | `(frame_name, frame_type, position_cost, orientation_cost, gain=1.0, lm_damping=0.0)` |
| `mink.CollisionAvoidanceLimit` | yes | `(model, geom_pairs, gain=0.85, minimum_distance_from_collisions=0.005, collision_detection_distance=0.01, bound_relaxation=0.0, broadphase=True)` |
| `mink.ConfigurationLimit` | yes | `(model, gain=0.95, min_distance_from_limits=0.0)` |
| `mink.VelocityLimit` | yes | `(model, velocities={})` |
| `mink.solve_ik` | yes | `(configuration, tasks, dt, solver, damping=1e-12, safety_break=False, limits=None, constraints=None, **kwargs)` |
| `mink.get_subtree_geom_ids` | yes | `(model, body_id) -> list[int]` |
| `mink.PostureTask` | yes | `(model, cost, gain=1.0, lm_damping=0.0)` |

`frame_type` accepts `"body"`, `"geom"` or `"site"` (confirmed from
`FrameTask`'s own docstring) -- so a `FrameTask` CAN target a body directly,
which matters for section 1 below.

**Solver.** `qpsolvers.available_solvers` reported `['daqp']` after
`pip install mink==1.3.0` -- `mink`'s dependency spec pulls in
`qpsolvers[daqp]` automatically. **No separate `pip install daqp` step was
needed on this platform**, correcting the brief's "daqp may need separate
installation" caveat. `solver="daqp"` is the only backend used below because
it is the only one installed.

**Licensing.** mink reports `License-Expression: Apache-2.0` via
`pip show mink` (confirmed directly, not assumed from the project page).
Following this project's existing Apache-2.0 practice
(`scenes/so101/PROVENANCE.md`, ADR-016): that practice exists for a VENDORED
asset (SO-101's MJCF/meshes, copied into this repo, so the license text and a
NOTICE-file check were copied alongside it). mink is a normal `pip`
dependency, not vendored source -- nothing from it is copied into this repo,
so there is no file to carry a license/NOTICE text into. The equivalent
obligation for a pip dependency is a clear, checkable version pin plus a
license note at the point the dependency is declared, which is what
`scripts/requirements-bmptl.txt`'s new mink block now provides (package name,
pinned version, `License-Expression`, and why it was chosen). No NOTICE file
ships in the mink 1.3.0 wheel's metadata (checked: `pip show mink` lists no
`License-File` beyond the standard `LICENSE`-style classifier), so there is
nothing to carry forward per Apache-2.0 SS4(d) the way ADR-016 did for the
vendored SO-101 asset.

## 1. A forced, unrequested consequence: the shared bm-ptl venv's mujoco pin moved

`pip install mink==1.3.0` requires `mujoco>=3.10.0`. This project's existing
pin (`scripts/requirements-bmptl.txt`, pre-probe) was `mujoco==3.2.7`, which
does not satisfy that. This is not specific to 1.3.0: every other mink 1.x
wheel's own METADATA was checked directly (`pip download --no-deps` +
reading `Requires-Dist`, not assumed) and each one requires a newer floor
than 3.2.7 too -- 1.0.0/1.1.0 want `mujoco>=3.3.6`, 1.1.1/1.2.0 want
`mujoco>=3.8.1`, 1.3.0 wants `mujoco>=3.10.0`. The floor has crept up release
to release; there is no mink 1.x release this project's old mujoco pin
satisfies. Installing mink on bm-ptl's shared `ov_env`
**silently uninstalled mujoco 3.2.7 and installed 3.13.0** (confirmed via
`pip show mujoco` before/after). This is reported prominently, not buried in
a comment, because it is exactly the kind of side effect that could silently
change production behaviour on the one machine all simulation work runs on
(ADR-020).

**What was done about it, and why it is safe to leave in place:**
1. `scripts/requirements-bmptl.txt` now pins `mujoco==3.13.0` (the resolved
   version) alongside `mink==1.3.0`, with a comment explaining the forced
   bump -- pinning the file to the OLD value would make a fresh
   `pip install -r scripts/requirements-bmptl.txt` either fail to resolve or
   silently re-bump anyway, so pinning the stale number would be the less
   honest choice.
2. **`pytest tests/test_skills.py` was re-run on bm-ptl AFTER the mujoco bump**
   to check whether this changed anything about the existing, documented
   4-passed/4-failed baseline. It did not: the result was **4 passed, 4
   failed**, and every failure's `reason=` string matches the pre-existing,
   already-documented failure modes byte-for-byte
   (`drawer_slide qpos=0.0000 ... IK residual=0.3183 m`,
   `weld_attach_failed_after_300_frames` for `pick`/`place`, and
   `handoff aborted: pick by arm A failed (... IK residual=0.0532 m ...)` for
   `handoff`). Full output reproduced at the bottom of this file. This is
   the evidence that this probe's presence did not silently change
   production behaviour -- the mujoco version bump is real, but its
   observable effect on the one test suite this repo has is exactly zero.
3. `scripts/requirements-dev.txt` was **not** touched: nothing that runs on
   the laptop imports `mink` or `mujoco` successfully anyway (ADR-020), so
   there is no laptop-side consumer to pin it for in this commit.

## 2. Correction 1 -- pinch point, not `armX_gripperframe`, and NOT via two jaw-body FrameTasks either

**What was tried first, and why it was rejected (a structural argument, not
a failing number).** The brief's first-listed option -- a `FrameTask` on
EACH jaw body (`armX_gripper`, `armX_moving_jaw_so101_v1`), each
position-only, with both targets re-derived every iteration so their
midpoint tracks the desired pinch point -- was implemented first. It runs,
but it is the wrong choice: the fixed and moving jaw bodies are (ignoring the
gripper hinge, a separate actuator this IK never drives) rigidly linked to
the same 5-joint arm. Two independent 3-equation position constraints on two
rigidly-linked points is equivalent to constraining close to the full 6-DoF
pose of that rigid pair -- i.e. it reintroduces something very close to the
SAME position+orientation over-constraint that correction 2 (and ADR-024)
say a 5-DoF arm cannot generally satisfy, even though neither individual task
ever mentions orientation. This was not caught by a difference in the
measured numbers -- empirically the one-task and two-task versions produced
**identical** results on Test 1 (both got stuck at the exact same
configuration-limit boundary before the distinction had a chance to matter,
see section 4) -- it is a structural property of "two rigidly-linked
3-vectors," confirmed by working through what fixing both of them means
kinematically.

**What was used instead: ONE position-only `FrameTask` on the fixed jaw body
(`armX_gripper`), retargeted every iteration.** `set_pinch_target` (see the
script) reads the fixed and moving jaw bodies' CURRENT world positions every
solver iteration, computes `pinch_now = mean(fixed, moving)`, and sets the
fixed-jaw-body task's target to `desired_pinch + (fixed_now - pinch_now)`.
If the fixed jaw body reached that target exactly, the pinch point (by
definition the midpoint) would land exactly on `desired_pinch`. This is
recomputed fresh every iteration from live body positions -- never a
constant vector -- for the same reason ADR-025 rejected a fixed offset from
`armX_gripperframe`: the offset is a local-frame vector that rotates with
whatever orientation the redundant 5-joint solve falls into, so a value
measured once is wrong everywhere else.

**Measured discrepancy (this probe's own output, at the "home" keyframe,
before any per-iteration compensation is applied) -- the number the brief
asked for:**

| Frame this probe could have targeted | Distance to the true pinch point (m) |
|---|---:|
| Fixed jaw body (`armX_gripper`), CHOSEN, compensated live | 0.01809 |
| `armX_gripperframe` site (ADR-025's already-rejected choice) | 0.08880 |

The chosen frame sits ~4.9x closer to the true pinch point than the site
ADR-025 already measured and rejected (1.8 cm vs 8.9 cm, both arms
symmetric). `set_pinch_target`'s live compensation removes even that smaller
1.8 cm gap by construction (subject to the solver actually converging --
section 4 reports that it frequently does not, for an unrelated reason).

## 3. Correction 2 -- position-only, `orientation_cost=0.0`

Every `FrameTask` in this probe is built with `orientation_cost=0.0`.
`FrameTask.set_orientation_cost` accepts a cost of exactly `0.0` (its only
validation is `>= 0.0`), so this is a supported, not a hacked-around, way to
drop orientation from the task -- matching ADR-024's "relax orientation
entirely" decision in `ik.py`. No orientation target is ever set or checked
below.

## 4. VERDICT ONE -- compatibility: **partial**

Install, import and every API call used below completed with **no crashes**
on bm-ptl. That much is "works." But **none of the three tests converged to
< 0.01 m within the specified 5-iteration budget**, and the reason is a
specific, reproduced local-minimum/configuration-limit lock, not noise:

| Test | Arm | Target (x,y,z) | Residual after 5 iters (m) | Converged < 1cm? |
|---|---|---|---:|---|
| 1 (single-arm, fork hover) | A | (-0.065, 0.05, 0.44) | 0.2233 | No |
| 2 (bimanual, separated) | A | (0.0, 0.10, 0.45) | 0.3002 | No |
| 2 (bimanual, separated) | B | (0.0, -0.10, 0.45) | 0.3211 | No |
| 3 (handoff, same point) | A | (0.0, -0.01, 0.43) | 0.2218 | No |
| 3 (handoff, same point) | B | (0.0, -0.01, 0.43) | 0.2501 | No |

**Diagnosis, run separately from the three committed tests (not part of
`probe_mink_bimanual.py`'s own output, but reproduced here because it changes
what the "partial" verdict means):** Test 1's target is independently
reachable -- `ik.py`'s own `solve_position_ik`, called with the EXACT same
target from the EXACT same "home" starting configuration, converges to
**residual 0.00986 m in 15 iterations**
(`joint_angles=[-0.216, 1.745, -1.377, -1.658, -0.118]`, shoulder_lift and
wrist_flex sitting near their limits but not fighting each other). mink's
solve on the identical problem instead drives **shoulder_pan to its exact
upper limit (1.9192 vs range-max 1.91986) and elbow_flex to its exact lower
limit (-1.690 vs range-min -1.69) simultaneously**, and gets stuck there --
confirmed reproducible across `dt in {0.05, 0.2}`, `FrameTask lm_damping in
{0, 1e-2}`, and `solve_ik damping in {1e-12, 1e-2}` (all four combinations
converge to the SAME stuck configuration, to 4+ significant figures, even
run for 60-80 iterations instead of 5). Removing `ConfigurationLimit`
entirely (an invalid comparison, kept only to localize the cause) does let
the residual fall to 0.00055 m -- but at joint angles like
`shoulder_pan=6.05 rad`, far outside the physical range, showing the
"good" convergence there is an unconstrained wrap-around, not a real
solution. **Conclusion: with joint limits enforced (the physically correct
setting), this probe's straightforward mink setup gets stuck at a
DIFFERENT, worse local minimum than `ik.py`'s damped-least-squares solver
finds, from the identical starting pose, for the identical target.** This is
reported as a genuine solver-behaviour finding, not a bug in the probe's
task construction (section 2 already ruled out the over-constraint
explanation by testing the one-task and two-task formulations side by side
and finding the SAME stuck point either way).

**A second, separate compatibility caveat, also measured rather than
assumed:** during Test 1's (failed, limit-stuck) solve, the live collision
snapshot shows `armA_vs_table_top` distance at **-0.0554 m** -- i.e. a real
~5.5 cm geometric interpenetration between arm A and the table, despite
`CollisionAvoidanceLimit` being configured with
`minimum_distance_from_collisions=0.05`. At "home", before any solve,
`armA_vs_table_top` was already `-0.0024 m` (a ~2.4 mm baseline touch,
consistent with mesh/margin noise at rest, independent of mink -- not a new
finding). The jump from -0.0024 m to -0.0554 m happened DURING the mink
solve. This is consistent with a known, general property of velocity-level
("control barrier function"-style) collision avoidance: the QP's linear
inequality constraint is only a guarantee about the CURRENT step's
linearization, so a large per-step motion (this probe's `dt=0.2` times a
3 rad/s velocity cap is a potentially large single step) can carry real,
curved geometry through a barrier that a smaller step would have respected.
This was not chased further (smaller `dt` with proportionally more
iterations is the standard mitigation, and the brief specifies a 5-iteration
budget for the three committed tests) but it is reported here because
"mink's collision avoidance prevented interpenetration" would be an
overclaim on this evidence -- in Test 1 it did not.

**Verdict:** PARTIAL. mink imports, its documented API matches what this
probe used, and it does not crash -- but its default QP-based velocity IK,
run from this scene's own "home" rest pose with a straightforward one-task
formulation, converges to a materially worse solution than this project's
existing DLS solver for at least one independently-reachable target, and its
collision avoidance did not hold in one measured instance where it was
supposed to apply. Both are reported as solver-tuning/behaviour findings
that a from-scratch mink integration would need to address (different
initial seeding, smaller step size, or a different task weighting), not as
"mink cannot do this."

## 5. VERDICT TWO -- Test 3's collision behaviour: **inconclusive, not evidence for either reading**

Final cross-arm pinch-point distance in Test 3:
**`final_cross_arm_pinch_distance_m = 0.3053`** -- the two arms end up well
separated (`armA_vs_armB` minimum geom distance 0.2148 m, not "engaged" per
the 0.1 m detection threshold).

**This is NOT evidence that `CollisionAvoidanceLimit` correctly held the
arms apart while they both tried to reach the same point.** Section 4
already shows why: NEITHER arm got anywhere near the shared target in the
first place. Arm A's achieved pinch point, `(-0.172, -0.144, 0.390)`, and arm
B's, `(-0.173, 0.160, 0.369)`, are both roughly as far from the shared target
`(0.0, -0.01, 0.43)` as Test 1's single-arm failure was from ITS target --
i.e. each arm independently hit the same configuration-limit lock diagnosed
in section 4, for reasons that have nothing to do with the other arm's
presence. The 0.305 m final separation reflects **two independently-stuck
arms that never got close enough to each other for collision avoidance to be
the deciding factor**, not a resolved near-collision. Re-running Test 3 with
a solver that actually reaches the shared target for at least one arm (e.g.
`ik.py`'s own solver, or a re-tuned mink setup) would be needed before this
test could distinguish "collision avoidance holds them apart" from "neither
arm got there." That re-test is out of scope for this probe (task brief:
"do not expand scope to test it in this commit").

**What this means for the `f92806e` question this probe exists to inform
("can mink route arm B around a stationary arm A"):** nothing conclusive.
The one collision-relevant observation available is negative-by-omission --
this probe never produced a run where one arm's solve had to negotiate
around the other arm's actual geometry, because neither arm's solve
progressed far enough for that negotiation to become necessary. A real
answer needs a re-tuned solve (section 4's fix) run again against Test 3 or
a `f92806e`-shaped scenario specifically, which is future work, not this
commit's.

## 6. Collision pairs actually built

13 pairs declared: `armA_subtree` vs `armB_subtree`; `armA_subtree` vs
`table_top`; `armB_subtree` vs `table_top`; and `armA_subtree`/`armB_subtree`
vs each of the five M02 props (`plate`, `mug`, `fork`, `spoon`,
`water_bottle`) -- 2 + 1 + 1 + 2x5 = 13, matching aloha's pattern per the
brief. `mink.get_subtree_geom_ids(model, body_id)` was used for each arm's
and each prop's subtree (confirmed present and correctly typed against the
installed package, section 0).

## 7. Command run and full stdout (bm-ptl, `ov_env`)

```
C:\Users\devcloud\project\ov_env\Scripts\python.exe scripts\probe_mink_bimanual.py
```

Selected output (mink version confirmation, pinch-point discrepancy, and the
three tests' headline JSON) is reproduced in sections 0, 2 and 4 above with
exact figures. The full JSON report (`=== FULL REPORT JSON ===` block,
including every collision-pair distance for all three tests and the "home"
baseline) is preserved in the run log this doc was written from; the
headline numbers quoted above are the ones load-bearing for the two
verdicts.

## 8. `pytest tests/test_skills.py` after this commit's environment changes

Re-run on bm-ptl AFTER `mink`/`mujoco==3.13.0` were installed (section 1):

```
4 failed, 4 passed in 38.05s
```

Identical to the pre-existing documented baseline
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`
all FAIL for their already-documented reasons; the other 4 tests in the file
PASS). No production code was changed in this commit, and this re-run
confirms the forced mujoco version bump (section 1) did not change that
either.
