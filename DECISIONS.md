# DECISIONS

User-ratified decisions, newest first. Each entry mirrors an ADR in
`ARCHITECTURE.md` section 4, which holds the full context, options and
consequences. Where the two disagree, `ARCHITECTURE.md` is authoritative and the
disagreement is a defect to be fixed.

ADR-001 through ADR-015 were authored by the planner and live only in
`ARCHITECTURE.md`. This file starts at ADR-016, the point at which decisions began
being ratified by the user rather than proposed.

---

## ADR-062 — Redesign branch verdict: `master` ships unchanged; `redesign` archived as a measured design alternative, hypothesis partially confirmed and rejected on net functional evidence

**Recorded:** Sept 15, 2026 · **Branch:** `redesign` (archived, not merged) ·
**Closes:** ADR-058 (workspace measurement), ADR-059 (base separation + home
keyframe), ADR-060 (eight-skill retest), ADR-061 (swept-path gate) ·
**Full report:** `docs/hardware/redesign-verdict.md`

**Hypothesis.** Day 1 geometry — 0.5 m base separation (ADR-021), prop
placement (ADR-026) — was fixed before the reachable workspace was ever
measured. Four persistent skill failures were hypothesised to be downstream of
that placement rather than of skill logic. Worth testing because ADR-057 had
already excluded the solver: 32 random restarts from ±0.15 rad converged to
identical residuals at four separate targets, i.e. genuine kinematic
boundaries, not local minima.

**Confirmed, for two of three walls.** At 0.40 m separation, `pick(A, mug)`'s
residuals no longer plateau (master: hard boundary at 0.0532 m) and
`handoff(B→A, fork)` became bimodal (master: hard boundary at 0.0954 m). Both
were real walls and both dissolved. Stage 1 also established structurally why
the handoff corridor had always been fragile: at 0.5 m separation there is **no
contiguous both-arms-reachable region of 10 × 10 cm at any height** in
0.35-0.59 m; the largest is 4 × 32 cm, thin along y — the separation axis.

**Rejected, on net functional evidence.** `place(A, water_bottle, table)`
plateaus at ~0.0150 m regardless of separation — genuinely irreducible. And the
net result is **1 of 8 skills passing versus master's 4 of 9**, with three
previously-working skills regressing on waypoint-1 approach collisions.

**The gate gap.** Stage 2's five-item gate verified endpoint reachability and
handoff co-occupancy but never the swept approach path. Geometry can be
reachable at every endpoint and still collide en route: Stage 4 measured **31 of
41 waypoints colliding**, category (b) arm-vs-table dominant by count (18,
mostly ~6 mm grazes) and category (a) arm-vs-prop holding the worst single
violation (water bottle, −0.098 m). A placement gate must test the trajectory,
not the endpoints.

**The finding that closes the approach, and the reason this is not merely
running out of budget.** A home pose optimised against the swept-path gate
improved the gate score from 5/7 to 3/7 violations and **regressed functional
success from 1/8 to 0/8**, every skill failing at GRIP. Cause: the gate cannot
see grasp orientation, because `ik.solve_position_ik` is position-only by
design (ADR-024; the SO-101's 5 pose-controlling joints cannot generally
satisfy a 6-DoF task, ADR-016). A configuration can be collision-optimal and
grasp-useless simultaneously. **Optimising geometry against a position-only
metric can move you away from working grasps**, and further placement search
cannot fix that while orientation is uncontrolled. The remedy was reverted
rather than kept for its better gate score.

**Instrument corrections that prevented false results.** (1) Stage 1's
`table_top` penetration filter falsely flagged `pick(A, fork)`'s own working
grasp target at **−0.39 m**; the raw cloud was declared authoritative, without
which every downstream stage would have inherited a carved-away workspace.
(2) The swept-path gate needed multi-seed endpoint sampling (the redundant
5-DoF-to-3-DoF null space drifts to different, sometimes colliding, branches)
and an implausible-penetration bound — the convex-hull mesh artifact of ADR-035
and ADR-058 appearing a **third** independent time — before it could pass
two-sided validation against known-good behaviour. (3) Stage 3 re-scored 200
home-pose candidates and found Stage 2's adopted pose ranked **91st of 117**
passing; it was swapped per rule.

**Decision.** `master` ships unchanged at `20e1012`. `redesign` is pushed and
preserved — not merged, not deleted; the branch is the evidence. Across all
five stages `skills_scripted.py` changed by exactly one line
(`HANDOFF_POSITION_XYZ`, geometry not logic) and `grasp.py`, `ik.py`,
`executor.py`, `env.py` were never touched, so every skill result was measured
against unchanged control code.

**A continuation would require** orientation-aware IK — a 6-DoF task spec, or a
constrained position-plus-approach-axis mode leaving roll free. The SO-101
cannot generally satisfy full 6-DoF, so this needs solver task-modes or
different hardware. Out of scope; recorded for whoever continues.

---

## ADR-061 — Redesign Stage 4: a validated swept-path collision gate built and hard-stop-checked against ground truth; 8-skill/41-waypoint diagnosis found 31 collisions, dominant category arm-vs-table by count; one geometry remedy (home-pose re-search scored by the gate) attempted, found to REGRESS real functional success (1/8 -> 0/8) and reverted; six-item gate 0/8, functional retest unchanged at 1/8

**Ratified:** Sept 15, 2026 · **Branch:** `redesign` · `master` unchanged at
`20e1012`.

**Step 1.** `scripts/check_swept_path.py`'s `swept_path_clear` interpolates
in JOINT SPACE between a waypoint's start `qpos` and one IK solution at its
target (never Cartesian-interpolate-and-resolve, per the task's explicit
instruction that the latter models a different motion than the real
controller executes), evaluates RAW `data.contact` at every step (never
the ADR-058-distrusted `table_top` mesh filter), and reports contact pairs
by geom name. Passed the mandatory ground-truth validation: on master's
geometry, `pick(A, fork)` waypoint 1 reports CLEAR (worst_penetration
-0.0019 m); on redesign's current geometry, the SAME check reports a
COLLISION (worst_penetration -0.0114 m, cross-arm: `armA_wrist` vs
`armB_upper_arm`/`armB_shoulder`) matching the collision CLASS Stage 3's
real 500-step run independently reported. Two refinements were added
beyond the literal single-IK-solution spec, both because the literal
version measurably failed the ground-truth check otherwise (a hard-stop
risk, not a style choice): (a) `num_endpoint_seeds=8` — a single
unperturbed joint-space interpolation is provably collision-clear for
`pick(A, fork)` waypoint 1 (0/101 steps), while the REAL closed loop (same
start/target) drifts through the position-only solver's un-regularized
null space (ADR-024) to a different branch and stalls in a genuine
collision by step 168 — sampling several starting-seed perturbations (seed
0 always unperturbed) is needed to see this; (b) `implausible_penetration_m
=-0.15` — raw contact data occasionally reproduces the SAME convex-hull
mesh artifact magnitude ADR-058/035 already documented (-0.39 m, -0.222 m)
at certain interpolated poses; flagged as `suspected_artifact` and excluded
from pass/fail rather than trusted at face value, a magnitude-only
sanity bound, not a reintroduction of the distrusted filter.

**Step 2.** `scripts/diagnose_swept_path.py` monkeypatches
`skills_scripted._run_waypoint`/`_run_dwell`/`run_pick` (captures the real
waypoint chain, never edits the frozen file) to check the FULL intended
approach sequence of all 8 skills regardless of where each currently fails
functionally: **31 of 41 waypoints collide.** Category tally: (a)
arm-vs-prop 13, (b) arm-vs-table 18, (c) arm-vs-arm 7, (d) self-collision
6. **Dominant by count: (b) arm-vs-table** — 18 occurrences, nearly all a
shallow, consistent ~-0.006 m graze recurring at almost every
descend-to-grasp/destination waypoint regardless of (x, y) position (a
static-finger-pad-vs-table signature, not a single prop's placement).
Flagged explicitly: count alone hides that (a) contains the single most
severe violations measured (`pick(A, water_bottle)`'s wrist ramming
`water_bottle_body` by -0.098 m, consistent across every sampled seed,
already past `CRUSH_THRESHOLD_M`) and (c)/(d) concentrate almost entirely
in `handoff`'s transfer-point approach and the first hop from home,
consistent with ADR-058's own too-narrow-shared-band finding.

**Step 3.** `scripts/verify_stage4_gate.py` extends Stage 2's 5-item gate
with item 6 (full approach-sequence swept-path clear from home, reusing
Step 2's own capture). On Stage 3's geometry: **0/8** — materially
stricter than the 1/8 functional pass rate, since the gate's multi-seed
sampling flags theoretical null-space branches the real single-seed run
does not necessarily hit. Remedy attempted:
`scripts/search_home_keyframe_stage4.py` re-runs `search_home_keyframe.py`'s
own `evaluate_candidate` UNCHANGED plus a new swept-path score (300
candidates tried, 187 passed static criteria, score distribution
`{2:3, 3:47, 4:62, 5:54, 6:14, 7:7}` violations-of-7-targets at search
fidelity). Best candidate, full-fidelity re-verified: 3/7 violations (down
from Stage 3's fold's 5/7) — fork, spoon and BOTH handoff directions newly
clear. **Regenerated into the scene and retested for real: functional
success REGRESSED 1/8 -> 0/8**, every skill now failing at GRIP
(`weld_attach_failed_after_300_frames`), including `pick(A, fork)`, whose
specific collision the new fold WAS measured to fix. Root cause: the
swept-path gate scores end-effector POSITION only; `ik.solve_position_ik`
never controls orientation (ADR-024), so a different home fold changes
which orientation the redundant solver lands on at each target even when
the position is identical and collision-clear — **a real, previously
unknown blind spot in position-only swept-path gates.** REVERTED (confirmed
`src/bimanual/sim/assets/so101_dual_table.xml` restored byte-identical to
redesign's committed geometry); the attempted fold is recorded in a comment
in `scripts/gen_dual_scene.py`, not silently discarded. No further
geometry change was attempted after this regression, given the freshly
demonstrated risk of iterating against a gate with a known blind spot and
this stage's remaining time. **Six-item gate: 0/8 on both geometries** —
short of the task's own ≥6/8 target and of master's 4/9 functional count;
per the task's explicit rule, reported as the signal to stop, not to keep
iterating.

**Step 4.** Full 8-skill retest, seed 0, oracle mode, `cameras=None`, fresh
env + fresh executor per skill (ADR-047) — identical harness to Stage 3.
Because Stage 4's net geometry change is nil (the one attempted change was
reverted), results are, as expected and confirmed by direct re-run,
byte-identical to Stage 3's: **1/8 pass.** `git diff master..redesign --
src/bimanual/control/skills_scripted.py` still shows exactly the one
`HANDOFF_POSITION_XYZ` line; `grasp.py`/`ik.py`/`executor.py`/`env.py`
confirmed zero diff against master. Full tables and per-diagnostic detail:
`docs/hardware/redesign-stage4.md`.

**Consequences, stated plainly.** A real, validated gate-coverage gap
(swept-path collision) that Stage 2 could not see is now built, checked
against ground truth, and used to diagnose all 8 skills' full approach
sequences, not just the 3 that happened to fail Stage 3's functional test.
The one geometry remedy attempted was measured, not assumed, to make real
behaviour WORSE despite improving the gate's own score — a finding worth
having even though it did not move the headline number, because it
identifies a class of fix (position-only geometry search) that cannot be
trusted alone without a grasp-orientation check this stage's frozen files
put out of scope. Net numbers: functional 1/8 (unchanged from Stage 3, still
short of master's 4/9), six-item gate 0/8 (short of the ≥6/8 target).

**Source:** this commit ("Redesign Stage 4: swept-path gate, geometry
re-placed (ADR-061).").

---

## ADR-060 — Redesign Stage 3: home pose swapped after a 200-candidate robustness check; `HANDOFF_POSITION_XYZ` moved to Stage 2's measured centroid; eight-skill retest — 1 of 8 pass, 3 regressions, 2 boundaries resolved to a different mechanism, 1 boundary reconfirmed

**Ratified:** Sept 15, 2026 · **Branch:** `redesign` · `master` unchanged at
`20e1012`.

**Step 0.** ADR-059's home fold was accepted after only 2 candidates were
ever scored. Re-scored 200 (`scripts/validate_home_pose_stage3.py`, reusing
Stage 2's own `evaluate_candidate`, not reimplemented) without stopping at
the first pass: 117/200 pass (58.5%, not near-degenerate). ADR-059's own
pose ranks 91st of 117 by total-residual margin — bottom quartile. Per this
stage's rule, swapped to the best of 200 (index 11, total residual 0.01780
m vs. ADR-059's 0.02451 m); `verify_stage2_gate.py` re-run: **ALL 5 ITEMS
PASS**.

**Step 1.** `HANDOFF_POSITION_XYZ` moved from `(0.0, -0.01, 0.43)` to
`(-0.035, 0.0, 0.49)` — Stage 2's own measured feasible centroid at 0.40 m
separation, resolving the file-freeze contradiction ADR-059 flagged but
could not resolve. Confirmed a single-line diff. Isolation test (corrected
from the task's own verification text, which would have wrongly expected
`frames_used=6610` to hold): with master's XML plus the new constant, the
three constant-independent skills reproduce byte-identically (0.3989 /
0.3588 / 0.6192); `handoff(A→B, fork)` changes as expected (lateral 0.1946
m → 0.0399 m, `frames_used` 6610 → 3655) and now fails against the OLD
geometry — correct, not a defect, since the constant was measured for the
NEW geometry.

**Step 2.** Eight-skill retest, seed 0, oracle, fresh env + fresh executor
per skill (ADR-047). `cameras=[]` as literally specified raises
`AssertionError` at the frozen `executor.py:212`; used `cameras=None`
instead (functionally identical, matches every other oracle-mode script in
this repo). **Result: 1/8 pass** (`pick(A, water_bottle)` only) vs. 4/9 on
master (`open_drawer` excluded, removed in Stage 2). **3 regressions**
(`pick(A, fork)`, `place(A, fork, table)`, `handoff(A→B, fork)`), one root
cause: a waypoint-1 approach **collision** (cross-arm + arm-vs-table) at an
otherwise fully IK-reachable pose — the 0.40 m separation's shared
workspace band is only ~14x15 cm, and Stage 2's 5-item gate checked only
static endpoint configs, never the swept approach path. A real,
newly-surfaced gate-coverage gap.

**Multi-seed diagnostics (ADR-056/057, `num_seeds=32`)** on all 4 distinct
failure root causes: `pick(A, fork)` waypoint 1 residuals 0.0029-0.0099 m
(no plateau — never an IK problem); `pick(A, mug)` waypoint 1 residuals
0.0056-0.0106 m, no plateau, vs. master's own documented 0.0532 m at this
target — **reachability fixed**, new blocker is a downstream grip/weld
mechanism failure; `place(A, water_bottle, table)` destination residuals
flat at 0.01502-0.01509 m across all 32 seeds — a textbook plateau,
**genuine boundary reconfirmed** (master: 0.0138 m, same signature);
`handoff(B→A, fork)` phase 1 residuals bimodal (half ~0.004-0.010 m, half
~0.030-0.031 m) vs. master's 0.0954 m identical across 32/32 seeds — **now
reachable by multiple configurations**, the one actually driven to brushes
the plate by 0.3 mm, a local-minimum/routing signature, not a kinematic
wall.

**Consequences, stated plainly.** Headline pass count regressed, 4/9 (master)
to 1/8 (redesign) — a real cost, traced to a newly-introduced collision
class Stage 2's static-endpoint gate could not have caught. Underneath that
headline: two previously-hard IK boundaries (mug pick, handoff B→A phase 1)
are now confirmed reachable, blocked by more tractable problems instead;
one boundary (water bottle place) is confirmed genuine on two different
geometries by the same method. No skill logic changed to chase these
numbers, per this stage's explicit rule. `grasp.py`, `ik.py`, `executor.py`,
`env.py` confirmed byte-identical to `a80ad5f`. Full tables and
per-diagnostic detail: `docs/hardware/redesign-skill-retest.md`; full ADR
text: `ARCHITECTURE.md` ADR-060.

---

## ADR-059 — Redesign Stage 2: base separation reduced 0.50 m -> 0.40 m by sweep; geometry replaced inside the measured workspace; home keyframe re-derived by search; drawer removed after a measured collision; supersedes ADR-021/ADR-025/ADR-026 on `redesign` only

**Ratified:** Sept 15, 2026 · **Branch:** `redesign` · `master` unchanged at
`20e1012`.

The axis question was resolved by tracing code, not inference:
`scripts/measure_workspace.py:291`'s `grid[y, x]` construction means
ADR-058's "4 x 32 cm" band is 4 cm in **Y** — the same axis the arm bases
are separated along — so a base-separation sweep was the correctly-targeted
instrument.

`scripts/sweep_base_separation.py` swept separation from 0.50 m down to
0.15 m (N=300000/arm, scratch scenes deleted after every use, none
committed) and found **0.40 m is the LARGEST separation with a contiguous
both-arms region >= 12x12 cm above z=0.40 m** (0.50 m and 0.45 m both have
zero qualifying slices, confirming ADR-058 was not an artifact of the exact
heights it tested).

`ARM_GAP_Y` moved 0.25 -> 0.20. All five props shifted in Y by their
assigned grasping arm's own base delta (preserves each prop's reachability
margin by construction — confirmed empirically, not just argued).
`HANDOFF_POSITION_XYZ` was **not** updated: the task text asked for it to
be set in `gen_dual_scene.py`, but it actually lives in the frozen
`skills_scripted.py` — flagged as a contradiction rather than resolved by
breaking either rule; the existing constant still passes at the new
separation. The drawer is **removed**: raising it to tabletop height
compiled but collided with three props' rest positions (13 contacts >1 mm,
measured), and there was no time left under the 120-minute cap to relocate
and re-verify. The home keyframe was found by search
(`scripts/search_home_keyframe.py`): ADR-026's old fold failed at the new
separation (candidate 0), the very next randomly-drawn candidate passed
every criterion (zero self/cross-arm/table/prop contact, all prop and
handoff IK targets <0.005 m).

**Verification gate, `scripts/verify_stage2_gate.py`, ALL FIVE ITEMS
PASS**, including item 5 (both arms' handoff-converged configs applied
simultaneously, zero cross-arm contact below -0.005 m) — the condition
Step 1's sweep does not test and the task flagged as the likeliest
failure.

Full sweep table, before/after positions, IK residuals and the drawer
collision detail: `docs/hardware/redesign-geometry.md`. No skill was run in
this stage; `verify_adr038_skills.py` is not expected to reproduce master's
numbers here and was not chased, per this stage's own rules.

**Source:** this commit ("Redesign Stage 2: base separation 0.40 m,
geometry placed inside measured workspace, home keyframe regenerated
(ADR-059).").

---

## ADR-058 — Redesign Stage 1: empirical per-arm workspace measurement; RAW cloud authoritative over the table_top penetration filter; NO contiguous 10x10 cm both-arms region exists at any tested height, at the current base separation

**Ratified:** Sept 15, 2026 · **Branch:** `redesign` · **Measures only —
changes nothing in `src/`, `scenes/so101/`, or any generated file.**

`scripts/measure_workspace.py` sampled N=50000 configurations per arm,
uniformly over each of the 5 IK-controlled joints' actual `model.jnt_range`
(ADR-016: the jaw joint contributes nothing to pose and is excluded),
recording the ADR-025 pinch point after `mj_forward`. Per-sample cost was
measured on a small batch first (0.063-0.065 ms/sample) and projected
(6.4 s total) before committing to the full run, per the task's own
instruction — the full run took 3.4 s + 5.0 s, far under the 90-minute cap.

Two things were checked before trusting the result, both because the task
explicitly required it:

1. **The `table_top` collision filter.** A known mesh-collision artifact
   (ADR-035 Part 3, a -0.22211 m false-positive penetration depth on a
   converged, sensible pose) was sanity-checked against four independently
   known-good targets. `fork_at_rest` for arm A — a target `pick(A, fork)`
   picks successfully every regression run — reported a **-0.39050 m**
   false penetration and was rejected by both the naive 1mm bar and a loose
   2cm bar. The filter is not trusted; the **RAW (unfiltered) cloud is
   authoritative** for every number in the report.
2. **Sampling density.** The primary N=50000 intersection grids were
   visibly sparse (salt-and-pepper), risking a false "no shared region"
   read from Monte Carlo gaps rather than real gaps. A 6x-denser
   cross-check (N=300000, same method, fresh seeds) confirmed the shared
   region is real and stable in shape: a long, thin band (short axis
   2-9 cm, long axis up to 46 cm) at every one of 13 tested heights
   (z = 0.35-0.59 m), never a square.

**Result: no contiguous both-arms-reachable region of at least 10 cm x
10 cm exists at any tested height, at the current base separation.** The
largest is ~4 cm x 32 cm (z=0.51 m). This does not contradict
ADR-032/035/048's near-single-point findings — this measurement checks a
strictly weaker condition (each arm's own reach alone, no cross-arm
collision check at all) and even that already fails the 10x10 cm bar
everywhere; adding the missing collision constraint back can only shrink
the true handoff-feasible region further. The conclusion for Stage 2: the
shared region's short axis is fundamentally too narrow at any height
tested, before any collision constraint is even applied — no per-prop or
per-target repositioning fixes this; relocating one or both arm bases is
the change with the leverage to widen it. No placement decision is made
here.

Regression gate confirmed unchanged on bm-ptl:
`scripts/verify_adr038_skills.py` → `0.3989 / 0.3588 / 0.6192 / 0.1946`,
`frames_used=6610`; `pytest tests/test_skills.py` → 4 passed / 4 failed,
same tests, same reasons.

---

## ADR-057 — Multi-seed IK diagnostic: every genuine IK-convergence failure measured is Case 1 (variance ≈ 0, one basin — a genuine boundary, not rescuable by multi-seed); a fifth target's documented "failure" turns out not to be an IK failure at all — Commit 3's retry-with-perturbed-target wrapper is not supported by this data

**Ratified:** Sept 15, 2026 · **Follows:** ADR-056 (the `num_seeds`
random-restart capability this diagnostic exercises via five separate
targets, unmodified by this commit) · **Diagnostic only — no skill or
solver change.** New file: `scripts/probe_multiseed_diagnostic.py`. Full
account, per-target tables and the raw 32-value residual lists:
`docs/hardware/m11-multiseed-diagnostic.md`.

**Why a new script instead of just calling `solve_position_ik(...,
num_seeds=32)` again.** `solve_position_ik` returns only the BEST of its
internal restarts — there is no way to recover the other 31 residuals
from its return value. The probe reconstructs each perturbed starting
configuration itself (a scratch `MjData` with the target arm's 5 joints
overridden) and calls `ik.solve_position_ik(..., num_seeds=1)` from each
one — so every one of the 32 residuals is still produced by `ik.py`'s own
unmodified DLS loop, just invoked from outside 32 times instead of once
from inside. The RNG seed derivation and the per-seed `uniform(size=5)`
draw sequence are reproduced verbatim from `ik.py`'s own formula, so the
distribution measured is provably the SAME one `num_seeds=32` samples
internally (`numpy.random.default_rng`'s stream is sequential and
deterministic, so slicing one 32-long list at `[:1]/[:8]/[:32]`
reproduces `num_seeds=1/8/32` exactly).

**Five targets measured, applying the task's own three-case interpretation
rule (variance ≈ 0/one value = genuine boundary; variance > 0 with a
sub-threshold best = local minimum, rescuable; variance > 0 with every
mode still above threshold = multi-modal-but-infeasible, not rescuable by
multi-seed but worth recording the spread for a target-perturbation
retry):**

| target | documented residual | Case |
|---|---:|---|
| (a) `place(A, water_bottle, table)` destination approach | 0.0138 m (ADR-034/056) | **1 — genuine boundary** (sanity-check match to ADR-056's own published number) |
| (b) `pick(A, mug)` waypoint-1 approach | 0.0532 m (this repo's own currently-failing `test_handoff_mug_ends_held_by_arm_b`) | **1 — genuine boundary** |
| (c) `handoff(B→A, fork)` Phase 1 pick approach | 0.0954 m (ADR-037) | **1 — genuine boundary** |
| (d) chain Phase 3 `to_arm` approach | 0.0875 m (overnight-batch-log.md, ADR-054) | **1 — genuine boundary** |
| (e) `pick(A, water_bottle)` at ADR-053's 7 failing seeds (10-19 range) | `weld_attach_failed_after_300_frames` | **not applicable** — IK already converges (residual < 0.010 m) at EVERY seed and EVERY `num_seeds` level; the documented failure is downstream, in the grasp/weld mechanism, not in IK |

At (a)-(d), variance across all 32 restart seeds is 9-21 orders of
magnitude below any physically meaningful scale, and every seed —
including, for (c) and (d), the ACTUAL terminal qpos of a real, physically
driven 500-step waypoint attempt, not merely an analytic starting state —
converges to the identical residual. (d) is the target most likely a
priori to show pose-history-dependent local-minimum behaviour (it exists
ONLY because ADR-054's `already_held` guard skips Phase 1, per
overnight-batch-log.md's own "likely mechanism" paragraph) and it does
not.

**(e) is a distinct, fourth finding, not a Case-1/2/3 instance.**
`docs/hardware/m08-extended-eval.md`'s own seed table's `frames_used=1300`
at every one of the 7 failing seeds is arithmetically waypoint 1 (≤500)
plus waypoint 2 (≤500) both already converging, then GRIP (300) exhausting
its budget with no weld attach — confirmed directly here: waypoint 1's own
one-shot residual is below 0.010 m at every seed, at every `num_seeds`
level (though with real, non-noise variance, 1e-6 to 1e-7, unlike (a)-(d)
— up to 32 genuinely distinct converged values). Neither multi-seed
(perturbs the solver's start) nor a target-perturbation retry (perturbs
the target) can influence a grasp-mechanism failure that only engages
after the arm has already arrived at a converged pose.

**Verdict: Commit 3's retry-with-perturbed-target wrapper is NOT supported
by this data.** Zero Case-2 or Case-3 instances were found across all five
targets measured. Proceeding would require a NEW, not-yet-run measurement
that actually perturbs a target position and finds a lower residual —
this diagnostic did not attempt that (out of scope: measurement only,
starting-point perturbation only, per ADR-056's own mechanism).

**Regression gate: PASSES, byte-identical before and after, on bm-ptl.**
`scripts/verify_adr038_skills.py`: `0.3989 / 0.3588 / 0.6192 / 0.1946`,
`frames_used=6610`, unchanged. `pytest tests/test_skills.py`: 4 passed / 4
failed both runs, including the identical `IK residual=0.0532 m` failure
this ADR's own target (b) reuses. Expected: neither `ik.py` nor
`skills_scripted.py` was touched — only a new, non-imported-elsewhere
probe script and a new doc were added.

**Not done.** No change to `ik.py`, `skills_scripted.py`, `grasp.py`,
`executor.py`, `env.py`, `randomization.py`, `scenes/so101/`,
`gen_dual_scene.py`, `SUBMISSION.md`, or any requirements file.
`num_seeds` remains un-wired into any skill's call site.

**Source:** this commit ("Multi-seed IK diagnostic: which failures are
local minima (ADR-057).").

---

## ADR-056 — Optional random-restart (multi-seed) capability added to `solve_position_ik`; `num_seeds=1` regression gate reproduces byte-identical on bm-ptl before/after; `num_seeds=32` measured against ADR-034's known-failing water-bottle place target finds NO improvement

**Ratified:** Sept 15, 2026 · **Follows:** ADR-024 (the DLS solve wrapped
here, unmodified), ADR-025 (pinch-point targeting, unmodified), ADR-035
(the `2115a1e` warm-start diagnostic's local-minimum finding, 0.14633 m →
0.00956 m across one 2 cm grid step — corrected here from the task
brief's own "ADR-045" citation for this finding, which is actually M10
Phase 4's unrelated PoseNet conversion entry), ADR-034 (the specific
known-failing target this ADR's own measurement reuses).

New keyword-only `num_seeds: int = 1` and `seed_noise_rad: float = 0.15`
on `solve_position_ik`; `IKSolution` gained `winning_seed: int = 0`
(defaulted, its one existing construction site unaffected). Seed 0 is
always the unperturbed current configuration, run through the exact
pre-existing statement sequence with no RNG constructed at `num_seeds=1`
— both runtime callers (`skills_scripted.py:873`, `:1208`) call
positionally with no keyword args and cannot reach either new parameter.
Seeds 1..N-1 perturb this arm's 5 joints only by `U(-seed_noise_rad,
+seed_noise_rad)` rad (clipped to `jnt_range`), reusing the identical DLS
loop; the lowest-residual result across all seeds wins. The RNG seed is
derived from pure integer arithmetic on `ord(arm)` and the target's
micron-rounded xyz — deliberately not Python's `hash()`, which salts
strings per-process (`PYTHONHASHSEED`) and would break cross-machine
reproducibility (ADR-047's own named risk).

**Regression gate: PASSES, byte-identical on bm-ptl before/after**
(modified `ik.py` `scp`'d into the working tree pre-commit, so the AFTER
gate ran before this commit existed; only then committed and pushed from
the laptop). `scripts/verify_adr038_skills.py`: fork z 0.3560 → 0.3989,
place final z 0.3588, bottle z 0.4400 → 0.6192, handoff lateral 0.1946 m,
`frames_used=6610` — all unchanged. `pytest tests/test_skills.py`: 4
passed / 4 failed both runs, including the identical 0.0532 m residual in
the same failing assertion.

**Measurement, not a fix: `num_seeds=32` (CuRobo's own cited default) on
ADR-034's `place(A, water_bottle, table)` destination-approach target
(`[0.300, 0.00077, 0.430]`, x on the place path's own `+0.30` clip bound)
finds NO improvement** — residual 0.0138 m at both `num_seeds=1` and
`num_seeds=32`, `winning_seed=0` (none of 31 extra restarts beat the
unperturbed seed), matching ADR-034's own figure exactly. This is
evidence AGAINST the local-minimum reading at this specific point (more
consistent with ADR-034's own clip-bound observation of a true reach
limit) — not proof, since a wider noise radius or a different restart
basis was not tried. New `scripts/probe_ik_num_seeds32.py`; not wired
into `place` or any other skill, which still fails exactly as ADR-034
documented.

**References (user-supplied, cited as such — not fetched, not described
beyond the claim attributed to each, same discipline as ADR-037/ADR-041):**
MATLAB Robotics System Toolbox,
https://www.mathworks.com/help/robotics/ug/inverse-kinematics-algorithms.html
(random restart as standard practice; CuRobo's `num_seeds=32` default);
https://arxiv.org/pdf/2606.15918 (local-minimum vs. true-reach-limit
signature: genuine unreachability ~10 cm median, vs. this project's
~1.4x-threshold misses); ManiBox, https://arxiv.org/pdf/2411.01850 (IK
baseline context, 68.75% ± 5.10% on full workspace).

**Not done:** no change to `skills_scripted.py`, `grasp.py`,
`executor.py`, `env.py`, `randomization.py`, `scenes/so101/`,
`gen_dual_scene.py`, `SUBMISSION.md`, or requirements files; `num_seeds`
not wired into any skill. Full account: `ARCHITECTURE.md`.

---

## ADR-055 — Perception-in-loop demo: `pick(A, fork)` run end to end with PoseNet driving its grasp-point targeting (M10 Phase 5 wiring, GPU FP16) — PASSES on oracle ground truth, reproducing ADR-046's own 8.4 mm outcome delta to within 0.03 mm

**Ratified:** Sept 15, 2026 · **Follows:** ADR-046 (M10 Phase 5's
targeting/verification split and opt-in wiring — exercised here, not
modified), ADR-045 (the PoseNet OpenVINO IR compiled here, GPU FP16),
ADR-047 (`WeldGrasp`/executor reset fix, relied on implicitly). Number
corrected mid-batch from the brief's original "056" to **055** after
ADR-054 was ratified first (`docs/hardware/overnight-batch-log.md`'s
"Fix F" entry).

New `scripts/perception_demo.py` calls the unmodified
`sk.run_pick(env, "A", "fork", weld=weld, position_provider=provider)` —
the same entry point `ScriptedSkillExecutor` itself would produce — with
`position_provider` a real `PoseNetInference(device='GPU')` +
`CachedPropPositions`. No skill, grasp, IK, executor, environment, or
randomization code was touched. Verification stays on oracle reads
(`env.data.xpos`, `WeldGrasp.is_holding`), independent of `run_pick`'s own
`.success`, per ADR-046 Correction 1: success requires `fork_z > 0.37`
(asserted equal to `TABLE_SURFACE_Z + WELD_PICK_SUCCESS_MARGIN_M`, the
skill's own internal threshold) AND `is_holding('A') == 'fork'`.
Randomization: `env.reset(seed=0)`, **no randomizer** — `ENVELOPES={}`
(ADR-048) makes a bare seed select nothing without one; fixed default was
chosen so this run is the SAME scenario ADR-046's own oracle-vs-PoseNet
table measured, for a direct comparison.

**Result: PASSES.** fork z 0.3560 → 0.3905, `is_holding('A')=='fork'`,
`frames_used=1655`, wall clock 1.487 s. Two delta quantities, kept
separate (a first-draft conflation caught before commit): the
perception-ESTIMATE delta at the one refresh (fork 2.19 mm, matching
ADR-046's own 2.2 mm at this seed) vs. the OUTCOME delta — this run's real
final z vs. the documented oracle-only baseline (0.3989) — 8.37 mm,
matching ADR-046's own headline "8.4 mm" figure to within 0.03 mm.
`cache refresh_count=1`, `inference_count=1` (one targeting read, per
`cached_access.py`'s own one-render-per-generation contract). Supplementary
`benchmark(n_runs=100)` on the same compiled model: GPU FP16 mean=0.5903 ms
max=0.6239 ms (ADR-046 measured mean=0.6648 ms, max=7.2228 ms — closely
tracking, not a regression). GPU compiled on the first attempt; the
CPU-fallback branch this script also implements was never exercised.

**Regression gates, re-run this commit, unchanged:** `pytest
tests/test_skills.py` 4 passed / 4 failed (same four tests); `scripts/
verify_adr038_skills.py` 0.3989 / 0.3588 / 0.6192 / 0.1946 m,
`frames_used=6610` — byte-identical to the documented baseline, as
expected since no skill code was touched.

**Video:** `docs/videos/perception-demo.mp4` — 40 PNG frames (`front`
camera, 640x480) rendered on bm-ptl from full-physics-state snapshots
(same observer-wrapper technique as `render_handoff_frames.py`), `scp`'d
to the laptop and encoded there with `ffmpeg` (bm-ptl has no imaging
libraries). Full account: `docs/hardware/overnight-batch-log.md`; full
ADR: `ARCHITECTURE.md`.

---

## ADR-054 — `run_handoff` Phase 1 `already_held` guard (mirrors ADR-034) — pick→handoff re-pick bug fixed, standalone regression gate byte-identical on laptop and bm-ptl before/after; the 3-skill chain still fails, now one phase later, at a NEW Phase 3 cross-arm collision

**Ratified:** Sept 15, 2026 · **Follows:** ADR-034 (`run_place`'s
already-held guard — the pattern copied here, not reinvented), ADR-037/038
(handoff's sequential choreography, unmodified), Fix C's chained-demo
attempt (found this bug, did not touch `skills_scripted.py`, left ADR-054
unratified).

`run_handoff`'s Phase 1 called its nested `run_pick` unconditionally, with
no check for whether `from_arm` already held the object — the exact bug
shape ADR-034 already fixed in `run_place`. In a chained episode
(`pick(A, fork)` then `handoff(A→B, fork)` in the same episode, one
`WeldGrasp`), Phase 1 tried to re-grasp a fork arm A already held;
`grasp.py`'s already-holds gate refused every step, exhausting
`GRIP_HOLD_FRAMES` with `weld_attach_failed_after_300_frames` before
Phase 2 ever ran.

**Fix:** the same `already_held = weld is not None and
weld.is_holding(from_arm) == body_name` guard ADR-034 introduced, applied
to Phase 1 only. When true, the nested pick is skipped (`pick_result =
None`); every downstream `pick_result.<attr>` access is guarded so the
skip cannot crash or misreport. When `weld is None` or the object is not
already held — every standalone `handoff` call — behaviour is unchanged.

**Regression gate, both machines, before and after, byte-identical:**
bm-ptl — `pick(A, fork)` 0.3560→0.3989, `place` final z 0.3588,
`pick(A,'bottle')` 0.4400→0.6192, `handoff` lateral sep 0.1946 m,
`frames_used=6610`. Laptop — same shape at 0.3987/0.3588/0.6191/0.1958 m
(the same pre-existing, ADR-047-documented cross-machine floating-point
divergence, present identically before and after). `pytest
tests/test_skills.py`: 4 passed / 4 failed, same four tests, both
machines, before and after. None of the hard gate numbers moved — the fix
is kept.

**Chain composition, genuinely tested:** `scripts/chained_demo.py`
(unmodified) re-run on bm-ptl. Step 1 (`pick(A, fork)`) PASSES. Step 2
(`handoff(A→B, fork)`) no longer hits the old Phase 1 bug — Phase 1
correctly skips and Phase 2 succeeds — but now FAILS at Phase 3 (`to_arm`
approach): a genuine cross-arm collision while staging
(`from_arm`'s post-Phase-2 pose differs from the standalone case because
Phase 1 no longer runs its own nested pick, so `from_arm` instead carries
over step 1's independent full-budget pick's own final pose). Step 3
(`place(B, fork, table)`) was never reached. The fallback
`place(A, fork, table)` ran and passed this time (fork left in a different
table position than Fix C's own fallback run, which is why Fix C's
arm-vs-mug near-miss did not reproduce here). **Not patched, per this
fix's own instructions** — the new Phase 3 collision is reported as a real
finding, not chased with a threshold or routing change. No video rendered
(chain did not fully succeed). Full account:
`docs/hardware/overnight-batch-log.md`; full ADR:
`ARCHITECTURE.md`.

---

## ADR-053 — M08 extended: 20-seed Track A robustness sweep, same file as ADR-049 — seeds 0-9 reproduce bit-for-bit, `pick(A,'bottle')`'s true rate revises to 45% (9/20), `handoff`'s 20/20 re-disclosed as degenerate everywhere it appears

**Ratified:** Sept 15, 2026 · **Follows:** ADR-049 (original two-track
10-seed eval), ADR-047 (fresh env/executor per trial, unchanged here),
ADR-051 (`handoff` fails 0/5 under the smallest non-placement
perturbation), ADR-046 (`handoff` fails under a 2.2 mm perception offset)
· **Adds:** `docs/hardware/m08-extended-eval.md`; extends
`scripts/eval_m08.py` (`--num-seeds`, default 10 unchanged; a per-trial
thread-based timeout).

**Scope.** Track A only (own-prop randomization), seeds 0-19, all four
working skills — this is what the task brief asked for. Track B
(multi-prop randomization) was not extended and still stands at its
original 10-seed report in `docs/hardware/m08-eval.md`, unmodified.
`scripts/eval_m08.py` was extended, not forked: `--num-seeds` defaults to
10, so `python scripts/eval_m08.py --track both --skill all --out-dir
out/m08_eval` (no new flags) still reproduces ADR-049's original run from
this same file.

**Reproducibility check, the task brief's "genuinely interesting
question," answered explicitly.** Seeds 0-9 inside this 20-seed run were
compared row-by-row against `m08-eval.md`'s original Track A tables for
every skill. Result: **bit-for-bit identical** — same offsets, same
pass/fail pattern, same frame counts, on all four skills, zero exceptions.
No reproducibility failure was found; this is the expected result for a
`ScenarioRandomizer` that is a pure function of `seed`, run through
ADR-047's fresh-env-per-trial harness, and it is reported here as a
checked, positive finding rather than assumed.

**Results, seeds 0-19, Track A, bm-ptl:**

| skill | 20-seed result | ADR-049's 10-seed result |
|---|---|---|
| `pick(A, fork)` | 20/20 | 10/10 |
| `place(A, fork, table)` | 20/20 | 10/10 |
| `handoff(B, A, fork)` | **20/20 — degenerate: envelope is a single point, zero displacement every trial; measures determinism, not robustness** | 10/10 — same qualifier |
| `pick(A, 'bottle')` | **9/20 (45%)** | 6/10 (60%) |

`pick(A, 'bottle')`'s pooled rate drops from 60% to 45% once seeds 10-19
are included (that decade alone: 3/10). Not a reproducibility problem
(seeds 0-9 match ADR-049 exactly, above) — it is what a larger sample
reveals about a smaller, luckier one. **45% supersedes 60% as this skill's
reported rate going forward**, per ADR-018's rule against reporting a
favourable subset as the whole picture. The same non-monotonic, sub-cm
failure structure ADR-049 found persists at double the sample (e.g. seed 11
at `dy=-0.01mm` PASSES beside seed 15 at `dy=+6.32mm`, which FAILS) — ten
more samples of already-known fine structure, not a new finding.

**`handoff`'s 20/20 carries ADR-049's exact qualifier, repeated at every
appearance** (summary table, per-skill section, demo-seed table in
`m08-extended-eval.md`) rather than stated once and left for the reader to
carry forward. All twenty trials are the byte-identical unperturbed
scenario (`frames_used=6610`, `from_arm_retreat_dist=0.2263`, matching
`scripts/verify_adr038_skills.py`'s own numbers). Twenty repeats of one
deterministic scenario is not a bigger robustness sample — it is the same
zero-variance measurement repeated twice as often. Three independent,
already-ratified measurements say the opposite of "robust": ADR-049 Track B
(0/10 when a prop `handoff` never touches is randomized), ADR-051 (0/5 at
the smallest tested arm-angle noise, no prop movement involved), and
ADR-046 (perception-mode failure at a 2.2 mm targeting offset). This
module does not change any of those three findings.

**Per-trial timeout (batch discipline), disclosed as soft, not OS-level.**
Each trial runs on a `threading.Thread` joined with a 300 s timeout; on a
hang the seed is logged as `trial_timeout_after_300s` and the sweep
continues. CPython cannot forcibly kill a thread, so an abandoned hung
thread is not terminated, only isolated (its own fresh, unshared
`env`/`executor`, per ADR-047) — unlike ADR-052's OS-level
subprocess-per-combo isolation. Zero timeouts occurred; the slowest skill,
`handoff`, averaged 17.4 s/trial, well under the 300 s bound.

**Regression gates, bm-ptl, before AND after, both identical:** `pytest
tests/test_skills.py` reproduced `4 passed / 4 failed`, same four tests as
ADR-047/048/049/051/052. `scripts/verify_adr038_skills.py` reproduced
`0.3989 / 0.3588 / 0.6192 / 0.1946`, unchanged.

**`SUBMISSION.md` was explicitly not touched.** Findings that would
otherwise have gone there are logged in
`docs/hardware/overnight-batch-log.md` per the standing overnight-batch
rule to leave that file for morning review.

**Source:** this commit ("M08 extended: 20-seed robustness sweep across 4
skills (ADR-053).").

---

## ADR-052 — M10 batch scaling: PoseNet FP16 throughput vs. batch (1/4/8/16) across CPU/iGPU/NPU via static `reshape()` on the existing IR — all 12 combos succeeded, including NPU at every batch size, contradicting the predicted destructive-crash trigger class

**Ratified:** Sept 15, 2026 · **Follows:** ADR-045 (M10 Phase 4: FP32/FP16 IR,
static batch-1), ADR-050 (INT8 extension) · **Provenance:** all numbers from
**bm-ptl**, per ADR-047's cross-machine float-divergence record — laptop
figures are not reported anywhere in this entry.

**Question.** Does PoseNet's FP16 IR throughput scale linearly, sub-linearly
or super-linearly with batch size (1, 4, 8, 16) on CPU, iGPU and NPU, and
does the NPU tolerate a batch dimension above 1 at all?

**Method, and the reshape-not-reconvert choice.** `artifacts/posenet_ir/
posenet_fp16.xml` (ADR-045) was converted at a STATIC batch-1 shape and
stays that way on disk — it does not become batch-4 by being loaded and
asked for four inferences. `scripts/benchmark_batch_scaling.py` instead
does, fresh per (device, batch) combo: `core.read_model(...)` then
`model.reshape({0: [N, 3, 224, 224]})` **before** `compile_model` — reshape
before compile, not reconversion, and no new `.xml`/`.bin` pair was
produced. 10 warm-up inferences discarded, 100 measured per combo (the
90-minute time cap was never hit, so no combo needed the 50-iteration
fallback). Throughput reported as `batch * 1000 / mean_ms`.

**Subprocess isolation, and why it mattered even though nothing crashed.**
`DECISIONS.md`'s own "M03 — OpenVINO conversion smoke test complete" entry
(not `bmptl-verification.md` verbatim — checked directly against both files
while writing this entry, a citation correction worth recording plainly)
documents that the NPU plugin can kill the whole process
(`STATUS_ACCESS_VIOLATION` / `0xC0000005`) on an unsupported graph, and that
a harness which buffers results in memory loses everything on that crash.
This module's task brief named batch>1 as "exactly the trigger class" for a
repeat of that failure. Every (device, batch) combo therefore ran in its
own `subprocess.run(...)`, order CPU → GPU → NPU with batches ascending
within a device, and every result — success, reshape failure, compile
failure, or a dead child with no result at all — was appended to
`artifacts/posenet_ir/batch_scaling_results.jsonl` the instant it was known.

**Result — the predicted crash did not occur.** All 12 (device, batch)
combos compiled and ran successfully, including NPU at batch 4, 8 and 16.
This does not contradict M03's finding: M03's crash was specifically on a
FULLY-OPEN dynamic batch dimension (`-1`, unbounded upper bound); a static
reshape to a fixed N is a narrower, different case, and on this
NPU5010/driver/OpenVINO-2026.3.1 combination it is tolerated at every N
tested. The defensive subprocess-per-combo/incremental-write discipline was
exercised on every row but never actually triggered by a crash — the same
posture ADR-045 recorded when none of its six combos crashed either.

**Scaling shape.** All three devices scale sub-linearly-to-linearly in
latency vs. batch (latency grows slower than batch size), so throughput
keeps climbing through batch 16 on every device: CPU 156.7 → 231.0 Hz
(1.47x), NPU 887.4 → 1477.6 Hz (1.67x), GPU 1697.0 → 5800.4 Hz (3.42x) —
GPU scales best, consistent with it having the most parallel compute
headroom relative to this model's size, though this script does not
instrument dispatch-vs-compute time separately, so that reading is an
inference from the curve shape, not a directly measured cause. Batch-1
numbers this run land within run-to-run measurement noise of the existing
Phase 4 table (GPU −13.7%, NPU +2.5%, CPU −1.7%), not evidence of
regression. Phase 4's own GPU FP16-internal-execution caveat (Arc plugin
likely running FP16 internally regardless of stored precision) is carried
forward as context for why the GPU curve looks the way it does, not
re-verified here (this script only benchmarks the FP16 IR).

**Regression gate, bm-ptl, before this run:** `pytest tests/test_skills.py`
reproduced `4 passed / 4 failed`, unchanged from ADR-047/048/049/051 —
untouched by this measurement-only work.

**Consequences.** New files only: `scripts/benchmark_batch_scaling.py`,
`artifacts/posenet_ir/batch_scaling_results.jsonl` (gitignored, same
`artifacts/` block Phase 4/INT8 already use). `docs/hardware/
m10-phase4-benchmark.md` gained one new "Batch Scaling Analysis" section;
every prior section, row and finding in that document is unchanged.
`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`scenes/so101/`, `gen_dual_scene.py`, `posenet.py`, `dataset.py`, the
checkpoint and every requirements file were not touched; nothing was
installed into `ov_env` or `train_env`. bm-ptl was synced to this commit's
parent (`c9a65be`) via the pull-only PAT fetch + `reset --hard FETCH_HEAD`
before this script ran, since bm-ptl's push access is not possible (pushes
return HTTP 403) — this commit is pushed from the laptop.

**Source:** this commit ("M10 batch scaling: throughput vs batch across
CPU/GPU/NPU (ADR-052).") — code and benchmark doc land together.

---

## ADR-051 — M06 handoff perturbation diagnostic: home-pose arm-angle noise is not tolerated at any tested magnitude (0/5 at +/-0.005 rad) — no robustness claim beyond exact determinism is made for `handoff`

**Ratified:** Sept 14, 2026 · **Follows:** ADR-049 (M08, disclosed `handoff`
Track A 10/10 as degenerate/determinism, not robustness).

**Diagnostic only** — no change to `skills_scripted.py`, `grasp.py`, `ik.py`,
`executor.py`, `env.py`, `randomization.py`, `scenes/so101/`, or
`gen_dual_scene.py`. New files: `scripts/probe_handoff_perturbation.py`,
`docs/hardware/m10-handoff-perturbation.md`.

**Question.** Does `run_handoff(B, A, fork)` tolerate a small perturbation
other than prop placement — specifically, +/-0.005/0.01/0.02 rad noise on
each arm's home `shoulder_lift`/`elbow_flex`, applied to `qpos` after
`env.reset()` then `mj_forward`, 5 seeds per magnitude, widening only if the
previous magnitude passed all 5?

**A proposed physics-timestep sweep (0.001/0.002/0.003 s) was explicitly NOT
run.** No `timestep` is set in the scene, so the model runs at MuJoCo's
default (0.002 s) and every skill's step budget is a FRAME count tuned at
that default. Varying `dt` without rescaling the frame budget changes how
much simulated time the fixed budget buys, not a physical perturbation of
the skill — it would show "frame-budget-tuned for dt=0.002" (already known
from ADR-046/049), not robustness.

**Result.** A zero-noise control (same direct-call harness) reproduced the
known oracle baseline exactly (success, 6610 frames,
`from_arm_retreat_dist=0.2263 m`, matching ADR-046's own number) — the
harness is sound. At the smallest tested magnitude, +/-0.005 rad, **0/5
seeds passed**; the ladder did not widen further. All 5 failures were
identical to four decimal places (Phase 3 `to_arm` approach, IK residual
0.0875 m, 3155 frames, `cross_arm contacts=1`) despite five different random
noise draws — the same "any nonzero perturbation reproduces the identical
failure" cliff ADR-049 documented for `water_bottle` placement, now shown to
extend to home-pose joint noise as well.

**Decision, against the bar fixed before the run (>10/15 supports a
robustness claim in the demo video, <10/15 means none is made): 0/5 (0/15 of
the possible grid) — well under the bar. No robustness claim is made for
`handoff` beyond the exact determinism ADR-049 already disclosed.** The demo
video may state `handoff` reproduces deterministically at its exact tuned
configuration; it must not claim tolerance to arm-pose noise or to prop
placement beyond the single point ADR-049 measured.

**Regression gate, bm-ptl, before this run:** `pytest tests/test_skills.py`
reproduced `4 passed / 4 failed`, same four tests as ADR-047/048/049.

---

## ADR-050 — M10 Phase 4 extension: INT8 PoseNet quantization via NNCF, benchmarked CPU/iGPU/NPU — 4x smaller than FP32 and 2-4x faster than FP16/FP32, but deviation (36-37 mm) is an order of magnitude above the model's own ground-truth MAE (2.6-3.2 mm)

**Ratified:** Sept 14, 2026 · **Follows:** ADR-045 (M10 Phase 4, FP32/FP16 IR
and benchmark), ADR-013 (precision/device mapping strategy, which said INT8
"targets the NPU5010") · **Adds:** `scripts/quantize_posenet.py`, an "INT8
quantization" section appended to `docs/hardware/m10-phase4-benchmark.md`.
IR artifacts (`artifacts/posenet_ir/posenet_int8.{xml,bin}`,
`int8_calibration_info.json`, `int8_convert_info.json`) are gitignored and
live only on bm-ptl, same block as Phase 4's own artifacts.

This module's task brief marked INT8 a bonus, not critical path ("if it
fails, STOP and report rather than fighting it"). It did not fail.

**Result.** The EXISTING FP32 IR (`artifacts/posenet_ir/posenet_fp32.xml`,
re-verified byte-for-byte against ADR-045's 45,221,444-byte `.bin` before
any new work started, not regenerated) was quantized with
`nncf.quantize(..., target_device=nncf.TargetDevice.NPU)`, calibrated on 300
images sampled without replacement from `data/posenet/images/`
(`numpy.random.default_rng(seed=42)`, exact sample_index list recorded in
`int8_calibration_info.json`), preprocessed identically to training. INT8
`.bin` is 10.82 MiB -- 0.251x FP32, 0.502x FP16, matching the ~0.25x an
INT8-vs-FP32 bit-width ratio predicts.

**Latency (10 warm-up discarded, 100 measured, static batch-1
`[1,3,224,224]`, identical methodology to Phase 4):** CPU 1.616 ms / 619 Hz
(vs FP32/FP16's 6.4-6.5 ms), GPU 0.340 ms / 2939 Hz (vs 0.59-0.68 ms), NPU
0.903 ms / 1107 Hz (vs 1.10-1.28 ms). All three devices compiled and
benchmarked INT8 without a crash; the subprocess-per-device isolation
(ADR-013's M03 lesson, restated in ADR-045) was exercised and never
triggered.

**Correctness -- the honest finding this ADR exists to record.** Max
absolute deviation vs the PyTorch-XPU reference (same 5 validation samples
and same `val_predictions_xpu.npy` array Phase 4 used, reused rather than
recomputed): CPU 36.600 mm, GPU 37.499 mm, NPU 36.391 mm. PoseNet's own
ground-truth MAE is 2.6-3.2 mm per prop, so this deviation is **an order of
magnitude above the model's own error scale**, not "well under" it as
FP16's ~0.17 mm deviation was. Judged against that scale rather than as a
bare number (this module's task brief's explicit instruction), INT8 is not
a free win here -- it would be material to control if this IR actually drove
a skill. This is a plausible outcome, not evidence of a bug: PoseNet
regresses precise millimetre-scale 3D coordinates rather than a
classification logit, and naive INT8 PTQ is well known to degrade
regression heads more than classification heads. Consistency across all
three devices (36.4-37.5 mm, not wildly divergent) supports "the
quantization itself is imprecise for this task" over "a device-specific
bug."

**Dependency install, `--no-deps` first, then only what import needed.**
`pip install --no-deps nncf` installed nncf 3.3.0 but `import nncf` failed
on a missing transitive import. Rather than re-resolving NNCF's full
declared dependency tree, missing imports were added ONE AT A TIME, each
also `--no-deps`, retrying `import nncf` after each and stopping the moment
it succeeded: `packaging`, `rich`, `tabulate`, `psutil`, `safetensors`,
`scipy` (6 packages; success after `scipy`). `pip show nncf` additionally
declares `ninja`, `pydot`, `scikit-learn` as requirements and `pip check`
correctly flags all three (plus `rich`'s own `markdown-it-py`/`pygments`)
as missing -- none was needed by `import nncf` or by a live
`nncf.quantize(...)` smoke test on a trivial model, so none is installed.
`torch.__version__` (`2.14.0+xpu`), `torch.xpu.is_available()` (`True`) and
`numpy.__version__` (`2.4.6`) were verified unchanged before and after
every one of the 7 install steps; per this module's task brief, any change
to either would have been an immediate abort-and-report, and none occurred.
`ov_env` (`scripts/requirements-bmptl.txt`) was never touched; the new
dependency is recorded in `scripts/requirements-train.txt` (the file
ADR-043 created for exactly this).

**On GPU INT8 vs GPU FP16.** ADR-045 found GPU FP32 and GPU FP16 report
byte-identical deviation and near-identical latency, consistent with the
Arc B390 plugin running its internal compute in FP16 regardless of stored
weight precision. If that holds, "GPU INT8 vs GPU FP16" here may compare
INT8 against an already-FP16-internal baseline rather than a genuinely
higher-precision one -- flagged as a caveat, not independently verified
(would require overriding `INFERENCE_PRECISION_HINT`, out of scope here).

**Per-device recommendation.** NPU is ADR-013's originally intended INT8
target and is fast (0.903 ms, competitive with its own FP16 row); GPU is
fastest overall at INT8 (0.340 ms). But given the ~36-37 mm deviation on
all three, **none is recommended as the demo's perception backend as
quantized here** -- the demo path stays on FP16 (ADR-046). This INT8 IR is
a documented benchmark artifact, not adopted. A future pass could try
excluding the regression head from quantization (`ignored_scope`) or a
larger calibration set before reconsidering; neither was attempted, since
this module's task brief scoped calibration-and-measure, not
accuracy-recovery.

**Source:** this commit ("M10 Phase 4 extension: INT8 PoseNet quantization
across CPU/iGPU/NPU (ADR-050).") -- code, benchmark doc and this ADR land
together.

---

## ADR-049 — M08: two-track 10-seed robustness eval — own-prop randomization (Track A) vs. M07 Round 1's multi-prop randomization (Track B); `handoff`'s Track A 10/10 is disclosed as a degenerate measurement, not a robustness result

**Ratified:** Sept 14, 2026 · **Follows:** ADR-047 (cross-trial state-leak
fix, the regression gate this module reproduces byte-identical), ADR-048
(measured `SKILL_ENVELOPES`, the ONLY source this module's envelopes are
allowed to disagree with) · **Adds:** `scripts/eval_m08.py`,
`docs/hardware/m08-eval.md`

**Context.** M08's brief calls for randomizing only a skill's own target
prop over seeds 0-9. Taken literally, that method cannot reproduce M07's
own Round 1 numbers (`docs/hardware/m07-envelopes.md`'s Task 3) — most
importantly `handoff`'s 0/10, which came from randomizing `water_bottle`
(the only prop with a non-degenerate individual envelope) underneath
**every** skill's seeds, including `handoff`, which never targets it.
`handoff`'s own fork envelope is a single point (`dx=0, dy=0`,
`m07-envelopes.md:163`), so an own-prop-only method draws the identical
zero offset every seed and scores `handoff` 10/10 by construction — a
determinism check, not a robustness result, and reporting it unqualified
next to the other three skills' real ranges would be actively misleading.

**Decision.** Run and report BOTH methods, always labelled, never merged
into one table: **Track A** (own-prop, this brief's method, reading
rectangles straight from `randomization.py`'s already-measured
`SKILL_ENVELOPES` — never re-measured) and **Track B** (M07 Round 1's
method, reconstructed exactly: `water_bottle: (0,0,-0.010,+0.010)`, `fork`
excluded, applied under all four skills). Every appearance of Track A's
`handoff` number — table, prose, this ADR — carries the qualifier "envelope
is a single point, zero displacement applied; this measures determinism,
not robustness." `randomization.py`'s shipped `ENVELOPES` stays `{}`,
untouched; both tracks construct their own `ScenarioRandomizer(envelopes=
{...})` locally in the new `scripts/eval_m08.py`, using the class's
existing, documented constructor argument.

**Results (bm-ptl, fresh env+executor per trial per ADR-047; full per-seed
offsets and failure reasons in `docs/hardware/m08-eval.md`):**

| skill | Track A | Track B |
|---|---|---|
| `pick(A, fork)` | 10/10 | 10/10 |
| `place(A, fork, table)` | 10/10 | 8/10 (seeds 5, 8 — mug collision via float coupling) |
| `handoff(B, A, fork)` | **10/10 — degenerate, not robustness** | **0/10 — every seed, identical failure** |
| `pick(A, 'bottle')` | 6/10 (seeds 0,1,2,4 fail) | 6/10 (identical to Track A — same target prop both tracks) |

Track B reproduces M07 Round 1's historical report (10/10, 8/10, 0/10,
6/10, same failing `place_fork` seeds) fresh, today — Round 1 was not a
one-off.

**Cross-prop coupling, confirmed a third time.** ADR-038 (moving the mug
alone broke `handoff` at phase 3, fork untouched) and M07's Commit 2
(0.1 mm-resolution probe: any nonzero `water_bottle` offset reproduces the
identical phase-3 failure) both said a skill can be broken by a prop it
never manipulates. This module's Track B run reproduces the same cliff
independently. Mechanism: MuJoCo recomputes the whole system's
contacts/forces every step, so a prop's position anywhere in the scene can
perturb floating-point rounding enough, over a long rollout, to tip an
already-marginal collision check (`handoff`'s cross-arm corridor) the wrong
way — this is *why* ADR-048's cross-skill-safe intersection collapsed to
empty, and it is an architectural constraint, not a scheduled bug fix.

**Interpretation, stated plainly.** Every PASS here was measured inside
envelopes on the order of ±10-20 mm, tuned to what these skills already
tolerate. The pre-M07/M08 audit measured 0/40 at a coarse ±50 mm jitter.
This is an honest robustness measurement within a narrow, tuned band — not
a general robustness claim.

**Regression gates re-verified, bm-ptl, before and after:**
`verify_adr038_skills.py` reproduces `0.3989/0.3588/0.6192/0.1946`;
`pytest tests/test_skills.py` reproduces 4 passed / 4 failed, same four
tests and reasons as ADR-047/ADR-048.

---

## ADR-048 — M07: fine-grid placement envelopes (7x7, 1 cm step, +/-3 cm), opt-in `ScenarioRandomizer` — measured envelopes for `fork`/`water_bottle` both collapse to a single point once `handoff`'s cross-prop fragility is included, so the shipped randomizer intentionally randomizes nothing this pass

**Ratified:** Sept 14, 2026 · **Follows:** ADR-038 (regenerated scene + four
working skills), ADR-046 (opt-in perception, default OFF — the pattern this
ADR mirrors for randomization), ADR-047 (cross-trial state-leak fix, the
regression gate this ADR must reproduce byte-identical) · **Adds:**
`src/bimanual/sim/randomization.py` (`ScenarioRandomizer`), an opt-in
`randomizer=` argument to `TableSettingEnv.reset()` (`src/bimanual/sim/env.py`),
`scripts/probe_envelope.py`, `docs/hardware/m07-envelopes.md`

**Context.** M07 (PLAN.md) calls for randomizing initial object placement.
The pre-M07/M08 audit (`docs/hardware/m10-pre-m07-audit.md`) had already
found, at a coarse ±5 cm jitter, that all four ADR-038-gated skills tolerate
essentially zero placement perturbation (0/40 aggregate), with a real but
narrow (~1 cm x 0.5 cm, asymmetric) band for `pick(A, fork)` visible only at
finer resolution. This module measures that band properly (7x7 grid, 1 cm
step, ±3 cm range, 5 reps/cell) for all four skills, then builds a
randomizer scoped to whatever the measurement actually supports.

**Correction 1 — randomization must be opt-in, default OFF.**
`scripts/verify_adr038_skills.py:19` and `tests/test_skills.py:76` both call
`env.reset(seed=N)` today and depend on it producing the FIXED baseline
scene (the four ADR-038 numbers; the 4-passed/4-failed pytest baseline).
`TableSettingEnv.reset()` therefore gained a new `randomizer=None` argument
(default `None`) — when omitted, `reset()` is byte-identical to its
pre-M07 behaviour for every `seed`, mirroring ADR-046's
`ScriptedSkillExecutor(inference=None)` default-OFF shape exactly. Verified
on bm-ptl after every change in this commit: `verify_adr038_skills.py`
reproduces 0.3989 / 0.3588 / 0.6192 / 0.1946 exactly; `pytest
tests/test_skills.py` reproduces 4 passed / 4 failed, same four failure
reasons/residuals.

**Correction 2 — the ADR number is 048, not 049** (highest existing was
ADR-047).

**Correction 3 — budget the sweep before running it.** Real per-cell cost
was measured on bm-ptl on a handful of cells first
(`scripts/probe_envelope.py timing`): failing trials ~4.2-4.3 s (500 steps),
passing `pick`/`place` trials ~2.3 s (1655-1656 steps), passing `handoff`
trials ~17 s (6610 steps). Projected total for the full 980-trial sweep:
~75-110 minutes. Measured actual: **~38 minutes** (four full 7x7 sweeps,
cheaper than projected because edge cells are REJECTED, not executed, at
~0 cost). Full detail in `docs/hardware/m07-envelopes.md`.

**Task 1 result (full 7x7 grids, `docs/hardware/m07-envelopes.md`).**
`pick(A, fork)`: 4/29 measured cells passed (`dx in [-0.010,+0.020]`,
`dy=0`). `place(A, fork, table)`: 2/29 (`dx in [-0.010,0]`, `dy=0`) —
narrower than `pick`, as expected (nests the identical pick, plus its own
destination waypoints). `handoff(B, A, fork)`: **1/29 — a single point,
the unperturbed default itself.** `pick(A, 'bottle')`: 8/49, scattered, with
a usable 3-cell rectangle (`dx=0`, `dy in [-0.010,+0.010]`). Rejection used
REAL MuJoCo collision detection (post-`mj_forward` contact penetration
depth), not `generate_posenet_data.py`'s bounding-circle heuristic, which
would have incorrectly rejected cells the shipped scene's own default
already occupies.

**A harness near-miss, caught and corrected, not swallowed.** An
early-stop attempt on `handoff` (permitted by this task's own instructions
for a plausibly-empty envelope) stopped after its first two rows came back
all-fail — WITHOUT ever reaching `dx=0`, the exact point already proven to
succeed by `verify_adr038_skills.py`. This would have reported "empty"
when the truth is "a single passing point." Caught before being written up;
the sweep was re-run in full (no early stop) and found the correct,
single-cell result. Recorded in `docs/hardware/m07-envelopes.md` as a
methodology lesson for any future use of that flag.

**Task 2 result — a cross-prop finding that changed the design.**
`ScenarioRandomizer.ENVELOPES` is the rectangle intersection, per prop,
across every skill that constrains that prop. `fork`'s three-way
intersection (`pick`, `place`, `handoff`) collapses to the single point
`(0,0)`. `water_bottle`'s FIRST-CUT intersection (only `pick_bottle`
targets it) is real and non-degenerate (`dx=0`, `dy in [-0.010,+0.010]`) —
but Task 3's `randomized_eval` found `handoff` (which never touches
`water_bottle`) fails 0/10 when only `water_bottle` is randomized, a binary
cliff confirmed at 0.1 mm resolution (every nonzero offset reproduces the
identical Phase-3 collision; exactly `0.0` reproduces the true baseline
bit-for-bit). Because a global, skill-agnostic randomizer cannot know which
skill will run after `reset()`, and `handoff` is one of the four gated
skills, this constraint was folded into `SKILL_ENVELOPES["handoff"]
["water_bottle"] = (0,0,0,0)` — an explicit, separately-labelled
cross-prop COMPATIBILITY entry, not a target-object envelope — and
intersected the same way as every other entry. **Result: both `fork`'s and
`water_bottle`'s final intersections are single points; `_is_degenerate_point`
excludes both rather than offering a fake always-zero `rng.uniform(0,0)`
"range."** `plate`/`mug`/`spoon` are absent from `ENVELOPES` for the
separate reason that no measured skill targets them at all.

**Net effect: the shipped `ScenarioRandomizer` randomizes nothing.**
`ENVELOPES = {}`. Verified: `randomized_eval` for all four skills against
the shipped (empty) randomizer reproduces 10/10 with `mean_frames_on_success`
bit-identical to the unperturbed baseline's own `frames_used` for every
skill. This is reported as the headline finding, not a null result it
would have been easier to omit — it precisely quantifies how little
placement margin the current `handoff` choreography has, not just to its
own target's position (already known) but to any other prop's position
anywhere in the shared scene.

**A second, non-blocking chaos finding.** `place_fork`'s Round-1 result
(8/10, `water_bottle`-only randomization, before the cross-prop fix) had 2
failures whose root cause was checked directly: `fork`'s own initial `qpos`
is bit-identical across all 10 seeds, and the SAME seed reproduces the SAME
outcome deterministically — yet different seeds' (far-away) `water_bottle`
positions measurably shifted where `place_fork`'s nested pick ended up
gripping the fork, occasionally landing a later waypoint in collision with
`mug`. This is global floating-point coupling in a 3455-step closed-loop
rollout (MuJoCo recomputes the whole system's contacts every step), not a
harness bug — 80% clears this task's own 60% floor, so no remediation was
required, but it is recorded as a caveat for any future harness that
assumes "prop X not randomized" implies "skills targeting X are fully
insulated."

**Verification (bm-ptl, `ov_env`).** `verify_adr038_skills.py` and `pytest
tests/test_skills.py` both reproduced byte-identical to the ADR-047
baseline, checked repeatedly through this commit's changes (including after
`env.py`'s `reset()` signature changed and after `randomization.py`'s
`ENVELOPES` computation changed twice, once mid-investigation).

**Consequences.** M08's future evaluation harness gets a `ScenarioRandomizer`
that is correct and honest about its own current scope (no-op), rather than
one that silently ships a degenerate or unsafe range. Any future work that
wants real randomization range must either fix `handoff`'s underlying
fragility (Phase 1's nested-pick tolerance and/or Phase 3's cross-arm
corridor) or build a skill-aware randomizer (out of this pass's scope,
since `env.py`'s `reset()` hook is necessarily skill-agnostic, matching how
a real seed loop randomizes a scenario before choosing which skill to run).

**Source:** this commit ("M07: fine-grid envelope measurement, deterministic
randomization, preliminary per-skill robustness (ADR-048).").

---

## ADR-047 — `WeldGrasp.reset()` / `ScriptedSkillExecutor.reset()` fix the cross-trial state-corruption bug the pre-M07/M08 audit found — regression gate (all four ADR-038 skill numbers, `pytest tests/test_skills.py`) reproduced byte-identical on bm-ptl

**Ratified:** Sept 14, 2026 · **Closes:** the pre-M07/M08 audit's Zero-th finding
(`docs/hardware/m10-pre-m07-audit.md`) · **Follows:** ADR-029 (weld mechanism),
ADR-030 (weld wiring, `WeldGrasp` reused across calls against the same `env`)
· **Adds:** `WeldGrasp.reset()` (`src/bimanual/sim/grasp.py`),
`ScriptedSkillExecutor.reset()` (`src/bimanual/control/executor.py`),
`scripts/probe_weld_reset.py`

`WeldGrasp.active_welds` is Python-side bookkeeping the class owns; `env.reset()`
resets MuJoCo's own `data.eq_active` (via `mj_resetData`) but has no way to reach
a Python object it does not know exists, so `active_welds` survives a bare
`env.reset()` untouched. Because `ScriptedSkillExecutor` deliberately reuses one
`WeldGrasp` across repeated calls against the same `env` (ADR-030 — correct for
a multi-skill `TaskPlan`), a caller that reuses the same executor across
trials/seeds and resets `env` directly between them — exactly M08's planned
`--seeds 0-9` shape — desyncs the two: MuJoCo believes nothing is welded while
`WeldGrasp` still believes an arm holds something. The next grasp attempt for
that (arm, object) pair is refused at `attempt_grasp`'s "already holds" gate and
surfaces as `weld_attach_failed_after_N_frames` — indistinguishable from a
genuine grasp failure. Left unfixed, this would have silently corrupted M08's
entire 10-seed evaluation the first time any seed's grasp succeeded.

**Decision.** `WeldGrasp.reset()` resets `self.active_welds` to
`{'A': None, 'B': None}` and sets `self.data.eq_active[eq_id] = 0` for every
pre-declared weld this instance resolved at construction — never
`model.eq_active0` (the model's compiled initial value; writing it would mutate
the compiled model and persist beyond a reset, a new side effect this module has
never had). `ScriptedSkillExecutor.reset(env, seed=0, cameras=None)` calls
`env.reset(seed=seed, cameras=cameras)` then, only if `self.weld` is bound to
that same `env`, `self.weld.reset()` — `env.reset()`'s own signature and return
value (the obs dict) are forwarded unchanged. Confirmed `self.data`
(`WeldGrasp.__init__` caches `env.data` once) stays the live buffer across
`env.reset()`: `TableSettingEnv.__init__` assigns `self.data` exactly once
(`env.py:133`); `reset()` never rebinds it, only mutates it in place via
`mj_resetData`. No stale-reference risk found.

**Verification (bm-ptl, `ov_env`).** `scripts/probe_weld_reset.py`: `pick(A,
fork)` succeeds (final_z=0.3989); calling `env.reset()` directly (skipping
`weld.reset()`) reproduces the bug exactly as the audit described —
`is_holding('A')` stays `'fork'` while `data.eq_active[fork_eq_id]` is already
`0`, and a retried `pick(A, fork)` fails with
`weld_attach_failed_after_300_frames`; `executor.reset(env, seed=0,
cameras=None)` then clears it (`is_holding('A') is None`,
`data.eq_active[fork_eq_id] == 0`), and a third `pick(A, fork)` through the SAME
executor succeeds again with final_z bit-identical to the first run
(0.398949 == 0.398949). **Regression gate, all reproduced byte-identical to the
pre-fix ADR-038 baseline:** `pick(A, fork)` 0.3560→0.3989, `place(A, fork,
table)` final z=0.3588, `pick(A, 'bottle')` 0.4400→0.6192, `handoff(A→B, fork)`
lateral separation=0.1946 m; `pytest tests/test_skills.py` 4 passed / 4 failed,
same four failure reasons. Both checks ran on bm-ptl before AND after the fix
(fresh `env`+`WeldGrasp` per skill/test in both scripts, per their own design —
neither exercises the reuse-across-reset path this fix addresses, so an exact
match was the expected, and confirmed, outcome, not a coincidence).

**A cross-machine finding, reported honestly, not silently absorbed.** The same
two regression checks run on the Windows dev laptop (mujoco 3.2.7, same
version) reproduce the SAME pass/fail structure but NOT the same floating-point
digits (e.g. `pick(A, fork)` final_z=0.3987 vs bm-ptl's 0.3989, handoff lateral
separation=0.1958 m vs 0.1946 m) — confirmed present on the laptop with the
UNMODIFIED pre-fix code too, so it is a pre-existing cross-machine
floating-point divergence (contact-solver iteration order over ~1000+ steps),
not a consequence of this fix. The authoritative regression numbers in this
ADR are bm-ptl's, matching the machine the original ADR-038 baseline was
measured on.

**Consequences.** M08's evaluation harness must call `executor.reset(env, ...)`
instead of `env.reset(...)` directly whenever it reuses one executor across
seeds/trials — this ADR makes that the documented, tested way to do it.
`grasp.py`'s and `executor.py`'s existing methods (`attempt_grasp`, `release`,
`is_holding`, `execute`, `_dispatch`) are unmodified; only new methods were
added, which is why the regression gate could not have failed by construction
for `verify_adr038_skills.py`/`test_skills.py` (neither reuses env/executor
across a reset) — the real proof of the fix is `probe_weld_reset.py`, which
specifically exercises the reuse-across-reset path.

**Source:** this commit ("M08 prep: WeldGrasp.reset() fixes cross-trial state
corruption (ADR-047).").

---

## ADR-046 — M10 Phase 5: PoseNet wired into the controller via cached per-skill inference, opt-in, oracle default — 3 of 4 skills PASS with perception on, `handoff` FAILS and is reported not tuned; a per-step render-cost bug found and fixed along the way

**Recorded:** Sept 14, 2026 · **Follows:** ADR-045 (M10 Phase 4, OpenVINO IR +
benchmark), ADR-044 (M10 Phase 3, held-out MAE), ADR-022 (opt-in rendering),
ADR-030/037 (weld wiring, handoff choreography) · **Adds:**
`src/bimanual/perception/inference.py`, `cached_access.py`,
`scripts/verify_m10_phase5.py`, `scripts/run_grounded_demo.py`,
`docs/hardware/m10-phase5-integration.md`. Full measured evidence lives in
that doc; this entry is the decision record.

**Context.** M10 Phase 4 produced a compiled, benchmarked OpenVINO IR that
nothing in the control loop consumed yet. This module wires it in — but the
four scripted skills back the entire 30-point bimanual-completion criterion,
so the wiring had to be **provably non-destructive to the oracle path**
before it could be trusted to add a second one.

**Decision.** Perception is opt-in everywhere, oracle is the unconditional
default:
1. **Targeting vs verification, split by call site, not by module.**
   `run_pick`'s grasp-point read and `run_place`'s destination-offset read
   take an optional `position_provider`; every skill's own success check
   (`final_z`, `final_pos`, `initial_z`) stays a direct `env.data.xpos`
   oracle read, unconditionally, in both modes — a skill must never grade
   itself with the same estimate it acted on.
2. **A held object never gets a fresh PoseNet estimate.**
   `CachedPropPositions.get()` returns `None` (fall through to oracle) for
   any prop currently held by either arm's `WeldGrasp`, checked on every
   call. ADR-044 measured PoseNet's z as a per-prop constant (every training
   frame shows a resting prop) — serving that constant for an airborne
   object would inject up to ~180 mm of error (the water bottle's own
   resting-vs-lifted delta, ADR-034). The brief's proposed rule
   ("invalidate after attach or release") is only half-adopted: invalidate
   on **release** (implemented — the object is back at rest), NOT on
   **attach** (deliberately not implemented — the held-object check in
   `get()` makes it unnecessary, and an attach-time refresh would produce a
   FRESH resting-z guess for an airborne object, which is worse than a
   stale one because it is more convincing).
3. **`ScriptedSkillExecutor(inference=None)`** — the default — is
   byte-identical to every pre-Phase-5 call site. Passing a
   `PoseNetInference` opts one executor instance into perception for
   `pick`/`place`/`handoff` targeting only; the camera-set assertion
   branches accordingly (`cameras=None` for oracle, `cameras=['posenet_cam']`
   for perception), and the cache invalidates at the start of every
   `execute()` call.

**A real cost bug found and fixed while verifying this, not before.**
`TableSettingEnv(cameras=['posenet_cam'])` makes that camera the env's
INSTANCE default; `env.step()` with no `cameras=` argument falls back to
that default (ADR-022). Every internal `env.step()` call inside
`skills_scripted.py`'s waypoint/dwell helpers was written under oracle-only
conditions and never overrode this — harmless when the instance default is
`None`, catastrophic once it is `['posenet_cam']`: every physics step of
every waypoint started rendering, not just the one render per skill this
design intended. Measured: `pick(A, fork)` alone did not return in 12+
CPU-minutes before being killed. **Fixed** by passing `cameras=[]`
explicitly at both `env.step()` call sites — verified to change nothing for
oracle mode (`pytest tests/test_skills.py` and
`scripts/verify_adr038_skills.py` both reproduced their exact pre-fix
numbers afterward).

**Result — oracle mode: exactly reproduced the ADR-038 baseline.**
`pick(A, fork)` z=0.3989, `place(A, fork, table)` z=0.3588, `pick(A,
'bottle')` z=0.6192, `handoff(A→B, fork)` lateral sep=0.1946 m — all four
byte-identical. `pytest tests/test_skills.py`: 4 passed / 4 failed,
unchanged.

**Result — vision mode (GPU FP16): 3 of 4 PASS, `handoff` FAILS, reported
not tuned.** `pick(A, fork)` final z=0.3905 (8.4 mm from baseline),
`place(A, fork, table)` final z=0.3579 (0.9 mm — see below for why this one
is nearly untouched), `pick(A, 'bottle')` final z=0.6286 (9.4 mm) — all
within the "~10 mm expected noise" this module's task brief anticipated, all
PASS. `place`'s near-zero delta is not a coincidence: its own targeting read
is mechanically wired to `position_provider.get()` exactly like `pick`'s,
but by the time it runs the object is always already held, so the
held-object fallback (point 2 above) serves it from oracle at runtime every
time — only the upstream pick's few-mm perturbation survives into its final
number. `handoff(A→B, fork)` **FAILS**: `phase 5 (from_arm retreat) did not
clear the 0.1 m gate ... from_arm_retreat_dist=0.0540 m` (oracle baseline:
0.2263 m). This gate is computed purely from site positions against a fixed
world-frame point — no direct perceptual dependency — yet a 2.2 mm
grasp-point perturbation at Phase 1 (the only perceptual input anywhere in
this call) propagates through `handoff`'s ~13-waypoint sequential
choreography (already the most kinematically fragile skill in this repo per
ADR-032 through ADR-038's five-revision history, several margins on what
ADR-037 itself called a "joint-limit knife-edge") and erodes a
0.2263 m clearance down to 0.0540 m. **No threshold, gate, or waypoint
constant was touched to make this pass** — a hybrid fallback (scripted
`handoff`, perception-driven `pick`/`place`) is available and undecided.

**Task 5 (grounder → executor → render), two more pre-existing findings,
neither a perception regression (both reproduce in oracle mode too):**
1. The brief's literal demo sentence does not parse under M05's frozen
   grammar (`hand` alone ≠ `hand off`/`handoff`/`pass`/`give`).
2. Grounding the two-clause grammar-supported form produces `pick` then
   `handoff` as separate `SkillCall`s, and running them in sequence fails —
   `run_handoff`'s Phase 1 unconditionally re-picks the object, with no
   `already_held` branch the way `run_place` has one. Verified directly in
   oracle mode. Out of this module's scope to fix (an M06 skill-semantics
   gap, not a perception issue).

   Worked around by grounding the single grammar-supported clause "Give the
   fork to arm B." (one `handoff` `SkillCall`, `from_arm` defaulting to "the
   other arm") — PASSED: `is_holding('A') is None`, `is_holding('B') ==
   'fork'`. The rendered `docs/images/m10-demo-end-to-end.png` (front
   camera) does **not** clearly show which arm ends up holding the fork at
   this framing — stated plainly, not implied otherwise; the programmatic
   check above is the real verification.

**Consequences.** `bimanual.perception.posenet`'s `import torch` at module
scope meant `inference.py`/`cached_access.py` cannot import `PROP_ORDER`
from it (no `torch` in `ov_env`, the only venv with both `mujoco` and
`openvino`) — both files duplicate the three constants instead, documented
inline as a disclosed departure with no automatic sync. The GPU shader-cache
warm-up cost (a first compile took 12+ CPU-minutes once, a second on the
same machine took 0.349 s) is a real, environment-dependent first-call cost
this document does not fully separate from the step-render bug it was
investigated alongside — flagged for anyone deploying to a freshly imaged
machine.

**Source:** this commit ("M10 Phase 5: PoseNet wired into controller via
cached per-skill inference (ADR-046, GPU FP16 runtime).").

---

## ADR-045 — M10 Phase 4: PoseNet converted to OpenVINO IR (FP32/FP16), benchmarked CPU/iGPU/NPU — all six combos succeeded (including NPU+FP32), one silent-default trap found and fixed, one threshold miss reported honestly

**Recorded:** Sept 14, 2026 · **Follows:** ADR-044 (M10 Phase 3, trained checkpoint),
ADR-043 (`train_env` with torch+openvino coexisting), ADR-013 (precision/device
mapping strategy and the NPU crash-isolation lesson from M03) · **Adds:**
`scripts/posenet_to_openvino.py`, `docs/hardware/m10-phase4-benchmark.md`. IR
artifacts (`artifacts/posenet_ir/`) are gitignored (`.gitignore:50`,
"Build artifacts (OpenVINO IR, reference tensors)") and live only on bm-ptl.

**Result.** `checkpoints/posenet_best.pth` (43.2 MiB, weights-only load via
`torch.load(..., weights_only=True)`, 11,310,153 params) converted to FP32 IR
(43.13 MiB `.bin`) and FP16 IR (21.56 MiB `.bin`, exactly 0.500x). Benchmarked
CPU/GPU/NPU x FP32/FP16 -- 10 warm-up inferences discarded, then 100 measured,
static batch-1 `[1,3,224,224]` throughout -- plus a PyTorch-XPU baseline with
the identical methodology, `torch.xpu.synchronize()`-guarded around both the
warm-up and the timed region (XPU kernel launches are asynchronous; without
the sync the timed region would measure launch overhead only). Headline mean
latency / throughput: **XPU (PyTorch) 3.59 ms / 278 Hz, CPU 6.4-6.5 ms / ~154
Hz, GPU 0.59-0.68 ms / ~1.5-1.7 kHz, NPU 1.10-1.28 ms / ~780-910 Hz.** Full
table with min/median/p95/std in `docs/hardware/m10-phase4-benchmark.md`.

**All six (device, precision) combos compiled and ran without a crash,
including NPU+FP32.** ADR-013's M03 lesson (an unsupported NPU graph does not
raise a catchable exception, it kills the whole interpreter with
`STATUS_ACCESS_VIOLATION`) was taken seriously here even though the static
`[1,3,224,224]` shape was not expected to trigger it: every (device,
precision) combo ran in its own `subprocess.run(...)`, in the fixed order
CPU -> GPU -> NPU, with every result appended to `results.jsonl` the instant
it was known, never buffered until the end. NPU ran last, after CPU's and
GPU's rows were already on disk. This defence was exercised on every
combo and never actually triggered by a crash this run -- recorded as a fact
about this run, not as evidence the defence was unnecessary to build.

**NPU accepted an FP32 static-batch-1 graph.** This module's task brief
flagged NPU+FP32 as a plausible capability limit (NPUs are typically
FP16-oriented) and its threshold table omitted it accordingly. On this
NPU5010 build it compiled and inferred successfully (deviation 1.653e-4 vs
the PyTorch-XPU reference) -- reported for the record since no threshold is
defined for it, not scored pass/fail.

**Two things found and handled honestly rather than smoothed over:**
1. **`ov.save_model`'s `compress_to_fp16` parameter defaults to `True`**
   (`help(ov.save_model)`: "Floating point weights are compressed to FP16 by
   default."). A first pass at the export step omitted the argument for the
   intended-FP32 save and produced a 21.56 MiB `.bin` -- byte-identical to
   the FP16 save, i.e. the "FP32" IR was silently FP16. Caught by checking
   the file size against the ~43 MiB an 11.3M-param FP32 dump implies, and
   fixed by passing `compress_to_fp16=False` explicitly for the FP32 save.
2. **GPU FP32 exceeds its own stated threshold**: 1.445e-4 vs the 1e-4
   CPU/GPU-FP32 threshold (a ~1.4x miss, well under the >10x flag margin).
   GPU FP32 and GPU FP16 report byte-identical deviation and near-identical
   latency, consistent with the Arc B390 GPU plugin defaulting its internal
   compute precision to FP16 regardless of the IR's stored weight precision
   (a documented Intel GPU-plugin behaviour, not unique to this model) --
   stated as a hypothesis about *why*, not a measured cause, since this
   script did not override `INFERENCE_PRECISION_HINT` to test it directly.
   Every other combo (CPU FP32/FP16, GPU FP16, NPU FP16) is within its
   threshold.

**Conversion form: brief's named form tried first, verified failing, fell
back per the brief's own instruction.** `input=[('image', [1,3,224,224])]`
was tried directly against the live PoseNet module first and failed with
`RuntimeError: Input for tensor name 'image' is not found.` (openvino
2026.3.1). Fell back to the plain-list form already proven in this repo
(`scripts/ov_smoke.py:181`, `input=list(INPUT_SHAPE)`), which succeeded.

**Process note.** Before this module's work began, bm-ptl's repo was one
commit behind (`7dcaaa6`, missing `9999379`'s ARCHITECTURE.md sync) with a
stale local `origin/master` tracking ref reporting a false "ahead by 103
commits" -- resynced via the documented `fetch` + `reset --hard FETCH_HEAD`
recipe (bm-ptl cannot authenticate a bare `git pull`) before any new work
started. All bm-ptl commands in this module ran in a foreground SSH session
per call, never detached, per this module's SSH-hygiene instruction.

**Source:** this commit ("M10 Phase 4: PoseNet OpenVINO conversion, benchmark
across CPU/GPU/NPU (ADR-045).") -- code, benchmark doc and this ADR land
together, so there is no separate prior commit to cite.

---

## ADR-044 — M10 Phase 3: PoseNet trained on Intel Arc B390, per-prop MAE 2.6-3.2 mm; verified input-dependent, not mean-collapse

**Recorded:** Sept 14, 2026 · **Follows:** ADR-039/040/041 (dataset), ADR-042
(architecture), ADR-043 (train_env + openvino) · **Adds:**
`docs/hardware/m10-phase3-training-log.md`. Checkpoints are gitignored
(`.gitignore:12`, `checkpoints/*.pth`, ~45 MB each) and live only on bm-ptl.

**Result.** 30 epochs, **956.1 s (15.9 min)**, **141.2 samples/sec** on the Arc
B390 iGPU via native `torch.xpu` (no IPEX, ADR-042). Best val loss **0.000011**
at epoch 30. Per-prop MAE at the best checkpoint: **fork 3.2 mm, water_bottle
2.6 mm, mug 2.8 mm** — all inside the "< 0.02 m excellent" band, so Phase 4 can
proceed without an accuracy caveat on the headline number.

**The run was ~4x faster than the 60-90 min estimate.** 15.9 minutes for 30
epochs over 4500 samples. Two contributors: `--num-workers 0` (ADR-043 measured
104.7 vs 53.3 samples/sec, spawn overhead dominating on Windows), and the Arc
B390 handling an 11.3 M-param ResNet18-scale model comfortably.

**Verified genuinely learned, not mean-collapse.** A regressor that outputs the
dataset mean for every input can post a plausible loss; with props randomised
over x,y in [-0.18, +0.18] such a model would score ~90 mm MAE. Tested directly
on 5 validation samples with the best checkpoint:

```
std of PREDICTIONS across 5 samples: [0.1533 0.0672 0.0017 0.0472 0.0981 0.0021 0.0549 0.1108 0.0008]
std of GROUND TRUTH  across 5 samples: [0.1537 0.0673 0.      0.0469 0.0978 0.      0.0558 0.1098 0.     ]
pred_spread / gt_spread = 1.009
```

Predictions vary as much as ground truth does. Worst single-axis error across
those 15 prop-samples is 5.0 mm; most are under 3 mm.

**Curve shape: still improving at the end.** Epochs 1-2 drop steeply, 3-15
oscillate under a high cosine learning rate (train and val move together on the
downswings — not overfitting), 16-30 descend monotonically. **The best epoch is
the final one and val_loss was still falling**, so the model is under-trained
rather than over-trained; more epochs would likely help. Final train 0.000017 vs
val 0.000011 — val *below* train, no overfitting signal anywhere.

**Four caveats that must not be lost when this number is quoted.**
1. **z is not meaningfully predicted.** Ground-truth z std across samples is
   exactly 0: each prop's height is pinned to its own resting value by design
   (ADR-039's correction). Only x and y carry signal, so this is effectively 2-D
   localisation and the MAE should be read that way.
2. **Synthetic, single camera, no augmentation.** One fixed `posenet_cam` pose,
   one lighting condition. These are in-distribution figures, not robustness.
3. **Arms are always at the home keyframe** in every training image. A scene with
   arms mid-motion is out of distribution.
4. **The occluded tail is uncharacterised.** All five sanity samples had
   `visibility_ratio == 1.00`. Accepted samples run down to ~0.44 (ADR-041's
   filter rejects below 0.30), and accuracy there was not measured.

**Process note.** The first training attempt was launched by a builder agent as a
detached background process on bm-ptl; it initialised correctly (device, split,
param count all logged) and then died silently when its SSH session closed,
leaving an empty error log and no checkpoints. The successful run held the
process in a foreground SSH session inside a supervising background job — the
same pattern that carried the 35-minute dataset generation — so it could not be
orphaned.

## ADR-043 — M10 Phase 3 prep: `openvino==2026.3.1` installed into `train_env` alongside torch (for Phase 4's live-PyTorch->OpenVINO conversion), `num_workers` default corrected 4 -> 0 — `ov_env` untouched

**Recorded:** Sept 14, 2026 · **Follows:** ADR-042 (created `train_env`, measured
`num_workers=0` at 104.7 samples/sec vs `num_workers=4` at 53.3 but left the default at 4
pending this correction), ADR-037 (`ov_env`'s load-bearing `mujoco==3.2.7` +
`openvino==2026.3.1` pairing, never installed into) · **Touches:**
`scripts/train_posenet.py`, `scripts/requirements-train.txt`.

**openvino installed into `train_env`, not `ov_env`.** `pip install --no-deps
openvino==2026.3.1` into `C:\Users\devcloud\project\train_env` succeeded on the first
attempt with no re-resolution of torch or numpy. Verified in ONE process, before and
after: `torch.__version__` unchanged at `2.14.0+xpu`, `torch.xpu.is_available()` still
`True`, `torch.xpu.get_device_name(0)` still `"Intel(R) Arc(TM) B390 GPU"`, `numpy`
unchanged at `2.4.6` (both before and after — the likeliest casualty did not occur), and
`openvino.__version__ == "2026.3.1-22476-759c5a6ab8c-releases/2026/3"`. The combined
conversion smoke test used the corrected call
(`ov.convert_model(model, example_input=x)`, matching `scripts/ov_smoke.py:181`'s proven
form) — the brief's own draft call passed a tensor to `input=` instead of
`example_input=`, which is the shape/type-spec parameter, not the traced-tensor one, and
would have raised. Ran clean: `[?,?]` output shape on a trivial `Linear(10, 4)`.

**`--no-deps` did skip one dependency, as the brief predicted.** `pip check` flagged
`openvino-telemetry` as an unmet requirement of `openvino`'s own metadata after the
`--no-deps` install (import itself did not raise or warn — the telemetry code path is
lazy — but `pip check` failed until this was fixed). Installed the single missing package
the same way: `pip install --no-deps openvino-telemetry==2025.2`, matching `ov_env`'s own
pin (`scripts/requirements-bmptl.txt:3`). Resolved to `2025.2.0`. `pip check` then
reported "No broken requirements found", and the combined smoke test above was re-run
after this second install to confirm nothing regressed. `train_env`'s final `pip list`:
torch, Pillow, numpy, openvino, openvino-telemetry, plus the unchanged Intel
oneAPI/SYCL/MKL transitive closure from ADR-042 — no other packages were touched.
`scripts/requirements-train.txt` records both additions with this reasoning; the wrong
target named in the brief, `requirements-bmptl.txt` (which describes `ov_env`, the venv
the brief itself says not to modify), was left untouched.

**`ov_env` re-verified functionally and byte-for-byte unchanged.** `pip list` under
`ov_env`: `mujoco==3.2.7`, `openvino==2026.3.1`, `numpy==2.4.6`, no torch, no Pillow.
`scripts/verify_adr038_skills.py` re-run under `ov_env`: all four skills still PASS with
the same measured numbers on record — `pick(A, fork)` lift +0.0429 m, `place(A, fork,
table)` final z=0.3588, `pick(A, water_bottle)` lift +0.1792 m, `handoff(A, B, fork)`
final separation lateral(y)=0.1946 m / vertical(z)=0.0046 m, frames_used=6610.
`pytest tests/test_skills.py` under `ov_env`: unchanged at 4 passed / 4 failed, same four
failing test names (`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`) for the same already-documented, out-of-scope
reasons (IK convergence/reach limits on the `plate`/`mug`/drawer path, not on the
`fork`/`water_bottle` path the four PASSING skills above cover).

**`num_workers` default corrected 4 -> 0 in `scripts/train_posenet.py`.** ADR-042 measured
0 workers at 104.7 samples/sec versus 4 workers at 53.3 samples/sec on this Windows host
(spawn overhead outweighs parallel-loading gain at this dataset size) but left the CLI
default at 4, unreconciled with its own measurement. That is fixed here: `--num-workers`
now defaults to `0`, with both the module docstring and the argparse help text citing the
two measured numbers directly, so the default is not "corrected" back to a higher value
later by someone assuming more workers is always faster. The `--num-workers` override
remains available and unchanged in mechanism (the `if __name__ == "__main__":` guard this
relies on for Windows `spawn` compatibility is untouched).

**No training was run.** This module is smoke/verification only, per instruction.

---

## ADR-042 — M10 Phase 2: PoseNet architecture + dataset loader + training script, in a NEW separate `train_env` on bm-ptl (native `torch.xpu`, not IPEX) — `ov_env` untouched

**Recorded:** Sept 14, 2026 · **Follows:** ADR-009 (ARCHITECTURE.md — a small trained
vision model keeps the 20-point OpenVINO criterion off the risky ML branch), ADR-037
(the load-bearing `mujoco==3.2.7` + `openvino==2026.3.1` pairing in `ov_env`, and the
mink probe's cautionary tale of an unpinned install silently bumping mujoco), ADR-041
(the 5,000-sample, 3-prop dataset this module trains against) · **Adds:**
`src/bimanual/perception/posenet.py`, `src/bimanual/perception/dataset.py`,
`scripts/train_posenet.py`, `scripts/requirements-train.txt`, `checkpoints/.gitkeep`,
`docs/hardware/m10-phase2-smoke.md`.

**The brief's environment assumptions were checked and found wrong for both machines**
(PIL is not "proven available on bm-ptl" — it is absent from both `ov_env` and the
laptop; that absence is why `generate_posenet_data.py` and `render_handoff_frames.py`
both hand-roll `write_png`). Rather than install torch into `ov_env` — the exact
mechanism by which ADR-037's mink probe silently upgraded mujoco to 3.13.0 — this
module creates a **new, separate venv**, `C:\Users\devcloud\project\train_env`,
containing only torch + Pillow + numpy. `ov_env` is not installed into and is
re-verified functionally unchanged at the end (`pip list` shows no torch/Pillow;
`verify_adr038_skills.py`'s four skills still PASS with the same numbers on record;
`pytest tests/test_skills.py` still reports 4 passed / 4 failed with the same four
failing test names — see `docs/hardware/m10-phase2-smoke.md` section 7 for the full
transcript of all three checks).

**Device path: native `torch.xpu` worked on the first attempt** —
`pip install torch --index-url https://download.pytorch.org/whl/xpu` produced
`torch==2.14.0+xpu` with `torch.xpu.is_available()==True` and
`torch.xpu.get_device_name(0)=="Intel(R) Arc(TM) B390 GPU"`. IPEX
(`intel-extension-for-pytorch`) was never installed and never needed — per the task's
own instruction to try native XPU before the older IPEX path, and native XPU
succeeded outright, so there is no IPEX error to report. `scripts/requirements-train.txt`
pins the exact `pip freeze` (torch's xpu wheel pulls in Intel's oneAPI/SYCL/MKL
runtime automatically as transitive dependencies).

**PoseNet** re-derives (not imports — `scripts/` is not a Python package) the exact
ResNet18-scale backbone from `scripts/ov_smoke.py::build_model` (M03's proven
CPU/GPU/NPU-converting topology), with a global-avg-pool + Linear(512,256)+ReLU +
Linear(256,9) head. Measured 11,310,153 parameters — within the "~11-12M expected"
range. Forward-passed a dummy batch successfully on both CPU and XPU.

**PoseNetDataset** reads the real, committed 5,000-sample dataset
(`data/posenet/images` + `labels`, gitignored bulk / `dataset_meta.json` tracked) per
the schema verified directly against `generate_posenet_data.py`'s own `label = {...}`
construction, not guessed. Deterministic `sample_index % 10` split produced exactly
4500 train / 500 val, and `val[0]`'s loaded labels matched
`data/posenet/labels/sample_00000.json`'s `objects.*.xyz_m` exactly on hand
verification.

**A real Windows-`spawn` hang was reproduced, but in throwaway diagnostic scaffolding,
not in the deliverable.** An ad hoc `DataLoader`-iteration probe written without the
`if __name__ == "__main__":` guard hung for a full 300s timeout at `num_workers=4` —
a live demonstration of exactly the trap the task brief warned about. `train_posenet.py`
itself already carries the guard (required for its own `num_workers` DataLoader) and
its `--num-workers 4` smoke run completed normally in 9.98s; a second, corrected probe
run (guard added) measured `num_workers=0` at 104.7 samples/sec and `num_workers=4` at
53.3 samples/sec over a 10-batch/320-sample window — slower net of a one-time ~4.6s
Windows spawn-startup cost that a 10-batch probe cannot amortize, not evidence that
`num_workers=4` is broken. `num_workers=4` was kept as `train_posenet.py`'s default;
it was not dropped to 0.

**Smoke-tested only, per instruction — no full training run.** Model instantiation +
forward pass, one real dataset sample, and one training batch (loss reported, exits
cleanly) on `--device xpu` and on `--device cpu`, both producing the identical loss
(0.301587) from the same seeded init and batch — a correctness signal that model init,
data loading and the loss computation agree across devices. Full transcript, all
measured numbers, and the `checkpoints/` `.gitignore` verification (`git check-ignore
-v`, read as `source:line:pattern<TAB>pathname`) are in
`docs/hardware/m10-phase2-smoke.md`.

---

## ADR-041 — M10 Phase 1.5 correction: 3-prop label scope (fork/water_bottle/mug), occlusion-ratio visibility filtering via MuJoCo segmentation (colour classification tried and measurably failed first), `posenet_cam` pushed further overhead — supersedes ADR-040's camera pose and label scope, keeps ADR-040's mechanism otherwise unchanged

**Recorded:** Sept 14, 2026 · **Follows:** ADR-016 (frozen upstream asset), ADR-020
(bm-ptl-only MuJoCo execution), ADR-022 (opt-in camera rendering), ADR-038 (a
scene change assumed cosmetic broke `handoff` at phase 3), ADR-039 (M10 Phase
1, superseded), ADR-040 (M10 Phase 1.5, camera pose and label scope
superseded here; its mechanism — dedicated `posenet_cam`, eased rejection
sampling, shuffled placement order — is KEPT, not rewritten) ·
**Modifies:** `scripts/gen_dual_scene.py`, `src/bimanual/sim/assets/so101_dual_table.xml`
(regenerated, `scenes/so101/` untouched), `scripts/generate_posenet_data.py`,
`data/posenet/dataset_meta.json` · **Adds:**
`docs/hardware/m10-scope-reduction-samples.md`.

**Starting state.** ADR-040's own 5,000-sample regeneration was in progress
when it was halted by the orchestrator — `data/posenet/images`/`labels` were
found ~half-populated (2,566 of 5,000) and were deleted, per this task's
explicit instruction to treat the dataset as absent and generate fresh.
Nothing from that partial run is reused.

**Correction 1 — the task brief's proposed camera `xyaxes` reintroduced the
EXACT sign error ADR-040 already fixed once, and the brief also misquoted
ADR-040's shipped value.** The brief stated the "current" `posenet_cam` was
`xyaxes="0 -1 0 -0.3 0 0.95"` — that is the *original, broken* pre-ADR-040
spec, not what is actually committed (`so101_dual_table.xml:402`, which
already reads the corrected `xyaxes="0 1 0 -0.3 0 0.95"`). The brief's NEW
proposal for this pass, `xyaxes="0 -1 0 -0.7 0 0.7"`, was checked by hand
before use (not accepted on faith, the same discipline ADR-040 applied) and
found to have the SAME defect: with `x=(0,-1,0)`, `y=(-0.7,0,0.7)`,
`Z=X×Y=(-0.7,0,-0.7)`, view `=-Z=(+0.7,0,+0.7)` — from `x=0.6`, further out
along `+x` and tilted UP, past the table, not at it. Fixed the same way
ADR-040 fixed it: flip the local x-axis sign to `x=(0,+1,0)`. With
`x=(0,1,0)`, `y=(-0.7,0,0.7)`: `Z=(0.7,0,0.7)`, view`=(-0.7,0,-0.7)` — toward
the table and down at ~45 degrees, from the raised, pulled-in
`POSENET_CAM_POS=(0.6,0,1.1)` this pass intends. `POSENET_CAM_FOVY=45`
unchanged from ADR-040. **Verified by rendering one probe frame before
generating anything** — both `front` and `posenet_cam` probes are in
`docs/hardware/m10-scope-reduction-samples.md`; table, arms and all five
props are clearly in frame, more overhead than ADR-040's own already-working
pose. `front` itself is untouched (byte-identical `FRONT_CAM_*` constants);
`git diff --stat -- scenes/so101/` stays empty (ADR-016).

**MANDATORY re-verification, done before any dataset generation.** All four
skills backing the 30-point bimanual criterion were re-run fresh against the
regenerated scene: `pick(A, fork)` PASS (z 0.3560→0.3989, weld attach frame
1155), `place(A, fork, table)` PASS (returns to z=0.3588), `pick(A, 'bottle')`
PASS (z 0.4400→0.6192, weld attach frame 1155), `handoff(A→B, fork)`
(`run_handoff(env, "B", "A", "fork", weld=weld)`) PASS (held by arm B,
z=0.5498, `from_arm` cleared, retreat 0.2263 m) — identical in kind to the
ADR-038/ADR-040 baseline. `pytest tests/test_skills.py`: 4 passed / 4 failed,
identical to the pre-existing baseline. No regression.

**Correction 2 — 3-prop label scope (task FIX 1).** All five props remain
placed, randomized and rendered (occlusion between all five is part of the
training signal; dropping `plate`/`spoon` from the scene would make the
remaining three trivially unoccluded). Only `TARGET_PROPS = ("fork",
"water_bottle", "mug")` receive a ground-truth label and count toward the
visibility gate below; `plate`/`spoon` stay in-scene as unlabelled
decoration (`decoration_props_in_scene_unlabelled`). Each label's
`positions_xyz_m` is now a fixed `(3, 3)` array in `TARGET_PROPS`' canonical
order (`output_shape: [3, 3]`).

**Correction 3 — the task brief's own description of the visibility metric
did not match the formula it then specified; the FORMULA was right, the
DESCRIPTION was wrong.** The brief described the filter as dropping frames
where "the ratio of visible pixels to bounding-box area falls below 0.3" — a
FILL-FRACTION metric (shape, not occlusion; a thin, fully-visible fork would
fail on shape alone). The formula the same brief then gave,
`full_scene_pixels / single_prop_pixels`, is a genuine OCCLUSION ratio
instead. **This is what is implemented**, described as an occlusion ratio
throughout code, `dataset_meta.json`, and the docs — not as Syn4D's fill
fraction. The 0.30 threshold is a **user-supplied reference to Syn4D**
(https://arxiv.org/pdf/2605.05207) for that numeric value only; no claim is
made about the paper's authorship, method, or other findings beyond the
threshold cited to it (same discipline as ADR-037's ScienceDirect citation).

**Correction 4 — the first implementation of the occlusion ratio (RGB colour
classification) was tried, measured, and found unreliable before being
trusted, and was replaced with MuJoCo's segmentation buffer.** Comparing
rendered pixels against each prop's declared `<material rgba>` measured two
real failures on bm-ptl, on this exact scene: (a) MuJoCo's lighting does not
preserve a material's own colour ratio — `fork`'s declared `(255,38,38)`
rendered with a dominant pixel colour of `(255,80,80)`, an exact-match count
of 1 against a visually obvious ~100+ pixel sliver; (b) widening the
tolerance to compensate then ALIASED with unrelated scene elements —
`water_bottle`'s blue collided with the background checker floor tile's own
rendered blue, and `fork`'s red collided with the table surface's warm tan.
Both measured directly with a pixel-histogram probe, not assumed. Fixed:
`compute_visibility_ratios` now uses
`mujoco.Renderer.enable_segmentation_rendering()`, whose `(H,W,2)` int32
output (channel 0 = geom id, channel 1 = constant `mjtObj.mjOBJ_GEOM`) was
verified on a probe render before use — exact per-pixel geom identity, no
lighting-dependent ambiguity. Classification is by `model.geom_bodyid`
membership (a prop can be >1 geom on one body, e.g. mug = cylinder + handle),
not a single assumed geom id.

**Correction 5 — the "hide other props" mechanism was verified to genuinely
remove them from the render, not assumed.** Other props are hidden for a
solo render by writing a real, far off-screen `(x,y)=(3.0,3.0)` into their
own free-joint qpos and calling `mujoco.mj_forward` — a genuine kinematic
relocation, not an `rgba`/`alpha=0` trick (which can still write depth or
leave faint pixels depending on the renderer path, per the task's own
caution). Verified directly: placing one target in view and teleporting the
other four away, the segmentation buffer contained ONLY that target's own
geom ids in every one of three trials (fork: 156 px, zero elsewhere;
water_bottle: 922 px, zero elsewhere; mug: 616 px, zero elsewhere) — see
`docs/hardware/m10-scope-reduction-samples.md`.

**Verification before the full run, per the task's explicit "measure before
committing to 5000."** A 10-sample run (0/10 rejections, 2.75 samples/s) was
followed by a 100-sample run for a statistically meaningful rejection-rate
estimate: **4/104 total draws rejected (3.85%)**, well under the 30%
go-threshold, at **2.40 samples/s** — projecting the full 5,000-sample run at
**~35 minutes**, far under the "2+ hours" worst case the task flagged as
plausible (visibility filtering costs 1 RGB render + 4 segmentation renders
per accepted sample, but segmentation-mode renders measured cheaper than
full RGB, so the realized per-sample cost came in below a naive 4-5x
estimate over Phase 1's 3.04 samples/s). The global minimum ACCEPTED
occlusion ratio observed across both probes was 0.310 (`mug`, nearly fully
hidden behind `water_bottle` from this camera angle) — 0.010 above the
threshold, direct evidence the gate is doing real work, not passing
everything through. Reported before launching the full run, per the task's
instruction.

**Decision — proceed to the full 5,000-sample run.** Camera correctly aimed
and more overhead than ADR-040's own pose, all four skills plus pytest
baseline unaffected, hide mechanism proven pixel-exact, rejection rate
comfortably under 30% with a projected time far under 2 hours. Full
regeneration overwrites `data/posenet/` in place (same seed `20260914` as
Phase 1/1.5). `dataset_meta.json` gains `target_props`, `output_shape:
[3,3]`, `decoration_props_in_scene_unlabelled`, `visibility_threshold: 0.30`,
`visibility_metric`/`visibility_threshold_reference` (the Syn4D citation),
the new `camera_pos_m`/`camera_xyaxes`, `visibility_rejections_total`/
`visibility_rejection_rate`, and a `supersedes` field naming BOTH Phase 1
(ADR-039) and Phase 1.5 (ADR-040, whose regeneration never completed) by
name. Measured full-run wall clock, rate, and rejection rate are recorded in
`dataset_meta.json` and in `docs/hardware/m10-scope-reduction-samples.md`.
As with Phase 1/1.5, the image/label bulk is not left on bm-ptl after
copy-off; only `dataset_meta.json` stays git-tracked.

**Baseline preserved.** `pytest tests/test_skills.py` re-run on bm-ptl
against the regenerated scene: 4 passed / 4 failed, identical to the
pre-existing baseline. No file outside this ADR's own `Modifies`/`Adds` list
(`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`scenes/so101/`, `README.md`, `SUBMISSION.md`) was touched. ADR-040 itself is
kept unedited above — this entry corrects its camera pose and label scope
going forward, it does not rewrite what ADR-040 recorded as true at the
time.

---

## ADR-040 — M10 Phase 1.5: dedicated `posenet_cam` perception camera separated from `front`'s presentation role; eased rejection sampling (range widened, margin reduced, radii kept); placement order shuffled per sample

**Recorded:** Sept 14, 2026 · **Follows:** ADR-016 (frozen upstream asset), ADR-020
(bm-ptl-only MuJoCo execution), ADR-022 (opt-in camera rendering), ADR-038 (a
scene change assumed cosmetic broke `handoff` at phase 3), ADR-039 (M10 Phase 1
dataset, now superseded) · **Modifies:** `scripts/gen_dual_scene.py`,
`src/bimanual/sim/assets/so101_dual_table.xml` (regenerated, `scenes/so101/`
untouched), `scripts/generate_posenet_data.py`, `data/posenet/dataset_meta.json`
· **Adds:** `docs/hardware/m10-camera-comparison.md`.

**Why.** ADR-039's dataset (M10 Phase 1) rendered through `front`
(`pos="1.55 0 0.85"`), a wide establishing shot built for the M06 handoff demo
video, not a perception viewpoint. Measured directly on the three archived
Phase 1 samples: props occupy only ~10-20 px of a 224x224 frame. Phase 1's
rejection sampler also measured a worst-case 5,054 draws for one prop against
a 5,000-per-prop cap on the full 5,000-sample run — already past the nominal
budget, not comfortably inside it.

**Decision 1 — a SEPARATE `posenet_cam`, `front` never touched.** `front` is
load-bearing for the M06 handoff render pipeline (ADR-038) and four working
skills; per that ADR's own lesson ("should be inert" was the assumption that
broke `handoff` before), this pass adds a NEW camera to
`gen_dual_scene.py`'s hand-authored region instead of widening or repointing
`front`. `front`'s own `FRONT_CAM_POS`/`FRONT_CAM_XYAXES`/`FRONT_CAM_FOVY`
constants are byte-identical before and after this change.

**Decision 2 — the task brief's proposed `xyaxes` was checked by hand, not
accepted, and was found to point AWAY from the table.** MuJoCo cameras look
along local -Z, and local Z = local X cross local Y. The brief's axes
(`x=(0,-1,0)`, `y=(-0.3,0,0.95)`) give `Z=(-0.95,0,-0.3)`, so the view
direction (`-Z`) is `(+0.95,0,+0.3)` — away from the table entirely, from
`x=1.05`. Fix: flip the sign of the local x-axis to match `front`'s own
handedness (`x=(0,+1,0)`, not `(0,-1,0)`): `Z=(0.95,0,0.3)`,
view=`(-0.95,0,-0.3)` — toward the table and angled down, from the intended
closer/lower position. `POSENET_CAM_POS=(1.05,0,0.62)`,
`POSENET_CAM_XYAXES="0 1 0 -0.3 0 0.95"`, `POSENET_CAM_FOVY=45` (MuJoCo's own
default; `front`'s widened 55 was for full-arm headroom this camera does not
need). **Verified by rendering one frame before generating anything** — both
`front` and `posenet_cam` probe renders are in
`docs/hardware/m10-camera-comparison.md`; the table, both arms and all five
props are clearly in frame through `posenet_cam`.

**MANDATORY re-verification, done before any dataset generation.** A camera
element has no geom/mass/collision so it should be inert, but ADR-038 found
"should be inert" was exactly the wrong assumption once before. All four
skills backing the 30-point bimanual criterion were re-run fresh against the
regenerated scene: `pick(A, fork)` PASS, `place(A, fork, table)` PASS,
`pick(A, 'bottle')` PASS, `handoff(A->B, fork)`
(`run_handoff(env, "B", "A", "fork", weld=weld)`) PASS — identical in kind to
the ADR-038 baseline. `pytest tests/test_skills.py`: 4 passed / 4 failed,
identical to the pre-existing baseline. No regression.

**Decision 3 — a second brief claim was also checked and found wrong: the
proposed flat 0.03 m spacing was rejected (kept the radius-based test), and a
THIRD brief claim (fork/spoon radii are "far larger than actual footprint")
was checked and found wrong too.** `gen_dual_scene.py`'s geometry gives fork's
true farthest point as `sqrt(0.08^2+0.012^2)=0.0809 m` (declared `0.08` is
~1mm UNDER, not over) and spoon's as `sqrt(0.068^2+0.012^2)=0.0690 m`
(declared `0.07` is ~1mm over — accurate to the millimetre). Since this is a
circular (isotropic) distance test and props are never rotated, reducing
either below its true reach risks genuine overlap for some relative bearing.
`FOOTPRINT_RADIUS_M` is UNCHANGED from Phase 1. Easing came instead from: (a)
`randomization_range_m` widened from +-0.15 m to +-0.18 m (0.36x0.36 m box,
1.44x the old area), (b) `clearance_margin_m` reduced from 0.015 m to 0.008 m
(still a real, positive gap). Combined, the tightest pair's (fork/spoon)
forbidden-disk fraction of the box drops from ~95% (Phase 1, matching the
observed 5,054-draw worst case) to ~61%.

**Decision 4 — placement order shuffled per sample, not fixed.** Phase 1
always placed in the same largest-first order (`fork, spoon, plate, mug,
water_bottle`) on every sample, so `water_bottle` (smallest radius) was
placed LAST every single time — a systematic bias a trained PoseNet could
pick up as a spurious identity-correlated signal. Fixed: a fresh
`rng.permutation` of the five props is drawn once per sample (same order
reused across any restarts within that one sample).
`placement_order_used` is recorded per sample label;
`dataset_meta.json` records `base_placement_order` and
`placement_order_shuffled_per_sample: true`.

**10-sample verification before the full run.** `placement_draws_stats`:
mean 18.6, max 34 (down from Phase 1's mean 54.8, max 5,054) — comfortably
under the task's <500 target. Pixel extent measured with a standalone
pure-stdlib PNG decoder + largest-connected-component analysis (no Pillow,
no new dependency): `mug`/`plate`/`water_bottle` (compact, blob-like props
not confounded by the arm's own grey housing aliasing `spoon`'s hue) show a
measured ~1.79x mean linear / ~3.2x area increase over Phase 1. `spoon` (and,
to a lesser extent, the very thin `fork`) is EXCLUDED from the quantitative
comparison and disclosed as such: the arm pose is never randomized, so a
fixed-position false "spoon" region (matching the arm's own silver/grey
housing) appeared identically in every sample before a largest-connected-
component filter was added; even after that fix, thin/elongated props are
not something this quick colour-based method can measure with confidence.
Three flagged 2D bounding-box "overlaps" out of 10 samples were checked
against the real label data and found to be a perspective artifact (a tall
prop's silhouette crossing a short, nearby prop's screen region from this
angled camera, e.g. `water_bottle` vs `plate` at true 3D centre distance
0.1107 m against a required 0.098 m minimum) — not a real geometric overlap.
Full detail, both probe renders, and three new sample images are in
`docs/hardware/m10-camera-comparison.md`.

**Decision 5 — full regeneration overwrites `data/posenet/` in place.**
Same seed (20260914) as Phase 1. `dataset_meta.json` gains `phase: "M10 Phase
1.5"`, a `supersedes` field describing exactly what Phase 1 configuration is
being replaced, `base_placement_order`/`placement_order_shuffled_per_sample`,
the new `camera`, `randomization_range_m`, `clearance_margin_m`, and scene
`scene_xml_sha256` (the regenerated scene's own camera addition changes this
hash even though `scenes/so101/` itself is untouched). Measured wall clock,
on-disk size, and the full run's own `placement_draws_stats` are recorded in
`dataset_meta.json` and in `docs/hardware/m10-camera-comparison.md`. As with
Phase 1, the image/label bulk is not left on bm-ptl after copy-off; only
`dataset_meta.json` stays git-tracked (`.gitignore`'s existing
`data/posenet/images/`/`data/posenet/labels/` entries, unchanged from
ADR-039).

**Baseline preserved.** `pytest tests/test_skills.py` re-run on bm-ptl against
the regenerated scene: 4 passed / 4 failed, identical to the pre-existing
baseline. No file outside this ADR's own `Modifies`/`Adds` list
(`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`scenes/so101/`, `README.md`, `SUBMISSION.md`) was touched.

---

## ADR-039 — M10 Phase 1 (PoseNet training data): runtime qpos randomization through `TableSettingEnv`'s opt-in `front` camera, per-prop resting z, sequential rejection-sampled placement; full-joint rejection sampling measurably failed and was replaced

**Recorded:** Sept 14, 2026 · **Follows:** ADR-016 (frozen upstream asset), ADR-020
(bm-ptl-only MuJoCo execution), ADR-022 (opt-in camera rendering), ADR-038 (a moved
prop can break `handoff` at phase 3) · **Adds:** `scripts/generate_posenet_data.py`,
`data/posenet/dataset_meta.json` (tracked), `data/posenet/{images,labels}/`
(gitignored), `docs/hardware/m10-training-data-samples.md`.

**Scope.** M10 Phase 1 only: generate labelled (image, ground-truth-xyz) pairs for
a future PoseNet. No model is defined or trained here, and no inference runs. This
is the dataset that will eventually let the scripted controller's oracle
`data.xpos` prop-position reads be replaced by camera-based inference.

**Decision 1 — render through the existing opt-in camera path, touch neither XML
file.** The `front` camera lives in the GENERATED scene
(`src/bimanual/sim/assets/so101_dual_table.xml`, `gen_dual_scene.py`'s output), not
in the frozen upstream asset (`scenes/so101/`, ADR-016). This script constructs
`TableSettingEnv(cameras=["front"], ...)` and calls `env.render("front")` (the
escape hatch documented in `env.py`) for each sample's capture — it never reads or
writes either XML file directly, and never re-invokes `gen_dual_scene.py`.

**Decision 2 — randomize x,y only, at runtime, via `data.qpos`; z stays per-prop.**
Each of the five props (`plate, mug, fork, spoon, water_bottle`) has its own
resting z baked into the scene's "home" keyframe (0.355 / 0.39 / 0.356 / 0.356 /
0.44 m against a 0.35 m table surface). This script reads each prop's z LIVE off
`data.qpos` immediately after `env.reset()` (not a hardcoded duplicate of
`gen_dual_scene.py`'s position constants) and overwrites only the x,y slots of
each prop's free-joint qpos before `mujoco.mj_forward`. A single shared z (e.g.
the table surface, 0.35) would sink every prop partway into the table slab; this
was flagged before any code was written and never implemented.

**Decision 3 — no Pillow; `write_png` copied with attribution.** `PIL` is not
installed in bm-ptl's `ov_env` and this script does not install it (an unrelated
install is what silently upgraded mujoco during the ADR-037 mink probe). The PNG
writer is copied verbatim from `scripts/render_handoff_frames.py`'s own
`write_png` (itself copied from `scripts/probe_render.py`), a stdlib-only
(zlib + hand-rolled IHDR/IDAT/IEND) encoder. Verified on HxWx3 uint8 input at
224x224 (this module's resolution) before generating any volume of images, via a
10-sample run whose images were copied back to the laptop and visually inspected.

**Decision 4 — sequential, largest-first placement with rejection sampling,
REPLACING a full-joint draw that measurably failed.** Initial implementation drew
all five props' x,y simultaneously from `[-0.15, 0.15]` m and rejected/retried the
whole draw on any pairwise overlap (radii `plate=0.06, mug=0.06, fork=0.08,
spoon=0.07, water_bottle=0.03` m, `+0.02` m clearance, 500 attempts). This failed
on the FIRST real run on bm-ptl: sample 0 exhausted all 500 attempts and raised
`RuntimeError` before writing anything. Diagnosis: five simultaneous pairwise
constraints inside a 0.30 m x 0.30 m (0.09 m^2) box is a tight packing problem —
the fork/spoon threshold alone (0.08+0.07+0.02=0.17 m) forbids a disk of area
~0.091 m^2, i.e. up to the entire box, once the first point lands near centre.
**Fix, not a workaround:** placement is now sequential — largest footprint first
(`fork, spoon, plate, mug, water_bottle`), each prop drawn against only the props
already placed (a 1-point rejection problem, not a 5-point joint one), clearance
margin reduced to 0.015 m, 5,000 draws budgeted per prop, up to 200 whole-sample
restarts if a prop's own budget is exhausted. Re-run after the fix: the 10-sample
verification succeeded with `placement_draws` ranging from single digits to 172
(mean 77.2, max 172) — comfortably inside budget, and the full 5,000-sample run
completed under the same scheme (see `dataset_meta.json` for its own measured
draw statistics).

**Decision 5 — `.gitignore` narrowed, not left as a blanket `data/` ignore.** M01
had gitignored the whole `data/` directory. That would have swept
`dataset_meta.json` (the reproducibility record: seed, ranges, mujoco version,
scene checksum) into the same ignore as the bulk images/labels. Replaced with two
specific entries, `data/posenet/images/` and `data/posenet/labels/`, verified with
`git check-ignore -v` (image/label paths matched; `dataset_meta.json` did not) and
`git status` (meta file shows as trackable; images/labels do not appear at all).

**On reachability, stated once more so it cannot be missed.** ADR-038 found that
moving even a single prop can break `handoff` at phase 3. This dataset's
randomized layouts are consequently expected to include many configurations in
which the scripted manipulation skills would fail — and that is fine, because no
skill is ever executed against any sampled layout here; this is a
perception-only dataset. `dataset_meta.json`'s own `note` field and
`docs/hardware/m10-training-data-samples.md` both say this explicitly, so the
dataset is never later cited as evidence of validated or reachable scene
configurations.

**Baseline preserved.** `pytest tests/test_skills.py` was re-run on bm-ptl before
any of this module's code touched the repo: 4 passed / 4 failed, identical to the
pre-existing baseline (no source file this module is scoped to touch —
`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`gen_dual_scene.py`, `scenes/so101/`, `so101_dual_table.xml` — was modified).

---

## ADR-038 — Handoff render legibility: red fork and lateral retreat ADOPTED; prop repositioning TRIED AND FULLY REVERTED because every prop move broke `handoff` at phase 3

**Recorded:** Sept 14, 2026 · **Follows:** ADR-037 (`311430e`, sequential
choreography) · **Modifies:** `gen_dual_scene.py` (fork colour only),
`skills_scripted.py` (retreat vectors only), `scripts/render_handoff_frames.py`

**Problem.** Four successive renders of the working `handoff(A, B, fork)` failed
to show a handoff. Three root causes were identified by inspecting the images
rather than guessing: (1) the fork was silver-grey (`0.72 0.73 0.76`),
indistinguishable from the off-white plate and the identically-coloured spoon;
(2) the retreat displaced only z, so there was no lateral separation for any
camera to show — measured 0.0197 m lateral against 0.1060 m vertical; (3) the
sequence strip re-centred its camera per panel, destroying the viewer's frame
of reference.

**Fix 1 — red fork (ADOPTED).** `fork_material` → `rgba="1.0 0.15 0.15 1.0"`.
Colour only; geometry, mass and collision untouched, so no skill behaviour
depends on it. `spoon_material` deliberately left silver so only the fork
stands out. This alone made the fork unmistakable in all three candidate
renders.

**Fix 2 — lateral retreat (ADOPTED at 0.10, NOT the proposed 0.15).** The
scalar z-only `HANDOFF_FROM_ARM_RETREAT_CLEARANCE_M` became per-arm vectors
`HANDOFF_A_RETREAT_XYZ = (0.0, -0.10, 0.15)` / `HANDOFF_B_RETREAT_XYZ =
(0.0, 0.10, 0.15)`. The proposed 0.15 **fails**: `handoff` reaches phase 5
holding the fork and then collides cross-arm during `from_arm`'s retreat. A
sweep of the lateral magnitude, all with props at their original positions:

| lateral | handoff | final lateral separation |
|---:|---|---:|
| 0.15 | FAIL — cross-arm collision, phase 5 | — |
| **0.10** | **PASS** | **0.1946 m** |
| 0.05 | PASS | 0.0913 m |
| 0.00 | FAIL — cross-arm collision, phase 5 | — |

0.10 is a genuine interior optimum — it fails on BOTH sides, so it was found by
measurement, not by picking the largest value that happened to work. Final
separation is now 0.1946 m lateral / 0.0046 m vertical, inverting the old
0.0197 / 0.1060 and giving the renders something real to show.
Note 0.00 failing is NOT a contradiction of ADR-037: the old code retreated only
`from_arm`, whereas this code retreats both arms, so zero lateral offset makes
them collide.

**Fix 3 — prop repositioning (TRIED, FULLY REVERTED).** Moving plate, mug,
spoon and bottle to the table corners was intended to declutter the renders. It
broke two skills and was reverted in full:
- `pick(A, bottle)` broke outright — the bottle at (0.20, 0.20) is outside arm
  A's reach (IK residual **0.1503 m** vs a 0.01 m tolerance). Reverting the
  bottle alone restored it (lift 0.4400 → 0.6199).
- `handoff` broke at phase 3 (`to_arm` approach) for **every** prop
  configuration tried: all four moved, bottle-reverted-only, bottle+spoon
  reverted, mug-moved-alone, and plate-moved-alone. Only the fully original
  layout passes.

**This is a finding worth keeping: the handoff corridor is sensitive to scene
composition in a way nothing predicted.** Moving a single prop that the skill
never touches — the mug, at the far corner — is enough to make `to_arm`'s
staging approach fail on a cross-arm collision. The mechanism is not understood
and was not chased; it is recorded here so nobody assumes prop placement is
cosmetically free. `gen_dual_scene.py` was restored from git and the fork colour
re-applied on top, so no stale "moved toward corner" comments survive.

**Fix 4 — fixed-camera sequence (PARTIAL).** `m06-handoff-sequence.png` now uses
one camera across all four panels, so the reference frame is stable. But the
chosen distance (1.2 m) and azimuth put the arms edge-on and too small to read,
and the fork is not visible in it. The strip is committed as-is and flagged:
**it still does not demonstrate the handoff and should not be used as evidence.**
The three stills do.

**Verification (bm-ptl, `mujoco==3.2.7`, seed 0), all four working skills after
the adopted changes:**

| skill | result |
|---|---|
| `pick(A, fork)` | success, z 0.3560 → 0.3989 |
| `place(A, fork, table)` | success, released, z 0.3588 |
| `pick(A, 'bottle')` | success, z 0.4400 → 0.6192 |
| `handoff(A→B, fork)` | success, A=None B='fork', 6610 frames, `from_arm_retreat_dist=0.2263` |

`pytest tests/test_skills.py`: 4 passed / 4 failed, the same four pre-existing
failures, unchanged.

**Note on prior measurements.** `docs/hardware/*` records predate nothing here —
the prop layout is unchanged from `311430e` — but the fork's colour differs, so
any render in those documents shows a silver fork.

**API note carried forward:** `run_handoff(env, to_arm, from_arm, obj)` is
receiver-first. An A→B handoff is `run_handoff(env, "B", "A", "fork")`.

### Addendum, Sept 14 2026 (same day, follow-up pass) — Fix 4 corrected; a
### measurement discrepancy flagged; independent re-verification

**ORCHESTRATOR CORRECTION (supersedes the two items below).**

*On provenance — there is no anomaly.* `cc1a329` was made by the orchestrator
session, not an unknown process. The builder agent working these fixes appeared
to have died (its log had been silent for ~100 minutes after SSH rate-limiting
on the jump host), so the orchestrator took the work over, found that agent's
in-progress edits in the shared working tree, corrected the retreat value from
0.15 to 0.10 on the basis of a measured sweep, verified all four skills, and
committed the result. The agent then resumed and correctly observed its own
prose inside an already-made commit. Its report of the facts was accurate; only
the framing as a provenance irregularity was wrong. Two agents editing one
working tree while the orchestrator commits it is the actual mechanism, and the
attribution line on `cc1a329` is this project's standard one.

*On the 0.1946 m vs 0.0983 m discrepancy — both figures are correct; they
measure different reference points.* Re-measured directly on bm-ptl in a single
run reporting both:

| reference | lateral (y) | vertical (z) | 3D |
|---|---:|---:|---:|
| pinch point — midpoint of `armX_gripper` and `armX_moving_jaw_so101_v1` | **0.1946 m** | 0.0046 m | 0.1946 m |
| `armX_gripperframe` site (`data.site_xpos`) | 0.0983 m | 0.1508 m | 0.1818 m |

The site-to-pinch-point offset measures **0.0888 m on each arm** — precisely the
~8 cm ADR-025 recorded when it retargeted IK away from the site for exactly this
reason. Fix 2's table uses the **pinch point**, which is what actually holds the
object and what ADR-025 established as this project's reference; the addendum
used the site. Nothing is unreproducible: the 3D separations (0.1946 vs 0.1818)
are close, and the two references distribute that distance differently between
y and z because the site sits along each gripper's own axis. Fix 2's figure
stands as written, now with its reference stated explicitly.

*Fix 4's correction below is accepted and verified.* The orchestrator inspected
the regenerated `m06-handoff-sequence.png`: the fixed camera holds a stable
frame, both arms are visible and separated in all four panels, and the red fork
is legible in the final panel. The azimuth=90 occlusion diagnosis is correct —
both arms sit near x≈0 and differ only in y, so that viewing ray puts one
behind the other. The strip is now usable as evidence.

**Provenance note, reported for transparency.** This addendum was written in
a session that found the four fixes above (red fork, 0.10 m lateral retreat,
full revert of prop repositioning) ALREADY present and already committed in
this file and in `skills_scripted.py`/`gen_dual_scene.py`, under a different
commit author/co-author line than this session's own attribution. The code
in that commit is, line for line, the same code this session had
independently arrived at (including this session's own comment prose),
which means the two were not truly independent — this session's own
in-progress edits were committed by another process before this session
finished. This is recorded here rather than silently built on top of,
per this project's own "reported honestly" convention.

**Fix 4 was NOT left in its "PARTIAL... does not demonstrate the handoff"
state above.** That entry's own azimuth=90/distance=1.2 m camera (the
task's suggested starting point) was rendered and INSPECTED (not merely
computed): only one arm is visible in any panel. At azimuth=90 the two
arms — offset only in y, both near x≈0 — sit almost exactly in line with
the viewing ray, so one occludes the other instead of separating
left/right as the task's own rationale for that angle assumed. Fixed by
reusing `m06-handoff-candidate-2.png`'s own already-good camera direction
(azimuth=130, elevation=-22, ALSO confirmed by inspection to show both
arms clearly separated plus the visible red fork), widened from that
candidate's 0.6 m distance to 0.9 m so arm B (still at HOME, farther from
`transfer_point`, in milestone 1) is not cropped out of the first panel.
Re-inspected after the change: both arms visible and clearly separated in
all four panels, with the red fork visible near arm B by the final panel.
`m06-handoff-sequence.png` is regenerated with this camera; the strip DOES
now demonstrate the handoff and can be used as evidence, superseding the
"should not be used as evidence" caveat above.

**Measurement discrepancy, flagged not silently corrected.** This entry's
Fix 2 table reports "0.1946 m" final lateral separation at 0.10 m lateral
retreat. Independently re-measured this session, via the exact same
shipped code/scene/seed, directly from `data.site_xpos` at the final
frame (`abs(site_a_final[1] - site_b_final[1])`, the most literal reading
of "final-frame lateral gripper separation"): **0.0983 m**, reproduced
identically across two separate runs. Every OTHER number in this entry's
own verification table (`frames_used=6610`, `from_arm_retreat_dist=0.2263`,
all four skills' z-values) matches this session's own re-measurement
exactly, so the underlying run is confirmed identical — only the "0.1946 m"
figure could not be reproduced by this direct method. Left unresolved
(not guessed at further) because chasing it would cost more bm-ptl round
trips than this pass's budget allowed; **0.0983 m is the number this
session verified and stands behind.**

**Also done this pass, not covered above:** `docs/images/m02-scene.png`
re-rendered against the (reverted-to-original-positions, red-fork) scene —
inspected: original prop layout, red fork visible on the table, both arms
in their fold-back home pose. `docs/images/m06-handoff-complete.png` was
NOT touched, per instruction.

**Re-verified this pass (bm-ptl, `mujoco==3.2.7`, seed 0), independently
from the table above, using the correct `target_object="bottle"` key
(`OBJECT_BODY_NAME["bottle"] == "water_bottle"`; a leftover diagnostic
script from an earlier session had used the wrong key `"water_bottle"` and
reported a false failure — not a real regression, a script bug, corrected
here):**

| skill | result |
|---|---|
| `pick(A, fork)` | success, frames_used=1655, weld_attach_frame=1155, z 0.3560 → 0.3989 |
| `place(A, fork, table)` | success, frames_used=3455, final xyz=(-0.0081, 0.0203, 0.3588) |
| `pick(A, bottle)` | success, frames_used=1655, weld_attach_frame=1155, z 0.4400 → 0.6192 |
| `handoff(A→B, fork)` | success, frames_used=6610, weld_attach_frame=5310, `from_arm_retreat_dist=0.2263`, lateral_y_sep=0.0983 |

`pytest tests/test_skills.py`: 4 passed / 4 failed, same four tests
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`) failing as before this whole ADR-038
body of work began — no regression. Note the specific failure REASON
strings for the plate/mug tests differ from some intermediate runs during
this work (e.g. `weld_attach_failed_after_300_frames` vs an IK-convergence
reason) purely because plate/mug are back at their original coordinates,
not a new defect — same tests still fail, same tests still pass.

**Constraints honored this pass:** only `gen_dual_scene.py` (documentation
only — the position constants were already back at their pre-ADR-038
values), `skills_scripted.py` (comment correction only — the retreat
constants were already 0.10), `scripts/render_handoff_frames.py` (the
sequence camera fix above), the regenerated XML, four images, and this
file were touched. `git diff --stat -- scenes/so101/` confirmed empty.
ADR-031's GRIP freeze, ADR-033's hover, ADR-034's guard and ADR-037's
choreography phases are unmodified.

## ADR-037 — `run_handoff` rewritten as sequential choreography (one arm moves at a time, the other genuinely frozen); the fix for the `_hold_ctrl` drift bug turned out to ALSO resolve the ADR-036/`f92806e` cross-arm collision — `handoff(A, B, fork)` now succeeds end to end, verified by direct measurement, not assumed

**Recorded:** Sept 14, 2026 · **Follows:** ADR-036 (`f92806e`, cross-arm
collision at `to_arm`'s staging sweep) · **Modifies:**
`src/bimanual/control/skills_scripted.py`, `scripts/requirements-bmptl.txt`,
this file, per task scope · **References (user-supplied, cited as such --
not fetched, not described beyond the quoted phrases the task itself gave):**
Wan, Ramos, Yang, Garrett 2025 (NVIDIA), "Learning to Plan & Schedule with
Reinforcement-Learned Bimanual Robot Skills",
https://arxiv.org/html/2510.25634v1 -- "single-arm waiting skill that keeps
one arm stationary"; and "Trajectory planning system for bimanual robots:
Achieving efficient collision-free manipulation" (2025),
https://www.sciencedirect.com/science/article/pii/S0921889025002155.

### Part 0 -- reverting the mink probe's environment damage, gated first

`9d0adde`'s mink probe silently bumped bm-ptl's shared `ov_env` from
`mujoco==3.2.7` to `3.13.0` as a forced transitive consequence of
`pip install mink==1.3.0`. mink was NOT adopted (that probe's own verdict:
on the identical target from the identical "home" pose, mink's QP-based
velocity IK converged to a 0.2233 m residual, stuck at a joint-limit local
minimum, where this project's own `ik.solve_position_ik` (DLS) converges to
0.00986 m; mink's `CollisionAvoidanceLimit` also allowed a measured ~5.5 cm
arm-vs-table interpenetration during that same stuck solve). With mink not
adopted, there was no remaining reason to carry its forced mujoco floor.

**Actions taken, in order, each verified before proceeding:**
1. `pip install mujoco==3.2.7` on bm-ptl (`ov_env`) -- confirmed via
   `python -c "import mujoco; print(mujoco.__version__)"` -> `3.2.7`.
2. `pip uninstall -y mink` -- confirmed removed via `pip show mink`
   (raises "not found").
3. `scripts/requirements-bmptl.txt` updated to match: `mujoco==3.2.7`
   restored, the `mink==1.3.0` line removed, with a comment explaining the
   revert and pointing at `docs/hardware/m06-mink-probe.md` for the
   evaluation record.
4. `pytest tests/test_skills.py` re-run under 3.2.7 BEFORE any code change
   in this commit: **4 passed, 4 failed** -- identical split, identical
   failure reasons, to every previously-documented baseline (ADR-036,
   `m06-mink-probe.md`'s own 3.13.0 re-run). This is necessary but NOT
   sufficient evidence (see point 5): no test in this suite exercises
   `pick(A, fork)`, `place(A, fork, table)`, or `pick(A, water_bottle)` with
   a real `WeldGrasp` -- that is exactly why the mink probe's "pytest
   unchanged" claim did not, by itself, establish that reverting mujoco
   would be safe either.
5. **The three working skills re-run directly** (via the real
   `ScriptedSkillExecutor` + `SkillCall` path, `WeldGrasp` attached, seed 0),
   BEFORE any code change in this commit, and their measured numbers
   compared byte-for-byte against the pre-mink-probe baselines already on
   record (ADR-031, ADR-033/034):

   | Skill | Measured under mujoco 3.2.7 (this commit) | Matches prior baseline? |
   |---|---|---|
   | `pick(A, fork)` | `success=True frames_used=1655 weld_attach_frame=1155 initial_z=0.3560 final_z=0.3989` | YES, identical (ADR-031) |
   | `place(A, fork, table)` | `success=True frames_used=3455 weld_attach_frame=1155 final_xyz=[-0.0081, 0.0203, 0.3588]` | YES, identical (ADR-031) |
   | `pick(A, water_bottle)` | `success=True frames_used=1655 weld_attach_frame=1155 initial_z=0.4400 final_z=0.6192` | YES, identical (ADR-034) |

   No regression. Only after this did any change to `skills_scripted.py`
   begin.

### Part 1 -- the `_hold_ctrl` drift bug: measured, then fixed

**Measured, per this task's correction, before touching anything:**
`_hold_ctrl` rebuilt its ENTIRE returned ctrl vector from the idle arm's
CURRENT qpos every single call, so it commanded zero position error at the
instant of each read and supplied no restoring force against gravity
between reads -- the idle arm's true setpoint ratchets away from its
original pose. This was already flagged, unfixed, in the module's own
docstring (measured previously at 0.04 rad by 100 steps, 0.546 rad
saturation -- a joint hard-limit stop -- by ~1500 steps). A single `pick`
alone runs ~655 frames (ADR-035's own trace), implying ~0.26 rad of drift
before choreography is even involved -- 26x a naive 0.01 rad bar.

**Fix.** `_hold_ctrl(env, frozen_base=None)`: if `frozen_base` is given, it
is returned as a plain copy -- nothing is re-derived from live qpos. A
caller doing a genuine long-duration idle hold captures a snapshot ONCE
(a plain `_hold_ctrl(env)` call, no override -- this still reads live qpos,
but only that one time) at the exact instant an arm becomes idle, and
passes that SAME array back in as `frozen_base` on every subsequent step of
the idle span, however many waypoint/dwell calls that span covers. This is
the same insight as ADR-031's GRIP-dwell freeze, generalized from "freeze
the ACTIVE arm for one dwell" to "freeze the IDLE arm for an entire
choreography phase." `frozen_base` was threaded as a new, purely additive
parameter through `_drive_to_target`, `_dwell`, `_run_waypoint`,
`_run_dwell`, `_run_interpolated_waypoint`, `_run_approach_with_staging`,
and `run_pick` (needed so `run_handoff`'s Phase 1 can freeze `to_arm` for
the WHOLE nested pick call, not just one waypoint of it) -- every one of
these defaults the new parameter to `None`, which reproduces the exact
pre-ADR-037 behaviour. `run_place` and `run_open_drawer` were NOT given new
call sites using this parameter (out of this task's scope; their own idle
holds are unchanged).

**Verification that this did not regress `pick`/`place` (which also use
`_hold_ctrl`):** the same three-skill re-run from Part 0's step 5, re-run
again AFTER this change (still with `hold_ctrl_base` left at its default
`None` for these standalone calls): **byte-identical** to Part 0's
just-recorded numbers (`pick(A,fork)`: 1655 frames, `weld_attach_frame=1155`,
`final_z=0.3989`; `place`: 3455 frames, same weld frame, same final xyz;
`pick(A,water_bottle)`: 1655 frames, `final_z=0.6192`). `pytest
tests/test_skills.py`: unchanged, 4 passed / 4 failed, identical reasons.

**Measured drift with the fix applied, over the actual choreography phases
(not a synthetic long dwell) -- this is the evidence the test threshold
below is based on**, via a real `handoff(A, B, fork)` run instrumented to
read each frozen arm's joint qpos at phase boundaries:

| Phase | Frozen arm | Frames this phase | Max joint drift from its frozen pose |
|---|---|---|---|
| 1 (`from_arm` picks) | `to_arm` (B), held at HOME | 1655 | **0.000774 rad** |
| 2 (`from_arm` approaches transfer point) | `to_arm` (B), SAME snapshot as phase 1 | 500 | **0.000774 rad** (unchanged -- confirms the snapshot itself is not decaying) |
| 3 (`to_arm` approaches receiving point) | `from_arm` (A), held at the transfer point | 3000 | **0.000237 rad** |

Both are more than an order of magnitude under the task's own proposed
0.01 rad bar, not merely under it -- so **0.01 rad is adopted as the test
threshold**, now with real evidence behind it (this was NOT achievable
under the OLD `_hold_ctrl`, where phase 1 alone would have implied ~0.26
rad; it IS achievable under the fix, measured directly, with roughly 13-40x
margin).

### Part 2 -- the cross-arm collision (`f92806e`'s finding): investigated, NOT solved by new routing geometry, but resolved anyway as a side effect of Part 1

**The brief's staging signs, corrected per measurement (unchanged from
ADR-035):** `HANDOFF_STAGING_Y_M = {"A": 0.06, "B": -0.06}` -- each arm
converges on the side OPPOSITE its own base, confirmed again this session,
not re-guessed.

**The hard part: applying those measured values puts `to_arm`'s (B's) own
staging cell on the SAME side of the midline `from_arm` (A) occupies while
parked at the transfer point.** Two routing alternatives from the task's
own list were tried, measured, and NOT adopted:

- **Elevated ("dodge") crossing** -- stage `to_arm` at a taller z, sweep
  laterally clear of `from_arm`'s operating height, then descend.
  Measured (`WeldGrasp`-backed, real closed-loop drives, not a one-shot IK
  check): the lateral sweep at z=0.53 converges collision-free in some
  runs, but sits on a reproducible JOINT-LIMIT KNIFE-EDGE (margin as small
  as -0.000001 rad -- flips pass/fail on essentially no perturbation:
  the SAME (0,-0.06,0.53)->(0,0.02,0.53) move measured `ok=True,
  cross_arm=0` in one run and `ok=False` at joint-limit margin -0.000001 in
  another run that differed only in `from_arm`'s parked height). The
  DESCEND back down to any real receiving height also reliably hit a
  genuine joint-limit wall around z~0.49-0.50 regardless of which elevated
  height was dodged to (tested 0.48/0.50/0.53/0.55/0.60/0.65/0.70).
  Rejected: not robust enough to ship in place of the one corridor already
  proven kinematically solid (ADR-035's lateral crossing at z=0.43).
- **Horizontal ("dodge-in-x") crossing** -- sweep `to_arm` out to
  x=+-0.15/+-0.20 before crossing y, then back. Measured: every variant ran
  out of step budget (500-1000 steps/hop, well past ordinary convergence
  time) before finishing. Inconclusive, not adopted as a positive result.
- **Retracting `from_arm`'s elbow while holding its pinch point fixed**
  (the task's third suggested option) was not attempted: `ik.py` is
  out of scope for this task, and its 5-DOF null-space is not otherwise
  exposed to a caller in this module.

**Given neither alternative was robust, Phase 3 uses the SAME
`HANDOFF_STAGING_Y_M`/`_run_approach_with_staging` lateral corridor
ADR-035 already verified -- i.e. this ADR did NOT change the routing
geometry `f92806e` found colliding.** The expectation, going into
verification, was therefore that Phase 3 would reproduce the SAME
cross-arm collision `f92806e` measured.

**That expectation was wrong, and the reason is instructive.** Verified
directly: `_contact_counts(env)['cross_arm']` is **0 both immediately
before and immediately after Phase 3's `to_arm` approach**, in the actual
`run_handoff` call (not a hand-assembled replay). The likely explanation,
consistent with Part 1's own measurement: under the OLD `_hold_ctrl`,
`from_arm` was the IDLE arm throughout the entirety of Phase 3 (up to 3000
frames in this run), and the OLD mechanism let it sag under gravity with NO
restoring force -- at the measured rate (0.04 rad/100 steps, saturating at
a hard joint limit by ~1500 steps), a 3000-frame idle span would have let
`from_arm`'s actual, physically-simulated arm collapse toward a mechanical
stop, unpredictably changing its real collision geometry from the clean,
fully-extended pose it converged to. The NEW frozen hold keeps `from_arm`
rigidly at exactly the pose it arrived at (holding the fork at the
transfer point), which -- combined with the SAME staging geometry that
previously collided -- turns out to be collision-free. **This is reported
as a genuine, directly-measured finding, not assumed or extrapolated: the
drift fix (Part 1) and the collision fix (Part 2) turned out to be the
SAME fix**, which was not anticipated going in.

### Part 3 -- simultaneous welds on one body during the transfer moment: tested explicitly

**Concern (this task's own):** Phase 4 has `to_arm`'s weld attach and get
verified BEFORE `from_arm`'s weld is released, so both
`from_arm`-vs-`fork` and `to_arm`-vs-`fork` weld equalities could be
active at once, forming a closed kinematic chain through the fork.

**Measured directly** (a manual replay of `run_handoff`'s own Phase
1-4 sequence, instrumented to inspect `env.data` at the exact frame attach
flips true, BEFORE the shipped code's own `weld.release(from_arm)` call):
at that instant, `weld.is_holding('A') == 'fork'` AND
`weld.is_holding('B') == 'fork'` are BOTH true simultaneously --
confirming the two-weld state is real, not merely theoretical.
`qfrc_constraint` norm at that instant: **4.214480** (finite, not
exploding). An EXPLORATORY extra physics step (deliberately run with BOTH
welds still active, NOT part of the shipped code path) produced a max
`|qpos delta|` of **0.010983** over that one step (small, not a jump/
explosion), `qfrc_constraint` norm unchanged to 6 decimal places, and
`env.data.warning.number` all zero (no MuJoCo warnings). **Conclusion:
MuJoCo appears to handle this configuration without instability, at least
for one step** -- but this is NOT relied upon: by code inspection, the
shipped `run_handoff` calls `weld.release(from_arm)` immediately after the
Phase 4a gate, with NO intervening `env.step()` call, so in the actual
production path the two-weld state exists only within `WeldGrasp`'s own
bookkeeping for a moment, never across an actual physics integration step.

### VERIFY -- phase-isolated and full-handoff results

All runs: seed 0, real `ScriptedSkillExecutor`-equivalent path
(`run_pick`/`run_handoff` called directly with a real `WeldGrasp(env)`,
matching what `ScriptedSkillExecutor._dispatch` does).

| Check | Result |
|---|---|
| Phase 1 (`from_arm` picks fork; `to_arm` frozen at HOME) | `ok=True`, frames=1655, `to_arm` drift=0.000774 rad (< 0.01 rad) |
| Phase 2 (`from_arm` approaches transfer point; `to_arm` still frozen) | `ok=True`, frames=500, `to_arm` drift=0.000774 rad (unchanged) |
| Phase 3 (`to_arm` approaches receiving point; `from_arm` frozen at transfer point) | `ok=True`, frames=3000, `from_arm` drift=0.000237 rad (< 0.01 rad), **cross_arm contacts: 0 before, 0 after** |
| Full `handoff(A, B, fork)` | **`success=True`**, `frames_used=6610`, `reason="held by arm B: z=0.4674 (initial 0.3560) dist_to_armA=0.0909 dist_to_armB=0.0582 (to_arm=B) weld_holding_to_arm=True weld_holding_from_arm=False from_arm_retreat_dist=0.1332 from_arm_clear=True"`, `weld_attach_frame=5310`, `is_holding('A')=None`, `is_holding('B')=='fork'` |
| `handoff(B, A, fork)` (opportunistic mirror, reported per this task, NOT gated) | `success=False`, fails at Phase 1: `pick(B, fork)`'s own APPROACH does not converge (`IK residual=0.0954 m`) -- a PRE-EXISTING, already-documented kinematic reach limit of arm B's own base placement to this fork position (unrelated to the choreography change; arm B has never been shown able to pick this fork from its own approach angle) |

Success condition (`is_holding(B)=='fork'` AND `is_holding(A) is None` AND
arm A clear of the shared workspace at the end) is met:
`from_arm_retreat_dist=0.1332 m > HANDOFF_RETREAT_GATE_M=0.10 m`.

`HANDOFF_POSITION_XYZ` is unchanged from ADR-036 (`(0.0, -0.01, 0.43)`).
New constants added: `HANDOFF_FROM_ARM_RETREAT_CLEARANCE_M = 0.12` (so
`from_arm`'s first retreat waypoint clears the 0.10 m gate -- a plain
`CLEARANCE_HEIGHT_M=0.08` lift alone is only 0.08 m of total displacement,
short of the requirement) and `HANDOFF_RETREAT_GATE_M = 0.10`, both
verified reachable/effective by this run, not assumed.

**Render.** `docs/images/m06-handoff-complete.png` -- front camera,
cropped to the table region, 1280x720 source. **Honest framing note, per
this task's own instruction:** the crop shows both arms near the transfer
point; arm B's jaw holds a small, thin white sliver (the fork) that is easy
to miss at this resolution, and arm A's retreat (`from_arm_retreat_dist`
=0.1332 m) is mostly VERTICAL, which a horizontal front camera does not
render as an obvious lateral separation between the two arms -- the
numeric state (`is_holding('A')=None`, `is_holding('B')=='fork'`,
`from_arm_clear=True`) is the reliable evidence; the image is a supporting
artifact, not independent visual proof, exactly as ADR-031's own render
note already cautioned for `m06-fork-lifted.png`.

**Constraints honored.** Only `skills_scripted.py`,
`scripts/requirements-bmptl.txt`, and this file were modified. `grasp.py`,
`ik.py`, `executor.py`, `gen_dual_scene.py`, `scenes/so101/`, and
`ARCHITECTURE.md` are untouched (confirmed by diff). ADR-031's GRIP freeze,
ADR-033's per-prop hover, and ADR-034's already-held guard are unmodified.
`pytest tests/test_skills.py`: 4 passed / 4 failed before and after this
entire commit, identical failure reasons throughout -- no regression.

## ADR-036 — Raise `HANDOFF_POSITION_XYZ`'s z above the table (0.35 -> 0.43) to remove the ADR-035 arm-vs-table_top collision: the targeted collision is gone, but `handoff(A, B, fork)` now fails one waypoint EARLIER, at a NEW cross-arm collision — the fix relocated the failure rather than resolving it, reported honestly, not patched

**Recorded:** Sept 13, 2026 · **Follows:** ADR-035 (target interpolation in
the `handoff` traverse, commit `f521775`) · **Modifies:**
`src/bimanual/control/skills_scripted.py` only, per task scope

**Diagnosis given, verified before changing anything.** ADR-035's own trace
showed waypoint 4 (`to_arm`'s DESCEND) failing its 2nd/3rd interpolation
step at target z=0.351 with a new `armB`-vs-`table_top` contact past
`TABLE_COLLISION_DEPTH_TOL_M`. `HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.35)`
and `TABLE_SURFACE_Z = 0.35` are indeed the same number, confirmed by
re-reading both constants (`skills_scripted.py`) before making any change —
the transfer point sat exactly at tabletop height, and DESCEND was asking
`to_arm` to put its wrist there.

**Fix applied.** `HANDOFF_POSITION_XYZ`'s z: 0.35 -> 0.43 (`TABLE_SURFACE_Z +
CLEARANCE_HEIGHT_M`) — the same height ADR-035's own fresh sweep already
found as a converged, joint-limit-clean staging cell for BOTH arms
(`HANDOFF_STAGING_Y_M`'s docstring), and the height waypoints 1/3 (APPROACH)
were already driving to as `transfer_point + CLEARANCE_HEIGHT_M` before this
change. Chosen over a fresh number specifically so the APPROACH waypoints'
numeric targets would not change at all — only the now-redundant DESCEND
step down to table height (the one that collided) would be removed. Checked
per the task's own warning before concluding anything: with the transfer
point now AT the old hover height, the separate hover-clearance term used to
compute each APPROACH's target is gone (transfer_point IS the hover height
now), so the retreat waypoints (5/6, formerly 7/8) are the only ones now
targeting a genuinely new, previously-untested height
(`0.43 + CLEARANCE_HEIGHT_M = 0.51`) — flagged explicitly in the code
comments, not assumed safe.

**DESCEND waypoints removed, not left as no-ops.** With `HANDOFF_POSITION_XYZ`
raised to the hover height, the old DESCEND targets (`transfer_point` /
`receiving_point` at z=0.35) became numerically identical to the preceding
APPROACH targets (z=0.43 both now) — i.e. start == end, zero distance to
interpolate. Per this task's own instruction ("removing them is reasonable
— but say plainly that you removed them and why, rather than leaving dead
waypoints that report success without moving"), the two DESCEND calls
(`_run_interpolated_waypoint` invocations, old waypoints 2 and 4) were
deleted outright and the skill's remaining waypoints renumbered 1-6 (was
1-8) throughout `run_handoff`'s docstring and its `SkillResult` failure
messages.

**Verification run: `handoff(A, B, fork)` (`to_arm=B, from_arm=A`), via
`scripts/probe_handoff_fork.py` (new, read-only, same
`ScriptedSkillExecutor`/`SkillCall` path `tests/test_skills.py` and
`scripts/probe_place_bottle.py` use), bm-ptl, seed 0, BEFORE and AFTER this
change:**

| waypoint (renumbered after this change) | BEFORE this change (probe run) | AFTER this change (probe run) |
|---|---|---|
| `pick(A, fork)` (nested) | succeeds, weld attaches | succeeds, weld attaches (unaffected) |
| 1: `from_arm` (A) APPROACH to transfer point | converges (same numeric target both runs — unaffected by this diff) | converges |
| old waypoint 2: `from_arm` (A) DESCEND to table-height transfer point | ran and converged in this probe (table-height transfer point still existed) | *(removed outright — no longer exists; see above)* |
| old waypoint 3 / **new waypoint 2: `to_arm` (B) APPROACH to receiving point** | direct shot fails (residual 0.0875), staged to y=-0.06 SUCCEEDS (residual 0.0032), interpolates onward, all steps converge | direct shot fails (residual 0.0875, identical — unaffected by this change), staging to y=-0.06 now **FAILS with a NEW `cross_arm` collision (contacts=1 vs baseline 0)** |
| old waypoint 4: `to_arm` (B) DESCEND to receiving point | reaches this waypoint, fails there with the ADR-035 `armB`-vs-`table_top` collision this task set out to fix | *(never reached in this run — skill now fails earlier, at new waypoint 2)* |

**Net result: the diagnosed arm-vs-table_top collision at the old waypoint 4
is confirmed gone — but `handoff(A, B, fork)` still does not complete.** It
now fails one waypoint EARLIER than before (at the renumbered waypoint 2,
`to_arm`'s APPROACH/staging), with a DIFFERENT failure kind: not a
convergence failure, not a table collision, but a `cross_arm` contact
between the two arms' own collision geometry. Root cause, confirmed by
comparing to ADR-035's own trace rather than assumed: before this change,
`from_arm` (A) had already DESCENDED away from the shared 0.43 m hover band
down to table height (0.35) by the time `to_arm` (B) staged into that band —
the two arms were never in the same z-plane at the same time. This change
removed that DESCEND, so `from_arm` now sits AT the transfer point (0.43)
holding the fork for the entire remainder of the skill, including while
`to_arm` drives its own staging move through the SAME 0.43 m plane at
y=-0.06 — and the two arms' geometry now intersects. **This is exactly the
"relocate the collision" outcome the task instructions warned to check for
explicitly, and it is what happened**, not the table collision resolving
cleanly.

**Also confirmed, per the task's own instruction to check rather than
assume:** the retreat waypoints' new height (0.51 m, `CLEARANCE_HEIGHT_M`
above the raised transfer point) was never reached in this run — the skill
fails at waypoint 2, three waypoints before retreat — so this change
neither confirms nor refutes reachability at 0.51 m; that remains untested.

**Regression check.** `pytest tests/test_skills.py`: 4 passed / 4 failed,
identical split and identical failure reasons to the pre-change baseline
(the one handoff test in the suite, `test_handoff_mug_ends_held_by_arm_b`,
still fails at the SAME earlier point, inside the nested `pick(A, mug)` call,
for the unrelated, already-documented reason ADR-035/the test file's own
docstring records — untouched by this change, since `run_pick` is not
modified here). No change to `grasp.py`, `ik.py`, `executor.py`,
`gen_dual_scene.py`, `scenes/so101/`, or `ARCHITECTURE.md`.

**Not patched further, per this task's hard time cap and explicit
instruction to stop and report rather than iterate.** A candidate next fix
(stagger `from_arm`'s retreat to happen BEFORE `to_arm`'s APPROACH, rather
than after `to_arm`'s GRIP/RELEASE as the skill currently orders things) is
visible from this trace but has not been attempted or verified — recording
it here as an unverified idea, not a recommendation, since the corrected
brief for THIS task specified raising `HANDOFF_POSITION_XYZ`, not
reordering the traverse. `handoff(A, B, fork)` remains unverified working
end to end after two consecutive fix attempts (ADR-035, ADR-036); no
`docs/images/m06-handoff-complete.png` was generated, since the task's own
render step was conditioned on success. `handoff(B, A, fork)` was not
attempted, since the same to_arm-approach failure this table diagnoses is
symmetric in `from_arm`/`to_arm` roles and not expected to behave
differently, and the task did not require it when the primary direction
fails.

---

## ADR-035 — Target interpolation in the `handoff` traverse: implemented per the corrected brief; genuinely progresses the skill three waypoints further, but `handoff(A, B, fork)` still fails at a NEW waypoint (arm-vs-table_top collision during `to_arm`'s interpolated descend) — reported honestly, not patched

**Recorded:** Sept 13, 2026 · **Follows:** `2115a1e` (warm-start IK diagnostic,
`docs/hardware/m06-handoff-warmstart-diagnostic.md`) · **Modifies:**
`src/bimanual/control/skills_scripted.py` only, per task scope

**What was built.** The diagnostic's own pseudocode had a Zeno bug (re-reading
"current position" from the sim every loop iteration means the arm only ever
covers a shrinking fraction of the remaining distance and never arrives) —
fixed by capturing `start_pos` ONCE, before the loop, and interpolating
`start_pos + (end_pos - start_pos) * (i / n_steps)` for a fixed `i`
(`_run_interpolated_waypoint`). The diagnostic's chain also never started from
"wherever the arm happened to be" — it started from a pre-verified converged
staging cell in each arm's own reachable band and walked inward. Verified
directly (not assumed) that this matters for the real skill: `to_arm`'s
APPROACH waypoint, solved directly from HOME, measured IK residual 0.0875 m
(a clear fail) at (y=0.02, z=0.43 — the actual hover height `run_handoff`
uses, not the z=0.35 the diagnostic's own table covered). A fresh sweep this
session at z=0.43 (mirroring the diagnostic's z=0.35 sweep) found the SAME
disjoint-band shape at the actual hover height: arm A's home-converged band
starts at y=+0.06 (residual 0.00999), arm B's mirror at y=-0.06 (residual
0.00999) — recorded as `HANDOFF_STAGING_Y_M`. `_run_approach_with_staging`
tries the direct shot first (the common case, e.g. `from_arm`, which is
already warm from `pick`); only on failure does it drive to the arm's own
staging cell, confirm THAT converges, then interpolate onward
(`_run_interpolated_waypoint`) to the real APPROACH target. Both DESCEND
waypoints (from_arm to the transfer point, to_arm to the receiving point)
are unconditionally interpolated the same way, since the corrected brief's
item 3 confirmed both real targets (`HANDOFF_POSITION_XYZ`'s y=-0.01, and
y=-0.04/y=+0.02 with `HANDOFF_SIDE_OFFSET_M`) sit inside the diagnostic's
own verified y in [-0.06, +0.02] chained band. Joint-limit margin
(`HANDOFF_JOINT_LIMIT_MARGIN_TOL`, matching
`scripts/probe_handoff_reachability_home.py`'s own gate) is checked after
every interpolation step against the PHYSICALLY-REALIZED qpos, closing the
diagnostic's own explicitly-left-open caveat that its chain checked residual
only.

**Verification run: `handoff(A, B, fork)` (`to_arm=B, from_arm=A`), the
skill this task specified, not the pre-existing `test_handoff_mug_...` (which
fails for an unrelated, already-documented pick-reach reason and does not
exercise the traverse this ADR touches).** Per-waypoint outcome:

| waypoint | outcome |
|---|---|
| `pick(A, fork)` (nested) | succeeds, weld attaches (frames 1-655) |
| 1: from_arm (A) APPROACH, hover `(-0.065, 0.05, 0.44)` then re-target `(0, -0.01, 0.43)` | direct shot converges, residual 0.0099 — no staging needed (A already warm from `pick`) |
| 2: from_arm (A) DESCEND to transfer point `(0, -0.01, 0.35)`, interpolated | **all 4 interpolation steps converge**, residuals 0.0100/0.0082/0.0085/0.0083/0.0036 |
| 3: to_arm (B) APPROACH, hover `(0, 0.02, 0.43)` | direct shot FAILS (residual 0.0875, matching the fresh sweep) → staged to `(0, -0.06, 0.43)` (residual 0.0032) → **interpolated in 5 steps to `(0, 0.02, 0.43)`, ALL converge** (residuals 0.0032/0.0098/0.0076/0.0046) — **this is the fix working**: waypoint 3 now succeeds where it failed 100% of the time before this change |
| 4: to_arm (B) DESCEND to receiving point `(0, 0.02, 0.35)`, interpolated (3 steps) | step 1 converges (residual 0.0045); **step 2/3 (target `(0.0077, 0.0043, 0.351)`) fails — not a convergence failure, a COLLISION failure: a new armB-vs-`table_top` contact past `TABLE_COLLISION_DEPTH_TOL_M`, versus zero at baseline** |

**Net result: three additional waypoints now pass that failed 100% of the
time before this commit (waypoint 3 specifically, the one the diagnostic
targeted, now succeeds), but the skill call still does not reach the end —
it fails at a new waypoint the old, single-shot code never reached in the
first place.** This is exactly the caveat the diagnostic's own "what this
did NOT check" section flagged: "a static IK chain is not an executable
trajectory... real execution is exactly what tests this." The IK-chain
diagnostic proved *kinematic* reachability and (separately, in its Part 3)
found the arm-vs-table_top collision check itself fires on otherwise-sensible
converged poses, calling it a probable mesh-collision artifact needing
recalibration before it can gate anything — but recalibrating that check is
explicitly out of this commit's scope (touches `ADR-027`'s existing
collision-detection semantics, not target interpolation), so this ADR does
NOT patch around it. Whether the physical contact at
`(0.0077, 0.0043, 0.351)` — 1 mm above `TABLE_SURFACE_Z` — is a genuine
graze or the same mesh-collision-hull artifact Part 3 flagged is an open
question this ADR leaves open, honestly, rather than tuning
`TABLE_COLLISION_DEPTH_TOL_M` or the waypoint height to make it disappear.

**Per this task's explicit instruction ("If `handoff(A, B, fork)` fails:
STOP. Report the specific waypoint and failure mode. Do not attempt further
fixes"), no further iteration was attempted.** `handoff(B, A, fork)` (the
opportunistic mirror check) was NOT run — the 60-minute cap for this task
was already consumed by the investigation above establishing WHERE and WHY
the interpolation needed to start from a converged pose (the fresh z=0.43
sweep was not optional groundwork: without it, `HANDOFF_STAGING_Y_M` would
have been guessed, not measured). No `docs/images/m06-handoff-complete.png`
was rendered — the run did not succeed, and rendering a failed handoff would
misrepresent the outcome.

**Test suite: unchanged, as required.** `pytest tests/test_skills.py`
before and after this commit: **4 passed, 4 failed**, identical set
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`
still fail, all for their own already-documented, unrelated reasons — the
mug test specifically still fails at `pick(A, mug)`'s own approach
convergence, never reaching this commit's code path at all). No regression;
per this task's own framing, this split was never expected to move, since no
test in the suite exercises `handoff`'s success path with a prop that
survives `pick`.

**Constraints honored:** only `skills_scripted.py` and this file were
modified. `grasp.py`, `ik.py`, `executor.py`, `gen_dual_scene.py`,
`scenes/so101/` and `ARCHITECTURE.md` are untouched. ADR-031's GRIP freeze,
ADR-033's per-prop hover and ADR-034's already-held guard are all unmodified
(confirmed by diff — this commit only adds new functions/constants and
replaces the direct `_run_waypoint` calls at handoff's own waypoints 1-4
with the staged/interpolated equivalents).

## ADR-034 — `place(A, water_bottle, table)` verification: `run_place` never checked whether the object it was told to place was already held, causing a redundant internal re-pick to target an unreachable height; fixed by skipping the nested pick when already held. A second, unrelated waypoint-1 reachability failure remains and is reported, not patched.

**Recorded:** Sept 13, 2026 · **Follows:** ADR-033 (`pick(A, water_bottle)`'s
per-prop hover fix, commit `d239a55`), which fixed `pick`'s own hover height
but was never exercised against `place`'s path (`run_place`'s destination
`approach_above_dest` uses the plain `TABLE_SURFACE_Z + CLEARANCE_HEIGHT_M`
formula, no per-prop term, `skills_scripted.py`) · **Task:** verify
`place(A, water_bottle, table)` by running `pick(A, water_bottle)` then
`place(A, water_bottle, table)` in the same episode (the destination string
`table_side` named in the brief is not implemented by `run_place` --
`destination != "table"` is rejected outright -- so `"table"`, the value
every other working `place` call in this repo already uses, was used
instead).

**Verification result: place did NOT succeed on first attempt.** Probe:
`scripts/probe_place_bottle.py`, bm-ptl, `mujoco==3.2.7`, seed 0.

**First failure, before any code change.** `pick(A, water_bottle)` succeeded
(`weld_attach_frame=1155`, final pick z=0.6192, `is_holding('A')==
'water_bottle'`). The immediately-following `place(A, water_bottle, table)`
failed at `"waypoint 1 (approach) failed [convergence (IK residual=0.1639 m
>= 0.01 m)]"` -- inside `run_place`'s own NESTED `run_pick` call, not `place`'s
own destination waypoints. Root cause: `run_place` calls `run_pick`
UNCONDITIONALLY every time, regardless of whether `arm` already holds
`target_object` -- its own docstring already said "pick the object up (if
not already held)" but the code never implemented that conditional. With the
bottle already lifted and held at z=0.6192 (not resting on the table), the
nested `run_pick` re-read the bottle's CURRENT (airborne) position as
`obj_pos0` and, per ADR-033's `OBJECT_TOP_LOCAL_Z_M["bottle"]=0.11`,
recomputed a hover roughly 0.17 m higher still -- a target the arm could not
kinematically reach in the 500-step waypoint budget, so `place` failed
before ever reaching its own destination logic, and the weld was never
released (`is_holding('A')` stayed `'water_bottle'`).

**This is NOT the ADR-033 failure mode, and ADR-033's `OBJECT_TOP_LOCAL_Z_M`
pattern does not address it.** ADR-033 fixed a hover point sitting BELOW a
STATIONARY object's own physical top during a fresh pick. Here the object
was already held and airborne; the defect is that `place` re-picks an object
it is already holding at all, not that any hover-height formula undershoots
the object's top. Applying ADR-033's pattern here would have been the wrong
fix -- confirmed by tracing the actual failing waypoint (the nested pick's
APPROACH, not any of `place`'s own destination waypoints) before writing any
code.

**Fix applied (`skills_scripted.py`, `run_place` only).** Added a guard:
`already_held = weld is not None and weld.is_holding(arm) == body_name`. If
true, the nested `run_pick` call is skipped entirely and `place` proceeds
straight to its own destination waypoints with the object already in hand;
`weld_attach_frame` correctly reports `None` in this path (no new attach
happened during this `place` call, per that field's own documented meaning).
If `weld is None` or the object is not already held, behaviour is
byte-for-byte unchanged (the nested `run_pick` call still runs exactly as
before). No change to `grasp.py`, `ik.py`, `executor.py`,
`scenes/so101/`, or `gen_dual_scene.py`.

**Result after the fix: the first failure is gone, but `place` still does
not succeed -- a second, different failure now surfaces, and it was NOT
patched.** Re-running the same probe: the nested-pick failure disappears
entirely (no more waypoint-1-inside-pick failure); `place` now fails at its
OWN `"waypoint 1 (approach destination) failed [convergence (IK
residual=0.0138 m >= 0.01 m)]"` -- a plain kinematic IK-solver residual that
misses the 0.01 m tolerance by only 0.0038 m. Diagnosed before touching any
code (`scripts/probe_place_waypoint1_diag.py`): driving toward the same
target for a further 2000 steps (four times the normal 500-step waypoint
budget) does not shrink this residual -- it plateaus, which rules out "just
needs more steps" and is consistent with a genuine reachability-envelope
edge, not a slow-convergence artifact. Independently, the destination x
computed for this run landed exactly on `run_place`'s own safety clip bound
(`dest_xy[0]` clipped to its `+0.30` limit), which is suggestive of the same
kind of arm-specific reachability-envelope boundary this repo has already
found and documented elsewhere (e.g. ADR-027's plate-rim direction fix,
ADR-032's handoff-position sweep) -- but this was not independently
re-measured across other start positions, so it is reported as a plausible
explanation, not a proven one.

**Why this was not also fixed here.** `PLACE_OFFSET_XY_M`/the destination
clip bounds are shared by every prop's `place` call, not bottle-specific;
changing them to dodge one measured edge case, without re-verifying every
other prop's place path (none of which currently reach this waypoint at all
-- `test_place_plate_returns_to_table_rest` fails earlier, at the nested
pick's own grip, per ADR-024's already-documented grasp-reliability gap) is
exactly the kind of speculative, unverified change the task instructions
say not to make. This is reported, not patched.

**Net honest status: `place(A, water_bottle, table)` still returns
`SkillResult.success=False`; `is_holding('A')` still ends as
`'water_bottle'`, not `None`; the bottle never reaches the table in this
run.** `pytest tests/test_skills.py`: 4 failed / 4 passed before this change
and 4 failed / 4 passed after (identical failure reasons/residuals for all
four pre-existing failures, confirmed line-by-line) -- this fix changed no
existing test's outcome, it only changes what a NEW bottle-place probe
(not part of the pytest suite) reports.

**Not done.** No change to `ARCHITECTURE.md` (out of scope for this task).
No change to `ik.py`, `grasp.py`, `executor.py`, `gen_dual_scene.py`, or
`scenes/so101/`. No change to ADR-031's GRIP freeze or ADR-033's pick hover.
No speculative fix applied to the second (waypoint-1-destination)
reachability failure -- it is left open and reported here for a follow-up
diagnostic pass, same as ADR-032's handoff-position gap was left open
rather than patched with an unverified guess.

---

## ADR-033 — `pick(A, water_bottle)`: per-prop APPROACH/RETREAT hover height for tall props, fixing a hover point that sat BELOW the bottle's own physical top

**Recorded:** Sept 13, 2026 · **Follows:** `docs/hardware/m06-water-bottle-diagnostic.md`
(commit `635a902`), which found the bottle displaced ~0.157 m in x / -0.061 m
in z from its reset position DURING (non-convergent) APPROACH/DESCEND,
entirely before GRIP starts, with ADR-031's GRIP-dwell freeze itself
confirmed working correctly (pinch point constant to 4 decimals across all
300 dwell frames) · **Task:** apply the smallest fix that makes
`pick(A, water_bottle)` succeed, choosing between (a) re-reading the
object's position at GRIP start or (b) not knocking it during approach.

**Branch chosen: (b), not (a) — and why (a) would not have worked at all,
not merely worked poorly.** `grasp.WeldGrasp.attempt_grasp`'s Gate 2
(proximity) already measures the pinch point against the object's OWN LIVE
`data.xpos` every call (`grasp.py:356-363`), not against any fixed
`grasp_point`/`hold_pos` value computed in `skills_scripted.py`. Re-reading
the bottle's position and recomputing `grasp_point` at GRIP start (option
a) would change ONLY the value logged/used for `_dwell`'s post-hoc IK
residual report — it is never used to re-aim the arm during the dwell
(ADR-031 freezes the arm's ctrl to wherever DESCEND already left it) and
never used by Gate 2 (which already reads the bottle live). So (a) is not
merely riskier here, as the task's framing anticipated — it is a no-op
against the actual failure: the arm's frozen GRIP-dwell pose is wherever
DESCEND converged to, and DESCEND converged near the bottle's ORIGINAL
resting spot while the bottle had already been knocked ~0.16 m away by
APPROACH's own motion, before GRIP or any re-read could matter.

**Root cause, found by measuring the scene geometry `run_pick` was already
targeting.** `scripts/gen_dual_scene.py`'s `water_bottle_cap` geom is
`pos="0 0 0.10" size="0.012 0.01"` sitting on `water_bottle_body`'s
`size="0.03 0.09"` cylinder — the cap's own top surface sits at local
z = 0.10 + 0.01 = 0.11 m above the body origin (reset z=0.44), i.e. world
z=0.55 m. The old `hover = grasp_point + CLEARANCE_HEIGHT_M` formula gave
hover.z = 0.46 + 0.08 = 0.54 m — **0.01 m BELOW the bottle's own physical
top**, not above it as "hover" is supposed to be. Every other pickable prop
(plate/mug/fork/spoon) has its `GRASP_POINT_OFFSET_M` sitting at or near its
own physical top already, so the same `CLEARANCE_HEIGHT_M` margin genuinely
clears them; only the bottle's grasp point (intentionally lower, near its
neck, partway down a ~0.20 m combined body+cap) leaves its own upper
structure un-cleared by the existing formula. This is consistent with (does
not contradict) the diagnostic's own finding that APPROACH/DESCEND both
report `converged=False` at their full 500-step budgets — a "hover" target
that is not actually clear of the object is exactly the kind of target that
can produce sustained, escalating contact during a redundant 5-DOF
incremental IK drive.

**Fix applied.** Added `OBJECT_TOP_LOCAL_Z_M` (`skills_scripted.py`), a
per-prop dict giving a prop's own physical top as a local z offset above its
body origin, currently populated only for `"bottle": 0.11` (the measured cap
top, from the scene geometry above). `run_pick`'s `hover` is now
`max(grasp_point.z, obj_pos0.z + OBJECT_TOP_LOCAL_Z_M.get(target_object,
offset.z)) + CLEARANCE_HEIGHT_M` instead of the old
`grasp_point.z + CLEARANCE_HEIGHT_M`. For every prop except the bottle,
`.get(..., offset.z)`'s fallback makes `obj_pos0.z + offset.z ==
grasp_point.z` exactly, so `max(...)` is a no-op and their hover height is
byte-for-byte unchanged — this is a per-prop, additive correction, not a
change to the shared formula or to `CLEARANCE_HEIGHT_M` itself. No change to
`grasp.py`, `ik.py`, `executor.py`, `scenes/so101/`, or `gen_dual_scene.py`
(the bottle's mass/geometry are read, not modified).

**Measured result (bm-ptl, `mujoco==3.2.7`, seed 0).** `pick(A,
water_bottle)`: bottle position at reset `(0.2200, 0.0000, 0.4400)`; at GRIP
start (post-DESCEND, reproduced via `run_pick`'s own `_run_waypoint`/
`_run_dwell` helpers) `(0.2372, 0.0440, 0.4404)` — displacement now ~0.017 m
in x / ~0.0004 m in z versus the old ~0.157 m / -0.061 m, a lateral nudge
during the redundant IK solve's approach, not a knock; weld attaches at
`attach_frame=1155` (whole-skill-call frame count via the real
`ScriptedSkillExecutor.execute`), `is_holding('A')=='water_bottle'`, final
z=0.6191 (initial 0.4400, success threshold 0.3700) — lifted 0.179 m.
`SkillResult.success=True`. `pytest tests/test_skills.py`: 4 failed / 4
passed before this change and 4 failed / 4 passed after (same four
pre-existing failures — `test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b` — all unrelated to the bottle and
unaffected by this change, confirmed by identical failure reasons/residuals
before and after).

**Not done.** No ADR-031 correction is needed — its GRIP freeze was already
confirmed working correctly by the referenced diagnostic and is untouched
here. This entry is not mirrored into `ARCHITECTURE.md`'s section 4: ADR-028
through ADR-032 (the preceding four M06 diagnostics/fixes) are likewise
recorded only here, not in `ARCHITECTURE.md`, matching this file's own
established recent practice; `ARCHITECTURE.md` itself is out of scope for
this task.

---

## ADR-032 (second pass) — Handoff-position sweep re-run with four seeding/collision corrections plus an extended-height grid; STILL no shared point, and the reach bands themselves do not overlap at x=0

**Recorded:** Sept 13, 2026 · **Follows:** the ADR-032 entry directly below (first
pass: home-seeded, residual+cross-arm-collision only, all 36 candidates FAIL) ·
**Task:** re-run the same sweep with four named corrections (C1-C4) plus two
additions, and relocate `HANDOFF_POSITION_XYZ` to a passing candidate if one
exists.

**What changed versus the first pass, and why each change was expected to
matter.**
- **C1 (grid targets the pinch point).** Unchanged in substance --
  `ik.solve_position_ik` already targets the pinch point (ADR-025), not the
  gripper body, in both passes. Made explicit this pass by also checking
  each solved configuration for a joint pinned at its `jnt_range` bound
  (margin < 1e-4 m), not merely residual convergence.
- **C2 (seed from the handoff-APPROACH pose, not home) -- the correction
  expected to matter most.** The first pass's own root-cause paragraph
  attributed the universal FAIL to every solve starting from the folded
  "home" pose, which lets `ik.py`'s redundant 5-DOF DLS solver fall into a
  table-tunneling local minimum when asked to reach centrally across the
  table. This pass stages each arm's solve exactly as
  `skills_scripted.run_handoff` itself does: solve HOME -> that arm's own
  APPROACH hover point (`CLEARANCE_HEIGHT_M` above the candidate, offset by
  `HANDOFF_SIDE_OFFSET_M` for the receiving arm), then -- from THAT
  resulting configuration, not home again -- solve -> the candidate itself.
  The residual gated on is this second, seeded solve's residual.
- **C3 (cross-arm collision, explicit threshold).** Both arms' seeded,
  converged configs applied SIMULTANEOUSLY via `mj_forward`; rejected if any
  cross-arm contact is deeper than -0.005 m.
- **C4 (one direction).** Swept only `from_arm=A, to_arm=B` (the commit
  gate, `handoff(A, B, fork)`); `handoff(B, A, fork)` is checked
  opportunistically by actually running the skill, not swept as a second
  grid (not reached this session -- see Consequences).
- **Addition 1 (kept, not dropped): arm-vs-world.** Each arm's own solved
  config applied ALONE (table_top AND every prop -- plate, mug, fork,
  spoon, water_bottle, drawer), same -0.005 m bar, so a seeded-but-still-
  tunneling candidate cannot pass merely because the OTHER arm's collision
  happened to be checked.
- **Addition 2: grid extended upward.** z in {0.35, 0.38, 0.40} (as
  specified) PLUS z in {0.44, 0.47} (extra rows), on the reasoning that
  z=0.35 IS `TABLE_SURFACE_Z` and a real handoff should happen in free space
  above it, not at or grazing the surface.

**What was measured.** `scripts/probe_handoff_reachability.py`, rewritten
for this pass, run on bm-ptl (ADR-020). Full 60-row table (12 y-values x 5
z-values, x=0 fixed):
`docs/hardware/m06-handoff-reachability.md`.

**Result: ALL 60 candidates FAIL, and `both_reachable` is False on EVERY
SINGLE row -- not merely the collision checks.** This is a stronger, more
precisely diagnosed non-go than the first pass, not a repeat of the same
ambiguous result: the seeding correction (C2) measurably did **not** move
armA's residual at all for most rows where it had previously failed badly
(e.g. `(0, -0.12, 0.35)`: 0.21540 m in BOTH the first pass and this one,
identical to 5 decimal places) -- because `CLEARANCE_HEIGHT_M` is only 0.08
m above the candidate, the seeded approach pose sits in the same
local-minimum basin as the candidate itself for targets the solver already
fails on from home. Reported honestly rather than claimed as a fix that
worked: **C2, applied exactly as instructed (a kinematic IK reseed, not a
physically-simulated pick-then-transfer), did not rescue any candidate this
session found.**

**A second, independent finding, confirmed by a direct control check (not
merely inferred from the sweep table): at x=0, the two arms' own
convergence bands do not overlap AT ALL, and each arm converges BETTER on
the side OPPOSITE its own base, not the side it is mounted on.** Measured
directly: `arm A -> (0, -0.20, 0.40)` (arm A's OWN side, base at y=-0.25):
residual 0.260, does not converge. `arm A -> (0, +0.20, 0.40)` (the far
side): residual 0.061, much closer (still not under the 0.01 m bar, but an
order of magnitude tighter). Arm B is the exact mirror
(`(0,+0.20,0.40)`=0.260, `(0,-0.20,0.40)`=0.061). A known-good off-
centerline control target, `(0.30, -0.05, 0.50)` for arm A / its mirror
`(-0.30, 0.05, 0.50)` for arm B, both converge cleanly (residual 0.00848 m
each) -- confirming the solver and the arm/geom lookups are not swapped or
broken; the crossed, non-overlapping reach pattern at x=0 is a real,
measured property of this scene's arm mounting, not a script defect. Given
this, at x=0 there is a wide dead band (roughly y in [-0.04, 0.10] for arm
A's failure side crossed with arm B's mirrored failure side) where NEITHER
arm converges well, and the two arms' respective "good" bands sit almost
entirely on each other's own base side -- the opposite of what a shared
midline transfer point needs.

**Decision: do NOT change `HANDOFF_POSITION_XYZ`.** Per this task's own
explicit stop condition ("If NOTHING passes even at z in {0.44, 0.47}, stop
and report... that would mean the arms have no collision-free shared
workspace at any tested height, which is an ADR-021 arm-placement decision,
not a constant change"), `skills_scripted.py` is left untouched by this
entry. The extended z rows (0.44, 0.47) do not change the verdict --
`both_reachable` fails identically at every height tested, so this is not a
height problem the way the first pass's own note speculated it might be;
it is an x=0 lateral-reach problem, orthogonal to z.

**Correction to this task's own pre-written framing.** The instruction text
supplied for this ADR entry asserts "ADR-021's assumed 0.10 m shared band
has... been superseded by measurement twice." The actual ledger in this
file is longer than that: ADR-026's home-pose re-measurement, then
`probe_reachability.py`'s residual-only envelope sweep, then the first
ADR-032 pass's collision-checked sweep, and now this second pass, have each
in turn found the shared band smaller or less real than the previous
measurement claimed -- more than two supersessions on the record, and this
entry is not the first to say so (the first ADR-032 entry below already
made the same correction, saying "a third time"). Restating the number
here as "twice" would understate the file's own history, so it is not
repeated as given.

**Consequences.** `handoff(A, B, fork)` is NOT re-verified this session --
no candidate exists to verify against, so the VERIFY step this task
specifies (is_holding checks, `docs/images/m06-handoff-complete.png`) is
not attempted; rendering a scene with no valid transfer point applied would
misrepresent a check that never ran. `handoff(B, A, fork)`'s opportunistic
check is likewise not run for the same reason (C4 gates it on a chosen
position that does not exist). `pytest tests/test_skills.py` is unchanged
by this entry (no source file changed): 4 passed / 4 failed, before and
after, identical to the first pass's own reported baseline, same four
tests, same reasons. This strengthens, not merely repeats, the
recommendation already on record: resolving this needs either (a) moving
one or both arm bases (ADR-021's own placement assumption -- now shown to
produce a crossed, non-overlapping reach pattern at the table's own
centerline, not merely an optimistic band), or (b) an IK-solver change (out
of `skills_scripted.py`'s scope) that does not depend on which basin the
seed pose happens to land in. Recommending, not deciding, per this task's
own instruction that an arm-placement change is a decision for the user.

---

## ADR-032 — Handoff-position re-measurement: NO collision-free shared point found in the specified sweep; NOT fixed, escalated instead of a constant change

**Recorded:** Sept 13, 2026 · **Follows:** ADR-031 (GRIP-dwell freeze, which made
`pick(A, fork)` succeed) · **Task:** relocate `HANDOFF_POSITION_XYZ` into the
measured shared reach envelope, per instruction to STOP and escalate rather than
guess if no candidate passes.

**Context.** `pick(A, fork)` succeeds; `handoff(A, B, fork)` gets through arm
A's pick and fails at waypoint 3 (`to_arm` APPROACH) -- arm B cannot reach the
current `HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.35)`. The existing comment on
that constant cites `m06-reachability-probe.md`'s "Shared handoff band: y in
[-0.12, 0.10]" line as justification, but that band was measured by
`probe_reachability.py`'s `run_envelope_sweep`, which (its own docstring says
so explicitly) checks IK residual convergence ONLY -- "collision not checked
for the sweep". A grid point can converge kinematically while the solved
configuration drives an arm segment through the table or a prop. That
caveat, not the band itself, is why this task re-measures instead of trusting
the existing constant.

**What was measured.** New script `scripts/probe_handoff_reachability.py`,
run on bm-ptl (ADR-020) and cross-checked byte-identical on this developer's
laptop (mujoco 3.2.7 both places): sweep x=0 (the constant's existing x),
y from -0.12 to 0.10 in 0.02 m steps (12 values), z in {0.35, 0.38, 0.40}.
For each of the 36 (y, z) points, IK is solved independently for arm A and
arm B (from the home-keyframe reset pose, ADR-026), and each arm's solved
joint configuration is applied to a scratch `MjData` and checked for any
NEW contact beyond that arm's measured reset-pose baseline (0 for both
arms) -- the exact same per-arm-independent residual+collision method
`probe_reachability.py`'s own primary probes use for their PASS bar,
copied (not imported) into the new script so it has no coupling to that
script's grid constants. Full table:
`docs/hardware/m06-handoff-reachability.md`.

**Result: ALL 36 candidates FAIL for at least one arm. NO (y, z) point in
the specified sweep passed for both arms.** This was not expected to be
uniform -- z=0.35 (exactly `TABLE_SURFACE_Z`) was flagged in advance as the
likeliest to fail, but z=0.38 and z=0.40 (3-5 cm clearance above the table)
failed identically. Inspecting the actual MuJoCo contacts for representative
FAIL rows (not merely trusting the boolean) confirms these are genuine,
non-trivial collisions, not a script artifact: e.g. arm A solved toward
(0.00, 0.06, 0.40) (residual 0.00926 m, well converged) produces
`armA_lower_arm`/`armA_wrist` vs. `table_top` contacts at up to -0.0237 m
penetration, plus contacts with the (stationary, unrelated) `mug` and
`fork` bodies at up to -0.031 m -- the solved arm literally swings through
the tabletop and through props resting nearby, not merely grazing. A
control check confirmed the machinery itself is not universally broken:
a known off-centerline target, (0.30, -0.05, 0.50), solved with residual
0.00848 m and **zero** new contacts for arm A -- so the collision check
correctly reports "no collision" when there genuinely is none; it is the
x=0 centerline candidates specifically, at this z band, that tunnel.

**Root cause, not fixed here (out of this task's permitted file list, and
already flagged as an existing, out-of-scope finding).** This is the same
`ik.py`/DLS-solver local-minimum behavior `m06-reachability-probe.md`'s own
"Step 4" section already documented for `plate_at_rest`/`mug_at_rest`/
`bottle_at_rest`: solving toward a point requires reaching centrally
across/over the table from the folded "home" seed, and the redundant 5-DOF
position-only solver (ADR-024) has no notion of the table's existence, so
it happily converges to a position-accurate configuration that gets there
by swinging the forearm through the table and through whatever sits on it,
rather than up and over. `ik.py` is on this task's do-not-touch list, and
fixing the solver (multi-start solving, an obstacle-aware cost term, or a
different seed) is exactly the kind of code change this task was not
scoped to make.

**Decision: do NOT change `HANDOFF_POSITION_XYZ`.** Per this task's own
explicit instruction ("If NO candidate passes for both arms, stop and
report the table... That would mean the two arms have no collision-free
shared workspace at any tested height... it would need an arm-placement
decision, not a constant change"), `skills_scripted.py` is left untouched
this commit. Picking any (y, z) from this sweep and writing it into the
constant anyway would repeat exactly the mistake this task was assigned to
fix: a plausible-looking constant that was never actually verified
collision-free.

**What this means for ADR-021.** ADR-021's original ~0.30 m reach / 0.50 m
base-gap layout assumption, already shown too optimistic once by ADR-026's
home-pose re-measurement and again by `probe_reachability.py`'s residual-only
band, is now superseded a third time: even the residual-only band's claimed
overlap does not survive a collision check at any of the three heights this
task specifies. Whether a collision-free shared point exists at some OTHER
(x, y, z) outside this specific sweep is not established either way by this
result -- only that none exists in the region this task was scoped to check.
Resolving this for real needs either (a) moving one or both arm bases
(ADR-021's own placement assumption) so a shared reach point exists further
from the table's central tunneling zone, or (b) an IK-solver fix (out of
`skills_scripted.py`'s scope) that avoids the table-tunneling local minimum.
Recommending, not deciding, per this task's own instruction that an
arm-placement change is a decision for the user, not this commit.

**Consequences.** `handoff(A, B, fork)` is NOT re-verified in this commit
(the task's VERIFY step is conditioned on a chosen position existing).
`docs/images/m06-handoff-complete.png` is not produced. `pytest
tests/test_skills.py` is unchanged by this commit (no source file changed)
-- the same 4 failed / 4 passed as before, `test_handoff_mug_ends_held_by_arm_b`
still failing for its own pre-existing, unrelated reason (arm A's `pick`
of the mug fails to converge, per that test's own captured output).

---

## ADR-031 — IK freeze during GRIP dwell to prevent shifting-pinch-point retreat

**Recorded:** Sept 13, 2026 · **Follows:** the "M06 Phase 2 follow-up" entry below
(Bug 2, pinch-point gate fix, which measured but did not chase the root cause) ·
**Touches:** `src/bimanual/control/skills_scripted.py`'s `_dwell` only. `ik.py`,
`grasp.py`, `executor.py`, `scenes/so101/` are unchanged.

**Context.** The prior entry's own instrumentation measured, during
`pick(A, fork)`'s GRIP dwell, the pinch-point distance to the fork growing
0.0351 -> 0.0793 m and the gripper-body distance growing 0.0299 -> 0.0795 m
over the 300-step dwell -- both starting BELOW `WeldGrasp`'s 0.05 m proximity
gate and ending ABOVE it. Root cause, confirmed by reading `ik.py` directly
this session (not merely inferred): `ik.solve_position_ik` targets the PINCH
POINT (ADR-025's midpoint of the fixed and moving jaw BODIES), recomputed
fresh from the CURRENT qpos on every solve. As a GRIP dwell closes the jaw,
the moving jaw body's own position shifts, so the midpoint shifts even though
the dwell's target (`hold_pos`, a fixed point on the object) does not. The old
`_dwell` loop re-solved IK every step to keep that SHIFTING midpoint pinned on
the fixed target -- which means it kept commanding the ARM (not just the jaw)
to move so the midpoint would track the jaw's own closing motion, i.e. the
arm physically retreated as the jaws closed. Meanwhile the closure gate
(`armX_gripper` qpos < 0.3) needs about 150 steps to close. The two gates
therefore passed/failed on opposite ends of the dwell and never held true on
the same frame, so `weld.attempt_grasp` never returned `True` and
`weld_attach_frame` stayed `None`.

**Decision.** `_dwell` now captures the driven arm's own 5 positioning-joint
ctrl targets ONCE, before the dwell loop starts (reading `env.data.ctrl`,
i.e. wherever the preceding APPROACH/DESCEND waypoint already converged and
left the arm commanded), and reapplies that SAME frozen ctrl vector every
step for the rest of the dwell -- no `ik.solve_position_ik` call at all
inside the loop. Only the gripper joint's ctrl changes step to step, toward
`gripper_fraction`. This applies to EVERY call of `_dwell` -- GRIP dwells
(`run_pick`, `run_handoff`'s receiving-arm GRIP, `run_open_drawer`'s GRIP)
and RELEASE dwells (`run_place`, `run_handoff`'s releasing-arm RELEASE,
`run_open_drawer`'s RELEASE) alike, since all six route through the one
shared `_dwell` implementation and the shifting-pinch-point problem is
symmetric for an opening jaw. The idle-arm `_hold_ctrl` pattern (M06 Phase 2
Commit 1) is unchanged and is a DIFFERENT mechanism (it governs the arm NOT
being driven this call; ADR-031 freezes the arm that IS being driven, only
during a GRIP/RELEASE dwell specifically).

**Verification (bm-ptl), `pick(A, fork)`, seed=0, monkey-patched
`WeldGrasp.attempt_grasp` instrumentation logging pinch-point distance and
gripper qpos every 30 GRIP-dwell frames (scratch diagnostic, not shipped,
same technique as the prior entry's own measurement):
```
GRIP frame 1:   pinch_point_distance=0.0351 m  gripper_qpos=1.7449 rad
GRIP frame 30:  pinch_point_distance=0.0372 m  gripper_qpos=1.5992 rad
GRIP frame 60:  pinch_point_distance=0.0374 m  gripper_qpos=1.3215 rad
GRIP frame 90:  pinch_point_distance=0.0373 m  gripper_qpos=1.0065 rad
GRIP frame 120: pinch_point_distance=0.0373 m  gripper_qpos=0.6807 rad
GRIP frame 150: pinch_point_distance=0.0373 m  gripper_qpos=0.3519 rad
```
Distance now stays flat (~0.035-0.037 m, comfortably under the 0.05 m gate)
instead of growing to 0.0793 m, while qpos falls steadily as the jaw closes
-- direct evidence the freeze removed the retreat. Result:
`success=True frames_used=1655 reason="lifted fork: initial_z=0.3560
final_z=0.3989 ... weld_attach_frame=1155 weld_active_at_end=True"`,
`is_holding('A')=='fork'`. `mj_warnings={}`,
`max_joint_limit_violation=0.00039` (unchanged, negligible). This is the
first `pick`/`place`/`handoff` call in this project to attach a weld and
lift a prop past the WELD success threshold.

**Also run (bm-ptl, seed=0), each its own genuinely different outcome, not
chased further under this ADR's scope:**
- `pick(A, mug)`: `success=False`, fails at waypoint 1 (approach),
  `IK residual=0.0532 m` -- the pre-existing, already-documented arm-A
  kinematic reach limit to `mug_at_rest` (ADR-027/`m06-reachability-probe.md`),
  unrelated to GRIP dwell and unaffected by this fix (never reaches GRIP).
- `pick(A, water_bottle)`: `success=False`,
  `weld_attach_failed_after_300_frames`, `weld_attach_frame=None` -- reaches
  GRIP but the gate still never fires for this object/offset combination;
  bottle z fell 0.4400 -> 0.3684 during the dwell. A different, not-yet-
  diagnosed proximity gap, flagged as a follow-up, not chased under this
  session's scope (this ADR's job was the GRIP-freeze mechanism, verified
  above on the fork).
- `handoff(A->B, fork)`: `success=False`, fails at waypoint 3 (`to_arm`/arm B
  APPROACH), `IK residual=0.0875 m` -- `from_arm` (A) had already picked the
  fork successfully (its own nested `pick` succeeded, weld attached); the
  failure is arm B's own reach limit to `receiving_point`, a different
  kinematic gap from arm A's, surfaced only because arm A's GRIP now
  actually completes.
- `place(A, fork, destination=table)`: **`success=True`**,
  `frames_used=3455`, `weld_attach_frame=1155`,
  `weld_active_at_end=False` (released cleanly before jaw-open, per
  ADR-030's ordering), final position `x=-0.0081 y=0.0203 z=0.3588`
  (table-resting height, in-bounds).

**Render.** `docs/images/m06-fork-lifted.png` (front camera, end-of-`pick`
state, 1280x720). Reported honestly: at this camera's distance the fork is a
small, thin white object near arm A's jaw and the frame does not, by itself,
visually PROVE the lift the numeric z-delta already establishes -- it is
included as a supporting artifact, not as independent confirmation.
`docs/images/m06-handoff-complete.png` was NOT produced: `handoff` did not
succeed (arm B's own reach limit above), and rendering a failed handoff would
misrepresent it as the requested "complete" state.

**`pytest tests/test_skills.py` (bm-ptl), before and after this change:
identical, 4 passed / 4 failed, same four tests
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`),
same reasons** (plate: `weld_attach_failed_after_300_frames` at
`final_z=0.3524`, an ADR-024 grasp-reliability/offset gap specific to the
plate, not the pinch-point-retreat mechanism this ADR fixes; drawer/mug: the
same already-documented arm-A/arm-B kinematic reach limits). No shift in
which tests pass, in either direction -- the four tests this suite already
tracked as blocked by OTHER, separately-documented gaps remain blocked by
those same gaps; this fix unblocks `fork` specifically (not covered by
`test_skills.py`'s own four object choices) and is verified above via
`scripts/run_skill.py`/a scratch instrumentation script instead.

**Consequences.** `_dwell`'s per-step loop no longer calls
`ik.solve_position_ik` at all -- a real behavioural narrowing (the arm
cannot correct its OWN dwell-time drift by re-solving), justified because
the thing it was "correcting" toward was itself the source of the retreat.
The post-hoc `_validate_against_baseline` convergence check (run once, after
the dwell, by `_run_dwell`) is unaffected -- it still re-solves IK once to
report a residual for logging/validation purposes. This does not fix the
plate's or water_bottle's own separate grasp-reliability gaps, or either
arm's own kinematic reach limits -- those remain open, tracked by the ADRs
that already found them (ADR-024, ADR-027 and the prior entry below).

---

## M06 Phase 2 follow-up — Bug 1 (jaw hull collision) audited, found already fixed; Bug 2 (grasp gate) fixed to measure from the pinch point

**Recorded:** Sept 13, 2026 · **Follows:** ADR-030 (weld wired into `pick`/`place`/
`handoff`, found `pick(A, fork)`/`pick(A, water_bottle)` both fail
`weld_attach_failed_after_300_frames`) · **Touches:** `src/bimanual/sim/grasp.py`
only. `scripts/gen_dual_scene.py` and `scenes/so101/` are **unchanged** by this
entry -- see Bug 1 below for why.

### Bug 1 — jaw hulls: audited, NOT currently broken; the given premise did not
### reproduce

The task handed to this session asserted, with a specific per-arm geom table,
that `sts3215_03a_v1`, `wrist_roll_follower_so101_v1` and `moving_jaw_so101_v1`
were still `COLLIDABLE` in the compiled model, and asked which of three causes
explained it (wrong geoms, overwritten later, or a mismatched assertion set) so
the fix would not regress.

**Directly checked, not assumed, on both machines:** compiled
`src/bimanual/sim/assets/so101_dual_table.xml` with `mujoco==3.2.7` and read
`model.geom_contype`/`model.geom_conaffinity` for every geom on
`arm{A,B}_gripper` and `arm{A,B}_moving_jaw_so101_v1`, first on this
developer's laptop, then independently on bm-ptl
(`C:\Users\devcloud\project\ov_env\Scripts\python.exe`, same mujoco version).
**Both runs agree: all three target meshes already report `contype=0
conaffinity=0` on both arms** (`sts3215_03a_v1` x2 per gripper body -- one at
`armA_gripper`, a second colocated copy from the wrist_roll servo housing --
`wrist_roll_follower_so101_v1` x1, `moving_jaw_so101_v1` x1), and both finger
pads (`arm{A,B}_static_finger_pad`, `arm{A,B}_moving_finger_pad`) report
`contype=1 conaffinity=1`, collidable, as required. Re-running
`scripts/gen_dual_scene.py` from a clean checkout reproduces the committed
`so101_dual_table.xml` byte-for-byte (`diff` empty) -- the generator is
deterministic and its output matches what both machines compiled.

**Conclusion: this is `8f09f8c`'s ("M06a fixes: target-prop exemption, complete
jaw collision disable, fork test") own completed fix, still in effect, not a
regression and not incomplete.** `disable_jaw_mesh_collision()` (see that
function's own docstring in `scripts/gen_dual_scene.py`) already matches by
BODY membership (`{prefix}gripper`, `{prefix}moving_jaw_so101_v1`), not by mesh
name, which is exactly the fix `8f09f8c` made after finding the mesh-name
filter missed the colocated `sts3215_03a_v1` servo-housing geom. Per this
session's honesty rules ("never claim a module works without running it" cuts
both ways -- a claimed *broken* state must be run and confirmed too), no change
was made to `scripts/gen_dual_scene.py` or `scenes/so101/` for Bug 1: there was
nothing to fix, and editing a generator that already produces the correct
output on unverified say-so would be the kind of unearned change this
project's conventions exist to prevent. The per-arm geom table this session was
handed does not match either machine's compiled model; it is not reproduced
here, and this entry does not speculate about its origin beyond what was
directly checked.

`scripts/probe_pad_separation.py` re-run on bm-ptl for completeness (VERIFY 1's
second requirement): **bit-for-bit identical** to ADR-028's own gate (fully
closed 0.00600 m, midway 0.07621 m, fully open 0.13188 m, spread 0.12589 m) --
expected, since nothing touching pad geometry changed.

### Bug 2 — grasp gate measured the wrong point, fixed; `pick(A, fork)` still
### fails, for a different, deeper reason

`grasp.py`'s `WeldGrasp.attempt_grasp` Gate 2 (proximity) measured the
`armX_gripper` BODY's world position to the object -- not the PINCH POINT
`ik.solve_position_ik` actually targets (ADR-025: the midpoint of
`armX_gripper` and `armX_moving_jaw_so101_v1`'s body positions, recomputed
every solve via `mj_jacBody`/`xpos` on both bodies, never a fixed local-axis
offset -- see `ik.py`'s `solve_position_ik`/`_pinch_point()`). Per this
session's explicit instruction, the fix mirrors `ik.py`'s own computation
(`0.5 * (xpos[fixed_jaw] + xpos[moving_jaw])`) rather than a fixed
`PINCH_POINT_OFFSET_M`-along-local-Z guess, which would not track jaw closure
the way the true midpoint does. `WeldGrasp.__init__` now also resolves each
arm's moving-jaw body id (via `ik.moving_jaw_body_name`) alongside the
already-resolved fixed-jaw body id; the weld's own attach frame (anchor/relpose
computed against the gripper body) is unchanged -- only the Gate 2 distance
measurement moved. `attempt_grasp`'s docstring carries the exact note this
task specified, plus a "Bug history" paragraph recording what changed and why.

**Verification, bm-ptl, before/after `pytest tests/test_skills.py`:** BEFORE
(pre-fix `grasp.py`, i.e. ADR-030's own state): 4 passed / 4 failed
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`
-- byte-for-byte ADR-030's own reported baseline). AFTER (pinch-point gate):
**identical, 4 passed / 4 failed, same four tests, same reasons** (plate still
`weld_attach_failed_after_300_frames` at `final_z=0.3524`; drawer/handoff
failures are the same already-documented kinematic-reach limits, untouched by
this change). No regression.

**`pick(A, fork)`, fresh env, seed=0, AFTER the gate fix:** `success=False,
frames_used=1300, reason=weld_attach_failed_after_300_frames`. `weld:
attach_frame=None active_at_end=False`. `fork z: initial=0.3560 final=0.3538
delta=-0.0022` (never lifted). `mj_warnings={}`,
`max_joint_limit_violation=0.00039` -- no MuJoCo warnings. **This is still a
FAILURE**, and per this task's own branch instruction ("If it FAILS: STOP and
report which of these occurred"), it is reported here rather than chased with
mug/water_bottle/handoff/place or a render: **the gate never fired** (not
"attached then released", not "held but did not lift" -- `weld_attach_frame`
is `None`, meaning `attempt_grasp` never returned `True` in any of the 300 GRIP
dwell steps).

**Why the gate fix did not flip this to a pass, measured directly rather than
assumed:** a monkey-patched instrumentation of `attempt_grasp` (scratch
diagnostic, not shipped) logged the fixed-jaw body position, the moving-jaw
body position, their midpoint (the corrected pinch point), and the fork's
position every 20 GRIP-dwell calls. Both the OLD gripper-body-only distance and
the NEW pinch-point distance grow **together, almost identically** through the
dwell (gripper-body: 0.0299 m -> 0.0795 m; pinch-point: 0.0351 m -> 0.0793 m,
both crossing the 0.05 m threshold around the same point in the closure
sweep) -- because the fixed-jaw and moving-jaw bodies were observed to move
*together* (both rising in z by several centimetres over the 300-step dwell),
not apart, so their average tracks almost the same trajectory as either body
alone. Root cause candidate, not chased further under this task's own
60-minute time box and its "not a wiring defect to be patched around by
loosening a threshold" instruction: `_dwell` (`skills_scripted.py`, out of
scope for this session) re-solves `ik.solve_position_ik` toward a FIXED
target every step of the GRIP dwell, using the CURRENT (including
still-closing) jaw angle each time; as the jaw sweeps through nearly its full
~2 rad range over the dwell, the instantaneous IK solution that keeps the
pinch point at target changes rapidly, and the arm's actual (PD-tracked)
pose appears to lag that fast-moving solution rather than the jaw's motion
being compensated for. The measured symptom -- both jaw bodies drifting
upward together, not one compensating for the other -- is consistent with
that lag, but this entry stops at "measured, not chased," per instruction:
confirming the lag mechanism precisely would mean touching `ik.py` or
`skills_scripted.py`, both off-limits here.

**What this means for ADR-030's own root-cause claim.** ADR-030 attributed the
divergence to `ik.solve_position_ik` targeting "the `armX_gripperframe` SITE"
-- but `ik.py`, read directly for this session, does not target that site at
all as of ADR-025; `solve_position_ik` targets the fixed/moving-jaw body
midpoint exclusively, with no site reference anywhere in the solver. ADR-030's
own diagnosis of *which* point diverges was therefore already imprecise; this
session's direct measurement (both bodies drifting together) is offered in its
place, not to relitigate ADR-030's wiring work, which is unaffected.

**Consequences.** The gate now measures the quantity the task specified and
`ik.py` actually controls -- a correctness fix, not a threshold retune -- and
does not regress anything (`pytest` identical before/after). It does not,
by itself, unblock `pick(A, fork)`; the remaining gap is a jaw-closure /
IK-tracking dynamics question in `skills_scripted.py`/`ik.py`, both out of
this session's scope, flagged as a follow-up rather than fixed here.

---

## ADR-030 — Weld wiring into scripted skills (Phase 2 Commit 2)

**Recorded:** Sept 13, 2026 · **Follows:** ADR-029 (weld mechanism verified, Phase 1),
M06 Phase 2 Commit 1 (ctrl-hold decision + measured drift gap).

**Context.** ADR-029 built and verified `WeldGrasp` in isolation
(`scripts/probe_weld_grasp.py`, `docs/hardware/m06-weld-verification.md`) but left it
unwired: "deliberately NOT wired into `pick`/`place`/`handoff` or `executor.py`". This
commit does that wiring and nothing else -- `ik.py`, `grasp.py` and `scenes/so101/` are
untouched.

**Decision.** `ScriptedSkillExecutor` constructs one `WeldGrasp` (`self.weld`), threaded
into `skills_scripted.run_pick`/`run_place`/`run_handoff` as a `weld` parameter:
- **`pick`'s GRIP** commands jaw closure, then calls `weld.attempt_grasp(arm, body_name)`
  every step until it returns `True` (frame recorded) or `GRIP_HOLD_FRAMES` (300) is
  exhausted, in which case the skill fails with the specific reason
  `weld_attach_failed_after_N_frames` -- distinguishable from an ordinary
  waypoint/collision failure.
- **`place`'s RELEASE** calls `weld.release(arm)` BEFORE commanding the jaw open, so the
  object is not kicked by the opening jaw's own moving collision geometry.
- **`handoff`** grips-and-attaches on `to_arm` the same way as `pick`, then adds a new
  gate BEFORE `from_arm` is ever released: `weld.is_holding(to_arm) == body_name` is
  checked explicitly; if false, the skill fails immediately with `handoff_transfer_failed`
  and `from_arm`'s weld is left untouched (object stays with `from_arm`, never ends up
  held by neither arm). Only past that gate does `from_arm` release (again,
  weld-then-jaws ordering) and the staggered retreat (`from_arm` first) proceed.
- **`SkillResult`** gained `weld_attach_frame: int | None` (the whole-skill-call frame at
  which `attempt_grasp` first returned `True`, or `None` if it never did) and
  `weld_active_at_end: bool` (`weld.is_holding` re-checked at return time), both with
  defaults so every pre-existing positional `SkillResult(...)` construction is unaffected.
- **`pick`'s success bar changes when `weld` is supplied**: `final_z > TABLE_SURFACE_Z +
  0.02` (a new constant, `WELD_PICK_SUCCESS_MARGIN_M`) AND `weld.is_holding(arm) ==
  body_name` -- an absolute-height-from-table-surface bar, per this task's own
  instructions, kept SEPARATE from the older initial-z-relative `PICK_LIFT_MARGIN_M`
  (0.03) that `run_pick` still uses when `weld=None`. `handoff`'s success similarly gains
  `weld.is_holding(to_arm) == body_name AND weld.is_holding(from_arm) is None` alongside
  its existing distance/lift check.

**Disclosed deviation: `ScriptedSkillExecutor.__init__` cannot construct `WeldGrasp`
eagerly.** The instruction as given was "`__init__` constructs it as `self.weld`" --
`WeldGrasp.__init__` requires a compiled `env` (it resolves equality-constraint/joint/
body ids against `env.model`), and `ScriptedSkillExecutor.__init__` takes no `env`
argument, matching every existing call site (`tests/test_skills.py`'s `executor()`
fixture, `scripts/run_skill.py`) which construct `ScriptedSkillExecutor()` bare and
supply `env` only later, per call, to `execute()`. `self.weld` therefore starts `None`
and is built lazily the first time `execute()` sees an `env` (`_ensure_weld`), rebuilt
only if a genuinely different `env` instance is later passed in. This is a correction to
the literal instruction, not a silent substitution -- recorded here per this task's own
"if the instructed text does not match what happened, correct it" rule.

**Verification (bm-ptl, `C:\Users\devcloud\project\ov_env\Scripts\python.exe`).**

- `pytest tests/test_skills.py`, BEFORE this commit's changes: **4 passed / 4 failed**
  (`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
  `test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b` fail;
  the other four pass) -- identical to Commit 1's own reported baseline.
- `pytest tests/test_skills.py`, AFTER: **4 passed / 4 failed, same four tests.** The
  drawer/mug-reach failures are byte-identical (kinematic reach limits this commit does
  not touch). The plate pick/place failures changed REASON, not outcome: previously
  `"did not lift plate: ... final_z=0.3523"`; now `weld_attach_failed_after_300_frames`
  (`final_z=0.3524`) -- the weld mechanism now engages and is exercised, and still does
  not attach for the plate's own rim-offset grasp point (see finding below); both are
  failures, so no regression.
- Isolated logic checks (FakeWeld stub, not the real `WeldGrasp` -- that mechanism's own
  correctness is ADR-029's job, already verified): confirmed `_dwell` breaks early at the
  exact step `attempt_grasp` first returns `True` and reports that step as `attach_frame`
  (`attach_on_call=7` -> `steps_taken=7, attach_frame=7`, `attempt_grasp` never called an
  8th time), reports `attach_frame=None` when it never attaches within budget, and that
  `run_pick`'s cumulative frame offset is correct (`grip_start_frames + local_index`
  measured as `1001` for a weld that attaches on its very first GRIP-dwell step, matching
  independently observed APPROACH+DESCEND frame counts from the real run below).

**New finding, measured directly, not assumed: for props whose grasp point IS reachable
(fork, water_bottle), `pick`'s GRIP now runs to completion but `WeldGrasp`'s own
proximity gate (`distance_threshold_m=0.05`, unmodified default) is missed by the time
the closure gate opens.** `pick(A, fork)` and `pick(A, water_bottle)` both return
`weld_attach_failed_after_300_frames` (frames_used=1300; GRIP_HOLD_FRAMES=300 exhausted).
A direct instrumented replay of `pick(A, fork)`'s GRIP dwell (`armA_gripper` BODY-to-fork
BODY distance, `armA_gripper` JOINT qpos, sampled every 20 steps) found:

| step | gripper qpos | body-to-body distance |
|---|---|---|
| 0 (jaw still open) | 1.7449 | 0.0299 m |
| 100 | 0.8876 | 0.0464 m |
| 120 | 0.6697 | 0.0501 m |
| 140 | 0.4506 | 0.0537 m |
| 160 | 0.2309 (closure gate now open, <0.3) | 0.0574 m |
| 299 (fully closed) | -0.1745 | 0.0795 m |

The distance grows monotonically, from 0.0299 m (well inside the 0.05 m gate, jaw fully
open) to 0.0795 m (jaw fully closed) -- and it crosses above 0.05 m (between step 120 and
140) BEFORE the closure gate opens (between step 140 and 160). The two gates' passing
windows do not overlap at these thresholds for this prop: by the time the jaw is closed
enough to attempt attach, the fixed-jaw body has already drifted too far from the object
to pass the proximity gate. **Root cause, not merely observed:** `ik.solve_position_ik`
(unmodified, out of scope) targets the `armX_gripperframe` SITE (the pinch point), not
the `armX_gripper` BODY `WeldGrasp`'s proximity gate reads (the naming-trap distinction
`grasp.py`'s own docstring names). As the jaw closes, the redundant 5-DOF solve keeps the
pinch-point SITE pinned at the grasp target by rotating the wrist -- and that same wrist
rotation carries the fixed-jaw BODY away from the site (and therefore away from the
object) at roughly 1.7 mm per closure step. This is the SAME site-vs-body divergence
ADR-029's own docstring already documents for a different maneuver (driving the pinch
point upward during LIFT); here it shows up during jaw CLOSURE instead. `pick(A, mug)`
fails earlier and for an unrelated, already-documented reason (`waypoint 1 (approach)
failed [convergence]` -- the pre-existing kinematic reach limit ADR-024/ADR-027 recorded).
`handoff(A, B, fork)` and `place(A, fork, table)` both fail as a direct, expected
consequence of the nested `pick` failing the same way (`weld_attach_failed_after_300_frames`
surfaces through `"handoff aborted: pick by arm A failed (...)"` /
`"place aborted: pick failed (...)"`), never reaching their own weld-specific gates
(`handoff`'s transfer check, `place`'s release-before-open ordering) in this run.

**Per this task's own instruction, this gap was NOT closed by loosening
`attempt_grasp`'s gates.** `distance_threshold_m`/`closure_threshold` were left at
`WeldGrasp`'s own defaults (0.05 m / 0.3 rad) exactly as ADR-029 designed and verified
them; `scripts/probe_weld_grasp.py`'s own positive-path verification of the fork used a
DIFFERENT technique (`_drive_gripper_body_to_target`, driving the gripper BODY directly)
than `skills_scripted.py`'s site-targeting `ik.solve_position_ik` path -- the divergence
between the mechanism's own verified test harness and the skill layer's actual IK-driving
pattern is this commit's real finding, not a wiring defect to be patched around by
loosening a threshold.

**`_hold_ctrl` drift (Commit 1) remains an open, compounding risk specifically for
`handoff`, not newly measured this session:** because no attach ever completed in this
run, `handoff`'s from-arm-idle-while-to-arm-moves window (where the drift matters most,
per Commit 1's own entry) was never actually reached with an object held. The risk stands
exactly as Commit 1 recorded it -- not re-measured, not resolved.

**Renders.** `docs/images/m06-phase2-fork-lifted.png` (after `pick(A, fork)`) and
`docs/images/m06-phase2-handoff-complete.png` (after `handoff(A, B, fork)`), both front
camera, 1280x720, both produced. **Neither shows what its filename claims, reported
plainly rather than implied:** both renders are visually near-identical -- arm A hovering
at clearance height above the STILL-RESTING fork (RETREAT ran after a failed GRIP, per
`run_pick`'s structure), arm B still at its rest pose off to the side (`handoff` aborted
inside the nested `from_arm` pick, before arm B's own APPROACH waypoint ever ran). At this
camera's distance the fork itself is a few pixels and not reliably distinguishable from
the tabletop by eye in either image -- this is stated here rather than left to imply a
visual confirmation neither render actually provides.

**Consequences.** Physical grasping stays abstracted (ADR-029) and, per ADR-015, the
README must disclose it -- **not done in this commit**: `README.md` is currently a
placeholder status doc owned by docs-writer per the agent assignment model
(`PLAN.md` section 2), and updating it is out of Builder's role; flagged here so it is
not silently dropped. `pick`/`place`/`handoff` are now wired end-to-end through the weld
abstraction and will complete successfully for a prop whose grasp geometry keeps the two
`WeldGrasp` gates' passing windows overlapping -- fork and water_bottle, as wired and
measured this session, do not; whether any prop's grasp offset can be retargeted to
produce an overlapping window (without touching `ik.py`/`grasp.py`) is an open follow-up,
not attempted here (out of this commit's scope: wiring, not re-tuning grasp geometry).

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

**Correction (Sept 13, 2026, M06 Phase 2 follow-up session).** A later
session was handed a claim that the jaw-body mesh disable this entry (and
`8f09f8c`, below) describes was still incomplete -- specifically, that
`sts3215_03a_v1`, `wrist_roll_follower_so101_v1` and `moving_jaw_so101_v1`
were still `COLLIDABLE` in the compiled model. **Checked directly, not
assumed, on both a development laptop and bm-ptl** (`mujoco==3.2.7`,
`model.geom_contype`/`geom_conaffinity` read for every geom on the relevant
bodies): all three meshes already report `contype=0 conaffinity=0` on both
arms, and both finger pads remain collidable, exactly as this entry and
`8f09f8c` intended. `8f09f8c`'s "complete jaw collision disable" (see that
entry below) was, in fact, complete -- the disable was never incomplete, and
no further code change was needed or made. The full audit (why the given
premise did not reproduce, and what was checked) is recorded in the "M06
Phase 2 follow-up" entry at the top of this file, above ADR-030. This
correction exists so a future reader who finds this ADR's own "no further
action taken" language does not go looking for a still-open gap that was
never there.

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
