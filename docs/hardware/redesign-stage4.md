# Redesign Stage 4 — swept-path collision gate, diagnosis, geometry re-placement attempt (ADR-061)

**Branch:** `redesign` (`master` unchanged at `20e1012`). **Cap:** 180 minutes, hard.
**Provenance:** all numbers in this report were produced by re-running the
same scripts on `bm-ptl` after pushing this stage's commit, per the task's
provenance rule; local (laptop) runs were used only during development to
iterate quickly (`mujoco` loads locally on this machine, unlike the Sept 11
Smart-App-Control block recorded for M02) and are not themselves cited as
results.

## Where this stage starts

Stage 3 (ADR-060): 1 of 8 skills pass at seed 0, versus 4 of 9 on master.
Three regressions (`pick(A, fork)`, `place(A, fork, table)`,
`handoff(A→B, fork)`) traced to one root cause: a waypoint-1 approach
collision Stage 2's static-endpoint gate never checked for, because it only
validated waypoint *targets*, never the swept motion between them.

## Step 1 — build and validate the gate

`scripts/check_swept_path.py` exposes:

```
swept_path_clear(model, data, arm, start_qpos, target_xyz,
                  n_steps=25, penetration_tol=-0.005,
                  target_geom_names=None, final_exempt_steps=3,
                  num_endpoint_seeds=8, seed_noise_rad=0.6,
                  implausible_penetration_m=-0.15)
  -> (clear, worst_penetration, worst_step, contact_pairs)
```

It interpolates in **joint space** between `start_qpos` and one IK solution
at `target_xyz` (never Cartesian-interpolate-and-resolve, per the task's
explicit instruction), evaluates RAW `data.contact` at every interpolated
step (never the ADR-058-distrusted `table_top` mesh filter), and reports
contact pairs by geom name (falling back to `{body}#geom{id}` for the
upstream asset's many unnamed arm mesh geoms).

**Three refinements were added beyond the literal single-IK-solution
spec, each because the literal version measurably failed to reproduce
Stage 3's own reported failure (a genuine hard-stop risk), not by
choice:**

1. **`num_endpoint_seeds` (default 8).** `ik.solve_position_ik` is a 5-DoF
   solve against a 3-DoF target (ADR-024) — a redundant null space with no
   regularization. Measured directly: a single unperturbed joint-space
   interpolation for `pick(A, fork)` waypoint 1 is 0/101-steps clear, while
   the REAL 500-step closed loop (same start, same target) drifts to a
   *different* null-space branch and stalls in a genuine
   `armA`-vs-`table_top` + cross-arm collision by step 168 — exactly what
   Stage 3 reported. A 30-trial sweep of randomly perturbed starting seeds
   (same start/target) found 8/30 land on colliding branches. A single
   endpoint sample is not representative of what the real controller can
   reach; sampling several (seed 0 always the unperturbed solve) is.
2. **`implausible_penetration_m` (default -0.15).** Raw MuJoCo contact
   data, at some sampled joint configurations, reproduces the SAME
   convex-hull mesh artifact ADR-058 (-0.39 m) and ADR-035 (-0.222 m)
   already documented on non-swept poses. -0.15 m sits above every
   genuine collision this stage measured (worst real one: -0.098 m) and
   below both documented artifact magnitudes — contacts past it are
   reported (never hidden) but flagged `suspected_artifact` and excluded
   from `clear`/`worst_penetration`.
3. **Crush-threshold-aware target exemption (`_effective_tol`).** Mirrors
   `skills_scripted.CRUSH_THRESHOLD_M` (-0.02 m) for contacts between the
   swept arm and its OWN declared target, at every step — not just a
   finger-pad-only, final-steps-only exemption — matching exactly what
   `skills_scripted._prop_collision_violations` already does for the real
   controller. Without this, several genuinely fine configurations were
   flagged purely because the arm's wrist (not the finger pads) grazes a
   wide/tall prop during final approach, which the real controller already
   tolerates up to -0.02 m.

### Hard-stop validation (`scripts/validate_swept_path_gate.py`)

Both required checks, using `pick(A, fork)` waypoint 1 (home → hover,
target `(-0.065, 0.100, 0.440)` — reproduced from `skills_scripted`'s own
constants, not hand-transcribed, and confirmed to match Stage 3's own
reported target exactly):

| Check | Geometry | Result | Detail |
|---|---|---|---|
| 1 (hard stop) | master (temporarily checked out, restored after) | **CLEAR** | worst_penetration = -0.0019 m |
| 2 | redesign, current HEAD | **COLLISION** | worst_penetration = -0.0114 m at step 21; `armA_wrist` vs `armB_upper_arm`/`armB_shoulder` (cross-arm) and `armA_lower_arm` vs `armB_upper_arm` |

Check 1 passes → **not a stop condition.** Check 2 finds a real collision
whose category (cross-arm) matches Stage 3's own report
("`cross_arm contacts=1`"); the specific geom pair differs slightly from
the one Stage 3's 500-step physical trace happened to stall on (expected —
see point 1 above: different null-space branches produce different exact
geom pairs, same collision class). Branch confirmed `redesign` before and
after; `so101_dual_table.xml` confirmed restored to redesign's committed
bytes after the master-XML swap (`git status` empty).

## Step 2 — diagnose

`scripts/diagnose_swept_path.py` monkeypatches `skills_scripted._run_waypoint`
(captures `(arm, start_qpos, target_xyz)` per real waypoint, then teleports
to the IK solution so the skill always advances — never edits the file:
`git diff master..redesign -- src/bimanual/control/skills_scripted.py`
still shows exactly the one `HANDOFF_POSITION_XYZ` line, verified below),
`_run_dwell` (GRIP/RELEASE are stationary holds, not swept motion — a
no-op that only sets gripper ctrl) and `run_pick` (forces `success=True` so
`run_place`/`run_handoff`'s own `if not pick_result.success: return` gates
don't truncate the captured chain before their later, geometrically
distinct waypoints ever run). This captured the FULL intended approach
sequence for all 8 skills — 41 waypoints total — even for skills that fail
early in real execution.

**Result on Stage 3's geometry: 31 of 41 waypoints collide.**

| Category | Colliding waypoints |
|---|---|
| (a) arm-vs-prop | 13 |
| (b) arm-vs-table | 18 |
| (c) arm-vs-arm (cross-arm) | 7 |
| (d) arm self-collision | 6 |

**Dominant category by count: (b) arm-vs-table.** 18 occurrences, nearly
all in a tight, shallow band (-0.0057 to -0.0060 m — just past the -0.005 m
bar) recurring at almost every DESCEND-to-grasp/destination waypoint across
every skill, at very different (x, y) positions — a signature of the
static finger pad grazing the table whenever the pinch point height sits
close to `TABLE_SURFACE_Z`, not a single prop's placement.

**Important nuance the count alone hides:** (a) arm-vs-prop contains the
single deepest, most severe violations in the whole dataset — `pick(A,
water_bottle)`'s DESCEND and RETREAT both show `armA_wrist` (not a finger
pad) penetrating `water_bottle_body` by **-0.098 m** and **-0.052 m**,
consistent across every sampled seed (not a rare null-space branch) —
these exceed even `CRUSH_THRESHOLD_M`, so a real physical run would likely
treat them as a genuine crush, not merely a gate false-alarm. (c)/(d) are
concentrated almost entirely in `handoff`'s Phase 2/3 transfer-point
approach and the first hop away from home — consistent with ADR-058's own
finding that the two arms' shared workspace band is fundamentally narrow
at 0.40 m separation.

Full per-waypoint table: `out/stage4_swept_path_diagnosis.json` (bm-ptl
run referenced in this commit).

## Step 3 — re-place under a six-item gate

`scripts/verify_stage4_gate.py` extends Stage 2's 5-item gate
(`verify_stage2_gate.py`, re-run unchanged) with **item 6**: every skill's
full approach sequence (Step 2's capture) is swept-path clear from home.
**On Stage 3's own geometry, item 6 fails for all 8 skills (0/8)** — the
six-item gate is materially stricter than the 1-of-8 functional pass rate
Stage 3 measured, because the gate's 8-seed adversarial sampling flags
theoretical null-space branches the real single-seed, seed-0 closed loop
does not necessarily visit.

**Remedy attempted: home-pose re-search scored by swept-path clearance**
(`scripts/search_home_keyframe_stage4.py`), addressing categories (c)/(d),
which concentrate at the very first hop away from home. Reuses
`search_home_keyframe.py`'s own `evaluate_candidate` UNCHANGED for the
static criteria (ADR-059/060's own precedent), adding a swept-path score
(home → every prop's grasp point + both arms' handoff point) evaluated at
reduced fidelity during the search (`num_endpoint_seeds=4`, `n_steps=15`)
and re-verified at full fidelity for the top 10.

**300 candidates tried; 187 passed the static (ADR-059/060) criteria.**
Score distribution (swept violations out of 7 targets, at search fidelity,
among static-passing candidates):

| violations | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|
| candidates | 3 | 47 | 62 | 54 | 14 | 7 |

The best candidate (full-fidelity re-verified) reduced swept violations
from Stage 3's fold's 5/7 to **3/7** — `fork`, `spoon` and **both handoff
directions** became fully clear; `plate`/`mug`/`bottle` remained
violations (the arm's own wrist crushing its OWN target beyond
`CRUSH_THRESHOLD_M` during final approach — a prop-geometry-vs-uncontrolled-
approach-angle conflict, ADR-024, not resolved by the home fold).

**This candidate was regenerated into the scene and RETESTED for real —
and REGRESSED functional success from 1/8 to 0/8**, with every skill now
failing at GRIP (`weld_attach_failed_after_300_frames`), including
`pick(A, fork)`, whose specific collision the new fold WAS measured to fix.
Root cause, reasoned from ADR-024: `ik.solve_position_ik` never controls
end-effector orientation — "whatever falls out of the redundant 5-joint
solve is accepted as-is." A different home fold seeds every downstream IK
solve from a different starting configuration, changing which orientation
the redundant solver lands on at each grasp target even when the TARGET
POSITION (what the swept-path gate scores) is identical and
collision-clear. **The swept-path gate has no visibility into grasp
orientation — a real, previously unknown blind spot** — so optimizing a
home fold against it alone can trade a working grasp angle for a
collision-clear but ungraspable one.

**This change was reverted.** `scripts/gen_dual_scene.py`'s home-fold
constants are back at ADR-060's values (the attempted alternative is
recorded in a comment, not silently discarded); confirmed
`git diff -- src/bimanual/sim/assets/so101_dual_table.xml` is empty against
the redesign-committed geometry after regenerating.

**No further geometry change (prop repositioning for categories a/b) was
attempted after this regression** — given the remaining time in this
stage's cap and the freshly-demonstrated risk that a gate-driven change can
regress real behaviour in a way the gate cannot see, further blind
iteration against the same gate was judged not worth the risk without a
way to also check grasp orientation, which is out of this stage's scope
(would require touching `ik.py`, frozen).

**Six-item gate result: 0 of 8, on both the original and the attempted
alternative geometry.** This does not reach the ≥6/8 target, and does not
exceed master's 4/9 functional count either. Per the task's own explicit
instruction, this is reported plainly as the signal to stop, not to keep
iterating.

## Step 4 — retest and report

Full eight-skill retest, seed 0, oracle mode, `cameras=None`, fresh
env + fresh executor per skill (ADR-047), receiver-first handoff calling
convention — identical harness to Stage 3
(`scripts/stage4_eight_skill_retest.py`, `git diff` against
`stage3_eight_skill_retest.py` is a pure rename/docstring change; skill
list, seed and per-skill call logic are byte-identical). Because Stage 4's
net geometry change is nil (the one attempted change was reverted), these
numbers are — as expected and confirmed by direct re-run — **identical to
Stage 3's**.

| skill | master | redesign Stage 3 | redesign Stage 4 | six-item gate (Stage 4) |
|---|---|---|---|---|
| `pick(A, fork)` | PASS | FAIL (waypoint 1 collision) | FAIL (same) | FAIL — `a_arm_vs_prop`+`c_arm_vs_arm` at wp1 (-0.0114 m), `b_arm_vs_table` at wp2/wp4 (-0.0058 m) |
| `place(A, fork, table)` | PASS | FAIL (nested pick fails) | FAIL (same) | FAIL — same wp1 + `b_arm_vs_table`/`c_arm_vs_arm` at destination approach (-0.0473 m) |
| `pick(A, water_bottle)` | (not in master's 9-skill list*) | PASS | PASS | FAIL — `a_arm_vs_prop`: `armA_wrist` vs `water_bottle_body`, -0.0983 m / -0.0522 m |
| `place(A, water_bottle, table)` | FAIL (genuine IK boundary, 0.0138 m) | FAIL (same boundary, 0.0150 m) | FAIL (same) | FAIL — same bottle collision plus `a_arm_vs_prop` at destination (-0.0438 m) |
| `pick(A, mug)` | FAIL (IK boundary, 0.0532 m) | FAIL (`weld_attach_failed`, reachability now fixed) | FAIL (same) | FAIL — `d_self_collision` at wp1 (-0.0131 m), `a_arm_vs_prop`+`b_arm_vs_table` at wp2/wp3 (-0.0358/-0.0339 m) |
| `place(A, mug, table)` | FAIL | FAIL (same nested pick failure) | FAIL (same) | FAIL — same as pick(mug) plus `b_arm_vs_table` at destination |
| `handoff(A→B, fork)` | PASS | FAIL (phase 1 = pick(fork) collision) | FAIL (same) | FAIL — same wp1 collision plus `c_arm_vs_arm`+`d_self_collision` at transfer approach (-0.0192 m) |
| `handoff(B→A, fork)` | FAIL (IK boundary, 0.0954 m) | FAIL (near-miss collision, -0.0053 m) | FAIL (same) | FAIL — `b_arm_vs_table`/`a_arm_vs_prop` at wp1/wp2, `c_arm_vs_arm`+`d_self_collision` at transfer approach (-0.0333 m) |

*`open_drawer` removed in Stage 2 (ADR-059); master's 9-skill list included
it, so the two headline counts (4/9 vs 1/8) are not directly comparable
skill-for-skill, per ADR-060's own note.

**Final: 1/8 pass (unchanged from Stage 3), six-item gate: 0/8.**

## What this stage actually accomplished, stated plainly

- A swept-path collision gate was built, and — critically — VALIDATED
  against ground truth on both master's working geometry (correctly
  reports clear) and redesign's known-broken geometry (correctly reports
  the same collision class Stage 3's real run hit). This closes a real,
  previously-missing gate-coverage gap Stage 3 identified.
- The gate diagnosed the 8-skill approach-sequence collision landscape in
  full (41 waypoints, all 8 skills), not just the 3 regressions Stage 3's
  functional test happened to surface — most other skills that "pass" or
  "fail for an unrelated reason" also have latent swept-path collisions
  the functional test's single seed simply did not trigger.
- One remedy (home-pose re-search scored by the new gate) was attempted,
  measured to improve the gate's own metric, and measured — by actually
  re-running the real 8-skill test, not assumed — to REGRESS real
  functional success. This is reported as a genuine finding about the
  gate's blind spot (grasp orientation, ADR-024), not hidden to make the
  stage look more successful.
- Net result: geometry is unchanged from Stage 3 (1/8 functional,
  identical numbers); the six-item gate sits at 0/8, short of both the
  ≥6/8 target and master's 4/9 functional count. Per the task's own rule,
  this is reported as the stopping condition, not chased further.

## Files

- `scripts/check_swept_path.py` — the gate (`swept_path_clear`).
- `scripts/validate_swept_path_gate.py` — Step 1's hard-stop validation.
- `scripts/diagnose_swept_path.py` — Step 2's per-skill, per-waypoint diagnosis.
- `scripts/search_home_keyframe_stage4.py` — Step 3's attempted (reverted) home-pose remedy.
- `scripts/verify_stage4_gate.py` — the six-item gate.
- `scripts/stage4_eight_skill_retest.py` — Step 4's functional retest.
- `scripts/gen_dual_scene.py` — home-fold constants: attempted change recorded in comment, reverted to ADR-060's values.
- `out/stage4_swept_path_diagnosis.json`, `out/stage4_six_item_gate.json`, `out/stage4_home_keyframe_search.json`, `out/stage4_eight_skill_retest.json` — raw data (bm-ptl run).
