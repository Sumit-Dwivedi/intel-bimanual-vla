# Redesign Stage 3 — home-pose validation, handoff-constant swap, eight-skill retest at 0.40 m geometry (ADR-060)

**Provenance:** bm-ptl (ADR-047: laptop and bm-ptl diverge in the last 1-3
decimal digits on floating-point results; bm-ptl is the reference machine
for every number in this document). **Branch:** `redesign`. `master`
unchanged at `20e1012`.

**Modifies:** `src/bimanual/control/skills_scripted.py` (ONE line —
`HANDOFF_POSITION_XYZ`), `scripts/gen_dual_scene.py` (the five `HOME_*`
constants), `src/bimanual/sim/assets/so101_dual_table.xml` (regenerated from
the above), three new scripts (`scripts/validate_home_pose_stage3.py`,
`scripts/stage3_eight_skill_retest.py`,
`scripts/stage3_multiseed_diagnostic.py`), this file, `DECISIONS.md`,
`ARCHITECTURE.md`. **Does NOT modify** `grasp.py`, `ik.py`, `executor.py`,
`env.py`, and does not touch `skills_scripted.py` beyond the single constant
named above.

---

## Step 0 — Home-pose robustness validation

Stage 2 (ADR-059) accepted its home fold after scoring only **2**
candidates (ADR-026's old fold, which failed, then the very next random
draw, which passed everything). That is too small a sample to tell "typical
passing configuration" apart from "lucky draw near a boundary."

`scripts/validate_home_pose_stage3.py` re-uses Stage 2's own
`evaluate_candidate` scoring (imported from `search_home_keyframe.py`, not
reimplemented) but does **not** stop at the first pass: it scores all 200
candidates from the same reproducible stream and ranks every passing one by
the sum of its 7 per-target IK residuals (5 props + both arms' handoff
point) — a "how comfortably passing" margin score, lower is better.

**Result: 117/200 pass (58.5%)** — comfortably above the 5% near-degenerate
floor, so the criteria are not degenerate. Stage 2's own pose (re-evaluated
explicitly, not merely wherever it happened to land in this run's stream)
**passes all 5 criteria** (0 violations) but **ranks 91st of 117** passing
candidates by total residual (0.02451 m) — solidly in the **bottom
quartile** (top-quartile threshold: 0.02093 m), and its single worst
per-target residual (fork, 0.00489 m) sat uncomfortably close to the 0.005 m
gate.

| | Stage 2's pose (ADR-059) | Best of 200 (index 11) |
|---|---:|---:|
| n_violations | 0 | 0 |
| total IK residual (7 targets) | 0.02451 m | 0.01780 m |
| worst single-target residual | 0.00489 m (fork) | 0.00350 m (bottle) |
| rank among 117 passing | 91 / 117 | 1 / 117 |

Per this stage's own decision rule ("ranks poorly → adopt the best
candidate, re-run Stage 2's five-item gate"), **swapped** to candidate 11:
`shoulder_pan=0.15753, shoulder_lift=-0.99304, elbow_flex=-0.90416,
wrist_flex=0.05731, wrist_roll=-0.24666`. `scripts/gen_dual_scene.py`'s
`HOME_*` constants were updated to these values, the scene regenerated, and
`scripts/verify_stage2_gate.py` (Stage 2's own 5-item gate) re-run against
the new keyframe:

```
[1] model compiles: PASS
[2] home keyframe contacts >1mm: PASS (0 found)
[3] prop grasp-point IK <0.005 from home: PASS
    plate    arm A: residual=0.00281 OK
    mug      arm B: residual=0.00159 OK
    fork     arm A: residual=0.00167 OK
    spoon    arm B: residual=0.00285 OK
    bottle   arm A: residual=0.00350 OK
[4] handoff-point IK <0.005, seeded from own approach pose: PASS
    arm A: hover_residual=0.00183 handoff_residual=0.00430 OK
    arm B: hover_residual=0.00278 handoff_residual=0.00443 OK
[5] cross-arm contact at simultaneous handoff config, below -0.005m: PASS (0 violations)
OVERALL: ALL GATE ITEMS PASS
```

All 8 skills below start from this swapped pose, not Stage 2's original.

---

## Step 1 — Handoff constant moved

`skills_scripted.py:485` diff (confirmed single-line before committing):

```diff
-HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.43)
+HANDOFF_POSITION_XYZ = (-0.035, 0.0, 0.49)  # ADR-060: Stage 2's (ADR-059) measured feasible centroid at 0.40 m base separation
```

**Isolation test (corrected per this task's own note — `frames_used=6610`
is NOT expected to hold, since the transfer point moved).** With master's
XML (`git checkout 20e1012 -- src/.../so101_dual_table.xml`, restored
afterward — confirmed below) plus the new constant,
`scripts/verify_adr038_skills.py`:

| skill | expected (unaffected by the edit) | measured |
|---|---|---|
| `pick(A, fork)` final z | 0.3989 | **0.3989** — identical |
| `place(A, fork, table)` final z | 0.3588 | **0.3588** — identical |
| `pick(A, 'bottle')` final z | 0.6192 | **0.6192** — identical |

All three unaffected skills reproduce byte-identically, confirming the edit
touched nothing it shouldn't. `handoff(A→B, fork)` — which DOES read
`HANDOFF_POSITION_XYZ` — moved as expected: lateral separation 0.1946 m →
**0.0399 m**, `frames_used` 6610 → **3655**, and outcome flipped
success → **FAIL** (`phase 3 (to_arm approach) failed [convergence (IK
residual=0.0707 m) ...  joint_limit_margin=0.000057 rad]`). This is the
expected consequence of testing a constant measured for the NEW 0.40 m
geometry against the OLD 0.50 m master geometry, not a defect: the new
transfer point is only guaranteed reachable in the scene it was measured
from (Stage 2's XML), which is exactly what this isolation test is designed
to show by NOT holding master's geometry compatible with it.

XML restored to `redesign`'s Stage-3-regenerated version afterward;
confirmed via `git diff --stat` (the XML shows as modified relative to
`a80ad5f`, matching the Step 0 swap, not master's content) and a fresh
`verify_stage2_gate.py` run (all 5 items PASS, handoff residuals
0.00430/0.00443 — reflecting the new constant on the new geometry).

---

## Step 2 — Eight-skill retest, seed 0, oracle, `cameras=None`

**Note on `cameras=[]` vs `cameras=None`.** The task brief said
`cameras=[]`. `bimanual/control/executor.py:212` (frozen, unmodified)
asserts `default_cameras is None` for oracle-mode `ScriptedSkillExecutor`
and raises `AssertionError` for ANY non-`None` camera list, including an
empty one — reproduced directly (`AssertionError: oracle-mode
ScriptedSkillExecutor ... requires TableSettingEnv(cameras=None)`) before
switching to `cameras=None`, which is also what every other oracle-mode
script in this repo already uses (`run_skill.py`,
`verify_adr038_skills.py`, `probe_multiseed_diagnostic.py`). Both mean the
same thing operationally — zero camera renders per step, oracle-only
state — so `cameras=None` was used to satisfy the frozen assertion rather
than edit `executor.py`.

**Fresh `TableSettingEnv` + fresh `ScriptedSkillExecutor` per skill**
(`scripts/stage3_eight_skill_retest.py`), per ADR-047.

| skill | master @ seed 0 | redesign @ seed 0 | delta |
|---|---|---|---|
| `pick(A, fork)` | **PASS**, z 0.3560→0.3989 | **FAIL**, `frames_used=500`, waypoint 1 (approach) collision (cross_arm contacts=1; armA-vs-table_top contacts=1); fork z 0.3560→0.3538 (no lift) | **REGRESSED** |
| `place(A, fork, table)` | **PASS**, final z=0.3588 | **FAIL**, `frames_used=500`, aborted — nested pick fails identically | **REGRESSED** (same root cause) |
| `pick(A, water_bottle)` | **PASS**, z 0.4400→0.6192 | **PASS**, `frames_used=1742`, `weld_attach_frame=1242`, z 0.4400→0.6256 (lift +0.1856) | unchanged outcome (numbers shifted with geometry) |
| `place(A, water_bottle, table)` | **FAIL**, dest IK 0.0138 m ≥ 0.010 (ADR-034; genuine boundary, ADR-057, 32/32 seeds identical) | **FAIL**, `frames_used=2242`, waypoint 1 (approach destination) IK residual=0.0150 m ≥ 0.010 | still FAIL, same failure mode, residual slightly worse |
| `pick(A, mug)` | **FAIL**, waypoint 1 (approach) convergence failure, residual 0.0532 m (genuine boundary) | **FAIL**, `frames_used=1300`, `weld_attach_failed_after_300_frames` (waypoint 1 AND descend already converge) | still FAIL, **different mechanism** — reachability fixed, grip mechanism now the blocker |
| `place(A, mug, table)` | untested | **FAIL**, `frames_used=1300`, aborted — nested pick fails identically | newly tested, FAIL (dependent) |
| `handoff(A→B, fork)` | **PASS**, `frames_used=6610` | **FAIL**, `frames_used=500`, phase 1 (`from_arm` pick) fails identically to `pick(A, fork)`'s own waypoint 1 collision | **REGRESSED** (same root cause as `pick(A, fork)`) |
| `handoff(B→A, fork)` | **FAIL**, Phase 1 IK 0.0954 m (ADR-057 Case 1, genuine boundary, 32/32 seeds identical) | **FAIL**, `frames_used=279`, phase 1 (arm B picks fork) near-miss collision (arm-vs-prop: plate, dist=-0.0053 m; threshold=-0.005 m) | still FAIL, **different mechanism** — no longer a hard IK boundary (see diagnostic below) |

**Score: 1 of 8 pass at the new geometry, versus 4 of 9 on master**
(`open_drawer`, the 9th, was removed in Stage 2 on measured evidence and is
out of scope here). Of the 4 skills that passed on master, **3 regressed**
(`pick(A, fork)`, `place(A, fork, table)`, `handoff(A→B, fork)`) — all
sharing one root cause. No previously-broken skill fully recovered, but two
(`pick(A, mug)`, `handoff(B→A, fork)`) changed failure MECHANISM in ways
described below.

### Root-cause consolidation

Only **4 distinct failures** exist across the 8 results (3 pairs share a
root cause with a skill already diagnosed):

1. `pick(A, fork)` waypoint 1 → also breaks `place(A, fork, table)`
   (nested pick) and `handoff(A→B, fork)` phase 1 (`run_pick(A, fork)`
   verbatim).
2. `pick(A, mug)` grip stage → also breaks `place(A, mug, table)` (nested
   pick).
3. `place(A, water_bottle, table)` destination approach — independent.
4. `handoff(B→A, fork)` phase 1 (arm B picks fork) — independent.

---

## Multi-seed diagnostics (ADR-056, `num_seeds=32`), one per root cause

`scripts/stage3_multiseed_diagnostic.py` reuses
`probe_multiseed_diagnostic.py`'s own `perturbed_residuals`/`summarize`
(ADR-057's exact perturbation formula), never reimplemented.

### (1) `pick(A, fork)` waypoint-1 approach — target `(-0.065, 0.100, 0.440)`, arm A

Documented failure type: **collision**, not a reported convergence failure.
32-seed residuals range **0.0029 – 0.0099 m** (one outlier seed at 0.291 m,
a genuine bad local minimum for that seed only) — i.e. `num_seeds=1` alone
(0.00638 m) is **already comfortably under the 0.01 m IK tolerance**.

**Verdict: does NOT plateau — but this was never an IK reachability
problem in the first place.** The residual is low at essentially every
seed; the actual blocker is a physical collision at an otherwise fully
IK-reachable pose. This is a **geometry/route problem introduced by the new
0.40 m separation** (the shared workspace band is now only ~14×15 cm,
Stage 2's own measurement — a tight corridor for an approach sweep to cross
without clipping the table edge or the other arm), not a kinematic
boundary. Stage 2's 5-item gate checked the HOME pose and IK-CONVERGED
endpoint configs for collision, but never checked the swept APPROACH
waypoint itself — a real gap in that gate's coverage, surfaced here for the
first time.

### (2) `pick(A, mug)` waypoint-1 approach — target `(0.098, -0.080, 0.470)`, arm A

Documented failure type: `weld_attach_failed_after_300_frames` — a
GRIP/weld-mechanism failure (waypoint 1 and the descend/grip waypoints
already converge; `frames_used=1300` shows this directly). 32-seed
residuals range **0.0056 – 0.0106 m**, no outlier plateau; `num_seeds=1`
(0.00864 m) already converges under the 0.01 m tolerance.

**Verdict: does NOT plateau, and was never an IK problem.** Compare to
master's OWN documented number for this exact waypoint: residual **0.0532
m**, a genuine boundary failure there. **Stage 2's redesign fixed mug's
reachability outright** (0.0532 m → ~0.006–0.01 m) — but a *different*,
downstream grip/weld-mechanism failure now blocks success instead.
Multi-seed IK restarts cannot rescue a failure that was never an IK
failure (same caveat `probe_multiseed_diagnostic.py`'s own target (e)
states).

### (3) `place(A, water_bottle, table)` destination approach — target `(0.300, 0.0529, 0.430)`, arm A

Documented failure type: convergence failure, residual 0.0150 m ≥ 0.010 m.
32-seed residuals: **0.01502 – 0.01509 m** — a dead-flat plateau, variance
~3×10⁻¹⁰.

**Verdict: PLATEAUS — still unreachable.** This is the textbook
"genuine boundary" signature (identical to master's own pre-established
finding for this same failure, ADR-034/ADR-057: 0.0138 m there, 32/32
seeds identical). Stage 2's redesign did **not** fix this boundary — if
anything it is very slightly worse (0.0150 m vs 0.0138 m). The target's `x`
component also sits exactly at the `[-0.30, 0.30]` table clamp, confirming
this is a table-extent limit compounding the reach limit, not solely an
arm-kinematics one.

### (4) `handoff(B→A, fork)` phase 1 (arm B picks fork) — target `(-0.065, 0.100, 0.440)`, arm B

Documented failure type: near-miss collision (arm-vs-prop: plate,
dist=-0.0053 m; threshold=-0.005 m — 0.3 mm past the gate). 32-seed
residuals are **bimodal**: roughly half cluster at 0.0038–0.0100 m
(comfortably converged) and half cluster tightly around **0.0303–0.0308 m**
(a genuine, well-separated local-minimum basin). `num_seeds=1` (seed 0,
0.00458 m) already converges comfortably.

**Verdict: VARIES and DROPS — a local-minimum / skill-logic signature at
genuinely reachable geometry, not a hard boundary.** Compare to master's
own documented number for this same phase: residual **0.0954 m**, 32/32
seeds identical (ADR-057 Case 1 — a hard boundary there). At the new
geometry this is markedly better: the position itself is reachable by
multiple distinct joint configurations (redundant 5-DOF IK), several of
which converge far below tolerance. The specific configuration
`_run_waypoint`'s real physical drive lands in happens to brush the plate
by a fraction of a millimetre — plausibly fixable by re-routing or
re-staging arm B's approach (out of scope here per this stage's "no skill
edits beyond Step 1" rule), not by anything multi-seed IK restarts alone
can do (the collision check runs on the one physically-driven trajectory,
not on alternate static IK solutions).

---

## Summary

- **1 of 8 skills pass** at the new 0.40 m geometry / new home pose / new
  handoff constant, versus **4 of 9 on master** (`open_drawer` excluded,
  removed in Stage 2).
- **3 previously-passing skills regressed**: `pick(A, fork)`,
  `place(A, fork, table)`, `handoff(A→B, fork)` — all one root cause, a
  waypoint-1 approach **collision** (not an IK failure) introduced by the
  tighter 0.40 m shared workspace. Stage 2's 5-item gate did not check the
  swept approach path, only static endpoint configs — a real coverage gap
  this stage surfaces.
- **No previously-broken skill started fully passing**, but two changed
  failure MECHANISM in a materially better direction:
  - `pick(A, mug)`: master's hard IK boundary (residual 0.0532 m) is now
    **fully resolved** (residual ~0.006–0.01 m); the new blocker is a
    grip/weld-mechanism failure, not reachability.
  - `handoff(B→A, fork)`: master's hard IK boundary (residual 0.0954 m,
    32/32 seeds identical) is now a **near-miss collision** with multiple
    reachable alternate configurations (residuals as low as 0.0038 m in
    16/32 seeds) — a local-minimum/routing problem, not a kinematic wall.
  - `place(A, water_bottle, table)`: **unchanged** — still a genuine,
    flat-plateau boundary (0.0150 m vs master's 0.0138 m), confirmed by
    the same 32-seed method that established it as genuine on master.
- Per skill, reachability-vs-logic verdict: `pick(A, fork)` /
  `place(A, fork, table)` / `handoff(A→B, fork)` = **reachable geometry,
  collision/routing problem**; `pick(A, mug)` / `place(A, mug, table)` =
  **reachable geometry, grip-mechanism problem**; `handoff(B→A, fork)` =
  **reachable geometry, local-minimum/routing problem**;
  `place(A, water_bottle, table)` = **genuine unreachable boundary,
  confirmed twice** (master and redesign).
- No file outside this stage's stated scope was touched:
  `skills_scripted.py` diff is the single `HANDOFF_POSITION_XYZ` line;
  `grasp.py`, `ik.py`, `executor.py`, `env.py` are byte-identical to
  `a80ad5f`.
