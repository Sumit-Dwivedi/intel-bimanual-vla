# M07 — fine-grid placement-envelope measurement and deterministic randomization (ADR-048)

**Recorded:** Sept 14, 2026 · HEAD at start: `cfb2c9c` (M08 prep, ADR-047).
**Provenance.** Every measurement below ran on bm-ptl,
`C:\Users\devcloud\project\ov_env\Scripts\python.exe` (`mujoco==3.2.7`,
`openvino==2026.3.1`) — never the laptop. ADR-047 recorded a pre-existing,
tiny (last-float-digit) cross-machine divergence between the laptop and
bm-ptl; this document's numbers are exclusively bm-ptl's.

**Scope.** Three tasks: (1) measure a 7x7 fine placement grid per skill for
the four skills the ADR-038 regression gate already treats as working;
(2) build `src/bimanual/sim/randomization.py`'s `ScenarioRandomizer` from
that measurement, scoped per prop; (3) run seeds 0-9 with the randomizer
opted in and report per-skill robustness. `env.py`'s only change is the new
opt-in `randomizer=` argument to `reset()` (ADR-048) — verified below to be
byte-identical to the pre-existing baseline when omitted.

---

## Budget projection (measured before committing to the full sweep)

Per this task's own instruction: real per-cell cost was measured on bm-ptl
on a handful of cells *before* running any full sweep, using
`scripts/probe_envelope.py timing`.

| case | measured wall time | steps |
|---|---|---|
| a FAILING trial (`pick_fork`/`place_fork`/`handoff`, all fail at the same ~500-step Phase-1/waypoint-1 non-convergence) | ~4.2-4.3 s/trial | 500 |
| a PASSING `pick`/`place` trial | ~2.3 s/trial | 1655-1656 |
| a PASSING `handoff` trial | ~17 s/trial | 6610 |

Projected total for the full 4-skill x 49-cell x 5-seed sweep (980 trials,
before any rejection): **~75-110 minutes**, comfortably inside the
2-4 hour ceiling this task's own instructions flagged as plausible.
**Measured actual total** (four full 7x7 sweeps, `sweep_wall_s` summed):
488.6 + 518.8 + 721.7 + 553.0 = **2282 s (~38 minutes)** — cheaper than
projected, because a large fraction of cells near the grid's edges are
REJECTED (off-table/intersects-another-prop) before any skill execution is
attempted at all (~0 cost), which the pre-sweep timing probe (run only near
the grid's edge, where nothing was yet known to be rejected) did not
account for.

**Handoff's early-stop, attempted then superseded — reported honestly.**
This task permits stopping a sweep early if "the first row or two...comes
back all-zero." A first attempt did exactly that
(`--early-stop-empty-rows 2`, ascending `dx` from -3 cm): both of the first
two rows (`dx=-0.03`, `dx=-0.02`) came back all-fail, and the sweep stopped
after only 14 cells / 55.1 s
(`out/m07_envelopes/handoff_earlystop_attempt1.jsonl`, not committed —
`out/` is gitignored). **This was the wrong call, caught before being
reported as a finding:** stopping there meant the sweep never reached
`dx=0`, the exact point `scripts/verify_adr038_skills.py` already proves
`handoff` succeeds at with a 0.2263 m margin — an early-stop conclusion of
"empty everywhere" would have directly contradicted evidence already in
this repo. The sweep was re-run in full (no early stop, 721.7 s) and found
handoff's envelope is a genuine single passing cell, not an empty one — see
below. Lesson for any future use of this early-stop flag: order the sweep
so the flag cannot fire before the known-good centre cell has been tested,
or do not use it at all when a skill's baseline-success point is already
established elsewhere.

---

## Rejection methodology

A candidate cell is **rejected** (not scored as a failure) if:
1. **Off-table**: the prop's own footprint radius (a conservative bounding
   circle, cited from `scripts/generate_posenet_data.py:276-281`, copied not
   imported) would extend past the table surface's half-extents
   (`x in [-0.40, 0.40]`, `y in [-0.25, 0.25]`, cited from
   `scripts/gen_dual_scene.py:1037`'s declared `table_top` geom size).
   **Zero cells were rejected for this reason** across all four sweeps — the
   +/-3 cm range around each prop's own (well-inboard) default never reaches
   the table edge.
2. **Intersects another prop**: after writing the candidate (x, y) into the
   target prop's `qpos` and calling `mujoco.mj_forward` (which runs MuJoCo's
   own collision detection), any PROP-vs-PROP contact in `env.data.contact`
   has penetration depth (`contact.dist`) deeper than -2 mm. This is REAL
   MuJoCo collision geometry, deliberately **not**
   `generate_posenet_data.py`'s own bounding-circle heuristic — that
   heuristic is documented there as a render-legibility check for a
   from-scratch layout generator and would incorrectly reject cells the
   shipped scene's own default already occupies (e.g. fork and spoon sit
   closer together, by that heuristic's own forbidden-disk sum, than the
   heuristic allows, yet do not actually overlap at their real, non-isotropic
   shapes and shipped relative bearing).

All 20 rejections in each of the three fork sweeps were `fork<->plate`
(negative dx/dy corner) or `fork<->spoon` (positive dy) real geometric
overlaps. `pick_bottle`'s sweep had **zero** rejections (water_bottle sits
far from every other prop across the whole +/-3 cm range).

---

## Task 1 — per-skill 7x7 grids (1 cm step, +/-3 cm range, 5 reps/cell, >=80% pass threshold)

**Why 5 reps/cell measures something real even though the sim has no
execution-time randomness on this path.** Each cell fixes the target prop's
(dx, dy) exactly; nothing in `skills_scripted.py`/`ik.py`/`grasp.py` consumes
any RNG on the scripted-controller path (grepped, confirmed — same finding
the pre-M07 audit already made). The 5 reps are 5 independent
fresh-env-fresh-executor repetitions (seeds 0-4, forwarded to
`env.reset(seed=...)` for bookkeeping) rather than a fifth randomization
axis — a live regression check against exactly the class of bug ADR-047
fixed (a cross-trial state leak that made supposedly-identical repeated
trials disagree). **Every one of the 105 non-rejected cells measured across
all four sweeps reported exactly 0/5 or 5/5 — never 1-4/5.** No
nondeterminism flag fired.

### `pick(A, fork)` — 4/29 measured cells passed, 20 rejected

```
  dx\dy      -3      -2      -1      +0      +1      +2      +3
     -3     REJ     REJ     0/5     0/5     0/5     REJ     REJ
     -2     REJ     0/5     0/5     0/5     0/5     REJ     REJ
     -1     0/5     0/5     0/5     5/5     0/5     REJ     REJ
     +0     0/5     0/5     0/5     5/5     0/5     REJ     REJ
     +1     0/5     0/5     0/5     5/5     REJ     REJ     REJ
     +2     0/5     0/5     0/5     5/5     REJ     REJ     REJ
     +3     0/5     0/5     0/5     0/5     REJ     REJ     REJ
```

**Chosen rectangle:** `dx in [-0.010, +0.020] m`, `dy in [0, 0] m` (4 cells,
100% coverage within it). Consistent with the pre-M07 audit's own finer
(0.5 cm step, +/-1 cm range) grid, which found a real, asymmetric tolerance
band with essentially zero positive-`dy` margin — this 1 cm grid is too
coarse to resolve `dy` any finer than "must be exactly 0," which reads as a
single-column pass here rather than a contradiction.

### `place(A, fork, table)` — 2/29 measured cells passed, 20 rejected

```
  dx\dy      -3      -2      -1      +0      +1      +2      +3
     -3     REJ     REJ     0/5     0/5     0/5     REJ     REJ
     -2     REJ     0/5     0/5     0/5     0/5     REJ     REJ
     -1     0/5     0/5     0/5     5/5     0/5     REJ     REJ
     +0     0/5     0/5     0/5     5/5     0/5     REJ     REJ
     +1     0/5     0/5     0/5     0/5     REJ     REJ     REJ
     +2     0/5     0/5     0/5     0/5     REJ     REJ     REJ
     +3     0/5     0/5     0/5     0/5     REJ     REJ     REJ
```

**Chosen rectangle:** `dx in [-0.010, 0] m`, `dy in [0, 0] m` (2 cells, 100%
coverage) — narrower than `pick_fork`'s own (`dx=+0.010`/`+0.020` fail here
but pass for `pick_fork`), consistent with `place`'s docstring: it nests the
identical `pick` at Phase 1 AND has its own separate destination-approach
waypoints, so its envelope can only be a subset of (never wider than)
`pick_fork`'s.

### `handoff(B, A, fork)` — 1/29 measured cells passed, 20 rejected

```
  dx\dy      -3      -2      -1      +0      +1      +2      +3
     -3     REJ     REJ     0/5     0/5     0/5     REJ     REJ
     -2     REJ     0/5     0/5     0/5     0/5     REJ     REJ
     -1     0/5     0/5     0/5     0/5     0/5     REJ     REJ
     +0     0/5     0/5     0/5     5/5     0/5     REJ     REJ
     +1     0/5     0/5     0/5     0/5     REJ     REJ     REJ
     +2     0/5     0/5     0/5     0/5     REJ     REJ     REJ
     +3     0/5     0/5     0/5     0/5     REJ     REJ     REJ
```

**Chosen rectangle:** `dx in [0, 0] m`, `dy in [0, 0] m` — a single point,
the unperturbed default itself. This is a real, measured single-point
envelope (see the early-stop narrative above for why the FIRST attempt at
this measurement almost mis-reported it as fully empty). Directly confirms
the pre-M07 audit's "effectively no placement-randomization headroom" and
ADR-032..038's "joint-limit knife-edge" characterisation — `handoff` fails
identically to `pick_fork` at Phase 1 for every non-zero cell (its own
nested pick), and even the one dx/dy combination the audit's earlier finer
grid found viable for `pick_fork` alone at ±0.5 cm resolution does not
survive at this grid's coarser ±1 cm columns once the full Phase 3-5
choreography after Phase 1 is also required to succeed.

### `pick(A, 'bottle')` — 8/49 measured cells passed, 0 rejected

```
  dx\dy      -3      -2      -1      +0      +1      +2      +3
     -3     0/5     0/5     0/5     0/5     0/5     0/5     0/5
     -2     0/5     0/5     0/5     0/5     0/5     0/5     0/5
     -1     0/5     0/5     0/5     0/5     0/5     5/5     5/5
     +0     0/5     0/5     5/5     5/5     5/5     0/5     0/5
     +1     0/5     5/5     0/5     0/5     0/5     0/5     0/5
     +2     0/5     5/5     5/5     0/5     0/5     0/5     0/5
     +3     0/5     0/5     0/5     0/5     0/5     0/5     0/5
```

**Chosen rectangle:** `dx in [0, 0] m`, `dy in [-0.010, +0.010] m` (3 cells,
100% coverage within it). The other five passing cells (`dx=-1,dy=+2/+3`;
`dx=+1,dy=-2`; `dx=+2,dy=-2/-1`) are scattered, non-contiguous with the
origin, and were left unused for the chosen rectangle — consistent with the
pre-M07 audit's own coarse-grid finding of a non-local, no-single-sharp-edge
pattern for this prop.

---

## Task 2 — `ScenarioRandomizer` (`src/bimanual/sim/randomization.py`)

`ScenarioRandomizer.randomize(seed) -> {prop: (dx, dy)}`, deterministic via
a fresh `np.random.default_rng(seed)` per call (never the env's own
`np_random`, never global numpy state), drawing props in sorted-name order.
Per-prop `ENVELOPES` is the rectangle **intersection across every skill that
constrains that prop** — described in full in the module's own docstring.

**Direct intersection across target-object skills:**
- `fork` (targeted by `pick`, `place`, `handoff`): intersecting
  `[-0.010,+0.020]x[0,0]`, `[-0.010,0]x[0,0]`, `[0,0]x[0,0]` gives
  `[0,0]x[0,0]` — a single point.
- `water_bottle` (targeted only by `pick_bottle`): `[0,0]x[-0.010,+0.010]`.
- `plate`, `mug`, `spoon`: targeted by no measured skill this pass — absent
  from `ENVELOPES` entirely (not zero-width, not present).

**A second, cross-prop constraint found empirically in Task 3, not assumed
at design time — see Task 3 below for the full measured evidence.**
`handoff` never touches `water_bottle` as its own target, but Task 3's
`randomized_eval` found `handoff` fails 0/10 when ONLY `water_bottle` is
randomized (fork stays fixed throughout). Directly probed at `dy` in
`{+-0.0001, +-0.001, +-0.005}` m: **every nonzero offset reproduces the
identical Phase-3 failure** (same IK residual, same frame count);
`dy=0.0` exactly reproduces the true baseline exactly
(`frames_used=6610`, `from_arm_retreat_dist=0.2263`). This is a genuine
binary cliff, not a smoothly shrinkable range — MuJoCo's simulation is a
fully globally-coupled system, so a prop with zero direct geometric
proximity to a maneuver can still perturb an already-knife-edge collision
check (ADR-037's own term) elsewhere in the scene. Because a global,
skill-agnostic randomizer cannot know which skill will run after `reset()`,
and `handoff` is one of the four ADR-038-gated skills, `water_bottle`'s
cross-skill-safe envelope must also satisfy this constraint — recorded as
`SKILL_ENVELOPES["handoff"]["water_bottle"] = (0, 0, 0, 0)` in
`randomization.py`, intersected the same way as every other entry.

**Final result: both `fork`'s and `water_bottle`'s intersections collapse
to a single point (zero width on BOTH axes).** `_is_degenerate_point`
excludes both — per this task's own instruction, a degenerate single-point
"range" is not offered as a fake always-zero `rng.uniform(0, 0)` call, it
is excluded outright, with the reason recorded in `EXCLUDED_PROPS`:

```
ENVELOPES = {}
EXCLUDED_PROPS = {
  'fork': "... is a single point (zero width on BOTH axes) ...",
  'water_bottle': "... is a single point (zero width on BOTH axes) ...",
}
```

**`plate`, `mug`, `spoon` are absent from `ENVELOPES` for a different
reason** (no measured skill targets them at all this pass) — distinct from
`fork`/`water_bottle`'s exclusion (measured, then found degenerate).

**Net effect, stated plainly: `ScenarioRandomizer`, run against the
evidence actually measured this pass, randomizes nothing.** Every prop
stays at its deterministic default on every `reset()`, randomizer-on or
not. This is reported as the headline finding, not hidden as a null
result — see "What this means for M08" below.

---

## Task 3 — seeds 0-9, oracle mode, randomization on

Two rounds are reported, in the order they actually happened, because the
first round is what SURFACED the cross-prop finding above.

### Round 1 — before the cross-prop fix (`ENVELOPES = {"water_bottle": (0,0,-0.010,+0.010)}`, `fork` already excluded from Round 1)

| skill | N/10 | mean frames on success | failure breakdown |
|---|---|---|---|
| `pick_fork` | **10/10** | 1655.0 | — |
| `place_fork` | **8/10** | 3455.0 | 2/10 (seeds 5, 8): `waypoint 1 (approach destination) failed [collision (arm-vs-prop: mug ...)]` |
| `handoff` | **0/10** | n/a | 10/10 (all seeds): `phase 3 (to_arm approach) failed [... collision (cross_arm ...)]`, byte-identical residual/frame count every seed |
| `pick_bottle` | **6/10** | 1672.2 | 4/10 (seeds 0,1,2,4): `weld_attach_failed_after_300_frames` |

**`place_fork`'s 2 failures: a genuine, reproducible chaos finding, not a
harness bug.** Confirmed directly: `fork`'s own INITIAL `qpos` is
bit-identical across all 10 seeds (checked explicitly — only `water_bottle`
ever differs). Re-running the SAME seed twice reproduces the SAME final
fork position exactly (deterministic in `seed`). Yet different seeds'
(different, far-away) `water_bottle` positions measurably shift where
`place_fork`'s OWN nested pick ends up gripping the fork — visible in the
successful seeds' own final resting positions, which are NOT
bit-identical across seeds despite identical fork start conditions
(seed 0 final `x=-0.0077`; seed 9 final `x=-0.0449`, a 3.7 cm spread). This
is global floating-point coupling in a long (3455-step) closed-loop
rollout, not a bug: MuJoCo recomputes the WHOLE system's contacts/forces
every step, so a far-away prop's differing position changes floating-point
rounding throughout the entire trajectory, and this closed-loop controller
is sensitive enough to that rounding, over enough steps, to occasionally
land somewhere that collides with `mug` during the destination approach.
`place_fork`'s 80% clears this task's own 60% floor, so no remediation was
required by that rule — reported for transparency regardless.

**`handoff`'s 0/10: the finding that drove Task 2's cross-prop fix.** Root
cause is documented in Task 2 above (binary cliff, confirmed at 0.1 mm
resolution). Below the 60% floor — this task's remediation clause was
followed: investigated (root-caused precisely, <30 minutes, well inside the
90-minute box), "shrink and retest" was attempted (offsets as small as
0.1 mm still reproduce the identical failure, so no non-degenerate shrink
exists), and the result — a single-point-only, i.e. degenerate, envelope
for `water_bottle` once `handoff`'s constraint is included — was folded
back into Task 2's `ENVELOPES` computation rather than left as an
unresolved gap.

**`pick_bottle`'s 6/10 (60%, exactly at the floor, not below it):** the
chosen rectangle (`dx=0`, `dy in [-0.010,+0.010]`) passed 5/5 at each of its
three SAMPLED grid points, but continuous sampling within that rectangle's
interior found real sub-cm structure the 1 cm grid does not resolve — the
same caveat the pre-M07 audit already raised for `pick_fork`'s asymmetric
band at finer resolution. Moot for the shipped randomizer either way, since
`water_bottle` is excluded in the final result below regardless.

### Round 2 — after the cross-prop fix (shipped `ENVELOPES = {}`)

| skill | N/10 | mean frames on success |
|---|---|---|
| `pick_fork` | **10/10** | 1655.0 |
| `place_fork` | **10/10** | 3455.0 |
| `handoff` | **10/10** | 6610.0 |
| `pick_bottle` | **10/10** | 1655.0 |

Every mean-frames value is bit-identical to the unperturbed
`scripts/verify_adr038_skills.py` baseline's own `frames_used`
(`handoff`: 6610, `pick`/`place`: 1655/3455) across all 10 seeds — proof
the shipped `ScenarioRandomizer`, with `ENVELOPES` empty, is a true no-op
on this measured evidence, not merely "close."

---

## Regression gates (`randomizer=None`, the default — must be byte-identical)

`python scripts/verify_adr038_skills.py` (bm-ptl, after every change in
this commit):

```
[1] pick(A, fork)        final z = 0.3989
[2] place(A, fork, table) final z = 0.3588
[3] pick(A, 'bottle')     final z = 0.6192
[4] handoff(A -> B, fork) lateral sep = 0.1946 m
```

Exact match to the four numbers this task cites as authoritative, every
time re-checked in this session (including after `env.py`'s `reset()`
signature changed).

`pytest tests/test_skills.py -q` (bm-ptl): **4 passed, 4 failed**, same four
failing tests (`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`), same failure reasons and residuals,
re-checked at the end of this session.

---

## What this means for M08 (and anyone building on `ScenarioRandomizer`)

- **`ScenarioRandomizer`, as measured this pass, is a documented no-op.**
  This is not a defect in the randomizer's own code — it is the honest
  output of intersecting real, measured, cross-skill-safe envelopes for the
  only two props any current skill targets, both of which collapsed to a
  single point. A future pass could recover real randomization range by
  either (a) fixing `handoff`'s underlying fragility (its Phase 3 cross-arm
  corridor, and separately its own Phase 1 nested-pick tolerance) so a
  wider envelope survives the intersection, or (b) building a
  skill-AWARE randomizer that only randomizes props a given upcoming skill
  call does not (transitively) depend on — a larger design change, out of
  this pass's scope (`env.py`'s `reset()` hook is skill-agnostic by
  construction, matching how a real M08 seed loop would randomize a whole
  scenario before knowing which skill executes against it).
- **Any future single-prop envelope measurement must also check the OTHER
  gated skills, not just the skills that directly target that prop.**
  Task 2's original design (intersect only across skills that target a
  prop) is INSUFFICIENT on its own, as `water_bottle`/`handoff` now proves.
  This document's `SKILL_ENVELOPES` table therefore carries
  `handoff`'s `water_bottle` entry as an explicit, separately-labelled
  compatibility constraint, not a target-object envelope, and any prop
  added to `ENVELOPES` in the future should be checked against all four
  gated skills the same way, not just its own direct targets.
- **`place_fork`'s chaos finding is a real, if narrower, version of the
  same lesson:** even a prop excluded from `ENVELOPES` entirely (as `fork`
  now is) can still see its own outcome perturbed by a DIFFERENT prop's
  randomized position, purely through global floating-point coupling in a
  long closed-loop rollout. This does not change today's shipped (empty)
  `ENVELOPES`, but it is a caveat any future evaluation harness (M08)
  should know before treating "prop X is not randomized" as "skills
  targeting prop X are therefore fully insulated from randomization."
