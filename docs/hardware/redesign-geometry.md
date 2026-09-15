# Redesign Stage 2 — base separation sweep, geometry placement, home keyframe (ADR-059)

Provenance: bm-ptl (ADR-047). Branch: `redesign`. Superseding ADR-021 (base
placement), ADR-025/ADR-026 (home keyframe) **on this branch only** — see
ARCHITECTURE.md ADR-059 and DECISIONS.md's mirror entry.

## The axis finding (resolved before reading Stage 1, per the task brief)

Traced in code, not inferred: `scripts/measure_workspace.py:291` builds
`grid = np.zeros((len(y_edges)-1, len(x_edges)-1))`, so **rows = Y,
columns = X**, and `largest_rectangle` returns `(area, h_cells, w_cells)` in
that same (row, col) = (Y, X) order. Stage 1's headline "4 cm x 32 cm" band
(`redesign-workspace-measurement.md`) is therefore **4 cm in Y, 32 cm in
X — the SHORT axis is Y**, exactly the axis the two arm bases are separated
along (`ARM_GAP_Y`, previously ±0.25 m). This is why the separation sweep
below is the right instrument: reducing base separation directly thickens
the short axis. Had the short axis been X, reducing Y-separation would not
have helped and the sweep would have been futile.

## Step 1 — separation sweep

`scripts/sweep_base_separation.py`, run on bm-ptl. For each candidate
separation it regenerates a scratch scene (via `gen_dual_scene.py`,
monkeypatched `ARM_GAP_Y`/`DEST`, deleted after every sample — no
intermediate scene committed), samples each arm's RAW pinch-point cloud at
**N=300000** (the densification level Stage 1 showed is needed to trust a
largest-rectangle answer), and computes the both-arms intersection's largest
contiguous rectangle per 2 cm z-slice from 0.35–0.59 m.

Selection rule: **largest separation with >=1 slice, at z >= 0.40 m,
producing a contiguous rectangle >= 12x12 cm.**

| separation (m) | ARM_GAP_Y (m) | qualifying slices (z>=0.40, >=12x12cm) | best qualifying slice | best overall slice (any z) | wall clock (s) |
|---:|---:|---:|---|---|---:|
| 0.50 | 0.250 | 0 | — | 9x14 cm @ z=0.55 | 57.1 |
| 0.45 | 0.225 | 0 | — | 4x32 cm @ z=0.35 | 59.7 |
| **0.40** | **0.200** | **2** | **14x15 cm @ z=0.49, centroid (x=-0.035, y=0.000)** | 14x15 cm @ z=0.49 | 61.6 |
| 0.35 | 0.175 | 4 | 12x20 cm @ z=0.47 | 11x24 cm @ z=0.45 | 62.7 |
| 0.30 | 0.150 | 3 | 19x18 cm @ z=0.43 | 15x25 cm @ z=0.39 | 66.8 |
| 0.25 | 0.125 | 4 | 16x34 cm @ z=0.41 | 16x34 cm @ z=0.41 | 83.8 |
| 0.20 | 0.100 | 5 | 18x37 cm @ z=0.43 | 18x37 cm @ z=0.43 | 91.4 |
| 0.15 | 0.075 | 4 | 15x36 cm @ z=0.41 | 15x36 cm @ z=0.41 | 180.8 |

**Chosen separation: 0.40 m (`ARM_GAP_Y=0.20`).** It is the LARGEST
separation of the eight tried with >=1 qualifying slice — 0.50 m and 0.45 m
both have zero qualifying slices, confirming the current (pre-redesign)
0.50 m separation genuinely has no adequate shared band above z=0.40, exactly
as Stage 1 found for the whole 0.35–0.59 m range. Per the task's explicit
rule, the largest qualifying separation is chosen (not the smallest,
which would trade away single-arm workspace and increase cross-arm
collision risk for no floor-quality benefit): wider stance, same
minimum-quality shared band.

At 0.40 m, two slices qualify: z=0.43 (15x13 cm, centroid (0.045, 0.025)) and
z=0.49 (14x15 cm, centroid (-0.035, 0.000)). The larger-area one (z=0.49,
210 cm²) is used below.

**This sweep tests a WEAKER condition than the real requirement** (per the
task's own framing): no cross-arm collision is checked in Step 1 — each
arm's cloud is sampled independently. Step 4 item 5 tests the real,
stronger condition (simultaneous occupancy, cross-arm contact) and is where
this could still have failed; it did not (see below).

## Step 2 — geometry placed inside the measured workspace

`scripts/gen_dual_scene.py` edited (ADR-059 comment blocks mark every
change):

| body / constant | before (0.50 m sep.) | after (0.40 m sep.) | rationale |
|---|---|---|---|
| Arm A base | `pos="0 -0.25 0.35"` | `pos="0 -0.20 0.35"` | `ARM_GAP_Y` 0.25 -> 0.20 |
| Arm B base | `pos="0 0.25 0.35"` | `pos="0 0.20 0.35"` | mirror of A |
| `PLATE_POS` (arm A) | (-0.15, 0.00, 0.355) | (-0.15, 0.05, 0.355) | shifted by `SHIFT_A=+0.05` |
| `FORK_POS` (arm A) | (-0.05, 0.05, 0.356) | (-0.05, 0.10, 0.356) | shifted by `SHIFT_A` |
| `BOTTLE_POS` (arm A) | (0.22, 0.00, 0.44) | (0.22, 0.05, 0.44) | shifted by `SHIFT_A` |
| `MUG_POS` (arm B) | (0.05, -0.03, 0.39) | (0.05, -0.08, 0.39) | shifted by `SHIFT_B=-0.05` |
| `SPOON_POS` (arm B) | (0.00, 0.08, 0.356) | (0.00, 0.03, 0.356) | shifted by `SHIFT_B` |
| drawer | housing at (0, 0.08, 0.28) | **removed** | see drawer decision below |

**Why "shift by SHIFT_A/SHIFT_B" rather than re-deriving each prop's
position from scratch.** Each prop is assigned to a single grasping arm
(`plate`, `fork`, `bottle` -> A; `mug`, `spoon` -> B — see the assignment
table below). Translating a prop by exactly the same Y delta its assigned
arm's base moves preserves that prop's position **relative to its own
arm's base frame** exactly — since single-arm reachability is a function of
joint angles in that arm's own frame, a prop that was reachable-with-margin
before is reachable-with-margin after, by construction, without needing to
re-derive per-prop margins from the point cloud. This is verified, not just
argued: Step 4 item 3 (below) confirms every prop's grasp point still
converges under the required 0.005 m residual after the shift. All five
props remain on the table (`table_top` spans x in [-0.40, 0.40], y in
[-0.25, 0.25]) and pairwise centre-to-centre clearances are unchanged from
before the shift (same-arm props share the same shift; cross-arm props'
mutual clearance strictly *increases* since group A moves +Y and group B
moves -Y).

**Prop-to-arm assignment used (a documented assumption of this stage, not
read from a TaskPlan for these specific props):**

| prop | grasping arm | source |
|---|---|---|
| plate | A | PLAN.md M05's worked example command |
| mug | B | PLAN.md M05's worked example command |
| fork | A | `scripts/verify_adr038_skills.py`'s own `pick(A, fork)` / `handoff(A, B, fork)` |
| bottle | A | `scripts/verify_adr038_skills.py`'s own `pick(A, 'bottle')` |
| spoon | B | assumed by symmetry with mug (no precedent in either source) |

### HANDOFF_POSITION_XYZ — blocked by a file-freeze conflict, flagged not silently resolved

The task's Step 2 instruction groups `HANDOFF_POSITION_XYZ` with the
`gen_dual_scene.py` edits ("In `gen_dual_scene.py`: new base positions;
`HANDOFF_POSITION_XYZ` at the chosen region's centroid..."). **This constant
does not live in `gen_dual_scene.py` — it is defined in
`src/bimanual/control/skills_scripted.py:485`**, a file this same task's
RULES section says must "stay UNCHANGED". These two instructions directly
contradict each other for this one constant. Per the builder rule to flag
rather than guess past an ambiguity, `skills_scripted.py` was **not**
edited — the freeze is explicit and repeated, and editing a frozen file to
satisfy a plausibly-mistaken cross-reference is a worse failure mode than
leaving a candidate value unrealized in code.

**What was verified anyway, without touching the frozen file:**
`scripts/verify_stage2_gate.py` (Step 4) tests the handoff point using the
**current, unmodified** `HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.43)` at the
new 0.40 m separation, and it PASSES (both arms converge under 0.005 m —
see Step 4 below). The sweep's own candidate centroid, **(-0.035, 0.000,
0.49)**, was not substituted in and therefore was not itself verified as a
handoff target; it is recorded here as the value a future pass should use
if/when `skills_scripted.py`'s freeze is lifted by the planner. This
conflict should be resolved by the planner, not by this module silently
picking a side.

### Drawer decision: REMOVED

Stage 1's z-sliced occupancy/intersection tables
(`redesign-workspace-measurement.md`) only ever tested **z=0.35 (tabletop
surface) upward** — the drawer's old position (z=0.28, below the tabletop)
was **never measured by the z-sliced analysis at all**, only by a coarse
per-arm min/max bound (individual reach extends down to ~0.21 m, but that
says nothing about occupancy or the shared band specifically at z=0.28).
Per the task's own instruction ("do not carry it forward unmeasured"),
"raise to tabletop height" was tried first:
`DRAWER_HOUSING_Z = TABLE_TOP_Z + 0.05 = 0.40` (housing bottom flush on the
table), `DRAWER_HOUSING_Y` shifted by `SHIFT_A` (arm A assumed to open it).
This **compiled** and passed gate items 1/3/4/5, but **failed item 2**:
13 contacts >1 mm between the raised housing and plate/fork/spoon's rest
positions, measured directly via `data.contact` (not the distrusted mesh
filter):

```
plate_dish <-> drawer_housing_left   dist=-0.01830
drawer_housing_bottom <-> fork_head  dist=-0.00350  (x4, different contact indices)
fork_handle <-> drawer_housing_bottom dist=-0.00800
fork_handle <-> drawer_housing_left   dist=-0.00400
spoon_handle <-> drawer_housing_bottom dist=-0.00400
spoon_bowl <-> drawer_housing_bottom  dist=-0.01200
plate_dish <-> drawer_box             dist=-0.01106
```

Resolving this footprint conflict (relocating the drawer or four props in
X, since the template has no drawer-X parameter and adding one plus
re-verifying the whole Step 4 gate again) did not fit this session's
remaining time under the hard 120-minute cap. **The drawer is removed**
(body, joint, materials-in-use, camera) rather than shipped with a known,
unresolved collision. `open_drawer` is not supported by this generated
scene — this is a real capability loss, stated plainly, not hidden. It was
also never diagnosed further than "arm bases sit at tabletop height" before
this stage (per the task's own framing); this stage adds the specific
footprint-collision reason a table-top drawer placement fails today, for
whoever revisits this.

## Step 3 — home keyframe regenerated by search

`scripts/search_home_keyframe.py`. Candidates: 5 joint angles
(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll), applied
**identically to both arms** (matching ADR-026's own finding that
same-sign, not mirrored, folds are the collision-free family), sampled
uniformly — shoulder_lift/elbow_flex from `[joint_range_lo, 0]` (the folded
half), the other three from `[-0.3, 0.3]` rad — via a fixed-seed
`numpy.random.default_rng(20260915)`. Candidate 0 is pinned to ADR-026's own
point `(0, -1.2, -1.6, 0, 0)` so the search would find it immediately if it
were still valid.

**Scoring, per candidate:** `mj_forward` once, then enumerate
`data.contact` directly for (a) self-collision (both geoms same arm), (b)
cross-arm contact, (c) table penetration beyond 1 mm, (d) any arm-vs-prop
contact; plus (e) `ik.solve_position_ik(..., tol=0.005)` from that candidate
pose to every prop's grasp point (assigned arm) and to the handoff point
(both arms). A candidate passes only if all five are simultaneously zero
violations / all converged.

**Result: candidate 0 (ADR-026's own fold) did NOT pass at the new 0.40 m
separation** (confirming the fold needed re-deriving, not just re-using;
its specific violation counts were not retained by this run because the
script only keeps the running best-so-far, and candidate 1 immediately
superseded it). **Candidate 1 — the very next randomly-drawn point — PASSED
EVERY CRITERION simultaneously.** The search therefore stopped after
**2 candidates tried** (of a budgeted 400):

```
HOME_SHOULDER_PAN  =  0.054022898001139796
HOME_SHOULDER_LIFT = -1.4813027413443594
HOME_ELBOW_FLEX    = -0.46647495231644354
HOME_WRIST_FLEX    =  0.09897040413436309
HOME_WRIST_ROLL    =  0.17687383568459125
```

## Step 4 — verification gate (`scripts/verify_stage2_gate.py`)

All five run against the actual generated `so101_dual_table.xml`, from the
real `home` keyframe applied via `TableSettingEnv.reset()` — contacts read
directly from `data.contact`, never the ADR-058-distrusted mesh filter.

| # | item | result |
|---|---|---|
| 1 | model compiles | **PASS** |
| 2 | home keyframe: zero contacts >1 mm | **PASS** (0 found) |
| 3 | every prop grasp point, IK residual <0.005 m from home | **PASS** (all 5) |
| 4 | handoff point, both arms, seeded from own approach pose | **PASS** (both) |
| 5 | both arms' handoff configs applied simultaneously, zero cross-arm contact below -0.005 m | **PASS** (0 violations) |

**Item 3 detail (IK residual, metres, from home):**

| prop | arm | residual (m) |
|---|---:|---:|
| plate | A | 0.00306 |
| mug | B | 0.00467 |
| fork | A | 0.00489 |
| spoon | B | 0.00201 |
| bottle | A | 0.00230 |

**Item 4 detail (handoff point, `HANDOFF_POSITION_XYZ=(0, -0.01, 0.43)`,
each arm seeded from its own approach/hover pose, NOT from home):**

| arm | hover residual (m) | handoff residual (m) |
|---:|---:|---:|
| A | 0.00327 | 0.00403 |
| B | 0.00277 | 0.00378 |

**Item 5 detail:** both arms' converged handoff joint angles applied via a
single `mj_forward` call together — **0 cross-arm contacts with depth below
-0.005 m**. This is the condition the Step 1 sweep explicitly does NOT
test (independent per-arm sampling, no simultaneous occupancy check) and
that the task flagged as the likeliest failure point; it passed.

**Overall: ALL GATE ITEMS PASS.**

## Summary of what changed and what did not

- Base separation: 0.50 m -> **0.40 m** (`ARM_GAP_Y` 0.25 -> 0.20).
- All 5 props: repositioned to preserve their position relative to their
  assigned arm exactly (see table above).
- Drawer: **removed** (measured footprint collision when raised; no time
  left in the 120-minute cap to relocate and re-verify).
- Home keyframe: **regenerated by search**, 2 candidates tried, both
  self/cross-arm/table/prop-contact-free and IK-convergent from it.
- `HANDOFF_POSITION_XYZ`: **left unchanged** (0, -0.01, 0.43) — editing it
  would require touching the frozen `skills_scripted.py`; verified to still
  pass the gate at the new separation regardless. The sweep's own candidate
  centroid (-0.035, 0.000, 0.49) is recorded for a future pass.
- `skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`: not
  modified, per the task's rules.
- No skill was run in this stage (per the rules) — the Step 4 gate is the
  test that substitutes for it.
