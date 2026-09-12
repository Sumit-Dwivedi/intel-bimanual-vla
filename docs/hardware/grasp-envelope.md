# Grasp envelope: what this gripper can physically hold (diagnostic only)

**Recorded:** Sept 12, 2026 · **Produced by:** `scripts/probe_grasp_envelope.py`, run on
bm-ptl · **Follows:** the Fix D revert (see `DECISIONS.md`) · **Status: diagnostic
report only. No fix is proposed or applied here.** This is a report to inform a user
decision, per this task's own constraints.

**Bottom line, stated plainly per the task's own instruction: the caliper sweep found
NO thickness, from 2 mm to 60 mm in 2 mm steps, that this gripper could be measured to
hold with sustained two-jaw contact.** This is reported as a genuine, physically
grounded result, not a failed measurement — see "Why", below, for the mechanical
explanation this session's own MuJoCo introspection surfaced.

---

## Method: functional, not geometric

Two purely geometric measurement attempts were explicitly warned against for this task,
and this script does neither:

1. **`mj_geomDistance` minimised over all geom pairs** returns the hinge-region overlap
   (the two jaw bodies' bounding volumes already overlap near the shared hinge), not the
   pinch gap.
2. **Geoms near the rotation axis barely move** as the jaw swings — the M06a Fix D
   revert (see `DECISIONS.md`) found a fine collision sphere placed exactly on the hinge
   axis has an *identical* 0.00618 m gap at both the closed (-0.1745 rad) and open
   (+1.7453 rad) joint limits, because the sphere sits exactly on the axis and never
   moves.

Instead, `scripts/probe_grasp_envelope.py` builds a **throwaway, single-arm MuJoCo
model** (arm A's own body tree, copied byte-faithfully from
`scenes/so101/so101_new_calib.xml`, ADR-016 — never edited, never written back) plus one
free-jointed test box, and runs a real physics simulation for each candidate thickness
`T`:

1. Command the gripper **OPEN**, let it settle (this alone catches a box too big to be
   accepted without being displaced).
2. Command the gripper **CLOSED**, step until settled.
3. Read `data.contact` / `mujoco.mj_contactForce` for genuine two-jaw contact and the
   total contact-normal force, and the box's net displacement from its insertion point.

Nothing here writes to `scenes/so101/` or to `src/bimanual/sim/assets/`; the scratch
MJCF lives in a `tempfile.TemporaryDirectory()` for the duration of one thickness's
measurement and is deleted immediately after (`_run_one_thickness`).

### Calibrating where to insert the test box

Finding *a* starting insertion point/axis for the box is itself calibration, not the
measurement — but it is worth reporting in full because two of the naive choices tried
in this same session failed silently rather than obviously:

1. **Midpoint of the two jaw collision geoms' `geom_xpos` at OPEN.** A dynamic test at
   this point, and a grid of 6 nearby hand-picked offsets, never produced two-jaw
   contact after closing — every candidate was either already deep in contact at OPEN
   or violently ejected once CLOSE was commanded (displacement up to 0.74 m for a 10 mm
   box in this session's own measurement).
2. **Midpoint of the two jaw *bodies'* `xpos`** (`ik.py`'s own pinch-point convention,
   ADR-025). Same outcome.
3. **A systematic static grid search** (offsets along the fixed→moving axis and two
   perpendiculars, ±4 cm, 125 points, a 2 mm probe, forward-kinematics only, no physics
   stepping): looking for a point that is contact-free at OPEN but touched by *both*
   jaw geoms at CLOSED. **Zero of 125 points qualified.**
4. **Checking whether the fixed and moving jaw collision geoms ever contact each other
   directly** (no test box) while sweeping the joint from OPEN to CLOSED: **never** —
   MuJoCo excludes contact generation between directly-connected parent/child bodies
   (the fixed jaw is the moving jaw's parent) regardless of geometric overlap, so this
   check cannot locate a "where they meet" point either.
5. **`mujoco.mj_geomDistance` between the exact named pair** (fixed vs. moving jaw
   collision geom — *not* minimised over all geom pairs in the model, the specific
   pitfall this task warned about), swept across 11 angles spanning the joint's full
   range. **Finding, measured, not assumed: the two geoms' convex hulls overlap
   (`dist` negative) at every single angle in the range** — from **-0.0139 m** at the
   *least*-overlapping angle (qpos ≈ +0.59 rad, roughly 42% of the way from OPEN toward
   CLOSED) to **-0.0377 m** at full closure:

   | qpos (rad) | mj_geomDistance (m) |
   |---:|---:|
   | +1.7453 (OPEN) | -0.02197 |
   | +1.5533 | -0.02069 |
   | +1.3614 | -0.01939 |
   | +1.1694 | -0.01803 |
   | +0.9774 | -0.01666 |
   | +0.7854 | -0.01528 |
   | **+0.5934 (least overlap)** | **-0.01390** |
   | +0.4014 | -0.01924 |
   | +0.2094 | -0.02583 |
   | +0.0175 | -0.03204 |
   | -0.1745 (CLOSED) | -0.03765 |

This is a plausible, mechanically complete explanation for findings 1-3: `<geom
type="mesh">` collision in this scene defaults to a single **convex hull** per geom (no
convex decomposition is declared anywhere in this asset), and both jaw parts
(`wrist_roll_follower_so101_v1`, the fixed side; `moving_jaw_so101_v1`, the moving side)
are complex, non-convex housing shapes — their hulls fill in concavities and extend well
past the true solid material, so they overlap by more than a centimetre at *every* joint
angle even though the real machined parts likely never truly touch. A third object
placed anywhere in that perpetually-overlapping hull region is read by the solver as
embedded in solid material on both sides and is forcibly ejected. This matches
`ADR-024`'s own independent finding that these two geoms' `geom_rbound` (bounding-sphere
radius) is **~8.4 cm** (fixed side) and **~6.35 cm** (moving side) — large relative to
every prop in this scene.

The dynamic sweep below uses the **least-overlap angle's nearest-point pair**
(`mj_geomDistance`'s `fromto` output — actual nearest points on the two convex hulls, not
a hinge-axis-locked or otherwise degenerate point) as its seed location. This is the best
available calibration point, reported with its own limitation (hull-based, not
true-mesh-based) rather than presented as exact.

Test box cross-section: **1.5 cm × 1.5 cm** (reduced from an initially-tried 4 cm × 4 cm,
which this session found intersected jaw housing geometry from directions unrelated to
the thickness axis — see the script's `BOX_HALF_WIDTH_M`/`BOX_HALF_HEIGHT_M` comment).
1.5 cm is close to this scene's own smallest real prop cross-sections (fork/spoon handle
capsule, 0.8 cm across).

---

## (a) Caliper sweep results — full force curve, 2 mm to 60 mm, 2 mm steps

Gravity is disabled in the throwaway model (isolates "can the jaws pinch and hold this
thickness" from an unrelated "did it also fall" confound — this is a caliper measurement,
not a pick-and-lift task). `OPEN_SETTLE_STEPS=300`, `CLOSE_STEPS=700`,
`FORCE_AVERAGE_WINDOW=100` (last 100 of the 700 CLOSE steps).

| T (mm) | open-settle max displacement (m) | open-settle max force (N) | both-jaw contact frac (last 100 close steps) | avg normal force at close (N) | final displacement (m) | pushed away | GRASPABLE |
|---:|---:|---:|---:|---:|---:|---|---|
| 2  | 0.19478 | 0.0682 | 0.00 | 0.0000 | 0.65746 | True | False |
| 4  | 0.19385 | 0.1573 | 0.00 | 0.0000 | 0.65600 | True | False |
| 6  | 0.19216 | 0.2739 | 0.00 | 0.0000 | 0.65080 | True | False |
| 8  | 0.16587 | 0.4218 | 0.00 | 0.0000 | 0.56110 | True | False |
| 10 | 0.20792 | 0.5383 | 0.00 | 0.0000 | 0.70472 | True | False |
| 12 | 0.21521 | 0.7122 | 0.00 | 0.0000 | 0.72798 | True | False |
| 14 | 0.23467 | 0.1887 | 0.00 | 0.0000 | 0.79154 | True | False |
| 16 | 0.24552 | 0.2527 | 0.00 | 0.0000 | 0.82786 | True | False |
| 18 | 0.25557 | 0.3259 | 0.00 | 0.0000 | 0.86080 | True | False |
| 20 | 0.27621 | 0.3625 | 0.00 | 0.0000 | 0.93079 | True | False |
| 22 | 0.29959 | 0.3611 | 0.00 | 0.0000 | 1.00900 | True | False |
| 24 | 0.33022 | 0.6442 | 0.00 | 0.0000 | 1.11372 | True | False |
| 26 | 0.32410 | 0.7052 | 0.00 | 0.0000 | 1.09141 | True | False |
| 28 | 0.36916 | 1.7729 | 0.00 | 0.0000 | 1.24578 | True | False |
| 30 | 0.36766 | 1.8173 | 0.00 | 0.0000 | 1.24059 | True | False |
| 32 | 0.34021 | 1.8627 | 0.00 | 0.0000 | 1.14532 | True | False |
| 34 | 0.42429 | 2.2624 | 0.00 | 0.0000 | 1.43261 | True | False |
| 36 | 0.40363 | 2.6847 | 0.00 | 0.0000 | 1.36057 | True | False |
| 38 | 0.43881 | 1.2795 | 0.00 | 0.0000 | 1.47812 | True | False |
| 40 | 0.45499 | 1.4066 | 0.00 | 0.0000 | 1.53252 | True | False |
| 42 | 0.46743 | 1.5161 | 0.00 | 0.0000 | 1.57373 | True | False |
| 44 | 0.48571 | 1.6892 | 0.00 | 0.0000 | 1.63645 | True | False |
| 46 | 0.48210 | 1.5979 | 0.00 | 0.0000 | 1.62284 | True | False |
| 48 | 0.51966 | 1.2957 | 0.00 | 0.0000 | 1.75075 | True | False |
| 50 | 0.47451 | 1.3950 | 0.00 | 0.0000 | 1.59812 | True | False |
| 52 | 0.48894 | 1.4974 | 0.00 | 0.0000 | 1.64658 | True | False |
| 54 | 0.53591 | 1.6761 | 0.00 | 0.0000 | 1.80703 | True | False |
| 56 | 0.51071 | 1.8377 | 0.00 | 0.0000 | 1.71977 | True | False |
| 58 | 0.52719 | 1.8229 | 0.00 | 0.0000 | 1.77516 | True | False |
| 60 | 0.55449 | 1.8729 | 0.00 | 0.0000 | 1.86774 | True | False |

**Minimum graspable thickness: none found in [2, 60] mm.**
**Maximum graspable thickness: none found in [2, 60] mm.**

**Reading the force curve.** `avg normal force at close` is 0.0000 N at every single `T`
— not because the gripper never touches the box, but because by the time the last 100
of 700 CLOSE steps are sampled, the box has already been ejected (`final displacement`
climbs from 0.66 m at T=2mm to 1.87 m at T=60mm) and is no longer in contact with
anything. The real force signal is in the **open-settle max force** column: it is
non-zero from the very first (T=2mm) row and grows roughly monotonically with `T` (broad
noise reflects which contacts happen to engage first for a given box size, not a stable
trend) — meaning the box registers a real, physically-computed contact force **even
before CLOSE is ever commanded**, growing as thickness grows, consistent with the
mechanical explanation above (the insertion point sits inside a region where the two
jaws' convex hulls already overlap, so the box overlaps solid-seeming material on both
sides from the moment it is placed).

**This is a legitimate negative result, reported plainly per this task's own
instruction:** as currently modelled (post Fix-D-revert — see `DECISIONS.md`), this
gripper could not be measured to hold ANY of the 30 test-box thicknesses swept, at the
one pinch-point calibration this script located and the six additional hand-picked
neighbours also tried during calibration (see "Method", above). This is consistent with
— and gives a concrete mechanical explanation for — every prior scripted-pick failure
already on record in `DECISIONS.md` (`pick(A, plate)`, `pick(A, mug)`, `pick(A, bottle)`
all failing to achieve a stable lift, for reasons ADR-024 attributed to jaw-geometry size
mismatch without yet identifying the specific convex-hull-overlap mechanism this session
found).

**Scope of this negative result, stated honestly.** This measured ONE arm (A), ONE pose
(the scene's "home" keyframe, ADR-026), and a local neighbourhood (a handful of hand
offsets plus a 125-point static grid) around ONE calibrated pinch axis. It is not an
exhaustive proof that no configuration of this gripper anywhere could ever pinch
anything — but combined with the mechanical root cause (both collision geoms are
non-convex parts collapsed to single, mutually-overlapping convex hulls across the
entire joint range) and the historical record of every previous scripted pick attempt
also failing to lift a prop, this is a convergent, multi-method finding, not an isolated
fluke.

---

## (b) Prop grip-dimension inventory (measured from the compiled model)

Measured via `TableSettingEnv` (the committed, compiled `so101_dual_table.xml`), reading
`model.geom_size` / `model.geom_pos` / `model.body_mass` directly — not read off the
generator's XML text by eye (`scripts/probe_grasp_envelope.py --skip-sweep`).

| Prop | Geom | Type | size (m) | local pos (m) |
|---|---|---|---|---|
| plate | `plate_foot` | cylinder | radius 0.0300, half-height 0.0050 | (0, 0, 0) |
| plate | `plate_dish` | cylinder | radius 0.0600, half-height 0.0040 | (0, 0, 0.009) |
| mug | `mug_body` | cylinder | radius 0.0350, half-height 0.0400 | (0, 0, 0) |
| mug | `mug_handle` | capsule | radius 0.0080, half-length 0.0160 | (0.0475, 0, 0) |
| fork | `fork_handle` | capsule | radius 0.0040, half-length 0.0450 | (-0.015, 0, 0) |
| fork | `fork_head` | box | half-extents (0.0250, 0.0120, 0.0030) | (0.055, 0, 0) |
| spoon | `spoon_handle` | capsule | radius 0.0040, half-length 0.0475 | (-0.0125, 0, 0) |
| spoon | `spoon_bowl` | ellipsoid | semi-axes (0.0180, 0.0120, 0.0040) | (0.05, 0, 0) |
| water_bottle | `water_bottle_body` | cylinder | radius 0.0300, half-height 0.0900 | (0, 0, 0) |
| water_bottle | `water_bottle_cap` | cylinder | radius 0.0120, half-height 0.0100 | (0, 0, 0.1) |

**Derived (human-readable) dimensions, and total `body_mass` per prop as compiled:**

| Prop | body_mass (kg) | Key dimensions |
|---|---:|---|
| **plate** | 0.1500 | dish diameter 0.1200 m; dish thickness at rim 0.0080 m; foot diameter 0.0600 m; dish rests directly on foot top (confirmed, zero gap there); open-air gap under the overhanging rim (radially outside the foot footprint) = 0.0100 m |
| **mug** | 0.2400 | outer diameter 0.0700 m; height (body only) 0.0800 m; wall thickness not separately modelled — `mug_body` is a single solid cylinder, no hollow interior geom; handle cross-section: 0.0160 m diameter (round capsule) |
| **fork** | 0.0300 | handle width = thickness = 0.0080 m (round capsule); head 0.0500 × 0.0240 × 0.0060 m (L×W×H) |
| **spoon** | 0.0300 | handle width = thickness = 0.0080 m (round capsule); bowl semi-axes 0.0180 × 0.0120 × 0.0040 m (ellipsoid) |
| **water_bottle** | 0.3100 | body diameter 0.0600 m; body height 0.1800 m; total height incl. cap ≈ 0.2000 m |

These confirm the task prompt's own XML citations exactly: mug `size="0.035 0.04"` (70 mm
across), plate dish `size="0.06 0.004"` (8 mm thick), fork handle capsule `r=0.004` (8 mm),
plate foot `r=0.03` — all match the compiled model.

**Jaw opening, for reference (measured previously, `scripts/probe_jaw_opening.py` /
DECISIONS.md's Fix D revert entry):** the two jaw collision geoms' centroid-to-centroid
separation is on the order of **3.1-5.5 cm** across the joint's full range (varies with
which reference point is used — see "Method" above for why this number is a coarse
proxy, not a true clear aperture). No prop's largest dimension needs to fit inside this;
only the SMALLEST cross-section of an edge-on or handle-on grasp does (see the matrix
below).

---

## (c) Compatibility matrix

Required grip force uses the task's own formula: **mass × g × 2 (support margin) × 2
(friction margin)**, i.e. `mass × 9.81 × 4`, compared against the gripper actuator's own
declared `forcerange` — **±3.35 N** (`scenes/so101/so101_new_calib.xml:162`, read-only,
confirmed in `DECISIONS.md`'s Fix C entry; this is the actuator's hard force ceiling
regardless of jaw geometry).

| Prop | Orientation | Cross-section vs. ~3-5.5 cm jaw span | Jaws open wide enough? | Two-jaw contact (this measurement) | Required grip force (N) | Within actuator's ±3.35 N? |
|---|---|---|---|---|---|---|
| **plate** | encircle whole disc (dish Ø12 cm or foot Ø6 cm) | 6-12 cm — exceeds jaw span | **N** | not applicable (jaws can't close around it) | 5.89 | N |
| **plate** | edge-on at the rim (8 mm thickness) | 0.8 cm — within jaw span | Y | **No two-jaw contact measured for T=8 mm (0.00 both-jaw-contact fraction)** | 5.89 | **N** (5.89 > 3.35) |
| **mug** | encircle the body (Ø7 cm) | 7 cm — exceeds jaw span | **N** | not applicable | 9.42 | N |
| **mug** | grasp the handle (Ø1.6 cm) | 1.6 cm — within jaw span | Y | **No two-jaw contact measured for T=16 mm (0.00 both-jaw-contact fraction)** | 9.42 | **N** (9.42 > 3.35) |
| **fork** | grasp the handle (0.8 cm) | 0.8 cm — well within jaw span | Y | **No two-jaw contact measured for T=8 mm** | 1.18 | Y (1.18 < 3.35) — force alone would be fine |
| **spoon** | grasp the handle (0.8 cm) | 0.8 cm — well within jaw span | Y | **No two-jaw contact measured for T=8 mm** | 1.18 | Y (1.18 < 3.35) — force alone would be fine |
| **water_bottle** | encircle the body (Ø6 cm) | 6 cm — exceeds jaw span | **N** | not applicable | 12.16 | N |
| **water_bottle** | encircle the cap (Ø2.4 cm) | 2.4 cm — near/above jaw span upper end | Marginal | not measured directly (cap not in the 2-60mm sweep's tested location) | 12.16 | **N** (12.16 > 3.35) |

---

## (d) Recommendation table

| Prop | Recommendation | Reason |
|---|---|---|
| **plate** | **NOT GRASPABLE** | Whole-disc orientations exceed the jaw's own opening span outright. The one orientation with a small-enough cross-section (rim, edge-on) both (i) measured zero sustained two-jaw contact anywhere in this session's search, and (ii) needs 5.89 N against an actuator ceiling of 3.35 N — two independent, compounding reasons. |
| **mug** | **NOT GRASPABLE** | Body is geometrically too wide for the jaws to open around. The handle is the only geometrically plausible orientation, but needs 9.42 N (2.8x the actuator's ±3.35 N ceiling) even before considering that no two-jaw contact was measured. |
| **fork** | **MARGINAL** — narrow window, unconfirmed by this measurement | The handle is the only plausible grasp (0.8 cm, well inside the jaw's span) and the force required (1.18 N) is comfortably inside the actuator's own ±3.35 N ceiling — force is not the blocker here. But this session's caliper sweep, at the one calibrated pinch location tested, measured **zero** sustained two-jaw contact at T=8 mm (or any nearby T). This is the one prop/orientation where geometry and force both look plausible on paper; whether it is actually graspable depends on finding a pinch location this session's search did not locate — flagged as the narrowest, most-worth-revisiting case, not ruled out with the same confidence as the others. |
| **spoon** | **MARGINAL** — same reasoning as fork | Identical handle cross-section (0.8 cm) and required force (1.18 N) to the fork; same unconfirmed-by-measurement caveat. |
| **water_bottle** | **NOT GRASPABLE** | Body is geometrically too wide for the jaws. The cap is borderline-fits geometrically but needs 12.16 N — the largest of any prop, 3.6x the actuator's ceiling — regardless of contact. |

**Overall:** of five props, **zero** are confirmed graspable by this functional
measurement; three (plate, mug, water_bottle) are ruled out by geometry and/or actuator
force alone, independent of the two-jaw-contact finding; two (fork, spoon) remain
force-plausible but were not measured to achieve sustained two-jaw contact at the one
pinch location this session calibrated and searched around.

---

## Files produced by this diagnostic

- `scripts/probe_grasp_envelope.py` — the caliper-sweep script (this report's numbers).
- `docs/hardware/grasp-envelope.md` — this file.

No other file is touched by this diagnostic. `scenes/so101/`, `ik.py`, `executor.py`,
grasp offsets, jaw geometry and prop dimensions are all unmodified — this is a report to
inform a user decision, not a fix.
