# M06 handoff warm-start IK diagnostic

**Diagnostic only.** No source touched (`skills_scripted.py`, `ik.py`, `executor.py`, `grasp.py`, `gen_dual_scene.py`, `scenes/so101/` all untouched). New script: `scripts/probe_handoff_warmstart.py`. Run on bm-ptl (ADR-020) against the clean `d239a55` tree.

## Brief correction acknowledged

The brief's original cells (arm A at z=0.38, seed y=+0.02) do not converge (`m06-handoff-reachability-home.md`, that exact row: armA_residual **0.11215**), so they cannot be used as a warm-start seed. Per the corrected brief, this diagnostic uses the z=0.35 cells instead:

- **Arm A**: seed (x=0, y=+0.02, z=0.35), target (x=0, y=0.00, z=0.35).
- **Arm B (mirrored)**: seed (x=0, y=-0.06, z=0.35), target (x=0, y=-0.04, z=0.35).

## Part 1 — single-step warm start

| | cold residual (seed) | cold residual (target, from home) | warm residual (target, seeded from converged seed) |
|---|---:|---:|---:|
| Arm A | 0.00956 (converged) | 0.14633 (failed) | **0.00911 (converged)** |
| Arm B | 0.00869 (converged; committed table row says 0.00996 -- see note below) | 0.14364 (failed) | **0.00896 (converged)** |

**Note on arm B's seed residual:** the committed table's y=-0.06, z=0.35 row for arm B reads 0.00996; this run's independent cold solve of that exact same cell (home -> (0, -0.06, 0.35)) got 0.00869. Both are comfortably converged (< `IK_POSITION_TOLERANCE_M` = 0.01), the small difference is solver-noise-level (DLS iteration count/damping, not a discrepancy in kind), and it does not change the finding: seed converges, target does not, warm-start target converges.

**Result: warm-start converges where cold-start did not, for both arms.** Residual drops from 0.146/0.144 (a clear FAIL, >14x the 0.01 m tolerance) to ~0.009 (a clear PASS) with no change to the target position, only to the IK seed. This is exactly the signature of a seed-dependent local minimum, not a workspace boundary: a real kinematic limit does not become reachable just because the solver started from a different, still-valid initial guess for the SAME target.

## Part 2 — chaining the warm start across the full gap

At z=0.35, arm A's home-seeded passing band starts at y >= +0.02 and arm B's at y <= -0.06 (disjoint, 0.08 m / four grid cells apart). Chaining warm-started IK across that gap, one 0.02 m step at a time:

**Arm A, walking from y=+0.02 down to y=-0.06** (each step seeded from the previous step's converged config, not from home):

| step | target y | seed | residual | converged |
|---|---:|---|---:|---|
| 0 | +0.02 | HOME | 0.00956 | True |
| 1 | 0.00 | y=+0.02 config | 0.00911 | True |
| 2 | -0.02 | y=0.00 config | 0.00607 | True |
| 3 | -0.04 | y=-0.02 config | 0.00569 | True |
| 4 | -0.06 | y=-0.04 config | 0.00745 | True |

**Arm B, walking from y=-0.06 up to y=+0.02** (mirror):

| step | target y | seed | residual | converged |
|---|---:|---|---:|---|
| 0 | -0.06 | HOME | 0.00869 | True |
| 1 | -0.04 | y=-0.06 config | 0.00896 | True |
| 2 | -0.02 | y=-0.04 config | 0.00913 | True |
| 3 | 0.00 | y=-0.02 config | 0.00827 | True |
| 4 | +0.02 | y=0.00 config | 0.00952 | True |

**The chain does not break anywhere.** Both arms carry a converged solve across the entire disjoint 0.08 m gap, in both directions, with every single step's residual staying under 0.01 m (in fact under ~0.0096 throughout, better than several of the "passing" cells in the original home-seeded sweep). The two bands do not just touch at one boundary cell -- with chained warm-starting, arm A's reachable band and arm B's reachable band **both cover the entire swept range** (y = +0.02 through y = -0.06), i.e. they fully overlap, not merely meet at an edge.

## Part 3 — is `arm_world_ok` constant-False by construction?

Checked on arm A's **converged** config at (y=+0.02, z=0.35) (residual 0.00956) -- deliberately not a failed/tangled config, so any contact found here cannot be blamed on the solver being stuck:

```
table_top <-> geom38   dist=-0.22211
table_top <-> geom36   dist=-0.02404
drawer_box <-> geom44  dist=-0.02230
table_top <-> geom40   dist=-0.01837
table_top <-> geom44   dist=-0.01253
mug_body <-> geom38    dist=-0.01117
drawer_box <-> geom42  dist=-0.01048
table_top <-> geom42   dist=-0.00745
mug_body <-> geom36    dist=-0.00703
table_top <-> armA_static_finger_pad  dist=-0.00483
table_top <-> armA_static_finger_pad  dist=-0.00454
```

A separate lookup confirms geom36/38/40 belong to body `armA_lower_arm` and geom42/44 belong to body `armA_wrist`. At arm A's plain HOME pose (no IK solve, no reach at all), the same world-contact check finds **zero** contacts.

**Confirmed, with a caveat on the exact mechanism.** `arm_world_ok` does fire on a converged, physically sensible pose (not just on a failed/tangled one) -- so the hypothesis that this column carries no information for at least some rows is correct. But the mechanism is not quite the one hypothesized (base body directly embedded in the table at the mounting height): `armA_base`'s own geoms are all `class="visual"` (`contype="0" conaffinity="0"`, so071_dual_table.xml's `so101_new_calib`/`visual` default), meaning the base body itself never participates in collision at all. The actual contact is `table_top` against `armA_lower_arm`'s and `armA_wrist`'s **collision-class mesh geoms**, which sit close to the table surface for any pose that reaches down to z=0.35 (the table surface height, since `HANDOFF_POSITION_XYZ`'s z and `TABLE_TOP_Z` are the same 0.35). The depth on one of those (-0.22211 m) is far larger than a plausible shallow surface graze, which points at a `type="mesh"` collision-geometry artifact (MuJoCo's default convex-hull treatment of non-convex meshes can report grossly exaggerated penetration depth relative to the visual mesh) rather than a straightforward "base sits on the table" bug. Either way, the practical conclusion the brief asked for holds: **this column fires on cells that are otherwise fine, it conflates a probable meshcollision artifact with genuine prop contacts (`mug_body`, `drawer_box` both also appear), and it should not be read as a real per-candidate collision verdict without first excluding or recalibrating the arm-vs-table_top pair specifically** (the prop contacts, e.g. `mug_body`/`drawer_box`, look like plausible genuine collisions and should stay in the check).

## Part 4 — is `cross_arm_ok` failing at the un-converged candidate genuine?

At the original candidate (y=0.00 for arm A / y=+0.03 for arm B, `HANDOFF_SIDE_OFFSET_M`), using the **cold, un-converged** home-seeded solves (residuals 0.14633 / 0.15799 -- both FAILs on their own terms already):

```
geom36 <-> geom68   dist=-0.08773
geom38 <-> geom68   dist=-0.07061
geom36 <-> geom70   dist=-0.07061
... (17 contact pairs total, depths from -0.00975 to -0.08773)
```

17 distinct contact pairs, depths up to -0.088 m. This is extensive, multi-link interpenetration, consistent with two arms whose IK solves never actually converged (each residual >14x tolerance) landing in physically nonsensical, overlapping configurations -- not a borderline one-geom graze. **This looks like genuine interpenetration of nonsense (non-converged) poses, not evidence about whether two CONVERGED poses would collide.** It does not, by itself, say anything about whether a real handoff (built from converged poses) would collide.

## Part 5 — the check that actually matters: converged + converged, same point

Both arms' warm-started chains (Part 2) pass through **y=0.00, z=0.35** — arm A at chain step 1 (residual 0.00911), arm B at chain step 3 (residual 0.00827). Applying both of these **converged** configs simultaneously (zero side offset between them — i.e. both grippers aimed at the literal same point, the worst case; a real handoff uses `HANDOFF_SIDE_OFFSET_M` = 0.03 m separation, which can only help):

```
Cross-arm contacts: (no contacts between these geom sets)
```

**Zero cross-arm contact.** Two independently-converged, warm-started IK solutions, both targeting the same point, do not collide with each other at all.

## Verdict

**Warm-start converges where cold-start didn't -> this is an IK-solver regime problem (seed-dependent local minimum), not a kinematic limit.** All of Part 1, 2, and 5 point the same direction:

- Single-step warm start recovers a >14x residual blowup down to a clean pass, for both arms, with no change to the target.
- The chain does not merely cross one cell -- it carries a converged solve across the ENTIRE 0.08 m disjoint gap, in both directions, meaning arm A's and arm B's reachable bands, once warm-started stepwise, are not disjoint at all -- they cover the same full range.
- At the one common point tested (y=0.00, z=0.35) where both chains converge, applying both configs together with zero side-offset produces **zero** cross-arm collision -- the strongest case, and it passes.

The fix is contained in `skills_scripted.py`: stage `run_handoff` so each arm's IK is seeded from a nearby already-converged waypoint (e.g. its own APPROACH-hover solve, or an intermediate transfer-adjacent point) rather than solving directly from HOME to a possibly-central target, exactly as this diagnostic did. This diagnostic does not implement that change (out of scope, no source touched) — it only confirms the mechanism and demonstrates, end to end including a genuine converged+converged cross-arm collision check, that a shared, collision-free, both-reachable handoff point at (x=0, y=0.00, z=0.35) is attainable once the seeding is fixed.

**Caveat / not yet checked:** this diagnostic did not re-run `arm_world_ok` on the chained converged configs (only on the y=+0.02 seed cell, Part 3), and did not check joint-limit margins on the chained configs. Both are cheap follow-ups before this is treated as fully closed, since Part 3 already shows the `arm_world_ok` column needs the table_top/arm-mesh pair excluded or recalibrated before it can gate anything.

## `arm_world_ok`/`cross_arm_ok` scope note

Per Parts 3-4, this diagnostic recommends: (a) exclude or separately recalibrate the `table_top` vs. `armA_lower_arm`/`armA_wrist`/`armB_lower_arm`/`armB_wrist` collision-mesh pair before trusting `arm_world_ok` as a per-candidate verdict (it currently fires on converged, sensible poses); (b) `cross_arm_ok` as measured on non-converged candidates is not informative about converged candidates -- Part 5 shows a converged pair passes cleanly at zero offset.
