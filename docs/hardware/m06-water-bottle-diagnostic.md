# M06 water_bottle GRIP-stage diagnostic: `pick(A, water_bottle)`

**Diagnostic only -- no behaviour change, no fix applied.** Produced by
`scripts/probe_water_bottle_grip.py`, which reproduces (does not modify)
`skills_scripted.run_pick`'s APPROACH/DESCEND waypoints and `_dwell`'s
ADR-031 frozen-arm GRIP dwell for `pick(A, water_bottle)`, calling the real
`grasp.WeldGrasp.attempt_grasp` every dwell step exactly as `_dwell` does.
Run on bm-ptl (ADR-020), `mujoco==3.2.7`. See that script's module docstring
for exactly what is reproduced vs. imported (same convention as
`scripts/probe_grip_stage.py`, `scripts/probe_weld_grasp.py`: no per-step
collision early-exit in this probe's driving loop -- one documented way this
could diverge from the real skill).

Context: `pick(A, fork)` succeeds -- weld attaches at frame 1155, fork lifts
43 mm (`docs/hardware/m06-weld-verification.md`,
`docs/hardware/m06-grip-diagnostic-after-fix.md`). `pick(A, water_bottle)`
reaches GRIP but fails `weld_attach_failed_after_300_frames`. This report
measures the run end to end -- APPROACH and DESCEND included, not just the
GRIP dwell -- because the measurements below show the anomaly starts before
GRIP, not during it.

## Setup

- `water_bottle` body position at reset (before any waypoint):
  (0.2200, 0.0000, 0.4400)
- `GRASP_POINT_OFFSET_M['bottle']` (unchanged, read-only):
  (0.0000, 0.0000, 0.0200)
- `grasp_point` (obj_pos0 + offset): (0.2200, 0.0000, 0.4600)
- `hover` (grasp_point + CLEARANCE_HEIGHT_M on z): (0.2200, 0.0000, 0.5400)
- APPROACH: converged=**False**, steps=500 (exhausted the budget without
  reaching within `POS_CONVERGENCE_TOL_M=0.015` m of `hover`)
- DESCEND: converged=**False**, steps=500 (same, target `grasp_point`)

Fork's own pre-fix probe (`docs/hardware/m06-grip-diagnostic.md`) also
reports `APPROACH: False, 500` / `DESCEND: False, 500` for the same reasons
(this arm's 5-DOF redundant IK solve does not reliably dip under the tight
0.015 m tolerance in 500 steps even when it gets physically close) -- so
non-convergence of these two waypoints, by itself, is not unique to the
bottle. What IS unique to the bottle, measured below, is what the arm's
non-convergent motion did to the target object in the process.

## Measurement 1+2: `water_bottle` position and Arm A's pinch point at GRIP start

1. `water_bottle` body position at GRIP start (frame 0 of the 300-frame
   dwell, i.e. immediately after DESCEND's 500 steps): **(0.3772, 0.0366, 0.3790)**
2. Arm A's pinch point at GRIP start (midpoint of `armA_gripper` and
   `armA_moving_jaw_so101_v1` body `xpos` -- exactly what `ik.py`'s
   `_pinch_point()` and `grasp.WeldGrasp.attempt_grasp`'s Gate 2 compute):
   **(0.2192, -0.0008, 0.4501)**

**The water_bottle body has already moved from its reset position
(0.2200, 0.0000, 0.4400) to (0.3772, 0.0366, 0.3790) by the time GRIP even
starts** -- a displacement of about 0.157 m in x, 0.037 m in y, and -0.061 m
in z, entirely during the 1000 combined APPROACH+DESCEND steps, before any
GRIP-dwell dynamics are in play at all. `grasp_point` was computed from the
bottle's ORIGINAL position (0.22, 0, 0.46, per `GRASP_POINT_OFFSET_M`), so
by GRIP start the bottle is no longer anywhere near where the arm is
converging -- it has been displaced away from the point the whole skill is
built around.

## Measurement 3: distance between pinch point and bottle, every 30 frames through the 300-frame dwell

| step | pinch_point | bottle_pos | distance(m) | qpos(rad) | attached |
|---:|---|---|---:|---:|---|
| 1 | (0.2192, -0.0008, 0.4501) | (0.3773, 0.0365, 0.3789) | 0.1774 | 1.7449 | False |
| 30 | (0.2192, -0.0008, 0.4499) | (0.3802, 0.0338, 0.3792) | 0.1792 | 1.5994 | False |
| 60 | (0.2192, -0.0008, 0.4499) | (0.3832, 0.0310, 0.3792) | 0.1814 | 1.3219 | False |
| 90 | (0.2192, -0.0008, 0.4499) | (0.3862, 0.0282, 0.3795) | 0.1835 | 1.0071 | False |
| 120 | (0.2192, -0.0008, 0.4499) | (0.3892, 0.0253, 0.3797) | 0.1857 | 0.6815 | False |
| 150 | (0.2192, -0.0008, 0.4499) | (0.3922, 0.0225, 0.3795) | 0.1882 | 0.3529 | False |
| 180 | (0.2192, -0.0008, 0.4499) | (0.3952, 0.0197, 0.3796) | 0.1907 | 0.0233 | False |
| 210 | (0.2192, -0.0008, 0.4499) | (0.3988, 0.0174, 0.3796) | 0.1937 | -0.2072 | False |
| 240 | (0.2192, -0.0008, 0.4499) | (0.4039, 0.0155, 0.3793) | 0.1984 | -0.1770 | False |
| 270 | (0.2192, -0.0008, 0.4499) | (0.4121, 0.0134, 0.3772) | 0.2066 | -0.1745 | False |
| 300 | (0.2192, -0.0008, 0.4499) | (0.4247, 0.0094, 0.3684) | 0.2213 | -0.1745 | False |

Note the pinch point column: it is constant to 4 decimal places across
every one of these 300 steps ((0.2192, -0.0008, 0.450x), x varying only in
the 4th decimal). **The arm is not moving during this dwell** -- ADR-031's
freeze (frozen 5-joint ctrl, only the gripper joint commanded) is working
exactly as designed on this path. The growing distance is entirely the
**bottle** moving away, not the arm chasing anything.

The bottle's own drift accelerates in the back half of the dwell: x moves
~0.0030 m per 30-step block from step 1-210, then 0.0051 (210-240), 0.0082
(240-270), 0.0126 (270-300) m per block, while z, flat at ~0.379 m through
step 240, drops to 0.3684 m by step 300. This is consistent with the bottle
(a free-jointed cylinder) still carrying residual velocity from an earlier
disturbance and continuing to slide/roll across the table under its own
momentum and friction, independent of the (frozen, stationary) arm --
possibly approaching a tip-over, though this run's 300-step window does not
resolve that further.

## Measurement 4: gripper joint qpos, same intervals

Included as the `qpos(rad)` column in the table above. It moves exactly as
expected for a commanded jaw closure: 1.7453 (fully open, GRIP start) down
through 0.3529 at step 150, crossing the `closure_threshold=0.3` gate
between steps 150 and 180 (0.0233 at step 180), continuing to the actuator's
closed limit (-0.1745) by step 270-300. **The closure gate is not the
problem here** -- qpos crosses below 0.3 well within the 300-frame budget,
the same shape fork's own successful run shows.

## Comparison against fork's successful trajectory

Fork (`pick(A, fork)`, post-ADR-031, per this task's supplied reference):
pinch-point distance to target held flat 0.0351 -> 0.0374 m, well under the
0.050 m gate, while qpos fell 1.7449 -> 0.3519 over its dwell.

Water bottle (this run): pinch-point distance to the bottle went
**0.1774 -> 0.2213 m** -- 3.5-4.4x fork's distance, never within an order of
magnitude of the 0.050 m gate -- while qpos fell 1.7449 -> -0.1745 (crosses
0.3 comfortably).

Final: `attach_frame=None`, `weld.is_holding('A')=None`. Gate 1 (closure)
passes partway through the dwell; Gate 2 (proximity) never comes close to
passing, at any point in any of the 300 frames.

## Branch determination

The task frames three anticipated branches (flat-but-high distance / growing
distance from an ADR-031 freeze failure / under-threshold-but-gate-fails).
**None applies cleanly, because the premise underlying all three -- that the
bottle is still near its scripted `grasp_point` when GRIP starts -- does not
hold here.** Specifically:

- It is not the "flat distance, wrong offset" branch in the simple sense:
  the distance is not flat, it grows monotonically (0.1774 -> 0.2213 m)
  across the dwell.
- It is not the "ADR-031 freeze isn't taking effect" branch either: the
  pinch-point column above is constant to 4 decimals for all 300 steps --
  the freeze demonstrably IS in effect on this path. The growth is 100% the
  *bottle* moving, not the arm.
- It is not a closure-gate problem: qpos crosses the 0.3 threshold by step
  180, comfortably inside the 300-frame budget.

**The actual root cause precedes GRIP entirely.** By the time GRIP starts,
the water_bottle body has already been displaced ~0.157 m in x / 0.061 m in
z from its reset position, during the 1000 non-converged APPROACH+DESCEND
steps (both waypoints report `converged=False`, exhausting their full
500-step budgets, the same as fork's own pre-fix probe -- so non-convergence
alone is not the differentiator). The most direct reading: sometime during
APPROACH or DESCEND, some part of arm A's non-converged trajectory made
contact with and knocked the water_bottle -- a free-standing, comparatively
tall/narrow cylinder resting at z=0.44 (vs. the fork lying flat at z=0.354)
-- off its resting spot, and it is still sliding/rolling away (with
apparent acceleration late in the dwell, per Measurement 3) when GRIP's
300-frame dwell samples it. `grasp_point`/`hover` were computed once, from
the bottle's PRE-disturbance position, and never re-read afterward, so the
arm spends its entire GRIP dwell converged on (frozen at) a point in space
the bottle is no longer anywhere near.

This is consistent with, though not identical to, the pattern
`skills_scripted.py`'s own `_prop_collision_violations` comment already
documents finding once before for `pick(A, bottle)` (bottle knocked off the
table entirely, landing on the floor at z=0.0298) -- this run's bottle
does not leave the table (z stays 0.37-0.38 for most of the dwell, dropping
to 0.3684 by step 300) but the same underlying mechanism -- a fast-moving,
non-converged arm trajectory contacting a tall, narrow, easily-disturbed
prop before the grasp point is ever reached -- is the most direct
explanation available from this measurement. This diagnostic does not
instrument APPROACH/DESCEND's per-step contact geometry (out of the
20-minute scope given), so the exact contact event (which geom, which
step) is not pinpointed here; that would be the natural next probe.
