# v2 Stage 4 — phase-based skills: measured results (WIP)

**Status: INCOMPLETE.** ADR-073 is deliberately NOT written yet — this stage is
not finished, and an ADR claiming otherwise would be false.

> **The "Five-criterion scoring" table immediately below is the PRE-PART-A
> state and is superseded.** Read **PART A** further down first: it retracts
> that table's `pick` PASS (it passed on a weld to a fork that was not between
> the jaws) and supersedes the `place` diagnosis (the 12.5 mm offset was a
> symptom of the grasp geometry, and is now fixed). The table is kept as the
> record of what was measured before the fix.

The Stage 4 builder stalled (no progress for 600 s) after writing
`skills_v2.py` and the `executor.py` routing flag but before validating or
committing. The code was preserved and validated by the orchestrator with an
**independent** monitor (`scripts/v2_criteria_monitor.py` — the same instrument
that produced `v2-master-baseline.md`), so no stage grades its own work.

Numbers below are from the laptop. They must be re-run on bm-ptl (ADR-047)
before any of them goes into an ADR.

## Five-criterion scoring, seed 0

| skill | steps | (a) outcome | (b) props ≤5 mm | (c) no arm-prop | (d) no cross-arm | (e) peak vel < 2.6 | verdict |
|---|---|---|---|---|---|---|---|
| `pick(A, fork)` | 2504 | OK | OK | OK | OK | OK 1.832 rad/s | **PASS** |
| `place(A, fork, ·)` | 2300 | **BAD** | OK | OK | OK | OK 0.081 rad/s | FAIL |
| `handoff(A→B, fork)` | 2500 | **BAD** | **BAD** | OK | **BAD** | OK 2.358 rad/s | FAIL |

Master, scored by the same instrument, failed all three — and passed (a) on all
three. See `v2-master-baseline.md`.

### `pick(A, fork)` — PASS

The first skill in this project to satisfy the full scene-integrity
definition. Peak joint velocity **1.832 rad/s** against master's **6.937**.
No prop displaced, no arm-prop contact, no cross-arm contact.

### `place(A, fork, target)` — fails (a) only, by 12.5 mm

Scene behaviour is excellent: nothing displaced, nothing touched, peak joint
velocity **0.081 rad/s**. It misses its target:

    target [-0.050,  0.050, 0.3560]
    final  [-0.0581, 0.0592, 0.3538]   -> 12.5 mm off

The fork rests correctly on the table (z=0.3538 against a 0.35 surface); the
error is in xy. Likely the object shifting in the jaws during grasp/release
rather than an IK error — IK round-trip is 2e-16 m (ADR-070). Not yet
diagnosed.

### `handoff(A→B, fork)` — fails at phase 2, diagnosed

    HANDOFF phase 2 (to_arm approach hover) motion failed at sub-step 4/4:
    final error 0.335940 rad > tol 0.01 rad

Arm A is correctly parked at `P_from` (pinch `[-0.030, -0.0003, 0.4589]`).
Arm B **stalls high at z=0.5577**, never reaching its 0.48 hover, blocked by:

    armB_moving_jaw_so101_v1  <->  armA_lower_arm     (129 contact-steps, worst -0.0047 m)

`mug` also displaced 0.0178 m as a consequence.

## The handoff diagnosis — what it is, and what does NOT fix it

**Every static endpoint pair is clean.** Verified directly: A at `P_from` with
B at `P_to`; A at `P_from` with B at hover(0.48); both at hover. Zero cross-arm
contacts in all three. The problem is strictly **in transit**.

This is v1's ADR-062 finding recurring at the trajectory level rather than the
geometry level: *reachable at every endpoint and still colliding en route*.

**Ideal straight-line transit (arm B home → hover), 101 samples, arm A parked:**

| handoff z / hover z | dx=0.03 | dx=0.05 | dx=0.07 | dx=0.09 |
|---|---|---|---|---|
| 0.46 / 0.48 | **-0.0269** | -0.0031 | clean | clean |
| 0.45 / 0.47 | **-0.0220** | -0.0034 | clean | clean |
| 0.44 / 0.46 | **-0.0485** | -0.0035 | clean | clean |

Collision consistently at s≈0.87, `armB_moving_jaw_so101_v1` ↔ `armA_lower_arm`.

*(An earlier 21-sample sweep by the orchestrator reported this path clean. That
was wrong: it stepped s by 0.05 and straddled the s≈0.87 window. Recorded
because the false negative is exactly the kind of sampling error that lets a
collision reach a demo video.)*

**Widening the grip separation does not work.** dx=0.07 (14 cm apart) is clean,
but the fork is only **0.14 m long overall**, with a **0.09 m handle**
(`fork_handle` capsule, local x -0.06..+0.03; `fork_head` box, +0.03..+0.08).
Grips 14 cm apart would not be on the fork. Measured, not assumed.

**Lateral staging does not work either.** Staging arm B at `(0.03, y, 0.47)`
before coming in:

| y_stage | home→stage | stage→hover | hover→grip |
|---|---|---|---|
| 0.06 | **-0.0300** | clean | clean |
| 0.08 | **-0.0157** | clean | clean |
| 0.10 | **-0.0130** | clean | clean |
| 0.12 | -0.0005 | **-0.0018** | clean |

Staging cleans the final legs but the **home→stage** leg then collides; pushing
the stage further out merely moves the collision into `stage→hover`
(`armB_moving_jaw` ↔ `armA_wrist`). No single straight-leg route clears it.

**Why widening in x helps so little:** both arms approach along **y** (bases at
y=∓0.25, handoff line at y=0), so their forearms converge regardless of an
**x** offset. The offset is perpendicular to the direction that would actually
create forearm clearance.

## A finding the stage made that the brief did not anticipate

`_split_move` (`MOVE_SPLIT_STEPS = 4`): a single joint-space cubic spanning all
5 joints from the folded home pose to a reach-down pose can drive the gripper
through the **table** mid-transit even when both endpoints are collision-free —
joint-space interpolation has no notion of the Cartesian path it traces. The
builder confirmed this was real contact-force blocking rather than slow PD
tracking by reading `data.qfrc_actuator` pinned at its `forcerange` bound while
`data.qfrc_constraint` exactly opposed it. Splitting each large move into 4
linear sub-legs measured clean. This is the same swept-path class of problem as
the handoff failure above, and it is why `pick` passes.

## What remains

1. Solve the handoff transit collision. Straight joint-space legs between
   home, staging, hover and grip are insufficient; the remaining options are a
   genuine multi-waypoint route found by search, an elbow-branch choice for arm
   A that keeps its forearm clear (Lab 7 states no elbow-up/down preference, so
   this is an open choice), or moving arm A out of the shared volume between
   phases.
2. Diagnose `place`'s 12.5 mm end-state error.
3. Re-run everything on bm-ptl.
4. Then, and only then, write ADR-073.

## PART A — the 12.5 mm place offset: root cause was the grasp, not the release

**Fixed.** `place`'s criterion (a) now passes. The cause was three stacked
constants, all traceable to one geometric fact the brief did not know.

### The geometric fact

In a genuine top-down pose the gripper's two finger pads sit at **very
different** offsets below the ADR-025 pinch point, and those offsets are
constant across the workspace (verified at six targets, identical to 5 dp):

    armA_static_finger_pad   -0.08298 m
    armA_moving_finger_pad   -0.02015 m

So the ADR-025 pinch point is **not** the grasp centre for this gripper. This
is why master can target the pinch directly at the object and still grasp:
master's position-only IK leaves orientation uncontrolled, so its pose is not
top-down and its pads sit only ~0.013 m below the pinch. I initially concluded
the Stage 4 builder had compensated for a phantom offset, measuring master's
pose; that was wrong, and the builder's measurement was right.

### The window

Two constraints bound the grasp clearance, and they nearly conflict:

  - the static pad must clear the table  ->  pinch >= 0.35 + 0.08298 = 0.43298
  - the fork must lie BETWEEN the pads   ->  pinch <= 0.356 + 0.08298 = 0.43898

Measured feasible clearances: **0.078, 0.080, 0.082** — a 4 mm window. Below
0.078 the static pad penetrates the table (3-4 arm-table contacts). At 0.084
the pad clears but the fork is **not** between the pads. At 0.086+ the target
is unreachable.

`GRIPPER_STATIC_PAD_CLEARANCE_M` was **0.084** — 2 mm past the top of the
window. That is why `attempt_grasp` only succeeded once its gate had been
widened to **0.12 m**: it was welding a fork 8.6 cm from the pinch and *not
between the jaws*, then dragging it along below the gripper. The lift
"succeeded" because criterion (a) only asks whether the fork rose.

**So the earlier "pick PASSES all five criteria" result was not trustworthy,
and is retracted.** It passed on a grasp that was not a grasp.

### Yaw: the second finding

With clearance corrected to 0.080 the grasp is sound but the lift fails —
only 4 mm of headroom remained. The reachable pinch ceiling at the fork's xy
turns out to depend strongly on **yaw**, which the brief treated as fixed at
`heading + 90 deg`:

| yaw | ceiling | headroom above the 0.436 grasp |
|---|---|---|
| 90 deg (current) | 0.440 | +0.004 |
| 0 / 180 deg | 0.466 | +0.030 |
| 225 deg | 0.476 | +0.040 |
| **270 deg** | **0.478** | **+0.042** |

270 deg is 90 deg + 180 deg: for a two-jaw gripper it is the **same grip**
(`d_static`/`d_moving` identical), but the arm is far less extended. The fork
sits at y=+0.05, 30 cm from arm A's base at y=-0.25, so it is near the edge of
the top-down workspace and the arm's configuration matters more than the grip
orientation does.

### Applied

    GRIPPER_STATIC_PAD_CLEARANCE_M   0.084 -> 0.080   (the measured window)
    WELD_GRASP_DISTANCE_THRESHOLD_M  0.12  -> 0.09    (actual is 0.0854)
    HOVER_CLEARANCE_M                0.02  -> 0.03    (= Lab 8's own value)

Verified: fork **between the pads**, |fork - pinch| = 0.0854 m (under the 0.09
gate, so the gate is a real check again), lifted to z=0.3768 > 0.3700.

### Result after Part A

| skill | (a) | (b) | (c) | (d) | (e) | verdict |
|---|---|---|---|---|---|---|
| `pick(A, fork)` | OK | OK | OK | OK | OK 1.832 | **PASS** (genuine grasp) |
| `place(A, fork, ·)` | **OK** | BAD plate 16.7 mm | BAD plate -1.6 mm | OK | OK 0.154 | FAIL |
| `handoff(A→B, fork)` | BAD | BAD mug 18.9 mm | OK | BAD 129 steps | OK 2.358 | FAIL |

`place`'s stated Part A defect is resolved. Its remaining failure is a new and
much smaller one: the plate is nudged 16.7 mm with 1.6 mm penetration, just
over the 1 mm bar.
