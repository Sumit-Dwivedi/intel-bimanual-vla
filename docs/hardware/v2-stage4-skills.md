# v2 Stage 4 — phase-based skills: measured results (WIP)

**Status: INCOMPLETE.** `pick` passes all five criteria. `place` fails only on
end-state accuracy. `handoff` fails on a genuine, diagnosed cross-arm transit
collision that is NOT yet solved. ADR-073 is deliberately NOT written yet —
this stage is not finished, and an ADR claiming otherwise would be false.

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
