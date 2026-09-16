# v2 verdict — replacing the motion stack, and what it cost

Closing document for the `redesign-v2` branch. Written to be read on its own by
someone who has not followed the branch.

`master` is frozen at `238cfed` throughout and is not modified by any of this.

> **Where these files live.** This document is on `master`, together with the
> relay video (`docs/videos/v2-relay-demo.mp4`), its eight inspection frames
> (`docs/images/v2-frames/`) and the scoring instrument
> (`scripts/v2_criteria_monitor.py`) — so every comparison below can be
> re-measured. **The v2 source (`ik_geometric.py`, `motion.py`,
> `skills_v2.py`), `scripts/v2_stage5_demo.py`, `docs/hardware/submission-path.md`
> and ADR-070 through ADR-074 are on the `redesign-v2` branch, not on
> `master`** — the branch is pushed to origin and preserved as the evidence,
> the same convention this repo uses for the earlier `redesign` branch.

## 1. What v2 replaced

Master's motion stack is **position-only damped-least-squares IK driving direct
position commands**. Three properties follow from that, and all three are
visible in `docs/videos/full-sequence-demo.mp4` on master:

- **Orientation is uncontrolled.** Position-only IK satisfies a 3-DoF target
  and leaves the gripper's approach direction to whatever the solver's null
  space lands on.
- **There is no swept-path check.** Waypoints are validated at their endpoints;
  what the arm does between them is unconstrained.
- **"Success" was measured on the target object only.** A skill passed if the
  fork moved as intended. Nothing asked what happened to the rest of the table.

The consequence is not hypothetical. In master's own demo video the mug is
knocked onto its side and the water bottle is knocked off the table, while
every skill reports success.

## 2. What v2 built

Three new files. **No file master depends on was modified** — `ik.py`,
`skills_scripted.py`, `grasp.py`, `env.py` and `scenes/so101/` are untouched,
and `executor.py` gained only an opt-in routing flag (`use_v2`, default
`False`).

| file | what it is | ADR |
|---|---|---|
| `src/bimanual/control/ik_geometric.py` | closed-form geometric top-down IK | ADR-070 |
| `src/bimanual/control/motion.py` | cubic-spline joint-space motion primitive | ADR-071 |
| `src/bimanual/control/skills_v2.py` | phase-based skills + scene-integrity criteria | ADR-073 |

The IK is exact: round-trip position error **2.0e-16 m mean, 5.9e-16 m max**
over 500 random Cartesian targets, **0 joint-limit violations in 800**
solutions, and the gripper frame's top-down alignment is exactly **-1.000000**
(min = mean = max). Verified independently with a separate instrument and a
different RNG seed, not taken from the implementing stage's own report.

## 3. The five criteria

A skill passes only if **all five** hold:

| | criterion |
|---|---|
| (a) | the target object ends where the skill intended |
| (b) | no non-target prop displaced more than **5 mm** |
| (c) | no arm-to-prop contact deeper than **1 mm**, at any step |
| (d) | no cross-arm contact, at any step |
| (e) | peak joint velocity below **2.6 rad/s** |

(a) alone is master's definition. **(b) through (e) are the ones master would
fail**, and the point of the redesign is that they are now measured rather
than assumed.

The (e) threshold is derived, not picked: peak velocity of a cubic with zero
endpoint velocities is `1.5 * delta / T`, and the largest single-joint sweep
any phase legitimately commands is the `wrist_roll` range (~3.4 rad) at
`T = 2.0 s`, implying 2.55 rad/s. 2.6 sits just above every legitimate move and
far below the 5.350 rad/s a one-shot direct command produces.

## 4. Master vs v2, on the same instrument

**The brief for this document assumed master's (b)-(e) could not be scored
retroactively. They can, and were** — `scripts/v2_criteria_monitor.py` is a
read-only observer that wraps `env.step`, and `skills_scripted.py` is unmodified
on this branch, so master's own skills were run through it directly. Both
columns below are measured. Full detail in `v2-master-baseline.md`.

| skill | stack | (a) | (b) props | (c) arm-prop | (d) cross-arm | (e) peak vel |
|---|---|---|---|---|---|---|
| `pick(A, fork)` | master | OK | OK | OK | OK | **6.937** FAIL |
| | v2 | OK | OK | OK | OK | **1.832** OK |
| `place(A, fork, ·)` | master | OK | **mug 63.2 mm** FAIL | **-4.5 mm** FAIL | OK | **6.937** FAIL |
| | v2 | OK | OK | OK | OK | **0.143** OK |
| `handoff(A→B, fork)` | master | OK | **mug 48.4 mm** FAIL | **-4.1 mm** FAIL | **3179 steps, -10.9 mm** FAIL | **6.937** FAIL |
| | v2 | **FAIL ph3** | mug 25.1 mm | -3.7 mm | **OK (0)** | **1.832** OK |

**Master: 0 of 3 pass. v2: 2 of 3 pass.**

Two figures deserve to be read slowly. Master's handoff spends **3179 of its
6610 steps — 48% — in cross-arm contact**, 1.1 cm deep at worst, and reports
success. And master's peak joint velocity of **6.937 rad/s** is higher than the
5.350 rad/s an isolated one-shot direct position command produces; that is the
quantified mechanism behind the knocked mug and the fallen bottle.

v2's handoff still fails, but note **which** criterion it now satisfies: (d),
cross-arm contact, went from 3179 steps to **zero**.

## 5. The bimanual relay

`scripts/v2_stage5_demo.py`, seed 0, **6 of 6 stages pass all five criteria**,
24.2 s simulated:

| stage | steps | peak vel | verdict |
|---|---|---|---|
| 1 `pick(A, fork)` | 2507 | 1.832 | PASS |
| 2 `place(A, fork, drop1)` | 2300 | 0.151 | PASS |
| 3 arm A returns home | 1500 | 1.831 | PASS |
| 4 `pick(B, fork)` | 2509 | 1.664 | PASS |
| 5 `place(B, fork, drop2)` | 2300 | 2.362 | PASS |
| 6 arm B returns home | 1000 | 1.581 | PASS |

The fork moves from arm A to arm B **via the table**, not hand to hand.

Three sequencing rules were found by measurement. They are **preconditions on
skill composition, not fixes to the skills** — each skill is unchanged; the
caller must satisfy these:

1. **The idle arm must be at home before the other arm approaches shared
   space.** Skipping arm A's return produced 240 cross-arm contact-steps.
2. **Returns must route via a raised waypoint.** A direct joint-space sweep
   home from a low pose dragged the mug **28.2 cm**.
3. **A place target must keep the jaw-OPENING swept volume clear of
   neighbouring props.** The moving finger pad swings out in the release phase
   and catches anything close: `(-0.05, 0.05)` catches the plate rim (plate
   displaced 16.7 mm), `(0.02,-0.05)` and `(-0.06,-0.04)` catch the mug,
   `(-0.12,-0.03)` catches the plate at -8.05 mm. `(0.00, 0.05)` and
   `(-0.10,-0.04)` are clean.

Rule 3 generalises the single most common failure in this branch: **endpoints
being clear says nothing about the volume swept getting there or letting go.**

## 6. What v2 does not do

**Direct hand-to-hand handoff.** The transit collision was solved —
`HANDOFF_YAW = 3*pi/2` took cross-arm contact from 129 steps (worst -4.7 mm) to
zero, and phase 2 converges. Phase 3 still fails: arm B's weld never attaches.
The 6 cm lateral grip offset the fork's length permits leaves the fork's body
origin **0.1079 m** from arm B's pinch against a **~2 cm jaw span**.

Widening the weld's distance gate to 0.115 m *does* make the handoff report
success. It was tested and **refused**: a per-step contact check proved **arm
B's finger pads never touch any fork geom at any point in the run**. That is a
weld across a 10.8 cm gap with no contact — a teleport dressed as a transfer,
and shipping it would have been a false claim.

Named fix, recorded and not attempted here: derive arm B's target from the
**fork's actual pose** (a point on its surface offset along its axis, plus the
pad clearance) rather than an abstract world-frame `P_to`, and gate on
**surface distance** rather than body-origin distance, which is a poor proxy on
a 0.14 m object.

**The water bottle.** Unreachable by both arms at every height under top-down
IK: 0.271 m of horizontal reach required against ~0.249 m available. Master
reached it 9/20 with position-only IK. This is a **direct consequence of
choosing orientation control over reach** — constraining the approach to
vertical spends the arm's redundancy that position-only IK was free to use.
It is a real capability regression, disclosed rather than dropped.

## 7. Instrument-quality notes

Four things that would have produced false results if unexamined.

**The ADR-025 pinch point is not this gripper's grasp centre.** Measured in
genuine top-down poses, constant across the workspace to 5 dp: the static
finger pad sits **-0.08298 m** below the pinch point, the moving pad
**-0.02015 m**. Master never hit this because its position-only poses are not
top-down — there its pads sit only ~0.013 m below the pinch. Two constraints
then nearly conflict: the static pad must clear the table
(pinch >= 0.43298) while the object must lie *between* the pads
(pinch <= 0.43898). The feasible clearance window is **4 mm wide**.

**A grasp that was not a grasp.** The first Stage 4 implementation used a
clearance 2 mm past that window, putting the fork outside the jaw span, then
widened the weld gate from 0.05 to 0.12 m so it would attach anyway — 8.6 cm
from the pinch. `pick` satisfied criterion (a) by dragging a fork welded below
the gripper, and **was reported as passing all five criteria. That result was
retracted.** The gate is now 0.09 against an actual 0.0854, so it is a real
check again.

**Two errors of my own, recorded rather than quietly corrected.** (i) I first
called the 0.083 m pad offset a phantom, having measured master's
non-top-down pose; the original measurement was right and my refutation was
wrong. (ii) A 21-sample sweep of the handoff transit reported it clean; it
stepped `s` by 0.05 and straddled a collision window at s≈0.87. A 101-sample
sweep found **-0.0269 m**. That false negative is exactly the sampling error
that lets a collision reach a demo video.

**Two harness bugs caught by inspection.** The relay harness printed "6/6"
while its counter was only reading criterion (a); the true score at that moment
was 5/6. And the render stride: the brief specified "every 6th step at 30 fps
for real-time speed", but `timestep = 0.002 s` makes stride 6 equal to
**0.360x** — 2.8x slow motion. Real time needs **stride 17**. Shipping stride 6
would have misrepresented how fast the arms actually move, which is the exact
quantity this redesign is about.

**Framing was validated by measurement, not by eye.** An 8-frame inspection of
the first take found the fork invisible at both release moments. Re-angled from
azimuth 130/-22 to 210/-35. Then an automatic per-frame check — counting
arm-coloured pixels on the frame border across all 712 frames — found the arm
still cropped at distance 1.00 and again at 1.12, at frames ~20-23 that no
8-frame sample would have caught. Minimum crop-free distance measured at
**1.30**.

## 8. Verdict

**Master** ships four skills that report success on target-object motion. Zero
of the three measured pass the scene-integrity criteria. Its handoff is
genuinely bimanual and genuinely completes — and spends half its runtime with
the two arms inside each other, ending with the mug knocked over and the bottle
on the floor.

**v2** ships two individual skills that pass all five criteria, and a six-stage
bimanual relay in which both arms work in sequence, the object changes hands
via the table, and **nothing else on the table moves**. Its direct handoff is
diagnosed, unsolved, and has a named fix. It cannot reach the water bottle at
all.

Neither is strictly better. Master does something v2 cannot (a direct
hand-to-hand transfer, and water-bottle reach); v2 does it without destroying
the scene, and knows — with numbers — what it is doing.

**Recommendation: keep `master` as the submission's functional baseline and
present v2 as the measured rebuild.** Master is tested, complete, and already
satisfies the deliverables. v2's value to a submission is not that it replaces
those four skills — it does not — but that it is a documented case of finding
hidden failure modes by building a better instrument, and then refusing the two
shortcuts (a widened grasp gate, a widened weld gate) that would have turned
failures into passing numbers. That is the Innovation and Rigour story, and it
is stronger told alongside master than in place of it.

The concrete form of that recommendation is the subject of
`docs/hardware/submission-path.md`.

## 9. What a continuation would need

1. **The direct handoff**, per the named fix in §6 and ADR-073: target from the
   object's actual pose, gate on surface distance.
2. **Reach recovery.** The water bottle is outside the top-down workspace by
   ~22 mm of horizontal reach. Recovering it needs a longer link or a
   repositioned base — both scene/hardware changes, outside v2's scope, which
   was deliberately the motion stack alone.
3. **A swept-volume planner.** Every hard failure on this branch reduces to the
   same thing: joint-space interpolation has no notion of the Cartesian path it
   traces. `_split_move`'s 4 linear sub-legs is a mitigation, not a solution.
