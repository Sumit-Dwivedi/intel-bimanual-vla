# Redesign branch — verdict

**Branch:** `redesign` · **Status:** archived, not merged · **Outcome:** `master`
ships unchanged at `20e1012`.

This document closes the `redesign` branch. It is written to stand alone: a
reader who has seen none of the rest of this project should be able to judge
the hypothesis, the method, and whether the conclusion follows from the
evidence.

All numbers were measured on bm-ptl (ADR-047 records a cross-machine
floating-point divergence; the laptop reports 0.3987 where bm-ptl reports
0.3989, so bm-ptl is authoritative throughout).

---

## 1. Hypothesis

The Day 1 scene geometry — 0.5 m arm-base separation (ADR-021) and prop
placement (ADR-026) — was chosen **before anyone measured what the arms could
reach**. Four skill failures persisted across the project:
`place(A, water_bottle, table)`, `pick(A, mug)`, `handoff(B→A, fork)`, and
`open_drawer`.

**Hypothesis:** those failures were downstream of that placement, not of the
skill logic. If geometry were placed inside an empirically measured reachable
volume, they would resolve.

The hypothesis was worth testing because ADR-057 had already ruled out the
obvious alternative. Multi-seed IK (32 random restarts from ±0.15 rad,
ADR-056) found four separate failing targets converging to the **identical**
residual from every restart — genuine kinematic boundaries, not solver local
minima. If the solver was not at fault, the geometry was the next suspect.

## 2. Method

Five stages, inverting the original order: **measure first, place second.**

| Stage | ADR | Work |
|---|---|---|
| 1 | ADR-058 | Empirical workspace sampling per arm (N=50000 forward-kinematics samples, with an N=300000 densification cross-check), intersection analysis at 13 heights from 0.35 to 0.59 m |
| 2 | ADR-059 | Base-separation sweep at N=300000, geometry re-placement, home keyframe regeneration |
| 3 | ADR-060 | Eight-skill retest, multi-seed diagnostic per failure |
| 4 | ADR-061 | Swept-path collision gate, collision diagnosis, remedy attempt |
| 5 | ADR-062 | This verdict |

Throughout, `skills_scripted.py` changed by exactly **one line** (the
`HANDOFF_POSITION_XYZ` constant, moved to the measured feasible centroid in
Stage 3 — geometry, not logic). `grasp.py`, `ik.py`, `executor.py` and `env.py`
were never modified. The redesign was geometry-only by construction, so any
skill that failed did so against unchanged logic.

## 3. What the hypothesis got right

**Two of master's three kinematic walls dissolved.** These were not marginal
improvements; they were qualitative changes in the failure mode.

| skill | on master | at 0.40 m separation |
|---|---|---|
| `pick(A, mug)` | hard boundary, residual plateaus at 0.0532 m under 32 restarts | **residuals no longer plateau** — reachability genuinely fixed; now blocked downstream at the grip mechanism |
| `handoff(B→A, fork)` | hard boundary, 0.0954 m (ADR-037, reconfirmed ADR-057 Case 1) | **bimodal** — multiple viable configurations exist; blocked by a 0.3 mm near-miss collision |

For those two skills the Day 1 placement **was** the upstream cause. That is a
real result: two failures that had been characterised as immovable kinematic
limits were shown to be artifacts of an arbitrary base separation.

Stage 1 also answered the structural question directly: at the original 0.5 m
separation there is **no contiguous both-arms-reachable region of 10 × 10 cm at
any height** between 0.35 and 0.59 m. The largest is 4 cm × 32 cm — thin in y,
the axis the bases are separated along. That single measurement explains why
the handoff corridor had been so fragile for so long (ADR-032, ADR-035,
ADR-048 all measured near-zero-width feasible bands without knowing why).

## 4. What the hypothesis got wrong

**One wall is genuinely irreducible.** `place(A, water_bottle, table)`'s
destination approach plateaus at ~0.0150 m regardless of base separation —
against a 0.010 m tolerance. Moving the geometry did not move it.

**And the net functional result is worse than master:**

| | master | redesign |
|---|---|---|
| skills passing at seed 0 | **4 of 9** | **1 of 8** |

Three previously-working skills regressed — `pick(A, fork)`,
`place(A, fork, table)`, `handoff(A→B, fork)` — all on waypoint-1 approach
collisions. `open_drawer` was removed in Stage 2 on measured evidence (raising
the drawer to tabletop height produced 13 contacts deeper than 1 mm against
plate, fork and spoon), which is why the denominator changed from 9 to 8.

## 5. The gate gap, and why it mattered

Stage 2's verification gate had five items: the model compiles; the home
keyframe is contact-free; every prop's grasp point solves IK from home; the
handoff point solves for both arms from their own approach poses; and both
arms' handoff configurations are simultaneously collision-free.

All five passed. **None of them checked the swept approach path.**

Geometry can be reachable at every endpoint and still collide on the way in.
Stage 4 built that missing check and measured the cost of its absence: **31 of
41 waypoints collide** across the eight skills' approach sequences.

- **Category (b), arm-vs-table** dominates by count — 18 waypoints, mostly
  shallow grazes around 6 mm.
- **Category (a), arm-vs-prop** holds the single worst violation: the water
  bottle at **−0.098 m**.

This is the methodological lesson of the branch, and it generalises beyond
this project: *a placement gate must test the trajectory, not the endpoints.*

## 6. The finding that closes the approach

Stage 4 attempted the obvious remedy — re-run the home-pose search with
swept-path clearance added to the scoring — and it produced a result that
ends the line of attack.

**The optimised pose improved the gate score from 5/7 to 3/7 violations and
regressed real functional success from 1/8 to 0/8.** Every skill failed at
GRIP.

The cause is precise: **the swept-path gate has no visibility into grasp
orientation**, because `ik.solve_position_ik` is position-only by design
(ADR-024 — the SO-101 has 5 pose-controlling joints, which cannot in general
satisfy a full 6-DoF pose task). A configuration can therefore be
collision-optimal and grasp-useless at the same time, and the gate cannot tell
the difference.

Generalised: **optimising geometry against a position-only metric can move you
away from working grasps.** No amount of further placement search fixes that
while orientation is uncontrolled — the search is hill-climbing on a metric
that is blind to the thing that actually has to work. This is why the redesign
cannot be rescued by more iteration *at this scope*, and it is a stronger
reason to stop than simply running out of budget.

The remedy was reverted rather than kept for its better gate score.

## 7. Instrument quality

The measurements are only as good as the instruments, and two of them needed
correction before they could be trusted.

**The swept-path gate required two evidence-driven refinements to pass
validation against known-good behaviour.** It was held to a two-sided test: it
had to report *clear* on master's working `pick(A, fork)` and *collision* on
redesign's failing one. Reaching that took:
- **Multi-seed endpoint sampling.** The redundant 5-DoF-to-3-DoF solve has an
  un-regularised null space, and it genuinely drifts to different solution
  branches — some of which collide. Measured directly, not assumed.
- **An implausible-penetration bound.** Raw MuJoCo contacts reproduce the
  convex-hull mesh artifact documented in ADR-035 and ADR-058. This was its
  **third independent appearance** in the project.

**Stage 1's `table_top` penetration filter was rejected as unreliable.** It
falsely flagged `pick(A, fork)`'s own working grasp target — a target the
shipped controller picks successfully in every regression run — at **−0.39 m**
penetration. The raw cloud was declared authoritative instead. Had the filter
been trusted, every downstream stage would have inherited an artificially
carved-away workspace and the entire branch would have been built on it.

**Stage 3's home-pose validation caught a bottom-quartile choice.** Stage 2
had adopted the first pose that passed, on a sample of two candidates.
Re-scoring 200 candidates found 117 passing (58.5%, so the criteria were not
near-degenerate) and ranked Stage 2's pose **91st of 117**. It was swapped for
the best-scoring candidate per rule, and Stage 2's gate was re-run against the
replacement.

Each of those three would have silently biased the result had it gone
unchecked.

## 8. Verdict

**`master` ships. `redesign` is archived as a measured design alternative.**

- Hypothesis **partially confirmed**: two of three kinematic walls were real
  consequences of unmeasured Day 1 placement, and moving the bases dissolved
  them.
- Hypothesis **rejected on net functional evidence**: 1 of 8 passing versus
  master's 4 of 9, with three regressions traced to one unchecked condition.
- The approach is **closed at this scope for a specific, named reason**
  (§6), not abandoned for lack of time.

The branch is preserved, not merged and not deleted. It is the evidence.

## 9. What a continuation would require

An **orientation-aware IK layer** — either a full 6-DoF task specification or
a constrained position-plus-approach-axis mode. The SO-101's 5 pose-controlling
joints cannot generally satisfy a 6-DoF task (ADR-016, ADR-024), so this would
need task-mode constraints in the solver (specifying position plus one axis,
leaving roll free) or different hardware.

With orientation controlled, the swept-path gate would become a usable
optimisation target rather than a misleading one, and the placement search that
failed in §6 would be worth re-running.

That is a different project. Recorded here for whoever picks it up.
