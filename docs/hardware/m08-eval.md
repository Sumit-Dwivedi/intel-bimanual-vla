# M08 — formal 10-seed robustness evaluation of the four working skills (ADR-049)

**Recorded:** Sept 14, 2026 · HEAD at start: `42c9e22` (M07, ADR-048).
**Provenance.** Every number below ran on bm-ptl,
`C:\Users\devcloud\project\ov_env\Scripts\python.exe` (`mujoco==3.2.7`).
ADR-047 recorded a pre-existing, last-float-digit divergence between the
laptop and bm-ptl; this document's numbers are exclusively bm-ptl's. The
regression gates below were re-verified on bm-ptl in this same session,
immediately before the eval run: `scripts/verify_adr038_skills.py` reproduced
`0.3989 / 0.3588 / 0.6192 / 0.1946`, `pytest tests/test_skills.py` reproduced
`4 passed / 4 failed` (same four tests: `test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`).

**Corrected ADR number: 049, not 050** — highest existing before this pass
was ADR-048 (`DECISIONS.md`, 33 entries before this one).

---

## Why two tracks, not one

The obvious way to read this module's brief — "randomize the skill's own
target prop; other props stay at default" — cannot reproduce M07's Round 1
numbers (`docs/hardware/m07-envelopes.md`'s Task 3), and reporting a single
table as if it had reproduced them would misrepresent what was measured.
Concretely:

- M07's Round 1 randomized **every prop with a non-empty individual
  envelope**, which turned out to be `water_bottle` alone (`fork`'s
  cross-skill intersection was already degenerate independent of that
  finding) — applied underneath **every** skill's ten seeds, including
  `handoff`, which never touches `water_bottle`. That cross-prop exposure is
  exactly what produced `handoff`'s reported 0/10.
- This brief's own method, applied literally, randomizes only the skill's
  own target. `handoff`'s own measured fork envelope
  (`docs/hardware/m07-envelopes.md:163`) is `dx in [0,0], dy in [0,0]` — a
  single point. Under this method `handoff` draws the identical (0, 0)
  offset on every seed, so all ten trials are the same deterministic
  scenario, and it scores 10/10 by construction. Reporting that 10/10
  without qualification next to the other three skills' real ranges would
  read as "handoff is the most robust skill measured," which is the
  opposite of what M07 already established about it.

So this document reports **both**, labelled, side by side:

- **Track A — own-prop randomization** (this brief's method). Only the
  skill's own target prop moves, drawn from that skill's own measured
  envelope (`randomization.py`'s `SKILL_ENVELOPES[skill][target]`, not the
  cross-skill-intersected, shipped `ENVELOPES`, which stays empty and
  unmodified — see "Constraints honoured" below).
- **Track B — multi-prop randomization** (M07 Round 1's method,
  `docs/hardware/m07-envelopes.md`'s Task 3). The same fixed
  `water_bottle: (0, 0, -0.010, +0.010)` randomizer runs underneath every
  skill's ten seeds, regardless of what that skill targets.

Neither track is "the" robustness number. Track A tells you how much margin
a skill's own placement tolerance has. Track B tells you how exposed a
skill is to a prop it never touches, through pure physics coupling. A
reader who only sees Track A's `handoff` row and does not see Track B would
conclude the opposite of what M07 already found.

---

## Constraints honoured

- `src/bimanual/sim/randomization.py`, `env.py`, `executor.py`,
  `skills_scripted.py`, `ik.py`, `grasp.py`, `scenes/so101/`,
  `gen_dual_scene.py` — **not modified**. `randomization.py`'s shipped
  module-level `ENVELOPES` stays `{}` (the union-safe default). This
  evaluation constructs its own `ScenarioRandomizer(envelopes={...})`
  instances directly in `scripts/eval_m08.py`, using `envelopes=` — a
  documented constructor argument the class already supports — with
  rectangles read from `randomization.py`'s own `SKILL_ENVELOPES` table
  (never re-measured, never invented).
- Prop offsets are applied via runtime free-joint `qpos` writes followed by
  `mujoco.mj_forward` — the same mechanism `env.py`'s `reset(randomizer=...)`
  already uses internally (ADR-038: regenerating the scene XML breaks
  `handoff`). Never a scene-generation change.
- Fresh `TableSettingEnv` + fresh `ScriptedSkillExecutor` per trial (ADR-047:
  reusing either across a `reset()`-based loop silently corrupts
  `WeldGrasp.active_welds`).
- Results written one JSON line at a time, flushed immediately
  (`scripts/eval_m08.py`'s `_write_jsonl`) — a mid-run failure loses at most
  the one in-flight trial. All 80 trials (2 tracks x 4 skills x 10 seeds)
  in fact completed without any failure of the harness itself; total
  wall-clock on bm-ptl was **~8 minutes** (09:27-09:35, this session's log
  timestamps).

---

## Per-skill envelopes used (own-prop, Track A)

Transcribed from `randomization.py`'s `SKILL_ENVELOPES`
(`docs/hardware/m07-envelopes.md` is the measurement this table is read off
of — never re-derived here):

| skill | target prop | dx range | dy range | shape |
|---|---|---|---|---|
| `pick(A, fork)` | `fork` | [-10, +20] mm | 0 | real 1-D range |
| `place(A, fork, table)` | `fork` | [-10, 0] mm | 0 | real 1-D range |
| `handoff(B, A, fork)` | `fork` | 0 | 0 | **single point — degenerate** |
| `pick(A, 'bottle')` | `water_bottle` | 0 | [-10, +10] mm | real 1-D range |

`handoff`'s row is a single point because its own fine-grid sweep
(`docs/hardware/m07-envelopes.md`'s Task 1) found exactly one passing cell
out of 29 measured — the unperturbed default itself. This is not a gap in
measurement; it was re-swept in full after an early-stop attempt nearly
mis-reported it as empty (see that document's own account).

---

## Track A — own-prop randomization, seeds 0-9, oracle mode

### `pick(A, fork)` — 10/10, mean frames on success 1655.0

| seed | dx (mm) | dy (mm) | result | frames |
|---:|---:|---:|---|---:|
| 0 | +9.11 | 0 | PASS | 1655 |
| 1 | +5.35 | 0 | PASS | 1655 |
| 2 | -2.15 | 0 | PASS | 1655 |
| 3 | -7.43 | 0 | PASS | 1655 |
| 4 | +18.29 | 0 | PASS | 1655 |
| 5 | +14.15 | 0 | PASS | 1655 |
| 6 | +6.14 | 0 | PASS | 1655 |
| 7 | +8.75 | 0 | PASS | 1655 |
| 8 | -0.19 | 0 | PASS | 1655 |
| 9 | +16.11 | 0 | PASS | 1655 |

No failures. Real variation carried on every seed (dx spans nearly the full
30 mm envelope width); the skill's own tuned tolerance absorbs all of it.

### `place(A, fork, table)` — 10/10, mean frames on success 3455.0

| seed | dx (mm) | dy (mm) | result | frames |
|---:|---:|---:|---|---:|
| 0 | -3.63 | 0 | PASS | 3455 |
| 1 | -4.88 | 0 | PASS | 3455 |
| 2 | -7.38 | 0 | PASS | 3455 |
| 3 | -9.14 | 0 | PASS | 3455 |
| 4 | -0.57 | 0 | PASS | 3455 |
| 5 | -1.95 | 0 | PASS | 3455 |
| 6 | -4.62 | 0 | PASS | 3455 |
| 7 | -3.75 | 0 | PASS | 3455 |
| 8 | -6.73 | 0 | PASS | 3455 |
| 9 | -1.30 | 0 | PASS | 3455 |

No failures. Same real-variation observation as `pick_fork`, over its
narrower (10 mm wide, one-sided) envelope.

### `handoff(B, A, fork)` — **10/10 — envelope is a single point, zero displacement applied; this measures determinism, not robustness.**

| seed | dx (mm) | dy (mm) | result | frames |
|---:|---:|---:|---|---:|
| 0-9 | 0 | 0 | PASS (all identical) | 6610 |

Every one of the ten trials is the byte-identical unperturbed baseline
scenario (`from_arm_retreat_dist=0.2263`, `dist_to_armB=0.0582`, matching
`scripts/verify_adr038_skills.py`'s own numbers exactly). **This row must
never be read as "handoff is 100% robust."** It shows that ten repeated
runs of the exact same deterministic scenario succeed ten times — a
regression check, not a robustness measurement. See Track B below for what
happens when a prop `handoff` does not even touch is allowed to move.

### `pick(A, 'bottle')` — 6/10, mean frames on success 1672.17

| seed | dx (mm) | dy (mm) | result | frames |
|---:|---:|---:|---|---:|
| 0 | 0 | -4.60 | FAIL (`weld_attach_failed_after_300_frames`) | 1300 |
| 1 | 0 | +9.01 | FAIL (`weld_attach_failed_after_300_frames`) | 1300 |
| 2 | 0 | -4.03 | FAIL (`weld_attach_failed_after_300_frames`) | 1300 |
| 3 | 0 | -5.26 | PASS | 1655 |
| 4 | 0 | +0.23 | FAIL (`weld_attach_failed_after_300_frames`) | 1300 |
| 5 | 0 | +6.16 | PASS | 1655 |
| 6 | 0 | -3.13 | PASS | 1655 |
| 7 | 0 | +7.94 | PASS | 1655 |
| 8 | 0 | +9.75 | PASS | 1695 |
| 9 | 0 | -4.26 | PASS | 1718 |

Failure mode: `weld_attach_failed_after_300_frames` x4 (seeds 0, 1, 2, 4).
The pass/fail pattern is **not monotonic in `dy`** (seed 3 at `dy=-5.26 mm`
passes; seed 2 at `dy=-4.03 mm` fails; seed 6 at `dy=-3.13 mm` passes) —
exactly the "real sub-cm structure the 1 cm grid does not resolve" caveat
`docs/hardware/m07-envelopes.md`'s Task 3 Round 1 already flagged for this
skill. This is a real, reproducible tolerance edge inside the chosen
rectangle, not eval noise (re-running these exact seeds reproduces these
exact offsets and outcomes, since `ScenarioRandomizer` is a pure function
of `seed`).

---

## Track B — multi-prop randomization (M07 Round 1's method), seeds 0-9, oracle mode

Fixed randomizer for **every** skill: `water_bottle: dx=0, dy in [-10,+10] mm`
— identical config, identical per-seed draws, run underneath all four
skills regardless of what each one targets.

### `pick(A, fork)` — 10/10, mean frames on success 1655.0

`water_bottle` moves; `fork` (this skill's own target) never does. All ten
pass, frames bit-for-bit identical to the unperturbed baseline (1655) on
every seed — `pick_fork` is fully insulated from `water_bottle`'s position.

### `place(A, fork, table)` — 8/10, mean frames on success 3455.0

| seed | water_bottle dy (mm) | result |
|---:|---:|---|
| 0 | -4.60 | PASS |
| 1 | +9.01 | PASS |
| 2 | -4.03 | PASS |
| 3 | -5.26 | PASS |
| 4 | +0.23 | PASS |
| 5 | +6.16 | **FAIL** — `waypoint 1 (approach destination) failed [collision (arm-vs-prop: mug, dist=-0.0060 m)]` |
| 6 | -3.13 | PASS |
| 7 | +7.94 | PASS |
| 8 | +9.75 | **FAIL** — `waypoint 1 (approach destination) failed [collision (arm-vs-prop: mug, dist=-0.0066 m)]` |
| 9 | -4.26 | PASS |

**`fork` itself never moves in this track (its own envelope is degenerate,
excluded).** The 2 failures (seeds 5, 8) are `place_fork`'s own nested pick
landing the fork at a slightly different final grip pose purely because a
*far-away, unrelated* prop's position perturbed floating-point rounding
through the whole ~3455-step closed-loop rollout — this is the exact
"chaos finding" `docs/hardware/m07-envelopes.md`'s Task 3 Round 1 reported,
reproduced here on the identical seeds (5, 8) in a fresh run.

### `handoff(B, A, fork)` — 0/10

| seed | water_bottle dy (mm) | result | frames |
|---:|---:|---|---:|
| 0-9 | (varies, -4.60 to +9.75) | **FAIL, every seed** | 3155 |

Every seed fails with the byte-identical reason: `phase 3 (to_arm approach)
failed [direct approach failed [convergence (IK residual=0.0875 m >= 0.01 m)];
staging to y=-0.06 also failed [collision (cross_arm contacts=1 vs baseline 0;
armB-vs-table_top contacts=0 vs baseline 0)]]`, at the identical frame count
(3155) regardless of `water_bottle`'s actual offset. `handoff` never
targets `water_bottle` and `fork` itself never moves in this track either —
this is a pure cross-prop physics-coupling failure, not a targeting error.
See "Cross-prop coupling" below.

### `pick(A, 'bottle')` — 6/10, mean frames on success 1672.17

Identical seeds, identical offsets, identical outcomes to Track A's
`pick_bottle` row above (seeds 0, 1, 2, 4 fail). This is expected, not a
bug: `water_bottle` is `pick_bottle`'s own target, so Track A and Track B
use the exact same rectangle and the exact same `np.random.default_rng(seed)`
draw order for this one skill — there is nothing for the two tracks to
disagree about here.

---

## Cross-prop coupling: why the intersection collapses to empty

**A skill can be broken by a prop it never manipulates.** Two independent
pieces of evidence, from two different eras of this repo, say the same
thing:

1. **ADR-038** (regenerated scene, Sept 14): moving the scene's props toward
   the table corners — to declutter a render — broke `handoff` at phase 3
   (`to_arm` approach) for **every** configuration tried, including moving
   the mug alone, at a far corner, while every other prop (including the
   fork `handoff` actually carries) stayed exactly where it was. The
   mechanism was not fully diagnosed and ADR-038 says so plainly: "the
   mechanism is not understood and was not chased; it is recorded here so
   nobody assumes prop placement is cosmetically free."
2. **M07's Commit 2** (`docs/hardware/m07-envelopes.md`'s Task 3) found the
   same shape of failure at far finer resolution: `handoff` fails 0/10 when
   only `water_bottle` is randomized, and the cliff was probed down to
   `dy = ±0.1 mm` — **every nonzero offset, however small, reproduces the
   identical phase-3 failure**; only exactly `dy = 0.0` reproduces the true
   baseline. This is not a smoothly shrinkable tolerance band; it is a
   binary cliff. This document's own Track B run reproduces it a third
   time, fresh, on bm-ptl, today: 0/10, same failure reason, same frame
   count, independent of which of the ten (varying) `water_bottle` offsets
   was actually applied.

**Why this happens, in plain terms.** MuJoCo recomputes the *entire*
system's contacts and forces at every single physics step — nothing about
one prop's position is computed in isolation from the rest of the scene.
`handoff`'s cross-arm transfer corridor is already a "knife-edge" collision
check (ADR-037's own term): arm B has to stage very close to arm A to
receive the fork without the two arms' own geometry colliding. A change
anywhere else in the scene shifts floating-point rounding throughout the
whole multi-thousand-step rollout leading up to that moment, and this
particular corridor is narrow enough that even a rounding-level nudge is
enough to tip a marginal collision check the wrong way. `place_fork`'s two
Track-B failures above are the same mechanism at a smaller scale: not a
hard cliff, just enough drift to occasionally clip the mug during a
different, but also fairly tight, approach waypoint.

**Why the intersection collapses to empty.** A single, skill-agnostic
randomizer that runs at `reset()` cannot know which skill will execute
next. If it is to be safe for `handoff` at all, it must offer `handoff` a
range `handoff` can tolerate for *every* prop in the scene, not just the
prop `handoff` itself targets — and the measured range for `water_bottle`
under that constraint is `(0, 0)`. `randomization.py`'s shipped, empty
`ENVELOPES` is the direct, correct consequence of taking that constraint
seriously (ADR-048) — it is not a bug to be fixed later, it is what the
evidence says today.

**Track A and Track B measure different things, and that is the whole
point of running both.** Track A asks "how much can a skill's *own* target
move." Track B asks "how much can *anything else in the scene* move before
this skill breaks." `handoff` scores 10/10 on the first question and 0/10
on the second, and both numbers are true and necessary — a reader who saw
only Track A's `handoff` row would have no way to know Track B's answer
exists at all.

---

## Interpretation — an honest robustness measurement, not a robustness claim

Every PASS number in this document was measured **inside envelopes on the
order of ±10-20 mm**, tuned specifically to what these four skills already
tolerate (`docs/hardware/m07-envelopes.md`'s fine-grid sweep). The
pre-M07/M08 audit (`docs/hardware/m10-pre-m07-audit.md`) measured **0/40**
aggregate across the same four skills at a coarse **±50 mm** jitter — no
skill in this repo, at this stage, tolerates real-world-scale placement
error. Track A's high pass rates say "within a narrow, specifically-tuned
band, these three skills have real margin, and `handoff` has none." They do
not say "these skills are robust to placement error" in any general sense,
and this document is not making that claim. Track B's numbers, especially
`handoff`'s 0/10, are the more informative figures for anyone deciding how
much confidence to place in the shipped controller under conditions it was
not specifically tuned for.

Read together with the failure-mode breakdowns above, the honest summary
is: **the scripted controller works precisely within the envelope it was
measured against, has a real single point of total fragility
(`handoff`'s cross-arm corridor), and that fragility is exposed by props
the skill never touches, not only by its own target.**

---

## Demo-friendly seeds (Track A)

Pre-selecting known-good seeds for a recorded demo is normal, disclosed
practice, not cherry-picking presented as a general robustness claim — the
distinction is that this document also publishes the failing seeds and the
reasons above, in the same place, rather than only the passing ones.

| skill | passing seeds (Track A) | count |
|---|---|---|
| `pick(A, fork)` | 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 | 10/10 |
| `place(A, fork, table)` | 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 | 10/10 |
| `handoff(B, A, fork)` | 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 (identical scenario every time — see the qualifier above) | 10/10 |
| `pick(A, 'bottle')` | 3, 5, 6, 7, 8, 9 | 6/10 |

For a demo sequence that chains all four skills (as the brief's example
command does), **seeds 3, 5, 6, 7, 8, 9** are the safe intersection under
Track A. Seed 5 and seed 8 are additionally the two seeds Track B found to
break `place_fork` via the `water_bottle` chaos coupling — if the demo ever
runs with multi-prop randomization enabled instead of the fixed default
scene, those two should be avoided for `place_fork` specifically.

---

## Reproduction

```
python scripts/eval_m08.py --track both --skill all --out-dir out/m08_eval
```

Regression gates, re-checked in this session before and after the eval run:

```
[1] pick(A, fork)         final z = 0.3989
[2] place(A, fork, table) final z = 0.3588
[3] pick(A, 'bottle')     final z = 0.6192
[4] handoff(A -> B, fork) lateral sep = 0.1946 m
```

`pytest tests/test_skills.py -q`: **4 passed, 4 failed** (same four failing
tests as ADR-047/ADR-048, same failure reasons and residuals).
