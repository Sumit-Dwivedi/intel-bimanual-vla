# M11 — Multi-seed IK diagnostic: which failing targets are local minima, which are genuine boundaries (ADR-057)

**Diagnostic only. No skill or solver changes.** This is Commit 2 of a
sequential, regression-gated series; Commit 1 (ADR-056) added the
`num_seeds` random-restart capability to `solve_position_ik` and measured
ONE target (`place(A, water_bottle, table)`'s destination approach) with
it, finding no improvement at `num_seeds=32` over `num_seeds=1` there —
evidence for a genuine reachability boundary, not proof. This commit
extends that measurement to four more targets and, critically, measures
the FULL distribution of residuals across all 32 restart seeds (not just
the best one `solve_position_ik` itself returns), because "the winner
didn't improve" and "every seed converged to the same answer" are
different claims — only the second one rules out a local minimum.

**Provenance.** All numbers in this document are from **bm-ptl**
(ADR-047: laptop and bm-ptl diverge in the last 1-3 decimal digits on
floating-point results; bm-ptl is the reference machine for any number
quoted here).

## Method

**Why a new script instead of calling `solve_position_ik(..., num_seeds=32)`
directly.** `solve_position_ik` returns only the best of its internal
restarts — there is no way to recover what the other 31 seeds found from
its return value alone. `scripts/probe_multiseed_diagnostic.py`
reconstructs each perturbed starting configuration ITSELF (a scratch
`mujoco.MjData` with the target arm's 5 positioning joints overridden) and
calls `ik.solve_position_ik(..., num_seeds=1)` from each one individually —
so all 32 residuals are produced by `ik.py`'s own unmodified DLS loop,
invoked 32 times from outside instead of once from inside. `ik.py` itself
is not imported for anything but its public API; it is not modified,
monkeypatched, or forked.

**The perturbation is mirrored exactly, not approximated.** The RNG seed
derivation (`(ord(arm)*1_000_003 + micron_x*7 + micron_y*13 +
micron_z*17) % 2**32`, `ik.py`'s own formula) and the draw sequence (one
`rng.uniform(-seed_noise_rad, +seed_noise_rad, size=5)` call per seed
index, from a single `numpy.random.default_rng(seed)`, in order) are
reproduced verbatim in `perturbed_residuals()`. Because
`numpy.random.default_rng`'s stream is sequential and deterministic, the
first 7 restart draws this script makes for a given `(arm, target)` pair
are IDENTICAL to the first 7 draws `ik.py` itself would make internally
for `num_seeds=8` on that same pair — so slicing one 32-long residual list
at `[:1]`, `[:8]`, `[:32]` reproduces exactly what `num_seeds=1`,
`num_seeds=8`, `num_seeds=32` would each have found, without ever calling
`solve_position_ik` with `num_seeds>1`.

**Sanity check on this claim, not merely asserted.** Target (a) below is
Commit 1's own known-failing target; ADR-056's own probe already measured
it at `num_seeds=1` and `num_seeds=32` directly through `ik.py`'s internal
path. This script's independent reimplementation of the same perturbation
is checked against those two already-published numbers.

**Starting state, per target.** Each target's base qpos is the state the
real skill's own failing waypoint actually starts from — home for a
target's first move, or a REAL `sk._run_waypoint`/`sk._run_pick`/
`sk._run_approach_with_staging` call's own terminal state for a
pose-history-dependent target (the chain, target (d)). Underscore-prefixed
helpers in `skills_scripted.py` are called (not modified) to reconstruct
these states exactly, matching the sequence `run_pick`/`run_handoff`
themselves execute.

## Interpretation rule (applied per target below)

1. **Variance ~= 0, one distinct value** -> every basin converges to the
   same solution. **Genuine boundary. NOT rescuable** by multi-seed.
2. **Variance > 0, multiple distinct values, best below threshold
   (0.010 m)** -> **local minimum. Rescuable by multi-seed.**
3. **Variance > 0, multiple distinct values, best still above threshold**
   -> **multi-modal but every mode is infeasible. Not rescuable by
   multi-seed** — but the spread is recorded, because retry-with-perturbed-
   TARGET (Commit 3's proposed mechanism) is a different lever than
   multi-seed's perturbed STARTING POINT, and may still help where
   multi-seed cannot.

## Results

All residuals below are in metres; `IK_POSITION_TOLERANCE_M = 0.010`.
"Variance" is the population variance (`numpy.var`, `ddof=0`) of the
residuals INCLUDED at that `num_seeds` count (e.g. the `num_seeds=8` row
covers seeds 0-7 only). "Distinct values" counts residuals after rounding
to 1e-5 m. Full 32-value list per target is in this script's own console
output (`scripts/probe_multiseed_diagnostic.py`, run via bm-ptl,
Sept 15 2026); not re-pasted here in full for length, condensed to the
required summary.

### (a) `place(A, water_bottle, table)` destination approach — sanity check against Commit 1

Target `[0.300, 0.00077, 0.430]` (x on the `+0.30` clip bound, as ADR-034
already found). Base qpos: arm A's qpos immediately after `pick(A,
water_bottle)` completes (matches `scripts/probe_ik_num_seeds32.py`
exactly).

| num_seeds | best residual | variance | distinct values | winning seed |
|---:|---:|---:|---:|---:|
| 1 | 0.01380 | 0.0 | 1 | 0 |
| 8 | 0.01380 | 7.08e-12 | 2 | 0 |
| 32 | 0.01380 | 5.82e-12 | 2 | 0 |

**Sanity check passed:** 0.0138 m at both `num_seeds=1` and `num_seeds=32`
reproduces ADR-056's own already-published figures exactly, confirming
this script's independent reimplementation of the perturbation agrees
with `ik.py`'s internal `num_seeds=32` path on the one target both have
now measured.

**Case 1 — genuine boundary, NOT rescuable by multi-seed.** The "2
distinct values" is a floating-point artifact, not a second attractor: the
32 raw residuals are either `0.0138000...` or `0.0138100...` — a ~1e-5 m
spread, i.e. the DLS solve's own convergence-tolerance noise floor (the
solver stops once `position_error_m < tol = 0.01`... no — it never
converges here at all in 60 iterations, so this is iteration-count/
floating-point rounding noise on an already-plateaued residual, not a
physically distinct solution). Variance (7e-12 m²) is 9 orders of
magnitude below anything a joint actuator could resolve. Every one of the
32 restart seeds — spanning `U(-0.15, +0.15)` rad per joint from the
post-pick qpos — lands in the SAME basin. Confirms Commit 1's own
"evidence against local-minimum" reading with variance data, as this
commit set out to do.

### (b) `pick(A, mug)` waypoint-1 approach

Target `[0.0975, -0.030, 0.470]`. Base qpos: HOME (`TableSettingEnv`
seed 0, no randomizer — `tests/test_skills.py`'s own `_make_env()`
convention, `SEED = 0`). **This target turns out to already be a
documented, currently-failing regression test**: it is exactly
`test_handoff_mug_ends_held_by_arm_b`'s Phase 1 nested `pick(A, mug)`
waypoint 1, which this batch's own regression gate (below) reproduces as
`IK residual=0.0532 m` — this script's `0.05324` m rounds to that same
figure, confirming this is the correct, currently-real failure, not a
constructed one.

| num_seeds | best residual | variance | distinct values | winning seed |
|---:|---:|---:|---:|---:|
| 1 | 0.05324 | 0.0 | 1 | 0 |
| 8 | 0.05324 | 1.17e-14 | 1 | 5 |
| 32 | 0.05324 | 1.08e-14 | 1 | 5 |

**Case 1 — genuine boundary, NOT rescuable by multi-seed.** Variance is
14 orders of magnitude below any physical scale; all 32 residuals round
to the identical value. (The "winning seed" moves from 0 to 5 despite an
unchanged residual — a pure floating-point tie-break at ~1e-9 m, not a
different solution; `np.argmin` picks the first minimal value it sees and
seed 5's raw float happens to sit fractionally below seed 0's.)

### (c) `handoff(B→A, fork)` Phase 1 pick approach

Target `[-0.065, 0.050, 0.440]` — `run_pick(env, "B", "fork", ...)`'s own
waypoint-1 hover, computed the same way `run_pick` computes it internally.
Base env: `TableSettingEnv`, seed 0, no randomizer (ADR-037's own VERIFY
convention and `scripts/chained_demo.py`'s fixed-layout convention).
Documented: **0.0954 m** (ADR-037 / `scripts/chained_demo.py`'s own
opportunistic mirror run).

Measured two ways, both in agreement: (1) a one-shot solve from arm B's
HOME qpos (the state the real waypoint-1 drive itself begins from), and
(2) the REAL 500-step waypoint-1 drive (`sk._run_waypoint`, unmodified),
then a one-shot solve at whatever qpos that drive actually terminates at.

| state | num_seeds | best residual | variance | distinct values | winning seed |
|---|---:|---:|---:|---:|---:|
| entry (HOME) | 1 | 0.09536 | 0.0 | 1 | 0 |
| entry (HOME) | 8 | 0.09536 | 1.33e-22 | 1 | 2 |
| entry (HOME) | 32 | 0.09536 | 1.80e-21 | 1 | 25 |
| post-real-drive terminal | 1 | 0.09536 | 0.0 | 1 | 0 |
| post-real-drive terminal | 8 | 0.09536 | 6.02e-21 | 1 | 4 |
| post-real-drive terminal | 32 | 0.09536 | 6.92e-21 | 1 | 28 |

The real 500-step drive (`real_drive_reason`) reports `convergence (IK
residual=0.0954 m >= 0.01 m)` — matching the documented figure exactly —
and terminates at a qpos whose own residual (0.09536) is identical to the
pre-drive entry state's, to 5 decimal places. Driving 500 real physics
steps toward this target does not move the arm any closer than the
single IK solve already predicted.

**Case 1 — genuine boundary, NOT rescuable by multi-seed.** Variance is
~19-21 orders of magnitude below physical scale at both states; every
seed, from both the entry pose and the real terminal pose, converges to
the identical residual.

### (d) Chain Phase 3 `to_arm` approach — pose-history-dependent, seeded from the actual chained-demo state

Reproduced by calling `run_pick(A, fork)` for real (weld-backed), then
Phase 1 (skipped — `already_held`, ADR-054) and Phase 2
(`sk._run_approach_with_staging(A, transfer_point, ...)`, run for real) in
the same order `run_handoff` itself executes — exactly
`scripts/chained_demo.py`'s own step 1 plus `run_handoff`'s own Phase 1/2,
not an approximation of that state. Target (`receiving_point`):
`[0.000, 0.020, 0.430]`. Documented: **0.0875 m**
(`docs/hardware/overnight-batch-log.md`'s ADR-054 entry: `"phase 3 (to_arm
approach) failed [direct approach failed [convergence (IK
residual=0.0875 m >= 0.01 m)]..."`).

| state | num_seeds | best residual | variance | distinct values | winning seed |
|---|---:|---:|---:|---:|---:|
| entry (instant Phase 2 finishes; B still at HOME) | 1 | 0.08748 | 0.0 | 1 | 0 |
| entry | 8 | 0.08748 | 8.37e-19 | 1 | 6 |
| entry | 32 | 0.08748 | 1.35e-18 | 1 | 17 |
| post-real-drive terminal | 1 | 0.08748 | 0.0 | 1 | 0 |
| post-real-drive terminal | 8 | 0.08748 | 7.67e-20 | 1 | 5 |
| post-real-drive terminal | 32 | 0.08748 | 7.34e-20 | 1 | 27 |

Same pattern as (c): the real Phase-3 direct-shot waypoint drive
(`sk._run_waypoint`, 500 steps, unmodified) reports `convergence (IK
residual=0.0875 m >= 0.01 m)`, matching the documented figure exactly, and
its terminal qpos's own residual (0.08748) is identical to the pre-drive
entry state's.

**Case 1 — genuine boundary, NOT rescuable by multi-seed.** Variance
~18-20 orders of magnitude below physical scale; every seed from both
states converges to the identical residual, despite this target being the
one MOST likely a priori to show pose-history-dependent local-minimum
behaviour (it only exists because Phase 1 got skipped, per
overnight-batch-log.md's own "likely mechanism" paragraph). It does not.

### (e) `pick(A, water_bottle)` at ADR-053's 7 failing seeds in the 10-19 range

**This target does not fit the interpretation rule as a failure case at
all — reported as its own, fourth finding.** `docs/hardware/
m08-extended-eval.md`'s own seed table records all 7 seeds
(10, 12, 13, 15, 17, 18, 19) failing with `weld_attach_failed_after_
300_frames`, `frames_used=1300` — arithmetically, that is waypoint 1
(approach, ≤500 steps) plus waypoint 2 (descend, ≤500 steps) BOTH already
converging, followed by GRIP (300 steps) exhausting its budget with no
weld attach. Measured here anyway, for completeness: waypoint 1's own
one-shot IK residual at every one of the 7 seeds, at every `num_seeds`
level:

| seed | n=1 best | n=8 best (var, distinct) | n=32 best (var, distinct) |
|---:|---:|---|---|
| 10 | 0.00958 | 0.00585 (1.13e-06, 7) | 0.00585 (1.15e-06, 29) |
| 12 | 0.00863 | 0.00756 (2.48e-07, 8) | 0.00721 (4.87e-07, 32) |
| 13 | 0.00818 | 0.00735 (2.29e-07, 8) | 0.00726 (4.54e-07, 30) |
| 15 | 0.00799 | 0.00700 (4.09e-07, 8) | 0.00700 (8.95e-07, 32) |
| 17 | 0.00943 | 0.00577 (1.31e-06, 8) | 0.00577 (1.37e-06, 31) |
| 18 | 0.00754 | 0.00677 (1.15e-06, 8) | 0.00670 (7.36e-07, 30) |
| 19 | 0.00852 | 0.00744 (5.02e-07, 8) | 0.00736 (6.49e-07, 31) |

**Every single value at every seed and every `num_seeds` level is already
below the 0.010 m tolerance** — waypoint 1 converges even at `num_seeds=1`
(no restart needed). Unlike (a)-(d), there IS real, non-floating-point-
noise variance here (1e-6 to 1e-7, with up to 30-32 genuinely distinct
converged values across the 32 seeds) — the arm's approach pose is
genuinely under-constrained enough that different starting joints land on
measurably different, but all comfortably tolerant, solutions.

**Not applicable to the local-minimum-vs-boundary framework: there is no
IK failure here to classify.** The documented failure (`weld_attach_
failed_after_300_frames`) happens strictly AFTER a converged IK approach
and descend — it is a grasp/weld-mechanism failure (`grasp.py`'s attach
logic), not a kinematic reachability problem. Neither `num_seeds`
(perturbs the IK solver's starting joint configuration) nor a
target-perturbation retry (perturbs the IK target position) can influence
a mechanism that only engages once the arm has ALREADY arrived at a
converged pose. This is itself the finding that matters for Commit 3:
whatever eventually addresses ADR-053's 7 failing seeds, it is not an IK
fix of any kind.

## Regression gate

**Before** (bm-ptl, this commit's own run, prior to adding
`scripts/probe_multiseed_diagnostic.py` and this document):
`scripts/verify_adr038_skills.py` → `0.3989 / 0.3588 / 0.6192 / 0.1946`,
`frames_used=6610`. `pytest tests/test_skills.py` → **4 passed / 4
failed**, including the identical `IK residual=0.0532 m` failure in
`test_handoff_mug_ends_held_by_arm_b` this document's own target (b)
reuses.

**After** (same commands, same machine, after this commit's additions —
`scripts/probe_multiseed_diagnostic.py` and this document, nothing else):
`scripts/verify_adr038_skills.py` → `0.3989 / 0.3588 / 0.6192 / 0.1946`,
`frames_used=6610` — **byte-identical to Before, to four decimal places on
every number.** `pytest tests/test_skills.py` → **4 passed / 4 failed**,
same four tests, same reasons, including the identical `IK
residual=0.0532 m` failure. No regression; expected, since neither `ik.py`
nor `skills_scripted.py` (nor any other file the regression gate
exercises) was touched by this commit.

## Verdict

Across all five targets, the diagnostic found **zero** Case 2
(rescuable-by-multi-seed) or Case 3 (multi-modal-but-worth-perturbing-the-
target) instances: (a), (b), (c) and (d) are all clean Case 1 genuine
boundaries — every one of 32 restart seeds, spanning ±0.15 rad per joint
from the real failing state (including, for (c) and (d), the ACTUAL
post-500-step terminal pose of a real physical drive, not just an
analytic entry state), converges to the identical residual, with variance
14-21 orders of magnitude below any physical scale. (e) is not an IK
failure at all — IK already converges at every seed tested; the
documented failure is downstream, in the grasp/weld mechanism.
**Commit 3's retry-with-perturbed-target wrapper is NOT supported by this
data: there is no diagnosed failure in this repository, among the five
measured here, for which varying either the solver's starting point
(multi-seed, already tried) or the target position (untried, Commit 3's
proposed lever) has any evidence of helping — proceed only if a future,
explicit target-perturbation measurement (not run in this commit) finds
signal that this one did not.**

