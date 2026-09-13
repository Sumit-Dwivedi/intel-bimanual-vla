# M06 handoff-transfer-point reachability sweep -- HOME-seeded (diagnostic)

**Diagnostic only.** Re-runs the `ba76190` sweep (`scripts/probe_handoff_reachability.py`, approach-pose seeded, C1-C4 + two additions) with exactly one change: each arm's IK solve is seeded directly from the HOME keyframe and solved in a single stage straight to the candidate target, instead of the two-stage home->approach, approach->target solve. `HANDOFF_POSITION_XYZ` is unchanged; `skills_scripted.py` is untouched.

**Correction to the brief this script was run under:** the brief claimed `ba76190`'s report showed "all 30 failing cells failed on arm B's IK residual, the failure is asymmetric." That is not what the committed report shows. In `ba76190`, `both_reachable` was `False` on **every** one of its 60 rows, and the pattern was **symmetric**: each arm converges better on the side OPPOSITE its own base (arm A: residual ~0.26 on its own side / ~0.061 on the far side at z=0.35, arm B mirroring arm A's numbers on the opposite side). No asymmetry is asserted or looked for below.

**Row-count reconciliation.** `ba76190`'s table has 60 rows: the 12 y-values x 3 specified z-values (0.35, 0.38, 0.40) this task asked for, PLUS 24 more rows at two "extra" z-values (0.44, 0.47) that `ba76190`'s own script added beyond the original spec. This diagnostic's task instruction names only the 3 original z-values, so this script's grid is exactly those 36 rows (12 y x 3 z) -- the comparison below is against those SAME 36 rows of `ba76190`'s table (the 24 extra-z rows are not part of this comparison).

**`pytest tests/test_skills.py` state (measurement only, no source touched by this diagnostic).** Run on bm-ptl immediately before this sweep, on the clean `ba76190` tree plus only this new script: 4 passed, 4 failed (`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`), all failing on IK/approach-waypoint convergence, unrelated to and unaffected by this diagnostic (`skills_scripted.py`, `ik.py`, `grasp.py`, `executor.py` are untouched). This is the pre-existing state of the repo at `ba76190`, reported per the task's instruction to confirm this suite is unchanged, not a result of this measurement.

Seed: 0, reset via `TableSettingEnv.reset()` (home keyframe, ADR-026).
Direction gated on: handoff(A, B, fork) (C4, unchanged).
Sweep: x = 0.0, y in [-0.12, -0.1, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.1], z in [0.35, 0.38, 0.4].
PASS per arm: residual < 0.01 m (seeded directly from HOME, single-stage solve) AND no joint-limit violation (margin > 0.0001 m from either `jnt_range` bound).
Collision bar (both arm_world_ok and cross_arm_ok): any contact deeper than -0.005 m is a FAIL (identical to `ba76190`).

## Row-by-row comparison: approach-pose seed (ba76190) vs. home seed (this run)

delta = (this run's residual) - (ba76190's residual). Negative delta = home seeding converged BETTER (smaller residual) than approach-pose seeding at that same (x, y, z).

| x | y | z | armA_residual (approach-seed) | armA_residual (home-seed) | armA delta | armB_residual (approach-seed) | armB_residual (home-seed) | armB delta | both_reachable (home-seed) | final_verdict (home-seed) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0.00 | -0.12 | 0.35 | 0.21540 | 0.21540 | +0.00000 | 0.00711 | 0.00588 | -0.00123 | False | FAIL |
| 0.00 | -0.10 | 0.35 | 0.20058 | 0.20058 | +0.00000 | 0.00935 | 0.00965 | +0.00030 | False | FAIL |
| 0.00 | -0.08 | 0.35 | 0.18673 | 0.18673 | +0.00000 | 0.00817 | 0.00885 | +0.00068 | False | FAIL |
| 0.00 | -0.06 | 0.35 | 0.17409 | 0.17409 | -0.00000 | 0.00959 | 0.00996 | +0.00037 | False | FAIL |
| 0.00 | -0.04 | 0.35 | 0.16292 | 0.16292 | -0.00000 | 0.14364 | 0.14364 | -0.00000 | False | FAIL |
| 0.00 | -0.02 | 0.35 | 0.15355 | 0.15355 | +0.00000 | 0.14965 | 0.14965 | +0.00000 | False | FAIL |
| 0.00 | 0.00 | 0.35 | 0.14633 | 0.14633 | +0.00000 | 0.15799 | 0.15799 | -0.00000 | False | FAIL |
| 0.00 | 0.02 | 0.35 | 0.00887 | 0.00956 | +0.00069 | 0.16830 | 0.16830 | -0.00000 | False | FAIL |
| 0.00 | 0.04 | 0.35 | 0.00944 | 0.00927 | -0.00017 | 0.18024 | 0.18024 | +0.00000 | False | FAIL |
| 0.00 | 0.06 | 0.35 | 0.00855 | 0.00869 | +0.00014 | 0.19352 | 0.19352 | +0.00000 | False | FAIL |
| 0.00 | 0.08 | 0.35 | 0.00977 | 0.00721 | -0.00256 | 0.20788 | 0.20788 | +0.00000 | False | FAIL |
| 0.00 | 0.10 | 0.35 | 0.00750 | 0.00792 | +0.00042 | 0.22311 | 0.22311 | +0.00000 | False | FAIL |
| 0.00 | -0.12 | 0.38 | 0.19730 | 0.19730 | -0.00000 | 0.00956 | 0.00696 | -0.00260 | False | FAIL |
| 0.00 | -0.10 | 0.38 | 0.18100 | 0.18100 | +0.00000 | 0.00919 | 0.00955 | +0.00036 | False | FAIL |
| 0.00 | -0.08 | 0.38 | 0.16552 | 0.16552 | +0.00000 | 0.00970 | 0.00974 | +0.00004 | False | FAIL |
| 0.00 | -0.06 | 0.38 | 0.15111 | 0.15111 | -0.00000 | 0.03192 | 0.03311 | +0.00119 | False | FAIL |
| 0.00 | -0.04 | 0.38 | 0.13809 | 0.13809 | +0.00000 | 0.11472 | 0.11472 | -0.00000 | False | FAIL |
| 0.00 | -0.02 | 0.38 | 0.12691 | 0.12691 | -0.00000 | 0.12216 | 0.12216 | +0.00000 | False | FAIL |
| 0.00 | 0.00 | 0.38 | 0.11808 | 0.11808 | -0.00000 | 0.13224 | 0.13224 | +0.00000 | False | FAIL |
| 0.00 | 0.02 | 0.38 | 0.11215 | 0.11215 | +0.00000 | 0.14440 | 0.14440 | +0.00000 | False | FAIL |
| 0.00 | 0.04 | 0.38 | 0.01731 | 0.01741 | +0.00010 | 0.15816 | 0.15816 | +0.00000 | False | FAIL |
| 0.00 | 0.06 | 0.38 | 0.00981 | 0.00930 | -0.00051 | 0.17315 | 0.17315 | -0.00000 | False | FAIL |
| 0.00 | 0.08 | 0.38 | 0.00990 | 0.00966 | -0.00024 | 0.18906 | 0.18906 | -0.00000 | False | FAIL |
| 0.00 | 0.10 | 0.38 | 0.00522 | 0.00369 | -0.00153 | 0.20569 | 0.20569 | -0.00000 | False | FAIL |
| 0.00 | -0.12 | 0.40 | 0.18693 | 0.18693 | +0.00000 | 0.00947 | 0.00586 | -0.00361 | False | FAIL |
| 0.00 | -0.10 | 0.40 | 0.16965 | 0.16965 | -0.00000 | 0.00979 | 0.00854 | -0.00125 | False | FAIL |
| 0.00 | -0.08 | 0.40 | 0.15302 | 0.15302 | +0.00000 | 0.01706 | 0.01701 | -0.00005 | False | FAIL |
| 0.00 | -0.06 | 0.40 | 0.13730 | 0.13730 | +0.00000 | 0.07009 | 0.07025 | +0.00016 | False | FAIL |
| 0.00 | -0.04 | 0.40 | 0.12284 | 0.12284 | -0.00000 | 0.09581 | 0.09581 | -0.00000 | False | FAIL |
| 0.00 | -0.02 | 0.40 | 0.11011 | 0.11011 | +0.00000 | 0.10461 | 0.10461 | -0.00000 | False | FAIL |
| 0.00 | 0.00 | 0.40 | 0.09980 | 0.09980 | +0.00000 | 0.11622 | 0.11622 | -0.00000 | False | FAIL |
| 0.00 | 0.02 | 0.40 | 0.09272 | 0.09272 | -0.00000 | 0.12989 | 0.12989 | -0.00000 | False | FAIL |
| 0.00 | 0.04 | 0.40 | 0.03694 | 0.03712 | +0.00018 | 0.14503 | 0.14503 | +0.00000 | False | FAIL |
| 0.00 | 0.06 | 0.40 | 0.00907 | 0.00926 | +0.00019 | 0.16124 | 0.16124 | +0.00000 | False | FAIL |
| 0.00 | 0.08 | 0.40 | 0.00990 | 0.00989 | -0.00001 | 0.17822 | 0.17822 | -0.00000 | False | FAIL |
| 0.00 | 0.10 | 0.40 | 0.00594 | 0.00648 | +0.00054 | 0.19577 | 0.19577 | +0.00000 | False | FAIL |

## Full home-seeded sweep table (same format as `ba76190`'s report)

| x | y | z | armA_residual | armB_residual | both_reachable | arm_world_ok | cross_arm_ok | final_verdict |
|---:|---:|---:|---:|---:|---|---|---|---|
| 0.00 | -0.12 | 0.35 | 0.21540 | 0.00588 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.35 | 0.20058 | 0.00965 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.35 | 0.18673 | 0.00885 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.35 | 0.17409 | 0.00996 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.35 | 0.16292 | 0.14364 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.35 | 0.15355 | 0.14965 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.35 | 0.14633 | 0.15799 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.35 | 0.00956 | 0.16830 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.35 | 0.00927 | 0.18024 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.35 | 0.00869 | 0.19352 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.35 | 0.00721 | 0.20788 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.35 | 0.00792 | 0.22311 | False | False | False | FAIL |
| 0.00 | -0.12 | 0.38 | 0.19730 | 0.00696 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.38 | 0.18100 | 0.00955 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.38 | 0.16552 | 0.00974 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.38 | 0.15111 | 0.03311 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.38 | 0.13809 | 0.11472 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.38 | 0.12691 | 0.12216 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.38 | 0.11808 | 0.13224 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.38 | 0.11215 | 0.14440 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.38 | 0.01741 | 0.15816 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.38 | 0.00930 | 0.17315 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.38 | 0.00966 | 0.18906 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.38 | 0.00369 | 0.20569 | False | False | False | FAIL |
| 0.00 | -0.12 | 0.40 | 0.18693 | 0.00586 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.40 | 0.16965 | 0.00854 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.40 | 0.15302 | 0.01701 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.40 | 0.13730 | 0.07025 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.40 | 0.12284 | 0.09581 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.40 | 0.11011 | 0.10461 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.40 | 0.09980 | 0.11622 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.40 | 0.09272 | 0.12989 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.40 | 0.03712 | 0.14503 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.40 | 0.00926 | 0.16124 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.40 | 0.00989 | 0.17822 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.40 | 0.00648 | 0.19577 | False | False | False | FAIL |

**NO candidate passed at any tested (y, z) with home seeding either.** No collision-free, both-reachable shared transfer point was found anywhere in this sweep.

**Overall: ALL FAIL (home-seeded).**

## Outcome

**Home-pose ALSO FAILS**, on the same residual grounds as the approach-pose sweep (see the delta columns above -- residuals are close to, not dramatically better than, `ba76190`'s numbers at the same (x, y, z), and `both_reachable` is `False` on every row here too). Seeding point is not the blocker. This is now two independently-seeded sweeps (home-only in the original first pass one commit prior to `ba76190`, and this home-only re-run) plus one approach-seeded sweep (`ba76190`) all agreeing: the shared band at x=0 genuinely does not support a static handoff position for this arm-base placement. This is strong enough evidence to act on ADR-021 (arm-base rearrangement) or a demo redesign, rather than to keep re-measuring.

