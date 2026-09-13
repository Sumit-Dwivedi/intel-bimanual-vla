# M06 handoff-transfer-point reachability sweep (ADR-032, second pass)

Second measurement pass after the first pass (all 36 candidates FAIL, seeded from home) -- four corrections (C1-C4) plus two additions applied; see `scripts/probe_handoff_reachability.py`'s module docstring for the full rationale.

Seed: 0, reset via `TableSettingEnv.reset()` (home keyframe, ADR-026).
Direction gated on: handoff(A, B, fork) (C4).
Sweep: x = 0.0, y in [-0.12, -0.1, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.1], z in [0.35, 0.38, 0.4] (specified) + [0.44, 0.47] (extra rows, addition 2).
PASS per arm: residual < 0.01 m (seeded from that arm's own handoff-APPROACH pose, C2) AND no joint-limit violation (margin > 0.0001 m from either `jnt_range` bound).
Collision bar (both arm_world_ok and cross_arm_ok): any contact deeper than -0.005 m is a FAIL (C3 / addition 1).

| x | y | z | armA_residual | armB_residual | both_reachable | arm_world_ok | cross_arm_ok | final_verdict |
|---:|---:|---:|---:|---:|---|---|---|---|
| 0.00 | -0.12 | 0.35 | 0.21540 | 0.00711 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.35 | 0.20058 | 0.00935 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.35 | 0.18673 | 0.00817 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.35 | 0.17409 | 0.00959 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.35 | 0.16292 | 0.14364 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.35 | 0.15355 | 0.14965 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.35 | 0.14633 | 0.15799 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.35 | 0.00887 | 0.16830 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.35 | 0.00944 | 0.18024 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.35 | 0.00855 | 0.19352 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.35 | 0.00977 | 0.20788 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.35 | 0.00750 | 0.22311 | False | False | False | FAIL |
| 0.00 | -0.12 | 0.38 | 0.19730 | 0.00956 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.38 | 0.18100 | 0.00919 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.38 | 0.16552 | 0.00970 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.38 | 0.15111 | 0.03192 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.38 | 0.13809 | 0.11472 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.38 | 0.12691 | 0.12216 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.38 | 0.11808 | 0.13224 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.38 | 0.11215 | 0.14440 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.38 | 0.01731 | 0.15816 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.38 | 0.00981 | 0.17315 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.38 | 0.00990 | 0.18906 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.38 | 0.00522 | 0.20569 | False | False | False | FAIL |
| 0.00 | -0.12 | 0.40 | 0.18693 | 0.00947 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.40 | 0.16965 | 0.00979 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.40 | 0.15302 | 0.01706 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.40 | 0.13730 | 0.07009 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.40 | 0.12284 | 0.09581 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.40 | 0.11011 | 0.10461 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.40 | 0.09980 | 0.11622 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.40 | 0.09272 | 0.12989 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.40 | 0.03694 | 0.14503 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.40 | 0.00907 | 0.16124 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.40 | 0.00990 | 0.17822 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.40 | 0.00594 | 0.19577 | False | False | False | FAIL |
| 0.00 | -0.12 | 0.44 (extra) | 0.17141 | 0.00703 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.44 (extra) | 0.15237 | 0.00906 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.44 (extra) | 0.13362 | 0.02016 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.44 (extra) | 0.11528 | 0.05150 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.44 (extra) | 0.09760 | 0.06013 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.44 (extra) | 0.08101 | 0.07335 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.44 (extra) | 0.06632 | 0.08913 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.44 (extra) | 0.05508 | 0.10634 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.44 (extra) | 0.03813 | 0.12439 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.44 (extra) | 0.00996 | 0.14295 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.44 (extra) | 0.01000 | 0.16186 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.44 (extra) | 0.00673 | 0.18101 | False | False | False | FAIL |
| 0.00 | -0.12 | 0.47 (extra) | 0.16526 | 0.00563 | False | False | False | FAIL |
| 0.00 | -0.10 | 0.47 (extra) | 0.14542 | 0.00671 | False | False | False | FAIL |
| 0.00 | -0.08 | 0.47 (extra) | 0.12563 | 0.00836 | False | False | False | FAIL |
| 0.00 | -0.06 | 0.47 (extra) | 0.10591 | 0.02418 | False | False | False | FAIL |
| 0.00 | -0.04 | 0.47 (extra) | 0.08634 | 0.03930 | False | False | False | FAIL |
| 0.00 | -0.02 | 0.47 (extra) | 0.06701 | 0.05752 | False | False | False | FAIL |
| 0.00 | 0.00 | 0.47 (extra) | 0.04823 | 0.07663 | False | False | False | FAIL |
| 0.00 | 0.02 | 0.47 (extra) | 0.03103 | 0.09610 | False | False | False | FAIL |
| 0.00 | 0.04 | 0.47 (extra) | 0.02012 | 0.11576 | False | False | False | FAIL |
| 0.00 | 0.06 | 0.47 (extra) | 0.00529 | 0.13551 | False | False | False | FAIL |
| 0.00 | 0.08 | 0.47 (extra) | 0.00619 | 0.15533 | False | False | False | FAIL |
| 0.00 | 0.10 | 0.47 (extra) | 0.00506 | 0.17519 | False | False | False | FAIL |

**NO candidate passed at any tested (y, z), including the extra z in [0.44, 0.47] rows.** No collision-free, both-reachable shared transfer point was found anywhere in this sweep.

**Overall: ALL FAIL.**

