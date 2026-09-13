# M06 handoff-transfer-point reachability probe (fix for the `handoff` waypoint-3 regression)

Re-measures the shared handoff band with BOTH IK residual AND collision checked (the previously-reported `m06-reachability-probe.md` "Shared handoff band" line came from a residual-only sweep -- see this script's module docstring).

Seed: 0, reset via `TableSettingEnv.reset()` (home keyframe, ADR-026).
Sweep: x = 0.0, y in [-0.12, -0.1, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.1], z in [0.35, 0.38, 0.4].
PASS (per arm) requires residual < 0.01 m and no NEW collision (contact-count delta against the home-pose baseline, exactly as `probe_reachability.py` computes it for its own primary probes).
Measured baseline (contacts involving each arm's geoms at reset, before any IK solve): arm A=0, arm B=0

Note: z=0.35 is exactly the tabletop surface (`TABLE_SURFACE_Z`), so candidates at that height are the most likely to collide with the table -- expected, not anomalous.

| x | y | z | arm_A_residual | arm_A_collision | arm_B_residual | arm_B_collision | both_pass |
|---:|---:|---:|---:|---|---:|---|---|
| 0.00 | -0.12 | 0.35 | 0.21540 | True | 0.00778 | True | FAIL |
| 0.00 | -0.10 | 0.35 | 0.20058 | True | 0.00792 | True | FAIL |
| 0.00 | -0.08 | 0.35 | 0.18673 | True | 0.00721 | True | FAIL |
| 0.00 | -0.06 | 0.35 | 0.17409 | True | 0.00869 | True | FAIL |
| 0.00 | -0.04 | 0.35 | 0.16292 | True | 0.00927 | True | FAIL |
| 0.00 | -0.02 | 0.35 | 0.15355 | True | 0.00956 | True | FAIL |
| 0.00 | 0.00 | 0.35 | 0.14633 | True | 0.14633 | True | FAIL |
| 0.00 | 0.02 | 0.35 | 0.00956 | True | 0.15355 | True | FAIL |
| 0.00 | 0.04 | 0.35 | 0.00927 | True | 0.16292 | True | FAIL |
| 0.00 | 0.06 | 0.35 | 0.00869 | True | 0.17409 | True | FAIL |
| 0.00 | 0.08 | 0.35 | 0.00721 | True | 0.18673 | True | FAIL |
| 0.00 | 0.10 | 0.35 | 0.00792 | True | 0.20058 | True | FAIL |
| 0.00 | -0.12 | 0.38 | 0.19730 | True | 0.00633 | True | FAIL |
| 0.00 | -0.10 | 0.38 | 0.18100 | True | 0.00369 | True | FAIL |
| 0.00 | -0.08 | 0.38 | 0.16552 | True | 0.00966 | True | FAIL |
| 0.00 | -0.06 | 0.38 | 0.15111 | True | 0.00930 | True | FAIL |
| 0.00 | -0.04 | 0.38 | 0.13809 | True | 0.01741 | True | FAIL |
| 0.00 | -0.02 | 0.38 | 0.12691 | True | 0.11215 | True | FAIL |
| 0.00 | 0.00 | 0.38 | 0.11808 | True | 0.11808 | True | FAIL |
| 0.00 | 0.02 | 0.38 | 0.11215 | True | 0.12691 | True | FAIL |
| 0.00 | 0.04 | 0.38 | 0.01741 | True | 0.13809 | True | FAIL |
| 0.00 | 0.06 | 0.38 | 0.00930 | True | 0.15111 | True | FAIL |
| 0.00 | 0.08 | 0.38 | 0.00966 | True | 0.16552 | True | FAIL |
| 0.00 | 0.10 | 0.38 | 0.00369 | True | 0.18100 | True | FAIL |
| 0.00 | -0.12 | 0.40 | 0.18693 | True | 0.00859 | True | FAIL |
| 0.00 | -0.10 | 0.40 | 0.16965 | True | 0.00648 | True | FAIL |
| 0.00 | -0.08 | 0.40 | 0.15302 | True | 0.00989 | True | FAIL |
| 0.00 | -0.06 | 0.40 | 0.13730 | True | 0.00926 | True | FAIL |
| 0.00 | -0.04 | 0.40 | 0.12284 | True | 0.03712 | True | FAIL |
| 0.00 | -0.02 | 0.40 | 0.11011 | True | 0.09272 | True | FAIL |
| 0.00 | 0.00 | 0.40 | 0.09980 | True | 0.09980 | True | FAIL |
| 0.00 | 0.02 | 0.40 | 0.09272 | True | 0.11011 | True | FAIL |
| 0.00 | 0.04 | 0.40 | 0.03712 | True | 0.12284 | True | FAIL |
| 0.00 | 0.06 | 0.40 | 0.00926 | True | 0.13730 | True | FAIL |
| 0.00 | 0.08 | 0.40 | 0.00989 | True | 0.15302 | True | FAIL |
| 0.00 | 0.10 | 0.40 | 0.00648 | True | 0.16965 | True | FAIL |

**NO candidate passed for both arms at any tested (y, z).** No collision-free shared reach point was found in this sweep.

**Overall: ALL FAIL**

