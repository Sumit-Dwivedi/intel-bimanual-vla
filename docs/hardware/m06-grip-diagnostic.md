# M06 GRIP-stage diagnostic: `pick(A, fork)`

Diagnostic only -- no behaviour change. Produced by `scripts/probe_grip_stage.py`, which reproduces (does not instrument) `skills_scripted.run_pick`'s four waypoints for `pick(A, fork)` -- see that script's module docstring for exactly what is reproduced vs. imported, and the one documented way this could diverge from the real skill (no per-step collision early-exit in this probe's driving loop).

Context: every waypoint of `pick(A, fork)` reports `ok=True` (APPROACH 500/500, DESCEND 500/500, GRIP 60/60, RETREAT 500/500), yet the fork never lifts (z 0.3560 -> 0.3538 in this run). This report looks inside the GRIP waypoint's 60 steps for the first time.

## Waypoint summary (reproduction)

| waypoint | converged | steps used |
|---|---|---:|
| APPROACH | False | 500 |
| DESCEND | False | 500 |
| GRIP (dwell, fully instrumented below) | -- | 60 |
| RETREAT | False | 500 |

grasp_point (world) = (-0.0650, 0.0500, 0.3560)
fork z: initial=0.3560  final=0.3538  delta=-0.0022

## GRIP waypoint: full 60-step timeline

Columns: gripper qpos/ctrl/delta (joint 2), stall flag (10+ consecutive steps of unchanged ctrl AND static qpos), pad separation distance and the LOWER pad's world z vs. the table surface (0.35 m, handle sits ~0.352 m -- radius 0.004 m above it), closing-axis-vs-handle angle (degrees; only meaningful when a pad-fork contact exists this step, else shown as `--`), fork body world z/xy, and contact flags (pad-vs-fork / pad-vs-table / pad-vs-other, with the other geom named).

| step | qpos | ctrl | delta | stall | pad_dist(m) | lower_pad_z | fork_z | fork_xy | angle(deg) | pad-fork | pad-table | pad-other | deepest_dist(m) | max_force(N) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 1.7449 | -0.1745 | -0.000452 | no | 0.1319 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00051 | 10.9650 |
| 2 | 1.7440 | -0.1745 | -0.000885 | no | 0.1319 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9605 |
| 3 | 1.7427 | -0.1745 | -0.001300 | no | 0.1318 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9563 |
| 4 | 1.7410 | -0.1745 | -0.001698 | no | 0.1318 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9524 |
| 5 | 1.7389 | -0.1745 | -0.002080 | no | 0.1317 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9489 |
| 6 | 1.7365 | -0.1745 | -0.002446 | no | 0.1316 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9456 |
| 7 | 1.7337 | -0.1745 | -0.002797 | no | 0.1315 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9426 |
| 8 | 1.7305 | -0.1745 | -0.003134 | no | 0.1314 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9399 |
| 9 | 1.7271 | -0.1745 | -0.003457 | no | 0.1312 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9375 |
| 10 | 1.7233 | -0.1745 | -0.003766 | no | 0.1311 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9354 |
| 11 | 1.7193 | -0.1745 | -0.004063 | no | 0.1309 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9336 |
| 12 | 1.7149 | -0.1745 | -0.004348 | no | 0.1307 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9321 |
| 13 | 1.7103 | -0.1745 | -0.004621 | no | 0.1305 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00050 | 10.9309 |
| 14 | 1.7054 | -0.1745 | -0.004883 | no | 0.1303 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.9300 |
| 15 | 1.7003 | -0.1745 | -0.005134 | no | 0.1301 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.9201 |
| 16 | 1.6949 | -0.1745 | -0.005375 | no | 0.1299 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.9084 |
| 17 | 1.6893 | -0.1745 | -0.005606 | no | 0.1296 | 0.3510 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8968 |
| 18 | 1.6835 | -0.1745 | -0.005827 | no | 0.1294 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8855 |
| 19 | 1.6774 | -0.1745 | -0.006039 | no | 0.1291 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8744 |
| 20 | 1.6712 | -0.1745 | -0.006243 | no | 0.1288 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8635 |
| 21 | 1.6647 | -0.1745 | -0.006438 | no | 0.1285 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8529 |
| 22 | 1.6581 | -0.1745 | -0.006625 | no | 0.1282 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8426 |
| 23 | 1.6513 | -0.1745 | -0.006805 | no | 0.1279 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8325 |
| 24 | 1.6443 | -0.1745 | -0.006977 | no | 0.1276 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00049 | 10.8228 |
| 25 | 1.6372 | -0.1745 | -0.007142 | no | 0.1273 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.8133 |
| 26 | 1.6299 | -0.1745 | -0.007300 | no | 0.1269 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.8041 |
| 27 | 1.6224 | -0.1745 | -0.007452 | no | 0.1266 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7951 |
| 28 | 1.6148 | -0.1745 | -0.007598 | no | 0.1262 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7865 |
| 29 | 1.6071 | -0.1745 | -0.007737 | no | 0.1258 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7782 |
| 30 | 1.5992 | -0.1745 | -0.007871 | no | 0.1255 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7702 |
| 31 | 1.5912 | -0.1745 | -0.008000 | no | 0.1251 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7625 |
| 32 | 1.5831 | -0.1745 | -0.008123 | no | 0.1247 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7551 |
| 33 | 1.5749 | -0.1745 | -0.008241 | no | 0.1243 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7480 |
| 34 | 1.5665 | -0.1745 | -0.008354 | no | 0.1239 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 10.7412 |
| 35 | 1.5580 | -0.1745 | -0.008463 | no | 0.1235 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 10.7347 |
| 36 | 1.5495 | -0.1745 | -0.008567 | no | 0.1230 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 12.9654 |
| 37 | 1.5408 | -0.1745 | -0.008667 | no | 0.1226 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 12.9753 |
| 38 | 1.5321 | -0.1745 | -0.008762 | no | 0.1222 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 12.9841 |
| 39 | 1.5232 | -0.1745 | -0.008854 | no | 0.1217 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 12.9919 |
| 40 | 1.5143 | -0.1745 | -0.008942 | no | 0.1213 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 12.9990 |
| 41 | 1.5052 | -0.1745 | -0.009027 | no | 0.1208 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 13.0054 |
| 42 | 1.4961 | -0.1745 | -0.009108 | no | 0.1204 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 13.0109 |
| 43 | 1.4869 | -0.1745 | -0.009185 | no | 0.1199 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.0150 |
| 44 | 1.4777 | -0.1745 | -0.009260 | no | 0.1194 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.0569 |
| 45 | 1.4683 | -0.1745 | -0.009331 | no | 0.1189 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.0957 |
| 46 | 1.4589 | -0.1745 | -0.009400 | no | 0.1184 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1308 |
| 47 | 1.4495 | -0.1745 | -0.009465 | no | 0.1179 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1630 |
| 48 | 1.4400 | -0.1745 | -0.009528 | no | 0.1174 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1683 |
| 49 | 1.4304 | -0.1745 | -0.009589 | no | 0.1169 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1721 |
| 50 | 1.4207 | -0.1745 | -0.009647 | no | 0.1164 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1756 |
| 51 | 1.4110 | -0.1745 | -0.009702 | no | 0.1159 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1789 |
| 52 | 1.4013 | -0.1745 | -0.009755 | no | 0.1153 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1821 |
| 53 | 1.3915 | -0.1745 | -0.009807 | no | 0.1148 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1851 |
| 54 | 1.3816 | -0.1745 | -0.009856 | no | 0.1143 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1882 |
| 55 | 1.3717 | -0.1745 | -0.009902 | no | 0.1137 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00048 | 13.1912 |
| 56 | 1.3617 | -0.1745 | -0.009948 | no | 0.1132 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 13.1942 |
| 57 | 1.3518 | -0.1745 | -0.009991 | no | 0.1126 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 13.1973 |
| 58 | 1.3417 | -0.1745 | -0.010032 | no | 0.1121 | 0.3511 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 10.6833 |
| 59 | 1.3317 | -0.1745 | -0.010072 | no | 0.1115 | 0.3512 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 10.6598 |
| 60 | 1.3215 | -0.1745 | -0.010110 | no | 0.1109 | 0.3512 | 0.3538 | (-0.0583, 0.0676) | -- | NO | YES | - | -0.00047 | 10.6352 |

## Per-contact raw log (every pad-involving contact, every step)

```
step  1: pad=static other=table_top                    dist=-0.00051 m force=10.9650 N
step  1: pad=static other=table_top                    dist=-0.00023 m force=2.4158 N
step  2: pad=static other=table_top                    dist=-0.00050 m force=10.9605 N
step  2: pad=static other=table_top                    dist=-0.00022 m force=2.4083 N
step  3: pad=static other=table_top                    dist=-0.00050 m force=10.9563 N
step  3: pad=static other=table_top                    dist=-0.00022 m force=2.4009 N
step  4: pad=static other=table_top                    dist=-0.00050 m force=10.9524 N
step  4: pad=static other=table_top                    dist=-0.00021 m force=2.3937 N
step  5: pad=static other=table_top                    dist=-0.00050 m force=10.9489 N
step  5: pad=static other=table_top                    dist=-0.00021 m force=2.3868 N
step  6: pad=static other=table_top                    dist=-0.00050 m force=10.9456 N
step  6: pad=static other=table_top                    dist=-0.00021 m force=2.3800 N
step  7: pad=static other=table_top                    dist=-0.00050 m force=10.9426 N
step  7: pad=static other=table_top                    dist=-0.00020 m force=2.3734 N
step  8: pad=static other=table_top                    dist=-0.00050 m force=10.9399 N
step  8: pad=static other=table_top                    dist=-0.00020 m force=2.3670 N
step  9: pad=static other=table_top                    dist=-0.00050 m force=10.9375 N
step  9: pad=static other=table_top                    dist=-0.00020 m force=2.3608 N
step 10: pad=static other=table_top                    dist=-0.00050 m force=10.9354 N
step 10: pad=static other=table_top                    dist=-0.00019 m force=2.3548 N
step 11: pad=static other=table_top                    dist=-0.00050 m force=10.9336 N
step 11: pad=static other=table_top                    dist=-0.00019 m force=2.3490 N
step 12: pad=static other=table_top                    dist=-0.00050 m force=10.9321 N
step 12: pad=static other=table_top                    dist=-0.00019 m force=2.3433 N
step 13: pad=static other=table_top                    dist=-0.00050 m force=10.9309 N
step 13: pad=static other=table_top                    dist=-0.00018 m force=2.3379 N
step 14: pad=static other=table_top                    dist=-0.00049 m force=10.9300 N
step 14: pad=static other=table_top                    dist=-0.00018 m force=2.3327 N
step 15: pad=static other=table_top                    dist=-0.00049 m force=10.9201 N
step 15: pad=static other=table_top                    dist=-0.00017 m force=2.3333 N
step 16: pad=static other=table_top                    dist=-0.00049 m force=10.9084 N
step 16: pad=static other=table_top                    dist=-0.00017 m force=2.3355 N
step 17: pad=static other=table_top                    dist=-0.00049 m force=10.8968 N
step 17: pad=static other=table_top                    dist=-0.00017 m force=2.3377 N
step 18: pad=static other=table_top                    dist=-0.00049 m force=10.8855 N
step 18: pad=static other=table_top                    dist=-0.00016 m force=2.3402 N
step 19: pad=static other=table_top                    dist=-0.00049 m force=10.8744 N
step 19: pad=static other=table_top                    dist=-0.00016 m force=2.3427 N
step 20: pad=static other=table_top                    dist=-0.00049 m force=10.8635 N
step 20: pad=static other=table_top                    dist=-0.00016 m force=2.3455 N
step 21: pad=static other=table_top                    dist=-0.00049 m force=10.8529 N
step 21: pad=static other=table_top                    dist=-0.00015 m force=2.3483 N
step 22: pad=static other=table_top                    dist=-0.00049 m force=10.8426 N
step 22: pad=static other=table_top                    dist=-0.00015 m force=2.3514 N
step 23: pad=static other=table_top                    dist=-0.00049 m force=10.8325 N
step 23: pad=static other=table_top                    dist=-0.00014 m force=2.3546 N
step 24: pad=static other=table_top                    dist=-0.00049 m force=10.8228 N
step 24: pad=static other=table_top                    dist=-0.00014 m force=2.3579 N
step 25: pad=static other=table_top                    dist=-0.00048 m force=10.8133 N
step 25: pad=static other=table_top                    dist=-0.00014 m force=2.3614 N
step 26: pad=static other=table_top                    dist=-0.00048 m force=10.8041 N
step 26: pad=static other=table_top                    dist=-0.00013 m force=2.3650 N
step 27: pad=static other=table_top                    dist=-0.00048 m force=10.7951 N
step 27: pad=static other=table_top                    dist=-0.00013 m force=2.3687 N
step 28: pad=static other=table_top                    dist=-0.00048 m force=10.7865 N
step 28: pad=static other=table_top                    dist=-0.00013 m force=2.3726 N
step 29: pad=static other=table_top                    dist=-0.00048 m force=10.7782 N
step 29: pad=static other=table_top                    dist=-0.00012 m force=2.3766 N
step 30: pad=static other=table_top                    dist=-0.00048 m force=10.7702 N
step 30: pad=static other=table_top                    dist=-0.00012 m force=2.3808 N
step 31: pad=static other=table_top                    dist=-0.00048 m force=10.7625 N
step 31: pad=static other=table_top                    dist=-0.00011 m force=2.3851 N
step 32: pad=static other=table_top                    dist=-0.00048 m force=10.7551 N
step 32: pad=static other=table_top                    dist=-0.00011 m force=2.3895 N
step 33: pad=static other=table_top                    dist=-0.00048 m force=10.7480 N
step 33: pad=static other=table_top                    dist=-0.00011 m force=2.3940 N
step 34: pad=static other=table_top                    dist=-0.00048 m force=10.7412 N
step 34: pad=static other=table_top                    dist=-0.00010 m force=2.3986 N
step 35: pad=static other=table_top                    dist=-0.00047 m force=10.7347 N
step 35: pad=static other=table_top                    dist=-0.00010 m force=2.4034 N
step 36: pad=static other=table_top                    dist=-0.00047 m force=12.9654 N
step 37: pad=static other=table_top                    dist=-0.00047 m force=12.9753 N
step 38: pad=static other=table_top                    dist=-0.00047 m force=12.9841 N
step 39: pad=static other=table_top                    dist=-0.00047 m force=12.9919 N
step 40: pad=static other=table_top                    dist=-0.00047 m force=12.9990 N
step 41: pad=static other=table_top                    dist=-0.00047 m force=13.0054 N
step 42: pad=static other=table_top                    dist=-0.00047 m force=13.0109 N
step 43: pad=static other=table_top                    dist=-0.00048 m force=13.0150 N
step 44: pad=static other=table_top                    dist=-0.00048 m force=13.0569 N
step 45: pad=static other=table_top                    dist=-0.00048 m force=13.0957 N
step 46: pad=static other=table_top                    dist=-0.00048 m force=13.1308 N
step 47: pad=static other=table_top                    dist=-0.00048 m force=13.1630 N
step 48: pad=static other=table_top                    dist=-0.00048 m force=13.1683 N
step 49: pad=static other=table_top                    dist=-0.00048 m force=13.1721 N
step 50: pad=static other=table_top                    dist=-0.00048 m force=13.1756 N
step 51: pad=static other=table_top                    dist=-0.00048 m force=13.1789 N
step 52: pad=static other=table_top                    dist=-0.00048 m force=13.1821 N
step 53: pad=static other=table_top                    dist=-0.00048 m force=13.1851 N
step 54: pad=static other=table_top                    dist=-0.00048 m force=13.1882 N
step 55: pad=static other=table_top                    dist=-0.00048 m force=13.1912 N
step 56: pad=static other=table_top                    dist=-0.00047 m force=13.1942 N
step 57: pad=static other=table_top                    dist=-0.00047 m force=13.1973 N
step 58: pad=static other=table_top                    dist=-0.00047 m force=10.6833 N
step 58: pad=static other=table_top                    dist=-0.00003 m force=2.8243 N
step 59: pad=static other=table_top                    dist=-0.00047 m force=10.6598 N
step 59: pad=static other=table_top                    dist=-0.00003 m force=2.8365 N
step 60: pad=static other=table_top                    dist=-0.00047 m force=10.6352 N
step 60: pad=static other=table_top                    dist=-0.00002 m force=2.8527 N
```

## Key findings (derived from the timeline above)

- Only the **static** pad ever registers a contact across all 60 steps; the contact is exclusively against `table_top`, never against any fork geom.
- The gripper joint (`armA_gripper`) closes continuously and is NOT stalled (qpos moves every step, delta magnitude growing from -0.000452 to -0.010110 rad/step) -- it goes from qpos=1.7449 to qpos=1.3215 over 60 steps, closing by only 0.4233 rad out of the joint's full ~1.92 rad range (-0.1745 to 1.7453 rad) -- i.e. `GRIP_HOLD_FRAMES=60` is not enough time for the jaw to travel anywhere near fully closed, exactly the documented, flagged limitation in `skills_scripted.GRIP_HOLD_FRAMES`'s own docstring.
- The fork body's position is **static to 4 decimal places** across all 60 GRIP steps (moved 0.000034 m total) -- consistent with zero force ever being transmitted to it, because neither pad ever touches it.
- The static pad's world z sits at 0.3510-0.3512 m, i.e. inside or below `TABLE_SURFACE_Z=0.35` m, while the fork body itself sits at z=0.3538 m -- the static pad is being driven toward a point BELOW the fork's own resting height, into the table, not toward the fork's handle.

## Verdict

**`Pads on table`**

![mid-GRIP frame, armA_wrist camera](../images/m06-grip-diagnostic-frame30.png)

Rendered at GRIP step 30 (of 60) from the `armA_wrist` camera, 1280x720, via `env.render('armA_wrist')` (the explicit escape hatch, ADR-022 -- this env was constructed with `cameras=None` for the rest of the run, so no per-step render cost was paid for the other 59 steps).
