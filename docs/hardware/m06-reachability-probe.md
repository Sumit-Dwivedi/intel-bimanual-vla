# M06 reachability probe (ADR-025)

Seed: 0. PASS requires residual < 0.01 m and no NEW collision
(a contact count above this arm's measured RESET-pose baseline -- see 'Baseline finding' below).

Measured baseline (contacts involving each arm's geoms at reset(seed=0), before any IK solve): arm A=29, arm B=29

## Primary targets

| target | target pos (x,y,z) | arm | residual (m) | converged | new collision | n_contacts (baseline) | PASS |
|---|---|---|---:|---|---|---:|---|
| closed_drawer_face | (0.0000, -0.2500, 0.2800) | A | 0.02258 | False | False | 11 (29) | FAIL |
| closed_drawer_face | (0.0000, -0.2500, 0.2800) | B | 0.21351 | False | False | 12 (29) | FAIL |
| plate_at_rest | (-0.1500, 0.0000, 0.3560) | A | 0.00377 | True | False | 8 (29) | PASS |
| plate_at_rest | (-0.1500, 0.0000, 0.3560) | B | 0.00429 | True | False | 9 (29) | PASS |
| mug_at_rest | (0.0500, -0.0300, 0.3900) | A | 0.00819 | True | False | 13 (29) | PASS |
| mug_at_rest | (0.0500, -0.0300, 0.3900) | B | 0.00530 | True | False | 11 (29) | PASS |
| bottle_at_rest | (0.2200, 0.0000, 0.4400) | A | 0.00429 | True | False | 5 (29) | PASS |
| bottle_at_rest | (0.2200, 0.0000, 0.4400) | B | 0.00385 | True | False | 5 (29) | PASS |

**Overall: AT LEAST ONE FAILURE**

## Baseline finding (separate from this probe's pass/fail; not fixed here)

At `reset(seed=0)`, BEFORE any IK solve runs, arm A and arm B's default rest poses already substantially interpenetrate -- e.g. `armA_lower_arm` vs. `armB_wrist` at up to ~6 cm penetration depth (measured via a one-off contact dump, not committed). This is independent of both fixes in this module (the drawer position and the IK pinch-point retarget) and independent of the target requested: it is a property of the compiled model's default qpos alone. This probe's collision check is defined as a DELTA against this measured baseline (see `_baseline_arm_contacts`) specifically so that this pre-existing, out-of-scope condition does not make every single target look like a solve-caused collision. Flagged here for the planner/compliance-reviewer as a genuine, separate finding -- ADR-021's ~0.30 m reach / 0.50 m base-gap layout assumption was never checked against the actual compiled rest pose, and this is that check, arriving late.

## Closed-drawer-face probe FAILED for at least one arm -- per instruction, no second scene guess. Reachable envelope measured instead.

Grid: x in [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3], y in [-0.35, -0.275, -0.2, -0.125, -0.05, 0.025, 0.1], z in [0.2, 0.3, 0.4, 0.5] (residual < tolerance only; collision not checked for the sweep -- see script docstring).

- **Arm A**: 95 reachable grid points; x in [-0.30, 0.30], y in [-0.28, 0.10], z in [0.30, 0.50]
- **Arm B**: 57 reachable grid points; x in [-0.30, 0.30], y in [-0.12, 0.10], z in [0.30, 0.50]

| arm | x | y | z |
|---|---:|---:|---:|
| A | -0.30 | -0.28 | 0.30 |
| A | -0.30 | -0.28 | 0.40 |
| A | -0.30 | -0.28 | 0.50 |
| A | -0.30 | -0.20 | 0.30 |
| A | -0.30 | -0.20 | 0.40 |
| A | -0.30 | -0.20 | 0.50 |
| A | -0.30 | -0.12 | 0.30 |
| A | -0.30 | -0.12 | 0.40 |
| A | -0.30 | -0.12 | 0.50 |
| A | -0.30 | -0.05 | 0.40 |
| A | -0.30 | -0.05 | 0.50 |
| A | -0.20 | -0.28 | 0.30 |
| A | -0.20 | -0.28 | 0.40 |
| A | -0.20 | -0.28 | 0.50 |
| A | -0.20 | -0.20 | 0.30 |
| A | -0.20 | -0.20 | 0.40 |
| A | -0.20 | -0.20 | 0.50 |
| A | -0.20 | -0.12 | 0.30 |
| A | -0.20 | -0.12 | 0.40 |
| A | -0.20 | -0.12 | 0.50 |
| A | -0.20 | -0.05 | 0.30 |
| A | -0.20 | -0.05 | 0.40 |
| A | -0.20 | -0.05 | 0.50 |
| A | -0.20 | 0.03 | 0.30 |
| A | -0.20 | 0.03 | 0.40 |
| A | -0.20 | 0.03 | 0.50 |
| A | -0.10 | -0.20 | 0.30 |
| A | -0.10 | -0.20 | 0.40 |
| A | -0.10 | -0.20 | 0.50 |
| A | -0.10 | -0.12 | 0.30 |
| A | -0.10 | -0.12 | 0.40 |
| A | -0.10 | -0.12 | 0.50 |
| A | -0.10 | -0.05 | 0.30 |
| A | -0.10 | -0.05 | 0.40 |
| A | -0.10 | -0.05 | 0.50 |
| A | -0.10 | 0.03 | 0.30 |
| A | -0.10 | 0.03 | 0.40 |
| A | -0.10 | 0.03 | 0.50 |
| A | -0.10 | 0.10 | 0.40 |
| A | -0.10 | 0.10 | 0.50 |
| A | 0.00 | -0.28 | 0.40 |
| A | 0.00 | -0.20 | 0.30 |
| A | 0.00 | -0.20 | 0.40 |
| A | 0.00 | -0.12 | 0.30 |
| A | 0.00 | -0.12 | 0.40 |
| A | 0.00 | -0.12 | 0.50 |
| A | 0.00 | -0.05 | 0.30 |
| A | 0.00 | -0.05 | 0.40 |
| A | 0.00 | -0.05 | 0.50 |
| A | 0.00 | 0.03 | 0.30 |
| A | 0.00 | 0.03 | 0.40 |
| A | 0.00 | 0.03 | 0.50 |
| A | 0.00 | 0.10 | 0.30 |
| A | 0.00 | 0.10 | 0.40 |
| A | 0.00 | 0.10 | 0.50 |
| A | 0.10 | -0.20 | 0.30 |
| A | 0.10 | -0.20 | 0.40 |
| A | 0.10 | -0.20 | 0.50 |
| A | 0.10 | -0.12 | 0.30 |
| A | 0.10 | -0.12 | 0.40 |
| A | 0.10 | -0.12 | 0.50 |
| A | 0.10 | -0.05 | 0.30 |
| A | 0.10 | -0.05 | 0.40 |
| A | 0.10 | -0.05 | 0.50 |
| A | 0.10 | 0.03 | 0.30 |
| A | 0.10 | 0.03 | 0.40 |
| A | 0.10 | 0.03 | 0.50 |
| A | 0.10 | 0.10 | 0.40 |
| A | 0.10 | 0.10 | 0.50 |
| A | 0.20 | -0.28 | 0.30 |
| A | 0.20 | -0.28 | 0.40 |
| A | 0.20 | -0.28 | 0.50 |
| A | 0.20 | -0.20 | 0.30 |
| A | 0.20 | -0.20 | 0.40 |
| A | 0.20 | -0.20 | 0.50 |
| A | 0.20 | -0.12 | 0.30 |
| A | 0.20 | -0.12 | 0.40 |
| A | 0.20 | -0.12 | 0.50 |
| A | 0.20 | -0.05 | 0.30 |
| A | 0.20 | -0.05 | 0.40 |
| A | 0.20 | -0.05 | 0.50 |
| A | 0.20 | 0.03 | 0.30 |
| A | 0.20 | 0.03 | 0.40 |
| A | 0.20 | 0.03 | 0.50 |
| A | 0.30 | -0.28 | 0.30 |
| A | 0.30 | -0.28 | 0.40 |
| A | 0.30 | -0.28 | 0.50 |
| A | 0.30 | -0.20 | 0.30 |
| A | 0.30 | -0.20 | 0.40 |
| A | 0.30 | -0.20 | 0.50 |
| A | 0.30 | -0.12 | 0.30 |
| A | 0.30 | -0.12 | 0.40 |
| A | 0.30 | -0.12 | 0.50 |
| A | 0.30 | -0.05 | 0.40 |
| A | 0.30 | -0.05 | 0.50 |
| B | -0.30 | 0.03 | 0.40 |
| B | -0.30 | 0.03 | 0.50 |
| B | -0.30 | 0.10 | 0.40 |
| B | -0.30 | 0.10 | 0.50 |
| B | -0.20 | -0.05 | 0.40 |
| B | -0.20 | -0.05 | 0.50 |
| B | -0.20 | 0.03 | 0.30 |
| B | -0.20 | 0.03 | 0.40 |
| B | -0.20 | 0.03 | 0.50 |
| B | -0.20 | 0.10 | 0.30 |
| B | -0.20 | 0.10 | 0.40 |
| B | -0.20 | 0.10 | 0.50 |
| B | -0.10 | -0.12 | 0.40 |
| B | -0.10 | -0.12 | 0.50 |
| B | -0.10 | -0.05 | 0.30 |
| B | -0.10 | -0.05 | 0.40 |
| B | -0.10 | -0.05 | 0.50 |
| B | -0.10 | 0.03 | 0.30 |
| B | -0.10 | 0.03 | 0.40 |
| B | -0.10 | 0.03 | 0.50 |
| B | -0.10 | 0.10 | 0.30 |
| B | -0.10 | 0.10 | 0.40 |
| B | -0.10 | 0.10 | 0.50 |
| B | 0.00 | -0.12 | 0.40 |
| B | 0.00 | -0.12 | 0.50 |
| B | 0.00 | -0.05 | 0.30 |
| B | 0.00 | -0.05 | 0.40 |
| B | 0.00 | -0.05 | 0.50 |
| B | 0.00 | 0.03 | 0.30 |
| B | 0.00 | 0.03 | 0.40 |
| B | 0.00 | 0.03 | 0.50 |
| B | 0.00 | 0.10 | 0.30 |
| B | 0.00 | 0.10 | 0.40 |
| B | 0.00 | 0.10 | 0.50 |
| B | 0.10 | -0.12 | 0.40 |
| B | 0.10 | -0.12 | 0.50 |
| B | 0.10 | -0.05 | 0.30 |
| B | 0.10 | -0.05 | 0.40 |
| B | 0.10 | -0.05 | 0.50 |
| B | 0.10 | 0.03 | 0.30 |
| B | 0.10 | 0.03 | 0.40 |
| B | 0.10 | 0.03 | 0.50 |
| B | 0.10 | 0.10 | 0.30 |
| B | 0.10 | 0.10 | 0.40 |
| B | 0.10 | 0.10 | 0.50 |
| B | 0.20 | -0.05 | 0.40 |
| B | 0.20 | -0.05 | 0.50 |
| B | 0.20 | 0.03 | 0.30 |
| B | 0.20 | 0.03 | 0.40 |
| B | 0.20 | 0.03 | 0.50 |
| B | 0.20 | 0.10 | 0.30 |
| B | 0.20 | 0.10 | 0.40 |
| B | 0.20 | 0.10 | 0.50 |
| B | 0.30 | 0.03 | 0.40 |
| B | 0.30 | 0.03 | 0.50 |
| B | 0.30 | 0.10 | 0.40 |
| B | 0.30 | 0.10 | 0.50 |

