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


---

# M06 reachability probe, RE-MEASURED (ADR-026)

Seed: 0, reset via `TableSettingEnv.reset()`, which now applies the "home" keyframe (arms folded back) instead of leaving qpos at the upstream all-zero default. **Every number below supersedes the corresponding number above**, which was measured from a rest pose in which the two arms interpenetrated by up to 6 cm before any IK solve ran. The section above is retained as the historical record of why this re-measurement exists, not as a currently-valid envelope.

PASS requires residual < 0.01 m and no NEW collision
(a contact count above this arm's measured RESET-pose baseline -- see 'Baseline finding' below).

Measured baseline (contacts involving each arm's geoms at reset(seed=0), before any IK solve, now AT THE HOME POSE): arm A=0, arm B=0

## Primary targets

| target | target pos (x,y,z) | arm | residual (m) | converged | new collision | n_contacts (baseline) | PASS |
|---|---|---|---:|---|---|---:|---|
| closed_drawer_face | (0.0000, -0.2500, 0.2800) | A | 0.32212 | False | False | 0 (0) | FAIL |
| closed_drawer_face | (0.0000, -0.2500, 0.2800) | B | 0.17857 | False | True | 15 (0) | FAIL |
| plate_at_rest | (-0.1500, 0.0000, 0.3560) | A | 0.00898 | True | True | 18 (0) | FAIL |
| plate_at_rest | (-0.1500, 0.0000, 0.3560) | B | 0.00844 | True | True | 15 (0) | FAIL |
| mug_at_rest | (0.0500, -0.0300, 0.3900) | A | 0.12016 | False | True | 1 (0) | FAIL |
| mug_at_rest | (0.0500, -0.0300, 0.3900) | B | 0.03941 | False | True | 13 (0) | FAIL |
| bottle_at_rest | (0.2200, 0.0000, 0.4400) | A | 0.00819 | True | True | 6 (0) | FAIL |
| bottle_at_rest | (0.2200, 0.0000, 0.4400) | B | 0.00916 | True | True | 6 (0) | FAIL |

**Overall: AT LEAST ONE FAILURE**

## Baseline finding, now fixed (was 'not fixed here' in the section above)

At `reset(seed=0)`, BEFORE any IK solve runs, the OLD default rest pose (all arm joints at 0 rad) put arm A and arm B up to ~6 cm deep into each other (29 of 34 total contacts were armA<->armB, deepest -0.0597 m, `armA_wrist` vs. `armB_wrist`). This was independent of the drawer position and the IK pinch-point retarget: it was a property of the compiled model's default qpos alone. **This is now fixed** by the "home" keyframe (ARCHITECTURE.md ADR-026): the measured baseline above, taken at the new reset pose, shows arm A=0, arm B=0 cross/self contacts. ADR-021's ~0.30 m reach / 0.50 m base-gap layout assumption was never checked against the actual compiled rest pose; this probe is that check, and the envelope below is the first one measured from a valid pose.

## Reachable envelope, re-measured from the home pose (always run this pass, per ADR-026 Step 2 -- not gated on primary-probe failure)

Grid: x in [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3], y in [-0.35, -0.275, -0.2, -0.125, -0.05, 0.025, 0.1], z in [0.2, 0.22, 0.24, 0.26, 0.28, 0.3, 0.32, 0.34, 0.36, 0.38, 0.4, 0.42, 0.44, 0.46, 0.48, 0.5] m steps=0.02 (residual < tolerance only; collision not checked for the sweep -- see script docstring).

- **Arm A**: 186 reachable grid points; x in [-0.30, 0.30], y in [-0.35, 0.10], z in [0.24, 0.50]
- **Arm B**: 154 reachable grid points; x in [-0.30, 0.30], y in [-0.12, 0.10], z in [0.24, 0.50]

**Shared handoff band (y-range intersection): y in [-0.12, 0.10] m.**

| arm | x | y | z |
|---|---:|---:|---:|
| A | -0.30 | -0.20 | 0.48 |
| A | -0.30 | -0.20 | 0.50 |
| A | -0.30 | -0.12 | 0.30 |
| A | -0.30 | -0.12 | 0.32 |
| A | -0.30 | -0.12 | 0.34 |
| A | -0.30 | -0.12 | 0.36 |
| A | -0.30 | -0.12 | 0.38 |
| A | -0.30 | -0.12 | 0.40 |
| A | -0.30 | -0.12 | 0.42 |
| A | -0.30 | -0.12 | 0.44 |
| A | -0.30 | -0.12 | 0.46 |
| A | -0.30 | -0.12 | 0.48 |
| A | -0.30 | -0.12 | 0.50 |
| A | -0.30 | -0.05 | 0.36 |
| A | -0.30 | -0.05 | 0.38 |
| A | -0.30 | -0.05 | 0.40 |
| A | -0.30 | -0.05 | 0.42 |
| A | -0.30 | -0.05 | 0.44 |
| A | -0.30 | -0.05 | 0.46 |
| A | -0.30 | -0.05 | 0.48 |
| A | -0.30 | -0.05 | 0.50 |
| A | -0.20 | -0.35 | 0.36 |
| A | -0.20 | -0.35 | 0.38 |
| A | -0.20 | -0.35 | 0.40 |
| A | -0.20 | -0.35 | 0.42 |
| A | -0.20 | -0.35 | 0.44 |
| A | -0.20 | -0.35 | 0.46 |
| A | -0.20 | -0.35 | 0.48 |
| A | -0.20 | -0.35 | 0.50 |
| A | -0.20 | -0.28 | 0.36 |
| A | -0.20 | -0.28 | 0.38 |
| A | -0.20 | -0.28 | 0.40 |
| A | -0.20 | -0.28 | 0.42 |
| A | -0.20 | -0.28 | 0.44 |
| A | -0.20 | -0.28 | 0.46 |
| A | -0.20 | -0.28 | 0.48 |
| A | -0.20 | -0.28 | 0.50 |
| A | -0.20 | -0.20 | 0.38 |
| A | -0.20 | -0.20 | 0.40 |
| A | -0.20 | -0.20 | 0.42 |
| A | -0.20 | -0.20 | 0.44 |
| A | -0.20 | -0.20 | 0.46 |
| A | -0.20 | -0.20 | 0.48 |
| A | -0.20 | -0.20 | 0.50 |
| A | -0.20 | -0.12 | 0.24 |
| A | -0.20 | -0.12 | 0.26 |
| A | -0.20 | -0.12 | 0.28 |
| A | -0.20 | -0.12 | 0.30 |
| A | -0.20 | -0.05 | 0.26 |
| A | -0.20 | -0.05 | 0.28 |
| A | -0.20 | -0.05 | 0.30 |
| A | -0.20 | -0.05 | 0.32 |
| A | -0.20 | -0.05 | 0.34 |
| A | -0.20 | -0.05 | 0.36 |
| A | -0.20 | -0.05 | 0.48 |
| A | -0.20 | -0.05 | 0.50 |
| A | -0.20 | 0.03 | 0.30 |
| A | -0.20 | 0.03 | 0.32 |
| A | -0.20 | 0.03 | 0.34 |
| A | -0.20 | 0.03 | 0.36 |
| A | -0.20 | 0.03 | 0.38 |
| A | -0.20 | 0.03 | 0.40 |
| A | -0.20 | 0.03 | 0.42 |
| A | -0.20 | 0.03 | 0.44 |
| A | -0.20 | 0.03 | 0.46 |
| A | -0.20 | 0.03 | 0.48 |
| A | -0.20 | 0.03 | 0.50 |
| A | -0.10 | -0.35 | 0.36 |
| A | -0.10 | -0.35 | 0.38 |
| A | -0.10 | 0.03 | 0.26 |
| A | -0.10 | 0.03 | 0.28 |
| A | -0.10 | 0.03 | 0.30 |
| A | -0.10 | 0.03 | 0.32 |
| A | -0.10 | 0.03 | 0.34 |
| A | -0.10 | 0.03 | 0.36 |
| A | -0.10 | 0.03 | 0.48 |
| A | -0.10 | 0.03 | 0.50 |
| A | -0.10 | 0.10 | 0.32 |
| A | -0.10 | 0.10 | 0.34 |
| A | -0.10 | 0.10 | 0.36 |
| A | -0.10 | 0.10 | 0.38 |
| A | -0.10 | 0.10 | 0.40 |
| A | -0.10 | 0.10 | 0.42 |
| A | -0.10 | 0.10 | 0.44 |
| A | -0.10 | 0.10 | 0.46 |
| A | -0.10 | 0.10 | 0.48 |
| A | -0.10 | 0.10 | 0.50 |
| A | 0.00 | 0.03 | 0.24 |
| A | 0.00 | 0.03 | 0.26 |
| A | 0.00 | 0.03 | 0.28 |
| A | 0.00 | 0.03 | 0.30 |
| A | 0.00 | 0.03 | 0.32 |
| A | 0.00 | 0.03 | 0.34 |
| A | 0.00 | 0.03 | 0.36 |
| A | 0.00 | 0.10 | 0.30 |
| A | 0.00 | 0.10 | 0.32 |
| A | 0.00 | 0.10 | 0.34 |
| A | 0.00 | 0.10 | 0.36 |
| A | 0.00 | 0.10 | 0.38 |
| A | 0.00 | 0.10 | 0.40 |
| A | 0.00 | 0.10 | 0.42 |
| A | 0.00 | 0.10 | 0.44 |
| A | 0.00 | 0.10 | 0.46 |
| A | 0.00 | 0.10 | 0.48 |
| A | 0.00 | 0.10 | 0.50 |
| A | 0.10 | -0.35 | 0.36 |
| A | 0.10 | -0.35 | 0.38 |
| A | 0.10 | 0.03 | 0.26 |
| A | 0.10 | 0.03 | 0.28 |
| A | 0.10 | 0.03 | 0.30 |
| A | 0.10 | 0.03 | 0.32 |
| A | 0.10 | 0.03 | 0.34 |
| A | 0.10 | 0.03 | 0.36 |
| A | 0.10 | 0.03 | 0.48 |
| A | 0.10 | 0.03 | 0.50 |
| A | 0.10 | 0.10 | 0.32 |
| A | 0.10 | 0.10 | 0.34 |
| A | 0.10 | 0.10 | 0.36 |
| A | 0.10 | 0.10 | 0.38 |
| A | 0.10 | 0.10 | 0.40 |
| A | 0.10 | 0.10 | 0.42 |
| A | 0.10 | 0.10 | 0.44 |
| A | 0.10 | 0.10 | 0.46 |
| A | 0.10 | 0.10 | 0.48 |
| A | 0.10 | 0.10 | 0.50 |
| A | 0.20 | -0.35 | 0.36 |
| A | 0.20 | -0.35 | 0.38 |
| A | 0.20 | -0.35 | 0.40 |
| A | 0.20 | -0.35 | 0.42 |
| A | 0.20 | -0.35 | 0.44 |
| A | 0.20 | -0.35 | 0.46 |
| A | 0.20 | -0.35 | 0.48 |
| A | 0.20 | -0.35 | 0.50 |
| A | 0.20 | -0.28 | 0.36 |
| A | 0.20 | -0.28 | 0.38 |
| A | 0.20 | -0.28 | 0.40 |
| A | 0.20 | -0.28 | 0.42 |
| A | 0.20 | -0.28 | 0.44 |
| A | 0.20 | -0.28 | 0.46 |
| A | 0.20 | -0.28 | 0.48 |
| A | 0.20 | -0.28 | 0.50 |
| A | 0.20 | -0.12 | 0.24 |
| A | 0.20 | -0.12 | 0.26 |
| A | 0.20 | -0.12 | 0.28 |
| A | 0.20 | -0.12 | 0.30 |
| A | 0.20 | -0.05 | 0.26 |
| A | 0.20 | -0.05 | 0.28 |
| A | 0.20 | -0.05 | 0.30 |
| A | 0.20 | -0.05 | 0.32 |
| A | 0.20 | -0.05 | 0.34 |
| A | 0.20 | -0.05 | 0.36 |
| A | 0.20 | -0.05 | 0.48 |
| A | 0.20 | -0.05 | 0.50 |
| A | 0.20 | 0.03 | 0.30 |
| A | 0.20 | 0.03 | 0.32 |
| A | 0.20 | 0.03 | 0.34 |
| A | 0.20 | 0.03 | 0.36 |
| A | 0.20 | 0.03 | 0.38 |
| A | 0.20 | 0.03 | 0.40 |
| A | 0.20 | 0.03 | 0.42 |
| A | 0.20 | 0.03 | 0.44 |
| A | 0.20 | 0.03 | 0.46 |
| A | 0.20 | 0.03 | 0.48 |
| A | 0.20 | 0.03 | 0.50 |
| A | 0.30 | -0.20 | 0.48 |
| A | 0.30 | -0.20 | 0.50 |
| A | 0.30 | -0.12 | 0.30 |
| A | 0.30 | -0.12 | 0.32 |
| A | 0.30 | -0.12 | 0.34 |
| A | 0.30 | -0.12 | 0.36 |
| A | 0.30 | -0.12 | 0.38 |
| A | 0.30 | -0.12 | 0.40 |
| A | 0.30 | -0.12 | 0.42 |
| A | 0.30 | -0.12 | 0.44 |
| A | 0.30 | -0.12 | 0.46 |
| A | 0.30 | -0.12 | 0.48 |
| A | 0.30 | -0.12 | 0.50 |
| A | 0.30 | -0.05 | 0.34 |
| A | 0.30 | -0.05 | 0.36 |
| A | 0.30 | -0.05 | 0.38 |
| A | 0.30 | -0.05 | 0.40 |
| A | 0.30 | -0.05 | 0.42 |
| A | 0.30 | -0.05 | 0.44 |
| A | 0.30 | -0.05 | 0.46 |
| A | 0.30 | -0.05 | 0.48 |
| A | 0.30 | -0.05 | 0.50 |
| B | -0.30 | 0.03 | 0.38 |
| B | -0.30 | 0.03 | 0.40 |
| B | -0.30 | 0.03 | 0.42 |
| B | -0.30 | 0.03 | 0.44 |
| B | -0.30 | 0.03 | 0.46 |
| B | -0.30 | 0.03 | 0.48 |
| B | -0.30 | 0.03 | 0.50 |
| B | -0.30 | 0.10 | 0.32 |
| B | -0.30 | 0.10 | 0.34 |
| B | -0.30 | 0.10 | 0.36 |
| B | -0.30 | 0.10 | 0.38 |
| B | -0.30 | 0.10 | 0.40 |
| B | -0.30 | 0.10 | 0.42 |
| B | -0.30 | 0.10 | 0.44 |
| B | -0.30 | 0.10 | 0.46 |
| B | -0.30 | 0.10 | 0.48 |
| B | -0.30 | 0.10 | 0.50 |
| B | -0.20 | -0.05 | 0.32 |
| B | -0.20 | -0.05 | 0.34 |
| B | -0.20 | -0.05 | 0.36 |
| B | -0.20 | -0.05 | 0.38 |
| B | -0.20 | -0.05 | 0.40 |
| B | -0.20 | -0.05 | 0.42 |
| B | -0.20 | -0.05 | 0.44 |
| B | -0.20 | -0.05 | 0.46 |
| B | -0.20 | -0.05 | 0.48 |
| B | -0.20 | -0.05 | 0.50 |
| B | -0.20 | 0.03 | 0.26 |
| B | -0.20 | 0.03 | 0.28 |
| B | -0.20 | 0.03 | 0.30 |
| B | -0.20 | 0.03 | 0.32 |
| B | -0.20 | 0.03 | 0.34 |
| B | -0.20 | 0.03 | 0.36 |
| B | -0.20 | 0.03 | 0.38 |
| B | -0.20 | 0.03 | 0.40 |
| B | -0.20 | 0.03 | 0.42 |
| B | -0.20 | 0.03 | 0.44 |
| B | -0.20 | 0.03 | 0.46 |
| B | -0.20 | 0.03 | 0.48 |
| B | -0.20 | 0.03 | 0.50 |
| B | -0.20 | 0.10 | 0.24 |
| B | -0.20 | 0.10 | 0.26 |
| B | -0.20 | 0.10 | 0.28 |
| B | -0.20 | 0.10 | 0.30 |
| B | -0.20 | 0.10 | 0.32 |
| B | -0.20 | 0.10 | 0.34 |
| B | -0.10 | -0.12 | 0.38 |
| B | -0.10 | -0.12 | 0.40 |
| B | -0.10 | -0.12 | 0.42 |
| B | -0.10 | -0.12 | 0.44 |
| B | -0.10 | -0.12 | 0.46 |
| B | -0.10 | -0.12 | 0.48 |
| B | -0.10 | -0.12 | 0.50 |
| B | -0.10 | -0.05 | 0.28 |
| B | -0.10 | -0.05 | 0.30 |
| B | -0.10 | -0.05 | 0.32 |
| B | -0.10 | -0.05 | 0.34 |
| B | -0.10 | -0.05 | 0.36 |
| B | -0.10 | -0.05 | 0.38 |
| B | -0.10 | -0.05 | 0.40 |
| B | -0.10 | -0.05 | 0.42 |
| B | -0.10 | -0.05 | 0.44 |
| B | -0.10 | -0.05 | 0.46 |
| B | -0.10 | -0.05 | 0.48 |
| B | -0.10 | -0.05 | 0.50 |
| B | -0.10 | 0.03 | 0.24 |
| B | -0.10 | 0.03 | 0.26 |
| B | -0.10 | 0.03 | 0.28 |
| B | 0.00 | -0.12 | 0.34 |
| B | 0.00 | -0.12 | 0.36 |
| B | 0.00 | -0.12 | 0.38 |
| B | 0.00 | -0.12 | 0.40 |
| B | 0.00 | -0.12 | 0.42 |
| B | 0.00 | -0.12 | 0.44 |
| B | 0.00 | -0.12 | 0.46 |
| B | 0.00 | -0.12 | 0.48 |
| B | 0.00 | -0.12 | 0.50 |
| B | 0.00 | -0.05 | 0.26 |
| B | 0.00 | -0.05 | 0.28 |
| B | 0.00 | -0.05 | 0.30 |
| B | 0.00 | -0.05 | 0.32 |
| B | 0.00 | -0.05 | 0.34 |
| B | 0.00 | -0.05 | 0.36 |
| B | 0.00 | -0.05 | 0.38 |
| B | 0.00 | -0.05 | 0.48 |
| B | 0.00 | -0.05 | 0.50 |
| B | 0.10 | -0.12 | 0.38 |
| B | 0.10 | -0.12 | 0.40 |
| B | 0.10 | -0.12 | 0.42 |
| B | 0.10 | -0.12 | 0.44 |
| B | 0.10 | -0.12 | 0.46 |
| B | 0.10 | -0.12 | 0.48 |
| B | 0.10 | -0.12 | 0.50 |
| B | 0.10 | -0.05 | 0.28 |
| B | 0.10 | -0.05 | 0.30 |
| B | 0.10 | -0.05 | 0.32 |
| B | 0.10 | -0.05 | 0.34 |
| B | 0.10 | -0.05 | 0.36 |
| B | 0.10 | -0.05 | 0.38 |
| B | 0.10 | -0.05 | 0.40 |
| B | 0.10 | -0.05 | 0.42 |
| B | 0.10 | -0.05 | 0.44 |
| B | 0.10 | -0.05 | 0.46 |
| B | 0.10 | -0.05 | 0.48 |
| B | 0.10 | -0.05 | 0.50 |
| B | 0.10 | 0.03 | 0.24 |
| B | 0.10 | 0.03 | 0.26 |
| B | 0.10 | 0.03 | 0.28 |
| B | 0.20 | -0.05 | 0.32 |
| B | 0.20 | -0.05 | 0.34 |
| B | 0.20 | -0.05 | 0.36 |
| B | 0.20 | -0.05 | 0.38 |
| B | 0.20 | -0.05 | 0.40 |
| B | 0.20 | -0.05 | 0.42 |
| B | 0.20 | -0.05 | 0.44 |
| B | 0.20 | -0.05 | 0.46 |
| B | 0.20 | -0.05 | 0.48 |
| B | 0.20 | -0.05 | 0.50 |
| B | 0.20 | 0.03 | 0.26 |
| B | 0.20 | 0.03 | 0.28 |
| B | 0.20 | 0.03 | 0.30 |
| B | 0.20 | 0.03 | 0.32 |
| B | 0.20 | 0.03 | 0.34 |
| B | 0.20 | 0.03 | 0.36 |
| B | 0.20 | 0.03 | 0.38 |
| B | 0.20 | 0.03 | 0.40 |
| B | 0.20 | 0.03 | 0.42 |
| B | 0.20 | 0.03 | 0.44 |
| B | 0.20 | 0.03 | 0.46 |
| B | 0.20 | 0.03 | 0.48 |
| B | 0.20 | 0.03 | 0.50 |
| B | 0.20 | 0.10 | 0.24 |
| B | 0.20 | 0.10 | 0.26 |
| B | 0.20 | 0.10 | 0.28 |
| B | 0.20 | 0.10 | 0.30 |
| B | 0.20 | 0.10 | 0.32 |
| B | 0.20 | 0.10 | 0.34 |
| B | 0.30 | 0.03 | 0.38 |
| B | 0.30 | 0.03 | 0.40 |
| B | 0.30 | 0.03 | 0.42 |
| B | 0.30 | 0.03 | 0.44 |
| B | 0.30 | 0.03 | 0.46 |
| B | 0.30 | 0.03 | 0.48 |
| B | 0.30 | 0.03 | 0.50 |
| B | 0.30 | 0.10 | 0.32 |
| B | 0.30 | 0.10 | 0.34 |
| B | 0.30 | 0.10 | 0.36 |
| B | 0.30 | 0.10 | 0.38 |
| B | 0.30 | 0.10 | 0.40 |
| B | 0.30 | 0.10 | 0.42 |
| B | 0.30 | 0.10 | 0.44 |
| B | 0.30 | 0.10 | 0.46 |
| B | 0.30 | 0.10 | 0.48 |
| B | 0.30 | 0.10 | 0.50 |


---

# M06 reachability probe, RE-MEASURED (ADR-026), Step 4: drawer repositioned

**This is the Step-4 final-validation run, AFTER `drawer_housing` was moved from
y=-0.17 (ADR-025, closed face y=-0.25) to y=0.08 (ADR-026, closed face y=0.0)
using the envelope measured in the section immediately above.** `closed_drawer_face`'s
target position below (0.0000, 0.0000, 0.2800) reflects the NEW drawer position;
`plate_at_rest`/`mug_at_rest`/`bottle_at_rest` are unchanged (those props did not move).

Seed: 0, reset via `TableSettingEnv.reset()`, which applies the "home" keyframe
(arms folded back) instead of leaving qpos at the upstream all-zero default.
PASS requires residual < 0.01 m and no NEW collision
(a contact count above this arm's measured RESET-pose baseline -- see 'Baseline finding' below).

Measured baseline (contacts involving each arm's geoms at reset(seed=0), before any IK solve, at the HOME POSE): arm A=0, arm B=0

## Primary targets

| target | target pos (x,y,z) | arm | residual (m) | converged | new collision | n_contacts (baseline) | PASS |
|---|---|---|---:|---|---|---:|---|
| closed_drawer_face | (0.0000, 0.0000, 0.2800) | A | 0.00914 | True | True | 6 (0) | FAIL |
| closed_drawer_face | (0.0000, 0.0000, 0.2800) | B | 0.00914 | True | True | 12 (0) | FAIL |
| plate_at_rest | (-0.1500, 0.0000, 0.3560) | A | 0.00898 | True | True | 16 (0) | FAIL |
| plate_at_rest | (-0.1500, 0.0000, 0.3560) | B | 0.00844 | True | True | 24 (0) | FAIL |
| mug_at_rest | (0.0500, -0.0300, 0.3900) | A | 0.12016 | False | True | 1 (0) | FAIL |
| mug_at_rest | (0.0500, -0.0300, 0.3900) | B | 0.03941 | False | True | 13 (0) | FAIL |
| bottle_at_rest | (0.2200, 0.0000, 0.4400) | A | 0.00819 | True | True | 6 (0) | FAIL |
| bottle_at_rest | (0.2200, 0.0000, 0.4400) | B | 0.00916 | True | True | 6 (0) | FAIL |

**Overall: AT LEAST ONE FAILURE.** `closed_drawer_face` and `bottle_at_rest`
now converge (residual < 0.01 m) for BOTH arms -- the kinematic envelope
fix worked. Every target still fails the COLLISION half of the PASS bar,
including `plate_at_rest` and `bottle_at_rest`, whose target positions did
not move at all in this pass. **STOPPING here per instruction ("if any
target still fails, STOP and report -- do not proceed and do not guess at
another position") rather than trying a fourth drawer position: see the
"Root cause of the remaining collision failures" section below, which
shows this is not a drawer-position problem.**

### Root cause of the remaining collision failures (not fixed here, out of this module's scope)

Inspecting the solved joint angles for the FAILING solves (e.g.
`plate_at_rest`, arm A: `shoulder_lift` solves to **1.7453 rad, its
positive joint-range limit**, `elbow_flex` to -0.19..-0.75 depending on
target) shows the Newton/DLS solver in `bimanual.control.ik.solve_position_ik`
converging to a configuration that swings the upper arm down and back
THROUGH the table rather than reaching over it -- confirmed directly via
the scratch `MjData`'s own contact list: `table_top` and `drawer_housing_back`
contacts at up to -0.049 m penetration are present for a solve whose
POSITION residual is a perfectly good 0.009 m. The IK solver
(ADR-024: position-only, no orientation control, no obstacle avoidance)
has no notion of the table's existence; it minimizes Cartesian pinch-point
error only, and the "home" keyframe's folded-back seed apparently lands
Newton's iteration in a different, worse local minimum than the old
fully-extended seed did for these same three prop targets (which is also
why the OLD `m06-reachability-probe.md` report above showed PASS for
`plate_at_rest`/`mug_at_rest`/`bottle_at_rest` -- **that PASS was never
trustworthy**: the old baseline-DELTA collision check compared a raw
contact COUNT against a baseline of ~29, so any solve producing fewer than
29 new contacts silently reported `collision=False` regardless of what
those contacts actually were. With the home-pose fix dropping the baseline
to 0, this masking is gone and the true collision state is now visible for
the first time -- it did not newly appear with this pass; it was already
there.

This is an `ik.py` solver limitation (redundant 5-DOF IK with no
table-avoidance term, ADR-024), independent of where any target sits
within the reachable envelope -- moving the drawer again would not fix it,
since `plate_at_rest` and `bottle_at_rest` (whose positions never changed
across this whole module) fail identically. Fixing it would mean changing
`src/bimanual/control/ik.py` (e.g. multi-start solving, a joint-limit- or
obstacle-aware cost term, or seeding from a target-specific initial guess
rather than always from "home"), which is out of this module's scope by
instruction ("Do NOT re-run the four skills -- that is a separate
follow-up invocation"). Flagged here, not fixed.

## Baseline finding, now fixed (was 'not fixed here' in the section above)

At `reset(seed=0)`, BEFORE any IK solve runs, the OLD default rest pose (all arm joints at 0 rad) put arm A and arm B up to ~6 cm deep into each other (29 of 34 total contacts were armA<->armB, deepest -0.0597 m, `armA_wrist` vs. `armB_wrist`). This was independent of the drawer position and the IK pinch-point retarget: it was a property of the compiled model's default qpos alone. **This is now fixed** by the "home" keyframe (ARCHITECTURE.md ADR-026): the measured baseline above, taken at the new reset pose, shows arm A=0, arm B=0 cross/self contacts. ADR-021's ~0.30 m reach / 0.50 m base-gap layout assumption was never checked against the actual compiled rest pose; this probe is that check, and the envelope below is the first one measured from a valid pose.

## Reachable envelope, re-measured from the home pose (always run this pass, per ADR-026 Step 2 -- not gated on primary-probe failure)

Grid: x in [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3], y in [-0.35, -0.275, -0.2, -0.125, -0.05, 0.025, 0.1], z in [0.2, 0.22, 0.24, 0.26, 0.28, 0.3, 0.32, 0.34, 0.36, 0.38, 0.4, 0.42, 0.44, 0.46, 0.48, 0.5] m steps=0.02 (residual < tolerance only; collision not checked for the sweep -- see script docstring).

- **Arm A**: 186 reachable grid points; x in [-0.30, 0.30], y in [-0.35, 0.10], z in [0.24, 0.50]
- **Arm B**: 154 reachable grid points; x in [-0.30, 0.30], y in [-0.12, 0.10], z in [0.24, 0.50]

**Shared handoff band (y-range intersection): y in [-0.12, 0.10] m.**

| arm | x | y | z |
|---|---:|---:|---:|
| A | -0.30 | -0.20 | 0.48 |
| A | -0.30 | -0.20 | 0.50 |
| A | -0.30 | -0.12 | 0.30 |
| A | -0.30 | -0.12 | 0.32 |
| A | -0.30 | -0.12 | 0.34 |
| A | -0.30 | -0.12 | 0.36 |
| A | -0.30 | -0.12 | 0.38 |
| A | -0.30 | -0.12 | 0.40 |
| A | -0.30 | -0.12 | 0.42 |
| A | -0.30 | -0.12 | 0.44 |
| A | -0.30 | -0.12 | 0.46 |
| A | -0.30 | -0.12 | 0.48 |
| A | -0.30 | -0.12 | 0.50 |
| A | -0.30 | -0.05 | 0.36 |
| A | -0.30 | -0.05 | 0.38 |
| A | -0.30 | -0.05 | 0.40 |
| A | -0.30 | -0.05 | 0.42 |
| A | -0.30 | -0.05 | 0.44 |
| A | -0.30 | -0.05 | 0.46 |
| A | -0.30 | -0.05 | 0.48 |
| A | -0.30 | -0.05 | 0.50 |
| A | -0.20 | -0.35 | 0.36 |
| A | -0.20 | -0.35 | 0.38 |
| A | -0.20 | -0.35 | 0.40 |
| A | -0.20 | -0.35 | 0.42 |
| A | -0.20 | -0.35 | 0.44 |
| A | -0.20 | -0.35 | 0.46 |
| A | -0.20 | -0.35 | 0.48 |
| A | -0.20 | -0.35 | 0.50 |
| A | -0.20 | -0.28 | 0.36 |
| A | -0.20 | -0.28 | 0.38 |
| A | -0.20 | -0.28 | 0.40 |
| A | -0.20 | -0.28 | 0.42 |
| A | -0.20 | -0.28 | 0.44 |
| A | -0.20 | -0.28 | 0.46 |
| A | -0.20 | -0.28 | 0.48 |
| A | -0.20 | -0.28 | 0.50 |
| A | -0.20 | -0.20 | 0.38 |
| A | -0.20 | -0.20 | 0.40 |
| A | -0.20 | -0.20 | 0.42 |
| A | -0.20 | -0.20 | 0.44 |
| A | -0.20 | -0.20 | 0.46 |
| A | -0.20 | -0.20 | 0.48 |
| A | -0.20 | -0.20 | 0.50 |
| A | -0.20 | -0.12 | 0.24 |
| A | -0.20 | -0.12 | 0.26 |
| A | -0.20 | -0.12 | 0.28 |
| A | -0.20 | -0.12 | 0.30 |
| A | -0.20 | -0.05 | 0.26 |
| A | -0.20 | -0.05 | 0.28 |
| A | -0.20 | -0.05 | 0.30 |
| A | -0.20 | -0.05 | 0.32 |
| A | -0.20 | -0.05 | 0.34 |
| A | -0.20 | -0.05 | 0.36 |
| A | -0.20 | -0.05 | 0.48 |
| A | -0.20 | -0.05 | 0.50 |
| A | -0.20 | 0.03 | 0.30 |
| A | -0.20 | 0.03 | 0.32 |
| A | -0.20 | 0.03 | 0.34 |
| A | -0.20 | 0.03 | 0.36 |
| A | -0.20 | 0.03 | 0.38 |
| A | -0.20 | 0.03 | 0.40 |
| A | -0.20 | 0.03 | 0.42 |
| A | -0.20 | 0.03 | 0.44 |
| A | -0.20 | 0.03 | 0.46 |
| A | -0.20 | 0.03 | 0.48 |
| A | -0.20 | 0.03 | 0.50 |
| A | -0.10 | -0.35 | 0.36 |
| A | -0.10 | -0.35 | 0.38 |
| A | -0.10 | 0.03 | 0.26 |
| A | -0.10 | 0.03 | 0.28 |
| A | -0.10 | 0.03 | 0.30 |
| A | -0.10 | 0.03 | 0.32 |
| A | -0.10 | 0.03 | 0.34 |
| A | -0.10 | 0.03 | 0.36 |
| A | -0.10 | 0.03 | 0.48 |
| A | -0.10 | 0.03 | 0.50 |
| A | -0.10 | 0.10 | 0.32 |
| A | -0.10 | 0.10 | 0.34 |
| A | -0.10 | 0.10 | 0.36 |
| A | -0.10 | 0.10 | 0.38 |
| A | -0.10 | 0.10 | 0.40 |
| A | -0.10 | 0.10 | 0.42 |
| A | -0.10 | 0.10 | 0.44 |
| A | -0.10 | 0.10 | 0.46 |
| A | -0.10 | 0.10 | 0.48 |
| A | -0.10 | 0.10 | 0.50 |
| A | 0.00 | 0.03 | 0.24 |
| A | 0.00 | 0.03 | 0.26 |
| A | 0.00 | 0.03 | 0.28 |
| A | 0.00 | 0.03 | 0.30 |
| A | 0.00 | 0.03 | 0.32 |
| A | 0.00 | 0.03 | 0.34 |
| A | 0.00 | 0.03 | 0.36 |
| A | 0.00 | 0.10 | 0.30 |
| A | 0.00 | 0.10 | 0.32 |
| A | 0.00 | 0.10 | 0.34 |
| A | 0.00 | 0.10 | 0.36 |
| A | 0.00 | 0.10 | 0.38 |
| A | 0.00 | 0.10 | 0.40 |
| A | 0.00 | 0.10 | 0.42 |
| A | 0.00 | 0.10 | 0.44 |
| A | 0.00 | 0.10 | 0.46 |
| A | 0.00 | 0.10 | 0.48 |
| A | 0.00 | 0.10 | 0.50 |
| A | 0.10 | -0.35 | 0.36 |
| A | 0.10 | -0.35 | 0.38 |
| A | 0.10 | 0.03 | 0.26 |
| A | 0.10 | 0.03 | 0.28 |
| A | 0.10 | 0.03 | 0.30 |
| A | 0.10 | 0.03 | 0.32 |
| A | 0.10 | 0.03 | 0.34 |
| A | 0.10 | 0.03 | 0.36 |
| A | 0.10 | 0.03 | 0.48 |
| A | 0.10 | 0.03 | 0.50 |
| A | 0.10 | 0.10 | 0.32 |
| A | 0.10 | 0.10 | 0.34 |
| A | 0.10 | 0.10 | 0.36 |
| A | 0.10 | 0.10 | 0.38 |
| A | 0.10 | 0.10 | 0.40 |
| A | 0.10 | 0.10 | 0.42 |
| A | 0.10 | 0.10 | 0.44 |
| A | 0.10 | 0.10 | 0.46 |
| A | 0.10 | 0.10 | 0.48 |
| A | 0.10 | 0.10 | 0.50 |
| A | 0.20 | -0.35 | 0.36 |
| A | 0.20 | -0.35 | 0.38 |
| A | 0.20 | -0.35 | 0.40 |
| A | 0.20 | -0.35 | 0.42 |
| A | 0.20 | -0.35 | 0.44 |
| A | 0.20 | -0.35 | 0.46 |
| A | 0.20 | -0.35 | 0.48 |
| A | 0.20 | -0.35 | 0.50 |
| A | 0.20 | -0.28 | 0.36 |
| A | 0.20 | -0.28 | 0.38 |
| A | 0.20 | -0.28 | 0.40 |
| A | 0.20 | -0.28 | 0.42 |
| A | 0.20 | -0.28 | 0.44 |
| A | 0.20 | -0.28 | 0.46 |
| A | 0.20 | -0.28 | 0.48 |
| A | 0.20 | -0.28 | 0.50 |
| A | 0.20 | -0.12 | 0.24 |
| A | 0.20 | -0.12 | 0.26 |
| A | 0.20 | -0.12 | 0.28 |
| A | 0.20 | -0.12 | 0.30 |
| A | 0.20 | -0.05 | 0.26 |
| A | 0.20 | -0.05 | 0.28 |
| A | 0.20 | -0.05 | 0.30 |
| A | 0.20 | -0.05 | 0.32 |
| A | 0.20 | -0.05 | 0.34 |
| A | 0.20 | -0.05 | 0.36 |
| A | 0.20 | -0.05 | 0.48 |
| A | 0.20 | -0.05 | 0.50 |
| A | 0.20 | 0.03 | 0.30 |
| A | 0.20 | 0.03 | 0.32 |
| A | 0.20 | 0.03 | 0.34 |
| A | 0.20 | 0.03 | 0.36 |
| A | 0.20 | 0.03 | 0.38 |
| A | 0.20 | 0.03 | 0.40 |
| A | 0.20 | 0.03 | 0.42 |
| A | 0.20 | 0.03 | 0.44 |
| A | 0.20 | 0.03 | 0.46 |
| A | 0.20 | 0.03 | 0.48 |
| A | 0.20 | 0.03 | 0.50 |
| A | 0.30 | -0.20 | 0.48 |
| A | 0.30 | -0.20 | 0.50 |
| A | 0.30 | -0.12 | 0.30 |
| A | 0.30 | -0.12 | 0.32 |
| A | 0.30 | -0.12 | 0.34 |
| A | 0.30 | -0.12 | 0.36 |
| A | 0.30 | -0.12 | 0.38 |
| A | 0.30 | -0.12 | 0.40 |
| A | 0.30 | -0.12 | 0.42 |
| A | 0.30 | -0.12 | 0.44 |
| A | 0.30 | -0.12 | 0.46 |
| A | 0.30 | -0.12 | 0.48 |
| A | 0.30 | -0.12 | 0.50 |
| A | 0.30 | -0.05 | 0.34 |
| A | 0.30 | -0.05 | 0.36 |
| A | 0.30 | -0.05 | 0.38 |
| A | 0.30 | -0.05 | 0.40 |
| A | 0.30 | -0.05 | 0.42 |
| A | 0.30 | -0.05 | 0.44 |
| A | 0.30 | -0.05 | 0.46 |
| A | 0.30 | -0.05 | 0.48 |
| A | 0.30 | -0.05 | 0.50 |
| B | -0.30 | 0.03 | 0.38 |
| B | -0.30 | 0.03 | 0.40 |
| B | -0.30 | 0.03 | 0.42 |
| B | -0.30 | 0.03 | 0.44 |
| B | -0.30 | 0.03 | 0.46 |
| B | -0.30 | 0.03 | 0.48 |
| B | -0.30 | 0.03 | 0.50 |
| B | -0.30 | 0.10 | 0.32 |
| B | -0.30 | 0.10 | 0.34 |
| B | -0.30 | 0.10 | 0.36 |
| B | -0.30 | 0.10 | 0.38 |
| B | -0.30 | 0.10 | 0.40 |
| B | -0.30 | 0.10 | 0.42 |
| B | -0.30 | 0.10 | 0.44 |
| B | -0.30 | 0.10 | 0.46 |
| B | -0.30 | 0.10 | 0.48 |
| B | -0.30 | 0.10 | 0.50 |
| B | -0.20 | -0.05 | 0.32 |
| B | -0.20 | -0.05 | 0.34 |
| B | -0.20 | -0.05 | 0.36 |
| B | -0.20 | -0.05 | 0.38 |
| B | -0.20 | -0.05 | 0.40 |
| B | -0.20 | -0.05 | 0.42 |
| B | -0.20 | -0.05 | 0.44 |
| B | -0.20 | -0.05 | 0.46 |
| B | -0.20 | -0.05 | 0.48 |
| B | -0.20 | -0.05 | 0.50 |
| B | -0.20 | 0.03 | 0.26 |
| B | -0.20 | 0.03 | 0.28 |
| B | -0.20 | 0.03 | 0.30 |
| B | -0.20 | 0.03 | 0.32 |
| B | -0.20 | 0.03 | 0.34 |
| B | -0.20 | 0.03 | 0.36 |
| B | -0.20 | 0.03 | 0.38 |
| B | -0.20 | 0.03 | 0.40 |
| B | -0.20 | 0.03 | 0.42 |
| B | -0.20 | 0.03 | 0.44 |
| B | -0.20 | 0.03 | 0.46 |
| B | -0.20 | 0.03 | 0.48 |
| B | -0.20 | 0.03 | 0.50 |
| B | -0.20 | 0.10 | 0.24 |
| B | -0.20 | 0.10 | 0.26 |
| B | -0.20 | 0.10 | 0.28 |
| B | -0.20 | 0.10 | 0.30 |
| B | -0.20 | 0.10 | 0.32 |
| B | -0.20 | 0.10 | 0.34 |
| B | -0.10 | -0.12 | 0.38 |
| B | -0.10 | -0.12 | 0.40 |
| B | -0.10 | -0.12 | 0.42 |
| B | -0.10 | -0.12 | 0.44 |
| B | -0.10 | -0.12 | 0.46 |
| B | -0.10 | -0.12 | 0.48 |
| B | -0.10 | -0.12 | 0.50 |
| B | -0.10 | -0.05 | 0.28 |
| B | -0.10 | -0.05 | 0.30 |
| B | -0.10 | -0.05 | 0.32 |
| B | -0.10 | -0.05 | 0.34 |
| B | -0.10 | -0.05 | 0.36 |
| B | -0.10 | -0.05 | 0.38 |
| B | -0.10 | -0.05 | 0.40 |
| B | -0.10 | -0.05 | 0.42 |
| B | -0.10 | -0.05 | 0.44 |
| B | -0.10 | -0.05 | 0.46 |
| B | -0.10 | -0.05 | 0.48 |
| B | -0.10 | -0.05 | 0.50 |
| B | -0.10 | 0.03 | 0.24 |
| B | -0.10 | 0.03 | 0.26 |
| B | -0.10 | 0.03 | 0.28 |
| B | 0.00 | -0.12 | 0.34 |
| B | 0.00 | -0.12 | 0.36 |
| B | 0.00 | -0.12 | 0.38 |
| B | 0.00 | -0.12 | 0.40 |
| B | 0.00 | -0.12 | 0.42 |
| B | 0.00 | -0.12 | 0.44 |
| B | 0.00 | -0.12 | 0.46 |
| B | 0.00 | -0.12 | 0.48 |
| B | 0.00 | -0.12 | 0.50 |
| B | 0.00 | -0.05 | 0.26 |
| B | 0.00 | -0.05 | 0.28 |
| B | 0.00 | -0.05 | 0.30 |
| B | 0.00 | -0.05 | 0.32 |
| B | 0.00 | -0.05 | 0.34 |
| B | 0.00 | -0.05 | 0.36 |
| B | 0.00 | -0.05 | 0.38 |
| B | 0.00 | -0.05 | 0.48 |
| B | 0.00 | -0.05 | 0.50 |
| B | 0.10 | -0.12 | 0.38 |
| B | 0.10 | -0.12 | 0.40 |
| B | 0.10 | -0.12 | 0.42 |
| B | 0.10 | -0.12 | 0.44 |
| B | 0.10 | -0.12 | 0.46 |
| B | 0.10 | -0.12 | 0.48 |
| B | 0.10 | -0.12 | 0.50 |
| B | 0.10 | -0.05 | 0.28 |
| B | 0.10 | -0.05 | 0.30 |
| B | 0.10 | -0.05 | 0.32 |
| B | 0.10 | -0.05 | 0.34 |
| B | 0.10 | -0.05 | 0.36 |
| B | 0.10 | -0.05 | 0.38 |
| B | 0.10 | -0.05 | 0.40 |
| B | 0.10 | -0.05 | 0.42 |
| B | 0.10 | -0.05 | 0.44 |
| B | 0.10 | -0.05 | 0.46 |
| B | 0.10 | -0.05 | 0.48 |
| B | 0.10 | -0.05 | 0.50 |
| B | 0.10 | 0.03 | 0.24 |
| B | 0.10 | 0.03 | 0.26 |
| B | 0.10 | 0.03 | 0.28 |
| B | 0.20 | -0.05 | 0.32 |
| B | 0.20 | -0.05 | 0.34 |
| B | 0.20 | -0.05 | 0.36 |
| B | 0.20 | -0.05 | 0.38 |
| B | 0.20 | -0.05 | 0.40 |
| B | 0.20 | -0.05 | 0.42 |
| B | 0.20 | -0.05 | 0.44 |
| B | 0.20 | -0.05 | 0.46 |
| B | 0.20 | -0.05 | 0.48 |
| B | 0.20 | -0.05 | 0.50 |
| B | 0.20 | 0.03 | 0.26 |
| B | 0.20 | 0.03 | 0.28 |
| B | 0.20 | 0.03 | 0.30 |
| B | 0.20 | 0.03 | 0.32 |
| B | 0.20 | 0.03 | 0.34 |
| B | 0.20 | 0.03 | 0.36 |
| B | 0.20 | 0.03 | 0.38 |
| B | 0.20 | 0.03 | 0.40 |
| B | 0.20 | 0.03 | 0.42 |
| B | 0.20 | 0.03 | 0.44 |
| B | 0.20 | 0.03 | 0.46 |
| B | 0.20 | 0.03 | 0.48 |
| B | 0.20 | 0.03 | 0.50 |
| B | 0.20 | 0.10 | 0.24 |
| B | 0.20 | 0.10 | 0.26 |
| B | 0.20 | 0.10 | 0.28 |
| B | 0.20 | 0.10 | 0.30 |
| B | 0.20 | 0.10 | 0.32 |
| B | 0.20 | 0.10 | 0.34 |
| B | 0.30 | 0.03 | 0.38 |
| B | 0.30 | 0.03 | 0.40 |
| B | 0.30 | 0.03 | 0.42 |
| B | 0.30 | 0.03 | 0.44 |
| B | 0.30 | 0.03 | 0.46 |
| B | 0.30 | 0.03 | 0.48 |
| B | 0.30 | 0.03 | 0.50 |
| B | 0.30 | 0.10 | 0.32 |
| B | 0.30 | 0.10 | 0.34 |
| B | 0.30 | 0.10 | 0.36 |
| B | 0.30 | 0.10 | 0.38 |
| B | 0.30 | 0.10 | 0.40 |
| B | 0.30 | 0.10 | 0.42 |
| B | 0.30 | 0.10 | 0.44 |
| B | 0.30 | 0.10 | 0.46 |
| B | 0.30 | 0.10 | 0.48 |
| B | 0.30 | 0.10 | 0.50 |

