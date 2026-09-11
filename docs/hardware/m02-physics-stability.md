# M02 physics stability probe

Verdict: **PASS**

- Seed: 0
- Steps requested: 1000
- Steps completed before stopping: 1000
- Action: zero action vector (length 12) every step
- Floor threshold (tunneling check): FLOOR_Z = 0.3 m
  - Justification: tabletop surface is at z=0.35 (box geom center z=0.34, half-thickness 0.01, src/bimanual/sim/assets/so101_dual_table.xml "table_top"). Normal contact settling loses at most ~1-2 mm of height, never 5 cm. FLOOR_Z=0.30 is 5 cm below the tabletop surface: unreachable by settling, but well above the floor (z=0) or other sub-table geometry a tunneled prop would land on.

## NaN check
No NaN observed in qpos or qvel at any completed step.

## Tunneling check
No free body's z-coordinate dropped below the floor threshold.

## Minimum z observed per free body (over completed steps)

| body | min z (m) |
|---|---|
| plate | 0.34990 |
| mug | 0.38810 |
| fork | 0.35333 |
| spoon | 0.35272 |
| water_bottle | 0.43944 |

