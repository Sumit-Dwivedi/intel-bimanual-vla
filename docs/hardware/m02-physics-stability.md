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

**Caveat:** this probe catches interpenetration only *indirectly*, via the floor
check (a large separation impulse launches a prop, which then falls and trips
`FLOOR_Z`) — it never asserts on `data.contact.dist` directly; acceptable for
verifying this static scene, to be revisited if M06's controller tests reveal
contact anomalies that an indirect check would miss.

**Re-run 2026-09-12 (ADR-025, drawer housing moved from y=-0.05 to y=-0.17):**
re-ran unchanged after the drawer reposition; verdict is still **PASS** with
the same numbers above. Kept here rather than re-appending, since nothing
about this probe's own findings changed -- only the scene's drawer geometry
did, and this probe does not touch the drawer.

**Re-run 2026-09-12 (ADR-026, "home" keyframe added, `reset()` now applies
it):** re-ran after `TableSettingEnv.reset()` was changed to apply the new
"home" keyframe (arms folded back) instead of leaving qpos at the upstream
all-zero default. Verdict is still **PASS**, with the identical numbers
above (this probe's own script always overwrites this file on each run --
the numbers were regenerated, not hand-copied, and happen to be unchanged
because none of the five free-body props' initial pose changed). **This
re-run is exactly the caveat above arriving late:** this probe never
asserted on `data.contact.dist` directly, so it never caught -- and could
not have caught -- the 34-contact, 29-cross-arm, -0.0597 m interpenetration
that existed at the OLD default rest pose for the entire time this file
said PASS. The floor/NaN check this probe performs and the cross-arm
contact check ADR-026 fixed are different properties; a scene can pass
this probe while two of its own bodies interpenetrate by 6 cm, because nothing
here ever launched a prop or produced a NaN. See ARCHITECTURE.md ADR-026 and
`docs/hardware/m06-reachability-probe.md` for the measurement that this
caveat predicted.

**Re-run 2026-09-12 (ADR-026, drawer_housing repositioned from y=-0.17 to
y=0.08):** re-ran again after the drawer was moved a second time (this pass's
Step 3, to land inside the newly-measured reachable envelope). Verdict is
still **PASS**, numbers unchanged. This probe does not touch the drawer's
position and would not catch a bad drawer placement either way -- see
`docs/hardware/m06-reachability-probe.md` for the reachability/collision
checks that actually cover this scene edit.
