# Master scored against v2's five criteria — the baseline

Measured by the orchestrator with an independent monitor
(`scripts/v2_criteria_monitor.py`), so the same instrument scores both stacks.
`skills_scripted.py` is unmodified on this branch, so this is master's real
behaviour, at seed 0.

The monitor wraps `env.step` as a read-only observer: it never injects a step,
never writes to `data`, and never changes control, timing or step counts.

## Result: all three skills FAIL — and all three pass criterion (a)

| skill | steps | (a) outcome | (b) props ≤5 mm | (c) no arm-prop contact | (d) no cross-arm | (e) peak vel < 2.6 |
|---|---|---|---|---|---|---|
| `pick(A, fork)` | 1655 | **OK** | OK | OK | OK | **BAD** 6.937 rad/s |
| `place(A, fork, table)` | 3455 | **OK** | **BAD** | **BAD** | OK | **BAD** 6.937 rad/s |
| `handoff(A→B, fork)` | 6610 | **OK** | **BAD** | **BAD** | **BAD** | **BAD** 6.937 rad/s |

Every skill satisfies its own target outcome and still fails the scene. That is
the whole case for the v2 redesign, in one table: master's success definition
measured only (a).

## The numbers behind the failures

**`place(A, fork, table)`**
- (b) `mug` displaced **0.0556 m**, `spoon` displaced **0.0288 m** (gate 5 mm)
- (c) `mug` penetrated **-0.0044 m**, `spoon` **-0.0022 m** (gate 1 mm)

**`handoff(A→B, fork)`**
- (b) `mug` displaced **0.0482 m**
- (c) `mug` penetrated **-0.0042 m**
- (d) **3185 steps** with cross-arm contact, worst penetration **-0.0109 m**

That last line is worth stating plainly: master's handoff spends **3185 of its
6610 steps (48%) in cross-arm contact**, 1.1 cm deep at worst, and still
reports success. ADR-054's Phase 3 collision was not an edge case; sustained
arm-on-arm contact is the normal operating condition of that skill.

**All three**: peak joint velocity **6.937 rad/s**, against the 2.6 rad/s gate
— and higher than the 5.350 rad/s measured for an isolated one-shot direct
position command (Stage 2, ADR-071). This is the direct, quantified cause of
what is visible in `docs/videos/full-sequence-demo.mp4` on master: the mug
knocked onto its side and the water bottle knocked off the table. Those were
disclosed in that commit as real; here is the mechanism, with a number.

## The (e) gate derivation

Peak velocity of a cubic with zero endpoint velocities is `1.5 * delta / T`.
The largest single-joint displacement any phase legitimately commands is the
`wrist_roll` range, ~3.4 rad (limits -2.7438 to +2.8412); at `T = 2.0 s` that
implies **2.55 rad/s**. The gate is set at **2.6 rad/s**: above every
legitimate commanded move, so a large sweep is not failed merely for being
large, and far below both the 5.350 rad/s direct-command baseline and the
6.937 rad/s master actually reaches.

## Caveat

Run on the laptop, not bm-ptl. ADR-047's cross-machine float divergence means
the low-order digits will differ there. The failures are not marginal —
0.0556 m against a 0.005 m gate, 6.937 against 2.6 — so the verdicts are not
at risk, but re-run on bm-ptl before these exact figures go into a final
comparison table.
