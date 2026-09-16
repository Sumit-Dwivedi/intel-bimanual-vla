# Submission path — decision and reasoning

Disposition of the `redesign-v2` branch with respect to the submission.
Recorded here rather than decided silently. `master` is frozen at `238cfed`.

**Decision: Path Y, with the framing inverted — `master` stays the functional
baseline and v2 is added as the measured rebuild, not as a replacement.**
Reasoning below, including what it costs and what could go wrong.

## What each path actually ships

**Path X — ship `master` unchanged.** Four skills reporting success, the
existing README, `SUBMISSION.md`, and `docs/videos/full-sequence-demo.mp4`.
v2 exists on a branch as a rigour item a reader may or may not find.

**Path Y — `master` plus v2 as the primary quality evidence.**
`skills_scripted.py` and its four skills stay in the tree unchanged; v2 lives
alongside, opt-in behind `ScriptedSkillExecutor(use_v2=True)`. `SUBMISSION.md`
and `README.md` gain a section pointing at `docs/videos/v2-relay-demo.mp4`,
`docs/hardware/v2-verdict.md` and ADR-074. The pre-v2 video stays linked as
the baseline.

## The case against Path X, which is the deciding factor

Master's headline demo video **shows the system destroying the table**. The mug
is knocked onto its side; the water bottle is knocked off entirely. This is
already disclosed honestly in that commit and in `SUBMISSION.md` — but
disclosure does not change what a judge watches. And it is not cosmetic: the
measured cause is a peak joint velocity of **6.937 rad/s**, higher than an
isolated one-shot direct position command produces, with master's handoff
spending **3179 of 6610 steps (48%) with the two arms inside each other**.

Path X therefore ships a submission whose most-watched artefact is its worst
result, while the branch that measured and fixed that failure sits unreferenced.
That is a poor trade even before the rubric is considered.

## The case for keeping `master` anyway

v2 does **not** replace master's four skills, and the temptation to present it
that way should be resisted:

- **v2 has no working direct handoff.** Master's `handoff(A→B, fork)`
  completes. v2's does not, and "End-to-end task completion & bimanual" is the
  rubric's largest line at 30 points. Leading with v2 would trade the
  strongest rubric claim for a cleaner-looking video.
- **v2 cannot reach the water bottle at all**, where master reached it 9/20.
  That is a real capability regression, the direct cost of spending the arm's
  redundancy on orientation control.
- **v2 is newer and less exercised.** Master's four skills have a 20-seed
  robustness record (ADR-053); v2 has seed 0 and a six-stage relay.

So the honest arrangement is not "v2 supersedes master". It is: **master is
what the system does; v2 is what we learned about how well it was doing it.**

## Why Path Y is worth the extra work

Three of the rubric lines are served by material that only exists because of
v2, and none of it requires demoting master:

- **Innovation (5).** Three negative results already existed (ADR-050 INT8,
  ADR-057 multi-seed IK, the v1 `redesign` branch). ADR-074 adds a fourth that
  is stronger than all of them, because it is the only one that produced a
  *working alternative* alongside the negative finding.
- **Rigour / reproducibility (10).** The comparison in ADR-074 is the rare
  kind: **both stacks measured on one read-only instrument**, master scored
  retroactively on criteria it was never designed against. Master 0 of 3, v2
  2 of 3, with per-criterion numbers.
- **End-to-end + bimanual (30).** Master's handoff remains the claim. v2's
  relay *adds* a second, independent bimanual demonstration in which nothing
  on the table moves — strengthening the line rather than replacing it.

The part most likely to be credited is not the rebuild itself but **the two
shortcuts that were refused**: widening the grasp gate from 0.05 to 0.12 m, and
widening the weld gate to 0.115 m. Both would have converted failures into
passing numbers; both were tested, measured, and rejected — the second after a
per-step contact check proved arm B's pads never touch the fork. A submission
that can point at a passing number it declined to take is making a claim about
its own reliability that few can.

## Cost and risk, stated plainly

Roughly two hours: `SUBMISSION.md` edits, `README.md` edits, a refreshed rubric
evidence map, and a dry-run of `run_demo.py` to confirm nothing regressed.

The real risk is **not** technical — `use_v2` defaults to `False`, and
`pytest tests/test_skills.py` is unchanged at 4 passed / 4 failed, so master's
behaviour is untouched by construction. The risk is **editorial**: every past
error on this project has come from documentation claiming slightly more than
the measurements support. The v2 material is full of numbers that are easy to
quote wrongly — the retracted "pick passes all five", the tautological "~900x
tracking", the mis-specified drift gate, the handoff that "passes" with a wide
enough gate. Any Stage 6 must treat ADR-074's caveats as binding, not as
background.

Mitigation: Stage 6 changes documentation only. No code, no scene, no skill
logic. `master`'s four-skill demo path stays the default everywhere.

## If the time is not there

Path X remains defensible and is not a failure. The fallback, in descending
order of value per minute spent:

1. One paragraph in `SUBMISSION.md` pointing at `v2-verdict.md` and the relay
   video. **~10 minutes, and it captures most of the Innovation/Rigour value.**
2. Swap the README's front-page video to the relay, keeping the pre-v2 one
   linked beneath as the baseline. ~20 minutes.
3. The full rubric-map refresh. The remainder.

Item 1 alone is worth doing even if nothing else is.

## Decision

**Path Y, framed as `master` + v2-as-rebuild.** Not started here: per the
governing brief, Stage 6 gets its own bounded prompt. This document is the
record of the disposition, and nothing in `master` has been touched.
