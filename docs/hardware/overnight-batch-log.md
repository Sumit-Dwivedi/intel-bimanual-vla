# Overnight batch log

Findings and process notes from the overnight fix batch (Fixes A-F,
ADR-053 through ADR-057 per the batch brief's corrected numbering — see
"ADR numbering" below) that would otherwise have been written directly
into `SUBMISSION.md`. The batch's own standing rule is **"DO NOT modify
`SUBMISSION.md` until morning-user reviews"**, and that rule governs every
fix in the batch, including any individual fix's own instructions that say
otherwise. This file exists so nothing found overnight is lost before that
review happens — morning-user should read this file and fold anything
relevant into `SUBMISSION.md` by hand.

---

## Fix B — M08 extended: 20-seed Track A robustness sweep (ADR-053)

**What changed.** `scripts/eval_m08.py` (ADR-049) gained a
backward-compatible `--num-seeds` argument (default 10, unchanged) and a
per-trial thread-based timeout, then ran Track A (own-prop randomization)
across seeds 0-19 for all four working skills. Full results, tables, and
the reproducibility check in `docs/hardware/m08-extended-eval.md`; ADR
mirror in `DECISIONS.md`/`ARCHITECTURE.md` (ADR-053).

**Findings that belong in `SUBMISSION.md`'s eventual robustness section,
once morning-user reviews this log:**

1. **Reproducibility confirmed, not merely assumed.** Seeds 0-9 inside the
   new 20-seed run reproduce ADR-049's original 10-seed numbers
   bit-for-bit on all four skills — same offsets, same pass/fail pattern,
   same frame counts, zero exceptions. This was checked explicitly per the
   batch brief's instruction ("the genuinely interesting question"), not
   asserted. No reproducibility failure occurred.
2. **`pick(A, 'bottle')`'s reported success rate should change from 60%
   (6/10, ADR-049) to 45% (9/20, ADR-053) if this extended sweep is cited
   anywhere in the submission.** The 10-seed figure was not wrong, but it
   sat on the better half of this skill's tolerance rectangle more often
   than the following ten seeds did; the 20-seed figure is the more
   reliable estimate and should supersede it, per ADR-018's standing rule
   against reporting a favourable subset as the whole picture.
3. **`handoff`'s 20/20 must never be quoted alone.** Its own Track A
   envelope is a single point (`docs/hardware/m07-envelopes.md`'s Task 1),
   so all 20 trials are the byte-identical unperturbed scenario — this
   measures determinism, not robustness. Three independent, already-
   ratified measurements say `handoff` is in fact the LEAST tolerant skill
   in the repo to any perturbation at all: ADR-049 Track B (0/10 when a
   prop it never touches is randomized), ADR-051 (0/5 at the smallest
   tested home-pose arm-angle noise), ADR-046 (fails at a 2.2 mm
   perception offset). If `SUBMISSION.md`'s rubric-evidence section (Fix C)
   or any other section cites `handoff`'s seed count, it must carry this
   qualifier every time, not once.
4. **Both regression gates (`pytest tests/test_skills.py`, `scripts/
   verify_adr038_skills.py`) reproduced byte-identical results before and
   after this sweep** — `4 passed / 4 failed` and `0.3989 / 0.3588 / 0.6192
   / 0.1946` respectively, matching every prior ADR in this chain. No skill,
   grasp, IK, executor, environment or randomization code was touched.

**ADR numbering.** The batch brief numbered this fix's ADR as 054, but
ADR-052 was the highest ratified number at the start of this batch and Fix
E (M10 batch scaling) had already taken it before this fix started. Per
the corrected sequence given for this fix (**B->053, C->054, F->055,
D->056**), this fix's ADR is **053**, not 054. `DECISIONS.md` and
`ARCHITECTURE.md` were updated together, in the same commit, with this
number; verified counts after the edit: `grep -c '^## ADR-' DECISIONS.md`
= 38, `grep -c '^### ADR-' ARCHITECTURE.md` = 54, `grep -c '^: '` = 0 in
both files.

**Push/pull discipline.** bm-ptl's PAT is pull-only (pushes return HTTP
403), so this fix's commit is pushed from the laptop only, matching the
batch rule. The modified `scripts/eval_m08.py` was transferred to bm-ptl
via `scp` (not a git operation) to run the sweep, since the code had not
yet landed on `master` at run time — bm-ptl's working tree carried it as
one uncommitted local file for the duration of the run and it was never
committed there. After the laptop pushes, bm-ptl should be synced via the
standard pull-only PAT fetch (`git fetch ... master` then `git reset
--hard FETCH_HEAD`) so laptop, origin and bm-ptl end this fix synced and
clean, matching the state this fix started in.

**`SUBMISSION.md` not modified.** Per the batch's standing rule
(overriding this individual fix's own brief, which did not ask for a
`SUBMISSION.md` edit in the first place — that instruction applied to
Fixes C and F, not B). Nothing in this fix required a `SUBMISSION.md`
change beyond the general robustness-evidence findings logged above for
morning-user's own edit.

---

## Fix C — Chained skill demo: pick -> handoff -> place (attempted; FAILS at step 2, no ADR)

**What changed.** New `scripts/chained_demo.py`: ONE continuous episode
(single `env.reset()`, single `WeldGrasp` instance, no reset between
steps -- deliberately the opposite shape from `run_demo.py`/
`run_grounded_demo.py`'s "fresh env+executor per skill" pattern, because
this script's whole point is to chain three skills within one live
physics state) running `pick(A, fork)` -> `handoff(A->B, fork)`
(receiver-first: `run_handoff(env, "B", "A", "fork", weld=weld)`) ->
`place(B, fork, table)`, verifying `WeldGrasp.is_holding()` and printing
scene state after every step. No skill, grasp, IK, executor, environment
or randomization code was touched -- `skills_scripted.py` was off-limits
for this batch and was not modified.

**Result, measured on bm-ptl this commit:**
1. **`pick(A, fork)`: PASS.** fork lifted 0.3560 -> 0.3989 m, weld attached
   at frame 1155, `is_holding('A')=='fork'`.
2. **`handoff(A->B, fork)`: FAIL.** `phase 1 (from_arm pick) failed
   (weld_attach_failed_after_300_frames)`. This is the KNOWN, pre-existing
   re-pick gap this fix's own task brief predicted and
   `scripts/run_grounded_demo.py`'s docstring (finding 2) had already found
   for the equivalent two-clause composition, now confirmed directly for
   this literal chain: `run_handoff`'s Phase 1 (`skills_scripted.py:2136`)
   calls `run_pick(env, from_arm, target_object, ...)` UNCONDITIONALLY,
   with no `already_held` guard analogous to `run_place`'s ADR-034 fix.
   Because arm A already holds the fork from step 1, the nested pick's
   GRIP waypoint calls `weld.attempt_grasp('A', 'fork')` every step, which
   `grasp.py`'s "already holds -- release first" gate refuses immediately
   every time, exhausting `GRIP_HOLD_FRAMES` (300) with no attach. Final
   state after step 2: `is_holding('A')=='fork'` (unchanged from step 1,
   never released), `is_holding('B') is None`. **Not fixed** --
   `skills_scripted.py` is off-limits for this batch; this is reported as
   an architectural finding, not patched.
3. Step 3 (`place(B, fork, table)`) was **never reached** -- the chain
   stopped honestly at step 2, per this fix's own instruction not to
   fall back silently.

**Fallback attempted and reported, explicitly labelled, NOT presented as
chain success.** Since arm A still genuinely held the fork after step 2's
failure, the script additionally ran `place(A, fork, table)` as a
clearly-labelled FALLBACK (not part of the 3-step chain). This ALSO
failed, at a NEW waypoint not seen in the ADR-038 regression-gate
baseline: `waypoint 1 (approach destination) failed [collision
(arm-vs-prop: mug (dist=-0.0051 m); threshold=-0.005 m)]` -- a marginal
(0.1 mm over threshold) arm-vs-mug collision. This differs from the
ADR-038 gate's own `place(A, fork, table)` number (z=0.3588, PASS)
because that baseline places immediately after a CLEAN, isolated
`pick(A, fork)`; here the fork's held position had already drifted during
handoff Phase 1's futile ~1300-frame re-approach/re-grip attempt (compare
step 1's post-pick fork_pos `[-0.0563, 0.0646, 0.3989]` to the
post-step-2 fork_pos `[-0.0619, 0.0762, 0.3600]`), landing the fallback
place's own waypoint-1 destination close enough to the mug to trip the
collision gate. Reported as a new, distinct finding, not patched or
retried with a different destination.

**Final fork position:** `[0.00364402, 0.05970431, 0.39474241]` -- held by
arm A throughout; the chain and its fallback both failed, so the fork was
never returned to rest on the table.

**Randomization.** `env.reset(seed=0, randomizer=None)` -- fixed default
layout, explicitly NOT randomized, stated in the script's own output.
Reasoning: ADR-048's `SKILL_ENVELOPES` intersection across
`pick_fork`/`place_fork`/`handoff` for `fork` (`randomization.py`) already
collapses to the single point (0,0,0,0), so once a chain includes
`handoff` there is no non-degenerate offset safe for every skill it calls.
`run_demo.py`'s own precedent (fixed layout for its `handoff` step) was
followed rather than inventing a second convention, per this fix's task
brief correction 3.

**Regression gate unaffected.** `pytest tests/test_skills.py` on bm-ptl
reproduced the documented baseline exactly: **4 passed, 4 failed** (the
same four tests failing for the same, already-documented reasons:
`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`).

**No ADR-054.** Per this fix's own task brief ("If step 2 fails on the
re-pick gap, that is a documented architectural finding — commit the
script and the log, no ADR-054") and the batch's corrected numbering
(C -> 054 conditional on full success), this fix does not ratify an ADR.
`DECISIONS.md`/`ARCHITECTURE.md` are UNCHANGED by this fix; counts remain
exactly Fix B's: `grep -c '^## ADR-' DECISIONS.md` = 38,
`grep -c '^### ADR-' ARCHITECTURE.md` = 54, `grep -c '^: '` = 0 in both.

**No video rendered.** The task brief's video step ("On full success,
render a clip to `docs/videos/chained-demo.mp4`") is conditioned on full
success; this run did not reach step 3, so no video was produced and none
of `ffmpeg`/`scp`/PNG rendering was invoked for this fix.

**`SUBMISSION.md` not modified.** Per the batch's standing rule, which
governs over this individual fix's own brief text (which asked for a
`SUBMISSION.md` line on success -- that condition was not met here
regardless). This outcome is logged here for morning-user to fold in by
hand: a chained pick -> handoff -> place demo was attempted and reached
step 1 of 3, with the specific architectural blocker identified above.

**Push/pull discipline.** bm-ptl's PAT is pull-only, so this fix's commit
(the script plus this log entry) is pushed from the laptop only; bm-ptl is
then synced via the standard pull-only PAT fetch (`git fetch ... master`
then `git reset --hard FETCH_HEAD`) so laptop, origin and bm-ptl end this
fix synced and clean, matching the state this fix started in.
