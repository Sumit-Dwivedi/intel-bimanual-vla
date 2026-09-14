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

---

## M06 handoff already_held guard — `run_handoff` Phase 1 fixed (ADR-054); chain now fails one phase later, at a NEW Phase 3 collision

**What changed.** `src/bimanual/control/skills_scripted.py`, `run_handoff`
Phase 1 only: added the same `already_held` guard ADR-034 already uses in
`run_place` (`already_held = weld is not None and
weld.is_holding(from_arm) == body_name`; skip the nested `run_pick` when
true, guard every downstream `pick_result.<attr>` access). This is the
exact bug Fix C's chained-demo attempt (above) found and explicitly left
unpatched, since `skills_scripted.py` was off-limits for that batch. No
other line in `run_handoff` changed; `run_pick`, `run_place`, and every
other ADR-031/033/035/037/038 mechanism are untouched.

**Findings that belong in `SUBMISSION.md`'s eventual robustness/bimanual
section, once morning-user reviews this log:**

1. **The pick→handoff re-pick bug is fixed, verified two ways.**
   `scripts/verify_adr038_skills.py`, run before and after this code
   change on BOTH bm-ptl and the laptop, reproduced byte-identical numbers
   in every case (bm-ptl: `0.3989 / 0.3588 / 0.6192 / 0.1946 m /
   frames_used=6610`; laptop: `0.3987 / 0.3588 / 0.6191 / 0.1958 m /
   frames_used=6610` — the laptop-bm-ptl digit gap is the SAME
   already-documented ADR-047 cross-machine floating-point divergence,
   present identically before and after, not caused by this fix).
   `pytest tests/test_skills.py`: 4 passed / 4 failed, same four tests and
   residuals, both machines, before and after. None of the hard revert
   conditions moved; the fix was kept.
2. **The chain composes one phase further than before, but still does not
   complete.** Re-running `scripts/chained_demo.py` (unmodified) on
   bm-ptl: step 1 (`pick(A, fork)`) PASSES exactly as before. Step 2
   (`handoff(A→B, fork)`) no longer hits the old bug — Phase 1 correctly
   skips its nested pick (0 frames) and Phase 2 (`from_arm` → transfer
   point) succeeds — but now FAILS at **Phase 3** (`to_arm` approaches the
   receiving point): `"phase 3 (to_arm approach) failed [direct approach
   failed [convergence (IK residual=0.0875 m >= 0.01 m)]; staging to
   y=-0.06 also failed [collision (cross_arm contacts=1 vs baseline 0;
   armB-vs-table_top contacts=0 vs baseline 0)]]"`, `frames_used=1500`.
   Step 3 (`place(B, fork, table)`) was never reached.
3. **This is a genuinely new failure mode, reasoned about but not chased
   or patched**, per this fix's own instructions. The likely mechanism:
   Phase 1 being skipped means `from_arm` carries over step 1's own
   independent, full-budget `pick(A, fork)` final pose instead of a fresh
   half-budget nested-pick RETREAT pose — a legitimately different
   starting point for Phase 2's approach to `transfer_point`, which
   apparently leaves `from_arm` close enough to `to_arm`'s staging
   corridor to produce a real cross-arm collision. Not independently
   re-measured with further instrumentation (out of scope for this fix).
4. **Fallback `place(A, fork, table)` passed this time** (fork left at
   `x=0.0835 y=-0.0585 z=0.3591`, `is_holding('A') is None`) — a different
   outcome than Fix C's own fallback run (which hit a 0.1 mm arm-vs-mug
   collision), because the fork was left in a different table position by
   this run's further-progressed (but still failed) handoff attempt. Not
   a contradiction, not adjusted to match either way.

**ADR-054, ratified.** This is the first fix in this batch to fully close
one of the conditionally-numbered slots (`docs/hardware/overnight-batch-log.md`'s
own "ADR numbering" note above: C→054 was conditional on the chain's full
success; that condition was never about THIS fix, which is a separate,
later task scoped only to the Phase 1 guard). `DECISIONS.md` and
`ARCHITECTURE.md` were updated together, in the same commit. Verified
counts after the edit: `grep -c '^## ADR-' DECISIONS.md` = 39,
`grep -c '^### ADR-' ARCHITECTURE.md` = 55, `grep -c '^: '` = 0 in both
files.

**No video rendered.** The chain did not fully succeed (fails at step 2,
Phase 3), so `docs/videos/chained-demo.mp4` was not produced and none of
`ffmpeg`/`scp`/PNG rendering was invoked for this fix, per this fix's own
"if the full chain succeeds" condition.

**`SUBMISSION.md` not modified.** Per the batch's standing rule; this
finding is logged here for morning-user to fold in by hand.

**Push/pull discipline.** The fixed `skills_scripted.py` was `scp`'d to
bm-ptl as an uncommitted working-tree diff to verify the standalone
regression gate and the chain attempt on the authoritative machine BEFORE
committing (bm-ptl's own pre-fix baseline was independently re-measured
first, matching this task's stated reference numbers exactly). After
verification, the change was committed and pushed from the laptop only
(bm-ptl's PAT remains pull-only); bm-ptl is synced afterward via the
standard pull-only PAT fetch (`git fetch ... master` then `git reset
--hard FETCH_HEAD`) so laptop, origin and bm-ptl end this fix synced and
clean.

---

## Fix F — Perception-in-loop demo: PoseNet drives `pick(A, fork)` (ADR-055)

**What changed.** New `scripts/perception_demo.py`. Runs the unmodified
`sk.run_pick(env, "A", "fork", weld=weld, position_provider=provider)` —
the same call `ScriptedSkillExecutor._dispatch` itself would produce for
`SkillCall(skill="pick", arm="A", target_object="fork")` — with
`position_provider` a real `PoseNetInference(device='GPU')` +
`CachedPropPositions` bound to a fresh `WeldGrasp` (M10 Phase 5's
opt-in wiring, ADR-046, exercised end to end, not modified). No skill,
grasp, IK, executor, environment, or randomization code was touched.
Verification reads `env.data.xpos`/`WeldGrasp.is_holding` directly —
oracle ground truth, never the perception estimate the skill acted on —
per ADR-046 Correction 1 and this fix's own task brief ("a skill graded by
the same estimate it acted on is unfalsifiable"). Full design record:
`ARCHITECTURE.md`'s ADR-055 entry.

**ADR numbering.** The batch brief numbered this fix's ADR as 056, but
ADR-054 (the `run_handoff` already-held guard, user-authorised mid-batch)
took 054 first, so this fix's number is **055**, per the corrected
sequence (B→053, C→054-conditional-then-unused, handoff-guard→054,
F→055, D→056). `DECISIONS.md` and `ARCHITECTURE.md` were updated together,
in the same commit. Verified counts after the edit: `grep -c '^## ADR-'
DECISIONS.md` = 40, `grep -c '^### ADR-' ARCHITECTURE.md` = 56,
`grep -c '^: '` = 0 in both files.

**Randomization (batch correction 3).** `env.reset(seed=0)`, **no
randomizer** — `ENVELOPES = {}` at module scope (`randomization.py`,
ADR-048) means a bare seed selects nothing without an explicit
randomizer. Fixed default was chosen (over `pick(A, fork)`'s own Track A
rectangle from `SKILL_ENVELOPES`, `run_demo.py`'s alternative precedent)
specifically so this run's numbers are the SAME scenario ADR-046's own
single-refresh oracle-vs-PoseNet table already measured — a real choice
between two legitimate options, stated in this script's own printed
output, not left implicit.

**The ADR-046 per-step render-cost trap (batch correction 4), confirmed
NOT reintroduced.** `pick(A, fork)` completed in 1.487 s wall clock (1655
physics steps) — not the 12+ CPU-minutes ADR-046 measured for the
per-step-render bug. `skills_scripted.py`'s existing `cameras=[]`
overrides at both internal `env.step()` call sites were inherited
unmodified; this script never duplicates or bypasses them.

**Result: PASSES, verified on oracle ground truth, not the perception
estimate.** `env.data.xpos[fork][2]`: 0.3560 → **0.3905**.
`WeldGrasp.is_holding('A')`: **`'fork'`**. `run_pick.success=True`
(`weld_attach_frame=1155`, `frames_used=1655`). **`fork_z (0.3905) > 0.37`
AND `is_holding == 'fork'` → ORACLE_SUCCESS = True.** `0.37` is asserted at
runtime equal to `sk.TABLE_SURFACE_Z + sk.WELD_PICK_SUCCESS_MARGIN_M`
(0.35 + 0.02) — the skill's own internal weld-success threshold — checked
by the script, not merely commented.

**Perception numbers, reported exactly as this fix's task brief asked:**
- **Cache refresh count: 1. Total inference count: 1.** `run_pick`'s
  targeting read is the ONLY `position_provider.get()` call in the whole
  call (`cached_access.py`'s own one-render-per-generation contract,
  confirmed by direct count).
- **Per-inference latency.** The one real in-loop `predict()` call: 1.83 ms
  (n=1 — a single sample, not a distribution, reported as such).
  Supplementary `PoseNetInference.benchmark(n_runs=100)` on the SAME
  already-compiled model (no second compile): **GPU FP16 mean=0.5903 ms,
  median=0.5889 ms, min=0.5773 ms, max=0.6239 ms, throughput=1694 Hz**
  (`execution_devices=['GPU.0']`) — closely tracking M10 Phase 5/ADR-046's
  own GPU FP16 figures (mean=0.6648 ms, max=7.2228 ms). Device: GPU, first
  attempt, no CPU fallback needed.
- **Oracle-vs-PoseNet delta at the one refresh** (`CachedPropPositions`'s
  own diagnostic log, never used for control): fork 2.19 mm, water_bottle
  21.71 mm, mug 13.43 mm. **Two distinct delta quantities were almost
  conflated in this script's first draft and were caught and fixed before
  commit:** (1) this 2.19 mm PERCEPTION-ESTIMATE delta (PoseNet's raw xyz
  guess vs. ground truth at the render instant — matches ADR-046's own
  reported 2.2 mm for fork at this identical seed/scenario), vs. (2) the
  **OUTCOME delta** — how far THIS run's real final z (achieved WITH
  perception driving the grasp-point target) lands from the documented
  oracle-only baseline final z (0.3989): **8.37 mm**. Quantity (2) is the
  one M10 Phase 5/ADR-046's own headline "8.4 mm" figure refers to, and
  this run reproduces it to within 0.03 mm.

**Regression gates, re-run this commit on bm-ptl, unchanged (expected,
since this fix touches no skill/grasp/ik/executor/env/randomization
code):**
- `pytest tests/test_skills.py`: **4 passed / 4 failed** — same four
  tests, same residuals (`test_open_drawer_reaches_near_limit`,
  `test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
  `test_handoff_mug_ends_held_by_arm_b`).
- `scripts/verify_adr038_skills.py`: **0.3989 / 0.3588 / 0.6192 / 0.1946,
  frames_used 6610** — byte-identical to the documented baseline.

**Video rendered:** `docs/videos/perception-demo.mp4`. 40 PNG frames
(`front` camera, 640x480) written on bm-ptl from
`mujoco.mjtState.mjSTATE_FULLPHYSICS` snapshots captured by an observer
wrapper around `env.step` (never altering the real run's control, timing,
or step count — the same technique `scripts/render_handoff_frames.py`
already uses), spanning physics-step indices 1..1655. `scp`'d to the
laptop (bm-ptl has no imaging libraries in `ov_env`), encoded there with
the WinGet-installed `ffmpeg` at 15 fps (`-pix_fmt yuv420p`). Both endpoint
frames inspected by eye: the mid-sequence frame shows arm A approaching
the fork with arm B parked at HOME; the final frame shows arm A retreated
upward with the fork visibly lifted in its gripper. Temporary PNG
directories (`perception_demo_frames_tmp/` on bm-ptl, and the laptop-side
scratch copy) were never `git add`ed and are cleaned up after this commit.

**`SUBMISSION.md` not modified.** Per the batch's standing rule. This
finding — a working perception-in-loop demo, GPU FP16, PASS on oracle
ground truth — is logged here for morning-user to fold into
`SUBMISSION.md`'s eventual perception/OpenVINO evidence section by hand.
A one-line pointer to `scripts/perception_demo.py` was added to
`README.md`'s "Documentation" section only — the Quickstart and the
ADR-015 "Grasping Abstraction and Documented Limitations" sections were
not touched.

**Push/pull discipline.** `scripts/perception_demo.py` was `scp`'d to
bm-ptl as an uncommitted working-tree file to run and verify on the
authoritative machine (ADR-047: cross-machine float divergence) BEFORE
committing. After verification, the script plus this log entry plus the
`DECISIONS.md`/`ARCHITECTURE.md` ADR-055 mirror plus the `README.md`
pointer plus `docs/videos/perception-demo.mp4` (rendered on the laptop
from bm-ptl-sourced PNGs) were committed and pushed from the laptop only
(bm-ptl's PAT remains pull-only); bm-ptl is synced afterward via the
standard pull-only PAT fetch (`git fetch ... master` then `git reset
--hard FETCH_HEAD`) so laptop, origin and bm-ptl end this fix synced and
clean.
