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
