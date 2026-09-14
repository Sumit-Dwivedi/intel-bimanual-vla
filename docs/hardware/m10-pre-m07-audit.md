# Pre-M07/M08 audit — diagnostic only, no fixes applied

**Recorded:** Sept 14, 2026 · HEAD at start: `649271b` (M10 Phase 5, ADR-046).
**Scope:** the eight variables below, measured before M07 (randomization) and
M08 (10-seed eval harness) are built on top of them. **This document changes
no behaviour.** `skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`,
`env.py`, `scenes/so101/`, `gen_dual_scene.py`, `posenet.py`, `dataset.py`,
the checkpoint and the IR are all byte-for-byte unmodified — confirmed by
`git status` on bm-ptl showing only this document and one new probe script
(`scripts/probe_pre_m07_audit.py`) as untracked, and by `pytest
tests/test_skills.py` reproducing the exact pre-existing baseline (**4
passed / 4 failed**, same four failure reasons/residuals, re-run at the end
of this audit — see "Baseline reproduced" below).

All live-execution measurement ran on bm-ptl, `ov_env`
(`C:\Users\devcloud\project\ov_env\Scripts\python.exe` — the one venv with
both `mujoco==3.2.7` and `openvino==2026.3.1`). The command-grammar coverage
check (variable 8) needs neither and ran directly on the laptop.

## Zero-th finding: this audit's own harness had a real bug, found and fixed
## before any of the eight variables' numbers below can be trusted

Not one of the eight numbered variables — a methodology problem in *this
audit's own tooling*, caught by re-running a supposedly-deterministic zero-
jitter control point twice and getting two different answers.

**What happened.** `scripts/probe_pre_m07_audit.py`'s first draft
constructed ONE `TableSettingEnv` and ONE `ScriptedSkillExecutor` per CLI
invocation and looped `env.reset()` + `executor.execute()` across many seeds
or grid points — the obvious way to write a sweep. A 25-point fine grid for
`pick(A, fork)` centred on its own default position then reported the exact
default position (`dx=0, dy=0`) as a **failure**, immediately after an
earlier grid point at `dx=-0.010, dy=-0.005` had **succeeded**. That
contradicted an isolated smoke-test run of the identical zero-jitter case
(`success=True, final_z=0.3989`) run moments earlier as its own process.

**Root cause, confirmed by direct reproduction
(`scripts/probe_pre_m07_audit.py`'s module docstring records the same
finding).** `WeldGrasp.active_welds` (`src/bimanual/sim/grasp.py:190`) is a
plain Python `dict` on the `WeldGrasp` **instance**, set to the held body's
name by a successful `attempt_grasp` and cleared only by `release()`.
`env.reset()` resets MuJoCo's `data` (`mj_resetData`/
`mj_resetDataKeyframe`) — including `data.eq_active`, which is why the
*physical* weld constraint correctly goes inactive again — but has no way to
reach into a Python object it does not know exists. `active_welds` therefore
survives `reset()` untouched. `ScriptedSkillExecutor._ensure_weld`
(`executor.py:112-118`) deliberately **reuses one `WeldGrasp` across
repeated calls against the same `env`** — that reuse is documented,
intentional behaviour (`executor.py`'s own module docstring, ADR-030) for
the case a `TaskPlan` runs several skills back-to-back without an
intervening reset. But it means: reuse one executor+env pair across a
**seed loop with `reset()` between iterations**, and the first successful
grasp poisons **every later iteration's** `attempt_grasp` for that (arm,
object) pair — `attempt_grasp` refuses immediately with `"arm=A already
holds 'fork' -- call release(arm) first"`, which surfaces to a caller as the
indistinguishable-looking `weld_attach_failed_after_300_frames`, not as an
`already_held`-flavoured message.

**Reproduced directly, isolated from any grid/seed loop**
(`scripts/_scratch_weld_state_leak.py`, not committed):

```
run 1 (fresh): True lifted fork: ... weld_attach_frame=1155 weld_active_at_end=True
  active_welds after run 1: {'A': 'fork', 'B': None}
  active_welds after env.reset(): {'A': 'fork', 'B': None}   <-- survives reset()
run 2 (same env+executor, after reset(), IDENTICAL position): False weld_attach_failed_after_300_frames
  active_welds after run 2: {'A': 'fork', 'B': None}
```

**Fix applied to the AUDIT SCRIPT only** (never to production code): every
subcommand in `scripts/probe_pre_m07_audit.py` now constructs a **fresh**
`TableSettingEnv` and a **fresh** `ScriptedSkillExecutor` for every single
trial. Confirmed to resolve it: the same zero-jitter case run twice back to
back now succeeds both times (`2/2`, identical `frames_used=1655`,
identical `final_z=0.3989`). Every sweep below was then **re-run** with the
corrected harness; results below are all from the corrected runs. Two grid
sweeps changed as a direct result: `grid_pick_bottle` went from 1/36 to
2/36 (the previously-poisoned last cell, `dx=+0.050, dy=-0.050`, is a real
success), and the fine `pick_fork` grid went from 1/25 to 9/25 — the
corrected number is the one reported under variable 6 below. Every sweep
that had reported 0/N was re-run and reproduced 0/N exactly, confirming
those were never actually contaminated (no success ever occurred in them to
poison a later trial).

**Why this belongs in a pre-M08 audit, not just a footnote.** M08's own
done-when 1 (`PLAN.md:598`) is a `--seeds 0-9` loop over one executor. If
M08's harness is written the "obvious" way — one `TableSettingEnv` /
`ScriptedSkillExecutor` pair, `reset()` between seeds — **and any seed in
the run contains a successful grasp**, every subsequent seed's grasp
attempts for that (arm, object) pair will silently refuse, and M08 will
report a false failure that reads exactly like a reachability problem
(`weld_attach_failed_after_300_frames`) with no hint that the true cause is
harness reuse, not the seed's geometry. **This is not a finding about
`skills_scripted.py` or `grasp.py`, and neither was touched** — it is a
finding about *how a multi-seed caller must be built*: either construct a
fresh `env`/executor per seed (this audit's fix), or have `WeldGrasp`
itself clear `active_welds` on `env.reset()` (a production-code change,
correctly out of this diagnostic-only task's scope, and not applied here).
M08 should read this section before its own harness is written.

## Methodology note common to variables 1, 2, 6, 7 (the two corrections)

Per this audit's own instructions: `TableSettingEnv.reset(seed=N)` is
byte-identical for every `N` on the scripted-controller path today —
`env.py`'s own `reset()` docstring says the seed governs only the env's own
RNG stream "for future randomized use," and M07 (the thing that will
actually consume that stream) is not built yet. Looping seeds 0-9 through
plain `reset()` would therefore run the identical deterministic scenario
ten times. **Every "seed" below is this audit's OWN randomization**,
applied the same way `scripts/generate_posenet_data.py`'s per-sample loop
already does (`scripts/generate_posenet_data.py:633-638`): after
`reset(seed=0)`, overwrite ONE prop's free-joint `qpos[adr]`/`qpos[adr+1]`
(x, y only — z and quaternion left exactly as `reset()` set them), then
`mujoco.mj_forward()`. Never by regenerating the scene XML (ADR-038 found
that breaks `handoff`). Range: x, y ∈ default ± 0.05 m, z fixed at the
prop's own resting height. **Scope note:** each trial randomizes only the
ONE prop the skill under test targets, not all five props at once —
`generate_posenet_data.py`'s `sample_nonoverlapping_positions` already
exists for a collision-avoiding, all-five-props randomization, but folding
that untouched mechanism into this report would conflate two unmeasured
things into one number. Left for M07 to build; flagged, not attempted here.

Per this audit's cost correction: variable 6's grid starts at a **2 cm
step, ±5 cm range (6×6 = 36 points per skill)**, not the original 1 cm
spec (121 points/skill). Measured coarse-sweep wall time is reported per
skill below; a 1 cm refinement was run **only** where the coarse sweep
showed a sharp edge worth resolving (`pick_fork`, see variable 6).

---

## Variable 1 — Handoff robustness, 10 audit-randomized layouts, oracle mode

`handoff(to_arm=B, from_arm=A, fork)`. Fork's default (x, y) =
**(-0.0500, 0.0500)**, read live off `reset(seed=0)` (`env.jnt_qposadr`
lookup, not hardcoded — same discipline `generate_posenet_data.py`'s
Correction 3 uses). Jitter ±0.05 m, one audit-seed per row (`np.random.
default_rng(seed).uniform(-0.05, 0.05, size=2)`).

| seed | fork (x, y) used | success | failing waypoint / reason |
|---|---|---|---|
| 0 | (-0.0363, 0.0270) | **FAIL** | phase 1 (from_arm pick), waypoint 1 approach — IK residual 0.0509 m |
| 1 | (-0.0488, 0.0950) | **FAIL** | phase 1 (from_arm pick), waypoint 2 descend — collision vs spoon (dist -0.0097 m) |
| 2 | (-0.0738, 0.0298) | **FAIL** | phase 1 (from_arm pick), waypoint 1 approach — IK residual 0.0269 m |
| 3 | (-0.0914, 0.0237) | **FAIL** | phase 1 (from_arm pick), waypoint 1 approach — IK residual 0.0251 m |
| 4 | (-0.0057, 0.0511) | **FAIL** | phase 1 (from_arm pick), waypoint 1 approach — IK residual 0.0157 m |
| 5 | (-0.0195, 0.0808) | **FAIL** | phase 1 (from_arm pick), waypoint 2 descend — collision vs spoon (dist -0.0075 m) |
| 6 | (-0.0462, 0.0343) | **FAIL** | phase 1 (from_arm pick), waypoint 1 approach — IK residual 0.0341 m |
| 7 | (-0.0375, 0.0897) | **FAIL** | phase 1 (from_arm pick), waypoint 2 descend — collision vs spoon (dist -0.0059 m) |
| 8 | (-0.0673, 0.0987) | **FAIL** | phase 1 (from_arm pick) — weld never attached in 300 frames |
| 9 | (-0.0130, 0.0287) | **FAIL** | phase 1 (from_arm pick), waypoint 1 approach — IK residual 0.0514 m |

**0/10.** Every failure occurs in Phase 1 — `from_arm`'s own nested pick of
the fork — never reaching Phase 2 onward (the choreographed cross-arm
transfer ADR-032..038 spent five revisions on). No "retreat clearance"
figure applies to any of these ten runs; none got far enough for a retreat
to happen. The oracle **baseline** (fork at its exact default position,
`dx=dy=0`, reproduced during this audit) succeeds with
`from_arm_retreat_dist=0.2263 m` against the `HANDOFF_RETREAT_GATE_M=0.10`
gate — i.e. the ONE point this audit tested where `handoff` works at all is
its own unperturbed default, with no margin demonstrated in any direction
by this 10-seed sweep. See variable 6 below for the finer picture (a real,
narrow, roughly ±1 cm/-0.5 cm tolerance band exists — the ±5 cm sweep here
is simply too coarse relative to that band's size to land inside it by
chance in 10 draws).

## Variable 2 — pick(fork), place(fork), pick(bottle) robustness, same 10 seeds

Same audit-randomization mechanism, same seed→jitter mapping (so
`pick_fork`'s and `handoff`'s seed 0 both jitter the fork to the same
`(-0.0363, 0.0270)` — this is deliberate, not a bug: it lets the two be
compared row-for-row).

**`pick(A, fork)`: 0/10.** Identical failures to `handoff`'s Phase 1 above
(same target, same positions) — waypoint-1 IK-convergence failures for
6/10 seeds (0, 2, 3, 4, 6, 9), waypoint-2 collision-vs-spoon for 3/10
(seeds 1, 5, 7), and one `weld_attach_failed_after_300_frames` (seed 8).

**`place(A, fork, table)`: 0/10.** `run_place`'s nested `run_pick` (not
gated by ADR-034's `already_held` guard, since the fork starts unheld every
trial) fails identically to `pick_fork` above at the same seeds for the same
reasons — every row reads `"place aborted: pick failed (...)"` wrapping the
exact same reason string as the corresponding `pick_fork` row. `place`'s
OWN destination-approach waypoints (the ones ADR-034 found a separate,
narrow-margin failure at for the water bottle) are never reached in any of
these 10 trials.

**`pick(A, bottle)`: 0/10.** Bottle default (x, y) = **(0.2200, 0.0000)**.
5/10 fail at waypoint-1 IK convergence (seeds 1, 4, 5, 7, 8), 5/10 reach
GRIP but never attach (`weld_attach_failed_after_300_frames`, seeds 0, 2,
3, 6, 9) — a different failure-mode mix than the fork (zero waypoint-2
spoon-collisions at the bottle's position, since it sits far from the
spoon, but a much higher weld-attach-failure share, consistent with
ADR-033's own finding that the bottle's taller hover point already runs
closer to the arm's reach envelope than the other props).

**Aggregate across all four skills, ±5 cm jitter, oracle mode: 0/40.** None
of the four skills this repo's own tests/ADRs record as currently *working*
in oracle mode at their exact default position tolerate a ±5 cm placement
perturbation in this audit's 10-seed sample. Read together with variable 6's
finer sweep, this is a real, narrow-margin finding, not an artifact — see
"What this means for M07" at the end of this document.

## Variable 3 — PoseNet accuracy on skill-execution frames

5 executions × 4 capture points (start ≈2% of the run, ¼, ½, ¾) = 20
frames, PoseNet GPU FP16 (`artifacts/posenet_ir/posenet_fp16.xml`,
ADR-045's default), `env.render("posenet_cam")` at 224×224 (the escape
hatch — `env.py`'s `render()` renders regardless of the `cameras=` opt-in
setting, so this ran through the same oracle-mode `TableSettingEnv
(cameras=None)` the executor's own assert requires). Executions: `pick(A,
fork)`, `place(A, fork, table)`, `pick(A, bottle)`, `handoff(B,A,fork)` at
default position, and a second `handoff` with the fork jittered to
(-0.0363, 0.0270) — this audit's own `np.random.default_rng(0)` draw,
identical to variable 1's seed-0 row above — for variety. Each execution's
own `frames_used` (measured first, full budget) sets the fraction
denominator; each capture point is a **separate, freshly-constructed**
env+executor run truncated to that step budget (see the zero-th finding
above for why fresh construction is required for correctness here too).

| capture fraction | mean abs xy error, all 3 props pooled (n=15) |
|---|---|
| ~2% (`start`) | 16.8 mm |
| 25% | 46.6 mm |
| 50% | 44.3 mm |
| 75% | 60.8 mm |

| prop | mean abs xy error (n=20) | median | min | max | ADR-044 held-out MAE | ratio (mean) |
|---|---|---|---|---|---|---|
| fork | 61.5 mm | 57.1 mm | 2.9 mm | 204.5 mm | 3.2 mm | **19.2×** |
| water_bottle | 32.7 mm | 24.5 mm | 15.6 mm | 92.9 mm | 2.6 mm | **12.6×** |
| mug | 32.2 mm | 32.7 mm | 13.8 mm | 100.1 mm | 2.8 mm | **11.5×** |

As anticipated in advance (per this audit's own brief): degradation is
real and large, and it is worst on the object a skill is actively
manipulating — the single largest fork errors (153–204 mm) occur during
`pick(A, bottle)`'s frames, where the fork sits UNTOUCHED at rest the
whole time (so its error there is attributable to the OTHER arm/gripper
now occluding or partially framing it out of `posenet_cam`'s fixed view,
not to the fork itself moving) — while the water_bottle and mug errors
during that SAME execution are far smaller relative to their own worst
cases elsewhere. The `start` capture point (≈2% into the run, arms barely
moved off "home") is already 5× the held-out baseline, not 1×, which
matters: even the least-disturbed in-episode frame does not reproduce
ADR-044's held-out accuracy, because every training image has both arms
exactly at "home" (ADR-044/ADR-046's own caveat) and this capture point is
already slightly off that. Errors roughly double again by 25% into a run
and continue climbing through 75%. **This is a direct, measured argument
against extending perception-mode `pick`/`place` targeting reads deeper
into a skill's own waypoint sequence** (re-querying mid-flight, as opposed
to the current single per-skill cache generation), and it independently
corroborates ADR-046's own finding that `handoff`'s Phase-5 retreat gate is
unusually sensitive to a few-mm perceptual perturbation at Phase 1 — the
error budget a real mid-flight read would inject is 10-60× larger than the
one ADR-046 already measured breaking that gate.

## Variable 4 — `run_handoff`'s re-pick mechanism

**`run_handoff`'s Phase 1 calls `run_pick` unconditionally**
(`src/bimanual/control/skills_scripted.py:2136`, inside `run_handoff`,
`skills_scripted.py:1977`) — no `already_held`-style guard at that call
site, and **`run_pick` itself has no `already_held` branch anywhere in its
own body** (`skills_scripted.py:1405-1583`, read in full for this audit;
confirmed by grep — `already_held` appears nowhere in `run_pick`, only in
`run_place`, `skills_scripted.py:1675-1676`). This is confirmed to be **the
same bug shape ADR-034 fixed in `run_place`, at a different call site,
still open here** — `run_place`'s guard
(`already_held = weld is not None and weld.is_holding(arm) == body_name`,
`skills_scripted.py:1675`) has no counterpart in either `run_pick` or
`run_handoff`'s Phase 1.

Independently re-verified by direct execution (not just by reading code),
fresh env + fresh executor, no state-leak risk per the zero-th finding
above:

```
Step 1: pick(A, fork): True | lifted fork: ... weld_attach_frame=1155 weld_active_at_end=True
Step 2: handoff(B, A, fork), fork ALREADY held by A: False | phase 1 (from_arm pick) failed (weld_attach_failed_after_300_frames)
```

This matches ADR-046's own Task-5 finding
(`docs/hardware/m10-phase5-integration.md:275-283`) exactly, including the
identical failure string. The mechanism, traced for this audit: `run_pick`'s
nested GRIP dwell calls `weld.attempt_grasp(arm, body_name)` every step;
`attempt_grasp` refuses immediately because `active_welds['A']` is already
`'fork'` (genuinely still held this time — no reset happened between the
two calls, so this is a correct refusal, not the zero-th finding's stale-
state artifact) — the dwell exhausts its full `GRIP_HOLD_FRAMES=300`
budget with `attempt_grasp` refusing every single step, surfacing as
`weld_attach_failed_after_300_frames`. **No fix applied** — reported only,
per task scope. An `already_held` guard analogous to ADR-034's, added to
either `run_pick` itself or to `run_handoff`'s Phase 1 call site, is the
shape of fix that would close both this variable and variable 8's
two-clause finding below, left for a future module.

## Variable 5 — Camera-default audit

`grep -rn 'cameras=' src/ scripts/ tests/` (excluding `__pycache__`),
every `TableSettingEnv(` construction site classified:

| Pattern | Sites | Risk |
|---|---|---|
| Explicit `cameras=None` (oracle/state-only) | `tests/test_skills.py:75`, 14 `scripts/probe_*.py` files, `scripts/run_grounded_demo.py:117`, `run_skill.py`'s/`verify_m10_phase5.py`'s oracle branch | None — matches `ScriptedSkillExecutor`'s own oracle-mode assert |
| Explicit `cameras=[camera_list]`, purposeful | `scripts/generate_posenet_data.py:574` (`["front"]`), `scripts/view_scene.py:96` (`["front"]`), `run_skill.py`'s/`verify_m10_phase5.py`'s vision branch (`["posenet_cam"]`) | None — none of these four scripts call `env.step()` themselves (grepped: zero `.step(` matches in any of the four); `generate_posenet_data.py` and `view_scene.py` only ever `reset()`+`render()`, and the vision-branch scripts delegate all stepping to the executor, which is already guarded (next row) |
| Omitted (`TableSettingEnv()`, inherits the `cameras=None` `__init__` default) | `src/bimanual/sim/env.py:382` (module's own `__main__` demo), `scripts/verify_adr038_skills.py:18` | None — the inherited default is `None`, the SAFE state-only path, not a non-`None` instance default. "Omitted" here is harmless because `__init__`'s own default happens to be the fast path, not because these sites were audited and found safe by design |
| Internal per-step loop with a NON-`None` instance default in scope | `skills_scripted.py`'s `_run_waypoint`/`_dwell` (`skills_scripted.py:890`, `:1004`) — the only two `env.step()` call sites inside `skills_scripted.py` (grepped, confirmed exhaustive) | **This is exactly the Phase 5 bug's location.** Already found and fixed under ADR-046: both sites pass `cameras=[]` explicitly, overriding whatever instance default (`None` in oracle mode, `["posenet_cam"]` in vision mode) the caller's env was constructed with. Verified during THIS audit (not just re-read): grepped for every `.step(` call across `src/`, `scripts/`, `tests/` outside `skills_scripted.py` — every other call site either passes an explicit `cameras=` override or sits inside an env whose own instance default is already `None` |

**No repeat of the Phase 5 bug found.** The one place a non-`None` instance
default and an unguarded internal step loop could coincide is exactly the
two sites ADR-046 already fixed; every other camera-carrying env in the
repo is either never stepped internally or never constructed with a
non-`None` default in the first place.

## Variable 6 — Placement tolerance, coarse grid (2 cm step, ±5 cm range)

6×6 = 36 grid points per skill (offsets `{-5,-3,-1,+1,+3,+5}` cm on both x
and y — note this excludes the exact default position, `dx=dy=0`, by
construction). Fresh env+executor per point (zero-th finding).

| skill | successes | coarse sweep wall time |
|---|---|---|
| `pick(A, fork)` | 0/36 | 91.9 s |
| `place(A, fork, table)` | 0/36 | 92.0 s |
| `pick(A, bottle)` | **2/36** (`dx=-0.010,dy=+0.030`; `dx=+0.050,dy=-0.050`) | 86.8 s |
| `handoff(B,A,fork)` | 0/36 | 91.0 s |

Total coarse sweep, all four skills: **~6.1 minutes** wall clock — well
inside the correction's own estimate, because most failures resolve inside
a few hundred `env.step()` calls (waypoint-1 IK non-convergence or a
waypoint-2 collision), not the full step budget; only the rare "reaches
GRIP but never attaches" case (`weld_attach_failed_after_300_frames`,
1300 frames) and successes (1655-1656 frames) run long.

**`pick_fork`'s coarse 0/36 was a sharp-enough edge to warrant the
correction's own conditional 1 cm refinement** (the exact envelope shown by
its default-only success elsewhere in this audit implied a working region
smaller than ±1 cm must exist, since the coarse grid — which never samples
`dy=0` — found nothing). Refined: `--step 0.005 --range 0.01
--include-zero`, 5×5 = 25 points, **71.1 s**.

| dx \ dy | -0.010 | -0.005 | 0.000 | +0.005 | +0.010 |
|---|---|---|---|---|---|
| **-0.010** | FAIL | **PASS** | **PASS** | FAIL | FAIL |
| **-0.005** | FAIL | **PASS** | **PASS** | FAIL | FAIL |
| **0.000** | FAIL | **PASS** | **PASS** | FAIL | FAIL |
| **+0.005** | FAIL | FAIL | **PASS** | FAIL | FAIL |
| **+0.010** | FAIL | FAIL | **PASS** | **PASS** | FAIL |

**9/25 (36%).** The true envelope is real, non-trivial, and asymmetric —
roughly the full ±1 cm range in x, but only `dy ∈ [-0.005, 0.000]` in y
(fails as soon as `dy` goes positive by 5 mm, in every column). This
explains variable 1/2's 0/10 at ±5 cm cleanly: the working band is
entirely inside a region a coarse, zero-excluding, ±5cm-range grid or a
±5cm-jitter random draw will rarely land inside by chance — not because
there is literally zero tolerance, but because the tolerance band is far
narrower than the randomization range this audit (and, unless scoped
tightly, M07) was asked to test at. `pick_bottle`'s two-successes-out-of-36
envelope was left at coarse resolution only (its pattern — one hit at
`dx=-0.010,dy=+0.030` and an unrelated-looking hit at the extreme corner
`dx=+0.050,dy=-0.050` — does not show the single sharp edge that justified
refining `pick_fork`; refining it would need a different, non-local search,
out of this audit's time budget). `place_fork` and `handoff` were not
independently refined (their Phase-1 failures are byte-identical to
`pick_fork`'s at every shared coordinate, since both nest the identical
`run_pick` call against the identical fork position) — their true envelope
is `pick_fork`'s envelope, read through table above.

## Variable 7 — `run_handoff`'s assumptions

Read against ADR-032 through ADR-038's five-revision history plus this
audit's own measurements:

- **Fork start position.** `run_handoff`'s Phase 1 assumes `from_arm` can
  reach the target object from HOME within `APPROACH_DESCENT_STEPS=500`
  steps at waypoint 1. Variables 1/2/6 above show this holds only inside a
  ~1 cm × 0.5 cm band around the fork's current default (-0.0500, 0.0500) —
  a position `gen_dual_scene.py` currently places, not one `run_handoff`
  chose; nothing in `run_handoff` validates that its nested pick's target is
  reachable before committing to Phase 1.
- **`HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.43)`** (`skills_scripted.py:485`)
  is a fixed world point, read straight, never derived from either prop or
  arm position — ADR-032's two sweep passes (`ARCHITECTURE.md:2219-2461`)
  found this exact point only after two full collision-checked searches
  failed to find ANY shared reachable point at first, and ADR-036 had to
  raise its z from 0.35 to 0.43 to clear one collision, which immediately
  exposed a NEW, earlier one (`ARCHITECTURE.md:2782-2891`) — five total
  revisions (ADR-032, 032-second-pass, 035, 036, 037) before `handoff`
  worked end to end at all, several of them on what ADR-037 itself calls a
  "joint-limit knife-edge" (margins as small as -0.000001 rad on an
  alternative routing that was tried and rejected,
  `skills_scripted.py:2036-2051`). This constant is not randomized by
  anything in this repo and M07 is not expected to touch it directly, but
  M07 WILL move the fork the constant implicitly assumes will already be
  correctly positioned relative to.
- **Arm home poses / `HANDOFF_STAGING_Y_M = {"A": 0.06, "B": -0.06}`**
  (`skills_scripted.py:559`) governs the lateral staging corridor Phase 3
  uses to route `to_arm` around `from_arm`'s now-static body — ADR-037's
  own docstring (`skills_scripted.py:2027-2058`) documents that the two
  ALTERNATIVE routings it tried (an elevated "dodge" and a horizontal
  "dodge-in-x") were both measured and REJECTED as less robust than the
  one currently shipped, meaning the shipped corridor is not merely
  "the first thing that worked" but "the best of three measured options,
  still fragile."
- **What breaks it (measured, this audit):** any of the ten randomized fork
  positions in variable 1's table — every one fails inside Phase 1, before
  `HANDOFF_POSITION_XYZ` or the staging corridor are ever exercised at all.
  **What it tolerates:** a band smaller than ±1 cm in x and asymmetric in y
  around the fork's exact current default (variable 6's fine grid).
- **Net assessment for M07.** `run_handoff` (and, transitively, `pick`/
  `place`, since they share the identical nested-pick reachability
  boundary) has effectively **no placement-randomization headroom at its
  currently-shipped constants**. M07 randomizing "initial object placement"
  per the brief's p2 objective 3, even at a conservative range, will very
  likely need EITHER a tighter placement range than ±5 cm for props these
  skills target, OR a fix to the underlying reachability margin (out of
  this audit's scope) before M08's 10-seed harness can report anything
  other than a near-zero success rate that reflects this narrow band, not a
  meaningful robustness measurement of the CONTROL logic itself.

## Variable 8 — Command parse coverage

31 commands tested against `RuleGrounder` directly (`src/bimanual/language/
rule_grounder.py`), laptop, no `mujoco` import needed. Full list and every
parsed `TaskPlan`/error message in `scripts/probe_pre_m07_audit.py`'s
sibling scratch script (not committed — reproduced inline below for the
two load-bearing cases; the other 29 are straightforward vocabulary/
grammar checks that all matched their expected outcome and are not worth
individually enumerating here).

**Both of this task's two flagged claims independently confirmed, verbatim
reproduction of ADR-046's Task-5 findings:**

1. **The brief's literal sentence does not parse.**
   `"Pick up the fork with arm A and hand it to arm B."` →
   `UngroundedCommandError: clause not recognized: 'hand it to arm B'`.
   `hand` alone is not in the `handoff` verb alternation (`hand off` /
   `handoff` / `pass` / `give` only, `docs/command-grammar.md:89`).
2. **The grammar-supported two-clause form parses to two separate
   `SkillCall`s that fail when executed in sequence.**
   `"Pick up the fork with arm A and give it to arm B."` grounds cleanly to
   `[pick(A, fork), handoff(B, fork, from_arm=A)]` — two `SkillCall`s, not
   an error. Executing them in sequence through one executor (variable 4's
   probe, above) reproduces `pick` succeeding then `handoff` failing with
   `weld_attach_failed_after_300_frames` — variable 4's re-pick bug,
   confirmed as the root cause of this specific parse-then-fail sequence,
   not a grammar defect.

Every other tested command parsed and grounded as expected: full vocabulary
coverage (`open_drawer`/`close_drawer`/`pick`/`place`/`handoff`/`pour`, all
six `pick` verb synonyms, `mug`/`cup`/`glass` canonicalization, pronoun
resolution with and without a valid referent, case-insensitivity, the
default-arm rule, `from_arm` defaulting for `handoff`, empty/whitespace-only
text returning an empty plan without raising, and hard `UngroundedCommandError`s
for an unrecognized verb, an unrecognized noun, a pronoun with no referent,
punctuation-only text, extra trailing words, and a multi-clause command
where only one clause is bad). **0 unexpected mismatches out of 31 cases.**
The frozen grammar itself is not in question — the two failures above are
both already-known, already-documented gaps in the CONTROL layer
(`run_handoff`'s re-pick, variable 4), not the grammar.

## Baseline reproduced

`pytest tests/test_skills.py` at the end of this audit, same host, same
`ov_env`: **4 passed, 4 failed** (`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b` — same four, same failure reasons and
residuals as the pre-audit baseline run at the start of this session and as
`ARCHITECTURE.md`'s documented ADR-038/046 baseline). Confirms this audit's
extensive live-execution measurement (40 robustness trials + 169 grid
trials + 25 posenet_acc executions/partial-runs, all against the identical
`ScriptedSkillExecutor`/`TableSettingEnv` code paths) leaves production
behaviour unchanged.

## What this means for M07 and M08, stated plainly (not decided here)

- **M07's "placement" randomization axis, as currently scoped (±5 cm-ish,
  per this audit's own reading of a "modest, M07-like range"), will collide
  directly with variable 6's measured ~1 cm × 0.5 cm true tolerance band**
  for every skill this audit could measure (`pick`, `place`, `handoff`, all
  on `fork`; `pick` on `bottle`). Shipping M07 at that range without either
  narrowing it or addressing the underlying reachability margin will very
  likely make M08's 10-seed success rate report something close to the
  0-6% this audit measured at ±5 cm (variables 1/2/6's coarse grid), which
  would look like a policy failure rather than what it actually is: a
  placement range wider than the skills' demonstrated working envelope.
  Variable 6's fine grid shows the envelope is not literally zero — 9/25
  (36%) inside a tighter ~1 cm × 0.5 cm band — so narrowing M07's own range
  for these specific props, rather than fixing the underlying margin, is a
  real option if M08 needs a non-trivial success rate on a tight schedule.
- **M08's harness must not reuse one `WeldGrasp`/executor across a
  `reset()`-based seed loop** without either (a) constructing a fresh
  `TableSettingEnv`+`ScriptedSkillExecutor` per seed (this audit's own
  fix), or (b) a production-code fix to `WeldGrasp` clearing
  `active_welds` on `reset()` (not attempted here, out of scope). Getting
  this wrong will silently understate M08's own reported success rate in a
  way that gets WORSE, not better, the more seeds succeed early in a run.
- **Command-grammar coverage is solid** (variable 8) — the demo path should
  keep using the single-clause `"Give the fork to arm B."` form
  `scripts/run_grounded_demo.py` already adopted, not the brief's literal
  two-clause sentence, until variable 4's re-pick gap is closed.
- **PoseNet-driven targeting should not be extended past its current
  single-cache-generation design** (variable 3) — mid-skill re-querying
  would inject errors 10-60× the held-out baseline into the control loop,
  an order of magnitude worse than the few-mm perturbation ADR-046 already
  found sufficient to break `handoff`'s Phase-5 retreat gate.

## Files touched by this audit

- `docs/hardware/m10-pre-m07-audit.md` (this file).
- `scripts/probe_pre_m07_audit.py` (new probe script; `robustness`, `grid`,
  `posenet_acc` subcommands — see its own module docstring for full usage
  and the zero-th finding's design rationale).
- Nothing else. `git status` on bm-ptl at the end of this audit shows only
  these two as untracked; `out/audit/*.jsonl` (raw per-trial data backing
  every table above) is gitignored (`out/`, `.gitignore:31`) and not
  committed — available on bm-ptl at
  `C:\Users\devcloud\intel-bimanual-vla\out\audit\` for anyone who wants
  the per-trial detail behind an aggregate number in this document.
