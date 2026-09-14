# M10 Phase 5 — PoseNet wired into the controller via cached per-skill inference (ADR-046)

Wires the OpenVINO-compiled PoseNet (M10 Phases 1-4, ADR-039..045) into
`ScriptedSkillExecutor`/`skills_scripted.py` as an **opt-in** replacement for
oracle prop-position reads, on **GPU FP16** (`artifacts/posenet_ir/posenet_fp16.xml`,
ADR-045). Oracle stays the default; every success/verification check stays
oracle unconditionally, in both modes. All work below ran on bm-ptl, `ov_env`
(`scripts/requirements-bmptl.txt` — the one bm-ptl venv with both
`mujoco==3.2.7` and `openvino==2026.3.1` installed together).

See `DECISIONS.md`'s ADR-046 entry / `ARCHITECTURE.md`'s ADR-046 section for
the design rationale (targeting-vs-verification split, the held-object
perception fallback, the opt-in camera/executor wiring). This document is the
measured evidence.

## What ships

- `src/bimanual/perception/inference.py` — `PoseNetInference(ir_path, device)`:
  compiles the IR once, runs 3 warm-up inferences, exposes `predict(image) ->
  {prop: xyz}` and `benchmark(n_runs)`. Fails loudly (`FileNotFoundError`) if
  the IR pair is missing, rather than a bare OpenVINO traceback — the IR lives
  under gitignored `artifacts/` and exists only on bm-ptl.
- `src/bimanual/perception/cached_access.py` — `CachedPropPositions(env,
  inference, weld=None)`: `get(body_name)` returns a cached PoseNet estimate,
  or `None` (signalling "fall through to oracle") for any prop outside
  PoseNet's 3-prop scope (`plate`, `spoon`, `drawer`) **or currently held by
  either arm** — the safety rule that keeps a lifted object's position off
  the perception path. `invalidate()`, `get_all_with_oracle()` (diagnostics),
  and `refresh_count`/`inference_count` counters for per-skill reporting.
- `src/bimanual/control/skills_scripted.py` — `run_pick`/`run_place`/
  `run_handoff` all take an optional `position_provider=None`. Two targeting
  reads are wired to it (`run_pick`'s `obj_pos0`, `run_place`'s `obj_xy`);
  every verification read stays oracle, untouched. Also: **both internal
  `env.step()` calls now pass `cameras=[]` explicitly** — see "The real cost
  bug found and fixed" below.
- `src/bimanual/control/executor.py` — `ScriptedSkillExecutor(inference=None)`.
  `None` (default) is byte-identical to pre-Phase-5 behaviour. A given
  `PoseNetInference` lazily builds a `CachedPropPositions` bound to
  `(env, weld)`, invalidates it at the start of every `execute()` call, and
  the camera-set assertion now branches on mode (`cameras=None` for oracle,
  `cameras=['posenet_cam']` for perception).
- `scripts/run_skill.py` — `--perception {oracle,vision}` CLI flag.
- `scripts/verify_m10_phase5.py` — the verification harness this document's
  numbers come from.
- `scripts/run_grounded_demo.py` — Task 5, grounder → executor → render.

## Oracle-vs-verification split (ADR-046's Correction 1), as actually wired

| Site | `skills_scripted.py` | Role | Wired to perception? |
|---|---|---|---|
| `run_pick`'s `obj_pos0` | grasp-point targeting | targeting | **yes** |
| `run_pick`'s `final_z` | lift success check | verification | **no — oracle, unconditional** |
| `run_place`'s `obj_xy` | destination-offset targeting | targeting | **yes (see below)** |
| `run_place`'s `final_pos` | table-rest success check | verification | **no — oracle, unconditional** |
| `run_handoff`'s `initial_z` | pre-handoff baseline | verification | **no — oracle, unconditional** |
| `run_handoff`'s `final_pos` | transfer success check | verification | **no — oracle, unconditional** |
| `run_open_drawer`'s drawer reads | drawer geometry | out of PoseNet's 3-prop scope | **no — oracle, always** |

`run_place`'s `obj_xy` is mechanically wired to `position_provider.get()`
exactly like `run_pick`'s targeting read — but by the time that line runs,
`arm` always already holds `target_object` (either the nested `run_pick`
just confirmed it, or the `already_held` branch skipped the pick because it
was already true). `CachedPropPositions.get()` refuses to serve a
perception estimate for a currently-held prop (next section), so this call
site is oracle-sourced **at runtime**, every time — not because the line was
left oracle, but because the safety rule inside `get()` recognises the
object is airborne. Confirmed by the measured deltas below: `place_fork`'s
final z differs from the oracle baseline by under 1 mm in vision mode,
consistent with only the upstream pick's few-mm perturbation surviving, not
a resting-height PoseNet guess being substituted for a lifted object.

## Why a held object never gets a fresh PoseNet estimate (Correction 2)

ADR-044 (M10 Phase 3) measured that PoseNet's z output carries no real
signal: every training image shows a prop resting on the table, so z is a
per-prop constant the network memorised. The moment an arm's weld lifts an
object, its true z rises well above that constant — the water bottle's own
prior measurement (`pick(A, water_bottle)`, ADR-034) goes from a resting
z=0.4400 to a held z=0.6192, a ~180 mm rise — while a fresh PoseNet render
would keep confidently predicting the resting height it always predicts.
`CachedPropPositions.get()` therefore returns `None` (fall through to
oracle) for any prop currently held by either arm, checked via
`WeldGrasp.is_holding`. This is checked on every `get()` call, not just at
cache-refresh time, so it composes correctly regardless of when the cache
was last populated.

The brief's originally suggested rule — "invalidate the cache after weld
`attempt_grasp` succeeds **or** release fires" — was only half adopted:
invalidating on **release** is implemented (the object is back at rest,
exactly PoseNet's training distribution). Invalidating on **attach** was
deliberately **not** implemented, because a refresh immediately after a
successful grasp would render the object mid-lift and get back its resting
z again — freshly computed, and therefore more convincing to a future log
reader than a merely-stale cached value. The held-object check in `get()`
makes an attach-time invalidation unnecessary: a held object never reaches
the cache at all, regardless of when it was last refreshed.

## The real cost bug found and fixed, during this module's own verification run

Constructing `TableSettingEnv(cameras=['posenet_cam'])` (ADR-022's opt-in
rendering, required so `CachedPropPositions` has a camera to render) sets
that camera as the env's **instance default**. `env.step()`'s own default
behaviour (per `_resolve_cameras`, ADR-022) is: no `cameras=` argument means
*fall back to the instance default*. Every internal `env.step()` call inside
`skills_scripted.py`'s `_run_waypoint`/`_run_dwell` helpers was written under
oracle-only conditions (`cameras=None` always) and never passed `cameras=`
explicitly — harmless when the instance default is `None`, but in
perception mode it meant **every single physics step of every waypoint
rendered `posenet_cam`**, not just the one render per skill this module's
design intended.

Measured directly: the first `verify_m10_phase5.py` run against
`pick(A, fork)` alone did not return in over 12 CPU-minutes (multi-threaded,
confirmed alive via `Get-Process`'s climbing `CPU` field, not hung) before
being killed — consistent with ~1655 steps × the per-render cost measured
below, repeated for every waypoint of every skill. **Fix:** both
`env.step()` call sites in `skills_scripted.py` now pass `cameras=[]`
explicitly (an empty list, not `None`) — per `_resolve_cameras`'s own
documented semantics this overrides the instance default for that one call,
restoring the physics-only per-step cost regardless of which mode the
executor is in. Verified this fix changes nothing for oracle mode (`cameras=[]`
and `cameras=None` both resolve to "no render" when the instance default is
already `None`) by re-running `pytest tests/test_skills.py` and
`scripts/verify_adr038_skills.py` afterward — both reproduced their exact
pre-fix numbers (below).

This is disclosed here, not smoothed over, because it is exactly the kind of
cost that ADR-022's opt-in design was built to make visible rather than
silently pay — and a naive perception wiring would have paid it on every
step rather than once per skill.

## Measured per-render / per-inference cost

From `scripts/_diag_phase5_timing.py` (bm-ptl, `ov_env`, GPU):

| Operation | Measured |
|---|---|
| `TableSettingEnv(cameras=['posenet_cam'], render_width=224, render_height=224)` construction | 0.106 s |
| `env.reset(seed=0)` (renders `posenet_cam` once internally, per ADR-022's opt-in default) | 0.346 s |
| `env.render('posenet_cam')` (renderer already constructed) | 0.133 s |
| `PoseNetInference(device='GPU')` construction (compile + 3 warm-up infers, warm shader cache) | 0.349 s |
| `PoseNetInference.predict()` | 0.001 s |

`PoseNetInference.benchmark(n_runs=100)`, per device (10 warm-up discarded,
100 measured, static batch-1 `[1,3,224,224]` — same methodology as ADR-045):

| Device | Mean (ms) | Median (ms) | Min (ms) | Max (ms) | Throughput (Hz) | execution_devices |
|---|---:|---:|---:|---:|---:|---|
| CPU | 5.9488 | 5.8846 | 5.4251 | 8.3286 | 168.10 | `['CPU']` |
| GPU | 0.6648 | 0.5960 | 0.5786 | 7.2228 | 1504.10 | `['GPU.0']` |
| NPU | 1.0638 | 1.1035 | 0.7875 | 1.3906 | 940.05 | `NPU` |

These track ADR-045's benchmark-only figures closely (CPU ~6.4-6.5 ms, GPU
~0.59-0.68 ms, NPU ~1.10-1.28 ms) — the runtime wrapper adds no measurable
overhead beyond OpenVINO's own `infer()` call.

**A first GPU compile on a cold shader cache took several minutes** (~12+
CPU-minutes observed once, before the `cameras=[]` bug above was isolated
and fixed — the two issues were investigated together and this figure is
reported for completeness, not cleanly separated from the step-render bug).
A second process on the same machine compiled in 0.349 s, consistent with
Intel's GPU driver persisting a compiled-kernel cache across process
launches. **This means the FIRST perception-mode skill call on a freshly
booted or freshly re-imaged bm-ptl instance should budget for a one-time
GPU compile of unknown-but-possibly-multi-minute cost**, not the steady-state
0.35 s measured here on a warm cache.

## The honest accuracy number (Correction 3)

ADR-045's ~1.4e-4 to ~1.7e-4 deviation figures are **OpenVINO-vs-PyTorch
conversion fidelity** — they say the exported graph computes almost the same
function the trained PyTorch model computes, nothing about whether that
function is a good pose estimator. The real perception error entering this
control loop is ADR-044's held-out MAE against ground truth: **fork 3.2 mm,
water_bottle 2.6 mm, mug 2.8 mm, x/y only** (z carries no signal by dataset
construction — every prop's z is a per-prop constant).

Measured oracle-vs-PoseNet deltas at the one refresh each skill actually
performed (seed 0, `posenet_cam`, logged for transparency, never used for
control):

| Prop | Oracle xyz (m) | PoseNet xyz (m) | Delta (m) |
|---|---|---|---:|
| fork | (-0.0500, 0.0500, 0.3560) | (-0.0505, 0.0503, 0.3539) | 0.0022 |
| water_bottle | (0.2200, 0.0000, 0.4400) | (0.1984, 0.0020, 0.4405) | 0.0217 |
| mug | (0.0500, -0.0300, 0.3900) | (0.0630, -0.0286, 0.3869) | 0.0134 |

fork's delta (2.2 mm) sits almost exactly on ADR-044's held-out MAE (3.2 mm).
water_bottle's delta (21.7 mm) and mug's (13.4 mm) are well above their
respective held-out MAE figures (2.6 mm, 2.8 mm) — this single seed's frame
is not a resampling of the held-out validation set, so a wider single-sample
spread than the aggregate MAE is expected, not a contradiction of ADR-044's
number. All three deltas are still small relative to the ~10 mm slop this
module's task brief anticipated as "expected noise, not regression."

## Four-skill verification: oracle (must match baseline exactly) vs vision

`scripts/verify_m10_phase5.py`, bm-ptl, `ov_env`, seed 0, fresh env per skill
(no cross-contamination), identical to `scripts/verify_adr038_skills.py`'s
established four calls.

### Oracle mode — confirmed byte-identical to the ADR-038 baseline

| Skill | Baseline (ADR-038 / `m10-camera-comparison.md`) | This run |
|---|---|---|
| `pick(A, fork)` final z | 0.3989 | **0.3989** |
| `place(A, fork, table)` final z | 0.3588 | **0.3588** |
| `pick(A, 'bottle')` final z | 0.6192 | **0.6192** |
| `handoff(A→B, fork)` lateral sep | 0.1946 m | **0.1946 m** |

`pytest tests/test_skills.py`: **4 passed / 4 failed** — identical reasons to
every prior run of this suite (drawer/pick/place/handoff-with-plate-or-mug
failures are the pre-existing ADR-024/reach-limit findings, unrelated to
this module). `scripts/verify_adr038_skills.py` re-run independently:
identical numbers to the table above, confirming the opt-in default (no
`inference`, no `position_provider`) reproduces every pre-Phase-5 number
exactly, both through the executor and through direct function calls.

### Vision mode (GPU FP16)

| Skill | Oracle final z | Vision final z | Delta | Result | Refreshes | Inferences | Skill wall-clock |
|---|---:|---:|---:|---|---:|---:|---:|
| `pick(A, fork)` | 0.3989 | 0.3905 | 8.4 mm | **PASS** | 1 | 1 | 2.900 s |
| `place(A, fork, table)` | 0.3588 | 0.3579 | 0.9 mm | **PASS** | 1 | 1 | 5.213 s |
| `pick(A, 'bottle')` | 0.6192 | 0.6286 | 9.4 mm | **PASS** | 1 | 1 | 1.965 s |
| `handoff(A→B, fork)` | success, lateral 0.1946 m | **FAIL** | — | **FAIL** | 1 | 1 | 18.430 s |

Three of four skills pass, with final-z deltas of 0.9-9.4 mm against the
oracle baseline — inside the "~10 mm expected noise" this module's task
brief anticipated. `place_fork`'s sub-millimetre delta is the direct,
measured consequence of the held-object fallback described above: its own
targeting read is oracle-sourced at runtime, so the only perceptual
perturbation surviving into its final number is whatever the upstream
`pick`'s few-mm grasp-point offset carried forward.

**`handoff(A→B, fork)` FAILS under perception, and is reported here exactly
as instructed rather than tuned:**

```
success=False
reason=phase 5 (from_arm retreat) did not clear the 0.1 m gate before
       to_arm's retreat: from_arm_retreat_dist=0.0540 m
```

The failure is at Phase 5 (`HANDOFF_RETREAT_GATE_M`), a gate computed purely
from `env.data.site_xpos` against the fixed world-frame `transfer_point` —
it has **no direct dependency on any object's position at all**. The only
perceptual input anywhere in this call is Phase 1's nested `run_pick`'s
2.2 mm grasp-point offset for `fork`. `handoff`'s own choreography
(ADR-032 through ADR-038's history: five ADR revisions, several of them
"still fails, reported not patched") is already the most kinematically
fragile skill in this repo — every one of those ADRs records a working
corridor with essentially no margin, several literally on a "joint-limit
knife-edge" (ADR-037's own words). A 2.2 mm perturbation at the very start
of a ~13-waypoint sequential choreography evidently propagates far enough
to turn a 0.2263 m retreat clearance (the oracle baseline's own measured
margin, itself already close to failing skills' historical near-misses) into
a 0.0540 m one — short of the 0.1 m gate.

**This is reported as a finding, not tuned.** No retreat-gate threshold,
waypoint constant, or IK tolerance was touched to make this pass. Per this
module's task instruction, a hybrid fallback (scripted/oracle `handoff`,
perception-driven `pick`/`place`) is an available, undecided option — this
document does not decide it.

## Task 5 — grounder → executor → render

`scripts/run_grounded_demo.py`. Two findings surfaced here too, both
reproducing identically in oracle mode (i.e. neither is a perception
regression):

1. The brief's literal command, "Pick up the fork with arm A and hand it to
   arm B", does not parse under M05's frozen grammar (`hand` alone is not
   `hand off`/`handoff`/`pass`/`give`).
2. Grounding the grammar-supported two-clause form ("Pick up the fork with
   arm A and give it to arm B.") produces `pick(A, fork)` then
   `handoff(B, A, fork)` as two separate `SkillCall`s — and running them in
   sequence fails, because `run_handoff`'s own Phase 1 unconditionally
   re-picks the object via a nested `run_pick`, with no `already_held`
   branch the way `run_place` has one. Verified directly, oracle mode:
   `pick(A, fork)` succeeds, then `handoff(B, A, fork)` fails immediately
   with `weld_attach_failed_after_300_frames`. This is a pre-existing M06
   gap, out of this module's scope to fix.

`scripts/run_grounded_demo.py` therefore grounds the single grammar-supported
clause that already carries the intended semantics — `to_arm` is mandatory
and `from_arm` defaults to "the other arm" (`docs/command-grammar.md`), so
"Give the fork to arm B." alone means "arm A picks up the fork and hands it
to arm B":

```
Command: 'Give the fork to arm B.'
TaskPlan (1 skills):
  SkillCall(skill='handoff', arm='B', target_object='fork', params={'from_arm': 'A'})
executed handoff(B, fork): success=True reason=held by arm B: z=0.5498 ...
Final state: is_holding('A')=None (expect None), is_holding('B')='fork' (expect 'fork')
VERIFICATION: PASS
```

`is_holding('A') is None` and `is_holding('B') == 'fork'` both confirmed.
This run used oracle mode (`ScriptedSkillExecutor()`, no `inference`) —
Task 5 asks for the grounder pipeline end to end, not specifically for
perception-mode execution, and `handoff` is the one skill this module's own
vision-mode run (above) found does not currently pass with perception on.

**Rendered `docs/images/m10-demo-end-to-end.png` (front camera) does NOT
clearly show the end state.** At this framing/zoom, the fork is a small red
sliver between the two gripper jaws and it is not visually obvious which arm
holds it — the actual verification is the programmatic `is_holding` check
above, not the image. Stated plainly rather than implied otherwise.

## Constraints honoured

- `grasp.py`, `ik.py`, `gen_dual_scene.py`, `scenes/so101/`, `env.py`, the
  checkpoint, and the IR were not modified.
- No retraining, no reconversion, `ov_env` untouched (verified: `pytest`,
  `verify_adr038_skills.py` both reproduce their pre-existing numbers).
- `bimanual.perception.posenet`'s `torch` import made this module's runtime
  classes (`inference.py`, `cached_access.py`) unable to import
  `PROP_ORDER`/etc. from it directly in `ov_env` (no `torch` there,
  `scripts/requirements-bmptl.txt`) — both files duplicate the three
  constants instead of importing them, documented inline as a deliberate,
  disclosed departure with a note to keep them in sync manually.
