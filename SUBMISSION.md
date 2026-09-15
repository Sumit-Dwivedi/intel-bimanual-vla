# Submission Checklist

Track every deliverable required by the hackathon platform. Docs-writer
updates status marks as artifacts land. Nothing here is written on Day 5
that wasn't planned from Day 1.

## Status snapshot (updated Sept 14, 2026)

Complete and evidenced: M01 (repo scaffold, pinned envs, `scripts/verify_env.py`),
M02 (dual-arm MuJoCo scene, `TableSettingEnv`, opt-in rendering — ADR-022), M03
(OpenVINO conversion smoke test on CPU/GPU/NPU — `benchmarks/ov-smoke-notes.md`),
M04 (`CommandSource` abstraction, text + voice stub, 11 tests), M05 (rule grounder,
43 tests, `docs/command-grammar.md`). **Updated Sept 15, 2026:** M15 (Speechmatics
voice input) — unscheduled/droppable as of Sept 12 — was finished afterward against a
live key: `VoiceCommandSource` now performs real Speechmatics batch transcription (the
M04 stub's placeholder text is gone) and `run_demo.py --voice PATH` runs a
transcribe-ground-execute path end to end, verified PASS on bm-ptl. Three disclosed
deviations from the original plan (execution location, credential variable name,
Speechmatics operating-point tier) — see `DECISIONS.md`/`ARCHITECTURE.md` ADR-058 and
the Speechmatics bonus row below. `tests/test_command_source.py` is now 19 tests (was
11), all network-mocked.

**M06 (scripted manipulation skills) is COMPLETE for four of the eight skill/arm/
object combinations exercised to date, verified by direct measurement on bm-ptl,
not assumed (ADR-029 through ADR-038).** Grasping is implemented as a MuJoCo weld
equality constraint, toggled on proximity plus jaw closure — an explicit
abstraction of contact-based grasping, not friction-based finger contact (ADR-029),
disclosed as such per ADR-015. The remaining four combinations fail for
already-diagnosed, documented reasons (below), not silently.

**Working, verified end-to-end (seed 0, bm-ptl, `mujoco==3.2.7`):**
- `pick(A, fork)`
- `place(A, fork, table)`
- `pick(A, water_bottle)`
- `handoff(A→B, fork)`

**Not working, documented not silently dropped:**
- `place(water_bottle, table)` — IK residual 0.0138 m against the 0.01 m
  tolerance at the destination-approach waypoint, unimproved at 4x the step
  budget, with the destination x landing on the place path's own `+0.30`
  safety-clip bound (ADR-034).
- `pick(A, mug)` — waypoint 1 (approach) convergence failure, IK residual
  0.0532 m against the 0.01 m tolerance: arm A cannot reach the mug's
  grasp-point hover position from its current base placement (already-
  documented reach limit, same class of failure ADR-027 first reported).
- `open_drawer` — structurally blocked: the arm bases are mounted at tabletop
  height and cannot reach beneath the slab.
- `handoff(B→A, fork)` — fails at phase 1: `pick(B, fork)`'s own APPROACH does
  not converge (IK residual 0.0954 m against the 0.01 m tolerance). This is a
  pre-existing kinematic reach limit of **arm B's** own base placement and
  approach angle to this fork position — arm B has never been shown able to
  pick this fork — not arm A's, and unrelated to the ADR-037 choreography
  change. Re-measured directly for this update via
  `scripts/run_skill.py --skill handoff --object fork --arm A --from-arm B
  --seed 0` on bm-ptl (HEAD `4d69c30`): identical result
  (`reason: phase 1 (from_arm pick) failed (waypoint 1 (approach) failed
  [convergence (IK residual=0.0954 m >= 0.01 m)])`), matching ADR-037's own
  recorded measurement.

`pytest tests/test_skills.py`: 4 passed / 4 failed (same four tests failing
throughout ADR-034 through ADR-038, no regression).

**Cut from scope:** ACT/policy training (ADR-023, user-ratified Sept 12) — no
learned policy exists; the scripted controller is the shipped policy, and brief p2
objective 4 (train/fine-tune with LeRobot) is unmet. `pour` was dropped at the
pre-committed cut-ladder gate — consequence: **the brief's own verbatim example
command, which ends "pour water into the mug," can no longer run end-to-end**, even
now that M06's four working skills are proven, because `pour` itself does not exist.
`docs/command-grammar.md` and M05's grounder still parse that exact command correctly
to a 5-skill plan; only execution of the last skill is out of scope.

**Updated Sept 14, 2026:** four skill/arm/object combinations (`pick(A, fork)`,
`place(A, fork, table)`, `pick(A, water_bottle)`, `handoff(A→B, fork)`) now complete
end-to-end and are available for the 10-seed robustness recording and demo video
content — see the Working list above. The four listed as not working are the ones
still at risk; a full brief-example command (which ends in `pour`, out of scope) is
still not reachable end-to-end.

## Basic Information
- [ ] Project title: [DRAFT: Bimanual VLA on Intel Core Ultra — a voice-driven table-setting demo]
- [ ] Short description (1-2 sentences): TODO — cannot be finalized until it's decided
  whether to describe the demo as "table-setting" (accurate to what M02-M05 support) or
  keep the brief's original "set the table and pour water" framing (currently unreachable
  end-to-end per the M06a/`pour` status above).
- [ ] Long description (~300 words): TODO — same dependency as above.
- [ ] Technology tags: OpenVINO, MuJoCo, PyTorch, Speechmatics, Intel Core Ultra
  (**Corrected Sept 12:** `LeRobot` removed — ADR-023 cut ACT training and the LeRobot
  dataset from the critical path, so nothing in the shipped pipeline uses it. `SmolVLA`
  removed — it appears nowhere in PLAN.md, ARCHITECTURE.md or the code and was never a
  grounded choice. Judge-facing copy must not claim tools we do not use, per ADR-015.
  **Re-verified Sept 12: still correct** — no code added since references LeRobot,
  SmolVLA, or trains any policy. **Updated Sept 15, 2026:** `Speechmatics` is now a
  shipped-and-verified tag, not merely a planned one — see the Speechmatics bonus
  section below and `DECISIONS.md`/`ARCHITECTURE.md` ADR-058.)
- [ ] Category tags: Robotics, Physical AI, Edge AI

## Cover Image & Presentation
- [ ] Cover image (1200x630 recommended): TODO — not produced. Candidate source material
  exists (`docs/images/m02-scene.png`, `docs/images/m02-drawer-view-open.png`,
  `docs/images/m02-drawer-view-closed.png`, `docs/images/m06-grip-diagnostic-frame30.png`,
  and now `docs/images/m06-handoff-complete.png`, which does show a completed
  `handoff(A→B, fork)` — see the Working list above). Brief still needs a decision on
  final crop/framing for the 1200x630 cover slot; not resolved by this update.
- [ ] Video presentation (demo + narration): TODO — see `docs/video-script.md` (not yet
  written). **At risk**: a script cannot honestly narrate an end-to-end pick/place/pour
  sequence today; see status snapshot above.
- [ ] Slide presentation (5-10 slides): TODO — see `docs/slides.md` (not yet written).

## App Hosting & Repository
- [ ] Public GitHub repo: [URL TODO — currently private, flip to public on Sept 15]
- [ ] Demo application platform: N/A (this is a local sim, not a hosted app) — clarify with lablab if in doubt
- [ ] Application URL: link to GitHub repo README or a recorded demo page

## Rubric Alignment (Intel + platform) — Evidence Map

Every row points to a file, ADR, or measured result a judge can open directly.
An empty Evidence cell means no artifact exists yet — marked TODO, not
asserted. Points and criteria wording are unchanged from the platform's own
rubric; only the presentation is new.

### Intel-specific (100 pts)

| Criterion (pts) | Status | Evidence | What it shows / where it stops |
|---|---|---|---|
| End-to-end task completion & bimanual (30) | **MET, narrowly** | ADR-037 (sequential one-arm-at-a-time choreography fix), ADR-038 (render legibility); `docs/images/m06-handoff-complete.png`; `docs/videos/full-sequence-demo.mp4`; `docs/videos/m06-handoff-clip.mp4`; `scripts/render_full_sequence.py`; `run_demo.py` (reported 4/4 PASS, exit 0, bm-ptl) | `handoff(A→B, fork)` completes end-to-end with genuine two-arm coordination (`weld.is_holding('B')=='fork'`, `weld.is_holding('A') is None`, `from_arm_clear=True`), following the sequential choreography in Wan, Ramos, Yang, Garrett 2025 (NVIDIA), "Learning to Plan & Schedule with Reinforcement-Learned Bimanual Robot Skills", https://arxiv.org/html/2510.25634v1, and consistent with "Trajectory planning system for bimanual robots" (2025), https://www.sciencedirect.com/science/article/pii/S0921889025002155 (title/URL only; no verified author). Three further single-arm skills also complete end-to-end: `pick(A, fork)`, `place(A, fork, table)`, `pick(A, water_bottle)`. Grasping is an explicit MuJoCo weld-constraint abstraction, not friction contact (ADR-029, disclosed per ADR-015). Four other skill/arm/object combinations do not work (see Working/Not working lists above), and `pour` is out of scope (ADR-023), so the brief's own literal example command does not run end-to-end. **On composition:** pick, handoff and place DO run as one continuous episode — but only when the sequence is expressed as TWO calls, not three. `run_handoff`'s own Phase 1 is a nested `run_pick`, so `run_handoff(env, 'B', 'A', 'fork')` followed by `run_place(env, 'B', 'fork', 'table')` covers all three phases in a single 8410-step episode (handoff 6610 steps success=True; place 1800 steps success=True; fork ends at z=0.3537 on a table surface at z=0.35, weld released). That is what `docs/videos/full-sequence-demo.mp4` records, in one continuous fixed-camera take. The literal THREE-call form still fails, and was re-verified as failing: `scripts/chained_demo.py` runs `pick(A, fork)`, then `handoff(A->B, fork)`, then `place(B, fork, table)`: step 1 passes; step 2 clears the already-held guard added in ADR-054 but fails at Phase 3 (`to_arm` approach, IK residual 0.0875 m, then a cross-arm collision when staging to y=-0.06); step 3 is never reached. The difference is the leading explicit `run_pick`, which is redundant with the handoff's own Phase 1 (whose guard then skips it) and leaves arm A entering Phase 2 in a different configuration than the nested pick would have. The mechanism has not been isolated further; what is measured is that the two-call form succeeds and the three-call form does not. Reported this way because "four skills complete end-to-end" should not be read as "any call sequence composes": the composition that works is the two-call one, and the three-call failure point is measured and named rather than glossed. **On the clips:** `docs/videos/full-sequence-demo.mp4` is now the primary video evidence — 40.0 s, 1280x720, 30 fps, one continuous fixed-camera take showing the pick, the transfer and the place. Note that two non-target props are disturbed during it: the mug is knocked onto its side and the water bottle falls off the table. The scripted skills have no collision avoidance for props they are not acting on; this is disclosed rather than reframed away, and it does not affect the fork result. The older `docs/videos/m06-handoff-clip.mp4` is retained but superseded: its first roughly two-thirds do not read as a handoff, because the camera is pinned to the final gripper pose, so only the last ~0.5 s is legible. |
| VLA / multi-modal reasoning (20) | Partial | M04 `CommandSource` ABC; M05 rule grounder; `tests/test_grounder.py` (43 tests); `docs/command-grammar.md` | Text command → grounded skill plan works and is the path the demo actually runs. This is a rule-based grounder, not a learned VLA model — ACT/policy training is cut (ADR-023). PoseNet perception is trained and benchmarked (M10) but is opt-in and **off by default** in the demo (oracle mode, ADR-046): the grounder is in the demo path, perception is not. Do not read this row as claiming perception-driven multi-modal reasoning. |
| OpenVINO & Core Ultra optimization (20) | Partial | ADR-045 (PoseNet → OpenVINO IR, FP32/FP16, benchmarked CPU/iGPU/NPU) and ADR-050 (INT8 PoseNet quantization via NNCF, benchmarked CPU/iGPU/NPU); `benchmarks/ov-smoke-notes.md`; `docs/hardware/m10-phase4-benchmark.md` | The conversion pipeline is proven on all three devices twice over: a smoke test on a placeholder ResNet18-scale encoder (`ov-smoke-notes.md`, max abs deviation vs PyTorch 5.7e-05 to 1.1e-04) and the real PoseNet model in FP32/FP16 (ADR-045). INT8 was also converted and benchmarked (ADR-050) but its 36-37 mm deviation — an order of magnitude above PoseNet's own 2.6-3.2 mm ground-truth MAE — made it unfit to ship; see the Innovation row for why that is reported as a finding, not a gap. |
| Robustness across 10 seeds (15) | Narrow, measured per skill | ADR-053, `docs/hardware/m08-extended-eval.md` (20 seeds); ADR-049, `docs/hardware/m08-eval.md` (original 10 seeds); ADR-051 and its section 9, `docs/hardware/m10-handoff-perturbation.md` | **Each skill carries its own rate — these are not interchangeable.** Track A (each skill's own target prop displaced within its own measured envelope, roughly ±10-20 mm), 20 seeds: `pick(A, fork)` **20/20**; `place(A, fork, table)` **20/20**; `pick(A, water_bottle)` **9/20 (45%)**; `handoff(A→B, fork)` **20/20 but degenerate** — its Track A envelope is a single point (`docs/hardware/m07-envelopes.md`), so every seed runs the byte-identical scenario and this measures determinism, not robustness. **Methodology:** the initial 10-seed evaluation (ADR-049) measured `pick(A, water_bottle)` at 6/10. Extending to 20 seeds (ADR-053) revised this to 9/20 (45%), with seeds 0-9 reproducing bit-for-bit — the original figure was small-sample optimism, not a behavioural change. We report the 20-seed figure. **`handoff` is not robust by any measurement taken:** it reproduces only at its single exact tuned configuration, and fails under prop displacement (Track B, 0/10, ADR-049), a 2.2 mm perception offset (ADR-046), and ±0.003 rad arm-angle noise (1/5, ADR-051 section 9). No robustness adjective should be attached to `handoff` anywhere in judge-facing copy, and its 20/20 must never appear without the degenerate qualifier. |
| Reproducibility (10) | Met, for what ships | `scripts/verify_env.py`; `scripts/requirements-dev.txt` / `scripts/requirements-bmptl.txt`; `run_demo.py` (reported 4/4 PASS, exit 0, bm-ptl) | Pinned environments plus a single entry point reproduce the four working skills end-to-end. `pytest tests/test_skills.py` is **4 passed / 4 failed** — never describe the suite as passing; the four failures are the same already-diagnosed combinations, not a regression. |
| Innovation (5) | Evidenced | ADR-029 (weld-constraint grasping abstraction), following the ADR-028 fix for MuJoCo issue #239's finger-pad mesh collapse; ADR-050 and `docs/hardware/m10-phase4-benchmark.md` (INT8 PoseNet quantization measured, then declined for shipping) | Two concrete instances, not one technique: an explicit, disclosed physics abstraction built only after diagnosing and fixing an upstream MuJoCo geometry bug, and a quantization pass that was measured against the model's own ground-truth error (36-37 mm vs. 2.6-3.2 mm MAE) and rejected on that evidence rather than shipped for its speedup alone. A measurement changing a decision is the rarer signal here. |

### Platform-general

No point values are published for these by the platform; no dedicated
write-up exists yet for any of them beyond what the Intel-specific table
above already documents.

| Criterion | Status | Evidence |
|---|---|---|
| Application of Technology | TODO | — |
| Presentation | TODO | — |
| Business Value | TODO | — |
| Originality | TODO | — |

### Speechmatics bonus

**Updated Sept 15, 2026.** M15 was finished (previously unscheduled/droppable, cut-ladder
rung 1). Three deviations from the original plan wording were required and are disclosed,
not silent — see `DECISIONS.md`/`ARCHITECTURE.md` ADR-058: (1) exercised end to end on
bm-ptl rather than laptop-only, because MuJoCo is bm-ptl-only (ADR-020); (2) the real
`.env` credential variable is `ai_infra`, not ADR-019's originally written
`SPEECHMATICS_API_KEY`; (3) Speechmatics `operating_point="enhanced"`, not "standard" —
measured necessary because "standard" misrecognized the verified test phrase's trailing
single-letter arm ID ("...arm be" instead of "...arm B").

| Criterion | Status | Evidence |
|---|---|---|
| Speech input wired to VLA text pipeline | **Done** | `src/bimanual/command/voice_source.py` — `VoiceCommandSource` performs real Speechmatics batch transcription (submit/poll/fetch) via `urllib` only (no SDK, no new venv); `run_demo.py --voice PATH` transcribes, grounds via the existing `RuleGrounder`, and executes via the existing `ScriptedSkillExecutor`, reporting in the same format the default 4-skill run uses. `tests/test_command_source.py`, 19/19 passed (network-mocked), covers success and six distinct failure modes (missing key, auth rejection, network unreachable, job timeout, empty transcript, malformed WAV). `DECISIONS.md`/`ARCHITECTURE.md` ADR-058. |
| Working demo clip with voice command | Not done | No video recorded for this path yet — TODO, distinct from the text-command evidence (`docs/videos/full-sequence-demo.mp4`, and the older `docs/videos/m06-handoff-clip.mp4`) already cited elsewhere in this document. The command-line run itself is verified end to end (below) and is reproducible for a future recording. |
| End-to-end run, measured (bm-ptl, `ov_env`, real Speechmatics API, `data/voice/give_fork_to_arm_b.wav` — synthesized speech, not a placeholder file) | **Done** | Transcript `'Give the fork to arm B'` (confidence 0.983) → grounded to `handoff(arm=B, target=fork, params={'from_arm': 'A'})` → **PASS** (`frames=6610`, `fork z=0.5498`, `lateral_separation=0.1946 m` — identical to the same handoff step in the default 4-skill run) → exit 0, ~20.6 s. The 32-character API key was confirmed, by an in-process substring check against the full captured output, to never appear in it. Regression gate (`run_demo.py`, no args) unaffected: 4/4 PASS, exit 0, ~24.8 s, byte-identical before/after this change. |

## Assets to Produce
- [ ] `docs/video-script.md` — narrated walkthrough script. TODO. Should not be written
  as if the pipeline completes end-to-end until M06a and the `pour` scope decision are
  resolved — see status snapshot above.
- [ ] `docs/slides.md` — outline for slide deck. TODO.
- [ ] `docs/cover-image-brief.md` — what the cover image shows. TODO.
- [ ] `benchmarks/bmptl-results.md` — device latency table for README. TODO — distinct
  from `benchmarks/ov-smoke-notes.md` (M03's correctness smoke test, done); this is M14's
  latency/throughput benchmark and has not been produced.
- [ ] `README.md` — judge-facing overview. TODO — file currently exists but is empty.
  Status section should list M01-M05 as complete and M06a as incomplete
  (pending weld-vs-reposition decision, Day 4 AM) once written, per this file.
