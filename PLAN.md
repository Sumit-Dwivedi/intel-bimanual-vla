# PLAN — Bimanual VLA Manipulation on Intel Core Ultra

Planner output. No code here. Every module below is a work package of **at most one
day**, with a named responsible agent, an execution target (laptop / kaggle / bm-ptl),
explicit dependencies, and done-when criteria that the Tester can verify by running a
command.

Grounding for this plan:
- `CONSTRAINTS.md:4-5` (deadlines), `:8-11` (bm-ptl hardware), `:21-27` (deliverables),
  `:30-36` (rubric weights), `:38-41` (Speechmatics bonus), `:43-48` (out of scope),
  `:50-52` (fallback), `:54-58` (compute targets), `:60-65` (Day 0 status).
- `docs/challenge/challenge-brief.pdf` p1 (target scenario), p2 (five technical
  objectives), p3 (deployment must run on Core Ultra Series 2/3), p4 (five required
  deliverables + recommended demo sequence), p5 (100-point rubric).
- `docs/hardware/bmptl-verification.md:4-7` (CPU / Arc B390 iGPU / NPU5010 / 32 GB),
  `:10` (OpenVINO sees CPU, GPU, NPU), `:12-16` (trivial-op latencies), `:17-20`
  (dispatch-overhead caveat).
- `SUBMISSION.md:8-12`, `:15-17`, `:20-22`, `:39-41`, `:44-48` (submission artifacts).
- `.claude/agents/builder.md:30-34` (Windows-first coding defaults this plan assumes).

I have not executed anything. No latency, success-rate, or throughput number appears in
this plan as a claim — only as a **target threshold for a decision gate**, to be measured
later by Tester.

---

## 1. Calendar and days remaining

| Day | Date | Status |
|---|---|---|
| Day 0 | Sept 10, 2026 | **Done.** bm-ptl reachable, OpenVINO sees CPU/GPU/NPU, trivial-model benchmark ran (`CONSTRAINTS.md:60-65`, `docs/hardware/bmptl-verification.md`). |
| Day 1 | Sept 11, 2026 | **Today.** Planning + scaffold + scene v0. |
| Day 2 | Sept 12, 2026 | Command layer, scripted controller, randomization, eval harness. |
| Day 3 | Sept 13, 2026 | Demo collection, perception model, ACT training launch. **GATE-1 at end of day.** |
| Day 4 | Sept 14, 2026 | OpenVINO export + quantization + bm-ptl benchmark, voice input. |
| Day 5 | Sept 15, 2026 | Full pipeline on bm-ptl, bug hunt, docs, repo public. |
| Day 6 | Sept 16, 2026 | Final 10-seed recorded run, submission assembly, submit. |

**Days remaining including today: 6 (Sept 11 through Sept 16).**
Submission is due Sept 16 (`CONSTRAINTS.md:4`). bm-ptl dies Sept 17 at 00:15 local
(`CONSTRAINTS.md:5`), i.e. there is **no bm-ptl access after the submission day**.
Therefore every artifact that must be produced on bm-ptl (benchmark table, final demo
recording) has a hard internal deadline of **Day 5 end**, with Day 6 reserved as the only
retry window. Treat Day 6 18:00 local as the freeze.

Day 1 is already partly consumed by planning. M01–M03 are deliberately small.

---

## 2. Agent assignment model

| Agent | Uses it for |
|---|---|
| **builder** | All implementation. One module at a time. Runs its own code before declaring done (`.claude/agents/builder.md:26-27`). |
| **tester** | Runs each finished module against its done-when criteria and reports PASS/FAIL/NOT TESTABLE with evidence. |
| **compliance-reviewer** | After each module, checks the code against this plan and against ARCHITECTURE.md ADRs. |
| **tutor** | After each module, writes `docs/learn/<module>.md` + GLOSSARY entries. Developer has no prior robotics experience (`CONSTRAINTS.md:14-15`). |
| **docs-writer** | README, DECISIONS.md mirroring of ADRs, SUBMISSION.md checkboxes, video script, slide outline, cover-image brief. |
| **bug-hunter** | **Day 5 only**, once the pipeline is end-to-end. |
| **planner** (me) | This file and ARCHITECTURE.md. Re-invoked only at GATE-1 or if a module is found to be mis-specified. |

Standing rule per module: **builder implements → tester verifies → compliance-reviewer
checks → tutor documents → docs-writer updates status.** Do not batch this; a module is
not closed until Tester has reported on it.

---

## 3. Target module map

Component names are fixed here and in ARCHITECTURE.md so the two files agree.

```
src/bimanual/
  command/      CommandSource, CommandEvent, TextCommandSource, VoiceCommandSource
  language/     Grounder, RuleGrounder, SkillCall, TaskPlan
  control/      Coordinator, IKSolver, scripted skill primitives, SkillExecutor
  policy/       PolicyBackend, TorchPolicyBackend, OpenVinoPolicyBackend
  perception/   PerceptionBackend, StatePerception, VisionPerception (PoseNet)
  sim/          TableSettingEnv, Randomizer, SceneConfig, assets/*.xml
  eval/         EvalHarness, EpisodeRecorder, metrics
scripts/        collect_demos, train_act, export_openvino, bench_openvino, run_demo
benchmarks/     bmptl-results.md, bmptl-environment.txt
docs/           learn/, video-script.md, slides.md, cover-image-brief.md
```

---

## 4. Modules

### Day 1

#### M01 — Repo scaffold and pinned environments
- **Purpose.** Create the package layout in section 3, three pinned requirement files
  (dev/laptop, kaggle, bm-ptl), and a `scripts/verify_env.py` that prints versions and
  exits non-zero on mismatch. Reproducibility is 10 rubric points
  (`CONSTRAINTS.md:34`, brief p5).
- **Inputs.** `scripts/requirements-bmptl.txt:1-3`, `benchmarks/bmptl-environment.txt`,
  `.gitignore`.
- **Outputs.** Package tree with `__init__.py` files, `scripts/requirements-dev.txt`
  (currently empty — see RISK-07), `scripts/requirements-kaggle.txt`,
  `scripts/verify_env.py`, `pyproject.toml` or `setup.cfg`.
- **Agent.** builder. **Runs on.** laptop.
- **Depends on.** Nothing.
- **Done when.**
  1. `python scripts/verify_env.py` exits 0 on the laptop and prints Python, numpy,
     mujoco, torch versions.
  2. `python -c "import bimanual"` succeeds from the repo root.
  3. The `openvino-telemetry` pin discrepancy in RISK-07 is resolved to one value in
     both files, and the resolution is noted in DECISIONS.md.
- **Budget.** 2 hours.

#### M02 — MuJoCo dual-SO-101 table scene v0
- **TODO(M02):** Dual-arm scene must include
  `<visual><global offwidth='1280' offheight='720'/></visual>` so demo video renders match
  target resolution. Do not modify upstream `scenes/so101/` per ADR-016; add the override in
  the new dual-arm scene file. (Upstream declares 640x480; confirmed by the bm-ptl probe.)
- **Purpose.** A loadable MJCF scene: table, two SO-101 arms mounted with an overlapping
  workspace, a drawer with a prismatic joint, plate, mug, fork, spoon; two cameras
  (overhead + front) plus per-arm wrist cameras if the asset allows. This is required
  deliverable 2 (brief p4) and the substrate for everything else.
- **Inputs.** An SO-101 MJCF asset — **source not yet decided, see RISK-01**. Brief p1
  ("Simulated Dual SO-101 Arms"), brief p3 (MuJoCo or compatible LeRobot Gym env).
- **Outputs.** `src/bimanual/sim/assets/so101_dual_table.xml` plus meshes, a
  `TableSettingEnv` wrapper exposing `reset(seed)`, `step(action)`,
  `render(camera)`, `get_state()`, and `scripts/view_scene.py`.
- **Agent.** builder. **Runs on.** laptop for MJCF authoring; bm-ptl for compile and
  render, per ADR-020 (MuJoCo cannot import on the laptop — Smart App Control blocks the
  unsigned `mujoco.dll`).
- **Depends on.** M01.
- **Done when.**
  1. `python scripts/view_scene.py --headless --save out/scene.png` writes a PNG showing
     both arms, the drawer and all four objects.
  2. `TableSettingEnv.reset(seed=0)` returns observations whose shapes are printed and
     recorded in the Tester report.
  3. The actuated DoF count per arm is **read off the asset and written into
     ARCHITECTURE.md's component table by the planner at GATE-1** — builder reports the
     number, does not assume it (see RISK-02).
  4. Physics is stable: 1000 steps with zero action produce no NaN and no object
     falling through the table.
- **Budget.** 6 hours. If the asset hunt exceeds 2 hours, escalate RISK-01 to the user
  rather than hand-authoring an arm.

#### M03 — Real-model OpenVINO conversion smoke test on bm-ptl
- **Purpose.** Day 0 proved device *enumeration* and a single-Add-op graph
  (`docs/hardware/bmptl-verification.md:12-16`). That is not a conversion path. This
  module proves **PyTorch → OpenVINO IR → compile → infer on CPU, GPU and NPU** for a
  small but real convolutional network with a static input shape, closing the
  "OpenVINO conversion tested on a trivial model on bm-ptl within the first 48 hours"
  requirement with something that actually exercises the converter.
- **Inputs.** bm-ptl access (`CONSTRAINTS.md:11`, `docs/hardware/Screenshot 2026-09-11
  132522.png` records the SSH jump-host string), `scripts/requirements-bmptl.txt`.
- **Outputs.** `scripts/ov_smoke.py`, `benchmarks/ov-smoke-notes.md` recording: IR files
  produced, which devices compiled successfully, which failed and the verbatim error,
  and whether the NPU required static shapes or a specific precision.
- **Agent.** builder. **Runs on.** bm-ptl (conversion may happen on laptop, compile and
  infer must happen on bm-ptl).
- **Depends on.** M01.
- **Done when.**
  1. An `.xml` + `.bin` IR pair exists on bm-ptl for a ResNet18-scale vision encoder.
  2. `ov_smoke.py --device CPU|GPU|NPU` completes on all three, or the failure is
     captured verbatim with the device name.
  3. Max absolute output deviation between the PyTorch reference and each OpenVINO
     device is recorded as a number in the notes file (no threshold asserted yet).
  4. Notes state explicitly whether dynamic batch/sequence shapes were accepted by NPU.
- **Budget.** 3 hours. **This module is a hard blocker for the whole OpenVINO story
  (20 rubric points, `CONSTRAINTS.md:32`). If it slips past Day 2, escalate immediately.**

---

### Day 2

#### M04 — CommandSource abstraction and text implementation
- **Purpose.** Isolate *how a command arrives* from *what the policy consumes*. The
  policy and grounder see only a `CommandEvent` carrying text; **no audio type crosses
  this boundary** (see ARCHITECTURE ADR-002). This is what makes the Speechmatics bonus
  (`CONSTRAINTS.md:38-41`) a drop-in rather than a rewrite.
- **Inputs.** Brief p1 natural-language command example.
- **Outputs.** `src/bimanual/command/events.py` (`CommandEvent`: `text`, `timestamp`,
  `source_id`, `confidence: float | None`, `raw_meta: dict`),
  `command/source.py` (abstract `CommandSource` with `poll() -> CommandEvent | None`
  and `close()`), `command/text_source.py` (from CLI arg, file, or stdin),
  plus unit tests.
- **Agent.** builder. **Runs on.** laptop.
- **Depends on.** M01.
- **Done when.**
  1. `pytest tests/test_command_source.py` passes, covering: normal text, empty string,
     whitespace-only, and an unknown-word command.
  2. `grep -ri "audio\|pcm\|wav\|microphone" src/bimanual/language src/bimanual/policy`
     returns no matches. This is the structural guarantee, not a comment.
  3. `TextCommandSource` and a stub voice source both satisfy the same abstract base
     (verified by a test that parameterises over both).
- **Budget.** 3 hours.

#### M05 — Rule grounder: instruction to skill plan
- **Purpose.** Turn `CommandEvent.text` into an ordered `TaskPlan` of `SkillCall`s with
  arm assignment. This is the deterministic floor of the "multi-modal reasoning" leg
  (20 rubric points, brief p5) and is the piece that must never fail on demo day.
- **Inputs.** M04 output; brief p1 example command ("Open the top drawer, pick up the
  plate with arm A, place it on the table, pick up the mug with arm B, pour water into
  the mug with arm A."); the skill vocabulary defined in ARCHITECTURE ADR-001.
- **Outputs.** `src/bimanual/language/skills.py` (`SkillCall`, `TaskPlan`),
  `language/grounder.py` (abstract `Grounder`), `language/rule_grounder.py`,
  `tests/test_grounder.py`, and `docs/command-grammar.md` listing every supported
  phrasing.
- **Agent.** builder. **Runs on.** laptop.
- **Depends on.** M04.
- **Done when.**
  1. The brief's verbatim example command parses to a `TaskPlan` whose skills, arms and
     target objects are printed and match `docs/command-grammar.md`.
  2. At least 8 command paraphrases parse correctly, and at least 3 unsupported commands
     raise a typed `UngroundedCommandError` rather than silently producing a partial plan.
  3. Arm assignment is explicit for every `SkillCall` (never `None`).
- **Budget.** 3 hours.

#### M06 — Scripted IK controller and skill primitives
- **Purpose.** Implement each skill as a scripted, IK-driven primitive:
  `open_drawer`, `pick(object, arm)`, `place(object, target, arm)`, `handoff(object,
  from_arm, to_arm)`, `pour(source, into, arm)`. **This module is dual-purpose:** it is
  the Day-3 fallback "policy" (`CONSTRAINTS.md:50-52`) *and* the demonstration generator
  that ACT will imitate (ARCHITECTURE ADR-004). It is therefore the single highest-value
  module in the plan and must not be deferred.
- **Inputs.** M02 env, M05 `TaskPlan`.
- **Outputs.** `src/bimanual/control/ik.py`, `control/skills_scripted.py`,
  `control/executor.py` (abstract `SkillExecutor` + `ScriptedSkillExecutor`),
  `scripts/run_skill.py`.
- **Agent.** builder. **Runs on.** laptop.
- **Depends on.** M02, M05.
- **Done when.**
  1. `python scripts/run_skill.py --skill pick --object plate --arm A --seed 0` ends with
     the plate's body attached/lifted above a stated height threshold, reported as a
     measured number by Tester.
  2. Every skill in the M05 vocabulary has an implementation that returns a typed
     `SkillResult(success: bool, reason: str, frames_used: int)`.
  3. No skill can run forever: each has a step budget and returns
     `success=False, reason="timeout"` when exceeded.
  4. Joint limits and self-collision are not violated during any single-skill run
     (MuJoCo contact/limit warnings captured in the Tester log).
- **Budget.** 8 hours. **If this overruns, split into M06a (ik + pick/place) and M06b
  (drawer + handoff + pour) and push M06b to Day 3 morning — but never past GATE-1.**

#### M07 — Domain randomization and deterministic seed mapping
- **Purpose.** Seed → `SceneConfig` → scene. Randomize initial object placement, object
  mass, friction, object scale/shape variant, lighting, and table/background texture,
  exactly the axes the brief names on p2 objective 3 and p5 (15 rubric points,
  `CONSTRAINTS.md:33`).
- **Inputs.** M02 env.
- **Outputs.** `src/bimanual/sim/randomize.py` (`SceneConfig` dataclass, `Randomizer`),
  `configs/randomization.yaml` with per-axis ranges, `scripts/dump_seed_configs.py`.
- **Agent.** builder. **Runs on.** laptop.
- **Depends on.** M02.
- **Done when.**
  1. `python scripts/dump_seed_configs.py --seeds 0-9 --out out/seeds.json` writes 10
     configs that are **pairwise distinct** on at least placement, mass and lighting
     (Tester asserts distinctness programmatically — this is bug-hunter focus area
     `.claude/agents/bug-hunter.md:19`).
  2. The same seed produces a byte-identical `SceneConfig` across two separate process
     invocations.
  3. Every randomized range is bounded such that the scene remains solvable: a
     `--validate` flag runs `reset()` on all 10 seeds and reports zero physics errors and
     zero objects initialized in penetration.
  4. `configs/randomization.yaml` documents inclusive/exclusive bounds per axis
     (off-by-one is an explicit bug-hunter target).

- **Budget.** 4 hours.

#### M08 — 10-seed evaluation harness and episode recorder
- **Purpose.** Built on Day 2, not Day 6. Everything downstream — the fallback decision
  at GATE-1, the Torch-vs-OpenVINO accuracy-preservation check the brief demands on p3,
  and the required demo video across 10 seeds (brief p4) — depends on this harness
  existing before there is anything good to evaluate.
- **Inputs.** M06 executor, M07 randomizer.
- **Outputs.** `src/bimanual/eval/harness.py`, `eval/metrics.py`, `eval/recorder.py`,
  `scripts/evaluate.py`. Artifacts per run: `results/<run-id>/summary.json`,
  `per_seed.csv`, `seed_<n>.mp4`, and `manifest.json` capturing git SHA, host, device,
  precision, and the command text used.
- **Agent.** builder. **Runs on.** laptop (must also run unchanged on bm-ptl).
- **Depends on.** M06, M07.
- **Done when.**
  1. `python scripts/evaluate.py --seeds 0-9 --executor scripted --command "<brief
     example>" --out results/dev` produces 10 videos, a CSV with one row per seed, and a
     summary reporting per-skill completion counts and overall success count.
  2. Metrics are **per-subtask, not just binary**: drawer opened, fork placed, spoon
     placed, plate placed, mug handed off, pour pose achieved. Partial credit must be
     visible, because the rubric scores sequencing and coordination (brief p5).
  3. The harness is executor-agnostic: swapping `--executor scripted` for
     `--executor learned` requires no change to harness code (verified by
     compliance-reviewer against ADR-006).
  4. Re-running with the same seeds and executor reproduces the same summary numbers.
- **Budget.** 5 hours.

---

### Day 3

#### M09 — Demonstration dataset collection
- **Purpose.** Run the scripted controller over many randomized seeds and log
  observation/action pairs in LeRobot dataset format, so ACT can be trained
  (brief p2 objective 4: train or fine-tune using LeRobot or compatible tooling).
- **Inputs.** M06, M07, M08.
- **Outputs.** `scripts/collect_demos.py`, a dataset on disk plus a Kaggle-uploadable
  archive, and `docs/dataset-card.md` (episode count, cameras, resolution, action space,
  seeds used, success filter applied).
- **Agent.** builder. **Runs on.** laptop (dataset upload target: kaggle).
- **Depends on.** M06, M07, M08.
- **Done when.**
  1. Only successful episodes are kept; the filter criterion and the kept/attempted
     counts are reported by Tester.
  2. A `scripts/replay_episode.py` replays a logged episode and reports max joint
     deviation from the log, proving actions and observations are aligned in time
     (off-by-one here silently destroys imitation learning).
  3. The training seed set and the evaluation seed set 0–9 are **disjoint**, asserted in
     code. Evaluating on training seeds would invalidate the robustness claim.
  4. Dataset card states exactly which observation streams the policy will receive.
- **Budget.** 4 hours, mostly wall-clock collection. Start it early and work M10 while
  it runs.

#### M10 — Vision perception model (PoseNet) and its OpenVINO path
- **Purpose.** A small CNN mapping camera images to object poses / keypoints, trained on
  sim data with ground-truth labels. This exists for a structural reason: it makes
  OpenVINO **load-bearing on the fallback branch too**. If ACT fails at GATE-1, the
  scripted controller still consumes vision through an OpenVINO-compiled network on
  iGPU/NPU, so the 20-point OpenVINO criterion (`CONSTRAINTS.md:32`) is not contingent on
  the risky ML. See ADR-005 and ADR-009.
- **Inputs.** M09 dataset (images + privileged ground-truth poses), M03 conversion recipe.
- **Outputs.** `src/bimanual/perception/backend.py` (abstract `PerceptionBackend`),
  `perception/state_perception.py` (privileged, dev only),
  `perception/posenet.py`, `perception/vision_perception.py`,
  `scripts/train_posenet.py`, a checkpoint.
- **Agent.** builder. **Runs on.** kaggle for training, laptop for wiring.
- **Depends on.** M03, M09.
- **Done when.**
  1. Training completes and Tester reports held-out position error in metres (a measured
     number, no target asserted here).
  2. `ScriptedSkillExecutor` runs a full `TaskPlan` with
     `--perception vision` on at least one seed, i.e. no privileged state in the loop.
  3. `StatePerception` is gated behind an explicit `--perception state` flag and the
     README will state plainly that it is a development aid, not the demo path.
- **Budget.** 5 hours.

#### M11 — ACT policy training on Kaggle
- **Purpose.** Fine-tune/train an ACT-style imitation policy (LeRobot) on the M09 dataset,
  conditioned on camera observations, joint state, and the language/skill token from the
  grounder. This is the learned-policy branch (brief p2 objective 4).
- **Inputs.** M09 dataset, LeRobot, Kaggle GPU (`CONSTRAINTS.md:57`: 30 hrs/week free,
  CLI-driven; token present at `.kaggle/access_token`, ignored by `.gitignore:25`).
- **Outputs.** `scripts/train_act.py`, `configs/act.yaml`, a Kaggle notebook/kernel
  spec committed to the repo, a checkpoint pulled back to the laptop, and a training log
  with the loss curve saved as an artifact.
- **Agent.** builder. **Runs on.** kaggle.
- **Depends on.** M09.
- **Done when.**
  1. A Kaggle run completes and a checkpoint is downloaded locally.
  2. `scripts/evaluate.py --executor learned --seeds 0-9` runs to completion without
     error, whatever the success count is.
  3. Training is reproducible from a single committed command, with the dataset version
     and config hash recorded in the checkpoint metadata.
- **Budget.** 6 hours including queue time. **This is the module most likely to fail.
  Its failure is planned for, not fatal — see GATE-1.**

---

### GATE-1 — End of Day 3 (Sept 13): learned or scripted

`CONSTRAINTS.md:50-52` — "If learned policy is not working by end of Day 3, ship a
scripted IK-based controller as the 'policy.' Rubric rewards complete pipeline over
half-working ML."

Decide with evidence from `scripts/evaluate.py`, reported by **tester**. Decision owner:
the **user**, on planner's recommendation. Record the outcome in DECISIONS.md against
ADR-008.

**Take the learned branch only if all of these hold:**
1. M11 produced a checkpoint and `--executor learned` runs 10/10 seeds without crashing.
2. The learned executor completes at least the first two subtasks (drawer + one utensil
   placement) on **≥ 3 of 10** evaluation seeds.
3. Single-step policy inference latency on the laptop is low enough to be plausible for
   the bm-ptl demo — measured, not assumed.

**Otherwise take the scripted branch (F-modules below).** Do not spend Day 4 debugging
training. The scripted branch is a legitimate submission: brief p2 objective 1
(coordinated control, hand-off, collision-aware sequencing) and objective 3 (robustness)
are satisfied by M06 + M07, and objectives 2 and 5 are satisfied by the grounder plus the
OpenVINO-served PoseNet from M10.

**Hybrid is permitted and is the expected outcome:** scripted `open_drawer`, `handoff` and
`pour`, learned `pick`/`place` on whichever objects pass criterion 2. The harness is
executor-agnostic (M08 done-when 3), so a per-skill executor map costs little.

---

### Day 4

#### M12 — PolicyBackend abstraction (Torch and OpenVINO)
- **Purpose.** One interface, two backends, so the demo can switch runtime and device
  without touching control code, and so the Torch-vs-OpenVINO equivalence check is a
  single script.
- **Inputs.** Whichever model GATE-1 selected (ACT checkpoint and/or PoseNet).
- **Outputs.** `src/bimanual/policy/backend.py` (`PolicyBackend.predict(obs) -> action`,
  `PolicyBackend.describe() -> dict` returning device, precision, IR path),
  `policy/torch_backend.py`, `policy/openvino_backend.py`.
- **Agent.** builder. **Runs on.** laptop (must import cleanly on bm-ptl).
- **Depends on.** GATE-1, M03.
- **Done when.**
  1. Both backends satisfy one shared test parameterised over `["torch", "openvino"]`.
  2. `describe()` returns the **actually compiled** device string queried from the
     OpenVINO compiled model, not the requested one. Silent NPU→CPU fallback is an
     explicit bug-hunter target (`.claude/agents/bug-hunter.md:20`).
  3. Requesting an unavailable device raises rather than falling back silently; an
     opt-in `--allow-device-fallback` flag makes fallback explicit and logged.
- **Budget.** 4 hours.

#### M13 — OpenVINO export and INT8 quantization
- **Purpose.** Convert the selected model(s) to IR at FP32 and FP16, and produce an INT8
  variant via NNCF post-training quantization using a calibration set drawn from the M09
  dataset. Precision/quantization choices are explicitly scored (brief p5).
- **Inputs.** M12, M03 notes (especially whether NPU demanded static shapes), M09 data
  for calibration.
- **Outputs.** `scripts/export_openvino.py`, IR artifacts under `models/ir/<name>/<fp32|
  fp16|int8>/`, and `docs/openvino-export.md` recording the conversion command, static
  shapes used, calibration subset size, and any layer left un-quantized.
- **Agent.** builder. **Runs on.** laptop for export, bm-ptl for compile verification.
- **Depends on.** M12.
- **Done when.**
  1. Three IR variants exist and each compiles on at least one bm-ptl device.
  2. `scripts/export_openvino.py --verify` reports max absolute and mean absolute output
     deviation from the Torch reference for each variant, on a fixed held-out batch.
  3. Any variant that fails to compile on a device has the verbatim error recorded in
     `docs/openvino-export.md` — a documented limitation scores better than a silent gap.
- **Budget.** 5 hours.

#### M14 — bm-ptl benchmark script and results table
- **Purpose.** Required deliverable 3 (brief p4): a bench script on Core Ultra reporting
  latency, throughput, device selection and precision. Twenty rubric points ride on this
  (`CONSTRAINTS.md:32`).
- **Inputs.** M13 IR artifacts, `docs/hardware/bmptl-verification.md` as the format
  precedent, `scripts/requirements-bmptl.txt`.
- **Outputs.** `benchmarks/bench_openvino.py`, `benchmarks/bmptl-results.md`
  (the file `SUBMISSION.md:47` is waiting on), raw JSON per run.
- **Agent.** builder implements; **tester executes on bm-ptl and owns the numbers.**
- **Runs on.** bm-ptl.
- **Depends on.** M13.
- **Done when.**
  1. The script sweeps {CPU, GPU, NPU} × {FP32, FP16, INT8} × {batch 1} and emits mean,
     median, p95 latency and throughput, with warm-up iterations excluded and the
     iteration count recorded.
  2. Output records the compiled device reported by OpenVINO, the OpenVINO version, CPU
     model string, and driver versions.
  3. Unsupported device/precision combinations are reported as `UNSUPPORTED` with the
     error, never omitted and never silently substituted.
  4. `bmptl-results.md` carries a short interpretation paragraph in the same spirit as
     `docs/hardware/bmptl-verification.md:17-20` — dispatch overhead dominates tiny
     graphs, so ordering is only meaningful for realistic models. Do not let the report
     overclaim NPU wins.
- **Budget.** 5 hours.

#### M15 — VoiceCommandSource (Speechmatics) — droppable
- **Purpose.** Bonus award (`CONSTRAINTS.md:38-41`,
  `docs/challenge/Screenshot 2026-09-11 132235.png`): Speechmatics as front end to the
  existing text pipeline. Standard model, laptop-side, non-blocking. **The VLA never sees
  audio** (ADR-002).
- **Inputs.** M04 abstraction, mic capture on the laptop (`CONSTRAINTS.md:55`), a
  Speechmatics API key — **not present in the repo, see RISK-04**.
- **Outputs.** `src/bimanual/command/voice_source.py`, `scripts/voice_demo.py`,
  `docs/voice-setup.md`.
- **Agent.** builder. **Runs on.** laptop only.
- **Depends on.** M04, and M14 being finished or on track.
- **Done when.**
  1. Speaking the brief's example command produces a `CommandEvent` whose `text` grounds
     to the same `TaskPlan` as the typed command — asserted by a test comparing plans.
  2. The simulation loop never blocks on transcription: a `poll()` with no pending
     transcript returns `None` immediately, verified by timing the call.
  3. With no API key or no network, the program prints one clear message and continues in
     text mode. It never crashes the demo.
  4. Voice runs **only** on the laptop; nothing in the bm-ptl run path imports it.
- **Budget.** 3 hours. **Drop rule:** if not working by Day 4 end, delete it from the
  demo path and remove the Speechmatics claim from SUBMISSION.md and README rather than
  shipping a broken bonus (`CONSTRAINTS.md:41`).

---

### Day 5

#### M16 — Full pipeline on bm-ptl
- **Purpose.** Brief p3 requires MuJoCo **and** the inference pipeline to execute on a
  Core Ultra Series 2/3 system for the final demonstration. This module proves it, on the
  real machine, with a day of slack before the instance expires.
- **Inputs.** Everything above; `scripts/run_demo.py` as the single entry point.
- **Outputs.** `scripts/run_demo.py`, a bm-ptl run log, and one recorded seed video
  produced on bm-ptl.
- **Agent.** builder to fix breakage; tester to run and report.
- **Runs on.** bm-ptl.
- **Depends on.** M08, M12, M13, GATE-1 branch modules.
- **Done when.**
  1. `python scripts/run_demo.py --seed 0 --device GPU --precision fp16 --command
     "<brief example>"` completes on bm-ptl and writes a video.
  2. MuJoCo rendering works on bm-ptl over the SSH session, or an offscreen recording
     path is documented. `MUJOCO_GL` is not set on Windows
     (`.claude/agents/builder.md:34`). See RISK-03.
  3. The run log states: host, device actually compiled, precision, executor, git SHA.
- **Budget.** 5 hours. **Hard deadline. bm-ptl is gone after Sept 16
  (`CONSTRAINTS.md:5`).**

#### M17 — Bug-hunter pass and triage
- **Purpose.** Adversarially break the demo before a judge does.
- **Inputs.** End-to-end pipeline from M16.
- **Outputs.** A bug list with reproduction commands and severities; builder fixes only
  severity-high items that threaten the recorded demo.
- **Agent.** bug-hunter finds; builder fixes; tester re-verifies.
- **Runs on.** laptop and bm-ptl.
- **Depends on.** M16.
- **Done when.**
  1. All five focus areas in `.claude/agents/bug-hunter.md:17-23` have been probed and
     reported on, including whether the 10-seed eval genuinely varies the seed and
     whether NPU silently falls back to CPU.
  2. Every high-severity bug is either fixed-and-reverified or written into README as a
     known limitation. No silent unknowns.
- **Budget.** 4 hours.

#### M18 — Documentation, submission assets, repo public
- **Purpose.** Required deliverable 5 (brief p4) and every TODO in `SUBMISSION.md`.
- **Inputs.** Tester reports (the only legitimate source of numbers), ARCHITECTURE.md
  ADRs, `benchmarks/bmptl-results.md`.
- **Outputs.** `README.md`, `DECISIONS.md` (ADR mirror), `docs/video-script.md`,
  `docs/slides.md`, `docs/cover-image-brief.md`, `SUBMISSION.md` checkboxes ticked only
  where a real artifact exists (`.claude/agents/docs-writer.md:25-26`), repo flipped
  public (`SUBMISSION.md:20` says Sept 15).
- **Agent.** docs-writer. **Runs on.** laptop.
- **Depends on.** M14, M16, M17.
- **Done when.**
  1. README states what the system does, exact reproduction commands, the benchmark
     table, the 10-seed result, known limitations, and credits — readable by a judge in
     five minutes.
  2. Every number in README traces to a named Tester report or artifact file.
  3. The repo is public and a clean clone plus the documented setup steps get to a
     running `scripts/view_scene.py` — verified by tester from a fresh directory.
  4. README is explicit about what the policy sees (cameras + joint state) versus what
     the scripted controller used during data generation. No implied end-to-end learning
     that does not exist.
- **Budget.** 6 hours.

---

### Day 6

#### M19 — Final 10-seed recorded run and behaviour-preservation check
- **Purpose.** Required deliverable 4 (brief p4): a video across 10 randomized seeds,
  with command, scene variation and outcome verifiable. Plus brief p3's "preserve system
  behavior" — optimization must not materially degrade task success, which requires the
  comparison to actually be run.
- **Inputs.** M08 harness, M16 pipeline, M13 precisions.
- **Outputs.** `results/final/` with 10 videos, `per_seed.csv`, `summary.json`, a stitched
  demo video, and a Torch-vs-OpenVINO side-by-side success comparison table.
- **Agent.** tester runs; docs-writer writes it up.
- **Runs on.** bm-ptl.
- **Depends on.** M16, M17, M18.
- **Done when.**
  1. Ten videos exist, each captioned with seed, command text and outcome.
  2. The success count is reported honestly — including partial and failed seeds. Brief
     p4 asks for the success rate to be summarized; a truthful sub-10/10 with per-subtask
     breakdown is a valid and defensible submission.
  3. The same 10 seeds are evaluated under the Torch backend and the chosen OpenVINO
     precision, and both success counts appear in the table.
- **Budget.** 5 hours. Start first thing; do not let this be the 22:00 task.

#### M20 — Submission assembly and submit
- **Purpose.** Close out `SUBMISSION.md`.
- **Inputs.** All artifacts.
- **Outputs.** Submitted entry: title, short and long description, tags, cover image,
  video, slides, repo URL, application URL (`SUBMISSION.md:8-22`).
- **Agent.** docs-writer prepares; **user submits**.
- **Runs on.** laptop.
- **Depends on.** M19.
- **Done when.**
  1. Every `SUBMISSION.md` checkbox is either ticked with a pointer to a real artifact or
     explicitly struck as not-applicable.
  2. Submission is filed before the Sept 16 deadline (`CONSTRAINTS.md:4`).
  3. A final repo tag marks the submitted commit.
- **Budget.** 3 hours. **Freeze at 18:00 local Sept 16.**

---

## 5. Fallback branch (activated only if GATE-1 chooses scripted)

These replace M11's role. They are pre-specified now so that Day 4 is never spent
improvising.

#### F1 — Promote the scripted controller to "the policy"
- **Purpose.** Make the scripted executor the demo path, with vision (not privileged
  state) in the loop so the pipeline remains perception-to-action as the brief's core
  goal requires (p1).
- **Inputs.** M06 scripted executor, M10 OpenVINO-served PoseNet.
- **Outputs.** `--executor scripted --perception vision` as the documented default;
  a README section titled honestly (for example "Policy: scripted closed-loop controller
  over learned visual perception"), and a DECISIONS.md entry against ADR-008 recording
  why, with the GATE-1 evidence.
- **Agent.** builder + docs-writer. **Runs on.** laptop, then bm-ptl.
- **Depends on.** GATE-1 = scripted.
- **Done when.**
  1. `scripts/evaluate.py --seeds 0-9 --executor scripted --perception vision` completes
     all 10 seeds with a reported per-subtask breakdown.
  2. The README and video script never use the words "learned policy", "VLA policy" or
     "imitation learning" to describe the shipped control path. Naming discipline is the
     whole point of an honest fallback.
  3. Where the multi-modal reasoning points come from is stated plainly: language
     grounding (M05) plus OpenVINO visual perception (M10), not action generation.
- **Budget.** 4 hours.

#### F2 — Keep OpenVINO load-bearing on the fallback branch
- **Purpose.** Guarantee the 20-point OpenVINO criterion survives an ML failure.
- **Inputs.** M10 PoseNet, M13 export pipeline.
- **Outputs.** PoseNet IR at FP32/FP16/INT8 benchmarked across CPU/GPU/NPU in M14;
  the demo loop calling PoseNet through `OpenVinoPolicyBackend` on iGPU or NPU.
- **Agent.** builder. **Runs on.** laptop for export, bm-ptl for benchmark.
- **Depends on.** F1, M13, M14.
- **Done when.**
  1. `bmptl-results.md` contains a full device × precision table for PoseNet.
  2. `run_demo.py --device NPU` demonstrably executes perception on the NPU, with the
     compiled-device string logged from OpenVINO itself.
  3. A per-seed success comparison between FP32 and INT8 perception exists, addressing
     brief p3 "preserve system behavior".
- **Budget.** 3 hours.

#### F3 — Optional: revive learning later, with a hard stop
- **Purpose.** If Day 5 finishes early, retry ACT with the extra demonstrations collected
  since Day 3.
- **Rule.** Allowed only after M18 is complete. If it does not beat the scripted branch
  on the 10 evaluation seeds by Day 6 12:00, it is abandoned and never mentioned in the
  submission as working. Do not re-record the demo video after 14:00 on Day 6.
- **Agent.** builder. **Runs on.** kaggle, then laptop.
- **Done when.** A written decision in DECISIONS.md, either way.

---

## 6. Daily checkpoint ritual

End of each day, before stopping:
1. tester runs `scripts/evaluate.py` on seeds 0–9 with the current best executor and
   files a report. The demo must be recordable *every* evening from Day 3 onward.
2. docs-writer updates README status and the relevant `SUBMISSION.md` checkboxes.
3. Commit and push. A demo that exists only on one laptop is not a deliverable.
4. If a module's done-when criteria were not met, it rolls to the next day and the day's
   lowest-value module is cut. Cut list in priority order: **M15 (voice) → F3 → INT8
   quantization in M13 → wrist cameras in M02 → the pour skill in M06.**

---

## 7. Risk register and missing constraints

Flagged rather than guessed, per my honesty rules.

| ID | Issue | Evidence | Needed from user |
|---|---|---|---|
| RISK-01 | **No SO-101 MuJoCo asset in the repo and no source named anywhere.** M02, and therefore the entire plan, blocks on it. Provenance and licence also matter for a public repo (`CONSTRAINTS.md:21`). | Repo contains no MJCF/URDF/mesh files (full-tree glob). | Confirm the asset source (LeRobot SO-101 MJCF, the Intel Hack-a-thon Resources bundle referenced in `docs/challenge/Screenshot 2026-09-11 132213.png`, or another) and its licence. **Resolve on Day 1.** |
| RISK-02 | **SO-101 DoF and gripper configuration unconfirmed.** Action-space dimensionality drives the ACT config, the IK solver and the OpenVINO input shapes. I will not guess it. | Nothing in the repo states it. | Builder reports the actuated joint count from the adopted asset in M02; ARCHITECTURE.md's component table is updated then. |
| RISK-03 | **MuJoCo rendering on bm-ptl over SSH is unproven.** Day 0 verified OpenVINO only (`docs/hardware/bmptl-verification.md:9-16`). The final demo must run on that machine (brief p3) and the demo recording is assigned to it (`CONSTRAINTS.md:56`). | No rendering evidence on bm-ptl. | Probe during M03 while already logged in — 15 minutes of work that de-risks M16. If offscreen rendering fails, decide early whether the video is recorded on the laptop with the benchmark on bm-ptl, and state that split honestly in the README. |
| RISK-04 | **No Speechmatics API key or credit confirmation.** | `.gitignore:23` reserves `.env`; no key present. | Provide a key before M15 or accept the drop rule. Bonus only (`CONSTRAINTS.md:38-41`). |
| RISK-05 | **Kaggle GPU availability and queue times are unverified.** M11 and M10 training both depend on it. | `CONSTRAINTS.md:57` claims 30 hrs/week; `.kaggle/access_token` exists but I have not and will not read it. | Verify the Kaggle CLI can launch a trivial GPU kernel on Day 2, not Day 3. Add it as a 20-minute task inside M08's day. |
| RISK-06 | **"10 randomized seeds" success bar is ambiguous.** Brief p4 says "demonstrating successful task execution across 10 randomized environment seeds"; p5 says "Demonstration should include results across 10 randomized seeds" and the demo sequence says "summarize the success rate". | Brief p4, p5. | Plan assumes: run all 10, report the true rate with per-subtask breakdown, and show the successful ones prominently. Confirm the user agrees this is the right reading rather than cherry-picking 10 successes. |
| RISK-07 | **Dependency pin discrepancy.** `scripts/requirements-bmptl.txt:3` pins `openvino-telemetry==2025.2`; `benchmarks/bmptl-environment.txt:3` records `openvino-telemetry==2025.2.` with a trailing period. One of the two is wrong and it will break a reproducibility check. Also `scripts/requirements-dev.txt` is **empty**. | Both files read directly. | Builder resolves both in M01. |
| RISK-08 | **Pouring liquid is not simulated.** Brief p1's example command says "pour water into the mug". MuJoCo fluid simulation is out of scope for this timeline and "custom novel architectures" are excluded (`CONSTRAINTS.md:44-46`). | Brief p1 vs `CONSTRAINTS.md:43-48`. | Plan scopes `pour` as a **tilt-and-hold pose over the mug with no fluid particles**, scored on pose achievement, and says so plainly in the README. Confirm this is acceptable framing. See ADR-011. |
| RISK-09 | **Empty deliverable files.** `README.md`, `DECISIONS.md`, `LEARN.md`, `GLOSSARY.md` are all empty. If Day 6 slips, these are what a judge sees. | All four read as empty. | docs-writer seeds README skeleton on Day 2, not Day 5, and tutor starts LEARN.md after M02. |
| RISK-10 | **No video length/format requirement recorded.** `SUBMISSION.md:16` says "video presentation (demo + narration)" with no duration cap. | `SUBMISSION.md:14-17`. | Confirm the platform's limit before M19, so the stitched video is not rejected on a technicality. |

---

## 8. What is explicitly not in this plan

Per `CONSTRAINTS.md:43-48`: no custom novel architecture, no VLA trained from scratch, no
physical hardware, no chasing the top prize. Also deliberately excluded: Intel Geti,
Intel Physical AI Studio, and Open Edge Platform (brief p3 lists them as optional
resources, and adopting a new toolchain inside six days is a schedule risk with no rubric
line of its own). If a spare half-day appears, a short README paragraph on how the
pipeline would map onto those tools is cheaper and safer than an integration.
