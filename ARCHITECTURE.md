# ARCHITECTURE — Bimanual VLA Manipulation on Intel Core Ultra

Companion to `PLAN.md`. Component names here are authoritative; `PLAN.md` section 3 uses
the same names. Every decision below is numbered ADR-NNN so `DECISIONS.md` can reference
it by ID (`.claude/agents/docs-writer.md:19`).

Sources read before writing this: `CONSTRAINTS.md`,
`docs/challenge/challenge-brief.pdf` (5 pages), `docs/hardware/bmptl-verification.md`,
`docs/hardware/Screenshot 2026-09-11 132505.png`,
`docs/hardware/Screenshot 2026-09-11 132522.png`,
`docs/challenge/Screenshot 2026-09-11 132213.png`,
`docs/challenge/Screenshot 2026-09-11 132235.png`,
`docs/challenge/Screenshot 2026-09-11 132305.png`, `SUBMISSION.md`, the seven agent
definitions in `.claude/agents/`, `scripts/requirements-bmptl.txt`,
`benchmarks/bmptl-environment.txt`, `.gitignore`.

**I have run nothing.** No latency, accuracy or success figure in this document is an
observation. Where a number appears it is a decision threshold or a quoted Day-0 record
from `docs/hardware/bmptl-verification.md:12-16`.

---

## 1. System components

Five layers. Each boundary is an interface with at least two implementations, because
every boundary in this system is also a risk-mitigation seam.

```
                          ┌──────────────────────────────┐
  microphone (laptop) ──▶ │ CommandSource                │
  typed text / CLI   ──▶  │  TextCommandSource            │──▶ CommandEvent{text,...}
                          │  VoiceCommandSource           │       (text only; no audio
                          └──────────────────────────────┘        crosses this line)
                                        │
                                        ▼
                          ┌──────────────────────────────┐
                          │ Grounder                     │
                          │  RuleGrounder (floor)        │──▶ TaskPlan[SkillCall]
                          │  VlmGrounder (stretch)       │
                          └──────────────────────────────┘
                                        │
                                        ▼
                          ┌──────────────────────────────┐
                          │ Coordinator                  │  arm arbitration,
                          │  serializes conflicting      │  handoff insertion,
                          │  skills, tracks task state   │  replan on skill failure
                          └──────────────────────────────┘
                                        │  SkillCall
                                        ▼
                          ┌──────────────────────────────┐
                          │ SkillExecutor                │
                          │  ScriptedSkillExecutor (IK)  │──▶ joint targets, 20 Hz
                          │  LearnedSkillExecutor (ACT)  │
                          └──────────────────────────────┘
                              │                    ▲
                   PolicyBackend│                    │ObservationBundle
                   (torch | openvino)                │
                              ▼                    │
                          ┌──────────────────────────────┐
                          │ PerceptionBackend            │
                          │  VisionPerception (OV PoseNet)│
                          │  StatePerception (dev only)  │
                          └──────────────────────────────┘
                                        ▲
                                        │ images + qpos
                          ┌──────────────────────────────┐
                          │ TableSettingEnv (MuJoCo)     │
                          │  + Randomizer(seed)→SceneConfig│
                          └──────────────────────────────┘
                                        ▲
                          ┌──────────────────────────────┐
                          │ EvalHarness + EpisodeRecorder │  10 seeds → csv/json/mp4
                          └──────────────────────────────┘
```

### Component contracts

| Component | Responsibility | Key type |
|---|---|---|
| `CommandSource` | Deliver a natural-language command, non-blocking. | `poll() -> CommandEvent | None` |
| `CommandEvent` | `text`, `timestamp`, `source_id`, `confidence: float | None`, `raw_meta: dict`. | dataclass |
| `Grounder` | Text → ordered plan with explicit arm assignment. | `ground(CommandEvent, SceneBelief) -> TaskPlan` |
| `SkillCall` | `skill`, `arm`, `target_object`, `params: dict`. | dataclass |
| `Coordinator` | Owns task state across a multi-step sequence; decides what runs next and on which arm; inserts a handoff when an object must cross workspaces; handles skill failure. | `next() -> SkillCall | None` |
| `SkillExecutor` | Drive one `SkillCall` to completion or failure. | `execute(SkillCall, env) -> SkillResult` |
| `PolicyBackend` | Run a neural network. | `predict(obs) -> np.ndarray`, `describe() -> dict` |
| `PerceptionBackend` | Observation → `SceneBelief` (object poses, drawer opening, gripper contents). | `observe(obs) -> SceneBelief` |
| `TableSettingEnv` | MuJoCo wrapper. | `reset(seed)`, `step(action)`, `render(camera)`, `get_state()` |
| `Randomizer` | Deterministic `seed -> SceneConfig`. | pure function |
| `EvalHarness` | Run N seeds with a given executor/backend/device, emit artifacts. | CLI |

Arm action-space dimensionality is **deliberately left unspecified** until the SO-101
asset is adopted in module M02. See `PLAN.md` RISK-02. Guessing it would propagate a
wrong number into the ACT config and the OpenVINO input shape.

---

## 2. Data flow, one control cycle

1. `CommandSource.poll()` returns a `CommandEvent` (once per episode, or mid-episode for
   a re-command demo).
2. `Grounder.ground()` produces a `TaskPlan`. Done once, then cached; the Coordinator may
   request re-grounding if the scene contradicts the plan.
3. `TableSettingEnv` yields an `ObservationBundle`: overhead camera, front camera,
   optional wrist cameras, joint positions and velocities, gripper states.
4. `PerceptionBackend.observe()` turns images into a `SceneBelief`. On the demo path this
   is `VisionPerception`, which calls a PoseNet through `OpenVinoPolicyBackend` — this is
   where Intel hardware does work every single control step.
5. `Coordinator.next()` picks the current `SkillCall` given `SceneBelief` and task state.
6. `SkillExecutor.execute()` emits joint targets. `LearnedSkillExecutor` calls
   `PolicyBackend.predict()`; `ScriptedSkillExecutor` calls the IK solver.
7. `TableSettingEnv.step()` advances physics. `EpisodeRecorder` appends a frame and the
   per-step record.
8. On skill completion the Coordinator advances; on skill failure it applies the retry /
   abort rule in ADR-010.

Two flows exist that are **not** the demo path and must be labelled as such in the README:
- **Data generation:** `ScriptedSkillExecutor` + `StatePerception` (privileged simulator
  state) → LeRobot dataset. Fast and reliable, but privileged.
- **Development debugging:** any executor + `StatePerception`, behind an explicit
  `--perception state` flag.

---

## 3. Where each piece runs

| Component | laptop | kaggle | bm-ptl |
|---|---|---|---|
| MuJoCo sim, eval harness | **no** — blocked, ADR-020 | — | **all of it**: dev, data gen, eval, final demo (brief p3) |
| CommandSource: text | yes | — | yes |
| CommandSource: voice | **only here** (`CONSTRAINTS.md:40`) | — | never |
| Grounder | yes | — | yes |
| ~~ACT~~ / PoseNet training | no | **yes** (`CONSTRAINTS.md:57`) | no |
| OpenVINO export to IR | yes | — | verify compile |
| OpenVINO inference + benchmark | optional | — | **yes, the scored artifact** |

**ACT training is cut as of Sept 12 (ADR-023);** Kaggle's role is now PoseNet training
only. `LearnedSkillExecutor` in the section 1 diagram is specified but not built — it
survives as F3's target and as the reason `SkillExecutor` is an interface at all.

bm-ptl reaches end of reservation Sept 17 00:15 (`CONSTRAINTS.md:5`, corroborated by the
reservation table in `docs/hardware/Screenshot 2026-09-11 132505.png`), accessed via
`ssh -J guest@146.152.207.201 devcloud@192.168.2.2`
(`docs/hardware/Screenshot 2026-09-11 132522.png`). Hardware: Core Ultra X7 358H,
16 cores, Arc B390 iGPU, NPU5010, 32 GB
(`docs/hardware/bmptl-verification.md:4-7`).

---

## 4. Decisions

### ADR-001 — Two-tier control: language-to-skill planning above closed-loop skills

**Context.** The brief asks for multi-step sequencing, hand-offs, maintained task context
and plan adaptation (p1, p2 objectives 1–2, p5). Six days remain
(`PLAN.md` section 1) and training a VLA from scratch is out of scope
(`CONSTRAINTS.md:46`).

**Options.**
- (a) One end-to-end VLA consuming image + instruction, emitting joint actions for the
  whole 5-step sequence. Maximum rubric alignment on paper, but long-horizon multi-object
  sequences from a few hundred demonstrations in a few days is the single most likely way
  to end Day 5 with nothing recordable.
- (b) Two tiers: a symbolic task layer (instruction → ordered skills) over short-horizon
  closed-loop skills.
- (c) Pure scripted state machine with no learning at all. Forfeits the learning objective
  (brief p2 objective 4) and weakens objective 2.

**Decision.** (b). The instruction is grounded into a `TaskPlan` of short skills; each
skill is 1–3 seconds of closed-loop control. Learning is applied where imitation learning
is actually tractable at this horizon — per-skill `pick`/`place` — and the long horizon is
carried by the Coordinator's explicit task state.

**Consequences.** Multi-step context becomes inspectable and debuggable rather than latent,
which matters when a judge asks why step 4 ran before step 5. Failure is localized: a bad
`pick` does not corrupt the whole episode. The cost is that we cannot claim a single
monolithic VLA; the README must describe the architecture precisely and let it be judged
on what it is. It also enables the per-skill hybrid at GATE-1 (`PLAN.md` GATE-1), which
is the most likely shipping configuration.

---

### ADR-002 — CommandSource abstraction; the VLA never sees audio

**Context.** Speechmatics voice input is a bonus award, laptop-side, non-blocking, and
explicitly droppable (`CONSTRAINTS.md:38-41`;
`docs/challenge/Screenshot 2026-09-11 132235.png`). The policy needs a text instruction.

**Options.**
- (a) Feed audio into a multi-modal model directly. Adds an audio modality to the
  inference path, to the OpenVINO conversion surface, and to the bm-ptl demo. No rubric
  line rewards it.
- (b) One `CommandSource` interface with `TextCommandSource` and `VoiceCommandSource`
  implementations; voice transcribes to text on the laptop before the boundary.
- (c) Wire Speechmatics directly into the control loop with no abstraction. Fastest to
  type, and makes the drop rule a code deletion under deadline pressure.

**Decision.** (b). `VoiceCommandSource` converts speech to a `CommandEvent` carrying a
`text` field and optional `confidence`. No audio buffer, sample rate, or codec type exists
anywhere downstream of the boundary. `poll()` is non-blocking on both implementations:
the simulator loop never waits on a network transcription call.

**Consequences.** Dropping voice becomes a one-line default change (`PLAN.md` M15 drop
rule) rather than surgery. The bm-ptl demo path has zero audio dependencies, so no mic or
audio driver is needed on a remote Windows instance. Enforcement is structural and
testable: `PLAN.md` M04 done-when 2 greps `src/bimanual/language` and
`src/bimanual/policy` for audio terms and requires zero matches. The cost is one extra
indirection and a stub voice source in tests from Day 2 onward.

---

### ADR-003 — Rule-based grounding as the floor, VLM grounding as a bounded extra

**Context.** "VLA / multi-modal reasoning" is 20 points (`CONSTRAINTS.md:31`, brief p5)
and covers interpreting language *and* visual observations, maintaining context, and
adapting the plan. A general LLM/VLM planner is impressive and unreliable; the demo must
not depend on a model improvising a plan on stage.

**Options.**
- (a) Rule/grammar grounder only.
- (b) VLM planner only (local small VLM, converted to OpenVINO).
- (c) Rule grounder as the default, with an optional `VlmGrounder` used for scene-state
  verification and disambiguation ("which plate?"), each implementing the same `Grounder`
  interface.

**Decision.** (c). `RuleGrounder` is the demo default and covers the brief's example
command verbatim plus documented paraphrases (`PLAN.md` M05). `VlmGrounder` is a stretch
item after M14, never a dependency.

**Consequences.** The reasoning claim in the README must be scoped honestly: language is
grounded deterministically, visual reasoning enters through `PerceptionBackend` and
through the Coordinator's replanning on perceived state. That is genuinely multi-modal
closed-loop behaviour and should be described as exactly that, without borrowing the
credibility of a large VLA. A documented command grammar (`docs/command-grammar.md`) also
gives a judge something concrete to test, whereas an LLM planner gives them something to
break.

---

### ADR-004 — The scripted controller is both the fallback and the demonstration generator

**Context.** `CONSTRAINTS.md:50-52` mandates a scripted IK controller as the fallback if
learning is not working by end of Day 3. Imitation learning needs demonstrations and there
is no teleoperation rig and no physical robot (`CONSTRAINTS.md:16-17`, `:47`).

**Options.**
- (a) Build the scripted controller only if the fallback triggers. Zero wasted work if
  learning succeeds, but leaves no demonstration source, so learning cannot succeed.
- (b) Build the scripted controller first, on Day 2, and use it to generate the ACT
  training set. It is then already finished and tested if the fallback triggers.
- (c) Source demonstrations externally. No SO-101 dual-arm table-setting dataset is
  identified anywhere in this repo, and licence/asset alignment is unknown.

**Decision.** (b). Module M06 is scheduled on Day 2 and is the highest-priority module in
the plan.

**Consequences.** The fallback is not a panic path; by GATE-1 it is already the
best-tested component in the system, with a 10-seed evaluation already run against it
(`PLAN.md` M08). Learning is a strict upgrade on top of a working demo rather than a
prerequisite for having one. The honesty cost is real and must be paid in writing: ACT
would be imitating a scripted policy, so it cannot be claimed to exceed the scripted
controller's competence, and the README must say the demonstrations are
scripted-generated, not human teleoperated.

**Correction, Sept 12, 2026.** The scheduling detail in this ADR's Decision — "Module M06
is scheduled on Day 2 and is the highest-priority module in the plan" — no longer holds.
Under ADR-023, M06 moves to **Day 3 (Sept 13), 09:00–17:30, where it owns the whole day**
(`PLAN.md` section 1A.5, Day 3). ADR-023 supersedes this ADR on scheduling. It also
supersedes the *rationale*: with M11 cut, M06 is no longer the demonstration generator of
option (b) — there is nothing left to imitate — so M06 is the highest-priority module for
a different reason than the one recorded above. It is the shipped policy, not the training
source for one. The Consequences paragraph's forward-looking claims about ACT imitating a
scripted policy are consequently moot rather than wrong; they return only if F3 revives
M11. The Context, Options and original Decision text above are left unedited as the record
of what was decided on Sept 10 and why.

---

### ADR-005 — The demo path is vision-driven; privileged simulator state is development-only

**Context.** The brief's core goal is a perception-to-action pipeline reasoning over camera
observations (p1, p2 objective 2). MuJoCo makes exact object poses available for free, and
using them is the fastest way to a working controller — and the fastest way to a
submission that is technically a state-machine over ground truth while being narrated as
perception.

**Options.**
- (a) Privileged state everywhere. Fast, robust, and misrepresents the system.
- (b) Vision everywhere including data generation. Slow to converge and puts perception
  error inside the training labels.
- (c) Privileged state for data generation and debugging only; the demo and all 10-seed
  evaluation runs use `VisionPerception`.

**Decision.** (c). `StatePerception` exists but requires an explicit `--perception state`
flag, is never the default, and is named in the README as a development aid.

**Consequences.** OpenVINO inference sits in the closed loop on every control step, which
is what makes the 20-point optimization criterion substantive rather than a detached
benchmark script. Task success will be lower than with privileged state; that is the
honest number and it gets reported (`PLAN.md` M19 done-when 2). Perception error becomes a
real failure mode that must be characterised in the robustness section, notably under the
lighting and background randomization of ADR-012.

---

### ADR-006 — Executor-agnostic evaluation harness, built Day 2

**Context.** GATE-1 needs comparative evidence; the brief requires 10 randomized seeds
(p4, p5) and requires that optimization not degrade behaviour (p3). Evaluation harnesses
written on the last day produce numbers nobody trusts, and the bug-hunter's first listed
suspicion is a harness that silently reuses one seed
(`.claude/agents/bug-hunter.md:19`).

**Options.**
- (a) Evaluate ad hoc, script the 10-seed run at the end.
- (b) Build `EvalHarness` on Day 2 with `--executor`, `--perception`, `--backend`,
  `--device`, `--precision` as orthogonal flags, before there is anything good to measure.

**Decision.** (b). The harness is a Day-2 module (`PLAN.md` M08) and is the single tool
used for GATE-1, for the Torch-vs-OpenVINO comparison, and for the final video.

**Consequences.** Every later comparison is apples-to-apples by construction, because
there is only one measurement path. Metrics are per-subtask, not binary, so partial
progress is visible and Day-3 debugging has a gradient to follow. The cost is a Day-2 day
spent on infrastructure while the arms barely move — which is precisely the trade this
plan is making on purpose.

---

### ADR-007 — Two-backend policy runtime with no silent device fallback

**Context.** The benchmark must report device selection and precision (brief p4
deliverable 3). OpenVINO can quietly fall back to CPU when a device cannot compile a
graph, which would turn a reported "NPU" number into a fabricated claim — flagged as a
bug-hunter focus area (`.claude/agents/bug-hunter.md:20`).

**Options.**
- (a) Call OpenVINO directly from control code.
- (b) `PolicyBackend` interface with `TorchPolicyBackend` and `OpenVinoPolicyBackend`,
  where `describe()` reports the device string read back from the **compiled model**, and
  an unavailable device raises unless `--allow-device-fallback` is passed explicitly.

**Decision.** (b).

**Consequences.** Torch-vs-OpenVINO equivalence becomes a single parameterised test.
Every logged and published latency line carries the device OpenVINO actually used. A
device that cannot run our graph produces a documented `UNSUPPORTED` row
(`PLAN.md` M14 done-when 3) instead of a misattributed number; a documented limitation is
a better submission than a quiet lie.

---

### ADR-008 — GATE-1: a pre-committed, evidence-based fallback decision at end of Day 3

**Context.** `CONSTRAINTS.md:50-52` sets the deadline and the rationale: a complete
pipeline beats half-working ML. Under deadline pressure, sunk cost makes teams keep
debugging training.

**Options.**
- (a) Decide by feel on Day 4 or 5.
- (b) Pre-commit numeric criteria now, evaluate them with the M08 harness, and have Tester
  report them, with the user making the call.

**Decision.** (b). Criteria are fixed in `PLAN.md` GATE-1: a checkpoint exists; 10/10
seeds run without crashing; the first two subtasks complete on at least 3 of 10 seeds;
measured single-step latency is plausible for the bm-ptl demo. Failing any of these, the
scripted branch (F1/F2) ships. Per-skill hybrid is explicitly allowed and expected.

**Consequences.** The Day-4 conversation is about a table of numbers, not about optimism.
The threshold "3 of 10 on the first two subtasks" is a planning judgement, not a
measurement — I have observed nothing — and the user may move it before Day 3; it must be
fixed *before* the data arrives, not after. Whichever branch is taken, the outcome and its
evidence go into `DECISIONS.md` against this ADR, so the README's description of the
control path is traceable.

---

### ADR-009 — Put a small vision model in the loop so OpenVINO survives an ML failure

**Context.** OpenVINO / Core Ultra optimization is 20 points (`CONSTRAINTS.md:32`, brief
p5). If the only OpenVINO-converted model were the ACT policy, then an ACT failure at
GATE-1 would take a fifth of the rubric with it and leave only a benchmark of a model the
demo does not use.

**Options.**
- (a) Convert only the ACT policy.
- (b) Convert only an off-the-shelf pretrained encoder and benchmark it standalone. Easy,
  but the demo would not depend on it, and a benchmark of an unused model is a weak claim.
- (c) Train a small PoseNet (images → object poses) on labelled simulator data, serve it
  through `OpenVinoPolicyBackend` as `VisionPerception`, and additionally convert ACT if it
  survives GATE-1.

**Decision.** (c). PoseNet is `PLAN.md` M10, scheduled on Day 3 **before** the GATE-1
verdict, trained on cheap ground-truth labels.

**Consequences.** The OpenVINO story is decoupled from the riskiest module: on either
branch there is a converted model doing real work on every control step on Intel silicon,
with a full device × precision table (F2). PoseNet is also small enough that INT8
quantization and NPU execution are plausible targets, unlike a full VLA on a 32 GB client
machine. The cost is one extra model to train, export and document — bought with Day-3
hours that would otherwise sit idle waiting on a Kaggle queue.

---

### ADR-010 — Bimanual coordination: single action vector, arbitrated skills, scripted handoff

**Context.** Brief p2 objective 1 requires collision-aware sequencing, shared-workspace
reasoning and object hand-off; p4's demo sequence requires at least one hand-off or
complementary dual-arm action; 30 points ride on bimanual execution
(`CONSTRAINTS.md:30`).

**Options.**
- (a) Two independent per-arm controllers running concurrently. Simple until both reach
  into the shared workspace, then it is a collision generator.
- (b) One policy over the concatenated both-arm action space, always controlling both arms
  simultaneously. Truly bimanual, but it doubles the learning problem and most of the task
  is genuinely one-armed.
- (c) A single concatenated action vector at the environment interface, with the
  `Coordinator` arbitrating which arm is active per skill, holding the idle arm at a safe
  pose, and treating the shared workspace as a mutex that only a `handoff` skill may hold
  for both arms.

**Decision.** (c). `handoff` is a scripted primitive throughout, on both branches: arm A
presents the object at a defined transfer pose, arm B closes, arm A opens and retreats,
with explicit grip-state checks between phases.

**Consequences.** Collisions are prevented structurally by the mutex rather than hoped
away by a learned policy. The environment still exposes one concatenated action space, so
a future genuinely-bimanual policy needs no environment change. The honest framing: this
is coordinated sequential bimanual manipulation with a synchronized dual-arm transfer, not
continuous simultaneous dual-arm control, and the README must say so. Keeping `handoff`
scripted guarantees the demo has its required hand-off moment regardless of GATE-1.

---

### ADR-011 — Task scope: five subtasks; `pour` is a tilt pose, with no fluid simulation

**Context.** Brief p1 names opening a drawer, retrieving spoons and forks, picking up a
plate and cup, organizing items, and a command that ends "pour water into the mug with arm
A". Fluid simulation in MuJoCo is expensive and novel architecture work is out of scope
(`CONSTRAINTS.md:44-46`).

**Options.**
- (a) Attempt particle-based liquid. High cost, high risk, worth nothing on the rubric
  beyond what a tilt already demonstrates.
- (b) Omit `pour`. Leaves the brief's own example command unexecutable end-to-end, which a
  judge would notice.
- (c) Implement `pour` as a bimanual tilt: arm B holds the mug, arm A brings a bottle over
  the mug and achieves a tilt-and-hold pose within a stated tolerance. Success is pose
  achievement. No liquid exists and the README and video narration say so plainly.

**Decision.** (c). Core sequence: `open_drawer` → `pick`/`place` fork → `pick`/`place`
spoon → `pick`/`place` plate → `handoff` mug → `pour` over the mug.

**Consequences.** The brief's verbatim example command runs end to end, and `pour` doubles
as a second complementary dual-arm action (one arm holds, the other pours), directly
matching brief p1's own illustration. Visual honesty is a hard requirement here: the video
must not be framed to imply liquid transfer. `pour` is also last on the cut list
(`PLAN.md` section 6), so it is droppable without touching anything upstream. Flagged for
user confirmation as RISK-08.

---

### ADR-012 — Seed-derived, declarative domain randomization

**Context.** Brief p2 objective 3 and p5 name object weight, friction, shape, lighting,
background and initial placement; 15 points (`CONSTRAINTS.md:33`). The demo must span 10
randomized seeds (brief p4).

**Options.**
- (a) Randomize inline inside `env.reset()` from global RNG state. Fast, and irreproducible
  the moment anything else draws a random number.
- (b) `Randomizer` as a pure function `seed -> SceneConfig`, with ranges declared in
  `configs/randomization.yaml` and the config serialized into every episode's artifacts.

**Decision.** (b). The seed determines the config; the config determines the scene. Nothing
in the control or perception path may consume the randomization RNG.

**Consequences.** Any seed is exactly reproducible from `results/.../per_seed.csv`, which
is what makes a reproducibility claim (10 points, brief p5) checkable by a judge rather
than asserted. Randomization ranges are reviewable in one YAML file, and inclusive versus
exclusive bounds are documented because off-by-one in those ranges is an explicit
bug-hunter target (`.claude/agents/bug-hunter.md:22`). The harness must assert that the 10
evaluation configs are pairwise distinct, since a harness that quietly reuses one seed is
the bug-hunter's first suspicion (`:19`). Training seeds and evaluation seeds 0–9 are
disjoint by assertion (`PLAN.md` M09 done-when 3); overlapping them would void the
generalization claim entirely.

---

### ADR-013 — Precision and device mapping strategy, with the Day-0 caveat carried forward

**Context.** Day 0 measured a single-Add-op graph at 0.061 ms on CPU, 0.218 ms on GPU and
0.635 ms on NPU, and the record itself warns that dispatch overhead dominates and that the
ordering should be trusted only for realistic models
(`docs/hardware/bmptl-verification.md:12-20`). The rubric weighs latency, throughput,
precision choices and device utilization (brief p5).

**Options.**
- (a) Benchmark FP32 on CPU only. Under-serves a 20-point criterion.
- (b) Sweep {CPU, GPU, NPU} × {FP32, FP16, INT8} for the models actually in the loop, and
  pick the demo device from measured results.
- (c) Assume NPU is fastest and ship that claim. Directly contradicted by the Day-0 note
  and unsupported by any measurement of our graphs.

**Decision.** (b). FP32 CPU is the correctness and latency baseline. FP16 targets the Arc
B390 iGPU. INT8 via NNCF post-training quantization, calibrated on M09 data, targets the
NPU5010, with static input shapes if M03 finds the NPU requires them. The demo device is
chosen from the M14 table, not in advance.

**Consequences.** The published table will contain honest `UNSUPPORTED` rows where a
device/precision pair fails, and the interpretation paragraph will restate the
dispatch-overhead caveat instead of implying a clean NPU victory. Behaviour preservation is
measured, not asserted: the same 10 seeds run under FP32 Torch and under the shipped
OpenVINO precision, both success counts published (brief p3 "preserve system behavior",
`PLAN.md` M19 done-when 3). INT8 is on the cut list (`PLAN.md` section 6) since it is the
most likely to consume a day for a table row.

**Correction, Sept 12, 2026 — M03 has run; the conditional above is now resolved.** This
ADR's Decision says INT8 "targets the NPU5010, with static input shapes **if** M03 finds the
NPU requires them." M03 ran on Sept 12 (`b50e300`, evidence in `benchmarks/ov-smoke-notes.md`,
logged in `DECISIONS.md`) and the answer is yes, with a sharper edge than the hedge
anticipated:

1. **The NPU requires static or bounded input shapes. A fully-open batch dimension (`-1`) is
   not merely unsupported — it kills the process.** Compiling a dynamic-batch IR on NPU exits
   with `STATUS_ACCESS_VIOLATION` (`3221225477` / `0xC0000005`), not a catchable OpenVINO or
   Python exception. The diagnostic names the unbounded dimension: `Upper bounds are not
   specified for node 'Multiply_11422' (type 'Convolution'): input '0' bounds are
   '[9223372036854775807, 3, 224, 224]'` — that value is `INT64_MAX`. M13's export must
   therefore emit static or upper-bounded batch dimensions for the NPU path. This is a hard
   constraint on the export, not a tuning preference.
2. **Operational consequence for M13 and M14: isolate every NPU compile attempt in its own
   subprocess.** Because the failure is a process kill rather than an exception, a harness
   that compiles several device/precision pairs in one process will lose the results of pairs
   that already succeeded. M03 hit exactly this — its first run discarded passing NPU *static*
   results when the dynamic variant crashed — and `scripts/ov_smoke.py` was restructured to run
   each check in a subprocess that writes its result immediately. M14's benchmark harness needs
   the same shape or its `UNSUPPORTED` rows will be indistinguishable from lost rows.
3. **There is no ONNX intermediate in the conversion pipeline.** M03 converts live
   `torch.nn.Module` objects with `openvino.convert_model`; the `torch.onnx.export` path was
   avoided because it needs `onnx` and `onnxscript`, neither pinned. M13 should inherit that
   recipe rather than reintroduce an ONNX step.

Baseline deviations from the PyTorch FP32 reference, for the behaviour-preservation check
this ADR's Consequences describe: CPU `5.674362e-05`, GPU `7.408857e-05`, NPU `1.122952e-04`.

---

### ADR-014 — Prove the PyTorch-to-IR conversion path inside the first 48 hours

**Context.** Day 0 verified device enumeration and a trivial graph
(`CONSTRAINTS.md:60-65`). That does not exercise the model converter, the NPU's shape
constraints, or the iGPU driver on a realistic graph. The final demo must run on bm-ptl
(brief p3) and bm-ptl is gone after Sept 16 (`CONSTRAINTS.md:5`).

**Options.**
- (a) Trust Day 0 and convert the real model on Day 4. If the converter or the NPU rejects
  the graph, that is discovered with two days left and no fallback rehearsed.
- (b) On Day 1–2, convert a ResNet18-scale network end to end — PyTorch → IR → compile →
  infer on all three devices — and record what each device accepted, including verbatim
  failures and whether dynamic shapes were rejected.

**Decision.** (b), as `PLAN.md` M03, budgeted at 3 hours on Day 1 and treated as a hard
blocker if it slips past Day 2. While logged in, also probe MuJoCo offscreen rendering on
bm-ptl (RISK-03) — fifteen minutes that de-risks the entire Day-5 demo module.

**Consequences.** Shape constraints, precision constraints and driver problems surface
while there is still time to change the model choice. The notes file becomes the input to
ADR-013's export plan rather than a Day-4 surprise. Small risk of re-work if the final
model differs materially from the probe network; that is a far cheaper failure than
discovering on Day 4 that the NPU will not take our graph.

---

### ADR-015 — Honesty constraints on the submission narrative

**Context.** Six agents contribute to one repo, five of the six deliverables are prose or
video (brief p4, `SUBMISSION.md`), and the fallback branch produces a system whose most
flattering description would be false. Every agent definition in `.claude/agents/` opens
with honesty rules; `.claude/agents/docs-writer.md:14-16` forbids invented numbers.

**Options.**
- (a) Let each document describe the system in its own words.
- (b) Fix binding naming and sourcing rules in this ADR and have compliance-reviewer check
  documents against it.

**Decision.** (b). The rules:
1. No number appears in README, slides, video narration or `benchmarks/bmptl-results.md`
   unless it traces to a named Tester report or artifact file.
2. If the scripted branch ships, it is called a scripted closed-loop controller. The words
   "learned policy", "VLA policy" and "imitation learning" are not used for that control
   path (`PLAN.md` F1 done-when 2).
3. The README states what the policy observes (cameras plus joint state) versus what data
   generation used (privileged state), per ADR-005.
4. The 10-seed result is reported as measured, including partial and failed seeds, with the
   per-subtask breakdown. No cherry-picking 10 successes out of 30 attempts.
5. `pour` is described as a tilt pose with no fluid simulation (ADR-011).
6. Device rows in the benchmark reflect the compiled device OpenVINO reports (ADR-007);
   unsupported combinations appear as `UNSUPPORTED`.

**Consequences.** Some scores will be lower than a more flattering write-up would invite.
In exchange, nothing in the submission breaks under a judge's follow-up question, and
"Technical Quality & Reproducibility" (brief p5) is earned rather than claimed. These rules
are checkable: compliance-reviewer can grep for the forbidden phrasings and trace each
number to its artifact.

---

### ADR-016 — Adopt the SO-101 asset's shipped DoF unmodified

**Context.** RISK-02 (`PLAN.md` section 7) left actuated DoF per arm undecided because no
asset had been selected; DoF drives IK, the ACT action-dimension config and the OpenVINO
input shapes (ADR-013). M01 selects the asset source.

**Options.**
- (a) Modify the kinematics to suit our controller — lock a wrist joint to simplify IK, or
  add a DoF for reachability. Every modification is a divergence from an upstream asset
  that a judge can diff, and it invalidates any provenance claim.
- (b) Adopt whatever the chosen source ships, unmodified, and adapt our IK, ACT config and
  export shapes to it.

**Decision.** (b). Whatever DoF the adopted SO-101 asset ships is authoritative. Builder
reports the actual value in M01/M02 and it is written into section 1's contract table then.
Scene composition — arm placement, table, objects, cameras — is ours; the arm model is not.

**Consequences.** The DoF question stops being a design decision and becomes a measurement,
which is the correct shape for it. Reproducibility improves: the asset is citable at a
version. The cost is that an awkward upstream kinematic choice must be absorbed downstream
rather than edited away. If the asset proves genuinely unusable, that is a Tier-C pivot
under M01's own escape clause, not a licence to start editing joints.

---

### ADR-017 — `pour` confirmed as a tilt-and-position motion, with mandatory disclosure

**Context.** ADR-011 scoped `pour` as a tilt pose with no fluid simulation and flagged it
as RISK-08 pending confirmation. Confirmed by the user on Sept 11, 2026.

**Options.** As enumerated in ADR-011. No new options; this ADR records the ratification
and the disclosure obligation that rides with it.

**Decision.** `pour` is a tilt-and-position motion. Arm B holds the mug, arm A brings the
bottle over it and achieves a tilt-and-hold pose within a stated tolerance. Success is pose
achievement. No fluid, particle or volume is simulated. The absence of fluid **must** be
stated in `README.md` and spoken in the video narration — not relegated to a caption, a
footnote or repo-only prose, since the video reaches judges who may never read the repo.

**Consequences.** The brief's verbatim example command (p1) executes end to end and `pour`
supplies a second complementary dual-arm action. The disclosure is now a checkable artifact
requirement: compliance-reviewer verifies the statement exists in both `README.md` and
`docs/video-script.md`, and its absence is a defect, not a polish item. This closes RISK-08
and makes ADR-015 rule 5 binding rather than assumed.

---

### ADR-018 — The 10-seed evaluation reports the true success rate; cherry-picking prohibited

**Context.** RISK-06 recorded an ambiguity in the brief: p4 asks for a video "demonstrating
successful task execution across 10 randomized environment seeds", while p5 asks to
"repeat evaluation across 10 randomized seeds and summarize the success rate". The first
reading permits selecting 10 successes; the second requires reporting what 10 seeds
actually did. Confirmed by the user on Sept 11, 2026.

**Options.**
- (a) Run until 10 seeds succeed and show those. Maximises apparent success and collapses
  under a single follow-up question about how many attempts it took.
- (b) Fix 10 seeds in advance, run them, report the outcome as measured.

**Decision.** (b). The 10 evaluation seeds are declared in the eval config before the run
and are not changed afterward to improve the result. The reported success rate is
successes over those 10. **Failure modes must be shown and narrated in the video**, not
merely tabulated in the repo — if 7 of 10 succeed, the video says 7 of 10 and shows what
the other 3 did. The per-subtask breakdown (ADR-006) accompanies the headline number.

**Consequences.** The headline number will be lower than a curated reel would show, and
that is the accepted trade. In exchange the result survives scrutiny, "Technical Quality &
Reproducibility" (brief p5, 10 pts) is earned, and the failure analysis itself becomes
evidence for "Robustness & Generalization" (15 pts) — a demonstrated understanding of where
the system breaks reads better to a technical judge than an unexplained clean sweep. This
closes RISK-06 and upgrades ADR-015 rule 4 from written-reporting-only to a video
obligation. Re-running a seed after a **code change** is normal iteration; swapping the
seed set after seeing results is not, and the seed list is committed to make the difference
auditable.

---

### ADR-019 — Speechmatics credentials via gitignored `.env` and environment variable

**Context.** RISK-04 recorded that no Speechmatics credential was available. ADR-002 keeps
voice at the edge of the system behind `CommandSource`, but the key still has to reach the
process. The repository goes public on Sept 15 (`SUBMISSION.md`), which makes any committed
secret unrecoverable by deletion — git history preserves it.

**Options.**
- (a) Hard-code the key, or commit a config file containing it. Disqualifying for a public
  repo.
- (b) Store the key in a gitignored `.env`, read it from an environment variable at
  runtime, and commit only a `.env.example` naming the variable.

**Decision.** (b). The key lives in `.env`, which is gitignored. Code reads it from the
environment as `SPEECHMATICS_API_KEY` and never from a literal. A committed `.env.example`
documents the variable name with an empty value so the setup path stays reproducible. A
missing key must fail loudly at startup with an actionable message and must not silently
disable voice — a silent disable would make the bonus appear broken rather than unconfigured.

**Consequences.** Verified at the time of writing: `.gitignore:23` already ignores `.env`;
`.env` exists locally and is untracked; and no `.env`, `*.pem` or `.kaggle` path appears
anywhere in git history. The remaining exposure is a future `git add -f` or a key pasted
into a committed file, so the pre-publication check before flipping the repo public on
Sept 15 includes a history scan for credential-shaped strings. This closes RISK-04 as a
handling question; whether a key is actually provisioned remains a scheduling matter under
the Day-4 drop rule.

---

### ADR-020 — Simulation runs on bm-ptl, not the laptop

**Context.** `mujoco.MjModel.from_xml_path` fails on the laptop with
`OSError: [WinError 4551] An Application Control policy has blocked this file`. Windows
Smart App Control is enforced (registry `VerifiedAndReputablePolicyState = 1`) and blocks
`mujoco.dll` as an unsigned binary with insufficient Microsoft cloud reputation, confirmed
via `Microsoft-Windows-CodeIntegrity/Operational` Events 3077/3118 and
`Get-AuthenticodeSignature`. Reproduced identically on `mujoco==3.13.0` and `3.2.7`, so it
is an OS policy issue, not an asset or package defect (`scenes/so101/VERIFICATION.md`).
This surfaced during M02's asset prerequisite and was filed as RISK-11.

**Options.**
- (a) Disable Smart App Control locally. It is a **one-way** change — Windows cannot
  re-enable it without a clean OS reinstall — and it is a standing security regression on
  the developer's daily machine for the sake of one week's project. Rejected.
- (b) Run all MuJoCo work on bm-ptl. Development then matches the deployment target the
  brief actually requires (p3: the simulator and inference pipeline must execute on Core
  Ultra Series 2/3), and gets the Arc B390 iGPU and 32 GB RAM. The cost is an SSH-mediated
  iteration loop for every simulation change.
- (c) Add WSL2 as a local Linux sim environment. Another environment to pin, reason about
  and reproduce, plus EGL/offscreen-rendering quirks under WSL. Deferred, not rejected —
  it stays available if the SSH tax proves worse than expected.

**Decision.** (b). The laptop is for code, git and the Speechmatics client. bm-ptl runs all
MuJoCo work.

**Consequences.** The outstanding MuJoCo compile-check of the SO-101 asset moves to bm-ptl,
as does M02's done-when rendered PNG and every subsequent simulation module's iteration
loop. This has an unplanned upside: it collapses the "does the demo actually run on Intel
hardware" question (brief p3) from a Day-5 integration risk into the everyday development
path, and it retires the laptop-versus-bm-ptl environment drift that the two separate
requirements files were tracking.

It also has a real cost that must be managed rather than assumed away. **RISK-03 — whether
MuJoCo offscreen rendering works on bm-ptl at all — is now load-bearing with no local
fallback**, because option (a) is rejected and (c) is not built. The rendering probe folded
into M03 (ADR-014) is therefore promoted from a fifteen-minute convenience check to a
blocking prerequisite: if offscreen rendering fails on bm-ptl, WSL2 under option (c) must be
built immediately, and that decision cannot wait for Day 5. Iteration latency rises for all
sim modules, and bm-ptl's expiry on Sept 17 00:15 (`CONSTRAINTS.md:5`) now bounds simulation
development, not merely benchmarking — so the working tree must stay pushed to git rather
than living only on the instance.

---

### ADR-021 — Dual-arm scene composed by scripted renaming, not MJCF `<include>`

**Context.** `PLAN.md` M02 names two candidate ways to duplicate the unmodified SO-101 arm
for the bimanual scene: an MJCF `<include>`, or hand-copying the body tree. MuJoCo requires
every body, joint, site and actuator name to be unique across the whole compiled model, and
`<include>` was suspected, not yet confirmed, to have no prefix mechanism to keep two copies
of the same arm from colliding on every name.

**Options.**
- (a) `<include>` `scenes/so101/so101_new_calib.xml` twice. Empirically tested on bm-ptl via
  `scripts/probe_include_namespace.py` (mujoco 3.2.7): the compiler rejects it before naming
  is even reached — `ValueError: XML Error: File 'scenes/so101/so101_new_calib.xml' already
  included / Element 'include', line 4`. MuJoCo treats repeated inclusion of the identical
  file as an error outright, and even a byte-identical second copy under a different filename
  would still collide on every body/joint/site/actuator name, since `<include>` has no prefix
  attribute. Ruled out.
- (b) Hand-copy the arm's body tree twice directly into the new scene file, retyping renamed
  copies of the ~120 lines of nested `<body>`/`<joint>`/`<site>` XML (7 bodies, 6 joints, two
  sites, ~40 mesh geoms per arm). Simple to read, but a future upstream recalibration (or a
  typo in one of the two copies) has no tooling to catch a missed or mismatched rename —
  exactly the transcription failure mode ADR-016 already worried about for a single arm.
- (c) Write `scripts/gen_dual_scene.py`: parse the unmodified upstream
  `so101_new_calib.xml` with `xml.etree.ElementTree` (stdlib only, no `mujoco` import, so it
  runs on the laptop despite ADR-020), deep-copy its single top-level body twice, prefix every
  body/joint/site name (`armA_`/`armB_`), reposition each copy's base, and generate a matching
  renamed actuator pair. Splice the two generated bodies into a hand-authored template holding
  the table, drawer, props and cameras. Mesh and material definitions are declared **once** in
  the shared `<asset>` block, since geometry is not per-instance data — only body transforms,
  joints and actuators need to be.

**Decision.** (c). `scripts/gen_dual_scene.py` generates
`src/bimanual/sim/assets/so101_dual_table.xml`.

This is deliberately recorded as **two** decisions, not one, because they rest on different
kinds of evidence and a future reader who revisits only the first will draw the wrong
conclusion about the second.

**Decision 1 — eliminate `<include>` (empirical).** `scripts/probe_include_namespace.py`,
run on bm-ptl under mujoco 3.2.7, produced a hard compiler rejection rather than the
predicted naming collision: `ValueError: XML Error: File '...so101_new_calib.xml' already
included`. MuJoCo refuses repeated inclusion of the same file outright, before name
resolution is ever reached, so no amount of file-splitting or copying-under-a-new-name
rescues `<include>` — and even if it did, `<include>` has no prefix attribute, so the two
copies would still collide on every body, joint, site and actuator name. This eliminates
(a) on measured behaviour, not on inspection.

**Decision 2 — prefer generation over hand-copying (judgement).** Decision 1 leaves (b) and
(c), both of which require renaming; it says nothing about which to pick. That choice rests
on maintainability, not on a compiler result: (b) has no mechanism to guarantee the two
copies stay in sync with each other or with upstream, while (c) makes resynchronization a
single command (`python scripts/gen_dual_scene.py`) that is provably faithful to the
upstream source because it is generated *from* it, not retyped *from reading* it. This half
is a considered preference and could reasonably be revisited; Decision 1 could not, short of
a change in MuJoCo itself.

**Consequences.** The dual-arm scene file is derived, not hand-authored, in its arm sections;
a header comment marks the generated regions and points back at the generator so a future
editor does not hand-edit a body that the next regeneration would silently discard. This gives
ADR-016's "adopt the asset's shipped kinematics unmodified" a mechanical guarantee rather than
a discipline one: the generator copies the upstream body tree by value, so an accidental
hand-edit of kinematics does not survive regeneration. The cost is one more script to maintain
and a full ~30 KB regenerated block in the diff whenever upstream or the layout constants
change, rather than a small hand-edited delta — accepted, since the alternative is exactly the
double-maintenance failure mode ADR-016 flagged. The table, drawer, prop and camera sections of
the generated file are hand-authored and marked safe to edit directly; only the arm bodies and
actuators are generated. Layout follows from a stated reach assumption (also recorded as
comments in the generator and the generated file): SO-ARM100 reach is approximately 0.30 m, so
arm bases are placed 0.50 m apart (0.25 m off the table centreline on each long edge) to leave
roughly a 0.10 m wide band at the table's centre reachable by both arms, where all five props
are placed.

---

### ADR-022 — Opt-in camera rendering in `TableSettingEnv`

**Context.** The M02 env rendered all five cameras unconditionally on every
`reset()` and `step()`, at roughly 750 ms per step (tester measured `step()` at
0.666–0.813 s). Measured properly in `docs/hardware/m02-render-cost.md`: physics
alone is **0.20 ms/step**, one camera is **152.25 ms/step**, all five are
**809.86 ms/step**. Rendering is therefore not a component of step cost — it *is*
step cost, by a factor of four thousand.

The cost is real GPU time, not a misconfiguration: `scripts/probe_gl_renderer.py`
confirms the offscreen context is `Intel(R) Arc(TM) B390 GPU` on OpenGL 4.6, not
a software rasterizer. And cameras scale linearly — 161.93 ms each across five
versus 152.05 ms for one, only 6.5% above linear — so there is no fixed setup
being amortised and no camera count at which rendering everything becomes cheap.

What that costs downstream, against a 50 ms/step budget in this document's own
component table and a bm-ptl reservation that ends Sept 17 (`CONSTRAINTS.md:5`):
- **M09 (demonstration collection)** — 100% waste. ADR-005 generates
  demonstrations from privileged simulator state; the policy is not in the loop
  and no image is ever read. Every rendered frame was discarded.
- **M08 (10-seed evaluation harness)** — ADR-006 builds this once and reuses it
  for GATE-1, the nightly checkpoint from Day 3, and M19. The cost is paid on
  every step of every seed of every run.
- **M19 (behaviour preservation)** — re-runs those seeds across backends and
  precisions, multiplying the harness cost again, on the last day before the
  instance expires.

**Options.**
- (a) Keep unconditional rendering. Rejected: it dominates step cost for every
  caller that never looks at an image, which on this plan is most of them.
- (b) Cache rendered frames and re-use them while the action is unchanged.
  Rejected: policy rollouts issue a non-zero, different action every step, so the
  cache would never hit on the workload that matters. It would only speed up the
  zero-action stability probe, which needs no pixels at all.
- (c) Opt in to cameras, via a constructor default plus a per-call override.

**Decision.** (c). `TableSettingEnv(cameras=None)` is the default and renders
nothing; `reset(seed, cameras=...)` and `step(action, cameras=...)` override it
for a single call without mutating the instance default. Callers name the cameras
they actually consume. `render(camera)` remains available as an explicit escape
hatch regardless of the setting, so `view_scene.py` and the probes are unaffected.
Camera names are validated against the model's discovered cameras — not a
hardcoded list — so adding a camera to the MJCF still requires no `env.py` change.

**Consequences.** Render cost becomes proportional to what a caller needs instead
of fixed overhead. Per-module intent:
- **M09 (demonstration collection)** collects **with cameras enabled** —
  `['front', 'armA_wrist', 'armB_wrist']`, the vision path ADR-005 specifies and
  the exact set M11 consumes. The scripted controller reads `get_state()` for
  *action selection*; the camera frames are recorded into the *dataset* for
  downstream ACT training. Cost: **~456 ms/step** (0.20 ms physics +
  3 x 152.05 ms/camera) per the M11 render-budget note in `PLAN.md`.
- **M11 (ACT training)** uses `['front', 'armA_wrist', 'armB_wrist']`, the views
  the policy actually consumes. Kaggle reads pre-rendered frames and never
  invokes MuJoCo, so the render cost lands on M09, not here.
- **M08 (evaluation harness)** depends on which executor is being scored, and the
  two cases are not interchangeable:
  - `--executor scripted` — state-only is correct: 0.20 ms/step, physics only.
    The scripted controller reads `get_state()` and never touches an image.
  - `--executor learned` — cameras are **required**. The ACT policy cannot
    produce an action from an obs dict with no images, so the harness must render
    the same camera set M11 trained on (`front`, `armA_wrist`, `armB_wrist`) at
    **~456 ms/step**. This is not an optimization the harness may skip.
  - Video capture on the final winning seed — all five cameras at
    **~809.86 ms/step**, run **once** on that one seed, not once per seed of
    every run.

The cost of this decision is that a caller who forgets to request a camera gets an
obs dict without images rather than a slow one — a `KeyError` at the point of use
instead of silent overhead. That is the right failure direction, and the
docstrings on `reset()`/`step()` state the schema. The ~133 ms per-frame cost
itself remains unexplained and is deliberately not pursued inside the hackathon
window; `docs/hardware/m02-render-cost.md` records the suspects for later.

**Correction, Sept 11, 2026 (this ADR's Consequences only; the Decision is
unchanged).** As first written, this section said M09 runs with no cameras and
that M08 defaults to state-only for GATE-1 scoring. Both were category errors,
and they shared one root cause: reading ADR-005 as a statement about *dataset
contents* when it is a statement about *how the scripted controller selects
actions*. ADR-005 permits privileged `get_state()` in the data-generation loop;
it says nothing about what must be logged. M11 trains a camera-conditioned ACT
policy, so M09's dataset must contain camera frames — a state-only M09 would
produce a dataset ACT cannot train on. The M08 error follows from the same
confusion in the opposite direction: state-only scoring is sound for
`--executor scripted`, but a learned executor fed an image-free obs dict cannot
emit an action at all, so state-only GATE-1 scoring of the learned branch is not
a cheaper measurement, it is an impossible one. The cost of the correction is
real and is carried in `PLAN.md`: M09's collection moves from 0.20 ms/step to
~456 ms/step, and the learned half of GATE-1 moves from seconds to roughly 76
minutes for 10 seeds x 1000 steps. The related note at
`docs/hardware/m02-render-cost.md:65-66` ("M09 requires no cameras per ADR-005")
repeats the superseded reading and is not authoritative; this paragraph governs.

---

### ADR-023 — Cut the learned-policy branch on schedule evidence; ship the scripted controller as the policy

**Context.** Sept 12 is Day 2 of a schedule with five days left, and the plan is one
module behind. M03 — "a hard blocker for the whole OpenVINO story (20 rubric points)",
forced into the first 48 hours by ADR-014 — did not ship on Day 1 and is verifiably absent
(`scripts/` has no `ov_smoke.py`; `benchmarks/` has no `ov-smoke-notes.md`). Day 2 as
previously planned therefore carried M03 (3 h) plus M04–M08 (23 h) = **26 builder-hours in
one calendar day**, before the per-module tester → compliance-reviewer → tutor →
docs-writer passes that `PLAN.md` section 2 mandates. Day 1's own shape is the evidence for
what a day actually holds: M01 (2 h budgeted) + M02 (6 h budgeted) plus a correction pass
(ADR-016..ADR-022 and the opt-in rendering refactor) consumed the whole day.

Two further facts bind. ADR-020 means every simulation module runs on bm-ptl, and bm-ptl
expires Sept 17 00:15 (`CONSTRAINTS.md:5`), one day past the Sept 16 submission
(`CONSTRAINTS.md:4`) — so Day 6 is a working day, not a retry window. And ADR-022's
correction put M09's demonstration collection at ~456 ms/step, i.e. ~8.9 h of bm-ptl wall
clock for 70,000 attempted steps, which can only be spent overnight and only after M06,
M07 and M08 have all closed.

The arithmetic that follows is not close. M09b needs M06 + M07 + M08; at ~8
builder-hours/day those close Day 4 evening at the earliest; an overnight collection then
lands Day 5 morning, M11 trains Day 5, and GATE-1 could not be held before Day 5 night.
`PLAN.md`'s GATE-1 block already forbids sliding the gate even into Day 4.

**Options.**
- (a) **Keep ACT, slide GATE-1 to Day 5 night.** Leaves the OpenVINO benchmark, the
  bm-ptl pipeline run, the 10-seed recording, the documentation and the submission all
  stacked on Day 6, on hardware that expires that night. Directly contradicts
  `CONSTRAINTS.md:50-52`, which rewards a complete pipeline over half-working ML.
- (b) **Keep ACT, shrink the dataset to the pre-committed 4 h / ~31,500-attempted-step
  run.** This was `PLAN.md`'s own recommended degradation, and it is still the right
  degradation *within* the learned branch — but it saves ~5 h of unattended wall clock,
  not the ~18 builder-hours the schedule is actually short. It treats a capacity problem
  as a wall-clock problem.
- (c) **Cut ACT (M11) and its demonstration dataset (M09b) from the critical path, ship
  the scripted controller as the policy, and keep a much cheaper frame-collection module
  (M09a) to feed the PoseNet that ADR-009 put in the loop.**
- (d) **Cut PoseNet (M10) instead and keep ACT.** Puts the 20-point OpenVINO criterion
  back onto the riskiest module — exactly the dependency ADR-009 exists to break — and
  leaves `--perception state` in the demo loop against ADR-005.
- (e) **Keep a minimal learned branch by decoupling a tiny unrandomized M09b spike from
  M07/M08, launched immediately after M06 closes on Day 3.** The appeal is that it
  attacks the real blocker: the ~18-builder-hour shortfall comes mostly from M09b's
  *dependencies*, not from M09b itself, so dropping randomization (M07) and harness
  scoring (M08) as prerequisites would in principle let a handful of seeds be collected
  and trained the same night, preserving something to put under "policy trained with
  LeRobot or compatible tooling". **Rejected on wall clock, not on merit.** M06 does not
  close until 17:30 at the earliest (`PLAN.md:209` — M06 owns 09:00–17:30 and 17:30 is
  the POUR GATE, i.e. the *best* case, not the expected one), and GATE-1 opens at 19:00
  (`PLAN.md:212`). That leaves under 90 minutes to collect frames, upload them, train,
  and evaluate — and the evaluation is the expensive half, because ADR-022's correction
  puts a learned-executor rollout at ~456 ms/step, so even a 2-seed check is not a
  minutes-scale job. This is the same wall-clock foreclosure that killed (b) and (c)'s
  alternatives: the deficit is builder-hours and bm-ptl hours *before* the gate, and no
  amount of decoupling manufactures hours that the clock does not contain. Option (e)
  also purchases its speed by deleting the two things that make a number trustworthy —
  randomization and the 10-seed harness — so what it would deliver to GATE-1 is an
  unrandomized, thinly-evaluated checkpoint that ADR-015 rule 2 would forbid describing
  as a working learned policy anyway. Recorded here because the plan holds itself to
  naming every option before rejecting it (ADR-004, ADR-005, ADR-009), and this one was
  reachable enough to deserve a written rejection rather than silence.

**Decision.** (c). M11 and M09b move wholesale into F3 (optional revival after M18, under
F3's existing hard stop, and realistically unreachable). The scripted controller from M06
is the policy: F1 and F2 are activated now rather than being contingent on a GATE-1
verdict. GATE-1 is retained on Day 3 as `CONSTRAINTS.md:50-52` requires, but is decided on
**schedule evidence** and its outcome is pre-committed to the scripted branch.

Three sub-decisions ride with it and are recorded here because they are structural, not
scheduling detail:

1. **A planning capacity is fixed at 8 builder-hours/day**, module budgets stay denominated
   in builder-hours, and the day rather than the module absorbs the ~30% review overhead.
   This is an assumption derived from Day 1's shape, not a measurement, and it is the
   number to correct if the user knows their throughput differs.
2. **Clock gates replace judgement calls.** Every day boundary in `PLAN.md` section 1A.5
   carries a pre-committed time, and `PLAN.md` section 1A.6 is an ordered cut ladder whose
   rungs fire on a missed gate without re-litigation. The specific hole this closes: M09b's
   "if M08 does not close in time" had no definition of "in time", so the most
   consequential decision in the plan was left to a tired developer at night. It is now
   22:00 / 23:00 / abort, with a 07:00 hard stop, retained in force for F3.
3. **M09a's camera set is re-derived from its surviving consumer.** ADR-022 fixed M09's
   cameras at `['front', 'armA_wrist', 'armB_wrist']` *because that was M11's training
   set*. With M11 cut, the set follows M10's PoseNet input instead: `front` +
   `drawer_view`, because the drawer is occluded from every other camera (ADR-021) and
   `SceneBelief` carries drawer opening. Using the measured marginal camera cost of
   152.05 ms (`docs/hardware/m02-render-cost.md:43-49`) that is **~304.3 ms/step**, and
   ~5,000 frames is ~25 min of bm-ptl wall clock rather than ~9 h. **This does not
   supersede ADR-022** — ADR-022's Decision (opt-in rendering) and its ~456 ms/step figure
   are unchanged and return with ACT if F3 revives it.

**Consequences.**

*What this costs, stated first and without softening.* **No policy is trained.** Brief p2
objective 4 asks for a policy trained or fine-tuned with LeRobot or compatible tooling;
with M11 cut, nothing satisfies it. PoseNet is a trained model but it is a perception
network, not a policy, and it is not LeRobot. ADR-015 rule 2 already forbids calling the
shipped control path learned; this ADR adds the positive obligation — `PLAN.md` M18
done-when 5 — that the README *state* the gap rather than merely avoid misdescribing it,
and that it record that the branch was chosen on schedule grounds. "We chose scripted
because we ran out of days" and "we chose scripted because learning underperformed" are
different claims and only the first is true.

*Second cost.* M16 (full pipeline on bm-ptl) loses its day of slack: it folds into M14's
Day-6 bm-ptl session, on the last day the instance exists. The Day-5 20:00 gate — at least
one IR compiling on at least one device before Day 6 begins — is the only insurance left
against that, and it is thin.

*Third cost.* ADR-006 committed the evaluation harness to Day 2 specifically so that it
would exist before there was anything good to measure. M08 now spans a Day-3 evening
skeleton and Day 4. That is a real weakening of ADR-006 and is recorded as such rather
than presented as equivalent. The mitigating fact is narrow but genuine: with M11 cut there
is no second executor for the harness to be quietly shaped around, so the specific failure
ADR-006 insured against — a harness written after the numbers exist, to fit them — is
smaller.

*What survives, and why the cut is survivable.* ADR-009's whole purpose was to stop the
20-point OpenVINO criterion riding on ACT, and that argument now carries the submission:
PoseNet is converted, quantized, benchmarked across CPU/GPU/NPU, and executes on Intel
silicon on every control step of the demo. ADR-004 made M06 dual-purpose; with M11 gone it
is simply the policy. Brief p2 objectives 1 and 3 are answered by M06 + M07, objectives 2
and 5 by M05's grounder plus M10's OpenVINO-served perception. ADR-002's CommandSource
seam is untouched — M04 still ships the abstraction and a stub voice source even though
the Speechmatics implementation is unscheduled — and ADR-010's scripted `handoff`,
ADR-012's seed-derived randomization and ADR-018's true-success-rate reporting are all
unaffected.

*Honest bottom line.* Even after this cut the plan needs ~9–10 builder-hours/day for five
consecutive days against a capacity of ~8: roughly one day of negative float, recorded as
RISK-12. This ADR does not make the plan comfortable. It makes the plan's failure mode
"some scope was cut and said so" instead of "the pipeline was incomplete on Sept 16".

---

### ADR-024 — IK strategy for a 5-DoF arm against a 6-DoF task space: relax orientation, solve position exactly

**Context.** M06a (`PLAN.md` M06, `src/bimanual/control/ik.py`) must turn a
`SkillCall` into joint targets for the SO-101 arm. The compiled model exposes
six `<position>` actuators per arm (`so101_dual_table.xml`'s `<actuator>`
block), but the sixth, `armX_gripper`, drives only the jaw hinge on
`armX_moving_jaw_so101_v1` — it contributes nothing to end-effector pose.
That leaves five *positioning* joints (`shoulder_pan`, `shoulder_lift`,
`elbow_flex`, `wrist_flex`, `wrist_roll`) against a 6-DoF task space (3
position + 3 orientation), documented in advance by `docs/learn/06-ik-and-
skills.md`: "a 5-DoF arm generally cannot hit an arbitrary position *and*
orientation." Something must be relaxed.

A second, easily-confused naming question sits next to this one and is
resolved here too: `<body name="armX_gripper">` is the body driven by the
`armX_wrist_roll` joint (it carries the fixed half of the gripper mechanism
and the `armX_gripperframe` site); `armX_moving_jaw_so101_v1` is a CHILD of
that body, driven by the separate `armX_gripper` joint/actuator (the jaw
open/close hinge), so its position additionally shifts as the jaw opens or
closes. An IK target aimed at the moving jaw instead of the gripperframe
site would drift every time a skill opens or closes the jaw.

**Options.**
- (a) Relax orientation about `wrist_roll`: solve position exactly (3
  equations) using all five joints in a damped-least-squares Jacobian solve,
  and accept whatever orientation the redundant (5 unknowns, 3 constraints)
  solution falls into. No orientation target is ever specified.
- (b) Constrain to a 5-DoF target: xyz plus two orientation angles (e.g. the
  gripper's approach direction), dropping only roll about the approach axis.
  This uses all 5 DoF against a fully-determined 5-equation system, closer
  to a real solution but requires building and maintaining an orientation
  error term (e.g. an axis-angle or rotation-log residual restricted to two
  axes) on top of the position Jacobian, and a decision about which two
  orientation axes matter for a parallel-jaw-style grasp.

**Decision.** (a). `ik.solve_position_ik` (`src/bimanual/control/ik.py`)
targets only the 3D position of the `armX_gripperframe` site, via damped
least squares (Levenberg-Marquardt) on `mujoco.mj_jacSite`'s 3xN position
Jacobian restricted to the 5 positioning joints' columns. No orientation
term exists in the solver. This is the simpler of the two options and was
chosen on that basis, not because it was measured to be more accurate —
see Consequences for what that trade cost in practice.

**Consequences, including what this cost in practice (measured, not
projected).**

*The `handoff` consequence, stated in advance and then observed.* With no
orientation control, the two arms' grippers meet in the shared ~0.10 m
overlap band (`skills_scripted.py`'s `HANDOFF_TRANSFER_XY`) at whatever
approach angle each arm's redundant solve happens to fall into — there is no
guarantee the two jaws are compatibly oriented at the meeting point.
`run_handoff` compensates only positionally (`HANDOFF_SIDE_OFFSET_M` keeps
the two gripper targets from occupying the identical point), not by aligning
either arm's approach orientation. This is exactly the risk this ADR's
Decision accepted, and it is not a hypothetical: M06a's tests measured it
directly (see below) — `handoff` never got far enough to test the meeting
itself, because the prerequisite `pick` by the origin arm did not reliably
grasp the object in the first place.

*The grasp-reliability consequence, discovered empirically while building
M06a and reported here rather than papered over.* Tracing a failed
`pick(mug)` attempt (`bimanual.control.skills_scripted`) found that the
`armX_gripperframe` site is measurably NOT co-located with the point where
the fixed and moving jaw surfaces actually meet: probing both at one
converged grasp configuration put the true "pinch center" roughly
(-0.024, -0.023, +0.050) m away from the site (`ik.py`'s target reference)
in world coordinates at that configuration. Because this module's IK never
controls orientation (this ADR's Decision), that offset is not a constant
correctable by a fixed vector — re-deriving it at a different target and
re-testing moved the *object* even further from the true pinch zone, not
closer, since a different position target produces a different arm
configuration and therefore a different world-frame offset. Separately, the
moving jaw's own collision mesh has a measured bounding-sphere radius
(`model.geom_rbound`) of ~6.35 cm (the fixed-side reference geoms up to
~8.4 cm) — large relative to the props (mug body radius 3.5 cm, plate
radius 9 cm but only 1.2 cm thick) — so the arm's approach frequently
contacts and displaces the light, freely-jointed prop before any controlled
pinch can form. Both effects were verified with `mujoco`'s own contact and
geometry introspection (`data.contact`, `model.geom_rbound`,
`model.geom_xpos` at controlled joint angles), not inferred from failure
alone.

**What this means for M06a's shipped result, stated plainly.** `pick`,
`place` and `handoff` are implemented as closed control loops with correct
step-budget, joint-limit and diagnostic behaviour (see DECISIONS.md's M06a
entry for the measured numbers), but did not reliably clear the required
lift margin for `plate` or `mug` in testing. This is recorded as the honest
result of this decision, not hidden behind a passing test: fixing it
without touching `src/bimanual/sim/` (out of this module's scope) would
need option (b) above — enough orientation control to consistently present
the jaw's pinch plane to the object — or a contact-feedback correction loop
this module does not build. Both are flagged here as the concrete follow-up
this ADR's Decision implies, rather than left as an unexplained gap.

---

### ADR-025 — Drawer housing repositioning and IK pinch-point retargeting

**Context.** M06a's four skills (`open_drawer`, `pick`, `place`, `handoff`) all
failed (DECISIONS.md's M06a entry). Two root causes were verified empirically,
not assumed: (1) `open_drawer` was blocked by scene geometry — MuJoCo's own
contact list showed the drawer housing (at `y=-0.05`) fully enclosed beneath
the solid `table_top` slab, with no top-down or side approach path; (2)
`pick`/`place`/`handoff` did not reliably grasp because `ik.py`'s IK target,
the upstream `armX_gripperframe` site, is measurably ~8 cm from the point
where the jaws actually pinch (measured at rest on arm A: the site at
(y=0.1414, z=0.5765) vs. the fixed jaw body `armA_gripper` at (y=0.0432,
z=0.5844) and the moving jaw body `armA_moving_jaw_so101_v1` at (y=0.0666,
z=0.6055)).

**Options.**
- (a) Leave the drawer position unchanged and try to fix grasping only. Does
  not address `open_drawer` at all — the contact evidence shows the housing
  has no reachable face regardless of grasp accuracy.
- (b) Move the drawer housing outward (toward the table edge) so its closed
  face is no longer entombed under the tabletop slab, and separately retarget
  IK from `armX_gripperframe` to the actual pinch point, computed from live
  body positions rather than a static site. **Considered and rejected within
  this option:** correcting the old site target by a fixed offset vector
  instead of recomputing a live pinch point — rejected because the site-to-
  pinch offset is a LOCAL-frame vector that rotates with whatever orientation
  the redundant 5-joint solve falls into (ADR-024), so its world-frame
  direction is not constant across targets even though its magnitude is; a
  fixed correction would be wrong everywhere except the one configuration it
  was measured at.
- (c) Redesign the gripper kinematics or add orientation control to guarantee
  a consistent approach angle. Out of scope: touches `src/bimanual/sim/`
  kinematics (ADR-016 forbids editing the adopted asset) and reopens ADR-024's
  Decision, which this module's task scope explicitly did not authorize
  revisiting.
- (d) Assume the drawer reposition alone fixes reachability without measuring
  it. Rejected on the grounds that M02 never checked reachability in the
  first place (see Consequences) — repeating that mistake here would be the
  same error twice.

**Decision.** (b), plus a mandatory measurement step before trusting it:
1. **Drawer housing moved 12 cm outward**, `y=-0.05` -> `y=-0.17`, via a
   generator edit in `scripts/gen_dual_scene.py`'s hand-authored
   table/drawer/props/cameras template (ADR-016/ADR-021 preserved — the
   upstream-derived arm/mesh/material substitution was not touched, and
   `git diff --stat -- scenes/so101/` remained empty). The closed drawer
   face now sits flush with the table edge (`y=-0.25`) instead of 12 cm
   inboard of it; the housing overhangs the table edge by 2 cm (accepted —
   real drawer fronts commonly overhang) and its z-span still stops at the
   tabletop underside, so arm bases mounted on top at `z=0.35` do not
   collide with it.
2. **IK retargeted to the computed pinch point.** `ik.solve_position_ik`
   (`src/bimanual/control/ik.py`) now targets the midpoint between the fixed
   jaw body (`armX_gripper`) and the moving jaw body
   (`armX_moving_jaw_so101_v1`), recomputed every solve from the scratch
   `MjData`'s current qpos (via `mujoco.mj_jacBody` on both bodies, averaged),
   so the target tracks the jaw as it opens and closes. `armX_gripperframe`
   is left in the XML and in `ik.gripperframe_site_name()`, unused by the
   solver, for reference. A new `PINCH_POINT_OFFSET_M` constant documents the
   measured ~8 cm offset that motivated the change.
3. **A reachability probe (`scripts/probe_reachability.py`) was run BEFORE
   any skill**, exactly because the risk above was explicitly flagged as
   something to test, not assume: with the closed drawer face at `y=-0.25`,
   it sits close to arm A's own base at `(0, -0.25, 0.35)`, plausibly inside
   a 5-DoF arm's minimum-reach dead zone (an arm generally cannot fold back
   onto its own mounting point). The probe measured this directly rather
   than guessing.

**Consequences.** M02's rendered PNGs regenerate (`docs/images/m02-scene.png`,
`m02-drawer-view-closed.png`, `m02-drawer-view-open.png`, all re-rendered at
1280x720; the physics-stability probe was re-run and still PASSes with
unchanged numbers). The demo command's grammar and skill vocabulary remain
valid — nothing in M04/M05 changed.

**The reachability probe's own result must be reported plainly, not
softened:** the drawer-face target still FAILED for both arms (residual
0.0226 m for arm A, 0.2135 m for arm B, tolerance 0.01 m; see
`docs/hardware/m06-reachability-probe.md`) — the predicted dead-zone risk was
real. Per this module's own stop rule, this triggered a STOP rather than a
third scene guess: the arm's actual reachable envelope was measured instead
(a coarse grid sweep, ~95 reachable points for arm A and ~57 for arm B,
reported in full in the probe's report) so a future drawer position can be
chosen from measured data. `pick`/`place`/`handoff`'s three non-drawer targets
(plate/mug/bottle at rest) all PASSED the same probe for both arms (residuals
0.004–0.008 m, no new collision) — the pinch-point retarget did not break
basic kinematic reachability for the objects those three skills actually use.
Per the task's explicit reporting rule ("if the probe fails, report the reach
envelope instead and stop"), the four skills were **not** re-run this pass;
re-running them is deferred until a reachable drawer position is chosen from
the envelope data.

**A genuine, separate finding surfaced by building this probe, flagged but
not fixed here:** at the default rest pose (`reset(seed=0)`, before any IK
solve), arm A and arm B already interpenetrate substantially — up to ~6 cm
penetration between `armA_lower_arm` and `armB_wrist`, independent of any
target. This is an ADR-021 arm-placement/rest-pose question (the ~0.30 m
reach / 0.50 m base-gap assumption was never checked against the compiled
model's actual rest configuration), not a consequence of either fix in this
ADR, and fixing it is out of this module's scope.

**M02's own done-when list is retroactively incomplete, and this is the
supplement, not a rewrite of history.** M02 verified the scene *compiles*,
*renders*, and is *physically stable* (no NaN, no tunneling) — it never
verified the scene is *reachable*. That gap is exactly why the drawer
geometry defect surfaced only now, at M06, rather than at M02 when the scene
was authored. `scripts/probe_reachability.py` is the check M02's done-when
list should have included from the start; it is added now, against M06's
findings, as a documented retroactive supplement to M02 rather than a claim
that M02's original done-when criteria were met with reachability included.

---

### ADR-026 — "Home" rest keyframe; envelope re-measured from a valid pose; drawer moved onto the correct axis

**Recorded:** Sept 12, 2026 · **Supersedes:** ADR-025's drawer reposition
(the axis it moved was wrong) · **Corrects (not amends) a load-bearing
assumption in:** ADR-021 (the ~0.30 m reach / 0.50 m base-gap "0.10 m shared
band" was never checked against the compiled rest pose) · **New ADR rather
than an ADR-025 amendment** because this is a structural correction — it
replaces the pose the whole reachability story was measured from, not a
follow-on tweak to ADR-025's own decision.

**Context.** ADR-025's own probe reported a "Baseline finding," flagged but
explicitly not fixed: at `reset(seed=0)`, *before any IK solve runs*, arm A
and arm B's default rest pose (every arm joint at its compiled default, 0
rad, unmodified per ADR-016) already interpenetrates. Measured directly
(not estimated) via MuJoCo's own contact list: **34 total contacts, 29 of
them armA↔armB geom pairs**, with penetration depths up to **-0.0597 m**
(`armA_wrist` vs. `armB_wrist`; also `armA_gripper` vs. `armB_lower_arm`
-0.0574 m, `armA_lower_arm` vs. `armB_gripper` -0.0574 m, `armA_wrist` vs.
`armB_lower_arm` -0.0379 m). Root cause: both arms' zero-angle configuration
extends fully forward into the shared handoff band — the arms' own rest
posture, independent of the drawer or any prop, was occupying the workspace
this whole project's reachability story depends on. `docs/hardware/m02-physics-stability.md`
already carried a caveat that predicted exactly this: its floor/NaN check
"catches interpenetration only indirectly" and "never asserts on
`data.contact.dist` directly" — this scene passed that probe for the entire
time it also carried a 6 cm self-interpenetration, because the two checks
measure different properties.

**Decision, four parts.**

1. **A named `<key name="home">` keyframe**, generated in the hand-authored
   region of `scripts/gen_dual_scene.py` (not a hand-edit of the compiled
   XML, and not a change to any upstream joint `ref` — ADR-016's "asset
   kinematics unmodified" holds; this is an ADDITIONAL key, not an edit to
   `qpos0`). Both arms fold back identically: `shoulder_pan=0`,
   `shoulder_lift=-1.2`, `elbow_flex=-1.6`, `wrist_flex=0`, `wrist_roll=0`,
   gripper at its range's high end (open, `+1.7453` rad — resolved from the
   asset's own `gripper` joint range, not guessed, matching the convention
   `control/skills_scripted.py`'s `GRIPPER_OPEN_FRACTION=1.0` already
   documents). **The originally-suggested `elbow_flex=-1.8` is OUTSIDE the
   asset's `-1.69..1.69` range; `-1.6` is the in-range fold used instead,
   asserted against the upstream range at generation time so this cannot
   silently drift out of bounds again.** Despite armB's mount carrying a
   180°-rotated quaternion (ADR-021), applying the SAME signs to both arms
   — not mirrored — was verified (not assumed) to be correct: an
   exploration pass rendered every sign combination of
   (`shoulder_lift`, `elbow_flex`) to PNG and counted contacts; identical
   signs on both arms produced a visually symmetric fold in both directions
   tried, mirrored signs produced a visibly lopsided pose (one arm folded
   low, the other raised). Of the two symmetric candidates, `-1.2/-1.6`
   measured **zero self-collision and zero cross-arm contacts**, vs.
   `+1.2/+1.6`'s 13 self-collisions per arm plus a -0.0189 m table
   penetration. `TableSettingEnv.reset()` (`src/bimanual/sim/env.py`,
   authorized for this task) now applies this key via
   `mj_resetDataKeyframe` when present, falling back to plain
   `mj_resetData` if a `scene_path=` has no "home" key. **Measured result:
   at `reset(seed=0)`, zero armA↔armB contacts (down from 29) — the
   acceptance bar (no cross-arm contact with negative `dist`) is met, not
   approximately but exactly (there are no cross-arm contacts to have a
   sign at all).**

2. **The reachable envelope was re-measured from this corrected pose**
   (`scripts/probe_reachability.py`, re-run on bm-ptl), with a finer z grid
   (0.02 m steps, 0.20–0.50 m, vs. the original 0.1 m steps that left the
   true floor unresolved in (0.20, 0.30]). **The previous envelope
   (`docs/hardware/m06-reachability-probe.md`'s first section) is
   superseded, not deleted** — it was sampled from an invalid, interpenetrating
   rest pose and is kept only as the historical record of why this
   correction exists; the probe script now APPENDS new sections rather than
   overwriting. New measured envelope: **Arm A** 186 reachable grid points,
   x∈[-0.30,0.30], y∈[-0.35,0.10], z∈[0.24,0.50]; **Arm B** 154 points,
   x∈[-0.30,0.30], y∈[-0.12,0.10], z∈[0.24,0.50]. **Shared handoff band
   (y-range intersection): y∈[-0.12, 0.10] m.** **ADR-021's "~0.10 m shared
   band" was a base-spacing-and-nominal-reach assumption, never checked
   against the compiled rest pose; it is now superseded by this measured
   band**, which happens to be close in width but was arrived at by
   measurement, not by the original arithmetic.

3. **The drawer was moved a second time, onto the correct axis.**
   ADR-025 moved `drawer_housing` outward in y (further from the table
   centre, to put the closed face flush with the table edge). Measured
   against the corrected envelope, that position (closed face at y=-0.25)
   is **unreachable by either arm** (residuals 0.32 m / 0.18 m against a
   0.01 m tolerance) — ADR-025 moved the wrong axis. **The actual
   constraint was height**: z=0.28 already fit between the reachable floor
   and the tabletop underside (0.33); it simply needed to sit over a y
   where the arms' envelope reaches that low. `drawer_housing` moves to
   y=0.08 (closed face y=0.0, table centre, inside the measured envelope
   for both arms — IK converges for both, residual ≈0.009 m each). z is
   UNCHANGED from ADR-025 (0.28); no height reduction was needed once the y
   position was corrected. Docs images (`m02-scene.png`,
   `m02-drawer-view-closed.png`, `m02-drawer-view-open.png`) were
   re-rendered at 1280×720. **Traded away, reported rather than hidden:**
   at y=-0.17 the open drawer protruded 15 cm past the table edge,
   legible from `drawer_view`; at y=0.08 the open drawer stays entirely
   under the tabletop footprint. The open/closed states remain visually
   distinguishable (box position and colour against the housing change
   between the two rendered images) but the "pops out past the table"
   framing is gone.

4. **Step 4's own validation surfaced a fourth, separate finding, and this
   ADR stops here rather than chasing it.** Re-running the full probe
   against the repositioned drawer: `closed_drawer_face` and
   `bottle_at_rest` now converge kinematically (residual <0.01 m) for BOTH
   arms — the envelope fix worked — but **every one of the four targets
   still fails the probe's collision check**, including `plate_at_rest` and
   `bottle_at_rest`, whose target positions never moved during this whole
   correction. Tracing the failing solves (e.g. `plate_at_rest`, arm A:
   `shoulder_lift` solves to its `+1.7453` rad limit) shows
   `bimanual.control.ik.solve_position_ik`'s Newton/DLS solve converging to
   a configuration that swings the arm down and back **through** the
   tabletop (confirmed directly: `table_top`/`drawer_housing_back` contacts
   at up to -0.049 m penetration for a solve whose *position* residual is a
   perfectly good 0.009 m) — the IK solver is position-only with no
   obstacle term (ADR-024), and the "home" seed apparently lands its
   Newton iteration in a worse local minimum for these targets than the old
   fully-extended seed did. **This collision was never actually fixed by
   ADR-025 either** — the old probe's collision check compared a raw
   contact COUNT against a ~29-contact baseline, so any solve producing
   fewer than 29 new contacts silently reported `collision=False`
   regardless of what those contacts were; dropping the baseline to 0 (via
   this ADR's keyframe) removed that masking and made the true, pre-existing
   collision state visible for the first time. Per this task's explicit
   stop rule ("if any target still fails, STOP and report — do not proceed
   and do not guess at another position"), **no further drawer position was
   tried**: `plate_at_rest`/`bottle_at_rest` fail identically without the
   drawer having moved at all, so repositioning again would not address it.
   Fixing it means changing `src/bimanual/control/ik.py` (multi-start
   solving, an obstacle-aware cost term, or target-specific seeding instead
   of always seeding from "home") — out of this task's scope by instruction,
   and left for a follow-up module, not silently absorbed here.

**Consequences.** `TableSettingEnv.reset(seed=0)` now returns different
`qpos`/observations than every prior caller assumed — this was flagged
before making the change, not discovered after. `scripts/probe_physics_stability.py`
was re-run (still PASS, unchanged numbers; its own caveat, recorded before
this ADR, predicted exactly why it could not have caught the original
interpenetration) and `python -m pytest tests/` was re-run: **4 tests in
`tests/test_skills.py` (`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`) now fail, timing out during IK
approach from the new folded starting pose.** These tests encode the OLD
rest pose's behavior; per this task's instruction they are reported here,
not silently edited — re-tuning the scripted skills/IK for the new home
pose is explicitly out of this task's scope and is deferred to a follow-up
invocation. `git diff --stat -- scenes/so101/` stays empty throughout (all
changes are in `scripts/gen_dual_scene.py`'s hand-authored region, the
generated `so101_dual_table.xml`, and `src/bimanual/sim/env.py`).

---

### ADR-027 -- Waypoint staging in scripted skills for collision-safe motion

**Recorded:** Sept 12, 2026 -- **Follows:** ADR-024 (position-only IK),
ADR-026 (home keyframe + re-measured envelope + drawer move) --
**Fixes:** the M06a follow-up ADR-026 deferred ("re-tuning skills/IK for
the new home pose ... deferred to a follow-up invocation") -- **Does NOT
touch:** `src/bimanual/control/ik.py`, `executor.py`'s interface,
`scenes/so101/`, `src/bimanual/command/`, `src/bimanual/language/`

**Context.** ADR-024's position-only IK has no collision awareness by
design (a deliberate simplification, not a bug: relaxing orientation
entirely was chosen over adding an obstacle term). M06a handed the
solver raw, single-shot targets -- "descend onto the grasp point" from
wherever the arm currently was -- which is exactly what let ADR-026 catch
solves that converge kinematically while tunneling through `table_top`
(measured up to -0.064 m penetration for `open_drawer`'s old top-down
approach at the drawer's new position). The reachability envelope ADR-026
measured also includes solutions of this kind: the sweep checked residual
only, never contacts, so a "reachable" grid point could be a tunneling
one.

**Options.**
- (a) A collision-aware solver (an obstacle/repulsion term added to
  `ik.solve_position_ik`, or a sampling-based planner). Rejected: 2+ days
  of work against a 5-day-remaining schedule, and out of this task's
  explicit scope (`ik.py` is declared not-to-be-modified -- "the solver is
  fine; the caller wasn't using it correctly").
- (b) Waypoint staging in the controller: every skill drives through a
  sequence of small, explicitly validated APPROACH/DESCEND/GRIP/RETREAT
  waypoints (tutor note 06's pattern) instead of one long reach, with each
  waypoint checked for BOTH IK convergence and new collision before the
  next one is attempted. **Chosen.**
- (c) Move the arms so the table leaves the reachable set entirely.
  Rejected: would shrink the measured shared handoff band
  (y in [-0.12, 0.10], `docs/hardware/m06-reachability-probe.md`'s
  ADR-026 section) and break `handoff`, which already depends on that
  band being as wide as measured.

**Decision.** (b). `src/bimanual/control/skills_scripted.py` now owns all
motion staging; `ik.py` stays an unmodified, pure position solver. Five
new module-level constants govern every staged skill:
`CLEARANCE_HEIGHT_M=0.08`, `APPROACH_DESCENT_STEPS=500`,
`GRIP_HOLD_FRAMES=30`, `PULL_DISTANCE_M=0.15`,
`HANDOFF_POSITION_XYZ=(0.0,-0.01,0.35)` (y is the midpoint of the measured
shared band, not ADR-021's superseded arithmetic estimate). Every waypoint
is validated against a baseline contact snapshot taken once at the start
of the skill call: IK convergence (`ik.solve_position_ik`'s own residual,
re-queried after the physical drive, `< ik.IK_POSITION_TOLERANCE_M`) AND no
new cross-arm or arm-vs-`table_top` contact. On failure a skill returns
`SkillResult(success=False, reason="waypoint N failed [convergence|collision]: ...")`
and stops -- later waypoints are never attempted on top of a bad one.
`DEFAULT_STEP_BUDGET` raised 3000 -> 12000 (4x, a single shared default,
not per-skill overrides -- every skill grew by roughly the same
waypoint-count factor: `pick`=4 waypoints, `place`=pick+4, `handoff`=pick+8,
`open_drawer`=6, each capped at `APPROACH_DESCENT_STEPS`).

**`open_drawer` needed a different approach direction, and the reason is
geometric, not a matter of taste.** With the drawer at `drawer_housing`
y=0.08 (ADR-026), its closed face sits at y=0.00 -- directly beneath the
`table_top` slab (y in [-0.25, 0.25], z in [0.33, 0.35]). A vertical
approach descending from above the handle necessarily crosses that z-range
at y=0.00, i.e. tunnels. `open_drawer` is rebuilt as a LATERAL approach:
APPROACH to a waypoint outside the table footprint at drawer height,
INSERT by translating in +y while staying at drawer height (never
crossing z in [0.33, 0.35]), GRIP, PULL by `PULL_DISTANCE_M`, RELEASE,
RETREAT back outside the table edge.

**The prior reachability question was answered empirically, not assumed,
and the answer is negative for BOTH candidate approaches.** A one-off
diagnostic (`ik.solve_position_ik` plus a real 1500-step physical
closed-loop drive, arm A, from the "home" reset pose) found:
- The drawer's own closed-face target (0, 0.00, 0.28) IS kinematically
  reachable (residual 0.009 m) -- but only via a solution that tunnels
  through `table_top`/`drawer_housing_back` at up to -0.064 m penetration
  (a real contact check on the solved configuration, not inferred).
- A lateral waypoint outside the table footprint at drawer height (e.g.
  (0, -0.30, 0.28)) does NOT converge at all: IK residual stalls at
  0.09-0.32 m regardless of how many steps are given, including the full
  1500-step physical drive. This holds across y in {-0.30, -0.32, -0.35}
  and z in {0.28, 0.30, 0.32}.
- The ADR-025 flush-with-table-edge drawer position (housing y=-0.17, face
  y=-0.25) -- hypothesised as the likely answer, since the re-measured
  envelope's bounding box nominally reaches y=-0.35 -- was checked directly
  against that same envelope and does NOT converge either (residual
  0.322 m for arm A, 0.179 m for arm B), because the envelope's bounding
  box is not a claim that every interior point is reachable: at low z
  (drawer height, 0.24-0.30 m), arm A's actually-reachable y only extends
  to about -0.12, never to -0.20 or beyond, at the grid's full 0.02 m z
  resolution. Arm A's shoulder is mounted at z=0.35 (the tabletop
  height); reaching *below* that height while positioned *outside or at*
  the table's own edge requires folding the arm back past its own
  mounting point, which this measurement shows the arm cannot do at this
  base placement.

**Per the explicit stop rule for this finding ("if the only solutions
tunnel, STOP and report -- do not invent a third drawer position"),
`open_drawer` is still built exactly as specified** (real, tested
waypoint code, not a stub) **and correctly, safely FAILS at waypoint 1
(APPROACH) with a convergence error** on the committed scene -- refusing
the unreachable lateral approach rather than silently reverting to the
vertical path that tunnels. This is the intended, honest behaviour of the
fix, not a defect in it.

**A second, independent reachability gap surfaced while fixing `pick`,
also stop-and-report, not routed around:** `mug_at_rest` itself (before
any grasp-point offset) does not converge for arm A from the home pose
(residual 0.120 m) -- independently confirmed by
`docs/hardware/m06-reachability-probe.md`'s own ADR-026 Step-4 table. This
blocks `handoff(mug, A->B)` at its internal `pick(A, mug)` stage, for the
same class of reason as the drawer (a kinematic reach limit from the
current arm base placement), not a staging defect.

**A separate, non-reachability gap: grasp reliability (ADR-024's existing,
out-of-scope finding), re-confirmed under staging.** `pick(A, plate)` now
clears every waypoint's validation (IK converges, jaw closes fully, zero
new arm-vs-table_top contact beyond the measured graze/tunneling boundary
-- see `TABLE_COLLISION_DEPTH_TOL_M`) but the plate still never lifts, even
when the GRIP dwell is extended to 300 steps (well past `GRIP_HOLD_FRAMES`,
tested directly). This is ADR-024's already-documented pinch-point/jaw-
geometry limitation (the site the physical loop tracks and the pinch point
the solver targets are offset and that offset rotates with the
uncontrolled approach orientation), not something waypoint staging can
fix without orientation control -- out of this task's scope by the same
instruction that keeps `ik.py` untouched.

**`GRASP_POINT_OFFSET_M["plate"]` direction corrected, magnitude
unchanged.** The original `(+0.09, 0, 0)` rim offset drove three joints
(`shoulder_lift`, `elbow_flex`, `wrist_flex`) simultaneously to their
range limits from the new home pose and stalled (residual 0.138 m at the
grasp point). Since the plate's rim is circular, any direction is an
equally valid physical grasp feature; `(0, +0.09, 0)` -- the same radius,
a different point on the rim -- converges cleanly (residual 0.008 m) with
no joint pegged at its limit. This is a caller-side reachability choice
(which side of a symmetric feature to aim at), not a change to `ik.py` or
to what "the plate's rim" means physically.

**`TABLE_COLLISION_DEPTH_TOL_M=0.001` m.** A bare "any new contact fails"
rule cannot distinguish a genuine tunneling penetration (measured 0.02-
0.065 m for the drawer) from an incidental, physically-inevitable graze:
every prop rests directly on `table_top` by design (the plate sits ~0.006 m
above the surface), and the jaw's own collision geometry is large relative
to the props (ADR-024: bounding-sphere radius up to ~8.4 cm), so a `pick`
descend onto a table-height grasp point was measured to create one contact
at `dist=-0.00007` m -- roughly three orders of magnitude shallower than
the measured tunneling depths. The threshold sits two-and-a-half orders of
magnitude above the measured graze and two orders of magnitude below the
measured tunneling depths.

**Test suite verdict (`pytest tests/test_skills.py`, bm-ptl), reported
plainly.** The four tests ADR-026 left failing
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`)
**still fail after this fix** -- not because the staging fix does not work,
but because each hits one of the two genuinely out-of-scope gaps above
(arm reach limits for `open_drawer`/the mug; ADR-024's grasp-reliability
gap for `pick`/`place`). They are reported here, unedited, rather than
weakened to pass, per this task's instruction ("if a test encodes stale
assumptions, report it rather than silently rewrite it"). Two new tests
(`test_open_drawer_fails_without_tunneling_through_table`,
`test_pick_plate_waypoints_progress_without_collision`) are this ADR's
actual regression coverage: they assert that a skill which cannot complete
its task still never creates a new collision and stops at the first
un-validatable waypoint -- this is what changed, and it now passes where
the old code would have silently tunneled. All of the suite's 58
non-`test_skills.py` tests pass, and 4 of `test_skills.py`'s own 8 pass;
no regression outside the two reachability/grasp gaps already known.

**Consequences.** `skills_scripted.py` (not `ik.py`) owns motion staging
going forward -- any future skill must be built the same way. Step count
per skill rose roughly 4x (matching `DEFAULT_STEP_BUDGET`'s new value).
`open_drawer` specifically requires a lateral, under-table approach
because its target lies beneath the slab, and that approach is currently
unreachable by arm A at this base placement -- opening the drawer remains
undemonstrated until either the arm base placement or the solver gains
obstacle awareness (both out of this task's scope). `handoff(mug)` is
blocked the same way. `pick(plate)`/`place(plate)` are blocked by ADR-024's
separate, already-known grasp-reliability gap. None of these three gaps
is new; this ADR's contribution is that failures now localise to a named
waypoint and a named reason (convergence vs. collision) instead of an
undifferentiated "the skill failed," and that a failing skill is now
verifiably collision-safe rather than silently tunneling.

---

### M06 Grasping Mechanism Overview

M06 implements `pick`, `place` and `handoff` for the bimanual SO-101 arms in
MuJoCo. The gripper's jaw meshes are non-convex, C-shaped housings that
MuJoCo collapses to overlapping convex hulls with no decomposition declared
(MuJoCo issue #239); 0 of 30 measured caliper thicknesses ever achieved
sustained two-jaw contact. **ADR-028** disables jaw-mesh collision and adds
small finger-pad collision primitives that verifiably move with jaw closure
— a geometry-level fix only; no skill reached GRIP under it in that same
commit. **ADR-029** answers grasping at the mechanism level instead: it
implements grasping as a MuJoCo weld equality constraint, toggled on
proximity and jaw closure — an explicit abstraction of physical grasping,
**not friction-based finger contact**, and disclosed as such per ADR-015.
**ADR-031** freezes the driven arm's commanded pose during the GRIP/RELEASE
dwell so the closing jaw's own shifting pinch point cannot make the arm
retreat mid-grasp. **ADR-033** gives the water bottle its own
APPROACH/RETREAT hover height so the hover point clears the bottle's own
physical top instead of sitting below it. **ADR-034** fixed a real bug in
`run_place` (it unconditionally re-picked an object the arm already held,
sending it to an unreachable hover) but `place(A, water_bottle, table)`
still does not work — a second, different IK residual failure (0.0138 m
against the 0.01 m tolerance, unimproved at four times the step budget, with
the destination x landing on the place path's own `+0.30` safety clip bound)
now surfaces at the destination-approach waypoint instead. **Handoff does
not currently work**, and the understanding of why has changed since the
original three reachability sweeps (ADR-032, two passes, plus an earlier
pass) reported zero mutually-reachable candidates: a later warm-start IK
diagnostic (`2115a1e`) showed those sweeps were measuring a seed-dependent
IK local minimum from the folded "home" pose, not a true kinematic limit of
the arms' reach. **ADR-035** built on that diagnostic and added target
interpolation to the `handoff` traverse, which genuinely carries the skill
three waypoints further than before — waypoint 3 (`to_arm`'s APPROACH,
previously the sweep's own failure point) now converges reliably. The skill
still fails, but one waypoint later than it used to: at waypoint 4, on an
arm-vs-`table_top` collision during `to_arm`'s interpolated descend, not on
unreachability.

---

### ADR-028 — Finger-pad primitives (MuJoCo convex-hull fix), pads verified to move, but no `pick()` reaches GRIP under the current approach-collision check

**Recorded:** Sept 12, 2026 · **Follows:** `docs/hardware/grasp-envelope.md`
(diagnostic: 0 of 30 caliper thicknesses achieved sustained two-jaw contact,
convex hulls overlap -0.0206..-0.0345 m at every joint angle) · **Cites:**
MuJoCo GitHub issue #239's documented finger-pad pattern for mesh-gripper
collision, https://ggando.com/blog/so101-rl-lift (reports working SO-101
grasping with this pattern), https://maegantucker.com/ECE4560/assignment8-so101/
(course material teaching it) · **Not accessed**, per instruction — cited only.

**Context.** `docs/hardware/grasp-envelope.md` measured the root cause
directly: MuJoCo collapses a `type="mesh"` collision geom to its convex hull
with no decomposition declared anywhere in this asset, and both jaw parts
(`wrist_roll_follower_so101_v1`, `moving_jaw_so101_v1`) are non-convex
C-shaped housings whose hulls overlap at every angle in the joint's range
(-0.03454 m closed to -0.02062 m at the least-overlapping angle). No object
placed there can ever be read as anything but embedded in solid material on
both sides.

**Options.** (a) weld-based grasping — rejected, a workaround that reads as
not-really-grasping and would need disclosure; (b) finger-pad primitives per
the cited pattern — **chosen**; (c) non-prehensile manipulation — rejected,
scope change.

**Decision (b), implemented in `scripts/gen_dual_scene.py` only** (never
`scenes/so101/`, confirmed empty diff below): (1) `disable_jaw_mesh_collision()`
sets `contype="0" conaffinity="0"` on the same two jaw MESH collision geoms
`JAW_COLLISION_MESHES` already identifies (visual rendering, a separate
`class="visual"` copy, untouched); (2) `add_finger_pads()` adds one
`type="box" size="0.00125 0.00125 0.00125"` collision geom per jaw, at the
task's own verbatim positions: `static_finger_pad` at local `pos="-0.008875
0.0 -0.100"` as a child of `{prefix}gripper` (the fixed jaw body), and
`moving_finger_pad` at local `pos="-0.01136 -0.076 0.019"` as a child of
`{prefix}moving_jaw_so101_v1` (the moving jaw body) — both bodies asserted to
resolve to a real match. `friction="1 0.05 0.001"`, `contype="1"
conaffinity="1"` on both pads, exactly as specified.

**Why this is not a repeat of the reverted Fix D.** Fix D's replacement
sphere sat at local `pos="0 0 0"` on the moving jaw body — exactly on that
body's own hinge rotation axis — so it never moved as the jaw opened or
closed (identical gap to five decimals at both joint limits, DECISIONS.md's
Fix D revert entry). `moving_finger_pad`'s local pos is offset in all three
axes from that origin, so this is a structurally different placement, not
merely a re-application of the same mistake — and Step 3 below exists
specifically to catch a repeat before anything downstream is trusted.

**Step 3 gate — pad separation across joint angle, measured on bm-ptl**
(`scripts/probe_pad_separation.py`, new diagnostic script, not shipped skill
code; reads `armA_static_finger_pad`/`armA_moving_finger_pad` world
`geom_xpos` directly, resetting to the "home" keyframe then overriding only
`armA_gripper`'s qpos per angle):

| angle | qpos (rad) | pad separation (m) |
|---|---:|---:|
| fully closed | -0.1745 | **0.00600** |
| midway | +0.7850 | **0.07621** |
| fully open | +1.7453 | **0.13188** |

Spread across the three angles: 0.12589 m. **GATE PASSED** — separation
changes materially and monotonically with joint angle (smallest near
closed, as expected for a pinch point), the opposite of Fix D's
identical-to-five-decimals failure. Both pad geoms genuinely move with
their respective bodies.

**Consequences.** Note: 6-132 mm is centre-to-centre distance between the
2.5 mm cube pads. The surface-to-surface gap — what actually fits between
the jaws — is smaller, measured at roughly 2.5 mm closed to 120 mm open via
`mj_geomDistance`. Quote the surface figure in judge-facing material and
say which quantity it is.

**Step 4 — pick(A, ·) in force order, run on bm-ptl, reported exactly as
measured, not softened.** `pytest tests/test_skills.py` first, to confirm no
new regression from the generator change: **5 failed / 3 passed**,
byte-for-byte the same specific failures already on record in the "M06a Fix
D reverted" entry above (including `test_pick_plate_waypoints_progress_without_collision`,
already flagged there as a pre-existing, undecided conflict with ADR-027's
own regression-test requirement — not newly broken by this change).

| Prop | Result | frames_used | reason | z delta | crossed lift threshold? |
|---|---|---:|---|---:|---|
| fork | FAIL | 313 | `waypoint 1 (approach) failed [collision (arm-vs-prop: fork (dist=-0.0072 m); threshold=-0.005 m)]` | -0.0035 | no |
| spoon | FAIL | 306 | `waypoint 1 (approach) failed [collision (arm-vs-prop: fork (dist=-0.0055 m); threshold=-0.005 m)]` — **anomaly:** target is spoon, the collision reported is against the *fork* prop, sitting nearby | -0.0021 | no |
| plate | FAIL | 296 | `waypoint 1 (approach) failed [collision (arm-vs-prop: plate (dist=-0.0081 m); threshold=-0.005 m)]` | -0.0034 | no |
| mug | FAIL | 317 | `waypoint 1 (approach) failed [collision (arm-vs-prop: mug (dist=-0.0090 m); threshold=-0.005 m)]` | -0.0028 | no |
| water_bottle | FAIL | 256 | `waypoint 1 (approach) failed [collision (arm-vs-prop: water_bottle (dist=-0.0061 m); threshold=-0.005 m)]` | -0.0007 | no |

**Honest interpretation: none of the three outcomes the task's own
interpretation guide anticipated is quite what happened, and that mismatch
is itself the finding.** All five props fail identically at **waypoint 1
(APPROACH)** — before the skill ever reaches DESCEND or GRIP. The pad
geometry this ADR adds is therefore **not exercised at all** by any of these
five runs; the pinch never gets a chance to form. This is not new: it is
**ADR-027 Step 5's own already-documented arm-vs-prop collision check**
(`skills_scripted.py`, out of this task's scope to touch) firing during the
blind, obstacle-unaware IK approach path (ADR-024) — the same condition that
entry already flagged for the plate specifically ("the arm's blind approach
path was apparently ALREADY grazing the plate during ordinary APPROACH even
with the original flush geometry"), now confirmed to occur identically for
**all five** props, not only the plate.

**Proof this is unrelated to the pad fix, not just an assertion:** `pick(A,
plate)`'s numbers here (`dist=-0.0081 m`, `frames_used=296`) are **bit-for-bit
identical** to the pre-ADR-028 baseline recorded in the "M06a Fix D reverted"
entry above, measured when the jaw mesh collision was still enabled and no
pads existed. Changing the jaw's collision geometry from mesh to pads
produced **zero** change to this failure, which is the expected result if —
and only if — the contact triggering it belongs to a different arm geom
entirely (most plausibly the wrist/forearm, brushing the prop during
approach), not the jaw. That is consistent with, not contradicted by, this
fix: the pad fix targets the PINCH, and the approach-phase collision check
fires well before any pinch is attempted.

**No further action taken in this commit, per its own explicit
instruction** ("Do NOT modify prop masses, IK strategy, or grasp offsets...
we are testing the pad fix in isolation"): `skills_scripted.py`'s
arm-vs-prop debounce/threshold logic and `ik.py`'s obstacle-unaware approach
path are both out of scope here, and neither was touched. **What this commit
proves:** the convex-hull geometry defect diagnosed in `docs/hardware/
grasp-envelope.md` is fixed at the geometry level (Step 3's gate) and does
not regress anything measured before (Step 4's pytest/plate parity). **What
it does not yet prove:** whether the fixed geometry actually grasps
anything, because no skill run in this commit reaches the GRIP waypoint for
any prop — that remains blocked by the separate, already-documented
approach-collision gap, and by the `±3.35 N` actuator ceiling for plate/mug/
bottle specifically, neither of which this commit addresses.

`git diff --stat -- scenes/so101/` confirmed empty before commit. Files
changed: `scripts/gen_dual_scene.py` (`disable_jaw_mesh_collision`,
`add_finger_pads`, wired into `main()`), the regenerated
`src/bimanual/sim/assets/so101_dual_table.xml`, and
`scripts/probe_pad_separation.py` (new diagnostic, not shipped skill code).

**Correction (Sept 13, 2026, M06 Phase 2 follow-up session).** A later
session was handed a claim that the jaw-body mesh disable this entry (and
`8f09f8c`, below) describes was still incomplete -- specifically, that
`sts3215_03a_v1`, `wrist_roll_follower_so101_v1` and `moving_jaw_so101_v1`
were still `COLLIDABLE` in the compiled model. **Checked directly, not
assumed, on both a development laptop and bm-ptl** (`mujoco==3.2.7`,
`model.geom_contype`/`geom_conaffinity` read for every geom on the relevant
bodies): all three meshes already report `contype=0 conaffinity=0` on both
arms, and both finger pads remain collidable, exactly as this entry and
`8f09f8c` intended. `8f09f8c`'s "complete jaw collision disable" (see that
entry below) was, in fact, complete -- the disable was never incomplete, and
no further code change was needed or made. The full audit (why the given
premise did not reproduce, and what was checked) is recorded in the "M06
Phase 2 follow-up" entry in `DECISIONS.md`, above ADR-030. This
correction exists so a future reader who finds this ADR's own "no further
action taken" language does not go looking for a still-open gap that was
never there.

**Source commit:** `c6feeb1` ("ADR-028: finger-pad primitives per
ggando/MuJoCo #239 pattern (fixes convex-hull grasp problem)."), found via
`git log`/`.git/logs/HEAD` — commit message confirmed to match this ADR
title. The Sept 13 correction paragraph above was recorded in a later
commit, `b9e9cb4` ("M06: audit ADR-028 hull disable..."), per `DECISIONS.md`.

---

### ADR-029 — Weld-based grasping mechanism (Phase 1: mechanism verified, not yet wired into skills)

**Recorded:** Sept 13, 2026 · **Follows:** `docs/hardware/grasp-envelope.md` (0 of 30
caliper thicknesses achieved sustained two-jaw contact — the gripper's jaw meshes
collapse to permanently-overlapping convex hulls, MuJoCo issue #239) and
`DECISIONS.md`'s ADR-028 entry (finger-pad primitives fixed the geometry, verified
6-132 mm pad separation sweep, but "no `pick()` reaches GRIP under the current
approach-collision check" -- the arm's reach envelope plus the ~8 cm pinch-point
kinematic offset, ADR-025, put every graspable target at the edge of what this 5-DOF
IK can reach; `docs/hardware/m06-grip-diagnostic.md` and
`m06-grip-diagnostic-after-fix.md`). **Phase 1 only** -- this entry covers the
mechanism's own verification; wiring it into `pick`/`place`/`handoff` is Phase 2,
contingent on this entry.

**Context.** Contact-based grasping in this scene is not a code bug to keep chasing;
it is a structural limit of the simulated gripper's geometry and this arm's
kinematics, independently confirmed by two prior, unrelated diagnostics (the caliper
sweep and the GRIP-stage instrumentation). A grasping mechanism is needed that
abstracts the failing contact subsystem while preserving the rest of the
perception-to-action pipeline.

**Options.** (a) keep debugging contact-based grasping -- rejected, the limit is
structural (reach envelope), not tactical; (b) reposition the arm bases -- rejected,
4-6 h with an uncertain outcome, and it would restart cross-arm collision and
reachability validation from scratch; (c) weld the object to the gripper via a MuJoCo
equality constraint, toggled at runtime -- **chosen**, standard sim-robotics practice
(MoveIt's attached objects, PyBullet's fixed constraints, academic sim-to-real work
all abstract grasp contact the same way).

**Decision (c), implemented as follows:**

1. **`src/bimanual/sim/grasp.py`'s `WeldGrasp`** tracks at most one held object per
   arm (`{'A': None, 'B': None}`). `attempt_grasp(arm, object_name,
   distance_threshold_m=0.05, closure_threshold=0.3)` attaches only if BOTH gates
   hold: the `armX_gripper` JOINT's qpos is below `closure_threshold` (jaws closing;
   "open" is the HIGH end of this joint's range, so "below threshold" correctly reads
   as "closing"), AND the `armX_gripper` BODY's (the fixed jaw, **not** the moving
   jaw and **not** the joint of the same name -- the exact naming trap `ik.py` and
   `GLOSSARY.md` already document) world position is within `distance_threshold_m` of
   the object body's world position. It refuses, logging the specific reason,
   otherwise. `release(arm)` deactivates the weld; `is_holding(arm)` reports it.
   Refusing when either gate fails (never "always weld") is the entire point --
   ADR-029's Consequences below.
2. **Activation path: (a) pre-declared, chosen over (b) runtime creation.** All 10
   `(armX_gripper, prop)` weld equality constraints (5 props x 2 arms) are declared
   in the generated scene XML with `active="false"`, by
   `scripts/gen_dual_scene.py`'s new `build_weld_constraints()` (HAND-AUTHORED region
   only -- `scenes/so101/` is untouched, confirmed by `git diff --stat -- scenes/so101/`
   before commit, same convention as every prior ADR-021/ADR-028 change).
   `WeldGrasp` only ever toggles `data.eq_active` and rewrites `model.eq_data` for
   constraints that already exist; option (b) (creating a constraint at runtime) was
   never needed -- pre-declaring compiled without incident on the first try.
3. **The eq_data teleport gotcha, resolved empirically for the installed
   mujoco==3.2.7, not assumed from documentation.** `mjNEQDATA == 11`
   (`mujoco/include/mujoco/mjmodel.h`): `eq_data[0:3]` = anchor, `eq_data[3:10]` =
   relpose (3 position + 4 quaternion), `eq_data[10]` = torquescale -- but the shipped
   headers do not document what "anchor" and "relpose" actually mean geometrically.
   Determined by direct experiment (5 randomized-pose trials, weld `body1`=object /
   `body2`=the reference body, gravity enabled, 3000-step rollout, position AND
   orientation checked before/after): for `body1`=object, `body2`=gripper,
   ```
   anchor       = R(gripper_quat)^T @ (object_pos - gripper_pos)   # object's position
                  in the gripper body's own local frame
   relpose_pos  = (0, 0, 0)
   relpose_quat = conj(object_quat) * gripper_quat                 # the GRIPPER's
                  orientation expressed in the OBJECT's frame (reversed order
                  relative to the naive "object relative to gripper" -- this was the
                  one sign that a position-only teleport check would NOT have caught;
                  it only shows up as orientation drift over many steps)
   torquescale  = 1.0
   ```
   This held position to 2.7e-5 m (solver settling noise, not error) and orientation
   exactly (quaternion delta 0.0) across 5 random trials, 3000 steps each, under
   gravity. `mujoco`'s own `mju_rotVecQuat`/`mju_negQuat`/`mju_mulQuat` are used in
   `grasp.py`, not hand-rolled quaternion math, so the implementation tracks MuJoCo's
   own convention rather than a reimplementation of it.
4. **`scripts/probe_weld_grasp.py`** verifies the mechanism end to end against the
   real `TableSettingEnv`, with no skill layer involved. Full log:
   `docs/hardware/m06-weld-verification.md`. Run on bm-ptl (ADR-020); mirrored
   locally first (mujoco imports and compiles on this developer's laptop as of this
   session, contrary to ADR-020's original finding -- noted, not relied upon; the
   authoritative run and the committed artifacts are bm-ptl's).

**Verification results, reported exactly as measured:**
- **Teleport check:** fork position immediately before vs. after `attempt_grasp`
  activates the weld: **0.000000 m** (both position components identical to 8
  decimal places).
- **Tracking:** stepping the arm up (see the finding below on how) for 50 steps, the
  fork's world z rose from 0.3538 to 0.3551 m (+0.0013 m), tracking the gripper's own
  rise (0.3810 -> 0.3824 m, +0.0013 m) essentially 1:1.
- **Release:** `release('A')` returned `True`; `is_holding('A')` became `None`.
  30 further steps showed the fork's z stop rising and settle down (0.3551 -> 0.3533 m).
- **Negative control 1 (gripper OPEN, object in range):** `attempt_grasp` returned
  `False`, logged reason "gripper not closed enough (joint qpos=1.7453 rad >=
  closure_threshold=0.3000 rad)".
- **Negative control 2 (gripper CLOSED, object far -- arm left at the "home" rest
  pose):** `attempt_grasp` returned `False`, logged reason "too far (distance=0.5633 m
  >= distance_threshold_m=0.0500 m)".
- **MuJoCo warnings:** none, at any point in the run (`data.warning` checked, same
  convention as `scripts/run_skill.py`'s diagnostic).

**An honest, unplanned finding surfaced while building the verification script, worth
recording because it is a real property of this system, not a defect in `WeldGrasp`:**
`ik.solve_position_ik`'s pinch-point target (ADR-025), combined with ADR-024's fully
relaxed orientation, let the redundant 5-DOF solve satisfy a progressively-rising
pinch-point target by rotating the WRIST rather than raising the arm -- the pinch
point tracked the rising target (solver residual under 0.01 m throughout) while the
`armA_gripper` BODY (the actual weld attach frame) **fell**. `ik.py` is out of scope
to modify for this task, so the verification script's UP phase instead drives
`armA_shoulder_lift` directly (holding every other actuator at its current qpos),
which reliably raises the whole downstream chain with no orientation ambiguity. The
resulting rise is modest (millimetre-scale over 50 steps / 0.1 s sim time), consistent
with the `sts3215` actuator class's own `forcerange=-2.94 2.94` N*m capping how fast
one joint can lift the downstream mass against gravity in that time -- the same kind
of actuator force ceiling `docs/hardware/grasp-envelope.md` already measured for the
gripper actuator's own `forcerange=-3.35 3.35` N.

**Consequences.** Grasping is now **abstracted, not physically simulated** --
README and video must say so explicitly, per ADR-015's honesty rules (no number or
description implies contact-based grasping where a weld is doing the work). Pick,
place and handoff become executable end-to-end **once wired** (Phase 2, not this
commit). ADR-028's finger-pad work is retained as scene correctness (the pads still
move correctly and are still the physically modelled jaw geometry) even though grip
contact itself is abstracted around. M07's randomization stays meaningful: arm poses
and prop positions still vary session to session; only the attach *moment* is
abstracted, not the scene state leading up to it. Nothing in `skills_scripted.py`,
`executor.py`, `ik.py` or `scenes/so101/` was touched by this commit.

**Source commit:** `325feac` ("ADR-029: weld-based grasping mechanism (Phase 1:
mechanism verified, not yet wired into skills)."), found via `git log`/
`.git/logs/HEAD` — commit message confirmed to match this ADR title.

---

### ADR-030 — Weld wiring into scripted skills (Phase 2 Commit 2)

**Recorded:** Sept 13, 2026 · **Follows:** ADR-029 (weld mechanism verified, Phase 1),
M06 Phase 2 Commit 1 (ctrl-hold decision + measured drift gap).

**Context.** ADR-029 built and verified `WeldGrasp` in isolation
(`scripts/probe_weld_grasp.py`, `docs/hardware/m06-weld-verification.md`) but left it
unwired: "deliberately NOT wired into `pick`/`place`/`handoff` or `executor.py`". This
commit does that wiring and nothing else -- `ik.py`, `grasp.py` and `scenes/so101/` are
untouched.

**Decision.** `ScriptedSkillExecutor` constructs one `WeldGrasp` (`self.weld`), threaded
into `skills_scripted.run_pick`/`run_place`/`run_handoff` as a `weld` parameter:
- **`pick`'s GRIP** commands jaw closure, then calls `weld.attempt_grasp(arm, body_name)`
  every step until it returns `True` (frame recorded) or `GRIP_HOLD_FRAMES` (300) is
  exhausted, in which case the skill fails with the specific reason
  `weld_attach_failed_after_N_frames` -- distinguishable from an ordinary
  waypoint/collision failure.
- **`place`'s RELEASE** calls `weld.release(arm)` BEFORE commanding the jaw open, so the
  object is not kicked by the opening jaw's own moving collision geometry.
- **`handoff`** grips-and-attaches on `to_arm` the same way as `pick`, then adds a new
  gate BEFORE `from_arm` is ever released: `weld.is_holding(to_arm) == body_name` is
  checked explicitly; if false, the skill fails immediately with `handoff_transfer_failed`
  and `from_arm`'s weld is left untouched (object stays with `from_arm`, never ends up
  held by neither arm). Only past that gate does `from_arm` release (again,
  weld-then-jaws ordering) and the staggered retreat (`from_arm` first) proceed.
- **`SkillResult`** gained `weld_attach_frame: int | None` (the whole-skill-call frame at
  which `attempt_grasp` first returned `True`, or `None` if it never did) and
  `weld_active_at_end: bool` (`weld.is_holding` re-checked at return time), both with
  defaults so every pre-existing positional `SkillResult(...)` construction is unaffected.
- **`pick`'s success bar changes when `weld` is supplied**: `final_z > TABLE_SURFACE_Z +
  0.02` (a new constant, `WELD_PICK_SUCCESS_MARGIN_M`) AND `weld.is_holding(arm) ==
  body_name` -- an absolute-height-from-table-surface bar, per this task's own
  instructions, kept SEPARATE from the older initial-z-relative `PICK_LIFT_MARGIN_M`
  (0.03) that `run_pick` still uses when `weld=None`. `handoff`'s success similarly gains
  `weld.is_holding(to_arm) == body_name AND weld.is_holding(from_arm) is None` alongside
  its existing distance/lift check.

**Disclosed deviation: `ScriptedSkillExecutor.__init__` cannot construct `WeldGrasp`
eagerly.** The instruction as given was "`__init__` constructs it as `self.weld`" --
`WeldGrasp.__init__` requires a compiled `env` (it resolves equality-constraint/joint/
body ids against `env.model`), and `ScriptedSkillExecutor.__init__` takes no `env`
argument, matching every existing call site (`tests/test_skills.py`'s `executor()`
fixture, `scripts/run_skill.py`) which construct `ScriptedSkillExecutor()` bare and
supply `env` only later, per call, to `execute()`. `self.weld` therefore starts `None`
and is built lazily the first time `execute()` sees an `env` (`_ensure_weld`), rebuilt
only if a genuinely different `env` instance is later passed in. This is a correction to
the literal instruction, not a silent substitution -- recorded here per this task's own
"if the instructed text does not match what happened, correct it" rule.

**Verification (bm-ptl, `C:\Users\devcloud\project\ov_env\Scripts\python.exe`).**

- `pytest tests/test_skills.py`, BEFORE this commit's changes: **4 passed / 4 failed**
  (`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
  `test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b` fail;
  the other four pass) -- identical to Commit 1's own reported baseline.
- `pytest tests/test_skills.py`, AFTER: **4 passed / 4 failed, same four tests.** The
  drawer/mug-reach failures are byte-identical (kinematic reach limits this commit does
  not touch). The plate pick/place failures changed REASON, not outcome: previously
  `"did not lift plate: ... final_z=0.3523"`; now `weld_attach_failed_after_300_frames`
  (`final_z=0.3524`) -- the weld mechanism now engages and is exercised, and still does
  not attach for the plate's own rim-offset grasp point (see finding below); both are
  failures, so no regression.
- Isolated logic checks (FakeWeld stub, not the real `WeldGrasp` -- that mechanism's own
  correctness is ADR-029's job, already verified): confirmed `_dwell` breaks early at the
  exact step `attempt_grasp` first returns `True` and reports that step as `attach_frame`
  (`attach_on_call=7` -> `steps_taken=7, attach_frame=7`, `attempt_grasp` never called an
  8th time), reports `attach_frame=None` when it never attaches within budget, and that
  `run_pick`'s cumulative frame offset is correct (`grip_start_frames + local_index`
  measured as `1001` for a weld that attaches on its very first GRIP-dwell step, matching
  independently observed APPROACH+DESCEND frame counts from the real run below).

**New finding, measured directly, not assumed: for props whose grasp point IS reachable
(fork, water_bottle), `pick`'s GRIP now runs to completion but `WeldGrasp`'s own
proximity gate (`distance_threshold_m=0.05`, unmodified default) is missed by the time
the closure gate opens.** `pick(A, fork)` and `pick(A, water_bottle)` both return
`weld_attach_failed_after_300_frames` (frames_used=1300; GRIP_HOLD_FRAMES=300 exhausted).
A direct instrumented replay of `pick(A, fork)`'s GRIP dwell (`armA_gripper` BODY-to-fork
BODY distance, `armA_gripper` JOINT qpos, sampled every 20 steps) found:

| step | gripper qpos | body-to-body distance |
|---|---|---|
| 0 (jaw still open) | 1.7449 | 0.0299 m |
| 100 | 0.8876 | 0.0464 m |
| 120 | 0.6697 | 0.0501 m |
| 140 | 0.4506 | 0.0537 m |
| 160 | 0.2309 (closure gate now open, <0.3) | 0.0574 m |
| 299 (fully closed) | -0.1745 | 0.0795 m |

The distance grows monotonically, from 0.0299 m (well inside the 0.05 m gate, jaw fully
open) to 0.0795 m (jaw fully closed) -- and it crosses above 0.05 m (between step 120 and
140) BEFORE the closure gate opens (between step 140 and 160). The two gates' passing
windows do not overlap at these thresholds for this prop: by the time the jaw is closed
enough to attempt attach, the fixed-jaw body has already drifted too far from the object
to pass the proximity gate. **Root cause, not merely observed:** `ik.solve_position_ik`
(unmodified, out of scope) targets the `armX_gripperframe` SITE (the pinch point), not
the `armX_gripper` BODY `WeldGrasp`'s proximity gate reads (the naming-trap distinction
`grasp.py`'s own docstring names). As the jaw closes, the redundant 5-DOF solve keeps the
pinch-point SITE pinned at the grasp target by rotating the wrist -- and that same wrist
rotation carries the fixed-jaw BODY away from the site (and therefore away from the
object) at roughly 1.7 mm per closure step. This is the SAME site-vs-body divergence
ADR-029's own docstring already documents for a different maneuver (driving the pinch
point upward during LIFT); here it shows up during jaw CLOSURE instead. `pick(A, mug)`
fails earlier and for an unrelated, already-documented reason (`waypoint 1 (approach)
failed [convergence]` -- the pre-existing kinematic reach limit ADR-024/ADR-027 recorded).
`handoff(A, B, fork)` and `place(A, fork, table)` both fail as a direct, expected
consequence of the nested `pick` failing the same way (`weld_attach_failed_after_300_frames`
surfaces through `"handoff aborted: pick by arm A failed (...)"` /
`"place aborted: pick failed (...)"`), never reaching their own weld-specific gates
(`handoff`'s transfer check, `place`'s release-before-open ordering) in this run.

**Per this task's own instruction, this gap was NOT closed by loosening
`attempt_grasp`'s gates.** `distance_threshold_m`/`closure_threshold` were left at
`WeldGrasp`'s own defaults (0.05 m / 0.3 rad) exactly as ADR-029 designed and verified
them; `scripts/probe_weld_grasp.py`'s own positive-path verification of the fork used a
DIFFERENT technique (`_drive_gripper_body_to_target`, driving the gripper BODY directly)
than `skills_scripted.py`'s site-targeting `ik.solve_position_ik` path -- the divergence
between the mechanism's own verified test harness and the skill layer's actual IK-driving
pattern is this commit's real finding, not a wiring defect to be patched around by
loosening a threshold.

**`_hold_ctrl` drift (Commit 1) remains an open, compounding risk specifically for
`handoff`, not newly measured this session:** because no attach ever completed in this
run, `handoff`'s from-arm-idle-while-to-arm-moves window (where the drift matters most,
per Commit 1's own entry) was never actually reached with an object held. The risk stands
exactly as Commit 1 recorded it -- not re-measured, not resolved.

**Renders.** `docs/images/m06-phase2-fork-lifted.png` (after `pick(A, fork)`) and
`docs/images/m06-phase2-handoff-complete.png` (after `handoff(A, B, fork)`), both front
camera, 1280x720, both produced. **Neither shows what its filename claims, reported
plainly rather than implied:** both renders are visually near-identical -- arm A hovering
at clearance height above the STILL-RESTING fork (RETREAT ran after a failed GRIP, per
`run_pick`'s structure), arm B still at its rest pose off to the side (`handoff` aborted
inside the nested `from_arm` pick, before arm B's own APPROACH waypoint ever ran). At this
camera's distance the fork itself is a few pixels and not reliably distinguishable from
the tabletop by eye in either image -- this is stated here rather than left to imply a
visual confirmation neither render actually provides.

**Consequences.** Physical grasping stays abstracted (ADR-029) and, per ADR-015, the
README must disclose it -- **not done in this commit**: `README.md` is currently a
placeholder status doc owned by docs-writer per the agent assignment model
(`PLAN.md` section 2), and updating it is out of Builder's role; flagged here so it is
not silently dropped. `pick`/`place`/`handoff` are now wired end-to-end through the weld
abstraction and will complete successfully for a prop whose grasp geometry keeps the two
`WeldGrasp` gates' passing windows overlapping -- fork and water_bottle, as wired and
measured this session, do not; whether any prop's grasp offset can be retargeted to
produce an overlapping window (without touching `ik.py`/`grasp.py`) is an open follow-up,
not attempted here (out of this commit's scope: wiring, not re-tuning grasp geometry).

**Source commit:** `37b4e51` ("M06 Phase 2 Commit 2/2: weld wired into pick, place,
handoff. ADR-030."), found via `git log`/`.git/logs/HEAD` — commit message confirmed
to match this ADR title.

---

### ADR-031 — IK freeze during GRIP dwell to prevent shifting-pinch-point retreat

**Recorded:** Sept 13, 2026 · **Follows:** the "M06 Phase 2 follow-up" entry in
`DECISIONS.md` (Bug 2, pinch-point gate fix, which measured but did not chase the root
cause) · **Touches:** `src/bimanual/control/skills_scripted.py`'s `_dwell` only. `ik.py`,
`grasp.py`, `executor.py`, `scenes/so101/` are unchanged.

**Context.** The prior entry's own instrumentation measured, during
`pick(A, fork)`'s GRIP dwell, the pinch-point distance to the fork growing
0.0351 -> 0.0793 m and the gripper-body distance growing 0.0299 -> 0.0795 m
over the 300-step dwell -- both starting BELOW `WeldGrasp`'s 0.05 m proximity
gate and ending ABOVE it. Root cause, confirmed by reading `ik.py` directly
this session (not merely inferred): `ik.solve_position_ik` targets the PINCH
POINT (ADR-025's midpoint of the fixed and moving jaw BODIES), recomputed
fresh from the CURRENT qpos on every solve. As a GRIP dwell closes the jaw,
the moving jaw body's own position shifts, so the midpoint shifts even though
the dwell's target (`hold_pos`, a fixed point on the object) does not. The old
`_dwell` loop re-solved IK every step to keep that SHIFTING midpoint pinned on
the fixed target -- which means it kept commanding the ARM (not just the jaw)
to move so the midpoint would track the jaw's own closing motion, i.e. the
arm physically retreated as the jaws closed. Meanwhile the closure gate
(`armX_gripper` qpos < 0.3) needs about 150 steps to close. The two gates
therefore passed/failed on opposite ends of the dwell and never held true on
the same frame, so `weld.attempt_grasp` never returned `True` and
`weld_attach_frame` stayed `None`.

**Decision.** `_dwell` now captures the driven arm's own 5 positioning-joint
ctrl targets ONCE, before the dwell loop starts (reading `env.data.ctrl`,
i.e. wherever the preceding APPROACH/DESCEND waypoint already converged and
left the arm commanded), and reapplies that SAME frozen ctrl vector every
step for the rest of the dwell -- no `ik.solve_position_ik` call at all
inside the loop. Only the gripper joint's ctrl changes step to step, toward
`gripper_fraction`. This applies to EVERY call of `_dwell` -- GRIP dwells
(`run_pick`, `run_handoff`'s receiving-arm GRIP, `run_open_drawer`'s GRIP)
and RELEASE dwells (`run_place`, `run_handoff`'s releasing-arm RELEASE,
`run_open_drawer`'s RELEASE) alike, since all six route through the one
shared `_dwell` implementation and the shifting-pinch-point problem is
symmetric for an opening jaw. The idle-arm `_hold_ctrl` pattern (M06 Phase 2
Commit 1) is unchanged and is a DIFFERENT mechanism (it governs the arm NOT
being driven this call; ADR-031 freezes the arm that IS being driven, only
during a GRIP/RELEASE dwell specifically).

**Verification (bm-ptl), `pick(A, fork)`, seed=0, monkey-patched
`WeldGrasp.attempt_grasp` instrumentation logging pinch-point distance and
gripper qpos every 30 GRIP-dwell frames (scratch diagnostic, not shipped,
same technique as the prior entry's own measurement):
```
GRIP frame 1:   pinch_point_distance=0.0351 m  gripper_qpos=1.7449 rad
GRIP frame 30:  pinch_point_distance=0.0372 m  gripper_qpos=1.5992 rad
GRIP frame 60:  pinch_point_distance=0.0374 m  gripper_qpos=1.3215 rad
GRIP frame 90:  pinch_point_distance=0.0373 m  gripper_qpos=1.0065 rad
GRIP frame 120: pinch_point_distance=0.0373 m  gripper_qpos=0.6807 rad
GRIP frame 150: pinch_point_distance=0.0373 m  gripper_qpos=0.3519 rad
```
Distance now stays flat (~0.035-0.037 m, comfortably under the 0.05 m gate)
instead of growing to 0.0793 m, while qpos falls steadily as the jaw closes
-- direct evidence the freeze removed the retreat. Result:
`success=True frames_used=1655 reason="lifted fork: initial_z=0.3560
final_z=0.3989 ... weld_attach_frame=1155 weld_active_at_end=True"`,
`is_holding('A')=='fork'`. `mj_warnings={}`,
`max_joint_limit_violation=0.00039` (unchanged, negligible). This is the
first `pick`/`place`/`handoff` call in this project to attach a weld and
lift a prop past the WELD success threshold.

**Also run (bm-ptl, seed=0), each its own genuinely different outcome, not
chased further under this ADR's scope:**
- `pick(A, mug)`: `success=False`, fails at waypoint 1 (approach),
  `IK residual=0.0532 m` -- the pre-existing, already-documented arm-A
  kinematic reach limit to `mug_at_rest` (ADR-027/`m06-reachability-probe.md`),
  unrelated to GRIP dwell and unaffected by this fix (never reaches GRIP).
- `pick(A, water_bottle)`: `success=False`,
  `weld_attach_failed_after_300_frames`, `weld_attach_frame=None` -- reaches
  GRIP but the gate still never fires for this object/offset combination;
  bottle z fell 0.4400 -> 0.3684 during the dwell. A different, not-yet-
  diagnosed proximity gap, flagged as a follow-up, not chased under this
  session's scope (this ADR's job was the GRIP-freeze mechanism, verified
  above on the fork).
- `handoff(A->B, fork)`: `success=False`, fails at waypoint 3 (`to_arm`/arm B
  APPROACH), `IK residual=0.0875 m` -- `from_arm` (A) had already picked the
  fork successfully (its own nested `pick` succeeded, weld attached); the
  failure is arm B's own reach limit to `receiving_point`, a different
  kinematic gap from arm A's, surfaced only because arm A's GRIP now
  actually completes.
- `place(A, fork, destination=table)`: **`success=True`**,
  `frames_used=3455`, `weld_attach_frame=1155`,
  `weld_active_at_end=False` (released cleanly before jaw-open, per
  ADR-030's ordering), final position `x=-0.0081 y=0.0203 z=0.3588`
  (table-resting height, in-bounds).

**Render.** `docs/images/m06-fork-lifted.png` (front camera, end-of-`pick`
state, 1280x720). Reported honestly: at this camera's distance the fork is a
small, thin white object near arm A's jaw and the frame does not, by itself,
visually PROVE the lift the numeric z-delta already establishes -- it is
included as a supporting artifact, not as independent confirmation.
`docs/images/m06-handoff-complete.png` was NOT produced: `handoff` did not
succeed (arm B's own reach limit above), and rendering a failed handoff would
misrepresent it as the requested "complete" state.

**`pytest tests/test_skills.py` (bm-ptl), before and after this change:
identical, 4 passed / 4 failed, same four tests
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`),
same reasons** (plate: `weld_attach_failed_after_300_frames` at
`final_z=0.3524`, an ADR-024 grasp-reliability/offset gap specific to the
plate, not the pinch-point-retreat mechanism this ADR fixes; drawer/mug: the
same already-documented arm-A/arm-B kinematic reach limits). No shift in
which tests pass, in either direction -- the four tests this suite already
tracked as blocked by OTHER, separately-documented gaps remain blocked by
those same gaps; this fix unblocks `fork` specifically (not covered by
`test_skills.py`'s own four object choices) and is verified above via
`scripts/run_skill.py`/a scratch instrumentation script instead.

**Consequences.** `_dwell`'s per-step loop no longer calls
`ik.solve_position_ik` at all -- a real behavioural narrowing (the arm
cannot correct its OWN dwell-time drift by re-solving), justified because
the thing it was "correcting" toward was itself the source of the retreat.
The post-hoc `_validate_against_baseline` convergence check (run once, after
the dwell, by `_run_dwell`) is unaffected -- it still re-solves IK once to
report a residual for logging/validation purposes. This does not fix the
plate's or water_bottle's own separate grasp-reliability gaps, or either
arm's own kinematic reach limits -- those remain open, tracked by the ADRs
that already found them (ADR-024, ADR-027 and the prior entry below).

**Source commit:** `06c7917` (given).

---

### ADR-032 — Handoff-position re-measurement: NO collision-free shared point found in the specified sweep; NOT fixed, escalated instead of a constant change

**Recorded:** Sept 13, 2026 · **Follows:** ADR-031 (GRIP-dwell freeze, which made
`pick(A, fork)` succeed) · **Task:** relocate `HANDOFF_POSITION_XYZ` into the
measured shared reach envelope, per instruction to STOP and escalate rather than
guess if no candidate passes.

**Context.** `pick(A, fork)` succeeds; `handoff(A, B, fork)` gets through arm
A's pick and fails at waypoint 3 (`to_arm` APPROACH) -- arm B cannot reach the
current `HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.35)`. The existing comment on
that constant cites `m06-reachability-probe.md`'s "Shared handoff band: y in
[-0.12, 0.10]" line as justification, but that band was measured by
`probe_reachability.py`'s `run_envelope_sweep`, which (its own docstring says
so explicitly) checks IK residual convergence ONLY -- "collision not checked
for the sweep". A grid point can converge kinematically while the solved
configuration drives an arm segment through the table or a prop. That
caveat, not the band itself, is why this task re-measures instead of trusting
the existing constant.

**What was measured.** New script `scripts/probe_handoff_reachability.py`,
run on bm-ptl (ADR-020) and cross-checked byte-identical on this developer's
laptop (mujoco 3.2.7 both places): sweep x=0 (the constant's existing x),
y from -0.12 to 0.10 in 0.02 m steps (12 values), z in {0.35, 0.38, 0.40}.
For each of the 36 (y, z) points, IK is solved independently for arm A and
arm B (from the home-keyframe reset pose, ADR-026), and each arm's solved
joint configuration is applied to a scratch `MjData` and checked for any
NEW contact beyond that arm's measured reset-pose baseline (0 for both
arms) -- the exact same per-arm-independent residual+collision method
`probe_reachability.py`'s own primary probes use for their PASS bar,
copied (not imported) into the new script so it has no coupling to that
script's grid constants. Full table:
`docs/hardware/m06-handoff-reachability.md`.

**Result: ALL 36 candidates FAIL for at least one arm. NO (y, z) point in
the specified sweep passed for both arms.** This was not expected to be
uniform -- z=0.35 (exactly `TABLE_SURFACE_Z`) was flagged in advance as the
likeliest to fail, but z=0.38 and z=0.40 (3-5 cm clearance above the table)
failed identically. Inspecting the actual MuJoCo contacts for representative
FAIL rows (not merely trusting the boolean) confirms these are genuine,
non-trivial collisions, not a script artifact: e.g. arm A solved toward
(0.00, 0.06, 0.40) (residual 0.00926 m, well converged) produces
`armA_lower_arm`/`armA_wrist` vs. `table_top` contacts at up to -0.0237 m
penetration, plus contacts with the (stationary, unrelated) `mug` and
`fork` bodies at up to -0.031 m -- the solved arm literally swings through
the tabletop and through props resting nearby, not merely grazing. A
control check confirmed the machinery itself is not universally broken:
a known off-centerline target, (0.30, -0.05, 0.50), solved with residual
0.00848 m and **zero** new contacts for arm A -- so the collision check
correctly reports "no collision" when there genuinely is none; it is the
x=0 centerline candidates specifically, at this z band, that tunnel.

**Root cause, not fixed here (out of this task's permitted file list, and
already flagged as an existing, out-of-scope finding).** This is the same
`ik.py`/DLS-solver local-minimum behavior `m06-reachability-probe.md`'s own
"Step 4" section already documented for `plate_at_rest`/`mug_at_rest`/
`bottle_at_rest`: solving toward a point requires reaching centrally
across/over the table from the folded "home" seed, and the redundant 5-DOF
position-only solver (ADR-024) has no notion of the table's existence, so
it happily converges to a position-accurate configuration that gets there
by swinging the forearm through the table and through whatever sits on it,
rather than up and over. `ik.py` is on this task's do-not-touch list, and
fixing the solver (multi-start solving, an obstacle-aware cost term, or a
different seed) is exactly the kind of code change this task was not
scoped to make.

**Decision: do NOT change `HANDOFF_POSITION_XYZ`.** Per this task's own
explicit instruction ("If NO candidate passes for both arms, stop and
report the table... That would mean the two arms have no collision-free
shared workspace at any tested height... it would need an arm-placement
decision, not a constant change"), `skills_scripted.py` is left untouched
this commit. Picking any (y, z) from this sweep and writing it into the
constant anyway would repeat exactly the mistake this task was assigned to
fix: a plausible-looking constant that was never actually verified
collision-free.

**What this means for ADR-021.** ADR-021's original ~0.30 m reach / 0.50 m
base-gap layout assumption, already shown too optimistic once by ADR-026's
home-pose re-measurement and again by `probe_reachability.py`'s residual-only
band, is now superseded a third time: even the residual-only band's claimed
overlap does not survive a collision check at any of the three heights this
task specifies. Whether a collision-free shared point exists at some OTHER
(x, y, z) outside this specific sweep is not established either way by this
result -- only that none exists in the region this task was scoped to check.
Resolving this for real needs either (a) moving one or both arm bases
(ADR-021's own placement assumption) so a shared reach point exists further
from the table's central tunneling zone, or (b) an IK-solver fix (out of
`skills_scripted.py`'s scope) that avoids the table-tunneling local minimum.
Recommending, not deciding, per this task's own instruction that an
arm-placement change is a decision for the user, not this commit.

**Consequences.** `handoff(A, B, fork)` is NOT re-verified in this commit
(the task's VERIFY step is conditioned on a chosen position existing).
`docs/images/m06-handoff-complete.png` is not produced. `pytest
tests/test_skills.py` is unchanged by this commit (no source file changed)
-- the same 4 failed / 4 passed as before, `test_handoff_mug_ends_held_by_arm_b`
still failing for its own pre-existing, unrelated reason (arm A's `pick`
of the mug fails to converge, per that test's own captured output).

**Source commit:** `92acc32` ("M06: handoff position re-measured with collision
check; NO candidate passed for both arms in the specified sweep, escalated
instead of guessing (ADR-032)."), per this session's git log.

---

### ADR-032 (second pass) — Handoff-position sweep re-run with four seeding/collision corrections plus an extended-height grid; STILL no shared point, and the reach bands themselves do not overlap at x=0

**Recorded:** Sept 13, 2026 · **Follows:** the ADR-032 entry above (first
pass: home-seeded, residual+cross-arm-collision only, all 36 candidates FAIL) ·
**Task:** re-run the same sweep with four named corrections (C1-C4) plus two
additions, and relocate `HANDOFF_POSITION_XYZ` to a passing candidate if one
exists.

**What changed versus the first pass, and why each change was expected to
matter.**
- **C1 (grid targets the pinch point).** Unchanged in substance --
  `ik.solve_position_ik` already targets the pinch point (ADR-025), not the
  gripper body, in both passes. Made explicit this pass by also checking
  each solved configuration for a joint pinned at its `jnt_range` bound
  (margin < 1e-4 m), not merely residual convergence.
- **C2 (seed from the handoff-APPROACH pose, not home) -- the correction
  expected to matter most.** The first pass's own root-cause paragraph
  attributed the universal FAIL to every solve starting from the folded
  "home" pose, which lets `ik.py`'s redundant 5-DOF DLS solver fall into a
  table-tunneling local minimum when asked to reach centrally across the
  table. This pass stages each arm's solve exactly as
  `skills_scripted.run_handoff` itself does: solve HOME -> that arm's own
  APPROACH hover point (`CLEARANCE_HEIGHT_M` above the candidate, offset by
  `HANDOFF_SIDE_OFFSET_M` for the receiving arm), then -- from THAT
  resulting configuration, not home again -- solve -> the candidate itself.
  The residual gated on is this second, seeded solve's residual.
- **C3 (cross-arm collision, explicit threshold).** Both arms' seeded,
  converged configs applied SIMULTANEOUSLY via `mj_forward`; rejected if any
  cross-arm contact is deeper than -0.005 m.
- **C4 (one direction).** Swept only `from_arm=A, to_arm=B` (the commit
  gate, `handoff(A, B, fork)`); `handoff(B, A, fork)` is checked
  opportunistically by actually running the skill, not swept as a second
  grid (not reached this session -- see Consequences).
- **Addition 1 (kept, not dropped): arm-vs-world.** Each arm's own solved
  config applied ALONE (table_top AND every prop -- plate, mug, fork,
  spoon, water_bottle, drawer), same -0.005 m bar, so a seeded-but-still-
  tunneling candidate cannot pass merely because the OTHER arm's collision
  happened to be checked.
- **Addition 2: grid extended upward.** z in {0.35, 0.38, 0.40} (as
  specified) PLUS z in {0.44, 0.47} (extra rows), on the reasoning that
  z=0.35 IS `TABLE_SURFACE_Z` and a real handoff should happen in free space
  above it, not at or grazing the surface.

**What was measured.** `scripts/probe_handoff_reachability.py`, rewritten
for this pass, run on bm-ptl (ADR-020). Full 60-row table (12 y-values x 5
z-values, x=0 fixed):
`docs/hardware/m06-handoff-reachability.md`.

**Result: ALL 60 candidates FAIL, and `both_reachable` is False on EVERY
SINGLE row -- not merely the collision checks.** This is a stronger, more
precisely diagnosed non-go than the first pass, not a repeat of the same
ambiguous result: the seeding correction (C2) measurably did **not** move
armA's residual at all for most rows where it had previously failed badly
(e.g. `(0, -0.12, 0.35)`: 0.21540 m in BOTH the first pass and this one,
identical to 5 decimal places) -- because `CLEARANCE_HEIGHT_M` is only 0.08
m above the candidate, the seeded approach pose sits in the same
local-minimum basin as the candidate itself for targets the solver already
fails on from home. Reported honestly rather than claimed as a fix that
worked: **C2, applied exactly as instructed (a kinematic IK reseed, not a
physically-simulated pick-then-transfer), did not rescue any candidate this
session found.**

**A second, independent finding, confirmed by a direct control check (not
merely inferred from the sweep table): at x=0, the two arms' own
convergence bands do not overlap AT ALL, and each arm converges BETTER on
the side OPPOSITE its own base, not the side it is mounted on.** Measured
directly: `arm A -> (0, -0.20, 0.40)` (arm A's OWN side, base at y=-0.25):
residual 0.260, does not converge. `arm A -> (0, +0.20, 0.40)` (the far
side): residual 0.061, much closer (still not under the 0.01 m bar, but an
order of magnitude tighter). Arm B is the exact mirror
(`(0,+0.20,0.40)`=0.260, `(0,-0.20,0.40)`=0.061). A known-good off-
centerline control target, `(0.30, -0.05, 0.50)` for arm A / its mirror
`(-0.30, 0.05, 0.50)` for arm B, both converge cleanly (residual 0.00848 m
each) -- confirming the solver and the arm/geom lookups are not swapped or
broken; the crossed, non-overlapping reach pattern at x=0 is a real,
measured property of this scene's arm mounting, not a script defect. Given
this, at x=0 there is a wide dead band (roughly y in [-0.04, 0.10] for arm
A's failure side crossed with arm B's mirrored failure side) where NEITHER
arm converges well, and the two arms' respective "good" bands sit almost
entirely on each other's own base side -- the opposite of what a shared
midline transfer point needs.

**Decision: do NOT change `HANDOFF_POSITION_XYZ`.** Per this task's own
explicit stop condition ("If NOTHING passes even at z in {0.44, 0.47}, stop
and report... that would mean the arms have no collision-free shared
workspace at any tested height, which is an ADR-021 arm-placement decision,
not a constant change"), `skills_scripted.py` is left untouched by this
entry. The extended z rows (0.44, 0.47) do not change the verdict --
`both_reachable` fails identically at every height tested, so this is not a
height problem the way the first pass's own note speculated it might be;
it is an x=0 lateral-reach problem, orthogonal to z.

**Correction to this task's own pre-written framing.** The instruction text
supplied for this ADR entry asserts "ADR-021's assumed 0.10 m shared band
has... been superseded by measurement twice." The actual ledger in
`DECISIONS.md` is longer than that: ADR-026's home-pose re-measurement, then
`probe_reachability.py`'s residual-only envelope sweep, then the first
ADR-032 pass's collision-checked sweep, and now this second pass, have each
in turn found the shared band smaller or less real than the previous
measurement claimed -- more than two supersessions on the record, and this
entry is not the first to say so (the first ADR-032 entry above already
made the same correction, saying "a third time"). Restating the number
here as "twice" would understate the file's own history, so it is not
repeated as given.

**Consequences.** `handoff(A, B, fork)` is NOT re-verified this session --
no candidate exists to verify against, so the VERIFY step this task
specifies (is_holding checks, `docs/images/m06-handoff-complete.png`) is
not attempted; rendering a scene with no valid transfer point applied would
misrepresent a check that never ran. `handoff(B, A, fork)`'s opportunistic
check is likewise not run for the same reason (C4 gates it on a chosen
position that does not exist). `pytest tests/test_skills.py` is unchanged
by this entry (no source file changed): 4 passed / 4 failed, before and
after, identical to the first pass's own reported baseline, same four
tests, same reasons. This strengthens, not merely repeats, the
recommendation already on record: resolving this needs either (a) moving
one or both arm bases (ADR-021's own placement assumption -- now shown to
produce a crossed, non-overlapping reach pattern at the table's own
centerline, not merely an optimistic band), or (b) an IK-solver change (out
of `skills_scripted.py`'s scope) that does not depend on which basin the
seed pose happens to land in. Recommending, not deciding, per this task's
own instruction that an arm-placement change is a decision for the user.

**Handoff status, stated plainly for a reader who only reads this
paragraph.** As of this pass, `handoff` does not work: two independent
sweeps (36 rows, then 60 rows) both found zero mutually-reachable transfer
candidates for arm A -> arm B. A third, home-seeded sweep (36 rows,
`c9d7d82`) was later run for comparison and likewise found none. This is a
recorded negative finding, not a pending diagnostic with an expected
imminent fix — the next step is an arm-placement or IK-solver decision
outside this module's own scope, per the Consequences above.

**Source commit:** `ba76190` ("M06: handoff position via measured
mutual-reachability sweep (ADR-032)."), given/confirmed against this
session's git log; the home-seeded comparison sweep referenced above is
`c9d7d82` ("M06 handoff diagnostic: home-pose reachability sweep for
comparison."), a diagnostic-only commit, not a further ADR-032 pass.

---

### ADR-033 — `pick(A, water_bottle)`: per-prop APPROACH/RETREAT hover height for tall props, fixing a hover point that sat BELOW the bottle's own physical top

**Recorded:** Sept 13, 2026 · **Follows:** `docs/hardware/m06-water-bottle-diagnostic.md`
(commit `635a902`), which found the bottle displaced ~0.157 m in x / -0.061 m
in z from its reset position DURING (non-convergent) APPROACH/DESCEND,
entirely before GRIP starts, with ADR-031's GRIP-dwell freeze itself
confirmed working correctly (pinch point constant to 4 decimals across all
300 dwell frames) · **Task:** apply the smallest fix that makes
`pick(A, water_bottle)` succeed, choosing between (a) re-reading the
object's position at GRIP start or (b) not knocking it during approach.

**Branch chosen: (b), not (a) — and why (a) would not have worked at all,
not merely worked poorly.** `grasp.WeldGrasp.attempt_grasp`'s Gate 2
(proximity) already measures the pinch point against the object's OWN LIVE
`data.xpos` every call (`grasp.py:356-363`), not against any fixed
`grasp_point`/`hold_pos` value computed in `skills_scripted.py`. Re-reading
the bottle's position and recomputing `grasp_point` at GRIP start (option
a) would change ONLY the value logged/used for `_dwell`'s post-hoc IK
residual report — it is never used to re-aim the arm during the dwell
(ADR-031 freezes the arm's ctrl to wherever DESCEND already left it) and
never used by Gate 2 (which already reads the bottle live). So (a) is not
merely riskier here, as the task's framing anticipated — it is a no-op
against the actual failure: the arm's frozen GRIP-dwell pose is wherever
DESCEND converged to, and DESCEND converged near the bottle's ORIGINAL
resting spot while the bottle had already been knocked ~0.16 m away by
APPROACH's own motion, before GRIP or any re-read could matter.

**Root cause, found by measuring the scene geometry `run_pick` was already
targeting.** `scripts/gen_dual_scene.py`'s `water_bottle_cap` geom is
`pos="0 0 0.10" size="0.012 0.01"` sitting on `water_bottle_body`'s
`size="0.03 0.09"` cylinder — the cap's own top surface sits at local
z = 0.10 + 0.01 = 0.11 m above the body origin (reset z=0.44), i.e. world
z=0.55 m. The old `hover = grasp_point + CLEARANCE_HEIGHT_M` formula gave
hover.z = 0.46 + 0.08 = 0.54 m — **0.01 m BELOW the bottle's own physical
top**, not above it as "hover" is supposed to be. Every other pickable prop
(plate/mug/fork/spoon) has its `GRASP_POINT_OFFSET_M` sitting at or near its
own physical top already, so the same `CLEARANCE_HEIGHT_M` margin genuinely
clears them; only the bottle's grasp point (intentionally lower, near its
neck, partway down a ~0.20 m combined body+cap) leaves its own upper
structure un-cleared by the existing formula. This is consistent with (does
not contradict) the diagnostic's own finding that APPROACH/DESCEND both
report `converged=False` at their full 500-step budgets — a "hover" target
that is not actually clear of the object is exactly the kind of target that
can produce sustained, escalating contact during a redundant 5-DOF
incremental IK drive.

**Fix applied.** Added `OBJECT_TOP_LOCAL_Z_M` (`skills_scripted.py`), a
per-prop dict giving a prop's own physical top as a local z offset above its
body origin, currently populated only for `"bottle": 0.11` (the measured cap
top, from the scene geometry above). `run_pick`'s `hover` is now
`max(grasp_point.z, obj_pos0.z + OBJECT_TOP_LOCAL_Z_M.get(target_object,
offset.z)) + CLEARANCE_HEIGHT_M` instead of the old
`grasp_point.z + CLEARANCE_HEIGHT_M`. For every prop except the bottle,
`.get(..., offset.z)`'s fallback makes `obj_pos0.z + offset.z ==
grasp_point.z` exactly, so `max(...)` is a no-op and their hover height is
byte-for-byte unchanged — this is a per-prop, additive correction, not a
change to the shared formula or to `CLEARANCE_HEIGHT_M` itself. No change to
`grasp.py`, `ik.py`, `executor.py`, `scenes/so101/`, or `gen_dual_scene.py`
(the bottle's mass/geometry are read, not modified).

**Measured result (bm-ptl, `mujoco==3.2.7`, seed 0).** `pick(A,
water_bottle)`: bottle position at reset `(0.2200, 0.0000, 0.4400)`; at GRIP
start (post-DESCEND, reproduced via `run_pick`'s own `_run_waypoint`/
`_run_dwell` helpers) `(0.2372, 0.0440, 0.4404)` — displacement now ~0.017 m
in x / ~0.0004 m in z versus the old ~0.157 m / -0.061 m, a lateral nudge
during the redundant IK solve's approach, not a knock; weld attaches at
`attach_frame=1155` (whole-skill-call frame count via the real
`ScriptedSkillExecutor.execute`), `is_holding('A')=='water_bottle'`, final
z=0.6191 (initial 0.4400, success threshold 0.3700) — lifted 0.179 m.
`SkillResult.success=True`. `pytest tests/test_skills.py`: 4 failed / 4
passed before this change and 4 failed / 4 passed after (same four
pre-existing failures — `test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b` — all unrelated to the bottle and
unaffected by this change, confirmed by identical failure reasons/residuals
before and after).

**Not done.** No ADR-031 correction is needed — its GRIP freeze was already
confirmed working correctly by the referenced diagnostic and is untouched
here.

**Skills working as of this commit, stated plainly:** `pick(A, fork)`,
`place(A, fork, table)`, and `pick(A, water_bottle)`. `pick(A, mug)`,
`open_drawer`, and `handoff` do not work as of this commit.
`pytest tests/test_skills.py` is 4 passed / 4 failed.

**Source commit:** `d239a55` (given).

---

### ADR-034 — `place(A, water_bottle, table)` verification: `run_place` never checked whether the object it was told to place was already held, causing a redundant internal re-pick to target an unreachable height; fixed by skipping the nested pick when already held. A second, unrelated waypoint-1 reachability failure remains and is reported, not patched.

**Recorded:** Sept 13, 2026 · **Follows:** ADR-033 (`pick(A, water_bottle)`'s
per-prop hover fix, commit `d239a55`), which fixed `pick`'s own hover height
but was never exercised against `place`'s path (`run_place`'s destination
`approach_above_dest` uses the plain `TABLE_SURFACE_Z + CLEARANCE_HEIGHT_M`
formula, no per-prop term, `skills_scripted.py`) · **Task:** verify
`place(A, water_bottle, table)` by running `pick(A, water_bottle)` then
`place(A, water_bottle, table)` in the same episode (the destination string
`table_side` named in the brief is not implemented by `run_place` —
`destination != "table"` is rejected outright — so `"table"`, the value
every other working `place` call in this repo already uses, was used
instead).

**Verification result: place did NOT succeed on first attempt.** Probe:
`scripts/probe_place_bottle.py`, bm-ptl, `mujoco==3.2.7`, seed 0.

**First failure, before any code change.** `pick(A, water_bottle)` succeeded
(`weld_attach_frame=1155`, final pick z=0.6192, `is_holding('A')==
'water_bottle'`). The immediately-following `place(A, water_bottle, table)`
failed at `"waypoint 1 (approach) failed [convergence (IK residual=0.1639 m
>= 0.01 m)]"` — inside `run_place`'s own NESTED `run_pick` call, not `place`'s
own destination waypoints. Root cause: `run_place` calls `run_pick`
UNCONDITIONALLY every time, regardless of whether `arm` already holds
`target_object` — its own docstring already said "pick the object up (if
not already held)" but the code never implemented that conditional. With the
bottle already lifted and held at z=0.6192 (not resting on the table), the
nested `run_pick` re-read the bottle's CURRENT (airborne) position as
`obj_pos0` and, per ADR-033's `OBJECT_TOP_LOCAL_Z_M["bottle"]=0.11`,
recomputed a hover roughly 0.17 m higher still — a target the arm could not
kinematically reach in the 500-step waypoint budget, so `place` failed
before ever reaching its own destination logic, and the weld was never
released (`is_holding('A')` stayed `'water_bottle'`).

**This is NOT the ADR-033 failure mode, and ADR-033's `OBJECT_TOP_LOCAL_Z_M`
pattern does not address it.** ADR-033 fixed a hover point sitting BELOW a
STATIONARY object's own physical top during a fresh pick. Here the object
was already held and airborne; the defect is that `place` re-picks an object
it is already holding at all, not that any hover-height formula undershoots
the object's top. Applying ADR-033's pattern here would have been the wrong
fix — confirmed by tracing the actual failing waypoint (the nested pick's
APPROACH, not any of `place`'s own destination waypoints) before writing any
code.

**Fix applied (`skills_scripted.py`, `run_place` only).** Added a guard:
`already_held = weld is not None and weld.is_holding(arm) == body_name`. If
true, the nested `run_pick` call is skipped entirely and `place` proceeds
straight to its own destination waypoints with the object already in hand;
`weld_attach_frame` correctly reports `None` in this path (no new attach
happened during this `place` call, per that field's own documented meaning).
If `weld is None` or the object is not already held, behaviour is
byte-for-byte unchanged (the nested `run_pick` call still runs exactly as
before). No change to `grasp.py`, `ik.py`, `executor.py`,
`scenes/so101/`, or `gen_dual_scene.py`.

**Result after the fix: the first failure is gone, but `place` still does
not succeed — a second, different failure now surfaces, and it was NOT
patched.** Re-running the same probe: the nested-pick failure disappears
entirely (no more waypoint-1-inside-pick failure); `place` now fails at its
OWN `"waypoint 1 (approach destination) failed [convergence (IK
residual=0.0138 m >= 0.01 m)]"` — a plain kinematic IK-solver residual that
misses the 0.01 m tolerance by only 0.0038 m. Diagnosed before touching any
code (`scripts/probe_place_waypoint1_diag.py`): driving toward the same
target for a further 2000 steps (four times the normal 500-step waypoint
budget) does not shrink this residual — it plateaus, which rules out "just
needs more steps" and is consistent with a genuine reachability-envelope
edge, not a slow-convergence artifact. Independently, the destination x
computed for this run landed exactly on `run_place`'s own safety clip bound
(`dest_xy[0]` clipped to its `+0.30` limit), which is suggestive of the same
kind of arm-specific reachability-envelope boundary this repo has already
found and documented elsewhere (e.g. ADR-027's plate-rim direction fix,
ADR-032's handoff-position sweep) — but this was not independently
re-measured across other start positions, so it is reported as a plausible
explanation, not a proven one.

**Why this was not also fixed here.** `PLACE_OFFSET_XY_M`/the destination
clip bounds are shared by every prop's `place` call, not bottle-specific;
changing them to dodge one measured edge case, without re-verifying every
other prop's place path (none of which currently reach this waypoint at all
— `test_place_plate_returns_to_table_rest` fails earlier, at the nested
pick's own grip, per ADR-024's already-documented grasp-reliability gap) is
exactly the kind of speculative, unverified change the task instructions
say not to make. This is reported, not patched.

**Net honest status: `place(A, water_bottle, table)` still returns
`SkillResult.success=False`; `is_holding('A')` still ends as
`'water_bottle'`, not `None`; the bottle never reaches the table in this
run.** `pytest tests/test_skills.py`: 4 failed / 4 passed before this change
and 4 failed / 4 passed after (identical failure reasons/residuals for all
four pre-existing failures, confirmed line-by-line) — this fix changed no
existing test's outcome, it only changes what a NEW bottle-place probe
(not part of the pytest suite) reports.

**Not done.** No change to `ARCHITECTURE.md` (out of scope for this task, at
the time this ADR was recorded — synced retroactively in a later
documentation pass). No change to `ik.py`, `grasp.py`, `executor.py`,
`gen_dual_scene.py`, or `scenes/so101/`. No change to ADR-031's GRIP freeze
or ADR-033's pick hover. No speculative fix applied to the second
(waypoint-1-destination) reachability failure — it is left open and
reported here for a follow-up diagnostic pass, same as ADR-032's
handoff-position gap was left open rather than patched with an unverified
guess.

**Net result, stated plainly: `place(A, water_bottle, table)` does not
work.** The already-held bug this ADR fixed was real and is fixed, but the
skill as a whole still fails, now at its own destination-approach waypoint.

**Source commit:** `1a97153` ("M06: place(A, water_bottle, table)
verification — fixed a real bug (run_place unconditionally re-picked an
already-held object, sending it to an unreachable hover), one different
reachability failure remains and reported not patched (ADR-034)").

---

### ADR-035 — Target interpolation in the `handoff` traverse: implemented per the corrected brief; genuinely progresses the skill three waypoints further, but `handoff(A, B, fork)` still fails at a NEW waypoint (arm-vs-table_top collision during `to_arm`'s interpolated descend) — reported honestly, not patched

**Recorded:** Sept 13, 2026 · **Follows:** `2115a1e` (warm-start IK diagnostic,
`docs/hardware/m06-handoff-warmstart-diagnostic.md`) · **Modifies:**
`src/bimanual/control/skills_scripted.py` only, per task scope

**What was built.** The diagnostic's own pseudocode had a Zeno bug (re-reading
"current position" from the sim every loop iteration means the arm only ever
covers a shrinking fraction of the remaining distance and never arrives) —
fixed by capturing `start_pos` ONCE, before the loop, and interpolating
`start_pos + (end_pos - start_pos) * (i / n_steps)` for a fixed `i`
(`_run_interpolated_waypoint`). The diagnostic's chain also never started from
"wherever the arm happened to be" — it started from a pre-verified converged
staging cell in each arm's own reachable band and walked inward. Verified
directly (not assumed) that this matters for the real skill: `to_arm`'s
APPROACH waypoint, solved directly from HOME, measured IK residual 0.0875 m
(a clear fail) at (y=0.02, z=0.43 — the actual hover height `run_handoff`
uses, not the z=0.35 the diagnostic's own table covered). A fresh sweep this
session at z=0.43 (mirroring the diagnostic's z=0.35 sweep) found the SAME
disjoint-band shape at the actual hover height: arm A's home-converged band
starts at y=+0.06 (residual 0.00999), arm B's mirror at y=-0.06 (residual
0.00999) — recorded as `HANDOFF_STAGING_Y_M`. `_run_approach_with_staging`
tries the direct shot first (the common case, e.g. `from_arm`, which is
already warm from `pick`); only on failure does it drive to the arm's own
staging cell, confirm THAT converges, then interpolate onward
(`_run_interpolated_waypoint`) to the real APPROACH target. Both DESCEND
waypoints (from_arm to the transfer point, to_arm to the receiving point)
are unconditionally interpolated the same way, since the corrected brief's
item 3 confirmed both real targets (`HANDOFF_POSITION_XYZ`'s y=-0.01, and
y=-0.04/y=+0.02 with `HANDOFF_SIDE_OFFSET_M`) sit inside the diagnostic's
own verified y in [-0.06, +0.02] chained band. Joint-limit margin
(`HANDOFF_JOINT_LIMIT_MARGIN_TOL`, matching
`scripts/probe_handoff_reachability_home.py`'s own gate) is checked after
every interpolation step against the PHYSICALLY-REALIZED qpos, closing the
diagnostic's own explicitly-left-open caveat that its chain checked residual
only.

**Verification run: `handoff(A, B, fork)` (`to_arm=B, from_arm=A`), the
skill this task specified, not the pre-existing `test_handoff_mug_...` (which
fails for an unrelated, already-documented pick-reach reason and does not
exercise the traverse this ADR touches).** Per-waypoint outcome:

| waypoint | outcome |
|---|---|
| `pick(A, fork)` (nested) | succeeds, weld attaches (frames 1-655) |
| 1: from_arm (A) APPROACH, hover `(-0.065, 0.05, 0.44)` then re-target `(0, -0.01, 0.43)` | direct shot converges, residual 0.0099 — no staging needed (A already warm from `pick`) |
| 2: from_arm (A) DESCEND to transfer point `(0, -0.01, 0.35)`, interpolated | **all 4 interpolation steps converge**, residuals 0.0100/0.0082/0.0085/0.0083/0.0036 |
| 3: to_arm (B) APPROACH, hover `(0, 0.02, 0.43)` | direct shot FAILS (residual 0.0875, matching the fresh sweep) → staged to `(0, -0.06, 0.43)` (residual 0.0032) → **interpolated in 5 steps to `(0, 0.02, 0.43)`, ALL converge** (residuals 0.0032/0.0098/0.0076/0.0046) — **this is the fix working**: waypoint 3 now succeeds where it failed 100% of the time before this change |
| 4: to_arm (B) DESCEND to receiving point `(0, 0.02, 0.35)`, interpolated (3 steps) | step 1 converges (residual 0.0045); **step 2/3 (target `(0.0077, 0.0043, 0.351)`) fails — not a convergence failure, a COLLISION failure: a new armB-vs-`table_top` contact past `TABLE_COLLISION_DEPTH_TOL_M`, versus zero at baseline** |

**Net result: three additional waypoints now pass that failed 100% of the
time before this commit (waypoint 3 specifically, the one the diagnostic
targeted, now succeeds), but the skill call still does not reach the end —
it fails at a new waypoint the old, single-shot code never reached in the
first place.** This is exactly the caveat the diagnostic's own "what this
did NOT check" section flagged: "a static IK chain is not an executable
trajectory... real execution is exactly what tests this." The IK-chain
diagnostic proved *kinematic* reachability and (separately, in its Part 3)
found the arm-vs-table_top collision check itself fires on otherwise-sensible
converged poses, calling it a probable mesh-collision artifact needing
recalibration before it can gate anything — but recalibrating that check is
explicitly out of this commit's scope (touches `ADR-027`'s existing
collision-detection semantics, not target interpolation), so this ADR does
NOT patch around it. Whether the physical contact at
`(0.0077, 0.0043, 0.351)` — 1 mm above `TABLE_SURFACE_Z` — is a genuine
graze or the same mesh-collision-hull artifact Part 3 flagged is an open
question this ADR leaves open, honestly, rather than tuning
`TABLE_COLLISION_DEPTH_TOL_M` or the waypoint height to make it disappear.

**Per this task's explicit instruction ("If `handoff(A, B, fork)` fails:
STOP. Report the specific waypoint and failure mode. Do not attempt further
fixes"), no further iteration was attempted.** `handoff(B, A, fork)` (the
opportunistic mirror check) was NOT run — the 60-minute cap for this task
was already consumed by the investigation above establishing WHERE and WHY
the interpolation needed to start from a converged pose (the fresh z=0.43
sweep was not optional groundwork: without it, `HANDOFF_STAGING_Y_M` would
have been guessed, not measured). No `docs/images/m06-handoff-complete.png`
was rendered — the run did not succeed, and rendering a failed handoff would
misrepresent the outcome.

**Test suite: unchanged, as required.** `pytest tests/test_skills.py`
before and after this commit: **4 passed, 4 failed**, identical set
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`, `test_handoff_mug_ends_held_by_arm_b`
still fail, all for their own already-documented, unrelated reasons — the
mug test specifically still fails at `pick(A, mug)`'s own approach
convergence, never reaching this commit's code path at all). No regression;
per this task's own framing, this split was never expected to move, since no
test in the suite exercises `handoff`'s success path with a prop that
survives `pick`.

**Constraints honored:** only `skills_scripted.py` and this file were
modified. `grasp.py`, `ik.py`, `executor.py`, `gen_dual_scene.py`,
`scenes/so101/` and `ARCHITECTURE.md` are untouched (this last is now synced
retroactively by this documentation pass). ADR-031's GRIP freeze, ADR-033's
per-prop hover and ADR-034's already-held guard are all unmodified (confirmed
by diff — this commit only adds new functions/constants and replaces the
direct `_run_waypoint` calls at handoff's own waypoints 1-4 with the
staged/interpolated equivalents).

**Net result, stated plainly: `handoff(A, B, fork)` does not work.**
Interpolation genuinely carried the skill past a waypoint it previously
failed 100% of the time (waypoint 3), but the call now fails one waypoint
later, at waypoint 4, on an arm-vs-`table_top` collision during `to_arm`'s
interpolated descend — a different failure mode than the reachability gap
this ADR targeted, and it was not patched.

**Source commit:** `f521775` ("M06 handoff: target interpolation in traverse
implemented (ADR-035, based on 2115a1e warm-start diagnostic) — fixes
waypoint 3's convergence as diagnosed, but handoff(A, B, fork) still fails at
a NEW waypoint (arm-vs-table_top collision on to_arm's interpolated
descend), reported not patched.").

---

### ADR-036 — Raise `HANDOFF_POSITION_XYZ`'s z above the table (0.35 -> 0.43) to remove the ADR-035 arm-vs-table_top collision: the targeted collision is gone, but `handoff(A, B, fork)` now fails one waypoint EARLIER, at a NEW cross-arm collision — the fix relocated the failure rather than resolving it, reported honestly, not patched

**Recorded:** Sept 13, 2026 · **Follows:** ADR-035 (target interpolation in
the `handoff` traverse, commit `f521775`) · **Modifies:**
`src/bimanual/control/skills_scripted.py` only, per task scope

**Diagnosis given, verified before changing anything.** ADR-035's own trace
showed waypoint 4 (`to_arm`'s DESCEND) failing its 2nd/3rd interpolation
step at target z=0.351 with a new `armB`-vs-`table_top` contact past
`TABLE_COLLISION_DEPTH_TOL_M`. `HANDOFF_POSITION_XYZ = (0.0, -0.01, 0.35)`
and `TABLE_SURFACE_Z = 0.35` are indeed the same number, confirmed by
re-reading both constants (`skills_scripted.py`) before making any change —
the transfer point sat exactly at tabletop height, and DESCEND was asking
`to_arm` to put its wrist there.

**Fix applied.** `HANDOFF_POSITION_XYZ`'s z: 0.35 -> 0.43 (`TABLE_SURFACE_Z +
CLEARANCE_HEIGHT_M`) — the same height ADR-035's own fresh sweep already
found as a converged, joint-limit-clean staging cell for BOTH arms
(`HANDOFF_STAGING_Y_M`'s docstring), and the height waypoints 1/3 (APPROACH)
were already driving to as `transfer_point + CLEARANCE_HEIGHT_M` before this
change. Chosen over a fresh number specifically so the APPROACH waypoints'
numeric targets would not change at all — only the now-redundant DESCEND
step down to table height (the one that collided) would be removed. Checked
per the task's own warning before concluding anything: with the transfer
point now AT the old hover height, the separate hover-clearance term used to
compute each APPROACH's target is gone (transfer_point IS the hover height
now), so the retreat waypoints (5/6, formerly 7/8) are the only ones now
targeting a genuinely new, previously-untested height
(`0.43 + CLEARANCE_HEIGHT_M = 0.51`) — flagged explicitly in the code
comments, not assumed safe.

**DESCEND waypoints removed, not left as no-ops.** With `HANDOFF_POSITION_XYZ`
raised to the hover height, the old DESCEND targets (`transfer_point` /
`receiving_point` at z=0.35) became numerically identical to the preceding
APPROACH targets (z=0.43 both now) — i.e. start == end, zero distance to
interpolate. Per this task's own instruction ("removing them is reasonable
— but say plainly that you removed them and why, rather than leaving dead
waypoints that report success without moving"), the two DESCEND calls
(`_run_interpolated_waypoint` invocations, old waypoints 2 and 4) were
deleted outright and the skill's remaining waypoints renumbered 1-6 (was
1-8) throughout `run_handoff`'s docstring and its `SkillResult` failure
messages.

**Verification run: `handoff(A, B, fork)` (`to_arm=B, from_arm=A`), via
`scripts/probe_handoff_fork.py` (new, read-only, same
`ScriptedSkillExecutor`/`SkillCall` path `tests/test_skills.py` and
`scripts/probe_place_bottle.py` use), bm-ptl, seed 0, BEFORE and AFTER this
change:**

| waypoint (renumbered after this change) | BEFORE this change (probe run) | AFTER this change (probe run) |
|---|---|---|
| `pick(A, fork)` (nested) | succeeds, weld attaches | succeeds, weld attaches (unaffected) |
| 1: `from_arm` (A) APPROACH to transfer point | converges (same numeric target both runs — unaffected by this diff) | converges |
| old waypoint 2: `from_arm` (A) DESCEND to table-height transfer point | ran and converged in this probe (table-height transfer point still existed) | *(removed outright — no longer exists; see above)* |
| old waypoint 3 / **new waypoint 2: `to_arm` (B) APPROACH to receiving point** | direct shot fails (residual 0.0875), staged to y=-0.06 SUCCEEDS (residual 0.0032), interpolates onward, all steps converge | direct shot fails (residual 0.0875, identical — unaffected by this change), staging to y=-0.06 now **FAILS with a NEW `cross_arm` collision (contacts=1 vs baseline 0)** |
| old waypoint 4: `to_arm` (B) DESCEND to receiving point | reaches this waypoint, fails there with the ADR-035 `armB`-vs-`table_top` collision this task set out to fix | *(never reached in this run — skill now fails earlier, at new waypoint 2)* |

**Net result: the diagnosed arm-vs-table_top collision at the old waypoint 4
is confirmed gone — but `handoff(A, B, fork)` still does not complete.** It
now fails one waypoint EARLIER than before (at the renumbered waypoint 2,
`to_arm`'s APPROACH/staging), with a DIFFERENT failure kind: not a
convergence failure, not a table collision, but a `cross_arm` contact
between the two arms' own collision geometry. Root cause, confirmed by
comparing to ADR-035's own trace rather than assumed: before this change,
`from_arm` (A) had already DESCENDED away from the shared 0.43 m hover band
down to table height (0.35) by the time `to_arm` (B) staged into that band —
the two arms were never in the same z-plane at the same time. This change
removed that DESCEND, so `from_arm` now sits AT the transfer point (0.43)
holding the fork for the entire remainder of the skill, including while
`to_arm` drives its own staging move through the SAME 0.43 m plane at
y=-0.06 — and the two arms' geometry now intersects. **This is exactly the
"relocate the collision" outcome the task instructions warned to check for
explicitly, and it is what happened**, not the table collision resolving
cleanly.

**Also confirmed, per the task's own instruction to check rather than
assume:** the retreat waypoints' new height (0.51 m, `CLEARANCE_HEIGHT_M`
above the raised transfer point) was never reached in this run — the skill
fails at waypoint 2, three waypoints before retreat — so this change
neither confirms nor refutes reachability at 0.51 m; that remains untested.

**Regression check.** `pytest tests/test_skills.py`: 4 passed / 4 failed,
identical split and identical failure reasons to the pre-change baseline
(the one handoff test in the suite, `test_handoff_mug_ends_held_by_arm_b`,
still fails at the SAME earlier point, inside the nested `pick(A, mug)` call,
for the unrelated, already-documented reason ADR-035/the test file's own
docstring records — untouched by this change, since `run_pick` is not
modified here). No change to `grasp.py`, `ik.py`, `executor.py`,
`gen_dual_scene.py`, `scenes/so101/`, or `ARCHITECTURE.md`.

**Not patched further, per this task's hard time cap and explicit
instruction to stop and report rather than iterate.** A candidate next fix
(stagger `from_arm`'s retreat to happen BEFORE `to_arm`'s APPROACH, rather
than after `to_arm`'s GRIP/RELEASE as the skill currently orders things) is
visible from this trace but has not been attempted or verified — recording
it here as an unverified idea, not a recommendation, since the corrected
brief for THIS task specified raising `HANDOFF_POSITION_XYZ`, not
reordering the traverse. `handoff(A, B, fork)` remains unverified working
end to end after two consecutive fix attempts (ADR-035, ADR-036); no
`docs/images/m06-handoff-complete.png` was generated, since the task's own
render step was conditioned on success. `handoff(B, A, fork)` was not
attempted, since the same to_arm-approach failure this table diagnoses is
symmetric in `from_arm`/`to_arm` roles and not expected to behave
differently, and the task did not require it when the primary direction
fails.

**Source commit:** `f92806e` (given).

---

### ADR-037 — `run_handoff` rewritten as sequential choreography (one arm moves at a time, the other genuinely frozen); the fix for the `_hold_ctrl` drift bug turned out to ALSO resolve the ADR-036/`f92806e` cross-arm collision — `handoff(A, B, fork)` now succeeds end to end, verified by direct measurement, not assumed

**Recorded:** Sept 14, 2026 · **Follows:** ADR-036 (`f92806e`, cross-arm
collision at `to_arm`'s staging sweep) · **Modifies:**
`src/bimanual/control/skills_scripted.py`, `scripts/requirements-bmptl.txt`,
this file, per task scope · **References (user-supplied, cited as such --
not fetched, not described beyond the quoted phrases the task itself gave):**
Wan, Ramos, Yang, Garrett 2025 (NVIDIA), "Learning to Plan & Schedule with
Reinforcement-Learned Bimanual Robot Skills",
https://arxiv.org/html/2510.25634v1 -- "single-arm waiting skill that keeps
one arm stationary"; and "Trajectory planning system for bimanual robots:
Achieving efficient collision-free manipulation" (2025),
https://www.sciencedirect.com/science/article/pii/S0921889025002155.

### Part 0 -- reverting the mink probe's environment damage, gated first

`9d0adde`'s mink probe silently bumped bm-ptl's shared `ov_env` from
`mujoco==3.2.7` to `3.13.0` as a forced transitive consequence of
`pip install mink==1.3.0`. mink was NOT adopted (that probe's own verdict:
on the identical target from the identical "home" pose, mink's QP-based
velocity IK converged to a 0.2233 m residual, stuck at a joint-limit local
minimum, where this project's own `ik.solve_position_ik` (DLS) converges to
0.00986 m; mink's `CollisionAvoidanceLimit` also allowed a measured ~5.5 cm
arm-vs-table interpenetration during that same stuck solve). With mink not
adopted, there was no remaining reason to carry its forced mujoco floor.

**Actions taken, in order, each verified before proceeding:**
1. `pip install mujoco==3.2.7` on bm-ptl (`ov_env`) -- confirmed via
   `python -c "import mujoco; print(mujoco.__version__)"` -> `3.2.7`.
2. `pip uninstall -y mink` -- confirmed removed via `pip show mink`
   (raises "not found").
3. `scripts/requirements-bmptl.txt` updated to match: `mujoco==3.2.7`
   restored, the `mink==1.3.0` line removed, with a comment explaining the
   revert and pointing at `docs/hardware/m06-mink-probe.md` for the
   evaluation record.
4. `pytest tests/test_skills.py` re-run under 3.2.7 BEFORE any code change
   in this commit: **4 passed, 4 failed** -- identical split, identical
   failure reasons, to every previously-documented baseline (ADR-036,
   `m06-mink-probe.md`'s own 3.13.0 re-run). This is necessary but NOT
   sufficient evidence (see point 5): no test in this suite exercises
   `pick(A, fork)`, `place(A, fork, table)`, or `pick(A, water_bottle)` with
   a real `WeldGrasp` -- that is exactly why the mink probe's "pytest
   unchanged" claim did not, by itself, establish that reverting mujoco
   would be safe either.
5. **The three working skills re-run directly** (via the real
   `ScriptedSkillExecutor` + `SkillCall` path, `WeldGrasp` attached, seed 0),
   BEFORE any code change in this commit, and their measured numbers
   compared byte-for-byte against the pre-mink-probe baselines already on
   record (ADR-031, ADR-033/034):

   | Skill | Measured under mujoco 3.2.7 (this commit) | Matches prior baseline? |
   |---|---|---|
   | `pick(A, fork)` | `success=True frames_used=1655 weld_attach_frame=1155 initial_z=0.3560 final_z=0.3989` | YES, identical (ADR-031) |
   | `place(A, fork, table)` | `success=True frames_used=3455 weld_attach_frame=1155 final_xyz=[-0.0081, 0.0203, 0.3588]` | YES, identical (ADR-031) |
   | `pick(A, water_bottle)` | `success=True frames_used=1655 weld_attach_frame=1155 initial_z=0.4400 final_z=0.6192` | YES, identical (ADR-034) |

   No regression. Only after this did any change to `skills_scripted.py`
   begin.

### Part 1 -- the `_hold_ctrl` drift bug: measured, then fixed

**Measured, per this task's correction, before touching anything:**
`_hold_ctrl` rebuilt its ENTIRE returned ctrl vector from the idle arm's
CURRENT qpos every single call, so it commanded zero position error at the
instant of each read and supplied no restoring force against gravity
between reads -- the idle arm's true setpoint ratchets away from its
original pose. This was already flagged, unfixed, in the module's own
docstring (measured previously at 0.04 rad by 100 steps, 0.546 rad
saturation -- a joint hard-limit stop -- by ~1500 steps). A single `pick`
alone runs ~655 frames (ADR-035's own trace), implying ~0.26 rad of drift
before choreography is even involved -- 26x a naive 0.01 rad bar.

**Fix.** `_hold_ctrl(env, frozen_base=None)`: if `frozen_base` is given, it
is returned as a plain copy -- nothing is re-derived from live qpos. A
caller doing a genuine long-duration idle hold captures a snapshot ONCE
(a plain `_hold_ctrl(env)` call, no override -- this still reads live qpos,
but only that one time) at the exact instant an arm becomes idle, and
passes that SAME array back in as `frozen_base` on every subsequent step of
the idle span, however many waypoint/dwell calls that span covers. This is
the same insight as ADR-031's GRIP-dwell freeze, generalized from "freeze
the ACTIVE arm for one dwell" to "freeze the IDLE arm for an entire
choreography phase." `frozen_base` was threaded as a new, purely additive
parameter through `_drive_to_target`, `_dwell`, `_run_waypoint`,
`_run_dwell`, `_run_interpolated_waypoint`, `_run_approach_with_staging`,
and `run_pick` (needed so `run_handoff`'s Phase 1 can freeze `to_arm` for
the WHOLE nested pick call, not just one waypoint of it) -- every one of
these defaults the new parameter to `None`, which reproduces the exact
pre-ADR-037 behaviour. `run_place` and `run_open_drawer` were NOT given new
call sites using this parameter (out of this task's scope; their own idle
holds are unchanged).

**Verification that this did not regress `pick`/`place` (which also use
`_hold_ctrl`):** the same three-skill re-run from Part 0's step 5, re-run
again AFTER this change (still with `hold_ctrl_base` left at its default
`None` for these standalone calls): **byte-identical** to Part 0's
just-recorded numbers (`pick(A,fork)`: 1655 frames, `weld_attach_frame=1155`,
`final_z=0.3989`; `place`: 3455 frames, same weld frame, same final xyz;
`pick(A,water_bottle)`: 1655 frames, `final_z=0.6192`). `pytest
tests/test_skills.py`: unchanged, 4 passed / 4 failed, identical reasons.

**Measured drift with the fix applied, over the actual choreography phases
(not a synthetic long dwell) -- this is the evidence the test threshold
below is based on**, via a real `handoff(A, B, fork)` run instrumented to
read each frozen arm's joint qpos at phase boundaries:

| Phase | Frozen arm | Frames this phase | Max joint drift from its frozen pose |
|---|---|---|---|
| 1 (`from_arm` picks) | `to_arm` (B), held at HOME | 1655 | **0.000774 rad** |
| 2 (`from_arm` approaches transfer point) | `to_arm` (B), SAME snapshot as phase 1 | 500 | **0.000774 rad** (unchanged -- confirms the snapshot itself is not decaying) |
| 3 (`to_arm` approaches receiving point) | `from_arm` (A), held at the transfer point | 3000 | **0.000237 rad** |

Both are more than an order of magnitude under the task's own proposed
0.01 rad bar, not merely under it -- so **0.01 rad is adopted as the test
threshold**, now with real evidence behind it (this was NOT achievable
under the OLD `_hold_ctrl`, where phase 1 alone would have implied ~0.26
rad; it IS achievable under the fix, measured directly, with roughly 13-40x
margin).

### Part 2 -- the cross-arm collision (`f92806e`'s finding): investigated, NOT solved by new routing geometry, but resolved anyway as a side effect of Part 1

**The brief's staging signs, corrected per measurement (unchanged from
ADR-035):** `HANDOFF_STAGING_Y_M = {"A": 0.06, "B": -0.06}` -- each arm
converges on the side OPPOSITE its own base, confirmed again this session,
not re-guessed.

**The hard part: applying those measured values puts `to_arm`'s (B's) own
staging cell on the SAME side of the midline `from_arm` (A) occupies while
parked at the transfer point.** Two routing alternatives from the task's
own list were tried, measured, and NOT adopted:

- **Elevated ("dodge") crossing** -- stage `to_arm` at a taller z, sweep
  laterally clear of `from_arm`'s operating height, then descend.
  Measured (`WeldGrasp`-backed, real closed-loop drives, not a one-shot IK
  check): the lateral sweep at z=0.53 converges collision-free in some
  runs, but sits on a reproducible JOINT-LIMIT KNIFE-EDGE (margin as small
  as -0.000001 rad -- flips pass/fail on essentially no perturbation:
  the SAME (0,-0.06,0.53)->(0,0.02,0.53) move measured `ok=True,
  cross_arm=0` in one run and `ok=False` at joint-limit margin -0.000001 in
  another run that differed only in `from_arm`'s parked height). The
  DESCEND back down to any real receiving height also reliably hit a
  genuine joint-limit wall around z~0.49-0.50 regardless of which elevated
  height was dodged to (tested 0.48/0.50/0.53/0.55/0.60/0.65/0.70).
  Rejected: not robust enough to ship in place of the one corridor already
  proven kinematically solid (ADR-035's lateral crossing at z=0.43).
- **Horizontal ("dodge-in-x") crossing** -- sweep `to_arm` out to
  x=+-0.15/+-0.20 before crossing y, then back. Measured: every variant ran
  out of step budget (500-1000 steps/hop, well past ordinary convergence
  time) before finishing. Inconclusive, not adopted as a positive result.
- **Retracting `from_arm`'s elbow while holding its pinch point fixed**
  (the task's third suggested option) was not attempted: `ik.py` is
  out of scope for this task, and its 5-DOF null-space is not otherwise
  exposed to a caller in this module.

**Given neither alternative was robust, Phase 3 uses the SAME
`HANDOFF_STAGING_Y_M`/`_run_approach_with_staging` lateral corridor
ADR-035 already verified -- i.e. this ADR did NOT change the routing
geometry `f92806e` found colliding.** The expectation, going into
verification, was therefore that Phase 3 would reproduce the SAME
cross-arm collision `f92806e` measured.

**That expectation was wrong, and the reason is instructive.** Verified
directly: `_contact_counts(env)['cross_arm']` is **0 both immediately
before and immediately after Phase 3's `to_arm` approach**, in the actual
`run_handoff` call (not a hand-assembled replay). The likely explanation,
consistent with Part 1's own measurement: under the OLD `_hold_ctrl`,
`from_arm` was the IDLE arm throughout the entirety of Phase 3 (up to 3000
frames in this run), and the OLD mechanism let it sag under gravity with NO
restoring force -- at the measured rate (0.04 rad/100 steps, saturating at
a hard joint limit by ~1500 steps), a 3000-frame idle span would have let
`from_arm`'s actual, physically-simulated arm collapse toward a mechanical
stop, unpredictably changing its real collision geometry from the clean,
fully-extended pose it converged to. The NEW frozen hold keeps `from_arm`
rigidly at exactly the pose it arrived at (holding the fork at the
transfer point), which -- combined with the SAME staging geometry that
previously collided -- turns out to be collision-free. **This is reported
as a genuine, directly-measured finding, not assumed or extrapolated: the
drift fix (Part 1) and the collision fix (Part 2) turned out to be the
SAME fix**, which was not anticipated going in.

### Part 3 -- simultaneous welds on one body during the transfer moment: tested explicitly

**Concern (this task's own):** Phase 4 has `to_arm`'s weld attach and get
verified BEFORE `from_arm`'s weld is released, so both
`from_arm`-vs-`fork` and `to_arm`-vs-`fork` weld equalities could be
active at once, forming a closed kinematic chain through the fork.

**Measured directly** (a manual replay of `run_handoff`'s own Phase
1-4 sequence, instrumented to inspect `env.data` at the exact frame attach
flips true, BEFORE the shipped code's own `weld.release(from_arm)` call):
at that instant, `weld.is_holding('A') == 'fork'` AND
`weld.is_holding('B') == 'fork'` are BOTH true simultaneously --
confirming the two-weld state is real, not merely theoretical.
`qfrc_constraint` norm at that instant: **4.214480** (finite, not
exploding). An EXPLORATORY extra physics step (deliberately run with BOTH
welds still active, NOT part of the shipped code path) produced a max
`|qpos delta|` of **0.010983** over that one step (small, not a jump/
explosion), `qfrc_constraint` norm unchanged to 6 decimal places, and
`env.data.warning.number` all zero (no MuJoCo warnings). **Conclusion:
MuJoCo appears to handle this configuration without instability, at least
for one step** -- but this is NOT relied upon: by code inspection, the
shipped `run_handoff` calls `weld.release(from_arm)` immediately after the
Phase 4a gate, with NO intervening `env.step()` call, so in the actual
production path the two-weld state exists only within `WeldGrasp`'s own
bookkeeping for a moment, never across an actual physics integration step.

### VERIFY -- phase-isolated and full-handoff results

All runs: seed 0, real `ScriptedSkillExecutor`-equivalent path
(`run_pick`/`run_handoff` called directly with a real `WeldGrasp(env)`,
matching what `ScriptedSkillExecutor._dispatch` does).

| Check | Result |
|---|---|
| Phase 1 (`from_arm` picks fork; `to_arm` frozen at HOME) | `ok=True`, frames=1655, `to_arm` drift=0.000774 rad (< 0.01 rad) |
| Phase 2 (`from_arm` approaches transfer point; `to_arm` still frozen) | `ok=True`, frames=500, `to_arm` drift=0.000774 rad (unchanged) |
| Phase 3 (`to_arm` approaches receiving point; `from_arm` frozen at transfer point) | `ok=True`, frames=3000, `from_arm` drift=0.000237 rad (< 0.01 rad), **cross_arm contacts: 0 before, 0 after** |
| Full `handoff(A, B, fork)` | **`success=True`**, `frames_used=6610`, `reason="held by arm B: z=0.4674 (initial 0.3560) dist_to_armA=0.0909 dist_to_armB=0.0582 (to_arm=B) weld_holding_to_arm=True weld_holding_from_arm=False from_arm_retreat_dist=0.1332 from_arm_clear=True"`, `weld_attach_frame=5310`, `is_holding('A')=None`, `is_holding('B')=='fork'` |
| `handoff(B, A, fork)` (opportunistic mirror, reported per this task, NOT gated) | `success=False`, fails at Phase 1: `pick(B, fork)`'s own APPROACH does not converge (`IK residual=0.0954 m`) -- a PRE-EXISTING, already-documented kinematic reach limit of arm B's own base placement to this fork position (unrelated to the choreography change; arm B has never been shown able to pick this fork from its own approach angle) |

Success condition (`is_holding(B)=='fork'` AND `is_holding(A) is None` AND
arm A clear of the shared workspace at the end) is met:
`from_arm_retreat_dist=0.1332 m > HANDOFF_RETREAT_GATE_M=0.10 m`.

`HANDOFF_POSITION_XYZ` is unchanged from ADR-036 (`(0.0, -0.01, 0.43)`).
New constants added: `HANDOFF_FROM_ARM_RETREAT_CLEARANCE_M = 0.12` (so
`from_arm`'s first retreat waypoint clears the 0.10 m gate -- a plain
`CLEARANCE_HEIGHT_M=0.08` lift alone is only 0.08 m of total displacement,
short of the requirement) and `HANDOFF_RETREAT_GATE_M = 0.10`, both
verified reachable/effective by this run, not assumed.

**Render.** `docs/images/m06-handoff-complete.png` -- front camera,
cropped to the table region, 1280x720 source. **Honest framing note, per
this task's own instruction:** the crop shows both arms near the transfer
point; arm B's jaw holds a small, thin white sliver (the fork) that is easy
to miss at this resolution, and arm A's retreat (`from_arm_retreat_dist`
=0.1332 m) is mostly VERTICAL, which a horizontal front camera does not
render as an obvious lateral separation between the two arms -- the
numeric state (`is_holding('A')=None`, `is_holding('B')=='fork'`,
`from_arm_clear=True`) is the reliable evidence; the image is a supporting
artifact, not independent visual proof, exactly as ADR-031's own render
note already cautioned for `m06-fork-lifted.png`.

**Constraints honored.** Only `skills_scripted.py`,
`scripts/requirements-bmptl.txt`, and this file were modified. `grasp.py`,
`ik.py`, `executor.py`, `gen_dual_scene.py`, `scenes/so101/`, and
`ARCHITECTURE.md` are untouched (confirmed by diff). ADR-031's GRIP freeze,
ADR-033's per-prop hover, and ADR-034's already-held guard are unmodified.
`pytest tests/test_skills.py`: 4 passed / 4 failed before and after this
entire commit, identical failure reasons throughout -- no regression.

**Source commit:** `311430e` (given).

---

### ADR-038 — Handoff render legibility: red fork and lateral retreat ADOPTED; prop repositioning TRIED AND FULLY REVERTED because every prop move broke `handoff` at phase 3

**Recorded:** Sept 14, 2026 · **Follows:** ADR-037 (`311430e`, sequential
choreography) · **Modifies:** `gen_dual_scene.py` (fork colour only),
`skills_scripted.py` (retreat vectors only), `scripts/render_handoff_frames.py`

**Problem.** Four successive renders of the working `handoff(A, B, fork)` failed
to show a handoff. Three root causes were identified by inspecting the images
rather than guessing: (1) the fork was silver-grey (`0.72 0.73 0.76`),
indistinguishable from the off-white plate and the identically-coloured spoon;
(2) the retreat displaced only z, so there was no lateral separation for any
camera to show — measured 0.0197 m lateral against 0.1060 m vertical; (3) the
sequence strip re-centred its camera per panel, destroying the viewer's frame
of reference.

**Fix 1 — red fork (ADOPTED).** `fork_material` → `rgba="1.0 0.15 0.15 1.0"`.
Colour only; geometry, mass and collision untouched, so no skill behaviour
depends on it. `spoon_material` deliberately left silver so only the fork
stands out. This alone made the fork unmistakable in all three candidate
renders.

**Fix 2 — lateral retreat (ADOPTED at 0.10, NOT the proposed 0.15).** The
scalar z-only `HANDOFF_FROM_ARM_RETREAT_CLEARANCE_M` became per-arm vectors
`HANDOFF_A_RETREAT_XYZ = (0.0, -0.10, 0.15)` / `HANDOFF_B_RETREAT_XYZ =
(0.0, 0.10, 0.15)`. The proposed 0.15 **fails**: `handoff` reaches phase 5
holding the fork and then collides cross-arm during `from_arm`'s retreat. A
sweep of the lateral magnitude, all with props at their original positions:

| lateral | handoff | final lateral separation |
|---:|---|---:|
| 0.15 | FAIL — cross-arm collision, phase 5 | — |
| **0.10** | **PASS** | **0.1946 m** |
| 0.05 | PASS | 0.0913 m |
| 0.00 | FAIL — cross-arm collision, phase 5 | — |

0.10 is a genuine interior optimum — it fails on BOTH sides, so it was found by
measurement, not by picking the largest value that happened to work. Final
separation is now 0.1946 m lateral / 0.0046 m vertical, inverting the old
0.0197 / 0.1060 and giving the renders something real to show.
Note 0.00 failing is NOT a contradiction of ADR-037: the old code retreated only
`from_arm`, whereas this code retreats both arms, so zero lateral offset makes
them collide.

**Fix 3 — prop repositioning (TRIED, FULLY REVERTED).** Moving plate, mug,
spoon and bottle to the table corners was intended to declutter the renders. It
broke two skills and was reverted in full:
- `pick(A, bottle)` broke outright — the bottle at (0.20, 0.20) is outside arm
  A's reach (IK residual **0.1503 m** vs a 0.01 m tolerance). Reverting the
  bottle alone restored it (lift 0.4400 → 0.6199).
- `handoff` broke at phase 3 (`to_arm` approach) for **every** prop
  configuration tried: all four moved, bottle-reverted-only, bottle+spoon
  reverted, mug-moved-alone, and plate-moved-alone. Only the fully original
  layout passes.

**This is a finding worth keeping: the handoff corridor is sensitive to scene
composition in a way nothing predicted.** Moving a single prop that the skill
never touches — the mug, at the far corner — is enough to make `to_arm`'s
staging approach fail on a cross-arm collision. The mechanism is not understood
and was not chased; it is recorded here so nobody assumes prop placement is
cosmetically free. `gen_dual_scene.py` was restored from git and the fork colour
re-applied on top, so no stale "moved toward corner" comments survive.

**Fix 4 — fixed-camera sequence (PARTIAL).** `m06-handoff-sequence.png` now uses
one camera across all four panels, so the reference frame is stable. But the
chosen distance (1.2 m) and azimuth put the arms edge-on and too small to read,
and the fork is not visible in it. The strip is committed as-is and flagged:
**it still does not demonstrate the handoff and should not be used as evidence.**
The three stills do.

**Verification (bm-ptl, `mujoco==3.2.7`, seed 0), all four working skills after
the adopted changes:**

| skill | result |
|---|---|
| `pick(A, fork)` | success, z 0.3560 → 0.3989 |
| `place(A, fork, table)` | success, released, z 0.3588 |
| `pick(A, 'bottle')` | success, z 0.4400 → 0.6192 |
| `handoff(A→B, fork)` | success, A=None B='fork', 6610 frames, `from_arm_retreat_dist=0.2263` |

`pytest tests/test_skills.py`: 4 passed / 4 failed, the same four pre-existing
failures, unchanged.

**Note on prior measurements.** `docs/hardware/*` records predate nothing here —
the prop layout is unchanged from `311430e` — but the fork's colour differs, so
any render in those documents shows a silver fork.

**API note carried forward:** `run_handoff(env, to_arm, from_arm, obj)` is
receiver-first. An A→B handoff is `run_handoff(env, "B", "A", "fork")`.

### Addendum, Sept 14 2026 (same day, follow-up pass) — Fix 4 corrected; a
### measurement discrepancy flagged; independent re-verification

**ORCHESTRATOR CORRECTION (supersedes the two items below).**

*On provenance — there is no anomaly.* `cc1a329` was made by the orchestrator
session, not an unknown process. The builder agent working these fixes appeared
to have died (its log had been silent for ~100 minutes after SSH rate-limiting
on the jump host), so the orchestrator took the work over, found that agent's
in-progress edits in the shared working tree, corrected the retreat value from
0.15 to 0.10 on the basis of a measured sweep, verified all four skills, and
committed the result. The agent then resumed and correctly observed its own
prose inside an already-made commit. Its report of the facts was accurate; only
the framing as a provenance irregularity was wrong. Two agents editing one
working tree while the orchestrator commits it is the actual mechanism, and the
attribution line on `cc1a329` is this project's standard one.

*On the 0.1946 m vs 0.0983 m discrepancy — both figures are correct; they
measure different reference points.* Re-measured directly on bm-ptl in a single
run reporting both:

| reference | lateral (y) | vertical (z) | 3D |
|---|---:|---:|---:|
| pinch point — midpoint of `armX_gripper` and `armX_moving_jaw_so101_v1` | **0.1946 m** | 0.0046 m | 0.1946 m |
| `armX_gripperframe` site (`data.site_xpos`) | 0.0983 m | 0.1508 m | 0.1818 m |

The site-to-pinch-point offset measures **0.0888 m on each arm** — precisely the
~8 cm ADR-025 recorded when it retargeted IK away from the site for exactly this
reason. Fix 2's table uses the **pinch point**, which is what actually holds the
object and what ADR-025 established as this project's reference; the addendum
used the site. Nothing is unreproducible: the 3D separations (0.1946 vs 0.1818)
are close, and the two references distribute that distance differently between
y and z because the site sits along each gripper's own axis. Fix 2's figure
stands as written, now with its reference stated explicitly.

*Fix 4's correction below is accepted and verified.* The orchestrator inspected
the regenerated `m06-handoff-sequence.png`: the fixed camera holds a stable
frame, both arms are visible and separated in all four panels, and the red fork
is legible in the final panel. The azimuth=90 occlusion diagnosis is correct —
both arms sit near x≈0 and differ only in y, so that viewing ray puts one
behind the other. The strip is now usable as evidence.

**Provenance note, reported for transparency.** This addendum was written in
a session that found the four fixes above (red fork, 0.10 m lateral retreat,
full revert of prop repositioning) ALREADY present and already committed in
this file and in `skills_scripted.py`/`gen_dual_scene.py`, under a different
commit author/co-author line than this session's own attribution. The code
in that commit is, line for line, the same code this session had
independently arrived at (including this session's own comment prose),
which means the two were not truly independent — this session's own
in-progress edits were committed by another process before this session
finished. This is recorded here rather than silently built on top of,
per this project's own "reported honestly" convention.

**Fix 4 was NOT left in its "PARTIAL... does not demonstrate the handoff"
state above.** That entry's own azimuth=90/distance=1.2 m camera (the
task's suggested starting point) was rendered and INSPECTED (not merely
computed): only one arm is visible in any panel. At azimuth=90 the two
arms — offset only in y, both near x≈0 — sit almost exactly in line with
the viewing ray, so one occludes the other instead of separating
left/right as the task's own rationale for that angle assumed. Fixed by
reusing `m06-handoff-candidate-2.png`'s own already-good camera direction
(azimuth=130, elevation=-22, ALSO confirmed by inspection to show both
arms clearly separated plus the visible red fork), widened from that
candidate's 0.6 m distance to 0.9 m so arm B (still at HOME, farther from
`transfer_point`, in milestone 1) is not cropped out of the first panel.
Re-inspected after the change: both arms visible and clearly separated in
all four panels, with the red fork visible near arm B by the final panel.
`m06-handoff-sequence.png` is regenerated with this camera; the strip DOES
now demonstrate the handoff and can be used as evidence, superseding the
"should not be used as evidence" caveat above.

**Measurement discrepancy, flagged not silently corrected.** This entry's
Fix 2 table reports "0.1946 m" final lateral separation at 0.10 m lateral
retreat. Independently re-measured this session, via the exact same
shipped code/scene/seed, directly from `data.site_xpos` at the final
frame (`abs(site_a_final[1] - site_b_final[1])`, the most literal reading
of "final-frame lateral gripper separation"): **0.0983 m**, reproduced
identically across two separate runs. Every OTHER number in this entry's
own verification table (`frames_used=6610`, `from_arm_retreat_dist=0.2263`,
all four skills' z-values) matches this session's own re-measurement
exactly, so the underlying run is confirmed identical — only the "0.1946 m"
figure could not be reproduced by this direct method. Left unresolved
(not guessed at further) because chasing it would cost more bm-ptl round
trips than this pass's budget allowed; **0.0983 m is the number this
session verified and stands behind.**

**Also done this pass, not covered above:** `docs/images/m02-scene.png`
re-rendered against the (reverted-to-original-positions, red-fork) scene —
inspected: original prop layout, red fork visible on the table, both arms
in their fold-back home pose. `docs/images/m06-handoff-complete.png` was
NOT touched, per instruction.

**Re-verified this pass (bm-ptl, `mujoco==3.2.7`, seed 0), independently
from the table above, using the correct `target_object="bottle"` key
(`OBJECT_BODY_NAME["bottle"] == "water_bottle"`; a leftover diagnostic
script from an earlier session had used the wrong key `"water_bottle"` and
reported a false failure — not a real regression, a script bug, corrected
here):**

| skill | result |
|---|---|
| `pick(A, fork)` | success, frames_used=1655, weld_attach_frame=1155, z 0.3560 → 0.3989 |
| `place(A, fork, table)` | success, frames_used=3455, final xyz=(-0.0081, 0.0203, 0.3588) |
| `pick(A, bottle)` | success, frames_used=1655, weld_attach_frame=1155, z 0.4400 → 0.6192 |
| `handoff(A→B, fork)` | success, frames_used=6610, weld_attach_frame=5310, `from_arm_retreat_dist=0.2263`, lateral_y_sep=0.0983 |

`pytest tests/test_skills.py`: 4 passed / 4 failed, same four tests
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`) failing as before this whole ADR-038
body of work began — no regression. Note the specific failure REASON
strings for the plate/mug tests differ from some intermediate runs during
this work (e.g. `weld_attach_failed_after_300_frames` vs an IK-convergence
reason) purely because plate/mug are back at their original coordinates,
not a new defect — same tests still fail, same tests still pass.

**Constraints honored this pass:** only `gen_dual_scene.py` (documentation
only — the position constants were already back at their pre-ADR-038
values), `skills_scripted.py` (comment correction only — the retreat
constants were already 0.10), `scripts/render_handoff_frames.py` (the
sequence camera fix above), the regenerated XML, four images, and this
file were touched. `git diff --stat -- scenes/so101/` confirmed empty.
ADR-031's GRIP freeze, ADR-033's hover, ADR-034's guard and ADR-037's
choreography phases are unmodified.

**Source commit:** `cc1a329` (given).

---

### M10 Perception Overview

M10 trains a small vision model, PoseNet, to localise the three props the
demo actually manipulates — `fork`, `water_bottle`, `mug` — while `plate`
and `spoon` stay in-scene as unlabelled decoration and occluders
(**ADR-041**). A dedicated overhead camera, `posenet_cam`, separate from
the presentation-facing `front` camera, was added because tall-prop-behind-
short-prop occlusion was measurably worse from `front`'s eye-level angle
(**ADR-040**). The training set is 5,000 samples, visibility-filtered on an
occlusion ratio (MuJoCo segmentation: full-scene pixels / single-prop
pixels) below a 0.30 threshold, rejecting 3.94% of draws (**ADR-041**). The
model reuses the ResNet18-scale, 11.3M-parameter backbone M03 already
proved converts to OpenVINO IR across CPU/GPU/NPU (**ADR-009, ADR-042**),
trained on the Arc B390 iGPU via native `torch.xpu` — no IPEX — for 30
epochs in 15.9 minutes at 141.2 samples/sec (**ADR-043, ADR-044**),
reaching per-prop MAE of fork 3.2 mm, water_bottle 2.6 mm, mug 2.8 mm.

Four caveats. Each prop's z is pinned to a fixed resting height, so
ground-truth z variance is zero and this is effectively 2-D localisation,
not 3-D pose. The data is synthetic, single-camera, un-augmented, with both
arms always at the home keyframe — in-distribution figures, not a
robustness claim. Phase 4 (OpenVINO conversion) and Phase 5 (controller
integration) are not done: PoseNet is trained but not wired into any skill,
and the controller still reads prop positions from simulator ground truth.

---

### ADR-039 — M10 Phase 1 (PoseNet training data): runtime qpos randomization through `TableSettingEnv`'s opt-in `front` camera, per-prop resting z, sequential rejection-sampled placement; full-joint rejection sampling measurably failed and was replaced

**Recorded:** Sept 14, 2026 · **Follows:** ADR-016 (frozen upstream asset), ADR-020
(bm-ptl-only MuJoCo execution), ADR-022 (opt-in camera rendering), ADR-038 (a moved
prop can break `handoff` at phase 3) · **Adds:** `scripts/generate_posenet_data.py`,
`data/posenet/dataset_meta.json` (tracked), `data/posenet/{images,labels}/`
(gitignored), `docs/hardware/m10-training-data-samples.md`.

**Scope.** M10 Phase 1 only: generate labelled (image, ground-truth-xyz) pairs for
a future PoseNet. No model is defined or trained here, and no inference runs. This
is the dataset that will eventually let the scripted controller's oracle
`data.xpos` prop-position reads be replaced by camera-based inference.

**Decision 1 — render through the existing opt-in camera path, touch neither XML
file.** The `front` camera lives in the GENERATED scene
(`src/bimanual/sim/assets/so101_dual_table.xml`, `gen_dual_scene.py`'s output), not
in the frozen upstream asset (`scenes/so101/`, ADR-016). This script constructs
`TableSettingEnv(cameras=["front"], ...)` and calls `env.render("front")` (the
escape hatch documented in `env.py`) for each sample's capture — it never reads or
writes either XML file directly, and never re-invokes `gen_dual_scene.py`.

**Decision 2 — randomize x,y only, at runtime, via `data.qpos`; z stays per-prop.**
Each of the five props (`plate, mug, fork, spoon, water_bottle`) has its own
resting z baked into the scene's "home" keyframe (0.355 / 0.39 / 0.356 / 0.356 /
0.44 m against a 0.35 m table surface). This script reads each prop's z LIVE off
`data.qpos` immediately after `env.reset()` (not a hardcoded duplicate of
`gen_dual_scene.py`'s position constants) and overwrites only the x,y slots of
each prop's free-joint qpos before `mujoco.mj_forward`. A single shared z (e.g.
the table surface, 0.35) would sink every prop partway into the table slab; this
was flagged before any code was written and never implemented.

**Decision 3 — no Pillow; `write_png` copied with attribution.** `PIL` is not
installed in bm-ptl's `ov_env` and this script does not install it (an unrelated
install is what silently upgraded mujoco during the ADR-037 mink probe). The PNG
writer is copied verbatim from `scripts/render_handoff_frames.py`'s own
`write_png` (itself copied from `scripts/probe_render.py`), a stdlib-only
(zlib + hand-rolled IHDR/IDAT/IEND) encoder. Verified on HxWx3 uint8 input at
224x224 (this module's resolution) before generating any volume of images, via a
10-sample run whose images were copied back to the laptop and visually inspected.

**Decision 4 — sequential, largest-first placement with rejection sampling,
REPLACING a full-joint draw that measurably failed.** Initial implementation drew
all five props' x,y simultaneously from `[-0.15, 0.15]` m and rejected/retried the
whole draw on any pairwise overlap (radii `plate=0.06, mug=0.06, fork=0.08,
spoon=0.07, water_bottle=0.03` m, `+0.02` m clearance, 500 attempts). This failed
on the FIRST real run on bm-ptl: sample 0 exhausted all 500 attempts and raised
`RuntimeError` before writing anything. Diagnosis: five simultaneous pairwise
constraints inside a 0.30 m x 0.30 m (0.09 m^2) box is a tight packing problem —
the fork/spoon threshold alone (0.08+0.07+0.02=0.17 m) forbids a disk of area
~0.091 m^2, i.e. up to the entire box, once the first point lands near centre.
**Fix, not a workaround:** placement is now sequential — largest footprint first
(`fork, spoon, plate, mug, water_bottle`), each prop drawn against only the props
already placed (a 1-point rejection problem, not a 5-point joint one), clearance
margin reduced to 0.015 m, 5,000 draws budgeted per prop, up to 200 whole-sample
restarts if a prop's own budget is exhausted. Re-run after the fix: the 10-sample
verification succeeded with `placement_draws` ranging from single digits to 172
(mean 77.2, max 172) — comfortably inside budget, and the full 5,000-sample run
completed under the same scheme (see `dataset_meta.json` for its own measured
draw statistics).

**Decision 5 — `.gitignore` narrowed, not left as a blanket `data/` ignore.** M01
had gitignored the whole `data/` directory. That would have swept
`dataset_meta.json` (the reproducibility record: seed, ranges, mujoco version,
scene checksum) into the same ignore as the bulk images/labels. Replaced with two
specific entries, `data/posenet/images/` and `data/posenet/labels/`, verified with
`git check-ignore -v` (image/label paths matched; `dataset_meta.json` did not) and
`git status` (meta file shows as trackable; images/labels do not appear at all).

**On reachability, stated once more so it cannot be missed.** ADR-038 found that
moving even a single prop can break `handoff` at phase 3. This dataset's
randomized layouts are consequently expected to include many configurations in
which the scripted manipulation skills would fail — and that is fine, because no
skill is ever executed against any sampled layout here; this is a
perception-only dataset. `dataset_meta.json`'s own `note` field and
`docs/hardware/m10-training-data-samples.md` both say this explicitly, so the
dataset is never later cited as evidence of validated or reachable scene
configurations.

**Baseline preserved.** `pytest tests/test_skills.py` was re-run on bm-ptl before
any of this module's code touched the repo: 4 passed / 4 failed, identical to the
pre-existing baseline (no source file this module is scoped to touch —
`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`gen_dual_scene.py`, `scenes/so101/`, `so101_dual_table.xml` — was modified).

**Source commit:** `87fd608` ("M10 Phase 1: PoseNet training data generation
(5000 samples, front camera, randomized prop positions).").

---

### ADR-040 — M10 Phase 1.5: dedicated `posenet_cam` perception camera separated from `front`'s presentation role; eased rejection sampling (range widened, margin reduced, radii kept); placement order shuffled per sample

**Recorded:** Sept 14, 2026 · **Follows:** ADR-016 (frozen upstream asset), ADR-020
(bm-ptl-only MuJoCo execution), ADR-022 (opt-in camera rendering), ADR-038 (a
scene change assumed cosmetic broke `handoff` at phase 3), ADR-039 (M10 Phase 1
dataset, now superseded) · **Modifies:** `scripts/gen_dual_scene.py`,
`src/bimanual/sim/assets/so101_dual_table.xml` (regenerated, `scenes/so101/`
untouched), `scripts/generate_posenet_data.py`, `data/posenet/dataset_meta.json`
· **Adds:** `docs/hardware/m10-camera-comparison.md`.

**Why.** ADR-039's dataset (M10 Phase 1) rendered through `front`
(`pos="1.55 0 0.85"`), a wide establishing shot built for the M06 handoff demo
video, not a perception viewpoint. Measured directly on the three archived
Phase 1 samples: props occupy only ~10-20 px of a 224x224 frame. Phase 1's
rejection sampler also measured a worst-case 5,054 draws for one prop against
a 5,000-per-prop cap on the full 5,000-sample run — already past the nominal
budget, not comfortably inside it.

**Decision 1 — a SEPARATE `posenet_cam`, `front` never touched.** `front` is
load-bearing for the M06 handoff render pipeline (ADR-038) and four working
skills; per that ADR's own lesson ("should be inert" was the assumption that
broke `handoff` before), this pass adds a NEW camera to
`gen_dual_scene.py`'s hand-authored region instead of widening or repointing
`front`. `front`'s own `FRONT_CAM_POS`/`FRONT_CAM_XYAXES`/`FRONT_CAM_FOVY`
constants are byte-identical before and after this change.

**Decision 2 — the task brief's proposed `xyaxes` was checked by hand, not
accepted, and was found to point AWAY from the table.** MuJoCo cameras look
along local -Z, and local Z = local X cross local Y. The brief's axes
(`x=(0,-1,0)`, `y=(-0.3,0,0.95)`) give `Z=(-0.95,0,-0.3)`, so the view
direction (`-Z`) is `(+0.95,0,+0.3)` — away from the table entirely, from
`x=1.05`. Fix: flip the sign of the local x-axis to match `front`'s own
handedness (`x=(0,+1,0)`, not `(0,-1,0)`): `Z=(0.95,0,0.3)`,
view=`(-0.95,0,-0.3)` — toward the table and angled down, from the intended
closer/lower position. `POSENET_CAM_POS=(1.05,0,0.62)`,
`POSENET_CAM_XYAXES="0 1 0 -0.3 0 0.95"`, `POSENET_CAM_FOVY=45` (MuJoCo's own
default; `front`'s widened 55 was for full-arm headroom this camera does not
need). **Verified by rendering one frame before generating anything** — both
`front` and `posenet_cam` probe renders are in
`docs/hardware/m10-camera-comparison.md`; the table, both arms and all five
props are clearly in frame through `posenet_cam`.

**MANDATORY re-verification, done before any dataset generation.** A camera
element has no geom/mass/collision so it should be inert, but ADR-038 found
"should be inert" was exactly the wrong assumption once before. All four
skills backing the 30-point bimanual criterion were re-run fresh against the
regenerated scene: `pick(A, fork)` PASS, `place(A, fork, table)` PASS,
`pick(A, 'bottle')` PASS, `handoff(A->B, fork)`
(`run_handoff(env, "B", "A", "fork", weld=weld)`) PASS — identical in kind to
the ADR-038 baseline. `pytest tests/test_skills.py`: 4 passed / 4 failed,
identical to the pre-existing baseline. No regression.

**Decision 3 — a second brief claim was also checked and found wrong: the
proposed flat 0.03 m spacing was rejected (kept the radius-based test), and a
THIRD brief claim (fork/spoon radii are "far larger than actual footprint")
was checked and found wrong too.** `gen_dual_scene.py`'s geometry gives fork's
true farthest point as `sqrt(0.08^2+0.012^2)=0.0809 m` (declared `0.08` is
~1mm UNDER, not over) and spoon's as `sqrt(0.068^2+0.012^2)=0.0690 m`
(declared `0.07` is ~1mm over — accurate to the millimetre). Since this is a
circular (isotropic) distance test and props are never rotated, reducing
either below its true reach risks genuine overlap for some relative bearing.
`FOOTPRINT_RADIUS_M` is UNCHANGED from Phase 1. Easing came instead from: (a)
`randomization_range_m` widened from +-0.15 m to +-0.18 m (0.36x0.36 m box,
1.44x the old area), (b) `clearance_margin_m` reduced from 0.015 m to 0.008 m
(still a real, positive gap). Combined, the tightest pair's (fork/spoon)
forbidden-disk fraction of the box drops from ~95% (Phase 1, matching the
observed 5,054-draw worst case) to ~61%.

**Decision 4 — placement order shuffled per sample, not fixed.** Phase 1
always placed in the same largest-first order (`fork, spoon, plate, mug,
water_bottle`) on every sample, so `water_bottle` (smallest radius) was
placed LAST every single time — a systematic bias a trained PoseNet could
pick up as a spurious identity-correlated signal. Fixed: a fresh
`rng.permutation` of the five props is drawn once per sample (same order
reused across any restarts within that one sample).
`placement_order_used` is recorded per sample label;
`dataset_meta.json` records `base_placement_order` and
`placement_order_shuffled_per_sample: true`.

**10-sample verification before the full run.** `placement_draws_stats`:
mean 18.6, max 34 (down from Phase 1's mean 54.8, max 5,054) — comfortably
under the task's <500 target. Pixel extent measured with a standalone
pure-stdlib PNG decoder + largest-connected-component analysis (no Pillow,
no new dependency): `mug`/`plate`/`water_bottle` (compact, blob-like props
not confounded by the arm's own grey housing aliasing `spoon`'s hue) show a
measured ~1.79x mean linear / ~3.2x area increase over Phase 1. `spoon` (and,
to a lesser extent, the very thin `fork`) is EXCLUDED from the quantitative
comparison and disclosed as such: the arm pose is never randomized, so a
fixed-position false "spoon" region (matching the arm's own silver/grey
housing) appeared identically in every sample before a largest-connected-
component filter was added; even after that fix, thin/elongated props are
not something this quick colour-based method can measure with confidence.
Three flagged 2D bounding-box "overlaps" out of 10 samples were checked
against the real label data and found to be a perspective artifact (a tall
prop's silhouette crossing a short, nearby prop's screen region from this
angled camera, e.g. `water_bottle` vs `plate` at true 3D centre distance
0.1107 m against a required 0.098 m minimum) — not a real geometric overlap.
Full detail, both probe renders, and three new sample images are in
`docs/hardware/m10-camera-comparison.md`.

**Decision 5 — full regeneration overwrites `data/posenet/` in place.**
Same seed (20260914) as Phase 1. `dataset_meta.json` gains `phase: "M10 Phase
1.5"`, a `supersedes` field describing exactly what Phase 1 configuration is
being replaced, `base_placement_order`/`placement_order_shuffled_per_sample`,
the new `camera`, `randomization_range_m`, `clearance_margin_m`, and scene
`scene_xml_sha256` (the regenerated scene's own camera addition changes this
hash even though `scenes/so101/` itself is untouched). Measured wall clock,
on-disk size, and the full run's own `placement_draws_stats` are recorded in
`dataset_meta.json` and in `docs/hardware/m10-camera-comparison.md`. As with
Phase 1, the image/label bulk is not left on bm-ptl after copy-off; only
`dataset_meta.json` stays git-tracked (`.gitignore`'s existing
`data/posenet/images/`/`data/posenet/labels/` entries, unchanged from
ADR-039).

**Baseline preserved.** `pytest tests/test_skills.py` re-run on bm-ptl against
the regenerated scene: 4 passed / 4 failed, identical to the pre-existing
baseline. No file outside this ADR's own `Modifies`/`Adds` list
(`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`scenes/so101/`, `README.md`, `SUBMISSION.md`) was touched.

**Source commit:** `d1157e1` ("M10 Phase 1.5 corrected: 3-prop scope,
visibility filtering, overhead camera (ADR-040, ADR-041).").

---

### ADR-041 — M10 Phase 1.5 correction: 3-prop label scope (fork/water_bottle/mug), occlusion-ratio visibility filtering via MuJoCo segmentation (colour classification tried and measurably failed first), `posenet_cam` pushed further overhead — supersedes ADR-040's camera pose and label scope, keeps ADR-040's mechanism otherwise unchanged

**Recorded:** Sept 14, 2026 · **Follows:** ADR-016 (frozen upstream asset), ADR-020
(bm-ptl-only MuJoCo execution), ADR-022 (opt-in camera rendering), ADR-038 (a
scene change assumed cosmetic broke `handoff` at phase 3), ADR-039 (M10 Phase
1, superseded), ADR-040 (M10 Phase 1.5, camera pose and label scope
superseded here; its mechanism — dedicated `posenet_cam`, eased rejection
sampling, shuffled placement order — is KEPT, not rewritten) ·
**Modifies:** `scripts/gen_dual_scene.py`, `src/bimanual/sim/assets/so101_dual_table.xml`
(regenerated, `scenes/so101/` untouched), `scripts/generate_posenet_data.py`,
`data/posenet/dataset_meta.json` · **Adds:**
`docs/hardware/m10-scope-reduction-samples.md`.

**Starting state.** ADR-040's own 5,000-sample regeneration was in progress
when it was halted by the orchestrator — `data/posenet/images`/`labels` were
found ~half-populated (2,566 of 5,000) and were deleted, per this task's
explicit instruction to treat the dataset as absent and generate fresh.
Nothing from that partial run is reused.

**Correction 1 — the task brief's proposed camera `xyaxes` reintroduced the
EXACT sign error ADR-040 already fixed once, and the brief also misquoted
ADR-040's shipped value.** The brief stated the "current" `posenet_cam` was
`xyaxes="0 -1 0 -0.3 0 0.95"` — that is the *original, broken* pre-ADR-040
spec, not what is actually committed (`so101_dual_table.xml:402`, which
already reads the corrected `xyaxes="0 1 0 -0.3 0 0.95"`). The brief's NEW
proposal for this pass, `xyaxes="0 -1 0 -0.7 0 0.7"`, was checked by hand
before use (not accepted on faith, the same discipline ADR-040 applied) and
found to have the SAME defect: with `x=(0,-1,0)`, `y=(-0.7,0,0.7)`,
`Z=X×Y=(-0.7,0,-0.7)`, view `=-Z=(+0.7,0,+0.7)` — from `x=0.6`, further out
along `+x` and tilted UP, past the table, not at it. Fixed the same way
ADR-040 fixed it: flip the local x-axis sign to `x=(0,+1,0)`. With
`x=(0,1,0)`, `y=(-0.7,0,0.7)`: `Z=(0.7,0,0.7)`, view`=(-0.7,0,-0.7)` — toward
the table and down at ~45 degrees, from the raised, pulled-in
`POSENET_CAM_POS=(0.6,0,1.1)` this pass intends. `POSENET_CAM_FOVY=45`
unchanged from ADR-040. **Verified by rendering one probe frame before
generating anything** — both `front` and `posenet_cam` probes are in
`docs/hardware/m10-scope-reduction-samples.md`; table, arms and all five
props are clearly in frame, more overhead than ADR-040's own already-working
pose. `front` itself is untouched (byte-identical `FRONT_CAM_*` constants);
`git diff --stat -- scenes/so101/` stays empty (ADR-016).

**MANDATORY re-verification, done before any dataset generation.** All four
skills backing the 30-point bimanual criterion were re-run fresh against the
regenerated scene: `pick(A, fork)` PASS (z 0.3560→0.3989, weld attach frame
1155), `place(A, fork, table)` PASS (returns to z=0.3588), `pick(A, 'bottle')`
PASS (z 0.4400→0.6192, weld attach frame 1155), `handoff(A→B, fork)`
(`run_handoff(env, "B", "A", "fork", weld=weld)`) PASS (held by arm B,
z=0.5498, `from_arm` cleared, retreat 0.2263 m) — identical in kind to the
ADR-038/ADR-040 baseline. `pytest tests/test_skills.py`: 4 passed / 4 failed,
identical to the pre-existing baseline. No regression.

**Correction 2 — 3-prop label scope (task FIX 1).** All five props remain
placed, randomized and rendered (occlusion between all five is part of the
training signal; dropping `plate`/`spoon` from the scene would make the
remaining three trivially unoccluded). Only `TARGET_PROPS = ("fork",
"water_bottle", "mug")` receive a ground-truth label and count toward the
visibility gate below; `plate`/`spoon` stay in-scene as unlabelled
decoration (`decoration_props_in_scene_unlabelled`). Each label's
`positions_xyz_m` is now a fixed `(3, 3)` array in `TARGET_PROPS`' canonical
order (`output_shape: [3, 3]`).

**Correction 3 — the task brief's own description of the visibility metric
did not match the formula it then specified; the FORMULA was right, the
DESCRIPTION was wrong.** The brief described the filter as dropping frames
where "the ratio of visible pixels to bounding-box area falls below 0.3" — a
FILL-FRACTION metric (shape, not occlusion; a thin, fully-visible fork would
fail on shape alone). The formula the same brief then gave,
`full_scene_pixels / single_prop_pixels`, is a genuine OCCLUSION ratio
instead. **This is what is implemented**, described as an occlusion ratio
throughout code, `dataset_meta.json`, and the docs — not as Syn4D's fill
fraction. The 0.30 threshold is a **user-supplied reference to Syn4D**
(https://arxiv.org/pdf/2605.05207) for that numeric value only; no claim is
made about the paper's authorship, method, or other findings beyond the
threshold cited to it (same discipline as ADR-037's ScienceDirect citation).

**Correction 4 — the first implementation of the occlusion ratio (RGB colour
classification) was tried, measured, and found unreliable before being
trusted, and was replaced with MuJoCo's segmentation buffer.** Comparing
rendered pixels against each prop's declared `<material rgba>` measured two
real failures on bm-ptl, on this exact scene: (a) MuJoCo's lighting does not
preserve a material's own colour ratio — `fork`'s declared `(255,38,38)`
rendered with a dominant pixel colour of `(255,80,80)`, an exact-match count
of 1 against a visually obvious ~100+ pixel sliver; (b) widening the
tolerance to compensate then ALIASED with unrelated scene elements —
`water_bottle`'s blue collided with the background checker floor tile's own
rendered blue, and `fork`'s red collided with the table surface's warm tan.
Both measured directly with a pixel-histogram probe, not assumed. Fixed:
`compute_visibility_ratios` now uses
`mujoco.Renderer.enable_segmentation_rendering()`, whose `(H,W,2)` int32
output (channel 0 = geom id, channel 1 = constant `mjtObj.mjOBJ_GEOM`) was
verified on a probe render before use — exact per-pixel geom identity, no
lighting-dependent ambiguity. Classification is by `model.geom_bodyid`
membership (a prop can be >1 geom on one body, e.g. mug = cylinder + handle),
not a single assumed geom id.

**Correction 5 — the "hide other props" mechanism was verified to genuinely
remove them from the render, not assumed.** Other props are hidden for a
solo render by writing a real, far off-screen `(x,y)=(3.0,3.0)` into their
own free-joint qpos and calling `mujoco.mj_forward` — a genuine kinematic
relocation, not an `rgba`/`alpha=0` trick (which can still write depth or
leave faint pixels depending on the renderer path, per the task's own
caution). Verified directly: placing one target in view and teleporting the
other four away, the segmentation buffer contained ONLY that target's own
geom ids in every one of three trials (fork: 156 px, zero elsewhere;
water_bottle: 922 px, zero elsewhere; mug: 616 px, zero elsewhere) — see
`docs/hardware/m10-scope-reduction-samples.md`.

**Verification before the full run, per the task's explicit "measure before
committing to 5000."** A 10-sample run (0/10 rejections, 2.75 samples/s) was
followed by a 100-sample run for a statistically meaningful rejection-rate
estimate: **4/104 total draws rejected (3.85%)**, well under the 30%
go-threshold, at **2.40 samples/s** — projecting the full 5,000-sample run at
**~35 minutes**, far under the "2+ hours" worst case the task flagged as
plausible (visibility filtering costs 1 RGB render + 4 segmentation renders
per accepted sample, but segmentation-mode renders measured cheaper than
full RGB, so the realized per-sample cost came in below a naive 4-5x
estimate over Phase 1's 3.04 samples/s). The global minimum ACCEPTED
occlusion ratio observed across both probes was 0.310 (`mug`, nearly fully
hidden behind `water_bottle` from this camera angle) — 0.010 above the
threshold, direct evidence the gate is doing real work, not passing
everything through. Reported before launching the full run, per the task's
instruction.

**Decision — proceed to the full 5,000-sample run.** Camera correctly aimed
and more overhead than ADR-040's own pose, all four skills plus pytest
baseline unaffected, hide mechanism proven pixel-exact, rejection rate
comfortably under 30% with a projected time far under 2 hours. Full
regeneration overwrites `data/posenet/` in place (same seed `20260914` as
Phase 1/1.5). `dataset_meta.json` gains `target_props`, `output_shape:
[3,3]`, `decoration_props_in_scene_unlabelled`, `visibility_threshold: 0.30`,
`visibility_metric`/`visibility_threshold_reference` (the Syn4D citation),
the new `camera_pos_m`/`camera_xyaxes`, `visibility_rejections_total`/
`visibility_rejection_rate`, and a `supersedes` field naming BOTH Phase 1
(ADR-039) and Phase 1.5 (ADR-040, whose regeneration never completed) by
name. Measured full-run wall clock, rate, and rejection rate are recorded in
`dataset_meta.json` and in `docs/hardware/m10-scope-reduction-samples.md`.
As with Phase 1/1.5, the image/label bulk is not left on bm-ptl after
copy-off; only `dataset_meta.json` stays git-tracked.

**Baseline preserved.** `pytest tests/test_skills.py` re-run on bm-ptl
against the regenerated scene: 4 passed / 4 failed, identical to the
pre-existing baseline. No file outside this ADR's own `Modifies`/`Adds` list
(`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`scenes/so101/`, `README.md`, `SUBMISSION.md`) was touched. ADR-040 itself is
kept unedited above — this entry corrects its camera pose and label scope
going forward, it does not rewrite what ADR-040 recorded as true at the
time.

**Source commit:** `d1157e1` ("M10 Phase 1.5 corrected: 3-prop scope,
visibility filtering, overhead camera (ADR-040, ADR-041)."). Same commit as
ADR-040 — both decisions were committed together.

---

### ADR-042 — M10 Phase 2: PoseNet architecture + dataset loader + training script, in a NEW separate `train_env` on bm-ptl (native `torch.xpu`, not IPEX) — `ov_env` untouched

**Recorded:** Sept 14, 2026 · **Follows:** ADR-009 (ARCHITECTURE.md — a small trained
vision model keeps the 20-point OpenVINO criterion off the risky ML branch), ADR-037
(the load-bearing `mujoco==3.2.7` + `openvino==2026.3.1` pairing in `ov_env`, and the
mink probe's cautionary tale of an unpinned install silently bumping mujoco), ADR-041
(the 5,000-sample, 3-prop dataset this module trains against) · **Adds:**
`src/bimanual/perception/posenet.py`, `src/bimanual/perception/dataset.py`,
`scripts/train_posenet.py`, `scripts/requirements-train.txt`, `checkpoints/.gitkeep`,
`docs/hardware/m10-phase2-smoke.md`.

**The brief's environment assumptions were checked and found wrong for both machines**
(PIL is not "proven available on bm-ptl" — it is absent from both `ov_env` and the
laptop; that absence is why `generate_posenet_data.py` and `render_handoff_frames.py`
both hand-roll `write_png`). Rather than install torch into `ov_env` — the exact
mechanism by which ADR-037's mink probe silently upgraded mujoco to 3.13.0 — this
module creates a **new, separate venv**, `C:\Users\devcloud\project\train_env`,
containing only torch + Pillow + numpy. `ov_env` is not installed into and is
re-verified functionally unchanged at the end (`pip list` shows no torch/Pillow;
`verify_adr038_skills.py`'s four skills still PASS with the same numbers on record;
`pytest tests/test_skills.py` still reports 4 passed / 4 failed with the same four
failing test names — see `docs/hardware/m10-phase2-smoke.md` section 7 for the full
transcript of all three checks).

**Device path: native `torch.xpu` worked on the first attempt** —
`pip install torch --index-url https://download.pytorch.org/whl/xpu` produced
`torch==2.14.0+xpu` with `torch.xpu.is_available()==True` and
`torch.xpu.get_device_name(0)=="Intel(R) Arc(TM) B390 GPU"`. IPEX
(`intel-extension-for-pytorch`) was never installed and never needed — per the task's
own instruction to try native XPU before the older IPEX path, and native XPU
succeeded outright, so there is no IPEX error to report. `scripts/requirements-train.txt`
pins the exact `pip freeze` (torch's xpu wheel pulls in Intel's oneAPI/SYCL/MKL
runtime automatically as transitive dependencies).

**PoseNet** re-derives (not imports — `scripts/` is not a Python package) the exact
ResNet18-scale backbone from `scripts/ov_smoke.py::build_model` (M03's proven
CPU/GPU/NPU-converting topology), with a global-avg-pool + Linear(512,256)+ReLU +
Linear(256,9) head. Measured 11,310,153 parameters — within the "~11-12M expected"
range. Forward-passed a dummy batch successfully on both CPU and XPU.

**PoseNetDataset** reads the real, committed 5,000-sample dataset
(`data/posenet/images` + `labels`, gitignored bulk / `dataset_meta.json` tracked) per
the schema verified directly against `generate_posenet_data.py`'s own `label = {...}`
construction, not guessed. Deterministic `sample_index % 10` split produced exactly
4500 train / 500 val, and `val[0]`'s loaded labels matched
`data/posenet/labels/sample_00000.json`'s `objects.*.xyz_m` exactly on hand
verification.

**A real Windows-`spawn` hang was reproduced, but in throwaway diagnostic scaffolding,
not in the deliverable.** An ad hoc `DataLoader`-iteration probe written without the
`if __name__ == "__main__":` guard hung for a full 300s timeout at `num_workers=4` —
a live demonstration of exactly the trap the task brief warned about. `train_posenet.py`
itself already carries the guard (required for its own `num_workers` DataLoader) and
its `--num-workers 4` smoke run completed normally in 9.98s; a second, corrected probe
run (guard added) measured `num_workers=0` at 104.7 samples/sec and `num_workers=4` at
53.3 samples/sec over a 10-batch/320-sample window — slower net of a one-time ~4.6s
Windows spawn-startup cost that a 10-batch probe cannot amortize, not evidence that
`num_workers=4` is broken. `num_workers=4` was kept as `train_posenet.py`'s default;
it was not dropped to 0.

**Smoke-tested only, per instruction — no full training run.** Model instantiation +
forward pass, one real dataset sample, and one training batch (loss reported, exits
cleanly) on `--device xpu` and on `--device cpu`, both producing the identical loss
(0.301587) from the same seeded init and batch — a correctness signal that model init,
data loading and the loss computation agree across devices. Full transcript, all
measured numbers, and the `checkpoints/` `.gitignore` verification (`git check-ignore
-v`, read as `source:line:pattern<TAB>pathname`) are in
`docs/hardware/m10-phase2-smoke.md`.

**Source commit:** `823168b` ("M10 Phase 2: PoseNet architecture, dataset loader,
training script (1-batch smoke tested).").

---

### ADR-043 — M10 Phase 3 prep: `openvino==2026.3.1` installed into `train_env` alongside torch (for Phase 4's live-PyTorch->OpenVINO conversion), `num_workers` default corrected 4 -> 0 — `ov_env` untouched

**Recorded:** Sept 14, 2026 · **Follows:** ADR-042 (created `train_env`, measured
`num_workers=0` at 104.7 samples/sec vs `num_workers=4` at 53.3 but left the default at 4
pending this correction), ADR-037 (`ov_env`'s load-bearing `mujoco==3.2.7` +
`openvino==2026.3.1` pairing, never installed into) · **Touches:**
`scripts/train_posenet.py`, `scripts/requirements-train.txt`.

**openvino installed into `train_env`, not `ov_env`.** `pip install --no-deps
openvino==2026.3.1` into `C:\Users\devcloud\project\train_env` succeeded on the first
attempt with no re-resolution of torch or numpy. Verified in ONE process, before and
after: `torch.__version__` unchanged at `2.14.0+xpu`, `torch.xpu.is_available()` still
`True`, `torch.xpu.get_device_name(0)` still `"Intel(R) Arc(TM) B390 GPU"`, `numpy`
unchanged at `2.4.6` (both before and after — the likeliest casualty did not occur), and
`openvino.__version__ == "2026.3.1-22476-759c5a6ab8c-releases/2026/3"`. The combined
conversion smoke test used the corrected call
(`ov.convert_model(model, example_input=x)`, matching `scripts/ov_smoke.py:181`'s proven
form) — the brief's own draft call passed a tensor to `input=` instead of
`example_input=`, which is the shape/type-spec parameter, not the traced-tensor one, and
would have raised. Ran clean: `[?,?]` output shape on a trivial `Linear(10, 4)`.

**`--no-deps` did skip one dependency, as the brief predicted.** `pip check` flagged
`openvino-telemetry` as an unmet requirement of `openvino`'s own metadata after the
`--no-deps` install (import itself did not raise or warn — the telemetry code path is
lazy — but `pip check` failed until this was fixed). Installed the single missing package
the same way: `pip install --no-deps openvino-telemetry==2025.2`, matching `ov_env`'s own
pin (`scripts/requirements-bmptl.txt:3`). Resolved to `2025.2.0`. `pip check` then
reported "No broken requirements found", and the combined smoke test above was re-run
after this second install to confirm nothing regressed. `train_env`'s final `pip list`:
torch, Pillow, numpy, openvino, openvino-telemetry, plus the unchanged Intel
oneAPI/SYCL/MKL transitive closure from ADR-042 — no other packages were touched.
`scripts/requirements-train.txt` records both additions with this reasoning; the wrong
target named in the brief, `requirements-bmptl.txt` (which describes `ov_env`, the venv
the brief itself says not to modify), was left untouched.

**`ov_env` re-verified functionally and byte-for-byte unchanged.** `pip list` under
`ov_env`: `mujoco==3.2.7`, `openvino==2026.3.1`, `numpy==2.4.6`, no torch, no Pillow.
`scripts/verify_adr038_skills.py` re-run under `ov_env`: all four skills still PASS with
the same measured numbers on record — `pick(A, fork)` lift +0.0429 m, `place(A, fork,
table)` final z=0.3588, `pick(A, water_bottle)` lift +0.1792 m, `handoff(A, B, fork)`
final separation lateral(y)=0.1946 m / vertical(z)=0.0046 m, frames_used=6610.
`pytest tests/test_skills.py` under `ov_env`: unchanged at 4 passed / 4 failed, same four
failing test names (`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`) for the same already-documented, out-of-scope
reasons (IK convergence/reach limits on the `plate`/`mug`/drawer path, not on the
`fork`/`water_bottle` path the four PASSING skills above cover).

**`num_workers` default corrected 4 -> 0 in `scripts/train_posenet.py`.** ADR-042 measured
0 workers at 104.7 samples/sec versus 4 workers at 53.3 samples/sec on this Windows host
(spawn overhead outweighs parallel-loading gain at this dataset size) but left the CLI
default at 4, unreconciled with its own measurement. That is fixed here: `--num-workers`
now defaults to `0`, with both the module docstring and the argparse help text citing the
two measured numbers directly, so the default is not "corrected" back to a higher value
later by someone assuming more workers is always faster. The `--num-workers` override
remains available and unchanged in mechanism (the `if __name__ == "__main__":` guard this
relies on for Windows `spawn` compatibility is untouched).

**No training was run.** This module is smoke/verification only, per instruction.

**Source commit:** `f0644f6` ("M10 Phase 3 prep: openvino in train_env,
num_workers=0 default (spawn overhead on Windows).").

---

### ADR-044 — M10 Phase 3: PoseNet trained on Intel Arc B390, per-prop MAE 2.6-3.2 mm; verified input-dependent, not mean-collapse

**Recorded:** Sept 14, 2026 · **Follows:** ADR-039/040/041 (dataset), ADR-042
(architecture), ADR-043 (train_env + openvino) · **Adds:**
`docs/hardware/m10-phase3-training-log.md`. Checkpoints are gitignored
(`.gitignore:12`, `checkpoints/*.pth`, ~45 MB each) and live only on bm-ptl.

**Result.** 30 epochs, **956.1 s (15.9 min)**, **141.2 samples/sec** on the Arc
B390 iGPU via native `torch.xpu` (no IPEX, ADR-042). Best val loss **0.000011**
at epoch 30. Per-prop MAE at the best checkpoint: **fork 3.2 mm, water_bottle
2.6 mm, mug 2.8 mm** — all inside the "< 0.02 m excellent" band, so Phase 4 can
proceed without an accuracy caveat on the headline number.

**The run was ~4x faster than the 60-90 min estimate.** 15.9 minutes for 30
epochs over 4500 samples. Two contributors: `--num-workers 0` (ADR-043 measured
104.7 vs 53.3 samples/sec, spawn overhead dominating on Windows), and the Arc
B390 handling an 11.3 M-param ResNet18-scale model comfortably.

**Verified genuinely learned, not mean-collapse.** A regressor that outputs the
dataset mean for every input can post a plausible loss; with props randomised
over x,y in [-0.18, +0.18] such a model would score ~90 mm MAE. Tested directly
on 5 validation samples with the best checkpoint:

```
std of PREDICTIONS across 5 samples: [0.1533 0.0672 0.0017 0.0472 0.0981 0.0021 0.0549 0.1108 0.0008]
std of GROUND TRUTH  across 5 samples: [0.1537 0.0673 0.      0.0469 0.0978 0.      0.0558 0.1098 0.     ]
pred_spread / gt_spread = 1.009
```

Predictions vary as much as ground truth does. Worst single-axis error across
those 15 prop-samples is 5.0 mm; most are under 3 mm.

**Curve shape: still improving at the end.** Epochs 1-2 drop steeply, 3-15
oscillate under a high cosine learning rate (train and val move together on the
downswings — not overfitting), 16-30 descend monotonically. **The best epoch is
the final one and val_loss was still falling**, so the model is under-trained
rather than over-trained; more epochs would likely help. Final train 0.000017 vs
val 0.000011 — val *below* train, no overfitting signal anywhere.

**Four caveats that must not be lost when this number is quoted.**
1. **z is not meaningfully predicted.** Ground-truth z std across samples is
   exactly 0: each prop's height is pinned to its own resting value by design
   (ADR-039's correction). Only x and y carry signal, so this is effectively 2-D
   localisation and the MAE should be read that way.
2. **Synthetic, single camera, no augmentation.** One fixed `posenet_cam` pose,
   one lighting condition. These are in-distribution figures, not robustness.
3. **Arms are always at the home keyframe** in every training image. A scene with
   arms mid-motion is out of distribution.
4. **The occluded tail is uncharacterised.** All five sanity samples had
   `visibility_ratio == 1.00`. Accepted samples run down to ~0.44 (ADR-041's
   filter rejects below 0.30), and accuracy there was not measured.

**Process note.** The first training attempt was launched by a builder agent as a
detached background process on bm-ptl; it initialised correctly (device, split,
param count all logged) and then died silently when its SSH session closed,
leaving an empty error log and no checkpoints. The successful run held the
process in a foreground SSH session inside a supervising background job — the
same pattern that carried the 35-minute dataset generation — so it could not be
orphaned.

**Source commit:** `7dcaaa6` ("M10 Phase 3: PoseNet trained, 30 epochs,
per-prop MAE fork=3.2mm bottle=2.6mm mug=2.8mm (ADR-044).").

---

### ADR-045 — M10 Phase 4: PoseNet converted to OpenVINO IR (FP32/FP16), benchmarked CPU/iGPU/NPU — all six combos succeeded (including NPU+FP32), one silent-default trap found and fixed, one threshold miss reported honestly

**Context.** M10 Phase 3 (ADR-044) produced a trained checkpoint. ADR-013's
Sept-12 correction, from M03, established the export/benchmark discipline
this phase must follow: static or bounded shapes for the NPU (a fully
dynamic batch dimension does not raise a catchable exception on this NPU --
it kills the whole interpreter with `STATUS_ACCESS_VIOLATION`), no ONNX
intermediate, and every NPU compile attempt isolated in its own subprocess
so one crash cannot destroy already-successful results from other devices.
ADR-043 put `torch` and `openvino` in the same `train_env` venv specifically
so this phase could convert AND benchmark in one place, on bm-ptl, without
the laptop/bm-ptl split M03's `ov_smoke.py` needed.

**Options.**
- (a) Convert once, benchmark all six (device, precision) combinations in one
  long-lived process, write one results file at the end.
- (b) Convert once (static batch-1 `[1,3,224,224]`, per ADR-013), then
  benchmark each (device, precision) combo in its OWN subprocess, in a fixed
  CPU -> GPU -> NPU order, appending each result to `results.jsonl` the
  instant it is known.

**Decision.** (b), mirroring `scripts/ov_smoke.py`'s
`do_single_variant`/`do_device_run` subprocess-per-variant pattern
(ARCHITECTURE.md's own M03 section) rather than reinventing a different
shape. (a) was rejected for the same reason ADR-013 already rejected it for
M03: a hard process kill during NPU's combos would silently cost CPU's and
GPU's already-obtained results too, and M03 already demonstrated this
happening once.

**Result.** `checkpoints/posenet_best.pth` (43.2 MiB, weights-only load,
11,310,153 params) converted to FP32 IR (43.13 MiB `.bin`) and FP16 IR (21.56
MiB `.bin`, exactly 0.500x). All six (device, precision) combos --
CPU/GPU/NPU x FP32/FP16 -- compiled and ran to completion **without a single
crash** on this hardware/driver/OpenVINO-version combination, including
NPU+FP32 (which the task brief flagged as a plausible capability limit and
deliberately left out of the correctness-threshold table; it compiled and
inferred successfully here, so its deviation is reported for the record with
no pass/fail threshold applied). The subprocess-isolation and incremental-
write discipline above was exercised on every combo and never actually
triggered by a crash this run -- worth recording as a fact about this run,
not as evidence the defence was unnecessary to build.

Headline mean latency / throughput (10 warm-up discarded, 100 measured,
static batch-1 `[1,3,224,224]`): **PyTorch-XPU baseline 3.59 ms / 278 Hz**
(`torch.xpu.synchronize()`-guarded around both warm-up and the timed region,
since XPU kernel launches are asynchronous and an unguarded region would
measure launch overhead only). **OpenVINO: CPU 6.4-6.5 ms / ~154 Hz, GPU
0.59-0.68 ms / ~1.5-1.7 kHz, NPU 1.10-1.28 ms / ~780-910 Hz.** Full
min/median/p95/std table in `docs/hardware/m10-phase4-benchmark.md`.

**Correctness vs the PyTorch-XPU reference, on the same 5 validation samples
ADR-044's mean-collapse check used** (thresholds: CPU/GPU FP32 < 1e-4,
CPU/GPU FP16 < 1e-3, NPU FP16 < 1e-2): CPU FP32 1.043e-07, CPU FP16 9.203e-05,
GPU FP16 1.445e-04, NPU FP16 1.653e-04 -- all within threshold. **GPU FP32
misses its own threshold** (1.445e-04 vs 1e-04, a ~1.4x miss, well under the
brief's >10x flag margin) and reports the exact same deviation value as GPU
FP16. The most likely explanation is that the Arc B390 GPU plugin defaults
its internal compute precision to FP16 regardless of the IR's stored weight
precision (a documented Intel GPU-plugin behaviour, not unique to this
model) -- stated here as a hypothesis about *why*, not a measured cause,
since this script did not override `INFERENCE_PRECISION_HINT` to confirm it
directly. Reported plainly rather than hidden; every other combo is within
its threshold.

**Two things caught during the build, not swept under the checkpoint.**
1. **`ov.save_model`'s `compress_to_fp16` parameter defaults to `True`**
   (`help(ov.save_model)`: "Floating point weights are compressed to FP16 by
   default."). A first pass at the export step omitted the argument for the
   intended-FP32 save and produced a 21.56 MiB `.bin` -- byte-identical to
   the FP16 save, i.e. the "FP32" IR was silently FP16-compressed. Caught by
   checking the file size against the ~43 MiB an 11.3M-param FP32 dump
   implies, fixed by passing `compress_to_fp16=False` explicitly.
2. **The brief's named conversion form was tried and verified failing, not
   assumed to fail.** `ov.convert_model(model, example_input=x,
   input=[("image", [1, 3, 224, 224])])` raised `RuntimeError: Input for
   tensor name 'image' is not found.` against this live PoseNet module on
   openvino 2026.3.1. Fell back to the plain-list form already proven in
   this repo (`scripts/ov_smoke.py:181`, `input=list(INPUT_SHAPE)`), which
   succeeded and is what both IR variants were built from.

**Consequences.** `scripts/posenet_to_openvino.py` is the one committed path
from checkpoint to IR to benchmark table; `artifacts/posenet_ir/` (the IR
files, `results.jsonl`, the 5-sample reference arrays) stays gitignored and
regenerable, per the same "Build artifacts" `.gitignore` block M03's
artifacts already used. The GPU FP32/FP16 precision-hint ambiguity (point 2
of Correctness above) is left open rather than chased further this phase --
confirming it would need explicitly setting and comparing
`INFERENCE_PRECISION_HINT` values, which is not required by this module's
done-when criteria and is noted here for anyone extending this benchmark
later.

**Process note.** Before this module's work began, bm-ptl's repo was one
commit behind (`7dcaaa6`, missing `9999379`'s ARCHITECTURE.md sync) with a
stale local `origin/master` tracking ref falsely reporting "ahead by 103
commits" -- resynced via `fetch` + `reset --hard FETCH_HEAD` (bm-ptl cannot
authenticate a bare `git pull`) before any new work started. Every bm-ptl
command in this module ran in its own foreground SSH invocation, never
detached, per this module's SSH-hygiene instruction.

**Source:** this commit ("M10 Phase 4: PoseNet OpenVINO conversion, benchmark
across CPU/GPU/NPU (ADR-045).") -- code, benchmark doc and this ADR land
together, so there is no separate prior commit to cite the way ADR-044 above
cites `7dcaaa6`.

---

### ADR-046 — M10 Phase 5: PoseNet wired into the controller via cached per-skill inference, opt-in, oracle default -- 3 of 4 skills PASS with perception on, `handoff` FAILS and is reported not tuned; a per-step render-cost bug found and fixed along the way

**Context.** M10 Phase 4 (ADR-045) produced a compiled, benchmarked OpenVINO
IR that nothing in the control loop consumed yet. This module wires it in --
but the four scripted skills back the entire 30-point bimanual-completion
criterion, so the wiring had to be provably non-destructive to the oracle
path before it could be trusted to add a second one. Full measured evidence
lives in `docs/hardware/m10-phase5-integration.md`; this section is the
design record.

**Options considered for the targeting/verification split.**
- (a) Replace every fork/bottle/mug position read with perception,
  uniformly.
- (b) Split by call site: replace only the reads that CHOOSE where to move
  (targeting), leave every read that DECIDES success/failure (verification)
  on oracle, unconditionally, in both modes.

**Decision.** (b). If a skill grades itself with the same noisy estimate it
acted on, success becomes unfalsifiable and a perception bug reads as a
pass. Concretely: `run_pick`'s grasp-point read (`obj_pos0`) and
`run_place`'s destination-offset read (`obj_xy`) take an optional
`position_provider`; `run_pick`'s `final_z`, `run_place`'s `final_pos`, and
`run_handoff`'s `initial_z`/`final_pos` stay direct `env.data.xpos` oracle
reads, unconditionally. `run_open_drawer`'s drawer reads are untouched --
the drawer is outside PoseNet's 3-prop scope (ADR-041) entirely.

**Options considered for a held object's position.**
- (a) Invalidate the cache after `attempt_grasp` succeeds (attach) or
  `release` fires, uniformly, per the task brief's initial proposal.
- (b) Invalidate only on release; have `CachedPropPositions.get()` itself
  refuse to serve a perception estimate for any prop currently held by
  either arm, falling through to oracle instead.

**Decision.** (b). ADR-044 (M10 Phase 3) measured that PoseNet's z output
carries no real signal: every training image shows a prop resting on the
table, so z is a per-prop constant the network memorised. The moment an
arm's weld lifts an object its TRUE z rises well above that constant -- the
water bottle's own prior measurement (ADR-034) goes from a resting z=0.4400
to a held z=0.6192, a ~180 mm rise -- while a FRESH render immediately after
attach would get back the SAME confident resting-height guess, now more
convincing than a stale cached value because it was just computed. Option
(a)'s attach-time invalidation would therefore be actively harmful, not
merely wasteful. Option (b)'s held-object check inside `get()` composes
correctly with all three call sites that use it without any of them needing
to duplicate the check themselves, and makes an attach-time invalidation
unnecessary: a held object never reaches the cache at all, regardless of
when it was last refreshed. Release-time invalidation IS implemented (the
object is back at rest, exactly PoseNet's training distribution).

**Opt-in wiring.** `ScriptedSkillExecutor(inference=None)` -- the default,
and every pre-Phase-5 call site (`tests/test_skills.py`,
`scripts/run_skill.py`) -- is byte-identical to this class's entire
pre-Phase-5 behaviour: no `CachedPropPositions` is ever constructed, the
`cameras=None` assertion is unchanged. Passing a constructed
`PoseNetInference` opts one executor instance into perception for
`pick`/`place`/`handoff` targeting only; the camera-set assertion then
requires exactly `cameras=['posenet_cam']`, and the cache invalidates at the
start of every `execute()` call so one skill never reuses a previous,
different skill's cache generation.

**A real cost bug, found and fixed during this module's own verification,
not assumed away.** `TableSettingEnv(cameras=['posenet_cam'])` sets that
camera as the env's INSTANCE default (ADR-022). `env.step()` with no
`cameras=` argument falls back to that instance default. Every internal
`env.step()` call inside `skills_scripted.py`'s waypoint/dwell helpers was
written under oracle-only conditions (instance default always `None`) and
never overrode this -- harmless there, but once the instance default became
`['posenet_cam']` it meant EVERY physics step of EVERY waypoint rendered,
not just the one render per skill this design intended. Measured: `pick(A,
fork)` alone did not return in 12+ CPU-minutes (confirmed alive, not
deadlocked, via climbing `Get-Process` CPU time) before being killed.
**Fixed** by passing `cameras=[]` explicitly (an empty list, not `None`) at
both `env.step()` call sites -- per `_resolve_cameras`'s own documented
semantics this overrides the instance default for that call only. Verified
to change nothing for oracle mode: `pytest tests/test_skills.py` (4 passed /
4 failed) and `scripts/verify_adr038_skills.py` (all four numbers) both
reproduced their exact pre-fix results afterward.

**Result -- oracle mode: exactly reproduced the ADR-038 baseline.**
`pick(A, fork)` z=0.3989, `place(A, fork, table)` z=0.3588, `pick(A,
'bottle')` z=0.6192, `handoff(A->B, fork)` lateral sep=0.1946 m -- all four
byte-identical.

**Result -- vision mode (GPU FP16): 3 of 4 PASS.** `pick(A, fork)` final
z=0.3905 (8.4 mm from baseline), `place(A, fork, table)` final z=0.3579
(0.9 mm), `pick(A, 'bottle')` final z=0.6286 (9.4 mm) -- all within the
"~10 mm expected noise" this module's task brief anticipated, all PASS.
`place`'s near-zero delta is the direct, measured consequence of the
held-object fallback: its targeting read IS wired to perception, but the
object is always already held by the time it runs, so `get()` serves it
from oracle at runtime every time -- only the upstream pick's few-mm
perturbation survives into its final number. **`handoff(A->B, fork)`
FAILS**: `phase 5 (from_arm retreat) did not clear the 0.1 m gate ...
from_arm_retreat_dist=0.0540 m` (oracle baseline: 0.2263 m). This gate is
computed purely from site positions against a fixed world-frame point -- no
direct perceptual dependency -- yet a 2.2 mm grasp-point perturbation at
Phase 1 (the only perceptual input anywhere in this call) propagates through
`handoff`'s ~13-waypoint sequential choreography (already the most
kinematically fragile skill in this repo per ADR-032 through ADR-038's
five-revision history, several margins on what ADR-037 itself called a
"joint-limit knife-edge") and erodes a 0.2263 m clearance down to 0.0540 m.
**No threshold, gate, or waypoint constant was touched to make this pass.**
A hybrid fallback (scripted `handoff`, perception-driven `pick`/`place`) is
available and left undecided by this ADR.

**Task 5, two more findings, neither a perception regression (both
reproduce in oracle mode).** The brief's literal demo sentence does not
parse under M05's frozen grammar (`hand` alone is not `hand
off`/`handoff`/`pass`/`give`); and grounding the grammar-supported two-clause
form produces `pick` then `handoff` as separate `SkillCall`s, which fails in
sequence because `run_handoff`'s Phase 1 unconditionally re-picks the
object, with no `already_held` branch the way `run_place` has one -- a
pre-existing M06 gap, out of this module's scope. Worked around by grounding
the single grammar-supported clause "Give the fork to arm B." (one
`handoff` call, `from_arm` defaulting to "the other arm") -- PASSED:
`is_holding('A') is None`, `is_holding('B') == 'fork'`. The rendered demo
image does not clearly show which arm ends up holding the fork at its
framing -- stated plainly rather than implied otherwise.

**Consequences.** `bimanual.perception.posenet`'s `import torch` at module
scope meant the new runtime modules (`inference.py`, `cached_access.py`)
cannot import `PROP_ORDER` from it (`ov_env`, the only venv with both
`mujoco` and `openvino`, has no `torch`) -- both duplicate the three
constants instead, documented inline as a disclosed departure with no
automatic sync if `posenet.py`'s values ever change. The GPU shader-cache
warm-up cost (one observed cold compile: 12+ CPU-minutes; a second, warm
compile on the same machine: 0.349 s) is real and environment-dependent,
flagged for anyone deploying to a freshly imaged machine, and not fully
separated in the measurement from the step-render bug investigated
alongside it.

**Source:** this commit ("M10 Phase 5: PoseNet wired into controller via
cached per-skill inference (ADR-046, GPU FP16 runtime).").

---

### ADR-047 — `WeldGrasp.reset()` / `ScriptedSkillExecutor.reset()` fix the cross-trial state-corruption bug the pre-M07/M08 audit found — regression gate (all four ADR-038 skill numbers, `pytest tests/test_skills.py`) reproduced byte-identical on bm-ptl

**Context.** The pre-M07/M08 audit (`docs/hardware/m10-pre-m07-audit.md`,
its Zero-th finding) found, diagnostic-only, that `WeldGrasp.active_welds`
(`src/bimanual/sim/grasp.py:190`) is a plain Python `dict` on the
`WeldGrasp` instance, set by a successful `attempt_grasp` and cleared only
by `release()`. `TableSettingEnv.reset()` (`env.py`) calls
`mujoco.mj_resetData` (and, when a "home" keyframe exists,
`mj_resetDataKeyframe`), which DOES reset MuJoCo's own `data.eq_active` for
every weld back to the compiled model's inactive default -- but
`mj_resetData` only touches the `mujoco.MjData` buffer; it has no way to
reach into a Python object it does not know exists, so `active_welds`
survives a bare `env.reset()` untouched. `ScriptedSkillExecutor._ensure_weld`
(ADR-030) deliberately reuses one `WeldGrasp` across repeated calls against
the SAME `env` -- correct, and load-bearing, for a `TaskPlan` that runs
several skills back-to-back without an intervening reset. But it means: a
caller that reuses one executor across trials/seeds -- exactly the shape
M08's planned `--seeds 0-9` evaluation harness takes -- and resets `env`
directly between them (the "obvious" way to write a seed loop) will desync
the two the first time any trial's grasp succeeds. MuJoCo believes nothing
is welded; `WeldGrasp` still believes an arm holds something. The next
`attempt_grasp` call for that (arm, object) pair is refused at the "already
holds" gate, and the ensuing timeout surfaces to a caller as
`weld_attach_failed_after_N_frames` -- indistinguishable from a genuine
grasp/reachability failure. Left unfixed, this would have silently
corrupted M08's entire 10-seed evaluation the first time it happened, with
no error, exception, or loud signal of any kind -- just a lower success
rate that reads as a real robotics limitation.

**Options considered.**
- (a) Fix it in a hypothetical M08 harness only: construct a fresh `env`
  and `WeldGrasp`/executor per seed, never reuse across a reset. This is
  what the audit's OWN diagnostic script did (out of necessity, since
  production code was frozen for that task) and it does work -- but it
  means every future caller that reuses an executor across a reset (not
  just M08) must independently remember to do the same, with no code-level
  guard against forgetting.
- (b) Fix it at the source: give `WeldGrasp` a `reset()` method that clears
  `active_welds` and the underlying `eq_active` constraints together, and
  give `ScriptedSkillExecutor` a `reset()` that calls `env.reset()` then
  `weld.reset()`, so ANY caller that uses `executor.reset(...)` instead of
  `env.reset(...)` directly is correct by construction, including a future
  M08 harness, without that harness needing its own workaround.

**Decision.** (b). `WeldGrasp.reset()` sets `self.active_welds` back to
`{'A': None, 'B': None}` and, for every pre-declared weld this instance
resolved at construction (`self._eq_ids.values()`), sets
`self.data.eq_active[eq_id] = 0` -- belt-and-suspenders with `env.reset()`'s
own `mj_resetData`, which should already have zeroed every one of these,
but this method clears them itself regardless rather than assuming it.
Per this module's own docstring (`grasp.py`'s header), `WeldGrasp` "only
ever toggles `data.eq_active` and rewrites `model.eq_data`" for constraints
that already exist -- `reset()` preserves that invariant exactly: it never
writes `model.eq_active0` (the model's COMPILED initial value, which would
persist across every future reset -- a new, unwanted side effect this
module has deliberately never had) and never touches `model.eq_data` (no
relative pose needs restating; the constraint is simply inactive).
`ScriptedSkillExecutor.reset(env, seed=0, cameras=None)` calls
`env.reset(seed=seed, cameras=cameras)`, then, ONLY if `self.weld` is
currently bound to that same `env` instance (mirroring `_ensure_weld`'s own
rebuild-on-different-`env` rule), calls `self.weld.reset()`. `env.reset()`'s
own signature and return value (the obs dict) are forwarded verbatim --
this method adds nothing to that contract, it only adds the `WeldGrasp`
clear-up after it.

**A cached-reference risk, checked rather than assumed.** `WeldGrasp.__init__`
caches `self.data = env.data` once, at construction (`grasp.py:186`), so
`WeldGrasp.reset()`'s and `ScriptedSkillExecutor.reset()`'s correctness both
depend on that cached reference staying the SAME live buffer `env.reset()`
mutates. Checked directly: `TableSettingEnv.__init__` assigns `self.data =
mujoco.MjData(self.model)` exactly once (`env.py:133`); `reset()` never
reassigns `self.data` to a new object anywhere in its body, only mutates
the existing buffer in place via `mujoco.mj_resetData` (and, conditionally,
`mj_resetDataKeyframe`) followed by `mj_forward`. No rebind occurs, so the
cached reference stays valid across every `env.reset()` call -- confirmed,
not assumed, and reported here per the task's own instruction to check and
report this specifically.

**Verification (bm-ptl, `C:\Users\devcloud\project\ov_env\Scripts\python.exe`).**
`scripts/probe_weld_reset.py`, one `TableSettingEnv` + one
`ScriptedSkillExecutor` reused across resets (the exact reuse pattern that
triggers the bug), in a single process:
1. `pick(A, fork)` succeeds: `final_z=0.3989`.
2. **Bug demonstrated**: `env.reset(seed=0, cameras=None)` called DIRECTLY
   (bypassing `executor.reset()`, i.e. exactly what every pre-ADR-047
   caller did) -- `weld.is_holding('A')` still reports `'fork'` (stale)
   while `env.data.eq_active[fork_eq_id]` is already `0` (MuJoCo's own
   reset, correctly cleared, independent of this fix). A retried `pick(A,
   fork)` through the SAME executor then fails with
   `weld_attach_failed_after_300_frames` -- the exact symptom the audit
   described, reproduced on demand.
3. **Fix demonstrated**: `executor.reset(env, seed=0, cameras=None)`
   (ADR-047) instead -- `is_holding('A') is None`,
   `data.eq_active[fork_eq_id] == 0`.
4. `pick(A, fork)` re-run through the SAME executor succeeds again, with
   `final_z` bit-identical to step 1's (`0.398949 == 0.398949`) -- the real
   proof that the corruption is fully gone, not merely that a later call
   happens to succeed.

**Regression gate, all four numbers reproduced byte-identical to the
pre-fix baseline, on bm-ptl.** `scripts/verify_adr038_skills.py`: `pick(A,
fork)` 0.3560 -> 0.3989, `place(A, fork, table)` final z=0.3588, `pick(A,
'bottle')` 0.4400 -> 0.6192, `handoff(A->B, fork)` lateral
separation=0.1946 m -- every digit identical before and after this change.
`pytest tests/test_skills.py`: 4 passed / 4 failed, before and after,
same four failure reasons and residuals. This match was not a coincidence
to be relieved about: both scripts construct a FRESH `env` (and, for
`verify_adr038_skills.py`, a fresh `WeldGrasp`; for `test_skills.py`, a
fresh `ScriptedSkillExecutor` per test via its own fixture) for every
skill/test and never reuse one across a `reset()` call -- neither exercises
the reuse-across-reset path this fix addresses at all, and `grasp.py`'s and
`executor.py`'s pre-existing methods (`attempt_grasp`, `release`,
`is_holding`, `execute`, `_dispatch`) were not modified, only new methods
added. An exact match was therefore the correctly-predicted outcome of a
purely additive change, confirmed rather than assumed.

**A cross-machine floating-point finding, reported rather than silently
absorbed.** The same two regression checks, run on the Windows dev laptop
(`mujoco==3.2.7`, the same pinned version), reproduce the identical
pass/fail STRUCTURE but not the identical floating-point digits -- e.g.
`pick(A, fork)` final_z=0.3987 there vs bm-ptl's 0.3989, `handoff` lateral
separation=0.1958 m there vs bm-ptl's 0.1946 m. Confirmed present on the
laptop with the UNMODIFIED pre-fix code too (checked before making any
edit), so this is a pre-existing, unrelated cross-machine floating-point
divergence -- almost certainly contact-solver iteration-order drift
compounding over the ~1000-6600 physics steps each skill takes on a
different CPU -- not a consequence of this fix. The authoritative numbers
cited above are bm-ptl's, matching the machine the original ADR-038
baseline was measured on (`docs/hardware/m10-pre-m07-audit.md:15-18`
records that all of that audit's live-execution measurement ran on bm-ptl
for the same reason).

**Consequences.** M08's evaluation harness (not yet built) must call
`executor.reset(env, seed=..., cameras=...)` instead of `env.reset(...)`
directly whenever it reuses one executor across seeds -- this ADR makes
that the documented, tested, and now code-supported way to do it, closing
the audit's own recommendation ("either construct a fresh `env`/executor
per seed ... or have `WeldGrasp` itself clear `active_welds` on
`env.reset()`") in favour of the second option, at the source, rather than
requiring every future multi-trial caller to remember the first one
independently. `CachedPropPositions`' (M10, ADR-046) own per-trial cache
staleness, if any, is explicitly out of this ADR's scope -- untouched,
unexamined here, and not claimed to be fixed.

**Source:** this commit ("M08 prep: WeldGrasp.reset() fixes cross-trial
state corruption (ADR-047).").

---

### ADR-048 — M07: fine-grid placement envelopes (7x7, 1 cm step, +/-3 cm), opt-in `ScenarioRandomizer` — measured envelopes for `fork`/`water_bottle` both collapse to a single point once `handoff`'s cross-prop fragility is included, so the shipped randomizer intentionally randomizes nothing this pass

**Context.** PLAN.md's M07 calls for randomizing initial object placement
(brief p2 objective 3, 15 rubric points, `CONSTRAINTS.md:33`). The pre-M07/
M08 audit (`docs/hardware/m10-pre-m07-audit.md`) had already found, at a
coarse ±5 cm jitter, that all four ADR-038-gated skills tolerate essentially
zero placement perturbation (0/40 aggregate), with a real but narrow
(~1 cm x 0.5 cm, asymmetric) band for `pick(A, fork)` visible only at finer
resolution. This module measures that band properly and builds a
randomizer scoped to whatever the measurement actually supports, rather
than to the brief's un-measured "modest range" framing.

**Decision 1 — randomization is opt-in, default OFF.**
`scripts/verify_adr038_skills.py:19` and `tests/test_skills.py:76` both
depend on `env.reset(seed=N)` producing the FIXED baseline scene (the four
ADR-038 numbers; the 4-passed/4-failed pytest baseline). `TableSettingEnv.
reset()` gained a new `randomizer=None` argument; omitted, it is
byte-identical to pre-M07 behaviour for every `seed` — the same
default-OFF shape ADR-046 established for perception
(`ScriptedSkillExecutor(inference=None)`). When given a
`ScenarioRandomizer`-shaped object, `reset()` applies its
`randomize(seed) -> {prop: (dx, dy)}` return value to each named prop's
free-joint `qpos` (x, y only) via a direct write plus one extra
`mj_forward`, AFTER the deterministic "home"-keyframe reset — the same
runtime-qpos mechanism `scripts/generate_posenet_data.py` and the pre-M07
audit's own probe already used, never scene-XML regeneration (ADR-038
found that breaks `handoff`).

**Decision 2 — measurement design.** `scripts/probe_envelope.py sweep`:
7x7 grid, 1 cm step, ±3 cm range, around each of `fork`'s and
`water_bottle`'s own default (x, y), for the four skills
`pick(A, fork)`, `place(A, fork, table)`, `handoff(B, A, fork)`,
`pick(A, 'bottle')`. 5 fresh-env-fresh-executor reps per cell (seeds 0-4;
every non-rejected cell reported exactly 0/5 or 5/5, never 1-4/5 — a live
regression check against the ADR-047 state-leak bug class, not a real
randomization axis, since nothing on this code path consumes any RNG given
a fixed pose). A candidate cell is REJECTED (not scored as a failure) if
off-table (checked against `gen_dual_scene.py`'s declared table half-
extents) or if it produces a real MuJoCo contact penetration deeper than
2 mm against another prop (post-`mj_forward` collision detection —
deliberately NOT `generate_posenet_data.py`'s own bounding-circle
heuristic, which is a render-legibility check that would incorrectly
reject cells the shipped scene's own default already occupies). Measured
result (full grids in `docs/hardware/m07-envelopes.md`): `pick_fork`
4/29 passed (`dx in [-0.010,+0.020]`, `dy=0`); `place_fork` 2/29
(`dx in [-0.010,0]`, `dy=0`, narrower than `pick` as expected); `handoff`
**1/29 — a single point, the unperturbed default itself**; `pick_bottle`
8/49, scattered, with a usable 3-cell rectangle (`dx=0`,
`dy in [-0.010,+0.010]`).

**A harness near-miss, caught and corrected before being reported as a
finding.** An early-stop attempt on `handoff`'s sweep (this task's own
instructions permit stopping early on a plausibly-empty envelope) stopped
after its first two rows came back all-fail — without ever reaching
`dx=0`, the point `verify_adr038_skills.py` already proves succeeds with a
0.2263 m margin. Caught before write-up; the sweep was re-run in full and
found the correct single-cell result. Recorded as a methodology lesson:
order any future early-stopping sweep so the flag cannot fire before a
known-good centre point has been tested.

**Decision 3 — per-prop scoping is an intersection across every skill that
CONSTRAINS a prop, not just skills that directly TARGET it.**
`ScenarioRandomizer.ENVELOPES[prop]` starts as the rectangle intersection
across every skill whose OWN target object is that prop (`fork`: `pick`,
`place`, `handoff` — intersects to the single point `(0,0)`; `water_bottle`:
`pick_bottle` only — `dx=0`, `dy in [-0.010,+0.010]`, real and
non-degenerate on this first cut). **Task 3 (seeds 0-9, randomizer opted
in) found this first cut insufficient**: with ONLY `water_bottle`
randomized (fork untouched, confirmed bit-identical initial `qpos` across
all 10 seeds), `handoff` — which never targets `water_bottle` — failed
0/10, byte-identical residual and frame count every seed. Probed directly
at `dy` in `{±0.0001, ±0.001, ±0.005}` m: every nonzero offset reproduces
the identical Phase-3 (cross-arm approach) collision failure; exactly
`0.0` reproduces the true baseline bit-for-bit. A genuine binary cliff, not
a smoothly shrinkable range — MuJoCo recomputes the whole system's
contacts every step, so a prop with zero direct geometric proximity to a
maneuver can still perturb an already-"joint-limit knife-edge" (ADR-037's
own term) collision check elsewhere in the scene. Since a global,
skill-agnostic `reset()` hook cannot know which skill will run next, and
`handoff` is one of the four ADR-038-gated skills, this was folded back in
as an explicit cross-prop COMPATIBILITY entry,
`SKILL_ENVELOPES["handoff"]["water_bottle"] = (0,0,0,0)`, intersected the
same way as every direct-target entry.

**Result: both `fork`'s and `water_bottle`'s final intersections are
single points (zero width on both axes).** `_is_degenerate_point` excludes
both from `ENVELOPES` rather than offering a fake always-zero
`rng.uniform(0,0)` "range" — the same "say so and exclude, don't silently
emit a zero-width range" rule this task specified for a geometrically empty
intersection, extended to this degenerate-but-technically-non-empty case.
`plate`/`mug`/`spoon` are absent from `ENVELOPES` for the separate, simpler
reason that no measured skill targets them at all. **Net effect: the
shipped `ScenarioRandomizer` randomizes nothing** (`ENVELOPES = {}`) —
verified by re-running `randomized_eval` for all four skills against the
shipped randomizer: 10/10 each, `mean_frames_on_success` bit-identical to
the unperturbed baseline's own `frames_used` for every skill.

**A second, non-blocking chaos finding, recorded for a future harness's
benefit.** Before the cross-prop fix, `place_fork`'s Task-3 round (8/10,
`water_bottle`-only randomization) had 2 failures traced to global
floating-point coupling: `fork`'s own initial `qpos` is bit-identical
across all 10 seeds and the SAME seed reproduces the SAME outcome
deterministically, yet different (far-away) `water_bottle` positions
measurably shifted where the nested pick ended up gripping the fork over a
3455-step rollout, occasionally landing a later waypoint in collision with
`mug`. 80% clears this task's own 60% floor (no remediation required by
that rule), but it is a real example of "a prop excluded from randomization
is not fully insulated from another randomized prop's effect on its own
skill" — relevant to any future M08 harness reasoning about which props are
"safe."

**Verification (bm-ptl, `ov_env`).** `verify_adr038_skills.py` and `pytest
tests/test_skills.py` both reproduced byte-identical to the ADR-047
baseline throughout this commit's changes (including after `env.py`'s
`reset()` signature changed and after `ENVELOPES`'s computation changed a
second time, mid-investigation, to fold in the cross-prop constraint).

**Consequences.** M08's future evaluation harness gets a
`ScenarioRandomizer` that is honest about its own current scope (a
documented no-op) rather than one that silently ships a degenerate or
unsafe range. Real randomization range, if wanted later, requires either
fixing `handoff`'s underlying fragility (Phase 1's nested-pick tolerance
and/or Phase 3's cross-arm corridor) or building a skill-aware randomizer
(out of this pass's scope — `env.py`'s `reset()` hook is necessarily
skill-agnostic, matching how a real seed loop randomizes a scenario before
choosing which skill to run against it).

**Source:** this commit ("M07: fine-grid envelope measurement, deterministic
randomization, preliminary per-skill robustness (ADR-048).").

---

### ADR-049 — M08: two-track 10-seed robustness eval — own-prop randomization (Track A) vs. M07 Round 1's multi-prop randomization (Track B); `handoff`'s Track A 10/10 is disclosed as a degenerate measurement, not a robustness result

**Context.** M08's brief (PLAN.md, as re-scoped after M07/ADR-048) calls for
randomizing only the skill's own target prop and evaluating over seeds 0-9.
Applied literally, that method cannot reproduce M07's own Round 1 numbers
(`docs/hardware/m07-envelopes.md`'s Task 3), most importantly `handoff`'s
0/10 — because Round 1 randomized `water_bottle` (the only prop with a
non-degenerate individual envelope) underneath **every** skill, including
`handoff`, which never targets it. `handoff`'s own measured fork envelope
(`m07-envelopes.md:163`) is `dx=0, dy=0` — a single point — so an own-prop-
only method draws the identical zero offset on every seed and scores
`handoff` 10/10 by construction, which is not evidence of robustness, only
of determinism.

**Decision.** Run and report both methods, explicitly labelled, never
collapsed into one table:
- **Track A — own-prop randomization** (this module's own brief). Only the
  skill's own target prop moves, drawn from that skill's own
  `SKILL_ENVELOPES` rectangle (`randomization.py`). A degenerate
  (single-point) rectangle is used AS-IS, not filtered out — the point of
  `handoff`'s Track A row is to show the degeneracy, not hide it.
- **Track B — multi-prop randomization** (M07 Round 1's method,
  reconstructed exactly: `water_bottle: (0, 0, -0.010, +0.010)`, `fork`
  excluded — applied underneath all four skills' ten seeds regardless of
  target).

Every mention of Track A's `handoff` result — in the results table, the
summary prose, and this ADR — carries the qualifier: **"envelope is a
single point, zero displacement applied; this measures determinism, not
robustness."** No mention of that number appears without it.

**Options considered for where the envelopes come from.**
- (a) Re-measure envelopes from scratch for this module.
- (b) Read `SKILL_ENVELOPES` directly out of `randomization.py` (already
  measured and committed by ADR-048) and construct per-track
  `ScenarioRandomizer(envelopes={...})` instances locally in a new
  `scripts/eval_m08.py`, never touching `randomization.py` itself.

**Decision.** (b). `randomization.py`'s own module-level `ENVELOPES` export
is the cross-skill-safe, degenerate-excluding default and must stay `{}`
(ADR-048's own finding, and this task's explicit instruction to leave it
so) — it is not what either track of this evaluation uses.
`ScenarioRandomizer.__init__` already accepts an explicit `envelopes=`
argument for exactly this kind of caller; using it is additive, not a
change to the shipped class or its default behaviour.

**Results (bm-ptl, seeds 0-9, oracle mode, fresh `TableSettingEnv` + fresh
`ScriptedSkillExecutor` per trial per ADR-047). Full per-seed offsets,
failure reasons and frame counts in `docs/hardware/m08-eval.md`.**

| skill | Track A (own-prop) | Track B (multi-prop, M07 Round 1's method) |
|---|---|---|
| `pick(A, fork)` | 10/10 | 10/10 |
| `place(A, fork, table)` | 10/10 | 8/10 (seeds 5, 8 fail — mug collision via floating-point coupling) |
| `handoff(B, A, fork)` | **10/10 — degenerate, zero displacement, not a robustness result** | **0/10 — every seed, identical failure reason and frame count** |
| `pick(A, 'bottle')` | 6/10 (seeds 0,1,2,4 fail) | 6/10 (identical seeds, offsets and outcomes to Track A — expected: `water_bottle` is this skill's own target in both tracks) |

Track B's numbers reproduce M07 Round 1's historical report
(`docs/hardware/m07-envelopes.md`'s Task 3 Round 1: 10/10, 8/10, 0/10, 6/10,
same failing seeds for `place_fork`) on a fresh run, today, confirming
Round 1 was not a one-off artifact.

**Cross-prop coupling — an architectural constraint, not a bug to schedule
a fix for.** Two independent findings agree: ADR-038 (moving the mug alone,
at a scene-composition level, broke `handoff` at phase 3 with the fork
itself untouched) and M07's Commit 2 (probed to 0.1 mm resolution: any
nonzero `water_bottle` offset reproduces the identical phase-3 failure,
only exactly zero reproduces baseline). This document's own Track B run is
a third, independent reproduction of the same cliff. The mechanism is
MuJoCo's whole-system contact/force recomputation at every step: a prop's
position anywhere in the scene can perturb floating-point rounding through
a long closed-loop rollout enough to tip an already-marginal collision
check (`handoff`'s cross-arm corridor, ADR-037's "knife-edge") the wrong
way, with zero geometric proximity required. This is *why* a single,
skill-agnostic randomizer's cross-skill-safe intersection collapses to
empty (ADR-048): safety for `handoff` requires tolerance to every prop's
position, not just its own target's, and the measured tolerance for
`water_bottle` under that requirement is a single point.

**Consequence for how these two tracks should be read.** Track A answers
"how much can a skill's own target move." Track B answers "how much can
anything else in the scene move before this skill breaks." They are not
two measurements of the same quantity at different rigor — they are
answers to different questions, and `handoff` is the clearest case where
the two disagree completely (10/10 vs 0/10). Any future consumer of this
evaluation (README, demo narration, submission writeup) must cite both
numbers together for `handoff`, never Track A alone.

**Interpretation, stated plainly.** Every PASS in this module was measured
inside envelopes on the order of ±10-20 mm, tuned specifically to what
these four skills already tolerate. The pre-M07/M08 audit measured 0/40
aggregate at a coarse ±50 mm jitter. This module is an honest measurement
of robustness within a narrow, specifically-tuned band — not a general
robustness claim, and `docs/hardware/m08-eval.md` says so in those words.

**Regression gates, re-verified on bm-ptl before and after this module's
run.** `scripts/verify_adr038_skills.py`: `0.3989 / 0.3588 / 0.6192 /
0.1946`, unchanged. `pytest tests/test_skills.py`: 4 passed / 4 failed,
same four tests and reasons as ADR-047/ADR-048.

**Source:** this commit ("M08: 10-seed robustness eval, per-skill success
rates within individual envelopes (ADR-049).").

---

### ADR-050 — M10 Phase 4 extension: INT8 PoseNet quantization via NNCF, benchmarked CPU/iGPU/NPU — 4x smaller than FP32 and 2-4x faster than FP16/FP32, but deviation (36-37 mm) is an order of magnitude above the model's own ground-truth MAE (2.6-3.2 mm)

**Context.** ADR-013's original precision/device-mapping decision said INT8
via NNCF post-training quantization "targets the NPU5010." ADR-045 (M10
Phase 4) benchmarked FP32 and FP16 only, leaving INT8 as a deferred row.
This module's task brief marks INT8 explicitly as a bonus, not critical
path: "if it fails, STOP and report rather than fighting it." It did not
fail, so this ADR records what was measured.

**Options.**
- (a) Regenerate the FP32 IR fresh before quantizing, in case Phase 4's
  artifact had drifted since ADR-045.
- (b) Quantize the EXISTING, already-verified FP32 IR
  (`artifacts/posenet_ir/posenet_fp32.xml`) with NNCF, calibrated on a
  freshly-sampled subset of M09a's images, and reuse Phase 4's own 5-sample
  PyTorch-XPU reference predictions (`val_images.npy` /
  `val_predictions_xpu.npy`) for the correctness check rather than
  recomputing them.

**Decision.** (b). Before any new work started, the FP32/FP16 `.bin` sizes
on bm-ptl were independently re-verified byte-for-byte against ADR-045's own
numbers (45,221,444 / 22,610,738 bytes) -- confirming the artifact this
module quantizes is the exact one ADR-045 benchmarked, not a re-derived
one. Regenerating it would have risked silently producing a different graph
than what Phase 4's FP32/FP16 rows describe.

**Dependency install, `--no-deps` first, then only what import actually
needed.** `pip install --no-deps nncf` installed nncf 3.3.0 itself but
`import nncf` then failed on a missing transitive import. Rather than
re-resolving NNCF's full declared dependency tree (which pulls scipy,
scikit-learn, pydot, ninja, rich and more -- several of which could
plausibly want to bump numpy or another pinned package), each missing
import was added ONE PACKAGE AT A TIME, each also with `--no-deps`, retrying
`import nncf` after each addition and stopping the moment it succeeded:
`packaging`, `rich`, `tabulate`, `psutil`, `safetensors`, `scipy` -- 6
packages, `import nncf` succeeded after `scipy`. `pip show nncf` also lists
`ninja`, `pydot` and `scikit-learn` as declared requirements, and `pip
check` correctly flags all three as missing (plus `rich`'s own
`markdown-it-py`/`pygments`) -- none of the three was needed by `import
nncf` or by a live `nncf.quantize(...)` smoke test against a trivial
OpenVINO model (the same MinMax-statistics + Fast-Bias-Correction algorithm
path the real quantization uses), so none is installed. `torch.__version__`
(`2.14.0+xpu`), `torch.xpu.is_available()` (`True`) and `numpy.__version__`
(`2.4.6`) were verified unchanged before and after every one of the 7
install steps (nncf itself plus the 6 additions) -- per this module's task
brief, any install that changed torch or numpy would have been an
immediate abort-and-report, and none did. `ov_env`
(`scripts/requirements-bmptl.txt`) was never touched; NNCF lives only in
`train_env`, recorded in `scripts/requirements-train.txt` (the file ADR-043
created for exactly this purpose).

**Calibration.** 300 images sampled without replacement from
`data/posenet/images/` (5,000 available), `numpy.random.default_rng(seed=42)`,
preprocessed identically to training (224x224 RGB -> float32 `[0,1]` -> CHW,
no mean/std normalization, matching `PoseNetDataset.__getitem__`). The exact
`sample_index` list drawn is recorded in
`artifacts/posenet_ir/int8_calibration_info.json` for reproducibility.
`nncf.quantize(ov_model, calib_dataset, subset_size=300,
target_device=nncf.TargetDevice.NPU)` per ADR-013's stated NPU target --
the produced IR is still a generic OpenVINO IR and was benchmarked on
CPU/GPU/NPU identically to the FP32/FP16 IRs, not restricted to NPU.

**Result — size.** INT8 `.bin` is 10.82 MiB, 0.251x FP32's 43.13 MiB and
0.502x FP16's 21.56 MiB -- almost exactly the 0.25x an INT8-vs-FP32
bit-width ratio predicts.

**Result — latency (10 warm-up discarded, 100 measured, static batch-1
`[1,3,224,224]`, identical methodology to Phase 4).** CPU 1.616 ms / 619 Hz
(vs FP32/FP16's 6.4-6.5 ms), GPU 0.340 ms / 2939 Hz (vs FP32/FP16's
0.59-0.68 ms), NPU 0.903 ms / 1107 Hz (vs FP32/FP16's 1.10-1.28 ms). All
three devices compiled and ran INT8 without a crash; the subprocess-per-
device isolation (ADR-013's M03 lesson, restated in ADR-045) was exercised
on every combo and never triggered by a crash this run.

**Result — correctness, and the honest finding this ADR exists to record.**
Max absolute deviation vs the PyTorch-XPU reference, same 5 validation
samples and same reference array Phase 4 used: CPU 36.600 mm, GPU 37.499 mm,
NPU 36.391 mm. This is roughly two orders of magnitude larger than FP16's
deviation (~1.7e-4 m = 0.17 mm) and is NOT small relative to what matters
for control: PoseNet's own ground-truth MAE is 2.6-3.2 mm per prop, so an
INT8 deviation of ~36-37 mm is **an order of magnitude above the model's own
error scale**, not "well under" it. Judged against that scale rather than as
a bare number (this module's task brief's own instruction), this INT8
quantization is **not** a free win the way FP16 was -- it is markedly less
accurate, in a way that would be material to control if this IR were
actually driving a skill, not just a benchmark row. This is plausible and
not evidence of a bug: PoseNet's head regresses precise 3D millimetre-scale
coordinates rather than a classification logit, and naive INT8 post-training
quantization is well known to degrade regression heads far more than
classification heads, which tolerate coarser activation quantization because
only the argmax needs to survive. Consistency across all three devices
(36.4-37.5 mm, not wildly different from each other) supports "the
quantization itself is imprecise for this task" over "a device-specific
bug."

**On GPU INT8 vs GPU FP16 specifically.** ADR-045 found GPU FP32 and GPU
FP16 report byte-identical deviation and near-identical latency, consistent
with the Arc B390 plugin running its internal compute in FP16 regardless of
the IR's stored weight precision. If that holds, comparing GPU INT8 against
GPU FP16 here may be comparing INT8 against an already-FP16-internal
baseline rather than against a genuinely higher-precision one -- flagged as
a caveat on the GPU row, not a claim this script verifies (would require
overriding `INFERENCE_PRECISION_HINT` directly, out of scope here).

**Per-device recommendation.** NPU is ADR-013's originally intended INT8
target and is the fastest of the three at INT8 while being competitively
fast even against FP16's NPU row (0.903 ms vs 1.10-1.28 ms); GPU is
fastest overall at INT8 (0.340 ms). But given the ~36-37 mm deviation on all
three, **none of the three is recommended as the demo's perception backend
in its current form** -- the demo path stays on FP16 (ADR-046's GPU FP16
runtime), and this INT8 IR is reported as a documented benchmark artifact,
not adopted. A future pass could try excluding the regression head from
quantization (`ignored_scope`) or a larger/more representative calibration
set before reconsidering INT8 for the control loop; neither was attempted
here, since this module's task brief scoped it as calibration-and-measure,
not as an accuracy-recovery exercise.

**Consequences.** `scripts/quantize_posenet.py` is the committed path from
the existing FP32 IR to the INT8 IR and its benchmark rows;
`artifacts/posenet_ir/posenet_int8.{xml,bin}`,
`int8_calibration_info.json` and `int8_convert_info.json` stay gitignored
and regenerable, in the same `.gitignore` block Phase 4's artifacts already
use. `docs/hardware/m10-phase4-benchmark.md` gained three new table rows
(CPU/GPU/NPU x INT8) and a new "INT8 quantization" section; Phase 4's own
table rows, findings and prose are unchanged.

**Process note.** bm-ptl's repo was already at the same commit as origin
(`98a0a25`) when this module began, with the same stale-tracking-ref pattern
ADR-045 already documented (a local `origin/master` ref reporting a false
"ahead by N commits" despite `git log` matching exactly) -- re-verified, not
re-fixed, since it does not block work. Every bm-ptl command in this module
ran in its own foreground SSH invocation, never detached, per this module's
SSH-hygiene instruction.

**Source:** this commit ("M10 Phase 4 extension: INT8 PoseNet quantization
across CPU/iGPU/NPU (ADR-050).") -- code, benchmark doc and this ADR land
together.

---

### ADR-051 — M06 handoff perturbation diagnostic: home-pose arm-angle noise is not tolerated at any tested magnitude (0/5 at +/-0.005 rad) — no robustness claim beyond exact determinism is made for `handoff`

**Context.** M08 (ADR-049) reported `handoff(B, A, fork)` Track A 10/10, but
disclosed that result as degenerate: `handoff`'s own placement envelope is a
single point, so all ten trials were the identical deterministic scenario —
that number measures determinism, not robustness. This diagnostic asks
whether `handoff` tolerates a genuinely different kind of small perturbation:
noise on each arm's HOME pose (`shoulder_lift`, `elbow_flex`, both arms)
immediately after `env.reset()`, before the skill starts moving. **Diagnostic
only — `skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`randomization.py`, `scenes/so101/`, `gen_dual_scene.py` were not touched.**

**A physics-timestep sweep was proposed alongside this and deliberately NOT
run.** No `timestep` is set anywhere in the scene XML or `gen_dual_scene.py`,
so the model runs at MuJoCo's default (0.002 s), and every skill's step
budget in this repo is a FRAME count tuned at that default (`handoff`'s
6610 frames = 13.2 s at dt=0.002). Varying `dt` without rescaling the frame
budget does not perturb the skill physically — it changes how much simulated
time the same frame budget buys, plus integrator accuracy and contact
resolution — and a near-certain failure there would only re-confirm "this
skill is frame-budget-tuned for dt=0.002," which ADR-046/ADR-049 already
establish, dressed up as a robustness result. Skipped for that reason, not
for lack of time in the diagnostic's 60-minute cap.

**Method.** 5 seeds (0-4), fresh `TableSettingEnv` + fresh `WeldGrasp(env)`
per trial (ADR-047 reuse hazard), noise drawn per (magnitude, seed) from an
independent `numpy.random.default_rng`, applied to `env.data.qpos` at each
of the four joints' own `jnt_qposadr` slot, followed by `mujoco.mj_forward`
(same re-derivation step `env.py`'s own randomizer branch already uses after
perturbing a prop's qpos), then called receiver-first exactly as
`scripts/verify_adr038_skills.py`'s own established direct-call pattern:
`run_handoff(env, "B", "A", "fork", weld=weld)`. Widens 0.005 -> 0.01 -> 0.02
rad only if the previous magnitude passes all 5 seeds. Full script:
`scripts/probe_handoff_perturbation.py`; full per-trial results and the
zero-noise sanity control: `docs/hardware/m10-handoff-perturbation.md`.

**Result.** A zero-noise control (same direct-call harness, 5 seeds)
reproduced the known oracle baseline exactly: `success=True`, 6610 frames,
`from_arm_retreat_dist=0.2263 m` every time (ADR-046's own oracle number),
confirming the harness is sound. At the SMALLEST tested magnitude
(+/-0.005 rad, roughly +/-0.3 degrees on two joints per arm), **0/5 seeds
passed**, and the ladder therefore did not widen to 0.01 or 0.02 — the
breaking point is at or below 0.005 rad. All 5 failures were, to four
decimal places, the IDENTICAL failure: Phase 3 (`to_arm` approach),
IK residual 0.0875 m, 3155 frames, `cross_arm contacts=1` — despite five
different random noise vectors (both sign and magnitude varying per seed).
This is the same signature ADR-049 documented for `water_bottle` placement
("any nonzero offset reproduces the identical phase-3 failure, only exactly
zero reproduces baseline") — evidence that `handoff`'s Phase 3 cross-arm
corridor (ADR-037's "joint-limit knife-edge") is a cliff with respect to
home-pose noise too, not merely to prop placement, and not a harness bug
(the matched-pattern zero-noise control rules that out).

**Decision, on the interpretation bar fixed before the run (>10/15 supports
a robustness claim in the demo video, <10/15 means none is made): result is
0/5 (0/15 of the full possible grid, since widening never triggered) — well
under the bar. No robustness claim is made for `handoff` beyond the exact
determinism ADR-049 already disclosed.** The demo video and any submission
prose may state that `handoff` reproduces deterministically at its exact
tuned configuration; it must not claim tolerance to arm-pose noise, prop
placement beyond a single point (ADR-049), or any other perturbation class
not separately measured and passing.

**Regression gate, re-verified on bm-ptl before this diagnostic's run:**
`pytest tests/test_skills.py` reproduced `4 passed / 4 failed`, same four
tests as ADR-047/048/049 (`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`).

**Source:** this commit ("M06 handoff: perturbation robustness measurement
(diagnostic, no logic change).").

---

### ADR-052 — M10 batch scaling: PoseNet FP16 throughput vs. batch (1/4/8/16) across CPU/iGPU/NPU via static `reshape()` on the existing IR — all 12 combos succeeded, including NPU at every batch size, contradicting the predicted destructive-crash trigger class

**Context.** M10 Phase 4 (ADR-045) benchmarked PoseNet's FP32/FP16 IR at a
STATIC batch-1 shape only. This module asks whether FP16 throughput scales
linearly, sub-linearly or super-linearly with batch size (1, 4, 8, 16) on
CPU, iGPU and NPU, and — the higher-risk half of the question — whether the
NPU tolerates a batch dimension above 1 at all. Provenance: all numbers
from bm-ptl, per ADR-047's cross-machine float-divergence record; laptop
figures are not reported.

**Reshape, not reconversion.** `artifacts/posenet_ir/posenet_fp16.xml`
(ADR-045) stays on disk exactly as Phase 4 produced it — static batch-1.
`scripts/benchmark_batch_scaling.py` obtains each batch size fresh, per
(device, batch) combo, via `core.read_model(...)` then
`model.reshape({0: [N, 3, 224, 224]})` **before** `compile_model`. No new
`.xml`/`.bin` pair was written. 10 warm-up inferences discarded, 100
measured per combo (the 90-minute cap was never approached, so the
50-iteration time-cap fallback this module's task brief allowed was never
needed). Throughput reported as `batch * 1000 / mean_ms`.

**Subprocess isolation carried forward from M03/ADR-045, even though it
was not needed this time.** `DECISIONS.md`'s "M03 — OpenVINO conversion
smoke test complete" entry records that the NPU plugin can kill the whole
process (`STATUS_ACCESS_VIOLATION` / `0xC0000005`) on an unsupported graph
rather than raising a catchable exception, and that a harness which
buffers results in memory loses everything already earned when that
happens. (Note for readers: that exact quote lives in `DECISIONS.md`, not
verbatim in `docs/hardware/bmptl-verification.md` — checked directly
against both files while writing this entry.) This module's task brief
named batch>1 as "exactly the trigger class" for a repeat of that failure.
Every (device, batch) combo therefore ran in its own `subprocess.run(...)`,
device order CPU → GPU → NPU, batches ascending within a device, with
every result — success, reshape failure, compile failure, or a dead child
with no result at all — appended to `artifacts/posenet_ir/
batch_scaling_results.jsonl` the instant it was known, and an
`NPU_ONLY_BATCH_1`-tagged skip path wired in for any larger NPU batch after
a first NPU batch>1 failure (never triggered this run, since none failed).

**Result — the predicted crash did not occur.** All 12 (device, batch)
combos compiled and ran successfully, including NPU at batch 4, 8 and 16.
This does not contradict M03: M03's crash was specifically on a
FULLY-OPEN dynamic batch dimension (`-1`, unbounded upper bound) — the
diagnostic named `Upper bounds are not specified for node 'Multiply_11422'
... bounds are '[9223372036854775807, 3, 224, 224]'`. A static reshape to a
fixed N is a narrower, different case, and on this
NPU5010/driver/OpenVINO-2026.3.1 combination it is tolerated at every N
tested here. The subprocess-per-combo/incremental-write discipline was
exercised on every row but never actually triggered by a crash — the same
posture ADR-045 recorded when none of its own six combos crashed either.

**Scaling shape and the numbers, anchored against Phase 4's batch-1 rows
(GPU FP16 0.683 ms / 1465 Hz, NPU FP16 1.099 ms / 910 Hz, CPU FP16 6.492 ms
/ 154 Hz).** All three devices scale sub-linearly-to-linearly in latency
vs. batch (latency grows slower than batch size), so throughput keeps
climbing through batch 16 on every device:

| Device | Batch 1 mean / throughput | Batch 16 mean / throughput | Throughput ratio |
|---|---:|---:|---:|
| CPU | 6.3828 ms / 156.7 Hz | 69.2524 ms / 231.0 Hz | 1.47x |
| GPU | 0.5893 ms / 1697.0 Hz | 2.7584 ms / 5800.4 Hz | 3.42x |
| NPU | 1.1269 ms / 887.4 Hz | 10.8286 ms / 1477.6 Hz | 1.67x |

GPU scales best — plausibly the most parallel compute headroom relative to
this model's size, though this script does not instrument
dispatch-vs-compute time separately, so that reading is an inference from
the curve shape, not a directly measured cause. Every device's batch-1
number this run lands within run-to-run measurement noise of Phase 4's own
table (GPU −13.7%, NPU +2.5%, CPU −1.7%), not a regression. Phase 4's own
finding that GPU FP32/FP16 report byte-identical deviation and
near-identical latency (consistent with the Arc plugin running FP16
internally regardless of stored precision) is carried forward as context
for the GPU curve's shape, not re-verified here — this script benchmarks
the FP16 IR only.

**Regression gate, bm-ptl, before this run:** `pytest tests/test_skills.py`
reproduced `4 passed / 4 failed`, same four tests as ADR-047/048/049/051 —
untouched by this measurement-only work.

**Consequences.** New files only: `scripts/benchmark_batch_scaling.py`,
`artifacts/posenet_ir/batch_scaling_results.jsonl` (gitignored, same
`artifacts/` block Phase 4/INT8 already use). `docs/hardware/
m10-phase4-benchmark.md` gained one new "Batch Scaling Analysis" section;
every prior section, row and finding in that document is unchanged.
`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`scenes/so101/`, `gen_dual_scene.py`, `posenet.py`, `dataset.py`, the
checkpoint and every requirements file were not touched; nothing was
installed into `ov_env` or `train_env`.

**Process note.** bm-ptl's local `master` had fallen behind origin (last
synced commit `471fcb3`, three commits behind this module's starting
`c9a65be`) — a stale-clone situation rather than the stale-tracking-ref
symptom ADR-045 previously documented. Synced via the pull-only PAT fetch
(`git fetch https://<PAT>@github.com/.../intel-bimanual-vla.git master`)
then `git reset --hard FETCH_HEAD` before this script ran; the PAT itself
was never written to a file. bm-ptl's push access is not possible (pushes
return HTTP 403 with this PAT), so this commit is pushed from the laptop,
not bm-ptl.

**Source:** this commit ("M10 batch scaling: throughput vs batch across
CPU/GPU/NPU (ADR-052).") — code and benchmark doc land together.

---

### ADR-053 — M08 extended: 20-seed Track A robustness sweep across all four working skills, same file as ADR-049 — reproducibility confirmed bit-for-bit on the shared ten seeds; `handoff`'s 20/20 re-disclosed as degenerate at every appearance, not a robustness claim

**Ratified:** Sept 15, 2026 · **Follows:** ADR-049 (M08's original two-track
10-seed eval, this module extends Track A only), ADR-047 (fresh
env/executor per trial, carried forward unchanged), ADR-051 (`handoff`
fails 0/5 under the smallest tested non-placement perturbation), ADR-046
(`handoff` fails under a 2.2 mm perception offset) · **Adds:**
`docs/hardware/m08-extended-eval.md`; extends `scripts/eval_m08.py`
(new `--num-seeds` argument, default 10 — unchanged from ADR-049 — and a
per-trial thread-based timeout).

**Question.** Does a doubled sample (seeds 0-19 instead of 0-9) change any
of ADR-049's Track A conclusions, and — the specific check this module's
task brief asked for by name — do seeds 0-9 inside the 20-seed run
reproduce ADR-049's original 10-seed numbers exactly, or does re-running
the same seeds through the same pure-function-of-seed randomizer produce
any drift?

**Decision on scope: extend the existing script, do not fork a second
one.** `scripts/eval_m08.py` gained `DEFAULT_NUM_SEEDS = 10` and a
`--num-seeds` CLI argument; every existing default, code path and the
Track A/Track B construction logic is unchanged, so `python
scripts/eval_m08.py --track both --skill all --out-dir out/m08_eval` (no
new flags) still reproduces ADR-049's original run from this same file —
verified, not assumed (see "Regression gates" below). Only Track A was
extended to 20 seeds, per the task brief; Track B stays at its original
10-seed report in `docs/hardware/m08-eval.md`, unmodified.

**Per-trial timeout, added defensively, not because a hang occurred.**
Each trial now runs on a `threading.Thread` joined with a 300 s (5 min)
timeout; on a hang, the seed is logged as `trial_timeout_after_300s` and
the sweep moves to the next seed rather than blocking indefinitely. This
is a SOFT timeout — CPython cannot forcibly kill a thread, so a genuinely
hung trial's thread would be abandoned (daemon, isolated to its own
already-fresh `env`/`executor`, unable to corrupt a later trial) rather
than terminated at the OS level, unlike ADR-052's subprocess-per-combo
isolation. Disclosed as a real limitation, not overstated as equivalent
protection. Zero timeouts occurred in this run; the slowest skill,
`handoff`, averaged 17.4 s/trial, nowhere close to the 300 s bound.

**Reproducibility check — the result this module's task brief called "the
genuinely interesting question."** Seeds 0-9 inside this 20-seed run were
checked row-by-row against `docs/hardware/m08-eval.md`'s original Track A
tables for all four skills: offsets, pass/fail pattern, and frame counts
are IDENTICAL on every shared seed, for every skill, with zero exceptions.
`pick(A, fork)`: 10/10 both runs, frames=1655 every seed both runs.
`place(A, fork, table)`: 10/10 both runs, frames=3455 every seed both runs.
`handoff`: (0,0) offset and 10/10 both runs (degenerate, see below).
`pick(A, 'bottle')`: identical failing seeds (0, 1, 2, 4) both runs, same
frame counts including the two seeds (8, 9) whose weld-attach frame
differs slightly from the rest (1695/1718 vs 1655) in both runs. **No
reproducibility failure was found.** This is the expected result for a
`ScenarioRandomizer` that is a pure function of `seed` run through a
harness that constructs a fresh `TableSettingEnv` + fresh
`ScriptedSkillExecutor` per trial (ADR-047) — there is no cross-trial state
left for a re-run to diverge through — and it is reported here as a
positive, checked finding rather than a formality, per the task brief's
explicit instruction to check it either way.

**Results, seeds 0-19, Track A (own-prop randomization), bm-ptl:**

| skill | 20-seed result | ADR-049's 10-seed result |
|---|---|---|
| `pick(A, fork)` | 20/20 | 10/10 |
| `place(A, fork, table)` | 20/20 | 10/10 |
| `handoff(B, A, fork)` | **20/20 — degenerate: envelope is a single point, zero displacement applied every trial; measures determinism, not robustness** | 10/10 — same qualifier |
| `pick(A, 'bottle')` | **9/20 (45%)** | 6/10 (60%) |

**`pick(A, 'bottle')`'s pooled rate drops from 60% to 45%** once seeds
10-19 are included (that decade alone: 3/10, worse than seeds 0-9's 6/10).
This is not a reproducibility problem (seeds 0-9 are bit-identical to
ADR-049, shown above) — it is exactly what a larger sample is supposed to
reveal about a small one: ADR-049's original 6/10 sat on the better half of
this skill's tolerance rectangle more often than the next ten seeds did.
**45% (9/20) is the more reliable estimate of this skill's true
within-envelope success rate and supersedes the 60% figure going
forward**, per ADR-018's standing rule against reporting a favourable
subset as the whole picture. The failure pattern remains non-monotonic in
`dy` at double the sample (e.g. seed 11 at `dy=-0.01mm` PASSES immediately
next to seed 15 at `dy=+6.32mm`, which FAILS, while seed 6 at
`dy=-3.13mm`, similar magnitude to seed 15, PASSES) — the same fine,
sub-cm structure ADR-049 already attributed to the 1 cm grid
`docs/hardware/m07-envelopes.md`'s sweep used to choose this rectangle, now
sampled ten more times rather than newly discovered.

**`handoff`'s 20/20 carries the SAME qualifier ADR-049 attached to its
10/10, repeated at every appearance in `docs/hardware/m08-extended-eval.md`
(summary table, per-skill section, demo-seed table) rather than stated
once.** All twenty trials are the byte-identical unperturbed scenario
(`frames_used=6610`, `dist_to_armB=0.0582`, `from_arm_retreat_dist=0.2263`
— all bit-identical to `scripts/verify_adr038_skills.py`'s own numbers).
Twenty repeats of one deterministic scenario is not a larger robustness
sample than ten repeats of the same scenario; it is the identical
zero-variance measurement run twice as many times. Reading this 20/20
unqualified next to `pick(A, 'bottle')`'s 45% would say `handoff` is the
most robust skill measured here — the opposite of what three independent,
already-ratified measurements say: ADR-049's own Track B (0/10 when
`water_bottle`, a prop `handoff` never touches, is randomized underneath
it), ADR-051 (0/5 at the smallest tested home-pose arm-angle noise,
±0.005 rad — no prop placement involved at all), and ADR-046 (perception
mode fails `handoff` at a 2.2 mm targeting offset on an object outside its
own success check). This module changes none of those three findings; it
only re-confirms, at double the sample, that `handoff`'s own envelope
still has zero width.

**Regression gates, bm-ptl, before AND after this sweep, both identical:**
`pytest tests/test_skills.py` reproduced `4 passed / 4 failed`, same four
tests and reasons as ADR-047/048/049/051/052.
`scripts/verify_adr038_skills.py` reproduced `0.3989 / 0.3588 / 0.6192 /
0.1946`, unchanged. Expected, not merely hoped for: this module changed no
skill, grasp, IK, executor, environment or randomization code, only
`scripts/eval_m08.py`'s CLI surface and internal timeout handling.

**Timing.** All 80 trials (4 skills x 20 seeds, Track A only) completed on
bm-ptl in under 8 minutes wall-clock; every trial finished well inside the
300 s per-trial timeout (mean per-trial time ranged 1.7 s for
`pick(A,'bottle')` to 17.4 s for `handoff`). Zero trials timed out; zero
harness errors were logged.

**Consequences.** `out/m08_eval_extended/` (gitignored, same `out/` block
every other M08/M10 artifact directory uses) holds the four per-skill
JSONL files plus a combined `summary_all.json`; ADR-049's original
`out/m08_eval/` directory was never touched, and its 10-seed,
two-track report in `docs/hardware/m08-eval.md` stands unmodified.
`SUBMISSION.md` was explicitly NOT edited for this module (overnight-batch
rule, logged in `docs/hardware/overnight-batch-log.md` instead of the
submission file it would otherwise have touched).

**Process note (mirrors ADR-052's own account of the same constraint).**
bm-ptl's PAT remains pull-only (pushes return HTTP 403), so this commit is
pushed from the laptop; bm-ptl was synced afterward via the pull-only PAT
fetch (`git fetch https://<PAT>@github.com/.../intel-bimanual-vla.git
master` then `git reset --hard FETCH_HEAD`, PAT never written to a file).
The modified `scripts/eval_m08.py` was transferred to bm-ptl via `scp`
(not a git operation) to run this sweep, since the code had not yet
landed on `master` at run time; bm-ptl's working tree carried this one
file as an uncommitted local diff for the duration of the run and was
never committed to there.

**Source:** this commit ("M08 extended: 20-seed robustness sweep across 4
skills (ADR-053).") — code and eval doc land together.

---

### ADR-054 — `run_handoff` Phase 1 `already_held` guard (mirrors ADR-034, same convention): fixes the pick→handoff re-pick gap so skills compose into a chain; standalone regression gate reproduced byte-identical on laptop and bm-ptl, before and after; the chain still fails, now at a NEW Phase 3 waypoint, reported not patched

**Ratified:** Sept 15, 2026 · **Follows:** ADR-034 (`run_place`'s
already-held guard, the pattern this ADR copies rather than reinvents),
ADR-037 (handoff's sequential choreography and frozen-hold snapshots,
unmodified here), ADR-038 (per-arm-identity retreat vectors, unmodified
here), Fix C (`docs/hardware/overnight-batch-log.md`'s "Fix B" entry —
mis-numbered in that log's own text but the same chained-demo attempt —
which found and diagnosed this exact bug, did not touch
`skills_scripted.py` per its own task scope, and explicitly left ADR-054
unratified pending this fix).

**The bug, exactly as measured before this change.** `run_handoff`'s Phase
1 (`skills_scripted.py`, previously line 2136) called the nested
`run_pick(env, from_arm, target_object, ...)` UNCONDITIONALLY, with no
check for whether `from_arm` already held `target_object` — unlike
`run_place`, which ADR-034 already fixed for exactly this shape of bug. In
a chained episode (`scripts/chained_demo.py`: one `env.reset()`, one
`WeldGrasp`, `pick(A, fork)` immediately followed by `handoff(A→B, fork)`
in the same episode — the composition a multi-step task plan produces),
Phase 1 tried to re-grasp a fork arm A already held. `grasp.py`'s
already-holds gate (`WeldGrasp.attempt_grasp`) correctly refuses every
step of that re-grasp attempt, so the nested pick's GRIP waypoint
exhausted `GRIP_HOLD_FRAMES` (300) with no attach and `handoff` died with
`weld_attach_failed_after_300_frames` before ever reaching Phase 2.

**Fix applied (`skills_scripted.py`, `run_handoff` Phase 1 only — copied
from ADR-034's `run_place` pattern, not a new convention).** Added
`already_held = weld is not None and weld.is_holding(from_arm) == body_name`
immediately before the nested `run_pick` call. If true, the nested pick is
skipped entirely and `pick_result` is set to `None`; every downstream
`pick_result.<attr>` access (`.frames_used`, `.success`, `.reason`,
`.weld_attach_frame`) is now inside an `if pick_result is not None:`
guard, so the skipped-pick case cannot crash or misreport frame counts —
the same care ADR-034 took with its own `pick_attach_frame` variable.
Unlike `run_place`, nothing later in `run_handoff` reads `pick_result`
again (checked explicitly, not assumed): Phase 4's own `weld_attach_frame`
reporting uses a separate `attach_frame` variable tied to `to_arm`'s own
grip, entirely independent of Phase 1's pick — so no second propagated
variable (`run_place`'s `pick_attach_frame`) was needed here. When `weld
is None` or the object is not already held — true for every standalone
`handoff` call, including `scripts/verify_adr038_skills.py`'s and
`scripts/run_demo.py`'s — `already_held` evaluates `False` and the nested
`run_pick` call runs exactly as before. No other line in `run_handoff` was
touched; `run_pick`, `run_place`, ADR-031's GRIP freeze, ADR-033's
per-prop hover, ADR-035's interpolation, and ADR-037/038's choreography
phases and retreat vectors are all unmodified. No change to `grasp.py`,
`ik.py`, `executor.py`, `env.py`, `randomization.py`, `scenes/so101/`, or
`gen_dual_scene.py`.

**Regression gate — the safety property this fix depends on, verified
directly rather than assumed.** `scripts/verify_adr038_skills.py` exercises
the standalone path (`weld` is fresh, never pre-attached, for every one of
its four calls), so `already_held` should evaluate `False` at every call
site and reproduce the exact prior behaviour. Run before and after the
code change, on both machines:

| machine | metric | before | after | diff |
|---|---|---|---|---|
| bm-ptl | `pick(A, fork)` z | 0.3560 → 0.3989 | 0.3560 → 0.3989 | none |
| bm-ptl | `place(A, fork, table)` final z | 0.3588 | 0.3588 | none |
| bm-ptl | `pick(A, 'bottle')` z | 0.4400 → 0.6192 | 0.4400 → 0.6192 | none |
| bm-ptl | `handoff(A→B, fork)` lateral sep / frames_used | 0.1946 m / 6610 | 0.1946 m / 6610 | none |
| laptop | `pick(A, fork)` z | 0.3560 → 0.3987 | 0.3560 → 0.3987 | none |
| laptop | `place(A, fork, table)` final z | 0.3588 | 0.3588 | none |
| laptop | `pick(A, 'bottle')` z | 0.4400 → 0.6191 | 0.4400 → 0.6191 | none |
| laptop | `handoff(A→B, fork)` lateral sep / frames_used | 0.1958 m / 6610 | 0.1958 m / 6610 | none |

The laptop-vs-bm-ptl digit difference (0.3987 vs 0.3989, 0.1958 m vs
0.1946 m) is the SAME pre-existing, already-disclosed cross-machine
floating-point divergence ADR-047 found and attributed to contact-solver
iteration-order drift across CPUs, not a consequence of this fix — it is
present identically before and after, on both machines, confirming this
fix changed no floating-point-sensitive code path in the standalone case.
Full raw output of both runs, both machines, both before and after, is
captured verbatim in this commit's terminal record; every digit above was
read directly off that output, not retyped from memory.

`pytest tests/test_skills.py`, both machines, before and after this
change: **4 passed / 4 failed**, the same four failing tests with the
same residuals every time —
`test_open_drawer_reaches_near_limit`,
`test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b` (the last one's own failure reason,
`phase 1 (from_arm pick) failed (waypoint 1 (approach) failed
[convergence (IK residual=0.0532 m >= 0.01 m)])`, is unchanged digit for
digit before and after — this test's own `handoff` call never has
`from_arm` already holding the mug, so `already_held` evaluates `False`
there too, exactly as this fix's safety property requires).

**None of the hard revert conditions moved.** All five numbers this task
was gated on (`pick(A, fork)` z, `place` final z, `pick(A, 'bottle')` z,
handoff lateral separation, `frames_used`) and the pytest count are
byte-identical before and after, on the authoritative machine (bm-ptl) and
on the laptop. The fix is kept, not reverted.

**Then: does the chain compose? Genuinely tested, not assumed — and the
answer is no, at a NEW waypoint.** `scripts/chained_demo.py` (Fix C built
it; unmodified here) re-run on bm-ptl with this fix applied:

1. **`pick(A, fork)` — PASS.** `frames_used=1655`, `weld_attach_frame=1155`,
   fork z 0.3560 → 0.3989, `is_holding('A')=='fork'`.
2. **`handoff(A→B, fork)` — FAILS, at a DIFFERENT waypoint than before this
   fix.** The old failure (`weld_attach_failed_after_300_frames` inside
   Phase 1's nested re-pick) is gone — this fix's own target bug is
   confirmed fixed, in the chained case, not just in isolation. Phase 1
   now correctly skips (0 frames spent) and Phase 2 (`from_arm` approaches
   `transfer_point`) succeeds. The NEW failure is Phase 3 (`to_arm`
   approaches the receiving point): `"phase 3 (to_arm approach) failed
   [direct approach failed [convergence (IK residual=0.0875 m >= 0.01 m)];
   staging to y=-0.06 also failed [collision (cross_arm contacts=1 vs
   baseline 0; armB-vs-table_top contacts=0 vs baseline 0)]]"`,
   `frames_used=1500`. **Diagnosed, not guessed: this is a plausible
   consequence of `from_arm`'s different starting pose, not of this fix's
   own logic.** In the standalone verification above, Phase 1 always runs
   a nested pick that ends its own RETREAT at a specific pose; in the
   chained case, Phase 1 is now skipped and `from_arm` instead carries
   over whatever pose STEP 1's own independent, full-budget `run_pick`
   call left it at (a legitimately different final pose — the standalone
   nested pick inside `handoff` only ever got `step_budget // 2`, and
   critically, `run_pick`'s own APPROACH/DESCEND targets depend on the
   object's position at the START of that specific call). That different
   `from_arm` starting pose propagates through Phase 2's approach to
   `transfer_point` and apparently leaves `from_arm` sitting in a slightly
   different final spot than the standalone case, which is then close
   enough to `to_arm`'s staging corridor (`HANDOFF_STAGING_Y_M`,
   ADR-035/037) to produce a genuine cross-arm collision when `to_arm`
   tries to stage through it. This was reasoned about, not verified by
   further instrumentation — the task instructions for this fix
   explicitly forbid modifying anything further to chase this down, so it
   is reported as a plausible mechanism, not a proven one, exactly the way
   ADR-034's own second (unpatched) reachability failure was reported.
3. **`place(B, fork, table)` — NEVER REACHED.** Step 2 failed, so the
   script never attempts step 3 (`chained_demo.py`'s own control flow
   stops the chain honestly at the first failure).

**Fallback, exactly as the task instructions anticipated (labelled, not
conflated with chain success).** `chained_demo.py`'s own fallback branch —
`place(A, fork, table)`, since arm A still held the fork after step 2's
failed handoff — ran and **PASSED this time**: `frames_used=1800`, final
fork position `x=0.0835 y=-0.0585 z=0.3591`, `is_holding('A') is None`.
This is a DIFFERENT outcome than Fix C's own fallback run (which hit an
arm-vs-mug collision at −0.0051 m against a −0.005 m threshold, a 0.1 mm
miss) — expected, not a contradiction: Fix C's fallback ran immediately
after Phase 1's OLD failure (fork essentially untouched since step 1), the
task instructions warn marginal collisions are live in this area, and this
fix's fallback runs after Phase 2 of a DIFFERENT, now-further-progressed
handoff attempt left the fork at a different table position before the
fallback place began. Reported as observed, not adjusted or re-run to
chase a match with Fix C's number — **no threshold, gate, or waypoint
constant was touched to produce or explain either fallback outcome.**

**Not done, deliberately.** No attempt was made to fix, route around, or
adjust any threshold for the new Phase 3 collision — per this fix's own
task instructions, that is a real finding to report, not a defect in this
fix to chase. No video was rendered: the chain did not fully succeed, so
`docs/videos/chained-demo.mp4` was not produced and none of
`ffmpeg`/`scp`/PNG rendering was invoked. `SUBMISSION.md` was not modified
(the batch's standing rule); this finding is logged in
`docs/hardware/overnight-batch-log.md` instead.

**Net honest status.** `run_handoff`'s Phase 1 re-pick bug (the bug this
ADR was scoped to fix) is fixed and verified two ways: the standalone
regression gate is byte-identical on two machines before and after, and
the chained case's OLD failure mode (`weld_attach_failed_after_300_frames`
at Phase 1) no longer occurs. The three-skill chain still does not
complete end to end — it now fails one phase later, at Phase 3's cross-arm
staging collision, a different and previously unmeasured failure mode.
`pytest tests/test_skills.py`: 4 passed / 4 failed, identical to every
prior ADR in this chain.

**Source:** this commit ("M06 handoff: already-held guard so skills
compose into a chain (ADR-054).").

---

### ADR-055 — Perception-in-loop demo: `pick(A, fork)` run end to end with PoseNet driving its grasp-point targeting through the M10 Phase 5 wiring (GPU FP16) — PASSES on oracle ground truth, reproducing ADR-046's own 8.4 mm outcome delta to within 0.03 mm

**Ratified:** Sept 15, 2026 · **Follows:** ADR-046 (M10 Phase 5's
targeting/verification split, held-object oracle fallback, and opt-in
camera/executor wiring — exercised here, not modified), ADR-045 (the
PoseNet OpenVINO IR this run compiles, GPU FP16 default), ADR-047
(`WeldGrasp`/`ScriptedSkillExecutor` reset fix, unmodified and implicitly
relied on by this run's fresh env/weld per call). This ADR's number was
corrected from the batch brief's original "056" to **055** mid-batch,
after ADR-054 (the `run_handoff` already-held guard) was ratified ahead of
it — see `docs/hardware/overnight-batch-log.md`'s "Fix F" entry.

**What this adds that `scripts/verify_m10_phase5.py` (ADR-046) did not.**
Phase 5's own verification script runs all four skills, oracle THEN
vision, as an A/B comparison, and never produces a standalone artifact of
one skill running under perception. This fix is narrower and
demonstration-focused: a new `scripts/perception_demo.py` runs exactly
`pick(A, fork)` once, with `position_provider` wired to a real
`PoseNetInference(device='GPU')` + `CachedPropPositions`, and renders the
result as `docs/videos/perception-demo.mp4`. **No skill, grasp, IK,
executor, environment, or randomization code was touched** — this fix
calls the same unmodified `sk.run_pick(env, "A", "fork", weld=weld,
position_provider=provider)` entry point `ScriptedSkillExecutor._dispatch`
itself would produce for `SkillCall(skill="pick", arm="A",
target_object="fork")`.

**Verification stays on oracle, per ADR-046 Correction 1, checked
independently of `run_pick`'s own `.success`.** This script reads
`env.data.xpos[fork][2]` and `WeldGrasp.is_holding('A')` directly — the
same privileged reads every pre-Phase-5 script already used — never
`position_provider.get()`. Success requires BOTH `fork_z > 0.37` AND
`is_holding('A') == 'fork'`. `0.37` is not an independently chosen number:
it is asserted at runtime to equal `sk.TABLE_SURFACE_Z (0.35) +
sk.WELD_PICK_SUCCESS_MARGIN_M (0.02)` — the exact threshold `run_pick`'s
own internal weld-based success check already uses
(`skills_scripted.py:1561`) — so this script's external verification and
the skill's internal one agree by construction.

**Randomization (batch correction 3, followed, not the alternative).**
`env.reset(seed=0)` with **no randomizer passed** — a deliberate,
non-default choice among two the task brief offered (the other being
`pick(A, fork)`'s own Track A rectangle from `SKILL_ENVELOPES`,
`run_demo.py`'s precedent). Fixed default was chosen specifically so this
run's numbers are the SAME scenario ADR-046's own single-refresh
oracle-vs-PoseNet table already measured, making a direct comparison
possible rather than presenting a different measurement as a replication.
`ENVELOPES = {}` at module scope (`randomization.py`, ADR-048) means a
bare seed with no explicit randomizer selects nothing — stated in this
script's own printed output, not left implicit.

**Result — PASSES, on oracle ground truth.** `env.data.xpos[fork][2]`:
0.3560 → 0.3905. `WeldGrasp.is_holding('A')`: `'fork'`. `run_pick.success`:
`True` (`weld_attach_frame=1155`, `frames_used=1655`, wall clock 1.487 s).
**ORACLE_SUCCESS: True** — `0.3905 > 0.37` and `is_holding == 'fork'`, both
independently re-derived from `env.data`, not read back from `run_pick`'s
own report.

**Two distinct delta quantities, kept separate (a first-draft mistake in
this script's own print statement conflated them; caught and fixed before
commit, not shipped silently wrong).**
1. **Perception-estimate delta** (PoseNet's raw xyz guess vs. ground truth
   AT the one render instant, `CachedPropPositions.deltas`): fork 2.19 mm,
   water_bottle 21.71 mm, mug 13.43 mm. Fork's figure matches ADR-046's own
   reported 2.2 mm for this identical seed/scenario almost exactly — this
   IS the same measurement, re-derived independently, not merely quoted.
2. **Outcome delta** (how far THIS run's real physical final z — achieved
   WITH perception driving the grasp-point target — lands from the
   well-established oracle-only baseline final z, 0.3989, reproduced
   byte-identical across ADR-038/045/046/047/053/054 and re-confirmed this
   same commit via `scripts/verify_adr038_skills.py`): `|0.3989 − 0.3905| =
   8.37 mm`. **This is the quantity ADR-046's own headline "8.4 mm" figure
   refers to**, and this run reproduces it to within 0.03 mm.

**Perception bookkeeping.** `cache refresh_count=1`, `inference_count=1`
— `run_pick`'s targeting read is the ONLY `position_provider.get()` call
across the whole call (`cached_access.py`'s own docstring: at most one
render+infer per cache generation), confirmed by direct count, not
assumed. The one real in-loop `predict()` call: 1.83 ms (n=1 — a single
sample, not a distribution; stated as such, not dressed up as a mean over
many runs). A supplementary `PoseNetInference.benchmark(n_runs=100)` on
the SAME already-compiled model (no second compile, not part of the live
pick) gives a proper distribution: **GPU FP16 mean=0.5903 ms,
median=0.5889 ms, min=0.5773 ms, max=0.6239 ms, throughput=1694 Hz**
(`execution_devices=['GPU.0']`) — closely tracking ADR-046/M10 Phase 5's
own GPU FP16 figures (mean=0.6648 ms, max=7.2228 ms), confirming this
run's compiled model performs consistently with Phase 5's own measurement.
Device: GPU throughout; the CPU-fallback branch this script also
implements (ADR-007: no silent device substitution) was never exercised —
the GPU compile succeeded on the first attempt.

**Regression gates, re-run this commit, unchanged.**
`pytest tests/test_skills.py`: **4 passed / 4 failed** — the same four
tests failing for the same, already-documented reasons
(`test_open_drawer_reaches_near_limit`, `test_pick_plate_lifts_above_table`,
`test_place_plate_returns_to_table_rest`,
`test_handoff_mug_ends_held_by_arm_b`). `scripts/verify_adr038_skills.py`:
**0.3989 / 0.3588 / 0.6192 / 0.1946 m, frames_used=6610** — byte-identical
to the documented baseline. Both were expected unchanged (this fix touches
no skill/grasp/ik/executor/env/randomization code) and both are.

**Video.** `docs/videos/perception-demo.mp4` — 40 PNG frames (`front`
camera, 640x480), rendered on bm-ptl from `mujoco.mjtState.mjSTATE_FULLPHYSICS`
snapshots an observer wrapper captured around `env.step` (never altering
the real run's control, timing, or step count — the same technique
`scripts/render_handoff_frames.py` already uses), spanning physics-step
indices 1..1655, `scp`'d to the laptop, encoded with `ffmpeg` there (bm-ptl
has no imaging libraries in `ov_env`).

**Consequences / disclosed departures.** `scripts/perception_demo.py`
defines its own local `_TimingInference` wrapper around the real
`PoseNetInference` purely to measure the one real `predict()` call's
wall-clock, WITHOUT modifying the off-limits `inference.py` — forwards
every other attribute via `__getattr__` so `CachedPropPositions` sees an
object indistinguishable from the real one. This script also duplicates
`render_handoff_frames.py`'s stdlib-only `write_png` verbatim rather than
importing it (no shared PNG-writing module exists in this repo — the same
justification `cached_access.py`/`inference.py` already gave for
duplicating `PROP_ORDER`, ADR-046).

**Source:** this commit ("Perception-in-loop demo: PoseNet drives
pick(A, fork) (ADR-055).").

---

## 5. Open items this document deliberately does not decide

These are flagged, not guessed. Full list with evidence in `PLAN.md` section 7.

- **Platform video length limit** (RISK-10) — nothing in `SUBMISSION.md` records one.

### Closed since first issue (Sept 11, 2026)

- **SO-101 asset source and licence** (RISK-01) — closed. TheRobotStudio/SO-ARM100 @
  `eecbe3e0`, Apache-2.0, unmodified per ADR-016; provenance in
  `scenes/so101/PROVENANCE.md`. M02 is unblocked.
- **MuJoCo rendering on bm-ptl** (RISK-03) — **closed Sept 11, 2026.** `scripts/probe_render.py`
  run on bm-ptl (`WIN-GLILH4PFDLN`) under `mujoco==3.2.7` compiled
  `scenes/so101/scene.xml` (`nq=6 nv=6 nu=6 nbody=8 ngeom=31`) and rendered offscreen
  headless over SSH with MuJoCo's default Windows backend — no `MUJOCO_GL` override, no
  osmesa, no X server. Output `out_probe.png`, 640x480, mean pixel 77.6, visually confirmed
  to show the arm lit with shadow and floor reflection. ADR-020's premise holds in both
  directions: mujoco imports on bm-ptl and does not on the laptop. **WSL2 (ADR-020 option
  (c)) is therefore not needed and stays unbuilt.** One caveat carried into M02: the
  upstream scene declares a 640x480 offscreen framebuffer, so larger renders need an
  explicit `<visual><global offwidth/offheight/></visual>` in our own scene.
- **Actuated DoF per arm** (RISK-02) — closed by ADR-016. No longer a decision: the adopted
  asset's shipped DoF is authoritative and Builder reports it in M01/M02.
- **Speechmatics credentials** (RISK-04) — handling closed by ADR-019 (`.env`, gitignored,
  read as `SPEECHMATICS_API_KEY`). Whether a key is provisioned in time remains a
  scheduling matter under the Day-4 drop rule, not an architectural one.
- **Interpretation of the 10-seed success bar** (RISK-06) — closed by ADR-018. True success
  rate as measured; failure modes narrated in the video.
- **Acceptability of `pour` without fluid** (RISK-08) — closed by ADR-017, with disclosure
  required in both `README.md` and the video narration.
- **Laptop cannot run MuJoCo** (RISK-11, raised by builder during the M02 asset
  prerequisite) — closed by ADR-020. Smart App Control stays enabled; all MuJoCo work moves
  to bm-ptl.
