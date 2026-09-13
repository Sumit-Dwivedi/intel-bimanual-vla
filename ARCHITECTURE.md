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
physical top instead of sitting below it. **Handoff does not currently
work**: three independent reachability sweeps (ADR-032, two passes, plus an
earlier pass) each found zero shared, collision-free transfer points
reachable by both arms; the cause is under active investigation and
unresolved as of this writing.

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
