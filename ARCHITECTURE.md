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
| MuJoCo sim, eval harness | dev + data gen | — | **final demo (brief p3)** |
| CommandSource: text | yes | — | yes |
| CommandSource: voice | **only here** (`CONSTRAINTS.md:40`) | — | never |
| Grounder | yes | — | yes |
| ACT / PoseNet training | no | **yes** (`CONSTRAINTS.md:57`) | no |
| OpenVINO export to IR | yes | — | verify compile |
| OpenVINO inference + benchmark | optional | — | **yes, the scored artifact** |

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

## 5. Open items this document deliberately does not decide

These are flagged, not guessed. Full list with evidence in `PLAN.md` section 7.

- **SO-101 asset source and licence** (RISK-01) — blocks M02 and every downstream module.
- **MuJoCo rendering on bm-ptl** (RISK-03) — if offscreen rendering fails, the split
  between where the video is recorded and where the benchmark runs must be decided and
  disclosed.
- **Platform video length limit** (RISK-10) — nothing in `SUBMISSION.md` records one.

### Closed since first issue (Sept 11, 2026)

- **Actuated DoF per arm** (RISK-02) — closed by ADR-016. No longer a decision: the adopted
  asset's shipped DoF is authoritative and Builder reports it in M01/M02.
- **Speechmatics credentials** (RISK-04) — handling closed by ADR-019 (`.env`, gitignored,
  read as `SPEECHMATICS_API_KEY`). Whether a key is provisioned in time remains a
  scheduling matter under the Day-4 drop rule, not an architectural one.
- **Interpretation of the 10-seed success bar** (RISK-06) — closed by ADR-018. True success
  rate as measured; failure modes narrated in the video.
- **Acceptability of `pour` without fluid** (RISK-08) — closed by ADR-017, with disclosure
  required in both `README.md` and the video narration.
