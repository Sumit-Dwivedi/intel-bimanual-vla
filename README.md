# Intel Physical AI Online Challenge — Bimanual VLA Manipulation

Simulation-first solution for the Intel Physical AI Online Challenge at the AI Infra
Summit Hackathon (Sept 10–16, 2026). Two SO-101 arms in MuJoCo perform a
language-conditioned table-setting task, with inference on Intel Core Ultra Series 3
(Panther Lake) via OpenVINO 2026.3 across CPU, iGPU (Arc B390), and NPU (NPU5010).

Licensed under the MIT License — see `LICENSE`.

## Quickstart

```bash
./run_demo.sh
```

`run_demo.sh` is the project's entry point for reproducing the demo. Before
running it, check your environment against `scripts/requirements-dev.txt`
(laptop) or `scripts/requirements-bmptl.txt` (Intel target), and run
`scripts/verify_env.py` to confirm the install. This is a simulation-only
project — no physical robot is involved anywhere in the pipeline. See the
Status section below for exactly which skill/arm/object combinations
currently work end-to-end, and `SUBMISSION.md` for the full, unembellished
submission checklist and rubric self-assessment.

## Demo Video

`docs/videos/full-sequence-demo.mp4` — one continuous, fixed-camera take of the
full manipulation sequence: arm A picks the fork, hands it to arm B, and arm B
places it on the table. 40.0 s, 1280x720, 30 fps, 4.2 MiB. No cuts, no camera
moves, no speed changes: 8410 physics steps rendered every 7th step.

Reproduce on bm-ptl with:

```bash
python scripts/render_full_sequence.py --seed 3 --stride 7 --distance 1.05
```

**It is two skill calls, not three.** `run_handoff`'s own Phase 1 *is* a nested
`run_pick`, so `run_handoff(env, "B", "A", "fork")` already performs the pick and
the transfer; `run_place(env, "B", "fork", "table")` then completes the sequence.
Both report success (handoff 6610 steps, place 1800). The literal three-call form
`pick` -> `handoff` -> `place` does NOT work — see the composition note in
`SUBMISSION.md`.

**Two things in the take are worth naming rather than leaving a viewer to wonder.**
The green mug is knocked onto its side early in the run, and the blue water bottle
is nudged and eventually falls off the table. Neither is the target object, and
neither changes the measured outcome — the scripted skills plan only around the
object they are acting on and have no collision avoidance for other props. The
fork task itself completes: the fork ends at z=0.3537 on a table surface at
z=0.35, with the weld released.

`docs/videos/m06-handoff-clip.mp4` is retained but superseded as primary evidence:
it is a 2.0 s close-up in which only the last ~0.5 s reads as a handoff.

## Status

**In active development through Sept 16, 2026.** The final README — with reproduction
instructions, benchmark tables, and the demo video — lands Sept 15. As of Sept 14,
four skill/arm/object combinations execute end to end and are verified by direct
measurement, including a bimanual handoff (`handoff(A→B, fork)`); four others are
documented as not yet working, with measured failure reasons. See `SUBMISSION.md`
for the current, unembellished status of every deliverable.

## Architecture

- Scripted closed-loop bimanual controller (see ADR-023 — no learned policy is trained)
- Rule-based language grounder (M05, 43 tests) — parses the challenge brief's own
  example command into an ordered plan with explicit per-arm assignment
- OpenVINO deployment path verified on all three devices (M03): PyTorch → IR → compile →
  infer on CPU, iGPU and NPU, with per-device numerical deviation recorded
- Full ADR log in `ARCHITECTURE.md` and `DECISIONS.md`

## Grasping Abstraction and Documented Limitations

Contact-based grasping is not physically simulated in this scene. The SO-101
gripper's jaw meshes are non-convex, C-shaped housings that MuJoCo collapses to
overlapping convex hulls with no collision decomposition declared anywhere in the
asset (MuJoCo issue #239); a direct caliper sweep found 0 of 30 measured jaw
thicknesses ever achieved sustained two-jaw contact (`docs/hardware/grasp-envelope.md`).
Two fixes address this, at two different levels:

- **Finger-pad collision primitives** (ADR-028) replace the jaw mesh's collision
  geometry with small box primitives, verified to move correctly with jaw closure
  across a **6-132 mm centre-to-centre** separation sweep (fully closed 6.00 mm,
  midway 76.21 mm, fully open 131.88 mm — `scripts/probe_pad_separation.py`). That
  figure is the distance between the two 2.5 mm cube pads' *centres*; the
  surface-to-surface gap that actually fits between the jaws is smaller (roughly
  2.5 mm closed to 120 mm open, via `mj_geomDistance`). This fix corrects the
  collision geometry only — it does not by itself make any skill grasp anything.
- **A MuJoCo weld equality constraint** (ADR-029/ADR-030) is the mechanism that
  actually performs the grasp. `WeldGrasp.attempt_grasp` attaches the object to the
  gripper body only when two measured gates both hold — the jaw is closing past a
  joint-angle threshold, and the gripper body is within a distance threshold of the
  object — and releases it explicitly on `place`/`handoff`'s own release step. This
  is an **explicit abstraction of physical grasping, not friction-based finger
  contact**: the object stays held because a constraint says so, not because
  simulated fingers grip it. This is disclosed here plainly rather than implied
  otherwise (ADR-015).

This is the same class of abstraction used elsewhere in sim robotics — MoveIt's
"attached objects" and PyBullet's fixed constraints are both real mechanisms that
attach a grasped object to a gripper frame without simulating finger-object
friction contact. The comparison here is made only at that mechanism level; it is
not a claim that this project's implementation is equivalent to either of those
specific published systems.

Current per-skill status (which combinations work and which do not, with measured
failure reasons) is tracked in `SUBMISSION.md`, updated as skills are verified.

## Voice Input (bonus, ADR-058)

Speechmatics batch transcription wired as a front end to the existing text pipeline,
via `VoiceCommandSource` (`src/bimanual/command/voice_source.py`), the second concrete
implementation of the `CommandSource` abstraction M04 shipped (`ARCHITECTURE.md`
ADR-002; the other is `TextCommandSource`). Nothing downstream of that boundary — the
`RuleGrounder`, the scripted skills, `run_demo.py` — changes at all: a voice command
becomes a `CommandEvent{text, confidence, ...}` and is grounded and executed exactly
like typed text.

**Setup.** Requires a [Speechmatics](https://www.speechmatics.com/) account and API
key.
1. Create a file named `.env` at the repo root (already gitignored — `.gitignore`'s
   Secrets section — never commit it).
2. Add one line: `ai_infra=<your Speechmatics API key>`. (This is the actual variable
   name this repo's credential uses; it does **not** match an earlier internal plan
   that named it `SPEECHMATICS_API_KEY` — see `DECISIONS.md` ADR-058 Deviation 2. Do
   not put the key anywhere else — not in a command-line argument, not in a script, not
   in a committed file.)
3. No extra Python package is required — `voice_source.py` talks to Speechmatics'
   batch REST API with the standard library only (`urllib.request`), so it runs in the
   same environment `run_demo.py` already uses.

**Usage.**
```bash
python run_demo.py --voice data/voice/give_fork_to_arm_b.wav
```
This transcribes the WAV file, prints the transcript, grounds it with the same
`RuleGrounder` the text path uses, executes every grounded skill with
`ScriptedSkillExecutor`, and reports pass/fail in the same format the default 4-skill
run uses. `data/voice/` is gitignored (arbitrary-size local audio, no reproducibility
value beyond the transcript already printed) — bring your own WAV file, or synthesize
one (e.g. Windows' built-in `System.Speech.Synthesis`, no microphone required). Use a
phrase `docs/command-grammar.md`'s grammar actually accepts — **"Give the fork to arm
B"** is the phrase this integration is verified against end to end (the challenge
brief's own literal example command does not parse under this grammar; see
`scripts/run_grounded_demo.py`'s docstring and `DECISIONS.md` ADR-058 for why).

**What it does not do.** No live microphone capture (file-based only); no audio ever
crosses the `CommandSource` boundary downstream (ADR-002 — the grounder and every
skill only ever see text); and, disclosed plainly rather than left implicit, this
integration is exercised end to end on bm-ptl rather than the laptop, a deliberate
deviation from `ARCHITECTURE.md`'s original laptop-only design, made because MuJoCo
only runs on bm-ptl (ADR-020) — see `DECISIONS.md`/`ARCHITECTURE.md` ADR-058 for the
full reasoning and every other disclosed deviation (credential variable name,
Speechmatics operating-point tier) this feature required.

## Documentation

- Learning journey in `docs/learn/` (notes 01–08), written for a software engineer with
  no prior robotics background
- Measurement artifacts in `docs/hardware/` — reachability envelopes, render costs,
  grasp diagnostics, device benchmarks
- Decisions log in `DECISIONS.md`; architecture and rationale in `ARCHITECTURE.md`
- Submission checklist and rubric self-assessment in `SUBMISSION.md` — tracks every
  deliverable the platform requires and what currently satisfies it, TODO otherwise
- `scripts/perception_demo.py` — `pick(A, fork)` run with PoseNet (OpenVINO, GPU FP16)
  driving its grasp-point targeting end to end (ADR-055); `docs/videos/perception-demo.mp4`

## Environment

See `scripts/requirements-bmptl.txt` (Intel target) and `scripts/requirements-dev.txt`
(laptop) for pinned dependencies, and `scripts/verify_env.py` to check an install.
