# Submission Checklist

Track every deliverable required by the hackathon platform. Docs-writer
updates status marks as artifacts land. Nothing here is written on Day 5
that wasn't planned from Day 1.

## Status snapshot (updated Sept 12, 2026)

Complete and evidenced: M01 (repo scaffold, pinned envs, `scripts/verify_env.py`),
M02 (dual-arm MuJoCo scene, `TableSettingEnv`, opt-in rendering — ADR-022), M03
(OpenVINO conversion smoke test on CPU/GPU/NPU — `benchmarks/ov-smoke-notes.md`),
M04 (`CommandSource` abstraction, text + voice stub, 11 tests), M05 (rule grounder,
43 tests, `docs/command-grammar.md`).

**M06a (scripted manipulation skills) is INCOMPLETE — pending weld-vs-reposition
decision, Day 4 AM.** No skill currently executes end-to-end: `pick` reaches every
waypoint but never achieves a lift (finger pads land below/beside the target rather
than forming a sustained pinch); `open_drawer` is structurally blocked because the
arm bases are mounted at tabletop height and cannot reach beneath the slab; `place`
and `handoff` both depend on `pick` and are therefore also unproven. Do not describe
any skill as working in judge-facing copy until this changes.

**Cut from scope:** ACT/policy training (ADR-023, user-ratified Sept 12) — no
learned policy exists; the scripted controller is the shipped policy, and brief p2
objective 4 (train/fine-tune with LeRobot) is unmet. `pour` was dropped at the
pre-committed cut-ladder gate — consequence: **the brief's own verbatim example
command, which ends "pour water into the mug," can no longer run end-to-end**, even
once M06a is unblocked, because `pour` itself does not exist. `docs/command-grammar.md`
and M05's grounder still parse that exact command correctly to a 5-skill plan; only
execution of the last skill is out of scope.

**At risk because of the above:** the 10-seed robustness recording, any end-to-end
task-completion claim, and the demo video's content all depend on at least one skill
completing successfully. As of this update none does, so none of these can be
produced yet. This is stated plainly here rather than left implied by an unchecked
box.

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
  SmolVLA, or trains any policy. Note `Speechmatics` itself is still a tag for a planned
  integration, not a shipped one — see the Speechmatics bonus section below.)
- [ ] Category tags: Robotics, Physical AI, Edge AI

## Cover Image & Presentation
- [ ] Cover image (1200x630 recommended): TODO — not produced. Candidate source material
  exists (`docs/images/m02-scene.png`, `docs/images/m02-drawer-view-open.png`,
  `docs/images/m02-drawer-view-closed.png`, `docs/images/m06-grip-diagnostic-frame30.png`)
  but none of these shows "both arms mid-task" completing a skill, since no skill
  currently completes. Brief needs a decision on whether the cover image shows the scene
  (accurate) or implies a completed manipulation (not yet true).
- [ ] Video presentation (demo + narration): TODO — see `docs/video-script.md` (not yet
  written). **At risk**: a script cannot honestly narrate an end-to-end pick/place/pour
  sequence today; see status snapshot above.
- [ ] Slide presentation (5-10 slides): TODO — see `docs/slides.md` (not yet written).

## App Hosting & Repository
- [ ] Public GitHub repo: [URL TODO — currently private, flip to public on Sept 15]
- [ ] Demo application platform: N/A (this is a local sim, not a hosted app) — clarify with lablab if in doubt
- [ ] Application URL: link to GitHub repo README or a recorded demo page

## Rubric Alignment (Intel + platform)
Intel-specific (100 pts):
- [ ] End-to-end task completion & bimanual (30) — **NOT MET.** M06a is incomplete
  (pending weld-vs-reposition decision, Day 4 AM); no skill executes end-to-end today.
  `pour` is additionally dropped from scope, so even after M06a resolves, the full
  brief-example command cannot complete without a further scope decision.
- [ ] VLA / multi-modal reasoning (20) — text command → grounded skill plan works
  (M04 CommandSource, M05 rule grounder, 43 tests, `docs/command-grammar.md`), but this
  is a rule-based grounder, not a learned VLA model — ACT/policy training is cut
  (ADR-023). No claim of learned multi-modal reasoning should be made.
- [ ] OpenVINO & Core Ultra optimization (20) — Day 0 device access proven; **M03
  complete Sept 12**: PyTorch→IR→compile→infer verified on CPU, GPU and NPU for a
  hand-rolled ResNet18-scale encoder, evidence in `benchmarks/ov-smoke-notes.md`
  (max abs deviation vs PyTorch: CPU 5.674362e-05, GPU 7.408857e-05, NPU 1.122952e-04;
  NPU rejects a fully-dynamic batch shape with a hard process crash and requires
  static/bounded shapes — see DECISIONS.md M03 / ADR-013). Note this smoke test used a
  placeholder encoder, not the project's actual PoseNet (M10/M13) — that conversion has
  not been done yet, so this criterion is not fully satisfied.
- [ ] Robustness across 10 seeds (15) — **at risk**, not started; blocked on M06a
  producing at least one working skill to run repeatedly across seeds.
- [ ] Reproducibility (10) — `scripts/verify_env.py` and pinned environments exist (M01);
  full-pipeline reproduction not yet demonstrable since M06a is incomplete.
- [ ] Innovation (5) — no claim made; TODO.

Platform-general:
- [ ] Application of Technology
- [ ] Presentation
- [ ] Business Value
- [ ] Originality

Speechmatics bonus:
- [ ] Speech input wired to VLA text pipeline — M04 ships a `CommandSource` ABC with a
  voice stub only (11 tests cover the abstraction and the text source); no real
  Speechmatics integration exists yet.
- [ ] Working demo clip with voice command — TODO, does not exist.

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
