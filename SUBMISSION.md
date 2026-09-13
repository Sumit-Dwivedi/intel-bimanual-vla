# Submission Checklist

Track every deliverable required by the hackathon platform. Docs-writer
updates status marks as artifacts land. Nothing here is written on Day 5
that wasn't planned from Day 1.

## Status snapshot (updated Sept 14, 2026)

Complete and evidenced: M01 (repo scaffold, pinned envs, `scripts/verify_env.py`),
M02 (dual-arm MuJoCo scene, `TableSettingEnv`, opt-in rendering — ADR-022), M03
(OpenVINO conversion smoke test on CPU/GPU/NPU — `benchmarks/ov-smoke-notes.md`),
M04 (`CommandSource` abstraction, text + voice stub, 11 tests), M05 (rule grounder,
43 tests, `docs/command-grammar.md`).

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
  SmolVLA, or trains any policy. Note `Speechmatics` itself is still a tag for a planned
  integration, not a shipped one — see the Speechmatics bonus section below.)
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

## Rubric Alignment (Intel + platform)
Intel-specific (100 pts):
- [x] End-to-end task completion & bimanual (30) — **MET.** `handoff(A→B, fork)`
  executes end-to-end, verified by direct measurement on bm-ptl (sequential
  choreography where one arm moves at a time while the other is held frozen,
  ADR-037; render legibility fixes, ADR-038): arm A picks the fork, transfers it
  to the handoff point, and retreats clear of the shared workspace, while arm B
  approaches, grips, and ends holding it (`weld.is_holding('B')=='fork'`,
  `weld.is_holding('A') is None`, `from_arm_clear=True`) — genuine two-arm
  coordination, not two independent single-arm scripts. This follows the
  sequential single-arm-at-a-time choreography described in Wan, Ramos, Yang,
  Garrett 2025 (NVIDIA), "Learning to Plan & Schedule with Reinforcement-Learned
  Bimanual Robot Skills", https://arxiv.org/html/2510.25634v1, and is consistent
  with the collision-free bimanual trajectory planning approach in "Trajectory
  planning system for bimanual robots: Achieving efficient collision-free
  manipulation" (2025),
  https://www.sciencedirect.com/science/article/pii/S0921889025002155 (title and
  URL cited; no author name for this paper has been verified, so none is given).
  Three further single-arm skills also complete end-to-end: `pick(A, fork)`,
  `place(A, fork, table)`, `pick(A, water_bottle)`. Grasping itself is an
  explicit weld-constraint abstraction, not friction-based contact (ADR-029,
  disclosed per ADR-015) — see README's "Grasping Abstraction and Documented
  Limitations". This is not a claim that the full brief-example command
  completes: four other skill/arm/object combinations do not yet work (see the
  Working/Not working lists above), and `pour` remains out of scope (ADR-023).
  The 30-point criterion is read here as "bimanual, end-to-end task completion,"
  which the measured `handoff` satisfies.
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
