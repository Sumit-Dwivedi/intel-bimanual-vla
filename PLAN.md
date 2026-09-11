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

**Corrected Sept 12, 2026.** The table below previously marked Day 1 as "Today" and
assumed M01–M03 all closed on Day 1. M03 did not close. This table is now the record of
what actually happened.

| Day | Date | Status |
|---|---|---|
| Day 0 | Sept 10, 2026 | **Done.** bm-ptl reachable, OpenVINO sees CPU/GPU/NPU, trivial-model benchmark ran (`CONSTRAINTS.md:60-65`, `docs/hardware/bmptl-verification.md`). |
| Day 1 | Sept 11, 2026 | **Done, partially.** Shipped: M01 (scaffold, pinned envs, `scripts/verify_env.py`) and M02 (dual-arm scene, `TableSettingEnv`, `scripts/view_scene.py`, physics probe, `drawer_view` camera, colour pass), plus a correction pass producing ADR-016..ADR-022 and the opt-in rendering refactor with its benchmark (`docs/hardware/m02-render-cost.md`). **Not shipped: M03.** No `scripts/ov_smoke.py`, no IR pair, no `benchmarks/ov-smoke-notes.md` — verified by directory listing of `scripts/` and `benchmarks/` on Sept 12. |
| Day 2 | Sept 12, 2026 | **Today.** **M03 carried over and runs first.** Then M04 (CommandSource) and M05 (grounder). See section 1A for the re-planned schedule and the clock gates. |
| Day 3 | Sept 13, 2026 | M06 scripted IK controller and skill primitives. GATE-1 at 20:00, on schedule evidence (see the retimed GATE-1 block). |
| Day 4 | Sept 14, 2026 | M07 randomization, M08 eval harness, M09a PoseNet frame collection. |
| Day 5 | Sept 15, 2026 | M10 PoseNet training, M12+M13 OpenVINO export, repo public (`SUBMISSION.md:20`). |
| Day 6 | Sept 16, 2026 | M14 bm-ptl benchmark, M16 pipeline run, M19 10-seed recorded run, M18 docs, M20 submit. Freeze 18:00. |

**Days remaining including today: 5 (Sept 12 through Sept 16).** One fewer than this
section claimed yesterday, and one module further behind than yesterday's schedule
assumed.

Submission is due Sept 16 (`CONSTRAINTS.md:4`). bm-ptl dies Sept 17 at 00:15 local
(`CONSTRAINTS.md:5`), i.e. there is **no bm-ptl access after the submission day**. Under
ADR-020 the laptop cannot run MuJoCo at all, so bm-ptl now bounds *simulation
development*, not merely benchmarking. Day 6 is therefore no longer a retry window — it
is a working day with benchmark, demo recording and submission on it. Treat Day 6 18:00
local as the freeze and assume **no slack behind it**.

---

## 1A. Re-plan of Sept 12 (Day 2) — the schedule does not fit, and what I cut

This section supersedes every day assignment made before Sept 12. Where a module block in
section 4 still carries an old day heading, **this section governs**.

### 1A.1 The arithmetic that forced the re-plan

Day 2 as previously written carried M04 (3h) + M05 (3h) + M06 (8h) + M07 (4h) + M08 (5h)
= **23 builder-hours**, plus the carried-over M03 (3h) = **26 builder-hours in one
calendar day**, before the per-module tester → compliance-reviewer → tutor → docs-writer
passes that section 2 mandates. That is not a tight plan; it is an impossible one.

**Planning capacity assumption, stated as an assumption and not a measurement:**
**8 builder-hours per calendar day.** Evidence for it is Day 1's observed shape — M01
(2h budgeted) + M02 (6h budgeted) = 8 budgeted builder-hours, plus a correction pass
(ADR-016..ADR-022 and the rendering refactor), consumed the entire day. That is
consistent with the review overhead being roughly 30% of each module's wall clock. I have
not timed anything; this is a planning figure, and if the user knows their real throughput
to be higher, the right response is to raise this number here rather than to quietly
overrun modules.

Against 8 builder-hours/day for 5 days = **40 builder-hours of capacity**, the plan as
written from M03 onward budgeted roughly **77 builder-hours**. The gap is not closable by
rescheduling. It is closable only by cutting.

### 1A.2 What I chose, against the three options offered

**I combined (a), (b) and (c), and added a scope cut, because none of the three closes a
26-hour day.** Explicitly:

- **(a) rescope budgets for review overhead** — adopted, in the form of the 8
  builder-hours/day capacity above rather than by inflating each module's number. Module
  budgets stay as *builder*-hours so they remain comparable to what has already been
  spent; the day, not the module, absorbs the overhead.
- **(b) move a Day-2 module to Day 3** — adopted and then some: M06, M07 and M08 all move
  off Day 2. M07 was the suggested candidate because the M04→M05→M06→M08 chain is
  tighter; that is correct as far as it goes, but moving only M07 leaves 22 hours on Day
  2, so it does not close the flag on its own.
- **(c) split M06 into M06a/M06b** — adopted, but repurposed. M06 is no longer split
  across two days to save Day 2; it now owns the whole of Day 3 as a single 8-hour
  module, with **`pour` time-boxed as its droppable tail** (see M06 below). The a/b split
  survives as the in-day pressure valve, not as a cross-day schedule device.
- **Scope cut** — the load-bearing change. See 1A.3.

### 1A.3 The cut: the ACT learned-policy branch leaves the critical path

**Recommendation, not a menu: cut the learned branch now. Ship the scripted controller as
the policy.**

`CONSTRAINTS.md:50-52` authorizes exactly this: "If learned policy is not working by end
of Day 3, ship a scripted IK-based controller as the 'policy.' Rubric rewards complete
pipeline over half-working ML." The re-plan takes that authorization on **schedule**
evidence rather than waiting to take it on **evaluation** evidence, because the schedule
evidence is already conclusive:

- M09's demonstration collection needs M06 **and** M07 **and** M08 closed before it can
  launch. Under the 8 h/day capacity those three close on **Day 4 evening at the
  earliest**.
- A ~9 h overnight collection then lands Day 5 morning, M11 trains Day 5, and GATE-1
  could not be held before Day 5 night. The plan's own rule forbids sliding the gate even
  into Day 4 (see the GATE-1 block). Sliding it to Day 5 would leave the benchmark, the
  bm-ptl pipeline run, the 10-seed recording, the docs and the submission all on Day 6.
- Therefore ACT cannot both exist and leave a complete pipeline behind it. Protecting it
  would be the wrong trade.

**What is cut, precisely:**

| Cut | Was | Now |
|---|---|---|
| **M11 — ACT training on Kaggle** | Day 3, 6 h, critical path | Cut from the critical path. Survives only as **F3** (optional revival after M18, with F3's existing hard stop). |
| **M09b — LeRobot scripted-demonstration dataset** (rollout trajectories with 3 cameras, ~9 h of bm-ptl wall clock) | Day 2 night → Day 3 08:00 | Cut. Exists only inside F3. |
| **M09a — PoseNet training frames** (randomized `reset()` frames + perturbed arm poses, 2 cameras) | — | **New, kept, on the critical path.** Day 4. ~1 h builder + well under an hour of bm-ptl wall clock. |

**What survives the cut, and why the cut is survivable:**
- ADR-009 exists precisely so the 20-point OpenVINO criterion does not ride on ACT: the
  PoseNet in M10 is the model that does real work on Intel silicon every control step, on
  the scripted branch. That argument is unchanged and is now the whole OpenVINO story.
- ADR-004 already made M06 dual-purpose. With M11 gone, M06 is simply the policy rather
  than also being a demonstration generator.
- Brief p2 objective 1 (coordinated control, hand-off, collision-aware sequencing) and
  objective 3 (robustness) are satisfied by M06 + M07; objectives 2 and 5 by M05's
  grounder plus M10's OpenVINO-served perception.

**What the cut actually costs, stated plainly rather than minimised:** brief p2 objective
4 asks for a policy trained or fine-tuned with LeRobot or compatible tooling. With M11
cut, **no policy is trained.** PoseNet is a trained model but it is a perception network,
not a policy, and it is not LeRobot. The README must say this in those words and must not
describe the shipped control path as learned (ADR-015 rule 2, F1 done-when 2). This is a
real hole in a 20-point rubric line and I am not going to pretend otherwise; the trade is
that the alternative risks holes in the 30-point completion line, the 15-point robustness
line and the 10-point reproducibility line simultaneously.

**Consequential change to M09a's camera set.** ADR-022 fixed M09's cameras at
`['front', 'armA_wrist', 'armB_wrist']` **because that is M11's training set**. With M11
off the critical path, the camera set is now derived from M10's PoseNet input instead:
`front` plus `drawer_view`, because the drawer is occluded from every other camera
(ADR-021, and M02 done-when 1). Using the measured marginal camera cost of 152.05 ms
(`docs/hardware/m02-render-cost.md:43-49`), that is 0.20 + 2 x 152.05 = **~304.3 ms/step**
rather than ~456 ms/step. This does not contradict ADR-022's Decision (opt-in rendering);
it re-derives the per-module camera set from the consumer that still exists. If F3 revives
ACT, the three-camera set and its ~456 ms/step return with it, and the dataset must be
re-collected — ADR-022's numbers are not superseded, they are deferred.

### 1A.4 Where M03 now sits and what it displaces

**M03 runs first thing on Day 2, 09:00–12:00, before M04.** It is not appended anywhere.

It displaces **M06's Day-2 start**: M06 moves to Day 3 in its entirety, which cascades M07
and M08 to Day 4 and M10 to Day 5. M03 is placed first rather than fitted around other
work for three reasons, all already in the plan:
1. It is "a hard blocker for the whole OpenVINO story (20 rubric points)" and the plan's
   own escalation rule says "if it slips past Day 2, escalate immediately" (M03 budget
   note). Today is Day 2. There is no further slack.
2. ADR-014 scopes it to the first 48 hours. Counting from Day 0 (Sept 10), that window
   closes at end of today.
3. M13's export work depends on M03's findings — specifically whether the NPU demanded
   static shapes and what precision it accepted (M13 Inputs). Discovering that on Day 5
   would be discovering it too late to change the model.

One part of M03 is already done and should not be re-run: the MuJoCo offscreen-rendering
probe folded into it by ADR-020 closed on Sept 11 (`ARCHITECTURE.md` section 5, RISK-03
closed). M03's budget stays at 3 h anyway; the recovered time is float, not a licence to
add scope.

M03 also pays a dividend that the re-plan leans on: if its probe network is a
ResNet18-scale convolutional encoder (M03 done-when 1), then PoseNet in M10 can use the
same backbone shape, and M03's conversion recipe transfers directly to M13. M13's budget
is reduced on that basis.

### 1A.5 The re-planned schedule, with pre-committed clock gates

All times local. Every gate below is a **clock decision made now**, not a judgement made
in the moment. Section 1A.6 is the ladder that fires when a gate is missed.

**Day 2 — Sept 12 (today). 9 builder-hours.**

| Clock | Work | Where |
|---|---|---|
| 09:00–12:00 | **M03** — PyTorch → IR → compile → infer on CPU/GPU/NPU | bm-ptl |
| **14:00 — HARD GATE** | If M03 has not produced an `.xml`+`.bin` IR pair on bm-ptl by 14:00, **stop and escalate to the user**. Do not keep debugging into the evening; a 20-point blocker that has resisted 5 hours needs a decision, not more hours. | — |
| 12:00–15:00 | **M04** — CommandSource + TextCommandSource + stub voice source | laptop |
| 15:00–18:00 | **M05** — RuleGrounder, TaskPlan, `docs/command-grammar.md` | laptop |
| **20:00 — DAY GATE** | All three closed (tester reported)? If yes, Day 3 starts on M06 as planned. If M04 or M05 is open, it carries to Day 3 morning and **rung 1 of the cut ladder fires**. If M03 is open, that is already an escalation under the 14:00 gate. | — |

M03 is bm-ptl work with real wall-clock waits (conversion, compile, per-device infer);
M04 and M05 are pure-Python laptop work with no MuJoCo import and therefore no ADR-020
dependency. The overlap in the table above is deliberate and is the only parallelism this
schedule claims.

**Day 3 — Sept 13. 9 builder-hours.**

| Clock | Work | Where |
|---|---|---|
| 09:00–17:30 | **M06** — IK + `open_drawer`, `pick`, `place`, `handoff` (6.5 h), then `pour` (1.5 h, time-boxed) | bm-ptl |
| **17:30 — POUR GATE** | If `open_drawer`/`pick`/`place`/`handoff` are not all passing by 17:30, `pour` is **dropped now** and not attempted. `pour` is already last on the cut list; `handoff` is not droppable because brief p4 requires a hand-off. | — |
| 18:00–19:00 | **M08 skeleton** — seed loop, per-subtask metric definitions, `manifest.json` schema, camera-derivation rules, against a null executor | laptop (no MuJoCo import) |
| 19:00–20:00 | **GATE-1** — ratification, see the retimed GATE-1 block | — |
| **20:00 — DAY GATE** | GATE-1 outcome written into DECISIONS.md against ADR-008. If M06 is open, **rung 2 fires**. | — |

The 18:00–19:00 M08 skeleton hour exists to keep faith with ADR-006's "harness built
early, not last". The re-plan moves M08's bulk to Day 4 and that is a genuine weakening of
ADR-006 — acknowledged, not hidden. The mitigation is that there is no competing executor
for it to be built around any more, so the risk ADR-006 was insuring against (a harness
written after the numbers exist, to fit them) is smaller than it was.

**Day 4 — Sept 14. 9 builder-hours.**

| Clock | Work | Where |
|---|---|---|
| 09:00–12:00 | **M07** — Randomizer, `configs/randomization.yaml`, `--validate` over seeds 0–9 | laptop for the pure function, bm-ptl for `--validate` |
| **13:00 — COLLECTION GATE** | M07 closed? If yes, launch M09a. If no, M09a slips to 16:00 and **rung 4 fires**. | — |
| 13:00–14:00 | **M09a** — PoseNet frame collection script; launch the run | bm-ptl |
| 14:00–~14:30 | M09a run, unattended. Arithmetic: 5,000 frames x 304.3 ms = 1,522 s ≈ **25 min**. This is a sub-hour job, not an overnight one. | bm-ptl |
| 14:00–18:00 | **M08** — the remaining 4 h on top of Day 3's skeleton | bm-ptl |
| **20:00 — DAY GATE** | M07, M08 and M09a all closed? M09a's frames uploaded for Kaggle? If M09a's frames have not landed by 20:00, **rung 6 fires** (M10 is cut). | — |

**Day 5 — Sept 15. ~9 builder-hours.**

| Clock | Work | Where |
|---|---|---|
| 09:00–12:00 | **M10** — `train_posenet.py`, launch Kaggle training, `VisionPerception` wiring (wiring is authored against M09a's fixed frame schema and does not wait on the checkpoint) | kaggle + laptop |
| 12:00–14:00 | **M12** — PolicyBackend, reduced scope (PoseNet only) | laptop |
| 14:00–17:00 | **M13** — export PoseNet to FP32/FP16 IR; INT8 as the last hour, droppable | laptop export, bm-ptl compile check |
| 17:00–18:00 | **F1** — promote the scripted controller to the documented demo path with `--perception vision` | bm-ptl |
| 18:00 | Repo flipped public (`SUBMISSION.md:20`) | laptop |
| **20:00 — DAY GATE** | Is there a PoseNet checkpoint and at least one IR that compiles on at least one bm-ptl device? If no IR compiles, **rung 6 fires** and Day 6's benchmark target changes to M03's probe network, with the README saying plainly that the benchmarked model is not the one in the demo loop. | — |

**Day 6 — Sept 16. ~10 builder-hours. No slack behind it.**

| Clock | Work | Where |
|---|---|---|
| 08:00–12:00 | **M14** — device x precision sweep, `benchmarks/bmptl-results.md`, **with M16 folded into the same bm-ptl session** (`run_demo.py` end to end, run log, one recorded seed) | bm-ptl |
| 12:00–13:00 | **M17** — bug-hunter pass, time-boxed to the five focus areas, high-severity fixes only | laptop + bm-ptl |
| 13:00–14:00 | **M19** — 10-seed recorded run. Arithmetic, using the measured per-camera cost: 10 seeds x 1000 steps x 152.25 ms with **one** camera (`front`) = **~25 min**. All five cameras would be 10 x 13.5 min = **~2 h 15 min** and does not fit; five-camera capture is used **once**, for the hero/cover shot only. | bm-ptl |
| 14:00–17:00 | **M18** — README, DECISIONS mirror, video script, slides, cover-image brief | laptop |
| 17:00–18:00 | **M20** — submission assembly; **user submits** | laptop |
| **18:00 — FREEZE** | `CONSTRAINTS.md:4`. bm-ptl dies Sept 17 00:15. | — |

**M15 (Speechmatics voice) is not scheduled.** It is the bonus, it is first on the cut
list, and there are no spare hours to give it. It runs only if a day gate closes early,
and it is pre-committed dropped if Day 5's 20:00 gate is missed. The CommandSource
abstraction and the stub voice source still ship in M04, so the ADR-002 seam is real and
testable whether or not Speechmatics is ever wired to it.

### 1A.6 Pre-committed cut ladder (replaces section 6's list)

Fires in order. Each rung is taken **without re-litigation**, by whoever notices the gate
was missed.

1. **Drop M15 (Speechmatics voice) from the plan entirely**, and remove the Speechmatics
   claim from `SUBMISSION.md` and README (`CONSTRAINTS.md:41`).
2. **Drop `pour` from M06.** Consequence to be written down, not swallowed: the brief's
   verbatim example command (p1) no longer executes end to end, and the README must say
   which clause was not implemented and why. ADR-011/ADR-017's no-fluid disclosure stands
   regardless.
3. **Drop INT8 from M13.** Benchmark FP32 and FP16 only; `bmptl-results.md` records INT8
   as not attempted rather than as unsupported.
4. **Reduce M07's randomization axes** to placement, mass, friction and lighting; drop
   shape-variant and background texture. Brief p2 objective 3 names all six, so the README
   states which four were randomized.
5. **Reduce M06's skill set further**, dropping `open_drawer` and re-scoping the demo
   command to the utensil/plate/mug sequence. `handoff` is the last skill standing — brief
   p4 requires a hand-off or complementary dual-arm action.
6. **Cut M10 (PoseNet).** The demo runs `--perception state` and the OpenVINO benchmark
   targets M03's probe network instead. This is ADR-009 option (b), which ADR-009 rejected
   on the grounds that benchmarking an unused model is a weak claim — so taking this rung
   means writing in the README, plainly, that the converted model is **not** in the demo
   loop, and that `StatePerception` is the shipped perception path in contradiction of
   ADR-005's intent. This is the worst rung and it is deliberately last.

### 1A.7 Is the re-scoped plan achievable? — the honest answer

**No, not at Day 1's observed throughput.** The re-scoped plan totals roughly **45–47
builder-hours across 5 days**, i.e. **~9–10 builder-hours per day, every day, for five
consecutive days**, against a planning capacity derived from Day 1 of ~8. The plan carries
roughly **one day of negative float**.

That is the state of it. Two things follow and both are already committed above:
1. The cut ladder in 1A.6 is not decoration. Expect to take rung 1 and rung 2, and plan
   emotionally for rung 3.
2. No further scope may be added between now and Sept 16 — including the "optional 20-min
   render-resolution lever", the `VlmGrounder` stretch of ADR-003, and F3. If a day
   finishes early, the recovered hours go to the next day's module, not to a new one.

This is recorded as **RISK-12** in section 7 and as **ADR-023** in `ARCHITECTURE.md`.

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
scripts/        collect_frames, train_posenet, export_openvino, bench_openvino, run_demo
                (collect_demos + train_act moved to F3 with M09b/M11 — section 1A.3)
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
  workspace, a drawer with a prismatic joint, and **five** manipulable props — plate, mug,
  fork, spoon, and a water bottle. The bottle is not optional decoration: ADR-011 and
  ADR-017 scope `pour` as a bimanual tilt-and-position motion in which arm B holds the mug
  while arm A tilts the bottle over it, so the brief's own example command (p1) cannot run
  end to end without it. Two cameras (overhead + front) plus per-arm wrist cameras if the
  asset allows. This is required deliverable 2 (brief p4) and the substrate for everything
  else.
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
  1. The scene is documented by two artifacts: `docs/images/m02-scene.png` (front
     camera, showing both arms and all five props on the table) and
     `docs/images/m02-drawer-view-open.png` (drawer camera, showing the drawer state
     that the front camera occludes). Together they demonstrate the full task
     workspace. Two artifacts rather than one because no single camera can show both:
     the drawer sits under an opaque tabletop, verified empirically by rendering it
     open from every existing camera (ADR-021, and the drawer_view fix in 9860072).
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
> **SLIPPED FROM DAY 1. NOW RUNS FIRST ON DAY 2, 09:00–12:00, with a hard escalation
> gate at 14:00** (section 1A.4, 1A.5). Verified not shipped on Sept 12: `scripts/`
> contains no `ov_smoke.py` and `benchmarks/` contains no `ov-smoke-notes.md`. It
> displaces M06's Day-2 start, which cascades M07/M08 to Day 4 and M10 to Day 5.
> The MuJoCo offscreen-rendering probe that ADR-020 folded into this module is
> **already done** (RISK-03 closed Sept 11, `ARCHITECTURE.md` section 5) — do not re-run it.
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
  (20 rubric points, `CONSTRAINTS.md:32`). It has now slipped one day. The escalation
  rule is upgraded to a clock: no IR pair on bm-ptl by 14:00 on Sept 12 → stop and
  escalate to the user (section 1A.5).**
- **Dividend to protect.** Use a ResNet18-scale convolutional encoder (done-when 1) so
  that M10's PoseNet can reuse the same backbone shape and M13's export inherits this
  module's recipe. M13's reduced budget assumes this.

---

### Day 2 — **re-planned Sept 12: M03 (carried) + M04 + M05 only**

M06, M07 and M08 have moved off this day. See section 1A.5 for the clock.

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
> **MOVED TO DAY 3 (Sept 13), where it owns the whole day.** With M11 cut (section 1A.3),
> this module is no longer the demonstration generator — **it is the policy.** It is the
> single most valuable module remaining in the plan.
- **Purpose.** Implement each skill as a scripted, IK-driven primitive:
  `open_drawer`, `pick(object, arm)`, `place(object, target, arm)`, `handoff(object,
  from_arm, to_arm)`, `pour(source, into, arm)`. **This module is now single-purpose and
  that purpose is everything:** with M11 cut (ADR-023), M06 is the *only* controller the
  submission ships. It is not a demonstration generator and nothing imitates it — it
  satisfies `CONSTRAINTS.md:50-52` not as a fallback held in reserve but as the shipped
  action-generation path, and every task-success number in the submission comes from it.
  It is therefore the single highest-value module in the plan and must not be deferred.
- **Inputs.** M02 env, M05 `TaskPlan`.
- **Outputs.** `src/bimanual/control/ik.py`, `control/skills_scripted.py`,
  `control/executor.py` (abstract `SkillExecutor` + `ScriptedSkillExecutor`),
  `scripts/run_skill.py`.
- **Agent.** builder. **Runs on.** **bm-ptl.** Corrected Sept 12: this module's done-when
  criteria execute `TableSettingEnv` (`run_skill.py` drives the scene, and criterion 4
  reads MuJoCo contact/limit warnings), and MuJoCo cannot import on the laptop
  (ADR-020). IK maths and skill-state-machine code may be *authored* on the laptop; every
  done-when check runs on bm-ptl. Same correction pattern as M09.
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
- **Budget.** 8 hours, **all of Day 3, 09:00–17:30**, structured as a time-boxed split
  rather than an overrun contingency (section 1A.5):
  - **M06a — 6.5 h, 09:00–15:30.** `ik.py`, `executor.py`, and `open_drawer`, `pick`,
    `place`, `handoff`. **Not droppable.** `handoff` in particular is required by brief
    p4 (at least one hand-off or complementary dual-arm action) and by ADR-010.
  - **M06b — 1.5 h, 16:00–17:30.** `pour` only.
  - **POUR GATE at 17:30.** If M06a's four skills are not all passing by 17:30, `pour` is
    dropped (cut-ladder rung 2) and its consequence is written into the README: the
    brief's verbatim example command no longer runs end to end. Do not trade `handoff`
    for `pour`.
  The previous text allowed pushing M06b to "Day 3 morning"; Day 3 *is* M06's day now, so
  that escape no longer exists. Overrun past 17:30 costs `pour`, not a later day.

#### M07 — Domain randomization and deterministic seed mapping
> **MOVED TO DAY 4 (Sept 14), 09:00–12:00.** Moved off Day 2 because the
> M04→M05→M06→M08 dependency chain is tighter than M07's (M07 depends only on M02), so
> M07 is the cheapest module to displace — this is flag option (b), applied to Day 4
> rather than Day 3 because Day 3 is fully consumed by M06.
- **Purpose.** Seed → `SceneConfig` → scene. Randomize initial object placement, object
  mass, friction, object scale/shape variant, lighting, and table/background texture,
  exactly the axes the brief names on p2 objective 3 and p5 (15 rubric points,
  `CONSTRAINTS.md:33`).
- **Inputs.** M02 env.
- **Outputs.** `src/bimanual/sim/randomize.py` (`SceneConfig` dataclass, `Randomizer`),
  `configs/randomization.yaml` with per-axis ranges, `scripts/dump_seed_configs.py`.
- **Agent.** builder. **Runs on.** **laptop for the pure `seed -> SceneConfig` function
  and its determinism test (stdlib + numpy only, no `mujoco` import — same pattern as
  `scripts/gen_dual_scene.py` under ADR-021); bm-ptl for done-when 3.** Corrected
  Sept 12: done-when 3's `--validate` flag calls `reset()` on all 10 seeds, which is a
  MuJoCo execution and cannot run on the laptop (ADR-020). Do not let `dump_seed_configs.py`
  import `mujoco` at module scope, or the laptop half stops working too.
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

- **Budget.** 4 hours (Day 4, 09:00–12:00 plus the `--validate` run). **COLLECTION GATE
  at 13:00:** M09a cannot launch until this closes; if it is open at 13:00, M09a slips to
  16:00 and cut-ladder rung 4 fires (randomization axes reduced to placement, mass,
  friction, lighting).

#### M08 — 10-seed evaluation harness and episode recorder
> **SPLIT ACROSS DAY 3 EVENING AND DAY 4.** 1 h skeleton on Day 3 18:00–19:00 (seed loop,
> per-subtask metric definitions, `manifest.json` schema, camera-derivation rules, against
> a null executor — no MuJoCo import, laptop), then 4 h on Day 4 14:00–18:00 on bm-ptl.
> This is a real weakening of ADR-006's "Day 2" commitment and is acknowledged as such in
> section 1A.5; the mitigating fact is that with M11 cut there is no second executor for
> the harness to be quietly shaped around.
- **Purpose.** Built early, not on Day 6. Everything downstream — the fallback decision
  at GATE-1, the Torch-vs-OpenVINO accuracy-preservation check the brief demands on p3,
  and the required demo video across 10 seeds (brief p4) — depends on this harness
  existing before there is anything good to evaluate.
- **Inputs.** M06 executor, M07 randomizer.
- **Outputs.** `src/bimanual/eval/harness.py`, `eval/metrics.py`, `eval/recorder.py`,
  `scripts/evaluate.py`. Artifacts per run: `results/<run-id>/summary.json`,
  `per_seed.csv`, `seed_<n>.mp4`, and `manifest.json` capturing git SHA, host, device,
  precision, and the command text used.
- **Agent.** builder. **Runs on.** **bm-ptl.** Corrected Sept 12 to remove the ambiguity
  in the old "laptop (must also run unchanged on bm-ptl)" parenthetical: every done-when
  criterion here runs episodes through `TableSettingEnv`, and MuJoCo cannot import on the
  laptop (ADR-020), so **no done-when criterion of this module can be satisfied on the
  laptop**. Only the Day-3 skeleton hour (null executor, no `mujoco` import) is laptop
  work. There is no "must also run on bm-ptl" — bm-ptl is the only place it runs.
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
  5. **Camera selection is derived from the executor, not from a global default**, per
     ADR-022's corrected Consequences. The two scoring modes are not interchangeable:
     - `--executor scripted` → `cameras=None`. State-only is *correct* here: the
       scripted controller reads `get_state()` and never consumes an image.
       0.20 ms/step, physics only.
     - `--executor learned` → cameras **required**, and the set must equal M11's
       training-time set (`front`, `armA_wrist`, `armB_wrist`), ~456 ms/step. Tester
       verifies that requesting `--executor learned` with rendering disabled **fails
       loudly** rather than feeding the policy an image-free obs dict. An ACT policy
       cannot produce an action without images, so a silent state-only learned run is a
       defect, not a fast path.
     - Video capture → all five cameras (~809.86 ms/step), enabled **only** on the final
       winning seed, once, not per seed of every run.
  6. Every `summary.json` / `manifest.json` records which cameras were rendered, so a
     scripted-branch number and a learned-branch number can never be compared without the
     difference in observation streams being visible.
- **Budget.** 5 hours (1 h Day 3 skeleton + 4 h Day 4). **DAY GATE 20:00 Day 4:** if M08
  is open, it has consumed the module that M14, M19 and GATE-1's evidence all rest on, and
  cut-ladder rung 3 fires.
- **Note on done-when 5 after the M11 cut.** The `--executor learned` branch of done-when 5
  stays in the code and stays tested (a learned executor requested without cameras must
  fail loudly), even though no learned executor will exist on the shipped branch. It costs
  little, it is the guard F3 would need if ACT is ever revived, and deleting a safety check
  because the unsafe path is currently unreachable is how it comes back unguarded.

---

### Day 3 — **superseded: this day is now M06 + the M08 skeleton + GATE-1 (section 1A.5)**

> **The schedule table immediately below is the Sept 11 plan and is retained as the record
> of how the overnight-collection plan was costed. It no longer describes Day 3.** M09's
> overnight collection is cut (section 1A.3), so the Day-2-night launch, the 08:00 stop and
> the 08:00–13:00 M10 slot described below do not happen. The arithmetic in it is still
> correct and is still the basis for F3's cost if ACT is ever revived.

**Day 3 schedule, re-planned after the ADR-022 correction (Sept 11).** At the corrected
~456 ms/step, M09's collection is ~8.9 h of bm-ptl wall clock for 70,000 steps, and the
learned half of GATE-1 is a further ~76 min. **Day 3 cannot hold a 9-hour daytime M09
alongside M10, M11 and an 80-minute gate — 9 + 5 + 76 min already overruns the day before
M11's Kaggle queue is counted.** The reshuffle, stated rather than absorbed silently:

| Slot | Work | Where |
|---|---|---|
| Day 2, on M08 close | Launch M09 collection, unattended, after a 15-min smoke run | bm-ptl |
| Day 2 night → Day 3 08:00 | M09 collects (~9 h of otherwise-idle wall clock) | bm-ptl |
| Day 3 08:00 | Stop collection on the clock; validate + upload dataset | bm-ptl → kaggle |
| Day 3 08:00–13:00 | M10 (laptop wiring can have started during collection) | laptop + kaggle |
| Day 3 ~12:30 | Launch M11 ACT training | kaggle |
| Day 3 18:30 | Start GATE-1: scripted (2 s) + learned (~76 min) | bm-ptl |
| Day 3 ~20:00 | GATE-1 decision recorded in DECISIONS.md | — |

**What I recommend cutting if this still does not fit: the dataset, not the gate and not
M10.** Degrade M09 to the pre-committed 4 h / ~31,500-step run and accept a smaller
demonstration set. The rubric cost is concentrated and bounded: a thin dataset mainly
raises the probability that GATE-1 selects the scripted branch, and the scripted branch
already carries end-to-end completion and bimanual coordination (30 pts, via M06+M07),
OpenVINO (20 pts, via M10+F2), robustness across 10 seeds (15 pts, via M07+M08) and
reproducibility (10 pts). The exposure is a slice of the 20-pt VLA/multi-modal line —
and even there, M05's grounder plus M10's OpenVINO-served perception still answer it.
By contrast, cutting M10 would put the 20-pt OpenVINO criterion on the ML branch's
critical path, which is exactly the dependency F2 exists to break; and skipping the
80-minute gate would mean choosing a branch without evidence. **Do not cut M10. Do not
shorten the gate. Cut dataset size.**

I have measured none of this. The 0.20 / 152.25 / 809.86 ms/step figures are Tester's
(`docs/hardware/m02-render-cost.md:11-15`); every duration above is arithmetic on them,
not an observation of a collection run.

#### M09 — Dataset collection — **split Sept 12 into M09a (kept) and M09b (cut)**

**M09a — PoseNet training frames. KEPT, on the critical path, Day 4 13:00–14:00.**
- **Purpose.** Produce labelled images for M10's PoseNet: randomized `reset()` frames plus
  frames at perturbed arm poses, each paired with privileged ground-truth object poses and
  drawer opening. This is a *per-frame supervised regression* dataset, not a trajectory
  dataset, so it does not need the scripted controller and does not need rollouts.
- **Inputs.** M02 env, M07 randomizer. **Not** M06, **not** M08 — this is the dependency
  break that makes the re-plan fit.
- **Outputs.** `scripts/collect_frames.py`, a frame dataset on disk plus a
  Kaggle-uploadable archive, and `docs/dataset-card.md` (frame count, cameras, resolution,
  label schema, seeds used).
- **Agent.** builder. **Runs on.** bm-ptl (upload target: kaggle).
- **Cameras.** `['front', 'drawer_view']` — derived from M10's PoseNet input, not from
  M11's training set, because M11 is cut (section 1A.3). The drawer is occluded from the
  front camera (ADR-021, M02 done-when 1), and `SceneBelief` carries drawer opening
  (`ARCHITECTURE.md` component table), so two cameras is the floor, not a preference.
  Cost: 0.20 + 2 x 152.05 = **~304.3 ms/step**
  (`docs/hardware/m02-render-cost.md:43-49`). 5,000 frames ≈ **25 min** of bm-ptl wall
  clock — a sub-hour unattended job, not an overnight one.
- **Done when.**
  1. Frame count, camera names, resolution and label schema are recorded in
     `docs/dataset-card.md`, and the schema is frozen before M10 starts wiring against it.
  2. The seeds used for frames and the evaluation seed set 0–9 are **disjoint**, asserted
     in code (same rule as M09b done-when 3 below — ADR-012's disjointness requirement is
     about generalization and does not care which dataset it is).
  3. A spot-check script renders 5 random frames with their labels overlaid, so a
     mislabelled axis or a frame/label off-by-one is visible rather than latent.
  4. The collector writes append-only and is resumable (same rationale as M09b below).
- **Budget.** 1 h builder + ~25 min unattended bm-ptl wall clock.
- **Known weakness, flagged not hidden (RISK-13).** Frames drawn from `reset()` and
  perturbed poses do not cover mid-manipulation occlusion by the arms, which is exactly
  what the demo loop feeds PoseNet. If M06 is closed by Day 4, add frames sampled from
  scripted rollouts within the same 25-minute budget. I have measured no accuracy and
  claim none.

**M09b — LeRobot scripted-demonstration dataset. CUT from the critical path Sept 12.**
Retained below in full because it is F3's specification if ACT is ever revived, and
because its budget block is where the pre-committed overnight clock rules live.

- **Purpose.** Run the scripted controller over many randomized seeds and log
  observation/action pairs in LeRobot dataset format, so ACT can be trained
  (brief p2 objective 4: train or fine-tune using LeRobot or compatible tooling).
  **Collection runs with cameras enabled — `['front', 'armA_wrist', 'armB_wrist']`.**
  The scripted controller reads `get_state()` to *choose* actions (ADR-005); the camera
  frames are *logged* because M11's ACT policy is camera-conditioned and M10's PoseNet
  trains on images. ADR-005 governs action selection, not dataset contents — see the
  correction paragraph in ARCHITECTURE ADR-022's Consequences.
- **Inputs.** M06, M07, M08.
- **Outputs.** `scripts/collect_demos.py`, a dataset on disk plus a Kaggle-uploadable
  archive, and `docs/dataset-card.md` (episode count, cameras, resolution, action space,
  seeds used, success filter applied).
- **Agent.** builder. **Runs on.** **bm-ptl** (dataset upload target: kaggle). Corrected
  from "laptop": MuJoCo cannot import on the laptop (ADR-020, `DECISIONS.md:147-170`), and
  the 456 ms/step render cost is Arc B390 time on bm-ptl. This wall clock is **not
  recoverable** — bm-ptl expires Sept 17 00:15 (`CONSTRAINTS.md:5`).
- **Depends on.** M06, M07, M08.
- **Done when.**
  1. Only successful episodes are kept; the filter criterion and the kept/attempted
     counts are reported by Tester.
  2. A `scripts/replay_episode.py` replays a logged episode and reports max joint
     deviation from the log, proving actions and observations are aligned in time
     (off-by-one here silently destroys imitation learning).
  3. The training seed set and the evaluation seed set 0–9 are **disjoint**, asserted in
     code. Evaluating on training seeds would invalidate the robustness claim.
  4. Dataset card states exactly which observation streams the policy will receive, and
     names the three rendered cameras and their resolution explicitly.
  5. The collector writes **episode-by-episode, append-only, and is resumable**: killing
     it mid-run loses at most the in-flight episode, and re-launching continues rather
     than restarting. Tester verifies by interrupting a short run and resuming it. This
     is what makes the unattended overnight window in the budget below safe to take.

- **Budget (revised — supersedes "4 hours, mostly wall-clock collection").**
  > **Applies to F3 only as of Sept 12.** The Day-2-night launch this budget assumes is
  > impossible: M09b needs M06 + M07 + M08, and under section 1A those close on Day 4
  > evening at the earliest. The clock rules below are retained *verbatim in force* so
  > that if F3 revives ACT there is a pre-committed launch/abort clock and nobody has to
  > invent one at 22:00 with a Kaggle queue in front of them.
  **0.5 h attended setup + a bounded unattended collection window on bm-ptl.**
  At the corrected ~456 ms/step (ADR-022; `docs/hardware/m02-render-cost.md:11-15`),
  70,000 attempted steps is **~8.9 h** of wall clock (70,000 x 0.456 s = 31,920 s), and a
  4 h window buys **~31,500 attempted steps** (14,400 s / 0.456 s).
  **Decision: take the overnight window, not the 4-hour Day-3 slot.**
  - **Primary plan.** Launch collection on bm-ptl as soon as M08 closes on Day 2
    (Sept 12) and let it run unattended overnight into Day 3 morning. Collection is
    bm-ptl wall clock, not developer hours, so ~9 h costs nothing from the Day-3 working
    day and leaves Day 3 intact for M10 and M11. Target 70,000 attempted steps; **the
    stop condition is the clock (08:00 Day 3), not the step count** — done-when 5 makes
    whatever has landed by then a usable dataset.
  - **Mandatory pre-flight (15 min, attended).** A 2-episode smoke run with cameras on,
    checked for correct frame shapes, non-blank images, and time-aligned action logging
    (done-when 2), before the unattended run starts. An unsupervised 9 h run that was
    broken at minute one is the single worst outcome available on Day 3.
  - **Pre-committed launch clock (closes the "in time" ambiguity, Sept 12).** "In time"
    previously had no definition, which meant the most consequential decision in the plan
    was a judgement made late at night by a tired developer. It is now three clock times,
    fixed in advance, on the calendar day that precedes the intended collection:
    - **M08 closed by 22:00** → launch the full overnight run. Target 70,000 attempted
      steps (~8.9 h at ~456 ms/step); **hard stop 07:00** the next morning whatever the
      step count, because the stop condition is the clock, not the count.
    - **M08 closes between 22:00 and 23:00** → launch at a **reduced target of 60,000
      attempted steps (~8.2 h)**, hard stop **07:00**. Do not attempt the full target in a
      shorter window and do not extend past 07:00.
    - **M08 closes after 23:00, or has not closed** → **take the fallback. Do not launch.**
      No "it's only half an hour late" exception. A collection run started at midnight
      finishes after the following morning's work has already been planned around its
      absence.
    - **Fallback shape:** a **4 h next-morning run at ~31,500 attempted steps**
      (14,400 s / 0.456 s), and the dataset ships at that size. No extension into the
      afternoon: M11 must launch on Kaggle by ~12:30 to have a checkpoint for GATE-1.
      **See the M10-in-fallback rule immediately below — the fallback re-plans M10 too.**
  - **Pre-committed M10 re-plan under the fallback (added Sept 12).** In the fallback the
    dataset lands at ~12:00, but M10's old slot was 08:00–13:00, which left one hour for a
    five-hour module. That was an unclosed hole. The rule, decided now:
    **M10 does not slide and does not compress — it is re-ordered.** M10's builder-side
    work (`train_posenet.py`, `PerceptionBackend`, `StatePerception`, `VisionPerception`
    wiring, the Kaggle kernel spec) is authored **08:00–12:00 against the frozen frame
    schema**, which exists before any frame does. Only the *training launch* waits for the
    dataset, and training is Kaggle wall clock, not developer hours. At 12:00 the dataset
    uploads and training launches unattended; M10's done-when 1 (held-out error reported)
    is verified whenever the run returns, including after GATE-1. **M11 is what the
    fallback squeezes, not M10** — the shorter dataset is M11's problem and GATE-1 is
    where that is priced. Sliding M10 into the afternoon would push M11's launch past
    12:30 and break GATE-1; compressing M10 would compromise the one model that keeps
    OpenVINO load-bearing on the scripted branch (ADR-009). Neither is acceptable, so
    neither is on the table.
  - **Attempted vs kept.** 456 ms/step is charged on *attempted* steps; done-when 1 keeps
    only successful episodes, so kept steps are strictly fewer by the success-filter
    ratio. That ratio is **unmeasured** — it is an output of M06, not an assumption here.
    Tester reports attempted-vs-kept; if it is poor, the dataset shrinks and that is
    reported honestly rather than fixed by overrunning the window.
  - **Optional 20-min lever, time-boxed, unverified.** The 152 ms/camera figure is at the
    env default 640x480, and ACT consumes a much smaller input. Rendering at ACT's input
    resolution *may* cut per-frame cost if the dominant term is pixel readback
    (suspect (a), `docs/hardware/m02-render-cost.md:58-63`). **I have not measured this
    and claim no speedup.** Measure it once in the pre-flight; if it does not clearly
    help, proceed at 640x480 unchanged and do not investigate further.
  - **Correction to the old "work M10 while it runs".** M10 *depends on* this dataset
    (M10 Inputs, below), so only M10's **laptop-side wiring** (`PerceptionBackend`,
    `StatePerception`, `VisionPerception` plumbing) overlaps with collection. M10's
    **training** cannot start until the dataset exists and is uploaded.

#### M10 — Vision perception model (PoseNet) and its OpenVINO path
- **Purpose.** A small CNN mapping camera images to object poses / keypoints, trained on
  sim data with ground-truth labels. This exists for a structural reason: it makes
  OpenVINO **load-bearing on the fallback branch too**. If ACT fails at GATE-1, the
  scripted controller still consumes vision through an OpenVINO-compiled network on
  iGPU/NPU, so the 20-point OpenVINO criterion (`CONSTRAINTS.md:32`) is not contingent on
  the risky ML. See ADR-005 and ADR-009.
> **MOVED TO DAY 5 (Sept 15), 09:00–12:00.** With M11 cut, this is now the **only trained
> model in the submission**, and therefore the sole basis for the claim that OpenVINO does
> real work in the demo loop (ADR-009). It is protected: it is rung 6 — the last rung — of
> the cut ladder.
- **Inputs.** **M09a** frames (images + privileged ground-truth poses), M03 conversion
  recipe. Changed Sept 12 from "M09 dataset": M09b is cut, and PoseNet needs labelled
  frames, not trajectories.
- **Outputs.** `src/bimanual/perception/backend.py` (abstract `PerceptionBackend`),
  `perception/state_perception.py` (privileged, dev only),
  `perception/posenet.py`, `perception/vision_perception.py`,
  `scripts/train_posenet.py`, a checkpoint.
- **Agent.** builder. **Runs on.** kaggle for training, laptop for wiring, **bm-ptl for
  done-when 2** (running a full `TaskPlan` with `--perception vision` executes the env,
  which the laptop cannot do — ADR-020).
- **Depends on.** M03, **M09a**.
- **Done when.**
  1. Training completes and Tester reports held-out position error in metres (a measured
     number, no target asserted here).
  2. `ScriptedSkillExecutor` runs a full `TaskPlan` with
     `--perception vision` on at least one seed, i.e. no privileged state in the loop.
  3. `StatePerception` is gated behind an explicit `--perception state` flag and the
     README will state plainly that it is a development aid, not the demo path.
- **Budget.** **3 hours builder** (reduced from 5 on Sept 12) + Kaggle wall clock. The
  reduction rests on two structural facts, not on optimism: M03's ResNet18-scale
  conversion recipe transfers directly to a ResNet18-backbone PoseNet, and the wiring is
  authored against M09a's frozen frame schema rather than against a finished checkpoint.
  If the reduction proves wrong, the hours come out of M12, not out of M14 or M19.

#### M11 — ACT policy training on Kaggle — **CUT FROM THE CRITICAL PATH, Sept 12**

> **This module is not scheduled.** It moves wholesale into **F3** (optional revival after
> M18, under F3's existing hard stop). Reason, in full, in section 1A.3: M11 needs M09b,
> M09b needs M06 + M07 + M08, and those close on Day 4 evening at the earliest, which
> would put GATE-1 on Day 5 night and leave the benchmark, the bm-ptl pipeline run, the
> 10-seed recording, the docs and the submission all stacked on Day 6.
> `CONSTRAINTS.md:50-52` authorizes shipping the scripted controller as the policy, and
> the re-plan takes that authorization now, on schedule evidence.
> **Cost, stated plainly:** brief p2 objective 4 (train or fine-tune a policy with LeRobot
> or compatible tooling) goes unmet. PoseNet is trained, but it is a perception network,
> not a policy, and it is not LeRobot. The README says so in those words.
> The specification below is retained unchanged as F3's input.

- **Purpose.** Fine-tune/train an ACT-style imitation policy (LeRobot) on the M09 dataset,
  conditioned on camera observations, joint state, and the language/skill token from the
  grounder. This is the learned-policy branch (brief p2 objective 4).
- **Inputs.** M09 dataset, LeRobot, Kaggle GPU (`CONSTRAINTS.md:57`: 30 hrs/week free,
  CLI-driven; token present at `.kaggle/access_token`, ignored by `.gitignore:25`).
- **Render budget (read before sizing the dataset).** ACT is conditioned on camera
  observations, so the M09 dataset must be collected **with** cameras — the render cost
  lands on M09 collection, not on this module. Kaggle training reads pre-rendered frames
  and never invokes MuJoCo. `TableSettingEnv` with three cameras (`front`, `armA_wrist`,
  `armB_wrist`) costs **~456 ms/step** per ADR-022's measured numbers (0.20 ms physics +
  3 x 152.05 ms/camera). At that rate ~70,000 **attempted** steps is about 9 hours of
  wall-clock collection on bm-ptl. ("Attempted", corrected Sept 12 — this note previously
  said "collected". M09b done-when 1 keeps only successful episodes, so kept steps are
  strictly fewer by an **unmeasured** success-filter ratio; 456 ms/step is charged on
  attempted steps, and sizing the dataset off a "collected" figure would silently
  over-promise the dataset by whatever that ratio turns out to be.) Size the
  demonstration dataset to fit one collection
  window, or plan a two-session split — and note bm-ptl expires Sept 17
  (`CONSTRAINTS.md:5`), so this wall-clock is not recoverable.
  **Resolved Sept 11.** This note previously flagged a contradiction with ADR-022's
  "M09 runs with no cameras". ADR-022's Consequences have since been corrected: ADR-005's
  privileged state governs how the scripted controller *chooses* actions and does not
  permit the logged dataset to omit images. M09 now collects with three cameras at
  ~456 ms/step, on the overnight window set out in M09's revised budget. No open
  contradiction remains.
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

### GATE-1 — End of Day 3 (Sept 13), 19:00–20:00: learned or scripted

`CONSTRAINTS.md:50-52` — "If learned policy is not working by end of Day 3, ship a
scripted IK-based controller as the 'policy.' Rubric rewards complete pipeline over
half-working ML."

**Retimed and re-based Sept 12. The gate stays on Day 3, but its evidence changes.**
`CONSTRAINTS.md:50-52` fixes the *date* of this decision, and the re-plan honours it. What
the re-plan cannot honour is the *basis*: on Day 3 evening there will be no ACT
checkpoint, no demonstration dataset and no completed M08, because M11 and M09b are cut
(section 1A.3) and M08's bulk is Day 4. So GATE-1 is decided on **schedule evidence**
rather than on `scripts/evaluate.py` output.

**The expected outcome is pre-committed: the scripted branch (F1/F2).** Recording it as a
ratification rather than pretending it is an open contest is the honest form. The gate is
still held, and still held on Day 3, because the decision has to be *written down* against
ADR-008 with its reasoning — and because there is one way it could go otherwise (below).

**Inputs to the gate, all of which exist by 19:00 on Day 3:**
1. Does an ACT checkpoint exist? (Pre-committed answer: no. If somehow yes, run the
   80-minute learned evaluation and apply the numeric criteria below unchanged.)
2. Did M06 close by 17:30, and with or without `pour`?
3. Was `handoff` demonstrated working? If not, brief p4's hand-off requirement is at risk
   and that is escalated to the user at this gate, not discovered on Day 6.

**Decision owner:** the **user**, on planner's recommendation. Record the outcome in
DECISIONS.md against ADR-008, including the fact that the branch was chosen on schedule
grounds — that sentence must survive into the README, because "we chose scripted because
we ran out of days" and "we chose scripted because learning underperformed" are different
claims and only one of them is true.

**Day 3 closes before GATE-1 in both branches — confirmed:**
- **Primary branch (no ACT, the expected case):** M06 runs 09:00–17:30, M08 skeleton
  18:00–19:00, GATE-1 19:00–20:00. The 80-minute learned-branch evaluation is not run
  because there is no learned executor, so the gate costs ~1 h, not ~80 min of bm-ptl wall
  clock. Day 3 closes at 20:00.
- **Fallback branch (M06 overruns):** the 17:30 pour gate drops `pour` and M06a's four
  skills are what ships. GATE-1 still starts at 19:00 with the same three inputs, and the
  answer is the same. If even M06a is open at 19:00, GATE-1 is still held and records that
  the scripted branch is chosen **and is incomplete**, with cut-ladder rung 5 fired. In no
  branch does GATE-1 slide.

The paragraphs below retain the full learned-branch criteria and the 80-minute wall-clock
budget. They are not dead text: they are exactly what F3 must satisfy if ACT is revived
after M18, and they are what input 1 above falls back to in the unlikely event a
checkpoint exists.

**Timing budget for the gate itself: reserve 80+ minutes of bm-ptl wall clock, and start
it no later than 18:30 on Day 3.** The gate is not free, and the earlier reading of
ADR-022 that implied it costs ~2 seconds was wrong (see ADR-022's correction paragraph).
Per M08 done-when 5:
- **Scripted branch**, `cameras=None`: 10 seeds x 1000 steps x 0.20 ms ≈ **2 s**.
- **Learned branch**, three cameras at ~456 ms/step: 10 x 1000 x 0.456 s = 4,560 s ≈
  **76 min**. This is not optional — a learned executor cannot be scored state-only.
- **Both branches, as the gate requires for a comparison:** ≈ **76 min**, so budget
  **80+ min** including setup and artifact writing.
- Video capture is **not** part of the gate. All five cameras at ~809.86 ms/step is
  ~13.5 min per 1000-step episode (`docs/hardware/m02-render-cost.md:47-51`); it runs
  once, on the final winning seed, in M19 — never across the gate's 10 seeds.

If the 80-minute learned-branch run cannot start by 18:30 on Day 3, **do not slide the
gate into Day 4** — take the scripted branch on the evidence available and record the
timeout itself as the deciding reason in DECISIONS.md. `CONSTRAINTS.md:50-52` sets the
end of Day 3 as the decision point; a gate that slips is the failure mode the fallback
exists to prevent.

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

### Day 4 — **superseded: Day 4 is now M07 + M08 + M09a (section 1A.5)**

> M12, M13 and M14 move to Days 5 and 6. M15 is unscheduled. The module specifications
> below are unchanged except where a budget or day is explicitly amended.

#### M12 — PolicyBackend abstraction (Torch and OpenVINO)
> **MOVED TO DAY 5, 12:00–14:00. Budget reduced from 4 h to 2 h.** With M11 cut there is
> exactly one model to serve — PoseNet — so this is a two-backend wrapper over one network
> rather than over a policy and a perception net. The `describe()`-reports-the-compiled-
> device requirement (done-when 2) is **not** part of the reduction; that is the whole
> point of the module and is a named bug-hunter target.
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
- **Budget.** ~~4 hours~~ **2 hours** (Day 5, 12:00–14:00).

#### M13 — OpenVINO export and INT8 quantization
> **MOVED TO DAY 5, 14:00–17:00. Budget reduced from 5 h to 3 h**, on the basis that M03's
> conversion recipe transfers directly to a ResNet18-backbone PoseNet and that the
> calibration set now comes from M09a's frames rather than from a trajectory dataset.
> **INT8 is the last hour and is cut-ladder rung 3.** If it is not done by 17:00, it is
> dropped and `bmptl-results.md` records INT8 as not attempted rather than as unsupported
> — those are different claims (ADR-007, ADR-015 rule 6).
- **Purpose.** Convert the selected model(s) to IR at FP32 and FP16, and produce an INT8
  variant via NNCF post-training quantization using a calibration set drawn from the M09
  dataset. Precision/quantization choices are explicitly scored (brief p5).
- **Inputs.** M12, M03 notes (especially whether NPU demanded static shapes), **M09a
  frames** for calibration (changed Sept 12 from "M09 data" — M09b is cut).
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
- **Budget.** ~~5 hours~~ **3 hours** (Day 5, 14:00–17:00), INT8 being the droppable last
  hour. Done-when 1 is relaxed accordingly if rung 3 fires: **two** IR variants (FP32,
  FP16) rather than three.

#### M14 — bm-ptl benchmark script and results table
> **MOVED TO DAY 6, 08:00–12:00, with M16 folded into the same bm-ptl session.** Both are
> bm-ptl work under one SSH login, both need the same IR artifacts, and Day 6 has no room
> for two separate remote sessions. Budget reduced from 5 h to 4 h covering both, on the
> basis that `run_demo.py` and the benchmark sweep share their setup. **This is the single
> most schedule-critical block left in the plan: bm-ptl dies Sept 17 00:15
> (`CONSTRAINTS.md:5`) and there is no day after this one.** If 12:00 arrives with no
> results table, stop optimising and publish whatever rows exist, with `UNSUPPORTED` and
> "not attempted" used honestly for the rest.
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
- **Budget.** ~~5 hours~~ **4 hours covering M14 + M16 together** (Day 6, 08:00–12:00).

#### M15 — VoiceCommandSource (Speechmatics) — droppable
> **UNSCHEDULED as of Sept 12, and cut-ladder rung 1.** There are no spare hours in
> section 1A.5 to give it. It runs only if a day gate closes early, and it is
> pre-committed dropped if the Day-5 20:00 gate is missed — at which point the
> Speechmatics claim comes out of `SUBMISSION.md` and README (`CONSTRAINTS.md:41`).
> **The ADR-002 seam survives regardless:** M04 ships `CommandSource`, `TextCommandSource`
> and a stub voice source satisfying the same abstract base (M04 done-when 3), and M04
> done-when 2's grep for audio terms in `language/` and `policy/` still runs. The
> abstraction is a Day-2 deliverable; only the Speechmatics implementation is at risk.
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

### Day 5 — **superseded: Day 5 is now M10 + M12 + M13 + F1 + repo public (section 1A.5)**

> M16, M17 and M18 move to Day 6.

#### M16 — Full pipeline on bm-ptl
> **MOVED TO DAY 6, folded into M14's 08:00–12:00 bm-ptl session.** Budget absorbed into
> M14's 4 hours. **The "day of slack before the instance expires" in the purpose below no
> longer exists** — that sentence was written when this module sat on Day 5. It now runs
> on the last available day. That is a real loss of insurance and is the sharpest edge in
> the re-plan; it is why the Day-5 20:00 gate insists on at least one IR compiling before
> Day 6 begins.
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
- **Budget.** ~~5 hours~~ **absorbed into M14's 4-hour Day-6 session.** **Hard deadline.
  bm-ptl is gone after Sept 16 (`CONSTRAINTS.md:5`).**

#### M17 — Bug-hunter pass and triage
> **MOVED TO DAY 6, 12:00–13:00. Budget reduced from 4 h to 1 h**, time-boxed to probing
> the five focus areas and fixing only what threatens the recorded demo. Done-when 1 is
> unchanged — all five areas must still be *probed and reported on*, because an unprobed
> area reported as clean would be a fabricated claim. Done-when 2's escape hatch ("written
> into README as a known limitation") is the pressure valve, not skipping the probe.
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
- **Budget.** ~~4 hours~~ **1 hour** (Day 6, 12:00–13:00).

#### M18 — Documentation, submission assets, repo public
> **MOVED TO DAY 6, 14:00–17:00. Budget reduced from 6 h to 3 h**, on the basis that
> docs-writer already runs a per-module pass under section 2's standing rule, so README
> status, DECISIONS mirroring and `SUBMISSION.md` checkboxes accumulate daily rather than
> being written on Day 6. **The repo-public step moves to Day 5 18:00**, matching
> `SUBMISSION.md:20`, and is therefore *not* part of this module's Day-6 budget — it must
> not be the thing that slips on the last afternoon. ADR-019's pre-publication git-history
> credential scan runs before that flip.
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
  5. **Added Sept 12.** README states that no policy was trained (brief p2 objective 4
     unmet), that the shipped control path is a scripted closed-loop controller over
     learned visual perception, and that the branch was chosen on schedule grounds at
     GATE-1. ADR-015 rule 2 forbids calling it learned; this criterion additionally
     forbids omitting the fact.
- **Budget.** ~~6 hours~~ **3 hours** (Day 6, 14:00–17:00), with the repo-public step
  pulled forward to Day 5 18:00.

---

### Day 6 — **now also holds M14+M16, M17 and M18 (section 1A.5)**

#### M19 — Final 10-seed recorded run and behaviour-preservation check
> **Retimed to Day 6, 13:00–14:00, and the camera set is now pre-committed.** Budget
> reduced from 5 h to 1 h attended plus ~25 min of bm-ptl wall clock, on this arithmetic
> from the measured per-camera cost (`docs/hardware/m02-render-cost.md:43-58`):
> - 10 seeds x 1000 steps with **one** camera (`front`): 10 x 152.25 s = **~25 min**.
> - 10 seeds x 1000 steps with **all five** cameras: 10 x 13.5 min = **~2 h 15 min**.
>
> Two hours fifteen is not available on Day 6. **The ten seed videos are recorded with the
> `front` camera only; all five cameras are used once, on the hero/cover shot.** This is
> consistent with M08 done-when 5's third bullet and with ADR-022's "once, on the final
> winning seed" rule — it is that rule applied with its arithmetic written out. Seed
> videos must still be individually captioned with seed, command and outcome (done-when 1)
> and failures must still be shown (ADR-018); a single camera does not license a
> highlight reel.
- **Purpose.** Required deliverable 4 (brief p4): a video across 10 randomized seeds,
  with command, scene variation and outcome verifiable. Plus brief p3's "preserve system
  behavior" — optimization must not materially degrade task success, which requires the
  comparison to actually be run.
- **Inputs.** M08 harness, M16 pipeline, M13 precisions.
- **Outputs.** `results/final/` with 10 videos, `per_seed.csv`, `summary.json`, a stitched
  demo video, and a Torch-vs-OpenVINO side-by-side success comparison table.
- **Agent.** tester runs; docs-writer writes it up.
- **Runs on.** bm-ptl.
- **Depends on.** M16, M17. **No longer M18** (changed Sept 12): M18 now runs *after* M19
  at 14:00–17:00 on Day 6, because the README must quote M19's measured 10-seed result
  (M18 done-when 2), not the other way round. The old ordering was circular.
- **Done when.**
  1. Ten videos exist, each captioned with seed, command text and outcome.
  2. The success count is reported honestly — including partial and failed seeds. Brief
     p4 asks for the success rate to be summarized; a truthful sub-10/10 with per-subtask
     breakdown is a valid and defensible submission.
  3. The same 10 seeds are evaluated under the Torch backend and the chosen OpenVINO
     precision, and both success counts appear in the table.
- **Budget.** ~~5 hours~~ **1 hour attended + ~25 min bm-ptl wall clock** (Day 6,
  13:00–14:00). Do not let this be the 22:00 task; at 14:00 it is done or it ships with
  whatever seeds completed, honestly labelled.

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
- **Budget.** ~~3 hours~~ **1 hour** (Day 6, 17:00–18:00). **Freeze at 18:00 local
  Sept 16.** The reduction is only defensible because M18 now accumulates daily rather
  than landing in a block; if `SUBMISSION.md` still has TODOs at 17:00 on Day 6, the
  submission goes out with them explicitly struck as not-applicable rather than late.

---

## 5. Fallback branch — **now the primary branch, activated Sept 12**

> **Status change, Sept 12.** These are no longer contingent on a GATE-1 that chooses
> scripted. Section 1A.3 cuts M11, so F1 and F2 *are* the plan. F1 is scheduled on Day 5
> 17:00–18:00 and F2 is folded into M13/M14's work. F3 is where M09b and M11 now live and
> is the only place ACT may be attempted.

These replace M11's role. They are pre-specified now so that no day is spent
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
- **Agent.** builder + docs-writer. **Runs on.** laptop for wiring, **bm-ptl for
  done-when 1** (a 10-seed run executes the env — ADR-020).
- **Depends on.** GATE-1 = scripted. **Pre-committed as the expected outcome (section
  1A.3); scheduled Day 5 17:00–18:00 with a 1-hour budget rather than 4**, because with
  M11 cut there is no learned path to unwire — F1 reduces to setting the default flags,
  running the 10 seeds, and writing the naming discipline into the README.
- **Done when.**
  1. `scripts/evaluate.py --seeds 0-9 --executor scripted --perception vision` completes
     all 10 seeds with a reported per-subtask breakdown.
  2. The README and video script never use the words "learned policy", "VLA policy" or
     "imitation learning" to describe the shipped control path. Naming discipline is the
     whole point of an honest fallback.
  3. Where the multi-modal reasoning points come from is stated plainly: language
     grounding (M05) plus OpenVINO visual perception (M10), not action generation.
- **Budget.** 1 hour (Day 5, 17:00–18:00). Reduced from 4 on Sept 12: with M11 cut
  (ADR-023) there is no learned path to unwire, so F1 reduces to setting the default
  flags, running the 10 seeds, and writing the naming discipline into the README.

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
> **F3 is now the only home for M09b and M11** (section 1A.3). Their full specifications
> are retained in section 4 as F3's inputs, including M09b's pre-committed 22:00 / 23:00
> launch clock and its 07:00 hard stop.
- **Purpose.** If the plan finishes early, retry ACT with demonstrations collected from
  the scripted controller.
- **Rule.** Allowed only after M18 is complete. If it does not beat the scripted branch
  on the 10 evaluation seeds by Day 6 12:00, it is abandoned and never mentioned in the
  submission as working. Do not re-record the demo video after 14:00 on Day 6.
- **Rule, added Sept 12.** Given section 1A.5, M18 completes at 17:00 on Day 6 and the
  freeze is 18:00. **F3 is therefore realistically unreachable and should be treated as
  cut, not as a plan.** It is retained so that the ACT specification is not lost and so
  that nobody reinvents it under pressure. Do not begin F3 by borrowing hours from an
  earlier day; section 1A.7 forbids adding scope.
- **Agent.** builder. **Runs on.** kaggle for training, bm-ptl for evaluation.
- **Done when.** A written decision in DECISIONS.md, either way.

---

## 6. Daily checkpoint ritual

End of each day, before stopping:
1. tester runs `scripts/evaluate.py` on seeds 0–9 with the current best executor and
   files a report. The demo must be recordable *every* evening **from Day 4 onward**
   (corrected Sept 12: M08 does not exist before Day 4, so "from Day 3" was unachievable).
   **Cost note (ADR-022 correction):** this is ~2 s on the scripted branch, which is the
   branch that ships. The ~76 min learned-branch figure (10 x 1000 x 456 ms) applies only
   under F3. ADR-018 fixes the seed set, so never shorten the run to fewer seeds.
2. docs-writer updates README status and the relevant `SUBMISSION.md` checkboxes.
3. Commit and push. A demo that exists only on one laptop is not a deliverable — and
   under ADR-020 the working tree also lives on an instance that expires Sept 17 00:15.
4. **Check the day's clock gate in section 1A.5.** If it was missed, fire the next rung of
   the cut ladder in **section 1A.6** — which supersedes the old one-line cut list here.
   The old list is preserved for traceability: *M15 (voice) → F3 → INT8 quantization in
   M13 → wrist cameras in M02 → the pour skill in M06*. The 1A.6 ladder replaces it
   because the ACT cut changed what "lowest value" means, and because "wrist cameras in
   M02" is already moot — M09a's camera set is `front` + `drawer_view`.

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
| RISK-12 | **The re-scoped plan needs ~9–10 builder-hours/day for five consecutive days against a Day-1-derived capacity of ~8, i.e. it carries roughly one day of negative float.** Raised Sept 12. | Section 1A.1 (26-hour Day 2 as previously written), section 1A.7 (45–47 builder-hours over 5 days). Day 1 shipped 8 budgeted builder-hours plus a correction pass and consumed the whole day. | Two things: (1) confirm or correct the 8 builder-hours/day planning capacity — if real throughput is higher, say so and I will re-plan upward rather than have modules overrun silently; (2) accept the section 1A.6 cut ladder as pre-authorised, so rungs fire on a missed gate without a conversation. |
| RISK-13 | **M09a's PoseNet frames come from `reset()` and perturbed arm poses, not from rollouts.** They therefore omit mid-manipulation occlusion by the arms — exactly the distribution M10's PoseNet faces in the demo loop. | Section 1A.3 / M09a. I have trained nothing and measured no accuracy. | None immediately. Mitigation is in M09a: if M06 has closed by Day 4 13:00, sample some frames from scripted rollouts inside the same 25-minute budget. If PoseNet's held-out error (M10 done-when 1) is poor, this is the first suspect, and the README records it as a known limitation rather than the demo quietly reverting to `--perception state`. |
| RISK-14 | **`handoff` is now single-point.** Brief p4 requires a hand-off or complementary dual-arm action; with `pour` on cut-ladder rung 2, `handoff` becomes the only bimanual moment in the demo. | Cut ladder 1A.6 rung 2; ADR-010; brief p4. | None immediately. GATE-1 input 3 checks `handoff` explicitly on Day 3 evening so this surfaces with three days left, not on Day 6. |

**RISK-03 and RISK-01 are closed** (`ARCHITECTURE.md` section 5, "Closed since first
issue"); they are retained above as the record of what was flagged, not as open items.
RISK-05 is **not** closed and its mitigation has drifted: it said "verify the Kaggle CLI
on Day 2, inside M08's day", and M08 is no longer on Day 2. Re-homed: the 20-minute Kaggle
trivial-GPU-kernel check runs inside **M09a's slot on Day 4**, before M10 depends on it on
Day 5. RISK-02 is closed by ADR-016.

---

## 8. What is explicitly not in this plan

Per `CONSTRAINTS.md:43-48`: no custom novel architecture, no VLA trained from scratch, no
physical hardware, no chasing the top prize. Also deliberately excluded: Intel Geti,
Intel Physical AI Studio, and Open Edge Platform (brief p3 lists them as optional
resources, and adopting a new toolchain inside six days is a schedule risk with no rubric
line of its own). If a spare half-day appears, a short README paragraph on how the
pipeline would map onto those tools is cheaper and safer than an integration.

**Added Sept 12.** Also now explicitly out: the ACT learned policy (M11) and its
demonstration dataset (M09b) — cut to F3, section 1A.3; the Speechmatics implementation
(M15) — unscheduled, cut-ladder rung 1; the `VlmGrounder` stretch of ADR-003; and the
optional render-resolution lever in M09b's budget block. **No spare half-day exists.**
Section 1A.7's rule stands: recovered hours go to the next day's module, never to a new
one.
