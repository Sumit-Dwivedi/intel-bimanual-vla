# Submission slide deck — canonical source

Seven slides. `docs/slides/submission.pptx` is **rendered from this file** by
`scripts/render_slides.py`; edit this markdown, then re-render. Do not hand-edit
the .pptx.

Every number below is traceable to a source document, and every one carries the
caveat that must travel with it (ADR-074's rules, plus the project's standing
disclosure obligations). A number without its caveat is a misquote.

---

## Slide 1 — Title

**Intel Bimanual VLA**

Language-grounded bimanual manipulation with OpenVINO on Intel Core Ultra

AI Infra Summit Hackathon 2026 · `github.com/Sumit-Dwivedi/intel-bimanual-vla`

*Background/visual:* `docs/images/submission-cover.png`

---

## Slide 2 — The challenge

- Two SO-101 arms in MuJoCo; language-conditioned table setting
- Inference on Intel Core Ultra Series 3 (Panther Lake) via OpenVINO 2026.3 —
  CPU, iGPU (Arc B390), NPU (NPU5010)
- Simulation-first: no physical robot anywhere in the pipeline
- The hard part is not the language. It is getting two 5-DoF arms to share one
  workspace without colliding — and *knowing* whether they did.

---

## Slide 3 — What we built

- **Four scripted skills, end to end:** `pick(A, fork)`, `place(A, fork, table)`,
  `pick(A, water_bottle)`, `handoff(A→B, fork)`
- **Bimanual handoff** with sequential choreography (ADR-037) — one arm moves,
  the other is genuinely frozen
- **Voice → plan → execution:** Speechmatics transcript at 0.983 confidence →
  rule-based grounder (43 tests) → executor (ADR-058)
- **Perception:** PoseNet trained on Arc B390, 2.6–3.2 mm MAE, exported to
  OpenVINO IR. Opt-in; the demo path uses oracle positions by default (ADR-046)
- **One entry point:** `run_demo.py` → 4/4 PASS on the Intel target

---

## Slide 4 — Working demo

*Image:* `docs/images/m06-handoff-complete.png`

- `handoff(A→B, fork)` completes: `weld.is_holding('B') == 'fork'`,
  `weld.is_holding('A') is None`, `from_arm_clear=True`
- Final lateral separation **0.1946 m**, `frames_used=6610`
- Full 40 s continuous take: `docs/videos/full-sequence-demo.mp4`
- Grasping is an explicit MuJoCo weld-constraint abstraction, **not friction
  contact** (ADR-029, disclosed per ADR-015)

---

## Slide 5 — OpenVINO on Core Ultra, and the evidence map

**PoseNet inference, batch 1 (ADR-045):**

| Device | Precision | Latency | Throughput |
|---|---|---|---|
| iGPU (Arc B390) | FP16 | 0.683 ms | **1465 Hz** |
| NPU (NPU5010) | FP16 | 1.099 ms | 910 Hz |
| CPU | FP16 | 6.492 ms | 154 Hz |

Batch 16: GPU **5800 Hz**, NPU 1478 Hz, CPU 231 Hz — throughput keeps climbing,
so scaling is sub-linear-to-linear on every device.

**Rubric evidence map:**

| Criterion | Status | Evidence |
|---|---|---|
| End-to-end + bimanual (30) | 4 skills; `handoff(A→B)` 4/4 PASS | `run_demo.py`, `m06-handoff-complete.png` |
| VLA / multi-modal (20) | Text + voice grounding; perception demoed | 43 grounder tests, ADR-058, ADR-055 |
| OpenVINO on Core Ultra (20) | 3 devices × 3 precisions + batch scaling | `m10-phase4-benchmark.md` |
| Robustness (15) | 20-seed eval, per-skill rates | `m08-extended-eval.md` |
| Reproducibility (10) | One entry point, pinned envs, 59 ADRs | `run_demo.py`, `requirements-*.txt` |
| Innovation (5) | Three measured negative results | ADR-050, ADR-057, ADR-074 |

---

## Slide 6 — Honest engineering

Three findings we measured, then acted against our own interest.

**1. INT8 quantization — declined (ADR-050).** 36–37 mm deviation against the
model's own 2.6–3.2 mm MAE. Faster, and not shippable. We kept FP16.

**2. Position-only geometry redesign — rejected (ADR-062).** A measure-first
rebuild confirmed two of three kinematic walls dissolve at 0.40 m base
separation, then failed on net evidence: 1 of 8 skills passing versus master's
4 of 9. The branch is preserved as the evidence.

**3. Motion-stack rewrite — the one that worked, partly (ADR-074).** Master's
skills were only ever scored on whether the *target* moved. Re-scored on five
scene-integrity criteria, **master passes 0 of 3**: `place` displaces the mug
63.2 mm; `handoff` spends 3179 of 6610 steps (48%) with the arms in contact, at
6.937 rad/s peak joint velocity. The rebuilt stack passes **2 of 3 individual
skills**, and a **six-stage bimanual relay passes all five scene-integrity
criteria** at every stage — worst peak joint velocity 2.362 rad/s against a 2.6
gate. `docs/videos/v2-relay-demo.mp4`.

Binding caveats on that result:

- The relay moves the fork **via the table**. v2 has **no working direct
  hand-to-hand handoff** — diagnosed, unsolved, named fix in ADR-073.
- `pick`'s pass required correcting the grasp geometry first: the ADR-025 pinch
  point is not this gripper's grasp centre in top-down poses, leaving a **4 mm**
  feasible clearance window. An earlier claim that `pick` passed was
  **retracted** when that proved wrong.
- v2 **cannot reach the water bottle at all** — a regression against master.
- Spline vs direct commands: **11.9× lower peak velocity, 21× lower peak
  acceleration**, at an identical 0.00014 rad final error.
- Idle-arm drift **0.000774 rad settled**, with a bounded transient at step 12
  disclosed.

**Two shortcuts we refused.** Widening the grasp gate 0.05 → 0.12 m, and the
weld gate to 0.115 m, each turned a failure into a passing number. Both were
measured and rejected — the second after a per-step contact check proved the
receiving gripper's pads never touch the fork.

---

## Slide 7 — Reproducibility

- **One command:** `./run_demo.sh` → `run_demo.py` → **4/4 PASS** on the Intel
  target, from a clean anonymous clone
- **Pinned environments:** `scripts/requirements-dev.txt` /
  `requirements-bmptl.txt`, checked by `scripts/verify_env.py`
- **59 ADR entries** in `ARCHITECTURE.md` (ADR-001 through ADR-058; 43 in
  `DECISIONS.md`), plus ADR-070–074 for the v2 rebuild on the `redesign-v2`
  branch — every decision, and every reversal, with the measurement behind it
- **Public repo:** `github.com/Sumit-Dwivedi/intel-bimanual-vla`
- Verified by cloning anonymously into a clean host and following the README
  with no prior knowledge — which is how we found the repo was still private,
  and three defects in our own environment check

**What does not work, stated plainly:** `pick(A, mug)`, `open_drawer`,
`place(A, water_bottle, table)`, `handoff(B→A, fork)`; `pour` is out of scope
(ADR-023), so the brief's literal example command does not run end to end.
`pytest tests/` is 66 passed / 4 failed.

---

## Number provenance — caveats that must travel

| number | caveat |
|---|---|
| `handoff` 20/20 | **degenerate; single-point envelope** — measures determinism, not robustness. Never quote bare. |
| `pick(A, water_bottle)` | **45%** (9/20, ADR-053's 20-seed extension) — not the superseded 60% |
| six-stage relay | always paired with **"all five scene-integrity criteria"** |
| v2 `pick` passes | always with the pad-geometry fix and the retraction |
| v2 handoff | **no working direct handoff**; relay via table only |
| spline vs direct | 11.9× / 21× — **never** the tracking-error ratio, which measures the command step |
| idle-arm drift | **settled** 0.000774 rad; transient at step 12 disclosed |
| `pytest` | 66 passed / **4 failed** — never "the suite passes" |
