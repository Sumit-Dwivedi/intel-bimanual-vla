# Rubric evidence map — slide version

One line per criterion. The full, caveated version with per-claim limits lives
in `SUBMISSION.md`; this is the compressed form for a slide. Every path below
was verified present before commit.

| Criterion | Status | Evidence |
|---|---|---|
| End-to-end + bimanual (30) | 4 skills; `handoff(A→B)` verified, 4/4 PASS; pick+handoff+place in one 40 s take | `docs/videos/full-sequence-demo.mp4`, `run_demo.py`, `docs/images/m06-handoff-complete.png`, plus a v2 six-stage relay passing all five scene-integrity criteria (`docs/videos/v2-relay-demo.mp4`, ADR-074) — via the table, not a direct handoff |
| VLA / multi-modal (20) | Text + voice grounding; perception demoed | 43 grounder tests, ADR-058 (voice), ADR-055 (perception) |
| OpenVINO on Core Ultra (20) | 3 devices × 3 precisions + batch scaling | `docs/hardware/m10-phase4-benchmark.md` |
| Robustness (15) | 20-seed eval, per-skill rates | `docs/hardware/m08-extended-eval.md` |
| Reproducibility (10) | One entry point, pinned envs, 59 ADRs | `run_demo.py`, `scripts/requirements-*.txt`, `ARCHITECTURE.md` |
| Innovation (5) | Four measured findings | ADR-050, ADR-057, `redesign` branch verdict |

## Notes for whoever presents this

**Numbers, so nothing is overstated on stage.**
- `run_demo.py` with no arguments: **4/4 PASS, exit 0**, ~25 s on bm-ptl.
- Voice: `--voice` → transcript at 0.983 confidence → grounded `handoff` → PASS.
- Perception: PoseNet drove `pick(A, fork)` at **0.590 ms** GPU FP16, 2.19 mm
  delta. It is **opt-in**; the demo path uses oracle positions by default
  (ADR-046).
- OpenVINO: GPU FP16 **1465 Hz** at batch 1, **5800 Hz** at batch 16.
- Robustness: `pick(A, fork)` and `place(A, fork, table)` **20/20**;
  `pick(A, water_bottle)` **9/20 (45%)**; `handoff` 20/20 but **degenerate** —
  its envelope is a single point, so that figure measures determinism, not
  robustness, and must never be quoted bare.
- Full-sequence video: pick + handoff + place in **one continuous 40.0 s take**, as **two** calls (`run_handoff`, whose Phase 1 *is* the pick, then `run_place`). The three-call form still fails at handoff Phase 3 — do not claim "any order composes". Two non-target props (mug, water bottle) are knocked over on camera; say so before a judge asks.
- v2 (`redesign-v2`) has **no working direct handoff** — say "relay via the
  table". Its relay is **6 of 6 stages on all five scene-integrity criteria**,
  never a bare "6/6". Do **not** quote any spline-vs-direct
  tracking-error ratio; that comparison measures the command step, not the
  controller — the defensible numbers are **11.9x
  lower peak velocity and 21x lower peak acceleration at an identical 0.00014
  rad final error**. Idle-arm drift is **0.000774 rad settled**, with a bounded
  transient at step 12 disclosed. v2 **cannot reach the water bottle at all**.
- `pytest tests/test_skills.py` is **4 passed / 4 failed** — never describe the
  suite as passing.

**The three negative results** are the Innovation row, and they are the
strongest part of the story because each one changed a decision:
1. **ADR-050** — INT8 quantization measured at 36-37 mm deviation against the
   model's own 2.6-3.2 mm MAE, and **declined** rather than shipped for its
   speedup.
2. **ADR-057** — multi-seed IK (32 restarts) proved four failing targets are
   genuine kinematic boundaries, not solver artifacts, which stopped a planned
   retry-wrapper series before it was built.
3. **`redesign` branch** — a measure-first geometry rebuild that partially
   confirmed its hypothesis (two of three kinematic walls dissolved at 0.40 m
   base separation) and was then **rejected on net evidence** (1/8 vs master's
   4/9), with a named reason it cannot be rescued at this scope.

4. **v2 motion-stack rewrite** (`redesign-v2`, ADR-074) — rebuilt the motion
   stack, then measured **both** stacks on one read-only instrument: the
   shipped stack passes **0 of 3** skills on scene-integrity criteria, v2
   **2 of 3** plus a six-stage relay passing all five at every stage. The only
   one of the four that produced a working alternative as well as a negative
   result — and two shortcuts that would have turned failures into passing
   numbers (grasp gate 0.05 → 0.12 m; weld gate → 0.115 m) were measured and
   **refused**.

**Path note.** The v2 verdict `docs/hardware/v2-verdict.md`, the relay video
and `scripts/v2_criteria_monitor.py` ARE on `master`; **ADR-070-074 and the v2
source are on the `redesign-v2` branch**. The redesign verdict is
`docs/hardware/redesign-verdict.md` on
the **`redesign` branch**, not on `master` — the branch is pushed to origin and
preserved deliberately as the evidence. Everything else cited here is on
`master`.
