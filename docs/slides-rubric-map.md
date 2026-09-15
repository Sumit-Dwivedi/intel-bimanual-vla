# Rubric evidence map — slide version

One line per criterion. The full, caveated version with per-claim limits lives
in `SUBMISSION.md`; this is the compressed form for a slide. Every path below
was verified present before commit.

| Criterion | Status | Evidence |
|---|---|---|
| End-to-end + bimanual (30) | 4 skills; `handoff(A→B)` verified, 4/4 PASS | `run_demo.py`, `docs/images/m06-handoff-complete.png` |
| VLA / multi-modal (20) | Text + voice grounding; perception demoed | 43 grounder tests, ADR-058 (voice), ADR-055 (perception) |
| OpenVINO on Core Ultra (20) | 3 devices × 3 precisions + batch scaling | `docs/hardware/m10-phase4-benchmark.md` |
| Robustness (15) | 20-seed eval, per-skill rates | `docs/hardware/m08-extended-eval.md` |
| Reproducibility (10) | One entry point, pinned envs, 59 ADRs | `run_demo.py`, `scripts/requirements-*.txt`, `ARCHITECTURE.md` |
| Innovation (5) | Three measured negative results | ADR-050, ADR-057, `redesign` branch verdict |

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

**Path note.** The redesign verdict is `docs/hardware/redesign-verdict.md` on
the **`redesign` branch**, not on `master` — the branch is pushed to origin and
preserved deliberately as the evidence. Everything else cited here is on
`master`.
