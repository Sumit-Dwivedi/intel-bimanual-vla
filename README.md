# Intel Physical AI Online Challenge — Bimanual VLA Manipulation

Simulation-first solution for the Intel Physical AI Online Challenge at the AI Infra
Summit Hackathon (Sept 10–16, 2026). Two SO-101 arms in MuJoCo perform a
language-conditioned table-setting task, with inference on Intel Core Ultra Series 3
(Panther Lake) via OpenVINO 2026.3 across CPU, iGPU (Arc B390), and NPU (NPU5010).

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

## Documentation

- Learning journey in `docs/learn/` (notes 01–08), written for a software engineer with
  no prior robotics background
- Measurement artifacts in `docs/hardware/` — reachability envelopes, render costs,
  grasp diagnostics, device benchmarks
- Decisions log in `DECISIONS.md`; architecture and rationale in `ARCHITECTURE.md`

## Environment

See `scripts/requirements-bmptl.txt` (Intel target) and `scripts/requirements-dev.txt`
(laptop) for pinned dependencies, and `scripts/verify_env.py` to check an install.
