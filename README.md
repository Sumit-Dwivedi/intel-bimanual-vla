# Intel Physical AI Online Challenge — Bimanual VLA Manipulation

Simulation-first solution for the Intel Physical AI Online Challenge at the AI Infra
Summit Hackathon (Sept 10–16, 2026). Two SO-101 arms in MuJoCo perform a
language-conditioned table-setting task, with inference on Intel Core Ultra Series 3
(Panther Lake) via OpenVINO 2026.3 across CPU, iGPU (Arc B390), and NPU (NPU5010).

## Status

**In active development through Sept 16, 2026.** The final README — with reproduction
instructions, benchmark tables, and the demo video — lands Sept 15. Manipulation skills
are still in development and are not yet demonstrated end to end; see `SUBMISSION.md`
for the current, unembellished status of every deliverable.

## Architecture

- Scripted closed-loop bimanual controller (see ADR-023 — no learned policy is trained)
- Rule-based language grounder (M05, 43 tests) — parses the challenge brief's own
  example command into an ordered plan with explicit per-arm assignment
- OpenVINO deployment path verified on all three devices (M03): PyTorch → IR → compile →
  infer on CPU, iGPU and NPU, with per-device numerical deviation recorded
- Full ADR log in `ARCHITECTURE.md` and `DECISIONS.md`

## Documentation

- Learning journey in `docs/learn/` (notes 01–08), written for a software engineer with
  no prior robotics background
- Measurement artifacts in `docs/hardware/` — reachability envelopes, render costs,
  grasp diagnostics, device benchmarks
- Decisions log in `DECISIONS.md`; architecture and rationale in `ARCHITECTURE.md`

## Environment

See `scripts/requirements-bmptl.txt` (Intel target) and `scripts/requirements-dev.txt`
(laptop) for pinned dependencies, and `scripts/verify_env.py` to check an install.
