"""EvalHarness, EpisodeRecorder and metrics (M08).

Runs N seeds against a given executor/backend/device/precision and emits
results/<run-id>/summary.json, per_seed.csv, seed_<n>.mp4 and manifest.json.
Executor-agnostic by construction (ARCHITECTURE.md ADR-006): swapping
scripted for learned requires no change to harness code.
"""
