"""M10 Phase 5 verification (ADR-046): all four working skills, BOTH ways.

Runs the exact four skill calls `scripts/verify_adr038_skills.py` established
as this project's baseline (`pick(A, fork)`, `place(A, fork, table)`,
`pick(A, 'bottle')`, `handoff(A->B, fork)`), each with a FRESH env (no
cross-contamination), twice:

  1. `--perception oracle` (default path, no CachedPropPositions/
     PoseNetInference ever constructed) -- must reproduce the ADR-038
     baseline numbers EXACTLY (fork pick final_z=0.3989, place final_z=0.3588,
     bottle pick final_z=0.6192, handoff lateral separation=0.1946 m).
  2. `--perception vision` (TableSettingEnv(cameras=['posenet_cam']),
     PoseNetInference(device=...)) -- numbers MAY differ by up to roughly a
     centimetre (ADR-044's held-out MAE is 2.6-3.2 mm x/y; a few mm of extra
     slop in a closed-loop grasp is expected, not a regression) -- and per-
     skill perception bookkeeping (refresh count, inference count, latency,
     oracle-vs-PoseNet deltas) is reported for transparency.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`. `--perception vision` additionally requires `artifacts/posenet_ir/`
(bm-ptl only, gitignored, ADR-045) and the `ov_env` venv (the one bm-ptl venv
with both `mujoco` and `openvino` installed together,
`scripts/requirements-bmptl.txt`).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np  # noqa: E402

from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402


_INFERENCE = None  # constructed once, lazily, and reused across every fresh()
                    # call in a "vision" run -- see this module's docstring
                    # note below and PoseNetInference's own docstring: compile
                    # once, infer many times. A previous version of this
                    # script constructed a fresh PoseNetInference per skill
                    # (4 GPU compiles instead of 1), which is NOT how the
                    # real ScriptedSkillExecutor uses it (one `self.inference`
                    # per executor instance, reused across an entire
                    # TaskPlan) and cost several extra minutes of pure GPU
                    # recompilation for no measurement benefit.


def _get_inference(device: str):
    global _INFERENCE
    if _INFERENCE is None:
        from bimanual.perception.inference import PoseNetInference

        print(f"Compiling PoseNetInference once (device={device})...", flush=True)
        _INFERENCE = PoseNetInference(device=device)
        print(f"  compiled. execution_devices={_INFERENCE.execution_devices}", flush=True)
    return _INFERENCE


def fresh(perception: str):
    """A new env + WeldGrasp, plus a position_provider (None in oracle mode).

    In "vision" mode, the ONE `PoseNetInference` for this whole run (see
    `_get_inference`) is reused; only `env`/`WeldGrasp`/`CachedPropPositions`
    are fresh per skill (a `CachedPropPositions` is cheap -- no compile, just
    a dict and a few counters -- so rebuilding it per skill costs nothing and
    keeps each skill's cache generation independent, matching
    `ScriptedSkillExecutor._ensure_position_provider`'s own per-env rebuild).
    """
    if perception == "vision":
        env = TableSettingEnv(cameras=["posenet_cam"], render_width=224, render_height=224)
    else:
        env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    weld = WeldGrasp(env)

    provider = None
    if perception == "vision":
        from bimanual.perception.cached_access import CachedPropPositions

        inference = _get_inference(_DEVICE)
        provider = CachedPropPositions(env, inference, weld=weld)

    return env, weld, provider


def obj_z(env, name: str) -> float:
    return float(env.data.xpos[sk._body_id(env.model, name)][2])


def pinch(env, arm: str) -> np.ndarray:
    fixed = env.data.xpos[sk._body_id(env.model, f"arm{arm}_gripper")]
    moving = env.data.xpos[sk._body_id(env.model, f"arm{arm}_moving_jaw_so101_v1")]
    return 0.5 * (np.asarray(fixed) + np.asarray(moving))


def _report_perception(label: str, provider, elapsed_s: float, result: dict) -> None:
    if provider is None:
        result["perception"] = None
        return
    entry = {
        "refresh_count": provider.refresh_count,
        "inference_count": provider.inference_count,
        "wall_clock_s": elapsed_s,
        "deltas": provider.deltas,
    }
    result["perception"] = entry
    print(f"    perception: refresh_count={provider.refresh_count} "
          f"inference_count={provider.inference_count} wall_clock={elapsed_s:.3f}s")
    for d in provider.deltas:
        print(f"      delta: prop={d['prop']} oracle={d['oracle_xyz']} "
              f"posenet={d['posenet_xyz']} delta_m={d['delta_m']:.4f}")


def run_one(perception: str) -> dict:
    print("=" * 70)
    print(f"M10 PHASE 5 VERIFICATION -- perception={perception}")
    print("=" * 70)
    results: dict = {}

    # --- 1. pick(A, fork) --------------------------------------------------
    env, weld, provider = fresh(perception)
    z0 = obj_z(env, "fork")
    t0 = time.perf_counter()
    r = sk.run_pick(env, "A", "fork", weld=weld, position_provider=provider)
    elapsed = time.perf_counter() - t0
    z1 = obj_z(env, "fork")
    print(f"\n[1] pick(A, fork): success={r.success} holding={weld.is_holding('A')!r} "
          f"z {z0:.4f} -> {z1:.4f} reason={r.reason}")
    results["pick_fork"] = {"success": r.success, "initial_z": z0, "final_z": z1, "reason": r.reason}
    _report_perception("pick_fork", provider, elapsed, results["pick_fork"])
    env.close()

    # --- 2. place(A, fork, table) -------------------------------------------
    env, weld, provider = fresh(perception)
    z0 = obj_z(env, "fork")
    t0 = time.perf_counter()
    r = sk.run_place(env, "A", "fork", "table", weld=weld, position_provider=provider)
    elapsed = time.perf_counter() - t0
    z1 = obj_z(env, "fork")
    print(f"\n[2] place(A, fork, table): success={r.success} holding={weld.is_holding('A')!r} "
          f"(expect None) z {z0:.4f} -> {z1:.4f} reason={r.reason}")
    results["place_fork"] = {"success": r.success, "initial_z": z0, "final_z": z1, "reason": r.reason}
    _report_perception("place_fork", provider, elapsed, results["place_fork"])
    env.close()

    # --- 3. pick(A, 'bottle') -----------------------------------------------
    env, weld, provider = fresh(perception)
    z0 = obj_z(env, "water_bottle")
    t0 = time.perf_counter()
    r = sk.run_pick(env, "A", "bottle", weld=weld, position_provider=provider)
    elapsed = time.perf_counter() - t0
    z1 = obj_z(env, "water_bottle")
    print(f"\n[3] pick(A, 'bottle'): success={r.success} holding={weld.is_holding('A')!r} "
          f"z {z0:.4f} -> {z1:.4f} reason={r.reason}")
    results["pick_bottle"] = {"success": r.success, "initial_z": z0, "final_z": z1, "reason": r.reason}
    _report_perception("pick_bottle", provider, elapsed, results["pick_bottle"])
    env.close()

    # --- 4. handoff(A -> B, fork) --------------------------------------------
    env, weld, provider = fresh(perception)
    z0 = obj_z(env, "fork")
    t0 = time.perf_counter()
    r = sk.run_handoff(env, "B", "A", "fork", weld=weld, position_provider=provider)
    elapsed = time.perf_counter() - t0
    z1 = obj_z(env, "fork")
    pa, pb = pinch(env, "A"), pinch(env, "B")
    lateral = abs(pa[1] - pb[1])
    vertical = abs(pa[2] - pb[2])
    print(f"\n[4] handoff(A -> B, fork): success={r.success} "
          f"holding A={weld.is_holding('A')!r} B={weld.is_holding('B')!r} "
          f"z {z0:.4f} -> {z1:.4f} lateral_sep={lateral:.4f} vertical_sep={vertical:.4f} "
          f"reason={r.reason}")
    results["handoff_fork"] = {
        "success": r.success, "initial_z": z0, "final_z": z1,
        "lateral_sep": lateral, "vertical_sep": vertical, "reason": r.reason,
    }
    _report_perception("handoff_fork", provider, elapsed, results["handoff_fork"])
    env.close()

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="GPU", choices=["CPU", "GPU", "NPU"],
                         help="OpenVINO device for --perception vision runs.")
    parser.add_argument("--out", type=Path, default=None,
                         help="Optional path to write both runs' results as JSON.")
    args = parser.parse_args()
    _DEVICE = args.device

    oracle_results = run_one("oracle")
    vision_results = run_one("vision")

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    for key in oracle_results:
        o, v = oracle_results[key], vision_results[key]
        print(f"{key}: oracle success={o['success']} final_z={o.get('final_z'):.4f} | "
              f"vision success={v['success']} final_z={v.get('final_z'):.4f}")

    if args.out is not None:
        args.out.write_text(
            json.dumps({"oracle": oracle_results, "vision": vision_results}, indent=2),
            encoding="utf-8",
        )
        print(f"\nWrote {args.out}")
