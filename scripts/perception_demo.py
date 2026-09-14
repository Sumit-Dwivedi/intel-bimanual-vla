"""Perception-in-loop demo: PoseNet drives `pick(A, fork)` (Fix F, ADR-055).

**Batch context.** Every prior demo/eval script in this repo
(`run_demo.py`, `scripts/verify_adr038_skills.py`,
`scripts/eval_m08.py`, `scripts/chained_demo.py`) runs the four scripted
skills in **oracle** mode -- `position_provider=None`, every prop position
read straight from `env.data.xpos`, no camera ever rendered, no OpenVINO
call ever made. M10 Phase 5 (ADR-046) wired PoseNet into the *targeting*
half of `pick`/`place`/`handoff` and verified it once
(`scripts/verify_m10_phase5.py`), but that script's whole point was the
oracle-vs-vision A/B comparison across all four skills together. This
script is narrower and single-purpose: run **one** skill, `pick(A, fork)`,
with perception actually driving the grasp-point target, and produce a
recorded demo of it -- the artifact `verify_m10_phase5.py` never itself
produced.

**What "PoseNet driving control" means here, precisely (ADR-046
Correction 1, unchanged by this script).** `position_provider` (a
`bimanual.perception.cached_access.CachedPropPositions` bound to a
`bimanual.perception.inference.PoseNetInference`) replaces exactly the ONE
targeting read `run_pick` uses to choose where to move
(`obj_pos0`/`grasp_point`, `src/bimanual/control/skills_scripted.py:1476`).
Every verification read this script makes to decide PASS/FAIL --
`env.data.xpos` for the fork's final z, `WeldGrasp.is_holding('A')` -- is
the SAME privileged oracle read every pre-Phase-5 script already used,
queried directly here, never through `position_provider`. A skill graded
by the same noisy estimate it acted on is unfalsifiable (ADR-046's own
words) -- this script does not repeat that mistake.

**Success criterion (this script's own, stated in the batch brief, not
invented here).** `env.data.xpos[fork][2] > 0.37` AND
`weld.is_holding('A') == 'fork'`, both read from oracle state after
`run_pick` returns. Note `0.37` is not an arbitrary number: it equals
`skills_scripted.TABLE_SURFACE_Z (0.35) + WELD_PICK_SUCCESS_MARGIN_M
(0.02)` -- the exact threshold `run_pick`'s own internal success check
already uses (`skills_scripted.py:1561`) -- so this script's external
verification and the skill's own internal one agree by construction, not
by coincidence.

**Randomization (batch correction 3).** `ENVELOPES = {}` at module scope
in `bimanual.sim.randomization` (ADR-048) and `TableSettingEnv.reset()`
defaults `randomizer=None`, so passing a bare `--seed` with no explicit
randomizer selects nothing -- seed 3 would be byte-identical to seed 0.
This script runs at `env.reset(seed=0)` with **no randomizer passed**
(fixed default layout), the same choice `scripts/verify_m10_phase5.py`
made for its own single-refresh oracle-vs-PoseNet delta table -- picked
deliberately so this run's numbers are directly comparable to that
module's own (ADR-046's 8.4 mm fork delta, GPU FP16 ~0.68 ms), not a
different scenario dressed up as a replication. `run_demo.py`'s own
alternative (`SKILL_ENVELOPES["pick_fork"]["fork"]`, a real per-seed
`ScenarioRandomizer`) was considered and rejected here specifically
because it would make this run's delta numbers a different measurement
than Phase 5's, defeating the comparison this script's own task brief asks
for.

**The ADR-046 per-step render-cost trap (batch correction 4).** Passing
`cameras=['posenet_cam']` as `TableSettingEnv`'s instance default makes
`env.step()` (no per-call override) render every physics step, not just
the one render this design intends per skill -- measured once at 12+
CPU-minutes for a single `pick` before being killed. The fix already lives
in `skills_scripted.py`'s two internal `env.step(ctrl, cameras=[])` call
sites (`_run_waypoint`/dwell helpers) -- unmodified by this script. This
script itself only ever calls `env.step` indirectly, through
`sk.run_pick`, so it inherits that fix automatically; it does not
duplicate the `cameras=[]` override anywhere. If this script's `run_pick`
call takes minutes instead of low single-digit seconds, that regression is
the diagnosis, not this script's own logic.

**Latency reporting.** The real, in-loop `pick(A, fork)` call makes
exactly ONE `PoseNetInference.predict()` call (`run_pick` reads its
targeting position once, before any waypoint runs -- see
`cached_access.py`'s own docstring: at most one render+infer per cache
generation, and nothing in `run_pick` calls `position_provider.get()` a
second time). That single call's wall-clock is measured directly (a
`_TimingInference` wrapper around the real `PoseNetInference`, entirely
local to this script -- `inference.py` itself is untouched). A mean/max
over a single sample is not a distribution, so this script ALSO runs the
already-compiled model's own `PoseNetInference.benchmark(n_runs=100)`
(same compiled model, same device, no second compile) immediately
afterward, to report a proper mean/max directly comparable to ADR-046's
GPU FP16 ~0.59-0.68 ms figure.

**IR / device fallback.** If `artifacts/posenet_ir/posenet_fp16.xml` is
missing, this script reconverts it via
`scripts/posenet_to_openvino.py --convert` rather than failing outright
(per this fix's own task brief). If compiling on the requested device
(default GPU) raises, this script falls back to CPU once and says so in
the printed output -- never a silent device substitution (ADR-007).

**Video.** `bm-ptl` has no imaging libraries (Pillow/imageio) in `ov_env`,
so this script writes PNG frames with the same stdlib-only `write_png`
`scripts/render_handoff_frames.py` already uses (duplicated here
verbatim, not imported, for the same reason that script does not import
from a third module: no shared PNG-writing module exists in this repo).
Frames are rendered post-hoc from full-physics-state snapshots captured
during the real run (`mujoco.mjtState.mjSTATE_FULLPHYSICS`, an OBSERVER
wrapper around `env.step` that never alters the real run's control,
timing, or step count -- identical technique to
`render_handoff_frames.py`), using the named `front` camera (a
presentation angle, deliberately NOT `posenet_cam`'s own top-down 224x224
training view, which is legible to the network but not to a human
viewer) at 640x480 -- independent of the 224x224 renderer
`CachedPropPositions` uses internally for actual perception input, since
this script builds its own separate `mujoco.Renderer` for the video
frames rather than reusing `env`'s fixed-size internal one. Encoding to
`docs/videos/perception-demo.mp4` happens on the LAPTOP (ffmpeg lives
there, not on bm-ptl) after these PNGs are `scp`'d over -- this script
only writes the PNGs.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
mujoco. Requires `ov_env` (`scripts/requirements-bmptl.txt`, the one bm-ptl
venv with both `mujoco` and `openvino`).

Usage (on bm-ptl, ov_env):
    python scripts/perception_demo.py
    python scripts/perception_demo.py --device CPU
    python scripts/perception_demo.py --no-render --out results.json
"""

from __future__ import annotations

import argparse
import json
import struct
import subprocess
import sys
import time
import zlib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402

SEED = 0
TARGET_ARM = "A"
TARGET_OBJECT = "fork"
TARGET_BODY = "fork"  # OBJECT_BODY_NAME["fork"] -- verified equal below at runtime

# The batch brief's own success threshold. Equals
# sk.TABLE_SURFACE_Z + sk.WELD_PICK_SUCCESS_MARGIN_M (0.35 + 0.02 = 0.37) --
# asserted below, not merely commented, so a future constant change in
# skills_scripted.py cannot silently desync this script's own criterion
# from the skill's internal one.
SUCCESS_Z_THRESHOLD = 0.37

IR_XML = REPO_ROOT / "artifacts" / "posenet_ir" / "posenet_fp16.xml"
CLIP_OUT_DIR = REPO_ROOT / "perception_demo_frames_tmp"
CLIP_N_FRAMES = 40
CLIP_W, CLIP_H = 640, 480


# ---------------------------------------------------------------------------
# stdlib-only PNG writer (verbatim copy of scripts/render_handoff_frames.py's
# own write_png / probe_render.py's original -- see this module's docstring
# for why it is duplicated rather than imported).
# ---------------------------------------------------------------------------
def write_png(path: Path, rgb: np.ndarray) -> None:
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def render_snapshot(model, data, snapshot: np.ndarray, camera: str, width: int, height: int) -> np.ndarray:
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    mujoco.mj_setState(model, data, snapshot, spec)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
    finally:
        renderer.close()
    return np.ascontiguousarray(pixels, dtype=np.uint8)


# ---------------------------------------------------------------------------
# Timing wrapper -- local to this script, does NOT modify inference.py.
# ---------------------------------------------------------------------------
class _TimingInference:
    """Wraps a real `PoseNetInference`, recording wall-clock per `predict()`
    call without touching `inference.py` (off-limits for this fix). Every
    other attribute (`device`, `execution_devices`, `benchmark`) is
    forwarded straight to the wrapped instance so `CachedPropPositions` and
    this script's own reporting code see the same object they would without
    this wrapper.
    """

    def __init__(self, inner) -> None:
        self._inner = inner
        self.latencies_ms: list[float] = []

    def predict(self, image: np.ndarray) -> dict[str, np.ndarray]:
        t0 = time.perf_counter()
        result = self._inner.predict(image)
        t1 = time.perf_counter()
        self.latencies_ms.append((t1 - t0) * 1000.0)
        return result

    def __getattr__(self, name):
        return getattr(self._inner, name)


def _ensure_ir() -> None:
    """Batch correction: reconvert rather than fail if the IR is missing."""
    if IR_XML.exists() and IR_XML.with_suffix(".bin").exists():
        print(f"  IR present: {IR_XML}")
        return
    print(f"  IR NOT FOUND at {IR_XML} -- attempting reconversion via "
          f"scripts/posenet_to_openvino.py --convert ...")
    convert_script = REPO_ROOT / "scripts" / "posenet_to_openvino.py"
    proc = subprocess.run([sys.executable, str(convert_script), "--convert"], cwd=str(REPO_ROOT))
    if proc.returncode != 0 or not IR_XML.exists():
        raise FileNotFoundError(
            f"Reconversion attempted (scripts/posenet_to_openvino.py --convert) but "
            f"{IR_XML} still does not exist (exit code {proc.returncode}). This "
            f"script cannot proceed with perception; see that script's own "
            f"requirements (a trained checkpoint under checkpoints/)."
        )
    print(f"  Reconversion succeeded: {IR_XML}")


def _compile_inference(device: str):
    """Construct PoseNetInference on `device`; fall back to CPU once on any
    compile failure, printing which happened (ADR-007: no SILENT fallback).
    Returns (wrapped_inference, device_actually_used, fell_back: bool).
    """
    from bimanual.perception.inference import PoseNetInference

    try:
        print(f"  Compiling PoseNetInference(device={device!r}) ...")
        real = PoseNetInference(device=device)
        print(f"  Compiled. execution_devices={real.execution_devices}")
        return _TimingInference(real), device, False
    except Exception as exc:  # noqa: BLE001 -- reported, not swallowed
        if device == "CPU":
            raise
        print(f"  GPU compile FAILED ({type(exc).__name__}: {exc}) -- falling back to CPU.")
        real = PoseNetInference(device="CPU")
        print(f"  Compiled on CPU fallback. execution_devices={real.execution_devices}")
        return _TimingInference(real), "CPU", True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="GPU", choices=["CPU", "GPU", "NPU"],
                         help="OpenVINO device to compile PoseNet on. Falls back to CPU on GPU compile failure.")
    parser.add_argument("--no-render", action="store_true",
                         help="Skip PNG frame rendering (faster re-runs while iterating on the numbers).")
    parser.add_argument("--out", type=Path, default=None, help="Optional JSON results path.")
    args = parser.parse_args()

    print("=" * 72)
    print("PERCEPTION-IN-LOOP DEMO: PoseNet drives pick(A, fork) (ADR-055)")
    print("=" * 72)

    # Sanity: this script's external success threshold must equal the
    # skill's own internal one (see module docstring) -- checked, not
    # merely asserted in a comment.
    internal_threshold = sk.TABLE_SURFACE_Z + sk.WELD_PICK_SUCCESS_MARGIN_M
    assert abs(SUCCESS_Z_THRESHOLD - internal_threshold) < 1e-9, (
        f"SUCCESS_Z_THRESHOLD={SUCCESS_Z_THRESHOLD} != "
        f"sk.TABLE_SURFACE_Z + sk.WELD_PICK_SUCCESS_MARGIN_M={internal_threshold}"
    )

    print("\n[1] IR check")
    _ensure_ir()

    print("\n[2] Compile PoseNetInference")
    inference, device_used, fell_back = _compile_inference(args.device)

    print("\n[3] Randomization")
    print(f"  env.reset(seed={SEED}), NO randomizer passed -> fixed default layout.")
    print(f"  (ENVELOPES={{}} at module scope, ADR-048 -- a bare seed selects nothing")
    print(f"   without an explicit randomizer; fixed default chosen so this run's")
    print(f"   numbers are directly comparable to ADR-046/M10 Phase 5's own single-")
    print(f"   refresh oracle-vs-PoseNet table, run under the identical scenario.)")

    print("\n[4] Build env (cameras=['posenet_cam'], 224x224 -- PoseNet's own input size)")
    env = TableSettingEnv(cameras=["posenet_cam"], render_width=224, render_height=224)
    env.reset(seed=SEED)
    weld = WeldGrasp(env)

    from bimanual.perception.cached_access import CachedPropPositions
    provider = CachedPropPositions(env, inference, weld=weld)

    body_id = sk._body_id(env.model, TARGET_BODY)

    def oracle_fork_z() -> float:
        return float(env.data.xpos[body_id][2])

    # --- Observer: capture full-physics state at every real step, for the
    # post-hoc demo video. Never alters control/timing/step count of the
    # real run -- identical technique to render_handoff_frames.py.
    capture = not args.no_render
    frame_states: dict[int, np.ndarray] = {}
    frame_counter = {"n": 0}
    original_step = env.step

    def observing_step(*call_args, **call_kwargs):
        result = original_step(*call_args, **call_kwargs)
        frame_counter["n"] += 1
        if capture:
            frame_states[frame_counter["n"]] = env.get_state().copy()
        return result

    env.step = observing_step

    z0 = oracle_fork_z()
    print(f"\n[5] Run pick(A, fork) with position_provider=PoseNet-backed CachedPropPositions")
    print(f"  fork oracle z before: {z0:.4f}")
    t_pick0 = time.perf_counter()
    result = sk.run_pick(env, TARGET_ARM, TARGET_OBJECT, weld=weld, position_provider=provider)
    t_pick1 = time.perf_counter()
    env.step = original_step  # restore, hygiene only

    z1 = oracle_fork_z()
    holding = weld.is_holding(TARGET_ARM)
    wall_clock_s = t_pick1 - t_pick0
    n_frames = frame_counter["n"]

    print(f"  fork oracle z after:  {z1:.4f}")
    print(f"  weld.is_holding('A'): {holding!r}")
    print(f"  run_pick.success:     {result.success}  reason={result.reason}")
    print(f"  frames_used:          {result.frames_used}  (real physics steps observed: {n_frames})")
    print(f"  wall clock:           {wall_clock_s:.3f} s")

    # --- The one verification the batch brief actually asks for: ORACLE
    # ground truth, not the perception estimate the skill acted on.
    oracle_success = (z1 > SUCCESS_Z_THRESHOLD) and (holding == TARGET_OBJECT)
    print(f"\n[6] ORACLE verification (independent of run_pick's own .success):")
    print(f"  fork z > {SUCCESS_Z_THRESHOLD} : {z1 > SUCCESS_Z_THRESHOLD} (z={z1:.4f})")
    print(f"  is_holding('A') == 'fork': {holding == TARGET_OBJECT}")
    print(f"  ORACLE_SUCCESS: {oracle_success}")

    # --- Perception bookkeeping ------------------------------------------
    print(f"\n[7] Perception bookkeeping")
    print(f"  cache refresh_count:   {provider.refresh_count}")
    print(f"  total inference_count: {provider.inference_count}")
    in_loop_latencies = inference.latencies_ms
    if in_loop_latencies:
        print(f"  in-loop predict() latency (this run's real inference call(s), n={len(in_loop_latencies)}):")
        print(f"    mean={sum(in_loop_latencies)/len(in_loop_latencies):.4f} ms  "
              f"max={max(in_loop_latencies):.4f} ms  values={['%.4f' % v for v in in_loop_latencies]}")
    else:
        print("  in-loop predict() latency: NO inference call was made "
              "(unexpected -- provider.get() should trigger exactly one refresh here).")

    print(f"\n  oracle-vs-PoseNet delta at each refresh (never used for control, ADR-046):")
    for d in provider.deltas:
        print(f"    refresh #{d['refresh_index']}: prop={d['prop']} "
              f"oracle={['%.4f' % v for v in d['oracle_xyz']]} "
              f"posenet={['%.4f' % v for v in d['posenet_xyz']]} "
              f"delta={d['delta_m']*1000:.2f} mm")

    fork_deltas = [d for d in provider.deltas if d["prop"] == "fork"]
    fork_delta_mm = fork_deltas[-1]["delta_m"] * 1000 if fork_deltas else None
    if fork_delta_mm is not None:
        print(f"\n  fork PERCEPTION-ESTIMATE delta at refresh time (PoseNet's raw xyz guess vs "
              f"where the fork actually was at that instant): {fork_delta_mm:.2f} mm "
              f"(M10 Phase 5 / ADR-046's own oracle-vs-PoseNet table reported 2.2 mm for "
              f"fork at this same seed/scenario -- this is the SAME quantity, so a close "
              f"match is expected, not merely hoped for)")
    else:
        print("\n  fork perception-estimate delta: N/A (no fork refresh recorded)")

    # Separate quantity: the OUTCOME delta -- how far this run's final oracle
    # z (achieved WITH perception driving the targeting) ends up from the
    # well-established oracle-only baseline final z (ADR-038/045/046/047/
    # 053/054 all independently reproduce 0.3989 m for pick(A, fork), no
    # randomizer, seed 0). This is the number ADR-046's "8.4 mm" headline
    # figure actually refers to -- NOT the refresh-time estimate error above,
    # which is a different quantity (perception accuracy vs. downstream
    # physical outcome). Conflating the two in one print statement was this
    # script's own first-draft mistake, caught and fixed before commit.
    ORACLE_ONLY_BASELINE_FINAL_Z = 0.3989
    outcome_delta_mm = abs(ORACLE_ONLY_BASELINE_FINAL_Z - z1) * 1000
    print(f"\n  fork OUTCOME delta (this run's final z under perception vs the documented "
          f"oracle-only baseline final z {ORACLE_ONLY_BASELINE_FINAL_Z}): "
          f"{outcome_delta_mm:.2f} mm "
          f"(M10 Phase 5 / ADR-046 measured 8.4 mm for this exact comparison -- "
          f"this run's own z1={z1:.4f} should reproduce that number closely)")

    # --- Supplementary benchmark: same compiled model, proper mean/max
    # over n_runs=100, directly comparable to ADR-046's GPU FP16 table.
    print(f"\n[8] Supplementary benchmark (same compiled model, n_runs=100 -- "
          f"NOT part of the live pick, reported for a proper mean/max distribution)")
    bench = inference.benchmark(n_runs=100)
    print(f"  device={bench['device']} execution_devices={bench['execution_devices']}")
    print(f"  mean={bench['mean_ms']:.4f} ms  median={bench['median_ms']:.4f} ms  "
          f"min={bench['min_ms']:.4f} ms  max={bench['max_ms']:.4f} ms  "
          f"throughput={bench['throughput_hz']:.2f} Hz")
    print(f"  (ADR-046/M10 Phase 5 measured GPU FP16 mean=0.6648 ms, max=7.2228 ms, "
          f"CPU mean=5.9488 ms -- compare against whichever device={device_used} used above)")

    if fell_back:
        print(f"\n  NOTE: requested device was {args.device!r}; GPU compile failed and this "
              f"run fell back to CPU (see [2] above). The benchmark numbers above are "
              f"CPU numbers, not GPU, and should be compared to ADR-046's CPU row, not "
              f"its GPU row.")

    # --- Regression gate reminder (not re-run here -- reported per the
    # batch brief's own instruction to report these, not re-derive them). --
    print(f"\n[9] Regression gates (unchanged by this fix -- reported, not re-run by this script)")
    print(f"  scripts/verify_adr038_skills.py must still report: 0.3989 / 0.3588 / 0.6192 / 0.1946, frames_used 6610")
    print(f"  pytest tests/test_skills.py must still report: 4 passed / 4 failed")
    print(f"  (this script touches no skill/grasp/ik/executor/env/randomization code -- see module docstring)")

    video_written = False
    if oracle_success and capture and n_frames > 0:
        print(f"\n[10] Rendering {CLIP_N_FRAMES} PNG frames to {CLIP_OUT_DIR} (camera='front', {CLIP_W}x{CLIP_H})")
        CLIP_OUT_DIR.mkdir(parents=True, exist_ok=True)
        for old in CLIP_OUT_DIR.glob("frame_*.png"):
            old.unlink()
        available = sorted(frame_states.keys())
        targets = np.linspace(1, n_frames, min(CLIP_N_FRAMES, n_frames))
        model, data = env.model, env.data
        chosen = []
        for i, target in enumerate(targets):
            nearest = min(available, key=lambda f: abs(f - target))
            chosen.append(nearest)
            img = render_snapshot(model, data, frame_states[nearest], "front", CLIP_W, CLIP_H)
            write_png(CLIP_OUT_DIR / f"frame_{i:03d}.png", img)
        print(f"  wrote {len(targets)} frames spanning physics-step indices {chosen[0]}..{chosen[-1]}")
        print(f"  NEXT STEPS (this script does not do these -- ffmpeg is laptop-only):")
        print(f"    1. scp -r {CLIP_OUT_DIR} <laptop>:<scratch>/perception_demo_frames_tmp")
        print(f"    2. On the laptop: ffmpeg -y -framerate 15 -i frame_%03d.png -pix_fmt yuv420p docs/videos/perception-demo.mp4")
        video_written = True
    elif not oracle_success:
        print(f"\n[10] ORACLE_SUCCESS is False -- refusing to render a demo video of a failed run "
              f"(same convention as scripts/render_handoff_frames.py).")
    elif args.no_render:
        print(f"\n[10] --no-render passed -- skipping PNG frame rendering.")

    env.close()

    report = {
        "adr": "ADR-055",
        "seed": SEED,
        "device_requested": args.device,
        "device_used": device_used,
        "fell_back_to_cpu": fell_back,
        "run_pick_success": bool(result.success),
        "run_pick_reason": result.reason,
        "frames_used": int(result.frames_used),
        "wall_clock_s": wall_clock_s,
        "fork_z_before": z0,
        "fork_z_after": z1,
        "holding_after": holding,
        "success_z_threshold": SUCCESS_Z_THRESHOLD,
        "oracle_success": bool(oracle_success),
        "cache_refresh_count": provider.refresh_count,
        "inference_count": provider.inference_count,
        "in_loop_latencies_ms": in_loop_latencies,
        "deltas": provider.deltas,
        "benchmark": bench,
        "video_frames_written": video_written,
    }

    if args.out is not None:
        args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nWrote {args.out}")

    print("\n" + "=" * 72)
    print(f"RESULT: run_pick.success={result.success}  ORACLE_SUCCESS={oracle_success}  "
          f"device={device_used}{' (fell back from ' + args.device + ')' if fell_back else ''}")
    print("=" * 72)

    return 0 if oracle_success else 1


if __name__ == "__main__":
    sys.exit(main())
