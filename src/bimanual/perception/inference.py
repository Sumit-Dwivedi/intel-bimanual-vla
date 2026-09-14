"""PoseNetInference -- load a converted OpenVINO IR and serve object-position
predictions at runtime (M10 Phase 5, ADR-046).

This is the ONE piece of M10 Phase 5 that actually calls into OpenVINO. Phase
4 (ADR-045, `scripts/posenet_to_openvino.py`) already proved conversion,
compilation and correctness on CPU/GPU/NPU with a benchmark-only harness that
never touches the control loop. This module is the runtime-facing wrapper
that a skill can actually call during an episode: compile once, predict many
times, never re-read the IR file per call.

For a reader new to robotics/ML
--------------------------------
"Inference" just means "running a trained model forward to get a prediction"
-- as opposed to "training", which also computes gradients and updates
weights. OpenVINO's `Core.compile_model` does the one-time, possibly slow
work of turning a portable IR (Intermediate Representation) graph into
device-specific optimized code; the resulting `CompiledModel` can then be
asked for many fast individual `infer()` calls. Compiling once and inferring
many times (rather than compiling per-call) is why this class is built as an
object you construct once and reuse, not a bare function.

WHERE THE IR LIVES, AND WHY THIS CLASS FAILS LOUDLY WITHOUT IT
------------------------------------------------------------------
`artifacts/posenet_ir/*.xml` / `*.bin` are produced by
`scripts/posenet_to_openvino.py --convert` (ADR-045) and are gitignored
(`.gitignore`'s "Build artifacts (OpenVINO IR, reference tensors)" block) --
they exist ONLY on bm-ptl, never in this git history and never on the
laptop. A caller who imports this module on the laptop, or on bm-ptl before
Phase 4's conversion has run, gets a clear `FileNotFoundError` naming the
missing path and the command that produces it -- never a bare
`RuntimeError` from deep inside OpenVINO's own IR reader, and never a silent
fallback to some other behaviour.

DEFAULT DEVICE IS GPU, NOT CPU
-----------------------------------
ADR-045 measured GPU FP16 at 0.59-0.68 ms mean latency against CPU FP16's
6.4-6.5 ms (roughly 10x) on this exact IR pair, on bm-ptl's Arc B390 iGPU.
GPU is therefore the sane default for anything actually in a control loop;
CPU/NPU remain available by passing `device=`.

THE REAL ACCURACY NUMBER TO QUOTE, AND THE ONE NOT TO
-----------------------------------------------------------
ADR-045's ~1.4e-4 deviation figures are OpenVINO-vs-PyTorch CONVERSION
fidelity -- they say the exported graph computes (almost) the same function
the trained PyTorch model computes, nothing about whether that function is a
good pose estimator. The actual perception error entering the control loop
is ADR-044's held-out MAE against ground truth: **fork 3.2 mm, water_bottle
2.6 mm, mug 2.8 mm**, x/y only (ADR-044 caveat 1: every prop's z is a
per-prop constant by dataset construction, so z carries no real signal --
see this package's `posenet.py` module docstring and `cached_access.py`'s
own docstring for why a held/lifted object's z must never be read from this
class at all).
"""

from __future__ import annotations

import logging
import statistics
import time
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

# NOT imported from `bimanual.perception.posenet` -- that module does
# `import torch` at module scope (it defines the trainable nn.Module), and
# `torch` is NOT installed in `ov_env` (scripts/requirements-bmptl.txt),
# which is the ONE bm-ptl venv with both `mujoco` and `openvino` installed
# together and therefore the only venv where a skill can actually call this
# class at runtime (`scripts/requirements-train.txt`'s `train_env`, which
# does have torch, deliberately has neither mujoco nor openvino -- see that
# file's own module docstring). Importing `posenet.py` from here would drag
# an unused, unavailable `torch` import into every runtime caller. These
# three constants are duplicated from `posenet.py` (verified byte-identical
# there as of this module) rather than imported, specifically so this module
# has zero dependency on torch. If `posenet.py`'s `PROP_ORDER` /
# `COORDS_PER_PROP` / `INPUT_HW` ever change, this copy must be updated too
# -- there is no import to keep them in sync automatically.
PROP_ORDER = ["fork", "water_bottle", "mug"]
COORDS_PER_PROP = 3
INPUT_HW = 224

# Repo root, derived the same way src/bimanual/perception/dataset.py derives
# DEFAULT_ROOT: this file is src/bimanual/perception/inference.py, so
# parents[0]=perception, [1]=bimanual, [2]=src, [3]=repo root.
_REPO_ROOT = Path(__file__).resolve().parents[3]

#: Matches scripts/posenet_to_openvino.py's own DEFAULT_ARTIFACT_DIR / FP16_XML.
DEFAULT_IR_PATH = _REPO_ROOT / "artifacts" / "posenet_ir" / "posenet_fp16.xml"

#: Static batch-1 NCHW input, identical to ADR-045's conversion/benchmark shape.
_INPUT_SHAPE = (1, 3, INPUT_HW, INPUT_HW)

#: `__init__` warm-up count, per this module's task brief ("3 warm-up
#: inferences"). Separate from `benchmark()`'s own (larger) warm-up -- the
#: two serve different purposes: this one just primes OpenVINO's lazy
#: kernel-compilation/memory-allocation machinery so the FIRST real
#: `predict()` call a skill makes is not paying that one-time cost; the
#: benchmark's warm-up is about measuring a clean, amortized steady-state
#: latency.
_INIT_WARMUP_RUNS = 3


class PoseNetInference:
    """Compile a PoseNet OpenVINO IR once; serve `predict()` calls many times.

    Args:
        ir_path: Path to the `.xml` half of an IR pair (its `.bin` sibling
            must sit alongside it, exactly as `ov.save_model` writes them).
            Defaults to the FP16 IR (`DEFAULT_IR_PATH`) -- ADR-045's
            correctness check found FP16 within its 1e-3 threshold on both
            CPU and GPU, and FP16 is the faster of the two on every device
            measured, so it is the sane default for a runtime caller who has
            not been told otherwise. A relative path (like the class
            default, matching this module's task brief verbatim) is
            resolved against this repo's root, so it works regardless of the
            caller's current working directory.
        device: An OpenVINO device string ("CPU", "GPU", "NPU"). Default
            "GPU" -- see this module's docstring for the ADR-045 latency
            numbers this default rests on.

    Raises:
        FileNotFoundError: if the `.xml` (or its `.bin` sibling) does not
            exist. This is the expected, documented failure mode on any
            machine other than bm-ptl (the IR is gitignored and produced
            only there, ADR-045) -- callers that want the oracle-only
            behaviour that predates M10 Phase 5 should simply never
            construct this class (ADR-046's opt-in design: perception is
            never on by accident).
        ImportError: if the `openvino` package itself is not installed in
            the current interpreter, reported with a clear message naming
            `ov_env` (the one bm-ptl venv verified to have BOTH `mujoco` and
            `openvino` installed together, `scripts/requirements-bmptl.txt`)
            rather than a bare `ModuleNotFoundError` traceback.
    """

    def __init__(
        self,
        ir_path: str | Path = DEFAULT_IR_PATH,
        device: str = "GPU",
    ) -> None:
        xml_path = Path(ir_path)
        if not xml_path.is_absolute():
            xml_path = _REPO_ROOT / xml_path
        bin_path = xml_path.with_suffix(".bin")

        if not xml_path.exists() or not bin_path.exists():
            raise FileNotFoundError(
                f"OpenVINO IR not found: {xml_path} (+ {bin_path.name}). This "
                f"artifact pair is gitignored (.gitignore's 'Build artifacts "
                f"(OpenVINO IR, reference tensors)' block) and exists only on "
                f"bm-ptl, produced by "
                f"'scripts/posenet_to_openvino.py --convert' (ADR-045). If you "
                f"are on the laptop or a fresh bm-ptl checkout, either run that "
                f"conversion step first or do not construct PoseNetInference at "
                f"all -- every skill defaults to oracle (privileged-state) "
                f"positions when no PoseNetInference/CachedPropPositions is "
                f"supplied (ADR-046)."
            )

        try:
            import openvino as ov
        except ImportError as exc:  # pragma: no cover -- environment-dependent
            raise ImportError(
                "The 'openvino' package is not installed in this interpreter. "
                "On bm-ptl, use ov_env "
                "(C:\\Users\\devcloud\\project\\ov_env\\Scripts\\python.exe), "
                "which scripts/requirements-bmptl.txt already pins with both "
                "mujoco==3.2.7 and openvino==2026.3.1 installed together."
            ) from exc

        self.ir_path = xml_path
        self.device = device

        core = ov.Core()
        model = core.read_model(str(xml_path))
        self._compiled = core.compile_model(model, device)
        # ADR-007 (no silent device fallback): record the device OpenVINO
        # actually used, not merely the string this caller asked for.
        try:
            self.execution_devices = self._compiled.get_property("EXECUTION_DEVICES")
        except Exception:  # noqa: BLE001 -- not every device/version reports this
            self.execution_devices = "<not reported by this device/version>"

        self._infer_request = self._compiled.create_infer_request()
        self._output = self._compiled.output(0)

        # 3 warm-up inferences (this module's task brief), so the first real
        # predict() a caller makes is not paying OpenVINO's first-call
        # lazy-initialization cost.
        dummy = np.zeros(_INPUT_SHAPE, dtype=np.float32)
        for _ in range(_INIT_WARMUP_RUNS):
            self._infer_request.infer({0: dummy})

        logger.info(
            "PoseNetInference ready: ir_path=%s device=%s execution_devices=%s "
            "(%d warm-up inferences complete)",
            self.ir_path, self.device, self.execution_devices, _INIT_WARMUP_RUNS,
        )

    def predict(self, image: np.ndarray) -> dict[str, np.ndarray]:
        """One forward pass: an (INPUT_HW, INPUT_HW, 3) uint8 RGB frame ->
        `{prop_name: (x, y, z) float64 ndarray, world frame, metres}` for
        every name in `PROP_ORDER` (fork, water_bottle, mug -- ADR-041's
        3-prop scope; PoseNet was never trained to predict plate/spoon).

        Preprocessing matches `PoseNetDataset.__getitem__` EXACTLY
        (`src/bimanual/perception/dataset.py`): scale uint8 [0, 255] to
        float32 [0, 1], transpose (H, W, 3) -> (3, H, W), add a batch axis.
        No ImageNet mean/std normalization -- this model was trained from
        scratch on simulator renders, not fine-tuned from an ImageNet
        checkpoint (`posenet.py`'s own docstring).
        """
        if image.shape != (INPUT_HW, INPUT_HW, 3):
            raise ValueError(
                f"expected an ({INPUT_HW}, {INPUT_HW}, 3) uint8 RGB image, got "
                f"shape {image.shape} -- render the 'posenet_cam' camera at "
                f"render_width=render_height={INPUT_HW} (matches "
                f"scripts/generate_posenet_data.py's RENDER_W/RENDER_H)."
            )

        x = (image.astype(np.float32) / 255.0)
        x = np.transpose(x, (2, 0, 1))[None, ...]  # (1, 3, H, W)
        # `transpose` returns a VIEW (non-contiguous strides), not a copy.
        # Make it contiguous explicitly before handing it to OpenVINO's
        # infer() -- the dummy warm-up array in __init__ (`np.zeros`) is
        # already contiguous, so a non-contiguous real image is the first
        # tensor layout this compiled model would ever see, an untested
        # path relative to the warm-up.
        x = np.ascontiguousarray(x)

        result = self._infer_request.infer({0: x})
        out = np.asarray(result[self._output]).reshape(-1)  # (9,)

        predictions: dict[str, np.ndarray] = {}
        for i, prop in enumerate(PROP_ORDER):
            start = i * COORDS_PER_PROP
            predictions[prop] = out[start:start + COORDS_PER_PROP].astype(np.float64)
        return predictions

    def benchmark(self, n_runs: int = 100) -> dict:
        """Latency benchmark on this already-compiled model, same
        methodology as `scripts/posenet_to_openvino.py::do_bench_one`
        (ADR-045): a handful of warm-up calls discarded, then `n_runs`
        measured, wall-clock per `infer()` call, on a fixed random input (the
        exact pixel content does not matter for latency -- only the shape
        does).
        """
        rng = np.random.default_rng(0)
        x = rng.standard_normal(_INPUT_SHAPE).astype(np.float32)

        for _ in range(min(10, n_runs)):
            self._infer_request.infer({0: x})

        latencies_ms = []
        for _ in range(n_runs):
            t0 = time.perf_counter()
            self._infer_request.infer({0: x})
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000.0)

        return {
            "device": self.device,
            "execution_devices": self.execution_devices,
            "n_runs": n_runs,
            "min_ms": min(latencies_ms),
            "mean_ms": statistics.mean(latencies_ms),
            "median_ms": statistics.median(latencies_ms),
            "max_ms": max(latencies_ms),
            "std_ms": statistics.pstdev(latencies_ms) if n_runs > 1 else 0.0,
            "throughput_hz": 1000.0 / statistics.mean(latencies_ms),
        }


if __name__ == "__main__":
    # Smoke test: construct against the default FP16 IR + GPU, run one
    # predict() on a real rendered frame if a live env is importable, else on
    # a synthetic frame, then benchmark. Runs only where both mujoco and
    # openvino are importable in the SAME interpreter -- ov_env on bm-ptl
    # (scripts/requirements-bmptl.txt).
    print(f"Default IR path: {DEFAULT_IR_PATH}")
    inference = PoseNetInference()
    print(f"Compiled on device={inference.device} "
          f"execution_devices={inference.execution_devices}")

    rng = np.random.default_rng(0)
    synthetic = rng.integers(0, 256, size=(INPUT_HW, INPUT_HW, 3), dtype=np.uint8)
    preds = inference.predict(synthetic)
    for prop, xyz in preds.items():
        print(f"  {prop}: {xyz}")

    stats = inference.benchmark(n_runs=100)
    print(f"benchmark: mean={stats['mean_ms']:.4f} ms "
          f"throughput={stats['throughput_hz']:.2f} Hz "
          f"(device={stats['device']}, execution_devices={stats['execution_devices']})")
