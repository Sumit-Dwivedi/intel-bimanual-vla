"""M10 Phase 4 extension -- INT8 post-training quantization of PoseNet via
NNCF, benchmarked CPU/iGPU/NPU (ADR-050, follows M10 Phase 4's ADR-045).

This is an EXTENSION of `scripts/posenet_to_openvino.py`, not a replacement.
It does NOT regenerate the FP32/FP16 IR that module already produced and
verified on bm-ptl (`artifacts/posenet_ir/posenet_fp32.{xml,bin}`,
45,221,444-byte `.bin`; `posenet_fp16.{xml,bin}`, 22,610,738-byte `.bin`).
It quantizes the EXISTING FP32 IR to INT8, benchmarks the result the same
way Phase 4 benchmarked FP32/FP16, and reuses Phase 4's own
`val_images.npy` / `val_predictions_xpu.npy` (5 validation samples, the
same PyTorch-XPU reference) for the correctness check, so INT8 is compared
on identical ground to FP32/FP16.

WHERE THIS RUNS AND WHAT IT NEEDS
------------------------------------
Entirely inside `train_env` on bm-ptl (the same venv Phase 2-4 used, per
ADR-042/ADR-043). This module adds exactly one new dependency family:
NNCF and the handful of pure-Python packages it imports at module load time.
`ov_env` (`scripts/requirements-bmptl.txt`) is never touched -- NNCF is not
installed there, matching the pattern ADR-043 already established for
openvino itself (a new capability goes into `train_env`, never into the
regression-gated `ov_env`).

INSTALL STRATEGY: --no-deps FIRST, THEN ONLY WHAT IMPORT ACTUALLY NEEDED
----------------------------------------------------------------------------
`pip install --no-deps nncf` was tried first (this module's task brief,
correction 3) and succeeded at the download/install step, but `import nncf`
then failed on a missing transitive import. Rather than re-resolving nncf's
full dependency tree (which pulls scipy, scikit-learn, pydot, ninja, rich,
etc. -- some of which could plausibly want to bump numpy or another pinned
package), each missing import was added ONE AT A TIME, each with its own
`--no-deps`, stopping the moment `import nncf` succeeded:

    pip install --no-deps nncf         # installs nncf 3.3.0 itself
    pip install --no-deps packaging    # nncf's __init__ needed it directly
    pip install --no-deps rich         # nncf's __init__ needed it directly
    pip install --no-deps tabulate     # nncf's __init__ needed it directly
    pip install --no-deps psutil       # nncf's __init__ needed it directly
    pip install --no-deps safetensors  # nncf's __init__ needed it directly
    pip install --no-deps scipy        # nncf's quantization algorithms needed it

`import nncf` succeeded after `scipy`. `pip show nncf` also lists `ninja`,
`pydot` and `scikit-learn` as declared requirements, and `pip check`
correctly flags all three as missing (plus `rich`'s own declared-but-unused-
here deps `markdown-it-py`/`pygments`) -- but neither `import nncf` nor a
live `nncf.quantize(...)` smoke test against a trivial OpenVINO model
(MinMax statistics collection + Fast Bias Correction, the same algorithm
path this script's real quantization uses) ever needed them. They are
NOT installed. `pip check`'s residual complaints are recorded here rather
than silently resolved, per this module's task brief.

Before and after EVERY install step above, `torch.__version__`
(`2.14.0+xpu`), `torch.xpu.is_available()` (`True`) and `numpy.__version__`
(`2.4.6`) were verified unchanged -- see `docs/hardware/m10-phase4-int8.md`
for the full before/after transcript. None of the 7 packages installed
touched torch or numpy; had any of them tried to, this module would have
stopped there and reported it rather than proceeding (this module's task
brief, correction 3).

WHY --bench-one RUNS EACH DEVICE IN ITS OWN SUBPROCESS
----------------------------------------------------------
Identical reasoning to `scripts/posenet_to_openvino.py`'s docstring: M03
found the NPU plugin can terminate the whole process
(`STATUS_ACCESS_VIOLATION`) rather than raising a catchable exception on an
unsupported graph. Every (device, INT8) result is appended to
`artifacts/posenet_ir/results.jsonl` -- the SAME file Phase 4 wrote to --
the moment it is known, and each device compiles+benchmarks in its own
`subprocess.run(...)`, in the fixed order CPU -> GPU -> NPU.

USAGE
-------
    train_env\\Scripts\\python.exe scripts\\quantize_posenet.py --all

    # Individual stages:
    ...  --quantize   # samples 300 calibration images, runs nncf.quantize(),
                       # writes posenet_int8.xml/.bin + int8_calibration_info.json
    ...  --bench       # benchmarks CPU/GPU/NPU INT8, each in its own subprocess
    ...  --bench-one --device CPU   # internal, invoked as a subprocess
    ...  --report      # appends INT8 rows + a new section to
                        # docs/hardware/m10-phase4-benchmark.md
"""

import argparse
import json
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = REPO_ROOT / "src"
ARTIFACT_DIR = REPO_ROOT / "artifacts" / "posenet_ir"
CALIB_IMAGES_DIR = REPO_ROOT / "data" / "posenet" / "images"
REPORT_PATH = REPO_ROOT / "docs" / "hardware" / "m10-phase4-benchmark.md"

FP32_XML = "posenet_fp32.xml"
INT8_XML = "posenet_int8.xml"

# Static batch-1 NCHW input -- identical to Phase 4 (ADR-013's NPU
# static-shape requirement), and to the FP32 IR being quantized here.
INPUT_SHAPE = (1, 3, 224, 224)

N_WARMUP = 10
N_MEASURED = 100

# 300 calibration images, sampled without replacement from the 5,000
# available under data/posenet/images/ (gitignored, bm-ptl only). Seed is
# fixed and recorded (alongside the exact sample_index list actually drawn)
# in int8_calibration_info.json so this draw is reproducible.
N_CALIB = 300
CALIB_SEED = 42

DEVICES = ["CPU", "GPU", "NPU"]

# Marker line prefix for handing a --bench-one child's result back to the
# parent via stdout -- same pattern as posenet_to_openvino.py's
# RESULT_MARKER, deliberately a different literal string so a stray stdout
# line from one script is never mistaken for the other's.
RESULT_MARKER = "POSENET_INT8_RESULT_JSON: "


# ---------------------------------------------------------------------------
# Stage 1: calibration sampling + NNCF quantization of the EXISTING FP32 IR
# ---------------------------------------------------------------------------

def _sample_calibration_indices(n_calib: int, seed: int) -> list:
    """Sample n_calib unique sample_index values from the images actually on
    disk under data/posenet/images/, using a seeded RNG. Returns a sorted
    list of ints. Raises loudly if fewer images exist than requested --
    never silently samples fewer than asked.
    """
    all_paths = sorted(CALIB_IMAGES_DIR.glob("sample_*.png"))
    if not all_paths:
        raise FileNotFoundError(
            f"no sample_*.png files found under {CALIB_IMAGES_DIR} -- this "
            f"directory is gitignored (.gitignore's M10 Phase 1 block) and "
            f"lives only on bm-ptl (scripts/generate_posenet_data.py's output)."
        )
    if len(all_paths) < n_calib:
        raise RuntimeError(
            f"only {len(all_paths)} calibration images available under "
            f"{CALIB_IMAGES_DIR}, need {n_calib} -- refusing to silently "
            f"calibrate on fewer than requested."
        )

    # sample_NNNNN.png sorts lexicographically == numerically (zero-padded
    # 5 digits), so the i-th entry of the sorted glob has sample_index == i.
    rng = np.random.default_rng(seed)
    chosen = rng.choice(len(all_paths), size=n_calib, replace=False)
    indices = sorted(int(i) for i in chosen)

    # Cross-check the filename's implied index against a couple of sampled
    # entries so a mismatch (renamed/reordered files) fails loudly rather
    # than silently mis-labelling the calibration set.
    for idx in (indices[0], indices[len(indices) // 2], indices[-1]):
        stem = all_paths[idx].stem  # "sample_00013"
        parsed = int(stem[len("sample_"):])
        if parsed != idx:
            raise ValueError(
                f"{all_paths[idx]}: filename implies sample_index={parsed}, "
                f"expected {idx} from the sorted glob position -- refusing "
                f"to trust the index mapping."
            )
    return indices, all_paths


def _load_calibration_images(indices: list, all_paths: list) -> list:
    """Load and preprocess the sampled calibration images IDENTICALLY to
    training (`src/bimanual/perception/dataset.py::PoseNetDataset.__getitem__`):
    224x224 RGB PNG -> float32 in [0, 1] -> CHW -> add a batch dim of 1.
    Returns a plain Python list of (1, 3, 224, 224) float32 ndarrays, the
    form `nncf.Dataset` expects for a single-input model (proven against a
    trivial model in this module's pre-flight smoke test).
    """
    from PIL import Image

    images = []
    for idx in indices:
        path = all_paths[idx]
        img = Image.open(path).convert("RGB")
        if img.size != (224, 224):
            raise ValueError(f"{path}: expected 224x224, got {img.size}")
        arr = np.asarray(img, dtype=np.float32) / 255.0  # (H, W, 3) in [0, 1]
        chw = np.transpose(arr, (2, 0, 1))                # (3, H, W)
        images.append(chw[np.newaxis, ...].astype(np.float32))  # (1, 3, 224, 224)
    return images


def do_quantize(artifact_dir: Path, n_calib: int = N_CALIB, seed: int = CALIB_SEED) -> dict:
    """Quantize the EXISTING FP32 IR (never regenerated here) to INT8 via
    NNCF post-training quantization, calibrated on n_calib images sampled
    from data/posenet/images/. Saves posenet_int8.xml/.bin and
    int8_calibration_info.json (seed + exact sample_index list, for
    reproducibility). Requires openvino + nncf + PIL + numpy -- deliberately
    NOT torch, since quantization operates on the already-exported IR, not
    the PyTorch module.
    """
    import openvino as ov
    import nncf

    fp32_xml = artifact_dir / FP32_XML
    if not fp32_xml.exists():
        raise FileNotFoundError(
            f"{fp32_xml} not found. This module quantizes the EXISTING FP32 "
            f"IR from M10 Phase 4 (scripts/posenet_to_openvino.py --convert) "
            f"-- it does not regenerate it. Run that script first."
        )
    fp32_bin_bytes = fp32_xml.with_suffix(".bin").stat().st_size
    print(f"Loading existing FP32 IR: {fp32_xml} "
          f"({fp32_bin_bytes / (1024 * 1024):.2f} MiB .bin)")

    indices, all_paths = _sample_calibration_indices(n_calib, seed)
    print(f"Sampled {len(indices)} calibration images (seed={seed}) from "
          f"{len(all_paths)} available under {CALIB_IMAGES_DIR}.")
    calib_images = _load_calibration_images(indices, all_paths)

    calibration_info = {
        "n_calib": n_calib,
        "seed": seed,
        "sample_indices": indices,
        "images_dir": str(CALIB_IMAGES_DIR),
        "preprocessing": "224x224 RGB PNG -> float32 [0,1] -> CHW (identical "
                          "to PoseNetDataset.__getitem__, no mean/std normalization)",
    }
    (artifact_dir / "int8_calibration_info.json").write_text(
        json.dumps(calibration_info, indent=2))

    core = ov.Core()
    ov_model = core.read_model(str(fp32_xml))

    calib_dataset = nncf.Dataset(calib_images)

    # target_device=NPU per ADR-013's decision ("INT8 via NNCF post-training
    # quantization ... targets the NPU5010"). This selects NNCF's
    # NPU-appropriate quantization scheme; the resulting IR is still a
    # generic OpenVINO IR and is benchmarked on CPU/GPU/NPU below exactly
    # like Phase 4's FP32/FP16 IRs were.
    print(f"Running nncf.quantize() with {len(calib_images)} calibration "
          f"samples, target_device=NPU ...")
    q_model = nncf.quantize(
        ov_model, calib_dataset,
        subset_size=n_calib,
        target_device=nncf.TargetDevice.NPU,
    )

    int8_xml = artifact_dir / INT8_XML
    ov.save_model(q_model, str(int8_xml))
    int8_bin = int8_xml.with_suffix(".bin")
    int8_bin_bytes = int8_bin.stat().st_size
    print(f"Wrote {int8_xml.name} / {int8_bin.name} "
          f"({int8_bin_bytes / (1024 * 1024):.2f} MiB .bin)")

    fp16_bin_path = artifact_dir / "posenet_fp16.bin"
    fp16_bin_bytes = fp16_bin_path.stat().st_size if fp16_bin_path.exists() else None

    quant_info = {
        "int8_bin_bytes": int8_bin_bytes,
        "fp32_bin_bytes": fp32_bin_bytes,
        "fp16_bin_bytes": fp16_bin_bytes,
        "int8_over_fp32_ratio": int8_bin_bytes / fp32_bin_bytes if fp32_bin_bytes else None,
        "int8_over_fp16_ratio": (int8_bin_bytes / fp16_bin_bytes) if fp16_bin_bytes else None,
        "target_device": "NPU",
        "n_calib": n_calib,
        "calib_seed": seed,
    }
    (artifact_dir / "int8_convert_info.json").write_text(json.dumps(quant_info, indent=2))
    print(f"INT8 .bin is {quant_info['int8_over_fp32_ratio']:.3f}x the FP32 .bin "
          f"size, {quant_info['int8_over_fp16_ratio']:.3f}x the FP16 .bin size "
          f"(expect roughly 0.25x FP32 given INT8 is 1/4 the bit-width of FP32 "
          f"weights, though NNCF may leave some layers unquantized).")
    return quant_info


# ---------------------------------------------------------------------------
# Stage 2: per-device INT8 benchmark, run as a subprocess (mirrors
# posenet_to_openvino.py's do_bench_one, precision fixed to INT8)
# ---------------------------------------------------------------------------

def _latency_stats(latencies_ms: list) -> dict:
    return {
        "min_ms": min(latencies_ms),
        "mean_ms": statistics.mean(latencies_ms),
        "median_ms": statistics.median(latencies_ms),
        "p95_ms": statistics.quantiles(latencies_ms, n=100)[94],
        "std_ms": statistics.pstdev(latencies_ms),
        "throughput_hz": 1000.0 / statistics.mean(latencies_ms),
    }


def _append_result(artifact_dir: Path, result: dict) -> None:
    """Append one JSON line to the SAME results.jsonl Phase 4 wrote to --
    immediately, never buffered. Keeping one file means a future
    `posenet_to_openvino.py --report` naturally picks up these INT8 rows
    too, since the entry schema matches Phase 4's exactly (combo/device/
    framework/precision/status/latency_ms/correctness).
    """
    results_path = artifact_dir / "results.jsonl"
    with open(results_path, "a", encoding="utf-8") as f:
        f.write(json.dumps(result) + "\n")


def do_bench_one(artifact_dir: Path, device: str) -> None:
    """Compile + benchmark + correctness-check INT8 on exactly ONE device,
    print one RESULT_MARKER JSON line, then exit. Invoked as a subprocess
    (see run_all_int8_benchmarks) so an NPU driver crash costs only this one
    combo. Requires openvino + numpy ONLY -- no torch, no nncf (this process
    only loads the already-quantized .xml/.bin).
    """
    import openvino as ov

    int8_xml = artifact_dir / INT8_XML
    entry = {
        "combo": f"{device}_INT8", "device": device, "framework": "openvino",
        "precision": "INT8", "host": platform.node(),
    }

    if not int8_xml.exists():
        entry["status"] = "SKIPPED"
        entry["error"] = f"{int8_xml} does not exist -- run --quantize first."
        print(RESULT_MARKER + json.dumps(entry), flush=True)
        return

    core = ov.Core()
    try:
        compiled = core.compile_model(str(int8_xml), device)
    except Exception as e:  # noqa: BLE001 -- a device rejecting INT8 is a real
        # finding (device capability limit), not a bug to hide.
        entry["status"] = "COMPILE_FAILED"
        entry["error_type"] = type(e).__name__
        entry["error"] = str(e)
        print(RESULT_MARKER + json.dumps(entry), flush=True)
        return

    # ADR-007: no silent device fallback -- read back the device OpenVINO
    # actually used from the compiled model, not the string we asked for.
    try:
        entry["execution_devices"] = compiled.get_property("EXECUTION_DEVICES")
    except Exception:
        entry["execution_devices"] = "<not reported by this device/version>"

    entry["status"] = "COMPILE_OK"
    infer_request = compiled.create_infer_request()

    # --- Latency benchmark: 10 warm-up (discarded), 100 measured. Identical
    # methodology to Phase 4's FP32/FP16 rows. ---------------------------
    rng = np.random.default_rng(0)
    x = rng.standard_normal(INPUT_SHAPE).astype(np.float32)

    try:
        for _ in range(N_WARMUP):
            infer_request.infer({0: x})

        latencies_ms = []
        for _ in range(N_MEASURED):
            t0 = time.perf_counter()
            infer_request.infer({0: x})
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000.0)

        entry["latency_ms"] = _latency_stats(latencies_ms)
        entry["n_warmup"] = N_WARMUP
        entry["n_measured"] = N_MEASURED
    except Exception as e:  # noqa: BLE001
        entry["infer_status"] = "INFER_FAILED"
        entry["error_type"] = type(e).__name__
        entry["error"] = str(e)
        print(RESULT_MARKER + json.dumps(entry), flush=True)
        return

    entry["infer_status"] = "OK"

    # --- Correctness: SAME 5 validation samples + SAME PyTorch-XPU
    # reference Phase 4 used (val_images.npy / val_predictions_xpu.npy),
    # reused here rather than recomputed, per this module's task brief. ---
    val_images_path = artifact_dir / "val_images.npy"
    val_preds_path = artifact_dir / "val_predictions_xpu.npy"
    if val_images_path.exists() and val_preds_path.exists():
        val_images = np.load(val_images_path)          # (5, 3, 224, 224)
        ref_preds = np.load(val_preds_path)             # (5, 9), metres
        int8_preds = np.zeros_like(ref_preds)
        for i in range(val_images.shape[0]):
            single = val_images[i:i + 1].astype(np.float32)
            result = infer_request.infer({0: single})
            int8_preds[i] = result[compiled.output(0)][0]

        max_abs_dev_m = float(np.max(np.abs(int8_preds - ref_preds)))
        max_abs_dev_mm = max_abs_dev_m * 1000.0
        # PoseNet's own ground-truth MAE (checkpoints/posenet_best.pth,
        # ADR-044): fork 3.211 mm, bottle 2.625 mm, mug 2.768 mm. Judge the
        # INT8 deviation against the SMALLEST of these (the model's best
        # per-prop accuracy), the more conservative comparison -- if INT8
        # deviation is small even against the tightest per-prop MAE, it is
        # small against all of them.
        min_mae_mm = 2.625
        entry["correctness"] = {
            "max_abs_deviation_vs_xpu": max_abs_dev_m,
            "max_abs_deviation_vs_xpu_mm": max_abs_dev_mm,
            "posenet_min_mae_mm": min_mae_mm,
            "deviation_over_min_mae_ratio": max_abs_dev_mm / min_mae_mm,
            "note": (
                f"INT8 max abs deviation vs the PyTorch-XPU reference is "
                f"{max_abs_dev_mm:.3f} mm, vs PoseNet's own smallest "
                f"per-prop ground-truth MAE of {min_mae_mm:.3f} mm "
                f"({'well under' if max_abs_dev_mm < min_mae_mm else 'at or above'} "
                f"the model's own error scale -- "
                f"{'immaterial to control' if max_abs_dev_mm < min_mae_mm else 'material to control'})."
            ),
        }
    else:
        entry["correctness"] = {"note": "val_images.npy / val_predictions_xpu.npy "
                                         "not found (Phase 4 output) -- cannot check correctness."}

    print(RESULT_MARKER + json.dumps(entry), flush=True)


def run_all_int8_benchmarks(artifact_dir: Path) -> list:
    """Benchmark INT8 on every device, EACH IN ITS OWN SUBPROCESS, fixed
    order CPU -> GPU -> NPU, appending every result to results.jsonl the
    moment it is obtained.
    """
    results = []
    for device in DEVICES:
        print(f"\n--- Benchmarking {device} / INT8 (isolated subprocess) ---")
        proc = subprocess.run(
            [sys.executable, "-u", str(Path(__file__).resolve()),
             "--bench-one", "--device", device,
             "--artifact-dir", str(artifact_dir)],
            capture_output=True, text=True,
        )
        entry = None
        for line in proc.stdout.splitlines():
            if line.startswith(RESULT_MARKER):
                entry = json.loads(line[len(RESULT_MARKER):])
                break

        if entry is None:
            entry = {
                "combo": f"{device}_INT8", "device": device,
                "framework": "openvino", "precision": "INT8",
                "status": "PROCESS_CRASHED", "exit_code": proc.returncode,
                "stdout": proc.stdout.strip(), "stderr": proc.stderr.strip(),
                "host": platform.node(),
            }
            print(f"[{device} | INT8] PROCESS CRASHED (exit code "
                  f"{proc.returncode}). Verbatim stderr:\n{proc.stderr.strip()}")
        else:
            _print_bench_summary(entry)

        results.append(entry)
        _append_result(artifact_dir, entry)

    print(f"\nAppended {len(results)} INT8 benchmark rows to "
          f"{artifact_dir / 'results.jsonl'}")
    return results


def _print_bench_summary(entry: dict) -> None:
    tag = f"[{entry['device']} | INT8]"
    if entry["status"] in ("SKIPPED", "COMPILE_FAILED"):
        print(f"{tag} {entry['status']}: {entry.get('error')}")
        return
    if entry.get("infer_status") == "INFER_FAILED":
        print(f"{tag} compile OK, INFER FAILED: {entry.get('error')}")
        return
    lat = entry.get("latency_ms", {})
    corr = entry.get("correctness", {})
    print(f"{tag} compile OK (EXECUTION_DEVICES={entry.get('execution_devices')}), "
          f"mean={lat.get('mean_ms', float('nan')):.4f} ms "
          f"throughput={lat.get('throughput_hz', float('nan')):.2f} Hz, "
          f"max_abs_dev_vs_xpu={corr.get('max_abs_deviation_vs_xpu_mm', 'n/a')} mm")


# ---------------------------------------------------------------------------
# Stage 3: append INT8 rows + a new section to the EXISTING Phase 4 report
# ---------------------------------------------------------------------------

def _fmt(v):
    return f"{v:.4f}" if isinstance(v, (int, float)) else "n/a"


def _load_int8_rows(artifact_dir: Path) -> list:
    """Read results.jsonl, return only the INT8 rows, in device order
    CPU -> GPU -> NPU (the same fixed order they were benchmarked in), one
    row per device (the most recent one, in case this was re-run)."""
    results_path = artifact_dir / "results.jsonl"
    if not results_path.exists():
        raise FileNotFoundError(f"{results_path} not found -- run --bench first.")

    by_device = {}
    with open(results_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if row.get("precision") == "INT8" and row.get("framework") == "openvino":
                by_device[row["device"]] = row  # last one wins if re-run

    return [by_device[d] for d in DEVICES if d in by_device]


def _build_table_row(r: dict) -> str:
    device = r.get("device", "?")
    precision = r.get("precision", "?")
    status = r.get("status", "?")
    lat = r.get("latency_ms", {})
    corr = r.get("correctness", {})
    max_dev = corr.get("max_abs_deviation_vs_xpu")
    max_dev_str = f"{max_dev:.3e}" if isinstance(max_dev, (int, float)) else "n/a"

    notes = []
    if status == "PROCESS_CRASHED":
        notes.append(f"PROCESS CRASHED (exit {r.get('exit_code')}): "
                      f"{(r.get('stderr') or '')[:200]}")
    elif status == "COMPILE_FAILED":
        notes.append(f"COMPILE FAILED: {(r.get('error') or '')[:200]}")
    elif r.get("infer_status") == "INFER_FAILED":
        notes.append(f"INFER FAILED: {(r.get('error') or '')[:200]}")
    else:
        dev_mm = corr.get("max_abs_deviation_vs_xpu_mm")
        if isinstance(dev_mm, (int, float)):
            notes.append(f"{dev_mm:.3f} mm vs {corr.get('posenet_min_mae_mm', '?')} mm min MAE")
        exec_dev = r.get("execution_devices")
        if exec_dev:
            notes.append(f"EXECUTION_DEVICES={exec_dev}")

    return (f"| {device} | {precision} | {status} | {_fmt(lat.get('min_ms'))} | "
            f"{_fmt(lat.get('mean_ms'))} | {_fmt(lat.get('median_ms'))} | "
            f"{_fmt(lat.get('p95_ms'))} | {_fmt(lat.get('std_ms'))} | "
            f"{_fmt(lat.get('throughput_hz'))} | {max_dev_str} | {'; '.join(notes)} |")


def append_int8_to_report(artifact_dir: Path) -> None:
    """Append INT8 rows to the EXISTING results table in
    docs/hardware/m10-phase4-benchmark.md (inserted right before the
    '## Interpretation' heading), and append a new section at the end of
    the file with calibration methodology, size comparison and a per-device
    recommendation. Does NOT touch anything else in the file -- Phase 4's
    own text, table rows and findings are left byte-identical.
    """
    if not REPORT_PATH.exists():
        raise FileNotFoundError(f"{REPORT_PATH} not found -- Phase 4 must run first.")

    text = REPORT_PATH.read_text(encoding="utf-8")
    anchor = "\n## Interpretation\n"
    if anchor not in text:
        raise ValueError(f"expected anchor {anchor!r} not found in {REPORT_PATH} -- "
                          f"refusing to guess where to insert INT8 rows.")

    int8_rows = _load_int8_rows(artifact_dir)
    if not int8_rows:
        raise RuntimeError("no INT8 rows found in results.jsonl -- run --bench first.")

    new_row_lines = "\n".join(_build_table_row(r) for r in int8_rows)
    before, after = text.split(anchor, 1)
    # `before` ends with the last existing table row line followed by a
    # blank line (write_report's join scheme puts a blank line before every
    # "\n## Heading\n" marker) -- insert the new rows right after the last
    # row, then restore that same blank line before the anchor's heading.
    updated = before.rstrip("\n") + "\n" + new_row_lines + "\n" + anchor + after

    # --- New section: calibration, sizes, correctness, recommendation. ---
    quant_info_path = artifact_dir / "int8_convert_info.json"
    calib_info_path = artifact_dir / "int8_calibration_info.json"
    quant_info = json.loads(quant_info_path.read_text()) if quant_info_path.exists() else {}
    calib_info = json.loads(calib_info_path.read_text()) if calib_info_path.exists() else {}

    fp32_mib = (quant_info.get("fp32_bin_bytes") or 0) / (1024 * 1024)
    fp16_mib = (quant_info.get("fp16_bin_bytes") or 0) / (1024 * 1024)
    int8_mib = (quant_info.get("int8_bin_bytes") or 0) / (1024 * 1024)
    r_fp32 = quant_info.get("int8_over_fp32_ratio")
    r_fp16 = quant_info.get("int8_over_fp16_ratio")

    cpu_row = next((r for r in int8_rows if r["device"] == "CPU"), None)
    gpu_row = next((r for r in int8_rows if r["device"] == "GPU"), None)
    npu_row = next((r for r in int8_rows if r["device"] == "NPU"), None)

    def dev_dev_mm(row):
        if row is None:
            return None
        return row.get("correctness", {}).get("max_abs_deviation_vs_xpu_mm")

    section = []
    section.append("\n## INT8 quantization (M10 Phase 4 extension, ADR-050)\n")
    section.append(
        "NNCF post-training quantization applied to the EXISTING FP32 IR "
        "(`artifacts/posenet_ir/posenet_fp32.xml`, not regenerated), producing "
        "`artifacts/posenet_ir/posenet_int8.xml/.bin`. `nncf` (3.3.0) plus 6 of "
        "its declared dependencies (`packaging`, `rich`, `tabulate`, `psutil`, "
        "`safetensors`, `scipy`) were installed into `train_env` with "
        "`--no-deps` each, one at a time, stopping as soon as `import nncf` "
        "succeeded -- `ninja`, `pydot`, `scikit-learn` (nncf's remaining "
        "declared deps) and `rich`'s own `markdown-it-py`/`pygments` were "
        "never needed and are not installed (`pip check` lists them as "
        "missing, informational only). `torch.__version__` (`2.14.0+xpu`), "
        "`torch.xpu.is_available()` (`True`) and `numpy.__version__` "
        "(`2.4.6`) were verified unchanged before and after every install "
        "step. `ov_env` (`scripts/requirements-bmptl.txt`) was never touched "
        "-- NNCF lives only in `train_env`, recorded in "
        "`scripts/requirements-train.txt`.\n"
    )
    section.append(
        f"**Calibration:** {calib_info.get('n_calib', N_CALIB)} images sampled "
        f"without replacement from `data/posenet/images/` (5,000 available), "
        f"`numpy.random.default_rng(seed={calib_info.get('seed', CALIB_SEED)})`, "
        f"preprocessed identically to training (224x224 RGB -> float32 [0,1] "
        f"-> CHW, no mean/std normalization). The exact sample_index list "
        f"drawn is recorded in `artifacts/posenet_ir/int8_calibration_info.json` "
        f"for reproducibility. `nncf.quantize(..., target_device=nncf.TargetDevice.NPU)` "
        f"per ADR-013's decision that INT8 \"targets the NPU5010\" -- the "
        f"produced IR is still generic OpenVINO IR and is benchmarked on "
        f"CPU/GPU/NPU below exactly like the FP32/FP16 IRs.\n"
    )
    if r_fp16 is not None:
        section.append(
            f"**Size:** INT8 `.bin` is {int8_mib:.2f} MiB, vs FP32's {fp32_mib:.2f} "
            f"MiB ({r_fp32:.3f}x) and FP16's {fp16_mib:.2f} MiB ({r_fp16:.3f}x).\n"
        )
    else:
        section.append(
            f"**Size:** INT8 `.bin` is {int8_mib:.2f} MiB, vs FP32's {fp32_mib:.2f} "
            f"MiB ({r_fp32:.3f}x).\n"
        )

    corr_lines = ["**Correctness (max abs deviation vs the PyTorch-XPU reference, "
                  "same 5 validation samples and same `val_predictions_xpu.npy` "
                  "Phase 4 used):**\n"]
    for label, row in (("CPU", cpu_row), ("GPU", gpu_row), ("NPU", npu_row)):
        dev_mm = dev_dev_mm(row)
        if row is None:
            corr_lines.append(f"- {label}: not benchmarked.\n")
        elif row.get("status") != "COMPILE_OK" or row.get("infer_status") != "OK":
            corr_lines.append(f"- {label}: {row.get('status')} "
                               f"({row.get('error', row.get('stderr', ''))[:150]}).\n")
        elif dev_mm is None:
            corr_lines.append(f"- {label}: compiled and ran, correctness not computed "
                               f"(reference predictions unavailable).\n")
        else:
            verdict = "well under" if dev_mm < 2.625 else "at or above"
            corr_lines.append(
                f"- {label}: {dev_mm:.3f} mm. PoseNet's own smallest per-prop "
                f"ground-truth MAE is 2.625 mm (bottle; fork 3.211 mm, mug "
                f"2.768 mm) -- {dev_mm:.3f} mm is **{verdict}** that scale, "
                f"so this deviation is judged "
                f"{'immaterial to control' if dev_mm < 2.625 else 'material to control, not safe to treat as free'}.\n"
            )
    section.append("".join(corr_lines))

    section.append(
        "\n**On comparing GPU INT8 to GPU FP16 specifically:** Phase 4 found "
        "GPU FP32 and GPU FP16 report byte-identical deviation "
        "(1.445e-4) and near-identical latency, consistent with the Arc "
        "B390 plugin running its internal compute in FP16 regardless of the "
        "IR's stored weight precision. If that holds, a \"GPU INT8 vs GPU "
        "FP16\" comparison here may be comparing INT8 against an "
        "already-FP16-internal baseline rather than against a genuinely "
        "higher-precision one -- worth keeping in mind when reading the GPU "
        "row above, not a claim this script can verify without overriding "
        "`INFERENCE_PRECISION_HINT` directly.\n"
    )

    def lat_mean(row):
        if row is None:
            return None
        return row.get("latency_ms", {}).get("mean_ms")

    def device_verdict(label: str, row: dict, is_npu_target: bool = False) -> str:
        """One recommendation line for a device, built only from what was
        actually measured -- never a canned claim."""
        if row is None:
            return f"- **{label}:** not benchmarked."
        if row.get("status") == "PROCESS_CRASHED":
            return (f"- **{label}:** process crashed (exit "
                    f"{row.get('exit_code')}) -- reported as a crash, not "
                    f"silently omitted. Not usable for INT8 on this device "
                    f"on this run.")
        if row.get("status") == "COMPILE_FAILED":
            return (f"- **{label}:** compile failed "
                    f"({(row.get('error') or '')[:150]}) -- INT8 on this "
                    f"device is a documented capability limit, not usable.")
        mean_ms = lat_mean(row)
        dev_mm = row.get("correctness", {}).get("max_abs_deviation_vs_xpu_mm")
        if mean_ms is None:
            return f"- **{label}:** compiled but latency not recorded."
        parts = [f"- **{label}:** {mean_ms:.4f} ms mean latency"]
        if isinstance(dev_mm, (int, float)):
            ok = dev_mm < 2.625
            verdict_text = "acceptable" if ok else "exceeds the model's own MAE scale -- treat with caution"
            parts.append(f", {dev_mm:.3f} mm deviation ({verdict_text})")
        tag = " (ADR-013's intended INT8 target device)" if is_npu_target else ""
        return "".join(parts) + tag + "."

    section.append("\n**Per-device recommendation, from the measurements above:**\n")
    section.append(device_verdict("CPU", cpu_row) + "\n")
    section.append(device_verdict("GPU", gpu_row) + "\n")
    section.append(device_verdict("NPU", npu_row, is_npu_target=True) + "\n")

    updated = updated.rstrip("\n") + "\n" + "\n".join(section) + "\n"
    REPORT_PATH.write_text(updated, encoding="utf-8")
    print(f"Appended INT8 rows and a new section to {REPORT_PATH}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--all", action="store_true",
                   help="Run every stage in order: quantize, bench, report.")
    p.add_argument("--quantize", action="store_true",
                   help="Sample calibration images, run nncf.quantize(), write posenet_int8.xml/.bin.")
    p.add_argument("--bench", action="store_true",
                   help="Benchmark INT8 on every device (each in its own subprocess).")
    p.add_argument("--bench-one", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--report", action="store_true",
                   help="Append INT8 rows + a new section to docs/hardware/m10-phase4-benchmark.md.")
    p.add_argument("--device", choices=DEVICES, default=None, help="Used with --bench-one.")
    p.add_argument("--artifact-dir", type=Path, default=ARTIFACT_DIR)
    p.add_argument("--n-calib", type=int, default=N_CALIB)
    p.add_argument("--seed", type=int, default=CALIB_SEED)
    return p.parse_args()


def main():
    args = parse_args()
    artifact_dir = args.artifact_dir

    if args.bench_one:
        if not args.device:
            raise SystemExit("--bench-one requires --device")
        do_bench_one(artifact_dir, args.device)
        return

    if not any([args.all, args.quantize, args.bench, args.report]):
        raise SystemExit("Specify --all, or one or more of --quantize/--bench/--report.")

    if args.all or args.quantize:
        do_quantize(artifact_dir, args.n_calib, args.seed)

    if args.all or args.bench:
        run_all_int8_benchmarks(artifact_dir)

    if args.all or args.report:
        append_int8_to_report(artifact_dir)


if __name__ == "__main__":
    main()
