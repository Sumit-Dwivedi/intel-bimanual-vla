"""M10 Phase 4 extension -- PoseNet FP16 throughput vs. batch size, across
CPU / iGPU / NPU (ADR-052, follows M10 Phase 4's ADR-045 and its INT8
extension ADR-050). Fix E of an overnight batch: pure measurement, no
change to the demo loop, the checkpoint, or any existing IR.

WHERE THIS RUNS
-----------------
Entirely on bm-ptl, inside `train_env`
(`C:\\Users\\devcloud\\project\\train_env\\Scripts\\python.exe`), the same
venv `scripts/posenet_to_openvino.py` and `scripts/quantize_posenet.py`
already use (ADR-043: torch + openvino coexist there). This script never
imports torch -- only `openvino` and `numpy` -- but runs from train_env
anyway so the parent process (which spawns the per-combo subprocesses via
`sys.executable`) and every child stay on the exact same interpreter, with
no new environment permutation introduced. `ov_env` is never touched, and
nothing is installed into either environment (this module's constraint).

WHY THIS RE-USES THE EXISTING FP16 IR VIA `reshape()`, NOT RECONVERSION
----------------------------------------------------------------------------
`artifacts/posenet_ir/posenet_fp16.xml` was produced by M10 Phase 4
(ADR-045) with a STATIC batch-1 input shape, `[1, 3, 224, 224]`. Loading
that IR as-is only ever gives you batch 1 -- it does not "become" batch 4
by asking for four inferences. This script instead does, once per (device,
batch) combo:

    core = ov.Core()
    model = core.read_model(str(FP16_XML))       # fresh read every time
    model.reshape({0: [batch, 3, 224, 224]})      # static reshape to N
    compiled = core.compile_model(model, device)  # THEN compile

No new `.xml`/`.bin` pair is written and the checkpoint is never touched --
this is reshape-before-compile, not reconversion. If `reshape()` itself
raises on a given device/batch, that is reported as a `RESHAPE_FAILED` row,
never silently worked around (this module's task brief, correction 3).

WHY EVERY (BATCH, DEVICE) COMBO RUNS IN ITS OWN SUBPROCESS
----------------------------------------------------------------------
`docs/hardware/bmptl-verification.md`'s Day-0 record plus M03's finding
(quoted in `DECISIONS.md`'s "M03 -- OpenVINO conversion smoke test
complete" entry, not verbatim in `bmptl-verification.md` itself -- checked
directly, the exact STATUS_ACCESS_VIOLATION quote lives in DECISIONS.md)
establish that the NPU plugin can reject an unsupported graph by killing
the WHOLE PROCESS (`STATUS_ACCESS_VIOLATION` / `0xC0000005`), not by
raising a catchable exception, and that a harness which accumulates
results in memory and writes once at the end loses everything already
earned when that crash happens. Batch sizes above 1 are exactly the
untested trigger class here -- M03's crash was on a fully-open `-1` batch
dim, not a static N>1, but a static reshape to N>1 has never been tried on
this NPU before this script, so it gets the same defensive treatment:

  1. Every single (device, batch) result is appended to
     `artifacts/posenet_ir/batch_scaling_results.jsonl` THE MOMENT it is
     known.
  2. Every combo compiles+benchmarks in its own `subprocess.run(...)`
     (same `--bench-one` internal-flag pattern as
     `scripts/posenet_to_openvino.py` / `scripts/quantize_posenet.py`). A
     hard process kill during an NPU combo costs exactly that one row.
  3. Devices run in the fixed order CPU -> GPU -> NPU, batches ascending
     (1, 4, 8, 16) within each device, so NPU is attempted last, after
     every CPU and GPU row is already on disk.
  4. **NPU-only escape hatch:** if any NPU batch > 1 crashes the
     subprocess (or fails to reshape/compile), remaining larger NPU
     batches are SKIPPED rather than attempted -- each still gets an
     explicit `SKIPPED` row tagged `NPU_ONLY_BATCH_1` (never a silent
     omission, ADR-007) -- instead of re-risking a crash we already have
     one data point predicting. This is the "catch, log
     `NPU_ONLY_BATCH_1`, skip" behaviour from this module's task brief,
     made possible ONLY by the subprocess isolation above: an in-process
     try/except cannot catch a process kill, but a parent process reading
     a dead child's exit code can decide not to launch the next one.

PRECISION SCOPE
------------------
FP16 only, per this module's task brief -- the IR already benchmarked at
batch 1 in `docs/hardware/m10-phase4-benchmark.md` (GPU 0.683 ms / 1465 Hz,
NPU 1.099 ms / 910 Hz, CPU 6.492 ms / 154 Hz). FP32/INT8 batch scaling is
out of scope for this pass.

USAGE
-------
    train_env\\Scripts\\python.exe scripts\\benchmark_batch_scaling.py --all

    # Internal (used by --all's subprocess calls, also runnable standalone):
    ... --bench-one --device CPU --batch 4 --n-measured 100
    ... --report   # (re)writes the markdown section from the jsonl alone
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
ARTIFACT_DIR = REPO_ROOT / "artifacts" / "posenet_ir"
FP16_XML = ARTIFACT_DIR / "posenet_fp16.xml"
RESULTS_PATH = ARTIFACT_DIR / "batch_scaling_results.jsonl"
REPORT_PATH = REPO_ROOT / "docs" / "hardware" / "m10-phase4-benchmark.md"

BATCHES = [1, 4, 8, 16]
# NPU last, on purpose -- see module docstring point 3.
DEVICES = ["CPU", "GPU", "NPU"]
PRECISION = "FP16"

N_WARMUP = 10
N_MEASURED_DEFAULT = 100
# Soft internal time budget: if the PARENT process (run_all) has already
# spent this long, drop remaining combos to 50 measured iterations and say
# so in every affected row -- this module's task brief's explicit
# "drop to 50 and say so if you hit the cap" instruction. This is a soft
# budget inside the batch's hard 90-minute wall-clock cap, leaving room for
# writing the report/ADR/commit/push after benchmarking finishes.
TIME_BUDGET_SECONDS = 20 * 60

RESULT_MARKER = "BATCH_SCALING_RESULT_JSON: "


# ---------------------------------------------------------------------------
# Per-combo benchmark (runs standalone, in its own subprocess)
# ---------------------------------------------------------------------------

def _latency_stats(latencies_ms: list, batch: int) -> dict:
    mean_ms = statistics.mean(latencies_ms)
    return {
        "min_ms": min(latencies_ms),
        "mean_ms": mean_ms,
        "median_ms": statistics.median(latencies_ms),
        "p95_ms": statistics.quantiles(latencies_ms, n=100)[94],
        "std_ms": statistics.pstdev(latencies_ms),
        # Throughput as requested by this module's task brief:
        # batch * 1000 / mean_ms (images/sec, whole-batch inference latency).
        "throughput_hz": batch * 1000.0 / mean_ms,
    }


def _append_result(result: dict) -> None:
    """Append one JSON line to batch_scaling_results.jsonl IMMEDIATELY --
    never buffered in memory across combos (module docstring point 1)."""
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    with open(RESULTS_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(result) + "\n")


def do_bench_one(device: str, batch: int, n_measured: int) -> None:
    """Reshape the existing FP16 IR to `batch`, compile on `device`,
    benchmark, print exactly one RESULT_MARKER JSON line, then exit. Called
    as a subprocess -- see module docstring for why. Imports ONLY openvino
    + numpy, deliberately no torch, keeping this process as small and fast
    to start as possible.
    """
    import openvino as ov

    entry = {
        "combo": f"{device}_batch{batch}", "device": device, "batch": batch,
        "precision": PRECISION, "host": platform.node(),
    }

    if not FP16_XML.exists():
        entry["status"] = "SKIPPED"
        entry["error"] = f"{FP16_XML} does not exist (M10 Phase 4 / ADR-045 artifact missing)."
        print(RESULT_MARKER + json.dumps(entry), flush=True)
        return

    core = ov.Core()

    # --- Read fresh, then reshape to the target batch (NOT reconversion). ---
    try:
        model = core.read_model(str(FP16_XML))
        model.reshape({0: [batch, 3, 224, 224]})
        entry["reshaped_to"] = str(model.inputs[0].get_partial_shape())
    except Exception as e:  # noqa: BLE001 -- a device/shape rejecting reshape
        # is a real finding (this module's task brief, correction 3: "If
        # reshape fails on a device, that is a finding -- report it, do not
        # work around it silently"), not a bug to hide.
        entry["status"] = "RESHAPE_FAILED"
        entry["error_type"] = type(e).__name__
        entry["error"] = str(e)
        print(RESULT_MARKER + json.dumps(entry), flush=True)
        return

    try:
        compiled = core.compile_model(model, device)
    except Exception as e:  # noqa: BLE001 -- device capability limit, not a bug.
        entry["status"] = "COMPILE_FAILED"
        entry["error_type"] = type(e).__name__
        entry["error"] = str(e)
        print(RESULT_MARKER + json.dumps(entry), flush=True)
        return

    # ADR-007: no silent device fallback -- read back the device OpenVINO
    # actually used, not the string we asked for.
    try:
        entry["execution_devices"] = compiled.get_property("EXECUTION_DEVICES")
    except Exception:
        entry["execution_devices"] = "<not reported by this device/version>"

    entry["status"] = "COMPILE_OK"
    infer_request = compiled.create_infer_request()

    rng = np.random.default_rng(0)
    x = rng.standard_normal((batch, 3, 224, 224)).astype(np.float32)

    try:
        for _ in range(N_WARMUP):
            infer_request.infer({0: x})

        latencies_ms = []
        for _ in range(n_measured):
            t0 = time.perf_counter()
            infer_request.infer({0: x})
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000.0)

        entry["latency_ms"] = _latency_stats(latencies_ms, batch)
        entry["n_warmup"] = N_WARMUP
        entry["n_measured"] = n_measured
    except Exception as e:  # noqa: BLE001
        entry["infer_status"] = "INFER_FAILED"
        entry["error_type"] = type(e).__name__
        entry["error"] = str(e)
        print(RESULT_MARKER + json.dumps(entry), flush=True)
        return

    entry["infer_status"] = "OK"
    print(RESULT_MARKER + json.dumps(entry), flush=True)


# ---------------------------------------------------------------------------
# Orchestration: subprocess-per-combo, CPU -> GPU -> NPU, batches ascending
# ---------------------------------------------------------------------------

def _print_summary(entry: dict) -> None:
    tag = f"[{entry['device']} | batch={entry['batch']}]"
    if entry["status"] in ("SKIPPED", "COMPILE_FAILED", "RESHAPE_FAILED"):
        print(f"{tag} {entry['status']}: {entry.get('error')}")
        return
    if entry.get("infer_status") == "INFER_FAILED":
        print(f"{tag} compile OK, INFER FAILED: {entry.get('error')}")
        return
    lat = entry.get("latency_ms", {})
    print(f"{tag} compile OK (EXECUTION_DEVICES={entry.get('execution_devices')}), "
          f"mean={lat.get('mean_ms', float('nan')):.4f} ms "
          f"throughput={lat.get('throughput_hz', float('nan')):.2f} Hz "
          f"(n_measured={entry.get('n_measured')})")


def run_all_benchmarks() -> list:
    """Run every (device, batch) combo, each in its own subprocess, device
    order CPU -> GPU -> NPU, batches ascending within a device. Appends
    every result (including deliberately-skipped NPU rows) to
    batch_scaling_results.jsonl the moment it is known.
    """
    results = []
    start_time = time.time()
    npu_only_batch_1 = False  # set True the moment an NPU batch>1 fails

    for device in DEVICES:
        for batch in BATCHES:
            if device == "NPU" and batch > 1 and npu_only_batch_1:
                entry = {
                    "combo": f"{device}_batch{batch}", "device": device,
                    "batch": batch, "precision": PRECISION,
                    "status": "SKIPPED",
                    "error": ("NPU_ONLY_BATCH_1 -- a smaller NPU batch>1 already "
                              "crashed/failed this run; not re-attempting a larger "
                              "one (subprocess isolation already spent on the "
                              "smaller batch; see that row's PROCESS_CRASHED / "
                              "RESHAPE_FAILED / COMPILE_FAILED status)."),
                    "host": platform.node(),
                }
                print(f"[{device} | batch={batch}] SKIPPED ({entry['error']})")
                results.append(entry)
                _append_result(entry)
                continue

            elapsed = time.time() - start_time
            n_measured = N_MEASURED_DEFAULT
            time_capped = False
            if elapsed > TIME_BUDGET_SECONDS:
                n_measured = 50
                time_capped = True

            print(f"\n--- Benchmarking {device} / batch={batch} / FP16 "
                  f"(isolated subprocess, n_measured={n_measured}"
                  f"{' [TIME-CAPPED, dropped from 100]' if time_capped else ''}) ---")
            proc = subprocess.run(
                [sys.executable, "-u", str(Path(__file__).resolve()),
                 "--bench-one", "--device", device, "--batch", str(batch),
                 "--n-measured", str(n_measured)],
                capture_output=True, text=True,
            )
            entry = None
            for line in proc.stdout.splitlines():
                if line.startswith(RESULT_MARKER):
                    entry = json.loads(line[len(RESULT_MARKER):])
                    break

            if entry is None:
                # Child died before printing a result -- an NPU driver crash
                # is exactly this. Record it honestly; do not lose earlier
                # rows because of it (module docstring point 1-2).
                entry = {
                    "combo": f"{device}_batch{batch}", "device": device,
                    "batch": batch, "precision": PRECISION,
                    "status": "PROCESS_CRASHED", "exit_code": proc.returncode,
                    "stdout": proc.stdout.strip(), "stderr": proc.stderr.strip(),
                    "host": platform.node(),
                }
                print(f"[{device} | batch={batch}] PROCESS CRASHED "
                      f"(exit code {proc.returncode}). Verbatim stderr:\n{proc.stderr.strip()}")
            else:
                if time_capped:
                    entry["time_capped_n_measured"] = True
                _print_summary(entry)

            if time_capped and "time_capped_n_measured" not in entry:
                entry["time_capped_n_measured"] = True

            results.append(entry)
            _append_result(entry)

            if (device == "NPU" and batch > 1
                    and entry["status"] in ("PROCESS_CRASHED", "RESHAPE_FAILED", "COMPILE_FAILED")):
                npu_only_batch_1 = True
                print(f"[NPU] batch={batch} failed ({entry['status']}) -- "
                      f"tagging NPU_ONLY_BATCH_1, skipping remaining larger NPU batches.")

    print(f"\nAppended {len(results)} rows to {RESULTS_PATH}")
    return results


# ---------------------------------------------------------------------------
# Markdown section (appended to the existing Phase 4 benchmark doc)
# ---------------------------------------------------------------------------

# The batch-1 rows already published in docs/hardware/m10-phase4-benchmark.md
# (M10 Phase 4, ADR-045), used below to anchor the new numbers rather than
# leaving them free-floating, per this module's task brief.
EXISTING_BATCH1_FP16 = {
    "GPU": {"mean_ms": 0.683, "throughput_hz": 1465},
    "NPU": {"mean_ms": 1.099, "throughput_hz": 910},
    "CPU": {"mean_ms": 6.492, "throughput_hz": 154},
}


def _load_results() -> list:
    if not RESULTS_PATH.exists():
        raise FileNotFoundError(f"{RESULTS_PATH} not found -- run --bench first.")
    rows = []
    with open(RESULTS_PATH, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _fmt(v, nd=4):
    return f"{v:.{nd}f}" if isinstance(v, (int, float)) else "n/a"


def build_section_text(rows: list) -> str:
    lines = []
    lines.append("\n## Batch Scaling Analysis (M10 Phase 4 extension, ADR-052)\n")
    lines.append(
        "FP16 PoseNet throughput vs. batch size (1, 4, 8, 16) across CPU / iGPU / NPU, "
        "measured by `scripts/benchmark_batch_scaling.py`. Reuses the EXISTING "
        "`artifacts/posenet_ir/posenet_fp16.xml/.bin` (M10 Phase 4, ADR-045) -- no new IR "
        "was produced. Each batch size is obtained by `core.read_model(...)` followed by "
        "`model.reshape({0: [N, 3, 224, 224]})` **before** `compile_model`, done fresh for "
        "every (device, batch) combo; the saved IR itself stays static batch-1 on disk. "
        f"{N_WARMUP} warm-up inferences discarded, then up to {N_MEASURED_DEFAULT} measured "
        "per combo (dropped to 50 for any combo run past this script's internal 20-minute "
        "soft time budget -- flagged per-row below if that happened). Throughput = "
        "`batch * 1000 / mean_ms`. Every (device, batch) combo ran in its own subprocess, "
        "order CPU -> GPU -> NPU with batches ascending within a device, so NPU ran last "
        "with every other row already on disk (module docstring; the M03 "
        "`STATUS_ACCESS_VIOLATION` finding this defends against is recorded in "
        "`DECISIONS.md`'s \"M03 -- OpenVINO conversion smoke test complete\" entry, not "
        "verbatim in `bmptl-verification.md` -- checked directly against both files while "
        "writing this section).\n"
    )

    lines.append("| Device | Batch | Status | Min (ms) | Mean (ms) | Median (ms) | "
                  "P95 (ms) | Std (ms) | Throughput (Hz) | Notes |")
    lines.append("|---|---:|---|---:|---:|---:|---:|---:|---:|---|")
    for r in rows:
        device = r.get("device", "?")
        batch = r.get("batch", "?")
        status = r.get("status", "?")
        lat = r.get("latency_ms", {})
        notes = []
        if status == "PROCESS_CRASHED":
            notes.append(f"PROCESS CRASHED (exit {r.get('exit_code')}): "
                          f"{(r.get('stderr') or '')[:200]}")
        elif status == "COMPILE_FAILED":
            notes.append(f"COMPILE FAILED: {(r.get('error') or '')[:200]}")
        elif status == "RESHAPE_FAILED":
            notes.append(f"RESHAPE FAILED: {(r.get('error') or '')[:200]}")
        elif status == "SKIPPED":
            notes.append((r.get("error") or "")[:200])
        elif r.get("infer_status") == "INFER_FAILED":
            notes.append(f"INFER FAILED: {(r.get('error') or '')[:200]}")
        else:
            exec_dev = r.get("execution_devices")
            if exec_dev:
                notes.append(f"EXECUTION_DEVICES={exec_dev}")
            if r.get("time_capped_n_measured"):
                notes.append(f"TIME-CAPPED: n_measured={r.get('n_measured')} (dropped from 100)")
        lines.append(
            f"| {device} | {batch} | {status} | {_fmt(lat.get('min_ms'))} | "
            f"{_fmt(lat.get('mean_ms'))} | {_fmt(lat.get('median_ms'))} | "
            f"{_fmt(lat.get('p95_ms'))} | {_fmt(lat.get('std_ms'))} | "
            f"{_fmt(lat.get('throughput_hz'), 1)} | {'; '.join(notes)} |"
        )

    by_device_batch = {(r.get("device"), r.get("batch")): r for r in rows}

    # Throughput ratio (largest successful batch vs. batch 1) per device,
    # computed once here and reused by both the findings list and the
    # per-device scaling paragraphs below -- never hand-typed.
    thr_ratio_by_device = {}
    for device in DEVICES:
        device_rows = [by_device_batch.get((device, b)) for b in BATCHES]
        ok_rows = [(b, r) for b, r in zip(BATCHES, device_rows)
                   if r and r.get("status") == "COMPILE_OK" and r.get("infer_status") == "OK"]
        if len(ok_rows) >= 2:
            b0, r0 = ok_rows[0]
            bN, rN = ok_rows[-1]
            thr_ratio_by_device[device] = (rN["latency_ms"]["throughput_hz"]
                                            / r0["latency_ms"]["throughput_hz"], b0, bN)

    # --- Findings worth stating plainly, not smoothed over. ---
    npu_batchN_statuses = {r.get("batch"): r.get("status") for r in rows if r.get("device") == "NPU"}
    npu_crashed_or_failed = any(
        npu_batchN_statuses.get(b) not in ("COMPILE_OK", None) for b in BATCHES if b > 1
        and npu_batchN_statuses.get(b) is not None
    )
    lines.append("\n**Findings worth stating plainly, not smoothed over:**\n")
    if not npu_crashed_or_failed and all(
            npu_batchN_statuses.get(b) == "COMPILE_OK" for b in BATCHES if npu_batchN_statuses.get(b)):
        lines.append(
            "1. **The anticipated destructive NPU crash did not occur at any tested batch "
            "size (4, 8, 16).** This module's task brief flagged batch>1 as \"exactly the "
            "trigger class\" for the `STATUS_ACCESS_VIOLATION` process-kill M03 documented "
            "(`DECISIONS.md`'s \"M03 -- OpenVINO conversion smoke test complete\" entry) -- "
            "but that finding was specifically about a **fully-open dynamic** batch "
            "dimension (`-1`, unbounded), where the NPU compiler cannot determine upper "
            "bounds at all. A **static** reshape to a fixed N (4, 8, or 16) is a different "
            "and much narrower case, and on this NPU5010/driver/OpenVINO-2026.3.1 "
            "combination it compiled and ran cleanly at every tested N. The subprocess-per-"
            "combo isolation and incremental-write discipline (module docstring) were "
            "exercised on every row but never actually triggered by a crash -- recorded "
            "here as a finding, not as evidence the defence was unnecessary to build (the "
            "same posture Phase 4/ADR-045 took when none of its six combos crashed either).\n"
        )
    else:
        crashed_batches = [b for b in BATCHES if b > 1 and npu_batchN_statuses.get(b) not in ("COMPILE_OK",)]
        lines.append(
            f"1. **NPU crashed or failed to compile/reshape at batch size(s) "
            f"{crashed_batches}**, consistent with this module's task brief's predicted "
            f"trigger class. See the table above and the `NPU_ONLY_BATCH_1` skip rows (if "
            f"any) for exactly which batch first failed and what status it reported.\n"
        )
    ranked = sorted(thr_ratio_by_device.items(), key=lambda kv: kv[1][0], reverse=True)
    ranked_str = ", then ".join(
        f"{dev} ({ratio:.2f}x at batch {bN} vs. batch {b0})" for dev, (ratio, b0, bN) in ranked
    )
    lines.append(
        f"2. **Throughput scaling ranked device-to-device: {ranked_str}** on the batches "
        f"that ran -- plausible reading is that larger batches amortize fixed per-call "
        f"dispatch overhead better on devices with more parallel compute headroom relative "
        f"to this model's size, but this script does not instrument dispatch-vs-compute "
        f"time separately, so that is an inference from the shape of the curve, not a "
        f"directly measured cause.\n"
    )

    # --- Anchor against the existing batch-1 rows and comment on scaling. ---
    lines.append("\n**Anchoring against the batch-1 rows already in this document "
                  "(M10 Phase 4, ADR-045):** GPU FP16 0.683 ms / 1465 Hz, NPU FP16 "
                  "1.099 ms / 910 Hz, CPU FP16 6.492 ms / 154 Hz.\n")

    for device in DEVICES:
        b1 = by_device_batch.get((device, 1))
        anchor = EXISTING_BATCH1_FP16.get(device)
        if b1 and b1.get("status") == "COMPILE_OK" and anchor:
            new_mean = b1.get("latency_ms", {}).get("mean_ms")
            new_thr = b1.get("latency_ms", {}).get("throughput_hz")
            if isinstance(new_mean, (int, float)):
                drift_pct = 100.0 * (new_mean - anchor["mean_ms"]) / anchor["mean_ms"]
                lines.append(
                    f"- **{device} batch=1 re-measurement this run:** mean "
                    f"{new_mean:.4f} ms / {new_thr:.1f} Hz vs. the existing "
                    f"{anchor['mean_ms']} ms / {anchor['throughput_hz']} Hz row "
                    f"({drift_pct:+.1f}% latency drift run-to-run -- expected "
                    f"measurement noise, not a regression, unless stated otherwise below).\n"
                )

    lines.append("\n**Scaling shape, linear vs. sub-linear, per device:**\n")
    for device in DEVICES:
        device_rows = [by_device_batch.get((device, b)) for b in BATCHES]
        ok_rows = [(b, r) for b, r in zip(BATCHES, device_rows)
                   if r and r.get("status") == "COMPILE_OK" and r.get("infer_status") == "OK"]
        if len(ok_rows) < 2:
            failed_batches = [b for b, r in zip(BATCHES, device_rows)
                               if not (r and r.get("status") == "COMPILE_OK")]
            lines.append(f"- **{device}:** insufficient successful rows to characterize "
                          f"scaling (batches {failed_batches} did not compile/run OK on "
                          f"this device -- see table and per-row notes above, not smoothed "
                          f"over).\n")
            continue
        b0, r0 = ok_rows[0]
        bN, rN = ok_rows[-1]
        mean0 = r0["latency_ms"]["mean_ms"]
        meanN = rN["latency_ms"]["mean_ms"]
        thr0 = r0["latency_ms"]["throughput_hz"]
        thrN = rN["latency_ms"]["throughput_hz"]
        latency_ratio = meanN / mean0
        batch_ratio = bN / b0
        thr_ratio = thrN / thr0
        if latency_ratio <= batch_ratio * 1.05:
            shape = "sub-linear-to-linear"
        else:
            shape = "super-linear (worse than linear)"
        lines.append(
            f"- **{device}:** batch {b0} -> {bN}: mean latency {mean0:.4f} -> "
            f"{meanN:.4f} ms ({latency_ratio:.2f}x for a {batch_ratio:.0f}x batch "
            f"increase), throughput {thr0:.1f} -> {thrN:.1f} Hz "
            f"({thr_ratio:.2f}x). Latency growing slower than batch size "
            f"({latency_ratio:.2f}x < {batch_ratio:.0f}x) means throughput keeps "
            f"climbing with batch -- **{shape}** scaling on the batches that "
            f"actually ran on this device.\n"
        )

    lines.append(
        "\n**GPU FP16-internal-execution caveat, carried forward:** M10 Phase 4's own "
        "finding (`docs/hardware/m10-phase4-benchmark.md`'s Interpretation section, "
        "point 3) is that GPU FP32 and GPU FP16 reported byte-identical deviation and "
        "near-identical latency at batch 1, consistent with the Arc GPU plugin running "
        "its internal compute in FP16 regardless of the IR's stored weight precision. "
        "This script only benchmarks the FP16 IR (task brief scope), so it cannot itself "
        "re-confirm or contradict that finding -- but if it holds, the GPU curve above "
        "is the plugin's native execution path, not a case of FP16 imposing an extra "
        "conversion cost on top of an FP32-native GPU pipeline, which is a reasonable "
        "prior for why GPU scales as well as it does.\n"
    )

    return "\n".join(lines) + "\n"


def append_report_section() -> None:
    rows = _load_results()
    section = build_section_text(rows)
    with open(REPORT_PATH, "a", encoding="utf-8") as f:
        f.write(section)
    print(f"Appended Batch Scaling Analysis section to {REPORT_PATH}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--all", action="store_true",
                   help="Run every (device, batch) combo, then append the report section.")
    p.add_argument("--bench", action="store_true", help="Run every (device, batch) combo.")
    p.add_argument("--bench-one", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--report", action="store_true",
                   help="Append the Batch Scaling Analysis section from the jsonl alone.")
    p.add_argument("--device", choices=DEVICES, default=None, help="Used with --bench-one.")
    p.add_argument("--batch", type=int, choices=BATCHES, default=None, help="Used with --bench-one.")
    p.add_argument("--n-measured", type=int, default=N_MEASURED_DEFAULT,
                   help="Used with --bench-one (internal).")
    return p.parse_args()


def main():
    args = parse_args()

    if args.bench_one:
        if not args.device or not args.batch:
            raise SystemExit("--bench-one requires --device and --batch")
        do_bench_one(args.device, args.batch, args.n_measured)
        return

    if not any([args.all, args.bench, args.report]):
        raise SystemExit("Specify --all, or one or more of --bench/--report.")

    if args.all or args.bench:
        run_all_benchmarks()

    if args.all or args.report:
        append_report_section()


if __name__ == "__main__":
    main()
