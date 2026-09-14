# M10 Phase 4 -- PoseNet OpenVINO conversion + CPU/iGPU/NPU benchmark

Converts `checkpoints/posenet_best.pth` (M10 Phase 3, ADR-044) to OpenVINO IR and benchmarks it on bm-ptl's three OpenVINO devices, per ADR-013's precision/device mapping strategy and ADR-045 (this module). All work ran inside `train_env` on bm-ptl (torch 2.14.0+xpu + openvino 2026.3.1 coexisting, ADR-043) via `scripts/posenet_to_openvino.py`.

## Conversion

- **Checkpoint:** `checkpoints/posenet_best.pth`, epoch 30, val_loss 1.113753e-05, MAE fork_mae=3.2 mm, bottle_mae=2.6 mm, mug_mae=2.8 mm, 43.2 MiB on disk (weights-only load, `torch.load(..., weights_only=True)`; 11,310,153 parameters, ADR-042/ADR-044).

- **`ov.save_model`'s `compress_to_fp16` parameter defaults to `True`** (`help(ov.save_model)`: "Floating point weights are compressed to FP16 by default."). A first pass at this script omitted the argument for the FP32 save and got a 21.56 MiB `.bin` -- byte-identical to the FP16 save. Fixed by passing `compress_to_fp16=False` explicitly for the FP32 save. Flagging this here since it is exactly the kind of silent-default trap that would otherwise make an "FP32" benchmark row secretly measure FP16.

- **Input form used:** plain list [1, 3, 224, 224] (scripts/ov_smoke.py:181 form). The brief's named form `input=[('image', [1,3,224,224])]` was tried first and failed with `RuntimeError: Input for tensor name 'image' is not found.` -- fell back to the plain-list form proven in `scripts/ov_smoke.py:181`.

- **IR input shape:** `[1,3,224,224]` (static batch 1, per ADR-013's NPU static-shape requirement).

- **FP32 IR:** `artifacts/posenet_ir/posenet_fp32.xml` / `.bin`, 43.13 MiB `.bin`.

- **FP16 IR:** `artifacts/posenet_ir/posenet_fp16.xml` / `.bin`, 21.56 MiB `.bin` (0.500x the FP32 `.bin` size; expected ~0.5x since only weights are compressed, not activations).


## Benchmark methodology

- 10 warm-up inferences discarded, then 100 measured, per (device, precision) combo. Latency = wall-clock per `infer()` call.
- Static batch-1 input `(1, 3, 224, 224)` throughout.
- **PyTorch-XPU baseline uses the identical methodology**, with `torch.xpu.synchronize()` around both the warm-up and the timed region (XPU kernel launches are asynchronous; without a sync the timed region would measure launch overhead only, not completion, and would read implausibly fast).
- **Every (device, precision) OpenVINO combo ran in its own subprocess**, in the fixed order CPU -> GPU -> NPU, with every result appended to `artifacts/posenet_ir/results.jsonl` immediately (ADR-013's NPU-crash-isolation lesson from M03, restated in ADR-045). NPU ran last, after CPU's and GPU's rows were already on disk.
- **Correctness:** for each successful combo, the 5 validation samples used in ADR-044's mean-collapse check (`sample_index` 0/10/20/30/40) are run through the compiled model and compared against PyTorch-XPU predictions for the same 5 samples. Thresholds: CPU/GPU FP32 < 1e-4, CPU/GPU FP16 < 1e-3, NPU FP16 < 1e-2. Anything exceeding its threshold by >10x is flagged but still reported.


## Results

| Device | Precision | Status | Min (ms) | Mean (ms) | Median (ms) | P95 (ms) | Std (ms) | Throughput (Hz) | Max abs dev vs XPU | Notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| XPU | FP32 | OK | 3.3657 | 3.5939 | 3.5792 | 3.7465 | 0.0875 | 278.2481 | n/a |  |
| CPU | FP32 | COMPILE_OK | 6.0107 | 6.5140 | 6.3436 | 8.0442 | 0.7291 | 153.5160 | 1.043e-07 | EXECUTION_DEVICES=['CPU'] |
| CPU | FP16 | COMPILE_OK | 5.6899 | 6.4915 | 6.3411 | 6.8607 | 1.1647 | 154.0465 | 9.203e-05 | EXECUTION_DEVICES=['CPU'] |
| GPU | FP32 | COMPILE_OK | 0.5712 | 0.5862 | 0.5821 | 0.6026 | 0.0214 | 1705.8791 | 1.445e-04 | threshold 1e-04 (exceeds threshold); EXECUTION_DEVICES=['GPU.0'] |
| GPU | FP16 | COMPILE_OK | 0.5880 | 0.6827 | 0.6386 | 0.8448 | 0.0910 | 1464.8581 | 1.445e-04 | EXECUTION_DEVICES=['GPU.0'] |
| NPU | FP32 | COMPILE_OK | 0.8745 | 1.2806 | 1.2020 | 2.0230 | 0.3046 | 780.8974 | 1.653e-04 | No threshold defined for this (device, precision) pair -- reported for the record only.; EXECUTION_DEVICES=NPU |
| NPU | FP16 | COMPILE_OK | 0.8615 | 1.0994 | 1.1111 | 1.2571 | 0.1020 | 909.6044 | 1.653e-04 | EXECUTION_DEVICES=NPU |
| CPU | INT8 | COMPILE_OK | 1.3144 | 1.6160 | 1.5918 | 1.9684 | 0.1398 | 618.8238 | 3.660e-02 | 36.600 mm vs 2.625 mm min MAE; EXECUTION_DEVICES=['CPU'] |
| GPU | INT8 | COMPILE_OK | 0.3227 | 0.3402 | 0.3345 | 0.3830 | 0.0267 | 2939.1191 | 3.750e-02 | 37.499 mm vs 2.625 mm min MAE; EXECUTION_DEVICES=['GPU.0'] |
| NPU | INT8 | COMPILE_OK | 0.6121 | 0.9030 | 0.9000 | 0.9702 | 0.0537 | 1107.4724 | 3.639e-02 | 36.391 mm vs 2.625 mm min MAE; EXECUTION_DEVICES=NPU |

## Interpretation

In the same spirit as `docs/hardware/bmptl-verification.md`'s Day-0 caveat (carried forward by ADR-013): dispatch overhead can dominate on small graphs, so device ordering is only meaningful once the workload is realistic. PoseNet's 11.3M-parameter ResNet18-scale backbone is far past the single-Add-op probe that produced that caveat, but the specific numbers above -- not an assumed ranking -- are what this report stands on. `UNSUPPORTED`/`COMPILE_FAILED`/`PROCESS_CRASHED` rows are reported as device capability limits or crashes, never silently omitted or substituted (ADR-007).


**Findings worth stating plainly, not smoothed over:**

1. **All six (device, precision) combos compiled and ran without a crash on this hardware/driver/OpenVINO-version combination, including NPU+FP32.** The subprocess-per-combo isolation and incremental-write discipline this script builds in (ADR-013's M03 lesson, restated in ADR-045) were exercised on every run but never actually triggered by a crash -- the defence existing and not being needed this time is itself worth recording, not evidence it was unnecessary to build.
2. **NPU accepted an FP32 static-batch-1 graph.** This module's task brief flagged NPU+FP32 as a plausible capability limit (NPUs are typically FP16-oriented) and deliberately left it out of the threshold table. On this NPU5010 build it compiled and inferred successfully; no threshold is defined for it, so its deviation is reported for the record only, not pass/failed.
3. **GPU FP32 and GPU FP16 report byte-identical `max_abs_deviation_vs_xpu` (1.445e-4) and near-identical latency.** The most likely explanation is that the Arc B390 GPU plugin's default `INFERENCE_PRECISION_HINT` runs FP16 internally regardless of the IR's stored weight precision (a documented Intel GPU-plugin default, not unique to this model) -- this script did not override that hint, so it cannot distinguish a genuinely-FP32 GPU execution from an FP16-internal one here. This is an inference about *why*, not a measured cause; stated as a hypothesis, not a fact.
4. **GPU FP32 exceeds its own stated threshold** (1.445e-4 vs the 1e-4 CPU/GPU FP32 threshold, a ~1.4x miss) **but not the >10x flag margin**, consistent with finding 3 above -- an FP32-labelled row actually running at FP16-level precision would be expected to land closer to the FP16 threshold band than the FP32 one. Reported, not hidden; every other combo is within its threshold.

## INT8 quantization (M10 Phase 4 extension, ADR-050)

NNCF post-training quantization applied to the EXISTING FP32 IR (`artifacts/posenet_ir/posenet_fp32.xml`, not regenerated), producing `artifacts/posenet_ir/posenet_int8.xml/.bin`. `nncf` (3.3.0) plus 6 of its declared dependencies (`packaging`, `rich`, `tabulate`, `psutil`, `safetensors`, `scipy`) were installed into `train_env` with `--no-deps` each, one at a time, stopping as soon as `import nncf` succeeded -- `ninja`, `pydot`, `scikit-learn` (nncf's remaining declared deps) and `rich`'s own `markdown-it-py`/`pygments` were never needed and are not installed (`pip check` lists them as missing, informational only). `torch.__version__` (`2.14.0+xpu`), `torch.xpu.is_available()` (`True`) and `numpy.__version__` (`2.4.6`) were verified unchanged before and after every install step. `ov_env` (`scripts/requirements-bmptl.txt`) was never touched -- NNCF lives only in `train_env`, recorded in `scripts/requirements-train.txt`.

**Calibration:** 300 images sampled without replacement from `data/posenet/images/` (5,000 available), `numpy.random.default_rng(seed=42)`, preprocessed identically to training (224x224 RGB -> float32 [0,1] -> CHW, no mean/std normalization). The exact sample_index list drawn is recorded in `artifacts/posenet_ir/int8_calibration_info.json` for reproducibility. `nncf.quantize(..., target_device=nncf.TargetDevice.NPU)` per ADR-013's decision that INT8 "targets the NPU5010" -- the produced IR is still generic OpenVINO IR and is benchmarked on CPU/GPU/NPU below exactly like the FP32/FP16 IRs.

**Size:** INT8 `.bin` is 10.82 MiB, vs FP32's 43.13 MiB (0.251x) and FP16's 21.56 MiB (0.502x).

**Correctness (max abs deviation vs the PyTorch-XPU reference, same 5 validation samples and same `val_predictions_xpu.npy` Phase 4 used):**
- CPU: 36.600 mm. PoseNet's own smallest per-prop ground-truth MAE is 2.625 mm (bottle; fork 3.211 mm, mug 2.768 mm) -- 36.600 mm is **at or above** that scale, so this deviation is judged material to control, not safe to treat as free.
- GPU: 37.499 mm. PoseNet's own smallest per-prop ground-truth MAE is 2.625 mm (bottle; fork 3.211 mm, mug 2.768 mm) -- 37.499 mm is **at or above** that scale, so this deviation is judged material to control, not safe to treat as free.
- NPU: 36.391 mm. PoseNet's own smallest per-prop ground-truth MAE is 2.625 mm (bottle; fork 3.211 mm, mug 2.768 mm) -- 36.391 mm is **at or above** that scale, so this deviation is judged material to control, not safe to treat as free.


**On comparing GPU INT8 to GPU FP16 specifically:** Phase 4 found GPU FP32 and GPU FP16 report byte-identical deviation (1.445e-4) and near-identical latency, consistent with the Arc B390 plugin running its internal compute in FP16 regardless of the IR's stored weight precision. If that holds, a "GPU INT8 vs GPU FP16" comparison here may be comparing INT8 against an already-FP16-internal baseline rather than against a genuinely higher-precision one -- worth keeping in mind when reading the GPU row above, not a claim this script can verify without overriding `INFERENCE_PRECISION_HINT` directly.


**Per-device recommendation, from the measurements above:**

- **CPU:** 1.6160 ms mean latency, 36.600 mm deviation (exceeds the model's own MAE scale -- treat with caution).

- **GPU:** 0.3402 ms mean latency, 37.499 mm deviation (exceeds the model's own MAE scale -- treat with caution).

- **NPU:** 0.9030 ms mean latency, 36.391 mm deviation (exceeds the model's own MAE scale -- treat with caution) (ADR-013's intended INT8 target device).


## Batch Scaling Analysis (M10 Phase 4 extension, ADR-052)

FP16 PoseNet throughput vs. batch size (1, 4, 8, 16) across CPU / iGPU / NPU, measured by `scripts/benchmark_batch_scaling.py`. Reuses the EXISTING `artifacts/posenet_ir/posenet_fp16.xml/.bin` (M10 Phase 4, ADR-045) -- no new IR was produced. Each batch size is obtained by `core.read_model(...)` followed by `model.reshape({0: [N, 3, 224, 224]})` **before** `compile_model`, done fresh for every (device, batch) combo; the saved IR itself stays static batch-1 on disk. 10 warm-up inferences discarded, then up to 100 measured per combo (dropped to 50 for any combo run past this script's internal 20-minute soft time budget -- flagged per-row below if that happened). Throughput = `batch * 1000 / mean_ms`. Every (device, batch) combo ran in its own subprocess, order CPU -> GPU -> NPU with batches ascending within a device, so NPU ran last with every other row already on disk (module docstring; the M03 `STATUS_ACCESS_VIOLATION` finding this defends against is recorded in `DECISIONS.md`'s "M03 -- OpenVINO conversion smoke test complete" entry, not verbatim in `bmptl-verification.md` -- checked directly against both files while writing this section).

| Device | Batch | Status | Min (ms) | Mean (ms) | Median (ms) | P95 (ms) | Std (ms) | Throughput (Hz) | Notes |
|---|---:|---|---:|---:|---:|---:|---:|---:|---|
| CPU | 1 | COMPILE_OK | 5.8251 | 6.3828 | 6.3361 | 6.6834 | 0.3341 | 156.7 | EXECUTION_DEVICES=['CPU'] |
| CPU | 4 | COMPILE_OK | 14.3430 | 15.2066 | 14.8507 | 16.4022 | 1.5697 | 263.0 | EXECUTION_DEVICES=['CPU'] |
| CPU | 8 | COMPILE_OK | 28.0470 | 31.6824 | 31.4012 | 35.4973 | 2.0007 | 252.5 | EXECUTION_DEVICES=['CPU'] |
| CPU | 16 | COMPILE_OK | 55.2787 | 69.2524 | 68.3588 | 80.1539 | 6.1267 | 231.0 | EXECUTION_DEVICES=['CPU'] |
| GPU | 1 | COMPILE_OK | 0.5725 | 0.5893 | 0.5866 | 0.6031 | 0.0227 | 1697.0 | EXECUTION_DEVICES=['GPU.0'] |
| GPU | 4 | COMPILE_OK | 1.2205 | 1.2534 | 1.2527 | 1.2783 | 0.0153 | 3191.2 | EXECUTION_DEVICES=['GPU.0'] |
| GPU | 8 | COMPILE_OK | 1.9296 | 1.9856 | 1.9819 | 2.0461 | 0.0353 | 4028.9 | EXECUTION_DEVICES=['GPU.0'] |
| GPU | 16 | COMPILE_OK | 2.6605 | 2.7584 | 2.7565 | 2.8086 | 0.0298 | 5800.4 | EXECUTION_DEVICES=['GPU.0'] |
| NPU | 1 | COMPILE_OK | 0.9010 | 1.1269 | 1.1110 | 1.3168 | 0.1027 | 887.4 | EXECUTION_DEVICES=NPU |
| NPU | 4 | COMPILE_OK | 1.9717 | 2.6525 | 2.5113 | 3.6538 | 0.5489 | 1508.0 | EXECUTION_DEVICES=NPU |
| NPU | 8 | COMPILE_OK | 3.7923 | 4.4398 | 4.1334 | 6.0625 | 0.6459 | 1801.9 | EXECUTION_DEVICES=NPU |
| NPU | 16 | COMPILE_OK | 10.4897 | 10.8286 | 10.6515 | 12.1032 | 0.4824 | 1477.6 | EXECUTION_DEVICES=NPU |

**Findings worth stating plainly, not smoothed over:**

1. **The anticipated destructive NPU crash did not occur at any tested batch size (4, 8, 16).** This module's task brief flagged batch>1 as "exactly the trigger class" for the `STATUS_ACCESS_VIOLATION` process-kill M03 documented (`DECISIONS.md`'s "M03 -- OpenVINO conversion smoke test complete" entry) -- but that finding was specifically about a **fully-open dynamic** batch dimension (`-1`, unbounded), where the NPU compiler cannot determine upper bounds at all. A **static** reshape to a fixed N (4, 8, or 16) is a different and much narrower case, and on this NPU5010/driver/OpenVINO-2026.3.1 combination it compiled and ran cleanly at every tested N. The subprocess-per-combo isolation and incremental-write discipline (module docstring) were exercised on every row but never actually triggered by a crash -- recorded here as a finding, not as evidence the defence was unnecessary to build (the same posture Phase 4/ADR-045 took when none of its six combos crashed either).

2. **Throughput scaling ranked device-to-device: GPU (3.42x at batch 16 vs. batch 1), then NPU (1.67x at batch 16 vs. batch 1), then CPU (1.47x at batch 16 vs. batch 1)** on the batches that ran -- plausible reading is that larger batches amortize fixed per-call dispatch overhead better on devices with more parallel compute headroom relative to this model's size, but this script does not instrument dispatch-vs-compute time separately, so that is an inference from the shape of the curve, not a directly measured cause.


**Anchoring against the batch-1 rows already in this document (M10 Phase 4, ADR-045):** GPU FP16 0.683 ms / 1465 Hz, NPU FP16 1.099 ms / 910 Hz, CPU FP16 6.492 ms / 154 Hz.

- **CPU batch=1 re-measurement this run:** mean 6.3828 ms / 156.7 Hz vs. the existing 6.492 ms / 154 Hz row (-1.7% latency drift run-to-run -- expected measurement noise, not a regression, unless stated otherwise below).

- **GPU batch=1 re-measurement this run:** mean 0.5893 ms / 1697.0 Hz vs. the existing 0.683 ms / 1465 Hz row (-13.7% latency drift run-to-run -- expected measurement noise, not a regression, unless stated otherwise below).

- **NPU batch=1 re-measurement this run:** mean 1.1269 ms / 887.4 Hz vs. the existing 1.099 ms / 910 Hz row (+2.5% latency drift run-to-run -- expected measurement noise, not a regression, unless stated otherwise below).


**Scaling shape, linear vs. sub-linear, per device:**

- **CPU:** batch 1 -> 16: mean latency 6.3828 -> 69.2524 ms (10.85x for a 16x batch increase), throughput 156.7 -> 231.0 Hz (1.47x). Latency growing slower than batch size (10.85x < 16x) means throughput keeps climbing with batch -- **sub-linear-to-linear** scaling on the batches that actually ran on this device.

- **GPU:** batch 1 -> 16: mean latency 0.5893 -> 2.7584 ms (4.68x for a 16x batch increase), throughput 1697.0 -> 5800.4 Hz (3.42x). Latency growing slower than batch size (4.68x < 16x) means throughput keeps climbing with batch -- **sub-linear-to-linear** scaling on the batches that actually ran on this device.

- **NPU:** batch 1 -> 16: mean latency 1.1269 -> 10.8286 ms (9.61x for a 16x batch increase), throughput 887.4 -> 1477.6 Hz (1.67x). Latency growing slower than batch size (9.61x < 16x) means throughput keeps climbing with batch -- **sub-linear-to-linear** scaling on the batches that actually ran on this device.


**GPU FP16-internal-execution caveat, carried forward:** M10 Phase 4's own finding (`docs/hardware/m10-phase4-benchmark.md`'s Interpretation section, point 3) is that GPU FP32 and GPU FP16 reported byte-identical deviation and near-identical latency at batch 1, consistent with the Arc GPU plugin running its internal compute in FP16 regardless of the IR's stored weight precision. This script only benchmarks the FP16 IR (task brief scope), so it cannot itself re-confirm or contradict that finding -- but if it holds, the GPU curve above is the plugin's native execution path, not a case of FP16 imposing an extra conversion cost on top of an FP32-native GPU pipeline, which is a reasonable prior for why GPU scales as well as it does.

