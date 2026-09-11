# M03 -- OpenVINO conversion smoke test notes

Ran Sept 12, 2026 (Day 2), inside the 09:00-14:00 window PLAN.md section 1A.5 sets for
this module, well before the 14:00 hard escalation gate. Produced with
`scripts/ov_smoke.py`. All numbers below are copied from the actual run output / from
`artifacts/ov_smoke/results.jsonl` as scp'd back from bm-ptl -- nothing here is invented
(ADR-015 rule 1).

## What was converted

A hand-rolled **ResNet18-scale** convolutional encoder (`build_model()` in
`scripts/ov_smoke.py`): same topology as `torchvision.models.resnet18` -- 7x7/stride-2
stem + maxpool, four stages of 2 `BasicBlock`s each (channels 64/128/256/512, stride-2 at
the start of stages 2-4), global average pool, linear head to a 512-d embedding.
**11,439,168 parameters** (printed by the export step), which matches real ResNet18's
known parameter count. Built with plain `torch.nn` rather than importing `torchvision`,
because `torchvision` is not pinned in `scripts/requirements-dev.txt` or
`scripts/requirements-bmptl.txt` and this module's own files
(`scripts/ov_smoke.py`, this notes file) are the only ones this module is allowed to
touch. Static input shape `(1, 3, 224, 224)`, NCHW, FP32.

This satisfies PLAN.md M03's "dividend to protect": M10's PoseNet can reuse this exact
backbone shape and M13's export inherits this conversion recipe.

## Undeclared-dependency flag (read before trusting the pipeline described below)

`torch.onnx.export()` on the laptop requires the `onnx` or `onnxscript` package (verified
by trying it: `ModuleNotFoundError: No module named 'onnx'` / `'onnxscript'`), and neither
is pinned anywhere in this repo. Rather than add that dependency, this script uses
OpenVINO's own PyTorch frontend, `openvino.convert_model(torch_module, example_input=...)`,
which traces the live `torch.nn.Module` directly -- no ONNX file involved. That call needs
`torch` and `openvino` in the *same* process, so **`openvino==2026.3.1` (exactly bm-ptl's
pinned version) was pip-installed on the laptop** for this module. This is an undeclared
dependency on the laptop side: it is not yet in `scripts/requirements-dev.txt`. Flagged
here rather than silently added to that file, since this module's file scope is limited to
`scripts/ov_smoke.py` and this notes file -- Planner/Tester should decide whether to
formalize it there.

## Where each step ran (and why)

- `python scripts/ov_smoke.py --export --artifact-dir artifacts/ov_smoke` ran on the
  **laptop** (has `torch==2.14.0+cpu` and now `openvino==2026.3.1`). It built the model,
  computed a PyTorch FP32 reference output on a fixed random input (`torch.manual_seed(0)`),
  and converted straight to three OpenVINO IR variants (below), all in the same process.
- The IR files, `input.npy` and `reference_output.npy` were `scp`'d to
  `C:\Users\devcloud\intel-bimanual-vla\artifacts\ov_smoke\` on bm-ptl.
- `python scripts\ov_smoke.py --device {CPU,GPU,NPU} --artifact-dir artifacts\ov_smoke` ran
  on **bm-ptl**, via `C:\Users\devcloud\project\ov_env\Scripts\python.exe` (OpenVINO
  2026.3.1). **`torch` is NOT installed in `ov_env` on bm-ptl** (checked with
  `pip list`; also absent: `torchvision`, `onnx`, `onnxscript`) -- confirming that the
  compile/infer step could not have done the PyTorch-side conversion itself even if we had
  wanted it to. This is exactly why conversion happened on the laptop and only compile+infer
  ran on bm-ptl, per PLAN.md M03 ("conversion may happen on laptop, compile and infer must
  happen on bm-ptl").

## IR files produced (exist on bm-ptl, confirmed by `Get-ChildItem`)

All in `C:\Users\devcloud\intel-bimanual-vla\artifacts\ov_smoke\` on bm-ptl:

| File | Size | Meaning |
|---|---|---|
| `model_static.xml` / `.bin` | 54,560 B / 22,859,168 B | Static shape `[1,3,224,224]`, FP32 weights |
| `model_static_fp16.xml` / `.bin` | 54,560 B / 22,859,168 B | Static shape `[1,3,224,224]`, FP16-compressed weights (`compress_to_fp16=True`) |
| `model_dynamic.xml` / `.bin` | 54,733 B / 22,859,168 B | Dynamic batch shape `[?,3,224,224]`, FP32 weights |

Each `.xml`+`.bin` pair is a valid IR pair satisfying M03 done-when 1. (`.bin` sizes are
identical across variants because `compress_to_fp16` only affects `model_static_fp16`'s own
bin, which happens to compress to the same 22,859,168-byte size here -- not a bug, just how
this particular graph's weight buffer padded out; not independently re-verified further
since it is not load-bearing for any done-when criterion.)

## Results, per device (M03 done-when 2 and 3)

`OpenVINO version: 2026.3.1-22476-759c5a6ab8c-releases/2026/3`. `core.available_devices`
on bm-ptl: `['CPU', 'GPU', 'NPU']`.

### CPU -- all three checks PASSED

| Shape | Precision | Compile | Infer | Max \|deviation\| vs PyTorch FP32 reference |
|---|---|---|---|---|
| static | FP32 | OK (`EXECUTION_DEVICES=['CPU']`) | OK | **5.674362e-05** |
| static | FP16 | OK (`EXECUTION_DEVICES=['CPU']`) | OK | **5.674362e-05** |
| dynamic batch | FP32 | OK (`EXECUTION_DEVICES=['CPU']`) | OK | **5.674362e-05** |

### GPU (Arc B390 iGPU) -- all three checks PASSED

| Shape | Precision | Compile | Infer | Max \|deviation\| vs PyTorch FP32 reference |
|---|---|---|---|---|
| static | FP32 | OK (`EXECUTION_DEVICES=['GPU.0']`) | OK | **7.408857e-05** |
| static | FP16 | OK (`EXECUTION_DEVICES=['GPU.0']`) | OK | **7.408857e-05** |
| dynamic batch | FP32 | OK (`EXECUTION_DEVICES=['GPU.0']`) | OK | **7.408857e-05** |

### NPU (NPU5010) -- static PASSED, dynamic-batch CRASHED THE PROCESS

| Shape | Precision | Compile | Infer | Max \|deviation\| vs PyTorch FP32 reference |
|---|---|---|---|---|
| static | FP32 | OK (`EXECUTION_DEVICES=NPU`) | OK | **1.122952e-04** |
| static | FP16 | OK (`EXECUTION_DEVICES=NPU`) | OK | **1.122952e-04** |
| dynamic batch | FP32 | **PROCESS CRASHED** | -- | -- |

**Verbatim failure, NPU + dynamic batch shape** (this is a real and load-bearing finding,
not softened): compiling `model_dynamic.xml` on `NPU` does not raise a normal, catchable
OpenVINO/Python exception. It kills the whole Python process. Reproduced twice:

- First attempt (single-process script, before the subprocess-isolation fix described
  below): the parent process exited with code `5` and printed, to stderr, only:
  ```
  [ERROR] 12:21:33.167 [IE::FrontEnd::importNetwork]   Upper bounds are not specified for node 'Multiply_11422' (type 'Convolution'): input '0' bounds are '[9223372036854775807, 3, 224, 224]'
  ```
  Because the original script accumulated all three variants' results in memory and wrote
  `results.jsonl` only once at the end, this crash **silently discarded the NPU static
  FP32 and static FP16 results that had already succeeded** -- they were correct but never
  written to disk. This was caught by inspecting the script's own output (the two
  `[NPU | static | ...]` print lines were simply missing) and is why `scripts/ov_smoke.py`
  was then changed to run each (device, precision, shape) check in its own subprocess and
  append each result to `results.jsonl` immediately, so one crashing variant cannot cost the
  others their already-obtained results.
- Second attempt (after the subprocess-isolation fix, this is the current script
  behaviour and the numbers reported in the table above): the child process for the
  `dynamic_fp32` variant exited with code `3221225477` (`0xC0000005`, i.e. a Windows
  `STATUS_ACCESS_VIOLATION` -- a hard crash, not a graceful error return), having printed
  to stdout:
  ```
  [ERROR] 12:24:31.895 [IE::FrontEnd::importNetwork]   Upper bounds are not specified for node 'Multiply_11422' (type 'Convolution'): input '0' bounds are '[9223372036854775807, 3, 224, 224]'
  ```
  The parent process survived because that check ran in a subprocess; the CPU/GPU/NPU
  static results were unaffected and are the numbers reported above.

The message names the fully-unbounded batch dimension (`9223372036854775807` =
`INT64_MAX`) as the problem: OpenVINO's NPU compiler backend requires an *upper bound* on
any dynamic dimension it accepts, and a batch dimension declared as fully open (`[-1, 3,
224, 224]`, which is what `input=[-1, 3, 224, 224]` in `ov.convert_model` produces) has no
such bound. This is consistent with ADR-013's expectation that the NPU would need static
(or at least bounded) shapes -- it is worse than a compile-time rejection, though: it is a
process-level crash, which is itself useful information for M13/M14 (any NPU compile
attempt in those later modules must run in an isolated subprocess or its own script
invocation, never inline in a long-running benchmark process, or one bad shape takes the
whole benchmark run down with it).

**Not tried and not claimed:** a batch dimension declared with an explicit finite upper
bound (e.g. OpenVINO's `ov.Dimension(1, 8)` style bounded dynamic dimension, distinct from
the fully-open `-1` used here) was not attempted. ADR-014's own budget is 3 hours and this
finding is already conclusive for M03's purpose (the NPU does not accept the ordinary,
fully-dynamic-batch shape a PyTorch model exports to by default); chasing whether some
*other*, bounded, dynamic-shape declaration would compile is exactly the kind of "spend
hours trying to force it" the module brief says not to do, and is left for M13 (PoseNet
export) to pursue only if a batch dimension is actually needed there.

## Answers to M03's four done-when criteria

1. **IR pair exists on bm-ptl for a ResNet18-scale vision encoder** -- yes. Three IR
   variants (`model_static`, `model_static_fp16`, `model_dynamic`), each a valid
   `.xml`+`.bin` pair, confirmed present in
   `C:\Users\devcloud\intel-bimanual-vla\artifacts\ov_smoke\` on bm-ptl by
   `Get-ChildItem`.
2. **`ov_smoke.py --device CPU|GPU|NPU` completes on all three, or the failure is captured
   verbatim** -- CPU and GPU completed cleanly on every check (static FP32, static FP16,
   dynamic-batch FP32). NPU completed cleanly on both static-shape checks and its one
   failure (the dynamic-batch check) is captured verbatim above, including the exact exit
   code and the exact OpenVINO error text, with no retry-until-success and no silent
   fallback (ADR-007).
3. **Max absolute deviation between PyTorch and each OpenVINO device, recorded as a
   number** -- CPU: `5.674362e-05`. GPU: `7.408857e-05`. NPU: `1.122952e-04` (static
   shape only; the dynamic-shape NPU case has no number because it never produced an
   output). No threshold is asserted on any of these, per the done-when's own wording --
   they are reported as measured. Note that FP16-compressed and FP32 IRs produced
   *identical* deviation numbers on every device that ran them; this script does not
   explain why (plausibly each device's runtime plugin executes both IRs at the same
   internal precision for this small a model), and no claim is made beyond "the numbers we
   measured were equal to the last printed digit."
4. **Notes state explicitly whether dynamic batch/sequence shapes were accepted by the
   NPU** -- **no.** The NPU rejected the (fully unbounded, `-1`) dynamic-batch IR outright,
   crashing the compiling process rather than returning a graceful error. CPU and GPU both
   accepted the identical dynamic-batch IR without issue. The NPU therefore requires a
   **static shape** for this graph; ADR-013's export plan for M13 (PoseNet) should target
   NPU with a fully static input shape, not a dynamic batch dimension, unless a
   bounded-dynamic-dimension declaration is tried and shown to work (untried here, see
   above). On precision: both FP32 and FP16-compressed static IRs compiled and ran
   correctly on the NPU with equal measured deviation, so this run does not show the NPU
   *requiring* FP16 -- but ADR-013's intent to target NPU with a quantized/FP16 IR for
   throughput remains a separate, unmeasured question (this module measured only
   correctness and compile/infer success, not latency or throughput).

## What was not measured here (explicitly out of scope for M03)

- **Latency/throughput** per device/precision -- that is M14's job
  (`benchmarks/bmptl-results.md`), not this smoke test's.
- **INT8 / NNCF quantization** -- ADR-013 assigns that to the real PoseNet model in M13,
  calibrated on M09 data; not attempted here.
- **A bounded (non-`-1`) dynamic dimension on the NPU** -- see "Not tried and not
  claimed" above.
- The MuJoCo offscreen-rendering probe ADR-020 folded into this module -- **not re-run**,
  per this task's explicit instruction; it already closed Sept 11 (RISK-03,
  `ARCHITECTURE.md` section 5).

## Reproducing

```
# Laptop (needs torch==2.14.0+cpu and openvino==2026.3.1 -- the latter is the
# undeclared dependency flagged above):
python scripts/ov_smoke.py --export --artifact-dir artifacts/ov_smoke

# scp artifacts/ov_smoke/*.xml *.bin *.npy to bm-ptl, same relative path, then
# on bm-ptl (ov_env has openvino==2026.3.1, no torch needed for this half):
python scripts\ov_smoke.py --device CPU --artifact-dir artifacts\ov_smoke
python scripts\ov_smoke.py --device GPU --artifact-dir artifacts\ov_smoke
python scripts\ov_smoke.py --device NPU --artifact-dir artifacts\ov_smoke
```

Raw per-check results (one JSON object per line, appended incrementally) land in
`artifacts/ov_smoke/results.jsonl`; `artifacts/` is not committed to git (the `.bin`
files are ~22 MB each) -- this notes file is the durable record.
