# M10 Phase 2 — PoseNet architecture, dataset loader, training script: smoke test

**Recorded:** Sept 14, 2026 · **Machine:** bm-ptl (Intel Core Ultra X7 358H /
Intel Arc B390 iGPU / Intel AI Boost NPU) · **Venv:** `C:\Users\devcloud\project\train_env`
(NEW, separate from `ov_env`) · **Repo checkout used:** `C:\Users\devcloud\intel-bimanual-vla`
at HEAD `d1157e1`.

This module (PLAN.md M10, builder task "M10 Phase 2") was **smoke-tested only**, per
explicit instruction: model instantiation + one forward pass, one real dataset sample,
one training batch on the working device. **No full training run was performed.**

---

## 1. Why a new venv, and why not `ov_env`

The brief's environment assumptions (checked, not assumed — see the task instructions)
were verified wrong for this repo's actual state:

| | torch | PIL | IPEX | dataset |
|---|---|---|---|---|
| bm-ptl `ov_env` | MISSING (confirmed below) | MISSING (confirmed below) | not installed | 5000 samples present |
| laptop (`requirements-dev.txt`) | 2.14.0+cpu | MISSING | not installed | not present (gitignored) |

`ov_env` holds the load-bearing `mujoco==3.2.7` + `openvino==2026.3.1` pairing
(ADR-037) that four verified skills and `scripts/generate_posenet_data.py` depend on.
Installing torch there risked exactly the kind of silent transitive dependency bump
ADR-037 already had to recover from once. Per instruction, a **new, separate venv**,
`C:\Users\devcloud\project\train_env`, was created instead, containing only
torch + Pillow + numpy (no mujoco, no openvino). `ov_env` was **not touched** at any
point — see section 5 for the verification that ran against it at the end.

## 2. Device path: native XPU worked on the first attempt

Per instruction, the order tried was: (1) native `torch.xpu` build, (2) IPEX only if
(1) failed, (3) CPU. Only step (1) was needed:

```
pip install torch --index-url https://download.pytorch.org/whl/xpu
```

Result (`C:\Users\devcloud\project\train_env`, verified by direct interrogation, not
assumed):

```
torch.__version__            == "2.14.0+xpu"
hasattr(torch, "xpu")        == True
torch.xpu.is_available()     == True
torch.xpu.device_count()     == 1
torch.xpu.get_device_name(0) == "Intel(R) Arc(TM) B390 GPU"
torch.cuda.is_available()    == False
```

**IPEX (`intel-extension-for-pytorch`) was never installed and its install path was
never exercised** — step (1) succeeded outright, so step (2) was skipped per the
brief's own "IPEX only if native XPU is unavailable" instruction. There is no IPEX
error text to report because IPEX was never attempted; native XPU is the path that
worked, first try, no fallback needed.

The full `pip install` transcript (`train_env_setup_log.txt` on bm-ptl) shows the xpu
wheel pulling in Intel's oneAPI/SYCL/MKL runtime packages automatically as transitive
dependencies (`intel-sycl-rt`, `intel-opencl-rt`, `mkl`, `onemkl-sycl-*`, `triton-xpu`,
`dpcpp-cpp-rt`, `tbb`, `umf`, etc.) — this is the mechanism by which a plain
`pip install torch --index-url .../xpu` becomes a fully-working GPU backend with no
separate driver-toolkit install step needed on top of whatever iGPU driver bm-ptl
already had.

`scripts/requirements-train.txt` pins the exact `pip freeze` from this install.

## 3. TASK 1 — `posenet.py`: model instantiation + forward pass

Ran on the laptop first (torch 2.14.0+cpu already present per `requirements-dev.txt`),
then re-ran on bm-ptl on both CPU and XPU:

```
PoseNet parameter count: 11,310,153
Input shape:  (4, 3, 224, 224)
Output shape: (4, 9)
Forward pass OK.
```

11,310,153 parameters — within the "~11-12M expected" range from the task brief.
Backbone is the exact ResNet18-scale topology from `scripts/ov_smoke.py::build_model`
(BasicBlock x [2,2,2,2], channels 64/128/256/512), re-derived (not imported, since
`scripts/` is not a Python package) with an added global-avg-pool + Linear(512,256) +
ReLU + Linear(256,9) head, per the task brief.

On bm-ptl, `PoseNet().to(torch.device("xpu"))` forward-passed a dummy batch
successfully:

```
param_count 11310153
input_shape (4, 3, 224, 224)
output_shape (4, 9)
output_device xpu:0
XPU forward pass OK
```

## 4. TASK 2 — `dataset.py`: real sample load

Against the real, committed 5000-sample dataset at
`C:\Users\devcloud\intel-bimanual-vla\data\posenet` (images/labels present on bm-ptl,
gitignored elsewhere per `.gitignore`'s "M10 Phase 1" block):

```
train split: 4500 samples
val split:   500 samples
train[0] image shape=(3, 224, 224) dtype=torch.float32 min=0.027 max=1.000
train[0] labels (xyz x 3 props, metres) = [-0.1706, -0.0619, 0.3560, 0.1490, -0.1306, 0.4400, 0.0005, -0.0220, 0.3900]
train[0] visibility (3 props)           = [0.9709, 1.0, 1.0]
val[0]   image shape=(3, 224, 224) dtype=torch.float32
val[0]   labels    = [0.0946, 0.1515, 0.3560, 0.0484, 0.0347, 0.4400, 0.1139, -0.0802, 0.3900]
val[0]   visibility= [1.0, 1.0, 1.0]
Dataset smoke test OK.
```

4500 + 500 = 5000, matching the "4500 train / 500 val" split from the task brief
exactly (`sample_index % 10 == 0` → val). The z-coordinates (0.356, 0.44, 0.39) match
`dataset_meta.json`'s `resting_z_m` for fork/water_bottle/mug respectively, and the
x/y values fall inside the documented `randomization_range_m` of ±0.18 m — plausible,
not garbage. Visibility ratios in [0, 1] as expected. `val[0]`'s labels match
`data/posenet/labels/sample_00000.json`'s `objects.*.xyz_m` exactly (hand-checked
against the raw JSON).

## 5. TASK 3 — `train_posenet.py`: one-batch smoke test

**`--device xpu --num-workers 0`:**
```
Using device: xpu
train samples: 4500, val samples: 500
PoseNet parameter count: 11,310,153
[SMOKE TEST] device=xpu batch_size=32 output_shape=(32, 9) loss=0.301587 wall_clock=4.28s
[SMOKE TEST] one batch trained and one optimizer step taken. Exiting cleanly.
```

**`--device cpu --num-workers 0`** (fallback-path proof, same seed):
```
Using device: cpu
train samples: 4500, val samples: 500
PoseNet parameter count: 11,310,153
[SMOKE TEST] device=cpu batch_size=32 output_shape=(32, 9) loss=0.301587 wall_clock=0.86s
[SMOKE TEST] one batch trained and one optimizer step taken. Exiting cleanly.
```

The identical loss (0.301587) on both devices, from the same `--seed 0` default, one
untrained randomly-initialized model and one fixed batch, is expected and is itself a
correctness signal: model init, data, and the forward/loss computation agree between
devices.

**`--device xpu --num-workers 4`** (properly guarded — see section 6) also ran to
completion in an earlier pass: wall_clock 9.98s for the full batch (forward + backward
+ optimizer step), loss 0.301587 (same value — same seed, same batch content once
workers deliver it).

No full training loop (multi-epoch) was run, per the module's explicit "SMOKE TEST
ONLY — NO FULL TRAINING" instruction.

## 6. Windows `num_workers` — a real hang was reproduced, but in scaffolding, not in the deliverable

This is the finding this module's task brief specifically asked to record honestly if
it happened, and it happened:

**First smoke run** included an ad hoc, throwaway `DataLoader`-iteration probe script
(`_smoke_workers_probe.py`, generated on the fly, NOT part of this module's deliverable
files) written **without** the `if __name__ == "__main__":` guard. With
`num_workers=4` it hung for the full 300-second subprocess timeout and had to be
killed by that timeout — a direct, live reproduction of the exact Windows-spawn trap
the task brief warned about (spawn re-imports the launching module in each worker
process; without the guard, the top-level `DataLoader(...)` construction and iteration
loop re-executes recursively in every worker).

**Critically, `scripts/train_posenet.py` itself already has the guard**
(`if __name__ == "__main__": main()`, required precisely because its own
`DataLoader(..., num_workers=args.num_workers)` needs it on Windows) — and its
`--num-workers 4` run completed normally (9.98s, reported above). So the hang was
caused by missing the guard in a one-off diagnostic script, not by a defect in the
deliverable.

**Second smoke run**, after adding the guard to the diagnostic probe too, gave a clean,
guarded `num_workers=0` vs `num_workers=4` comparison over 10 batches (320 samples) of
pure iteration (no training):

```
num_workers=0: wall_clock=3.06s, samples_per_sec=104.7
num_workers=4: wall_clock=6.01s, samples_per_sec=53.3
  (per-batch trace shows batch 0 arriving at elapsed=4.60s -- worker spawn/startup
   cost -- then batches 1-9 arriving quickly, 4.62s..5.17s cumulative)
```

**Interpretation:** for a short 10-batch probe, `num_workers=4` is *slower* than
`num_workers=0` because Windows `spawn` pays a one-time ~4.6s process-startup cost
(each worker re-imports torch + the xpu backend) that a 10-batch run cannot amortize.
Over the 4500-sample / ~141-batch-per-epoch training run this script is meant for, that
one-time cost is paid once per training process launch, not per batch, so it should
amortize away — but this was **not measured** here (no full epoch was run, per the
"no full training" instruction), so that amortization claim is reasoning from the
measured per-batch numbers above, not itself a measurement.

**Conclusion, stated as the brief asked:** `num_workers=4` **does work** on this
machine when the entry point is properly guarded (both `train_posenet.py`'s own smoke
test and the corrected probe prove this). It was NOT dropped to 0 as the shipped
default. The one real hang observed was in unguarded ad hoc scaffolding, which is
exactly the failure mode the guard exists to prevent, and both deliverable files
(`train_posenet.py`) already carry the guard.

## 7. `ov_env` byte-unchanged verification

Run at the end, against the real `ov_env` (NOT `train_env`), from
`C:\Users\devcloud\intel-bimanual-vla`:

**`pip list` filtered to `torch`/`pillow`/`mujoco`/`openvino`:**
```
mujoco             3.2.7
openvino           2026.3.1 22476
openvino-telemetry 2025.2.0
```
No `torch`, no `Pillow` — confirms nothing from this module's work leaked into
`ov_env`, and `mujoco`/`openvino` are still exactly their ADR-037-pinned versions.

**`python scripts/verify_adr038_skills.py` (ov_env) — all four skills still PASS:**
```
[1] pick(A, fork):              success=True, z 0.3560 -> 0.3989, weld_attach_frame=1155
[2] place(A, fork, table):      success=True, z -> 0.3588 (table surface 0.35)
[3] pick(A, 'bottle'):          success=True, z 0.4400 -> 0.6192, weld_attach_frame=1155
[4] handoff(A -> B, fork):      success=True, held by B, z 0.3560 -> 0.5498, frames_used=6610
```
These numbers match the ADR-038/ADR-041 baseline already on record — unchanged.

**`pytest tests/test_skills.py` (ov_env):**
```
FFFF....                                                    [100%]
FAILED tests/test_skills.py::test_open_drawer_reaches_near_limit
FAILED tests/test_skills.py::test_pick_plate_lifts_above_table
FAILED tests/test_skills.py::test_place_plate_returns_to_table_rest
FAILED tests/test_skills.py::test_handoff_mug_ends_held_by_arm_b
4 failed, 4 passed in 29.01s
```
Exactly the pre-existing baseline (4 passed / 4 failed, same 4 test names, same
failure reasons — drawer/plate/handoff-mug were never the four verified-working
skills; fork pick/place/pick-bottle/handoff-fork are, and those are what
`verify_adr038_skills.py` checks directly). This module changed nothing about that
baseline.

**Conclusion: `ov_env` is verified functionally byte-unchanged** — same package
versions, same skill behaviour, same test outcome, before and after this module's
work in the separate `train_env`.

## 8. `checkpoints/` gitignore verification

```
$ git check-ignore -v checkpoints/posenet_best.pth checkpoints/posenet_final.pth checkpoints/.gitkeep
.gitignore:12:checkpoints/*.pth	checkpoints/posenet_best.pth
.gitignore:12:checkpoints/*.pth	checkpoints/posenet_final.pth
.gitignore:15:!checkpoints/.gitkeep	checkpoints/.gitkeep
```

Read per the noted format `source:line:pattern<TAB>pathname` (not
`pattern:pathname:source` or any other ordering): `checkpoints/*.pth` (`.gitignore`
line 12) matches both `.pth` paths, so they are ignored — correct, since an 11-12M
parameter checkpoint is ~45 MB and must not enter the repo. `checkpoints/.gitkeep`
matches the negation pattern on line 15 (`!checkpoints/.gitkeep`), so it is **not**
ignored — it was added by this module (previously referenced by `.gitignore` but not
present on disk) so a fresh clone has the `checkpoints/` directory to write into.

## 9. Files touched by this module

- `src/bimanual/perception/posenet.py` (new)
- `src/bimanual/perception/dataset.py` (new)
- `scripts/train_posenet.py` (new)
- `scripts/requirements-train.txt` (new)
- `checkpoints/.gitkeep` (new)
- `docs/hardware/m10-phase2-smoke.md` (this file, new)

No existing file listed in this module's "do not modify" constraints was touched.
`ov_env` was not installed into. `C:\Users\devcloud\project\train_env` is outside the
git repository and is not committed.
