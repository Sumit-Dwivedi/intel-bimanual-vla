# Setup — reproducing the demo

This is the step-by-step reproduction path for `./run_demo.sh` /
`run_demo.py`. `README.md`'s quickstart and `CONTRIBUTING.md`'s "Reproducing
the demo" section both point here for the detail they don't repeat.

## 0. Read this first: which machine can actually run this

**MuJoCo does not import on the authoring laptop under Windows Smart App
Control** (`ARCHITECTURE.md` ADR-020): Smart App Control blocks the
unsigned `mujoco.dll`, and `scripts/verify_env.py` documents this exact
failure signature (`OSError` mentioning `4551` / "Application Control") as
an *expected*, non-fatal state on that machine — it is not something this
setup guide can work around, and it is not a bug in this repo.

**All MuJoCo simulation work — the four scripted skills, `pytest
tests/test_skills.py`, and this demo — runs on bm-ptl (Intel Core Ultra 7
358H / Panther Lake, `CONSTRAINTS.md:8-11`), not the laptop.** The laptop is
used only for code editing, non-MuJoCo tooling, and `scripts/verify_env.py`
(which deliberately never imports `mujoco` to check its version — see that
script's own docstring).

**Numbers differ between machines, and bm-ptl's are authoritative.**
ADR-047 documents a measured cross-machine floating-point divergence: the
exact same pinned `mujoco==3.2.7`, the exact same code, run on the laptop
vs. bm-ptl, reproduces identical PASS/FAIL *structure* but different digits
(e.g. `handoff`'s lateral separation: 0.1958 m on the laptop vs. 0.1946 m on
bm-ptl) — almost certainly contact-solver iteration-order drift compounding
over the ~1000–6600 physics steps each skill takes, not a bug in either
environment. This module's own bm-ptl run reproduced that exact 0.1946 m
figure; a laptop run of the same command will show 0.1958 m instead and
that is expected, not a regression. **Every number in `SUBMISSION.md`,
`docs/hardware/`, and this demo's own reported results is bm-ptl's, and any
verification should be done there.**

## 1. Clone

```bash
git clone https://github.com/Sumit-Dwivedi/intel-bimanual-vla.git
cd intel-bimanual-vla
```

## 2. Create `ov_env` (required — this is the venv the demo runs in)

`ov_env` holds the load-bearing `mujoco==3.2.7` + `openvino==2026.3.1`
pairing every verified skill and this demo depend on
(`scripts/requirements-bmptl.txt`, ADR-037). Create it as its own venv —
do not install these pins into a venv shared with anything else.

```bash
python -m venv ov_env
# Windows (bm-ptl): ov_env\Scripts\activate
# Linux/macOS:      source ov_env/bin/activate
pip install -r scripts/requirements-bmptl.txt
```

On bm-ptl specifically, the already-provisioned interpreter is:

```
C:\Users\devcloud\project\ov_env\Scripts\python.exe
```

## 3. (Optional) Create `train_env` — only needed for OpenVINO/PoseNet
   benchmarks and quantization, NOT for this demo

`train_env` is a **separate** venv (`scripts/requirements-train.txt`) that
adds `torch` (an Arc-iGPU-enabled XPU build), plus `openvino` and `nncf` for
the PoseNet export/quantization pipeline
(`scripts/train_posenet.py`, `scripts/posenet_to_openvino.py`,
`scripts/quantize_posenet.py`). `torch` deliberately lives **only** here,
never in `ov_env` — see that requirements file's own header comment for why
(ADR-037 already recovered once from an unscoped install silently upgrading
`ov_env`'s pinned `mujoco`).

```bash
python -m venv train_env
# activate it, same as step 2
pip install -r scripts/requirements-train.txt --extra-index-url https://download.pytorch.org/whl/xpu
```

**`run_demo.py`'s four scripted skills do not need `train_env` at all** —
they run in oracle mode (ADR-046's default), which never calls PoseNet or
OpenVINO. `run_demo.py` reports `torch`'s absence as an informational
warning, not an error, for exactly this reason.

## 4. Verify the install

```bash
python scripts/verify_env.py --requirements scripts/requirements-bmptl.txt
```

This checks every pinned package's *installed* version against the pin via
package metadata only (never importing `mujoco` to do it — see the
script's own docstring), then separately reports whether each package is
actually importable on this host, classifying the documented ADR-020
laptop signature as expected rather than fatal. Run it with
`scripts/requirements-dev.txt` instead if you're checking the laptop-side
(non-MuJoCo) tooling.

## 5. Run the demo

```bash
./run_demo.sh
# or, identically, on any OS:
python run_demo.py
```

`run_demo.sh` is a thin wrapper (`exec`s a `python`/`python3` it finds on
`PATH`) around the real, portable logic in `run_demo.py` — written this way
because bm-ptl is Windows with no native bash to run a `.sh` file directly,
but `README.md`/`CONTRIBUTING.md` already reference `./run_demo.sh`, so
that filename has to exist. Activate `ov_env` first (step 2) so the
`python`/`python3` the wrapper finds is the right one.

`run_demo.py`:
1. Checks the environment (python/mujoco/openvino/torch versions) — fails
   early with a clear message if `mujoco` is missing (the one hard
   requirement); `openvino`/`torch` absence is reported as a warning only.
2. Checks required scene assets and reports whether the (gitignored,
   bm-ptl-only) PoseNet OpenVINO IR is present — its absence is expected on
   a fresh clone and does not block the demo (ADR-046: oracle is the
   default; this demo never opts into perception).
3. Prints the four-skill sequence it is about to run, honestly labelled:
   `pick(A, fork)`, `place(A, fork, table)` and `pick(A, bottle)` genuinely
   vary their target prop's placement by `--seed` (drawn from that skill's
   own measured envelope, ADR-048/ADR-049 Track A); `handoff` always runs
   at a fixed default layout and says so, because its own measured envelope
   is a single point (ADR-048) — see `run_demo.py`'s own docstring for why
   a seed label is deliberately withheld there.
4. Executes all four in oracle mode (no perception, no OpenVINO call) and
   prints one `[PASS]`/`[FAIL]` line per skill plus a final summary.
5. Exits 0 only if all four skills passed.

`--seed` defaults to 3 (one of the seeds where `pick(A, bottle)`'s own
measured Track A envelope passes — seeds 0, 1, 2 and 4 are documented
failures for that skill alone, `docs/hardware/m08-eval.md`); pass
`--seed 0` to see that documented failure mode directly instead of a
curated all-pass run.

## 6. Run the regression test suite

```bash
pytest tests/test_skills.py -v
```

As of this writing this reports **4 passed / 4 failed** — that is the
documented, expected baseline (`CONTRIBUTING.md`), not a broken install.
The four failures are pre-existing, already-diagnosed kinematic/grasp-
reliability gaps (see `tests/test_skills.py`'s own module docstring), not
regressions from this demo script.

## 7. Where the authoritative numbers live

- `docs/hardware/m10-phase4-benchmark.md` — OpenVINO CPU/iGPU/NPU ×
  FP32/FP16/INT8 latency and throughput.
- `docs/hardware/m08-eval.md` — the 10-seed robustness evaluation
  (Track A own-prop / Track B multi-prop), all measured on bm-ptl.
- `SUBMISSION.md` — the unembellished, checkbox-level status of every
  required deliverable.
