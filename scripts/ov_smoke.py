"""M03 -- Real-model OpenVINO conversion smoke test (PLAN.md M03, ARCHITECTURE.md ADR-013/ADR-014).

Day 0 proved device *enumeration* and a single-Add-op graph
(docs/hardware/bmptl-verification.md). That exercises neither the model
converter nor the NPU's shape constraints. This script proves the real path:

    PyTorch model --(openvino.convert_model)--> OpenVINO IR --> compile --> infer

for a ResNet18-scale convolutional encoder (hand-rolled: same block layout as
torchvision's resnet18 -- BasicBlock x [2,2,2,2], channel stages
64/128/256/512 -- but implemented with plain torch.nn so no torchvision
dependency is added; torchvision is not pinned in requirements-dev.txt or
requirements-bmptl.txt). Using a ResNet18-scale backbone is deliberate per
PLAN.md M03 "Dividend to protect": M10's PoseNet can reuse the same backbone
shape and M13's export inherits this recipe.

WHY CONVERSION HAPPENS ON THE LAPTOP, NOT ON bm-ptl (read this before moving code):
    `ov.convert_model(torch_module, example_input=...)` traces a *live* PyTorch
    module, so it needs `torch` and `openvino` in the *same* process. As found
    while building this script:
        - bm-ptl's ov_env has openvino==2026.3.1 but NO torch, NO torchvision,
          NO onnx, NO onnxscript.
        - The laptop has torch==2.14.0+cpu (requirements-dev.txt) but no
          torchvision, no onnx, no onnxscript, and (until this module) no
          openvino.
    torch.onnx.export() (the ONNX-intermediate route) needs the `onnx` or
    `onnxscript` package, neither of which is installed or pinned anywhere in
    this repo -- confirmed by trying it and hitting
    `ModuleNotFoundError: No module named 'onnx'` / 'onnxscript'.
    Rather than add an ONNX dependency nobody asked for, this script installs
    openvino==2026.3.1 (the exact version already pinned in
    requirements-bmptl.txt) on the laptop and uses OpenVINO's own PyTorch
    frontend (`ov.convert_model`) to go straight from the live torch.nn.Module
    to an IR, no ONNX file involved. This *is* an undeclared dependency on the
    laptop side (openvino is not yet in requirements-dev.txt) -- flagged
    honestly in benchmarks/ov-smoke-notes.md and in the module completion
    report, not silently added to that pinned file (out of scope for this
    module: only scripts/ov_smoke.py and benchmarks/ov-smoke-notes.md are
    this module's files).

    Net effect: `--export` (needs torch + openvino) runs on the laptop and
    produces the .xml/.bin IR pair directly -- no bm-ptl step is needed to
    *produce* the IR. `--device {CPU,GPU,NPU}` (needs openvino only, no torch)
    runs on bm-ptl against the IR files copied there, which is what
    PLAN.md M03 actually requires ("compile and infer must happen on bm-ptl").

Usage:
    # On the laptop (has torch + now openvino):
    python scripts/ov_smoke.py --export --artifact-dir artifacts/ov_smoke

    # scp the artifact dir to bm-ptl, then, on bm-ptl (has openvino, no torch):
    python scripts/ov_smoke.py --device CPU --artifact-dir artifacts/ov_smoke
    python scripts/ov_smoke.py --device GPU --artifact-dir artifacts/ov_smoke
    python scripts/ov_smoke.py --device NPU --artifact-dir artifacts/ov_smoke
"""

import argparse
import json
import platform
import sys
from pathlib import Path

import numpy as np

# Static input shape used throughout -- NCHW, batch 1, a standard vision-model
# resolution. "static" because M03's purpose (ADR-014) is specifically to find
# out whether the NPU accepts dynamic shapes, which requires first having a
# known-good static baseline to compare against.
INPUT_SHAPE = (1, 3, 224, 224)

DEFAULT_ARTIFACT_DIR = Path(__file__).resolve().parent.parent / "artifacts" / "ov_smoke"


def build_model():
    """Hand-rolled ResNet18-scale convolutional encoder.

    Same topology as torchvision.models.resnet18: a 7x7/stride-2 stem +
    maxpool, four stages of 2 BasicBlocks each (channels 64/128/256/512,
    stride-2 at the start of stages 2-4), global average pool, linear head.
    Implemented directly with torch.nn (no torchvision import) because
    torchvision is not a pinned dependency anywhere in this repo.

    For a reader new to robotics/ML: a "BasicBlock" here is the standard
    ResNet building block -- two 3x3 convolutions with a "skip connection"
    that adds the block's input back onto its output. That skip connection
    is what lets ResNets be trained much deeper than plain conv stacks
    without their gradients vanishing.
    """
    import torch
    import torch.nn as nn

    class BasicBlock(nn.Module):
        def __init__(self, in_planes, planes, stride=1, downsample=None):
            super().__init__()
            self.conv1 = nn.Conv2d(in_planes, planes, 3, stride, 1, bias=False)
            self.bn1 = nn.BatchNorm2d(planes)
            self.relu = nn.ReLU(inplace=True)
            self.conv2 = nn.Conv2d(planes, planes, 3, 1, 1, bias=False)
            self.bn2 = nn.BatchNorm2d(planes)
            self.downsample = downsample

        def forward(self, x):
            identity = x
            out = self.relu(self.bn1(self.conv1(x)))
            out = self.bn2(self.conv2(out))
            if self.downsample is not None:
                identity = self.downsample(x)
            out = out + identity
            return self.relu(out)

    class ResNet18Scale(nn.Module):
        def __init__(self, num_classes=512):
            super().__init__()
            self.in_planes = 64
            self.stem = nn.Sequential(
                nn.Conv2d(3, 64, 7, 2, 3, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(3, 2, 1),
            )
            self.layer1 = self._make_layer(64, 2, stride=1)
            self.layer2 = self._make_layer(128, 2, stride=2)
            self.layer3 = self._make_layer(256, 2, stride=2)
            self.layer4 = self._make_layer(512, 2, stride=2)
            self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
            self.fc = nn.Linear(512, num_classes)

        def _make_layer(self, planes, blocks, stride):
            downsample = None
            if stride != 1 or self.in_planes != planes:
                downsample = nn.Sequential(
                    nn.Conv2d(self.in_planes, planes, 1, stride, bias=False),
                    nn.BatchNorm2d(planes),
                )
            layers = [BasicBlock(self.in_planes, planes, stride, downsample)]
            self.in_planes = planes
            for _ in range(1, blocks):
                layers.append(BasicBlock(self.in_planes, planes))
            return nn.Sequential(*layers)

        def forward(self, x):
            x = self.stem(x)
            x = self.layer1(x)
            x = self.layer2(x)
            x = self.layer3(x)
            x = self.layer4(x)
            x = self.avgpool(x)
            x = torch.flatten(x, 1)
            return self.fc(x)

    return ResNet18Scale()


def do_export(artifact_dir: Path) -> None:
    """Build the model, compute a PyTorch reference output, and convert
    straight to OpenVINO IR in three variants. Requires torch AND openvino
    in this process -- run this on the laptop.
    """
    import torch
    import openvino as ov

    artifact_dir.mkdir(parents=True, exist_ok=True)

    torch.manual_seed(0)
    model = build_model()
    model.eval()

    param_count = sum(p.numel() for p in model.parameters())
    print(f"Built ResNet18-scale encoder: {param_count:,} parameters.")

    x = torch.randn(*INPUT_SHAPE)
    with torch.no_grad():
        reference_output = model(x)

    np.save(artifact_dir / "input.npy", x.numpy())
    np.save(artifact_dir / "reference_output.npy", reference_output.numpy())
    print(f"Saved input.npy {tuple(x.shape)} and reference_output.npy "
          f"{tuple(reference_output.shape)} (PyTorch FP32 reference).")

    # --- Variant 1: static shape, FP32 -----------------------------------
    ov_static = ov.convert_model(model, example_input=x, input=list(INPUT_SHAPE))
    static_xml = artifact_dir / "model_static.xml"
    ov.save_model(ov_static, str(static_xml))
    print(f"Wrote {static_xml.name} / {static_xml.with_suffix('.bin').name} "
          f"(static FP32), input shape "
          f"{ov_static.inputs[0].get_partial_shape()}.")

    # --- Variant 2: static shape, FP16-compressed weights -----------------
    # compress_to_fp16 stores weights as FP16 in the .bin; this is the
    # standard "FP16 IR" used to target the Arc iGPU (ADR-013).
    static_fp16_xml = artifact_dir / "model_static_fp16.xml"
    ov.save_model(ov_static, str(static_fp16_xml), compress_to_fp16=True)
    print(f"Wrote {static_fp16_xml.name} / "
          f"{static_fp16_xml.with_suffix('.bin').name} (static, FP16 weights).")

    # --- Variant 3: dynamic batch dimension, FP32 --------------------------
    # This is the variant used to answer M03 done-when 4: does the NPU accept
    # a dynamic batch dimension, or does it require the fully static shape?
    dyn_shape = [-1] + list(INPUT_SHAPE[1:])
    ov_dynamic = ov.convert_model(model, example_input=x, input=dyn_shape)
    dynamic_xml = artifact_dir / "model_dynamic.xml"
    ov.save_model(ov_dynamic, str(dynamic_xml))
    print(f"Wrote {dynamic_xml.name} / {dynamic_xml.with_suffix('.bin').name} "
          f"(dynamic batch), input shape "
          f"{ov_dynamic.inputs[0].get_partial_shape()}.")

    print(f"\nAll artifacts written to {artifact_dir.resolve()}")
    print("Copy this directory to bm-ptl, then run:")
    print("  python scripts/ov_smoke.py --device CPU --artifact-dir <dir>")
    print("  python scripts/ov_smoke.py --device GPU --artifact-dir <dir>")
    print("  python scripts/ov_smoke.py --device NPU --artifact-dir <dir>")


def _try_compile_and_infer(core, xml_path: Path, device: str, x: np.ndarray,
                            reference_output: np.ndarray | None):
    """Compile one IR on one device and, if reference_output is given, run
    one inference and compute max abs deviation vs the PyTorch reference.

    Returns a dict describing exactly what happened -- success or verbatim
    failure -- never raises. This is deliberate: a device rejecting the
    graph is a real, useful finding (ADR-013), not a bug to hide.
    """
    entry = {"ir_file": xml_path.name, "device": device}
    if not xml_path.exists():
        entry["status"] = "SKIPPED"
        entry["reason"] = f"{xml_path} does not exist (export step not run / not copied)"
        return entry

    try:
        compiled = core.compile_model(str(xml_path), device)
    except Exception as e:  # noqa: BLE001 -- we want the verbatim message, any exception type
        entry["status"] = "COMPILE_FAILED"
        entry["error_type"] = type(e).__name__
        entry["error"] = str(e)
        return entry

    # ADR-007: no silent device fallback. Read back the device OpenVINO
    # actually used from the *compiled* model rather than trusting the
    # string we asked for.
    try:
        entry["execution_devices"] = compiled.get_property("EXECUTION_DEVICES")
    except Exception:
        entry["execution_devices"] = "<not reported by this device/version>"

    entry["status"] = "COMPILE_OK"

    if reference_output is None:
        return entry

    try:
        infer_request = compiled.create_infer_request()
        result = infer_request.infer({0: x})
        output = result[compiled.output(0)]
        max_abs_dev = float(np.max(np.abs(output.astype(np.float64) -
                                           reference_output.astype(np.float64))))
        entry["infer_status"] = "OK"
        entry["max_abs_deviation_vs_torch"] = max_abs_dev
    except Exception as e:  # noqa: BLE001
        entry["infer_status"] = "INFER_FAILED"
        entry["error_type"] = type(e).__name__
        entry["error"] = str(e)

    return entry


# (xml filename, precision label, shape-kind label) for each of the three
# checks every device is put through.
VARIANTS = {
    "static_fp32": ("model_static.xml", "FP32", "static"),
    "static_fp16": ("model_static_fp16.xml", "FP16", "static"),
    "dynamic_fp32": ("model_dynamic.xml", "FP32", "dynamic_batch"),
}

# Marker line prefix used to hand a result dict from a `--single-variant`
# child process back to the parent via stdout, deliberately distinct from
# ordinary log output so it is easy to find even mixed with other prints.
RESULT_MARKER = "OV_SMOKE_RESULT_JSON: "


def do_single_variant(artifact_dir: Path, device: str, variant: str) -> None:
    """Run exactly one (device, precision, shape-kind) check and print its
    result as one JSON line prefixed with RESULT_MARKER, then exit 0.

    This is invoked as a *subprocess* by do_device_run(), one process per
    variant. Why: discovered empirically (see benchmarks/ov-smoke-notes.md)
    that OpenVINO's NPU plugin does not raise a normal, catchable C++/Python
    exception when compile_model() is given a fully dynamic (unbounded)
    batch dimension -- it terminates the whole process (observed exit code
    5, stderr message about "Upper bounds are not specified"). Running each
    variant in its own process means that crash cannot destroy the results
    already obtained for the other variants on the same device.
    """
    import openvino as ov

    core = ov.Core()
    x = np.load(artifact_dir / "input.npy")
    reference_output = np.load(artifact_dir / "reference_output.npy")

    xml_name, precision, shape_kind = VARIANTS[variant]
    r = _try_compile_and_infer(core, artifact_dir / xml_name, device, x, reference_output)
    r["precision"] = precision
    r["shape_kind"] = shape_kind
    r["host"] = platform.node()
    # Flush explicitly before the marker line: if this process is about to
    # be killed by the NPU driver/plugin on a later line, we want anything
    # already printed to have actually reached the pipe.
    print(RESULT_MARKER + json.dumps(r), flush=True)


def do_device_run(artifact_dir: Path, device: str) -> list:
    """Compile + infer the static FP32 IR, the static FP16 IR, and attempt
    the dynamic-batch IR, all on `device`. Requires openvino only -- run
    this on bm-ptl. Each variant runs in its own subprocess (see
    do_single_variant's docstring for why) and its result is appended to
    results.jsonl immediately, so a crash on one variant does not lose the
    others. Returns the list of result dicts actually obtained.
    """
    import openvino as ov
    import subprocess

    core = ov.Core()
    available = core.available_devices
    print(f"OpenVINO version: {ov.__version__}")
    print(f"Devices visible to this Core: {available}")
    if device not in available:
        print(f"WARNING: requested device '{device}' is not in the visible "
              f"device list {available}. Attempting anyway -- OpenVINO may "
              f"still reject it at compile_model().")
    del core  # only used above to enumerate devices; each variant gets its own Core in its subprocess

    x_path = artifact_dir / "input.npy"
    ref_path = artifact_dir / "reference_output.npy"
    if not x_path.exists() or not ref_path.exists():
        print(f"ERROR: {x_path} / {ref_path} not found in {artifact_dir}. "
              f"Run --export first (on the laptop) and copy the artifact "
              f"dir here.")
        sys.exit(1)

    results_path = artifact_dir / "results.jsonl"
    results = []

    for variant, (xml_name, precision, shape_kind) in VARIANTS.items():
        proc = subprocess.run(
            [sys.executable, "-u", str(Path(__file__).resolve()),
             "--single-variant", variant, "--device", device,
             "--artifact-dir", str(artifact_dir)],
            capture_output=True, text=True,
        )
        r = None
        for line in proc.stdout.splitlines():
            if line.startswith(RESULT_MARKER):
                r = json.loads(line[len(RESULT_MARKER):])
                break

        if r is None:
            # The child process died before it could report a result --
            # this is itself the finding for this variant (e.g. the NPU
            # dynamic-shape crash). Record it honestly, verbatim.
            r = {
                "ir_file": xml_name,
                "device": device,
                "precision": precision,
                "shape_kind": shape_kind,
                "status": "PROCESS_CRASHED",
                "exit_code": proc.returncode,
                "stderr": proc.stderr.strip(),
                "stdout": proc.stdout.strip(),
                "host": platform.node(),
            }

        results.append(r)
        _print_result(r)

        # Append immediately -- do not wait for all three variants, since
        # variant N+1 crashing must not cost us variant N's result.
        with open(results_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(r) + "\n")

    print(f"\nAppended {len(results)} result rows to {results_path}")
    return results


def _print_result(r: dict) -> None:
    tag = f"[{r['device']} | {r['shape_kind']} | {r['precision']}]"
    if r["status"] == "PROCESS_CRASHED":
        print(f"{tag} PROCESS CRASHED (exit code {r['exit_code']}). "
              f"Verbatim stderr:\n{r['stderr']}")
        return
    if r["status"] == "SKIPPED":
        print(f"{tag} SKIPPED: {r['reason']}")
        return
    if r["status"] == "COMPILE_FAILED":
        print(f"{tag} COMPILE FAILED: {r['error_type']}: {r['error']}")
        return
    exec_devices = r.get("execution_devices", "?")
    if r.get("infer_status") == "OK":
        print(f"{tag} compile OK (EXECUTION_DEVICES={exec_devices}), "
              f"infer OK, max |deviation| vs torch = "
              f"{r['max_abs_deviation_vs_torch']:.6e}")
    elif r.get("infer_status") == "INFER_FAILED":
        print(f"{tag} compile OK (EXECUTION_DEVICES={exec_devices}), "
              f"INFER FAILED: {r['error_type']}: {r['error']}")
    else:
        print(f"{tag} compile OK (EXECUTION_DEVICES={exec_devices}), "
              f"no reference available to infer against.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--export", action="store_true",
                         help="Build model + convert to IR (needs torch+openvino; run on laptop).")
    parser.add_argument("--device", choices=["CPU", "GPU", "NPU"], default=None,
                         help="Compile + infer the IR on this device (needs openvino only; run on bm-ptl).")
    parser.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR,
                         help=f"Where IR/input/reference/results live. Default: {DEFAULT_ARTIFACT_DIR}")
    parser.add_argument("--single-variant", choices=list(VARIANTS.keys()), default=None,
                         help=argparse.SUPPRESS)  # internal: used by do_device_run's subprocess calls
    args = parser.parse_args()

    if args.single_variant:
        # Internal child-process mode: run exactly one check, print one
        # RESULT_MARKER line, exit. Requires --device too.
        if not args.device:
            parser.error("--single-variant requires --device")
        do_single_variant(args.artifact_dir, args.device, args.single_variant)
        return

    if not args.export and not args.device:
        parser.error("Specify --export (laptop, produces IR) or --device CPU|GPU|NPU (bm-ptl, compiles+infers).")

    if args.export:
        do_export(args.artifact_dir)

    if args.device:
        do_device_run(args.artifact_dir, args.device)


if __name__ == "__main__":
    main()
