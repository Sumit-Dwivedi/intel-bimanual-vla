# 03 — PyTorch to OpenVINO IR, and What "Compiled on NPU" Means

*Written before M03 runs. Nothing here is a result.*

**Problem.** Day 0 proved bm-ptl is reachable and that OpenVINO enumerates CPU, GPU and
NPU — but it proved it with one Add op (`docs/hardware/bmptl-verification.md:12-16`). That
exercised neither the model converter nor any device's shape rules.

**Key concept: IR is a compiler's intermediate representation.** OpenVINO does not execute
PyTorch. A converter reads a framework graph (PyTorch, ONNX, TensorFlow) and lowers it to
device-neutral IR — an `.xml` topology plus a `.bin` of weights — much as clang lowers C to
LLVM IR.

**Compiling to a device is the backend pass.** `compile_model(ir, "NPU")` hands IR to that
device's plugin, which picks kernels, memory layouts and precision for that silicon. One IR,
three backends, three sets of constraints. On our NPU5010 the open question is dynamic
shapes; M03 must state plainly whether they were accepted (`PLAN.md` M03 done-when 4).

**Why now, on a small model.** ADR-014 in `ARCHITECTURE.md`: discovering on Day 4 that
the NPU rejects our graph is unrecoverable, since bm-ptl expires Sept 17. A ResNet18-scale
encoder converts in minutes yet exercises real convolutions.

**Try this.** After it runs, compare per-device max deviation from the PyTorch reference.
Predict which device differs most, and why precision — not bugs — explains it.
