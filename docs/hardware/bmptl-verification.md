# BM-PTL Verification (Sept 10, 2026)

## Hardware
- CPU: Intel(R) Core(TM) Ultra X7 358H, 16 cores
- iGPU: Intel(R) Arc(TM) B390 GPU
- NPU: Intel(R) AI Boost (Panther Lake NPU5010)
- RAM: 32 GB

## OpenVINO devices detected
['CPU', 'GPU', 'NPU']

## Trivial-model latency (single Add op, 1x3x224x224 fp32)
- CPU: 0.061 ms
- GPU: 0.218 ms
- NPU: 0.635 ms

## Note on interpretation
On this trivial op, dispatch overhead dominates. NPU wins only when
compute time exceeds ~200μs dispatch overhead — expected on real
vision-encoder workloads. Trust the ordering only for realistic models.

## Environment pinned in requirements-bmptl.txt