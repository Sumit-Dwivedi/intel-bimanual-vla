"""PerceptionBackend abstraction (M10).

StatePerception reads privileged MuJoCo simulator state and is a development
aid only, gated behind an explicit --perception state flag. VisionPerception
runs a PoseNet (camera images -> object poses) through OpenVinoPolicyBackend
and is the only perception path used on the demo path and in all 10-seed
evaluation runs (ARCHITECTURE.md ADR-005).
"""
