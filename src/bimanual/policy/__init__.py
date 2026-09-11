"""PolicyBackend abstraction: one interface, two runtimes (M12).

TorchPolicyBackend and OpenVinoPolicyBackend both implement
`predict(obs) -> action` and `describe() -> dict`. describe() reports the
device OpenVINO actually compiled to, never the requested one -- silent
device fallback is explicitly disallowed (ARCHITECTURE.md ADR-007).
"""
