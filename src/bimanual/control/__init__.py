"""Coordinator, IK solver and skill primitives (M06).

Coordinator arbitrates which arm runs which SkillCall next, tracks task state
across a multi-step sequence, and owns the shared-workspace mutex for
`handoff` (ARCHITECTURE.md ADR-010). ScriptedSkillExecutor drives each skill
with closed-loop IK control; it is both the Day-3 fallback policy
(CONSTRAINTS.md fallback strategy) and the demonstration generator for
imitation learning (ADR-004).
"""
