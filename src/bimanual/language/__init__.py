"""Grounder abstraction (M05).

Turns a CommandEvent's text into an ordered TaskPlan of SkillCalls with
explicit arm assignment. RuleGrounder is the deterministic floor
(ARCHITECTURE.md ADR-003); a VlmGrounder is a bounded stretch item, never a
dependency of the demo path.
"""
