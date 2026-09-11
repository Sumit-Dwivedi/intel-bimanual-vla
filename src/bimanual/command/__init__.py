"""CommandSource abstraction (M04).

Delivers a natural-language command as a CommandEvent, non-blocking.
Implementations: TextCommandSource (CLI/file/stdin), VoiceCommandSource
(Speechmatics, laptop-only, ADR-002). No audio type may cross the boundary
out of this package -- see ARCHITECTURE.md ADR-002 and PLAN.md M04 done-when 2.
"""
