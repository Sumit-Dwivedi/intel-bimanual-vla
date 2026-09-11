"""CommandSource abstraction (M04).

Delivers a natural-language command as a CommandEvent, non-blocking.
Implementations: TextCommandSource (CLI/file/stdin), VoiceCommandSource
(Speechmatics, laptop-only, ADR-002). No audio type may cross the boundary
out of this package -- see ARCHITECTURE.md ADR-002 and PLAN.md M04 done-when 2.
"""

from bimanual.command.events import CommandEvent
from bimanual.command.source import CommandSource
from bimanual.command.text_source import TextCommandSource
from bimanual.command.voice_source import VoiceCommandSource

__all__ = [
    "CommandEvent",
    "CommandSource",
    "TextCommandSource",
    "VoiceCommandSource",
]
