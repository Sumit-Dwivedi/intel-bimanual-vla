"""VoiceCommandSource: the voice CommandSource -- STUB, not wired (M04/M15).

This module gives `VoiceCommandSource` the right *shape* to satisfy
`CommandSource` -- constructible from an audio file path, emits a
`CommandEvent` whose `source_id` marks it as voice and whose `confidence`
is populated -- without wiring an actual speech-to-text engine.
Speechmatics integration is M15, the droppable bonus
(`CONSTRAINTS.md:38-41`), and is explicitly out of scope for M04.

Per ARCHITECTURE.md section 3, voice only ever runs on the laptop
("CommandSource: voice | only here | never"), and per ADR-002 nothing
past this class ever sees an audio buffer, sample rate, or codec -- this
stub already enforces that: it takes a `Path` to an audio file and stores
only the path string in `raw_meta`, never any audio bytes.

Because this is a stub, `poll()` always returns the same fixed placeholder
transcription, once, then None -- it does not actually read or decode the
audio file's contents. When M15 wires Speechmatics, only this class's
internals change; `CommandSource`, `CommandEvent`, and every caller of
`poll()`/`close()` are unaffected, which is the entire point of the
ADR-002 seam.
"""

from __future__ import annotations

from pathlib import Path

from bimanual.command.events import CommandEvent
from bimanual.command.source import CommandSource

# Fixed placeholder text returned by every stub transcription. Chosen to
# look like a real command (matching the brief's p1 example) so downstream
# code exercising this stub sees realistic-shaped text, not a dummy string
# like "TODO" that would never parse.
_PLACEHOLDER_TRANSCRIPTION = (
    "open the top drawer and pick up the plate with arm A"
)

# Fixed placeholder confidence. Not measured -- there is no real speech
# engine behind this stub -- chosen only to prove the field is populated
# and plumbed through, as the real Speechmatics integration (M15) will
# populate it from an actual recognition confidence.
_PLACEHOLDER_CONFIDENCE = 0.5


class VoiceCommandSource(CommandSource):
    """Stub voice CommandSource. Does not transcribe; returns a fixed line.

    Args:
        audio_file: Path to an audio file that a real implementation would
            transcribe. Stored only as a path string in `raw_meta` -- this
            stub never opens or reads the file's audio content.
    """

    SOURCE_ID = "voice_stub"

    def __init__(self, audio_file: Path) -> None:
        self._audio_file = audio_file
        self._closed = False
        self._emitted = False

    def poll(self) -> CommandEvent | None:
        if self._closed or self._emitted:
            return None
        self._emitted = True
        return CommandEvent.now(
            text=_PLACEHOLDER_TRANSCRIPTION,
            source_id=self.SOURCE_ID,
            confidence=_PLACEHOLDER_CONFIDENCE,
            raw_meta={
                "audio_file": str(self._audio_file),
                "engine": "stub",
                "wired": False,  # explicit: Speechmatics (M15) not attached
            },
        )

    def close(self) -> None:
        self._closed = True
