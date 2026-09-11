"""CommandEvent: the one type that crosses the CommandSource boundary.

Per ARCHITECTURE.md ADR-002 and the component-contract table (section 1),
`CommandEvent` is what every CommandSource implementation emits and what
everything downstream (Grounder, eventually PolicyBackend) consumes. No
audio type -- buffer, sample rate, codec -- exists anywhere past this
dataclass. Whether the words came from a keyboard or a speech-to-text
engine, by the time a caller sees a CommandEvent it is just text plus
metadata about where it came from.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class CommandEvent:
    """A single natural-language command, already reduced to text.

    Attributes:
        text: The command text itself, e.g. "pick up the plate with arm A".
            Passed through verbatim by CommandSource implementations --
            CommandSource does not validate or normalize the text; that is
            the Grounder's job (M05, ARCHITECTURE.md ADR-003).
        timestamp: Unix epoch seconds (as returned by `time.time()`) at the
            moment the event was created. Lets a caller reason about how
            stale a command is, e.g. for a mid-episode re-command demo.
        source_id: A short string identifying which CommandSource instance
            (and which "channel" of it) produced this event, e.g.
            "text_cli", "text_file", "text_stdin", "voice_stub". Useful for
            logging and for `manifest.json` provenance (see M08's
            done-when 6 pattern of recording where data came from).
        confidence: A transcription confidence in [0, 1], or None when the
            source has no notion of confidence (text sources: always None,
            since typed text has no recognition uncertainty). Voice sources
            populate this from the speech-to-text engine.
        raw_meta: A free-form dict for source-specific extra detail (e.g.
            which file a line was read from, or which audio file a stub
            voice transcription came from) that does not deserve its own
            field on this shared dataclass.
    """

    text: str
    timestamp: float
    source_id: str
    confidence: float | None = None
    raw_meta: dict = field(default_factory=dict)

    @classmethod
    def now(
        cls,
        text: str,
        source_id: str,
        confidence: float | None = None,
        raw_meta: dict | None = None,
    ) -> "CommandEvent":
        """Convenience constructor that stamps the current time.

        Every CommandSource implementation needs `time.time()` at the
        moment of creation; centralizing that here means the timestamp
        policy (wall-clock epoch seconds) is decided in exactly one place.
        """
        return cls(
            text=text,
            timestamp=time.time(),
            source_id=source_id,
            confidence=confidence,
            raw_meta=raw_meta if raw_meta is not None else {},
        )
