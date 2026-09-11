"""Abstract CommandSource: the strategy interface (ARCHITECTURE.md ADR-002).

A CommandSource is "how a command arrives". A caller (eventually the
Coordinator, per ARCHITECTURE.md section 2 step 1) holds a `CommandSource`
and calls `poll()` on it once per episode, or mid-episode for a
re-command demo. The caller never needs to know or branch on which
concrete implementation it was handed -- that is the whole point of the
seam (see docs/learn/04-command-source.md, "Try this").

Non-blocking is a load-bearing property, not a nicety: ADR-002 states
"poll() is non-blocking on both implementations: the simulator loop never
waits on a network transcription call." Every implementation must return
promptly from poll() -- either a CommandEvent that is already available,
or None -- and must never block the caller waiting on I/O (a network
call, a live microphone, etc.) inside poll() itself. Any such waiting
must happen elsewhere (e.g. eagerly at construction time).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from bimanual.command.events import CommandEvent


class CommandSource(ABC):
    """Strategy interface for delivering natural-language commands.

    Concrete implementations: `TextCommandSource` (typed/CLI/file/stdin)
    and `VoiceCommandSource` (speech, laptop-only per ARCHITECTURE.md
    section 3 -- "CommandSource: voice | only here | never").
    """

    @abstractmethod
    def poll(self) -> CommandEvent | None:
        """Return the next available command, or None if none is ready.

        Must not block. Returns exactly one CommandEvent per call when a
        command is available; returns None when the source is exhausted
        or has nothing new since the last call.
        """
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        """Release any resources held by this source (files, handles).

        Idempotent: calling close() more than once must not raise. After
        close(), poll() should return None rather than raising, so a
        caller that closes a source mid-loop does not need a try/except
        around a stray extra poll().
        """
        raise NotImplementedError
