"""TextCommandSource: the always-available CommandSource implementation.

Constructible three ways (PLAN.md M04 outputs: "from a CLI arg, a file, or
stdin"):

    TextCommandSource(text="pick up the plate with arm A")   # one command
    TextCommandSource(file=Path("commands.txt"))              # one per line
    TextCommandSource()                                        # reads stdin

In every case all input is read eagerly at construction time and buffered
in a queue; `poll()` only ever pops from that queue, so it can never block
on I/O (ADR-002's non-blocking requirement -- see source.py's docstring).

TextCommandSource deliberately does not validate, strip, or reject
anything: an empty string, a whitespace-only string, and a string full of
words the Grounder will not recognize are all valid `CommandEvent.text`
values as far as this class is concerned. Deciding what counts as a
"real" command is the Grounder's job (M05, ARCHITECTURE.md ADR-003), not
this transport's job -- keeping that judgement out of CommandSource is
what lets `TextCommandSource` and `VoiceCommandSource` share one simple
contract.
"""

from __future__ import annotations

import sys
from collections import deque
from pathlib import Path

from bimanual.command.events import CommandEvent
from bimanual.command.source import CommandSource


class TextCommandSource(CommandSource):
    """Delivers one CommandEvent per buffered line of text.

    Exactly one of `text` or `file` should be given. If neither is given,
    all of stdin is read at construction time (one command per line).
    """

    def __init__(
        self,
        text: str | None = None,
        file: Path | None = None,
    ) -> None:
        if text is not None and file is not None:
            raise ValueError(
                "TextCommandSource accepts at most one of `text` or `file`, "
                "not both."
            )

        self._closed = False

        if text is not None:
            # Single literal command, e.g. from a CLI arg like
            # `--command "open the drawer"`. Not split into lines: a CLI
            # arg is one command, verbatim, even if it happens to contain
            # a newline.
            self._queue: deque[tuple[str, dict]] = deque([(text, {"origin": "cli"})])
            self._source_id = "text_cli"
        elif file is not None:
            # One command per line. Strip only the trailing newline that
            # the file format imposes -- not the line's own content -- so
            # a genuinely whitespace-only line still reaches the Grounder
            # as whitespace-only rather than being silently dropped here.
            lines = file.read_text(encoding="utf-8").splitlines()
            self._queue = deque(
                (line, {"origin": "file", "path": str(file), "line_no": i + 1})
                for i, line in enumerate(lines)
            )
            self._source_id = "text_file"
        else:
            # No text and no file: read everything stdin has to offer,
            # right now, at construction time. This keeps poll() itself
            # non-blocking (ADR-002) even though *constructing* a
            # stdin-backed source is expected to happen once at startup,
            # e.g. `echo "open the drawer" | python run_demo.py`.
            lines = sys.stdin.read().splitlines()
            self._queue = deque(
                (line, {"origin": "stdin", "line_no": i + 1})
                for i, line in enumerate(lines)
            )
            self._source_id = "text_stdin"

    def poll(self) -> CommandEvent | None:
        if self._closed or not self._queue:
            return None
        text, raw_meta = self._queue.popleft()
        return CommandEvent.now(
            text=text,
            source_id=self._source_id,
            confidence=None,  # typed text has no recognition uncertainty
            raw_meta=raw_meta,
        )

    def close(self) -> None:
        # Idempotent per the CommandSource contract: draining the queue is
        # enough to make every subsequent poll() return None, and setting
        # _closed guards against a poll() racing a partially-drained queue.
        self._closed = True
        self._queue.clear()


def print_all_commands(source: CommandSource) -> None:
    """Consume a CommandSource to exhaustion, printing each event's text.

    This is the Liskov check made executable (see the M04 task brief and
    docs/learn/04-command-source.md, "Try this"): it takes a
    `CommandSource` and only ever calls the two methods the abstract base
    promises -- `poll()` and `close()`. It never does `isinstance(source,
    TextCommandSource)` or any other branch on the concrete type, so this
    function works identically whether it is handed a `TextCommandSource`
    or a `VoiceCommandSource` (see tests/test_command_source.py for the
    parameterised proof).
    """
    while True:
        event = source.poll()
        if event is None:
            break
        print(f"[{event.source_id}] {event.text!r} (confidence={event.confidence})")
    source.close()


if __name__ == "__main__":
    # Demo / manual smoke test, guarded per the Windows-first entry-point
    # convention. Run with a literal command:
    #     python -m bimanual.command.text_source --command "open the drawer"
    # or pipe stdin:
    #     echo "open the drawer" | python -m bimanual.command.text_source
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--command",
        type=str,
        default=None,
        help="A single literal command. If omitted, reads from stdin.",
    )
    parser.add_argument(
        "--file",
        type=Path,
        default=None,
        help="A file with one command per line.",
    )
    args = parser.parse_args()

    demo_source: CommandSource = TextCommandSource(text=args.command, file=args.file)
    print_all_commands(demo_source)
