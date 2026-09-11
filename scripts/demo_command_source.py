"""Demo / manual smoke test for the CommandSource abstraction (M04).

Demonstrates both implementations that ship with M04:

    TextCommandSource   -- from a literal CLI string, and from a file
    VoiceCommandSource  -- the stub (no real transcription, fixed output)

This script's only job is to show usage -- it contains no new logic beyond
wiring together classes that already exist in `bimanual.command`. It calls
`print_all_commands()` (imported from `bimanual.command.text_source`, its
original home) on each source in turn. That helper only ever calls
`poll()` and `close()` -- the two methods the abstract `CommandSource`
promises -- and never branches on the concrete type via `isinstance` or
otherwise. That is the Liskov substitution property M04 set out to prove:
the same consumer function works unmodified whether it is handed a
`TextCommandSource` or a `VoiceCommandSource`.

Previously this demo lived in a `if __name__ == "__main__":` block inside
`bimanual/command/text_source.py` itself. Running that file with
`python -m bimanual.command.text_source` re-imported it under a second
module identity (`__main__`), *in addition to* its normal package identity
(`bimanual.command.text_source`, already imported eagerly by
`bimanual/command/__init__.py`). Python flags exactly this double-import
situation with:

    RuntimeWarning: 'bimanual.command.text_source' found in sys.modules
    after import of package 'bimanual.command', but prior to execution of
    'bimanual.command.text_source'; this may result in unpredictable
    behaviour

Harmless today because these classes hold no module-level mutable state,
but it would silently break the moment two identities of the same class
existed at once (e.g. an `isinstance` check against the wrong identity).
Moving the demo here, where it only ever imports `bimanual.command` via
its normal package path, removes the second identity entirely.

Run with a literal command:
    python scripts/demo_command_source.py --command "open the drawer"
or a file:
    python scripts/demo_command_source.py --file commands.txt
or pipe stdin:
    echo "open the drawer" | python scripts/demo_command_source.py
"""

from __future__ import annotations

import argparse
import tempfile
from pathlib import Path

from bimanual.command import CommandSource, TextCommandSource, VoiceCommandSource
from bimanual.command.text_source import print_all_commands


def _demo_text_from_cli(command: str) -> None:
    """TextCommandSource fed a single literal command (e.g. a CLI arg)."""
    print("--- TextCommandSource: from a CLI string ---")
    source: CommandSource = TextCommandSource(text=command)
    print_all_commands(source)


def _demo_text_from_file(file: Path | None) -> None:
    """TextCommandSource fed a file with one command per line.

    If no file was given on the command line, a small temporary one is
    written so the demo has something to read without requiring the
    caller to supply `--file` every time.
    """
    print("--- TextCommandSource: from a file ---")
    if file is not None:
        source: CommandSource = TextCommandSource(file=file)
        print_all_commands(source)
        return

    with tempfile.TemporaryDirectory() as tmp_dir:
        demo_file = Path(tmp_dir) / "demo_commands.txt"
        demo_file.write_text(
            "open the top drawer\npick up the plate with arm A\n",
            encoding="utf-8",
        )
        source = TextCommandSource(file=demo_file)
        print_all_commands(source)


def _demo_voice_stub() -> None:
    """VoiceCommandSource stub: same consumer, no real audio involved."""
    print("--- VoiceCommandSource: stub ---")
    source: CommandSource = VoiceCommandSource(audio_file=Path("unused.wav"))
    print_all_commands(source)


if __name__ == "__main__":
    # Guarded per the Windows-first entry-point convention.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--command",
        type=str,
        default="open the top drawer and pick up the plate with arm A",
        help="A single literal command for the CLI-string demo.",
    )
    parser.add_argument(
        "--file",
        type=Path,
        default=None,
        help="A file with one command per line, for the file demo. "
        "If omitted, a temporary demo file is used.",
    )
    args = parser.parse_args()

    _demo_text_from_cli(args.command)
    _demo_text_from_file(args.file)
    _demo_voice_stub()
