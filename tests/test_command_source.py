"""Tests for the CommandSource abstraction (PLAN.md M04).

Covers M04's three done-when criteria:
  1. Normal text, empty string, whitespace-only, and an unknown-word
     command all round-trip through TextCommandSource unchanged.
  2. (Structural, not a pytest test -- see the grep gate the Builder report
     runs separately: `grep -ri "audio|pcm|wav|microphone"
     src/bimanual/language src/bimanual/policy` must return no matches.)
  3. TextCommandSource and the stub VoiceCommandSource both satisfy the
     same CommandSource abstract base -- proven here by parameterising a
     single test over both, not by writing two separate tests.
"""

from __future__ import annotations

import io
from pathlib import Path

import pytest

from bimanual.command import (
    CommandEvent,
    CommandSource,
    TextCommandSource,
    VoiceCommandSource,
)


# ---------------------------------------------------------------------------
# TextCommandSource: normal text, empty string, whitespace-only, unknown word
# ---------------------------------------------------------------------------


def test_normal_text_command_round_trips():
    """A typical brief-style command comes back unchanged."""
    source = TextCommandSource(text="pick up the plate with arm A")
    event = source.poll()

    assert event is not None
    assert event.text == "pick up the plate with arm A"
    assert event.source_id == "text_cli"
    # Typed text has no recognition uncertainty.
    assert event.confidence is None
    assert isinstance(event.timestamp, float)

    # Only one command was given; the source is now exhausted.
    assert source.poll() is None
    source.close()


def test_empty_string_command_is_delivered_not_rejected():
    """CommandSource is a dumb transport: it must not reject empty text.

    Deciding whether an empty command is "valid" is the Grounder's job
    (M05), not CommandSource's -- see text_source.py's module docstring.
    """
    source = TextCommandSource(text="")
    event = source.poll()

    assert event is not None
    assert event.text == ""
    source.close()


def test_whitespace_only_command_is_delivered_unstripped():
    """Whitespace-only text is passed through verbatim, not stripped."""
    whitespace_text = "   \t  "
    source = TextCommandSource(text=whitespace_text)
    event = source.poll()

    assert event is not None
    # Exact equality, not just "is blank-ish" -- proves nothing strips it.
    assert event.text == whitespace_text
    source.close()


def test_unknown_word_command_is_delivered_without_validation():
    """A command containing words no Grounder will recognize still passes.

    TextCommandSource does not know the skill vocabulary (ARCHITECTURE.md
    ADR-001) and must not try to validate against it -- that check belongs
    to the Grounder (M05), which raises a typed UngroundedCommandError.
    """
    nonsense = "teleport the flibbertigibbet to the moon"
    source = TextCommandSource(text=nonsense)
    event = source.poll()

    assert event is not None
    assert event.text == nonsense
    source.close()


# ---------------------------------------------------------------------------
# TextCommandSource construction modes: CLI arg, file, stdin
# ---------------------------------------------------------------------------


def test_text_source_from_file_yields_one_event_per_line(tmp_path: Path):
    commands_file = tmp_path / "commands.txt"
    commands_file.write_text(
        "open the top drawer\npick up the plate with arm A\n",
        encoding="utf-8",
    )

    source = TextCommandSource(file=commands_file)

    first = source.poll()
    second = source.poll()
    third = source.poll()

    assert first is not None and first.text == "open the top drawer"
    assert second is not None and second.text == "pick up the plate with arm A"
    assert third is None  # exhausted
    assert first.source_id == "text_file"
    source.close()


def test_text_source_from_stdin_reads_eagerly_and_polls_without_blocking(
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr("sys.stdin", io.StringIO("open the top drawer\n"))

    # No `text=` and no `file=` given -> reads stdin, per the constructor
    # contract documented in text_source.py.
    source = TextCommandSource()
    event = source.poll()

    assert event is not None
    assert event.text == "open the top drawer"
    assert event.source_id == "text_stdin"
    source.close()


def test_text_source_rejects_both_text_and_file(tmp_path: Path):
    commands_file = tmp_path / "commands.txt"
    commands_file.write_text("open the top drawer\n", encoding="utf-8")

    with pytest.raises(ValueError):
        TextCommandSource(text="pick up the plate", file=commands_file)


def test_close_is_idempotent_and_poll_returns_none_after_close():
    source = TextCommandSource(text="open the top drawer")
    source.close()
    source.close()  # must not raise

    assert source.poll() is None


# ---------------------------------------------------------------------------
# VoiceCommandSource stub: right shape, no Speechmatics wiring
# ---------------------------------------------------------------------------


def test_voice_stub_emits_placeholder_transcription_with_confidence(
    tmp_path: Path,
):
    fake_audio = tmp_path / "command.wav"
    fake_audio.write_bytes(b"")  # stub never reads this file's contents

    source = VoiceCommandSource(audio_file=fake_audio)
    event = source.poll()

    assert event is not None
    assert isinstance(event.text, str) and len(event.text) > 0
    assert event.source_id == "voice_stub"
    # The voice path must populate confidence (unlike text, which cannot).
    assert event.confidence is not None
    assert 0.0 <= event.confidence <= 1.0
    assert event.raw_meta["audio_file"] == str(fake_audio)
    assert event.raw_meta["wired"] is False  # Speechmatics (M15) not attached

    # A stub transcribes the file once; a second poll yields nothing new.
    assert source.poll() is None
    source.close()


# ---------------------------------------------------------------------------
# The Liskov proof: one test, parameterised over both implementations.
# ---------------------------------------------------------------------------


def _make_text_source() -> CommandSource:
    return TextCommandSource(text="open the top drawer")


def _make_voice_source(tmp_path: Path) -> CommandSource:
    fake_audio = tmp_path / "command.wav"
    fake_audio.write_bytes(b"")
    return VoiceCommandSource(audio_file=fake_audio)


@pytest.mark.parametrize(
    "source_factory",
    [
        lambda tmp_path: _make_text_source(),
        lambda tmp_path: _make_voice_source(tmp_path),
    ],
    ids=["TextCommandSource", "VoiceCommandSource"],
)
def test_both_implementations_satisfy_the_same_command_source_contract(
    source_factory, tmp_path: Path
):
    """Proves the abstraction holds: a caller can treat either the same way.

    This is the test PLAN.md M04 done-when 3 asks for -- "verified by a
    test that parameterises over both" -- and it is the only place in this
    file that both classes are exercised through the same code path with
    no branch on which one it is.
    """
    source: CommandSource = source_factory(tmp_path)

    # Both implementations satisfy the abstract base.
    assert isinstance(source, CommandSource)

    # poll() -> CommandEvent | None, non-blocking, on both.
    event = source.poll()
    assert isinstance(event, CommandEvent)
    assert isinstance(event.text, str)
    assert isinstance(event.timestamp, float)
    assert isinstance(event.source_id, str)
    assert isinstance(event.raw_meta, dict)

    # Both eventually exhaust and return None rather than raising.
    while source.poll() is not None:
        pass

    # close() is safe to call on both, and is idempotent on both.
    source.close()
    source.close()
