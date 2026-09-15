"""Tests for the CommandSource abstraction (PLAN.md M04) and its two
implementations, including the real Speechmatics wiring added in M15
(`voice_source.py`, ADR-058).

Covers M04's three done-when criteria:
  1. Normal text, empty string, whitespace-only, and an unknown-word
     command all round-trip through TextCommandSource unchanged.
  2. (Structural, not a pytest test -- see the grep gate the Builder report
     runs separately: `grep -ri "audio|pcm|wav|microphone"
     src/bimanual/language src/bimanual/policy` must return no matches.)
  3. TextCommandSource and VoiceCommandSource both satisfy the same
     CommandSource abstract base -- proven here by parameterising a single
     test over both, not by writing two separate tests.

The VoiceCommandSource tests below never touch the real network or the
real `.env`/API key -- see that section's own header comment.
"""

from __future__ import annotations

import io
import json
import time
import urllib.error
import wave
from pathlib import Path

import pytest

from bimanual.command import (
    CommandEvent,
    CommandSource,
    TextCommandSource,
    VoiceCommandSource,
)
from bimanual.command import voice_source


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
# VoiceCommandSource: real Speechmatics wiring (M15/ADR-058), network mocked
# ---------------------------------------------------------------------------
#
# These tests never touch the real network or the real .env/API key -- every
# test here monkeypatches `urllib.request.urlopen` with a fake Speechmatics
# server (see `_FakeResponse`/`_fake_urlopen_returning`) and points
# `VoiceCommandSource` at a temporary `.env` (see `_write_env`). This keeps
# the suite fast and offline while still exercising the real
# request/response/error-translation code paths in `voice_source.py`, not a
# stand-in for them.


def _write_wav(path: Path, seconds: float = 0.1) -> None:
    """Write a minimal, genuinely valid mono 16 kHz WAV file (silence)."""
    n_frames = int(16000 * seconds)
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(16000)
        wf.writeframes(b"\x00\x00" * n_frames)


def _write_env(path: Path, api_key: str | None = "fake-test-key") -> Path:
    """Write a temporary `.env` (or an empty one if `api_key` is None)."""
    env_file = path / ".env"
    env_file.write_text(f"ai_infra={api_key}\n" if api_key else "", encoding="utf-8")
    return env_file


class _FakeHTTPResponse:
    """Enough of `http.client.HTTPResponse` for `voice_source._call` to use:
    a context manager whose `.read()` returns pre-canned JSON bytes."""

    def __init__(self, payload: dict) -> None:
        self._data = json.dumps(payload).encode("utf-8")

    def read(self) -> bytes:
        return self._data

    def __enter__(self) -> "_FakeHTTPResponse":
        return self

    def __exit__(self, *exc_info: object) -> bool:
        return False


def _words_as_speechmatics_results(text: str) -> list[dict]:
    """Turn plain text into Speechmatics json-v2-shaped word results."""
    return [
        {"type": "word", "alternatives": [{"content": w, "confidence": 0.9}]}
        for w in text.split()
    ]


def _install_fake_speechmatics(
    monkeypatch: pytest.MonkeyPatch,
    *,
    transcript_results: list[dict],
    job_status: str = "done",
) -> None:
    """Monkeypatch urlopen with a fake server: submit -> poll -> transcript."""

    def fake_urlopen(request, timeout=None):  # noqa: ANN001 -- test double
        url = request.full_url
        if url == voice_source._JOBS_URL:
            return _FakeHTTPResponse({"id": "job123"})
        if url == f"{voice_source._JOBS_URL}/job123":
            return _FakeHTTPResponse({"job": {"status": job_status}})
        if url.startswith(f"{voice_source._JOBS_URL}/job123/transcript"):
            return _FakeHTTPResponse({"results": transcript_results})
        raise AssertionError(f"unexpected URL in test: {url}")

    monkeypatch.setattr(voice_source.urllib.request, "urlopen", fake_urlopen)


def test_voice_source_transcribes_via_speechmatics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """A real (mocked) Speechmatics round trip produces a populated event."""
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)
    _install_fake_speechmatics(
        monkeypatch,
        transcript_results=_words_as_speechmatics_results("open the top drawer"),
    )

    source = VoiceCommandSource(audio_file=wav, env_path=env_file)
    event = source.poll()

    assert event is not None
    assert event.text == "open the top drawer"
    assert event.source_id == "voice_speechmatics"
    assert event.confidence is not None
    assert 0.0 <= event.confidence <= 1.0
    assert event.raw_meta["audio_file"] == str(wav)
    assert event.raw_meta["wired"] is True  # unlike the old M04 stub
    assert event.raw_meta["job_id"] == "job123"

    # One transcription per construction; a second poll yields nothing new.
    assert source.poll() is None
    source.close()
    source.close()  # idempotent
    assert source.poll() is None  # None after close, per the ABC contract


def test_voice_source_poll_is_near_instant_after_construction(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """poll() itself must not block -- PLAN.md M15 done-when 2.

    The (necessarily slower, network-bound) work happens in __init__; see
    voice_source.py's module docstring, "Non-blocking poll(), honestly".
    This times poll() alone, after construction has already finished.
    """
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)
    _install_fake_speechmatics(
        monkeypatch, transcript_results=_words_as_speechmatics_results("pick up the plate")
    )

    source = VoiceCommandSource(audio_file=wav, env_path=env_file)

    start = time.monotonic()
    source.poll()
    elapsed = time.monotonic() - start

    assert elapsed < 0.05  # generous; a buffer pop, not a network call
    source.close()


def test_voice_source_missing_key_raises_before_any_network_call(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """No `ai_infra` value -> MissingApiKeyError, and no network attempted."""
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path, api_key=None)  # blank .env

    def fail_if_called(request, timeout=None):  # noqa: ANN001
        raise AssertionError("network must not be touched when the key is missing")

    monkeypatch.setattr(voice_source.urllib.request, "urlopen", fail_if_called)

    with pytest.raises(voice_source.MissingApiKeyError):
        VoiceCommandSource(audio_file=wav, env_path=env_file)


def test_voice_source_malformed_wav_raises_before_any_network_call(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """A non-WAV file -> InvalidAudioError, and no network attempted."""
    not_a_wav = tmp_path / "command.wav"
    not_a_wav.write_bytes(b"this is not a wav file")
    env_file = _write_env(tmp_path)

    def fail_if_called(request, timeout=None):  # noqa: ANN001
        raise AssertionError("network must not be touched for an unreadable WAV")

    monkeypatch.setattr(voice_source.urllib.request, "urlopen", fail_if_called)

    with pytest.raises(voice_source.InvalidAudioError):
        VoiceCommandSource(audio_file=not_a_wav, env_path=env_file)


def test_voice_source_auth_rejection_raises_authentication_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)

    def fake_urlopen(request, timeout=None):  # noqa: ANN001
        raise urllib.error.HTTPError(request.full_url, 401, "Unauthorized", hdrs=None, fp=None)

    monkeypatch.setattr(voice_source.urllib.request, "urlopen", fake_urlopen)

    with pytest.raises(voice_source.AuthenticationError):
        VoiceCommandSource(audio_file=wav, env_path=env_file)


def test_voice_source_network_unreachable_raises_network_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)

    def fake_urlopen(request, timeout=None):  # noqa: ANN001
        raise urllib.error.URLError("no route to host")

    monkeypatch.setattr(voice_source.urllib.request, "urlopen", fake_urlopen)

    with pytest.raises(voice_source.NetworkUnreachableError):
        VoiceCommandSource(audio_file=wav, env_path=env_file)


def test_voice_source_job_timeout_raises_job_timeout_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """A job stuck at "running" forever must raise, not hang the test suite."""
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)
    _install_fake_speechmatics(monkeypatch, transcript_results=[], job_status="running")

    with pytest.raises(voice_source.JobTimeoutError):
        VoiceCommandSource(
            audio_file=wav, env_path=env_file, timeout_s=0.05, poll_interval_s=0.01
        )


def test_voice_source_rejected_job_raises_invalid_audio_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)
    _install_fake_speechmatics(monkeypatch, transcript_results=[], job_status="rejected")

    with pytest.raises(voice_source.InvalidAudioError):
        VoiceCommandSource(audio_file=wav, env_path=env_file)


def test_voice_source_empty_transcript_raises_empty_transcript_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)
    _install_fake_speechmatics(monkeypatch, transcript_results=[])  # no words at all

    with pytest.raises(voice_source.EmptyTranscriptError):
        VoiceCommandSource(audio_file=wav, env_path=env_file)


# ---------------------------------------------------------------------------
# The Liskov proof: one test, parameterised over both implementations.
# ---------------------------------------------------------------------------


def _make_text_source(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> CommandSource:
    del tmp_path, monkeypatch  # unused by this factory; shared signature only
    return TextCommandSource(text="open the top drawer")


def _make_voice_source(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> CommandSource:
    wav = tmp_path / "command.wav"
    _write_wav(wav)
    env_file = _write_env(tmp_path)
    _install_fake_speechmatics(
        monkeypatch, transcript_results=_words_as_speechmatics_results("open the top drawer")
    )
    return VoiceCommandSource(audio_file=wav, env_path=env_file)


@pytest.mark.parametrize(
    "source_factory",
    [_make_text_source, _make_voice_source],
    ids=["TextCommandSource", "VoiceCommandSource"],
)
def test_both_implementations_satisfy_the_same_command_source_contract(
    source_factory, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """Proves the abstraction holds: a caller can treat either the same way.

    This is the test PLAN.md M04 done-when 3 asks for -- "verified by a
    test that parameterises over both" -- and it is the only place in this
    file that both classes are exercised through the same code path with
    no branch on which one it is.
    """
    source: CommandSource = source_factory(tmp_path, monkeypatch)

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
