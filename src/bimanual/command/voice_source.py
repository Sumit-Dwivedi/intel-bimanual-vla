"""VoiceCommandSource: Speechmatics-backed voice CommandSource (M15/ADR-002).

**This module used to be a stub** (M04) that returned a fixed placeholder
line without reading its audio file or talking to a network at all. It now
does the real thing: reads a WAV file, sends it to the Speechmatics batch
transcription API, and turns the result into a `CommandEvent` -- the exact
same typed boundary `TextCommandSource` produces (`command/events.py`), so
nothing downstream (Grounder, run_demo.py) needs to know or care that this
command came from speech rather than a keyboard (ARCHITECTURE.md ADR-002:
"the VLA never sees audio").

**Credential.** Per this repo's actual `.env` (not the `SPEECHMATICS_API_KEY`
name `ARCHITECTURE.md` ADR-019 originally specified), the API key lives in
a gitignored `.env` file at the repo root under the variable name
`ai_infra`. This is a disclosed deviation from ADR-019's stated variable
name, recorded in `DECISIONS.md` ADR-058 -- ADR-019's *mechanism* (gitignored
`.env`, read from the environment/file rather than a literal, fail loudly if
absent) is unchanged; only the literal variable name differs from what was
originally written down. The key is read from that file in-process, in
`_load_api_key` below, and is never logged, printed, or placed in an
exception message anywhere in this module.

**Non-blocking `poll()`, honestly.** `CommandSource.poll()` (`source.py`)
must not block, and `CONSTRAINTS.md:40` calls voice input "non-blocking".
A Speechmatics batch job genuinely takes seconds (submit -> poll status ->
fetch transcript) -- there is no way to make that instantaneous. This
class resolves the tension the way `source.py`'s own docstring anticipates
("waiting must happen elsewhere, e.g. eagerly at construction time") and
the way `TextCommandSource` already does for its own (much cheaper) I/O
(`text_source.py`'s docstring): **the entire Speechmatics round trip runs
inside `__init__`, eagerly, once.** `__init__` blocks; `poll()` never does
-- by the time anything can call `poll()`, transcription has already
succeeded (and the `CommandEvent` is sitting in `self._event`) or `__init__`
has already raised a `VoiceTranscriptionError` subclass and no instance
exists to call `poll()` on. `poll()` itself is just a one-slot buffer pop,
exactly like `TextCommandSource`'s queue pop, just sized to one event.
The alternative (kick off the job in `__init__`, return `None` from
`poll()` until it completes) would need this class to hold a background
thread or to be polled repeatedly by the caller's own loop; nothing in this
demo's control flow polls a `CommandSource` more than once, so that shape
would just move the wait into "the caller loops on `poll()` until non-None"
-- still a wait, just relabelled. The chosen shape is simpler and matches
the pattern this codebase already uses.

**Execution location.** `ARCHITECTURE.md:131` ("CommandSource: voice | only
here (laptop) | never [bm-ptl]") and `CONSTRAINTS.md:40` describe voice as
laptop-side. This module itself has no platform check -- it is pure stdlib
(`urllib`, `wave`, `json`) plus a network call, and runs wherever Python and
internet access exist. The demo entry point (`run_demo.py --voice`) is, by a
deliberate and disclosed exception recorded in `DECISIONS.md`/`ARCHITECTURE.md`
ADR-058, exercised end-to-end on bm-ptl instead, because MuJoCo (ADR-020)
only runs there and shuttling a transcript between machines for every run
was judged not worth the friction for this bonus feature. See ADR-058 for
the full reasoning; this docstring only flags that the deviation exists.

**Operating point: "enhanced", not "standard".** A second disclosed
deviation, alongside the execution-location one above, both recorded under
the same ADR-058. `CONSTRAINTS.md:40` says "Standard model"; `_submit_job`
below actually requests Speechmatics' `operating_point="enhanced"`. Measured
directly (`docs/hardware/voice-transcription-probe.md`): on this module's
own demo phrase, "Give the fork to arm B", `operating_point="standard"`
transcribed the trailing single-letter arm ID as the word "be" on every
audio variant tried (a plain reading, and a paused/spelled-out one), which
`RuleGrounder`'s `to\s+arm\s+[ab]$` pattern cannot match; `"enhanced"` got it
right immediately. Both operating points are Speechmatics' own generic
hosted service -- neither is a custom-trained model -- so this does not
touch `CONSTRAINTS.md:45`'s "no custom novel architectures" line; the only
real cost is Speechmatics' own higher usage cost for the "enhanced" tier.

**Errors never leak raw exceptions.** Every failure mode this class can hit
-- missing/blank key, rejected auth, unreachable host, a job that never
finishes, a job that finishes with no words in it, or a file that is not a
readable WAV -- is caught internally and re-raised as one of the
`VoiceTranscriptionError` subclasses below. No `urllib.error.*`,
`wave.Error`, `json.JSONDecodeError`, `socket.*`, or other raw
library/stdlib exception is ever allowed to escape this module.
"""

from __future__ import annotations

import io
import json
import time
import urllib.error
import urllib.request
import uuid
import wave
from pathlib import Path

from bimanual.command.events import CommandEvent
from bimanual.command.source import CommandSource

# ---------------------------------------------------------------------------
# Speechmatics batch (REST) API -- https://docs.speechmatics.com/api-ref/batch
# Three calls, in order: POST a job (multipart: audio + JSON config), GET the
# job's status until it is "done" (or "rejected"), then GET the transcript.
# ---------------------------------------------------------------------------
_SPEECHMATICS_BASE_URL = "https://asr.api.speechmatics.com/v2"
_JOBS_URL = f"{_SPEECHMATICS_BASE_URL}/jobs"

#: The `.env` variable name this repo's actual credential uses (see this
#: module's docstring -- a disclosed deviation from ADR-019's originally
#: written `SPEECHMATICS_API_KEY`, recorded in DECISIONS.md ADR-058).
ENV_VAR_NAME = "ai_infra"

#: How long (seconds) a single HTTP request may take before this module
#: treats it as unreachable, and how long/how often this module polls a
#: submitted job before giving up. "Standard model" (`CONSTRAINTS.md:40`)
#: on a short command-length clip is normally done in a few seconds; these
#: are generous, not tight, budgets.
_HTTP_TIMEOUT_S = 15.0
DEFAULT_JOB_TIMEOUT_S = 60.0
DEFAULT_POLL_INTERVAL_S = 2.0


# ---------------------------------------------------------------------------
# Distinct, actionable exceptions. A caller that only wants "did voice work"
# can catch the base class; a caller that wants to react differently to,
# say, a missing key versus a network outage catches the specific subclass.
# ---------------------------------------------------------------------------
class VoiceTranscriptionError(RuntimeError):
    """Base class for every voice-transcription failure this module raises.

    See this module's docstring, "Errors never leak raw exceptions".
    """


class MissingApiKeyError(VoiceTranscriptionError):
    """`.env` is missing, or its `ai_infra` line is absent/blank."""


class AuthenticationError(VoiceTranscriptionError):
    """Speechmatics rejected the API key (HTTP 401/403)."""


class NetworkUnreachableError(VoiceTranscriptionError):
    """The Speechmatics host could not be reached (DNS, connect, timeout)."""


class JobTimeoutError(VoiceTranscriptionError):
    """The batch transcription job did not reach "done" within the budget."""


class EmptyTranscriptError(VoiceTranscriptionError):
    """Speechmatics finished the job but recognized no words at all."""


class InvalidAudioError(VoiceTranscriptionError):
    """The file is missing/unreadable, not a valid WAV, or Speechmatics
    itself rejected the submitted audio as malformed."""


# ---------------------------------------------------------------------------
# .env reading -- deliberately no third-party dotenv dependency (PLAN.md
# task brief: prefer stdlib-only; a single `KEY=VALUE` line does not need a
# library). Read-only, and the value is returned to the caller, never
# printed or logged by this function or anything that calls it.
# ---------------------------------------------------------------------------
def _default_env_path() -> Path:
    """Repo-root `.env` path, computed the same way `run_demo.py` computes
    its own `REPO_ROOT` (`run_demo.py:79`): this file lives at
    `src/bimanual/command/voice_source.py`, four directory levels below the
    repo root, so walking up from `__file__` (not the process's current
    working directory) keeps the lookup stable regardless of where `python`
    was invoked from.
    """
    return Path(__file__).resolve().parents[3] / ".env"


def _read_env_var(env_path: Path, var_name: str) -> str | None:
    """Return `var_name`'s value from a simple `KEY=VALUE`-per-line file.

    Returns None if the file does not exist, the variable is absent, or its
    value is blank -- all three are "not configured" as far as the caller
    is concerned. Comments (`#...`) and blank lines are skipped. One layer
    of surrounding matching quotes is stripped so `KEY="value"` and
    `KEY=value` behave the same.
    """
    if not env_path.exists():
        return None
    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        if key.strip() != var_name:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        return value or None
    return None


# ---------------------------------------------------------------------------
# WAV validation -- a cheap local check (no network call spent) that turns a
# missing/empty/corrupt file into InvalidAudioError before this module ever
# reaches for the network.
# ---------------------------------------------------------------------------
def _validate_wav_bytes(audio_bytes: bytes, audio_file: Path) -> None:
    try:
        with wave.open(io.BytesIO(audio_bytes), "rb") as wf:
            n_frames = wf.getnframes()
    except (wave.Error, EOFError) as exc:
        raise InvalidAudioError(
            f"{audio_file} is not a readable WAV file: {type(exc).__name__}: {exc}"
        ) from exc
    if n_frames <= 0:
        raise InvalidAudioError(f"{audio_file} is a valid WAV container but contains 0 audio frames.")


# ---------------------------------------------------------------------------
# Multipart/form-data body construction (RFC 7578). Written by hand against
# the stdlib rather than pulling in `requests` -- the pre-flight already
# proved plain `urllib` reaches this API (see task brief), and this is the
# one piece `urllib` does not build for you.
# ---------------------------------------------------------------------------
def _build_multipart_body(boundary: str, filename: str, audio_bytes: bytes, config: dict) -> bytes:
    crlf = b"\r\n"
    parts: list[bytes] = [
        f"--{boundary}".encode(),
        b'Content-Disposition: form-data; name="config"',
        b"",
        json.dumps(config).encode("utf-8"),
        f"--{boundary}".encode(),
        f'Content-Disposition: form-data; name="data_file"; filename="{filename}"'.encode(),
        b"Content-Type: audio/wav",
        b"",
        audio_bytes,
        f"--{boundary}--".encode(),
        b"",  # trailing element so crlf.join leaves a terminating CRLF
    ]
    return crlf.join(parts)


class VoiceCommandSource(CommandSource):
    """Speechmatics-backed voice CommandSource (M15, ADR-002/ADR-058).

    Args:
        audio_file: Path to a WAV file to transcribe. Read and validated
            (never merely trusted) at construction time; see this module's
            docstring for why transcription itself also happens here.
        env_path: Optional override for where to read the `ai_infra`
            credential from. Defaults to the repo-root `.env`
            (`_default_env_path`). Exists mainly so tests can point this at
            a temporary `.env` without touching the real one.
        timeout_s: Maximum time to wait for the Speechmatics job to reach
            "done" before raising `JobTimeoutError`.
        poll_interval_s: Delay between job-status polls.

    Raises:
        One of the `VoiceTranscriptionError` subclasses above if anything
        goes wrong -- construction either fully succeeds (an instance whose
        first `poll()` will return the transcribed `CommandEvent`) or does
        not happen at all. There is no partially-constructed state.
    """

    #: Distinguishes this from "text_cli"/"text_file"/"text_stdin"
    #: (`text_source.py`) and from the old stub's "voice_stub" in
    #: `CommandEvent.source_id` / `manifest.json` provenance.
    SOURCE_ID = "voice_speechmatics"

    def __init__(
        self,
        audio_file: Path,
        *,
        env_path: Path | None = None,
        timeout_s: float = DEFAULT_JOB_TIMEOUT_S,
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
    ) -> None:
        self._audio_file = audio_file
        self._closed = False
        self._emitted = False
        # Eager, blocking transcription -- see this module's docstring,
        # "Non-blocking poll(), honestly".
        self._event = self._transcribe_eagerly(audio_file, env_path, timeout_s, poll_interval_s)

    # -- poll()/close(): the CommandSource contract itself -------------
    def poll(self) -> CommandEvent | None:
        """Return the transcribed CommandEvent once, then None.

        Never touches the network -- see this module's docstring. This is
        a one-slot version of `TextCommandSource.poll()`'s queue pop
        (`text_source.py`): both just hand back work that was already done
        at construction time.
        """
        if self._closed or self._emitted:
            return None
        self._emitted = True
        return self._event

    def close(self) -> None:
        # Idempotent per the CommandSource contract (source.py); no
        # resources are held open past __init__ (the HTTP connections used
        # to submit/poll/fetch are each already closed by their own `with
        # urllib.request.urlopen(...)` block), so this is just a flag.
        self._closed = True

    # -- internals -------------------------------------------------------
    def _transcribe_eagerly(
        self,
        audio_file: Path,
        env_path: Path | None,
        timeout_s: float,
        poll_interval_s: float,
    ) -> CommandEvent:
        api_key = self._load_api_key(env_path)

        try:
            audio_bytes = audio_file.read_bytes()
        except OSError as exc:
            raise InvalidAudioError(f"Could not read {audio_file}: {exc}") from exc
        _validate_wav_bytes(audio_bytes, audio_file)

        job_id = self._submit_job(audio_bytes, audio_file.name, api_key)
        self._await_job_done(job_id, api_key, timeout_s, poll_interval_s)
        text, confidence = self._fetch_transcript(job_id, api_key)

        if not text.strip():
            raise EmptyTranscriptError(
                f"Speechmatics job {job_id} finished but the transcript contains no words "
                f"(audio file: {audio_file})."
            )

        return CommandEvent.now(
            text=text,
            source_id=self.SOURCE_ID,
            confidence=confidence,
            raw_meta={
                "audio_file": str(audio_file),
                "engine": "speechmatics",
                "wired": True,  # unlike the old stub -- see module docstring
                "job_id": job_id,
            },
        )

    @staticmethod
    def _load_api_key(env_path: Path | None) -> str:
        path = env_path if env_path is not None else _default_env_path()
        value = _read_env_var(path, ENV_VAR_NAME)
        if not value:
            raise MissingApiKeyError(
                f"No `{ENV_VAR_NAME}` value found in {path}. Create or edit that "
                f"gitignored file with a line `{ENV_VAR_NAME}=<your Speechmatics API "
                "key>` -- see README.md's Voice Input section for setup."
            )
        return value

    def _call(self, request: urllib.request.Request) -> dict:
        """Send `request`, translate transport failures, return parsed JSON.

        The one place every Speechmatics HTTP call funnels through, so the
        HTTPError-vs-URLError-vs-timeout-vs-bad-JSON translation into this
        module's own exception types lives in exactly one spot.
        """
        try:
            with urllib.request.urlopen(request, timeout=_HTTP_TIMEOUT_S) as response:
                raw = response.read()
        except urllib.error.HTTPError as exc:
            # HTTPError is also a URLError subclass, so it must be caught
            # first -- it carries an HTTP status code the plain URLError
            # branch below does not.
            if exc.code in (401, 403):
                raise AuthenticationError(
                    f"Speechmatics rejected the API key (HTTP {exc.code}). Check the "
                    f"`{ENV_VAR_NAME}` value in .env -- see README.md's Voice Input section."
                ) from exc
            raise VoiceTranscriptionError(
                f"Speechmatics returned HTTP {exc.code} for {request.full_url}."
            ) from exc
        except urllib.error.URLError as exc:
            # DNS failure, connection refused, TLS failure, etc. -- the
            # host was not reachable at all.
            raise NetworkUnreachableError(
                f"Could not reach Speechmatics ({request.full_url}): {exc.reason}."
            ) from exc
        except (TimeoutError, OSError) as exc:
            raise NetworkUnreachableError(
                f"Network error talking to Speechmatics ({request.full_url}): {exc}."
            ) from exc

        try:
            return json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise VoiceTranscriptionError(
                "Speechmatics returned a response this client could not parse as JSON."
            ) from exc

    def _submit_job(self, audio_bytes: bytes, filename: str, api_key: str) -> str:
        boundary = uuid.uuid4().hex
        config = {
            "type": "transcription",
            # operating_point="enhanced", NOT "standard" -- a disclosed,
            # measured deviation from CONSTRAINTS.md:40's "Standard model"
            # wording, recorded in DECISIONS.md/ARCHITECTURE.md ADR-058.
            # Both operating points are Speechmatics' own off-the-shelf
            # hosted service (neither is a custom-trained model, so this
            # does not touch CONSTRAINTS.md:45's "no custom novel
            # architectures" line) -- the difference is accuracy tier only.
            # Measured directly (`docs/hardware/voice-transcription-probe.md`):
            # on the grammar's own "Give the fork to arm B" test phrase,
            # operating_point="standard" transcribed the single-letter arm
            # ID as the word "be" ('Give the fork to arm be') on every
            # phrasing/pause variant tried, which RuleGrounder's
            # `to\s+arm\s+[ab]$` pattern cannot match (`rule_grounder.py`);
            # operating_point="enhanced" transcribed it correctly as the
            # single letter ('Give the fork to arm B') on the first try.
            # `punctuation_overrides` disables punctuation insertion, which
            # separately fixes a second failure mode: the default
            # punctuation model split this same sentence into two
            # ('Give the fork to arm. Be.'), which would not match the
            # grammar's single-clause pattern either way.
            "transcription_config": {
                "language": "en",
                "operating_point": "enhanced",
                "punctuation_overrides": {"permitted_marks": []},
            },
        }
        body = _build_multipart_body(boundary, filename, audio_bytes, config)
        request = urllib.request.Request(
            _JOBS_URL,
            data=body,
            method="POST",
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": f"multipart/form-data; boundary={boundary}",
            },
        )
        payload = self._call(request)
        job_id = payload.get("id")
        if not job_id:
            raise VoiceTranscriptionError(
                f"Speechmatics job submission did not return a job id: {payload!r}"
            )
        return job_id

    def _await_job_done(self, job_id: str, api_key: str, timeout_s: float, poll_interval_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        status_url = f"{_JOBS_URL}/{job_id}"
        while True:
            request = urllib.request.Request(status_url, headers={"Authorization": f"Bearer {api_key}"})
            payload = self._call(request)
            job = payload.get("job", payload)
            status = job.get("status")

            if status == "done":
                return
            if status == "rejected":
                errors = job.get("errors") or job.get("error") or "no further detail returned"
                raise InvalidAudioError(
                    f"Speechmatics rejected job {job_id} (likely malformed/unsupported audio): {errors}"
                )
            if time.monotonic() >= deadline:
                raise JobTimeoutError(
                    f"Speechmatics job {job_id} did not finish within {timeout_s:.0f}s "
                    f"(last status: {status!r})."
                )
            time.sleep(poll_interval_s)

    def _fetch_transcript(self, job_id: str, api_key: str) -> tuple[str, float | None]:
        url = f"{_JOBS_URL}/{job_id}/transcript?format=json-v2"
        request = urllib.request.Request(url, headers={"Authorization": f"Bearer {api_key}"})
        payload = self._call(request)

        # Speechmatics' json-v2 transcript is a list of per-token results,
        # each either a "word" (gets a leading space, unless it is the
        # first token) or "punctuation" (attaches directly to the previous
        # token, no leading space) -- reconstructing readable text from
        # this list is exactly what `format=txt` would do server-side; we
        # do it ourselves here so the SAME response also yields a mean
        # per-word confidence for `CommandEvent.confidence`.
        pieces: list[str] = []
        confidences: list[float] = []
        for result in payload.get("results", []):
            alternatives = result.get("alternatives") or []
            if not alternatives:
                continue
            content = alternatives[0].get("content", "")
            conf = alternatives[0].get("confidence")
            if isinstance(conf, (int, float)):
                confidences.append(float(conf))
            attaches_to_previous = (
                result.get("type") == "punctuation" or result.get("attaches_to") == "previous"
            )
            if pieces and not attaches_to_previous:
                pieces.append(" ")
            pieces.append(content)

        text = "".join(pieces).strip()
        confidence = (sum(confidences) / len(confidences)) if confidences else None
        return text, confidence
