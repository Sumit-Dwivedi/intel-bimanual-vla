# Voice transcription probe -- `operating_point` and punctuation, M15/ADR-058

Ad hoc probe run while wiring `VoiceCommandSource`
(`src/bimanual/command/voice_source.py`), on the laptop (this module is pure
stdlib -- `urllib`/`wave`/`json` -- and needs only network access, so it does
not require bm-ptl or MuJoCo to exercise on its own). Answers one question:
does Speechmatics' `operating_point="standard"` (`CONSTRAINTS.md:40`, "Standard
model") reliably transcribe the demo phrase this module's tests and
`run_demo.py --voice` use, **"Give the fork to arm B"**, into text
`RuleGrounder`'s handoff pattern (`rule_grounder.py`'s `_R_HANDOFF`,
`to\s+arm\s+(?P<to_arm>[ab])$`) actually accepts?

## Audio

Synthesized locally with Windows' built-in `System.Speech.Synthesis`
(SAPI), no physical microphone involved -- a legitimate way to produce a
real, non-placeholder WAV file to send to a real speech-to-text API without
requiring live mic capture on this laptop. Two variants of the "B" at the
end were tried:

1. Plain: `SpeechSynthesizer.Speak("Give the fork to arm B")`.
2. Paused + spelled: `PromptBuilder` with `AppendText("Give the fork to arm")`,
   `AppendBreak(PromptBreak.Medium)`, `AppendTextWithHint("B", SayAs.SpellOut)`
   -- deliberately trying to make the "B" sound as unambiguously like an
   isolated spelled letter as this TTS engine can produce, in case the
   plain phrasing's failure was an artifact of "arm" and "B" running
   together acoustically rather than of the STT model's own text
   formatting choice.

## Results (`format=json-v2`, results joined into plain text; `sounds_like`
custom vocabulary biasing for "B" tried too, no effect)

| `operating_point` | punctuation | audio variant | Transcript |
|---|---|---|---|
| `standard` | default | plain | `Give the fork to arm. Be.` (split into two sentences) |
| `standard` | disabled (`permitted_marks: []`) | plain | `Give the fork to arm be` |
| `standard` | disabled | plain, + `additional_vocab: [{"content":"B","sounds_like":["be","bee"]}]` | `Give the fork to arm be` (no change) |
| `standard` | disabled | paused + spelled-out | `Give the fork to arm be` (no change) |
| `enhanced` | disabled | plain | **`Give the fork to arm B`** (correct, first try) |

A related finding for "arm A" (not the phrase this module's tests/demo use,
tried only out of curiosity while probing): both operating points merged it
into the single word "Army" ("Give the fork to Army") -- the vowel sound
blends with "arm" acoustically before the STT model ever gets a chance to
segment it, so this is a harder problem than the "B" case and additional
vocabulary/config changes were not explored further for it. This is a
reason (not the only one) `run_demo.py --voice` and this module's own tests
standardize on the "to arm B" phrasing specifically -- it is the one
verified to round-trip correctly.

## Conclusion / decision

`operating_point="enhanced"` is what `voice_source.py`'s `_submit_job`
actually requests, a disclosed deviation from `CONSTRAINTS.md:40`'s
"Standard model" wording, recorded in `DECISIONS.md`/`ARCHITECTURE.md`
ADR-058 alongside the (separate) execution-location deviation. Both
operating points are Speechmatics' own generic hosted service -- neither
is a custom-trained or fine-tuned model -- so this does not touch
`CONSTRAINTS.md:45`'s "no custom novel architectures" line; the only real
difference is Speechmatics' own accuracy/cost tier, and the accuracy
difference above is measured, not assumed.

Punctuation is also disabled (`punctuation_overrides: {"permitted_marks":
[]}`) for a second, independent reason: the default punctuation model
split the plain "standard"-tier transcript into two sentences
("...arm. Be."), which `RuleGrounder` (one pattern per whole clause) would
not match even if the letter itself had come out right.
