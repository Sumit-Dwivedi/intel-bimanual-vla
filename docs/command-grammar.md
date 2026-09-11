# Command grammar — what `RuleGrounder` accepts

**Scope of this document, stated plainly.** This documents the grammar
`RuleGrounder` (`src/bimanual/language/rule_grounder.py`) **accepts as
input text**, i.e. what a human or a `CommandSource` can type and have
grounded into a `TaskPlan`. It is **not** a description of the demo's full
task sequence. ADR-011's core scene sequence (`open_drawer` → pick/place
fork → pick/place spoon → pick/place plate → `handoff` mug → `pour`) is
*richer* than any single command below — it is what the scripted
controller (M06) executes across the whole demo, built from several
grounded commands (or one long one), not a claim about what one sentence
must contain. Do not read this file as a claim about what the demo
performs; read `ARCHITECTURE.md` ADR-011 for that.

## Skill vocabulary (ADR-011's core sequence)

`open_drawer`, `close_drawer`, `pick`, `place`, `handoff`, `pour`.
`close_drawer` is included because its pattern is the mirror of
`open_drawer` and fell out for free — it is not required by any
done-when criterion and nothing in the demo path currently emits it.

## Object vocabulary and synonyms

`plate`, `mug`, `drawer`, `spoon`, `fork`, `bottle`, `table`.

`cup` and `glass` are synonyms for `mug` — both canonicalize to
`target_object="mug"` on the resulting `SkillCall`. No other synonyms
exist. An optional `top`/`bottom` modifier on `drawer` is accepted but
ignored (canonical object is always `"drawer"`); brief p1 only ever
refers to "the top drawer," and the scene has exactly one.

## How a command becomes a plan

1. The command text is split into **clauses** on commas and the word
   "and" (e.g. `"Open the drawer, pick up the plate with arm A"` and
   `"Pick up the mug and place it on the table"` both split into two
   clauses).
2. Each clause is matched, independently and in the order it appears,
   against exactly one of the six patterns below. A clause must match
   completely (start to end) — a clause with an unrecognized verb, an
   unrecognized noun, or extra trailing words does not partially match.
3. The `SkillCall`s produced, in clause order, form the `TaskPlan`.

## Arm assignment (never `None` — PLAN.md M05 done-when 3)

- **If a clause names an arm explicitly** (`"... with arm A"` /
  `"... with arm B"`), that arm is used, case-insensitively.
- **If a clause does not name an arm, the default is arm A.** This is a
  global rule with no exceptions beyond the one below — it does not try
  to infer which arm is "logically" already holding an object (that would
  require `SceneBelief`, which `RuleGrounder` does not consume; see
  "SceneBelief" below). A judge can predict the arm on any clause without
  an explicit `"with arm ..."` by this one rule: **it is A.**
- **Exception — `handoff` only.** The destination is mandatory
  (`"... to arm X"`). If the origin is not given (`"... from arm Y"`),
  it defaults to **the other of the two arms** — with exactly two arms,
  "not X" is unambiguous. Example: `"Pass the bottle to arm A"` implies
  `from_arm="B"`.

## Pronoun resolution ("it")

`place`, `pour`, and `handoff` accept `"it"` in place of an explicit
object noun. `"it"` resolves to the most recently mentioned object in an
earlier clause of the **same command** (e.g. `"pick up the mug ... place
it on the table"` → `it` = mug). A pronoun with no earlier referent in the
same command (e.g. a command that opens with `"Place it on the
table."`) cannot be resolved and raises `UngroundedCommandError` — see
"Failure behaviour" below.

## Accepted patterns, by skill

Below, `<obj>` is one of the pickable nouns (`plate | mug | cup | glass |
spoon | fork | bottle`) unless narrower; `<mug-word>` is `mug | cup |
glass`; `[with arm A|B]` and `[from arm A|B]` denote the optional,
arm-defaulted phrases described above.

| Skill | Pattern | Example |
|---|---|---|
| `open_drawer` | `open [the] [top\|bottom] drawer [with arm A\|B]` | "Open the top drawer." |
| `close_drawer` | `close [the] [top\|bottom] drawer [with arm A\|B]` | "Close the drawer with arm B." |
| `pick` | `(pick up\|pick\|grab\|fetch\|get\|take) [the] <obj> [with arm A\|B]` | "Grab the fork with arm A." |
| `place` | `(place\|put\|set\|drop) (it\|[the] <obj>) (on\|in) [the] table [with arm A\|B]` | "Place the spoon on the table with arm B." |
| `handoff` | `(hand off\|handoff\|pass\|give) (it\|[the] <obj>) [from arm A\|B] to arm A\|B` | "Hand off the mug from arm B to arm A." |
| `pour` | `pour [water] into (it\|[the] <mug-word>) [with arm A\|B]` | "Pour water into the mug with arm A." |

Verb synonyms are exactly the alternations listed above (no others):
`pick` accepts `pick up`, `pick`, `grab`, `fetch`, `get`, `take`; `place`
accepts `place`, `put`, `set`, `drop`; `handoff` accepts `hand off` /
`handoff`, `pass`, `give`.

## The brief's verbatim example command

> "Open the top drawer, pick up the plate with arm A, place it on the
> table, pick up the mug with arm B, pour water into the mug with arm A."

Parses to:

| # | skill | arm | target_object | params |
|---|---|---|---|---|
| 1 | `open_drawer` | A | drawer | `{}` |
| 2 | `pick` | A | plate | `{}` |
| 3 | `place` | A | plate | `{"destination": "table"}` |
| 4 | `pick` | B | mug | `{}` |
| 5 | `pour` | A | mug | `{"source": "bottle"}` |

Note step 1's arm: the brief's sentence never names an arm for opening the
drawer, so the default-arm rule fires and it is grounded as arm A.

## `pour` is a pose, not a liquid transfer

Per ADR-011 and ADR-017, `pour` grounds to a `SkillCall(skill="pour", ...)`
that M06's executor will later realize as a **tilt-and-position motion**:
arm B holds the mug, arm A brings the bottle over it and holds a tilt
pose. **No fluid, particle, or volume is simulated or implied.** This
grammar's use of the English word "pour" and the phrase "pour water into"
should not be read as a claim that water moves in the simulation — it is
the vocabulary the brief itself uses (p1), grounded to a pose-achievement
skill.

## Failure behaviour — two distinct "nothing happens" cases

1. **Empty or whitespace-only command text** → `ground()` returns an
   **empty** `TaskPlan` (`skills == []`) and does **not** raise. Nothing
   was asked, so there is nothing to fail at. This matches M04's
   `TextCommandSource`, which deliberately passes empty and
   whitespace-only text through unvalidated
   (`src/bimanual/command/text_source.py`) — the Grounder is where that
   judgement is made, and "no-op" is the judgement for no text.
2. **Non-empty command text that cannot be fully grounded** — an
   unrecognized verb, an unrecognized noun, a pronoun with no earlier
   referent, or content that is entirely punctuation once split into
   clauses — raises a typed `UngroundedCommandError`
   (`src/bimanual/language/grounder.py`). This is a hard failure, on
   purpose: `RuleGrounder` never returns a plan containing only the
   clauses it understood and silently drops the rest. A command with four
   good clauses and one bad one raises for the whole command, not a
   four-step plan.

## `SceneBelief` (not consumed today)

`ARCHITECTURE.md`'s component-contract table fixes the interface as
`Grounder.ground(CommandEvent, SceneBelief) -> TaskPlan`. `SceneBelief` is
`PerceptionBackend`'s output (M10, not built yet). `RuleGrounder.ground()`
accepts it as an optional second parameter defaulting to `None` purely so
this signature already matches the architecture; it is not read or used
anywhere in this module. A future `VlmGrounder` (ADR-003's bounded
stretch item) is the one that would use it, e.g. to disambiguate "which
plate" from what is actually on the table.

## Low-confidence commands

`CommandEvent.confidence` (populated by voice sources, always `None` for
text sources) is not part of this grammar and never blocks grounding.
If `confidence` is present and below `RuleGrounder.LOW_CONFIDENCE_THRESHOLD`
(`0.5`), a warning is logged and the command is grounded exactly as it
would be otherwise.
