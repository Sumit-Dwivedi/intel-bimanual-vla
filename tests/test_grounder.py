"""Tests for RuleGrounder (PLAN.md M05).

Covers M05's three done-when criteria:
  1. The brief's verbatim example command parses to a TaskPlan whose
     skills, arms and target objects match docs/command-grammar.md.
  2. At least 8 paraphrases parse correctly; at least 3 unsupported
     commands raise UngroundedCommandError rather than a partial plan.
  3. Every SkillCall in every successful parse has a non-None arm.

Plus the two cases M05's task brief calls out explicitly: empty /
whitespace-only input yields an empty TaskPlan without raising (distinct
from "unsupported"), and cup/glass both resolve to "mug".
"""

from __future__ import annotations

import pytest

from bimanual.command import CommandEvent
from bimanual.language import RuleGrounder, SkillCall, TaskPlan, UngroundedCommandError


def _event(text: str, confidence: float | None = None) -> CommandEvent:
    return CommandEvent.now(text=text, source_id="test", confidence=confidence)


def _as_tuples(plan: TaskPlan) -> list[tuple[str, str, str]]:
    """(skill, arm, target_object) triples, for compact assertions."""
    return [(s.skill, s.arm, s.target_object) for s in plan.skills]


# ---------------------------------------------------------------------------
# Done-when 1: the brief's verbatim example command
# ---------------------------------------------------------------------------

BRIEF_COMMAND = (
    "Open the top drawer, pick up the plate with arm A, place it on the "
    "table, pick up the mug with arm B, pour water into the mug with arm A."
)


def test_brief_verbatim_command_parses_to_expected_plan():
    grounder = RuleGrounder()
    plan = grounder.ground(_event(BRIEF_COMMAND))

    assert _as_tuples(plan) == [
        ("open_drawer", "A", "drawer"),
        ("pick", "A", "plate"),
        ("place", "A", "plate"),
        ("pick", "B", "mug"),
        ("pour", "A", "mug"),
    ]
    # Explicit params match docs/command-grammar.md too.
    assert plan.skills[2].params == {"destination": "table"}
    assert plan.skills[4].params == {"source": "bottle"}
    for skill_call in plan.skills:
        assert skill_call.arm is not None


# ---------------------------------------------------------------------------
# Done-when 2a: at least 8 paraphrases parse correctly.
# ---------------------------------------------------------------------------

PARAPHRASES: list[tuple[str, list[tuple[str, str, str]]]] = [
    ("Open the drawer.", [("open_drawer", "A", "drawer")]),
    ("Pick up the plate with arm A.", [("pick", "A", "plate")]),
    (
        "Pick up the cup with arm B and place it on the table.",
        [("pick", "B", "mug"), ("place", "A", "mug")],
    ),
    ("Grab the fork with arm A.", [("pick", "A", "fork")]),
    ("Place the spoon on the table with arm B.", [("place", "B", "spoon")]),
    (
        "Hand off the mug from arm B to arm A.",
        [("handoff", "A", "mug")],
    ),
    ("Pass the bottle to arm A.", [("handoff", "A", "bottle")]),
    ("Pour water into the mug with arm A.", [("pour", "A", "mug")]),
    ("Pick up the glass with arm B.", [("pick", "B", "mug")]),
    (
        "Fetch the spoon and place it on the table.",
        [("pick", "A", "spoon"), ("place", "A", "spoon")],
    ),
    (
        "Open the top drawer and pick up the plate with arm A.",
        [("open_drawer", "A", "drawer"), ("pick", "A", "plate")],
    ),
]


def test_at_least_eight_paraphrases_supported():
    assert len(PARAPHRASES) >= 8


@pytest.mark.parametrize("text,expected", PARAPHRASES, ids=[p[0] for p in PARAPHRASES])
def test_paraphrase_parses_correctly(text, expected):
    grounder = RuleGrounder()
    plan = grounder.ground(_event(text))
    assert _as_tuples(plan) == expected


def test_handoff_from_arm_default_is_the_other_arm():
    """'Pass ... to arm A' with no explicit 'from' -> from_arm defaults to B."""
    grounder = RuleGrounder()
    plan = grounder.ground(_event("Pass the bottle to arm A."))
    assert plan.skills[0].params["from_arm"] == "B"

    plan_b = grounder.ground(_event("Pass the bottle to arm B."))
    assert plan_b.skills[0].params["from_arm"] == "A"


# ---------------------------------------------------------------------------
# Done-when 2b: at least 3 unsupported commands raise UngroundedCommandError.
# ---------------------------------------------------------------------------

UNSUPPORTED_COMMANDS = [
    "Teleport the flibbertigibbet to the moon",
    "Cook dinner for the arms.",
    "Pick up the plate and dance with arm A.",
    "Do a backflip.",
]


def test_at_least_three_unsupported_commands_covered():
    assert len(UNSUPPORTED_COMMANDS) >= 3


@pytest.mark.parametrize("text", UNSUPPORTED_COMMANDS)
def test_unsupported_command_raises_typed_error(text):
    grounder = RuleGrounder()
    with pytest.raises(UngroundedCommandError):
        grounder.ground(_event(text))


def test_unsupported_command_never_produces_a_partial_plan():
    """A command with one good clause and one bad clause must raise, not
    silently return a plan containing only the good clause."""
    grounder = RuleGrounder()
    with pytest.raises(UngroundedCommandError):
        grounder.ground(_event("Pick up the plate with arm A and dance."))


def test_pronoun_with_no_earlier_referent_is_ungrounded():
    """'it' with nothing preceding it is a real failure, not a partial plan."""
    grounder = RuleGrounder()
    with pytest.raises(UngroundedCommandError):
        grounder.ground(_event("Place it on the table."))


def test_punctuation_only_nonempty_command_is_ungrounded():
    """Non-whitespace input with no parseable content still raises (it is
    not the same case as an empty/whitespace-only command -- see below)."""
    grounder = RuleGrounder()
    with pytest.raises(UngroundedCommandError):
        grounder.ground(_event(",,,"))


# ---------------------------------------------------------------------------
# Empty / whitespace-only input: empty plan, NOT an UngroundedCommandError.
# ---------------------------------------------------------------------------


def test_empty_string_yields_empty_plan_without_raising():
    grounder = RuleGrounder()
    plan = grounder.ground(_event(""))
    assert isinstance(plan, TaskPlan)
    assert plan.is_empty()
    assert plan.skills == []


def test_whitespace_only_yields_empty_plan_without_raising():
    grounder = RuleGrounder()
    plan = grounder.ground(_event("   \t  "))
    assert plan.is_empty()


# ---------------------------------------------------------------------------
# Synonym handling: cup / glass both mean mug.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("word", ["mug", "cup", "glass"])
def test_mug_synonyms_all_canonicalize_to_mug(word):
    grounder = RuleGrounder()
    plan = grounder.ground(_event(f"Pick up the {word} with arm A."))
    assert plan.skills[0].target_object == "mug"


# ---------------------------------------------------------------------------
# Done-when 3: arm assignment is explicit for every SkillCall, never None.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("text,_", PARAPHRASES + [(BRIEF_COMMAND, None)], ids=lambda v: str(v)[:40])
def test_every_skill_call_has_a_non_none_arm(text, _):
    grounder = RuleGrounder()
    plan = grounder.ground(_event(text))
    for skill_call in plan.skills:
        assert skill_call.arm is not None
        assert skill_call.arm in ("A", "B")


# ---------------------------------------------------------------------------
# Low-confidence hook: grounding still succeeds, it is not a refusal signal.
# ---------------------------------------------------------------------------


def test_low_confidence_still_grounds(caplog):
    grounder = RuleGrounder()
    event = _event("Pick up the plate with arm A.", confidence=0.1)
    with caplog.at_level("WARNING"):
        plan = grounder.ground(event)
    assert _as_tuples(plan) == [("pick", "A", "plate")]
    assert any("low-confidence" in r.message.lower() or "confidence" in r.message.lower() for r in caplog.records)


def test_high_confidence_does_not_warn(caplog):
    grounder = RuleGrounder()
    event = _event("Pick up the plate with arm A.", confidence=0.99)
    with caplog.at_level("WARNING"):
        grounder.ground(event)
    assert len(caplog.records) == 0


def test_none_confidence_does_not_warn(caplog):
    """Text sources leave confidence as None; that must never be treated as
    low confidence (see rule_grounder.py's docstring)."""
    grounder = RuleGrounder()
    event = _event("Pick up the plate with arm A.", confidence=None)
    with caplog.at_level("WARNING"):
        grounder.ground(event)
    assert len(caplog.records) == 0


# ---------------------------------------------------------------------------
# scene_belief is accepted (matches ARCHITECTURE.md's signature) and ignored.
# ---------------------------------------------------------------------------


def test_scene_belief_parameter_is_accepted_and_ignored():
    grounder = RuleGrounder()
    plan_without = grounder.ground(_event("Pick up the plate with arm A."))
    plan_with = grounder.ground(
        _event("Pick up the plate with arm A."), scene_belief={"anything": "goes"}
    )
    assert _as_tuples(plan_without) == _as_tuples(plan_with)
