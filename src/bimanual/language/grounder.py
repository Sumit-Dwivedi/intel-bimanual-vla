"""Abstract Grounder: text (+ optional scene belief) -> TaskPlan (M05).

Per ARCHITECTURE.md's component-contract table (section 1):

    Grounder.ground(CommandEvent, SceneBelief) -> TaskPlan

`RuleGrounder` (this package's deterministic floor, ADR-003) and any future
`VlmGrounder` (a bounded stretch item, never a demo-path dependency) both
implement this one interface, so the Coordinator (M06+) never branches on
which concrete grounder it was handed -- the same Liskov pattern M04 used
for `CommandSource` (see `command/source.py`'s docstring).

`SceneBelief` is `PerceptionBackend`'s output (M10, not built yet -- see
ARCHITECTURE.md section 1's component table and PLAN.md's M10 entry). It is
accepted here as an optional second parameter, defaulting to `None`, purely
so this signature already matches the architecture and nothing has to
change when M10 lands. `RuleGrounder` ignores it completely today: it has
no visual grounding, no "which plate" disambiguation, nothing. That
disambiguation job is exactly what a future `VlmGrounder` would use
`scene_belief` for (ADR-003's option (c)).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from bimanual.command.events import CommandEvent
from bimanual.language.skills import TaskPlan


class UngroundedCommandError(ValueError):
    """Raised when a non-empty command cannot be turned into a TaskPlan.

    This is the "fails loudly" half of M05's contract
    (docs/learn/05-rule-grounder.md: "It fails loudly... A clean refusal
    beats executing three of five steps on stage."). A Grounder must never
    silently drop an unrecognized clause and return a partial plan -- it
    either grounds the whole command or raises this, typed, so a caller
    (Coordinator, or a test) can catch it specifically rather than an
    ambient `Exception`.

    Deliberately NOT raised for empty or whitespace-only `CommandEvent.text`
    -- see `rule_grounder.py`'s module docstring for why that is a
    different case (nothing was asked, vs. something was asked and could
    not be honoured).
    """

    def __init__(self, command_text: str, reason: str) -> None:
        self.command_text = command_text
        self.reason = reason
        super().__init__(
            f"Could not ground command {command_text!r}: {reason}"
        )


class Grounder(ABC):
    """Strategy interface: natural-language command -> ordered TaskPlan.

    Concrete implementations: `RuleGrounder` (this module's sibling,
    `rule_grounder.py`) today; a `VlmGrounder` stretch item later
    (ADR-003), behind this same interface.
    """

    @abstractmethod
    def ground(
        self,
        command: CommandEvent,
        scene_belief: object | None = None,
    ) -> TaskPlan:
        """Turn one CommandEvent into an ordered TaskPlan of SkillCalls.

        Args:
            command: The instruction to ground. `command.text` is
                unvalidated and unstripped (M04's `TextCommandSource`
                passes it through verbatim) -- implementations must handle
                empty and whitespace-only text themselves.
            scene_belief: Optional current `SceneBelief` (M10's output,
                not built yet). `None` on every call until M10 lands and a
                caller starts passing one. `RuleGrounder` ignores this
                argument entirely; see this module's top docstring.

        Returns:
            A `TaskPlan`. Empty (`TaskPlan(skills=[])`) when `command.text`
            was empty or whitespace-only. Otherwise a non-empty, fully
            arm-assigned plan.

        Raises:
            UngroundedCommandError: `command.text` was non-empty but could
                not be fully grounded (an unrecognized clause, or a
                pronoun with no earlier referent). Never returns a partial
                plan instead of raising.
        """
        raise NotImplementedError
