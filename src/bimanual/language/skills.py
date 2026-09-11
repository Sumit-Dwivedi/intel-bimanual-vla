"""SkillCall and TaskPlan: the typed IR a Grounder produces (M05).

ARCHITECTURE.md's component-contract table (section 1) fixes `SkillCall`'s
fields as `skill`, `arm`, `target_object`, `params: dict`. This module is
data only -- no execution logic lives here. `SkillExecutor` (M06) is what
turns a `SkillCall` into joint targets; this module never imports
`bimanual.control` or `bimanual.sim`.

Arm is a first-class, mandatory field. PLAN.md M05 done-when 3 forbids a
`SkillCall` with `arm=None` -- see `rule_grounder.py`'s docstring for the
deterministic default-arm rule that guarantees this.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class SkillCall:
    """One grounded instruction to run a single skill on a single arm.

    Attributes:
        skill: One of the fixed vocabulary from ADR-011's core sequence:
            "open_drawer", "close_drawer", "pick", "place", "handoff",
            "pour". Not an enum (dataclasses.Enum would be reasonable, but
            a plain str keeps this module dependency-free and keeps the
            vocabulary readable straight out of `docs/command-grammar.md`).
        arm: "A" or "B". Never None -- every SkillCall names the arm that
            performs it (PLAN.md M05 done-when 3). For "handoff", this is
            the *receiving* arm; the *origin* arm is recorded in
            `params["from_arm"]` (there is only one `arm` field on this
            dataclass, and the receiving arm is the one that "does" the
            handoff from the Coordinator's point of view -- it is the arm
            that ends up holding the object and is therefore the arm whose
            state changes for every subsequent skill).
        target_object: The canonical object name this skill acts on, e.g.
            "plate", "mug", "drawer". Synonyms (e.g. "cup", "glass") are
            already resolved to their canonical form by the Grounder --
            this field never holds a synonym.
        params: Skill-specific extra detail that does not deserve its own
            field, e.g. `{"destination": "table"}` for "place", or
            `{"from_arm": "B"}` for "handoff". Empty dict when a skill
            needs nothing extra (e.g. "pick", "open_drawer").
    """

    skill: str
    arm: str
    target_object: str
    params: dict = field(default_factory=dict)


@dataclass
class TaskPlan:
    """An ordered sequence of SkillCalls produced by one `Grounder.ground()`.

    Attributes:
        skills: The ordered list of SkillCalls. Empty when the source
            command was empty or whitespace-only (see `rule_grounder.py`
            for why that case does not raise).
        source_text: The exact `CommandEvent.text` this plan was grounded
            from, kept for logging/manifest provenance (same rationale as
            `CommandEvent.source_id`, `command/events.py:30-34`).
    """

    skills: list[SkillCall] = field(default_factory=list)
    source_text: str = ""

    def __len__(self) -> int:
        return len(self.skills)

    def __iter__(self):
        return iter(self.skills)

    def is_empty(self) -> bool:
        """True when this plan has no skills (empty/whitespace-only command)."""
        return len(self.skills) == 0
