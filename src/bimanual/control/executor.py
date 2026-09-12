"""SkillExecutor: drive one SkillCall to completion or failure (M06a).

Per ARCHITECTURE.md's component-contract table:

    SkillExecutor.execute(SkillCall, env) -> SkillResult

`ScriptedSkillExecutor` (this module) dispatches to the closed-loop IK
primitives in `skills_scripted.py`. `LearnedSkillExecutor` is specified in
ARCHITECTURE.md's diagram but not built -- ADR-023 cut the learned-policy
branch from the critical path, so `SkillExecutor` stays an interface with
exactly one concrete implementation today.

`pour` is M06b, out of scope for this module by task instruction: requesting
it here returns a clearly-labelled failed `SkillResult` rather than raising,
so a caller iterating a `TaskPlan` that happens to include `pour` gets a
readable "not implemented yet" instead of a crash.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from bimanual.control import ik
from bimanual.control import skills_scripted as skills
from bimanual.control.skills_scripted import SkillResult
from bimanual.language.grounder import UngroundedCommandError
from bimanual.language.skills import SkillCall


class SkillExecutor(ABC):
    """Strategy interface: one grounded SkillCall -> one SkillResult."""

    @abstractmethod
    def execute(self, skill_call: SkillCall, env, step_budget: int = ik.DEFAULT_STEP_BUDGET) -> SkillResult:
        """Drive `skill_call` to completion or failure against `env`.

        Args:
            skill_call: One grounded instruction (M05's `SkillCall`).
            env: A `TableSettingEnv` already `reset()` to the state this
                skill should act on.
            step_budget: Maximum `env.step()` calls this skill may use
                before returning `success=False, reason="timeout"` (PLAN.md
                M06 done-when 3 -- a STEP budget, not a wall-clock timer).
        """
        raise NotImplementedError


#: The M06a skill vocabulary this executor implements. `pour` (M06b) and
#: `close_drawer` (present in the M05 grammar, not required by M06a's task
#: scope) are deliberately absent -- see this module's docstring.
_SUPPORTED_SKILLS = ("open_drawer", "pick", "place", "handoff")


class ScriptedSkillExecutor(SkillExecutor):
    """IK-driven scripted skills -- the shipped policy under ADR-023."""

    def execute(self, skill_call: SkillCall, env, step_budget: int = ik.DEFAULT_STEP_BUDGET) -> SkillResult:
        # Vision-based skills out of scope per ADR-023. All skills execute
        # state-only for ~0.20ms/step budget.
        #
        # `env._default_cameras` is a private attribute of TableSettingEnv
        # (src/bimanual/sim/env.py), read here rather than added as a new
        # public getter because that file is out of scope for this module.
        # Accessed defensively with `getattr` so a future TableSettingEnv
        # refactor that renames/removes the attribute fails this assertion
        # loudly rather than raising an unrelated AttributeError.
        assert getattr(env, "_default_cameras", None) is None, (
            "ScriptedSkillExecutor requires TableSettingEnv(cameras=None); "
            "this executor reads only privileged state and never renders."
        )

        try:
            return self._dispatch(skill_call, env, step_budget)
        except UngroundedCommandError:
            # This layer never calls the Grounder, so UngroundedCommandError
            # cannot legitimately originate here today -- but it subclasses
            # ValueError (bimanual/language/grounder.py), and the `except
            # ValueError` below would silently swallow it if this branch
            # were removed. That is exactly the trap Tester flagged on M05:
            # a broad `except ValueError` hiding a typed grounding error.
            # Re-raise rather than reporting it as an ordinary skill
            # failure, so it stays loud if some future caller ever wires a
            # Grounder call into this path.
            raise
        except ValueError as exc:
            return SkillResult(False, f"ValueError: {exc}", 0)

    @staticmethod
    def _dispatch(skill_call: SkillCall, env, step_budget: int) -> SkillResult:
        if skill_call.skill == "open_drawer":
            return skills.run_open_drawer(env, arm=skill_call.arm, step_budget=step_budget)

        if skill_call.skill == "pick":
            return skills.run_pick(env, skill_call.arm, skill_call.target_object, step_budget=step_budget)

        if skill_call.skill == "place":
            destination = skill_call.params.get("destination", "table")
            return skills.run_place(
                env, skill_call.arm, skill_call.target_object, destination=destination, step_budget=step_budget
            )

        if skill_call.skill == "handoff":
            from_arm = skill_call.params.get("from_arm")
            if from_arm is None:
                return SkillResult(False, "handoff SkillCall missing params['from_arm']", 0)
            return skills.run_handoff(
                env,
                to_arm=skill_call.arm,
                from_arm=from_arm,
                target_object=skill_call.target_object,
                step_budget=step_budget,
            )

        return SkillResult(
            False,
            f"unsupported skill {skill_call.skill!r}: M06a implements {_SUPPORTED_SKILLS}; "
            "'pour' is M06b and 'close_drawer' is out of M06a's scope",
            0,
        )
