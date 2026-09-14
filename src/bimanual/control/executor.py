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

**M06 Phase 2 Commit 2 (ADR-030): `WeldGrasp` wiring.** `pick`, `place` and
`handoff` now attach/release props via `bimanual.sim.grasp.WeldGrasp`
(ADR-029's mechanism), threaded through from here. `WeldGrasp.__init__`
needs a compiled `env` (it resolves equality-constraint/joint/body ids
against `env.model`, see `grasp.py`) which this class's own `__init__` does
not receive -- every caller in this repo (`tests/test_skills.py`'s
`executor()` fixture, `scripts/run_skill.py`) constructs
`ScriptedSkillExecutor()` with no arguments and only supplies `env` later,
per call, to `execute()`. So `self.weld` starts `None` in `__init__` and is
lazily constructed the first time `execute()` sees an `env` -- rebuilt if a
DIFFERENT `env` instance ever comes through the same executor (a fresh
`WeldGrasp` per compiled model), but reused across repeated calls against
the SAME `env` (e.g. a `TaskPlan` with several skills run one after
another), since `reset()` does not recompile the model and the resolved
ids stay valid. This is a deliberate, disclosed departure from a literal
reading of "`__init__` constructs it as `self.weld`" -- constructing it
eagerly in `__init__` is not possible without an `env` argument this class
has never taken, and adding one would break every existing call site.

**M10 Phase 5 (ADR-046): perception is opt-in, oracle is the default.**
`ScriptedSkillExecutor(inference=None)` -- the default, and every existing
call site (`tests/test_skills.py`, `scripts/run_skill.py` before this
module) -- reproduces this class's ENTIRE pre-Phase-5 behaviour
byte-for-byte: no `CachedPropPositions` is ever constructed, every skill's
targeting reads fall straight to the oracle `env.data.xpos` path they
always used, and the `cameras=None` assertion below is unchanged. Passing a
constructed `bimanual.perception.inference.PoseNetInference` opts a single
executor instance into perception for `pick`/`place`/`handoff`'s targeting
reads ONLY -- every skill's own success/verification check stays oracle
regardless (ADR-046's Correction 1), and `open_drawer` never sees a
`position_provider` at all (the drawer is out of PoseNet's 3-prop scope).
`self._position_provider` follows the exact same lazy, per-`env`
rebuild-on-change pattern as `self.weld` above, for the same reason: a
`CachedPropPositions` binds to one `env` (it needs `env.model`/`env.data`
for its oracle fallback and `env.render` for its own refresh) and one
`WeldGrasp` (for its held-object check), neither of which exist yet at this
class's own `__init__` time.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from bimanual.control import ik
from bimanual.control import skills_scripted as skills
from bimanual.control.skills_scripted import SkillResult
from bimanual.language.grounder import UngroundedCommandError
from bimanual.language.skills import SkillCall
from bimanual.sim.grasp import WeldGrasp


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
    """IK-driven scripted skills -- the shipped policy under ADR-023.

    `inference` (M10 Phase 5, ADR-046): optional
    `bimanual.perception.inference.PoseNetInference`. `None` (the default)
    means oracle-only -- see this module's docstring's "perception is
    opt-in" section.
    """

    def __init__(self, inference=None) -> None:
        # See this module's docstring (ADR-030) for why this cannot be a
        # constructed `WeldGrasp` yet: no `env` exists at this point.
        self.weld: WeldGrasp | None = None
        # ADR-046: stored, not consumed, until an `env` exists (same reason
        # as `self.weld` above). `None` -> oracle-only, permanently, for the
        # life of this executor instance.
        self.inference = inference
        self._position_provider = None

    def _ensure_weld(self, env) -> WeldGrasp:
        """Return `self.weld`, constructing (or reconstructing, if `env`
        is a different instance than last time) it on demand.
        """
        if self.weld is None or self.weld.env is not env:
            self.weld = WeldGrasp(env)
        return self.weld

    def _ensure_position_provider(self, env, weld: WeldGrasp):
        """Return the `CachedPropPositions` bound to (env, weld), or `None`
        if this executor has no `inference` (oracle-only mode, ADR-046).

        Mirrors `_ensure_weld`'s lazy-construct / rebuild-on-different-`env`
        pattern exactly, for the identical reason: a `CachedPropPositions`
        needs a compiled `env` and a `WeldGrasp` that do not exist at this
        executor's own `__init__` time.
        """
        if self.inference is None:
            return None
        # Imported here, not at module scope, so an oracle-only executor
        # (the default, every test/CLI call site before M10 Phase 5) never
        # pays for importing `bimanual.perception.cached_access` at all.
        from bimanual.perception.cached_access import CachedPropPositions

        if self._position_provider is None or self._position_provider.env is not env:
            self._position_provider = CachedPropPositions(env, self.inference, weld=weld)
        return self._position_provider

    def execute(self, skill_call: SkillCall, env, step_budget: int = ik.DEFAULT_STEP_BUDGET) -> SkillResult:
        # `env._default_cameras` is a private attribute of TableSettingEnv
        # (src/bimanual/sim/env.py), read here rather than added as a new
        # public getter because that file is out of scope for this module.
        # Accessed defensively with `getattr` so a future TableSettingEnv
        # refactor that renames/removes the attribute fails this assertion
        # loudly rather than raising an unrelated AttributeError.
        #
        # ADR-046: the required camera set now depends on whether THIS
        # executor instance was opted into perception. Oracle mode (the
        # default, `self.inference is None`) keeps ADR-023's original
        # assertion unchanged byte-for-byte: `cameras=None`, no renderer
        # ever constructed, ~0.20 ms/step. Perception mode requires exactly
        # `["posenet_cam"]` -- PoseNet's own training camera (ADR-040/041) --
        # so a caller cannot accidentally opt into perception while also
        # asking for a different (slower, or simply wrong-distribution)
        # camera set without this assertion catching it immediately.
        default_cameras = getattr(env, "_default_cameras", None)
        if self.inference is None:
            assert default_cameras is None, (
                "oracle-mode ScriptedSkillExecutor (inference=None, the default) requires "
                "TableSettingEnv(cameras=None); this executor reads only privileged state "
                "and never renders. Pass inference=PoseNetInference(...) to opt into "
                "perception (ADR-046)."
            )
        else:
            assert default_cameras == ["posenet_cam"], (
                "perception-mode ScriptedSkillExecutor (inference given) requires "
                "TableSettingEnv(cameras=['posenet_cam']) -- PoseNet's own training camera "
                f"(ADR-040/041); got cameras={default_cameras!r}."
            )

        weld = self._ensure_weld(env)
        provider = self._ensure_position_provider(env, weld)
        if provider is not None:
            # ADR-046: invalidate at the START of every skill call, so a
            # skill never reuses a cache generation left over from a
            # DIFFERENT, previous skill's targeting reads (e.g. a TaskPlan
            # running pick(A, fork) then place(A, fork, table) back-to-back
            # through the same executor).
            provider.invalidate()
        try:
            return self._dispatch(skill_call, env, step_budget, weld, provider)
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
    def _dispatch(skill_call: SkillCall, env, step_budget: int, weld: WeldGrasp, position_provider=None) -> SkillResult:
        # `weld` (ADR-030): threaded to every skill that grasps. `open_drawer`
        # does not take one -- the drawer is not a `WeldGrasp.GRASPABLE_OBJECTS`
        # prop (slide joint, not free joint, ADR-029) -- so it is left exactly
        # as M06a built it.
        #
        # `position_provider` (M10 Phase 5, ADR-046): threaded to every skill
        # that grasps, same as `weld` -- `None` in oracle mode, reproducing
        # every call below byte-for-byte as it was before this module.
        # `open_drawer` never receives one: the drawer is out of PoseNet's
        # 3-prop scope (ADR-041), so there is nothing for it to consume.
        if skill_call.skill == "open_drawer":
            return skills.run_open_drawer(env, arm=skill_call.arm, step_budget=step_budget)

        if skill_call.skill == "pick":
            return skills.run_pick(
                env, skill_call.arm, skill_call.target_object, step_budget=step_budget, weld=weld,
                position_provider=position_provider,
            )

        if skill_call.skill == "place":
            destination = skill_call.params.get("destination", "table")
            return skills.run_place(
                env, skill_call.arm, skill_call.target_object, destination=destination, step_budget=step_budget,
                weld=weld, position_provider=position_provider,
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
                weld=weld,
                position_provider=position_provider,
            )

        return SkillResult(
            False,
            f"unsupported skill {skill_call.skill!r}: M06a implements {_SUPPORTED_SKILLS}; "
            "'pour' is M06b and 'close_drawer' is out of M06a's scope",
            0,
        )
