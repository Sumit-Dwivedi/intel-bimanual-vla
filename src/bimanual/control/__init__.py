"""Coordinator, IK solver and skill primitives (M06).

`ik.py` (position-only damped-least-squares IK, ADR-024), `skills_scripted.py`
(`open_drawer`, `pick`, `place`, `handoff` -- M06a; `pour` is M06b, not yet
built) and `executor.py` (`SkillExecutor` / `ScriptedSkillExecutor`) are
implemented. A `Coordinator` that arbitrates which arm runs which `SkillCall`
next across a multi-step `TaskPlan` is not yet built.

Per ARCHITECTURE.md ADR-020, MuJoCo cannot be imported on this laptop
(Windows Smart App Control blocks the unsigned `mujoco.dll`). This
`__init__.py` stays side-effect free, same as `bimanual.sim`'s (see that
package's docstring), so `import bimanual.control` itself does not fail on
the laptop -- only importing `ik`/`skills_scripted`/`executor` (which import
`mujoco`) will, as expected.
"""
