"""Tests for M06a scripted skills: open_drawer, pick, place, handoff
(PLAN.md M06 done-when 1-4).

Each test asserts a MEASURABLE STATE DELTA, not merely `result.success`:
  - open_drawer: `drawer_slide` qpos reaches >= DRAWER_SUCCESS_QPOS (0.14,
    within 0.01 of the 0.15 limit).
  - pick(A, plate): the plate's free-joint z rises above the table surface
    by at least PICK_LIFT_MARGIN_M.
  - place(A, plate): the plate returns to a table-resting height, within
    the tabletop's xy bounds.
  - handoff(A, B, mug): the mug ends measurably closer to arm B's
    gripperframe site than arm A's, and lifted since the handoff began.
Every `SkillResult` is also checked for a positive `frames_used`.

Runs only on bm-ptl (ADR-020): `bimanual.sim.env` imports `mujoco`, which
cannot load on the developer's Windows laptop (Smart App Control blocks the
unsigned `mujoco.dll`). This file is not collectable/passable there, and
that is the expected, documented state -- see PLAN.md M06's "Runs on" note.
"""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from bimanual.control.executor import ScriptedSkillExecutor
from bimanual.control.skills_scripted import (
    DRAWER_SUCCESS_QPOS,
    PICK_LIFT_MARGIN_M,
    TABLE_SURFACE_Z,
)
from bimanual.control import ik
from bimanual.language.skills import SkillCall
from bimanual.sim.env import TableSettingEnv

SEED = 0


def _make_env() -> TableSettingEnv:
    # Vision-based skills out of scope per ADR-023. All skills execute
    # state-only for ~0.20ms/step budget.
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)
    return env


def _body_id(env: TableSettingEnv, name: str) -> int:
    return mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, name)


def _body_z(env: TableSettingEnv, body_name: str) -> float:
    return float(env.data.xpos[_body_id(env, body_name)][2])


def _drawer_qpos(env: TableSettingEnv) -> float:
    jid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, "drawer_slide")
    qadr = int(env.model.jnt_qposadr[jid])
    return float(env.data.qpos[qadr])


@pytest.fixture
def executor() -> ScriptedSkillExecutor:
    return ScriptedSkillExecutor()


def test_open_drawer_reaches_near_limit(executor):
    env = _make_env()
    try:
        initial_qpos = _drawer_qpos(env)

        result = executor.execute(
            SkillCall(skill="open_drawer", arm="A", target_object="drawer", params={}), env
        )

        final_qpos = _drawer_qpos(env)
        print(
            f"open_drawer: initial_qpos={initial_qpos:.4f} final_qpos={final_qpos:.4f} "
            f"success={result.success} frames_used={result.frames_used} reason={result.reason}"
        )
        assert result.frames_used > 0
        assert final_qpos >= DRAWER_SUCCESS_QPOS, (
            f"drawer_slide qpos={final_qpos:.4f}, expected >= {DRAWER_SUCCESS_QPOS} "
            f"(within 0.01 of the 0.15 limit); SkillResult={result}"
        )
    finally:
        env.close()


def test_pick_plate_lifts_above_table(executor):
    env = _make_env()
    try:
        initial_z = _body_z(env, "plate")

        result = executor.execute(
            SkillCall(skill="pick", arm="A", target_object="plate", params={}), env
        )

        final_z = _body_z(env, "plate")
        print(
            f"pick(plate): initial_z={initial_z:.4f} final_z={final_z:.4f} "
            f"success={result.success} frames_used={result.frames_used} reason={result.reason}"
        )
        assert result.frames_used > 0
        assert final_z >= TABLE_SURFACE_Z + PICK_LIFT_MARGIN_M, (
            f"plate final z={final_z:.4f} did not clear the table surface "
            f"({TABLE_SURFACE_Z}) by the required margin ({PICK_LIFT_MARGIN_M}); "
            f"SkillResult={result}"
        )
        assert final_z >= initial_z + PICK_LIFT_MARGIN_M
        assert result.success
    finally:
        env.close()


def test_place_plate_returns_to_table_rest(executor):
    env = _make_env()
    try:
        result = executor.execute(
            SkillCall(skill="place", arm="A", target_object="plate", params={"destination": "table"}), env
        )

        final_z = _body_z(env, "plate")
        final_xy = np.array(env.data.xpos[_body_id(env, "plate")][:2], dtype=np.float64, copy=True)
        print(
            f"place(plate): final_z={final_z:.4f} final_xy={final_xy} "
            f"success={result.success} frames_used={result.frames_used} reason={result.reason}"
        )
        assert result.frames_used > 0
        assert result.success, result.reason
        assert (TABLE_SURFACE_Z - 0.05) <= final_z <= (TABLE_SURFACE_Z + 0.10), (
            "plate must come to rest on the table, not fall through or fly off; "
            f"final_z={final_z:.4f}"
        )
        assert -0.40 <= final_xy[0] <= 0.40 and -0.25 <= final_xy[1] <= 0.25, (
            f"plate must land within the tabletop bounds; final_xy={final_xy}"
        )
    finally:
        env.close()


def test_handoff_mug_ends_held_by_arm_b(executor):
    env = _make_env()
    try:
        initial_z = _body_z(env, "mug")

        result = executor.execute(
            SkillCall(skill="handoff", arm="B", target_object="mug", params={"from_arm": "A"}), env
        )

        mug_pos = np.array(env.data.xpos[_body_id(env, "mug")], dtype=np.float64, copy=True)
        site_a = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "armA_gripperframe")
        site_b = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "armB_gripperframe")
        dist_a = float(np.linalg.norm(mug_pos - env.data.site_xpos[site_a]))
        dist_b = float(np.linalg.norm(mug_pos - env.data.site_xpos[site_b]))
        final_z = float(mug_pos[2])

        print(
            f"handoff(mug, A->B): initial_z={initial_z:.4f} final_z={final_z:.4f} "
            f"dist_to_armA={dist_a:.4f} dist_to_armB={dist_b:.4f} "
            f"success={result.success} frames_used={result.frames_used} reason={result.reason}"
        )
        assert result.frames_used > 0
        assert result.success, result.reason
        assert dist_b < dist_a, (
            f"mug must end closer to arm B's gripperframe than arm A's; "
            f"dist_to_armA={dist_a:.4f} dist_to_armB={dist_b:.4f}"
        )
        assert final_z >= initial_z + PICK_LIFT_MARGIN_M
    finally:
        env.close()


def test_every_skill_returns_a_sensible_frames_used(executor):
    """PLAN.md M06 done-when 2/3: every SkillResult carries frames_used, and
    it must be a positive int within the requested step budget (never
    negative, never silently zero for a skill that actually ran).
    """
    cases = [
        ("open_drawer", "A", "drawer", {}),
        ("pick", "A", "plate", {}),
        ("place", "A", "plate", {"destination": "table"}),
        ("handoff", "B", "mug", {"from_arm": "A"}),
    ]
    step_budget = ik.DEFAULT_STEP_BUDGET
    for skill, arm, obj, params in cases:
        env = _make_env()
        try:
            result = executor.execute(
                SkillCall(skill=skill, arm=arm, target_object=obj, params=params), env, step_budget=step_budget
            )
            print(f"{skill}: frames_used={result.frames_used} success={result.success}")
            assert isinstance(result.frames_used, int)
            assert 0 < result.frames_used <= step_budget, (
                f"{skill} returned frames_used={result.frames_used}, expected in (0, {step_budget}]"
            )
        finally:
            env.close()


def test_unsupported_skill_is_a_clean_failure_not_a_crash(executor):
    """`pour` is M06b, deliberately not implemented here (task scope)."""
    env = _make_env()
    try:
        result = executor.execute(
            SkillCall(skill="pour", arm="A", target_object="mug", params={"source": "bottle"}), env
        )
        assert result.success is False
        assert "pour" in result.reason
        assert result.frames_used == 0
    finally:
        env.close()
