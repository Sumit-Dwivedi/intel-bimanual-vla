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

**ADR-027 status of the four tests above, reported here rather than
silently edited to match the code:** all four still FAIL after ADR-027's
waypoint-staging fix, for two DIFFERENT, already-documented, and explicitly
out-of-scope reasons that are NOT staging defects:
  - `test_open_drawer_reaches_near_limit`: arm A cannot converge to ANY
    waypoint at drawer height once it is at or outside the table's own
    footprint (measured: IK residual stalls at 0.09-0.32 m regardless of
    physical steps given, including a 1500-step real closed-loop drive) --
    a kinematic reach limit of the arm's own base placement, not fixable by
    a different waypoint choice (see `run_open_drawer`'s docstring).
  - `test_pick_plate_lifts_above_table`, `test_place_plate_returns_to_table_rest`,
    `test_handoff_mug_ends_held_by_arm_b`: every waypoint now converges and
    clears collision validation (confirmed: no new arm-vs-table_top or
    cross-arm contact beyond `TABLE_COLLISION_DEPTH_TOL_M`), the jaw closes
    fully, and the object STILL never lifts even after 300 dwell steps
    (measured directly, well beyond `GRIP_HOLD_FRAMES`) -- this is ADR-024's
    already-documented pinch-point/jaw-geometry grasp-reliability gap, out
    of this module's scope (fixing it means orientation control or a
    contact-feedback loop, neither of which `ik.py` may gain here).
    `test_handoff_mug_ends_held_by_arm_b` additionally hits the SAME
    kinematic reach problem as the drawer: `mug_at_rest` itself (before any
    grasp offset) does not converge for arm A from the "home" pose
    (residual 0.12 m, independently confirmed by
    `docs/hardware/m06-reachability-probe.md`'s own Step-4 table).
The two NEW tests below (`test_open_drawer_fails_without_tunneling_through_table`
and `test_pick_plate_waypoints_progress_without_collision`) are this
module's actual regression coverage for ADR-027's fix: they assert that a
skill which cannot complete its TASK still never creates a new collision
and stops at the first waypoint that could not be validated, rather than
silently proceeding (or silently tunneling, which is what the pre-ADR-027
code did).
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


def _arm_table_contacts(env: TableSettingEnv, arm: str) -> int:
    """Count CURRENT real-physics contacts between `arm`'s geoms and
    `table_top`, deeper than `skills_scripted.TABLE_COLLISION_DEPTH_TOL_M`
    (same method `skills_scripted._contact_counts` uses) -- the direct,
    independent check that ADR-027's waypoint staging actually prevents
    tunneling, not merely that a `SkillResult` happens to say `False`.
    """
    from bimanual.control.skills_scripted import TABLE_COLLISION_DEPTH_TOL_M

    prefix = f"arm{arm}_"
    arm_body_ids = {
        b
        for b in range(env.model.nbody)
        if (mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(prefix)
    }
    arm_geoms = {g for g in range(env.model.ngeom) if int(env.model.geom_bodyid[g]) in arm_body_ids}
    table_gid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    n = 0
    for i in range(env.data.ncon):
        c = env.data.contact[i]
        if float(c.dist) >= -TABLE_COLLISION_DEPTH_TOL_M:
            continue
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in arm_geoms and g2 == table_gid) or (g2 in arm_geoms and g1 == table_gid):
            n += 1
    return n


def test_open_drawer_fails_without_tunneling_through_table(executor):
    """ADR-027's actual regression test for `open_drawer`. The skill is NOT
    expected to succeed (see this module's docstring: arm A cannot converge
    on any waypoint at drawer height outside the table footprint, a
    kinematic reach limit, not a staging defect) -- but per ADR-027 it must
    fail CLEANLY at waypoint 1 (APPROACH), via a convergence failure, never
    by tunneling through `table_top` the way the pre-ADR-027 top-down
    approach did (ADR-026 measured that old path at up to -0.064 m
    penetration).
    """
    env = _make_env()
    try:
        result = executor.execute(
            SkillCall(skill="open_drawer", arm="A", target_object="drawer", params={}), env
        )
        print(f"open_drawer (ADR-027 regression): {result}")

        assert result.success is False
        assert "waypoint 1" in result.reason, (
            f"expected the skill to stop at the FIRST waypoint (approach), not proceed further; "
            f"reason={result.reason!r}"
        )
        assert "convergence" in result.reason, (
            f"expected a convergence failure (arm A cannot reach outside the table at drawer "
            f"height), not a collision failure; reason={result.reason!r}"
        )
        # The actual, independent proof: no tunneling occurred, regardless of
        # what the SkillResult claims.
        assert _arm_table_contacts(env, "A") == 0, (
            "open_drawer must never create a real arm-vs-table_top contact, even on failure"
        )
        # Waypoint 1 alone was attempted, not the whole step budget --
        # "intermediate poses cleared before advancing" means a failure at
        # waypoint 1 must not silently consume budget meant for waypoints
        # 2-6.
        from bimanual.control.skills_scripted import APPROACH_DESCENT_STEPS

        assert result.frames_used <= APPROACH_DESCENT_STEPS, (
            f"a waypoint-1 failure should use at most one waypoint's step cap "
            f"({APPROACH_DESCENT_STEPS}), not the full skill budget; "
            f"frames_used={result.frames_used}"
        )
    finally:
        env.close()


def test_pick_plate_waypoints_progress_without_collision(executor):
    """ADR-027's actual regression test for `pick`. Whether or not the
    GRASP itself succeeds (ADR-024's pinch-point/jaw-geometry limitation,
    out of this module's scope -- see this module's docstring), the staged
    APPROACH -> DESCEND -> GRIP -> RETREAT sequence must never create a new
    arm-vs-table_top contact: each waypoint is validated before the next
    one runs ("intermediate poses cleared before advancing"), so a failure
    to lift the plate must show up as "did not lift", never as a silent
    tunnel through the tabletop.
    """
    env = _make_env()
    try:
        result = executor.execute(
            SkillCall(skill="pick", arm="A", target_object="plate", params={}), env
        )
        print(f"pick(plate) (ADR-027 regression): {result}")

        assert result.frames_used > 0
        # Every waypoint that ran must have cleared collision validation --
        # if it hadn't, `run_pick` would have returned at that waypoint with
        # a "failed [collision (...)]" reason instead of reaching its final
        # lift check.
        assert "collision" not in result.reason, (
            f"pick's waypoints must clear collision validation even when the grasp itself "
            f"does not lift the object; reason={result.reason!r}"
        )
        assert _arm_table_contacts(env, "A") == 0, (
            "pick must never leave arm A in real contact with table_top beyond the measured "
            "graze/tunneling boundary, whether or not the grasp itself succeeded"
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
