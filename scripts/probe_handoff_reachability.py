"""Re-measure the shared handoff transfer point (ADR-032, second pass).

**Why a second pass, not a re-run of the first.** The first pass (see
`DECISIONS.md`'s ADR-032 entry, `docs/hardware/m06-handoff-reachability.md`'s
prior committed contents) solved every candidate's IK straight from the
folded "home" reset pose and found ALL 36 candidates FAIL -- residual and
collision were both checked, but every solve started from home, and
`ik.py`'s redundant 5-DOF position-only DLS solver (no obstacle term,
ADR-024) is well documented elsewhere in this project (ADR-032's own root
-cause paragraph, `m06-reachability-probe.md` Step 4) to converge to
table-tunneling local minima specifically when asked to reach centrally
across the table FROM that folded seed. This pass applies four corrections
plus two additions, per instruction, before concluding anything about
whether a shared point exists:

  C1 - grid targets the pinch point. Unchanged in substance from the first
       pass: `ik.solve_position_ik` already targets the pinch point
       (ADR-025), not the gripper body. PASS requires residual < 0.01 m
       (`ik.IK_POSITION_TOLERANCE_M`) AND no joint-limit violation (checked
       explicitly below, not merely inferred from `ik.py`'s own per-iteration
       clipping -- see `_joint_limit_margin`).
  C2 - seed each arm's IK from its handoff-APPROACH pose, not from home.
       This is the correction expected to matter most. For each candidate
       and each arm, this script stages the SAME two solves
       `skills_scripted.run_handoff` itself would issue for that arm at that
       candidate: first solve from HOME toward that arm's own APPROACH hover
       point (`skills_scripted.CLEARANCE_HEIGHT_M` above the candidate, plus
       `skills_scripted.HANDOFF_SIDE_OFFSET_M` for the receiving arm), then
       -- starting from THAT resulting configuration, not from home again --
       solve toward the candidate itself. The residual reported and gated on
       is this second, seeded solve's residual.
  C3 - cross-arm collision: apply BOTH arms' converged (stage-2) configs
       SIMULTANEOUSLY via `mujoco.mj_forward` and reject any candidate with
       a cross-arm contact (an armA geom touching an armB geom) deeper than
       -0.005 m.
  C4 - one direction is sufficient. This sweep fixes from_arm=A, to_arm=B
       (matching the commit gate, `handoff(A, B, fork)`). `handoff(B, A,
       fork)` is checked opportunistically at VERIFY time by actually
       running the skill, not swept here as a second grid.

  Addition 1 (kept from the first pass) - arm-vs-world collision: table_top
  AND every prop (plate, mug, fork, spoon, water_bottle, drawer) checked
  per arm, that arm's OWN stage-2 config applied ALONE (the other arm left
  at the reset/home baseline), same -0.005 m depth bar as C3. This is what
  the first pass's own diagnosis named as the actual failure mode
  (`armA_lower_arm`/`armA_wrist` tunneling through `table_top`/`mug`/`fork`
  at up to -0.031 m) -- dropping it would let a seeded-but-still-tunneling
  candidate through unnoticed.
  Addition 2 - grid extended upward: z in {0.35, 0.38, 0.40} (as specified)
  PLUS z in {0.44, 0.47} (extra rows, reported ALONGSIDE the specified rows,
  not replacing them), on the reasoning that z=0.35 IS the tabletop surface
  and a real bimanual handoff happens in free space above the table.

PASS bar per candidate, per arm: residual < `ik.IK_POSITION_TOLERANCE_M` AND
no joint-limit violation. `both_reachable` requires both arms to pass.
`arm_world_ok` requires BOTH arms' individually-applied stage-2 configs to
be free of any world (table/prop) contact deeper than -0.005 m.
`cross_arm_ok` requires the two arms' stage-2 configs, applied together, to
be free of any cross-arm contact deeper than -0.005 m. `final_verdict`
requires all three (`both_reachable and arm_world_ok and cross_arm_ok`).

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control.skills_scripted import (  # noqa: E402
    CLEARANCE_HEIGHT_M,
    HANDOFF_SIDE_OFFSET_M,
    OBJECT_BODY_NAME,
)
from bimanual.sim.env import TableSettingEnv  # noqa: E402

SEED = 0
FROM_ARM = "A"  # C4: fixed direction, matches the commit gate handoff(A, B, fork).
TO_ARM = "B"
COLLISION_DEPTH_THRESHOLD_M = -0.005  # C3 / addition 1: both use this same bar.
JOINT_LIMIT_MARGIN_TOL_M = 1e-4  # "pinned at its range bound" tolerance, radians.

REPORT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / "docs" / "hardware" / "m06-handoff-reachability.md"
)

X_CANDIDATE = 0.0
GRID_Y = np.round(np.arange(-0.12, 0.10 + 1e-9, 0.02), 2)
GRID_Z_SPECIFIED = (0.35, 0.38, 0.40)
GRID_Z_EXTRA = (0.44, 0.47)
GRID_Z = GRID_Z_SPECIFIED + GRID_Z_EXTRA

# `to_arm` side sign, copied from `skills_scripted.run_handoff`'s own
# `side = -1.0 if to_arm == "A" else 1.0` (arm A base at y=-0.25, arm B at
# y=+0.25) -- reproduced, not imported, since it is a local variable inside
# `run_handoff`, not a module-level constant.
_TO_ARM_SIDE = -1.0 if TO_ARM == "A" else 1.0


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _arm_body_ids(model, arm: str) -> set[int]:
    """Every body id whose name is prefixed `arm{arm}_` (the arm's whole
    kinematic subtree). Copied, not imported, from `skills_scripted.py` /
    `probe_reachability.py`'s identical helper -- same convention the first
    pass of this script used, so this script keeps no coupling to either
    module's private names.
    """
    prefix = f"arm{arm}_"
    ids = set()
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name and name.startswith(prefix):
            ids.add(b)
    return ids


def _arm_geom_ids(model, arm: str) -> set[int]:
    body_ids = _arm_body_ids(model, arm)
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}


def _world_geom_ids(model) -> set[int]:
    """table_top's own geom, plus every geom belonging to a prop body
    (`OBJECT_BODY_NAME`'s values: plate, mug, fork, spoon, water_bottle,
    drawer) -- "table and props" per this task's own second addition.
    """
    geoms: set[int] = set()
    table_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    if table_gid != -1:
        geoms.add(table_gid)
    for body_name in set(OBJECT_BODY_NAME.values()):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid == -1:
            continue
        geoms |= {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == bid}
    return geoms


def _joint_limit_margin(model, arm: str, joint_angles: np.ndarray) -> float:
    """Smallest distance, over this arm's 5 positioning joints, between the
    SOLVED angle and either end of that joint's `jnt_range`. `ik.py`'s own
    solver already clips every iteration to stay inside `jnt_range`
    (`solve_position_ik`'s per-iteration `np.clip`), so a solved angle can
    never be OUTSIDE range; what this catches instead is a solve that is
    PINNED AT a range bound, i.e. the unconstrained solve wanted to go
    further and was stopped by the joint limit -- exactly the situation
    `GRASP_POINT_OFFSET_M["plate"]`'s own docstring (`skills_scripted.py`)
    already measured and rejected for a different target ("drives
    shoulder_lift, elbow_flex AND wrist_flex all simultaneously to their
    joint-range limits and the DLS solve stalls there").
    """
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ik.arm_joint_names(arm)
    ]
    margin = float("inf")
    for jid, angle in zip(joint_ids, joint_angles):
        lo, hi = model.jnt_range[jid]
        margin = min(margin, float(angle) - float(lo), float(hi) - float(angle))
    return margin


def _seeded_two_stage_solve(
    model, home_data, arm: str, approach_pos: np.ndarray, final_pos: np.ndarray
) -> tuple[ik.IKSolution, ik.IKSolution]:
    """C2: solve HOME -> approach_pos (stage 1), then, starting from stage
    1's resulting configuration (not from home again), solve -> final_pos
    (stage 2). Returns (stage1_solution, stage2_solution); callers gate on
    stage2's residual, matching what `run_handoff` itself actually reaches
    after its own APPROACH waypoint converges (or nearly does) and DESCEND
    then runs from there.
    """
    stage1_scratch = mujoco.MjData(model)
    stage1_scratch.qpos[:] = home_data.qpos
    stage1_scratch.qvel[:] = 0.0
    mujoco.mj_forward(model, stage1_scratch)
    stage1 = ik.solve_position_ik(model, stage1_scratch, arm, approach_pos)

    # Build the seeded starting point for stage 2: identical to home except
    # this arm's 5 positioning joints are set to stage 1's result.
    seeded = mujoco.MjData(model)
    seeded.qpos[:] = home_data.qpos
    seeded.qvel[:] = 0.0
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ik.arm_joint_names(arm)
    ]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    for qadr, angle in zip(qpos_adrs, stage1.joint_angles):
        seeded.qpos[qadr] = float(angle)
    mujoco.mj_forward(model, seeded)

    stage2 = ik.solve_position_ik(model, seeded, arm, final_pos)
    return stage1, stage2


def _apply_arm_solution(model, home_data, arm: str, joint_angles: np.ndarray, into: "mujoco.MjData") -> None:
    """Write `joint_angles` (this arm's 5 positioning joints) into `into`,
    which the caller has already initialised from `home_data` -- so every
    OTHER joint (the other arm, every prop's free joint, the drawer slide)
    stays exactly at its home/reset value.
    """
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ik.arm_joint_names(arm)
    ]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    for qadr, angle in zip(qpos_adrs, joint_angles):
        into.qpos[qadr] = float(angle)


def _deepest_contact(data, geoms_a: set[int], geoms_b: set[int]) -> float | None:
    """Deepest (most negative) `dist` among contacts between `geoms_a` and
    `geoms_b`, or None if there is no such contact at all.
    """
    deepest = None
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in geoms_a and g2 in geoms_b) or (g2 in geoms_a and g1 in geoms_b):
            dist = float(c.dist)
            if deepest is None or dist < deepest:
                deepest = dist
    return deepest


def run_sweep(env: TableSettingEnv) -> list[dict]:
    model, home_data = env.model, env.data
    arm_geoms = {arm: _arm_geom_ids(model, arm) for arm in ("A", "B")}
    world_geoms = _world_geom_ids(model)

    rows = []
    for z in GRID_Z:
        for y in GRID_Y:
            transfer_point = np.array([X_CANDIDATE, float(y), float(z)])
            receiving_point = transfer_point + np.array([0.0, _TO_ARM_SIDE * HANDOFF_SIDE_OFFSET_M, 0.0])

            targets = {
                FROM_ARM: transfer_point,
                TO_ARM: receiving_point,
            }
            approach = {
                arm: targets[arm] + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M]) for arm in ("A", "B")
            }

            stage2 = {}
            residual = {}
            reachable = {}
            for arm in ("A", "B"):
                s1, s2 = _seeded_two_stage_solve(model, home_data, arm, approach[arm], targets[arm])
                stage2[arm] = s2
                residual[arm] = s2.position_error_m
                margin = _joint_limit_margin(model, arm, s2.joint_angles)
                residual_ok = s2.position_error_m < ik.IK_POSITION_TOLERANCE_M
                limit_ok = margin > JOINT_LIMIT_MARGIN_TOL_M
                reachable[arm] = residual_ok and limit_ok

            both_reachable = reachable["A"] and reachable["B"]

            # Addition 1: arm-vs-world, each arm's stage-2 config applied
            # ALONE (the other arm stays at home).
            arm_world_ok = True
            world_detail = {}
            for arm in ("A", "B"):
                scratch = mujoco.MjData(model)
                scratch.qpos[:] = home_data.qpos
                scratch.qvel[:] = 0.0
                _apply_arm_solution(model, home_data, arm, stage2[arm].joint_angles, scratch)
                mujoco.mj_forward(model, scratch)
                deepest = _deepest_contact(scratch, arm_geoms[arm], world_geoms)
                world_detail[arm] = deepest
                if deepest is not None and deepest < COLLISION_DEPTH_THRESHOLD_M:
                    arm_world_ok = False

            # C3: both arms' stage-2 configs applied SIMULTANEOUSLY.
            both_scratch = mujoco.MjData(model)
            both_scratch.qpos[:] = home_data.qpos
            both_scratch.qvel[:] = 0.0
            _apply_arm_solution(model, home_data, "A", stage2["A"].joint_angles, both_scratch)
            _apply_arm_solution(model, home_data, "B", stage2["B"].joint_angles, both_scratch)
            mujoco.mj_forward(model, both_scratch)
            cross_arm_deepest = _deepest_contact(both_scratch, arm_geoms["A"], arm_geoms["B"])
            cross_arm_ok = cross_arm_deepest is None or cross_arm_deepest >= COLLISION_DEPTH_THRESHOLD_M

            final_verdict = both_reachable and arm_world_ok and cross_arm_ok

            rows.append(
                {
                    "x": X_CANDIDATE,
                    "y": float(y),
                    "z": float(z),
                    "specified": z in GRID_Z_SPECIFIED,
                    "armA_residual": residual["A"],
                    "armB_residual": residual["B"],
                    "both_reachable": both_reachable,
                    "arm_world_ok": arm_world_ok,
                    "cross_arm_ok": cross_arm_ok,
                    "final_verdict": final_verdict,
                    "world_detail_A": world_detail["A"],
                    "world_detail_B": world_detail["B"],
                    "cross_arm_detail": cross_arm_deepest,
                }
            )
    return rows


def main() -> int:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)  # home keyframe, ADR-026.

    rows = run_sweep(env)
    passing = [r for r in rows if r["final_verdict"]]

    lines = []
    lines.append("# M06 handoff-transfer-point reachability sweep (ADR-032, second pass)")
    lines.append("")
    lines.append(
        "Second measurement pass after the first pass (all 36 candidates FAIL, seeded from "
        "home) -- four corrections (C1-C4) plus two additions applied; see "
        "`scripts/probe_handoff_reachability.py`'s module docstring for the full rationale."
    )
    lines.append("")
    lines.append(f"Seed: {SEED}, reset via `TableSettingEnv.reset()` (home keyframe, ADR-026).")
    lines.append(f"Direction gated on: handoff({FROM_ARM}, {TO_ARM}, fork) (C4).")
    lines.append(f"Sweep: x = {X_CANDIDATE}, y in {GRID_Y.tolist()}, z in {list(GRID_Z_SPECIFIED)} (specified) "
                 f"+ {list(GRID_Z_EXTRA)} (extra rows, addition 2).")
    lines.append(
        f"PASS per arm: residual < {ik.IK_POSITION_TOLERANCE_M} m (seeded from that arm's own "
        "handoff-APPROACH pose, C2) AND no joint-limit violation (margin > "
        f"{JOINT_LIMIT_MARGIN_TOL_M} m from either `jnt_range` bound)."
    )
    lines.append(
        f"Collision bar (both arm_world_ok and cross_arm_ok): any contact deeper than "
        f"{COLLISION_DEPTH_THRESHOLD_M} m is a FAIL (C3 / addition 1)."
    )
    lines.append("")
    lines.append("| x | y | z | armA_residual | armB_residual | both_reachable | arm_world_ok | cross_arm_ok | final_verdict |")
    lines.append("|---:|---:|---:|---:|---:|---|---|---|---|")
    for r in rows:
        marker = "" if r["specified"] else " (extra)"
        lines.append(
            f"| {r['x']:.2f} | {r['y']:.2f} | {r['z']:.2f}{marker} | "
            f"{r['armA_residual']:.5f} | {r['armB_residual']:.5f} | "
            f"{r['both_reachable']} | {r['arm_world_ok']} | {r['cross_arm_ok']} | "
            f"{'PASS' if r['final_verdict'] else 'FAIL'} |"
        )
    lines.append("")

    if passing:
        chosen = min(passing, key=lambda r: (abs(r["y"]), r["z"]))
        lines.append(
            f"**Chosen candidate: (x={chosen['x']:.2f}, y={chosen['y']:.2f}, z={chosen['z']:.2f})** -- "
            f"armA_residual={chosen['armA_residual']:.5f}, armB_residual={chosen['armB_residual']:.5f}, "
            "both_reachable=True, arm_world_ok=True, cross_arm_ok=True. Preference order applied: "
            "y=0 if it qualifies, else nearest to y=0; ties broken toward the lower z (more clearance "
            "margin was not distinguishing here, so the lowest passing z among equal |y| was kept)."
        )
        lines.append("")
        lines.append(f"**Overall: {len(passing)} of {len(rows)} candidates PASS.**")
    else:
        lines.append(
            "**NO candidate passed at any tested (y, z), including the extra z in "
            f"{list(GRID_Z_EXTRA)} rows.** No collision-free, both-reachable shared transfer "
            "point was found anywhere in this sweep."
        )
        lines.append("")
        lines.append("**Overall: ALL FAIL.**")
    lines.append("")

    report = "\n".join(lines) + "\n"
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report, newline="\n")

    print(report)
    print(f"(report written to {REPORT_PATH})")

    env.close()
    return 0 if passing else 1


if __name__ == "__main__":
    raise SystemExit(main())
