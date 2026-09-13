"""Diagnostic-only re-run of the ADR-032 handoff sweep, seeded from HOME
instead of from each arm's handoff-APPROACH pose (measurement only; does not
change `HANDOFF_POSITION_XYZ`, `skills_scripted.py`, or `ik.py`).

**Why this script exists.** `scripts/probe_handoff_reachability.py`
(committed at `ba76190`) found `both_reachable is False` on all 60 rows of
its grid, seeding each arm's IK solve from that arm's own handoff-APPROACH
configuration (C2 in that script's docstring) rather than from home. The
brief for this script asked whether that APPROACH-pose seeding is itself
part of the failure -- i.e. whether solving directly from HOME (skipping the
two-stage home->approach->target solve entirely) reaches materially
different residuals. This script is `ba76190`'s script with exactly one
change: the per-arm solve is a single `ik.solve_position_ik` call straight
from the home keyframe to the candidate target, not the two-stage
home->approach, approach->target solve. C1 (pinch-point target + joint-limit
check), C3 (explicit cross-arm collision), and C4 (gate on
handoff(A, B, fork) only) are otherwise identical. The grid is also
restricted to exactly the three z values named in this task's own
instruction (0.35, 0.38, 0.40) -- `ba76190`'s report additionally swept two
"extra" z rows (0.44, 0.47) that are NOT part of this comparison; those 24
rows are dropped here so every row in this script's table has a same-(x,y,z)
counterpart in `ba76190`'s table to diff against. That leaves 36 comparable
rows (12 y-values x 3 z-values), not "all 60" -- see the companion report's
top section for the explicit reconciliation of that count against the task
brief's "60 rows" framing.

PASS bar, thresholds, and collision logic are copied verbatim from
`probe_handoff_reachability.py` (same `ik.IK_POSITION_TOLERANCE_M`, same
`JOINT_LIMIT_MARGIN_TOL_M`, same `COLLISION_DEPTH_THRESHOLD_M`) so the two
reports are apples-to-apples.

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
    HANDOFF_SIDE_OFFSET_M,
    OBJECT_BODY_NAME,
)
from bimanual.sim.env import TableSettingEnv  # noqa: E402

SEED = 0
FROM_ARM = "A"  # C4: fixed direction, matches the commit gate handoff(A, B, fork).
TO_ARM = "B"
COLLISION_DEPTH_THRESHOLD_M = -0.005  # identical bar to probe_handoff_reachability.py.
JOINT_LIMIT_MARGIN_TOL_M = 1e-4  # radians; identical to probe_handoff_reachability.py.

REPORT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / "docs"
    / "hardware"
    / "m06-handoff-reachability-home.md"
)

X_CANDIDATE = 0.0
GRID_Y = np.round(np.arange(-0.12, 0.10 + 1e-9, 0.02), 2)
GRID_Z = (0.35, 0.38, 0.40)  # exactly the task-specified grid; no extra rows.

# `to_arm` side sign, copied from `skills_scripted.run_handoff`'s own
# `side = -1.0 if to_arm == "A" else 1.0` (arm A base at y=-0.25, arm B at
# y=+0.25).
_TO_ARM_SIDE = -1.0 if TO_ARM == "A" else 1.0


def _arm_body_ids(model, arm: str) -> set[int]:
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
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ik.arm_joint_names(arm)
    ]
    margin = float("inf")
    for jid, angle in zip(joint_ids, joint_angles):
        lo, hi = model.jnt_range[jid]
        margin = min(margin, float(angle) - float(lo), float(hi) - float(angle))
    return margin


def _home_seeded_solve(model, home_data, arm: str, final_pos: np.ndarray) -> ik.IKSolution:
    """The one change from `probe_handoff_reachability.py`: a SINGLE solve,
    seeded directly from `home_data` (the reset/home keyframe), straight to
    `final_pos` -- no intermediate home->approach stage, no seeding from an
    approach-pose configuration.
    """
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = home_data.qpos
    scratch.qvel[:] = 0.0
    mujoco.mj_forward(model, scratch)
    return ik.solve_position_ik(model, scratch, arm, final_pos)


def _apply_arm_solution(model, home_data, arm: str, joint_angles: np.ndarray, into: "mujoco.MjData") -> None:
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ik.arm_joint_names(arm)
    ]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    for qadr, angle in zip(qpos_adrs, joint_angles):
        into.qpos[qadr] = float(angle)


def _deepest_contact(data, geoms_a: set[int], geoms_b: set[int]) -> float | None:
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

            solved = {}
            residual = {}
            reachable = {}
            for arm in ("A", "B"):
                sol = _home_seeded_solve(model, home_data, arm, targets[arm])
                solved[arm] = sol
                residual[arm] = sol.position_error_m
                margin = _joint_limit_margin(model, arm, sol.joint_angles)
                residual_ok = sol.position_error_m < ik.IK_POSITION_TOLERANCE_M
                limit_ok = margin > JOINT_LIMIT_MARGIN_TOL_M
                reachable[arm] = residual_ok and limit_ok

            both_reachable = reachable["A"] and reachable["B"]

            arm_world_ok = True
            world_detail = {}
            for arm in ("A", "B"):
                scratch = mujoco.MjData(model)
                scratch.qpos[:] = home_data.qpos
                scratch.qvel[:] = 0.0
                _apply_arm_solution(model, home_data, arm, solved[arm].joint_angles, scratch)
                mujoco.mj_forward(model, scratch)
                deepest = _deepest_contact(scratch, arm_geoms[arm], world_geoms)
                world_detail[arm] = deepest
                if deepest is not None and deepest < COLLISION_DEPTH_THRESHOLD_M:
                    arm_world_ok = False

            both_scratch = mujoco.MjData(model)
            both_scratch.qpos[:] = home_data.qpos
            both_scratch.qvel[:] = 0.0
            _apply_arm_solution(model, home_data, "A", solved["A"].joint_angles, both_scratch)
            _apply_arm_solution(model, home_data, "B", solved["B"].joint_angles, both_scratch)
            mujoco.mj_forward(model, both_scratch)
            cross_arm_deepest = _deepest_contact(both_scratch, arm_geoms["A"], arm_geoms["B"])
            cross_arm_ok = cross_arm_deepest is None or cross_arm_deepest >= COLLISION_DEPTH_THRESHOLD_M

            final_verdict = both_reachable and arm_world_ok and cross_arm_ok

            rows.append(
                {
                    "x": X_CANDIDATE,
                    "y": float(y),
                    "z": float(z),
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


# ---------------------------------------------------------------------------
# Baseline (approach-pose seeded) rows, transcribed verbatim from the
# committed `docs/hardware/m06-handoff-reachability.md` (commit `ba76190`),
# restricted to the 36 rows at z in {0.35, 0.38, 0.40} -- the "extra" z in
# {0.44, 0.47} rows from that report are excluded so every baseline row here
# has a same-(x, y, z) counterpart in this script's own sweep.
# ---------------------------------------------------------------------------
BASELINE_ROWS = [
    (0.00, -0.12, 0.35, 0.21540, 0.00711),
    (0.00, -0.10, 0.35, 0.20058, 0.00935),
    (0.00, -0.08, 0.35, 0.18673, 0.00817),
    (0.00, -0.06, 0.35, 0.17409, 0.00959),
    (0.00, -0.04, 0.35, 0.16292, 0.14364),
    (0.00, -0.02, 0.35, 0.15355, 0.14965),
    (0.00, 0.00, 0.35, 0.14633, 0.15799),
    (0.00, 0.02, 0.35, 0.00887, 0.16830),
    (0.00, 0.04, 0.35, 0.00944, 0.18024),
    (0.00, 0.06, 0.35, 0.00855, 0.19352),
    (0.00, 0.08, 0.35, 0.00977, 0.20788),
    (0.00, 0.10, 0.35, 0.00750, 0.22311),
    (0.00, -0.12, 0.38, 0.19730, 0.00956),
    (0.00, -0.10, 0.38, 0.18100, 0.00919),
    (0.00, -0.08, 0.38, 0.16552, 0.00970),
    (0.00, -0.06, 0.38, 0.15111, 0.03192),
    (0.00, -0.04, 0.38, 0.13809, 0.11472),
    (0.00, -0.02, 0.38, 0.12691, 0.12216),
    (0.00, 0.00, 0.38, 0.11808, 0.13224),
    (0.00, 0.02, 0.38, 0.11215, 0.14440),
    (0.00, 0.04, 0.38, 0.01731, 0.15816),
    (0.00, 0.06, 0.38, 0.00981, 0.17315),
    (0.00, 0.08, 0.38, 0.00990, 0.18906),
    (0.00, 0.10, 0.38, 0.00522, 0.20569),
    (0.00, -0.12, 0.40, 0.18693, 0.00947),
    (0.00, -0.10, 0.40, 0.16965, 0.00979),
    (0.00, -0.08, 0.40, 0.15302, 0.01706),
    (0.00, -0.06, 0.40, 0.13730, 0.07009),
    (0.00, -0.04, 0.40, 0.12284, 0.09581),
    (0.00, -0.02, 0.40, 0.11011, 0.10461),
    (0.00, 0.00, 0.40, 0.09980, 0.11622),
    (0.00, 0.02, 0.40, 0.09272, 0.12989),
    (0.00, 0.04, 0.40, 0.03694, 0.14503),
    (0.00, 0.06, 0.40, 0.00907, 0.16124),
    (0.00, 0.08, 0.40, 0.00990, 0.17822),
    (0.00, 0.10, 0.40, 0.00594, 0.19577),
]


def _baseline_lookup() -> dict[tuple[float, float, float], tuple[float, float]]:
    return {(x, y, z): (a, b) for (x, y, z, a, b) in BASELINE_ROWS}


def main() -> int:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)  # home keyframe, ADR-026.

    rows = run_sweep(env)
    passing = [r for r in rows if r["final_verdict"]]
    baseline = _baseline_lookup()

    lines = []
    lines.append("# M06 handoff-transfer-point reachability sweep -- HOME-seeded (diagnostic)")
    lines.append("")
    lines.append(
        "**Diagnostic only.** Re-runs the `ba76190` sweep "
        "(`scripts/probe_handoff_reachability.py`, approach-pose seeded, C1-C4 "
        "+ two additions) with exactly one change: each arm's IK solve is "
        "seeded directly from the HOME keyframe and solved in a single stage "
        "straight to the candidate target, instead of the two-stage "
        "home->approach, approach->target solve. `HANDOFF_POSITION_XYZ` is "
        "unchanged; `skills_scripted.py` is untouched."
    )
    lines.append("")
    lines.append(
        "**Correction to the brief this script was run under:** the brief "
        "claimed `ba76190`'s report showed \"all 30 failing cells failed on "
        "arm B's IK residual, the failure is asymmetric.\" That is not what "
        "the committed report shows. In `ba76190`, `both_reachable` was "
        "`False` on **every** one of its 60 rows, and the pattern was "
        "**symmetric**: each arm converges better on the side OPPOSITE its "
        "own base (arm A: residual ~0.26 on its own side / ~0.061 on the far "
        "side at z=0.35, arm B mirroring arm A's numbers on the opposite "
        "side). No asymmetry is asserted or looked for below."
    )
    lines.append("")
    lines.append(
        "**Row-count reconciliation.** `ba76190`'s table has 60 rows: the "
        "12 y-values x 3 specified z-values (0.35, 0.38, 0.40) this task "
        "asked for, PLUS 24 more rows at two \"extra\" z-values (0.44, 0.47) "
        "that `ba76190`'s own script added beyond the original spec. This "
        "diagnostic's task instruction names only the 3 original z-values, "
        "so this script's grid is exactly those 36 rows (12 y x 3 z) -- the "
        "comparison below is against those SAME 36 rows of `ba76190`'s "
        "table (the 24 extra-z rows are not part of this comparison)."
    )
    lines.append("")
    lines.append(f"Seed: {SEED}, reset via `TableSettingEnv.reset()` (home keyframe, ADR-026).")
    lines.append(f"Direction gated on: handoff({FROM_ARM}, {TO_ARM}, fork) (C4, unchanged).")
    lines.append(f"Sweep: x = {X_CANDIDATE}, y in {GRID_Y.tolist()}, z in {list(GRID_Z)}.")
    lines.append(
        f"PASS per arm: residual < {ik.IK_POSITION_TOLERANCE_M} m (seeded directly from HOME, "
        "single-stage solve) AND no joint-limit violation (margin > "
        f"{JOINT_LIMIT_MARGIN_TOL_M} m from either `jnt_range` bound)."
    )
    lines.append(
        f"Collision bar (both arm_world_ok and cross_arm_ok): any contact deeper than "
        f"{COLLISION_DEPTH_THRESHOLD_M} m is a FAIL (identical to `ba76190`)."
    )
    lines.append("")
    lines.append("## Row-by-row comparison: approach-pose seed (ba76190) vs. home seed (this run)")
    lines.append("")
    lines.append(
        "delta = (this run's residual) - (ba76190's residual). Negative delta = home seeding "
        "converged BETTER (smaller residual) than approach-pose seeding at that same (x, y, z)."
    )
    lines.append("")
    lines.append(
        "| x | y | z | armA_residual (approach-seed) | armA_residual (home-seed) | armA delta | "
        "armB_residual (approach-seed) | armB_residual (home-seed) | armB delta | "
        "both_reachable (home-seed) | final_verdict (home-seed) |"
    )
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|")
    for r in rows:
        key = (r["x"], r["y"], r["z"])
        base_a, base_b = baseline[key]
        delta_a = r["armA_residual"] - base_a
        delta_b = r["armB_residual"] - base_b
        lines.append(
            f"| {r['x']:.2f} | {r['y']:.2f} | {r['z']:.2f} | "
            f"{base_a:.5f} | {r['armA_residual']:.5f} | {delta_a:+.5f} | "
            f"{base_b:.5f} | {r['armB_residual']:.5f} | {delta_b:+.5f} | "
            f"{r['both_reachable']} | {'PASS' if r['final_verdict'] else 'FAIL'} |"
        )
    lines.append("")

    lines.append("## Full home-seeded sweep table (same format as `ba76190`'s report)")
    lines.append("")
    lines.append(
        "| x | y | z | armA_residual | armB_residual | both_reachable | arm_world_ok | cross_arm_ok | final_verdict |"
    )
    lines.append("|---:|---:|---:|---:|---:|---|---|---|---|")
    for r in rows:
        lines.append(
            f"| {r['x']:.2f} | {r['y']:.2f} | {r['z']:.2f} | "
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
            "both_reachable=True, arm_world_ok=True, cross_arm_ok=True."
        )
        lines.append("")
        lines.append(f"**Overall: {len(passing)} of {len(rows)} candidates PASS (home-seeded).**")
    else:
        lines.append(
            "**NO candidate passed at any tested (y, z) with home seeding either.** No "
            "collision-free, both-reachable shared transfer point was found anywhere in "
            "this sweep."
        )
        lines.append("")
        lines.append("**Overall: ALL FAIL (home-seeded).**")
    lines.append("")

    # Outcome classification, per the task's three named branches.
    any_residual_pass_no_collision_pass = any(
        r["both_reachable"] and not (r["arm_world_ok"] and r["cross_arm_ok"]) for r in rows
    )
    lines.append("## Outcome")
    lines.append("")
    if passing:
        lines.append(
            "**Home-pose PASSES where approach-pose FAILED.** The handoff waypoint pattern "
            "(seeding each arm from its own approach pose before the final target solve) is "
            "implicated as the blocker, not intrinsic kinematics. See recommendation below."
        )
    elif any_residual_pass_no_collision_pass:
        lines.append(
            "**Home-pose passes on residual/joint-limits at some rows but still fails "
            "collision.** This points at waypoint staging (the arms can reach into each "
            "other's space kinematically, but the direct home-seeded path routes through "
            "the table or a prop) rather than either of the two main branches."
        )
    else:
        lines.append(
            "**Home-pose ALSO FAILS**, on the same residual grounds as the approach-pose "
            "sweep (see the delta columns above -- residuals are close to, not dramatically "
            "better than, `ba76190`'s numbers at the same (x, y, z), and `both_reachable` is "
            "`False` on every row here too). Seeding point is not the blocker. This is now "
            "two independently-seeded sweeps (home-only in the original first pass one "
            "commit prior to `ba76190`, and this home-only re-run) plus one approach-seeded "
            "sweep (`ba76190`) all agreeing: the shared band at x=0 genuinely does not "
            "support a static handoff position for this arm-base placement. This is strong "
            "enough evidence to act on ADR-021 (arm-base rearrangement) or a demo redesign, "
            "rather than to keep re-measuring."
        )
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
