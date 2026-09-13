"""Re-measure the shared handoff transfer point (fix for the M06 `handoff`
regression: arm B cannot reach the current `HANDOFF_POSITION_XYZ`).

**Why this script exists, not just trusting `m06-reachability-probe.md`'s
"Shared handoff band (y-range intersection): y in [-0.12, 0.10]" line.**
That band came from `run_envelope_sweep` in `probe_reachability.py`, which
(per its own docstring, "collision not checked for the sweep") only checks
IK residual convergence -- NOT collision. A grid point can converge to a
kinematically-valid position while the solved joint configuration drives an
arm segment through the table. This script re-checks the SAME kind of
candidate (a shared-band transfer point) with BOTH halves of the PASS bar
`probe_reachability.py`'s own PRIMARY probes use: residual < tolerance AND
no NEW collision (a contact-count delta against the home-pose baseline,
computed with the exact same machinery -- `_arm_body_ids`/`_arm_geom_ids`/
`_count_arm_contacts`/`_apply_solution_and_check_collision`, copied here
rather than imported so this script has no import-time dependency on
`probe_reachability.py`'s module-level grid constants).

Sweep: x=0 (centreline, matching `HANDOFF_POSITION_XYZ`'s existing x=0),
y from -0.12 to 0.10 in 0.02 m steps (the previously-reported band's own
extent), z in {0.35, 0.38, 0.40} (0.35 = `TABLE_SURFACE_Z`, the current
constant's height; 0.38/0.40 = 3 cm / 5 cm above the table, in case the
tabletop height itself is the problem -- see module note on z=0.35 below).

**Both `both_pass` is measured, not assumed identical to `probe_reachability.
py`'s single-arm PASS.** For a handoff, BOTH arms must simultaneously (well,
each independently reach it collision-free -- the transfer happens with the
two solves computed independently, matching how `run_handoff` actually
drives each arm one at a time in `skills_scripted.py`, so this script's
per-arm-independent solve/collision-check IS the physically relevant check,
not a simplification of it) pass. `both_pass = arm_A_residual < TOL and
arm_B_residual < TOL and not arm_A_collision and not arm_B_collision`.

Expectation, stated up front per instruction: z=0.35 is exactly the tabletop
surface (`TABLE_SURFACE_Z` in `skills_scripted.py`), so candidates at that
height are the ones most likely to have an arm segment graze or tunnel
through the table -- a FAIL there is an expected, not anomalous, outcome.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`. (Also verified to import locally on this developer's laptop, per
the precedent recorded in DECISIONS.md's ADR-029 entry -- not relied upon;
the authoritative run and committed artifacts are bm-ptl's.)
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

SEED = 0
ARMS = ("A", "B")

REPORT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / "docs" / "hardware" / "m06-handoff-reachability.md"
)

X_CANDIDATE = 0.0
GRID_Y = np.round(np.arange(-0.12, 0.10 + 1e-9, 0.02), 2)
GRID_Z = (0.35, 0.38, 0.40)


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _arm_body_ids(model, arm: str) -> set[int]:
    """Every body id whose name is prefixed `arm{arm}_` -- the arm's whole
    kinematic subtree, per `scripts/gen_dual_scene.py`'s renaming convention.
    Copied from `probe_reachability.py` (see module docstring for why this
    is a copy, not an import).
    """
    prefix = f"arm{arm}_"
    ids = set()
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name and name.startswith(prefix):
            ids.add(b)
    return ids


def _arm_geom_ids(model, body_ids: set[int]) -> set[int]:
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}


def _count_arm_contacts(data, arm_geom_ids: set[int]) -> int:
    n = 0
    for i in range(data.ncon):
        c = data.contact[i]
        if int(c.geom1) in arm_geom_ids or int(c.geom2) in arm_geom_ids:
            n += 1
    return n


def _baseline_arm_contacts(model, data) -> dict[str, int]:
    """Contacts involving each arm's geoms at the (home-keyframe) reset
    pose, before any IK solve -- same measured-not-assumed convention as
    `probe_reachability.py`'s `_baseline_arm_contacts`.
    """
    baseline = {}
    for arm in ARMS:
        arm_geoms = _arm_geom_ids(model, _arm_body_ids(model, arm))
        baseline[arm] = _count_arm_contacts(data, arm_geoms)
    return baseline


def _apply_solution_and_check_collision(
    model, data, arm: str, solution: ik.IKSolution, baseline_contacts: int
) -> tuple[bool, int]:
    """Write `solution.joint_angles` into a scratch MjData (starting from
    `data`'s current qpos so the OTHER arm and every prop stay exactly where
    they are), run mj_forward, and report whether this arm's solved pose
    creates any NEW contact beyond `baseline_contacts`.

    Returns (collision, n_contacts_for_this_arm).
    """
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    scratch.qvel[:] = 0.0

    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ik.arm_joint_names(arm)]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    for qadr, angle in zip(qpos_adrs, solution.joint_angles):
        scratch.qpos[qadr] = float(angle)

    mujoco.mj_forward(model, scratch)

    arm_geoms = _arm_geom_ids(model, _arm_body_ids(model, arm))
    n_contacts = _count_arm_contacts(scratch, arm_geoms)
    return n_contacts > baseline_contacts, n_contacts


def run_sweep(env: TableSettingEnv) -> list[dict]:
    model, data = env.model, env.data
    baseline_contacts = _baseline_arm_contacts(model, data)

    rows = []
    for z in GRID_Z:
        for y in GRID_Y:
            target = np.array([X_CANDIDATE, float(y), float(z)])
            row = {"x": X_CANDIDATE, "y": float(y), "z": float(z)}
            per_arm_pass = {}
            for arm in ARMS:
                solution = ik.solve_position_ik(model, data, arm, target)
                collision, n_contacts = _apply_solution_and_check_collision(
                    model, data, arm, solution, baseline_contacts[arm]
                )
                residual_ok = solution.position_error_m < ik.IK_POSITION_TOLERANCE_M
                row[f"arm_{arm}_residual"] = solution.position_error_m
                row[f"arm_{arm}_collision"] = collision
                row[f"arm_{arm}_n_contacts"] = n_contacts
                per_arm_pass[arm] = residual_ok and not collision
            row["both_pass"] = per_arm_pass["A"] and per_arm_pass["B"]
            rows.append(row)
    return rows, baseline_contacts


def main() -> int:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)  # home keyframe, ADR-026.

    rows, baseline_contacts = run_sweep(env)
    any_pass = any(r["both_pass"] for r in rows)

    lines = []
    lines.append("# M06 handoff-transfer-point reachability probe (fix for the `handoff` waypoint-3 regression)")
    lines.append("")
    lines.append(
        "Re-measures the shared handoff band with BOTH IK residual AND collision checked "
        "(the previously-reported `m06-reachability-probe.md` \"Shared handoff band\" line came "
        "from a residual-only sweep -- see this script's module docstring)."
    )
    lines.append("")
    lines.append(f"Seed: {SEED}, reset via `TableSettingEnv.reset()` (home keyframe, ADR-026).")
    lines.append(f"Sweep: x = {X_CANDIDATE}, y in {GRID_Y.tolist()}, z in {list(GRID_Z)}.")
    lines.append(
        f"PASS (per arm) requires residual < {ik.IK_POSITION_TOLERANCE_M} m and no NEW collision "
        "(contact-count delta against the home-pose baseline, exactly as `probe_reachability.py` "
        "computes it for its own primary probes)."
    )
    lines.append(
        "Measured baseline (contacts involving each arm's geoms at reset, before any IK solve): "
        + ", ".join(f"arm {arm}={n}" for arm, n in baseline_contacts.items())
    )
    lines.append("")
    lines.append(
        "Note: z=0.35 is exactly the tabletop surface (`TABLE_SURFACE_Z`), so candidates at that "
        "height are the most likely to collide with the table -- expected, not anomalous."
    )
    lines.append("")
    lines.append(
        "| x | y | z | arm_A_residual | arm_A_collision | arm_B_residual | arm_B_collision | both_pass |"
    )
    lines.append("|---:|---:|---:|---:|---|---:|---|---|")
    for r in rows:
        lines.append(
            f"| {r['x']:.2f} | {r['y']:.2f} | {r['z']:.2f} | "
            f"{r['arm_A_residual']:.5f} | {r['arm_A_collision']} | "
            f"{r['arm_B_residual']:.5f} | {r['arm_B_collision']} | "
            f"{'PASS' if r['both_pass'] else 'FAIL'} |"
        )
    lines.append("")

    passing = [r for r in rows if r["both_pass"]]
    if passing:
        # Prefer y=0 if it qualifies, else the qualifying point nearest y=0.
        chosen = min(passing, key=lambda r: (abs(r["y"]), r["z"]))
        lines.append(
            f"**Chosen candidate: (x={chosen['x']:.2f}, y={chosen['y']:.2f}, z={chosen['z']:.2f})** -- "
            f"arm_A_residual={chosen['arm_A_residual']:.5f}, arm_B_residual={chosen['arm_B_residual']:.5f}, "
            "both collision-free."
        )
    else:
        lines.append(
            "**NO candidate passed for both arms at any tested (y, z).** No collision-free shared "
            "reach point was found in this sweep."
        )
    lines.append("")
    lines.append(f"**Overall: {'AT LEAST ONE PASS' if any_pass else 'ALL FAIL'}**")
    lines.append("")

    report = "\n".join(lines) + "\n"
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report, newline="\n")

    print(report)
    print(f"(report written to {REPORT_PATH})")

    env.close()
    return 0 if any_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
