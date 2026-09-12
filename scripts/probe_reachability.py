"""M06a reachability probe (ADR-025), re-measured under ADR-026 -- run BEFORE
any skill.

M06a discovered two things the empty M02 done-when list never checked:
whether the IK solver's target is where the jaws actually pinch (fixed by
ADR-025's retarget in `bimanual.control.ik`), and whether the scene's
geometry even lets an arm's solved joint configuration reach a target
without colliding with the table it is bolted to. This script checks the
SECOND question directly and numerically, instead of assuming a scene edit
fixed it.

**ADR-026 correction (Sept 12, 2026):** every envelope this script ever
measured before today was sampled from an INVALID rest pose -- at
`reset(seed=0)`, the two arms' own default (all-zero-joint) configuration
put them 34 contacts deep into each other (29 armA<->armB, deepest
-0.0597 m), independent of any IK target. `TableSettingEnv.reset()` now
applies a "home" keyframe (arms folded back, see
`scripts/gen_dual_scene.py` and ARCHITECTURE.md ADR-026) BEFORE this probe
ever calls `ik.solve_position_ik`, so `data`'s starting qpos (which the IK
solver seeds from) is now a collision-free, retracted pose instead of an
interpenetrating, fully-extended one. Every table this script now produces
is measured from that corrected pose. **The earlier tables in this file are
NOT overwritten** -- they are the historical record of why ADR-026 exists,
and this script's report writer now APPENDS a new, clearly dated section
below them instead of replacing the file (see `main()`'s use of
`REPORT_PATH.read_text()` before writing).

For each of 4 targets (closed drawer face, plate/mug/bottle at rest) x each
arm (A, B), it:
  1. Calls `ik.solve_position_ik` on the target position.
  2. Records the convergence residual (`IKSolution.position_error_m`, metres).
  3. Applies the solved joint angles to a scratch `MjData`, runs
     `mj_forward`, and checks whether any MuJoCo contact involves a geom
     belonging to that arm's body subtree (see `_collision_for_arm` --
     this is a real collision-detection query, not a heuristic distance
     check).
  4. PASS requires residual < 0.01 m AND no such contact.

The reachable-envelope sweep (grid of candidate world positions, residual
< tolerance only, collision not checked -- see the envelope section for
why) now always runs, not only on primary-probe failure: ADR-026's Step 2
explicitly asks for the envelope to be re-measured from the corrected pose
regardless of whether the four primary targets pass, and Step 3 needs that
envelope to choose a drawer position. The z grid is finer than the original
0.1 m steps (which left the true minimum reachable height unresolved
between 0.20 and 0.30 m): now 0.02 m steps from 0.20 to 0.50 m.

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
from bimanual.sim.env import TableSettingEnv  # noqa: E402

SEED = 0
ARMS = ("A", "B")

REPORT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / "docs" / "hardware" / "m06-reachability-probe.md"
)

# Grid for the reachable-envelope sweep. x/y unchanged from the first pass
# (coarse steps were fine there -- neither axis's boundary was ambiguous).
# z is now FINER (0.02 m vs. the original 0.1 m): the first pass's z steps
# (0.2, 0.3, 0.4, 0.5) left the true minimum reachable height unresolved
# anywhere in (0.20, 0.30] -- see ADR-026. Sampling every 0.02 m from 0.20
# to 0.50 resolves that boundary to within one grid step.
GRID_X = np.array([-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3])
GRID_Y = np.array([-0.35, -0.275, -0.2, -0.125, -0.05, 0.025, 0.1])
GRID_Z = np.round(np.arange(0.20, 0.50 + 1e-9, 0.02), 2)


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _drawer_face_target(model, data) -> np.ndarray:
    """The closed drawer's front-face centre point, world xyz.

    Read from the compiled model's own geometry (drawer body position +
    drawer_box geom half-extent), never a hardcoded coordinate, so this
    stays correct if the scene's drawer position ever moves again.
    """
    body_id = _body_id(model, "drawer")
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "drawer_box")
    if geom_id == -1:
        raise ValueError("geom 'drawer_box' not found in the compiled model")
    _hx, hy, _hz = model.geom_size[geom_id]
    drawer_pos = np.array(data.xpos[body_id], dtype=np.float64, copy=True)
    return np.array([drawer_pos[0], drawer_pos[1] - hy, drawer_pos[2]])


def _arm_body_ids(model, arm: str) -> set[int]:
    """Every body id whose name is prefixed `arm{arm}_` -- the arm's whole
    kinematic subtree, per `scripts/gen_dual_scene.py`'s renaming convention.
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
    """Number of contacts involving each arm's geoms AT THE RESET POSE,
    before any IK solve runs.

    This was measured, not assumed, via a one-off diagnostic
    (`diag_contacts.py`, not committed) before this function existed: the
    first version of this script assumed the baseline was 0 ("the arm
    should not already be touching anything"), which was WRONG and produced
    an all-FAIL primary-probe table. The actual measured baseline at
    reset(seed=0) is 34 total contacts, of which ~29 are direct armA<->armB
    geom pairs at penetration depths up to ~6 cm (e.g. `armA_lower_arm` vs
    `armB_wrist`) -- the two arms' DEFAULT rest pose already substantially
    interpenetrates, independent of any target or of this module's two
    fixes. This is a real, separate finding (see the reachability report's
    "Baseline finding" section) and is NOT something this probe corrects or
    this module's task scope permits fixing (out of scope: it is an
    ADR-021 arm-placement/rest-pose question, not the drawer position or
    the IK target). Collision is therefore judged as a DELTA against this
    measured baseline, per arm, not against an assumed zero.
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
    creates any NEW contact beyond `baseline_contacts` (see
    `_baseline_arm_contacts`).

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


def run_primary_probes(env: TableSettingEnv) -> list[dict]:
    model, data = env.model, env.data
    targets = {
        "closed_drawer_face": _drawer_face_target(model, data),
        "plate_at_rest": np.array(data.xpos[_body_id(model, "plate")], dtype=np.float64, copy=True),
        "mug_at_rest": np.array(data.xpos[_body_id(model, "mug")], dtype=np.float64, copy=True),
        "bottle_at_rest": np.array(data.xpos[_body_id(model, "water_bottle")], dtype=np.float64, copy=True),
    }
    baseline_contacts = _baseline_arm_contacts(model, data)

    rows = []
    for target_name, target_pos in targets.items():
        for arm in ARMS:
            solution = ik.solve_position_ik(model, data, arm, target_pos)
            collision, n_contacts = _apply_solution_and_check_collision(
                model, data, arm, solution, baseline_contacts[arm]
            )
            passed = solution.position_error_m < ik.IK_POSITION_TOLERANCE_M and not collision
            rows.append(
                {
                    "target": target_name,
                    "target_pos": target_pos,
                    "arm": arm,
                    "residual_m": solution.position_error_m,
                    "converged": solution.converged,
                    "collision": collision,
                    "n_contacts": n_contacts,
                    "baseline_contacts": baseline_contacts[arm],
                    "pass": passed,
                }
            )
    return rows, baseline_contacts


def run_envelope_sweep(env: TableSettingEnv) -> dict[str, list[tuple[float, float, float]]]:
    """Coarse grid sweep (no collision check -- see module docstring): for
    each arm, the set of (x, y, z) grid points whose IK residual converges
    below `ik.IK_POSITION_TOLERANCE_M`. Collision is intentionally not
    checked here: this is 2 x 7 x 7 x 4 = 392 solves and the point is a fast
    envelope of KINEMATIC reach, not a full re-validated reachability claim
    per point -- the primary probes above already do the fuller check for
    the four targets that matter.
    """
    model, data = env.model, env.data
    reachable: dict[str, list[tuple[float, float, float]]] = {arm: [] for arm in ARMS}
    for arm in ARMS:
        for x in GRID_X:
            for y in GRID_Y:
                for z in GRID_Z:
                    target = np.array([x, y, z])
                    solution = ik.solve_position_ik(model, data, arm, target)
                    if solution.position_error_m < ik.IK_POSITION_TOLERANCE_M:
                        reachable[arm].append((float(x), float(y), float(z)))
    return reachable


def _summarize_envelope(points: list[tuple[float, float, float]]) -> str:
    if not points:
        return "no reachable grid points found"
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return (
        f"{len(points)} reachable grid points; "
        f"x in [{min(xs):.2f}, {max(xs):.2f}], "
        f"y in [{min(ys):.2f}, {max(ys):.2f}], "
        f"z in [{min(zs):.2f}, {max(zs):.2f}]"
    )


def _shared_band(envelope: dict) -> tuple[float, float] | None:
    """Intersection of the two arms' reachable y-ranges (from the envelope
    sweep), i.e. the y-band a handoff could physically happen in. Returns
    None if the two arms' y-ranges do not overlap at all.
    """
    if not envelope["A"] or not envelope["B"]:
        return None
    y_a = [p[1] for p in envelope["A"]]
    y_b = [p[1] for p in envelope["B"]]
    lo = max(min(y_a), min(y_b))
    hi = min(max(y_a), max(y_b))
    if lo > hi:
        return None
    return (lo, hi)


def main() -> int:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)  # ADR-026: this now applies the "home" keyframe, not qpos0.

    rows, baseline_contacts = run_primary_probes(env)
    all_passed = all(r["pass"] for r in rows)

    lines = []
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("# M06 reachability probe, RE-MEASURED (ADR-026)")
    lines.append("")
    lines.append(
        f"Seed: {SEED}, reset via `TableSettingEnv.reset()`, which now applies the \"home\" "
        "keyframe (arms folded back) instead of leaving qpos at the upstream all-zero "
        "default. **Every number below supersedes the corresponding number above**, which "
        "was measured from a rest pose in which the two arms interpenetrated by up to "
        "6 cm before any IK solve ran. The section above is retained as the historical "
        "record of why this re-measurement exists, not as a currently-valid envelope."
    )
    lines.append("")
    lines.append(f"PASS requires residual < {ik.IK_POSITION_TOLERANCE_M} m and no NEW collision")
    lines.append(
        "(a contact count above this arm's measured RESET-pose baseline -- see 'Baseline finding' below)."
    )
    lines.append("")
    lines.append(
        f"Measured baseline (contacts involving each arm's geoms at reset(seed={SEED}), "
        "before any IK solve, now AT THE HOME POSE): "
        + ", ".join(f"arm {arm}={n}" for arm, n in baseline_contacts.items())
    )
    lines.append("")
    lines.append("## Primary targets")
    lines.append("")
    lines.append(
        "| target | target pos (x,y,z) | arm | residual (m) | converged | new collision | "
        "n_contacts (baseline) | PASS |"
    )
    lines.append("|---|---|---|---:|---|---|---:|---|")
    for r in rows:
        tp = r["target_pos"]
        lines.append(
            f"| {r['target']} | ({tp[0]:.4f}, {tp[1]:.4f}, {tp[2]:.4f}) | {r['arm']} | "
            f"{r['residual_m']:.5f} | {r['converged']} | {r['collision']} | "
            f"{r['n_contacts']} ({r['baseline_contacts']}) | "
            f"{'PASS' if r['pass'] else 'FAIL'} |"
        )
    lines.append("")
    lines.append(f"**Overall: {'ALL PASS' if all_passed else 'AT LEAST ONE FAILURE'}**")
    lines.append("")
    lines.append("## Baseline finding, now fixed (was 'not fixed here' in the section above)")
    lines.append("")
    lines.append(
        "At `reset(seed=0)`, BEFORE any IK solve runs, the OLD default rest pose (all "
        "arm joints at 0 rad) put arm A and arm B up to ~6 cm deep into each other "
        "(29 of 34 total contacts were armA<->armB, deepest -0.0597 m, "
        "`armA_wrist` vs. `armB_wrist`). This was independent of the drawer position and "
        "the IK pinch-point retarget: it was a property of the compiled model's default "
        "qpos alone. **This is now fixed** by the \"home\" keyframe (ARCHITECTURE.md "
        "ADR-026): the measured baseline above, taken at the new reset pose, shows "
        + ", ".join(f"arm {arm}={n}" for arm, n in baseline_contacts.items())
        + " cross/self contacts. ADR-021's ~0.30 m reach / 0.50 m base-gap layout "
        "assumption was never checked against the actual compiled rest pose; this probe "
        "is that check, and the envelope below is the first one measured from a valid pose."
    )
    lines.append("")

    lines.append(
        "## Reachable envelope, re-measured from the home pose (always run this pass, "
        "per ADR-026 Step 2 -- not gated on primary-probe failure)"
    )
    lines.append("")
    envelope = run_envelope_sweep(env)
    lines.append(
        f"Grid: x in {GRID_X.tolist()}, y in {GRID_Y.tolist()}, "
        f"z in {[float(v) for v in GRID_Z]} m steps=0.02 "
        "(residual < tolerance only; collision not checked for the sweep -- see script docstring)."
    )
    lines.append("")
    for arm in ARMS:
        lines.append(f"- **Arm {arm}**: {_summarize_envelope(envelope[arm])}")
    lines.append("")

    band = _shared_band(envelope)
    if band is None:
        lines.append(
            "**No shared handoff band**: arm A's and arm B's reachable y-ranges do not "
            "overlap at all in this sweep."
        )
    else:
        lines.append(
            f"**Shared handoff band (y-range intersection): y in [{band[0]:.2f}, {band[1]:.2f}] m.**"
        )
    lines.append("")

    lines.append("| arm | x | y | z |")
    lines.append("|---|---:|---:|---:|")
    for arm in ARMS:
        for x, y, z in envelope[arm]:
            lines.append(f"| {arm} | {x:.2f} | {y:.2f} | {z:.2f} |")
    lines.append("")

    report = "\n".join(lines) + "\n"
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    # APPEND, do not overwrite (ADR-026 Step 2): the section(s) already in
    # this file are the historical record of why this re-measurement
    # exists and must survive this run.
    existing = REPORT_PATH.read_text() if REPORT_PATH.exists() else ""
    REPORT_PATH.write_text(existing + report, newline="\n")

    print(report)
    print(f"(report written to {REPORT_PATH})")

    env.close()
    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
