"""Redesign Stage 1: empirical reachable-workspace measurement, per arm.

**Measurement only.** This script does not modify `gen_dual_scene.py`, any
generated XML, `skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`,
`env.py`, `scenes/so101/`, or any requirements file. It only reads the
compiled model/data and the public functions `bimanual.control.ik` already
exposes (`arm_joint_names`, `fixed_jaw_body_name`, `moving_jaw_body_name`,
`solve_position_ik`) -- the same pattern `scripts/probe_reachability.py`
already uses for its own reachability probes.

**Why this script exists (see docs/hardware/redesign-workspace-measurement.md
and ARCHITECTURE.md ADR-058 for the full report).** Every prior reachability
finding in this repo (ADR-025, ADR-032, ADR-035, ADR-048, ADR-057) was
measured incidentally, while chasing a specific skill failure, against a
handful of hand-picked grid points. This script instead samples each arm's
**actual** joint-limit box (`model.jnt_range`, read from the compiled model,
never assumed) uniformly at N=50000 configurations, records where the pinch
point ends up, and reports the two arms' reachable clouds and their
intersection -- the question Stage 2 (arm-base placement) depends on.

**The 5-DoF sampling rule (ADR-016).** Each arm has 6 `<position>` actuators,
but the 6th (`armX_gripper`) only opens/closes the jaw and contributes
nothing to the pinch-point's position -- `ik.py`'s own module docstring
documents this DoF split in detail. This script therefore samples only the
5 joints `ik.arm_joint_names(arm)` returns (shoulder_pan, shoulder_lift,
elbow_flex, wrist_flex, wrist_roll), each uniformly over its own
`model.jnt_range`, exactly the joints IK itself ever moves.

**The pinch point, not the gripperframe site (ADR-025).** The recorded point
for each sample is the midpoint of `armX_gripper` (the fixed jaw body) and
`armX_moving_jaw_so101_v1` (the moving jaw body) -- the same point
`ik.solve_position_ik`'s internal `_pinch_point()` targets and
`scripts/verify_adr038_skills.py`'s own `pinch()` helper computes for its
regression printouts. `armX_gripperframe` sits ~8.9 cm away (this session's
own measurement, see the report) and would describe the wrong cloud.

**The table_top penetration filter, and why it is reported at more than one
threshold.** ADR-035's own warm-start diagnostic (Part 3,
`docs/hardware/m06-handoff-warmstart-diagnostic.md`) found a `table_top`
vs. arm-mesh contact reporting **-0.22211 m** penetration depth on a
CONVERGED, physically sensible pose -- a `type="mesh"` convex-hull collision
artifact (MuJoCo's default hull treatment of a non-convex mesh can report
grossly exaggerated depth relative to the visual mesh), not real
interpenetration, and it was explicitly left uncalibrated by that ADR ("out
of scope"). A naive 1 mm depth bar (matching
`skills_scripted.TABLE_COLLISION_DEPTH_TOL_M`, reproduced here as a bare
float constant rather than an import, since importing from
`skills_scripted` for a threshold value is unnecessary coupling for a
read-only probe) could therefore reject real, usable workspace for the
wrong reason. This script does NOT decide that question by fiat: it
records, for every sample, the worst (most negative) `table_top` contact
depth found for that arm's geoms, so the report can show the raw cloud,
the 1 mm-filtered cloud, and the sanity check against four independently
known-good targets (fork/mug/bottle at-rest positions, ADR-025's own
primary-probe targets, plus the current `HANDOFF_POSITION_XYZ`) side by
side, and state which the report treats as authoritative.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

ARMS = ("A", "B")
RESET_SEED = 0

# Per-arm sampling RNG seeds. Fixed and distinct so arm A and arm B draw
# independent, but fully reproducible, samples -- re-running this script
# reproduces the identical cloud, which matters for a measurement whose
# whole point is to be trusted (ADR-047 cross-machine provenance).
SAMPLE_SEEDS = {"A": 20260915, "B": 20260916}

#: Depth bar matching `skills_scripted.TABLE_COLLISION_DEPTH_TOL_M`'s own
#: value (0.001 m). Reproduced as a bare constant, not an import, since this
#: is a read-only probe and the task's own do-not-touch list is about
#: behavior, not about a diagnostic script being allowed to know a published
#: number. See module docstring for why this bar alone is not trusted blind.
NAIVE_TABLE_TOL_M = 0.001

#: A second, much looser bar for comparison -- if the naive 1 mm bar and this
#: one disagree wildly on rejection count, that itself is evidence the 1 mm
#: bar is catching the ADR-035 mesh artifact, not real interpenetration.
LOOSE_TABLE_TOL_M = 0.02

Z_SLICE_LO = 0.35
Z_SLICE_HI = 0.60
Z_SLICE_STEP = 0.02
XY_RES_M = 0.01

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "out" / "workspace_measurement"
REPORT_PATH = REPO_ROOT / "docs" / "hardware" / "redesign-workspace-measurement.md"


# ---------------------------------------------------------------------------
# Model introspection helpers (same style as scripts/probe_reachability.py)
# ---------------------------------------------------------------------------


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _arm_body_ids(model, arm: str) -> set[int]:
    prefix = f"arm{arm}_"
    ids = set()
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name and name.startswith(prefix):
            ids.add(b)
    return ids


def _arm_geom_ids(model, body_ids: set[int]) -> set[int]:
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}


def _table_top_geom_id(model) -> int:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    if gid == -1:
        raise ValueError("geom 'table_top' not found in the compiled model")
    return gid


def _pinch_point(data, fixed_id: int, moving_id: int) -> np.ndarray:
    """ADR-025 pinch point: midpoint of the two jaw bodies' world origins."""
    return 0.5 * (np.asarray(data.xpos[fixed_id]) + np.asarray(data.xpos[moving_id]))


def _worst_table_depth(data, arm_geoms: set[int], table_gid: int) -> float:
    """Most-negative `table_top` contact distance touching this arm's geoms,
    or 0.0 if no such contact exists this step. See module docstring for why
    this is reported raw rather than pre-thresholded.
    """
    worst = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in arm_geoms and g2 == table_gid) or (g2 in arm_geoms and g1 == table_gid):
            d = float(c.dist)
            if d < worst:
                worst = d
    return worst


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------


def _arm_sampling_context(model, arm: str):
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ik.arm_joint_names(arm)]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    joint_ranges = np.array([model.jnt_range[j] for j in joint_ids], dtype=np.float64)  # (5, 2)
    fixed_id = _body_id(model, ik.fixed_jaw_body_name(arm))
    moving_id = _body_id(model, ik.moving_jaw_body_name(arm))
    arm_geoms = _arm_geom_ids(model, _arm_body_ids(model, arm))
    return qpos_adrs, joint_ranges, fixed_id, moving_id, arm_geoms


def sample_arm(model, home_qpos: np.ndarray, arm: str, n: int, seed: int, report_every: int = 10000):
    """Uniformly sample `n` configurations of `arm`'s 5 positioning joints
    over `model.jnt_range`, holding everything else (other arm, props,
    drawer) at `home_qpos`. Returns (points (n,3), table_depths (n,),
    joint_ranges (5,2), elapsed_seconds).
    """
    qpos_adrs, joint_ranges, fixed_id, moving_id, arm_geoms = _arm_sampling_context(model, arm)
    table_gid = _table_top_geom_id(model)

    scratch = mujoco.MjData(model)
    scratch.qpos[:] = home_qpos
    scratch.qvel[:] = 0.0

    rng = np.random.default_rng(seed)
    lo, hi = joint_ranges[:, 0], joint_ranges[:, 1]
    draws = rng.uniform(lo, hi, size=(n, 5))

    points = np.empty((n, 3), dtype=np.float64)
    depths = np.empty(n, dtype=np.float64)

    t0 = time.perf_counter()
    for i in range(n):
        angles = draws[i]
        for k, qadr in enumerate(qpos_adrs):
            scratch.qpos[qadr] = angles[k]
        mujoco.mj_forward(model, scratch)
        points[i] = _pinch_point(scratch, fixed_id, moving_id)
        depths[i] = _worst_table_depth(scratch, arm_geoms, table_gid)
        if report_every and (i + 1) % report_every == 0:
            elapsed = time.perf_counter() - t0
            print(f"  arm {arm}: {i + 1}/{n} samples, {elapsed:.1f}s elapsed "
                  f"({1000.0 * elapsed / (i + 1):.4f} ms/sample)", flush=True)
    elapsed = time.perf_counter() - t0
    return points, depths, joint_ranges, elapsed


def _calibrate_impl(model, home_qpos: np.ndarray, arm: str, n_calib: int) -> float:
    """Time a small batch (a distinct RNG seed from the real run, so this
    calibration batch is never accidentally reused as part of the reported
    cloud) and return measured ms/sample for `arm`."""
    _, _, _, elapsed = sample_arm(model, home_qpos, arm, n_calib, seed=999_000 + ord(arm), report_every=0)
    return 1000.0 * elapsed / n_calib


# ---------------------------------------------------------------------------
# Known-good sanity-check targets
# ---------------------------------------------------------------------------


def sanity_check_targets(model, home_data) -> list[dict]:
    """Solve IK toward four independently-known-good targets (ADR-025's own
    primary-probe targets: fork/mug/bottle at rest, plus the current
    `HANDOFF_POSITION_XYZ`) for both arms, and report whether the solved
    config survives the naive 1 mm table_top penetration filter. This is the
    caveat's own required check: if these targets get REJECTED by the
    filter, the filter is untrustworthy (see module docstring / the report's
    'Caveat on the penetration filter' section).
    """
    from bimanual.control.skills_scripted import HANDOFF_POSITION_XYZ  # noqa: E402 (read-only import)

    targets = {
        "fork_at_rest": np.array(home_data.xpos[_body_id(model, "fork")], dtype=np.float64, copy=True),
        "mug_at_rest": np.array(home_data.xpos[_body_id(model, "mug")], dtype=np.float64, copy=True),
        "bottle_at_rest": np.array(home_data.xpos[_body_id(model, "water_bottle")], dtype=np.float64, copy=True),
        "handoff_point": np.array(HANDOFF_POSITION_XYZ, dtype=np.float64),
    }

    table_gid = _table_top_geom_id(model)
    rows = []
    for name, target in targets.items():
        for arm in ARMS:
            solution = ik.solve_position_ik(model, home_data, arm, target)
            _, _, fixed_id, moving_id, arm_geoms = _arm_sampling_context(model, arm)
            joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ik.arm_joint_names(arm)]
            qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
            scratch = mujoco.MjData(model)
            scratch.qpos[:] = home_data.qpos
            scratch.qvel[:] = 0.0
            for qadr, angle in zip(qpos_adrs, solution.joint_angles):
                scratch.qpos[qadr] = float(angle)
            mujoco.mj_forward(model, scratch)
            depth = _worst_table_depth(scratch, arm_geoms, table_gid)
            rows.append({
                "target": name,
                "target_pos": target.tolist(),
                "arm": arm,
                "residual_m": solution.position_error_m,
                "converged": bool(solution.position_error_m < ik.IK_POSITION_TOLERANCE_M),
                "table_depth_m": depth,
                "survives_naive_1mm": depth > -NAIVE_TABLE_TOL_M,
                "survives_loose_2cm": depth > -LOOSE_TABLE_TOL_M,
            })
    return rows


# ---------------------------------------------------------------------------
# Grid / occupancy / largest-rectangle helpers
# ---------------------------------------------------------------------------


def build_edges(lo: float, hi: float, res: float) -> np.ndarray:
    lo_r = np.floor(lo / res) * res - res
    hi_r = np.ceil(hi / res) * res + res
    n = int(round((hi_r - lo_r) / res)) + 1
    return np.round(lo_r + res * np.arange(n), 6)


def occupancy_grid(points: np.ndarray, keep: np.ndarray, x_edges: np.ndarray, y_edges: np.ndarray,
                    z_center: float, z_half: float) -> np.ndarray:
    """Boolean (len(y_edges)-1, len(x_edges)-1) grid: True where >=1 kept
    sample with |z - z_center| < z_half falls in that (x, y) cell.
    """
    mask = keep & (np.abs(points[:, 2] - z_center) < z_half)
    grid = np.zeros((len(y_edges) - 1, len(x_edges) - 1), dtype=bool)
    if not np.any(mask):
        return grid
    xs = points[mask, 0]
    ys = points[mask, 1]
    xi = np.clip(np.searchsorted(x_edges, xs, side="right") - 1, 0, len(x_edges) - 2)
    yi = np.clip(np.searchsorted(y_edges, ys, side="right") - 1, 0, len(y_edges) - 2)
    grid[yi, xi] = True
    return grid


def grid_to_ascii(grid: np.ndarray) -> str:
    return "\n".join("".join("#" if v else "." for v in row) for row in grid[::-1])  # +y at top


def largest_rectangle(grid: np.ndarray) -> tuple[int, int, int]:
    """Largest axis-aligned all-True rectangle in a boolean 2D grid.
    Returns (area_cells, height_cells, width_cells). Standard histogram DP,
    O(rows*cols).
    """
    if grid.size == 0 or not grid.any():
        return (0, 0, 0)
    rows, cols = grid.shape
    heights = np.zeros(cols, dtype=np.int64)
    best_area, best_h, best_w = 0, 0, 0
    for r in range(rows):
        heights = np.where(grid[r], heights + 1, 0)
        stack = []  # (index, height)
        for c in range(cols + 1):
            h = heights[c] if c < cols else 0
            start = c
            while stack and stack[-1][1] >= h:
                idx, sh = stack.pop()
                width = c - idx
                area = sh * width
                if area > best_area:
                    best_area, best_h, best_w = area, sh, width
                start = idx
            stack.append((start, h))
    return (int(best_area), int(best_h), int(best_w))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n", type=int, default=50000, help="samples per arm")
    parser.add_argument("--calib-n", type=int, default=500, help="calibration batch size")
    parser.add_argument("--calib-only", action="store_true", help="run calibration and exit")
    parser.add_argument("--time-cap-s", type=float, default=1200.0,
                         help="abort the full run if projected total exceeds this many seconds")
    parser.add_argument(
        "--densify-n", type=int, default=0,
        help=(
            "OPTIONAL supplementary per-arm sample count, used ONLY to sanity-check "
            "whether the primary N=--n intersection/contiguity answer is a Monte Carlo "
            "sampling-density artifact (see report's 'Densification check' section). "
            "0 (default) skips this entirely. Does not change the primary --n reported "
            "bounds/grids/rejection numbers at all -- purely an additional, clearly "
            "labeled cross-check appended to the same report."
        ),
    )
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    env = TableSettingEnv(cameras=None)
    env.reset(seed=RESET_SEED)
    model = env.model
    home_qpos = np.array(env.data.qpos, dtype=np.float64, copy=True)
    home_data = env.data

    print("=" * 70)
    print("CALIBRATION")
    print("=" * 70)
    ms_per_sample = {}
    for arm in ARMS:
        ms = _calibrate_impl(model, home_qpos, arm, args.calib_n)
        ms_per_sample[arm] = ms
        print(f"arm {arm}: {ms:.4f} ms/sample (n={args.calib_n})")

    worst_ms = max(ms_per_sample.values())
    projected_s = worst_ms * args.n * len(ARMS) / 1000.0
    print(f"\nProjected total for N={args.n} x {len(ARMS)} arms at worst-observed "
          f"{worst_ms:.4f} ms/sample: {projected_s:.1f} s ({projected_s / 60.0:.1f} min)")

    calib_record = {
        "ms_per_sample": ms_per_sample,
        "n_requested": args.n,
        "projected_total_s": projected_s,
        "time_cap_s": args.time_cap_s,
    }
    (OUT_DIR / "calibration.json").write_text(json.dumps(calib_record, indent=2), newline="\n", encoding="utf-8")

    if args.calib_only:
        env.close()
        return 0

    if projected_s > args.time_cap_s:
        print(f"\nABORTING: projected {projected_s:.1f}s exceeds --time-cap-s={args.time_cap_s}. "
              "Re-run with a smaller --n rather than letting this silently truncate.")
        env.close()
        return 1

    print("\n" + "=" * 70)
    print("SANITY CHECK: four known-good targets vs. the table_top filter")
    print("=" * 70)
    sanity_rows = sanity_check_targets(model, home_data)
    for r in sanity_rows:
        print(f"  {r['target']:16s} arm {r['arm']}: residual={r['residual_m']:.5f} "
              f"converged={r['converged']} table_depth={r['table_depth_m']:.5f} "
              f"survives_1mm={r['survives_naive_1mm']} survives_2cm={r['survives_loose_2cm']}")
    (OUT_DIR / "sanity_check.json").write_text(json.dumps(sanity_rows, indent=2), newline="\n", encoding="utf-8")

    print("\n" + "=" * 70)
    print(f"FULL SAMPLING: N={args.n} per arm")
    print("=" * 70)
    results = {}
    for arm in ARMS:
        pts, depths, joint_ranges, elapsed = sample_arm(model, home_qpos, arm, args.n, SAMPLE_SEEDS[arm])
        print(f"arm {arm}: done in {elapsed:.1f}s ({1000.0 * elapsed / args.n:.4f} ms/sample actual)")
        results[arm] = {"points": pts, "depths": depths, "joint_ranges": joint_ranges, "elapsed_s": elapsed}
        # Incremental save -- a mid-run failure on the SECOND arm must not
        # lose the first arm's already-completed 50000-sample cloud.
        np.savez(
            OUT_DIR / f"arm_{arm}_raw.npz",
            points=pts, depths=depths, joint_ranges=joint_ranges,
            seed=SAMPLE_SEEDS[arm], n=args.n,
        )
        print(f"  saved -> {OUT_DIR / f'arm_{arm}_raw.npz'}")

    env.close()

    # -----------------------------------------------------------------
    # Post-processing: bounds, rejection counts, grids, intersection.
    # -----------------------------------------------------------------
    summary = {"calibration": calib_record, "sanity_check": sanity_rows, "arms": {}}

    keep_masks = {}
    for arm in ARMS:
        pts = results[arm]["points"]
        depths = results[arm]["depths"]
        keep_1mm = depths > -NAIVE_TABLE_TOL_M
        keep_2cm = depths > -LOOSE_TABLE_TOL_M
        keep_masks[arm] = {"raw": np.ones(len(pts), dtype=bool), "1mm": keep_1mm, "2cm": keep_2cm}

        def bounds(mask):
            if not np.any(mask):
                return None
            p = pts[mask]
            return {
                "x": [float(p[:, 0].min()), float(p[:, 0].max())],
                "y": [float(p[:, 1].min()), float(p[:, 1].max())],
                "z": [float(p[:, 2].min()), float(p[:, 2].max())],
                "n": int(mask.sum()),
            }

        summary["arms"][arm] = {
            "n_total": len(pts),
            "n_rejected_1mm": int((~keep_1mm).sum()),
            "n_rejected_2cm": int((~keep_2cm).sum()),
            "bounds_raw": bounds(keep_masks[arm]["raw"]),
            "bounds_1mm": bounds(keep_masks[arm]["1mm"]),
            "bounds_2cm": bounds(keep_masks[arm]["2cm"]),
        }

    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2), newline="\n", encoding="utf-8")
    print("\nSummary written to", OUT_DIR / "summary.json")
    print(json.dumps(summary, indent=2))

    # -----------------------------------------------------------------
    # Grids: union bounding box across both arms' RAW clouds (1 cm res),
    # z-slices every 2 cm from Z_SLICE_LO to Z_SLICE_HI (same construction
    # `probe_reachability.py` uses for its own z grid).
    # -----------------------------------------------------------------
    all_pts = np.concatenate([results[a]["points"] for a in ARMS], axis=0)
    x_edges = build_edges(float(all_pts[:, 0].min()), float(all_pts[:, 0].max()), XY_RES_M)
    y_edges = build_edges(float(all_pts[:, 1].min()), float(all_pts[:, 1].max()), XY_RES_M)
    z_centers = np.round(np.arange(Z_SLICE_LO, Z_SLICE_HI + 1e-9, Z_SLICE_STEP), 2)
    z_half = Z_SLICE_STEP / 2.0

    report_lines: list[str] = []
    report_lines.append("# Redesign Stage 1 — empirical workspace measurement")
    report_lines.append("")
    report_lines.append(
        "Provenance: bm-ptl (ADR-047 cross-machine float divergence — all numbers below "
        "are measured on bm-ptl, not the developer's laptop, which cannot import MuJoCo "
        "at all under ADR-020)."
    )
    report_lines.append("")
    report_lines.append(
        f"Script: `scripts/measure_workspace.py`. Per-arm N={args.n}, sampled uniformly "
        "over each of the 5 IK-controlled joints' `model.jnt_range` (ADR-016: the 6th "
        "actuator, `armX_gripper`, drives only the jaw and is excluded — see "
        "`bimanual.control.ik.arm_joint_names`). Recorded point: the ADR-025 pinch point "
        "(midpoint of `armX_gripper` and `armX_moving_jaw_so101_v1` body origins), not the "
        "`armX_gripperframe` site."
    )
    report_lines.append("")

    report_lines.append("## Calibration")
    report_lines.append("")
    for arm in ARMS:
        report_lines.append(f"- arm {arm}: {ms_per_sample[arm]:.4f} ms/sample (n={args.calib_n} calibration batch)")
    report_lines.append(
        f"- Projected total for N={args.n} x {len(ARMS)} arms at the worst observed rate "
        f"({worst_ms:.4f} ms/sample): {projected_s:.1f} s ({projected_s / 60.0:.1f} min). "
        f"Time cap for aborting the full run: {args.time_cap_s:.0f} s."
    )
    for arm in ARMS:
        report_lines.append(
            f"- arm {arm} ACTUAL full-run rate: {1000.0 * results[arm]['elapsed_s'] / args.n:.4f} ms/sample, "
            f"{results[arm]['elapsed_s']:.1f} s total for N={args.n}."
        )
    report_lines.append("")

    report_lines.append("## Joint ranges actually used (read from `model.jnt_range`, not assumed)")
    report_lines.append("")
    report_lines.append("| arm | joint | range_lo (rad) | range_hi (rad) |")
    report_lines.append("|---|---|---:|---:|")
    for arm in ARMS:
        jr = results[arm]["joint_ranges"]
        for name, (lo, hi) in zip(ik.ARM_JOINT_SUFFIXES, jr):
            report_lines.append(f"| {arm} | {name} | {lo:.6f} | {hi:.6f} |")
    report_lines.append("")

    report_lines.append("## Caveat on the penetration filter — measured, not assumed")
    report_lines.append("")
    report_lines.append(
        "ADR-035's own warm-start diagnostic (Part 3) found a `table_top`-vs-arm-mesh "
        "contact reporting **-0.22211 m** penetration depth on a converged, physically "
        "sensible pose — a convex-hull mesh artifact, not real interpenetration. This "
        "script therefore records the raw worst-contact-depth per sample and reports "
        "rejection at two bars: the naive 1 mm bar "
        f"(`NAIVE_TABLE_TOL_M={NAIVE_TABLE_TOL_M}`, matching "
        "`skills_scripted.TABLE_COLLISION_DEPTH_TOL_M`) and a loose 2 cm bar "
        f"(`LOOSE_TABLE_TOL_M={LOOSE_TABLE_TOL_M}`)."
    )
    report_lines.append("")
    report_lines.append("### Sanity check: four independently-known-good targets vs. the filter")
    report_lines.append("")
    report_lines.append(
        "Targets: `fork_at_rest`, `mug_at_rest`, `bottle_at_rest` (ADR-025's own "
        "primary-probe targets — plate/mug/bottle at rest all PASSED that probe for both "
        "arms) and `handoff_point` (the literal current `HANDOFF_POSITION_XYZ` constant, "
        "checked for both arms without the per-arm `HANDOFF_SIDE_OFFSET_M` adjustment — a "
        "simplification, noted here, not the exact receiving-side point either arm's "
        "skill actually targets)."
    )
    report_lines.append("")
    report_lines.append("| target | arm | IK residual (m) | converged | table depth (m) | survives 1mm | survives 2cm |")
    report_lines.append("|---|---|---:|---|---:|---|---|")
    for r in sanity_rows:
        report_lines.append(
            f"| {r['target']} | {r['arm']} | {r['residual_m']:.5f} | {r['converged']} | "
            f"{r['table_depth_m']:.5f} | {r['survives_naive_1mm']} | {r['survives_loose_2cm']} |"
        )
    report_lines.append("")
    converged_rows = [r for r in sanity_rows if r["converged"]]
    n_converged = len(converged_rows)
    n_survive_1mm = sum(1 for r in converged_rows if r["survives_naive_1mm"])
    report_lines.append(
        f"**Of {n_converged} converged (target, arm) sanity pairs, {n_survive_1mm} survive the naive "
        "1 mm filter.** See the hand-written interpretation below for which cloud this report "
        "treats as authoritative."
    )
    report_lines.append("")

    report_lines.append("## Per-arm reachable bounds")
    report_lines.append("")
    report_lines.append("| arm | cloud | n kept | x range (m) | y range (m) | z range (m) |")
    report_lines.append("|---|---|---:|---|---|---|")
    for arm in ARMS:
        a = summary["arms"][arm]
        for cloud_key, label in (("bounds_raw", "raw"), ("bounds_1mm", "1mm-filtered"), ("bounds_2cm", "2cm-filtered")):
            b = a[cloud_key]
            if b is None:
                report_lines.append(f"| {arm} | {label} | 0 | -- | -- | -- |")
            else:
                report_lines.append(
                    f"| {arm} | {label} | {b['n']} | "
                    f"[{b['x'][0]:.3f}, {b['x'][1]:.3f}] | [{b['y'][0]:.3f}, {b['y'][1]:.3f}] | "
                    f"[{b['z'][0]:.3f}, {b['z'][1]:.3f}] |"
                )
    report_lines.append("")
    for arm in ARMS:
        a = summary["arms"][arm]
        pct_1mm = 100.0 * a["n_rejected_1mm"] / a["n_total"]
        pct_2cm = 100.0 * a["n_rejected_2cm"] / a["n_total"]
        report_lines.append(
            f"- arm {arm}: {a['n_rejected_1mm']}/{a['n_total']} ({pct_1mm:.1f}%) rejected at the 1mm bar; "
            f"{a['n_rejected_2cm']}/{a['n_total']} ({pct_2cm:.1f}%) rejected at the 2cm bar."
        )
    report_lines.append("")

    # -----------------------------------------------------------------
    # Per z-slice occupancy: RAW cloud gets full ASCII grids (primary,
    # collision-filter-independent kinematic view). The 1mm-filtered cloud
    # gets a compact per-slice cell-count table only (not full grids), to
    # keep this report a reasonable size -- see the module docstring /
    # report caveat section for why RAW is treated as the primary view.
    # -----------------------------------------------------------------
    report_lines.append("## Per-arm occupancy, z-slices every 2 cm (RAW cloud, 1 cm x/y grid)")
    report_lines.append("")
    report_lines.append(
        f"Grid: x in [{x_edges[0]:.2f}, {x_edges[-1]:.2f}], y in [{y_edges[0]:.2f}, {y_edges[-1]:.2f}], "
        "1 cm cells, `#`=reachable (>=1 sample), `.`=not sampled reachable at this slice. "
        "+y is the TOP row of each grid, +x is to the right."
    )
    report_lines.append("")

    grids_raw = {arm: {} for arm in ARMS}
    for arm in ARMS:
        pts = results[arm]["points"]
        keep = keep_masks[arm]["raw"]
        for z in z_centers:
            grids_raw[arm][z] = occupancy_grid(pts, keep, x_edges, y_edges, z, z_half)

    for z in z_centers:
        report_lines.append(f"### z = {z:.2f} m")
        report_lines.append("")
        for arm in ARMS:
            n_cells = int(grids_raw[arm][z].sum())
            report_lines.append(f"**Arm {arm}** ({n_cells} occupied cells)")
            report_lines.append("```")
            report_lines.append(grid_to_ascii(grids_raw[arm][z]) or "(empty)")
            report_lines.append("```")
        report_lines.append("")

    # -----------------------------------------------------------------
    # Intersection: both RAW and 1mm-filtered, per z-slice.
    # -----------------------------------------------------------------
    report_lines.append("## Intersection (handoff-feasible region)")
    report_lines.append("")
    report_lines.append(
        "Per-slice occupancy AND'd cell-by-cell between arm A's and arm B's grids at that "
        "slice (same 1 cm grid as above). Largest contiguous axis-aligned rectangle "
        "computed per slice (standard largest-rectangle-in-binary-matrix DP)."
    )
    report_lines.append("")

    intersection_summary = {"raw": [], "1mm": []}
    for cloud_key, cloud_label in (("raw", "raw"), ("1mm", "1mm-filtered")):
        report_lines.append(f"### {cloud_label} cloud")
        report_lines.append("")
        report_lines.append("| z (m) | A cells | B cells | intersection cells | largest rect (cells h x w) | largest rect (cm h x w) | >= 10x10 cm? |")
        report_lines.append("|---:|---:|---:|---:|---|---|---|")
        best_overall = {"z": None, "area_cm2": -1, "h_cm": 0, "w_cm": 0}
        for z in z_centers:
            grid_a = occupancy_grid(results["A"]["points"], keep_masks["A"][cloud_key], x_edges, y_edges, z, z_half)
            grid_b = occupancy_grid(results["B"]["points"], keep_masks["B"][cloud_key], x_edges, y_edges, z, z_half)
            inter = grid_a & grid_b
            area_cells, h_cells, w_cells = largest_rectangle(inter)
            h_cm, w_cm = h_cells * XY_RES_M * 100.0, w_cells * XY_RES_M * 100.0
            meets_10x10 = h_cm >= 10.0 and w_cm >= 10.0
            report_lines.append(
                f"| {z:.2f} | {int(grid_a.sum())} | {int(grid_b.sum())} | {int(inter.sum())} | "
                f"{h_cells}x{w_cells} | {h_cm:.0f}x{w_cm:.0f} | {meets_10x10} |"
            )
            area_cm2 = h_cm * w_cm
            if area_cm2 > best_overall["area_cm2"]:
                best_overall = {"z": float(z), "area_cm2": area_cm2, "h_cm": h_cm, "w_cm": w_cm}
            intersection_summary[cloud_key].append({
                "z": float(z), "a_cells": int(grid_a.sum()), "b_cells": int(grid_b.sum()),
                "intersection_cells": int(inter.sum()), "rect_h_cm": h_cm, "rect_w_cm": w_cm,
            })
        report_lines.append("")
        if best_overall["z"] is not None and best_overall["area_cm2"] > 0:
            report_lines.append(
                f"**Largest intersection rectangle ({cloud_label}): {best_overall['h_cm']:.0f} cm x "
                f"{best_overall['w_cm']:.0f} cm at z={best_overall['z']:.2f} m "
                f"({'meets' if (best_overall['h_cm'] >= 10.0 and best_overall['w_cm'] >= 10.0) else 'does NOT meet'} "
                "the 10x10 cm bar).**"
            )
        else:
            report_lines.append(f"**No intersection cells found at any slice ({cloud_label}).**")
        report_lines.append("")

    (OUT_DIR / "intersection_summary.json").write_text(json.dumps(intersection_summary, indent=2), newline="\n", encoding="utf-8")

    report_lines.append("## Centroid / extent of the largest intersection region (raw cloud)")
    report_lines.append("")
    # Overall 3D centroid of every cell that is ever both-reachable across
    # any slice (raw cloud), for a single headline centroid/extent figure.
    any_inter_points = []
    for z in z_centers:
        grid_a = grids_raw["A"][z]
        grid_b = grids_raw["B"][z]
        inter = grid_a & grid_b
        ys_idx, xs_idx = np.nonzero(inter)
        for yi, xi in zip(ys_idx, xs_idx):
            cx = (x_edges[xi] + x_edges[xi + 1]) / 2.0
            cy = (y_edges[yi] + y_edges[yi + 1]) / 2.0
            any_inter_points.append((cx, cy, z))
    if any_inter_points:
        arr = np.array(any_inter_points)
        report_lines.append(
            f"- n intersecting cells (any slice): {len(arr)}\n"
            f"- centroid: x={arr[:,0].mean():.3f}, y={arr[:,1].mean():.3f}, z={arr[:,2].mean():.3f}\n"
            f"- extent: x in [{arr[:,0].min():.3f}, {arr[:,0].max():.3f}], "
            f"y in [{arr[:,1].min():.3f}, {arr[:,1].max():.3f}], "
            f"z in [{arr[:,2].min():.3f}, {arr[:,2].max():.3f}]"
        )
    else:
        report_lines.append("- No intersecting cells found at any slice (raw cloud).")
    report_lines.append("")

    # -----------------------------------------------------------------
    # Densification check (opt-in, --densify-n > 0): is the N=--n
    # contiguity/largest-rectangle answer above a real property of the
    # workspace, or a Monte Carlo sampling-density artifact? With ~50000
    # samples spread across 13 z-slices and a footprint of order 1000+
    # occupied 1cm cells per slice, the average density is only a few
    # samples per occupied cell -- comfortably enough to fix bounds (a
    # min/max is density-INsensitive) but not obviously enough to trust a
    # single-cell gap as a genuine hole rather than sampling noise. This
    # section re-samples at a much higher N (same method, same filter,
    # fresh RNG seeds) and recomputes ONLY the intersection largest-rectangle
    # table, to see whether the answer is stable.
    # -----------------------------------------------------------------
    if args.densify_n > 0:
        print("\n" + "=" * 70)
        print(f"DENSIFICATION CHECK: N={args.densify_n} per arm (contiguity cross-check only)")
        print("=" * 70)
        dense_results = {}
        for arm in ARMS:
            pts_d, depths_d, _, elapsed_d = sample_arm(
                model, home_qpos, arm, args.densify_n, SAMPLE_SEEDS[arm] + 500_000, report_every=100_000
            )
            print(f"arm {arm} (dense): done in {elapsed_d:.1f}s "
                  f"({1000.0 * elapsed_d / args.densify_n:.4f} ms/sample)")
            dense_results[arm] = {
                "points": pts_d,
                "keep_raw": np.ones(len(pts_d), dtype=bool),
                "keep_1mm": depths_d > -NAIVE_TABLE_TOL_M,
            }

        report_lines.append("## Densification check (sampling-density sanity cross-check)")
        report_lines.append("")
        report_lines.append(
            f"Same method, same grid, same filter, fresh RNG seeds, N={args.densify_n} per arm "
            f"({args.densify_n / args.n:.0f}x the primary run) -- purely to check whether the "
            "primary run's intersection/contiguity numbers above are a Monte Carlo "
            "sampling-density artifact rather than a real property of the workspace. This "
            "does NOT replace the primary N={} numbers reported above.".format(args.n)
        )
        report_lines.append("")
        for cloud_key, cloud_label in (("raw", "raw"), ("1mm", "1mm-filtered")):
            report_lines.append(f"### {cloud_label} cloud, densified")
            report_lines.append("")
            report_lines.append("| z (m) | A cells | B cells | intersection cells | largest rect (h x w cm) | >= 10x10 cm? |")
            report_lines.append("|---:|---:|---:|---:|---|---|")
            best_d = {"z": None, "area": -1, "h": 0, "w": 0}
            mask_key = "keep_raw" if cloud_key == "raw" else "keep_1mm"
            for z in z_centers:
                grid_a = occupancy_grid(dense_results["A"]["points"], dense_results["A"][mask_key], x_edges, y_edges, z, z_half)
                grid_b = occupancy_grid(dense_results["B"]["points"], dense_results["B"][mask_key], x_edges, y_edges, z, z_half)
                inter = grid_a & grid_b
                area_cells, h_cells, w_cells = largest_rectangle(inter)
                h_cm, w_cm = h_cells * XY_RES_M * 100.0, w_cells * XY_RES_M * 100.0
                meets = h_cm >= 10.0 and w_cm >= 10.0
                report_lines.append(
                    f"| {z:.2f} | {int(grid_a.sum())} | {int(grid_b.sum())} | {int(inter.sum())} | "
                    f"{h_cm:.0f}x{w_cm:.0f} | {meets} |"
                )
                if h_cm * w_cm > best_d["area"]:
                    best_d = {"z": float(z), "area": h_cm * w_cm, "h": h_cm, "w": w_cm}
            report_lines.append("")
            if best_d["z"] is not None and best_d["area"] > 0:
                report_lines.append(
                    f"**Largest densified intersection rectangle ({cloud_label}): {best_d['h']:.0f} cm x "
                    f"{best_d['w']:.0f} cm at z={best_d['z']:.2f} m.**"
                )
            else:
                report_lines.append(f"**No intersection cells found at any slice ({cloud_label}, densified).**")
            report_lines.append("")

    report_lines.append("## The explicit question")
    report_lines.append("")
    report_lines.append(
        "**Placeholder — filled in by hand after reading the tables above; see the "
        "'Interpretation' section appended below this line, added in a follow-up edit "
        "once this script's raw output has been read.**"
    )
    report_lines.append("")

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(report_lines) + "\n", newline="\n", encoding="utf-8")
    print(f"\nReport written to {REPORT_PATH}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
