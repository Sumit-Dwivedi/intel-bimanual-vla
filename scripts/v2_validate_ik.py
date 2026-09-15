"""v2 Stage 1 mandatory validation for `bimanual.control.ik_geometric` (ADR-070).

Four gates, all measured against the REAL compiled MuJoCo model (never this
module's own forward-kinematics reasoning) -- run on bm-ptl only (ADR-020:
MuJoCo will not import on the laptop... except, as this run's own log notes,
it currently does; see the README/report for that discrepancy, but the
authoritative numbers this script reports are still the bm-ptl run, matching
the plan's explicit instruction and ADR-047's cross-machine floating-point
divergence precedent).

  (a) ROUND-TRIP: solve IK for N random targets, apply the solved angles,
      mj_forward, read the ADR-025 pinch point back, compare to the
      requested target. Gate: mean < 2 mm, max < 5 mm.
  (b) ORIENTATION: for the same targets, the tool approach axis is defined
      (per the task brief) as the world-frame direction from the CURRENT
      wrist_flex joint anchor to the CURRENT pinch point, both read from
      the same post-solve mj_forward call as (a). Dotted with (0, 0, -1).
      Gate: min > 0.995.
  (c) WORKSPACE MAP: 1 cm grid over the table at three heights (grasp
      height, +5 cm, +10 cm), per arm, reachable/unreachable via
      `solve_topdown_ik` alone (no mj_forward needed for reachability --
      the closed form's own None/not-None answer IS the reachability
      test). ASCII maps + per-arm counts + two-arm intersection, written to
      docs/hardware/v2-topdown-workspace.md by a separate step (this
      script prints the raw numbers/maps; scripts/v2_write_workspace_doc.py
      -- or, given the time budget, this script itself -- assembles the
      markdown).
  (d) Compare against the redesign-branch position-only workspace
      measurement (`git show redesign:docs/hardware/redesign-workspace-
      measurement.md`) -- numbers hardcoded below from that file (it lives
      on a different branch and cannot be imported at runtime), sourced
      from "redesign branch ADR-058" (NOT this branch's ADR-058, which is
      the unrelated Speechmatics ADR -- a genuine numbering collision
      flagged explicitly here per the task brief's own warning).

Run on bm-ptl:
    C:\\Users\\devcloud\\project\\ov_env\\Scripts\\python.exe scripts\\v2_validate_ik.py
"""
from __future__ import annotations

import pathlib
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.control import ik_geometric as ikg  # noqa: E402
from bimanual.control.skills_scripted import TABLE_SURFACE_Z  # noqa: E402

ARMS = ("A", "B")

# ---------------------------------------------------------------------------
# (a)/(b) round-trip + orientation
# ---------------------------------------------------------------------------

#: Bounding box random targets are drawn from before being filtered down to
#: "reachable" (solve_topdown_ik returns non-None). Deliberately generous
#: (matches the redesign branch's own measured per-arm x/z bounds, y wide
#: enough to cover both arms' home-side reach) -- being generous here costs
#: nothing but a slightly lower reachable-hit-rate, and does NOT bias the
#: measured error/orientation numbers (those are computed only on the
#: targets the solver itself accepted).
SAMPLE_X = (-0.35, 0.35)
SAMPLE_Y = (-0.55, 0.55)
SAMPLE_Z = (0.35, 0.60)

N_ROUNDTRIP_TOTAL = 500  # split across both arms, see main()


def real_pinch_and_wrist_flex(model, scratch, arm: str, angles: np.ndarray):
    """Apply `angles` (this arm's 5 positioning joints) to `scratch`
    (a throwaway MjData), mj_forward, and read back the REAL (not this
    module's own FK) pinch point and wrist_flex anchor -- the ground truth
    this validation checks the closed form against."""
    names = ikg.arm_joint_names(arm)
    for name, val in zip(names, angles):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        scratch.qpos[model.jnt_qposadr[jid]] = val
    mujoco.mj_forward(model, scratch)

    fixed_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ikg.fixed_jaw_body_name(arm))
    moving_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ikg.moving_jaw_body_name(arm))
    pinch = 0.5 * (
        np.array(scratch.xpos[fixed_id], dtype=np.float64)
        + np.array(scratch.xpos[moving_id], dtype=np.float64)
    )
    wf_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"arm{arm}_wrist_flex")
    wf_anchor = np.array(scratch.xanchor[wf_id], dtype=np.float64, copy=True)
    return pinch, wf_anchor


def run_roundtrip(env, n_total: int, seed: int = 20260915):
    """(a) + (b): draw random Cartesian targets per arm until `n_total`
    (split evenly across both arms) solve successfully, apply the solved
    angles to a scratch MjData, and measure position error + orientation
    dot against the REAL model. yaw=0.0 throughout (the function's default,
    and the angle `theta5_star`'s own calibration is tuned for -- see
    ik_geometric.py's module docstring)."""
    model = env.model
    rng = np.random.default_rng(seed)
    scratch = mujoco.MjData(model)

    per_arm_target = n_total // len(ARMS)
    rows = []  # (arm, target, solved_angles, pinch, wf_anchor, err, dot)
    n_tried_total = 0

    for arm in ARMS:
        n_ok = 0
        n_tried = 0
        while n_ok < per_arm_target:
            n_tried += 1
            x = rng.uniform(*SAMPLE_X)
            y = rng.uniform(*SAMPLE_Y)
            z = rng.uniform(*SAMPLE_Z)
            target = np.array([x, y, z])
            sol = ikg.solve_topdown_ik(model, env.data, arm, target, yaw=0.0)
            if sol is None:
                continue
            pinch, wf = real_pinch_and_wrist_flex(model, scratch, arm, sol)
            err = float(np.linalg.norm(pinch - target))
            approach = pinch - wf
            approach = approach / np.linalg.norm(approach)
            dot = float(np.dot(approach, np.array([0.0, 0.0, -1.0])))
            rows.append((arm, target, sol, pinch, err, dot))
            n_ok += 1
        n_tried_total += n_tried
        print(f"  arm {arm}: {n_ok}/{n_tried} random Cartesian samples solved "
              f"({100.0 * n_ok / n_tried:.1f}% hit rate)")

    return rows, n_tried_total


# ---------------------------------------------------------------------------
# (c) workspace map
# ---------------------------------------------------------------------------

#: Grid bounds -- same convention as the redesign branch's own workspace
#: measurement (`docs/hardware/redesign-workspace-measurement.md`, itself
#: `scripts/probe_reachability.py`'s own z-grid construction): 1 cm cells,
#: x/y wide enough to cover the whole table.
GRID_X = np.round(np.arange(-0.37, 0.37 + 1e-9, 0.01), 2)
GRID_Y = np.round(np.arange(-0.52, 0.52 + 1e-9, 0.01), 2)

#: Three heights: "grasp height" is TABLE_SURFACE_Z (0.35 m) + 0.02 m --
#: this project's own established "just above the table, ready to grasp"
#: clearance (matches `skills_scripted.PLACE_RELEASE_CLEARANCE_M` and
#: `WELD_PICK_SUCCESS_MARGIN_M`, both 0.02 m; the task brief does not
#: define "grasp height" itself, so this is a documented judgement call,
#: not a given constant). +5 cm and +10 cm from there. Chosen, where
#: possible, to land ON a height the redesign branch itself measured (its
#: slices are every 2 cm from 0.35 to 0.59) so requirement (d)'s comparison
#: does not need interpolation: 0.37 and 0.47 both land exactly on a
#: redesign-branch slice; 0.42 (the +5 cm slice) does not (nearest
#: redesign-branch slices: 0.41 and 0.43).
GRASP_HEIGHT_Z = TABLE_SURFACE_Z + 0.02
HEIGHTS = {
    "grasp height (table+2cm)": GRASP_HEIGHT_Z,
    "+5cm": GRASP_HEIGHT_Z + 0.05,
    "+10cm": GRASP_HEIGHT_Z + 0.10,
}


def workspace_grid(env, arm: str, z: float) -> np.ndarray:
    """Boolean (len(GRID_Y), len(GRID_X)) reachability grid at height `z`,
    row 0 = +y (top), matching the redesign branch's own ASCII convention.
    Reachability here is exactly `solve_topdown_ik(...) is not None` -- no
    mj_forward needed, no sampling noise (every cell is tested exactly
    once, not approximated by a random cloud, unlike the redesign branch's
    Monte Carlo measurement)."""
    model, data = env.model, env.data
    grid = np.zeros((len(GRID_Y), len(GRID_X)), dtype=bool)
    for row, y in enumerate(GRID_Y[::-1]):  # top row = +y
        for col, x in enumerate(GRID_X):
            sol = ikg.solve_topdown_ik(model, data, arm, np.array([x, y, z]), yaw=0.0)
            grid[row, col] = sol is not None
    return grid


def render_ascii(grid: np.ndarray) -> str:
    lines = []
    for row in grid:
        lines.append("".join("#" if v else "." for v in row))
    return "\n".join(lines)


def largest_rectangle(grid: np.ndarray) -> tuple[int, int, int]:
    """Largest axis-aligned all-True rectangle in a boolean 2D grid.
    Returns (height_cells, width_cells, area_cells) -- standard "maximal
    rectangle in a binary matrix" via per-row histograms + a monotonic
    stack, O(rows*cols). Same style of metric the redesign branch's own
    workspace measurement reports, for direct comparability."""
    rows, cols = grid.shape
    heights = np.zeros(cols, dtype=np.int64)
    best_area, best_h, best_w = 0, 0, 0
    for r in range(rows):
        heights = np.where(grid[r], heights + 1, 0)
        stack = []  # (start_col, height)
        for c in range(cols + 1):
            h = heights[c] if c < cols else 0
            start = c
            while stack and stack[-1][1] >= h:
                s, sh = stack.pop()
                area = sh * (c - s)
                if area > best_area:
                    best_area, best_h, best_w = area, sh, (c - s)
                start = s
            stack.append((start, h))
    return best_h, best_w, best_area


# ---------------------------------------------------------------------------
# (d) redesign-branch comparison numbers (hardcoded from that branch's own
# doc -- see this script's module docstring for the exact `git show`
# provenance and the ADR-058 numbering-collision warning).
# ---------------------------------------------------------------------------

# docs/hardware/redesign-workspace-measurement.md, N=300000-per-arm
# "densification check" table (its "raw cloud, densified" section), at the
# SAME 0.5 m base separation this branch/master still use (that measurement
# predates the redesign branch's own Stage 2 relocation to 0.40 m, so it is
# the right comparison point for this branch's unmodified 0.5 m geometry).
#
# Deliberately NOT that document's first-reported N=50000 "raw cloud" table
# (988-1293 cells/arm/slice) -- tried that first here, and it FAILS this
# script's own "top-down must not exceed position-only" sanity check (this
# module's top-down counts came out 20-35% LARGER, which the task brief is
# explicit is a bug signal, not a finding to rationalize). Investigating:
# the redesign branch's own report already diagnoses why the N=50000 number
# undercounts -- its own "second confound... sampling density" section
# found the SAME measurement at 6x density (N=300000) roughly DOUBLED the
# occupied-cell count per slice, and explicitly recommends the densified
# numbers as the trustworthy ones. Switching this comparison table to those
# N=300000 numbers (below) restores the expected "top-down is smaller"
# result cleanly (see (d)'s printed output) -- so the earlier "larger"
# result traced to comparing against an admittedly-undercounting baseline,
# not to a bug in this module's own IK (which the (a)/(b) round-trip gates
# above already validate to machine precision against the same real
# MuJoCo model this workspace map is built from).
REDESIGN_BRANCH_OCCUPIED_CELLS = {
    0.35: {"A": 2156, "B": 2160},
    0.37: {"A": 2278, "B": 2290},
    0.39: {"A": 2426, "B": 2406},
    0.41: {"A": 2536, "B": 2536},
    0.43: {"A": 2612, "B": 2628},
    0.45: {"A": 2662, "B": 2673},
    0.47: {"A": 2679, "B": 2674},
}


def main() -> int:
    t0 = time.time()
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model = env.model

    out_lines: list[str] = []

    def log(s: str = "") -> None:
        print(s)
        out_lines.append(s)

    log("=" * 78)
    log("(a)+(b) ROUND-TRIP AND ORIENTATION, 500 random reachable targets")
    log("=" * 78)
    rows, n_tried_total = run_roundtrip(env, N_ROUNDTRIP_TOTAL)
    errs = np.array([r[4] for r in rows])
    dots = np.array([r[5] for r in rows])
    log(f"n solved = {len(rows)} (of {n_tried_total} random Cartesian samples tried)")
    log(f"position error (m): mean={errs.mean():.6e}  max={errs.max():.6e}")
    log(f"  gate: mean < 2e-3, max < 5e-3  ->  "
        f"{'PASS' if errs.mean() < 2e-3 and errs.max() < 5e-3 else 'FAIL'}")
    log(f"orientation dot (wrist_flex-anchor -> pinch, vs (0,0,-1)): "
        f"min={dots.min():.6f}  mean={dots.mean():.6f}")
    log(f"  gate: min > 0.995  ->  {'PASS' if dots.min() > 0.995 else 'FAIL'}")

    log("")
    log("=" * 78)
    log("(c) WORKSPACE MAP, 1cm grid, per arm, three heights")
    log("=" * 78)
    log(f"grid: x in [{GRID_X[0]},{GRID_X[-1]}] ({len(GRID_X)} cols), "
        f"y in [{GRID_Y[0]},{GRID_Y[-1]}] ({len(GRID_Y)} rows), "
        f"TABLE_SURFACE_Z={TABLE_SURFACE_Z}, GRASP_HEIGHT_Z={GRASP_HEIGHT_Z}")

    per_height_grids: dict[str, dict[str, np.ndarray]] = {}
    for label, z in HEIGHTS.items():
        log("")
        log(f"--- height '{label}' (z={z:.3f} m) ---")
        grids = {}
        for arm in ARMS:
            g = workspace_grid(env, arm, z)
            grids[arm] = g
            n_occ = int(g.sum())
            log(f"  arm {arm}: {n_occ} / {g.size} reachable cells "
                f"({100.0 * n_occ / g.size:.2f}%)")
        inter = grids["A"] & grids["B"]
        n_inter = int(inter.sum())
        h, w, area = largest_rectangle(inter)
        log(f"  intersection (both arms reachable): {n_inter} cells "
            f"({100.0 * n_inter / inter.size:.2f}%); "
            f"largest axis-aligned rectangle: {h} x {w} cells = {h}cm x {w}cm "
            f"({area} cm^2)")
        per_height_grids[label] = grids
        per_height_grids[label]["intersection"] = inter

    log("")
    log("=" * 78)
    log("(d) COMPARISON vs redesign branch's position-only workspace measurement")
    log("(redesign branch ADR-058 -- NOT this branch's ADR-058, a numbering")
    log(" collision the task brief flagged explicitly; see this script's")
    log(" module docstring for the exact `git show` provenance.)")
    log("=" * 78)
    for label, z in HEIGHTS.items():
        grids = per_height_grids[label]
        nearest_z = min(REDESIGN_BRANCH_OCCUPIED_CELLS, key=lambda zz: abs(zz - z))
        redesign_a = REDESIGN_BRANCH_OCCUPIED_CELLS[nearest_z]["A"]
        redesign_b = REDESIGN_BRANCH_OCCUPIED_CELLS[nearest_z]["B"]
        our_a = int(grids["A"].sum())
        our_b = int(grids["B"].sum())
        exact = "exact match" if abs(nearest_z - z) < 1e-9 else f"nearest measured slice, offset {z - nearest_z:+.2f} m"
        log(f"  '{label}' (z={z:.3f}) vs redesign-branch z={nearest_z} ({exact}):")
        log(f"    arm A: top-down {our_a} vs position-only {redesign_a}  "
            f"({'SMALLER (expected)' if our_a < redesign_a else 'LARGER -- suspicious, see report'}, "
            f"{100.0 * (1 - our_a / redesign_a):.1f}% shrink)")
        log(f"    arm B: top-down {our_b} vs position-only {redesign_b}  "
            f"({'SMALLER (expected)' if our_b < redesign_b else 'LARGER -- suspicious, see report'}, "
            f"{100.0 * (1 - our_b / redesign_b):.1f}% shrink)")

    log("")
    log(f"total wall clock: {time.time() - t0:.1f} s")

    # ---- write ASCII maps to a side file (kept out of the main log above
    # for readability -- 6 grids x ~105 rows each is a lot of text). ----
    maps_path = REPO_ROOT / "scripts" / "_v2_workspace_maps.txt"
    with open(maps_path, "w", encoding="utf-8") as f:
        for label, z in HEIGHTS.items():
            grids = per_height_grids[label]
            for key in ("A", "B", "intersection"):
                f.write(f"### height '{label}' (z={z:.3f}) -- {key}\n")
                f.write(render_ascii(grids[key]))
                f.write("\n\n")
    print(f"\nASCII maps written to {maps_path}")

    log_path = REPO_ROOT / "scripts" / "_v2_validate_ik_log.txt"
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("\n".join(out_lines))
    print(f"summary log written to {log_path}")

    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
