"""Redesign Stage 2, Step 1: base-separation sweep (ADR-059).

**Measurement only, in this script's own scope.** It generates TEMPORARY
scene XML files (one per candidate separation), samples each arm's raw
reachable cloud at N=300000 (the density Stage 1's densification check
showed is needed to trust a largest-rectangle answer -- see
docs/hardware/redesign-workspace-measurement.md's "second confound" section
and ARCHITECTURE.md ADR-058), computes the both-arms intersection per
z-slice, and reports which separations yield a contiguous region >= 12x12 cm
above z=0.40. It does **not** overwrite the real
`src/bimanual/sim/assets/so101_dual_table.xml` and does not commit any
intermediate scene -- the one temp XML it writes
(`src/bimanual/sim/assets/_sweep_tmp.xml`) is deleted after every sample and
again at exit (see `finally` in `main()`), and is also listed in
`.gitignore`-equivalent hygiene by simply never being added.

**Why this sweep is the right instrument (task's own framing, verified by
tracing the code rather than assumed).** `scripts/measure_workspace.py:291`
builds `grid = np.zeros((len(y_edges)-1, len(x_edges)-1))` -- rows are Y,
columns are X -- and `largest_rectangle` returns `(area, h_cells, w_cells)`
in that same (row, col) = (y, x) order. Stage 1's headline "4 cm x 32 cm"
band is therefore 4 cm in Y, 32 cm in X: the SHORT axis is Y, which is
exactly the axis the two arm bases are separated along (ARM_GAP_Y, +/-0.25 m
in the current scene). Reducing base separation directly thickens the short
axis; it would NOT have helped had the short axis been X, which is why this
sweep was worth running at all.

**Reuses, does not duplicate, existing code.** Scene generation reuses
`scripts/gen_dual_scene.py` by importing it and monkeypatching its
`ARM_GAP_Y` (half the base-to-base separation) and `DEST` (write target)
globals before calling its own `main()` -- the identical arm-building,
friction, finger-pad, weld-constraint and home-keyframe logic Stage 1's
scene already uses, so the sampled scene differs from the shipped one ONLY
in base separation. Sampling reuses `scripts/measure_workspace.py`'s own
`sample_arm`, `build_edges`, `occupancy_grid` and `SAMPLE_SEEDS` -- the same
functions, same seeds, same per-sample cost profile Stage 1 already
calibrated and reported.

Run from the repo root on bm-ptl (ADR-020: imports `mujoco` via
`TableSettingEnv`):
    python scripts/sweep_base_separation.py
"""

from __future__ import annotations

import importlib
import json
import pathlib
import sys
import time

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import mujoco  # noqa: E402

from bimanual.sim.env import TableSettingEnv  # noqa: E402

import gen_dual_scene as gen  # noqa: E402
import measure_workspace as mw  # noqa: E402

ARMS = ("A", "B")

#: Full sweep list: the six separations the task names, PLUS the two 0.05 m
#: extension steps (0.20, 0.15) pre-included so this script needs only one
#: bm-ptl invocation rather than a conditional second round-trip. If a
#: qualifying separation is found among the first six, the extension rows
#: are still computed (cheap relative to the whole run) but are irrelevant
#: to the selection rule below and are reported for completeness only.
SEPARATIONS_M = [0.50, 0.45, 0.40, 0.35, 0.30, 0.25, 0.20, 0.15]

N_PER_ARM = 300_000
Z_LO, Z_HI, Z_STEP = 0.35, 0.59, 0.02
XY_RES_M = 0.01
MIN_SIDE_CM = 12.0
MIN_Z_FOR_SELECTION_M = 0.40

TMP_XML = REPO_ROOT / "src" / "bimanual" / "sim" / "assets" / "_sweep_tmp.xml"
OUT_DIR = REPO_ROOT / "out" / "sweep_base_separation"


def build_scene_for_separation(separation_m: float) -> pathlib.Path:
    """Monkeypatch gen_dual_scene's globals and regenerate a scene with this
    base-to-base separation, written to a scratch path (never the real
    DEST). `importlib.reload` is NOT used -- reassigning the two globals on
    the already-imported module object is sufficient because `main()` looks
    them up dynamically at call time, and reload would re-run the module's
    own top-level asserts/derivations for no benefit.
    """
    gen.ARM_GAP_Y = separation_m / 2.0
    gen.DEST = TMP_XML
    gen.main()
    return TMP_XML


def largest_rect_with_pos(grid: np.ndarray):
    """Same DP as measure_workspace.largest_rectangle, but also returns the
    winning rectangle's (row_start, col_start) so callers can recover a
    world-coordinate centroid, not just a size.
    Returns (area_cells, h_cells, w_cells, row_start, col_start).
    """
    if grid.size == 0 or not grid.any():
        return (0, 0, 0, 0, 0)
    rows, cols = grid.shape
    heights = np.zeros(cols, dtype=np.int64)
    best = (0, 0, 0, 0, 0)
    for r in range(rows):
        heights = np.where(grid[r], heights + 1, 0)
        stack = []  # (start_col, height)
        for c in range(cols + 1):
            h = int(heights[c]) if c < cols else 0
            start = c
            while stack and stack[-1][1] >= h:
                idx, sh = stack.pop()
                width = c - idx
                area = sh * width
                if area > best[0]:
                    r0 = r - sh + 1
                    best = (area, sh, width, r0, idx)
                start = idx
            stack.append((start, h))
    return best


def sample_one_separation(separation_m: float) -> dict:
    t_scene0 = time.perf_counter()
    scene_path = build_scene_for_separation(separation_m)
    try:
        env = TableSettingEnv(scene_path=scene_path, cameras=None)
        env.reset(seed=mw.RESET_SEED)
        model = env.model
        home_qpos = np.array(env.data.qpos, dtype=np.float64, copy=True)

        points = {}
        elapsed_per_arm = {}
        for arm in ARMS:
            pts, depths, joint_ranges, elapsed = mw.sample_arm(
                model, home_qpos, arm, N_PER_ARM, mw.SAMPLE_SEEDS[arm], report_every=0
            )
            points[arm] = pts
            elapsed_per_arm[arm] = elapsed
        env.close()
    finally:
        # Never leave the scratch scene behind, success or failure.
        if TMP_XML.exists():
            TMP_XML.unlink()

    all_pts = np.concatenate([points[a] for a in ARMS], axis=0)
    x_edges = mw.build_edges(float(all_pts[:, 0].min()), float(all_pts[:, 0].max()), XY_RES_M)
    y_edges = mw.build_edges(float(all_pts[:, 1].min()), float(all_pts[:, 1].max()), XY_RES_M)
    z_centers = np.round(np.arange(Z_LO, Z_HI + 1e-9, Z_STEP), 2)
    z_half = Z_STEP / 2.0
    keep_all = {a: np.ones(len(points[a]), dtype=bool) for a in ARMS}

    per_slice = []
    for z in z_centers:
        grid_a = mw.occupancy_grid(points["A"], keep_all["A"], x_edges, y_edges, z, z_half)
        grid_b = mw.occupancy_grid(points["B"], keep_all["B"], x_edges, y_edges, z, z_half)
        inter = grid_a & grid_b
        area, h, w, r0, c0 = largest_rect_with_pos(inter)
        h_cm, w_cm = h * XY_RES_M * 100.0, w * XY_RES_M * 100.0
        if area > 0:
            cy = float((y_edges[r0] + y_edges[r0 + h]) / 2.0)
            cx = float((x_edges[c0] + x_edges[c0 + w]) / 2.0)
        else:
            cy = cx = None
        per_slice.append({
            "z": float(z),
            "a_cells": int(grid_a.sum()),
            "b_cells": int(grid_b.sum()),
            "intersection_cells": int(inter.sum()),
            "rect_h_cm": h_cm,
            "rect_w_cm": w_cm,
            "centroid_x": cx,
            "centroid_y": cy,
            "meets_12x12_above_0p40": bool(
                z >= MIN_Z_FOR_SELECTION_M and h_cm >= MIN_SIDE_CM and w_cm >= MIN_SIDE_CM
            ),
        })

    total_elapsed = time.perf_counter() - t_scene0
    return {
        "separation_m": separation_m,
        "elapsed_s": total_elapsed,
        "elapsed_per_arm_s": elapsed_per_arm,
        "per_slice": per_slice,
    }


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    all_results = []
    for sep in SEPARATIONS_M:
        print("=" * 70)
        print(f"SEPARATION = {sep:.2f} m  (ARM_GAP_Y = {sep / 2.0:.3f} m)")
        print("=" * 70)
        result = sample_one_separation(sep)
        print(f"  total {result['elapsed_s']:.1f}s, per-arm: "
              f"{ {a: round(result['elapsed_per_arm_s'][a], 1) for a in ARMS} }")
        best_slice = max(result["per_slice"], key=lambda s: s["rect_h_cm"] * s["rect_w_cm"])
        print(f"  best overall slice: z={best_slice['z']:.2f} "
              f"{best_slice['rect_h_cm']:.0f}x{best_slice['rect_w_cm']:.0f} cm")
        qualifying = [s for s in result["per_slice"] if s["meets_12x12_above_0p40"]]
        print(f"  slices meeting >=12x12cm above z=0.40: {len(qualifying)}")
        for s in qualifying:
            print(f"    z={s['z']:.2f} {s['rect_h_cm']:.0f}x{s['rect_w_cm']:.0f} cm "
                  f"centroid=({s['centroid_x']:.3f}, {s['centroid_y']:.3f})")
        all_results.append(result)
        (OUT_DIR / "sweep_results.json").write_text(
            json.dumps(all_results, indent=2), newline="\n", encoding="utf-8"
        )

    # ---- Selection rule: LARGEST separation with >=1 qualifying slice ----
    print("\n" + "=" * 70)
    print("SELECTION")
    print("=" * 70)
    chosen = None
    for result in sorted(all_results, key=lambda r: -r["separation_m"]):
        qualifying = [s for s in result["per_slice"] if s["meets_12x12_above_0p40"]]
        if qualifying:
            best = max(qualifying, key=lambda s: s["rect_h_cm"] * s["rect_w_cm"])
            chosen = {"separation_m": result["separation_m"], "slice": best}
            break

    if chosen is None:
        print("NO separation in the sweep (down to 0.15 m) produced a contiguous "
              ">=12x12cm both-arms region above z=0.40. This is a hard statement "
              "about SO-101 geometry, per the task's own stop condition.")
        (OUT_DIR / "selection.json").write_text(
            json.dumps({"chosen": None, "reason": "no qualifying separation down to 0.15 m"}, indent=2),
            newline="\n", encoding="utf-8",
        )
        return 1

    print(f"CHOSEN separation: {chosen['separation_m']:.2f} m")
    print(f"  slice z={chosen['slice']['z']:.2f}, "
          f"{chosen['slice']['rect_h_cm']:.0f}x{chosen['slice']['rect_w_cm']:.0f} cm, "
          f"centroid=({chosen['slice']['centroid_x']:.4f}, {chosen['slice']['centroid_y']:.4f}, "
          f"{chosen['slice']['z']:.4f})")
    (OUT_DIR / "selection.json").write_text(json.dumps(chosen, indent=2), newline="\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
