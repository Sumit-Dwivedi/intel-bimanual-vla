"""v2 Stage 3 mandatory validation: home pose and scene geometry under
top-down IK (ADR-072).

**What this script checks, and why it exists.** Stages 1 (`ik_geometric.py`,
ADR-070) and 2 (`motion.py`, ADR-071) built two new *primitives*. Neither
stage touched the SCENE (`scenes/so101/`, `gen_dual_scene.py`) or the
existing "home" rest keyframe (ADR-026). This stage's job is not to build a
new primitive at all -- it is to point the two new primitives at the
EXISTING scene and EXISTING home pose and measure, honestly, whether they
still hold up now that reachability is judged by the stricter top-down
solver instead of `ik.py`'s position-only one. Four questions, matching the
task brief's parts A-D:

  (A) Does the CURRENT "home" keyframe (`shoulder_lift=-1.2,
      elbow_flex=-1.6`, ADR-026) still pass a fresh set of collision/
      clearance criteria? (The task brief's premise that it does NOT --
      "start from all-zeros instead" -- is checked and shown to be
      backwards: all-zeros is the pose ADR-026 already measured colliding
      by up to -0.0597 m. This script re-measures both poses on THIS
      machine to confirm that finding still holds, then evaluates the
      CURRENT home pose against the newer, stricter criteria the task
      brief asks for -- approach-column clearance and pinch-point height --
      which ADR-026 never checked because they did not exist as concepts
      yet.)
  (B) Do the four props (fork/mug/plate/water_bottle) reach under
      `ik_geometric.solve_topdown_ik`, at a small hover sweep, for both
      arms? And does the two-arm handoff corridor (the xy region BOTH arms
      can top-down-reach) stay wide enough at 0.5 m base separation, at
      five heights, WITHOUT sweeping separation itself (a deliberate scope
      limit -- see the task brief and this script's own corridor section)?
  (C) Is `water_bottle` reachable at all, by either arm, at any tested
      hover height? (Measured, not assumed -- and if not, WHY not,
      diagnosed via the closed form's own internal reachability terms
      rather than left as an opaque "solver returned None".)
  (D) Two fork handoff grasp points, ~6 cm apart along world +x (the
      fork's own long axis, since `fork.xquat` is the identity quaternion
      -- see `gen_dual_scene.py`'s `<body name="fork">` element, which
      carries no `quat=` override at all), each top-down reachable by its
      own arm with a measured (not assumed) margin to that arm's own
      reachable boundary.

**Methodology notes shared by every section below:**
  - Reachability is judged the SAME way `scripts/v2_validate_ik.py`'s own
    workspace map judges it (ADR-070's own convention, reused here rather
    than re-invented): `ik_geometric.solve_topdown_ik(...)` returning a
    non-`None` array IS the reachability answer. This is a closed form with
    joint-range checks baked in (see that module's docstring), so "reachable"
    here already means "every one of the five solved joint angles lies
    inside `model.jnt_range`" -- it is a genuine kinematic answer, not a
    heuristic distance check. It does NOT include a collision check (same
    limitation ADR-070's own workspace map has) -- a cell can be
    "kinematically reachable" and still tunnel through the table the way
    ADR-026/ADR-027 found for the OLD solver. Collision is checked
    separately, at the home pose only (section A), not across the sweeps.
  - Every number in the ADR/report that matters is a bm-ptl run (ADR-047:
    bm-ptl is authoritative for any cross-machine floating-point question).
    This script is written to run identically on the laptop first (this
    session's own premise, like Stage 1/2, is that MuJoCo now imports and
    steps here too) purely for fast iteration; only the bm-ptl numbers are
    treated as ground truth in DECISIONS.md/ARCHITECTURE.md.

Run:
    python scripts\\v2_validate_scene.py                  (laptop, iteration)
    C:\\Users\\devcloud\\project\\ov_env\\Scripts\\python.exe scripts\\v2_validate_scene.py
                                                            (bm-ptl, authoritative)
"""
from __future__ import annotations

import math
import pathlib
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control import ik_geometric as ikg  # noqa: E402
from bimanual.control.skills_scripted import TABLE_SURFACE_Z  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

ARMS = ("A", "B")

# ---------------------------------------------------------------------------
# (A) Home-pose collision/clearance criteria
# ---------------------------------------------------------------------------

#: Props checked for "approach column" clearance -- every manipulable body
#: this scene declares (`gen_dual_scene.py`'s WELD_PROP_NAMES), not only the
#: four the task brief tabulated for hover reachability, since a general
#: safety check ("both arms clear of every prop's approach column") reads
#: most naturally as "every prop", and it costs nothing extra to check all
#: five.
ALL_PROP_NAMES = ("plate", "mug", "fork", "spoon", "water_bottle")

#: The task brief specifies "a vertical cylinder above each prop out to
#: +15 cm" but does not specify a RADIUS -- there is no such number in this
#: repo's prior art either (`ik_geometric`'s reachability sweeps use a point
#: target, not a column). Two methodologies were tried during this script's
#: own development (both disclosed in the ADR, not just the one that was
#: kept): first, one constant shared by every prop (this value, matching
#: the largest prop footprint in the scene, `PLATE_DISH_RADIUS_M` in
#: `gen_dual_scene.py`); this OVERSTATED how much clearance a much smaller,
#: thinner object (the spoon, the fork) genuinely needs directly above it,
#: so the check now measures each prop's OWN radius instead
#: (`_prop_planar_radius_m`, called from `check_home_pose`) and this
#: constant is kept only as the number quoted in that rationale, not used
#: as the actual gate any more.
APPROACH_COLUMN_RADIUS_M = 0.06
APPROACH_COLUMN_HEIGHT_M = 0.15

# ---------------------------------------------------------------------------
# (B) Prop hover-reachability sweep -- reproduces the task brief's own table.
# ---------------------------------------------------------------------------

#: (dx, dy, dz) is not needed -- every prop's REST position is read live
#: from the compiled model (data.xpos), never hardcoded, so this script
#: stays correct even if gen_dual_scene.py's PLATE_POS/MUG_POS/etc. ever
#: move again. Only the four props the task brief's own table names are
#: swept here; `spoon` is covered by the approach-column check above
#: instead (it was not in the brief's hover table).
HOVER_PROP_NAMES = ("fork", "mug", "plate", "water_bottle")

#: Hover offsets above each prop's own rest z, in metres. 0.0 = "grasp"
#: (the resting height itself); the rest match the task brief's table
#: exactly (+0.03, +0.05, +0.08, +0.10).
HOVER_OFFSETS_M = (0.0, 0.03, 0.05, 0.08, 0.10)

# ---------------------------------------------------------------------------
# (B) Handoff corridor sweep -- CONFIRMS the task brief's own 5-height table
# at the EXISTING 0.5 m base separation. Per the task brief's explicit
# instruction, this does NOT sweep separation itself -- ARM_GAP_Y in
# gen_dual_scene.py is read live and reported, never changed here.
# ---------------------------------------------------------------------------

CORRIDOR_HEIGHTS_M = (0.38, 0.42, 0.45, 0.50, 0.55)
#: 2 cm grid, generous bounds (wider than the task brief's own reported
#: extents, [-0.18, 0.18] x and 0.12 m y) so a boundary that has shifted
#: since the laptop measurement would still be caught rather than clipped
#: by too-tight sweep bounds.
CORRIDOR_X = np.round(np.arange(-0.34, 0.34 + 1e-9, 0.02), 2)
CORRIDOR_Y = np.round(np.arange(-0.20, 0.20 + 1e-9, 0.02), 2)

# ---------------------------------------------------------------------------
# (D) Fork handoff grasp points.
# ---------------------------------------------------------------------------

#: Handoff line: z chosen in the middle of the task brief's recommended
#: "z ~= 0.40-0.42" band (itself inside the two widest measured corridor
#: heights, 0.38 and 0.42), y at the centre of the measured shared band
#: (|y| <= 0.06), x split symmetrically about 0 so the two grasp points are
#: exactly 0.06 m (6 cm) apart along world +x -- the fork's own long axis
#: (see module docstring: fork.xquat is identity, so local +x IS world +x).
HANDOFF_Z_M = 0.40
HANDOFF_Y_M = 0.0
HANDOFF_HALF_SEP_M = 0.03  # each point is +/- this from x=0 -> 0.06 m apart
P_FROM_XYZ = (-HANDOFF_HALF_SEP_M, HANDOFF_Y_M, HANDOFF_Z_M)  # arm A
P_TO_XYZ = (HANDOFF_HALF_SEP_M, HANDOFF_Y_M, HANDOFF_Z_M)     # arm B

#: Grasp yaw for the fork = fork heading (world +x, i.e. 0 rad) + 90 deg,
#: so the gripper jaws close ACROSS the fork's shaft rather than trying to
#: close along it (closing along a thin, rigid shaft the jaws cannot
#: straddle would not form a pinch at all). See module docstring.
FORK_GRASP_YAW_RAD = math.pi / 2.0

#: How far (metres) to march when measuring a reachability margin -- see
#: `_measure_margin_m`. 2 mm steps, matching the sub-mm precision the
#: closed form itself is capable of (ADR-070's own round-trip error is
#: ~1e-16 m); the margin only needs to be resolved to about 1 cm for the
#: >= 3 cm gate to be a meaningful pass/fail, so 2 mm steps are generous.
MARGIN_STEP_M = 0.002
MARGIN_MAX_SEARCH_M = 0.30


# ---------------------------------------------------------------------------
# Shared MuJoCo lookups (same conventions as scripts/probe_reachability.py
# and scripts/v2_validate_ik.py -- reused, not reinvented).
# ---------------------------------------------------------------------------


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _geom_name(model, gid: int) -> str:
    """Human-readable label for geom `gid`. Many of the upstream SO-101
    mesh collision geoms (`scenes/so101/so101_new_calib.xml`) carry no
    explicit `name=` attribute at all -- MJCF does not require one -- so
    `mj_id2name` returns `None`/empty for a lot of the geoms that matter
    most here (the ones that produced ADR-026's own -0.0597 m finding are
    among them). Falling back to the geom's PARENT BODY name (always
    present -- `gen_dual_scene.py`'s renaming pass requires it) keeps the
    report legible instead of printing an opaque `<geom#44>`."""
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
    if name:
        return name
    body_id = int(model.geom_bodyid[gid])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return f"<unnamed geom#{gid} on body {body_name!r}>"


def _arm_body_ids(model, arm: str) -> set[int]:
    """Every body id whose name is prefixed `arm{arm}_` (the arm's whole
    kinematic subtree) -- same convention as probe_reachability.py."""
    prefix = f"arm{arm}_"
    ids = set()
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name and name.startswith(prefix):
            ids.add(b)
    return ids


def _geom_ids_for_bodies(model, body_ids: set[int]) -> set[int]:
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}


def _table_and_drawer_geom_ids(model) -> set[int]:
    """Every geom belonging to the table or the drawer housing/box -- the
    static furniture an arm must never tunnel into. Matched by BODY name
    prefix, same pattern as `_arm_body_ids`, so this stays correct if the
    hand-authored template ever adds another table/drawer sub-body."""
    names_wanted = {"table", "drawer_housing", "drawer"}
    body_ids = set()
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name in names_wanted:
            body_ids.add(b)
    return _geom_ids_for_bodies(model, body_ids)


def _prop_planar_radius_m(model, data, prop_body_id: int, prop_xy: np.ndarray) -> float:
    """Measured (not assumed) planar "how far from its own centre does this
    prop's geometry actually reach" radius, in metres -- used as this
    prop's OWN approach-column radius in check (A3) below, instead of one
    constant shared by every prop regardless of size.

    Why per-prop, not one shared constant: a spoon (a ~0.4-1.2 cm-radius
    capsule) and a plate (a 6 cm dish) are wildly different sizes, and using
    the plate's own footprint as a stand-in "conservative" radius for the
    spoon would overstate how much clearance the spoon's column genuinely
    needs -- a false positive risk this measurement avoids by reading each
    prop's ACTUAL geometry instead.

    For every geom belonging to this prop's body, the WORLD-frame distance
    from the geom's own centre (`data.geom_xpos`) to the prop body's own
    world xy position, plus that geom's own bounding-sphere radius
    (`model.geom_rbound`) -- i.e. "how far this one geom's own extent
    reaches, measured from the body's origin". The prop's radius is the MAX
    of that over all its geoms (the single farthest-reaching piece of
    geometry), exactly the same "distance + rbound" pattern the arm-side of
    this same check (A3's main loop) already uses for arm geoms, just
    applied to the prop instead.
    """
    max_reach = 0.0
    for g in range(model.ngeom):
        if int(model.geom_bodyid[g]) != prop_body_id:
            continue
        gxy = np.array(data.geom_xpos[g][:2], dtype=np.float64)
        reach = float(np.linalg.norm(gxy - prop_xy)) + float(model.geom_rbound[g])
        max_reach = max(max_reach, reach)
    return max_reach


def _pinch_point(model, data, arm: str) -> np.ndarray:
    """The ADR-025 pinch point (midpoint of the fixed and moving jaw
    bodies), read LIVE from `data.xpos` -- i.e. the real pinch point at
    whatever pose `data` currently holds (unlike `ik_geometric._calibrate`'s
    cached zero-pose value, this one reflects the CURRENT jaw angle and arm
    configuration, which is exactly what section (A) below needs to check
    the home pose's actual pinch height)."""
    fixed_id = _body_id(model, ik.fixed_jaw_body_name(arm))
    moving_id = _body_id(model, ik.moving_jaw_body_name(arm))
    return 0.5 * (
        np.array(data.xpos[fixed_id], dtype=np.float64)
        + np.array(data.xpos[moving_id], dtype=np.float64)
    )


# ---------------------------------------------------------------------------
# (A) Home-pose validation.
# ---------------------------------------------------------------------------


def _classify_contacts(model, data) -> dict:
    """Classify every current contact into: self_A, self_B (both geoms on
    the SAME arm), cross_arm (one geom on each arm), arm_vs_table (one arm
    geom, one table/drawer geom), other (everything else -- e.g. a prop
    resting on the table, which is expected and not a defect). Returns
    counts plus the deepest (most negative `dist`) contact in each category,
    with both geom names, for a human-readable report -- same convention
    ADR-026's own measurement used ("34 total, 29 armA<->armB, deepest
    -0.0597 m, armA_wrist vs armB_wrist")."""
    arm_geoms = {arm: _geom_ids_for_bodies(model, _arm_body_ids(model, arm)) for arm in ARMS}
    table_geoms = _table_and_drawer_geom_ids(model)

    buckets = {
        "self_A": [], "self_B": [], "cross_arm": [], "arm_vs_table": [], "other": [],
    }
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        # Lists, not sets: a contact between TWO geoms of the same arm
        # (self-collision) must count as 2 hits, not collapse to {True}.
        in_a = [g1 in arm_geoms["A"], g2 in arm_geoms["A"]]
        in_b = [g1 in arm_geoms["B"], g2 in arm_geoms["B"]]
        in_table = [g1 in table_geoms, g2 in table_geoms]
        row = (g1, g2, float(c.dist), _geom_name(model, g1), _geom_name(model, g2))
        a_hits = sum(1 for x in in_a if x)
        b_hits = sum(1 for x in in_b if x)
        table_hit = any(in_table)
        if a_hits >= 1 and b_hits >= 1:
            buckets["cross_arm"].append(row)
        elif a_hits == 2:
            buckets["self_A"].append(row)
        elif b_hits == 2:
            buckets["self_B"].append(row)
        elif (a_hits >= 1 or b_hits >= 1) and table_hit:
            buckets["arm_vs_table"].append(row)
        else:
            buckets["other"].append(row)
    return buckets


def check_home_pose(env: TableSettingEnv, log) -> dict:
    model = env.model

    log("--- (A0) Sanity re-check: all-zeros pose (Lab 8's suggested home) ---")
    scratch = mujoco.MjData(model)
    mujoco.mj_resetData(model, scratch)  # qpos0: every joint at its compiled default, i.e. all zeros
    mujoco.mj_forward(model, scratch)
    zeros_buckets = _classify_contacts(model, scratch)
    zeros_cross = zeros_buckets["cross_arm"]
    zeros_total = sum(len(v) for v in zeros_buckets.values())
    if zeros_cross:
        deepest = min(zeros_cross, key=lambda r: r[2])
        log(f"  all-zeros: {zeros_total} total contacts, {len(zeros_cross)} armA<->armB, "
            f"deepest {deepest[2]:.4f} m ({deepest[3]} vs {deepest[4]})")
    else:
        log(f"  all-zeros: {zeros_total} total contacts, 0 armA<->armB")
    log("  (ADR-026 recorded 34 total / 29 armA<->armB / -0.0597 m at this pose; "
        "re-measured here on this machine as a live cross-check, not assumed.)")

    log("")
    log("--- (A1) CURRENT home keyframe (reset(seed=0), ADR-026) ---")
    env.reset(seed=0)
    data = env.data
    home_buckets = _classify_contacts(model, data)
    home_total = sum(len(v) for v in home_buckets.values())
    log(f"  total contacts at home: {home_total}")
    for key in ("self_A", "self_B", "cross_arm", "arm_vs_table"):
        rows = home_buckets[key]
        if rows:
            deepest = min(rows, key=lambda r: r[2])
            log(f"  {key}: {len(rows)} contact(s), deepest {deepest[2]:.4f} m "
                f"({deepest[3]} vs {deepest[4]})")
        else:
            log(f"  {key}: 0 contacts")
    other_rows = home_buckets["other"]
    log(f"  other (e.g. prop resting on table -- expected, not a defect): "
        f"{len(other_rows)} contact(s)")

    no_self_collision = not home_buckets["self_A"] and not home_buckets["self_B"]
    no_table_contact = not home_buckets["arm_vs_table"]
    no_cross_arm = not home_buckets["cross_arm"]
    log(f"  gate 1 (no self-collision): {'PASS' if no_self_collision else 'FAIL'}")
    log(f"  gate 2 (no arm-vs-table contact): {'PASS' if no_table_contact else 'FAIL'}")
    log(f"  gate 3 (no cross-arm contact): {'PASS' if no_cross_arm else 'FAIL'}")

    log("")
    log("--- (A2) Pinch-point height above table (home pose) ---")
    pinch_results = {}
    for arm in ARMS:
        p = _pinch_point(model, data, arm)
        margin = float(p[2] - TABLE_SURFACE_Z)
        pinch_results[arm] = (p, margin)
        log(f"  arm {arm}: pinch point z={p[2]:.4f} m, table surface z={TABLE_SURFACE_Z} m, "
            f"margin={margin:+.4f} m -> {'PASS' if margin > 0 else 'FAIL'}")
    pinch_pass = all(m > 0 for (_, m) in pinch_results.values())

    log("")
    log("--- (A3) Both arms clear of every prop's approach column ---")
    log(f"  column height=[prop_z, prop_z+{APPROACH_COLUMN_HEIGHT_M}] m; column radius is "
        f"MEASURED per prop (see _prop_planar_radius_m docstring), not one shared constant --")
    log(f"  a fixed {APPROACH_COLUMN_RADIUS_M} m radius (the plate's own footprint) applied "
        f"uniformly to a spoon or fork would overstate how much space those much thinner "
        f"objects actually need directly above them.")
    all_arm_geoms = _geom_ids_for_bodies(
        model, _arm_body_ids(model, "A") | _arm_body_ids(model, "B")
    )
    column_rows = []
    for prop in ALL_PROP_NAMES:
        prop_body = _body_id(model, prop)
        prop_xyz = np.array(data.xpos[prop_body], dtype=np.float64, copy=True)
        column_radius = _prop_planar_radius_m(model, data, prop_body, prop_xyz[:2])
        z_lo, z_hi = prop_xyz[2], prop_xyz[2] + APPROACH_COLUMN_HEIGHT_M
        min_clearance = None
        min_clearance_geom = None
        for g in all_arm_geoms:
            gz = float(data.geom_xpos[g][2])
            rbound = float(model.geom_rbound[g])
            # Only a geom whose own bounding sphere can possibly reach into
            # the column's z-range matters -- cheap early-out.
            if gz + rbound < z_lo or gz - rbound > z_hi:
                continue
            gxy = np.array(data.geom_xpos[g][:2], dtype=np.float64)
            xy_dist = float(np.linalg.norm(gxy - prop_xyz[:2]))
            clearance = xy_dist - rbound - column_radius
            if min_clearance is None or clearance < min_clearance:
                min_clearance = clearance
                min_clearance_geom = g
        passed = (min_clearance is None) or (min_clearance > 0)
        geom_label = _geom_name(model, min_clearance_geom) if min_clearance_geom is not None else "(no geom in z-range)"
        clearance_label = "n/a (no arm geom in this column's z-range)" if min_clearance is None else f"{min_clearance:+.4f} m"
        column_rows.append((prop, min_clearance, geom_label, passed, column_radius))
        log(f"  {prop} (measured column radius={column_radius:.4f} m): min clearance = {clearance_label} "
            f"(closest: {geom_label}) -> {'PASS' if passed else 'FAIL'}")
    column_pass = all(row[3] for row in column_rows)

    return {
        "zeros_total": zeros_total,
        "zeros_cross_arm": len(zeros_cross),
        "zeros_deepest": min((r[2] for r in zeros_cross), default=None),
        "home_total": home_total,
        "home_buckets": {k: len(v) for k, v in home_buckets.items()},
        "home_deepest": {
            k: (min(v, key=lambda r: r[2]) if v else None) for k, v in home_buckets.items()
        },
        "no_self_collision": no_self_collision,
        "no_table_contact": no_table_contact,
        "no_cross_arm": no_cross_arm,
        "pinch_results": pinch_results,
        "pinch_pass": pinch_pass,
        "column_rows": column_rows,
        "column_pass": column_pass,
        "overall_pass": no_self_collision and no_table_contact and no_cross_arm and pinch_pass and column_pass,
    }


# ---------------------------------------------------------------------------
# (B)/(C) Prop hover-reachability sweep.
# ---------------------------------------------------------------------------


def _diagnose_unreachable(model, data, arm: str, target_xyz) -> str:
    """For a target `solve_topdown_ik` rejects, report WHY in terms of the
    solver's own internal reachability tests (the shoulder-offset circle and
    the 2-link annulus -- see `ik_geometric`'s module docstring), rather than
    leaving a bare None unexplained. Does not modify `ik_geometric.py` --
    this re-derives the same two scalar checks from `_calibrate`'s already-
    public dataclass fields, which is a read, not an edit."""
    consts = ikg._calibrate(model, arm)  # noqa: SLF001 -- read-only use of a cached, public dataclass
    target = np.asarray(target_xyz, dtype=np.float64).reshape(3)
    p0 = consts.anchors[0]
    target_rel = target - p0
    r_target_sq = float(target_rel[0] ** 2 + target_rel[1] ** 2)

    # Same `d` computation solve_topdown_ik uses at yaw=0 (theta5_star
    # alone) -- close enough for a diagnostic (yaw shifts it only slightly;
    # see ik_geometric's "Yaw and the lateral offset" docstring section).
    theta5 = consts.theta5_star
    p4, a4 = consts.anchors[4], consts.axes[4]
    known_tail_x = ikg._rotate_about(p4, a4, theta5, consts.pinch0)[0] - consts.anchors[3][0]
    d = float((consts.anchors[1] - p0)[0] + known_tail_x)

    if r_target_sq < d * d:
        return f"target is INSIDE the shoulder-offset circle (d={abs(d):.4f} m, target radius={math.sqrt(max(r_target_sq,0)):.4f} m) -- unreachable at any pan angle"
    r_perp = math.sqrt(max(r_target_sq - d * d, 0.0))
    annulus_lo = abs(consts.l1_len - consts.l2_len)
    annulus_hi = consts.l1_len + consts.l2_len
    return (
        f"planar horizontal reach required (r_perp) = {r_perp:.4f} m vs 2-link annulus "
        f"[{annulus_lo:.4f}, {annulus_hi:.4f}] m (l1={consts.l1_len:.4f}, l2={consts.l2_len:.4f}) "
        f"-> {'inside' if annulus_lo <= r_perp <= annulus_hi else 'OUTSIDE -- horizontal reach problem'}"
    )


def prop_hover_sweep(env: TableSettingEnv, log) -> list[dict]:
    model, data = env.model, env.data
    rows = []
    log("--- (B) Prop hover-reachability sweep (both arms, solve_topdown_ik) ---")
    header = "prop".ljust(14) + "arm  " + "  ".join(f"+{o:.2f}m".rjust(8) for o in HOVER_OFFSETS_M)
    log(header)
    for prop in HOVER_PROP_NAMES:
        body_id = _body_id(model, prop)
        prop_xyz = np.array(data.xpos[body_id], dtype=np.float64, copy=True)
        for arm in ARMS:
            cells = []
            for dz in HOVER_OFFSETS_M:
                target = prop_xyz + np.array([0.0, 0.0, dz])
                sol = ikg.solve_topdown_ik(model, data, arm, target, yaw=0.0)
                ok = sol is not None
                cells.append(ok)
                rows.append({"prop": prop, "arm": arm, "dz": dz, "target": target, "ok": ok})
            row_str = prop.ljust(14) + f" {arm}   " + "  ".join(("OK".rjust(8) if ok else "FAIL".rjust(8)) for ok in cells)
            log(row_str)
            if not any(cells):
                # Every height failed for this (prop, arm) -- diagnose the
                # grasp-height target specifically, since that is the one
                # the brief's table calls out as unreachable (water_bottle).
                diag = _diagnose_unreachable(model, data, arm, prop_xyz)
                log(f"    -> unreachable at every tested height; diagnosis at grasp height: {diag}")
    return rows


# ---------------------------------------------------------------------------
# (B) Handoff corridor sweep.
# ---------------------------------------------------------------------------


def handoff_corridor_sweep(env: TableSettingEnv, log) -> list[dict]:
    model, data = env.model, env.data
    log("")
    log("--- (B) Handoff corridor sweep (two-arm top-down intersection, 2cm grid) ---")
    log(f"  NOTE: base separation is read live, not swept -- see module docstring "
        f"for why the task brief's contingency (sweep separation) is deliberately "
        f"NOT exercised here.")
    rows = []
    for z in CORRIDOR_HEIGHTS_M:
        grid_a = np.zeros((len(CORRIDOR_Y), len(CORRIDOR_X)), dtype=bool)
        grid_b = np.zeros_like(grid_a)
        for yi, y in enumerate(CORRIDOR_Y):
            for xi, x in enumerate(CORRIDOR_X):
                target = np.array([x, y, z], dtype=np.float64)
                grid_a[yi, xi] = ikg.solve_topdown_ik(model, data, "A", target, yaw=0.0) is not None
                grid_b[yi, xi] = ikg.solve_topdown_ik(model, data, "B", target, yaw=0.0) is not None
        both = grid_a & grid_b
        n_both = int(both.sum())
        if n_both:
            xs = CORRIDOR_X[np.any(both, axis=0)]
            ys = CORRIDOR_Y[np.any(both, axis=1)]
            x_extent = (float(xs.min()), float(xs.max()))
            y_extent_m = float(ys.max() - ys.min())
            y_range = (float(ys.min()), float(ys.max()))
        else:
            x_extent, y_extent_m, y_range = None, 0.0, None
        rows.append({"z": z, "n_both": n_both, "x_extent": x_extent, "y_extent_m": y_extent_m, "y_range": y_range})
        log(f"  z={z:.2f}: {n_both} cells reachable by BOTH  "
            f"x_extent={x_extent}  y_extent={y_extent_m:.3f} m  y_range={y_range}")
    return rows


# ---------------------------------------------------------------------------
# (D) Fork handoff grasp points.
# ---------------------------------------------------------------------------


def _measure_margin_m(model, data, arm: str, base_xyz, direction_xy, yaw: float) -> float:
    """March from `base_xyz` along `direction_xy` (a unit-ish 2-vector in
    the xy plane, z held fixed) in `MARGIN_STEP_M` steps until
    `solve_topdown_ik` first returns None for `arm`. Returns the distance
    travelled before that first failure -- i.e. a measured (not assumed)
    margin to `arm`'s own top-down-reachable boundary in that direction.

    This assumes (verified by the caller checking `base_xyz` itself solves
    first) that `base_xyz` is reachable, and that reachability does not
    flicker back on after failing once along this ray -- true for the
    smooth, convex-ish regions this solver's own annulus/circle geometry
    produces locally, and cheap to sanity-check by eye against the coarse
    corridor grid above.
    """
    direction_xy = np.asarray(direction_xy, dtype=np.float64)
    direction_xy = direction_xy / np.linalg.norm(direction_xy)
    base = np.asarray(base_xyz, dtype=np.float64)
    dist = 0.0
    while dist < MARGIN_MAX_SEARCH_M:
        dist += MARGIN_STEP_M
        probe = base + np.array([direction_xy[0] * dist, direction_xy[1] * dist, 0.0])
        if ikg.solve_topdown_ik(model, data, arm, probe, yaw=yaw) is None:
            return dist - MARGIN_STEP_M
    return MARGIN_MAX_SEARCH_M  # did not fail within the search radius


def fork_handoff_points(env: TableSettingEnv, log) -> dict:
    model, data = env.model, env.data
    log("")
    log("--- (D) Fork handoff grasp points ---")
    log(f"  P_from (arm A) = {P_FROM_XYZ}   P_to (arm B) = {P_TO_XYZ}   "
        f"separation along x = {P_TO_XYZ[0] - P_FROM_XYZ[0]:.3f} m")

    results = {}
    for label, xyz, arm in (("P_from", P_FROM_XYZ, "A"), ("P_to", P_TO_XYZ, "B")):
        for yaw_label, yaw in (("yaw=0.0 (best top-down alignment)", 0.0),
                                ("yaw=+pi/2 (real fork grasp orientation)", FORK_GRASP_YAW_RAD)):
            sol = ikg.solve_topdown_ik(model, data, arm, xyz, yaw=yaw)
            reachable = sol is not None
            log(f"  {label} (arm {arm}) @ {yaw_label}: {'REACHABLE' if reachable else 'UNREACHABLE'}")
            if reachable:
                margins = {}
                for dir_name, direction in (("+x", (1, 0)), ("-x", (-1, 0)), ("+y", (0, 1)), ("-y", (0, -1))):
                    m = _measure_margin_m(model, data, arm, xyz, direction, yaw)
                    margins[dir_name] = m
                min_dir = min(margins, key=margins.get)
                log(f"    margins (m): " + ", ".join(f"{k}={v:.3f}" for k, v in margins.items())
                    + f"  -> tightest {min_dir}={margins[min_dir]:.3f} m "
                    + f"({'PASS >= 0.03 m' if margins[min_dir] >= 0.03 else 'FAIL < 0.03 m'})")
                results[(label, yaw_label)] = {"reachable": True, "margins": margins}
            else:
                results[(label, yaw_label)] = {"reachable": False, "margins": None}
    return results


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main() -> int:
    t0 = time.time()
    out_lines: list[str] = []

    def log(s: str = "") -> None:
        print(s)
        out_lines.append(s)

    log("=" * 78)
    log("v2 Stage 3 scene/home-pose validation (ADR-072)")
    log(f"mujoco version: {mujoco.__version__}")
    log("=" * 78)

    env = TableSettingEnv()
    log(f"scene: {env.scene_path}")
    log(f"nq={env.model.nq} nv={env.model.nv} nu={env.model.nu}")
    # mj_forward once, up front, purely so data.xpos below is populated --
    # a freshly-constructed MjData has not run forward kinematics yet and
    # every derived quantity (xpos included) reads as zero until it does.
    # check_home_pose() below calls env.reset() (which itself calls
    # mj_forward) immediately after, so this is not load-bearing for
    # anything past this one log line.
    mujoco.mj_forward(env.model, env.data)
    # Base separation, read LIVE from the compiled model (never hardcoded --
    # see module docstring: this script deliberately does not sweep this
    # value, but it does report the exact one the rest of this run assumes).
    armA_base_id = _body_id(env.model, "armA_base")
    armB_base_id = _body_id(env.model, "armB_base")
    base_sep_m = abs(
        float(env.data.xpos[armA_base_id][1]) - float(env.data.xpos[armB_base_id][1])
    )
    log(f"arm base separation (live, not swept): {base_sep_m:.3f} m")
    log("")

    home_report = check_home_pose(env, log)
    log("")
    hover_rows = prop_hover_sweep(env, log)
    corridor_rows = handoff_corridor_sweep(env, log)
    handoff_report = fork_handoff_points(env, log)

    log("")
    log("=" * 78)
    log(f"OVERALL HOME-POSE VERDICT: {'PASS' if home_report['overall_pass'] else 'FAIL'}")
    log(f"total wall clock: {time.time() - t0:.1f} s")
    log("=" * 78)

    log_path = REPO_ROOT / "scripts" / "_v2_validate_scene_log.txt"
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("\n".join(out_lines))
    print(f"\nsummary log written to {log_path}")

    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
