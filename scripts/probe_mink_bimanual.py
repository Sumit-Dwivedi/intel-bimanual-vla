"""Scaffolding probe: is `kevinzakka/mink` (Apache-2.0) usable for
collision-aware bimanual IK on this project's SO-101 dual-arm scene?

**This is a probe, not production code.** It does not replace, wire into, or
change `src/bimanual/control/ik.py`, `skills_scripted.py` or `executor.py` --
see `docs/hardware/m06-mink-probe.md` for the write-up, the measured numbers
and the verdicts this probe exists to produce. Runs on bm-ptl only (ADR-020:
`mujoco` -- and now `mink`, which imports `mujoco` itself -- cannot import
on the developer's Windows laptop).

**What mink is, briefly, for a reader new to this library.** MuJoCo's own
`mj_jacSite`/`mj_jacBody` (used by `ik.py`) give you a Jacobian; turning a
target pose into a joint-velocity command, subject to joint limits, velocity
limits AND collision avoidance, is left to the caller. mink packages that
loop: you declare `Task` objects (e.g. `FrameTask`, "drive this named frame
toward this target pose") and `Limit` objects (e.g. `CollisionAvoidanceLimit`,
"never let these geom groups get closer than X"), and `mink.solve_ik` casts
one iteration of that as a quadratic program (a QP: minimise a weighted sum
of task errors subject to the limits as linear inequality constraints) and
returns a joint *velocity* to integrate for one small time step `dt`. You
call it in a loop, integrating `dt` each time, exactly like `ik.py`'s own
Newton/damped-least-squares loop -- the difference is the limits are *solved
against*, not merely checked after the fact.

**Four corrected assumptions this probe follows (see the task brief and
`docs/hardware/m06-mink-probe.md` for why each one is NOT what a naive
reading of mink's aloha example would produce):**
  1. Targets the PINCH POINT (midpoint of the fixed and moving jaw BODIES,
     ADR-025), not the `armX_gripperframe` SITE, and NOT via a `FrameTask` on
     EACH jaw body either -- that was tried first and rejected; see
     `make_pinch_task`'s docstring for why two independent 3-equation
     position tasks on two (nearly) rigidly-coupled points reintroduces
     something close to a 6-DoF pose constraint, exactly what correction 2
     below says a 5-DoF arm cannot generally satisfy. Instead: ONE
     position-only `FrameTask` on the FIXED jaw body (`armX_gripper`), with
     its target re-derived every solver iteration so that frame's own
     position, if reached exactly, would put the (fixed, moving)-jaw
     midpoint at the desired pinch target (see `set_pinch_target` below).
     Targeting the site directly would repeat the ~8 cm miss ADR-025 already
     measured and fixed; targeting the fixed jaw body's OWN position
     unmodified would repeat a smaller but still real ~1.8 cm miss (measured
     below, at the "home" pose) -- the per-iteration offset removes both.
  2. Position-only: every `FrameTask` below is built with
     `orientation_cost=0.0` (ADR-024: SO-101 has 5 positioning DoF against a
     6-DoF task space; constraining orientation too would over-constrain a
     5-DoF arm and produce a large, uninformative residual that looks like
     "mink doesn't work" when the real cause is the task specification).
  3. `(0, 0, 0.43)` (ADR-036's current `HANDOFF_POSITION_XYZ`, close enough
     to the brief's `(0,0,0.43)` to use directly) is NOT claimed here to be
     "mutually reachable" in advance -- Test 3 measures whether mink's
     *simultaneous* solve gets both arms there, or whether
     `CollisionAvoidanceLimit` holds them apart, and reports whichever
     actually happens.
  4. mink's version is RESOLVED from PyPI at run time (`1.3.0` as of
     2026-09-14, not the brief's assumed, unverified `1.2.0`), and every
     class/function used below (`Configuration`, `FrameTask`,
     `CollisionAvoidanceLimit`, `ConfigurationLimit`, `VelocityLimit`,
     `solve_ik`, `get_subtree_geom_ids`) was checked against the INSTALLED
     package's own `inspect.signature`/docstring before being used here, not
     assumed from the brief.

**A consequence discovered while installing, reported here rather than
buried in a requirements-file comment:** mink 1.3.0 requires
`mujoco>=3.10.0`; the pin this project shipped (`mujoco==3.2.7`) does not
satisfy that, and `pip install mink==1.3.0` silently upgraded the shared
bm-ptl venv's mujoco to 3.13.0. `scripts/requirements-bmptl.txt` now pins
3.13.0 for that reason (see its comment block) and
`docs/hardware/m06-mink-probe.md` records that `pytest tests/test_skills.py`
was re-run under 3.13.0 and reproduced the pre-existing 4 passed / 4 failed
baseline byte-for-byte, so this probe's own presence did not silently change
production behaviour.
"""

from __future__ import annotations

import importlib.metadata
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import mink  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control.skills_scripted import (  # noqa: E402
    CLEARANCE_HEIGHT_M,
    GRASP_POINT_OFFSET_M,
    HANDOFF_POSITION_XYZ,
)

SCENE_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / "src"
    / "bimanual"
    / "sim"
    / "assets"
    / "so101_dual_table.xml"
)

ARMS = ("A", "B")

#: Props declared by M02 (`so101_dual_table.xml`) -- the five bodies the
#: brief's "each arm vs each prop body" collision pair asks for.
PROP_BODIES = ("plate", "mug", "fork", "spoon", "water_bottle")

#: Number of solver iterations per test, per the brief ("up to 5
#: iterations"). Each iteration is one `mink.solve_ik` QP solve + one
#: `Configuration.integrate_inplace` velocity step -- NOT 5 physical
#: `env.step()` calls; mink never touches MuJoCo's actuators or physics
#: here, it only manipulates a kinematic `Configuration` (a MjModel +
#: scratch MjData pair), exactly as `ik.py`'s own scratch-`MjData` solve
#: does. Nothing in this probe advances real simulation time.
MAX_ITERS = 5

#: Integration time step handed to `mink.solve_ik`/`integrate_inplace`, in
#: seconds. Not a physical control-loop rate -- mink's IK is a quasi-static
#: velocity solve, so `dt` just scales how far one iteration's solved
#: velocity is allowed to move the configuration before the next
#: iteration's Jacobians/limits are recomputed. Chosen empirically (see
#: `docs/hardware/m06-mink-probe.md`): large enough that 5 iterations make
#: visible progress on a ~0.3 m move, small enough that `VelocityLimit`
#: below (not this constant) remains the thing actually capping per-step
#: motion.
DT_S = 0.2

#: Per-joint velocity cap fed to `mink.VelocityLimit`, rad/s (all SO-101
#: joints are revolute). Generous relative to the real servo limits in
#: `so101_dual_table.xml`'s `<actuator>` block (this is a kinematics-only
#: probe, not a torque/servo-accurate one) but still a real, finite bound --
#: an unbounded velocity limit would let one QP solve "teleport" arbitrarily
#: far in one `dt`, which would make the 5-iteration budget meaningless as a
#: point of comparison against `ik.py`'s own 60-iteration Newton loop.
JOINT_VELOCITY_LIMIT_RAD_S = 3.0

#: mink's `CollisionAvoidanceLimit` parameters, exactly as specified in the
#: task brief.
MIN_DISTANCE_FROM_COLLISIONS_M = 0.05
COLLISION_DETECTION_DISTANCE_M = 0.1

#: The only QP backend actually installed (`qpsolvers.available_solvers`
#: reported `['daqp']` on bm-ptl after `pip install mink==1.3.0`, which pulls
#: in `qpsolvers[daqp]` automatically -- no separate `pip install daqp` step
#: was needed on this platform, correcting the brief's "may need separate
#: installation" caveat).
SOLVER = "daqp"


# ---------------------------------------------------------------------------
# Small MuJoCo name/id helpers (same pattern as scripts/probe_reachability.py)
# ---------------------------------------------------------------------------


def _body_id(model, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return bid


def _geom_id(model, name: str) -> int:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if gid == -1:
        raise ValueError(f"geom {name!r} not found in the compiled model")
    return gid


def min_pair_distance(model, data, ids1: list[int], ids2: list[int]) -> float:
    """Smallest signed distance (metres) between any geom in `ids1` and any
    geom in `ids2`, via MuJoCo's own `mj_geomDistance` -- the same primitive
    `mink.CollisionAvoidanceLimit` uses internally, queried directly here so
    this probe can report "did collision avoidance actually have something
    to push against" without reaching into mink's private QP-assembly
    state. `distmax` is set generously (0.5 m) so a pair that is nowhere
    near touching still reports its real distance rather than a clipped
    value; a pair is called "engaged" below when this distance drops under
    `COLLISION_DETECTION_DISTANCE_M`.
    """
    best = float("inf")
    for g1 in ids1:
        for g2 in ids2:
            if g1 == g2:
                continue
            d = mujoco.mj_geomDistance(model, data, g1, g2, 0.5, None)
            best = min(best, d)
    return best


# ---------------------------------------------------------------------------
# Collision pairs (aloha's pattern, per the task brief): armA vs armB,
# each arm vs table_top, each arm vs each prop body.
# ---------------------------------------------------------------------------


def build_collision_pairs(model):
    arm_a_ids = mink.get_subtree_geom_ids(model, _body_id(model, "armA_base"))
    arm_b_ids = mink.get_subtree_geom_ids(model, _body_id(model, "armB_base"))
    table_top_ids = [_geom_id(model, "table_top")]

    pairs = [
        (arm_a_ids, arm_b_ids),
        (arm_a_ids, table_top_ids),
        (arm_b_ids, table_top_ids),
    ]
    prop_id_lists = {}
    for prop in PROP_BODIES:
        prop_ids = mink.get_subtree_geom_ids(model, _body_id(model, prop))
        prop_id_lists[prop] = prop_ids
        pairs.append((arm_a_ids, prop_ids))
        pairs.append((arm_b_ids, prop_ids))

    return pairs, arm_a_ids, arm_b_ids, table_top_ids, prop_id_lists


# ---------------------------------------------------------------------------
# Pinch-point task (correction 1). See module docstring point 1.
# ---------------------------------------------------------------------------


def make_pinch_task(arm: str):
    """ONE position-only FrameTask, on the FIXED jaw BODY, for one arm.

    **Why not a FrameTask per jaw body (tried first, rejected).** The first
    version of this probe put an independent position-only FrameTask on
    EACH jaw body (fixed and moving), each with `orientation_cost=0.0`, and
    re-derived both targets every iteration so their midpoint tracked the
    desired pinch point (the same idea as this function, just doubled). It
    ran without crashing but is WRONG in a way worth stating precisely:
    the fixed and moving jaw bodies are (ignoring the small gripper-hinge
    rotation, which is a separate actuator this IK never touches) rigidly
    attached to each other and therefore to the same 5-joint arm. Knowing
    BOTH bodies' 3D positions is equivalent to knowing that rigid pair's
    full 6-DoF pose (3 position + a 2-axis-worth of orientation, since the
    jaw-to-jaw axis is one specific direction, not a free 3rd rotation) --
    i.e. two "position-only" 3-equation tasks on two rigidly-linked points
    combine into something close to the SAME 6-DoF-vs-5-DoF over-constraint
    that correction 2 and ADR-024 say a 5-DoF arm cannot generally satisfy,
    even though neither task alone ever mentions orientation. Measured
    empirically (`docs/hardware/m06-mink-probe.md`): the one-task and
    two-task versions produced IDENTICAL results on this probe's Test 1
    target (both got stuck at the same configuration-limit boundary before
    the distinction could matter), so this was not caught by a difference
    in the numbers -- it is a structural argument, confirmed by inspecting
    what "two rigidly-linked 3-vectors" means, not by a failing test. The
    one-task version below is simpler (one QP task instead of two) AND is
    the DoF-accurate choice, so it is what this probe actually uses.

    `orientation_cost=0.0` (correction 2) -- no orientation target is ever
    specified for the one frame this DOES use, matching ADR-024's decision
    in `ik.py`.
    """
    return mink.FrameTask(
        frame_name=ik.fixed_jaw_body_name(arm),
        frame_type="body",
        position_cost=1.0,
        orientation_cost=0.0,
    )


def pinch_point(data, fixed_id: int, moving_id: int) -> np.ndarray:
    """The pinch point: midpoint of the two jaw bodies' world positions,
    read from `data.xpos` -- identical definition to `ik.py`'s
    `_pinch_point()`."""
    return 0.5 * (data.xpos[fixed_id] + data.xpos[moving_id])


def set_pinch_target(
    data,
    fixed_task,
    fixed_id: int,
    moving_id: int,
    target_xyz: np.ndarray,
) -> None:
    """Re-derive the fixed jaw body's FrameTask target from the CURRENT
    configuration so that, were that body to reach its own target exactly,
    the (fixed, moving)-jaw MIDPOINT -- the pinch point -- would land
    exactly on `target_xyz`.

    Why this is not the same "fixed offset" ADR-025 rejected: ADR-025
    rejected correcting `armX_gripperframe` by a constant vector because
    that vector is a LOCAL-frame offset that rotates with whatever
    orientation the redundant 5-joint solve falls into, so a value measured
    once is wrong everywhere else. This function does not use a value
    measured once -- it reads `fixed_pos`/`moving_pos` fresh from `data`
    every call (every solver iteration, via the caller's loop), exactly the
    way `ik.py`'s `_pinch_point()`/Jacobian averaging is recomputed every
    Newton iteration:

        offset       = fixed_pos_now - pinch_now   (half the fixed-to-moving
                        separation, in WORLD coordinates, at THIS instant)
        target_fixed = target_xyz + offset

    If the fixed jaw body then moved exactly to `target_fixed` without the
    rest of the configuration changing, the pinch point would sit exactly at
    `target_xyz` (since pinch = fixed - offset by definition). Recomputing
    `offset` every iteration is what keeps this correct as the arm moves and
    the jaw's own local geometry changes the offset's world direction.
    """
    fixed_pos = np.array(data.xpos[fixed_id], dtype=np.float64, copy=True)
    moving_pos = np.array(data.xpos[moving_id], dtype=np.float64, copy=True)
    pinch_now = 0.5 * (fixed_pos + moving_pos)
    target_xyz = np.asarray(target_xyz, dtype=np.float64).reshape(3)
    offset = fixed_pos - pinch_now

    fixed_task.set_target(mink.SE3.from_translation(target_xyz + offset))


# ---------------------------------------------------------------------------
# The solve loop itself -- shared by all three tests. `targets` maps
# arm letter -> desired world-frame pinch-point xyz; passing one arm gives
# Test 1's single-arm solve, passing both gives Tests 2 and 3's simultaneous
# bimanual solve.
# ---------------------------------------------------------------------------


def solve(model, config, targets: dict, limits, posture_task, max_iters=MAX_ITERS):
    per_arm = {}
    for arm, target_xyz in targets.items():
        per_arm[arm] = {
            "fixed_task": make_pinch_task(arm),
            "fixed_id": _body_id(model, ik.fixed_jaw_body_name(arm)),
            "moving_id": _body_id(model, ik.moving_jaw_body_name(arm)),
            "target": np.asarray(target_xyz, dtype=np.float64).reshape(3),
        }

    tasks = [posture_task] + [arm_data["fixed_task"] for arm_data in per_arm.values()]

    for _ in range(max_iters):
        for arm_data in per_arm.values():
            set_pinch_target(
                config.data,
                arm_data["fixed_task"],
                arm_data["fixed_id"],
                arm_data["moving_id"],
                arm_data["target"],
            )
        velocity = mink.solve_ik(config, tasks, DT_S, solver=SOLVER, limits=limits)
        config.integrate_inplace(velocity, DT_S)

    results = {}
    for arm, arm_data in per_arm.items():
        achieved = pinch_point(config.data, arm_data["fixed_id"], arm_data["moving_id"])
        residual = float(np.linalg.norm(achieved - arm_data["target"]))
        results[arm] = {
            "target": arm_data["target"].tolist(),
            "achieved_pinch_point": achieved.tolist(),
            "residual_m": residual,
            "converged_lt_1cm": residual < 0.01,
        }
    return results


# ---------------------------------------------------------------------------
# Main: build the model/config/limits once, run all three tests, print a
# JSON blob of every measured number this probe produces (the doc,
# docs/hardware/m06-mink-probe.md, is hand-written FROM this output -- not
# generated by this script -- so the numbers below are exactly what to grep
# the printed JSON for when checking the doc's claims).
# ---------------------------------------------------------------------------


def main() -> None:
    print(f"mink version (importlib.metadata, not hardcoded): {importlib.metadata.version('mink')}")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    config = mink.Configuration(model)
    config.update_from_keyframe("home")

    posture_task = mink.PostureTask(model, cost=1e-2)
    posture_task.set_target_from_configuration(config)

    collision_pairs, arm_a_ids, arm_b_ids, table_top_ids, prop_id_lists = build_collision_pairs(
        model
    )
    collision_limit = mink.CollisionAvoidanceLimit(
        model,
        collision_pairs,
        minimum_distance_from_collisions=MIN_DISTANCE_FROM_COLLISIONS_M,
        collision_detection_distance=COLLISION_DETECTION_DISTANCE_M,
    )
    configuration_limit = mink.ConfigurationLimit(model)
    velocity_limit = mink.VelocityLimit(
        model,
        velocities={
            name: JOINT_VELOCITY_LIMIT_RAD_S
            for arm in ARMS
            for name in ik.arm_joint_names(arm) + [ik.gripper_joint_name(arm)]
        },
    )
    limits = [configuration_limit, velocity_limit, collision_limit]

    report: dict = {"pairs_declared": len(collision_pairs)}

    def collision_snapshot(data) -> dict:
        """Minimum distance (m) for each collision-pair CATEGORY at the
        current configuration, plus whether it is "engaged" (inside
        COLLISION_DETECTION_DISTANCE_M)."""
        snap = {
            "armA_vs_armB": min_pair_distance(model, data, arm_a_ids, arm_b_ids),
            "armA_vs_table_top": min_pair_distance(model, data, arm_a_ids, table_top_ids),
            "armB_vs_table_top": min_pair_distance(model, data, arm_b_ids, table_top_ids),
        }
        for prop, prop_ids in prop_id_lists.items():
            snap[f"armA_vs_{prop}"] = min_pair_distance(model, data, arm_a_ids, prop_ids)
            snap[f"armB_vs_{prop}"] = min_pair_distance(model, data, arm_b_ids, prop_ids)
        engaged = {k: v < COLLISION_DETECTION_DISTANCE_M for k, v in snap.items()}
        return {"min_distance_m": snap, "engaged": engaged}

    report["home_collision_snapshot"] = collision_snapshot(config.data)

    # Correction 1's "report the measured discrepancy" requirement: how far
    # is the chosen frame (the fixed jaw BODY) from the true pinch point
    # (ik.py's own target) BEFORE this probe's per-iteration compensation is
    # applied -- i.e. the raw gap `set_pinch_target` has to correct for.
    # Measured once, at "home", for both arms (symmetric by construction of
    # the scene, so both should match).
    discrepancy = {}
    for arm in ARMS:
        fixed_id = _body_id(model, ik.fixed_jaw_body_name(arm))
        moving_id = _body_id(model, ik.moving_jaw_body_name(arm))
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, ik.gripperframe_site_name(arm)
        )
        fixed_pos = np.array(config.data.xpos[fixed_id], dtype=np.float64, copy=True)
        pinch = pinch_point(config.data, fixed_id, moving_id)
        site_pos = np.array(config.data.site_xpos[site_id], dtype=np.float64, copy=True)
        discrepancy[arm] = {
            "fixed_jaw_body_to_pinch_point_m": float(np.linalg.norm(fixed_pos - pinch)),
            "gripperframe_site_to_pinch_point_m": float(np.linalg.norm(site_pos - pinch)),
        }
    report["pinch_point_discrepancy_at_home"] = discrepancy
    print("Pinch-point discrepancy at 'home' (this probe's chosen frame vs. the")
    print("gripperframe site ADR-025 already rejected):")
    print(json.dumps(discrepancy, indent=2))

    # -------------------------------------------------------------
    # TEST 1 -- single arm, known-reachable: arm A to the fork's CURRENT
    # position (read from the compiled model's own keyframe-reset state,
    # never hardcoded) plus the same hover offset `pick(A, fork)` itself
    # uses (GRASP_POINT_OFFSET_M["fork"] + CLEARANCE_HEIGHT_M above it).
    # -------------------------------------------------------------
    config.update_from_keyframe("home")
    fork_pos = np.array(config.data.xpos[_body_id(model, "fork")], dtype=np.float64, copy=True)
    fork_grasp_point = fork_pos + GRASP_POINT_OFFSET_M["fork"]
    fork_hover_target = fork_grasp_point + np.array([0.0, 0.0, CLEARANCE_HEIGHT_M])
    print(f"TEST 1 target (arm A, fork hover): {fork_hover_target.tolist()}")

    test1 = solve(model, config, {"A": fork_hover_target}, limits, posture_task)
    test1["collision_snapshot_after"] = collision_snapshot(config.data)
    test1["final_qpos_arm_A"] = [
        float(config.data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]])
        for n in ik.arm_joint_names("A")
    ]
    report["test1_single_arm_fork_hover"] = test1
    print(json.dumps(test1, indent=2))

    # -------------------------------------------------------------
    # TEST 2 -- simultaneous bimanual, well separated.
    # -------------------------------------------------------------
    config.update_from_keyframe("home")
    targets2 = {"A": np.array([0.0, 0.10, 0.45]), "B": np.array([0.0, -0.10, 0.45])}
    test2 = solve(model, config, targets2, limits, posture_task)
    test2["collision_snapshot_after"] = collision_snapshot(config.data)
    report["test2_bimanual_separated"] = test2
    print(json.dumps(test2, indent=2))

    # -------------------------------------------------------------
    # TEST 3 -- handoff scenario: both arms to the SAME point,
    # HANDOFF_POSITION_XYZ (ADR-036), close to the brief's (0,0,0.43).
    # -------------------------------------------------------------
    config.update_from_keyframe("home")
    handoff_target = np.array(HANDOFF_POSITION_XYZ, dtype=np.float64)
    targets3 = {"A": handoff_target, "B": handoff_target}
    test3 = solve(model, config, targets3, limits, posture_task)
    test3["collision_snapshot_after"] = collision_snapshot(config.data)
    pinch_a = np.array(test3["A"]["achieved_pinch_point"])
    pinch_b = np.array(test3["B"]["achieved_pinch_point"])
    test3["final_cross_arm_pinch_distance_m"] = float(np.linalg.norm(pinch_a - pinch_b))
    report["test3_handoff_same_point"] = test3
    print(json.dumps(test3, indent=2))

    print("\n=== FULL REPORT JSON ===")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
