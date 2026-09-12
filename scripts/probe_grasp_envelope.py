"""Diagnostic-only: measure the SO-101 gripper's grasp envelope FUNCTIONALLY,
by closing the real jaw around a test box of known thickness and reading
MuJoCo's own contact/force data -- not by reasoning about mesh geometry.

**Why functional, not geometric (read this before changing the method).**
Two geometric measurement attempts on this same gripper were misleading:
`mj_geomDistance` minimised over all geom pairs returns the hinge-region
overlap (the two jaw bodies' collision meshes are both large convex hulls
whose bounding spheres already overlap near the hinge -- ADR-024 measured
`geom_rbound` up to ~8.4 cm on the fixed side, ~6.35 cm on the moving side),
not the pinch gap; and geoms near the rotation axis barely move as the jaw
swings (the M06a Fix D revert found a fine collision sphere placed exactly
on the hinge axis has an IDENTICAL 0.00618 m gap at both joint limits), so
distance-based reasoning near the axis looks constant even on a gripper that
can otherwise make contact elsewhere. This script never calls
`mj_geomDistance` and never reasons about which mesh vertex is "the tip" --
it inserts a real, physically-simulated object and watches whether the real
collision engine reports two-sided contact and how much normal force
results, exactly like a mechanical caliper.

**Method.**
1. Read the COMMITTED dual-arm scene (`TableSettingEnv`, read-only -- no
   scene file is written here) to measure, at arm A's "home" pose with the
   gripper fully OPEN, the world-frame positions of the two jaw collision
   geoms that actually participate in contact post Fix-D-revert (matched by
   their upstream mesh name, `JAW_COLLISION_MESHES`, the same set
   `scripts/gen_dual_scene.py` uses). This gives an "intended pinch point"
   (their midpoint) and an "intended pinch axis" (the unit vector from the
   fixed jaw geom to the moving jaw geom) for THIS specific arm/pose --
   calibration only, not a claim about where contact must occur.
2. Build a THROWAWAY, single-arm MuJoCo model (arm A's own body tree,
   unprefixed, copied byte-faithfully from the same upstream
   `scenes/so101/so101_new_calib.xml` ADR-016 requires, placed at the exact
   same base pose the committed scene uses for arm A) plus one free-jointed
   test box, centered at the calibration midpoint above with its thickness
   axis aligned to the calibration pinch axis. Nothing is written to
   `scenes/so101/` or to `src/bimanual/sim/assets/`; the scratch XML lives
   only in a temp file for the duration of one measurement and is deleted
   immediately after.
3. For each thickness T in a caliper sweep (2 mm .. 60 mm, 2 mm steps):
   compile a FRESH model with a box of that thickness (fresh compile, not a
   post-hoc `geom_size` mutation, because mutating `geom_size` after compile
   would leave `geom_rbound`/mass/inertia stale and could silently miss
   broadphase contacts for larger boxes), command the gripper OPEN and let
   it settle (catches a box too big to be accepted without displacement),
   then command the gripper CLOSED and step until settled, then read
   `data.contact`/`mj_contactForce` for genuine two-jaw contact and the
   total normal force, plus the box's net displacement from where it was
   inserted (a proxy for "was it pushed away").

Runs only on bm-ptl (ADR-020): imports `mujoco` via `bimanual.sim.env`.

This script makes NO scene, control or grasp-offset changes. It is read-only
against the committed scene and writes only to a temp file it deletes
itself. See `docs/hardware/grasp-envelope.md` for the measured results.
"""

from __future__ import annotations

import argparse
import dataclasses
import pathlib
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
UPSTREAM = REPO_ROOT / "scenes" / "so101" / "so101_new_calib.xml"
MESHDIR_ABS = (REPO_ROOT / "scenes" / "so101" / "assets").as_posix()

# Same set `scripts/gen_dual_scene.py` uses to identify the two jaw
# collision geoms by their upstream mesh reference (stable across renaming).
JAW_COLLISION_MESHES = {
    "wrist_roll_follower_so101_v1": "fixed",
    "moving_jaw_so101_v1": "moving",
}

# Same HOME_* values `scripts/gen_dual_scene.py` bakes into the committed
# scene's "home" keyframe (ADR-026) -- copied here as plain numbers (not
# imported from that module) so this diagnostic has no import-time coupling
# to the generator's internals, only to the same, already-ratified constants.
HOME_SHOULDER_PAN = 0.0
HOME_SHOULDER_LIFT = -1.2
HOME_ELBOW_FLEX = -1.6
HOME_WRIST_FLEX = 0.0
HOME_WRIST_ROLL = 0.0

# The same base pose `scripts/gen_dual_scene.py` gives arm A, so world-frame
# calibration numbers measured against the COMMITTED scene transfer directly
# to this throwaway single-arm model without any coordinate transform.
ARM_A_POS = "0 -0.25 0.35"
ARM_A_QUAT = "0.7071068 0 0 0.7071068"

# ---- caliper sweep parameters -------------------------------------------
T_MIN_M = 0.002
T_MAX_M = 0.060
T_STEP_M = 0.002
# The two non-thickness box half-sizes. Originally tried at 0.020 m (a
# generous 4x4 cm face); this session's own dynamic calibration runs found
# that footprint intersected the jaws' housing geometry from directions
# UNRELATED to the thickness axis (see `_measure_pinch_calibration`'s
# docstring: both jaw collision geoms overlap each other by more than a
# centimetre at every joint angle, being non-convex parts collapsed to
# single convex hulls), confounding the thickness measurement with an
# oversized cross-section. 0.0075 m (1.5 cm total width/height) is closer
# to this scene's own smallest real props (`fork_handle`/`spoon_handle`
# capsule radius 0.004 m, i.e. 0.008 m across) while still presenting more
# than a knife-edge.
BOX_HALF_WIDTH_M = 0.0075
BOX_HALF_HEIGHT_M = 0.0075
BOX_DENSITY = 500.0  # kg/m^3, light wood/plastic-ish shim, not load-bearing

OPEN_SETTLE_STEPS = 300
CLOSE_STEPS = 700
FORCE_AVERAGE_WINDOW = 100      # average the last N steps of CLOSE_STEPS
DISPLACEMENT_PUSHED_AWAY_M = 0.03  # box centre moved this far -> "pushed away"
FORCE_EPS_N = 1e-3               # below this, treat as "no real contact"


@dataclasses.dataclass
class SweepRow:
    thickness_m: float
    open_settle_max_disp_m: float
    open_settle_max_force_n: float
    close_both_jaw_contact_frac: float  # fraction of the averaging window with BOTH jaws in contact
    close_avg_normal_force_n: float
    close_final_displacement_m: float
    pushed_away: bool
    graspable: bool


def _find_collision_geom(model, body_id: int, mesh_name: str) -> int:
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid]) != body_id:
            continue
        if int(model.geom_contype[gid]) != 1:
            continue
        if int(model.geom_type[gid]) != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = int(model.geom_dataid[gid])
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id) == mesh_name:
            return gid
    raise RuntimeError(
        f"no contype=1 mesh={mesh_name!r} collision geom found on body {body_id} -- "
        "has the jaw collision geometry changed since this script was written?"
    )


def _measure_pinch_calibration() -> tuple[np.ndarray, np.ndarray, float]:
    """Read the COMMITTED scene (read-only) to locate a starting insertion
    point/axis for the dynamic caliper sweep below. CALIBRATION ONLY -- this
    function's output seeds where the test box starts; it is never itself
    treated as a pass/fail measurement (that is entirely `_run_one_thickness`,
    which uses real contacts/forces from a real physics step).

    **What was tried and rejected, in this exact session, before this
    method (reported here, not silently discarded, per the task's own
    warning about misleading geometric shortcuts):**
    1. The midpoint of the two jaw COLLISION GEOMS' `geom_xpos` at the fully
       OPEN angle. A dynamic test at this point (and a grid of nearby
       offsets) NEVER produced two-jaw contact after closing -- every
       candidate was either already in heavy contact at OPEN or got
       violently ejected once CLOSE was commanded (this session's own
       measured `open_settle_max_disp_m`/`close_final_displacement_m`, up to
       0.7 m, for a 10 mm test box).
    2. The midpoint of the two jaw BODIES' `xpos` (`ik.py`'s own pinch-point
       convention). Same outcome.
    3. A systematic grid search (offsets along the fixed->moving axis and
       two perpendiculars, +/-4 cm, 125 points) for a location that is
       contact-free at OPEN but touched by BOTH jaw geoms at CLOSED (a
       static forward-kinematics check, no physics stepping): ZERO of 125
       points qualified.
    4. Checking whether the fixed and moving jaw COLLISION geoms ever
       register a MUTUAL contact against each other (no test box) while
       sweeping the joint from OPEN to CLOSED: NEVER -- MuJoCo excludes
       contact generation between directly connected parent/child bodies
       (the fixed jaw is the moving jaw's parent) regardless of geometric
       overlap, so this check cannot reveal a "where they meet" location
       either.
    5. `mujoco.mj_geomDistance` between the SAME NAMED PAIR (fixed vs
       moving jaw collision geom; NOT minimised over all geom pairs in the
       model, which is the specific pitfall this task warned about) at 11
       angles spanning the joint's full range. Finding, measured not
       assumed: the two geoms' CONVEX HULLS overlap (`dist` negative) at
       EVERY angle in the range, from -0.0139 m at the LEAST-overlapping
       angle (qpos ~= +0.59 rad, roughly 42% toward closed from OPEN) to
       -0.0377 m at full closure. This is a plausible, mechanically
       complete explanation for findings 1-3 above: `<geom type="mesh">`
       collision defaults to a single CONVEX HULL per geom (no convex
       decomposition is declared anywhere in this asset), and both jaw
       parts are complex, non-convex housing shapes -- their hulls fill in
       concavities and "balloon" out well past the true solid material, so
       they overlap by more than a centimetre at every joint angle even
       though the real printed/machined parts may never truly touch. A
       third object (the test box) placed anywhere in that perpetually
       overlapping hull region is read by the solver as embedded in solid
       material on both sides and is forcibly ejected -- exactly what was
       observed, and exactly why "two-jaw contact never sustains" is a
       different, and more informative, finding than "the arm is simply
       out of reach".

    This function uses METHOD 5's least-overlapping angle's nearest-point
    pair (from `mj_geomDistance`'s `fromto` output, which reports actual
    nearest points on the two hulls, not a hinge-axis-locked or otherwise
    degenerate point) purely as the seed location for the dynamic sweep --
    the best available calibration, reported with its own limitation
    (hull-based, not true-mesh-based) rather than presented as exact.

    Returns (midpoint_xyz, unit_axis_fixed_to_moving, min_overlap_m).
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model, data = env.model, env.data
    arm = "A"

    fixed_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.fixed_jaw_body_name(arm))
    moving_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.moving_jaw_body_name(arm))
    fixed_gid = _find_collision_geom(model, fixed_body, "wrist_roll_follower_so101_v1")
    moving_gid = _find_collision_geom(model, moving_body, "moving_jaw_so101_v1")

    gripper_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ik.gripper_joint_name(arm))
    qadr = int(model.jnt_qposadr[gripper_jid])
    lo, hi = model.jnt_range[gripper_jid]  # lo = CLOSED, hi = OPEN (ik.py / skills_scripted.py convention)

    best_dist = None
    best_mid = None
    best_axis = None
    print("calibration (committed scene, read-only): arm A, mj_geomDistance(fixed, moving) "
          "swept across the full gripper joint range (11 angles):")
    for angle in np.linspace(hi, lo, 11):
        data.qpos[qadr] = angle
        mujoco.mj_forward(model, data)
        fromto = np.zeros(6)
        dist = mujoco.mj_geomDistance(model, data, fixed_gid, moving_gid, 10.0, fromto)
        p_fixed = np.array(fromto[:3], dtype=np.float64, copy=True)
        p_moving = np.array(fromto[3:], dtype=np.float64, copy=True)
        print(f"  qpos={angle:+.4f} dist={dist:.5f} p_fixed={p_fixed} p_moving={p_moving}")
        if best_dist is None or dist > best_dist:  # least-negative = least overlap
            best_dist = dist
            gap_vec = p_moving - p_fixed
            norm = np.linalg.norm(gap_vec)
            best_axis = gap_vec / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])
            best_mid = 0.5 * (p_fixed + p_moving)

    print(
        f"  -> least-overlap angle used for calibration: dist={best_dist:.5f} m "
        f"(negative = hulls overlap at every angle in range, see docstring), "
        f"axis={best_axis}, midpoint={best_mid}\n"
    )

    env.close()
    return best_mid, best_axis, float(best_dist)


def _orthonormal_basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors orthogonal to `axis` (and to each other), used as
    the box's other two local axes. Picks whichever of world-z/world-x is
    less parallel to `axis` as the Gram-Schmidt helper, so this never
    degenerates regardless of the calibrated axis's own direction.
    """
    helper = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, helper))) > 0.9:
        helper = np.array([1.0, 0.0, 0.0])
    b1 = np.cross(axis, helper)
    b1 = b1 / np.linalg.norm(b1)
    b2 = np.cross(axis, b1)
    b2 = b2 / np.linalg.norm(b2)
    return b1, b2


def _rotmat_to_quat(rotmat_columns: np.ndarray) -> str:
    """`rotmat_columns` is a 3x3 matrix whose COLUMNS are the body's local
    x/y/z axes expressed in world coordinates. Returns a MuJoCo-formatted
    "w x y z" quaternion string.
    """
    r = np.asarray(rotmat_columns, dtype=np.float64).reshape(9)
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, r)
    return "%.8f %.8f %.8f %.8f" % tuple(quat)


def _build_probe_xml(
    thickness_m: float,
    box_pos: np.ndarray,
    box_quat: str,
) -> str:
    """Assemble the throwaway single-arm + test-box MJCF as a string.

    Copies arm A's body tree and its gripper actuator from the SAME
    upstream file `scripts/gen_dual_scene.py` copies (ADR-016: unmodified
    upstream kinematics), unprefixed (only one arm here, no namespacing
    collision risk -- ADR-021's <include>-can't-namespace problem is what
    forces gen_dual_scene.py to prefix two copies; one copy needs no
    prefix). No positioning-joint actuators are added: with gravity
    disabled (see below) and zero initial velocity, an unactuated hinge
    joint with only damping/armature and no applied force stays exactly at
    its initial qpos, which is exactly what "hold the arm rigidly at its
    home pose while only the gripper moves" requires.
    """
    tree = ET.parse(UPSTREAM)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    base_body = worldbody.find("body")
    assert base_body.get("name") == "base"
    asset = root.find("asset")
    actuator = root.find("actuator")

    arm_body = ET.tostring(base_body, encoding="unicode").rstrip("\n")

    gripper_actuator_elem = None
    for act in actuator.findall("position"):
        if act.get("joint") == "gripper":
            gripper_actuator_elem = act
            break
    assert gripper_actuator_elem is not None, "gripper actuator not found in upstream asset"
    gripper_actuator_xml = ET.tostring(gripper_actuator_elem, encoding="unicode").strip()

    meshes_xml = "\n    ".join(
        ET.tostring(m, encoding="unicode").strip() for m in asset.findall("mesh")
    )
    materials_xml = "\n    ".join(
        ET.tostring(m, encoding="unicode").strip() for m in asset.findall("material")
    )

    box_pos_str = "%.6f %.6f %.6f" % tuple(box_pos)
    half_t = thickness_m / 2.0

    # NOTE: gravity is disabled ("0 0 0"). This is a caliper measurement --
    # "can the jaws physically close on and hold a slab of thickness T at
    # the pinch point", not a full pick-and-lift task -- so nothing needs
    # to support the box's own weight against gravity for the measurement
    # to be meaningful; disabling it isolates the pinch-force question from
    # an unrelated "did it also fall" confound. This is the throwaway
    # model's own <option>, not a change to the committed scene's physics.
    xml_text = f"""<?xml version="1.0"?>
<mujoco model="grasp_envelope_probe">
  <compiler angle="radian" meshdir="{MESHDIR_ABS}" autolimits="true"/>
  <option gravity="0 0 0"/>

  <default>
    <default class="so101_new_calib">
      <joint damping="1" frictionloss="0.1" armature="0.005"/>
      <position kp="50"/>
      <default class="visual">
        <geom type="mesh" contype="0" conaffinity="0" group="2"/>
      </default>
      <default class="collision">
        <geom group="3"/>
      </default>
    </default>
  </default>
  <default>
    <default class="sts3215">
      <geom contype="0" conaffinity="0"/>
      <joint damping="0.60" frictionloss="0.052" armature="0.028"/>
      <position kp="998.22" kv="2.731" forcerange="-2.94 2.94"/>
    </default>
    <default class="backlash">
      <joint damping="0.01" frictionloss="0" armature="0.01" limited="true" range="-0.008726646259971648 0.008726646259971648"/>
    </default>
  </default>

  <asset>
    {meshes_xml}
    {materials_xml}
    <material name="probe_box_material" rgba="0.9 0.2 0.2 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <body name="arm" pos="{ARM_A_POS}" quat="{ARM_A_QUAT}">
      {arm_body}
    </body>
    <body name="probe_box" pos="{box_pos_str}" quat="{box_quat}">
      <freejoint/>
      <geom name="probe_box_geom" type="box" size="{half_t:.6f} {BOX_HALF_WIDTH_M:.6f} {BOX_HALF_HEIGHT_M:.6f}" density="{BOX_DENSITY}" material="probe_box_material" friction="0.9 0.005 0.0001"/>
    </body>
  </worldbody>

  <actuator>
    {gripper_actuator_xml}
  </actuator>
</mujoco>
"""
    return xml_text


def _run_one_thickness(thickness_m: float, midpoint: np.ndarray, box_quat: str) -> SweepRow:
    xml_text = _build_probe_xml(thickness_m, midpoint, box_quat)

    with tempfile.TemporaryDirectory() as tmpdir:
        scratch_path = pathlib.Path(tmpdir) / "grasp_envelope_probe.xml"
        scratch_path.write_text(xml_text)
        model = mujoco.MjModel.from_xml_path(str(scratch_path))
    # scratch_path is deleted with the TemporaryDirectory context -- nothing
    # written by this script survives past one _run_one_thickness() call.

    data = mujoco.MjData(model)

    # Fix the arm at its home pose (see _build_probe_xml's docstring: no
    # positioning actuators exist, so this initial qpos is never disturbed).
    for suffix, value in (
        ("shoulder_pan", HOME_SHOULDER_PAN),
        ("shoulder_lift", HOME_SHOULDER_LIFT),
        ("elbow_flex", HOME_ELBOW_FLEX),
        ("wrist_flex", HOME_WRIST_FLEX),
        ("wrist_roll", HOME_WRIST_ROLL),
    ):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, suffix)
        data.qpos[int(model.jnt_qposadr[jid])] = value

    gripper_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper")
    gripper_qadr = int(model.jnt_qposadr[gripper_jid])
    lo, hi = model.jnt_range[gripper_jid]
    data.qpos[gripper_qadr] = hi  # start fully OPEN, matching the calibration pose
    mujoco.mj_forward(model, data)

    gripper_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")

    box_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "probe_box")
    box_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "probe_box_geom")
    fixed_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    moving_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    fixed_geom_id = None
    moving_geom_id = None
    for gid in range(model.ngeom):
        if int(model.geom_contype[gid]) != 1 or int(model.geom_type[gid]) != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, int(model.geom_dataid[gid]))
        if int(model.geom_bodyid[gid]) == fixed_body_id and mesh_name == "wrist_roll_follower_so101_v1":
            fixed_geom_id = gid
        if int(model.geom_bodyid[gid]) == moving_body_id and mesh_name == "moving_jaw_so101_v1":
            moving_geom_id = gid
    assert fixed_geom_id is not None and moving_geom_id is not None

    initial_box_pos = np.array(data.xpos[box_body_id], dtype=np.float64, copy=True)

    def _contacts_and_force():
        """Returns (touches_fixed, touches_moving, total_normal_force_n)
        for the CURRENT step's contact list, restricted to contacts between
        `box_geom_id` and one of the two jaw collision geoms.
        """
        touches_fixed = False
        touches_moving = False
        total_force = 0.0
        force_buf = np.zeros(6, dtype=np.float64)
        for i in range(data.ncon):
            con = data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            other = None
            if g1 == box_geom_id:
                other = g2
            elif g2 == box_geom_id:
                other = g1
            else:
                continue
            if other == fixed_geom_id:
                touches_fixed = True
            elif other == moving_geom_id:
                touches_moving = True
            else:
                continue
            mujoco.mj_contactForce(model, data, i, force_buf)
            total_force += abs(float(force_buf[0]))  # index 0: contact-normal force
        return touches_fixed, touches_moving, total_force

    # Phase 1: hold OPEN, let any initial-placement interpenetration resolve.
    # This is what catches "T is bigger than the jaws can even open around":
    # the box gets shoved away here, before CLOSE is ever commanded.
    open_max_disp = 0.0
    open_max_force = 0.0
    data.ctrl[gripper_aid] = hi
    for _ in range(OPEN_SETTLE_STEPS):
        mujoco.mj_step(model, data)
        disp = float(np.linalg.norm(data.xpos[box_body_id] - initial_box_pos))
        open_max_disp = max(open_max_disp, disp)
        _, _, force = _contacts_and_force()
        open_max_force = max(open_max_force, force)

    # Phase 2: command CLOSED, step, and log the final averaging window.
    data.ctrl[gripper_aid] = lo
    both_jaw_hits = 0
    force_samples = []
    for step in range(CLOSE_STEPS):
        mujoco.mj_step(model, data)
        if step >= CLOSE_STEPS - FORCE_AVERAGE_WINDOW:
            touches_fixed, touches_moving, force = _contacts_and_force()
            if touches_fixed and touches_moving:
                both_jaw_hits += 1
            force_samples.append(force)

    both_jaw_frac = both_jaw_hits / float(FORCE_AVERAGE_WINDOW)
    avg_force = float(np.mean(force_samples)) if force_samples else 0.0
    final_disp = float(np.linalg.norm(data.xpos[box_body_id] - initial_box_pos))

    pushed_away = (open_max_disp > DISPLACEMENT_PUSHED_AWAY_M) or (
        final_disp > DISPLACEMENT_PUSHED_AWAY_M
    )
    # "Graspable" per the task's own definition: sustained two-jaw contact
    # (most of the averaging window, not one lucky step), real force (not
    # solver noise), and the box did not get shoved away in the process.
    graspable = (both_jaw_frac >= 0.8) and (avg_force > FORCE_EPS_N) and not pushed_away

    return SweepRow(
        thickness_m=thickness_m,
        open_settle_max_disp_m=open_max_disp,
        open_settle_max_force_n=open_max_force,
        close_both_jaw_contact_frac=both_jaw_frac,
        close_avg_normal_force_n=avg_force,
        close_final_displacement_m=final_disp,
        pushed_away=pushed_away,
        graspable=graspable,
    )


_GEOM_TYPE_NAMES = {
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_ELLIPSOID): "ellipsoid",
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
}

# (prop body name, [geom names on that body]) -- read from the COMPILED
# model below (geom_size/geom_pos/body_mass), not eyeballed from
# scripts/gen_dual_scene.py's XML text, per this task's explicit
# instruction. Geom NAMES are declared in gen_dual_scene.py's TEMPLATE
# (see e.g. `<geom name="plate_foot" .../>`) -- only the geom *names* are
# read from source (needed to look them up at all); every dimension
# printed comes from `model.geom_size`/`model.geom_pos`/`model.body_mass`.
PROP_GEOMS = {
    "plate": ["plate_foot", "plate_dish"],
    "mug": ["mug_body", "mug_handle"],
    "fork": ["fork_handle", "fork_head"],
    "spoon": ["spoon_handle", "spoon_bowl"],
    "water_bottle": ["water_bottle_body", "water_bottle_cap"],
}


def _measure_prop_dimensions() -> None:
    """Print each prop's geometry as read from the COMPILED dual-arm scene
    (`model.geom_size`, `model.geom_pos`, `model.body_mass`) -- answers
    task section (b), "measured from the compiled model, not read off the
    XML by eye".

    MuJoCo's `geom_size` meaning depends on `geom_type`:
      - cylinder/capsule: size = (radius, half-length)
      - box: size = (half-x, half-y, half-z)
      - ellipsoid: size = (half-x, half-y, half-z) (the three semi-axes)
      - sphere: size = (radius,)
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    model = env.model

    print("Prop grip-dimension inventory (measured from the compiled model):")
    for body_name, geom_names in PROP_GEOMS.items():
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        total_mass = float(model.body_mass[body_id])
        print(f"\n  {body_name}: total body_mass={total_mass:.4f} kg")
        for gname in geom_names:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            if gid == -1:
                print(f"    geom {gname!r} not found")
                continue
            gtype = _GEOM_TYPE_NAMES.get(int(model.geom_type[gid]), str(int(model.geom_type[gid])))
            size = np.array(model.geom_size[gid], dtype=np.float64, copy=True)
            local_pos = np.array(model.geom_pos[gid], dtype=np.float64, copy=True)
            print(f"    geom={gname!r} type={gtype} size(m)={size} local_pos(m)={local_pos}")

    # Derived, human-readable dimensions, computed from the same arrays
    # printed above (shown alongside them so the derivation is checkable).
    def _gsize(name):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        return np.array(model.geom_size[gid], dtype=np.float64, copy=True)

    def _gpos(name):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        return np.array(model.geom_pos[gid], dtype=np.float64, copy=True)

    foot_r, foot_hh = _gsize("plate_foot")[0], _gsize("plate_foot")[1]
    dish_r, dish_hh = _gsize("plate_dish")[0], _gsize("plate_dish")[1]
    dish_local_z = _gpos("plate_dish")[2]
    # The dish rests directly ON the foot's top surface (touches it, zero
    # vertical gap there by construction: dish_local_z == foot_hh + dish_hh,
    # confirmed below). The gap a gripper jaw could slide INTO is instead
    # the OPEN AIR radially outside the foot's own footprint (foot_r <
    # radius < dish_r), from the table surface (z=0, local) up to the
    # dish's bottom face (z = dish_local_z - dish_hh, local) -- i.e. the
    # total foot height, 2*foot_hh.
    dish_touches_foot_top = abs(dish_local_z - (foot_hh + dish_hh)) < 1e-6
    gap_under_overhang = 2 * foot_hh
    print(
        f"\n  DERIVED plate: dish diameter={2*dish_r:.4f} m, dish thickness (at rim)={2*dish_hh:.4f} m, "
        f"foot diameter={2*foot_r:.4f} m, dish rests directly on foot top (zero gap there)="
        f"{dish_touches_foot_top}, open-air gap under the OVERHANGING rim (radially outside the foot, "
        f"i.e. the total foot height)={gap_under_overhang:.4f} m"
    )

    mug_r, mug_hh = _gsize("mug_body")[0], _gsize("mug_body")[1]
    print(
        f"  DERIVED mug: outer diameter={2*mug_r:.4f} m, height={2*mug_hh:.4f} m "
        f"(wall thickness not separately modelled -- mug_body is a single solid cylinder, "
        f"no hollow interior geom)"
    )

    fork_handle_r = _gsize("fork_handle")[0]
    fork_head_size = _gsize("fork_head")
    print(
        f"  DERIVED fork: handle width=thickness=diameter={2*fork_handle_r:.4f} m (round capsule), "
        f"head size={2*fork_head_size[0]:.4f} x {2*fork_head_size[1]:.4f} x {2*fork_head_size[2]:.4f} m (LxWxH)"
    )

    spoon_handle_r = _gsize("spoon_handle")[0]
    spoon_bowl_size = _gsize("spoon_bowl")
    print(
        f"  DERIVED spoon: handle width=thickness=diameter={2*spoon_handle_r:.4f} m (round capsule), "
        f"bowl semi-axes={spoon_bowl_size} m (ellipsoid)"
    )

    bottle_r, bottle_hh = _gsize("water_bottle_body")[0], _gsize("water_bottle_body")[1]
    cap_r, cap_hh = _gsize("water_bottle_cap")[0], _gsize("water_bottle_cap")[1]
    print(
        f"  DERIVED water_bottle: body diameter={2*bottle_r:.4f} m, body height={2*bottle_hh:.4f} m, "
        f"total height incl. cap~={2*bottle_hh + 2*cap_hh:.4f} m"
    )

    env.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--t-min", type=float, default=T_MIN_M)
    parser.add_argument("--t-max", type=float, default=T_MAX_M)
    parser.add_argument("--t-step", type=float, default=T_STEP_M)
    parser.add_argument(
        "--skip-sweep",
        action="store_true",
        help="print only the prop-dimension inventory (task section (b)), skip the caliper sweep",
    )
    args = parser.parse_args(argv)

    _measure_prop_dimensions()
    print()
    if args.skip_sweep:
        return 0

    midpoint, axis, min_overlap = _measure_pinch_calibration()
    b1, b2 = _orthonormal_basis(axis)
    # Columns = box local x (thickness axis), y, z in world coordinates.
    rotmat_columns = np.column_stack([axis, b1, b2])
    box_quat = _rotmat_to_quat(rotmat_columns)

    print(f"{'T (mm)':>8} {'open_max_disp':>14} {'open_max_F':>11} "
          f"{'both_jaw_frac':>14} {'avg_F(N)':>9} {'final_disp':>11} {'pushed':>7} {'GRASPABLE':>10}")

    rows: list[SweepRow] = []
    t = args.t_min
    while t <= args.t_max + 1e-9:
        row = _run_one_thickness(round(t, 6), midpoint, box_quat)
        rows.append(row)
        print(
            f"{row.thickness_m * 1000:8.1f} {row.open_settle_max_disp_m:14.5f} "
            f"{row.open_settle_max_force_n:11.4f} {row.close_both_jaw_contact_frac:14.2f} "
            f"{row.close_avg_normal_force_n:9.4f} {row.close_final_displacement_m:11.5f} "
            f"{str(row.pushed_away):>7} {str(row.graspable):>10}"
        )
        t += args.t_step

    graspable_rows = [r for r in rows if r.graspable]
    print()
    if not graspable_rows:
        print(
            "RESULT: NO thickness in the swept range achieved sustained two-jaw "
            "contact without the box being pushed away. This gripper, as currently "
            "modelled, could not be measured to hold ANY test-box thickness at this "
            "pinch configuration."
        )
    else:
        min_t = min(r.thickness_m for r in graspable_rows)
        max_t = max(r.thickness_m for r in graspable_rows)
        print(f"RESULT: minimum graspable thickness ~= {min_t * 1000:.1f} mm")
        print(f"RESULT: maximum graspable thickness ~= {max_t * 1000:.1f} mm")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
