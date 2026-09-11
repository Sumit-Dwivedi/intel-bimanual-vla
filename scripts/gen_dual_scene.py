"""Generate src/bimanual/sim/assets/so101_dual_table.xml (M02).

Why this script exists (ADR-021): MuJoCo's <include> element does not support
namespacing. Every body, joint, site and actuator name in an MJCF file must be
globally unique across the whole compiled model, and <include> is a plain
textual splice with no prefix mechanism. Trying to <include> the same SO-101
arm file twice therefore fails to compile with duplicate-name errors on every
joint, body, site and actuator. This was verified empirically on bm-ptl before
this script was written (see ARCHITECTURE.md ADR-021 for the verbatim error).

The alternative -- hand-copying and manually renaming ~120 lines of nested
<body>/<joint>/<site> XML twice, by hand, in a text editor -- is exactly the
kind of transcription work a machine should do instead of a human, so this
script does it: it parses the UNMODIFIED upstream
scenes/so101/so101_new_calib.xml (never edited, per ADR-016), deep-copies its
single top-level <body name="base"> twice, prefixes every body/joint/site name
with "armA_" / "armB_", repositions each copy's base at the opposite long edge
of the table, and appends a matching pair of renamed <position> actuators. The
<asset> mesh/material definitions are declared ONCE and shared by both arm
copies, because mesh geometry is not per-instance data -- only body transforms,
joints and actuators are.

Run from the repo root (paths below are repo-root-relative):
    python scripts/gen_dual_scene.py

This is a code-generation utility, not a runtime module: nothing else in
`bimanual` imports it. Its only job is to keep the dual-arm scene file
reproducible from the single-arm upstream source, rather than hand-maintained
by two people independently drifting apart.
"""

import copy
import math
import pathlib
import xml.etree.ElementTree as ET

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
UPSTREAM = REPO_ROOT / "scenes" / "so101" / "so101_new_calib.xml"
DEST = REPO_ROOT / "src" / "bimanual" / "sim" / "assets" / "so101_dual_table.xml"

# ---- M02(d) front-camera fix -------------------------------------------
# The original front camera (pos="1.15 0 0.75", mode="targetbody" target=
# "table") aimed at the table body's own origin, which sits at z=0 (the
# floor level the table body is anchored at -- see the hand-authored <body
# name="table" pos="0 0 0"> below). That put the camera's aim point far
# below the arms, so its vertical field of view was centered near the
# tabletop and cropped both arms above the gripper. Measured on bm-ptl via
# a probe script (scripts/probe_arm_extent.py-equivalent one-off, see
# DECISIONS.md M02(d) entry): at the reset pose, the highest arm geometry
# (armA_moving_jaw_so101_v1) reaches z~0.667, roughly 0.32m above the
# 0.35m tabletop surface.
#
# Fix: aim the camera at a point mid-way between the tabletop and a
# comfortable margin above the measured arm height (z=0.5, not z=0), move
# it back and up slightly for a wider working margin, and widen the
# vertical field of view (fovy) so the full arm height stays in frame
# even as the arms move during a task, not only at the rest pose. Because
# MuJoCo's mode="targetbody" only aims at a body's own origin -- it cannot
# aim at an arbitrary point -- we drop that mode and specify the camera's
# orientation directly via `xyaxes`, computed below with a standard
# look-at construction (stdlib `math` only, no numpy, so this generator
# keeps running on the laptop per ADR-020).
FRONT_CAM_POS = (1.55, 0.0, 0.85)
FRONT_CAM_TARGET = (0.0, 0.0, 0.50)
FRONT_CAM_FOVY = 55

# ---- M02(e) drawer_view camera -----------------------------------------
# Neither `overhead` (straight down from z=1.3) nor `front` (looking along
# -x from x=1.55) shows the drawer opening -- verified empirically on
# bm-ptl, not assumed. `overhead` looks straight down onto the opaque
# table_top box (table_top geom: box, center z=0.34, half-thickness 0.01,
# top surface z=0.35), which occludes the drawer (housed at z=0.28,
# entirely below the tabletop) from directly above. `front` looks along
# -x, roughly parallel to the drawer's slide axis (0,-1,0) rather than
# across it, so the 3 cm the open drawer protrudes past the table's
# y=-0.25 edge (drawer body world y goes from -0.05 at slide=0 to -0.20 at
# slide=0.15=range max; its front face -- box half-extent 0.08 in y --
# reaches y=-0.20-0.08=-0.28, i.e. 3 cm past the table edge) is not legible
# from that angle either.
#
# Fix: a camera BELOW tabletop height (table surface z=0.35), on the -y
# side past where the open drawer protrudes, angled UP and toward +y so it
# looks across the drawer's slide axis instead of along it.
#
# M02 defect fix (two bugs found on re-render, both confirmed by rendering
# the scene open and closed, not assumed):
#
# 1. This camera originally used mode="targetbody" target="drawer", which
#    makes MuJoCo re-aim the camera at the `drawer` body's own origin every
#    frame -- exactly the point that moves as drawer_slide goes from 0 to
#    0.15. That re-aiming is the bug, not a feature: it keeps the drawer
#    centred in frame while the background (housing, table) swings past
#    behind it, so a viewer reads the drawer opening as the CAMERA moving,
#    not the drawer -- the opposite of what this camera is for. The fix is
#    a FIXED orientation, computed once via the same look_at_xyaxes()
#    helper the front camera above uses, aimed at the drawer's CLOSED rest
#    position (DRAWER_CAM_TARGET, the world position of the `drawer` body
#    at drawer_slide=0 -- see the drawer body's pos in the template below).
#    With a fixed background, the drawer visibly slides toward the camera
#    as it opens instead of the scene appearing to rotate around it.
# 2. The drawer box and its housing shared `drawer_material` (see the
#    template's `<body name="drawer">` below), so a single still frame
#    could not be told apart as open or closed by colour alone -- only a
#    before/after pair was legible. Fixed in the template's <asset> block:
#    the drawer box now gets its own `drawer_box_material` (blue),
#    contrasting with the housing's brown `drawer_material`.
DRAWER_CAM_POS = (0.0, -0.55, 0.15)
# Drawer body world position at drawer_slide=0 (closed): the `drawer`
# body sits at local pos (0,0,0) inside `drawer_housing`, which is placed
# at world (0,-0.05,0.28) -- see the template's <body name="drawer_housing">
# below. Aiming here (rather than at the housing's own origin, which is
# the same point) is what keeps the background fixed while the drawer
# slides toward -y and toward the camera as it opens.
DRAWER_CAM_TARGET = (0.0, -0.05, 0.28)
DRAWER_CAM_FOVY = 50


def _sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(a):
    n = math.sqrt(sum(c * c for c in a))
    return tuple(c / n for c in a)


def look_at_xyaxes(pos, target, world_up=(0.0, 0.0, 1.0)):
    """Compute a MuJoCo <camera xyaxes="..."/> value for a camera at `pos`
    looking at `target`.

    MuJoCo's xyaxes gives the camera's local +x ("right") and local +y
    ("up") axes in world coordinates; the camera looks along its local -z,
    so local z = -forward = cross(right, up) by construction here. This is
    the standard graphics look-at basis construction, done with plain
    tuples so the generator has no numpy dependency.
    """
    forward = _normalize(_sub(target, pos))
    right = _normalize(_cross(forward, world_up))
    up = _cross(right, forward)  # already unit length: right, forward orthonormal
    return right, up


_FRONT_RIGHT, _FRONT_UP = look_at_xyaxes(FRONT_CAM_POS, FRONT_CAM_TARGET)
FRONT_CAM_XYAXES = "%.4f %.4f %.4f %.4f %.4f %.4f" % (_FRONT_RIGHT + _FRONT_UP)
FRONT_CAM_POS_STR = "%.4f %.4f %.4f" % FRONT_CAM_POS

_DRAWER_RIGHT, _DRAWER_UP = look_at_xyaxes(DRAWER_CAM_POS, DRAWER_CAM_TARGET)
DRAWER_CAM_XYAXES = "%.4f %.4f %.4f %.4f %.4f %.4f" % (_DRAWER_RIGHT + _DRAWER_UP)

# ---- geometry / layout constants (see ADR-021 for the reach-derived rationale) ----
TABLE_TOP_Z = 0.35   # table surface height above the floor (m)
ARM_GAP_Y = 0.25     # each arm base offset from y=0 (table centreline, m)
# Two SO-ARM100 arms, ~0.30 m reach each (stated assumption, ADR-021), facing
# each other across the table's short (width) axis. A base gap of 2*0.25=0.50 m
# leaves an approximately 0.10 m wide overlap band (y in [-0.05, 0.05]) reachable
# by both arms -- this is where props are placed.


def rename_recursive(elem, prefix):
    """Prepend `prefix` to every body/joint/site name in this subtree (in place)."""
    if elem.tag in ("body", "joint", "site") and elem.get("name") is not None:
        elem.set("name", prefix + elem.get("name"))
    for child in elem:
        rename_recursive(child, prefix)


def build_arm(base_body, prefix, pos, quat):
    b = copy.deepcopy(base_body)
    rename_recursive(b, prefix)
    b.set("pos", pos)
    b.set("quat", quat)
    return b


def build_actuators(actuator_root, prefix):
    out = []
    for act in actuator_root.findall("position"):
        a = copy.deepcopy(act)
        a.set("name", prefix + a.get("name"))
        a.set("joint", prefix + a.get("joint"))
        out.append(a)
    return out


def find_wrist_body(arm_root, prefix):
    # wrist is nested four <body> levels deep: base -> shoulder -> upper_arm
    # -> lower_arm -> wrist.
    b = arm_root
    for _ in range(4):
        b = b.find("body")
    assert b.get("name") == prefix + "wrist", b.get("name")
    return b


def indent(elem, level=0):
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for child in elem:
            indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = i
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def main():
    tree = ET.parse(UPSTREAM)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    base_body = worldbody.find("body")
    assert base_body.get("name") == "base"
    asset = root.find("asset")
    actuator = root.find("actuator")

    arm_a = build_arm(base_body, "armA_", pos=f"0 -{ARM_GAP_Y} {TABLE_TOP_Z}", quat="0.7071068 0 0 0.7071068")
    arm_b = build_arm(base_body, "armB_", pos=f"0 {ARM_GAP_Y} {TABLE_TOP_Z}", quat="0.7071068 0 0 -0.7071068")
    act_a = build_actuators(actuator, "armA_")
    act_b = build_actuators(actuator, "armB_")

    # Wrist camera: a bare <camera>, no mesh geometry. so101_new_calib_camera.xml
    # (also in scenes/so101/, provenance in scenes/so101/PROVENANCE.md) shows a
    # modeled camera mount, but its two extra mesh files
    # (wrist_camera_mount_so101_v1.stl, wrist_camera_so101_v1.stl) were never
    # fetched in M01 -- only the 13 meshes so101_new_calib.xml itself needs are
    # present under scenes/so101/assets. Fetching new upstream files is out of
    # scope for this module, so this camera has no physical mount modeled.
    wrist_cam_pos = "0 -0.05 0.02"
    wrist_cam_xyaxes = "1 0 0 0 0 1"
    for arm_root, prefix in ((arm_a, "armA_"), (arm_b, "armB_")):
        wrist = find_wrist_body(arm_root, prefix)
        cam = ET.SubElement(wrist, "camera")
        cam.set("name", prefix + "wrist")
        cam.set("pos", wrist_cam_pos)
        cam.set("xyaxes", wrist_cam_xyaxes)

    for e in (arm_a, arm_b):
        indent(e, level=2)
    arm_a_xml = ET.tostring(arm_a, encoding="unicode").rstrip("\n")
    arm_b_xml = ET.tostring(arm_b, encoding="unicode").rstrip("\n")

    actuators_xml = "\n    ".join(
        ET.tostring(a, encoding="unicode").strip() for a in act_a + act_b
    )
    meshes_xml = "\n    ".join(
        ET.tostring(m, encoding="unicode").strip() for m in asset.findall("mesh")
    )
    materials_xml = "\n    ".join(
        ET.tostring(m, encoding="unicode").strip() for m in asset.findall("material")
    )

    out = TEMPLATE.format(
        arm_a=arm_a_xml,
        arm_b=arm_b_xml,
        actuators=actuators_xml,
        meshes=meshes_xml,
        materials=materials_xml,
        front_cam_pos=FRONT_CAM_POS_STR,
        front_cam_xyaxes=FRONT_CAM_XYAXES,
        front_cam_fovy=FRONT_CAM_FOVY,
        drawer_cam_pos="%.4f %.4f %.4f" % DRAWER_CAM_POS,
        drawer_cam_xyaxes=DRAWER_CAM_XYAXES,
        drawer_cam_fovy=DRAWER_CAM_FOVY,
    )
    DEST.parent.mkdir(parents=True, exist_ok=True)
    DEST.write_text(out, newline="\n")
    print(f"wrote {DEST} ({len(out)} bytes)")


TEMPLATE = """<?xml version="1.0"?>
<!--
  so101_dual_table.xml: M02 dual-arm table scene (bimanual VLA project).
  GENERATED by scripts/gen_dual_scene.py: do not hand-edit the arm bodies,
  actuators, meshes or materials sections below; edit the upstream source
  (scenes/so101/so101_new_calib.xml) or this generator instead and re-run it.
  The table, drawer, props and cameras below are hand-authored and are safe
  to edit directly.

  Two unmodified SO-101 arms (ADR-016: kinematics adopted byte-for-byte from
  scenes/so101/so101_new_calib.xml, only renamed and repositioned; see
  ADR-021 for why renaming was necessary and why <include> could not do it)
  mounted on opposite long sides of a table, facing each other over a shared
  working area, plus a drawer (prismatic joint) and five manipulable props
  (plate, mug, fork, spoon, water bottle).

  Geometry assumption (see ADR-021): SO-ARM100 reach is approximately 0.30 m.
  Arm bases are placed 0.50 m apart (0.25 m each side of the table centreline)
  so their reach envelopes overlap in a band roughly 0.10 m wide at the centre
  of the table; this is where props are placed. Table surface height 0.35 m.

  Mesh geometry is NOT duplicated: both arm copies share one asset block,
  which is why "reference scenes/so101, do not copy-edit it" (ADR-016) is
  satisfied by pointing meshdir at the upstream assets directory rather than
  vendoring a second copy of the STL files.
-->
<mujoco model="so101_dual_table">
  <compiler angle="radian" meshdir="../../../../scenes/so101/assets" autolimits="true"/>

  <!-- TODO(M02): offwidth/offheight override. Upstream scenes/so101/scene.xml
       declares 640x480 (confirmed by scripts/probe_render.py on bm-ptl), which is
       too small for the 1280x720 demo video target. This scene declares its own
       framebuffer size instead of editing the upstream file (ADR-016). -->
  <visual>
    <global offwidth="1280" offheight="720" azimuth="140" elevation="-25"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
  </visual>

  <!-- Unmodified defaults from scenes/so101/so101_new_calib.xml (ADR-016): these
       set joint damping/friction/armature and the position-actuator gains for the
       real servos (sts3215). Declared once; both arm copies use childclass. -->
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
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>

    <!-- Furniture / prop materials (ours, not upstream) -->
    <material name="table_material" rgba="0.55 0.40 0.25 1"/>
    <material name="drawer_material" rgba="0.42 0.28 0.18 1"/>
    <!-- drawer_box_material: distinct blue for the drawer box itself (M02
         drawer_view fix, defect 2). It previously shared drawer_material
         (brown) with the static housing, so a single still frame could not
         be read as open or closed -- only a before/after pair was legible.
         Blue, not red, to stay visually distinct from mug_material below
         (0.75 0.15 0.15, red) elsewhere in the same scene. -->
    <material name="drawer_box_material" rgba="0.15 0.35 0.85 1"/>
    <material name="plate_material" rgba="0.92 0.92 0.88 1"/>
    <material name="mug_material" rgba="0.75 0.15 0.15 1"/>
    <material name="mug_handle_material" rgba="0.75 0.15 0.15 1"/>
    <material name="fork_material" rgba="0.72 0.73 0.76 1"/>
    <material name="spoon_material" rgba="0.72 0.73 0.76 1"/>
    <material name="bottle_material" rgba="0.25 0.55 0.85 0.55"/>

    <!-- SO-101 meshes and part materials, referenced by BOTH arm copies below.
         Shared, not duplicated, because mesh geometry is not per-instance data
         (ADR-021). Unmodified from scenes/so101/so101_new_calib.xml. -->
    {meshes}
    {materials}
  </asset>

  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <light pos="0.6 -0.6 1.5" dir="-0.4 0.4 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" pos="0 0 0" type="plane" material="groundplane"/>

    <!-- Table. Long axis = x (0.80 m), short axis = y (0.50 m). Arms mount on
         the two long edges (y = -0.25 and y = +0.25) and face each other
         across the short axis, per the "opposite long sides" layout
         requirement. -->
    <body name="table" pos="0 0 0">
      <geom name="table_top" type="box" size="0.40 0.25 0.01" pos="0 0 0.34" material="table_material"/>
      <geom name="table_leg_fl" type="cylinder" size="0.018 0.165" pos="-0.35 -0.20 0.165" material="table_material"/>
      <geom name="table_leg_fr" type="cylinder" size="0.018 0.165" pos="0.35 -0.20 0.165" material="table_material"/>
      <geom name="table_leg_bl" type="cylinder" size="0.018 0.165" pos="-0.35 0.20 0.165" material="table_material"/>
      <geom name="table_leg_br" type="cylinder" size="0.018 0.165" pos="0.35 0.20 0.165" material="table_material"/>
    </body>

    <!-- Drawer (prismatic joint). A small cabinet tucked under the tabletop
         near arm A's edge (y=-0.25). The drawer body slides out along -y
         (towards arm A) on a slide joint; housing walls are static geoms
         with no joint. -->
    <body name="drawer_housing" pos="0 -0.05 0.28">
      <geom name="drawer_housing_bottom" type="box" size="0.12 0.10 0.005" pos="0 0 -0.045" material="drawer_material"/>
      <geom name="drawer_housing_back" type="box" size="0.12 0.005 0.05" pos="0 0.095 0" material="drawer_material"/>
      <geom name="drawer_housing_left" type="box" size="0.005 0.10 0.05" pos="-0.115 0 0" material="drawer_material"/>
      <geom name="drawer_housing_right" type="box" size="0.005 0.10 0.05" pos="0.115 0 0" material="drawer_material"/>
      <body name="drawer" pos="0 0 0">
        <joint name="drawer_slide" type="slide" axis="0 -1 0" range="0 0.15" damping="5" frictionloss="0.5"/>
        <geom name="drawer_box" type="box" size="0.10 0.08 0.04" mass="0.30" material="drawer_box_material" friction="0.6 0.005 0.0001"/>
      </body>
    </body>

    <!-- Manipulable props (free joints). All placed within the arms'
         overlapping reach band, y in [-0.05, 0.08], resting on the table
         surface (z=0.35). -->
    <body name="plate" pos="-0.15 0.00 0.356">
      <freejoint name="plate_free"/>
      <geom name="plate_geom" type="cylinder" size="0.09 0.006" mass="0.15" material="plate_material" friction="0.9 0.005 0.0001"/>
    </body>

    <body name="mug" pos="0.05 -0.03 0.39">
      <freejoint name="mug_free"/>
      <geom name="mug_body" type="cylinder" size="0.035 0.04" mass="0.22" material="mug_material" friction="0.9 0.005 0.0001"/>
      <geom name="mug_handle" type="capsule" size="0.008" fromto="0.035 0 0.01 0.06 0 -0.01" mass="0.02" material="mug_handle_material" friction="0.9 0.005 0.0001"/>
    </body>

    <body name="fork" pos="-0.05 0.05 0.356">
      <freejoint name="fork_free"/>
      <geom name="fork_handle" type="capsule" size="0.004" fromto="-0.06 0 0 0.03 0 0" mass="0.02" material="fork_material" friction="0.5 0.003 0.0001"/>
      <geom name="fork_head" type="box" size="0.025 0.012 0.003" pos="0.055 0 0" mass="0.01" material="fork_material" friction="0.5 0.003 0.0001"/>
    </body>

    <body name="spoon" pos="0.00 0.08 0.356">
      <freejoint name="spoon_free"/>
      <geom name="spoon_handle" type="capsule" size="0.004" fromto="-0.06 0 0 0.035 0 0" mass="0.018" material="spoon_material" friction="0.5 0.003 0.0001"/>
      <geom name="spoon_bowl" type="ellipsoid" size="0.018 0.012 0.004" pos="0.05 0 0" mass="0.012" material="spoon_material" friction="0.5 0.003 0.0001"/>
    </body>

    <body name="water_bottle" pos="0.22 0.00 0.44">
      <freejoint name="water_bottle_free"/>
      <geom name="water_bottle_body" type="cylinder" size="0.03 0.09" mass="0.30" material="bottle_material" friction="0.7 0.004 0.0001"/>
      <geom name="water_bottle_cap" type="cylinder" size="0.012 0.01" pos="0 0 0.10" mass="0.01" material="bottle_material" friction="0.7 0.004 0.0001"/>
    </body>

    <!-- Arm A (armA_ prefix). Base mounted at the table's -y long edge,
         rotated +90 deg about z so the arm's local +x (forward reach) points
         toward +y, i.e. toward arm B and the shared working area. Kinematics
         below are an unmodified, renamed copy of
         scenes/so101/so101_new_calib.xml's body tree (ADR-016, ADR-021). -->
    {arm_a}

    <!-- Arm B (armB_ prefix). Base mounted at the table's +y long edge,
         rotated -90 deg about z so the arm's local +x points toward -y,
         facing arm A. -->
    {arm_b}

    <!-- Cameras. targetbody mode: MuJoCo computes the look-at orientation
         automatically from camera pos and the target body's position, so no
         manual quat/xyaxes math is needed for these two. -->
    <camera name="overhead" pos="0 0 1.3" mode="targetbody" target="table"/>
    <!-- front: fixed orientation via xyaxes (M02(d) fix), NOT mode="targetbody",
         because targetbody can only aim at a body's own origin (the table
         body sits at z=0) and that aim point was too low, cropping both
         arms above the gripper. See the FRONT_CAM_* constants and
         look_at_xyaxes() above for the derivation: aims at (0,0,0.5),
         roughly the midpoint between the 0.35m tabletop and the measured
         ~0.667m rest-pose arm height, with a widened fovy for headroom as
         arms move during a task. -->
    <camera name="front" pos="{front_cam_pos}" xyaxes="{front_cam_xyaxes}" fovy="{front_cam_fovy}"/>
    <!-- drawer_view: low, angled up, on the -y side past the open drawer's
         protrusion (see DRAWER_CAM_* comment above for the empirical
         reasoning). FIXED orientation via xyaxes (M02 drawer_view fix,
         defect 1), NOT mode="targetbody" -- targetbody re-aimed this
         camera at the moving `drawer` body every frame, which kept the
         drawer centred while the background swung past it, reading as
         camera motion rather than the drawer opening. xyaxes is aimed
         once at the drawer's CLOSED rest position (DRAWER_CAM_TARGET), so
         the background stays fixed and the drawer visibly slides toward
         the camera as drawer_slide goes from 0 to 0.15. -->
    <camera name="drawer_view" pos="{drawer_cam_pos}" xyaxes="{drawer_cam_xyaxes}" fovy="{drawer_cam_fovy}"/>
  </worldbody>

  <actuator>
    {actuators}
  </actuator>

  <equality/>
</mujoco>
"""


if __name__ == "__main__":
    main()
