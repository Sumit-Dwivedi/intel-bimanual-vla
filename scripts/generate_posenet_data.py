"""M10 Phase 1 / Phase 1.5 / Phase 1.5 correction (ADR-041): PoseNet
training-data generation.

This script ONLY generates labelled images. It does not define, train, or run
any model -- that is M10 Phase 2+. PoseNet will eventually replace the
scripted controller's oracle `data.xpos` prop-position reads with inference
from a camera; this script produces the (image, ground-truth-xyz) pairs that
later training will consume.

**M10 Phase 1.5 (ADR-040, supersedes Phase 1's dataset).** Phase 1 rendered
through the `front` camera and measured props occupying only ~10-20 px of a
224x224 frame -- too small to train a useful pose regressor -- and its
rejection sampler's worst-case draw count (5054) had already exceeded the
5000-per-prop cap on the full run. Phase 1.5 fixed both: rendering moved to a
dedicated `posenet_cam` (added to `gen_dual_scene.py`, `front` itself
untouched), the randomization range widened from +-0.15 to +-0.18 m, the
clearance margin eased from 0.015 to 0.008 m, and placement order shuffled
per sample instead of fixed (see `BASE_PLACEMENT_ORDER`'s comment below).
See `docs/hardware/m10-camera-comparison.md` for that pass's measured
before/after. **ADR-040 shipped and its 5000-sample regeneration was in
progress when it was halted by the orchestrator** (`data/posenet/images`
and `labels` deleted mid-run) -- this pass treats that dataset as absent.

**M10 Phase 1.5 correction (ADR-041, supersedes ADR-040's camera pose and
label scope -- ADR-040's mechanism is otherwise kept unchanged).** Three
fixes, all checked by hand before use rather than accepted on the task
brief's say-so (see `gen_dual_scene.py`'s `POSENET_CAM_*` comment block for
the camera arithmetic, and this module's own comments below for the other
two):

1. **Camera moved further overhead**, `pos="0.6 0 1.1"`,
   `xyaxes="0 1 0 -0.7 0 0.7"` -- the brief's proposed `xyaxes` for this
   pass reintroduced the exact sign error ADR-040 already fixed once (view
   direction pointed away from the table); corrected the same way, verified
   by rendering one probe frame before any sample was generated.
2. **Label scope reduced to 3 props.** All five props stay in the rendered
   scene (visual realism / occlusion training signal), but labels are only
   emitted for `TARGET_PROPS = ("fork", "water_bottle", "mug")`. `plate` and
   `spoon` remain in-scene, randomized, and unlabelled -- see
   `DECORATION_PROP_NAMES` below.
3. **Visibility (occlusion) filtering.** A sample is rejected and
   re-drawn if any target prop's occlusion ratio (full-scene visible pixel
   count / that same prop's unoccluded-position pixel count, see
   `compute_visibility_ratios` below) falls below `VISIBILITY_THRESHOLD`.
   This is an OCCLUSION ratio, not a fill-fraction/shape metric -- see that
   function's docstring for why the two are different and which one this
   script implements. The 0.30 threshold value is a **user-supplied
   reference** to Syn4D (https://arxiv.org/pdf/2605.05207); this script
   does not claim to have read that paper beyond the threshold value cited
   to it, and invents no authors or further findings from it.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`, which cannot load on the developer's Windows laptop. Uses the
`mujoco==3.2.7` pin unmodified (`scripts/requirements-bmptl.txt`) -- this
script never upgrades or reinstalls anything, and that pairing with
`openvino==2026.3.1` is load-bearing for the rest of the OpenVINO story.

Four corrections this script embodies (Phase 1 -> Phase 1.5), each
traceable to a real mistake a prior pass would otherwise have made:

1. **No Pillow.** `PIL` is not installed on bm-ptl and this script does not
   `pip install` it (an unrelated install is exactly what silently upgraded
   mujoco during the mink probe, ADR-037). `write_png` below is copied
   verbatim, with attribution, from `scripts/render_handoff_frames.py`'s own
   `write_png` (itself copied from `scripts/probe_render.py`), which is a
   stdlib-only (zlib + hand-rolled PNG chunks) RGB writer already proven to
   handle arbitrary HxWx3 uint8 arrays (1280x720 and 640x480 in that script;
   224x224 here -- verified in the 10-sample smoke run before the full 5000).

2. **No touching `scenes/so101/` or `so101_dual_table.xml`.** `posenet_cam`
   (like `front` before it) is defined in the GENERATED dual-arm scene
   (`src/bimanual/sim/assets/so101_dual_table.xml`, emitted by
   `gen_dual_scene.py`), not in the frozen upstream asset (`scenes/so101/`,
   ADR-016). This script renders through `TableSettingEnv`'s own opt-in
   camera path (ADR-022) -- `cameras=[CAMERA]` at construction, plus the
   `render()` escape hatch for the actual per-sample capture -- and imports
   neither XML file's contents directly; it only loads the compiled model
   through `TableSettingEnv`.

3. **Per-prop z, not one shared table height.** Each of the five props has
   its OWN resting z (`gen_dual_scene.py`'s `PLATE_POS`/`MUG_POS`/`FORK_POS`/
   `SPOON_POS`/`BOTTLE_POS`: 0.355 / 0.39 / 0.356 / 0.356 / 0.44, against a
   table surface of 0.35). This script reads each prop's resting z LIVE off
   `data.qpos` immediately after `env.reset()` (not a hardcoded duplicate of
   those constants) and only ever overwrites the x,y slots of each prop's
   free-joint qpos, leaving z (and orientation) exactly as `reset()` set it.
   Using a single z would sink every prop into the table slab.

4. **Runtime qpos randomization, never scene regeneration.** Positions are
   randomized by writing directly into `data.qpos` after `env.reset()`,
   followed by `mujoco.mj_forward` to recompute derived kinematic quantities
   (`data.xpos` etc.) before rendering. `gen_dual_scene.py` is never invoked
   by this script and `so101_dual_table.xml` is never written to -- that file
   is load-bearing for four verified working skills, and ADR-038 recorded
   exactly how a regenerated/edited scene broke them.

**On reachability (ADR-038).** ADR-038 found that moving even ONE prop (the
mug alone, to a far corner) breaks the `handoff` skill at phase 3. This
script's randomized layouts are therefore expected to include many
configurations in which the scripted manipulation skills would fail. That is
fine here: this dataset trains a PERCEPTION model and no skill is ever
executed against any sampled layout. Nothing in this script, its output, or
`dataset_meta.json` should be read as asserting that these layouts are
validated or reachable scene configurations for manipulation -- see the
`note` field in the written `dataset_meta.json`.

**Collision handling.** x and y are drawn independently and uniformly per
prop from `RANDOMIZATION_RANGE_M`; a full draw (all five props) is rejected
and retried if any pair's centre-to-centre planar distance is below that
pair's `FOOTPRINT_RADIUS_M` sum plus `CLEARANCE_MARGIN_M`. This is a
render-legibility check (props visibly not overlapping in the image), not a
physics settle -- no simulation step is taken between placement and render,
so there is no physics to "resolve" an accepted overlap-free draw.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import struct
import sys
import time
import zlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.sim.env import TableSettingEnv  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE_PATH = REPO_ROOT / "src" / "bimanual" / "sim" / "assets" / "so101_dual_table.xml"
OUT_ROOT = REPO_ROOT / "data" / "posenet"

RENDER_W, RENDER_H = 224, 224
# M10 Phase 1.5: switched from `front` to the dedicated `posenet_cam`
# (`scripts/gen_dual_scene.py`). Phase 1 measured props occupying only
# ~10-20 px of a 224x224 `front` frame -- `front` is a wide establishing
# shot built for the M06 handoff demo video, not a perception viewpoint.
# `posenet_cam` sits closer to the table and angled down at the working
# area instead. See docs/hardware/m10-camera-comparison.md for the measured
# before/after pixel extent.
CAMERA = "posenet_cam"

# The five manipulable props (M02), each with its own MJCF free joint named
# "<body>_free" (`gen_dual_scene.py`'s TEMPLATE, e.g. `<freejoint name="plate_free"/>`).
PROP_BODY_NAMES = ("plate", "mug", "fork", "spoon", "water_bottle")

# ---- ADR-041: 3-prop label scope ---------------------------------------
# All five props are still placed and rendered (occlusion between all five
# is part of the training signal, and removing plate/spoon from the scene
# entirely would make the remaining three trivially unoccluded -- the
# opposite of what a useful PoseNet needs to learn). Only THESE three get a
# ground-truth label and count toward the visibility-filtering gate below.
# Order here is the CANONICAL order used for every sample's fixed-shape
# `positions_xyz_m` (3, 3) array in the label JSON -- see main()'s label
# construction.
TARGET_PROPS = ("fork", "water_bottle", "mug")
# Stay in-scene, randomized identically to the target props, but never
# labelled -- pure visual/occlusion decoration for this pass.
DECORATION_PROP_NAMES = tuple(n for n in PROP_BODY_NAMES if n not in TARGET_PROPS)
assert DECORATION_PROP_NAMES == ("plate", "spoon"), DECORATION_PROP_NAMES

# ---- ADR-041: visibility (occlusion) filtering --------------------------
# **This is an OCCLUSION ratio, not Syn4D's fill-fraction metric.** An
# earlier draft of this task's brief described the filter as "the ratio of
# visible pixels to bounding-box area" (a FILL-FRACTION metric: how much of
# its own 2D bounding box an object occupies -- a property of the object's
# SHAPE, not of occlusion by other objects; a thin, fully-visible fork
# would fail a fill-fraction test on shape alone). The formula the same
# brief then specified, `full_scene_pixels / single_prop_pixels`, is a
# genuine occlusion ratio instead: 1.0 when nothing else in the scene
# covers any of the prop's pixels, falling toward 0 as other props
# increasingly cover it. This script implements THAT formula --
# `compute_visibility_ratios` below -- and the 0.30 threshold is a
# **user-supplied reference to Syn4D** (https://arxiv.org/pdf/2605.05207)
# for that specific numeric value; nothing about that paper's authorship,
# method, or findings beyond this one threshold is claimed here (the same
# citation discipline ADR-037 already established for the ScienceDirect
# reference in this repo -- cite the number, do not invent the content).
VISIBILITY_THRESHOLD = 0.30
# ---- Occlusion measurement: MuJoCo segmentation buffer, NOT per-pixel RGB
# colour classification -- a real attempt at colour classification was
# made first and measurably failed before this was adopted. -----------
# The first implementation of this filter classified pixels by comparing
# each prop's declared `<material rgba=...>` (`gen_dual_scene.py`'s
# TEMPLATE, e.g. fork_material="1.0 0.15 0.15") against the RENDERED
# image. Two things were measured wrong with that, on bm-ptl, on this exact
# scene, before any ratio was trusted:
#   1. MuJoCo's lighting model does not preserve the material's own colour
#      ratio under shading -- fork's declared (255, 38, 38) rendered as a
#      DOMINANT colour of (255, 80, 80), an exact-match pixel count of 1
#      against a real, visually-obvious ~100+ pixel sliver (verified by
#      solo-rendering just the fork and eyeballing the PNG -- clearly
#      visible, not 1 pixel).
#   2. Widening the match tolerance to compensate then ALIASED with other
#      scene elements: `water_bottle`'s blue (0.10, 0.40, 0.95) collided
#      with the background checker floor tile's own rendered blue
#      ((104,154,207) / (51,104,154) -- both B-dominant, same magnitude of
#      B-R and B-G separation as the bottle's own rendered colour), and
#      `fork`'s red collided with the table surface's warm tan
#      ((201,146,91), which also satisfies a loose "R much greater than G
#      and B" test). Both collisions were measured directly (see this
#      task's completion notes / docs/hardware/m10-scope-reduction-samples.md
#      for the exact pixel dumps), not assumed.
# Colour classification was therefore abandoned in favour of
# `mujoco.Renderer.enable_segmentation_rendering()`, which returns each
# pixel's OWN geom id (verified empirically: the returned (H, W, 2) int32
# array's channel 0 is the geom id, channel 1 is a constant
# `mjtObj.mjOBJ_GEOM` -- confirmed on a probe render before use, not
# assumed from documentation alone, since this project's own convention is
# not to trust API shape claims on faith -- see the camera-xyaxes and
# footprint-radius corrections elsewhere in this file's history for the
# same discipline). This is EXACT (a pixel's geom id has no lighting-
# dependent ambiguity at all), not a heuristic.
_MJOBJ_GEOM = int(mujoco.mjtObj.mjOBJ_GEOM)
# Where a prop is teleported to when computing another prop's SOLO
# (unoccluded-position) pixel count -- see `compute_visibility_ratios`.
# Genuinely removes it from the render (a plain kinematic translation, not
# an alpha/rgba trick that may still write depth or leave faint pixels
# depending on the renderer path -- see this function's own verification
# note). 3 m away from a scene whose whole table span is well under 1 m,
# seen through this camera's fovy centred on the table, is far outside any
# possible view frustum.
OFFSCREEN_XY = (3.0, 3.0)
VISIBILITY_MAX_RESAMPLES = 50

# M10 Phase 1.5: widened from [-0.15, 0.15] to [-0.18, 0.18] (a 0.36m x
# 0.36m square, 1.44x the old 0.09 sq m area) -- both to make PoseNet see a
# wider variety of prop positions and, combined with the eased clearance
# below, to bring the worst-case rejection-sampling draw count down from
# Phase 1's measured 5054 (against a 5000-per-prop cap) to comfortably
# inside budget. Still centred on the table's own origin; z is never
# touched here (read live per correction 3 instead, see main()).
RANDOMIZATION_RANGE_M = {"x": (-0.18, 0.18), "y": (-0.18, 0.18)}

# Per-prop planar "footprint radius" (m): the farthest any of that prop's own
# geoms reaches from its body origin in the xy-plane, read off
# `scripts/gen_dual_scene.py`'s geom sizes/fromtos (not measured empirically):
#   plate  : PLATE_DISH_RADIUS_M = 0.06 m (the larger of its two stacked geoms)
#   mug    : mug_handle capsule reaches x=0.06 m from the body origin
#   fork   : fork_head box reaches 0.055 + 0.025 = 0.08 m
#   spoon  : spoon_bowl ellipsoid reaches 0.05 + 0.018 = 0.068 m, rounded up
#   bottle : water_bottle_body cylinder radius 0.03 m (the cap sits higher in
#            z, not wider in xy, so it does not add planar extent)
#
# M10 Phase 1.5 re-check of these numbers, done before touching either
# number (the task brief for this pass claimed fork=0.08/spoon=0.07 are
# "far larger than those objects' actual footprint" -- checked by hand
# below rather than accepted, the same way the posenet_cam xyaxes claim in
# this same pass was checked and turned out to be wrong; this one turned
# out to be right in spirit but wrong on these two specific numbers):
#   fork:  handle capsule reaches x=-0.06-0.004=-0.064; fork_head box
#          (size 0.025x0.012x0.003, pos x=0.055) reaches its far corner at
#          (0.08, +-0.012) -> distance from origin = sqrt(0.08^2+0.012^2)
#          = 0.0809 m. Declared 0.08 m is NOT over-generous -- it is
#          already ~1mm UNDER the true farthest point.
#   spoon: handle capsule reaches x=-0.064; spoon_bowl ellipsoid (size
#          0.018x0.012x0.004, pos x=0.05) far corner at (0.068, +-0.012)
#          -> distance = sqrt(0.068^2+0.012^2) = 0.0690 m. Declared 0.07 m
#          is ~1mm OVER, i.e. accurate to the mm, not "far larger."
# Both are already tight, correct bounds on the true farthest point of each
# prop's geometry from its own body origin -- reducing either below its
# measured true reach would let two props' near-facing tips genuinely
# overlap for SOME relative bearing angle, since this is a circular
# (isotropic) distance test with no orientation term (props are never
# rotated, only translated -- see the module docstring's "Collision
# handling" paragraph). FOOTPRINT_RADIUS_M is therefore left UNCHANGED
# from Phase 1. Easing is done entirely via the widened randomization range
# above and the reduced clearance margin below instead -- the safe lever,
# confirmed by the 10-sample visual check before the full regeneration
# (docs/hardware/m10-camera-comparison.md).
FOOTPRINT_RADIUS_M = {
    "plate": 0.06,
    "mug": 0.06,
    "fork": 0.08,
    "spoon": 0.07,
    "water_bottle": 0.03,
}

# Extra gap enforced ON TOP OF the two radii being summed, so an accepted
# layout has props visibly separated in the rendered image rather than just
# barely not touching. Reduced from Phase 1's 0.015 m to 0.008 m for M10
# Phase 1.5 (still a real, positive render-legibility gap -- not a physics
# tolerance, since no simulation step runs between placement and render).
# Combined with the widened 0.36m x 0.36m box above, the worst-case pair
# (fork/spoon: 0.08+0.07+0.008=0.158 m forbidden-disk radius) now forbids
# pi*0.158^2=0.0785 sq m out of the box's 0.1296 sq m (~61%), down from
# Phase 1's pi*0.165^2=0.0855 out of 0.09 sq m (~95%, which is what drove
# the measured 5054-draw worst case against the 5000 cap). See
# docs/hardware/m10-camera-comparison.md for the measured post-fix draw
# counts on the 10-sample verification run.
CLEARANCE_MARGIN_M = 0.008

# Placement strategy, and why it is SEQUENTIAL rather than "draw all five at
# once and reject the whole draw": five independent uniform draws inside a
# 0.30x0.30 m box (area 0.09 m^2), all satisfying pairwise separations of
# ~0.11-0.17 m simultaneously, is a tight packing problem -- e.g. the
# fork<->spoon threshold alone (0.08+0.07+0.015=0.165 m) has a forbidden disk
# of area pi*0.165^2 = 0.0855 m^2, i.e. up to ~95% of the WHOLE box can be
# invalid for the second point once the first is placed anywhere near centre.
# A first real attempt confirmed this is not theoretical: with a full-draw
# rejection scheme and 500 attempts (this constant's original value), sample
# 0 exhausted every attempt and raised RuntimeError on the very first call,
# on bm-ptl, before any of the 10 verification samples were produced.
# Sequential placement -- one prop at a time, each drawn against only the
# props already placed -- turns this into five easier 1-point rejection
# problems instead of one hard 5-point joint one, and is what is actually
# implemented below.
#
# M10 Phase 1.5: Phase 1 always placed in this SAME largest-footprint-first
# order (fork, spoon, plate, mug, water_bottle) on every one of the 5000
# samples. That is a systematic bias, not just a performance heuristic:
# `water_bottle` (smallest radius) was placed LAST in every single sample,
# so its accepted position always had to dodge four already-placed props --
# whatever positional distribution that produces is correlated with
# "placed last", not with the prop's own identity, and a PoseNet trained on
# it could pick up that spurious correlation instead of the actual
# appearance-to-position mapping this dataset exists to teach. Fixed by
# drawing a FRESH random permutation of BASE_PLACEMENT_ORDER per sample
# (`rng.permutation`, see `sample_nonoverlapping_positions` below) -- every
# prop is placed first, last, and everywhere in between across the dataset,
# not systematically last. `BASE_PLACEMENT_ORDER`'s largest-first content is
# kept only as the SET of props to place (order no longer matters for
# packing difficulty once the range/margin easing above is in effect); the
# per-sample `placement_order_used` is recorded in each sample's label JSON
# and in `dataset_meta.json`'s stats for auditability.
BASE_PLACEMENT_ORDER = ("fork", "spoon", "plate", "mug", "water_bottle")
PER_PROP_ATTEMPTS = 5000
MAX_SAMPLE_RESTARTS = 200


def write_png(path: pathlib.Path, rgb: np.ndarray) -> None:
    """Write an HxWx3 uint8 array as a PNG using only the standard library.

    Copied verbatim, with attribution, from `scripts/render_handoff_frames.py`'s
    `write_png` (itself copied from `scripts/probe_render.py`) -- Pillow/imageio
    are NOT in `scripts/requirements-bmptl.txt` and this script does not add
    them (correction 1). Reused rather than reimplemented so this exact,
    already-proven encoder (zlib + hand-rolled IHDR/IDAT/IEND chunks) is not
    duplicated with a subtle bug.
    """
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def build_target_geom_ids(model) -> dict[str, list[int]]:
    """Resolve each `TARGET_PROPS` body's OWN geom ids via `model.geom_bodyid`,
    once, right after the model is compiled. A prop can be more than one geom
    on one body (e.g. `mug` = a cylinder + a handle capsule, both on the
    `mug` body, per `gen_dual_scene.py`'s TEMPLATE) -- classifying by BODY,
    not by a single assumed geom, is what makes this correct regardless of
    how many geoms make up a given prop.
    """
    out: dict[str, list[int]] = {}
    for name in TARGET_PROPS:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid == -1:
            raise RuntimeError(f"expected body {name!r} not found in compiled model")
        out[name] = [g for g in range(model.ngeom) if model.geom_bodyid[g] == bid]
    return out


def _segmentation_pixel_counts(
    seg_renderer, data, camera: str, target_geom_ids: dict[str, list[int]]
) -> dict[str, int]:
    """One segmentation render -> per-target pixel counts, all at once
    (classifying the SAME image by different geom-id sets costs nothing
    extra, unlike a separate RGB render per target would).
    """
    seg_renderer.update_scene(data, camera=camera)
    seg = seg_renderer.render()  # (H, W, 2) int32: ch0=geom id, ch1=mjtObj type (constant GEOM, verified on probe)
    is_geom = seg[:, :, 1] == _MJOBJ_GEOM
    ids = seg[:, :, 0]
    return {name: int((is_geom & np.isin(ids, gids)).sum()) for name, gids in target_geom_ids.items()}


def compute_visibility_ratios(
    seg_renderer,
    model,
    data,
    camera: str,
    prop_qpos_adr: dict[str, int],
    positions: dict[str, tuple[float, float]],
    target_geom_ids: dict[str, list[int]],
) -> dict[str, float]:
    """Per-target-prop OCCLUSION ratio for the CURRENT placement (all five
    props already written into `data.qpos` and `mujoco.mj_forward`-ed by the
    caller).

    **This is an occlusion ratio, not Syn4D's fill-fraction metric** -- see
    `VISIBILITY_THRESHOLD`'s module-level comment for the distinction and
    why this formula (not "visible pixels / bounding-box area") is what is
    implemented: `ratio = full_scene_pixel_count / solo_pixel_count`, where
    `solo_pixel_count` is that SAME prop's own pixel count with every OTHER
    prop moved off-screen (so nothing else in the scene can occlude it) but
    this prop left at its exact accepted position -- i.e. the ratio measures
    how much of the prop's own maximum-visible-at-this-pose silhouette
    survives once the other four props are back in the scene. Both counts
    come from `mujoco.Renderer`'s segmentation buffer (geom id per pixel,
    `_segmentation_pixel_counts` above), not RGB colour classification --
    see the module-level comment above `_MJOBJ_GEOM` for the two measured
    colour-aliasing failures (table tan vs. fork red, floor-tile blue vs.
    bottle blue) that made a colour-based version of this function
    untrustworthy.

    **Hiding mechanism, and why it is trusted.** Other props are "hidden" by
    writing a real, far-off-screen (x, y) into their own free-joint qpos
    (`OFFSCREEN_XY`) and calling `mujoco.mj_forward` -- a genuine kinematic
    relocation, not an rgba/alpha=0 trick (the task's own caution: alpha
    tricks can still write depth or leave faint pixels depending on the
    renderer path). Verified empirically, not assumed: a dedicated probe
    (docs/hardware/m10-scope-reduction-samples.md) places one prop in view,
    teleports the other four to `OFFSCREEN_XY`, and confirms the
    segmentation buffer contains ONLY that one prop's geom ids -- zero
    pixels for every other prop's geom ids -- before any ratio computed this
    way is trusted.

    Original (x, y) for every OTHER prop is restored, and `mj_forward` is
    called again, before this function returns -- `data.qpos` is left
    exactly as the caller passed it in.
    """
    full_counts = _segmentation_pixel_counts(seg_renderer, data, camera, target_geom_ids)

    ratios: dict[str, float] = {}
    all_names = list(positions.keys())
    for target in TARGET_PROPS:
        others = [n for n in all_names if n != target]
        saved_xy = {}
        for name in others:
            adr = prop_qpos_adr[name]
            saved_xy[name] = (float(data.qpos[adr]), float(data.qpos[adr + 1]))
            data.qpos[adr] = OFFSCREEN_XY[0]
            data.qpos[adr + 1] = OFFSCREEN_XY[1]
        mujoco.mj_forward(model, data)

        solo_counts = _segmentation_pixel_counts(seg_renderer, data, camera, target_geom_ids)
        solo_count = solo_counts[target]

        for name, (ox, oy) in saved_xy.items():
            adr = prop_qpos_adr[name]
            data.qpos[adr] = ox
            data.qpos[adr + 1] = oy
        mujoco.mj_forward(model, data)

        if solo_count <= 0:
            ratios[target] = 0.0
        else:
            ratios[target] = min(1.0, full_counts[target] / solo_count)
    return ratios


def _min_dist(a: str, b: str) -> float:
    return FOOTPRINT_RADIUS_M[a] + FOOTPRINT_RADIUS_M[b] + CLEARANCE_MARGIN_M


def _try_place_all(
    rng: np.random.Generator, order: tuple[str, ...]
) -> tuple[dict[str, tuple[float, float]], int] | None:
    """One attempt at placing every prop in `order`, sequentially, each
    rejected against only the props already placed so far. Returns
    (positions, total_draws_used) on success, or None if any single prop
    exhausts `PER_PROP_ATTEMPTS` draws without finding a valid spot (the
    caller retries the whole sample from scratch -- see
    `sample_nonoverlapping_positions`).
    """
    placed: dict[str, tuple[float, float]] = {}
    total_draws = 0
    for name in order:
        for attempt in range(1, PER_PROP_ATTEMPTS + 1):
            total_draws += 1
            x = float(rng.uniform(*RANDOMIZATION_RANGE_M["x"]))
            y = float(rng.uniform(*RANDOMIZATION_RANGE_M["y"]))
            ok = True
            for other_name, (ox, oy) in placed.items():
                dist = ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
                if dist < _min_dist(name, other_name):
                    ok = False
                    break
            if ok:
                placed[name] = (x, y)
                break
        else:
            return None  # this prop never found a valid spot -- restart the whole sample
    return placed, total_draws


def sample_nonoverlapping_positions(
    rng: np.random.Generator,
) -> tuple[dict[str, tuple[float, float]], int, tuple[str, ...]]:
    """Place all five props with no pair closer than its combined footprint
    + clearance, using SEQUENTIAL per-prop rejection sampling (see
    `BASE_PLACEMENT_ORDER`'s comment for why this replaced an earlier
    full-joint-draw scheme that measurably failed on the first real bm-ptl
    run).

    M10 Phase 1.5: the placement order is a FRESH random permutation of
    `BASE_PLACEMENT_ORDER`, drawn from `rng` once per sample (not a fixed
    largest-first order every time -- see `BASE_PLACEMENT_ORDER`'s comment
    for why a fixed order is a labelled bias, not just a packing
    heuristic). The SAME shuffled order is reused across all restarts
    within this one sample (a sample either succeeds or exhausts restarts;
    reshuffling mid-sample would not change the difficulty of a
    packing problem it has already failed at).

    Returns (positions, total_draws_used_across_all_restarts,
    order_used). Raises RuntimeError if `MAX_SAMPLE_RESTARTS` whole-sample
    restarts are exhausted -- not expected in practice (a valid 5-prop
    layout inside the 0.36 m x 0.36 m box demonstrably exists, e.g. four
    corners + a centre point), but guarded rather than looping forever.
    """
    order = tuple(BASE_PLACEMENT_ORDER[i] for i in rng.permutation(len(BASE_PLACEMENT_ORDER)))
    total_draws = 0
    for _restart in range(1, MAX_SAMPLE_RESTARTS + 1):
        result = _try_place_all(rng, order)
        if result is not None:
            positions, draws_this_restart = result
            return positions, total_draws + draws_this_restart, order
        total_draws += PER_PROP_ATTEMPTS  # at least this many draws were burned before giving up

    raise RuntimeError(
        f"could not find a non-overlapping 5-prop layout after {MAX_SAMPLE_RESTARTS} "
        f"whole-sample restarts ({total_draws} total draws), order={order}"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=5000, help="number of samples to generate")
    parser.add_argument(
        "--seed", type=int, default=20260914,
        help="explicit RNG seed for prop-position randomization (recorded in dataset_meta.json)",
    )
    parser.add_argument(
        "--env-reset-seed", type=int, default=0,
        help="seed passed to TableSettingEnv.reset() every sample (arm pose / baseline state)",
    )
    parser.add_argument(
        "--out-dir", type=pathlib.Path, default=OUT_ROOT,
        help="output directory (images/, labels/, dataset_meta.json written under it)",
    )
    parser.add_argument("--progress-every", type=int, default=250)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    images_dir = args.out_dir / "images"
    labels_dir = args.out_dir / "labels"
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)

    # cameras=["front"] at construction is the opt-in declaration (ADR-022,
    # correction 2); the actual per-sample capture below uses render()
    # (the always-available escape hatch, `src/bimanual/sim/env.py`'s
    # `render()` docstring) so reset() itself can skip rendering (cameras=
    # None override) until AFTER this sample's props are placed.
    env = TableSettingEnv(cameras=[CAMERA], render_width=RENDER_W, render_height=RENDER_H)
    model, data = env.model, env.data

    # ADR-041: a SECOND, dedicated `mujoco.Renderer` in segmentation mode,
    # built once (renderer construction has real GL/EGL setup cost, per
    # `TableSettingEnv._ensure_renderer`'s own docstring -- not something to
    # pay per-sample). Independent of `env`'s own RGB renderer -- this
    # script never toggles segmentation mode on/off on the SAME renderer,
    # which sidesteps any question of whether that would be safe.
    seg_renderer = mujoco.Renderer(model, height=RENDER_H, width=RENDER_W)
    seg_renderer.enable_segmentation_rendering()
    target_geom_ids = build_target_geom_ids(model)

    prop_qpos_adr: dict[str, int] = {}
    body_ids: dict[str, int] = {}
    for name in PROP_BODY_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_free")
        if jid == -1:
            raise RuntimeError(f"expected free joint '{name}_free' not found in compiled model")
        prop_qpos_adr[name] = int(model.jnt_qposadr[jid])

        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid == -1:
            raise RuntimeError(f"expected body '{name}' not found in compiled model")
        body_ids[name] = bid

    # Correction 3: read each prop's own resting z LIVE off the post-reset
    # state, rather than hardcoding a duplicate of gen_dual_scene.py's
    # position constants. This is captured once (reset() is deterministic
    # for a fixed seed) purely to RECORD it in dataset_meta.json; the
    # per-sample loop below re-reset()s every iteration anyway, which is
    # what actually re-establishes each prop's z before x,y are overwritten.
    env.reset(seed=args.env_reset_seed, cameras=None)
    resting_z_m = {name: float(data.qpos[prop_qpos_adr[name] + 2]) for name in PROP_BODY_NAMES}
    print("Resting z per prop (read live from post-reset qpos):")
    for name, z in resting_z_m.items():
        print(f"  {name}: {z:.4f} m")

    rng = np.random.default_rng(args.seed)
    attempts_log: list[int] = []
    visibility_rejections_log: list[int] = []  # rejections BURNED per accepted sample
    total_visibility_rejections = 0
    t_start = time.time()

    for i in range(args.count):
        # ADR-041: the outer loop redraws a WHOLE new placement (and re-
        # renders) whenever the visibility gate below rejects one -- bounded
        # by VISIBILITY_MAX_RESAMPLES so a pathological seed fails loudly
        # rather than looping forever.
        sample_rejections = 0
        for _resample in range(VISIBILITY_MAX_RESAMPLES + 1):
            # Full reset every sample: restores arm "home" pose and every
            # prop's own resting (x, y, z, quat) BEFORE this sample's x,y
            # overwrite -- cameras=None here so reset() itself does not
            # render (props are not yet at their randomized positions).
            env.reset(seed=args.env_reset_seed, cameras=None)

            positions, attempts, order_used = sample_nonoverlapping_positions(rng)

            for name, (x, y) in positions.items():
                adr = prop_qpos_adr[name]
                data.qpos[adr] = x       # overwrite x only
                data.qpos[adr + 1] = y   # overwrite y only -- z (adr+2) and
                # the quaternion (adr+3:adr+7) are left exactly as reset() set them.
            mujoco.mj_forward(model, data)  # recompute data.xpos etc. from the new qpos

            img = env.render(CAMERA)  # escape hatch; (224, 224, 3) uint8 -- the saved dataset image

            # ADR-041 visibility gate: 1 full-scene + 3 per-target solo
            # SEGMENTATION renders (independent of the RGB render above) --
            # see compute_visibility_ratios's docstring for why segmentation,
            # not colour classification.
            visibility_ratios = compute_visibility_ratios(
                seg_renderer, model, data, CAMERA, prop_qpos_adr, positions, target_geom_ids
            )
            if all(visibility_ratios[t] >= VISIBILITY_THRESHOLD for t in TARGET_PROPS):
                break
            sample_rejections += 1
            total_visibility_rejections += 1
        else:
            raise RuntimeError(
                f"sample {i}: could not find a placement clearing the "
                f"visibility_threshold={VISIBILITY_THRESHOLD} for all of "
                f"{TARGET_PROPS} after {VISIBILITY_MAX_RESAMPLES} resamples "
                f"(last ratios: {visibility_ratios})"
            )

        attempts_log.append(attempts)
        visibility_rejections_log.append(sample_rejections)

        sample_id = f"sample_{i:05d}"
        write_png(images_dir / f"{sample_id}.png", img)

        # ADR-041: labels only for TARGET_PROPS, in TARGET_PROPS' fixed
        # canonical order -- both the `objects` dict (keyed, easy to read)
        # and `positions_xyz_m` (a (3, 3) array, the fixed-shape form a
        # training loop actually consumes) carry the SAME three props in
        # the SAME order.
        objects: dict[str, dict] = {}
        positions_xyz_m: list[list[float]] = []
        for name in TARGET_PROPS:
            xyz = data.xpos[body_ids[name]]
            objects[name] = {
                "xyz_m": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                "randomized_xy_m": [positions[name][0], positions[name][1]],
                "z_fixed_to_own_resting_height": True,
                "visibility_ratio": visibility_ratios[name],
            }
            positions_xyz_m.append([float(xyz[0]), float(xyz[1]), float(xyz[2])])

        label = {
            "sample_id": sample_id,
            "sample_index": i,
            "env_reset_seed": args.env_reset_seed,
            "rng_seed": args.seed,
            "camera": CAMERA,
            "resolution": [RENDER_W, RENDER_H],
            "image_path": f"images/{sample_id}.png",
            "placement_draws": attempts,
            "placement_order_used": list(order_used),
            "visibility_resamples": sample_rejections,
            "target_props": list(TARGET_PROPS),
            "output_shape": [len(TARGET_PROPS), 3],
            "positions_xyz_m": positions_xyz_m,
            "objects": objects,
            "decoration_props_in_scene_unlabelled": list(DECORATION_PROP_NAMES),
        }
        (labels_dir / f"{sample_id}.json").write_text(json.dumps(label, indent=2))

        if (i + 1) % args.progress_every == 0 or (i + 1) == args.count:
            elapsed = time.time() - t_start
            rate = (i + 1) / elapsed if elapsed > 0 else float("nan")
            n_accepted = i + 1
            n_total_draws = n_accepted + total_visibility_rejections
            rejection_rate = total_visibility_rejections / n_total_draws if n_total_draws else 0.0
            print(
                f"[{i + 1}/{args.count}] elapsed={elapsed:.1f}s rate={rate:.2f} samples/s "
                f"visibility_rejections={total_visibility_rejections} "
                f"rejection_rate={rejection_rate:.3f}"
            )

    wall_clock_s = time.time() - t_start
    seg_renderer.close()
    env.close()

    scene_sha256 = hashlib.sha256(SCENE_PATH.read_bytes()).hexdigest()
    samples_per_second = (args.count / wall_clock_s) if wall_clock_s > 0 else None
    n_total_draws = args.count + total_visibility_rejections
    visibility_rejection_rate = (
        total_visibility_rejections / n_total_draws if n_total_draws else None
    )

    meta = {
        "sample_count": args.count,
        "resolution": [RENDER_W, RENDER_H],
        "camera": CAMERA,
        "camera_pos_m": [0.6, 0.0, 1.1],
        "camera_xyaxes": "0 1 0 -0.7 0 0.7",
        "rng_seed": args.seed,
        "env_reset_seed": args.env_reset_seed,
        "randomization_range_m": RANDOMIZATION_RANGE_M,
        "props_randomized": list(PROP_BODY_NAMES),
        "target_props": list(TARGET_PROPS),
        "output_shape": [len(TARGET_PROPS), 3],
        "decoration_props_in_scene_unlabelled": list(DECORATION_PROP_NAMES),
        "visibility_threshold": VISIBILITY_THRESHOLD,
        "visibility_metric": (
            "occlusion ratio = full_scene_pixel_count / solo_unoccluded_pixel_count "
            "for the same prop at the same placement (NOT a fill-fraction / "
            "bounding-box-area metric -- see the module docstring / "
            "compute_visibility_ratios for the distinction). Both pixel counts "
            "are measured via mujoco.Renderer's segmentation buffer (exact "
            "per-pixel geom id), not RGB colour classification -- an initial "
            "colour-classification implementation was tried and measurably "
            "failed (table tan aliased fork red, floor-tile blue aliased "
            "water_bottle blue) before this pivot; see the module-level "
            "comment above _MJOBJ_GEOM."
        ),
        "visibility_threshold_reference": (
            "0.30 threshold value is a user-supplied reference to Syn4D "
            "(https://arxiv.org/pdf/2605.05207); this repository cites only "
            "that numeric threshold from it and does not claim to have read "
            "or verified the paper's authorship, method, or other findings."
        ),
        "visibility_rejections_total": total_visibility_rejections,
        "visibility_rejection_rate": visibility_rejection_rate,
        "resting_z_m": resting_z_m,
        "footprint_radius_m": FOOTPRINT_RADIUS_M,
        "clearance_margin_m": CLEARANCE_MARGIN_M,
        "base_placement_order": list(BASE_PLACEMENT_ORDER),
        "placement_order_shuffled_per_sample": True,
        "per_prop_attempts": PER_PROP_ATTEMPTS,
        "max_sample_restarts": MAX_SAMPLE_RESTARTS,
        "placement_draws_stats": {
            "mean": float(np.mean(attempts_log)) if attempts_log else None,
            "max": int(np.max(attempts_log)) if attempts_log else None,
        },
        "mujoco_version": mujoco.__version__,
        "scene_xml_path": str(SCENE_PATH.relative_to(REPO_ROOT)).replace("\\", "/"),
        "scene_xml_sha256": scene_sha256,
        "wall_clock_seconds": wall_clock_s,
        "samples_per_second": samples_per_second,
        "phase": "M10 Phase 1.5 correction (ADR-041)",
        "supersedes": (
            "M10 Phase 1 (ADR-039: camera='front', randomization_range_m=+-0.15, "
            "clearance_margin_m=0.015, fixed largest-first placement_order, "
            "placement_draws_stats.max=5054 against a 5000-per-prop cap, all "
            "five props labelled) AND M10 Phase 1.5 (ADR-040: posenet_cam at "
            "pos=(1.05,0,0.62)/xyaxes='0 1 0 -0.3 0 0.95', all five props "
            "labelled, no visibility filtering -- its 5000-sample regeneration "
            "was halted mid-run by the orchestrator and data/posenet/images+"
            "labels were deleted, so that dataset never completed). This run "
            "(ADR-041) is a fresh generation, not a merge: 3-prop label scope "
            "(fork/water_bottle/mug; plate/spoon stay in-scene as unlabelled "
            "decoration), an overhead posenet_cam pose "
            "(pos=(0.6,0,1.1)/xyaxes='0 1 0 -0.7 0 0.7'), and occlusion-ratio "
            "visibility filtering (threshold 0.30) are all new in this run -- "
            "see docs/hardware/m10-scope-reduction-samples.md."
        ),
        "note": (
            "Object x,y positions are randomized independently per sample within "
            "randomization_range_m, subject to pairwise non-overlap rejection "
            "sampling (see footprint_radius_m / clearance_margin_m). Placement "
            "order is a fresh random permutation of base_placement_order per "
            "sample (see each label's placement_order_used), not a fixed order, "
            "so no prop is systematically placed last. z is held fixed per-prop "
            "at its own resting height (resting_z_m) -- never at one shared "
            "table-surface value. Rendered through posenet_cam (overhead pose, "
            "ADR-041), a dedicated perception camera separate from 'front'. "
            "All five props are placed and rendered, but only target_props "
            "(fork, water_bottle, mug) are labelled -- decoration_props_in_"
            "scene_unlabelled (plate, spoon) remain in-frame for visual realism "
            "and occlusion signal only. A sample is rejected and re-drawn "
            "(visibility_resamples in each label) if any target prop's "
            "occlusion ratio falls below visibility_threshold. Randomized "
            "layouts are NOT validated or claimed as physically reachable/"
            "solvable scene configurations: ADR-038 found that moving even a "
            "single prop can break the handoff skill at phase 3. This dataset "
            "trains a perception model (PoseNet) only; no manipulation skill "
            "is ever executed against any sampled layout in this script."
        ),
    }
    (args.out_dir / "dataset_meta.json").write_text(json.dumps(meta, indent=2))

    print(f"Wrote {args.count} samples to {images_dir} and {labels_dir}")
    print(f"Wall clock: {wall_clock_s:.2f}s ({samples_per_second:.2f} samples/s)" if samples_per_second else f"Wall clock: {wall_clock_s:.2f}s")
    print(f"Visibility rejections: {total_visibility_rejections} (rate={visibility_rejection_rate:.4f})" if visibility_rejection_rate is not None else "Visibility rejections: n/a")
    print(f"Meta written to {args.out_dir / 'dataset_meta.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
