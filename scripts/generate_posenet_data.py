"""M10 Phase 1: PoseNet training-data generation.

This script ONLY generates labelled images. It does not define, train, or run
any model -- that is M10 Phase 2+. PoseNet will eventually replace the
scripted controller's oracle `data.xpos` prop-position reads with inference
from the `front` camera; this script produces the (image, ground-truth-xyz)
pairs that later training will consume.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`, which cannot load on the developer's Windows laptop. Uses the
`mujoco==3.2.7` pin unmodified (`scripts/requirements-bmptl.txt`) -- this
script never upgrades or reinstalls anything, and that pairing with
`openvino==2026.3.1` is load-bearing for the rest of the OpenVINO story.

Four corrections this script embodies, each traceable to a real mistake a
prior pass would otherwise have made:

1. **No Pillow.** `PIL` is not installed on bm-ptl and this script does not
   `pip install` it (an unrelated install is exactly what silently upgraded
   mujoco during the mink probe, ADR-037). `write_png` below is copied
   verbatim, with attribution, from `scripts/render_handoff_frames.py`'s own
   `write_png` (itself copied from `scripts/probe_render.py`), which is a
   stdlib-only (zlib + hand-rolled PNG chunks) RGB writer already proven to
   handle arbitrary HxWx3 uint8 arrays (1280x720 and 640x480 in that script;
   224x224 here -- verified in the 10-sample smoke run before the full 5000).

2. **No touching `scenes/so101/` or `so101_dual_table.xml`.** The `front`
   camera is defined in the GENERATED dual-arm scene
   (`src/bimanual/sim/assets/so101_dual_table.xml`, emitted by
   `gen_dual_scene.py`), not in the frozen upstream asset (`scenes/so101/`,
   ADR-016). This script renders through `TableSettingEnv`'s own opt-in
   camera path (ADR-022) -- `cameras=["front"]` at construction, plus the
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
CAMERA = "front"

# The five manipulable props (M02), each with its own MJCF free joint named
# "<body>_free" (`gen_dual_scene.py`'s TEMPLATE, e.g. `<freejoint name="plate_free"/>`).
PROP_BODY_NAMES = ("plate", "mug", "fork", "spoon", "water_bottle")

# Correction (2)/(4) inputs: randomize x,y only, over a 0.30m x 0.30m square
# centred on the table's own origin -- z is never touched here (read live
# per correction 3 instead, see main()).
RANDOMIZATION_RANGE_M = {"x": (-0.15, 0.15), "y": (-0.15, 0.15)}

# Per-prop planar "footprint radius" (m): the farthest any of that prop's own
# geoms reaches from its body origin in the xy-plane, read off
# `scripts/gen_dual_scene.py`'s geom sizes/fromtos (not measured empirically):
#   plate  : PLATE_DISH_RADIUS_M = 0.06 m (the larger of its two stacked geoms)
#   mug    : mug_handle capsule reaches x=0.06 m from the body origin
#   fork   : fork_head box reaches 0.055 + 0.025 = 0.08 m
#   spoon  : spoon_bowl ellipsoid reaches 0.05 + 0.018 = 0.068 m, rounded up
#   bottle : water_bottle_body cylinder radius 0.03 m (the cap sits higher in
#            z, not wider in xy, so it does not add planar extent)
FOOTPRINT_RADIUS_M = {
    "plate": 0.06,
    "mug": 0.06,
    "fork": 0.08,
    "spoon": 0.07,
    "water_bottle": 0.03,
}

# Extra gap enforced ON TOP OF the two radii being summed, so an accepted
# layout has props visibly separated in the rendered image rather than just
# barely not touching. 0.015 m is a chosen render-legibility margin (about
# half the smallest prop radius, water_bottle's 0.03 m) -- not a physics
# tolerance, since no simulation step runs between placement and render.
# (Reduced from an initial 0.02 m -- see PER_PROP_ATTEMPTS's comment below
# for why 0.02 combined with a full-joint-draw rejection scheme measurably
# failed on bm-ptl's first real run.)
CLEARANCE_MARGIN_M = 0.015

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
# props already placed, largest footprint first -- turns this into five
# easier 1-point rejection problems instead of one hard 5-point joint one,
# and is what is actually implemented below.
PLACEMENT_ORDER = ("fork", "spoon", "plate", "mug", "water_bottle")  # largest radius first
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


def _min_dist(a: str, b: str) -> float:
    return FOOTPRINT_RADIUS_M[a] + FOOTPRINT_RADIUS_M[b] + CLEARANCE_MARGIN_M


def _try_place_all(rng: np.random.Generator) -> tuple[dict[str, tuple[float, float]], int] | None:
    """One attempt at placing every prop in `PLACEMENT_ORDER`, sequentially,
    each rejected against only the props already placed so far. Returns
    (positions, total_draws_used) on success, or None if any single prop
    exhausts `PER_PROP_ATTEMPTS` draws without finding a valid spot (the
    caller retries the whole sample from scratch -- see
    `sample_nonoverlapping_positions`).
    """
    placed: dict[str, tuple[float, float]] = {}
    total_draws = 0
    for name in PLACEMENT_ORDER:
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
) -> tuple[dict[str, tuple[float, float]], int]:
    """Place all five props (order: `PLACEMENT_ORDER`, largest footprint
    first) with no pair closer than its combined footprint + clearance,
    using SEQUENTIAL per-prop rejection sampling (see `PLACEMENT_ORDER`'s
    comment for why this replaced an earlier full-joint-draw scheme that
    measurably failed on the first real bm-ptl run).

    Returns (positions, total_draws_used_across_all_restarts). Raises
    RuntimeError if `MAX_SAMPLE_RESTARTS` whole-sample restarts are
    exhausted -- not expected in practice (a valid 5-prop layout inside the
    0.30 m x 0.30 m box demonstrably exists, e.g. four corners + a centre
    point at radius ~0.17 m from each corner), but guarded rather than
    looping forever.
    """
    total_draws = 0
    for _restart in range(1, MAX_SAMPLE_RESTARTS + 1):
        result = _try_place_all(rng)
        if result is not None:
            positions, draws_this_restart = result
            return positions, total_draws + draws_this_restart
        total_draws += PER_PROP_ATTEMPTS  # at least this many draws were burned before giving up

    raise RuntimeError(
        f"could not find a non-overlapping 5-prop layout after {MAX_SAMPLE_RESTARTS} "
        f"whole-sample restarts ({total_draws} total draws)"
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
    t_start = time.time()

    for i in range(args.count):
        # Full reset every sample: restores arm "home" pose and every prop's
        # own resting (x, y, z, quat) BEFORE this sample's x,y overwrite --
        # cameras=None here so reset() itself does not render (props are not
        # yet at their randomized positions).
        env.reset(seed=args.env_reset_seed, cameras=None)

        positions, attempts = sample_nonoverlapping_positions(rng)
        attempts_log.append(attempts)

        for name, (x, y) in positions.items():
            adr = prop_qpos_adr[name]
            data.qpos[adr] = x       # overwrite x only
            data.qpos[adr + 1] = y   # overwrite y only -- z (adr+2) and the
            # quaternion (adr+3:adr+7) are left exactly as reset() set them.
        mujoco.mj_forward(model, data)  # recompute data.xpos etc. from the new qpos

        img = env.render(CAMERA)  # escape hatch; (224, 224, 3) uint8

        sample_id = f"sample_{i:05d}"
        write_png(images_dir / f"{sample_id}.png", img)

        objects: dict[str, dict] = {}
        for name in PROP_BODY_NAMES:
            xyz = data.xpos[body_ids[name]]
            objects[name] = {
                "xyz_m": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                "randomized_xy_m": [positions[name][0], positions[name][1]],
                "z_fixed_to_own_resting_height": True,
            }

        label = {
            "sample_id": sample_id,
            "sample_index": i,
            "env_reset_seed": args.env_reset_seed,
            "rng_seed": args.seed,
            "camera": CAMERA,
            "resolution": [RENDER_W, RENDER_H],
            "image_path": f"images/{sample_id}.png",
            "placement_draws": attempts,
            "objects": objects,
        }
        (labels_dir / f"{sample_id}.json").write_text(json.dumps(label, indent=2))

        if (i + 1) % args.progress_every == 0 or (i + 1) == args.count:
            elapsed = time.time() - t_start
            rate = (i + 1) / elapsed if elapsed > 0 else float("nan")
            print(f"[{i + 1}/{args.count}] elapsed={elapsed:.1f}s rate={rate:.2f} samples/s")

    wall_clock_s = time.time() - t_start
    env.close()

    scene_sha256 = hashlib.sha256(SCENE_PATH.read_bytes()).hexdigest()
    samples_per_second = (args.count / wall_clock_s) if wall_clock_s > 0 else None

    meta = {
        "sample_count": args.count,
        "resolution": [RENDER_W, RENDER_H],
        "camera": CAMERA,
        "rng_seed": args.seed,
        "env_reset_seed": args.env_reset_seed,
        "randomization_range_m": RANDOMIZATION_RANGE_M,
        "props_randomized": list(PROP_BODY_NAMES),
        "resting_z_m": resting_z_m,
        "footprint_radius_m": FOOTPRINT_RADIUS_M,
        "clearance_margin_m": CLEARANCE_MARGIN_M,
        "placement_order": list(PLACEMENT_ORDER),
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
        "note": (
            "Object x,y positions are randomized independently per sample within "
            "randomization_range_m, subject to pairwise non-overlap rejection "
            "sampling (see footprint_radius_m / clearance_margin_m). z is held "
            "fixed per-prop at its own resting height (resting_z_m) -- never at "
            "one shared table-surface value. Randomized layouts are NOT validated "
            "or claimed as physically reachable/solvable scene configurations: "
            "ADR-038 found that moving even a single prop can break the handoff "
            "skill at phase 3. This dataset trains a perception model (PoseNet) "
            "only; no manipulation skill is ever executed against any sampled "
            "layout in this script."
        ),
    }
    (args.out_dir / "dataset_meta.json").write_text(json.dumps(meta, indent=2))

    print(f"Wrote {args.count} samples to {images_dir} and {labels_dir}")
    print(f"Wall clock: {wall_clock_s:.2f}s ({samples_per_second:.2f} samples/s)" if samples_per_second else f"Wall clock: {wall_clock_s:.2f}s")
    print(f"Meta written to {args.out_dir / 'dataset_meta.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
