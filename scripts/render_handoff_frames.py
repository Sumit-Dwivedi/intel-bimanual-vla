"""Render CANDIDATE images (image-selection only, none of these overwrite
`docs/images/m06-handoff-complete.png`) for the A->B fork handoff
(`311430e`'s ADR-037 sequential-choreography `run_handoff`), plus a 4-panel
progression strip.

**This script does not change any skill's execution logic.** It calls the
real, unmodified `run_handoff(env, to_arm, from_arm, target_object, weld=...)`
exactly as `ScriptedSkillExecutor._dispatch` would -- `to_arm` FIRST, i.e. an
A->B handoff is `run_handoff(env, "B", "A", "fork", weld=weld)`. The only
thing added is an OBSERVER: `env.step` is wrapped so that, after each real
physics step `run_handoff` itself requests (never a step this script injects),
it checks read-only state (`WeldGrasp.is_holding`, site positions) and -- the
first time each of four milestones becomes true -- stores a full physics-state
snapshot (`mujoco.mjtState.mjSTATE_FULLPHYSICS`, the exact spec
`TableSettingEnv.get_state` already uses). No control, timing, or step count
of the real run is altered by the wrapper; it only reads.

Milestones:
  1. arm A holding the fork, lifted clear of the table (Phase 1).
  2. arm A parked at the transfer point, fork still only in A's grip
     (Phase 2/3 -- `from_arm` is frozen here for up to 3000 frames, so any
     point in that window is representative).
  3. arm B's weld first reports holding the fork (Phase 4 grip).
  4. the frame `run_handoff` itself returns on (Phase 5 complete, the
     success/failure state) -- the candidate-image and final-panel state.

Snapshots are restored one at a time, AFTER `run_handoff` has already
returned, purely to pick a camera and render -- this never feeds back into
the choreography and does not re-run any physics beyond `mj_forward`'s
derived-quantity recompute.

Cameras: `overhead` is a real named camera already in the compiled scene
(`scripts/gen_dual_scene.py`, untouched by this script). The brief's side/
3-4 angles do not exist as named cameras -- built here as a plain
`mujoco.MjvCamera()` (azimuth/elevation/distance/lookat), MuJoCo's ordinary
"free camera" object, which requires no scene edit.

PNG writing is stdlib-only (zlib + a hand-rolled IHDR/IDAT/IEND), copied from
`scripts/probe_render.py`'s own `write_png` -- Pillow/imageio are not in
`scripts/requirements-bmptl.txt`, and this script does not add them (checked:
`ModuleNotFoundError` for PIL on bm-ptl's `ov_env` at the time of writing).
The 4-panel strip is glued with plain `numpy.hstack`, and each panel gets a
stdlib/numpy-only "digit stamp" (a 7-segment-style 1/2/3/4 drawn as filled
rectangles) in its top-left corner -- ordering (left to right) plus the
digit stamp is this script's answer to "label or order so the progression is
legible" without adding a text-rendering dependency.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
mujoco, which cannot load on the developer's Windows laptop.
"""

from __future__ import annotations

import pathlib
import struct
import sys
import zlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402

SEED = 0
REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "docs" / "images"
FULL_W, FULL_H = 1280, 720
PANEL_W, PANEL_H = 640, 480

TARGET_OBJECT = "fork"
TO_ARM, FROM_ARM = "B", "A"  # run_handoff(env, to_arm, from_arm, ...) -- to_arm FIRST
# How many physics steps after from_arm first arrives at the transfer point
# (still holding the object, to_arm not yet gripping) to wait before
# snapshotting milestone 2 -- comfortably under the ~3000-frame idle window
# ADR-037 measured for this phase, so the captured frame is a distinct
# "parked, waiting" instant rather than landing immediately next to
# milestone 3 (measured on this run's own first attempt: 1 frame apart).
M2_OFFSET_FRAMES = 2500

# Fix B (demo motion clip): number of frames to sample, evenly spaced by
# physics-step index, across the "interesting span" -- from milestone 1
# (arm A holding the fork, lifted) through milestone 4 (run_handoff's own
# return / retreat-complete) -- rather than across all ~6610 steps of the
# run, most of which is A's initial approach to the fork (not part of the
# handoff itself). 30 frames @ 15 fps (see ffmpeg step) is a 2-second clip.
CLIP_N_FRAMES = 30
CLIP_OUT_DIR = REPO_ROOT / "clip_frames_tmp"


def write_png(path: pathlib.Path, rgb: np.ndarray) -> None:
    """Write an HxWx3 uint8 array as a PNG using only the standard library
    (verbatim copy of `scripts/probe_render.py`'s own `write_png`)."""
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


def restore_snapshot(model, data, snapshot: np.ndarray) -> None:
    """Restore a full-physics snapshot (`mujoco.mjtState.mjSTATE_FULLPHYSICS`,
    the same spec `TableSettingEnv.get_state` uses) into `data` and recompute
    derived quantities (xpos/site_xpos/...) via `mj_forward`."""
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    mujoco.mj_setState(model, data, snapshot, spec)
    mujoco.mj_forward(model, data)


def render_current(model, data, camera, width: int, height: int) -> np.ndarray:
    """Render one frame of `data`'s CURRENT state (caller must have already
    restored whatever snapshot it wants via `restore_snapshot`).

    `camera`: a camera NAME string (named camera, e.g. "overhead") or a
    dict with azimuth/elevation/distance/lookat for a programmatic
    `mujoco.MjvCamera` free camera.
    """
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        if isinstance(camera, str):
            renderer.update_scene(data, camera=camera)
        else:
            cam = mujoco.MjvCamera()
            cam.azimuth = camera["azimuth"]
            cam.elevation = camera["elevation"]
            cam.distance = camera["distance"]
            cam.lookat[:] = camera["lookat"]
            renderer.update_scene(data, camera=cam)
        pixels = renderer.render()
    finally:
        renderer.close()
    return np.ascontiguousarray(pixels, dtype=np.uint8)


def render_snapshot(model, data, snapshot: np.ndarray, camera, width: int, height: int) -> np.ndarray:
    """Convenience: `restore_snapshot` + `render_current` in one call."""
    restore_snapshot(model, data, snapshot)
    return render_current(model, data, camera, width, height)


# --- stdlib/numpy-only digit stamp (1-4), 7-segment style -----------------
_SEGMENTS = {
    "1": {"top_right", "bottom_right"},
    "2": {"top", "top_right", "middle", "bottom_left", "bottom"},
    "3": {"top", "top_right", "middle", "bottom_right", "bottom"},
    "4": {"top_left", "top_right", "middle", "bottom_right"},
}


def digit_stamp(img: np.ndarray, digit: str, x0: int = 20, y0: int = 20, w: int = 50, h: int = 84) -> None:
    """Draw a 7-segment-style digit (mutates `img` in place) plus a dark
    plate behind it so it reads against a busy scene, without any font or
    Pillow/imageio dependency."""
    t = max(6, w // 6)
    plate_pad = 12
    y1, x1 = y0 + h, x0 + w
    img[
        max(0, y0 - plate_pad):min(img.shape[0], y1 + plate_pad),
        max(0, x0 - plate_pad):min(img.shape[1], x1 + plate_pad),
    ] = (20, 20, 20)

    def bar(y_a, y_b, x_a, x_b):
        img[max(0, y_a):min(img.shape[0], y_b), max(0, x_a):min(img.shape[1], x_b)] = (255, 210, 0)

    segs = _SEGMENTS[digit]
    mid_y = y0 + h // 2
    if "top" in segs:
        bar(y0, y0 + t, x0, x1)
    if "bottom" in segs:
        bar(y1 - t, y1, x0, x1)
    if "middle" in segs:
        bar(mid_y - t // 2, mid_y + t // 2, x0, x1)
    if "top_left" in segs:
        bar(y0, mid_y, x0, x0 + t)
    if "top_right" in segs:
        bar(y0, mid_y, x1 - t, x1)
    if "bottom_left" in segs:
        bar(mid_y, y1, x0, x0 + t)
    if "bottom_right" in segs:
        bar(mid_y, y1, x1 - t, x1)


def build_strip(panels: list[np.ndarray], sep_px: int = 6) -> np.ndarray:
    h = panels[0].shape[0]
    sep = np.full((h, sep_px, 3), 0, dtype=np.uint8)
    pieces = []
    for i, p in enumerate(panels):
        if i > 0:
            pieces.append(sep)
        pieces.append(p)
    return np.hstack(pieces)


def main() -> int:
    env = TableSettingEnv(cameras=None, render_width=FULL_W, render_height=FULL_H)
    env.reset(seed=SEED)
    weld = WeldGrasp(env)

    model, data = env.model, env.data
    body_id = sk._body_id(model, sk.OBJECT_BODY_NAME[TARGET_OBJECT])
    site_a = sk._site_id(model, ik.gripperframe_site_name("A"))
    transfer_point = np.array(sk.HANDOFF_POSITION_XYZ, dtype=np.float64)

    capture_clip = "--clip" in sys.argv

    milestones: dict[str, dict] = {}
    frame_counter = {"n": 0}
    # Fix B: every physics-step snapshot from milestone 1 onward, ONLY when
    # --clip is requested (state vectors are small, but there's no reason to
    # pay this memory/copy cost on ordinary candidate-image runs). Keyed by
    # frame index so the post-run sampler below can pick ~CLIP_N_FRAMES of
    # them evenly spaced between m1 and m4 without having known m4's frame
    # index in advance (m4 is only known once run_handoff returns).
    clip_span_states: dict[int, np.ndarray] = {}
    original_step = env.step

    def observing_step(*args, **kwargs):
        result = original_step(*args, **kwargs)
        frame_counter["n"] += 1
        n = frame_counter["n"]
        holding_a = weld.is_holding("A")
        holding_b = weld.is_holding("B")
        fork_z = float(data.xpos[body_id][2])

        if "m1_holding_fork" not in milestones and holding_a == TARGET_OBJECT and (
            fork_z > sk.TABLE_SURFACE_Z + sk.PICK_LIFT_MARGIN_M
        ):
            milestones["m1_holding_fork"] = {"frame": n, "state": env.get_state().copy()}

        if capture_clip and "m1_holding_fork" in milestones:
            # Recorded AFTER the m1 check above so frame n == m1's own frame
            # is included (dict order doesn't matter here, only membership).
            clip_span_states[n] = env.get_state().copy()

        if (
            "m2_at_transfer" not in milestones
            and "m1_holding_fork" in milestones
            and holding_a == TARGET_OBJECT
            and holding_b is None
        ):
            # from_arm sits frozen, holding the object, for a long window
            # here (Phase 2 arrival through the end of Phase 3, up to ~3000
            # frames per ADR-037's own measurement) before to_arm ever
            # grips. A first attempt captured the LAST qualifying frame
            # (i.e. the instant immediately before m3) -- measured, on this
            # run, to land only 1 physics step before m3 (frame 5310 vs
            # 5311), which rendered as a visually indistinguishable
            # duplicate of panel 3. Fixed by tracking when the window
            # STARTS (`m2_window_start`) and deliberately capturing a frame
            # a fixed, comfortably-within-window offset later
            # (`M2_OFFSET_FRAMES`, chosen well under the ~3000-frame window
            # this run measured), so panel 2 reads as its own distinct
            # "parked, waiting" instant rather than a near-duplicate of
            # either neighbour.
            if "m2_window_start" not in milestones:
                milestones["m2_window_start"] = {"frame": n}
            # Always keep a fallback snapshot of the LATEST qualifying frame
            # too, in case the window turns out shorter than M2_OFFSET_FRAMES
            # (would otherwise leave m2 uncaptured).
            milestones["m2_fallback_latest"] = {"frame": n, "state": env.get_state().copy()}
            if n - milestones["m2_window_start"]["frame"] >= M2_OFFSET_FRAMES:
                milestones["m2_at_transfer"] = {"frame": n, "state": env.get_state().copy()}

        if "m3_b_taking" not in milestones and holding_b == TARGET_OBJECT:
            milestones["m3_b_taking"] = {"frame": n, "state": env.get_state().copy()}

        return result

    env.step = observing_step
    result = sk.run_handoff(env, TO_ARM, FROM_ARM, TARGET_OBJECT, weld=weld)
    env.step = original_step  # restore, purely for hygiene -- nothing calls env.step after this point
    milestones["m4_final"] = {"frame": frame_counter["n"], "state": env.get_state().copy()}

    if "m2_at_transfer" not in milestones and "m2_fallback_latest" in milestones:
        # The "parked, waiting" window turned out shorter than
        # M2_OFFSET_FRAMES on this run -- fall back to the latest
        # qualifying frame rather than leaving milestone 2 uncaptured.
        milestones["m2_at_transfer"] = milestones["m2_fallback_latest"]
        print(f"  NOTE: m2_at_transfer fell back to the latest qualifying frame (window shorter than {M2_OFFSET_FRAMES} steps)")

    print("run_handoff(env, %r, %r, %r, weld=weld):" % (TO_ARM, FROM_ARM, TARGET_OBJECT))
    print(f"  success={result.success}")
    print(f"  reason={result.reason}")
    print(f"  frames_used={result.frames_used}")
    for name in ("m1_holding_fork", "m2_at_transfer", "m3_b_taking", "m4_final"):
        if name in milestones:
            print(f"  {name}: frame_index={milestones[name]['frame']}")
        else:
            print(f"  {name}: NEVER REACHED")

    if not result.success:
        print("run_handoff did NOT succeed -- refusing to render candidate images of a failed run.")
        env.close()
        return 1

    # Recompute arm-A separation directly from the final snapshot, the same
    # quantity run_handoff's own reason string reports (from_arm_retreat_dist),
    # independently re-derived here rather than parsed out of the string.
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    mujoco.mj_setState(model, data, milestones["m4_final"]["state"], spec)
    mujoco.mj_forward(model, data)
    site_a_final = np.array(data.site_xpos[site_a], dtype=np.float64, copy=True)
    from_arm_separation = float(np.linalg.norm(site_a_final - transfer_point))
    print(f"  measured from_arm (A) separation from transfer_point at m4_final: {from_arm_separation:.4f} m")
    site_b_id = sk._site_id(model, ik.gripperframe_site_name("B"))
    site_b_final = np.array(data.site_xpos[site_b_id], dtype=np.float64, copy=True)
    fork_pos_final = np.array(data.xpos[body_id], dtype=np.float64, copy=True)
    print(f"  DIAG site_a_final(xyz)={site_a_final} site_b_final(xyz)={site_b_final} fork_pos_final(xyz)={fork_pos_final} armA_to_armB_dist={np.linalg.norm(site_a_final-site_b_final):.4f}")
    # ADR-038 fix 2 (lateral retreat): the metric this task's own risk
    # section asks be REPORTED, not assumed -- the achieved LATERAL (y)
    # gripper separation at the final frame, a number, not a pass/fail.
    lateral_y_separation = float(abs(site_a_final[1] - site_b_final[1]))
    print(f"  ADR-038 fix 2 measured: final-frame LATERAL (y) gripper separation = {lateral_y_separation:.4f} m")

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    if capture_clip:
        # Fix B: render ~CLIP_N_FRAMES PNG frames, evenly spaced by physics-
        # step index across [m1_holding_fork, m4_final], using CANDIDATE 1's
        # exact camera (azimuth=20, elevation=-10, distance=0.35, lookat=the
        # midpoint between the two arms' FINAL gripper sites) -- the same
        # camera that produced docs/images/m06-handoff-complete.png, chosen
        # there because the red fork and the empty gripper both read
        # clearly. Reused verbatim (not recomputed per-frame) so the clip is
        # a fixed reference frame, same rationale as the 4-panel strip's
        # cam_fixed_sequence above: a moving camera would make the fork
        # appear to "jump" even when nothing physically jumped.
        clip_mid = (site_a_final + site_b_final) / 2.0
        clip_camera = {"azimuth": 20, "elevation": -10, "distance": 0.35, "lookat": list(clip_mid)}
        m1_frame = milestones["m1_holding_fork"]["frame"]
        m4_frame = milestones["m4_final"]["frame"]
        available = sorted(clip_span_states.keys())
        # Evenly spaced target frame indices across the span; snap each to
        # the nearest frame index actually recorded in clip_span_states
        # (recording started exactly at m1_frame, so this is always exact
        # for endpoints and near-exact in between).
        targets = np.linspace(m1_frame, m4_frame, CLIP_N_FRAMES)
        CLIP_OUT_DIR.mkdir(parents=True, exist_ok=True)
        # Clear any stale frames from a previous run of this script.
        for old in CLIP_OUT_DIR.glob("frame_*.png"):
            old.unlink()
        chosen_frames = []
        for i, target in enumerate(targets):
            nearest = min(available, key=lambda f: abs(f - target))
            chosen_frames.append(nearest)
            img = render_snapshot(model, data, clip_span_states[nearest], clip_camera, FULL_W, FULL_H)
            write_png(CLIP_OUT_DIR / f"frame_{i:03d}.png", img)
        print(
            f"  Fix B clip: wrote {len(targets)} frames to {CLIP_OUT_DIR} "
            f"spanning frame_index {m1_frame}..{m4_frame} "
            f"(camera={clip_camera})"
        )
        print(f"  Fix B clip: chosen frame indices = {chosen_frames}")

    if "--sweep" in sys.argv:
        # Tuning-only diagnostic: an azimuth sweep at fixed elevation/distance,
        # lookat at the midpoint between the two final gripper positions, so
        # the camera direction that best separates arm A (up-and-toward-B's
        # side, per the DIAG line above) from arm B can be picked by eye
        # rather than guessed. Not part of the shipped candidate set.
        mid = (site_a_final + site_b_final) / 2.0
        sweep_dir = OUT_DIR.parent / "sweep_tmp"
        sweep_dir.mkdir(parents=True, exist_ok=True)
        for az in (0, 20, 45, 70, 90, 110, 135, 160):
            cam = {"azimuth": az, "elevation": -5, "distance": 0.26, "lookat": list(mid)}
            img = render_snapshot(model, data, milestones["m4_final"]["state"], cam, FULL_W, FULL_H)
            write_png(sweep_dir / f"az{az:03d}.png", img)
            print(f"  wrote sweep_tmp/az{az:03d}.png")
        # Second sweep: tight close-up centred on the FORK itself (not the
        # midpoint between grippers), to positively identify which gripper
        # the fork is actually visible in/near, disambiguating it from the
        # (untouched, still on the table) spoon prop.
        for az in (0, 45, 90, 135):
            cam = {"azimuth": az, "elevation": -10, "distance": 0.15, "lookat": list(fork_pos_final)}
            img = render_snapshot(model, data, milestones["m4_final"]["state"], cam, FULL_W, FULL_H)
            write_png(sweep_dir / f"fork_az{az:03d}.png", img)
            print(f"  wrote sweep_tmp/fork_az{az:03d}.png")
        for az in (0, 15, 340):
            for dist in (0.32, 0.4):
                cam = {"azimuth": az, "elevation": -10, "distance": dist, "lookat": list(mid)}
                img = render_snapshot(model, data, milestones["m4_final"]["state"], cam, FULL_W, FULL_H)
                write_png(sweep_dir / f"wide_az{az:03d}_d{int(dist*100):03d}.png", img)
                print(f"  wrote sweep_tmp/wide_az{az:03d}_d{int(dist*100):03d}.png")
        # Disambiguation: zoom tightly on EACH arm's own gripper individually
        # (lookat = that arm's own final site position, not the midpoint),
        # labelled by arm name in the filename so there is no doubt which
        # structure in the wider shots is A and which is B.
        for arm_label, site_pos in (("A", site_a_final), ("B", site_b_final)):
            cam = {"azimuth": 20, "elevation": -10, "distance": 0.14, "lookat": list(site_pos)}
            img = render_snapshot(model, data, milestones["m4_final"]["state"], cam, FULL_W, FULL_H)
            write_png(sweep_dir / f"gripper_{arm_label}.png", img)
            print(f"  wrote sweep_tmp/gripper_{arm_label}.png")
        for dist in (0.32, 0.35, 0.38):
            cam = {"azimuth": 20, "elevation": -10, "distance": dist, "lookat": list(mid)}
            img = render_snapshot(model, data, milestones["m4_final"]["state"], cam, FULL_W, FULL_H)
            write_png(sweep_dir / f"both_az020_d{int(dist*100):03d}.png", img)
            print(f"  wrote sweep_tmp/both_az020_d{int(dist*100):03d}.png")
        env.close()
        return 0

    # --- Candidate cameras ---------------------------------------------------
    # Parameters below are NOT guesses -- they were picked from an actual
    # azimuth/elevation/distance sweep rendered and inspected on this run
    # (see the `--sweep` branch above; sweep images are not committed, only
    # the conclusion is used here), PLUS a disambiguation step: two extra
    # tight shots, each centred on ONE arm's own final gripper site only
    # (`gripper_A.png`/`gripper_B.png` in the sweep dir), confirmed which
    # structure is which in the wider shots -- gripper_A renders as a clean,
    # empty, fully OPEN claw (nothing held, as expected: A released and
    # retreated); gripper_B renders with the fork (a white cylindrical
    # sliver) visible right at its mount, confirming B is the one holding it.
    # This matters because at this zoom level the two arms' actual
    # end-effector height difference (0.114 m in z) is visually dwarfed by
    # their much taller, near-identical forearm/elbow geometry -- a viewer
    # cannot reliably read "A is higher than B" from silhouette alone, only
    # "these are two distinct, separated arms, and the one with the fork
    # visible is not the same one that is now empty and off to the side."
    # That is the honest, verified content of candidate 1 below -- NOT a
    # clean vertical up/down read, which this run's own sweep could not
    # produce at any tried angle (see the printed caveat on candidate 3).
    #   - Candidate 1: a tight shot centred on the midpoint between the two
    #     final gripper sites, azimuth=20 (chosen from the sweep so neither
    #     arm's structure is cropped out of frame), zoomed enough that the
    #     measured 0.1331 m separation and the visible fork are a real
    #     fraction of the frame, not lost in a whole-table shot.
    #   - Candidate 2: a wider 3/4 elevated hero shot (azimuth=130) -- a
    #     genuinely different angle from candidate 1's tight shot. (The
    #     4-panel sequence strip below now uses its OWN separate ADR-038
    #     fix-4 fixed side camera, not this one -- see that section's own
    #     comment.)
    #   - Candidate 3: `overhead`, the brief-mandated named camera. Per this
    #     run's own sweep and the geometry (ADR-038 fix 2's retreat now has
    #     a deliberate LATERAL component, but a straight-down camera still
    #     foreshortens the vertical component of the same retreat), a
    #     straight-down camera is not the most legible angle for this
    #     motion. Rendered anyway, per the brief, and reported honestly
    #     below rather than silently dropped.
    mid = (site_a_final + site_b_final) / 2.0
    cam_tight_separation = {"azimuth": 20, "elevation": -10, "distance": 0.35, "lookat": list(mid)}
    cam_threequarter_wide = {"azimuth": 130, "elevation": -22, "distance": 0.6, "lookat": [0.0, 0.0, 0.45]}

    candidates = {
        "m06-handoff-candidate-1.png": (
            "tight shot centred on the A/B gripper gap, angle chosen so neither arm "
            "is cropped -- the fork is visible near arm B; arm A is a distinct, "
            "separated structure, confirmed empty by a disambiguation render (see "
            "this script's own comment above); NOT a clean vertical up/down read",
            cam_tight_separation,
        ),
        "m06-handoff-candidate-2.png": (
            "wide 3/4 elevated hero shot, both arms + table in frame",
            cam_threequarter_wide,
        ),
        "m06-handoff-candidate-3.png": (
            "overhead (named camera, brief-mandated) -- SEE PRINTED CAVEAT: this "
            "angle foreshortens the (near-purely-vertical) separation",
            "overhead",
        ),
    }
    for filename, (label, cam) in candidates.items():
        img = render_snapshot(model, data, milestones["m4_final"]["state"], cam, FULL_W, FULL_H)
        write_png(OUT_DIR / filename, img)
        print(f"  wrote {filename}  ({label})  mean_pixel={img.mean():.1f}")

    # --- 4-panel sequence strip ---------------------------------------------
    # ADR-038 fix 4: ONE fixed camera (identical azimuth/elevation/distance/
    # lookat) across ALL FOUR panels -- a fixed reference frame is the whole
    # point of a progression strip: it is what lets a viewer track the SAME
    # fork moving between the two arms, panel to panel, rather than a
    # per-panel-recentred camera that would make the fork (and the arms)
    # appear to jump around the frame even when nothing "jumped" physically.
    # The previous per-panel-recomputed lookat/distance (a moving camera)
    # is replaced outright, not kept as a fallback.
    #
    # Centred on the transfer point (`transfer_point`, ADR-036's
    # `HANDOFF_POSITION_XYZ`) rather than on either arm's own gripper --
    # the transfer point is the one location common to the whole story (A
    # arrives there holding the fork, B takes it from there, both retreat
    # away from it), so it stays meaningful across all four milestones even
    # though milestone 1 (B still at HOME) and milestone 4 (both arms
    # retreated) put the arms themselves at very different places.
    #
    # Parameters, ACTUALLY VERIFIED BY INSPECTING THE RENDERED IMAGE, not
    # assumed from the task's own suggested starting point. The suggested
    # start (elevation=-15, azimuth=90, distance=1.2) was tried FIRST and
    # rendered -- inspection showed only ONE arm visible in any panel; at
    # azimuth=90 the two arms (offset only in y from a shared x~0 base
    # line) sit almost exactly in line with the viewing ray, so one
    # occludes the other instead of separating left/right as intended.
    # `m06-handoff-candidate-2.png`'s own camera (azimuth=130,
    # elevation=-22) was ALREADY confirmed (by inspection, same method) to
    # show both arms clearly separated plus the red fork -- reused here,
    # with distance widened from that candidate's 0.6 m to 0.9 m so arm B
    # (still at HOME, farther from `transfer_point`, in milestone 1) is not
    # cropped out of the first panel. Re-inspected after this change:
    # both arms visible, clearly separated, in all four panels.
    cam_fixed_sequence = {"azimuth": 130, "elevation": -22, "distance": 0.9, "lookat": list(transfer_point)}
    seq_order = [
        ("m1_holding_fork", "1"),
        ("m2_at_transfer", "2"),
        ("m3_b_taking", "3"),
        ("m4_final", "4"),
    ]
    panels = []
    for key, digit in seq_order:
        restore_snapshot(model, data, milestones[key]["state"])
        img = render_current(model, data, cam_fixed_sequence, PANEL_W, PANEL_H)
        digit_stamp(img, digit)
        panels.append(img)
    strip = build_strip(panels)
    write_png(OUT_DIR / "m06-handoff-sequence.png", strip)
    print(f"  wrote m06-handoff-sequence.png  shape={strip.shape}  camera(fixed)={cam_fixed_sequence}")

    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
