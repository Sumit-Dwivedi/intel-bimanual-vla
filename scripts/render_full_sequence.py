"""Render ONE continuous, fixed-camera take of the full manipulation sequence:
arm A picks the fork, hands it to arm B, and arm B places it on the table.

Why this is TWO skill calls and not three
-----------------------------------------
The obvious reading of "pick -> handoff -> place" is three calls. That exact
three-call chain is MEASURED TO FAIL on master: `scripts/chained_demo.py` runs
`run_pick`, then `run_handoff`, then `run_place`, and step 2 dies at Phase 3
(`to_arm` approach: IK residual 0.0875 m, then a cross-arm collision when
staging to y=-0.06). Step 3 is never reached (ADR-054).

It fails because the explicit leading `run_pick` is REDUNDANT: `run_handoff`'s
own **Phase 1 IS the pick** (a nested `run_pick`, with `weld` threaded through
so `from_arm`'s grasp attaches exactly as a standalone pick would). So the
single call `run_handoff(env, "B", "A", "fork", weld=weld)` already renders
picking AND transfer, and it is the configuration verified 4/4 PASS at
`frames_used=6610`.

`run_place` then behaves the same way from the other side: it picks the object
up "if not already held", and at the post-handoff state arm B IS holding the
fork, so ADR-034's `already_held` guard skips its nested pick and it proceeds
straight to APPROACH -> DESCEND -> RELEASE -> RETREAT.

Net: two calls, no redundant re-pick, one continuous episode, and every phase
a judge needs to see is on screen.

This script does NOT alter any skill's execution logic. It wraps `env.step`
with a read-only OBSERVER that renders the CURRENT state after each physics
step the skills themselves request. It never injects a step, never writes to
`data`, and never changes control, timing, or step counts.

Run on bm-ptl only (ADR-020) -- it imports `mujoco`.

  python scripts/render_full_sequence.py --probe
  python scripts/render_full_sequence.py --stride 6 --outdir full_seq_frames
"""
from __future__ import annotations

import argparse
import pathlib
import struct
import sys
import time
import zlib

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402

# run_handoff(env, to_arm, from_arm, ...) -- to_arm FIRST. An A->B handoff
# (A gives to B) is run_handoff(env, "B", "A", "fork"). Easy trap.
TO_ARM, FROM_ARM = "B", "A"
TARGET_OBJECT = "fork"
PLACE_DESTINATION = "table"

FULL_W, FULL_H = 1280, 720  # == the scene's <global offwidth/offheight>


class _FramingCheckDone(Exception):
    """Raised by the observer to abort a --max-frames framing check early.

    Only ever raised when --max-frames is set, which exists purely so a
    camera distance can be judged from the first few frames without paying
    for a full take. It unwinds through the skill call, so the run it
    aborts is INCOMPLETE and its result is meaningless -- never treat a
    capped run as a take.
    """


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


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seed", type=int, default=3)
    p.add_argument("--stride", type=int, default=6,
                   help="render every Nth physics step")
    p.add_argument("--probe", action="store_true",
                   help="run the sequence with NO rendering; report outcome "
                        "and true step counts so --stride can be computed")
    p.add_argument("--azimuth", type=float, default=130.0)
    p.add_argument("--elevation", type=float, default=-22.0)
    # 1.05, not 0.9: at 0.9 arm A's gripper is CLIPPED off the left edge at the
    # home pose (the widest pose in the take -- the arms fold inward from there,
    # so a mid-run check does not catch it). 1.20 fits with room to spare but
    # shrinks the fork, which is the actual subject, so 1.05 is the balance.
    p.add_argument("--distance", type=float, default=1.05)
    p.add_argument("--lookat", type=float, nargs=3, default=None,
                   help="default: HANDOFF_POSITION_XYZ, the transfer point")
    p.add_argument("--outdir", default="full_seq_frames")
    p.add_argument("--max-frames", type=int, default=0,
                   help="stop the run after this many frames have been "
                        "rendered (0 = no limit). Framing checks only -- a "
                        "capped run is NOT a valid take.")
    p.add_argument("--width", type=int, default=FULL_W)
    p.add_argument("--height", type=int, default=FULL_H)
    args = p.parse_args(argv)

    lookat = args.lookat if args.lookat is not None else list(sk.HANDOFF_POSITION_XYZ)

    env = TableSettingEnv(cameras=None, render_width=args.width, render_height=args.height)
    env.reset(seed=args.seed)
    weld = WeldGrasp(env)
    model, data = env.model, env.data

    renderer = None
    outdir = None
    cam = None
    if not args.probe:
        outdir = REPO_ROOT / args.outdir
        outdir.mkdir(parents=True, exist_ok=True)
        for stale in outdir.glob("frame_*.png"):
            stale.unlink()
        renderer = mujoco.Renderer(model, height=args.height, width=args.width)
        cam = mujoco.MjvCamera()
        cam.azimuth = args.azimuth
        cam.elevation = args.elevation
        cam.distance = args.distance
        cam.lookat[:] = lookat

    state = {"steps": 0, "frames": 0}
    orig_step = env.step

    def observed_step(action, cameras=None):
        out = orig_step(action, cameras=cameras)
        state["steps"] += 1
        if renderer is not None and state["steps"] % args.stride == 0:
            renderer.update_scene(data, camera=cam)
            px = np.ascontiguousarray(renderer.render(), dtype=np.uint8)
            write_png(outdir / ("frame_%05d.png" % state["frames"]), px)
            state["frames"] += 1
            if args.max_frames and state["frames"] >= args.max_frames:
                raise _FramingCheckDone()
        return out

    env.step = observed_step  # instance attribute shadows the bound method

    print("seed=%d stride=%d probe=%s camera=(az=%s, el=%s, d=%s, lookat=%s)" % (
        args.seed, args.stride, args.probe, args.azimuth, args.elevation,
        args.distance, [round(v, 4) for v in lookat]), flush=True)

    t0 = time.time()
    try:
        r1 = sk.run_handoff(env, TO_ARM, FROM_ARM, TARGET_OBJECT, weld=weld)
    except _FramingCheckDone:
        print("FRAMING_CHECK_ABORTED after %d frames (%d steps) -- NOT a take"
              % (state["frames"], state["steps"]))
        if renderer is not None:
            renderer.close()
        return 0
    s_after_handoff = state["steps"]
    print("[1] run_handoff(env, %r, %r, %r) -> success=%s frames_used=%s steps_so_far=%d" % (
        TO_ARM, FROM_ARM, TARGET_OBJECT, r1.success, r1.frames_used, s_after_handoff), flush=True)
    print("    reason: %s" % r1.reason, flush=True)
    print("    weld holding after handoff: A=%s B=%s" % (
        weld.is_holding("A"), weld.is_holding("B")), flush=True)

    r2 = None
    if r1.success:
        r2 = sk.run_place(env, TO_ARM, TARGET_OBJECT, PLACE_DESTINATION, weld=weld)
        print("[2] run_place(env, %r, %r, %r) -> success=%s frames_used=%s steps_so_far=%d" % (
            TO_ARM, TARGET_OBJECT, PLACE_DESTINATION, r2.success, r2.frames_used,
            state["steps"]), flush=True)
        print("    reason: %s" % r2.reason, flush=True)
    else:
        print("[2] run_place SKIPPED -- handoff did not succeed", flush=True)

    total = state["steps"]
    print("TOTAL_STEPS=%d" % total)
    print("HANDOFF_STEPS=%d" % s_after_handoff)
    print("PLACE_STEPS=%d" % (total - s_after_handoff))
    print("FRAMES_WRITTEN=%d" % state["frames"])
    print("SEQUENCE_OK=%s" % bool(r1.success and r2 is not None and r2.success))
    print("elapsed_s=%.1f" % (time.time() - t0))

    if renderer is not None:
        renderer.close()
        print("at 30 fps that is %.1f s" % (state["frames"] / 30.0))
    else:
        for target_s in (30, 45):
            need = target_s * 30
            print("for %ds @30fps -> stride %d" % (target_s, max(1, round(total / need))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
