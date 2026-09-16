"""v2 Stage 5 demo: a two-arm relay, recorded in one continuous fixed-camera
take at REAL-TIME speed, with every step scored against all five
scene-integrity criteria.

WHY A RELAY AND NOT A HANDOFF
-----------------------------
`handoff(A->B, fork)` does NOT work on this branch (ADR-073): the transit
collision was solved by `HANDOFF_YAW = 3*pi/2` (cross-arm contact 129 steps ->
zero), but phase 3 still fails because the 6 cm lateral grip offset leaves the
fork's body origin 0.108 m from arm B's pinch against a ~2 cm jaw span. The
gate widening that makes it *report* success was tested and REFUSED: a per-step
contact check proved arm B's finger pads never touch any fork geom. That is a
weld across a 10.8 cm gap, not a transfer.

So the object moves from arm A to arm B **via the table**, which is a real
bimanual sequence and an honest one. The video caption must say the direct
transfer is not shown, and why.

REAL-TIME SPEED
---------------
`model.opt.timestep = 0.002 s`. At 30 fps, real time needs a stride of
1/30 / 0.002 = 16.67, so **stride 17** (1.02x real speed). The brief's
"every 6th step" would be 0.360x -- 2.8x slow motion.

Renders locally: MuJoCo 3.2.7 imports, builds the scene and renders offscreen
at 1280x720 on the laptop WHEN Windows Application Control permits it --
which is intermittent: it later blocked the same import in the same session.
ADR-020 STANDS and bm-ptl is the only reliable host; local rendering is a
convenience, not a guarantee. bm-ptl remains authoritative for reported numbers (ADR-047).

  python scripts/v2_stage5_demo.py --probe          # score only, no render
  python scripts/v2_stage5_demo.py --outdir frames  # score + render
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

from bimanual.control import ik_geometric as ikg  # noqa: E402
from bimanual.control import motion  # noqa: E402
from bimanual.control import skills_v2 as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

#: Both drops were CHOSEN BY MEASUREMENT, not guessed. A place target must keep
#: the jaw-OPENING swept volume clear of neighbouring props: the moving finger
#: pad swings out in phase 3 and catches anything close. Measured rejects:
#: (-0.05, 0.05) catches the plate rim (-0.00157 m, plate displaced 16.7 mm);
#: (0.02, -0.05) and (-0.06, -0.04) catch the mug (-0.0011 m); (-0.12, -0.03)
#: catches the plate (-0.00805 m).
DROP_A = np.array([0.00, 0.05, 0.356])
DROP_B = np.array([-0.10, -0.04, 0.356])

VEL_GATE = 2.6      # rad/s; see the ADR for the 1.5*delta/T derivation
PROP_GATE = 0.005   # m
PEN_GATE = 0.001    # m
FULL_W, FULL_H = 1280, 720


def write_png(path: pathlib.Path, rgb: np.ndarray) -> None:
    """stdlib-only PNG writer (Pillow/imageio are not installed)."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def free_joint_bodies(model):
    out = []
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            b = model.jnt_bodyid[j]
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            if nm:
                out.append((nm, b))
    return out


class Recorder:
    """Read-only observer on env.step: scores criteria (b)-(e) and renders.

    Never injects a step, never writes to `data`, never changes control,
    timing, or step counts.
    """

    def __init__(self, env, renderer=None, cam=None, outdir=None, stride=17):
        self.env, self.model, self.data = env, env.model, env.data
        self.renderer, self.cam, self.outdir, self.stride = renderer, cam, outdir, stride
        self.props = free_joint_bodies(self.model)
        self.adrs = []
        for arm in ("A", "B"):
            for j in JOINTS:
                k = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                      "arm%s_%s" % (arm, j))
                if k >= 0:
                    self.adrs.append(self.model.jnt_qposadr[k])
        self.prev_q = np.array([self.data.qpos[a] for a in self.adrs])
        self.dt = float(self.model.opt.timestep)
        self.steps = self.frames = 0
        self.worst_crop = 0
        self.worst_crop_frame = -1
        self.target = None
        self.seg_start = None
        self.peak_vel = 0.0
        self.cross = 0
        self.hits = {}
        self.orig_step = env.step
        env.step = self._observed

    def begin(self, target_body):
        """Start a new scored segment."""
        self.target = target_body
        self.seg_start = {nm: np.array(self.data.xpos[b]) for nm, b in self.props}
        self.peak_vel = 0.0
        self.cross = 0
        self.hits = {}

    def _observed(self, action, cameras=None):
        out = self.orig_step(action, cameras=cameras)
        self.steps += 1
        d, m = self.data, self.model

        q = np.array([d.qpos[a] for a in self.adrs])
        self.peak_vel = max(self.peak_vel, float(np.max(np.abs(q - self.prev_q)) / self.dt))
        self.prev_q = q

        for i in range(d.ncon):
            c = d.contact[i]
            n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1]) or ""
            n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2]) or ""
            a1, a2 = "arm" in n1, "arm" in n2
            if ("armA" in n1 and "armB" in n2) or ("armB" in n1 and "armA" in n2):
                self.cross += 1
            elif a1 ^ a2:
                other = n2 if a1 else n1
                if other != self.target and any(other == nm for nm, _ in self.props):
                    if c.dist < -PEN_GATE:
                        self.hits[other] = min(self.hits.get(other, 0.0), float(c.dist))

        if self.renderer is not None and self.steps % self.stride == 0:
            self.renderer.update_scene(d, camera=self.cam)
            px = np.ascontiguousarray(self.renderer.render(), dtype=np.uint8)
            # Automatic crop check: arm-coloured pixels touching any border.
            # A cropped arm misrepresents the take, and eyeballing eight
            # sampled frames missed it once already.
            arm = ((px[:, :, 0] > 170) & (px[:, :, 1] > 140) & (px[:, :, 2] < 110))
            touch = int(arm[:, 0].sum() + arm[:, -1].sum() + arm[0, :].sum() + arm[-1, :].sum())
            if touch > self.worst_crop:
                self.worst_crop, self.worst_crop_frame = touch, self.frames
            write_png(self.outdir / ("frame_%05d.png" % self.frames), px)
            self.frames += 1
        return out

    def score(self):
        moved = {}
        for nm, b in self.props:
            if nm == self.target:
                continue
            dist = float(np.linalg.norm(np.array(self.data.xpos[b]) - self.seg_start[nm]))
            if dist > PROP_GATE:
                moved[nm] = dist
        return dict(moved=moved, hits=dict(self.hits), cross=self.cross,
                    peak_vel=self.peak_vel,
                    b=not moved, c=not self.hits, d=self.cross == 0,
                    e=self.peak_vel < VEL_GATE)

    def stop(self):
        self.env.step = self.orig_step


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--stride", type=int, default=17, help="17 = real time at 30 fps")
    p.add_argument("--probe", action="store_true")
    p.add_argument("--outdir", default="v2_demo_frames")
    # Low resolution is for the crop sweep only: a border pixel is a border
    # pixel at any resolution, so framing can be validated ~16x faster than
    # at full res. Never ship a low-res take.
    p.add_argument("--width", type=int, default=FULL_W)
    p.add_argument("--height", type=int, default=FULL_H)
    # Azimuth 210 / elevation -35, NOT the 130/-22 used for master's video.
    # At 130/-22 the fork is OCCLUDED at the moments that matter: invisible at
    # the drop1 release and the drop2 release, and only red fragments while
    # arm B holds it -- the arms' own bodies and the mug sit between camera and
    # action, because every action point lies at x in [-0.10, 0.00] while the
    # mug (0.05,-0.03) and bottle (0.22,0) are on the camera side. Verified by
    # inspecting 8 frames of a full render at 130/-22 before changing this.
    # From 210/-35 the props sit in front of the arms rather than behind them.
    p.add_argument("--azimuth", type=float, default=210.0)
    p.add_argument("--elevation", type=float, default=-35.0)
# 1.30 is the MINIMUM crop-free distance, found by sweeping the whole
    # take at low resolution and counting arm-coloured pixels on the frame
    # border -- 0 at 1.30, but 17 at 1.00 and still 7 at 1.12. Twice I assumed
    # the home pose was the widest and twice that was wrong: the binding frame
    # is ~20-23, mid-approach in stage 1, which no 8-frame sample would catch.
    # Hence the automatic per-frame check below rather than eyeballing.
    p.add_argument("--distance", type=float, default=1.30)
    args = p.parse_args(argv)

    env = TableSettingEnv(cameras=None, render_width=args.width, render_height=args.height)
    env.reset(seed=args.seed)
    model = env.model
    weld = WeldGrasp(env)

    home = {}
    for arm in ("A", "B"):
        home[arm] = np.array([env.data.qpos[model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "arm%s_%s" % (arm, j))]]
            for j in JOINTS])

    renderer = cam = outdir = None
    if not args.probe:
        outdir = REPO_ROOT / args.outdir
        outdir.mkdir(parents=True, exist_ok=True)
        for stale in outdir.glob("frame_*.png"):
            stale.unlink()
        renderer = mujoco.Renderer(model, height=args.height, width=args.width)
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = args.azimuth, args.elevation, args.distance
        cam.lookat[:] = [-0.04, 0.0, 0.40]

    rec = Recorder(env, renderer, cam, outdir, args.stride)

    def home_via_lift(arm, pt):
        hi = ikg.solve_topdown_ik(model, env.data, arm,
                                  np.array([pt[0], pt[1], 0.47]), yaw=3 * np.pi / 2)
        if hi is not None:
            r = motion.move_to_config(env, arm, hi, 1.0, hold_other_arm=True)
            if not r.success:
                return r
        return motion.move_to_config(env, arm, home[arm], 2.0, hold_other_arm=True)

    stages = [
        ("1 pick(A, fork)",          lambda: sk.run_pick(env, "A", "fork", weld)),
        ("2 place(A, fork, drop1)",  lambda: sk.run_place(env, "A", "fork", DROP_A, weld)),
        ("3 arm A returns home",     lambda: home_via_lift("A", DROP_A)),
        ("4 pick(B, fork)",          lambda: sk.run_pick(env, "B", "fork", weld)),
        ("5 place(B, fork, drop2)",  lambda: sk.run_place(env, "B", "fork", DROP_B, weld)),
        ("6 arm B returns home",     lambda: home_via_lift("B", DROP_B)),
    ]

    print("seed=%d stride=%d (%.3fx real speed at 30 fps) probe=%s"
          % (args.seed, args.stride, args.stride * float(model.opt.timestep) * 30.0, args.probe),
          flush=True)
    t0 = time.time()
    passes = 0
    for label, fn in stages:
        rec.begin("fork")
        s0 = rec.steps
        try:
            r = fn()
            ok, why = bool(getattr(r, "success", True)), (getattr(r, "reason", "") or "")
        except Exception as exc:
            ok, why = False, "EXCEPTION: %r" % (exc,)
        sc = rec.score()
        allfive = ok and sc["b"] and sc["c"] and sc["d"] and sc["e"]
        passes += int(allfive)
        print("%-26s %-4s  steps=%-5d (a)%s (b)%s (c)%s (d)%s (e)%s peak_vel=%.3f"
              % (label, "PASS" if allfive else "FAIL", rec.steps - s0,
                 "OK " if ok else "BAD", "OK " if sc["b"] else "BAD",
                 "OK " if sc["c"] else "BAD", "OK " if sc["d"] else "BAD",
                 "OK " if sc["e"] else "BAD", sc["peak_vel"]), flush=True)
        if not ok:
            print("      reason: %s" % why[:150], flush=True)
        if sc["moved"]:
            print("      moved: %s" % {k: round(v, 4) for k, v in sc["moved"].items()}, flush=True)
        if sc["hits"]:
            print("      hits:  %s" % {k: round(v, 5) for k, v in sc["hits"].items()}, flush=True)

    rec.stop()
    if renderer is not None:
        renderer.close()

    fb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fork")
    print("\nfork final xpos = %s" % np.round(env.data.xpos[fb], 4))
    print("RELAY_PASSED=%d/%d" % (passes, len(stages)))
    print("TOTAL_STEPS=%d  (%.1f s simulated)" % (rec.steps, rec.steps * float(model.opt.timestep)))
    print("FRAMES_WRITTEN=%d  (%.1f s at 30 fps)" % (rec.frames, rec.frames / 30.0))
    if rec.frames:
        print("WORST_BORDER_ARM_PIXELS=%d (frame %d) -> %s"
              % (rec.worst_crop, rec.worst_crop_frame,
                 "CROPPED -- widen --distance" if rec.worst_crop > 0 else "no arm cropped in any frame"))
    print("elapsed_s=%.1f" % (time.time() - t0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
