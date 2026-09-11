"""RISK-03 probe: can MuJoCo create a GL context and render offscreen on bm-ptl?

ADR-020 moved all MuJoCo work to bm-ptl and rejected the laptop as a sim host, so
there is no local fallback if offscreen rendering fails here. This script is the
blocking prerequisite that ADR-020 promoted out of M03.

It is a DIAGNOSTIC, not a deliverable. Two deliberate choices follow from that:

1. Stdlib-only PNG writing. Pillow/imageio are not imported, so a missing-image-
   library error can never be mistaken for a GL failure. The only third-party
   imports are mujoco and numpy, both already pinned in requirements-bmptl.txt.
2. Model compile and renderer construction are in separate try blocks with full
   tracebacks. A scene problem and a rendering problem must not look alike --
   that distinction is exactly what the probe exists to establish.

M02 extension: this script now takes two OPTIONAL positional argv args --
a scene path and an output PNG path -- so the same RISK-03 probe can be
reused to render the M02 dual-arm scene at 1280x720 instead of the single-arm
upstream scene at its native 640x480. Called with no arguments, behaviour is
byte-for-byte what it was before this change (scene=scenes/so101/scene.xml,
out=out_probe.png), so it remains valid RISK-03 evidence for ARCHITECTURE.md
section 5.

Usage:
    python scripts/probe_render.py                              (unchanged default)
    python scripts/probe_render.py <scene.xml> <out.png>        (M02 and later)

Exit codes:  0 rendered   2 model failed to compile   3 GL/renderer failed
"""

import os
import platform
import struct
import sys
import traceback
import zlib

SCENE = sys.argv[1] if len(sys.argv) > 1 else os.path.join("scenes", "so101", "scene.xml")
OUT = sys.argv[2] if len(sys.argv) > 2 else "out_probe.png"


def write_png(path, rgb):
    """Write an HxWx3 uint8 array as a PNG using only the standard library."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


print("Host / backend")
print("--------------")
print("  node            :", platform.node())
print("  system          :", platform.system(), platform.release())
print("  MUJOCO_GL       :", os.environ.get("MUJOCO_GL", "(unset -- MuJoCo picks its default)"))
print("  PYOPENGL_PLATFORM:", os.environ.get("PYOPENGL_PLATFORM", "(unset)"))
print()

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

print("  mujoco          :", mujoco.__version__)
print("  numpy           :", np.__version__)
print()

if not os.path.exists(SCENE):
    print("FAIL: scene not found at %s (run from the repo root)" % SCENE)
    sys.exit(2)

print("Stage 1 -- compile model (no GL involved)")
print("----------------------------------------")
try:
    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
except Exception:
    traceback.print_exc()
    print()
    print("FAIL (stage 1): the model did not compile. This is a SCENE problem,")
    print("not a rendering problem -- do not chase GL backends on this result.")
    sys.exit(2)

print("  COMPILE OK  nq=%d nv=%d nu=%d nbody=%d ngeom=%d"
      % (model.nq, model.nv, model.nu, model.nbody, model.ngeom))
print()

print("Stage 2 -- create GL context and render offscreen (the actual RISK-03 probe)")
print("---------------------------------------------------------------------------")

# MuJoCo validates the requested size against the model's offscreen framebuffer and
# raises BEFORE it ever touches GL. Asking for more than the scene declares therefore
# produces a failure that looks like a rendering problem but is not one -- it would
# send us chasing GL backends over a number in the XML. Clamp instead, and say so.
REQ_W, REQ_H = 1280, 720
fb_w = int(model.vis.global_.offwidth)
fb_h = int(model.vis.global_.offheight)
width, height = min(REQ_W, fb_w), min(REQ_H, fb_h)
print("  scene offscreen framebuffer : %dx%d" % (fb_w, fb_h))
if (width, height) != (REQ_W, REQ_H):
    print("  requested %dx%d exceeds it -- clamped to %dx%d."
          % (REQ_W, REQ_H, width, height))
    print("  Not a GL problem. To render larger, the scene XML needs:")
    print("      <visual><global offwidth=\"1280\" offheight=\"720\"/></visual>")
    print("  That belongs in our own M02 scene, not in the unmodified upstream asset (ADR-016).")
print()

try:
    renderer = mujoco.Renderer(model, height=height, width=width)
except Exception:
    traceback.print_exc()
    print()
    print("FAIL (stage 2): could not create a renderer / GL context.")
    print("The model compiled fine, so this is a RENDERING problem.")
    print("Next step per ADR-020: retry with MUJOCO_GL=osmesa, then escalate to the")
    print("WSL2 decision. Do not install WSL2 without explicit authorization.")
    sys.exit(3)

try:
    with renderer:
        renderer.update_scene(data)
        pixels = renderer.render()
    write_png(OUT, np.ascontiguousarray(pixels, dtype=np.uint8))
except Exception:
    traceback.print_exc()
    print()
    print("FAIL (stage 2): renderer constructed but rendering/writing failed.")
    sys.exit(3)

print("  RENDER OK -> %s  (%dx%d, mean pixel %.1f)"
      % (OUT, pixels.shape[1], pixels.shape[0], float(pixels.mean())))
print()
print("PASS: MuJoCo compiles and renders offscreen on this host. RISK-03 clear.")
sys.exit(0)
