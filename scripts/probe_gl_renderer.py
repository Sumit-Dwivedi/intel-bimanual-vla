"""Diagnostic: which OpenGL implementation is MuJoCo actually rendering with?

Motivation. Tester measured ~157 ms per 640x480 camera render on bm-ptl
(Arc B390 iGPU, 80-geom scene). That is roughly two orders of magnitude slower
than an iGPU should manage, which raises the question of whether the offscreen
context is hardware-accelerated at all or has silently fallen back to a software
rasterizer. MuJoCo on Windows renders through WGL, and an SSH session has no
interactive desktop, so a CPU fallback is a plausible explanation.

This matters beyond speed: the submission claims Intel device utilization
(20 rubric points), and the demo video plus any vision-based policy both run
through this path.

Read-only. Creates a GL context, prints the driver strings, exits.
"""

import sys
import traceback

import mujoco

SCENE = r"src/bimanual/sim/assets/so101_dual_table.xml"


def make_context(width=640, height=480):
    """Create a MuJoCo GL context, tolerating either API spelling."""
    attempts = []
    for label, factory in (
        ("mujoco.GLContext", lambda: mujoco.GLContext(width, height)),
        ("mujoco.gl_context.GLContext",
         lambda: __import__("mujoco.gl_context", fromlist=["GLContext"]).GLContext(width, height)),
    ):
        try:
            ctx = factory()
            print("context created via %s" % label)
            return ctx
        except Exception as exc:
            attempts.append("%s -> %r" % (label, exc))
    print("FAILED to create a GL context. Attempts:")
    for a in attempts:
        print("  ", a)
    return None


print("mujoco version:", mujoco.__version__)
print()

ctx = make_context()
if ctx is None:
    sys.exit(2)

try:
    ctx.make_current()
except Exception:
    traceback.print_exc()
    print("FAIL: context created but make_current() raised")
    sys.exit(3)

try:
    from OpenGL import GL
except ImportError:
    print("FAIL: PyOpenGL not installed in this interpreter.")
    print("Install with: python -m pip install pyopengl")
    sys.exit(4)


def gl_str(name):
    try:
        raw = GL.glGetString(getattr(GL, name))
        return raw.decode() if raw is not None else "(null)"
    except Exception as exc:
        return "(query failed: %r)" % (exc,)


print()
print("GL_VENDOR:  ", gl_str("GL_VENDOR"))
print("GL_RENDERER:", gl_str("GL_RENDERER"))
print("GL_VERSION: ", gl_str("GL_VERSION"))
print()

# Time one real render through the same path the env uses, so the driver string
# and the cost are reported from a single run rather than compared across runs.
try:
    import time
    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    r = mujoco.Renderer(model, height=480, width=640)
    r.update_scene(data)
    r.render()  # warm up: first render pays one-time setup
    n = 10
    t0 = time.perf_counter()
    for _ in range(n):
        r.update_scene(data)
        r.render()
    dt = (time.perf_counter() - t0) / n * 1000.0
    print("single-camera render, %d iterations after warmup: %.2f ms/frame" % (n, dt))
    r.close()
except Exception:
    traceback.print_exc()
    print("NOTE: driver strings above are still valid; only the timing failed.")
