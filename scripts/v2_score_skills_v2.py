"""Score skills_v2 with the ORCHESTRATOR's monitor, not the stage's own.

Same Monitor class that produced the master baseline, so one instrument scores
both stacks. Adapted only for skills_v2's real signatures, which differ from
master's in two disclosed ways:
  - `weld` is required, not optional
  - `run_place(env, arm, obj, target_xyz, weld)` takes a Cartesian target and
    does NOT nested-pick; its precondition is that the arm already holds the
    object. So a prerequisite PICK runs BEFORE the monitor attaches, and is
    therefore not scored as part of PLACE.
  - `run_handoff` likewise expects from_arm to already hold the object.
"""
import pathlib
import sys

REPO = pathlib.Path.cwd()
assert (REPO / "src").is_dir(), "run from the repo root"
sys.path.insert(0, str(REPO / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from criteria_monitor import Monitor, show, VEL_GATE  # noqa: E402

from bimanual.control import skills_v2 as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402


def fresh():
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    return env, WeldGrasp(env)


def fork_pos(env):
    b = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "fork")
    return np.array(env.data.xpos[b])


def main():
    print("=" * 74)
    print("INDEPENDENT FIVE-CRITERION SCORING -- v2 (skills_v2.py)")
    print("=" * 74)

    # ---- 1. PICK(A, fork) --------------------------------------------
    env, weld = fresh()
    mon = Monitor(env, "fork")
    try:
        r = sk.run_pick(env, "A", "fork", weld)
        ok, txt = bool(r.success), (r.reason or "")[:110]
    except Exception as exc:
        ok, txt = False, "EXCEPTION: %r" % (exc,)
    mon.stop()
    show("pick(A, fork)", ok, txt, mon.report())
    print("      weld after pick: A=%r B=%r" % (weld.is_holding("A"), weld.is_holding("B")))

    # ---- 2. PLACE(A, fork, target) -- prerequisite pick NOT scored ----
    env, weld = fresh()
    tgt = fork_pos(env).copy()
    pre = sk.run_pick(env, "A", "fork", weld)
    print("\n   [prereq] pick(A,fork) success=%s (not scored)" % pre.success)
    mon = Monitor(env, "fork")
    try:
        r = sk.run_place(env, "A", "fork", tgt, weld)
        ok, txt = bool(r.success), (r.reason or "")[:110]
    except Exception as exc:
        ok, txt = False, "EXCEPTION: %r" % (exc,)
    mon.stop()
    show("place(A, fork, %s)" % np.round(tgt, 3), ok, txt, mon.report())

    # ---- 3. HANDOFF(A->B, fork) -- prerequisite pick NOT scored -------
    env, weld = fresh()
    pre = sk.run_pick(env, "A", "fork", weld)
    print("\n   [prereq] pick(A,fork) success=%s (not scored)" % pre.success)
    mon = Monitor(env, "fork")
    try:
        r = sk.run_handoff(env, "B", "A", "fork", weld)
        ok, txt = bool(r.success), (r.reason or "")[:110]
    except Exception as exc:
        ok, txt = False, "EXCEPTION: %r" % (exc,)
    mon.stop()
    show("handoff(A->B, fork)", ok, txt, mon.report())
    print("      weld after handoff: A=%r B=%r" % (weld.is_holding("A"), weld.is_holding("B")))

    print("\n" + "-" * 74)
    print("(e) gate = %.1f rad/s. master baseline for comparison: 6.937 rad/s,"
          % VEL_GATE)
    print("    and every master skill FAILED (b)/(c)/(d) or (e) while passing (a).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
