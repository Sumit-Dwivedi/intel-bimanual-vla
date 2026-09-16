"""Orchestrator's INDEPENDENT five-criterion scene-integrity monitor.

Written so Stage 4's claims can be checked with an instrument Stage 4 did not
write, and so MASTER's skills can be scored against the SAME criteria to give
the Stage 5 comparison table a real baseline.

Criteria (from the v2 brief, Stage 4):
  (b) no non-target prop moved > 5 mm from its start position
  (c) no arm-vs-non-target-prop contact with penetration > 1 mm, at ANY step
  (d) no cross-arm contact at any step
  (e) peak joint velocity below a stated threshold

The monitor wraps env.step as a read-only observer: it never injects a step,
never writes to data, and never changes control, timing or step counts.

Usage:
  python criteria_monitor.py master       # score skills_scripted (baseline)
  python criteria_monitor.py v2           # score skills_v2
"""
import pathlib
import sys

REPO = pathlib.Path.cwd()
assert (REPO / "src").is_dir(), "run from the repo root"
sys.path.insert(0, str(REPO / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
VEL_GATE = 2.6  # rad/s -- see report footer for the derivation
PROP_MOVE_GATE = 0.005
PEN_GATE = 0.001


def free_joint_bodies(model):
    """Every body with a free joint == a movable prop."""
    out = []
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            b = model.jnt_bodyid[j]
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            if nm:
                out.append((nm, b))
    return out


class Monitor:
    def __init__(self, env, target_body: str):
        self.env = env
        self.model = env.model
        self.data = env.data
        self.target = target_body
        self.props = free_joint_bodies(self.model)
        self.start = {nm: np.array(self.data.xpos[b]) for nm, b in self.props}
        self.arm_adrs = []
        for arm in ("A", "B"):
            for j in JOINTS:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                        "arm%s_%s" % (arm, j))
                if jid >= 0:
                    self.arm_adrs.append(self.model.jnt_qposadr[jid])
        self.prev_q = np.array([self.data.qpos[a] for a in self.arm_adrs])
        self.dt = float(self.model.opt.timestep)

        self.steps = 0
        self.peak_vel = 0.0
        self.cross_arm_steps = 0
        self.cross_arm_worst = 0.0
        self.prop_hits = {}      # prop -> worst penetration
        self.orig_step = env.step
        env.step = self._observed

    def _observed(self, action, cameras=None):
        out = self.orig_step(action, cameras=cameras)
        self.steps += 1
        d, m = self.data, self.model

        q = np.array([d.qpos[a] for a in self.arm_adrs])
        self.peak_vel = max(self.peak_vel, float(np.max(np.abs(q - self.prev_q)) / self.dt))
        self.prev_q = q

        for i in range(d.ncon):
            c = d.contact[i]
            n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1]) or ""
            n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2]) or ""
            a1, a2 = "arm" in n1, "arm" in n2
            if (("armA" in n1 and "armB" in n2) or ("armB" in n1 and "armA" in n2)):
                self.cross_arm_steps += 1
                self.cross_arm_worst = min(self.cross_arm_worst, float(c.dist))
            elif a1 ^ a2:
                other = n2 if a1 else n1
                if other != self.target and any(other == nm for nm, _ in self.props):
                    if c.dist < -PEN_GATE:
                        self.prop_hits[other] = min(self.prop_hits.get(other, 0.0),
                                                    float(c.dist))
        return out

    def stop(self):
        self.env.step = self.orig_step

    def report(self):
        moved = {}
        for nm, b in self.props:
            if nm == self.target:
                continue
            dist = float(np.linalg.norm(np.array(self.data.xpos[b]) - self.start[nm]))
            if dist > PROP_MOVE_GATE:
                moved[nm] = dist
        return dict(steps=self.steps, peak_vel=self.peak_vel,
                    cross_arm_steps=self.cross_arm_steps,
                    cross_arm_worst=self.cross_arm_worst,
                    prop_hits=dict(self.prop_hits), moved=moved,
                    b=not moved, c=not self.prop_hits,
                    d=self.cross_arm_steps == 0, e=self.peak_vel < VEL_GATE)


def show(label, a_ok, a_txt, r):
    verdict = "PASS" if (a_ok and r["b"] and r["c"] and r["d"] and r["e"]) else "FAIL"
    print("\n%s  ->  %s   (%d steps)" % (label, verdict, r["steps"]))
    print("   (a) target outcome      : %s   %s" % ("OK " if a_ok else "BAD", a_txt))
    print("   (b) props moved <=5mm   : %s" % ("OK " if r["b"] else "BAD"), end="")
    print("   " + (", ".join("%s moved %.4f m" % (k, v) for k, v in sorted(
        r["moved"].items(), key=lambda kv: -kv[1])) if r["moved"] else ""))
    print("   (c) no arm-prop contact : %s" % ("OK " if r["c"] else "BAD"), end="")
    print("   " + (", ".join("%s pen %.4f m" % (k, v) for k, v in sorted(
        r["prop_hits"].items(), key=lambda kv: kv[1])) if r["prop_hits"] else ""))
    print("   (d) no cross-arm contact: %s   %s" % (
        "OK " if r["d"] else "BAD",
        "" if r["d"] else "%d contact-steps, worst %.4f m" % (
            r["cross_arm_steps"], r["cross_arm_worst"])))
    print("   (e) peak joint velocity : %s   %.3f rad/s (gate %.1f)" % (
        "OK " if r["e"] else "BAD", r["peak_vel"], VEL_GATE))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "master"
    if which == "master":
        from bimanual.control import skills_scripted as sk
        title = "MASTER (skills_scripted.py) -- BASELINE"
    else:
        from bimanual.control import skills_v2 as sk
        title = "v2 (skills_v2.py)"

    print("=" * 74)
    print("INDEPENDENT FIVE-CRITERION SCORING -- %s" % title)
    print("=" * 74)

    cases = [
        ("pick(A, fork)", "fork",
         lambda e, w: sk.run_pick(e, "A", "fork", weld=w)),
        ("place(A, fork, table)", "fork",
         lambda e, w: sk.run_place(e, "A", "fork", "table", weld=w)),
        ("handoff(A->B, fork)", "fork",
         lambda e, w: sk.run_handoff(e, "B", "A", "fork", weld=w)),
    ]

    for label, target, fn in cases:
        env = TableSettingEnv(cameras=None)
        env.reset(seed=0)
        weld = WeldGrasp(env)
        mon = Monitor(env, target)
        try:
            res = fn(env, weld)
            a_ok, a_txt = bool(res.success), (res.reason or "")[:110]
        except Exception as exc:  # a crash is a failure, recorded not hidden
            a_ok, a_txt = False, "EXCEPTION: %r" % (exc,)
        mon.stop()
        show(label, a_ok, a_txt, mon.report())

    print("\n" + "-" * 74)
    print("(e) gate derivation: largest single-joint delta any phase commands is")
    print("    ~3.4 rad (wrist_roll full sweep); cubic peak = 1.5*delta/T, so at")
    print("    T=2.0 s that is 2.55 rad/s. Gate set at 2.6 rad/s -- above every")
    print("    legitimate commanded move, and far below the 5.350 rad/s measured")
    print("    for a one-shot direct position command (Stage 2).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
