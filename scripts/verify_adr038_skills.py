"""ADR-038 verification: all four working skills against the regenerated scene.

The scene changed (fork recoloured, plate/mug/spoon/bottle repositioned) and
`run_handoff`'s retreat became a per-arm xyz vector, so every baseline measured
before ADR-038 is stale. Each skill gets a FRESH env -- no cross-contamination.

run_handoff(env, to_arm, from_arm, obj) is receiver-first: A->B is ("B", "A").
"""
import sys
import numpy as np
sys.path.insert(0, "src")
from bimanual.sim.env import TableSettingEnv
from bimanual.sim.grasp import WeldGrasp
from bimanual.control import skills_scripted as sk


def fresh():
    env = TableSettingEnv()
    env.reset(seed=0)
    return env, WeldGrasp(env)


def obj_z(env, name):
    return float(env.data.xpos[sk._body_id(env.model, name)][2])


def pinch(env, arm):
    """ADR-025 pinch point: midpoint of the two jaw bodies."""
    fixed = env.data.xpos[sk._body_id(env.model, f"arm{arm}_gripper")]
    moving = env.data.xpos[sk._body_id(env.model, f"arm{arm}_moving_jaw_so101_v1")]
    return 0.5 * (np.asarray(fixed) + np.asarray(moving))


print("=" * 62)
print("ADR-038 FOUR-SKILL VERIFICATION (regenerated scene + lateral retreat)")
print("=" * 62)

# --- 1. pick(A, fork) -------------------------------------------------
env, weld = fresh()
z0 = obj_z(env, "fork")
r = sk.run_pick(env, "A", "fork", weld=weld)
print("\n[1] pick(A, fork)")
print("    success=%s  holding=%r" % (r.success, weld.is_holding("A")))
print("    fork z: %.4f -> %.4f  (lift %+.4f)" % (z0, obj_z(env, "fork"), obj_z(env, "fork") - z0))
print("    reason: %s" % r.reason)
env.close()

# --- 2. place(A, fork, table) -----------------------------------------
env, weld = fresh()
z0 = obj_z(env, "fork")
r = sk.run_place(env, "A", "fork", "table", weld=weld)
print("\n[2] place(A, fork, table)")
print("    success=%s  holding=%r (expect None)" % (r.success, weld.is_holding("A")))
print("    fork z: %.4f -> %.4f   (table surface 0.35)" % (z0, obj_z(env, "fork")))
print("    reason: %s" % r.reason)
env.close()

# --- 3. pick(A, water_bottle) -- highest risk, the bottle moved -------
env, weld = fresh()
z0 = obj_z(env, "water_bottle")
r = sk.run_pick(env, "A", "bottle", weld=weld)
print("\n[3] pick(A, 'bottle')  [skill key 'bottle' -> body 'water_bottle']")
print("    success=%s  holding=%r" % (r.success, weld.is_holding("A")))
print("    bottle z: %.4f -> %.4f  (lift %+.4f)" % (z0, obj_z(env, "water_bottle"),
                                                    obj_z(env, "water_bottle") - z0))
print("    reason: %s" % r.reason)
env.close()

# --- 4. handoff(A -> B, fork) -----------------------------------------
env, weld = fresh()
z0 = obj_z(env, "fork")
r = sk.run_handoff(env, "B", "A", "fork", weld=weld)   # to_arm=B, from_arm=A
pa, pb = pinch(env, "A"), pinch(env, "B")
print("\n[4] handoff(A -> B, fork)")
print("    success=%s" % r.success)
print("    holding: A=%r  B=%r  (expect None / 'fork')" % (weld.is_holding("A"), weld.is_holding("B")))
print("    fork z: %.4f -> %.4f" % (z0, obj_z(env, "fork")))
print("    frames_used=%d" % r.frames_used)
print("    FINAL SEPARATION  lateral(y)=%.4f m   vertical(z)=%.4f m   3D=%.4f m"
      % (abs(pa[1] - pb[1]), abs(pa[2] - pb[2]), float(np.linalg.norm(pa - pb))))
print("    armA pinch=(%.3f, %.3f, %.3f)   armB pinch=(%.3f, %.3f, %.3f)"
      % (pa[0], pa[1], pa[2], pb[0], pb[1], pb[2]))
print("    reason: %s" % r.reason)
env.close()

print("\n" + "=" * 62)
