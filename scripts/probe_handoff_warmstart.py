"""Diagnostic: is the M06 handoff reachability discontinuity a seed-dependent
IK local minimum, or a genuine kinematic limit? (measurement only; does not
modify skills_scripted.py, ik.py, executor.py, grasp.py, or the scene).

Per the corrected task brief (the original brief's z=0.38/y=+0.02 cell is
NOT converged per `docs/hardware/m06-handoff-reachability-home.md`'s own
committed table -- 0.11215 residual -- so it cannot be used as a warm-start
seed). This script uses the z=0.35 cells instead, read from that same table:

  Arm A: seed (x=0, y=+0.02, z=0.35) cold residual 0.00956 (converged) ->
         target (x=0, y=0.00, z=0.35) cold residual 0.14633 (failed).
  Arm B (mirrored, per the brief's correction -- arm B fails at both of
  arm A's cells, its own discontinuity is on the opposite side):
         seed (x=0, y=-0.06, z=0.35) cold residual 0.00996 (converged) ->
         target (x=0, y=-0.04, z=0.35) cold residual 0.14364 (failed).

Test, per arm:
  1. Solve IK from HOME to the seed target. Confirm convergence (residual
     < ik.IK_POSITION_TOLERANCE_M) and record the joint config.
  2. Solve IK to the adjacent (failing) target, WARM-STARTED from that
     converged config (i.e. `scratch.qpos` initialised to the seed's solved
     joint angles, not to home). Record the residual.
  3. If warm start succeeds, CHAIN: continue in 0.02 m steps toward the
     other arm's passing band (A walks y: +0.02 -> 0.00 -> -0.02 -> -0.04
     -> -0.06; B walks the mirror: -0.06 -> -0.04 -> -0.02 -> 0.00 -> +0.02),
     each step warm-started from the previous step's converged config, and
     report every residual until the chain breaks or the two bands meet.

Also checks (cheap, gates whether a kinematic fix is even actionable):
  - Whether `arm_world_ok` is constant-False by construction: prints the
    contacting geom pair names for one row's arm-vs-world check, since the
    arm bases are mounted at z=TABLE_TOP_Z=0.35 (so101_dual_table.xml,
    `armA_base pos="0 -0.25 0.35"` / `armB_base pos="0 0.25 0.35"`) and
    `table_top` is in `_world_geom_ids`.
  - Whether `cross_arm_ok` failing everywhere at HANDOFF_SIDE_OFFSET_M=0.03
    is genuine interpenetration: prints the contacting geom pair + depth
    for the cross-arm check at the z=0.35, y=0.00 candidate (both arms'
    stage-2 configs from step 1/2 above, applied simultaneously).

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
mujoco.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

SEED = 0
Z = 0.35


def _joint_ids(model, arm):
    return [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ik.arm_joint_names(arm)]


def _seed_scratch(model, home_data, arm, joint_angles=None):
    """A scratch MjData at home, optionally with one arm's 5 joints
    overwritten by `joint_angles` (the warm-start config)."""
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = home_data.qpos
    scratch.qvel[:] = 0.0
    if joint_angles is not None:
        qpos_adrs = [int(model.jnt_qposadr[j]) for j in _joint_ids(model, arm)]
        for qadr, angle in zip(qpos_adrs, joint_angles):
            scratch.qpos[qadr] = float(angle)
    mujoco.mj_forward(model, scratch)
    return scratch


def solve_from(model, home_data, arm, start_joint_angles, target_xyz):
    """One IK solve, seeded either from home (start_joint_angles=None) or
    from a previously-converged config for this arm (all else at home)."""
    scratch = _seed_scratch(model, home_data, arm, start_joint_angles)
    return ik.solve_position_ik(model, scratch, arm, np.asarray(target_xyz, dtype=np.float64))


def chain(model, home_data, arm, y_values, z, label):
    """Walk `arm`'s target through `y_values` (consecutive 0.02 m steps),
    each solve warm-started from the previous step's converged joint
    angles. Returns list of (y, residual, converged, joint_angles)."""
    print(f"\n--- {label}: arm {arm} chain at z={z} ---")
    rows = []
    prev_angles = None
    for i, y in enumerate(y_values):
        target = (0.0, y, z)
        sol = solve_from(model, home_data, arm, prev_angles, target)
        converged = sol.position_error_m < ik.IK_POSITION_TOLERANCE_M
        seed_desc = "HOME" if prev_angles is None else f"y={y_values[i-1]:+.2f} converged config"
        print(
            f"  step {i}: target y={y:+.2f} z={z}  seed={seed_desc}  "
            f"residual={sol.position_error_m:.5f}  converged={converged}"
        )
        rows.append((y, sol.position_error_m, converged, sol.joint_angles))
        if converged:
            prev_angles = sol.joint_angles
        else:
            print(f"  -> chain BROKE at y={y:+.2f} (residual {sol.position_error_m:.5f} >= tol)")
            # Do not keep warm-starting from a non-converged config; report
            # remaining steps cold-start-only for visibility, then stop
            # advancing the chain.
            break
    return rows


def report_contacts(model, data, geoms_a, geoms_b, names_a="A", names_b="B", limit=20):
    found = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in geoms_a and g2 in geoms_b) or (g2 in geoms_a and g1 in geoms_b):
            n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or f"geom{g1}"
            n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or f"geom{g2}"
            found.append((n1, n2, float(c.dist)))
    found.sort(key=lambda t: t[2])
    for n1, n2, d in found[:limit]:
        print(f"    contact: {n1}  <->  {n2}   dist={d:.5f}")
    if not found:
        print("    (no contacts between these geom sets)")
    return found


def main() -> int:
    from bimanual.control.skills_scripted import HANDOFF_SIDE_OFFSET_M, OBJECT_BODY_NAME

    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)
    model, home_data = env.model, env.data

    print("=" * 70)
    print("PART 1: single-step warm start at the z=0.35 discontinuity")
    print("=" * 70)

    # --- Arm A ---
    seedA = solve_from(model, home_data, "A", None, (0.0, 0.02, Z))
    print(f"A cold seed  (y=+0.02): residual={seedA.position_error_m:.5f} converged={seedA.position_error_m < ik.IK_POSITION_TOLERANCE_M}")
    coldA_target = solve_from(model, home_data, "A", None, (0.0, 0.00, Z))
    print(f"A cold target(y=0.00): residual={coldA_target.position_error_m:.5f} (expect 0.14633)")
    warmA_target = solve_from(model, home_data, "A", seedA.joint_angles, (0.0, 0.00, Z))
    print(f"A WARM target(y=0.00), warm-started from seed: residual={warmA_target.position_error_m:.5f} converged={warmA_target.position_error_m < ik.IK_POSITION_TOLERANCE_M}")

    # --- Arm B (mirror) ---
    seedB = solve_from(model, home_data, "B", None, (0.0, -0.06, Z))
    print(f"B cold seed  (y=-0.06): residual={seedB.position_error_m:.5f} converged={seedB.position_error_m < ik.IK_POSITION_TOLERANCE_M}")
    coldB_target = solve_from(model, home_data, "B", None, (0.0, -0.04, Z))
    print(f"B cold target(y=-0.04): residual={coldB_target.position_error_m:.5f} (expect 0.14364)")
    warmB_target = solve_from(model, home_data, "B", seedB.joint_angles, (0.0, -0.04, Z))
    print(f"B WARM target(y=-0.04), warm-started from seed: residual={warmB_target.position_error_m:.5f} converged={warmB_target.position_error_m < ik.IK_POSITION_TOLERANCE_M}")

    print("\n" + "=" * 70)
    print("PART 2: chain the warm start toward the opposite band")
    print("=" * 70)
    a_y_values = [0.02, 0.00, -0.02, -0.04, -0.06]
    b_y_values = [-0.06, -0.04, -0.02, 0.00, 0.02]
    a_chain = chain(model, home_data, "A", a_y_values, Z, "A walking toward B's band")
    b_chain = chain(model, home_data, "B", b_y_values, Z, "B walking toward A's band")

    print("\n" + "=" * 70)
    print("PART 3: arm_world_ok check -- does it fire even on a CONVERGED,")
    print("physically sensible pose, not just on a failed/tangled one?")
    print("=" * 70)
    world_geoms = set()
    table_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    if table_gid != -1:
        world_geoms.add(table_gid)
    for body_name in set(OBJECT_BODY_NAME.values()):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid != -1:
            world_geoms |= {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == bid}

    def arm_geom_ids(arm):
        prefix = f"arm{arm}_"
        body_ids = {
            b for b in range(model.nbody)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(prefix)
        }
        return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in body_ids}

    armA_geoms = arm_geom_ids("A")
    armB_geoms = arm_geom_ids("B")

    # Use arm A's HOME-seeded solve to y=+0.02 (the WARM-START SEED cell,
    # residual 0.00956, converged per the committed table) -- deliberately
    # NOT the failed y=0.00 cell, so a hit here cannot be blamed on the arm
    # being tangled from a failed solve.
    scratchA_seed = seedA  # already solved above (HOME -> y=+0.02, z=0.35)
    applied = _seed_scratch(model, home_data, "A", scratchA_seed.joint_angles)
    print(f"Arm A converged config at (y=+0.02, z=0.35), residual={scratchA_seed.position_error_m:.5f}:")
    print("All world contacts (table_top + props), with which body owns each arm geom:")
    contacts = report_contacts(model, applied, armA_geoms, world_geoms)
    for n1, n2, d in contacts:
        for n in (n1, n2):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
            if gid != -1 and gid in armA_geoms:
                bid = int(model.geom_bodyid[gid])
                bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
                print(f"    ({n} belongs to arm body: {bname})")

    # ALSO check arm A at its plain HOME pose (no IK solve at all) vs world,
    # to see if the contact is present even before any target-seeking --
    # i.e. whether it is baked into the mounting geometry itself, independent
    # of any reach.
    scratchA_home = _seed_scratch(model, home_data, "A", None)
    print("Arm A at plain HOME (no IK solve, no reach at all) vs world geoms:")
    report_contacts(model, scratchA_home, armA_geoms, world_geoms)

    print("\n" + "=" * 70)
    print("PART 4: cross_arm_ok genuine-interpenetration check (y=0.00/-0.03 pair, z=0.35)")
    print("=" * 70)
    side = -1.0  # to_arm == B -> side = +1.0 per skills_scripted's own convention;
    # reproduced here only for the receiving point offset direction actually used
    # by probe_handoff_reachability*.py (TO_ARM="B" -> _TO_ARM_SIDE = 1.0).
    to_arm_side = 1.0
    transfer_point = (0.0, 0.00, Z)
    receiving_point = (0.0, 0.00 + to_arm_side * HANDOFF_SIDE_OFFSET_M, Z)
    solA = solve_from(model, home_data, "A", None, transfer_point)
    solB = solve_from(model, home_data, "B", None, receiving_point)
    both_scratch = mujoco.MjData(model)
    both_scratch.qpos[:] = home_data.qpos
    both_scratch.qvel[:] = 0.0
    for arm, sol in (("A", solA), ("B", solB)):
        qpos_adrs = [int(model.jnt_qposadr[j]) for j in _joint_ids(model, arm)]
        for qadr, angle in zip(qpos_adrs, sol.joint_angles):
            both_scratch.qpos[qadr] = float(angle)
    mujoco.mj_forward(model, both_scratch)
    print(f"armA residual={solA.position_error_m:.5f}  armB residual={solB.position_error_m:.5f}")
    print("Cross-arm contacts (A geoms <-> B geoms):")
    report_contacts(model, both_scratch, armA_geoms, armB_geoms)

    print("\n" + "=" * 70)
    print("PART 5: cross-arm collision at the CONVERGED chained configs")
    print("(both arms' chains pass through y=0.00 -- A step 1, B step 3) --")
    print("this is the case that actually matters: can the two arms occupy")
    print("a SHARED point without colliding, once each individually converges?")
    print("=" * 70)
    a_converged_y0 = a_chain[1][3]  # (y, residual, converged, joint_angles) at y=0.00
    b_converged_y0 = b_chain[3][3]  # B's chain step index 3 is y=0.00
    both2 = mujoco.MjData(model)
    both2.qpos[:] = home_data.qpos
    both2.qvel[:] = 0.0
    for arm, angles in (("A", a_converged_y0), ("B", b_converged_y0)):
        qpos_adrs = [int(model.jnt_qposadr[j]) for j in _joint_ids(model, arm)]
        for qadr, angle in zip(qpos_adrs, angles):
            both2.qpos[qadr] = float(angle)
    mujoco.mj_forward(model, both2)
    print(f"A residual at y=0.00 (chained): {a_chain[1][1]:.5f}   B residual at y=0.00 (chained): {b_chain[3][1]:.5f}")
    print("Cross-arm contacts (both at the SAME point y=0.00, z=0.35 -- zero side")
    print("offset, i.e. the worst case; a real handoff uses +/-0.03 m offset):")
    report_contacts(model, both2, armA_geoms, armB_geoms)

    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
