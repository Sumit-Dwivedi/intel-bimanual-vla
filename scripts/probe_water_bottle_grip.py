"""DIAGNOSTIC ONLY -- no behaviour change (M06, water_bottle weld-attach
failure investigation).

Context: `pick(A, fork)` succeeds -- weld attaches at frame 1155, fork lifts
43 mm (see `docs/hardware/m06-grip-diagnostic-after-fix.md`,
`docs/hardware/m06-weld-verification.md`). `pick(A, water_bottle)` reaches
GRIP but fails `weld_attach_failed_after_300_frames`. This script measures
(does not fix) exactly what happens during that 300-frame GRIP dwell, the
same reproduction convention `scripts/probe_grip_stage.py` and
`scripts/probe_weld_grasp.py` already use: it drives arm A through
APPROACH/DESCEND using the same public building blocks `skills_scripted.py`
itself calls (`ik.solve_position_ik`, `env.step`), reproducing (not
modifying) `skills_scripted._drive_to_target` and `_dwell`'s ADR-031
frozen-arm-ctrl GRIP logic, then logs the pinch point (the midpoint of the
fixed and moving jaw bodies, exactly as `ik.py`'s `_pinch_point` and
`grasp.WeldGrasp.attempt_grasp`'s Gate 2 compute it), the water_bottle body
position, the distance between them, and the gripper joint qpos every 30
frames through the dwell -- and calls the real `WeldGrasp.attempt_grasp`
every step (exactly as `_dwell` does) so the true gate outcomes are observed,
not just distances.

Does not modify `grasp.py`, `skills_scripted.py`, `gen_dual_scene.py`,
`ik.py`, `executor.py`, or `scenes/so101/`. Read-only imports throughout.

Runs only on bm-ptl (ADR-020): imports `mujoco` via `bimanual.sim.env`.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control import skills_scripted as skills  # noqa: E402  (constants only, read-only)
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DOC = REPO_ROOT / "docs" / "hardware" / "m06-water-bottle-diagnostic.md"

ARM = "A"
TARGET_OBJECT = "bottle"  # skills_scripted.OBJECT_BODY_NAME["bottle"] == "water_bottle"
LOG_EVERY = 30


def _gripper_ctrl(model, arm: str, fraction: float) -> float:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ik.gripper_joint_name(arm))
    lo, hi = model.actuator_ctrlrange[aid]
    return float(lo + fraction * (hi - lo))


def _hold_ctrl(env) -> np.ndarray:
    ctrl = np.zeros(env.model.nu, dtype=np.float64)
    for aid in range(env.model.nu):
        jid = int(env.model.actuator_trnid[aid, 0])
        qadr = int(env.model.jnt_qposadr[jid])
        ctrl[aid] = env.data.qpos[qadr]
    return ctrl


def _write_arm_ctrl(ctrl: np.ndarray, model, arm: str, joint_angles, gripper_ctrl: float) -> None:
    for name, angle in zip(ik.arm_joint_names(arm), joint_angles):
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        ctrl[aid] = float(angle)
    gaid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ik.gripper_joint_name(arm))
    ctrl[gaid] = gripper_ctrl


def _drive_to_target(env, arm: str, target_pos, gripper_fraction: float, max_steps: int, pos_tol: float):
    """Reproduction of skills_scripted._drive_to_target (IK re-solved every
    step, no ADR-031 freeze -- that freeze is GRIP/RELEASE-dwell only), minus
    the per-step collision bookkeeping (out of scope for this diagnostic)."""
    target = np.asarray(target_pos, dtype=np.float64).reshape(3)
    gripper_ctrl = _gripper_ctrl(env.model, arm, gripper_fraction)
    steps = 0
    while steps < max_steps:
        solution = ik.solve_position_ik(env.model, env.data, arm, target)
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
        env.step(ctrl)
        steps += 1
        site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, ik.gripperframe_site_name(arm))
        actual = env.data.site_xpos[site_id]
        if np.linalg.norm(target - actual) < pos_tol:
            return True, steps
    return False, steps


def main() -> int:
    OUT_DOC.parent.mkdir(parents=True, exist_ok=True)

    env = TableSettingEnv(cameras=None, render_width=1280, render_height=720)
    env.reset(seed=0)
    model, data = env.model, env.data
    weld = WeldGrasp(env)

    body_name = skills.OBJECT_BODY_NAME[TARGET_OBJECT]
    bottle_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    fixed_jaw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.fixed_jaw_body_name(ARM))
    moving_jaw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ik.moving_jaw_body_name(ARM))
    gripper_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ik.gripper_joint_name(ARM))
    gripper_qadr = int(model.jnt_qposadr[gripper_jid])
    assert min(bottle_body_id, fixed_jaw_id, moving_jaw_id, gripper_jid) >= 0

    obj_pos0 = np.array(data.xpos[bottle_body_id], dtype=np.float64, copy=True)
    initial_z = float(obj_pos0[2])
    offset = skills.GRASP_POINT_OFFSET_M[TARGET_OBJECT]
    grasp_point = obj_pos0 + offset
    hover = grasp_point + np.array([0.0, 0.0, skills.CLEARANCE_HEIGHT_M])
    open_frac, close_frac = skills.GRIPPER_OPEN_FRACTION, skills.GRIPPER_CLOSE_FRACTION

    print(f"pick(A, water_bottle) reproduction -- bottle_pos0={obj_pos0}, offset={offset}, "
          f"grasp_point={grasp_point}, hover={hover}")

    # ---- Waypoint 1: APPROACH ----------------------------------------------
    ok1, used1 = _drive_to_target(env, ARM, hover, open_frac, skills.APPROACH_DESCENT_STEPS, skills.POS_CONVERGENCE_TOL_M)
    print(f"APPROACH: converged={ok1} steps={used1}")

    # ---- Waypoint 2: DESCEND ------------------------------------------------
    ok2, used2 = _drive_to_target(env, ARM, grasp_point, open_frac, skills.APPROACH_DESCENT_STEPS, skills.POS_CONVERGENCE_TOL_M)
    print(f"DESCEND: converged={ok2} steps={used2}")

    def pinch_point() -> np.ndarray:
        return 0.5 * (data.xpos[fixed_jaw_id] + data.xpos[moving_jaw_id])

    # ---- GRIP start measurement (before any dwell stepping) ----------------
    pp0 = pinch_point().copy()
    bottle0 = np.array(data.xpos[bottle_body_id], dtype=np.float64, copy=True)
    dist0 = float(np.linalg.norm(bottle0 - pp0))
    qpos0 = float(data.qpos[gripper_qadr])
    print(f"GRIP start: pinch_point={pp0} bottle_pos={bottle0} distance={dist0:.4f} qpos={qpos0:.4f}")

    # ---- Waypoint 3: GRIP -- reproduce _dwell's ADR-031 frozen-arm logic ---
    gripper_ctrl_value = _gripper_ctrl(model, ARM, close_frac)
    arm_actuator_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ik.arm_joint_names(ARM)
    ]
    frozen_arm_ctrl = np.array([data.ctrl[aid] for aid in arm_actuator_ids], dtype=np.float64)

    rows = []
    attach_frame = None
    n_steps = skills.GRIP_HOLD_FRAMES
    for i in range(n_steps):
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, model, ARM, frozen_arm_ctrl, gripper_ctrl_value)
        env.step(ctrl)

        step = i + 1
        pp = pinch_point().copy()
        bottle_pos = np.array(data.xpos[bottle_body_id], dtype=np.float64, copy=True)
        distance_m = float(np.linalg.norm(bottle_pos - pp))
        qpos_now = float(data.qpos[gripper_qadr])

        attached = weld.attempt_grasp(ARM, body_name)
        if attached and attach_frame is None:
            attach_frame = step

        if step == 1 or step % LOG_EVERY == 0 or attached:
            rows.append(dict(step=step, pinch=pp.copy(), bottle=bottle_pos.copy(),
                              distance=distance_m, qpos=qpos_now, attached=attached))
            print(f"  step {step:3d}: pinch={pp} bottle={bottle_pos} distance={distance_m:.4f} "
                  f"qpos={qpos_now:.4f} attached={attached}")

        if attached:
            break

    final_holding = weld.is_holding(ARM)
    print(f"\nattach_frame={attach_frame} final weld.is_holding(A)={final_holding!r}")

    env.close()

    _write_report(
        obj_pos0=obj_pos0, initial_z=initial_z, offset=offset, grasp_point=grasp_point, hover=hover,
        approach=(ok1, used1), descend=(ok2, used2),
        pp0=pp0, bottle0=bottle0, dist0=dist0, qpos0=qpos0,
        rows=rows, attach_frame=attach_frame, final_holding=final_holding,
    )
    print(f"\nWrote {OUT_DOC}")
    return 0


def _fmt_vec(v) -> str:
    return "(%.4f, %.4f, %.4f)" % (v[0], v[1], v[2])


def _write_report(*, obj_pos0, initial_z, offset, grasp_point, hover, approach, descend,
                   pp0, bottle0, dist0, qpos0, rows, attach_frame, final_holding) -> None:
    lines = []
    lines.append("# M06 water_bottle GRIP-stage diagnostic: `pick(A, water_bottle)`")
    lines.append("")
    lines.append(
        "**Diagnostic only -- no behaviour change, no fix applied.** Produced by "
        "`scripts/probe_water_bottle_grip.py`, which reproduces (does not modify) "
        "`skills_scripted.run_pick`'s APPROACH/DESCEND waypoints and `_dwell`'s ADR-031 "
        "frozen-arm GRIP dwell for `pick(A, water_bottle)`, calling the real "
        "`grasp.WeldGrasp.attempt_grasp` every dwell step exactly as `_dwell` does. See that "
        "script's module docstring for exactly what is reproduced vs. imported."
    )
    lines.append("")
    lines.append(
        f"Context: `pick(A, fork)` succeeds -- weld attaches at frame 1155, fork lifts 43 mm "
        f"(`docs/hardware/m06-weld-verification.md`, `docs/hardware/m06-grip-diagnostic-after-fix.md`). "
        f"`pick(A, water_bottle)` reaches GRIP but fails `weld_attach_failed_after_300_frames`. "
        f"This report measures the GRIP dwell to find out why."
    )
    lines.append("")

    lines.append("## Setup")
    lines.append("")
    lines.append(f"- `water_bottle` body position at reset (before any waypoint): {_fmt_vec(obj_pos0)} "
                  f"(z={initial_z:.4f})")
    lines.append(f"- `GRASP_POINT_OFFSET_M['bottle']` (unchanged, read-only): {_fmt_vec(offset)}")
    lines.append(f"- `grasp_point` (obj_pos0 + offset): {_fmt_vec(grasp_point)}")
    lines.append(f"- `hover` (grasp_point + CLEARANCE_HEIGHT_M on z): {_fmt_vec(hover)}")
    lines.append(f"- APPROACH: converged={approach[0]} steps={approach[1]}")
    lines.append(f"- DESCEND: converged={descend[0]} steps={descend[1]}")
    lines.append("")

    lines.append("## GRIP start (frame 0 of the 300-frame dwell)")
    lines.append("")
    lines.append(f"1. `water_bottle` body position at GRIP start: {_fmt_vec(bottle0)}")
    lines.append(f"2. Arm A pinch point at GRIP start (midpoint of `armA_gripper` and "
                  f"`armA_moving_jaw_so101_v1` body `xpos`, exactly as `ik.py`'s `_pinch_point` "
                  f"and `grasp.WeldGrasp.attempt_grasp`'s Gate 2 compute it): {_fmt_vec(pp0)}")
    lines.append(f"3. Distance between them at GRIP start: **{dist0:.4f} m** "
                  f"(gate threshold: `distance_threshold_m=0.05` m)")
    lines.append(f"4. Gripper joint (`armA_gripper`) qpos at GRIP start: **{qpos0:.4f}** rad "
                  f"(closure gate: must fall below `closure_threshold=0.3` rad)")
    lines.append("")

    lines.append("## GRIP dwell timeline (every 30 frames, plus the first frame and the "
                  "attach frame if one occurred)")
    lines.append("")
    lines.append("| step | pinch_point | bottle_pos | distance(m) | qpos(rad) | attached |")
    lines.append("|---:|---|---|---:|---:|---|")
    for r in rows:
        lines.append(
            f"| {r['step']} | {_fmt_vec(r['pinch'])} | {_fmt_vec(r['bottle'])} | "
            f"{r['distance']:.4f} | {r['qpos']:.4f} | {r['attached']} |"
        )
    lines.append("")

    lines.append("## Comparison against fork's successful trajectory")
    lines.append("")
    lines.append(
        "Fork (`pick(A, fork)`, post-ADR-031): pinch-point distance to target held flat "
        "0.0351 -> 0.0374 m, well under the 0.050 m gate, while qpos fell 1.7449 -> 0.3519 "
        "over its dwell (attaches once qpos also crosses the 0.3 closure gate)."
    )
    dist_start = rows[0]["distance"] if rows else dist0
    dist_end = rows[-1]["distance"] if rows else dist0
    qpos_start = rows[0]["qpos"] if rows else qpos0
    qpos_end = rows[-1]["qpos"] if rows else qpos0
    lines.append(
        f"Water bottle (this run): pinch-point distance to bottle went "
        f"{dist_start:.4f} -> {dist_end:.4f} m; qpos went {qpos_start:.4f} -> {qpos_end:.4f}."
    )
    lines.append("")
    lines.append(f"Final: `attach_frame={attach_frame}`, `weld.is_holding('A')={final_holding!r}`.")
    lines.append("")

    lines.append("## Branch determination")
    lines.append("")
    if dist_end >= 0.05 and abs(dist_end - dist_start) < 0.01:
        lines.append(
            f"**Distance holds flat but stays ABOVE 0.050 m** (start {dist_start:.4f} m, "
            f"end {dist_end:.4f} m). This points to the grasp point being offset wrong for the "
            f"bottle's geometry, not a freeze/closure-gate problem -- see recommendation below."
        )
    elif dist_end > dist_start + 0.01:
        lines.append(
            f"**Distance GROWS** across the dwell ({dist_start:.4f} -> {dist_end:.4f} m). "
            f"This would mean the ADR-031 freeze is not taking effect on this path."
        )
    elif dist_end < 0.05:
        lines.append(
            f"**Distance stays UNDER the 0.050 m threshold** ({dist_start:.4f} -> {dist_end:.4f} m) "
            f"but the gate still fails. This points to the closure gate (qpos), not proximity -- "
            f"see the qpos trajectory above for whether it crosses 0.3 within the 300-frame budget."
        )
    else:
        lines.append(
            f"Distance trajectory ({dist_start:.4f} -> {dist_end:.4f} m) does not cleanly match "
            f"one of the three anticipated branches -- reported as measured, no branch forced."
        )
    lines.append("")

    OUT_DOC.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
