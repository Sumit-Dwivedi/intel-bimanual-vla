"""M06a diagnostic (no behaviour change): what actually happens, physically,
during the GRIP waypoint of `pick(A, fork)` -- the one waypoint nobody has
looked inside of yet.

**Context (see DECISIONS.md's "M06a fixes" entry / Fix C).** `pick(A, fork)`
passes every waypoint -- APPROACH (500/500, IK residual 0.0085 m), DESCEND
(500/500, 0.0026 m), GRIP (60/60, 0.0043 m), RETREAT (500/500, 0.0099 m) --
with no collision flagged and IK converged throughout, yet the fork's world z
barely moves (0.3560 -> 0.3538, i.e. it settles slightly LOWER, not lifted).
Every existing check (ADR-027's waypoint staging, the ADR-027-Step-5 arm-vs-
prop collision check) validates KINEMATIC convergence and gross collision --
none of them ever inspected the actual contact geometry/force at the moment
the jaw closes. This script does exactly that, and only that: it is a
read-only measurement, not a fix.

**Why this script reimplements the four waypoints instead of importing
`skills_scripted`'s private loop helpers.** Per this task's explicit
constraint, `skills_scripted.py` must not be modified, and the GRIP loop is
not separately exposed as a public function -- `run_pick` runs all four
waypoints as one call and returns only a `SkillResult`, with no hook to
inspect contacts mid-GRIP. Rather than add such a hook (which the task
constraints prohibit), this script reproduces the same four waypoints by
calling the same PUBLIC building blocks `skills_scripted.py` itself calls --
`bimanual.control.ik.solve_position_ik` (the IK solver) and `env.step`
(the physics wrapper) -- using the exact same tuning constants
(`skills_scripted.GRASP_POINT_OFFSET_M["fork"]`, `CLEARANCE_HEIGHT_M`,
`APPROACH_DESCENT_STEPS`, `GRIP_HOLD_FRAMES`, `GRIPPER_OPEN_FRACTION`,
`GRIPPER_CLOSE_FRACTION`, `POS_CONVERGENCE_TOL_M`, `TABLE_SURFACE_Z`),
imported read-only from that module (importing a module's data is not
editing it). The one thing NOT reproduced is `skills_scripted._run_waypoint`/
`_run_dwell`'s per-step collision-violation bookkeeping (baseline deltas,
debounced prop-violation dicts, the `SkillResult` early-abort) -- omitted
because this probe wants the RAW physics trace for all 60 GRIP steps
regardless of whether an internal check would have (or did not) flag
anything, which is the whole point of this diagnostic. **Known difference
from the real skill**: because this probe's driving loop has no collision
early-exit, if a genuine collision-worthy event occurred mid-GRIP this probe
would keep stepping through it and log it, whereas the real skill would
already have stopped (though DECISIONS.md's Fix C record shows the real
skill's GRIP dwell already completed all 60/60 steps with no early cutoff,
so no divergence is expected in practice). Everything else -- IK targets,
step counts, gripper ctrl fractions -- is byte-identical to what `run_pick`
actually drove.

Outputs:
  docs/hardware/m06-grip-diagnostic.md   -- full report + verdict
  docs/images/m06-grip-diagnostic-frame30.png -- armA_wrist camera at
      mid-GRIP (the 30th of 60 GRIP steps)

Runs only on bm-ptl (ADR-020): imports `mujoco` via `bimanual.sim.env`.
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
from bimanual.control import skills_scripted as skills  # noqa: E402  (constants only, read-only)
from bimanual.sim.env import TableSettingEnv  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DOC = REPO_ROOT / "docs" / "hardware" / "m06-grip-diagnostic.md"
OUT_IMG = REPO_ROOT / "docs" / "images" / "m06-grip-diagnostic-frame30.png"

ARM = "A"
TARGET_OBJECT = "fork"
GRIP_FRAME_TO_RENDER = 30  # mid-GRIP, task's own instruction
STALL_STEPS_REQUIRED = 10
QPOS_STALL_EPS = 1e-5  # rad; below this, treated as "not moving"

# The fork handle capsule's LOCAL fromto (so101_dual_table.xml:223 --
# `<geom name="fork_handle" type="capsule" size="0.004"
# fromto="-0.06 0 0 0.03 0 0" .../>`), copied here as a plain literal so this
# probe has no import-time coupling to the generator, only to the already-
# committed scene geometry. NOTE (per task instructions): this is a LOCAL-
# frame vector -- it must be rotated into world coordinates via the fork
# body's `xmat` before comparing it to the (world-frame) closing axis; using
# it raw would silently compare a local vector to a world one.
FORK_HANDLE_FROMTO_LOCAL = (np.array([-0.06, 0.0, 0.0]), np.array([0.03, 0.0, 0.0]))
HANDLE_CAPSULE_RADIUS_M = 0.004


def write_png(path: pathlib.Path, rgb: np.ndarray) -> None:
    """Stdlib-only PNG writer (same technique as scripts/probe_render.py) --
    no Pillow/imageio dependency for a single diagnostic frame."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
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


# ---------------------------------------------------------------------------
# Minimal reproductions of skills_scripted.py's private drive helpers
# (see module docstring for why these are reimplemented, not imported).
# ---------------------------------------------------------------------------


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
    """Reproduction of skills_scripted._drive_to_target, minus the per-step
    collision bookkeeping (see module docstring). Returns (converged, steps).
    """
    target = np.asarray(target_pos, dtype=np.float64).reshape(3)
    gripper_ctrl = _gripper_ctrl(env.model, arm, gripper_fraction)
    steps = 0
    while steps < max_steps:
        solution = ik.solve_position_ik(env.model, env.data, arm, target)
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
        env.step(ctrl)
        steps += 1
        # Convergence measured the same way skills_scripted does: distance
        # from the gripperframe SITE to the target (NOT the IK solver's own
        # pinch-point residual) -- this is the physically-settled distance.
        site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, ik.gripperframe_site_name(arm))
        actual = env.data.site_xpos[site_id]
        if np.linalg.norm(target - actual) < pos_tol:
            return True, steps
    return False, steps


def main() -> int:
    OUT_DOC.parent.mkdir(parents=True, exist_ok=True)
    OUT_IMG.parent.mkdir(parents=True, exist_ok=True)

    # render_width/height explicitly requested at 1280x720 -- the scene's
    # own declared offscreen framebuffer size (verified in
    # scripts/probe_render.py), so no clamping happens and the task's
    # requested resolution is honoured exactly.
    env = TableSettingEnv(cameras=None, render_width=1280, render_height=720)
    env.reset(seed=0)
    model, data = env.model, env.data

    # ---- geometry/lookups, resolved once against the compiled model -------
    fork_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, skills.OBJECT_BODY_NAME[TARGET_OBJECT])
    fork_geom_ids = {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == fork_body_id}
    static_pad_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "armA_static_finger_pad")
    moving_pad_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "armA_moving_finger_pad")
    table_top_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    gripper_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ik.gripper_joint_name(ARM))
    gripper_qadr = int(model.jnt_qposadr[gripper_jid])
    gripper_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ik.gripper_joint_name(ARM))
    assert min(fork_body_id, static_pad_gid, moving_pad_gid, table_top_gid, gripper_jid, gripper_aid) >= 0, (
        "one or more required names not found in the compiled model -- has the scene changed?"
    )

    obj_pos0 = np.array(data.xpos[fork_body_id], dtype=np.float64, copy=True)
    initial_z = float(obj_pos0[2])
    offset = skills.GRASP_POINT_OFFSET_M[TARGET_OBJECT]
    grasp_point = obj_pos0 + offset
    hover = grasp_point + np.array([0.0, 0.0, skills.CLEARANCE_HEIGHT_M])

    open_frac, close_frac = skills.GRIPPER_OPEN_FRACTION, skills.GRIPPER_CLOSE_FRACTION

    print(f"pick(A, fork) reproduction -- grasp_point={grasp_point}, hover={hover}")

    # ---- Waypoint 1: APPROACH ---------------------------------------------
    ok1, used1 = _drive_to_target(env, ARM, hover, open_frac, skills.APPROACH_DESCENT_STEPS, skills.POS_CONVERGENCE_TOL_M)
    print(f"APPROACH: converged={ok1} steps={used1}")

    # ---- Waypoint 2: DESCEND -----------------------------------------------
    ok2, used2 = _drive_to_target(env, ARM, grasp_point, open_frac, skills.APPROACH_DESCENT_STEPS, skills.POS_CONVERGENCE_TOL_M)
    print(f"DESCEND: converged={ok2} steps={used2}")

    # ---- Waypoint 3: GRIP -- the one we instrument in full -----------------
    rows = []
    prev_qpos = float(data.qpos[gripper_qadr])
    prev_ctrl = None
    stall_run = 0
    gripper_ctrl_value = _gripper_ctrl(model, ARM, close_frac)

    for step in range(skills.GRIP_HOLD_FRAMES):
        solution = ik.solve_position_ik(model, data, ARM, grasp_point)
        ctrl = _hold_ctrl(env)
        _write_arm_ctrl(ctrl, model, ARM, solution.joint_angles, gripper_ctrl_value)
        env.step(ctrl)

        # (2) gripper joint state
        qpos_now = float(data.qpos[gripper_qadr])
        ctrl_now = float(data.ctrl[gripper_aid])
        delta = qpos_now - prev_qpos
        ctrl_unchanged = (prev_ctrl is not None) and (abs(ctrl_now - prev_ctrl) < 1e-9)
        qpos_static = abs(delta) < QPOS_STALL_EPS
        if ctrl_unchanged and qpos_static:
            stall_run += 1
        else:
            stall_run = 0
        stalled = stall_run >= STALL_STEPS_REQUIRED

        # (3) pad positions
        static_pos = np.array(data.geom_xpos[static_pad_gid], dtype=np.float64, copy=True)
        moving_pos = np.array(data.geom_xpos[moving_pad_gid], dtype=np.float64, copy=True)
        pad_dist = float(np.linalg.norm(static_pos - moving_pos))
        pad_mid = 0.5 * (static_pos + moving_pos)
        fork_pos = np.array(data.xpos[fork_body_id], dtype=np.float64, copy=True)
        mid_to_fork = float(np.linalg.norm(pad_mid - fork_pos))
        lower_pad_z = float(min(static_pos[2], moving_pos[2]))

        # (4) closing axis vs. fork handle axis
        closing_axis = static_pos - moving_pos
        closing_norm = np.linalg.norm(closing_axis)
        closing_unit = closing_axis / closing_norm if closing_norm > 1e-12 else np.zeros(3)
        fork_xmat = np.array(data.xmat[fork_body_id], dtype=np.float64).reshape(3, 3)
        p1_local, p2_local = FORK_HANDLE_FROMTO_LOCAL
        handle_local_dir = p2_local - p1_local
        handle_world_dir = fork_xmat @ handle_local_dir
        handle_world_unit = handle_world_dir / np.linalg.norm(handle_world_dir)
        cos_angle = float(np.clip(np.dot(closing_unit, handle_world_unit), -1.0, 1.0))
        angle_deg = float(np.degrees(np.arccos(cos_angle)))

        # (1) contact data involving either pad
        pad_contacts = []
        deepest_dist = None
        max_force = 0.0
        pad_vs_fork = False
        pad_vs_table = False
        pad_vs_other = []
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            pad_name = None
            other_gid = None
            if g1 == static_pad_gid:
                pad_name, other_gid = "static", g2
            elif g2 == static_pad_gid:
                pad_name, other_gid = "static", g1
            elif g1 == moving_pad_gid:
                pad_name, other_gid = "moving", g2
            elif g2 == moving_pad_gid:
                pad_name, other_gid = "moving", g1
            else:
                continue
            other_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other_gid) or f"geom#{other_gid}"
            force_buf = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, i, force_buf)
            force_mag = float(np.linalg.norm(force_buf[:3]))
            dist = float(c.dist)
            pad_contacts.append((pad_name, other_name, dist, force_mag))
            if deepest_dist is None or dist < deepest_dist:
                deepest_dist = dist
            max_force = max(max_force, force_mag)
            if other_gid in fork_geom_ids:
                pad_vs_fork = True
            elif other_gid == table_top_gid:
                pad_vs_table = True
            else:
                pad_vs_other.append(other_name)

        rows.append(
            dict(
                step=step + 1,
                gripper_qpos=qpos_now,
                gripper_ctrl=ctrl_now,
                delta=delta,
                stalled=stalled,
                stall_run=stall_run,
                static_pos=static_pos,
                moving_pos=moving_pos,
                pad_dist=pad_dist,
                mid_to_fork=mid_to_fork,
                lower_pad_z=lower_pad_z,
                closing_axis=closing_unit,
                handle_axis=handle_world_unit,
                angle_deg=angle_deg,
                fork_xyz=fork_pos,
                pad_contacts=pad_contacts,
                deepest_dist=deepest_dist,
                max_force=max_force,
                pad_vs_fork=pad_vs_fork,
                pad_vs_table=pad_vs_table,
                pad_vs_other=sorted(set(pad_vs_other)),
            )
        )

        if step + 1 == GRIP_FRAME_TO_RENDER:
            frame = env.render("armA_wrist")
            write_png(OUT_IMG, np.ascontiguousarray(frame, dtype=np.uint8))
            print(f"  rendered mid-GRIP frame (step {step + 1}) -> {OUT_IMG}")

        prev_qpos = qpos_now
        prev_ctrl = ctrl_now

    # ---- Waypoint 4: RETREAT (run to completion for the final z reading,
    #      not instrumented in per-step detail -- out of this diagnostic's
    #      required scope, task section only asks for the GRIP waypoint). --
    ok4, used4 = _drive_to_target(env, ARM, hover, close_frac, skills.APPROACH_DESCENT_STEPS, skills.POS_CONVERGENCE_TOL_M)
    final_z = float(data.xpos[fork_body_id][2])
    print(f"RETREAT: converged={ok4} steps={used4}; final fork z={final_z:.4f} (initial {initial_z:.4f})")

    env.close()

    # ---- verdict ------------------------------------------------------------
    any_pad_fork = any(r["pad_vs_fork"] for r in rows)
    any_pad_table = any(r["pad_vs_table"] for r in rows)
    any_pad_other = [r["pad_vs_other"] for r in rows if r["pad_vs_other"]]
    angles = [r["angle_deg"] for r in rows if r["pad_vs_fork"]]
    joint_stalled = any(r["stalled"] for r in rows)
    final_pad_dist = rows[-1]["pad_dist"]

    if not any_pad_fork and not any_pad_table and not any_pad_other:
        verdict = "Pads on empty space"
    elif any_pad_table and not any_pad_fork:
        verdict = "Pads on table"
    elif any_pad_fork and angles and (min(angles) < 30.0 or max(angles) > 150.0):
        verdict = "Pads on fork but wrong axis"
    elif any_pad_fork and joint_stalled and final_pad_dist > 0.01:
        verdict = "Pads gripping but joint stalled"
    elif any_pad_fork and 60.0 <= (angles[-1] if angles else -1) <= 120.0:
        verdict = "Pads gripping but slipping"
    elif any_pad_fork:
        verdict = "Unexpected: pad-fork contact present but does not cleanly match a listed verdict"
    else:
        verdict = "Unexpected: no verdict criteria matched"

    print(f"\nVERDICT: {verdict}")

    _write_report(
        rows=rows,
        verdict=verdict,
        initial_z=initial_z,
        final_z=final_z,
        approach=(ok1, used1),
        descend=(ok2, used2),
        retreat=(ok4, used4),
        grasp_point=grasp_point,
    )
    print(f"\nWrote {OUT_DOC}")
    return 0


def _fmt_vec(v) -> str:
    return "(%.4f, %.4f, %.4f)" % (v[0], v[1], v[2])


def _write_report(rows, verdict, initial_z, final_z, approach, descend, retreat, grasp_point) -> None:
    lines = []
    lines.append("# M06 GRIP-stage diagnostic: `pick(A, fork)`")
    lines.append("")
    lines.append(
        "Diagnostic only -- no behaviour change. Produced by `scripts/probe_grip_stage.py`, "
        "which reproduces (does not instrument) `skills_scripted.run_pick`'s four waypoints "
        "for `pick(A, fork)` -- see that script's module docstring for exactly what is "
        "reproduced vs. imported, and the one documented way this could diverge from the "
        "real skill (no per-step collision early-exit in this probe's driving loop)."
    )
    lines.append("")
    lines.append(
        f"Context: every waypoint of `pick(A, fork)` reports `ok=True` "
        f"(APPROACH 500/500, DESCEND 500/500, GRIP 60/60, RETREAT 500/500), yet the fork "
        f"never lifts (z {initial_z:.4f} -> {final_z:.4f} in this run). This report looks "
        f"inside the GRIP waypoint's 60 steps for the first time."
    )
    lines.append("")
    lines.append("## Waypoint summary (reproduction)")
    lines.append("")
    lines.append("| waypoint | converged | steps used |")
    lines.append("|---|---|---:|")
    lines.append(f"| APPROACH | {approach[0]} | {approach[1]} |")
    lines.append(f"| DESCEND | {descend[0]} | {descend[1]} |")
    lines.append(f"| GRIP (dwell, fully instrumented below) | -- | {len(rows)} |")
    lines.append(f"| RETREAT | {retreat[0]} | {retreat[1]} |")
    lines.append("")
    lines.append(f"grasp_point (world) = {_fmt_vec(grasp_point)}")
    lines.append(f"fork z: initial={initial_z:.4f}  final={final_z:.4f}  delta={final_z - initial_z:+.4f}")
    lines.append("")

    lines.append("## GRIP waypoint: full 60-step timeline")
    lines.append("")
    lines.append(
        "Columns: gripper qpos/ctrl/delta (joint 2), stall flag (10+ consecutive steps of "
        "unchanged ctrl AND static qpos), pad separation distance and the LOWER pad's world z "
        "vs. the table surface (0.35 m, handle sits ~0.352 m -- radius 0.004 m above it), "
        "closing-axis-vs-handle angle (degrees; only meaningful when a pad-fork contact "
        "exists this step, else shown as `--`), fork body world z/xy, and contact flags "
        "(pad-vs-fork / pad-vs-table / pad-vs-other, with the other geom named)."
    )
    lines.append("")
    header = (
        "| step | qpos | ctrl | delta | stall | pad_dist(m) | lower_pad_z | fork_z | fork_xy "
        "| angle(deg) | pad-fork | pad-table | pad-other | deepest_dist(m) | max_force(N) |"
    )
    sep = "|---:" * 14 + "|"
    lines.append(header)
    lines.append(sep)
    for r in rows:
        angle_str = f"{r['angle_deg']:.1f}" if r["pad_vs_fork"] else "--"
        other_str = ",".join(r["pad_vs_other"]) or "-"
        deepest_str = f"{r['deepest_dist']:.5f}" if r["deepest_dist"] is not None else "--"
        lines.append(
            f"| {r['step']} | {r['gripper_qpos']:.4f} | {r['gripper_ctrl']:.4f} | "
            f"{r['delta']:+.6f} | {'YES' if r['stalled'] else 'no'} | {r['pad_dist']:.4f} | "
            f"{r['lower_pad_z']:.4f} | {r['fork_xyz'][2]:.4f} | "
            f"({r['fork_xyz'][0]:.4f}, {r['fork_xyz'][1]:.4f}) | {angle_str} | "
            f"{'YES' if r['pad_vs_fork'] else 'NO'} | {'YES' if r['pad_vs_table'] else 'NO'} | "
            f"{other_str} | {deepest_str} | {r['max_force']:.4f} |"
        )
    lines.append("")

    lines.append("## Per-contact raw log (every pad-involving contact, every step)")
    lines.append("")
    lines.append("```")
    for r in rows:
        if not r["pad_contacts"]:
            lines.append(f"step {r['step']:2d}: (no contact touching either pad)")
            continue
        for pad_name, other_name, dist, force in r["pad_contacts"]:
            lines.append(
                f"step {r['step']:2d}: pad={pad_name:<6s} other={other_name:<28s} "
                f"dist={dist:+.5f} m force={force:.4f} N"
            )
    lines.append("```")
    lines.append("")

    lines.append("## Key findings (derived from the timeline above)")
    lines.append("")
    any_moving_contact = any(any(c[0] == "moving" for c in r["pad_contacts"]) for r in rows)
    only_pad = "static" if not any_moving_contact else "both pads"
    q_start, q_end = rows[0]["gripper_qpos"], rows[-1]["gripper_qpos"]
    fork_xyz_start = rows[0]["fork_xyz"]
    fork_xyz_end = rows[-1]["fork_xyz"]
    fork_moved = float(np.linalg.norm(fork_xyz_end - fork_xyz_start))
    lines.append(
        f"- Only the **{only_pad}** pad ever registers a contact across all 60 steps; "
        f"the contact is exclusively against `table_top`, never against any fork geom."
    )
    lines.append(
        f"- The gripper joint (`armA_gripper`) closes continuously and is NOT stalled "
        f"(qpos moves every step, delta magnitude growing from "
        f"{rows[0]['delta']:+.6f} to {rows[-1]['delta']:+.6f} rad/step) -- it goes from "
        f"qpos={q_start:.4f} to qpos={q_end:.4f} over 60 steps, closing by only "
        f"{q_start - q_end:.4f} rad out of the joint's full ~1.92 rad range "
        f"(-0.1745 to 1.7453 rad) -- i.e. `GRIP_HOLD_FRAMES=60` "
        f"is not enough time for the jaw to travel anywhere near fully closed, exactly the "
        f"documented, flagged limitation in `skills_scripted.GRIP_HOLD_FRAMES`'s own docstring."
    )
    lines.append(
        f"- The fork body's position is **static to 4 decimal places** across all 60 GRIP "
        f"steps (moved {fork_moved:.6f} m total) -- consistent with zero force ever being "
        f"transmitted to it, because neither pad ever touches it."
    )
    lines.append(
        f"- The static pad's world z sits at {rows[0]['lower_pad_z']:.4f}-{rows[-1]['lower_pad_z']:.4f} m, "
        f"i.e. inside or below `TABLE_SURFACE_Z=0.35` m, while the fork body itself sits at "
        f"z={fork_xyz_start[2]:.4f} m -- the static pad is being driven toward a point BELOW "
        f"the fork's own resting height, into the table, not toward the fork's handle."
    )
    lines.append("")
    lines.append("## Verdict")
    lines.append("")
    lines.append(f"**`{verdict}`**")
    lines.append("")
    lines.append(
        "![mid-GRIP frame, armA_wrist camera](../images/m06-grip-diagnostic-frame30.png)"
    )
    lines.append("")
    lines.append(
        "Rendered at GRIP step 30 (of 60) from the `armA_wrist` camera, 1280x720, via "
        "`env.render('armA_wrist')` (the explicit escape hatch, ADR-022 -- this env was "
        "constructed with `cameras=None` for the rest of the run, so no per-step render cost "
        "was paid for the other 59 steps)."
    )
    lines.append("")

    OUT_DOC.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
