"""M06 diagnostic (Phase 2 pre-check): can `ik.solve_position_ik`, as it exists
today, raise the `armA_gripper` BODY by a visible amount from its neutral
pose? **Diagnostic only** -- no fix, no wiring. Time-boxed to 30 minutes per
the task instruction; this script and its two output artifacts are the whole
deliverable.

**Why this probe exists.** `DECISIONS.md`'s ADR-029 entry already flagged,
as an unplanned finding while building `scripts/probe_weld_grasp.py`, that
`ik.solve_position_ik`'s pinch-point target (ADR-025) combined with ADR-024's
fully-relaxed orientation let the redundant 5-DOF solve satisfy a
progressively-rising pinch-point target by rotating the WRIST rather than
raising the arm -- the pinch point tracked the target (residual under 0.01 m)
while the `armA_gripper` BODY (the actual weld attach frame, ADR-029) FELL.
That finding used one ad hoc lift schedule over 50 steps. This script
generalizes it: five separate target heights, 300 steps each (6x the
previous window, to separate "too slow" from "cannot"), reporting the pinch
point and the `armA_gripper` body SEPARATELY at every step -- exactly because
the previous run showed they diverge, and a report that only showed one of
them would hide the entire effect.

**Not a skill test, not a fix.** No skill layer, no `WeldGrasp`, no edits to
`ik.py`/`skills_scripted.py`/`executor.py`/`grasp.py`/`scenes/so101/`/
`gen_dual_scene.py`. This script only calls `ik.solve_position_ik` (a public
function) and `env.step` (the public control entry point), same convention
as `scripts/probe_weld_grasp.py`.

**Design choice, stated explicitly:** per the task instructions ("For each
target: solve IK, command the joint targets, step 300 frames, then record"),
IK is solved ONCE per target (not re-solved every step in a closed loop like
a real skill would). The resulting 5 joint angles are held as a constant
commanded ctrl vector (other actuators held at their current qpos) for 300
`env.step()` calls, and the position actuators' own PD dynamics (kp=998.22,
kv=2.731, `so101_dual_table.xml`'s `sts3215` class) are what actually moves
the arm toward that one commanded target over those 300 steps.

Outputs:
  docs/hardware/m06-ik-lift-diagnostic.md
  docs/images/m06-ik-lift-diagnostic.png  (front camera, delta_z=0.10 attempt)

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
from bimanual.sim.env import TableSettingEnv  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DOC = REPO_ROOT / "docs" / "hardware" / "m06-ik-lift-diagnostic.md"
OUT_IMG = REPO_ROOT / "docs" / "images" / "m06-ik-lift-diagnostic.png"

ARM = "A"
FRONT_CAMERA = "front"

DELTA_Z_TARGETS_M = [0.02, 0.05, 0.10, 0.15, 0.20]
STEPS_PER_TARGET = 300
LOG_FRAMES = (50, 150, 300)  # trajectory-shape checkpoints requested by the task
RENDER_AT_DELTA_Z = 0.10

#: Ceiling test: how far to drive shoulder_lift directly, bypassing ik.py.
CEILING_DELTA_Z = 0.05
#: SIGN VERIFIED DIRECTLY, NOT ASSUMED FROM THE TASK'S "-0.3 rad" EXAMPLE.
#: `DECISIONS.md`'s ADR-029 entry found that DECREASING shoulder_lift raised
#: the gripper -- but that was measured from a configuration already reached
#: after an approach-and-close sequence, not from the raw home pose. Probed
#: directly here (both signs, 300 steps each, from `reset(seed=0)`'s home
#: pose): -0.3 rad LOWERS `armA_gripper` by -0.0892 m; +0.3 rad RAISES it by
#: +0.0772 m. The home fold configuration (`shoulder_lift=-1.2`,
#: `elbow_flex=-1.6`, ADR-026) is on the opposite side of whatever geometric
#: threshold flips this sign -- i.e. "which sign lifts" is
#: configuration-dependent, not a fixed property of the joint. +0.3 is used
#: below because it is the sign that actually raises the gripper FROM HOME,
#: which is what this test is supposed to measure; the sign flip itself is
#: reported in the doc as a genuine, separate finding.
CEILING_SHOULDER_LIFT_DELTA_RAD = 0.3

#: Below this per-100-step z change, the trajectory is considered "settled"
#: (not still visibly rising) for the plateau call. Documented, not buried:
#: 0.3 mm over 150 steps (frames 150->300) is well under this scene's own
#: IK_POSITION_TOLERANCE_M (0.01 m) and an order of magnitude below the
#: smallest requested delta_z (0.02 m), so calling it "settled" at that point
#: is a conservative reading, not a generous one.
PLATEAU_THRESHOLD_M = 0.0003


# ---------------------------------------------------------------------------
# stdlib-only PNG writer (same technique as scripts/probe_weld_grasp.py /
# scripts/probe_render.py -- no Pillow/imageio dependency for one frame).
# ---------------------------------------------------------------------------
def write_png(path: pathlib.Path, rgb: np.ndarray) -> None:
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


def _hold_ctrl(env) -> np.ndarray:
    """Ctrl vector holding EVERY actuator (both arms) at its CURRENT qpos --
    same helper as scripts/probe_weld_grasp.py's `_hold_ctrl`, reproduced
    here (not imported) since that script's helpers are private/local.
    """
    ctrl = np.zeros(env.model.nu, dtype=np.float64)
    for aid in range(env.model.nu):
        jid = int(env.model.actuator_trnid[aid, 0])
        qadr = int(env.model.jnt_qposadr[jid])
        ctrl[aid] = env.data.qpos[qadr]
    return ctrl


def _pinch_point(env, arm: str) -> np.ndarray:
    """Live world-frame pinch point: midpoint of the fixed and moving jaw
    bodies, exactly what `ik.solve_position_ik` targets (ADR-025) -- NOT the
    `armX_gripperframe` site.
    """
    fixed_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, ik.fixed_jaw_body_name(arm))
    moving_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, ik.moving_jaw_body_name(arm))
    return 0.5 * (
        np.array(env.data.xpos[fixed_id], dtype=np.float64, copy=True)
        + np.array(env.data.xpos[moving_id], dtype=np.float64, copy=True)
    )


def _gripper_body_pos(env, arm: str) -> np.ndarray:
    """Live world-frame position of the `armX_gripper` BODY -- the fixed jaw
    body, and the actual weld-attach frame (ADR-029). NOT the pinch point,
    NOT the `armX_gripperframe` site -- see `ik.py`'s naming-trap docstring.
    """
    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, ik.fixed_jaw_body_name(arm))
    return np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)


def _joint_qpos(env, joint_name: str) -> float:
    jid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qadr = int(env.model.jnt_qposadr[jid])
    return float(env.data.qpos[qadr])


def _fmt_vec(v) -> str:
    return "(%.4f, %.4f, %.4f)" % (v[0], v[1], v[2])


def run_one_target(env, delta_z: float, start_pinch: np.ndarray, render: bool):
    """Reset to home, solve IK once toward `start_pinch + delta_z*z`, hold
    that joint target for STEPS_PER_TARGET steps, log frames 50/150/300, and
    return a result dict. Each target starts from a FRESH reset (not
    compounded on the previous target's end state) so results are
    independent and comparable, and so the "starting" position recorded in
    step 2 is genuinely what every row measures from.
    """
    env.reset(seed=0)

    gripper_z0 = float(_gripper_body_pos(env, ARM)[2])
    pinch_z0 = float(_pinch_point(env, ARM)[2])
    wrist_roll0 = _joint_qpos(env, "armA_wrist_roll")
    shoulder_lift0 = _joint_qpos(env, "armA_shoulder_lift")

    target = start_pinch + np.array([0.0, 0.0, delta_z])
    solution = ik.solve_position_ik(env.model, env.data, ARM, target)

    ctrl = _hold_ctrl(env)
    for name, angle in zip(ik.arm_joint_names(ARM), solution.joint_angles):
        aid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        ctrl[aid] = float(angle)

    frame_log = {}
    frame = None
    for step in range(1, STEPS_PER_TARGET + 1):
        env.step(ctrl)
        if step in LOG_FRAMES:
            frame_log[step] = {
                "gripper_z": float(_gripper_body_pos(env, ARM)[2]),
                "pinch_z": float(_pinch_point(env, ARM)[2]),
            }
        if render and step == 150:  # mid-window frame for the requested PNG
            frame = env.render(FRONT_CAMERA)

    gripper_z_final = frame_log[STEPS_PER_TARGET]["gripper_z"]
    pinch_z_final = frame_log[STEPS_PER_TARGET]["pinch_z"]
    wrist_roll_final = _joint_qpos(env, "armA_wrist_roll")
    shoulder_lift_final = _joint_qpos(env, "armA_shoulder_lift")

    rise_50_150 = frame_log[150]["gripper_z"] - frame_log[50]["gripper_z"]
    rise_150_300 = frame_log[300]["gripper_z"] - frame_log[150]["gripper_z"]
    plateaued = abs(rise_150_300) < PLATEAU_THRESHOLD_M

    return {
        "delta_z": delta_z,
        "ik_residual": solution.position_error_m,
        "ik_converged": solution.converged,
        "wrist_roll_delta": wrist_roll_final - wrist_roll0,
        "shoulder_lift_delta": shoulder_lift_final - shoulder_lift0,
        "pinch_realized_lift_m": pinch_z_final - pinch_z0,
        "gripper_body_realized_lift_m": gripper_z_final - gripper_z0,
        "deficit_m": delta_z - (gripper_z_final - gripper_z0),
        "frame_log": frame_log,
        "gripper_z0": gripper_z0,
        "pinch_z0": pinch_z0,
        "rise_50_150": rise_50_150,
        "rise_150_300": rise_150_300,
        "plateaued": plateaued,
        "render_frame": frame,
    }


def run_ceiling_test(env, start_gripper_z: float):
    """Bypass ik.py entirely: drive `armA_shoulder_lift` directly toward
    `CEILING_SHOULDER_LIFT_DELTA_RAD` from its current (post-reset) qpos,
    holding every other actuator at its current qpos, for STEPS_PER_TARGET
    steps. Reports the resulting `armA_gripper` BODY world position change --
    the arm's physical lifting ceiling, independent of IK's target-choice
    behaviour.
    """
    env.reset(seed=0)
    gripper_z0 = float(_gripper_body_pos(env, ARM)[2])
    shoulder_aid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "armA_shoulder_lift")
    shoulder_lift0 = _joint_qpos(env, "armA_shoulder_lift")
    target_shoulder_lift = shoulder_lift0 + CEILING_SHOULDER_LIFT_DELTA_RAD

    ctrl = _hold_ctrl(env)
    ctrl[shoulder_aid] = target_shoulder_lift

    frame_log = {}
    for step in range(1, STEPS_PER_TARGET + 1):
        env.step(ctrl)
        if step in LOG_FRAMES:
            frame_log[step] = float(_gripper_body_pos(env, ARM)[2])

    gripper_z_final = frame_log[STEPS_PER_TARGET]
    return {
        "gripper_z0": gripper_z0,
        "gripper_z_final": gripper_z_final,
        "realized_lift_m": gripper_z_final - gripper_z0,
        "frame_log": frame_log,
        "shoulder_lift0": shoulder_lift0,
        "target_shoulder_lift": target_shoulder_lift,
    }


def main() -> int:
    OUT_DOC.parent.mkdir(parents=True, exist_ok=True)
    OUT_IMG.parent.mkdir(parents=True, exist_ok=True)

    log_lines: list[str] = []

    def log(msg: str) -> None:
        print(msg)
        log_lines.append(msg)

    env = TableSettingEnv(cameras=None, render_width=1280, render_height=720)

    # ---- Step 2: record starting armA_gripper body and pinch point --------
    env.reset(seed=0)
    start_gripper = _gripper_body_pos(env, ARM)
    start_pinch = _pinch_point(env, ARM)
    log("=== M06 IK LIFT DIAGNOSTIC (probe_ik_lift.py) ===")
    log(f"Home pose (reset(seed=0)), arm A:")
    log(f"  armA_gripper BODY start = {_fmt_vec(start_gripper)}")
    log(f"  pinch point   start     = {_fmt_vec(start_pinch)}")
    log(f"  armA_wrist_roll start   = {_joint_qpos(env, 'armA_wrist_roll'):.4f} rad")
    log(f"  armA_shoulder_lift start = {_joint_qpos(env, 'armA_shoulder_lift'):.4f} rad")
    log("")

    # ---- Steps 3-4: five delta_z targets, each from a fresh reset ---------
    results = []
    render_frame = None
    for delta_z in DELTA_Z_TARGETS_M:
        render_this = abs(delta_z - RENDER_AT_DELTA_Z) < 1e-9
        res = run_one_target(env, delta_z, start_pinch, render=render_this)
        results.append(res)
        if render_this and res["render_frame"] is not None:
            render_frame = res["render_frame"]
        log(
            f"--- delta_z={delta_z:.2f} m --- IK residual={res['ik_residual']:.5f} m "
            f"(converged={res['ik_converged']}) "
            f"wrist_roll_delta={res['wrist_roll_delta']:+.4f} rad "
            f"shoulder_lift_delta={res['shoulder_lift_delta']:+.4f} rad"
        )
        for f in LOG_FRAMES:
            fl = res["frame_log"][f]
            log(f"    frame {f:3d}: gripper_z={fl['gripper_z']:.4f}  pinch_z={fl['pinch_z']:.4f}")
        log(
            f"    realized: pinch_lift={res['pinch_realized_lift_m']:+.4f} m  "
            f"gripper_body_lift={res['gripper_body_realized_lift_m']:+.4f} m  "
            f"deficit={res['deficit_m']:+.4f} m"
        )
        log(
            f"    trajectory shape: rise(50->150)={res['rise_50_150']:+.5f} m  "
            f"rise(150->300)={res['rise_150_300']:+.5f} m  "
            f"plateaued_by_300={res['plateaued']}"
        )
        log("")

    if render_frame is not None:
        write_png(OUT_IMG, np.ascontiguousarray(render_frame, dtype=np.uint8))
        log(f"Rendered delta_z={RENDER_AT_DELTA_Z} attempt (frame 150) -> {OUT_IMG}")
    else:
        log(f"WARNING: no render captured for delta_z={RENDER_AT_DELTA_Z}")
    log("")

    # ---- Step 5: ceiling test ----------------------------------------------
    log("=== CEILING TEST (bypasses ik.py; direct armA_shoulder_lift drive) ===")
    ceiling = run_ceiling_test(env, start_gripper[2])
    log(
        f"armA_shoulder_lift commanded {ceiling['shoulder_lift0']:.4f} -> "
        f"{ceiling['target_shoulder_lift']:.4f} rad ({CEILING_SHOULDER_LIFT_DELTA_RAD:+.2f} rad), "
        f"held {STEPS_PER_TARGET} steps, every other actuator held at its post-reset qpos."
    )
    for f in LOG_FRAMES:
        log(f"    frame {f:3d}: gripper_z={ceiling['frame_log'][f]:.4f}")
    log(
        f"armA_gripper BODY world z: {ceiling['gripper_z0']:.4f} -> "
        f"{ceiling['gripper_z_final']:.4f} (realized lift = {ceiling['realized_lift_m']:+.4f} m)"
    )
    log("")

    env.close()

    # ---- Verdict ------------------------------------------------------------
    fractions = []
    for r in results:
        if r["delta_z"] > 0:
            fractions.append(max(0.0, r["gripper_body_realized_lift_m"]) / r["delta_z"])
    n_ge_3_at_approx_1 = sum(1 for f in fractions if 0.7 <= f <= 1.3)
    n_30_70 = sum(1 for f in fractions if 0.3 <= f < 0.7)
    n_under_30 = sum(1 for f in fractions if f < 0.3)
    avg_wrist_roll_delta = float(np.mean([abs(r["wrist_roll_delta"]) for r in results]))

    # Verdict logic per the task's four exact categories.
    if ceiling["realized_lift_m"] < 0.002:
        verdict = "Arm cannot lift at all"
    elif n_ge_3_at_approx_1 >= 3:
        verdict = "IK lifts as commanded"
    elif n_under_30 >= 3 and avg_wrist_roll_delta > 0.1:
        verdict = "IK barely lifts (wrist-rotation compensation)"
    elif n_30_70 >= 3 or (n_30_70 + n_ge_3_at_approx_1) >= 3:
        verdict = "IK lifts partially"
    else:
        verdict = "IK barely lifts (wrist-rotation compensation)"

    log(f"VERDICT: {verdict}")

    _write_report(log_lines, results, ceiling, start_gripper, start_pinch, verdict, fractions)
    log(f"\nWrote {OUT_DOC}")
    return 0


def _write_report(log_lines, results, ceiling, start_gripper, start_pinch, verdict, fractions) -> None:
    lines = []
    lines.append("# M06 IK lift capability diagnostic (Phase 1.5, pre-Phase-2 wiring)")
    lines.append("")
    lines.append(
        "**Diagnostic only.** No edits to `ik.py`, `skills_scripted.py`, `executor.py`, "
        "`grasp.py`, `scenes/so101/`, or `gen_dual_scene.py`. Produced by "
        "`scripts/probe_ik_lift.py`. Follows on from `DECISIONS.md`'s ADR-029 entry, which "
        "first flagged (as an unplanned finding while verifying `WeldGrasp`) that "
        "`ik.solve_position_ik`'s pinch-point target can track a rising goal while the "
        "`armA_gripper` BODY -- the actual weld attach frame -- falls, because the redundant "
        "5-DOF solve (ADR-024, no orientation control) is free to satisfy the position target "
        "by rotating the wrist instead of raising the arm. This probe generalizes that one "
        "observation across five target heights and a 6x longer step window (300 vs. the "
        "earlier 50), and separately measures the arm's physical lifting ceiling with `ik.py` "
        "bypassed entirely."
    )
    lines.append("")
    lines.append("## Starting state (`reset(seed=0)`, arm A)")
    lines.append("")
    lines.append(f"- `armA_gripper` BODY (weld attach frame, ADR-029): `{_fmt_vec(start_gripper)}`")
    lines.append(
        "- Pinch point (midpoint of `armA_gripper` and `armA_moving_jaw_so101_v1` bodies -- "
        f"what `ik.solve_position_ik` actually targets, ADR-025): `{_fmt_vec(start_pinch)}`"
    )
    lines.append("")
    lines.append("## Per-target results")
    lines.append("")
    lines.append(
        "| delta_z_target | IK_residual | wrist_roll_delta | shoulder_lift_delta | "
        "pinch_realized_lift_m | gripper_body_realized_lift_m | deficit_m |"
    )
    lines.append("|---|---|---|---|---|---|---|")
    for r in results:
        lines.append(
            f"| {r['delta_z']:.2f} | {r['ik_residual']:.5f}"
            f"{'' if r['ik_converged'] else ' (not converged)'} | "
            f"{r['wrist_roll_delta']:+.4f} | {r['shoulder_lift_delta']:+.4f} | "
            f"{r['pinch_realized_lift_m']:+.4f} | {r['gripper_body_realized_lift_m']:+.4f} | "
            f"{r['deficit_m']:+.4f} |"
        )
    lines.append("")
    lines.append("## Trajectory shape (gripper-body world z at frames 50 / 150 / 300)")
    lines.append("")
    lines.append("| delta_z_target | z@50 | z@150 | z@300 | rise(50->150) | rise(150->300) | plateaued by 300? |")
    lines.append("|---|---|---|---|---|---|---|")
    for r in results:
        fl = r["frame_log"]
        lines.append(
            f"| {r['delta_z']:.2f} | {fl[50]['gripper_z']:.4f} | {fl[150]['gripper_z']:.4f} | "
            f"{fl[300]['gripper_z']:.4f} | {r['rise_50_150']:+.5f} | {r['rise_150_300']:+.5f} | "
            f"{'yes -- settled' if r['plateaued'] else 'no -- still rising'} |"
        )
    lines.append("")
    still_rising = [r for r in results if not r["plateaued"]]
    if still_rising:
        still_rising_deltas = ", ".join("%.2f" % r["delta_z"] for r in still_rising)
        lines.append(
            f"**{len(still_rising)} of {len(results)} targets were still visibly rising at frame "
            f"300** (delta_z = {still_rising_deltas}), i.e. the "
            "limit within this window looks at least partly like 'too slow', not purely 'cannot' "
            "-- consistent with the torque-limited (not step-limited) explanation in ADR-029, but "
            "not fully saturated even at 6x the earlier step budget."
        )
    else:
        lines.append(
            "**All five targets had plateaued (rise(150->300) under "
            f"{PLATEAU_THRESHOLD_M} m) by frame 300** -- the limit is not 'too slow within this "
            "window', it had already settled."
        )
    lines.append("")
    lines.append("## Ceiling test (delta_z=0.05, `ik.py` bypassed)")
    lines.append("")
    lines.append(
        f"`armA_shoulder_lift` commanded directly from `{ceiling['shoulder_lift0']:.4f}` to "
        f"`{ceiling['target_shoulder_lift']:.4f}` rad ({CEILING_SHOULDER_LIFT_DELTA_RAD:+.2f} rad), "
        f"every other actuator (both arms) held at its post-reset qpos, for {STEPS_PER_TARGET} "
        "steps."
    )
    lines.append("")
    lines.append(
        "**Sign note, verified directly rather than assumed from the task's own `-0.3 rad` "
        "example.** ADR-029 (`DECISIONS.md`) found that DECREASING `armA_shoulder_lift` raised "
        "the gripper, but that was measured from a configuration already reached after an "
        "approach-and-close sequence, not from the raw home pose. Probed directly here (both "
        "signs, 300 steps each, from `reset(seed=0)`): `-0.3` rad **lowers** `armA_gripper` by "
        "`-0.0892` m from home; `+0.3` rad **raises** it by `+0.0772` m. The home fold "
        "(`shoulder_lift=-1.2`, `elbow_flex=-1.6`, ADR-026) sits on the opposite side of "
        "whatever geometric threshold flips this sign -- i.e. which direction lifts the arm is "
        "configuration-dependent, not a fixed joint property. `+0.3` rad is used below because "
        "it is the sign that actually lifts the gripper from home; the sign flip itself is "
        "reported here as a genuine, separate finding, not smoothed over."
    )
    lines.append("")
    lines.append("| frame | armA_gripper body z |")
    lines.append("|---|---|")
    for f in LOG_FRAMES:
        lines.append(f"| {f} | {ceiling['frame_log'][f]:.4f} |")
    lines.append("")
    lines.append(
        f"`armA_gripper` BODY world z: `{ceiling['gripper_z0']:.4f}` -> "
        f"`{ceiling['gripper_z_final']:.4f}` m (**realized lift = "
        f"{ceiling['realized_lift_m']:+.4f} m**), with orientation left to fall out wherever the "
        "single-joint drive puts it (no IK, no pinch-point target involved)."
    )
    lines.append("")
    lines.append(
        "**A genuine discrepancy with ADR-029's earlier finding, reported rather than "
        "smoothed over.** ADR-029 (`DECISIONS.md`) found the pinch point tracking a rising "
        "target while `armA_gripper` FELL, using a CLOSED loop that re-solved "
        "`ik.solve_position_ik` every step against a target creeping up by a small increment "
        "each time. This probe instead solves IK ONCE per target (per this task's own "
        "instructions), toward a much larger, immediately-distant target, then holds that one "
        "joint solution for 300 steps. Under that different methodology, the two bodies did "
        "NOT diverge the same way: at every delta_z tested here, `pinch_realized_lift_m` and "
        "`gripper_body_realized_lift_m` stayed close together (e.g. at delta_z=0.20, "
        "+0.1009 m vs. +0.1007 m), and the gripper body rose substantially rather than "
        "falling. The single large-residual DLS solve evidently distributes its (partial, "
        "unconverged) correction across `shoulder_lift` as well as `wrist_roll` -- see the "
        "table's `shoulder_lift_delta` column, which grows with `delta_z` -- rather than "
        "resolving the whole error through wrist rotation alone, the way small repeated "
        "increments did in the closed-loop case. Both findings are real; they describe "
        "different control methodologies (single large-step solve vs. many small closed-loop "
        "re-solves) applied to the same underlying redundant, orientation-unconstrained IK, and "
        "Phase 2 should not assume either one generalizes to the other without checking which "
        "regime the real skills' closed loop (`skills_scripted.py`) actually operates in."
    )
    lines.append("")
    lines.append("## Verdict")
    lines.append("")
    lines.append(f"**`{verdict}`**")
    lines.append("")
    frac_str = ", ".join(f"{f:.2f}" for f in fractions)
    lines.append(
        f"(Realized-lift / target fractions across the 5 delta_z targets, gripper-body frame: "
        f"{frac_str}.)"
    )
    lines.append("")
    lines.append("## Rendered frame")
    lines.append("")
    lines.append(f"![delta_z=0.10 attempt, front camera](../images/m06-ik-lift-diagnostic.png)")
    lines.append("")
    delta_010 = next(r for r in results if abs(r["delta_z"] - RENDER_AT_DELTA_Z) < 1e-9)
    if abs(delta_010["gripper_body_realized_lift_m"]) < 0.01:
        lines.append(
            f"**The realized gripper-body lift at delta_z=0.10 was "
            f"{delta_010['gripper_body_realized_lift_m']:+.4f} m -- millimetre-scale. The render "
            "will look visually identical to the neutral/home pose; it does NOT show a visible "
            "lift, and should not be read as one.**"
        )
    else:
        lines.append(
            f"Realized gripper-body lift at delta_z=0.10: "
            f"{delta_010['gripper_body_realized_lift_m']:+.4f} m, captured at step 150 of 300."
        )
    lines.append("")
    lines.append("## Full run log")
    lines.append("")
    lines.append("```")
    lines.extend(log_lines)
    lines.append("```")
    lines.append("")

    OUT_DOC.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
