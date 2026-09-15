"""Cubic-spline joint-space motion primitive (v2 Stage 2, ADR-071).

**Why this file exists.** Every skill in `skills_scripted.py` (Stage-1-and-
earlier, unmodified by this stage) drives an arm with a *closed-loop* IK
solve every control step: "where do I want the gripper next, solve joints,
step, repeat." That is robust to disturbances but says nothing about *how
smoothly* the arm gets from A to B -- each step's IK target can jump by
whatever the outer loop asked for. This module is a different, simpler
primitive: **given two JOINT configurations (not Cartesian points) and a
duration, produce a smooth, bounded-velocity, bounded-acceleration reference
trajectory between them, and drive the real PD position actuators along
it.** It has no IK in it at all -- the caller (a later v2 stage, or a
hand-written test) supplies `q_target` directly, e.g. from
`ik_geometric.solve_topdown_ik` or `ik.solve_position_ik`.

**A note for a reader new to robotics: why not just write the target
straight to `ctrl` and let the PD controller "figure it out"?** MuJoCo's
`<position>` actuators here are PD (proportional-derivative) controllers: at
every physics step they compute a force proportional to
`kp * (ctrl - qpos) - kv * qvel` (roughly -- see the `sts3215` actuator
class in the compiled scene, `kp=998.22`). If you write a DISTANT target to
`ctrl` in one shot and hold it there (the "direct command" comparison this
module's validation script measures), that proportional term is huge at
first (large position error) and shrinks as the joint catches up -- the
joint accelerates hard immediately, then decelerates as it approaches,
which is a valid way to get there but is not a *chosen* velocity/
acceleration profile, just whatever the servo gains produce. Writing a
smoothly-changing intermediate SETPOINT instead (this module's whole job)
means the position error the PD controller ever sees is always small, so
the realized motion tracks a trajectory *we* designed to have bounded,
predictable velocity and acceleration -- useful for choosing a duration
that respects real actuator/velocity limits, and useful for holding a
gentler, more controlled motion (e.g. when carrying something) than
"floor it and let the gains sort it out."

**The cubic.** For each of the 5 driven joints independently, fit a
1-D cubic `q(t) = a0 + a1*t + a2*t^2 + a3*t^3` on `t in [0, T]` with the
boundary conditions `q(0)=q_start`, `q(T)=q_target`, `q'(0)=0`, `q'(T)=0`
(start and end at rest -- appropriate between discrete skill phases, where
the arm really is momentarily stationary). Solving those four conditions
for the four unknowns gives the closed form used here (this task's own
brief, already verified analytically by the task; not re-derived or
"improved" here):

    a0 = q_start
    a1 = 0
    a2 =  3*(q_target - q_start) / T**2
    a3 = -2*(q_target - q_start) / T**3

and, since `q(t)` is a plain cubic, `q'(t) = a1 + 2*a2*t + 3*a3*t**2`
follows by ordinary differentiation. `t` is always clamped to `[0, T]`
before being plugged into either formula, so a caller that (for any
reason) asks for a `t` outside the move's own duration gets the exact
boundary value/velocity rather than extrapolating the cubic past where its
boundary conditions were derived.

**The idle-arm hold (ADR-037 pattern, reused here, not re-derived).**
`DECISIONS.md`'s ADR-037 entry measured a bug in the OLDER hold mechanism
(`skills_scripted._hold_ctrl` with no snapshot argument): rebuilding the
held arm's ctrl vector from its CURRENT qpos on every single step supplies
zero position error -- and therefore zero PD restoring force against
gravity -- at the instant of every read, so the held arm's true setpoint
never actually opposes gravity and the arm sags: measured at 0.04 rad by
100 steps, saturating at a joint hard limit by ~1500 steps. The fix (and
the pattern this module reuses) is to capture ONE snapshot, at the exact
instant the hold begins, and pass that SAME array back in unchanged on
every subsequent step -- so the PD controller always sees the ORIGINAL
pose as its target and genuinely resists gravity/disturbance the whole
time, instead of chasing wherever the arm has already drifted to.
`_frozen_hold_snapshot`/`_ctrl_with_driven_slice` below are this module's
OWN, self-contained implementation of that same pattern (motion.py does not
import `skills_scripted.py`'s private helpers -- it is a separate,
independent primitive per this stage's brief, so it re-implements the same
verified idea rather than reaching into that module's internals).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from bimanual.control import ik

# ---------------------------------------------------------------------------
# Tuning constants, centralized (project convention -- see ik.py's own
# module-level constants block for the same pattern).
# ---------------------------------------------------------------------------

#: How close (radians, per joint, Euclidean over the 5-vector) the driven
#: joints' ACTUAL qpos must land to `q_target` at the end of the move for
#: `MotionResult.success` to be True. Not zero -- a PD-tracked reference
#: always carries a small residual position error even at rest (the
#: proportional term needs a nonzero error to produce the force that
#: cancels gravity/friction). Set to the same order of magnitude as
#: ADR-037's own "genuinely held" bar (0.01 rad) since both are asking the
#: same underlying question: "is this joint sitting where we asked it to,
#: not just approximately"-- rather than inventing an unrelated number.
MOTION_SUCCESS_TOL_RAD = 0.01


# ---------------------------------------------------------------------------
# Result type.
# ---------------------------------------------------------------------------


@dataclass
class MotionResult:
    """Outcome of one `move_to_config` or `set_gripper` call.

    Carries everything `scripts/v2_validate_motion.py`'s mandatory
    validation needs: (a) TRACKING wants commanded-vs-actual position at
    every physics step for the driven joint(s) (`t_log`/`commanded_log`/
    `actual_log`), plus, since the idle-arm-drift gate (c) needs to see the
    OTHER arm's (and every other joint's) actual qpos too, the full model
    qpos at every step (`qpos_full_log`) -- logging the whole vector is
    cheap (this scene's `nq` is well under 100) and means the validation
    script needs no second, separately-instrumented copy of this loop to
    check drift.

    Attributes:
        success: True if every driven joint's final actual qpos is within
            `MOTION_SUCCESS_TOL_RAD` of `q_target` (Euclidean over the
            driven joints). False does not necessarily mean a bug --a
            target near a joint limit, or a duration too short for the
            actuator's real gain to keep up, can legitimately fail this.
        reason: Human-readable explanation, e.g. "reached target" or
            "final error 0.0123 rad > tol 0.01 rad".
        frames_used: Number of `env.step()` calls actually issued --
            always `round(duration_s / model.opt.timestep)` for this
            primitive (it always runs the full commanded duration; there is
            no early-exit/timeout path, unlike the closed-loop skills in
            `skills_scripted.py`, because the whole point here is a FIXED,
            predetermined-duration reference, not a converge-when-you-can
            servo).
        arm: Which arm this call drove ("A" or "B").
        joint_names: Names of the driven joint(s)/actuator(s), in the order
            `commanded_log`/`actual_log`'s columns are laid out. Length 5
            for `move_to_config`, length 1 for `set_gripper`.
        duration_s: The commanded move duration `T`, seconds.
        q_start: The driven joints' qpos at the instant the move began
            (this call's own measured `a0` -- see module docstring).
        q_target: The requested end configuration.
        t_log: Simulated time (seconds, 0..duration_s) at each logged
            sample, length `frames_used + 1` (sample 0 is the pre-move
            state, before any `env.step()`; every following sample is
            taken immediately AFTER that step's `env.step()` call).
        commanded_log: `(frames_used+1, len(joint_names))` -- the spline
            reference position written to `data.ctrl` at (or, for sample 0,
            the value that WOULD have been written at t=0, i.e. `q_start`)
            each logged instant.
        actual_log: `(frames_used+1, len(joint_names))` -- the driven
            joints' REAL qpos read back after each step (sample 0 is the
            pre-move qpos).
        qpos_full_log: `(frames_used+1, model.nq)` -- the ENTIRE model's
            qpos at each logged instant (same sample timing as the above).
            This is what lets a validation script measure "how much did
            some joint that this call never touched move" without a
            second, separately-instrumented loop.
        hold_other_arm: Echoes the argument this call was made with.
        frozen_snapshot: The ADR-037-pattern ctrl snapshot captured once at
            the start of the move (and held fixed for every non-driven
            actuator throughout), if `hold_other_arm=True`; `None`
            otherwise. Exposed so a caller/validator can directly compare
            "what was the hold target" against `qpos_full_log`.
    """

    success: bool
    reason: str
    frames_used: int
    arm: str
    joint_names: list[str]
    duration_s: float
    q_start: np.ndarray
    q_target: np.ndarray
    t_log: np.ndarray
    commanded_log: np.ndarray
    actual_log: np.ndarray
    qpos_full_log: np.ndarray
    hold_other_arm: bool
    frozen_snapshot: np.ndarray | None = field(default=None)


# ---------------------------------------------------------------------------
# Cubic spline coefficients and evaluation -- exactly the formulas given in
# the task brief, not "improved". See module docstring for the derivation
# and boundary conditions.
# ---------------------------------------------------------------------------


def _cubic_coeffs(q_start: np.ndarray, q_target: np.ndarray, duration_s: float):
    """Per-joint cubic coefficients for a zero-start/zero-end-velocity move
    from `q_start` to `q_target` over `duration_s` seconds. Returns
    `(a0, a1, a2, a3)`, each shaped like `q_start`."""
    T = float(duration_s)
    a0 = np.array(q_start, dtype=np.float64, copy=True)
    a1 = np.zeros_like(a0)
    a2 = 3.0 * (q_target - q_start) / (T ** 2)
    a3 = -2.0 * (q_target - q_start) / (T ** 3)
    return a0, a1, a2, a3


def _cubic_eval(a0, a1, a2, a3, t: float, duration_s: float):
    """Evaluate position and velocity at time `t`, clamped to `[0, duration_s]`
    (see module docstring: never extrapolate past the boundary conditions
    the coefficients were derived for)."""
    t_c = min(max(t, 0.0), duration_s)
    pos = a0 + a1 * t_c + a2 * (t_c ** 2) + a3 * (t_c ** 3)
    vel = a1 + 2.0 * a2 * t_c + 3.0 * a3 * (t_c ** 2)
    return pos, vel


# ---------------------------------------------------------------------------
# Low-level MuJoCo lookups, kept local to this module (see module docstring:
# deliberately NOT importing skills_scripted.py's private helpers).
# ---------------------------------------------------------------------------


def _actuator_id(model, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid == -1:
        raise ValueError(f"actuator {name!r} not found in the compiled model")
    return aid


def _joint_id(model, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid == -1:
        raise ValueError(f"joint {name!r} not found in the compiled model")
    return jid


def _other_arm(arm: str) -> str:
    if arm == "A":
        return "B"
    if arm == "B":
        return "A"
    raise ValueError(f"arm must be 'A' or 'B', got {arm!r}")


def _full_ctrl_from_current_qpos(env) -> np.ndarray:
    """A length-`model.nu` ctrl vector where every actuator's target equals
    ITS OWN joint's CURRENT qpos -- i.e. "hold exactly where the whole
    model is right now". Read ONCE per the ADR-037 freeze pattern (see
    module docstring); a caller must not call this again mid-hold, or it
    reproduces the exact per-step qpos-chasing drift bug ADR-037 fixed.
    """
    ctrl = np.zeros(env.model.nu, dtype=np.float64)
    for aid in range(env.model.nu):
        jid = int(env.model.actuator_trnid[aid, 0])
        qadr = int(env.model.jnt_qposadr[jid])
        ctrl[aid] = env.data.qpos[qadr]
    return ctrl


# ---------------------------------------------------------------------------
# The public primitives.
# ---------------------------------------------------------------------------


def move_to_config(
    env,
    arm: str,
    q_target,
    duration_s: float,
    hold_other_arm: bool = True,
) -> MotionResult:
    """Drive `arm`'s 5 positioning actuators from their CURRENT
    configuration to `q_target` along a per-joint cubic (zero start/end
    velocity), over `duration_s` seconds of SIMULATED time.

    Writes an interpolated setpoint to `data.ctrl` every physics step (via
    `env.step()`) so the PD position actuators track a smooth reference
    instead of racing at a distant target in one shot -- see the module
    docstring for why that distinction matters.

    Args:
        env: A live `TableSettingEnv` (or anything exposing the same
            `.model`/`.data`/`.step()` contract).
        arm: "A" or "B" -- which arm's 5 positioning joints to drive.
        q_target: Length-5 array-like, target joint angles in
            `ik.ARM_JOINT_SUFFIXES` order (shoulder_pan, shoulder_lift,
            elbow_flex, wrist_flex, wrist_roll), radians. Every entry must
            lie inside that joint's `model.jnt_range` -- out-of-range
            entries raise `ValueError` immediately (fail loud, never a
            silently-clamped target the caller did not ask for).
        duration_s: Total move duration, simulated seconds. Must be > 0.
            Converted to a step count as `round(duration_s /
            model.opt.timestep)` (the model's own timestep is read live,
            not hardcoded, though this scene's is 0.002 s).
        hold_other_arm: If True (default), the OTHER arm (and this arm's
            own gripper, which `move_to_config` never drives) is frozen via
            the ADR-037 pattern: one ctrl snapshot captured from live qpos
            at the instant this call begins, rewritten UNCHANGED on every
            subsequent step -- never re-derived from live qpos mid-move
            (that re-derivation is the exact bug ADR-037 fixed; see module
            docstring). If False, every non-driven actuator's ctrl is left
            at whatever it already was (read once, held constant, but from
            the PRE-EXISTING `data.ctrl` rather than a fresh qpos read) --
            deliberately NOT the old per-step qpos-chasing behaviour either
            way, since reproducing a known, already-fixed drift bug as a
            new code path's "off" switch would be a worse default than
            simply not touching those actuators' commands at all.

    Returns:
        A `MotionResult`. See that dataclass's docstring for every field.
    """
    if arm not in ("A", "B"):
        raise ValueError(f"arm must be 'A' or 'B', got {arm!r}")
    if duration_s <= 0.0:
        raise ValueError(f"duration_s must be > 0, got {duration_s!r}")

    model = env.model
    joint_names = ik.arm_joint_names(arm)  # 5 names, ARM_JOINT_SUFFIXES order
    actuator_ids = [_actuator_id(model, name) for name in joint_names]
    joint_ids = [_joint_id(model, name) for name in joint_names]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    jnt_ranges = [tuple(model.jnt_range[j]) for j in joint_ids]

    q_target = np.asarray(q_target, dtype=np.float64).reshape(-1)
    if q_target.shape[0] != len(joint_names):
        raise ValueError(
            f"q_target has length {q_target.shape[0]}, expected {len(joint_names)} "
            f"(one angle per positioning joint: {joint_names})"
        )
    for name, val, (lo, hi) in zip(joint_names, q_target, jnt_ranges):
        if not (lo <= val <= hi):
            raise ValueError(
                f"q_target[{name!r}]={val!r} is outside this joint's jnt_range "
                f"[{lo}, {hi}] -- move_to_config never silently clamps a target."
            )

    q_start = np.array([env.data.qpos[a] for a in qpos_adrs], dtype=np.float64)

    dt = float(model.opt.timestep)
    n_steps = int(round(duration_s / dt))
    if n_steps < 1:
        n_steps = 1

    a0, a1, a2, a3 = _cubic_coeffs(q_start, q_target, duration_s)

    # ADR-037 freeze pattern (see module docstring): capture ONE snapshot
    # before any step of this move, never re-derived from live qpos again.
    frozen_snapshot: np.ndarray | None
    if hold_other_arm:
        frozen_snapshot = _full_ctrl_from_current_qpos(env)
        base_ctrl = frozen_snapshot
    else:
        # Leave every non-driven actuator at whatever data.ctrl already
        # commanded -- read once, held constant, NOT re-derived from qpos
        # (see this function's own `hold_other_arm` docstring paragraph).
        frozen_snapshot = None
        base_ctrl = np.array(env.data.ctrl, dtype=np.float64, copy=True)

    n_joints = len(joint_names)
    t_log = np.zeros(n_steps + 1, dtype=np.float64)
    commanded_log = np.zeros((n_steps + 1, n_joints), dtype=np.float64)
    actual_log = np.zeros((n_steps + 1, n_joints), dtype=np.float64)
    qpos_full_log = np.zeros((n_steps + 1, model.nq), dtype=np.float64)

    # Sample 0: pre-move state, before any env.step() this call issues.
    t_log[0] = 0.0
    commanded_log[0] = q_start
    actual_log[0] = q_start
    qpos_full_log[0] = np.array(env.data.qpos, dtype=np.float64, copy=True)

    for i in range(1, n_steps + 1):
        t = i * dt
        pos, _vel = _cubic_eval(a0, a1, a2, a3, t, duration_s)

        ctrl = np.array(base_ctrl, dtype=np.float64, copy=True)
        for k, aid in enumerate(actuator_ids):
            ctrl[aid] = pos[k]

        # cameras=[] (explicit empty list, not None/omitted): this env's
        # instance camera default could be non-empty (ADR-046's own
        # per-call-override note in skills_scripted.py makes the identical
        # point) -- omitting this would pay a full render cost on every one
        # of up to 1000 physics steps for a stream nobody reads here.
        env.step(ctrl, cameras=[])

        t_c = min(t, duration_s)
        t_log[i] = t_c
        commanded_log[i] = pos
        actual_log[i] = np.array([env.data.qpos[a] for a in qpos_adrs], dtype=np.float64)
        qpos_full_log[i] = np.array(env.data.qpos, dtype=np.float64, copy=True)

    final_error = float(np.linalg.norm(actual_log[-1] - q_target))
    success = final_error < MOTION_SUCCESS_TOL_RAD
    reason = (
        "reached target"
        if success
        else f"final error {final_error:.6f} rad > tol {MOTION_SUCCESS_TOL_RAD} rad"
    )

    return MotionResult(
        success=success,
        reason=reason,
        frames_used=n_steps,
        arm=arm,
        joint_names=joint_names,
        duration_s=float(duration_s),
        q_start=q_start,
        q_target=q_target,
        t_log=t_log,
        commanded_log=commanded_log,
        actual_log=actual_log,
        qpos_full_log=qpos_full_log,
        hold_other_arm=hold_other_arm,
        frozen_snapshot=frozen_snapshot,
    )


def set_gripper(env, arm: str, open: bool, duration_s: float = 0.6) -> MotionResult:
    """Apply the SAME cubic-spline profile as `move_to_config`, restricted
    to `arm`'s single gripper (jaw open/close) joint alone.

    Every other actuator (both arms' 5 positioning joints, and the OTHER
    arm's gripper) is frozen via the identical ADR-037 snapshot-once
    pattern `move_to_config` uses when `hold_other_arm=True` -- there is no
    `hold_other_arm` parameter here because this call always freezes
    everything it does not itself drive (opening/closing a gripper has no
    "let the rest of the scene react freely" use case the way a
    multi-second arm reposition might).

    Args:
        env: A live `TableSettingEnv`.
        arm: "A" or "B".
        open: True drives the jaw to its actuator's `ctrlrange` upper bound
            (fully open); False drives it to the lower bound (fully
            closed) -- the same `GRIPPER_OPEN_FRACTION=1.0` /
            `GRIPPER_CLOSE_FRACTION=0.0` convention `skills_scripted.py`
            uses (`model.actuator_ctrlrange`, read fresh from the compiled
            model, not hardcoded).
        duration_s: Move duration, seconds. Default 0.6 s.

    Returns:
        A `MotionResult` with `joint_names` of length 1 (the gripper joint
        alone) -- see `MotionResult`'s docstring for the full field list.
    """
    if arm not in ("A", "B"):
        raise ValueError(f"arm must be 'A' or 'B', got {arm!r}")
    if duration_s <= 0.0:
        raise ValueError(f"duration_s must be > 0, got {duration_s!r}")

    model = env.model
    gripper_name = ik.gripper_joint_name(arm)
    aid = _actuator_id(model, gripper_name)
    jid = int(model.actuator_trnid[aid, 0])
    qpos_adr = int(model.jnt_qposadr[jid])
    lo, hi = model.actuator_ctrlrange[aid]

    q_start = np.array([env.data.qpos[qpos_adr]], dtype=np.float64)
    q_target = np.array([hi if open else lo], dtype=np.float64)

    dt = float(model.opt.timestep)
    n_steps = int(round(duration_s / dt))
    if n_steps < 1:
        n_steps = 1

    a0, a1, a2, a3 = _cubic_coeffs(q_start, q_target, duration_s)

    # Always freeze everything else (see docstring above) -- ADR-037
    # pattern, one snapshot, never re-derived mid-move.
    frozen_snapshot = _full_ctrl_from_current_qpos(env)

    t_log = np.zeros(n_steps + 1, dtype=np.float64)
    commanded_log = np.zeros((n_steps + 1, 1), dtype=np.float64)
    actual_log = np.zeros((n_steps + 1, 1), dtype=np.float64)
    qpos_full_log = np.zeros((n_steps + 1, model.nq), dtype=np.float64)

    t_log[0] = 0.0
    commanded_log[0] = q_start
    actual_log[0] = q_start
    qpos_full_log[0] = np.array(env.data.qpos, dtype=np.float64, copy=True)

    for i in range(1, n_steps + 1):
        t = i * dt
        pos, _vel = _cubic_eval(a0, a1, a2, a3, t, duration_s)

        ctrl = np.array(frozen_snapshot, dtype=np.float64, copy=True)
        ctrl[aid] = pos[0]

        env.step(ctrl, cameras=[])

        t_c = min(t, duration_s)
        t_log[i] = t_c
        commanded_log[i] = pos
        actual_log[i] = env.data.qpos[qpos_adr]
        qpos_full_log[i] = np.array(env.data.qpos, dtype=np.float64, copy=True)

    final_error = float(abs(actual_log[-1, 0] - q_target[0]))
    success = final_error < MOTION_SUCCESS_TOL_RAD
    reason = (
        "reached target"
        if success
        else f"final error {final_error:.6f} rad > tol {MOTION_SUCCESS_TOL_RAD} rad"
    )

    return MotionResult(
        success=success,
        reason=reason,
        frames_used=n_steps,
        arm=arm,
        joint_names=[gripper_name],
        duration_s=float(duration_s),
        q_start=q_start,
        q_target=q_target,
        t_log=t_log,
        commanded_log=commanded_log,
        actual_log=actual_log,
        qpos_full_log=qpos_full_log,
        hold_other_arm=True,
        frozen_snapshot=frozen_snapshot,
    )


if __name__ == "__main__":
    # Guarded per builder convention. No meaningful standalone run without a
    # compiled MuJoCo model (ADR-020's premise is stale THIS session per the
    # task brief -- MuJoCo does import on the laptop -- but the mandatory
    # validation is still scripts/v2_validate_motion.py, kept as one script
    # so its bm-ptl re-run is a single invocation).
    print(__doc__)
    print(
        "\nThis module has no standalone entry point. Run "
        "scripts/v2_validate_motion.py instead (works on both laptop and bm-ptl "
        "this session, per the task brief; the numbers reported in DECISIONS.md "
        "ADR-071 are the bm-ptl run per ADR-047)."
    )
