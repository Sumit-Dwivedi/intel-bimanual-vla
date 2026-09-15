"""Geometric, closed-form, top-down-only IK for the SO-101 arm (v2 Stage 1,
ADR-070).

**Why this file exists next to `ik.py`.** `ik.py` (unmodified, per this
stage's instructions) is a general-purpose *numerical* solver: damped least
squares, position-only, any target reachable from any starting pose, no
orientation guarantee at all ("roll falls out", ADR-024). This module is the
opposite trade: it only ever produces ONE specific kind of grasp -- the
gripper pointing straight down at the target, optionally yawed about the
vertical -- but for that restricted family it is a **closed-form** solution:
no iteration, no local minima, no starting-pose dependence, sub-millimetre
accuracy by construction (see the validation numbers in
`docs/hardware/v2-topdown-workspace.md`). Think of `ik.py` as "reach this
point, I don't care how the hand is oriented" and this module as "reach this
point with the hand pointing straight down, like a human reaching into a
drawer from above."

**A note for a reader new to robotics: what a "closed form" IK even means.**
`ik.py`'s solver works by *trial and improvement*: guess a joint
configuration, see how far the fingertip is from the goal, nudge the joints
a little in the direction that reduces that error, repeat until close enough
(or give up). That is robust but approximate and can get stuck. A
*closed-form* solution instead writes down the exact algebra relating joint
angles to end-effector position for THIS SPECIFIC arm shape, then solves
that algebra directly for the joint angles that hit an EXACT target -- like
solving `2x + 3 = 7` for `x` directly (`x = 2`) instead of guessing `x=1`,
checking, guessing `x=3`, checking, and converging. This is only possible
because the SO-101, restricted to top-down grasps, happens to decompose into
geometry simple enough to solve by hand (a 2-link "elbow" triangle, plus a
few trig angle constraints) -- see the "Kinematic structure" section below.

**Kinematic structure (why this arm decomposes this way).** The five
*positioning* joints (`ik.py`'s `ARM_JOINT_SUFFIXES`, reused here rather than
redefined) are, in chain order: `shoulder_pan`, `shoulder_lift`,
`elbow_flex`, `wrist_flex`, `wrist_roll`. Measured directly from the compiled
model (`_calibrate()` below, via `mujoco.mj_forward` on a scratch `MjData`
reset to qpos=0 for this arm -- NOT hardcoded from the numbers in the task
brief, per that brief's own instruction to re-verify from the MJCF):

  - `shoulder_lift`, `elbow_flex` and `wrist_flex` all rotate about the
    SAME axis direction (measured: very close to world `[-1, 0, 0]` at the
    zero pose). Three hinges that share an axis direction, even though each
    pivots about its OWN anchor point, form a classic **planar 3R chain**:
    the standard "2-link arm plus a wrist that sets the final link's
    absolute angle" shape. This is why a closed form exists at all -- a
    chain of hinges with three DIFFERENT axis directions generally does not
    decompose this cleanly.
  - `shoulder_pan` rotates about a vertical axis (measured: very close to
    world `[0, 0, -1]`), so it is the classic "waist" joint that swings the
    entire planar sub-chain's vertical plane around like a lighthouse beam.
  - `wrist_roll`'s axis is tilted (not aligned with any single world axis)
    and, critically, the ADR-025 pinch point (see below) does NOT sit
    exactly on that axis -- rolling the wrist therefore traces the pinch
    point on a small (~1.6 cm radius) cone/circle rather than leaving it
    fixed. This is the "geometric subtlety" the task brief calls out, and
    it is why `wrist_roll` cannot be a pure "spin the fingers, nothing else
    moves" joint here (see "Yaw and the lateral offset" below).

**The pinch point (ADR-025) is not on the pan axis's own vertical plane.**
`ik.py`'s own module docstring already establishes ADR-025: the point that
actually pinches an object is the midpoint of bodies `armX_gripper` and
`armX_moving_jaw_so101_v1`, not the `armX_gripperframe` site. This module
goes one step further and notes a SECOND, independent offset: even the
*shoulder_lift/elbow_flex/wrist_flex* sub-chain's own plane (measured at
local x=0.01828 m at the zero pose) does not pass through the `shoulder_pan`
rotation axis (which sits at local x=0). So there are, in effect, TWO
lateral ("out of the naive plane") offsets stacked on top of each other:
(a) the constant ~0.018 m the whole arm's elbow plane sits away from the pan
axis, and (b) a SECOND, `wrist_roll`-angle-dependent offset from the pinch
point's asymmetric position relative to the wrist_roll axis. A naive
`theta1 = atan2(target_y, target_x)` ignores both and is systematically
wrong. The closed form below solves `theta1` accounting for BOTH offsets at
once, using the classic "shoulder-offset" (sometimes called "left-arm/
right-arm") technique: the true lateral offset traces a circle of some
radius `d` around the pan axis as pan sweeps, so the target must first be
projected onto that circle (`r_perp = sqrt(dist_to_pan_axis**2 - d**2)`)
before `theta1` is read off with `atan2`. See `_solve_pan_and_planar` below
for the exact algebra.

**Yaw and the lateral offset -- why "top-down" cannot be geometrically
perfect for every yaw.** Section "Kinematic structure" above already flagged
that the pinch point is not exactly on the `wrist_roll` axis. Consequence:
there is a genuine, small, UNAVOIDABLE residual tilt away from pure
straight-down, whose size depends on `wrist_roll`'s angle. This module
picks `wrist_roll`'s angle as `theta5_star(arm) + yaw`, where
`theta5_star(arm)` is calibrated (once per arm, cached) to be the ONE
`wrist_roll` angle that gets that residual tilt as close to zero as
mechanically possible (see `_theta5_star`) -- so `yaw=0.0` (the function's
default) gives the BEST achievable top-down alignment, and larger `|yaw|`
trades a little of that alignment away in exchange for rotating the
approach direction about (approximately) the vertical, exactly as a real
wrist-roll-sets-yaw mechanism does once the arm is posed to point down. The
mandatory validation (`scripts/v2_validate_ik.py`) is run at the default
`yaw=0.0`, which is the case this trade-off is tuned for.

**5 joints, 5 constraints.** Position (3: x, y, z) + "point straight down"
(1: the net pitch of the 3R sub-chain, chosen so the wrist_flex-to-pinch
segment's in-plane component points along -z) + yaw (1: `wrist_roll`'s
angle, offset from its calibrated optimum) = 5 scalar equations for the 5
unknown joint angles. This is why the system is exactly determined rather
than needing a numerical (least-squares, potentially non-converging) solve.

**Solve order** (see `solve_topdown_ik` for the implementation):
  1. `wrist_roll` (`theta5`) is set directly from `yaw` (no dependency on
     the target position at all).
  2. The net pitch `Theta = theta2+theta3+theta4` of the 3R sub-chain is
     computed in closed form so the tool points straight down, given
     `theta5` from step 1.
  3. `shoulder_pan` (`theta1`) and the individual `shoulder_lift`/
     `elbow_flex` split (`theta2`, `theta3`) are solved together: `theta1`
     from the shoulder-offset circle method, `theta2`/`theta3` from the
     standard law-of-cosines 2-link IK (tried at up to 4 branches: two
     choices of which side of the offset circle the elbow lands on, times
     two choices of "elbow up" vs "elbow down" -- the first branch that
     lands inside every joint's `model.jnt_range` is returned).
  4. `wrist_flex` (`theta4`) is whatever is left over: `Theta - theta2 -
     theta3`.

Every angle is checked against `model.jnt_range` before being accepted; if
no branch satisfies all five ranges (or the target is outside the
`shoulder_pan` offset circle's radius, or outside the 2-link arm's
reachable annulus), `solve_topdown_ik` returns `None` -- exactly like a
"this target is unreachable this way" answer, never a silently-wrong pose.

**Development note.** This algebra was derived and independently verified
against its own forward-kinematics function in pure Python/NumPy (no
MuJoCo) before ever touching bm-ptl: solving IK for a random target and then
re-running forward kinematics on the solved angles reproduces the target to
machine precision (~1e-16 m) when working from the SAME calibrated
constants used to derive the closed form. That self-consistency check does
not, by itself, prove the CONSTANTS this module reads via `_calibrate()`
match the real compiled MJCF -- that is exactly what
`scripts/v2_validate_ik.py`'s round-trip-against-real-MuJoCo gate checks
next, on bm-ptl (ADR-020: MuJoCo will not import on the laptop at all).
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from bimanual.control.ik import (
    ARM_JOINT_SUFFIXES,
    arm_joint_names,
    fixed_jaw_body_name,
    moving_jaw_body_name,
)

# ---------------------------------------------------------------------------
# Small linear-algebra helpers. Deliberately generic (they take an arbitrary
# axis vector, not an assumed "x" or "z") so this code is correct even if a
# future re-measurement finds an axis slightly off the values quoted in the
# task brief -- see the module docstring's "Development note".
# ---------------------------------------------------------------------------


def _unit(v: np.ndarray) -> np.ndarray:
    """Normalize a vector to unit length."""
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError(f"cannot normalize a near-zero vector: {v}")
    return v / n


def _skew(v: np.ndarray) -> np.ndarray:
    """The 3x3 skew-symmetric "cross product matrix" of `v`, i.e. the matrix
    K such that `K @ x == np.cross(v, x)` for any x. Used by the Rodrigues
    rotation formula below."""
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ]
    )


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    """The 3x3 rotation matrix for rotating by `angle` radians about `axis`
    (right-hand rule -- point your right thumb along `axis`, fingers curl in
    the direction of positive `angle`). `axis` need not be unit length; it is
    normalized here.

    This is exactly the convention MuJoCo hinge joints use: for a joint whose
    axis (in world frame, at the current configuration) is `data.xaxis[j]`,
    increasing that joint's `qpos` by `angle` rotates everything downstream
    of it by `_rodrigues(data.xaxis[j], angle)` about the joint's anchor
    (`data.xanchor[j]`). Because this module always feeds MEASURED axis
    vectors (read via `mj_forward`, never hand-typed sign-flips) into this
    function, there is no separate "verify the sign convention" step needed
    per joint -- using the real axis vector directly reproduces MuJoCo's
    actual rotation direction by construction. (The task brief's own
    sign-convention numerical check for `shoulder_pan` is still reproduced
    by `scripts/v2_validate_ik.py` as a cross-check, but this solver does not
    depend on it.)
    """
    a = _unit(axis)
    K = _skew(a)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def _rotate_about(anchor: np.ndarray, axis: np.ndarray, angle: float, point: np.ndarray) -> np.ndarray:
    """Rotate `point` by `angle` about the line through `anchor` in direction
    `axis` -- i.e. what one hinge joint does to any point rigidly attached
    downstream of it."""
    R = _rodrigues(axis, angle)
    return anchor + R @ (np.asarray(point, dtype=np.float64) - np.asarray(anchor, dtype=np.float64))


def _signed_angle_about_axis(u: np.ndarray, v: np.ndarray, axis: np.ndarray) -> float:
    """The signed angle (radians) such that `_rodrigues(axis, angle) @ u`
    points in the same direction as `v`, i.e. "how far to rotate u onto v,
    turning about axis". Both `u` and `v` are first projected perpendicular
    to `axis` (their component along `axis` is irrelevant to this rotation
    and is dropped) -- this is what makes it safe to call with vectors that
    are not already exactly perpendicular to `axis`.

    Standard formula: for vectors u_perp, v_perp perpendicular to a unit
    axis a, `cos(angle) = u_perp . v_perp` (after normalizing) and
    `sin(angle) = a . (u_perp x v_perp)`; `atan2` recovers the signed angle
    in one step without a quadrant-ambiguity headache.
    """
    a = _unit(axis)
    u_perp = np.asarray(u, dtype=np.float64) - np.dot(u, a) * a
    v_perp = np.asarray(v, dtype=np.float64) - np.dot(v, a) * a
    cos_t = np.dot(u_perp, v_perp)
    sin_t = np.dot(a, np.cross(u_perp, v_perp))
    return float(np.arctan2(sin_t, cos_t))


#: Arbitrary fixed reference direction for `_chain_angle` below. Only
#: requirement: not parallel to the shared shoulder_lift/elbow_flex/
#: wrist_flex axis (measured close to world x, either sign -- see
#: `_chain_angle`'s docstring for why "either sign" matters here). World y
#: is safely perpendicular to a world-x-ish axis on this arm.
_CHAIN_ANGLE_REF = np.array([0.0, 1.0, 0.0])


def _chain_angle(v: np.ndarray, axis: np.ndarray) -> float:
    """The signed angle (radians) from a fixed reference direction
    (`_CHAIN_ANGLE_REF`) to `v`, measured about `axis`, using the SAME
    convention `_rodrigues(axis, theta)` uses for positive rotation.

    This is a thin wrapper around `_signed_angle_about_axis` -- the point of
    giving it its own name is the GUARANTEE it relies on: for any two
    vectors `v1`, `v2` and any angle `theta` with `v2 =
    _rodrigues(axis, theta) @ v1`, `_chain_angle(v2, axis) - _chain_angle(v1,
    axis) == theta` exactly (angles measured about a common axis, from a
    common reference, compose by simple subtraction/addition). The
    law-of-cosines algebra in `solve_topdown_ik` leans on that guarantee to
    convert between "absolute angle of a link" and "this joint's own signed
    rotation from zero".

    **A caught, arm-specific bug this replaced.** An earlier version of
    this function hand-picked the `(y, z)` plane with a HARDCODED sign
    (`-atan2(v.z, v.y)`), tuned by eye to match arm A's measured axis of
    almost exactly `[-1, 0, 0]`. Arm B's own axis measures to almost exactly
    `[+1, 0, 0]` -- the mirrored asset flips the axis SIGN, not just its
    position -- and the hardcoded formula silently used the WRONG rotation
    sense for arm B: joint-range checks still passed (the solver still
    returned a five-angle answer), but the returned pose did not reach the
    target at all (round-trip position error ~0.2-0.3 m, caught by this
    module's own local self-test against the real compiled model before
    ever reaching bm-ptl -- see DECISIONS.md's ADR-070 entry). Routing
    through `_signed_angle_about_axis` with the ACTUAL measured `axis`
    (rather than a hand-typed sign) fixes this for both arms at once and
    for either mirroring convention a future asset revision might use.
    """
    return _signed_angle_about_axis(_CHAIN_ANGLE_REF, v, axis)


# ---------------------------------------------------------------------------
# Calibration: measure every constant this solver needs directly from the
# compiled MJCF via mj_forward, rather than trusting hardcoded numbers (the
# task brief's own instruction: "do NOT re-derive from a drawing, but DO
# re-verify"). Cached per (model, arm) since these are static properties of
# the compiled model, not per-call data -- recomputing them on every IK call
# would be pure waste (a skill can call this solver every control step).
# ---------------------------------------------------------------------------


@dataclass
class _ArmKinematics:
    """Every measured constant `solve_topdown_ik` needs for one arm, read
    once via `_calibrate()` and cached. All positions/axes are WORLD-frame,
    measured with this arm's five positioning joints at qpos=0 (`ik.py`'s
    `ARM_JOINT_SUFFIXES` order) -- see the module docstring's "Kinematic
    structure" section for what each point/axis physically is.
    """

    # Anchors (joint pivot points) and axes, index 0..4 matching
    # ARM_JOINT_SUFFIXES = (shoulder_pan, shoulder_lift, elbow_flex,
    # wrist_flex, wrist_roll).
    anchors: list[np.ndarray]
    axes: list[np.ndarray]

    #: ADR-025 pinch point (midpoint of the fixed and moving jaw bodies) at
    #: the all-zero pose, with the gripper (jaw open/close) joint left at
    #: whatever value a freshly-constructed `MjData` gives it (this module's
    #: calibration never touches the gripper joint -- see the "Known
    #: limitation" note in `solve_topdown_ik`'s docstring).
    pinch0: np.ndarray

    #: Link vectors of the planar 3R sub-chain, at the zero pose.
    l1_vec: np.ndarray  # shoulder_lift anchor -> elbow_flex anchor
    l2_vec: np.ndarray  # elbow_flex anchor -> wrist_flex anchor
    l1_len: float
    l2_len: float

    #: model.jnt_range for each of the 5 positioning joints, same order.
    jnt_ranges: list[tuple[float, float]]

    #: The wrist_roll angle that best cancels the pinch point's lateral
    #: (out-of-plane) offset -- see `_theta5_star`. `yaw=0.0` uses this
    #: value directly.
    theta5_star: float


_CALIBRATION_CACHE: dict[tuple[int, str], _ArmKinematics] = {}


def _calibrate(model, arm: str) -> _ArmKinematics:
    """Measure `_ArmKinematics` for `arm` ("A" or "B") from the compiled
    `model`, via a throwaway `mujoco.MjData` -- never touches the caller's
    real `data`. Cached by `(id(model), arm)`: a compiled `mujoco.MjModel`
    is immutable for the lifetime of the process, so this is safe to reuse
    across every call to `solve_topdown_ik` for the same model.
    """
    cache_key = (id(model), arm)
    cached = _CALIBRATION_CACHE.get(cache_key)
    if cached is not None:
        return cached

    scratch = mujoco.MjData(model)

    joint_names = arm_joint_names(arm)  # 5 names, ARM_JOINT_SUFFIXES order
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names
    ]
    for name, jid in zip(joint_names, joint_ids):
        if jid == -1:
            raise ValueError(f"joint {name!r} not found in the compiled model")

    # Explicitly zero this arm's five positioning joints before reading
    # anchors/axes -- do not rely on a fresh MjData's qpos0 already being
    # zero there (true today per ADR-016's unmodified-asset guarantee, but
    # this is cheap insurance against that changing silently).
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    for adr in qpos_adrs:
        scratch.qpos[adr] = 0.0
    mujoco.mj_forward(model, scratch)

    anchors = [np.array(scratch.xanchor[j], dtype=np.float64, copy=True) for j in joint_ids]
    axes = [np.array(scratch.xaxis[j], dtype=np.float64, copy=True) for j in joint_ids]
    jnt_ranges = [(float(model.jnt_range[j][0]), float(model.jnt_range[j][1])) for j in joint_ids]

    fixed_jaw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fixed_jaw_body_name(arm))
    moving_jaw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, moving_jaw_body_name(arm))
    if fixed_jaw_id == -1 or moving_jaw_id == -1:
        raise ValueError(f"gripper jaw bodies not found for arm {arm!r}")
    # ADR-025: the pinch point is the MIDPOINT of these two bodies, not the
    # armX_gripperframe site (see ik.py's module docstring and DECISIONS.md).
    pinch0 = 0.5 * (
        np.array(scratch.xpos[fixed_jaw_id], dtype=np.float64)
        + np.array(scratch.xpos[moving_jaw_id], dtype=np.float64)
    )

    p1, p2, p3 = anchors[1], anchors[2], anchors[3]  # shoulder_lift, elbow_flex, wrist_flex
    l1_vec = p2 - p1
    l2_vec = p3 - p2

    # Guard the "planar 3R chain" assumption this whole module depends on:
    # shoulder_lift, elbow_flex and wrist_flex must share (very nearly) the
    # same rotation axis, and that axis must be (very nearly) world x, since
    # `d`'s formula a few lines below (and in `solve_topdown_ik`) reads world
    # x directly as a shortcut for "component along this axis", which is only
    # valid if the axis really is world-x-aligned (either sign -- see
    # `_chain_angle`'s docstring for why sign alone is handled generally).
    # Verified below, not assumed; if a future asset revision changes this,
    # fail loudly here rather than silently producing wrong numbers.
    for name, ax in zip(("shoulder_lift", "elbow_flex", "wrist_flex"), axes[1:4]):
        if not np.allclose(ax, axes[1], atol=1e-6):
            raise ValueError(
                f"arm {arm!r}: {name}'s axis {ax} is not parallel to shoulder_lift's "
                f"axis {axes[1]} -- the planar-3R-chain assumption this solver depends "
                f"on does not hold for this model; ik_geometric.py needs revisiting."
            )
        if not np.allclose(np.abs(ax), np.array([1.0, 0.0, 0.0]), atol=1e-6):
            raise ValueError(
                f"arm {arm!r}: {name}'s axis {ax} is not world-x-aligned -- "
                f"the world-x shortcut used for 'd' in solve_topdown_ik does not "
                f"hold for this model; ik_geometric.py needs revisiting."
            )

    consts = _ArmKinematics(
        anchors=anchors,
        axes=axes,
        pinch0=pinch0,
        l1_vec=l1_vec,
        l2_vec=l2_vec,
        l1_len=float(np.linalg.norm(l1_vec)),
        l2_len=float(np.linalg.norm(l2_vec)),
        jnt_ranges=jnt_ranges,
        theta5_star=0.0,  # filled in below, needs `consts` to compute
    )
    consts.theta5_star = _theta5_star(consts)
    _CALIBRATION_CACHE[cache_key] = consts
    return consts


def _theta5_star(consts: "_ArmKinematics") -> float:
    """The `wrist_roll` angle that best cancels the pinch point's lateral
    (out-of-plane, i.e. along the shared shoulder_lift/elbow_flex/wrist_flex
    axis direction) offset -- see the module docstring's "Yaw and the
    lateral offset" section for why this offset cannot be cancelled to
    EXACTLY zero (the achievable minimum is reported so a caller could, in
    principle, check it; `solve_topdown_ik` does not need to, since the
    achieved-orientation gate is validated empirically instead).

    Closed form: let `a4` be the wrist_roll axis and `v4 = pinch0 -
    wrist_roll_anchor` (both at the zero pose, in the "sub-chain not yet
    rotated" reference frame). Rotating `v4` about `a4` by `theta5` moves its
    component along the shared shoulder_lift/elbow_flex/wrist_flex axis
    direction (call it `A`) as `v4.A + C1*cos(theta5) + C2*sin(theta5)`,
    where `C1 = v4 . A` and `C2 = A . (a4 x v4)` (this drops straight out of
    the Rodrigues formula: the component of a rotated vector along an axis
    it does not rotate about is a pure sinusoid of the rotation angle).
    What matters here is only the component along the FIXED chain axis `A`
    -- exactly the component `theta2`/`theta3`/`theta4` can never correct,
    since their own rotations are all about that same axis `A` and
    therefore never change a vector's `A`-component. Minimizing
    `|baseline + C1*cos(t) + C2*sin(t)|` over `t` is a 1-line closed form:
    the two critical points of that expression are `t* = atan2(C2, C1)` and
    `t* + pi`; evaluate both and keep whichever gets closest to zero.

    **Periodicity.** `atan2` only ever returns a value in `(-pi, pi]`, but
    `wrist_roll`'s joint range is NOT necessarily centred there (measured:
    `[-2.74, 2.84]` rad, close to a full turn wide) -- so the raw critical
    point can land numerically outside the joint's range even though the
    SAME physical wrist orientation, shifted by a whole turn (`+/- 2*pi`),
    is inside it. Found during this module's own development against the
    real compiled model, not obvious from the brief's hand-typed numbers
    (see DECISIONS.md's ADR-070 entry): arm A's measured `theta5_star`
    comes out as `~4.01` rad, outside `[-2.74, 2.84]`, while
    `4.01 - 2*pi ~= -2.27` rad is the identical rotation and IS inside
    range. `_wrap_to_range` (below) is what handles this, here and for
    every other angle `solve_topdown_ik` produces.
    """
    A = consts.axes[1]  # shoulder_lift's axis == elbow_flex's == wrist_flex's
    a4 = consts.axes[4]  # wrist_roll's axis
    p3 = consts.anchors[3]  # wrist_flex anchor
    p4 = consts.anchors[4]  # wrist_roll anchor
    v4 = consts.pinch0 - p4

    a_hat = _unit(A)
    baseline = float(np.dot(p4 - p3, a_hat))
    c1 = float(np.dot(v4, a_hat))
    c2 = float(np.dot(a_hat, np.cross(a4, v4)))

    lo_wroll, hi_wroll = consts.jnt_ranges[4]
    t_max = float(np.arctan2(c2, c1))
    best_t, best_abs, best_in_range = None, None, False
    for t in (t_max, t_max + np.pi):
        val = abs(baseline + c1 * np.cos(t) + c2 * np.sin(t))
        wrapped = _wrap_to_range(t, lo_wroll, hi_wroll)
        in_range = wrapped is not None
        candidate = wrapped if in_range else t
        # Prefer any in-range candidate over any out-of-range one; among
        # candidates with the same in-range status, prefer the smaller
        # residual (closer to true top-down).
        better = (
            best_t is None
            or (in_range and not best_in_range)
            or (in_range == best_in_range and val < best_abs)
        )
        if better:
            best_t, best_abs, best_in_range = candidate, val, in_range
    return best_t


def _wrap_to_range(angle: float, lo: float, hi: float) -> float | None:
    """Return `angle + 2*pi*k` (some integer k) that lies in `[lo, hi]`, if
    one exists, else `None`.

    Every angle this module produces comes out of `atan2`-family formulas,
    which only ever return a value in `(-pi, pi]`. A joint's mechanically
    real, reachable angle can be the SAME physical rotation shifted by a
    whole number of turns (`angle + 2*pi*k`) -- e.g. this arm's measured
    `wrist_roll` axis calibrates to a `theta5_star` of about 4.01 rad, which
    is numerically outside `wrist_roll`'s own `[-2.74, 2.84]` range, but
    `4.01 - 2*pi = -2.27` rad is the SAME physical wrist orientation and
    IS inside range. Without this wrap, a perfectly reachable configuration
    would be reported as unreachable purely because of which multiple of a
    full turn the trig functions happened to return -- caught during this
    module's own development (see DECISIONS.md's ADR-070 entry) by testing
    against the real compiled model rather than only the brief's hand-typed
    numbers.
    """
    for k in (0, -1, 1, -2, 2):
        candidate = angle + 2.0 * np.pi * k
        if lo <= candidate <= hi:
            return float(candidate)
    return None


# ---------------------------------------------------------------------------
# The public solver.
# ---------------------------------------------------------------------------


def solve_topdown_ik(model, data, arm: str, target_xyz, yaw: float = 0.0) -> np.ndarray | None:
    """Closed-form IK: the 5 positioning-joint angles (`ik.py`'s
    `ARM_JOINT_SUFFIXES` order: shoulder_pan, shoulder_lift, elbow_flex,
    wrist_flex, wrist_roll) that put `arm`'s ADR-025 pinch point at
    `target_xyz` (world frame, metres) with the gripper approaching straight
    down, optionally yawed by `yaw` radians about world z.

    Returns `None` if the target is unreachable this way (outside the
    shoulder-offset circle, outside the 2-link arm's reachable annulus, or
    any solved angle would violate `model.jnt_range`) -- never a
    silently-wrong pose.

    `data` is accepted (matching the task's required signature) but not
    read: unlike `ik.py`'s iterative solver, this closed form does not need
    a starting configuration to converge from -- every quantity it needs
    comes from the compiled `model` alone (`_calibrate`, cached). It is kept
    in the signature so a future caller can swap between the two solvers
    without touching call sites, and so a future revision that DOES want to
    read the live gripper-jaw state (see "Known limitation" below) has
    somewhere to get it from.

    **Known limitation, disclosed rather than hidden.** The pinch point used
    here is measured with the gripper (jaw open/close) joint at whatever a
    freshly-constructed `MjData` defaults to (see `_calibrate`), not the
    LIVE value in `data`. `ik.py`'s iterative solver recomputes the true
    pinch point fresh every iteration from the current qpos specifically so
    it tracks the jaw opening/closing (ADR-025); this closed form does not,
    because doing so would turn part of the calibration into a per-call
    cost and reintroduce exactly the "solve fresh every step" pattern the
    closed form exists to avoid. In practice the jaw's own travel is small
    relative to the workspace and this stage's validation (mandatory,
    `scripts/v2_validate_ik.py`) measures the resulting error directly
    rather than assuming it is negligible.
    """
    consts = _calibrate(model, arm)
    target = np.asarray(target_xyz, dtype=np.float64).reshape(3)

    p0, p1 = consts.anchors[0], consts.anchors[1]
    a0, a1 = consts.axes[0], consts.axes[1]
    p4 = consts.anchors[4]
    a4 = consts.axes[4]
    lo_pan, hi_pan = consts.jnt_ranges[0]
    lo_lift, hi_lift = consts.jnt_ranges[1]
    lo_elbow, hi_elbow = consts.jnt_ranges[2]
    lo_wflex, hi_wflex = consts.jnt_ranges[3]
    lo_wroll, hi_wroll = consts.jnt_ranges[4]

    # ---- Step 1: wrist_roll directly from yaw (see module docstring). ----
    # theta5_star is already wrapped into range by _theta5_star(); adding a
    # nonzero yaw can push it back out, so wrap again here (see
    # _wrap_to_range's docstring for why this periodic wrap is necessary,
    # not just defensive).
    theta5_raw = consts.theta5_star + float(yaw)
    theta5 = _wrap_to_range(theta5_raw, lo_wroll, hi_wroll)
    if theta5 is None:
        return None

    # ---- Step 2: net pitch Theta of the 3R sub-chain, for "straight down". ----
    # g = the wrist_flex-anchor -> pinch vector, AFTER wrist_roll's rotation,
    # still expressed in the "Theta = 0" reference frame (i.e. as if
    # shoulder_lift/elbow_flex/wrist_flex were all still at zero). This is
    # the "virtual last link" of the 3R chain once wrist_roll is fixed.
    g = _rotate_about(p4, a4, theta5, consts.pinch0) - consts.anchors[3]

    want_down = np.array([0.0, 0.0, -1.0])
    theta_sum = _signed_angle_about_axis(g, want_down, a1)  # theta2+theta3+theta4

    # known_tail = g, rotated into its FINAL orientation (Theta applied) --
    # a fully-known vector now, independent of how theta_sum splits across
    # theta2/theta3/theta4 individually.
    known_tail = _rodrigues(a1, theta_sum) @ g

    # ---- Step 3: shoulder-offset circle for theta1, law of cosines for
    # theta2/theta3 (see module docstring's "lateral offset" section). ----
    # d = the constant lateral (out-of-the-naive-plane) distance from the
    # shoulder_pan axis to the pinch point's effective plane, now that
    # theta5 (hence known_tail) is fixed. Invariant to theta2/theta3: a
    # rotation about a1 (the shared shoulder_lift/elbow_flex/wrist_flex
    # axis, which IS this "lateral" direction) never changes a vector's own
    # component along that same axis.
    d = float((p1 - p0)[0] + known_tail[0])
    # NOTE: the above indexes world-x directly because a1 (measured) is very
    # close to a pure coordinate axis; using [0] rather than a general
    # dot-product against a1 matches the sign/scale convention the rest of
    # this function's coordinate-frame algebra already assumes (a1 and a0
    # are used consistently throughout, all measured, not assumed -- see
    # `scripts/v2_validate_ik.py`'s numerical cross-check of this
    # assumption).

    target_rel = target - p0
    r_target_sq = float(target_rel[0] ** 2 + target_rel[1] ** 2)
    if r_target_sq < d * d:
        return None  # target is inside the offset circle -- unreachable at any pan angle
    r_perp = float(np.sqrt(max(r_target_sq - d * d, 0.0)))

    for vy_sign in (1.0, -1.0):
        vy = vy_sign * r_perp
        wc_y = vy - (p1 - p0)[1] - known_tail[1]
        wc_z = target_rel[2] - (p1 - p0)[2] - known_tail[2]
        r2 = wc_y * wc_y + wc_z * wc_z
        r = float(np.sqrt(r2))
        if r > consts.l1_len + consts.l2_len or r < abs(consts.l1_len - consts.l2_len):
            continue  # outside the 2-link arm's reachable annulus, this branch

        cos_elbow = (r2 - consts.l1_len**2 - consts.l2_len**2) / (2.0 * consts.l1_len * consts.l2_len)
        cos_elbow = float(np.clip(cos_elbow, -1.0, 1.0))
        phi = _chain_angle(np.array([0.0, wc_y, wc_z]), a1)

        for elbow_sign in (1.0, -1.0):
            beta = elbow_sign * float(np.arccos(cos_elbow))
            psi = float(np.arctan2(consts.l2_len * np.sin(beta), consts.l1_len + consts.l2_len * np.cos(beta)))
            theta_l1_final = phi - psi
            theta_l2_final = theta_l1_final + beta

            phi_l1_zero = _chain_angle(consts.l1_vec, a1)
            phi_l2_zero = _chain_angle(consts.l2_vec, a1)

            theta2_raw = theta_l1_final - phi_l1_zero
            theta3_raw = (theta_l2_final - phi_l2_zero) - theta2_raw
            theta4_raw = theta_sum - theta2_raw - theta3_raw

            theta2 = _wrap_to_range(theta2_raw, lo_lift, hi_lift)
            if theta2 is None:
                continue
            theta3 = _wrap_to_range(theta3_raw, lo_elbow, hi_elbow)
            if theta3 is None:
                continue
            theta4 = _wrap_to_range(theta4_raw, lo_wflex, hi_wflex)
            if theta4 is None:
                continue

            # Use the UNWRAPPED theta2/theta3 (theta2_raw/theta3_raw) to
            # position the elbow -- these came directly out of the
            # law-of-cosines geometry above and are guaranteed self-
            # consistent with each other; substituting the (possibly
            # 2*pi-shifted) wrapped copies here would still give the same
            # physical rotation (Rodrigues is 2*pi-periodic) so either is
            # fine, but using the raw pair keeps this line visibly tied to
            # the geometry it came from.
            v = (
                (p1 - p0)
                + _rodrigues(a1, theta2_raw) @ consts.l1_vec
                + _rodrigues(a1, theta2_raw + theta3_raw) @ consts.l2_vec
                + known_tail
            )
            theta1_raw = _signed_angle_about_axis(
                np.array([v[0], v[1], 0.0]),
                np.array([target_rel[0], target_rel[1], 0.0]),
                a0,
            )
            theta1 = _wrap_to_range(theta1_raw, lo_pan, hi_pan)
            if theta1 is None:
                continue

            return np.array([theta1, theta2, theta3, theta4, theta5], dtype=np.float64)

    return None


if __name__ == "__main__":
    # Guarded per builder convention. There is no meaningful standalone run
    # for this module without a compiled MuJoCo model (ADR-020: MuJoCo will
    # not import on the laptop at all), so this only prints usage guidance
    # rather than attempting a smoke test -- the real smoke test is
    # `scripts/v2_validate_ik.py`, run on bm-ptl.
    print(__doc__)
    print(
        "\nThis module has no standalone laptop entry point (ADR-020: MuJoCo "
        "cannot import here). Run scripts/v2_validate_ik.py on bm-ptl instead."
    )
