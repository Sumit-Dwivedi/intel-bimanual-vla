"""WeldGrasp: equality-constraint-based grasping abstraction (M06 Phase 1, ADR-029).

**Why this module exists at all.** `docs/hardware/grasp-envelope.md` measured,
functionally (not geometrically), that this gripper's jaw meshes collapse to
permanently-overlapping convex hulls (MuJoCo issue #239): zero of 30 caliper
thicknesses from 2-60 mm achieved sustained two-jaw contact. ADR-028's
finger-pad primitives fixed the geometry (verified: pad separation sweeps
6-132 mm centre-to-centre, monotonic with joint angle) but the arm's reach
envelope plus the ~8 cm pinch-point kinematic offset (`ik.py` ADR-025) put
every graspable target at the edge of what a 5-DoF position-only IK solve can
actually reach (`docs/hardware/m06-grip-diagnostic.md`,
`m06-grip-diagnostic-after-fix.md`). Contact-based grasping is therefore not
a code bug to keep chasing -- it is a structural limit of this simulated
gripper's geometry and this arm's kinematics (ADR-029's Context).

**What this module does instead.** It welds the target prop's body rigidly to
the gripper's fixed-jaw body via a MuJoCo `weld` equality constraint, toggled
on/off at runtime. This is standard practice, not a hidden shortcut: MoveIt's
"attached objects", PyBullet's fixed constraints, and academic sim-to-real
manipulation work all abstract the grasp-contact subsystem the same way.
**Grasping is therefore ABSTRACTED, not physically simulated** -- this must be
stated plainly in the README and demo video (ADR-015 rule set), and this
module's own success criterion (`attempt_grasp` refusing when either gate
fails) is what keeps the abstraction from degrading into "always welds",
which would not be a grasp mechanism at all.

**Phase 1 scope (this module only).** This file implements the mechanism and
nothing else. It is deliberately NOT wired into `pick`/`place`/`handoff` or
`executor.py` -- that wiring is Phase 2, contingent on `scripts
/probe_weld_grasp.py` verifying this mechanism end to end. Nothing in
`skills_scripted.py`, `executor.py` or `ik.py` is touched or imported for
side effects by this module.

**Which body is the attach frame, and why (the naming trap, again).** Per
`ik.py`'s own module docstring and `GLOSSARY.md`'s "gripper (naming caveat)"
entries: `<body name="armX_gripper">` is the FIXED half of the gripper
mechanism, driven by the `armX_wrist_roll` joint -- NOT the jaw that opens
and closes. The joint/actuator named `armX_gripper` drives a *different*
body, `armX_moving_jaw_so101_v1`, which is a child of `armX_gripper`. This
module welds objects to `armX_gripper` (the body) precisely because it is the
one half of the mechanism that does not itself move relative to the wrist --
the natural, stable attach frame -- while still reading the `armX_gripper`
JOINT's qpos to decide whether the jaws are closed enough to grasp (see
`attempt_grasp` below). Getting this pair backwards (welding to the moving
jaw, or gating on the wrong body's position) is exactly the kind of mistake
this docstring exists to prevent a future reader from repeating.

**The eq_data teleport gotcha, and how this module avoids it.** A MuJoCo weld
constraint's target relative pose lives in `model.eq_data`, not wherever the
two bodies happen to be when `eq_active` flips to 1. Toggling `eq_active`
without first writing the CURRENT relative transform into `eq_data` snaps the
object to whatever pose (often the XML-declared all-zero default) `eq_data`
already holds, the instant the constraint is next evaluated -- a dramatic,
easy-to-miss bug. `attempt_grasp` therefore always (1) reads the gripper and
object's CURRENT `xpos`/`xquat`, (2) computes the object's pose relative to
the gripper body, (3) writes that into the pre-declared constraint's
`eq_data`, (4) THEN sets `eq_active`, then (5) calls `mj_forward` and logs the
object's position immediately before/after so a caller (and
`probe_weld_grasp.py`) can verify no teleport occurred.

**`eq_data` layout, confirmed empirically for the installed mujoco==3.2.7,
not assumed from documentation.** `mjNEQDATA == 11`
(`mujoco/include/mujoco/mjmodel.h`): `eq_data[0:3]` = anchor, `eq_data[3:7]`
= relpose position, `eq_data[7:11]`... wait, precisely: `eq_data[0:3]` =
anchor (3), `eq_data[3:10]` = relpose (7 = 3 position + 4 quaternion),
`eq_data[10]` = torquescale (1). The exact semantics of `anchor` and
`relpose` are NOT documented in the shipped headers, so this module's
constants were derived empirically (5 randomized-pose trials, weld body1 =
object / body2 = reference body, gravity enabled, 3000-step rollout,
`scripts/gen_dual_scene.py`-independent scratch models -- not committed,
reasoning recorded here) and are, for `body1="object"`, `body2="gripper"`:

    anchor       = R(gripper_quat)^T @ (object_pos - gripper_pos)   # object's
                   position expressed in the gripper body's own local frame
    relpose_pos  = (0, 0, 0)                                        # unused
                   for a body1/body2 pair with no additional lever arm
    relpose_quat = conj(object_quat) * gripper_quat                 # the
                   GRIPPER's orientation expressed in the OBJECT's frame --
                   note the reversed order relative to "object relative to
                   gripper"; this reversal was the one sign convention this
                   empirical check caught (the position/anchor term is
                   symmetric either way, but the naive "object relative to
                   gripper" quaternion order silently rotated the held object
                   away from its grasped orientation over the following
                   steps while leaving position exactly correct -- a bug that
                   passed a position-only teleport check and failed only on
                   sustained orientation drift, which is why this module's
                   own docstring records the sign explicitly).
    torquescale  = 1.0

Across 5 randomized (position, orientation) trials under gravity, this
formula held both position (drift 2.7e-5 m -- solver settling noise, not
error) and orientation (0.0 quaternion delta) exactly, for 3000 steps.
`mujoco`'s own `mju_rotVecQuat`/`mju_negQuat`/`mju_mulQuat` are used below
rather than hand-rolled quaternion math, so this module's convention tracks
MuJoCo's internal one rather than a reimplementation of it.

**Where the weld constraints themselves live.** ADR-029 chose option (a):
all 10 `(armX_gripper, prop)` weld equality constraints (5 props x 2 arms)
are pre-declared in the generated scene XML with `active="false"`, by
`scripts/gen_dual_scene.py`'s HAND-AUTHORED region (never `scenes/so101/`,
per ADR-016) -- see that script's `build_weld_constraints()`. This module
only ever toggles `data.eq_active` and rewrites `model.eq_data` for
constraints that already exist; it never creates an equality constraint at
runtime (option (b), the documented fallback, was not needed -- pre-declaring
compiled without incident).
"""

from __future__ import annotations

import logging

import mujoco
import numpy as np

from bimanual.control import ik

logger = logging.getLogger(__name__)

#: Arms this mechanism tracks. Matches `ik.py`'s "A"/"B" convention throughout
#: the control stack.
ARMS = ("A", "B")

#: Prop body names declared by `scripts/gen_dual_scene.py`'s hand-authored
#: prop section (`<body name="plate">`, `"mug"`, `"fork"`, `"spoon"`,
#: `"water_bottle"`) -- see `docs/hardware/grasp-envelope.md`(b) for the
#: measured geometry of each. Kept here (not imported from the generator,
#: which is a code-gen utility script, not a runtime module) so this file has
#: no dependency on `scripts/`.
GRASPABLE_OBJECTS = ("plate", "mug", "fork", "spoon", "water_bottle")

#: A weld's position drift immediately after activation, in metres, above
#: which `attempt_grasp` logs a loud warning that the eq_data convention may
#: be wrong (see this module's docstring for the "teleport gotcha"). This is
#: a diagnostic tripwire, not a hard failure -- the grasp still attaches --
#: because a caller (or `probe_weld_grasp.py`) explicitly checking the
#: before/after log is the real verification; this just makes a regression
#: loud in ordinary use too.
TELEPORT_WARNING_THRESHOLD_M = 0.002


def weld_constraint_name(arm: str, object_name: str) -> str:
    """Name of the pre-declared weld equality constraint for (arm, object).

    Must match `scripts/gen_dual_scene.py`'s `build_weld_constraints()`
    exactly -- both sides of this contract are string-formatted from the
    same `(arm, object_name)` pair, but there is no shared import between a
    runtime module and a code-gen script (see this module's docstring), so a
    drift here would fail silently as "constraint not found" rather than a
    Python `ImportError`. `WeldGrasp.__init__` asserts all 10 names resolve
    against the compiled model specifically to catch that class of drift
    early and loudly.
    """
    return f"weld_arm{arm}_{object_name}"


def _quat_conj(dst: np.ndarray, quat: np.ndarray) -> None:
    mujoco.mju_negQuat(dst, quat)


class WeldGrasp:
    """Toggle weld equality constraints to attach/detach props from a gripper.

    Tracks at most one held object per arm (`{'A': None, 'B': None}` at
    construction). This class never creates a `mujoco` equality constraint at
    runtime -- all 10 `(armX_gripper, prop)` welds are pre-declared, inactive,
    in the compiled model (ADR-029); this class only reads/writes
    `model.eq_data` and `data.eq_active` for constraints that already exist.
    """

    def __init__(self, env) -> None:
        """Bind to one `TableSettingEnv` and resolve every weld constraint's id.

        Args:
            env: A `TableSettingEnv` (or any object exposing `.model` and
                `.data` mujoco handles) whose compiled scene declares the 10
                `weld_arm{A,B}_{prop}` equality constraints this class
                expects (`scripts/gen_dual_scene.py`'s
                `build_weld_constraints()`). Stored by reference, not
                copied -- `attempt_grasp`/`release` read and write the
                SAME `model`/`data` the caller's `env.step()` advances, so a
                weld toggled here is visible to the very next physics step.
        """
        self.env = env
        self.model = env.model
        self.data = env.data

        # Per-arm held-object tracker, exactly as specified: None means the
        # arm's jaw is not currently welded to anything.
        self.active_welds: dict[str, str | None] = {"A": None, "B": None}

        # Resolve every (arm, object) -> equality-constraint-id up front,
        # rather than doing a name lookup on every attempt_grasp() call, and
        # fail loudly here (constructor time) rather than inside a skill's
        # closed loop if the generated scene ever falls out of sync with this
        # module's naming convention (see `weld_constraint_name`'s
        # docstring).
        self._eq_ids: dict[tuple[str, str], int] = {}
        missing: list[str] = []
        for arm in ARMS:
            for object_name in GRASPABLE_OBJECTS:
                name = weld_constraint_name(arm, object_name)
                eq_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, name)
                if eq_id == -1:
                    missing.append(name)
                else:
                    self._eq_ids[(arm, object_name)] = eq_id
        if missing:
            raise ValueError(
                f"WeldGrasp expected {len(ARMS) * len(GRASPABLE_OBJECTS)} pre-declared "
                f"weld equality constraints (scripts/gen_dual_scene.py's "
                f"build_weld_constraints(), ADR-029); {len(missing)} not found in the "
                f"compiled model: {missing}. Regenerate "
                f"src/bimanual/sim/assets/so101_dual_table.xml with "
                f"`python scripts/gen_dual_scene.py` and recheck the naming convention "
                f"in `weld_constraint_name`."
            )

        # Also resolve each arm's gripper JOINT id (for the closure check) and
        # fixed-jaw BODY id (the attach frame) up front, for the same
        # fail-loud-at-construction reason. Deliberately reuses `ik.py`'s own
        # naming helpers rather than re-deriving the name strings here --
        # see this module's docstring on the gripper naming trap.
        self._gripper_joint_qpos_adr: dict[str, int] = {}
        self._gripper_body_id: dict[str, int] = {}
        # Bug 2 fix (this task): the moving-jaw BODY id is resolved here too,
        # purely so Gate 2 (proximity) can compute the PINCH POINT -- the
        # midpoint of this body and the fixed-jaw body -- exactly the same
        # quantity `ik.solve_position_ik` targets (ADR-025, `ik.py`'s
        # `_pinch_point()`). This does NOT change the weld attach frame: the
        # weld still attaches to `self._gripper_body_id[arm]` (the fixed jaw),
        # unchanged from ADR-029. Only the distance measurement below moves.
        self._moving_jaw_body_id: dict[str, int] = {}
        for arm in ARMS:
            joint_name = ik.gripper_joint_name(arm)
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id == -1:
                raise ValueError(f"gripper joint {joint_name!r} not found in the compiled model")
            self._gripper_joint_qpos_adr[arm] = int(self.model.jnt_qposadr[joint_id])

            body_name = ik.fixed_jaw_body_name(arm)
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id == -1:
                raise ValueError(f"gripper body {body_name!r} not found in the compiled model")
            self._gripper_body_id[arm] = body_id

            moving_jaw_name = ik.moving_jaw_body_name(arm)
            moving_jaw_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, moving_jaw_name)
            if moving_jaw_id == -1:
                raise ValueError(f"moving jaw body {moving_jaw_name!r} not found in the compiled model")
            self._moving_jaw_body_id[arm] = moving_jaw_id

        self._object_body_id: dict[str, int] = {}
        for object_name in GRASPABLE_OBJECTS:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, object_name)
            if body_id == -1:
                raise ValueError(f"prop body {object_name!r} not found in the compiled model")
            self._object_body_id[object_name] = body_id

    # ------------------------------------------------------------------
    # Public contract
    # ------------------------------------------------------------------

    def attempt_grasp(
        self,
        arm: str,
        object_name: str,
        distance_threshold_m: float = 0.05,
        closure_threshold: float = 0.3,
    ) -> bool:
        """Attach `object_name` to `arm`'s gripper iff BOTH gates pass.

        Gate 1 -- closure: the `armX_gripper` JOINT's qpos (the jaw hinge,
        range roughly [-0.1745 (closed), 1.7453 (open)] per
        `scenes/so101/so101_new_calib.xml`) must be BELOW `closure_threshold`
        -- i.e. the jaw must have closed materially past its own midpoint
        (+0.785 rad), not merely be resting somewhere. "Open" is the HIGH end
        of this range (`scripts/gen_dual_scene.py`'s `gripper_open_value =
        gripper_hi`), so "qpos below threshold" correctly reads as "jaws
        closing", matching the task's own convention.

        Gate 2 -- proximity: the Euclidean distance from the PINCH POINT to
        `object_name`'s body world position must be below
        `distance_threshold_m`. Distance is measured from the pinch point --
        the midpoint of the fixed and moving jaw bodies, which is what
        `ik.solve_position_ik` targets per ADR-025 -- not from the gripper
        body. The weld still attaches to the gripper body; the pinch point is
        where the arm is actually positioned, so it is the correct quantity to
        gate on.

        (Bug history, this task: an earlier version of this gate measured
        from the `armX_gripper` BODY's world position alone -- the fixed jaw,
        not the pinch point -- which diverges from the pinch point by a
        growing margin as the jaw closes, because `ik.py`'s redundant 5-DOF
        solve keeps the pinch point pinned at its target by rotating the
        wrist, carrying the fixed-jaw body away from the object in the
        process. That divergence (measured: 0.069 m at GRIP start, climbing
        to 0.0795 m fully closed, against this gate's own 0.05 m default
        threshold) meant the gate could never open. Fixed here by measuring
        the same pinch point IK controls, mirroring `ik.py`'s own
        `_pinch_point()` computation rather than a fixed local-axis offset,
        which would not track jaw closure the way the true midpoint does.)

        Refusing when either gate fails (rather than always welding) is the
        entire point of this mechanism per ADR-029's Consequences: a
        mechanism that always welds is glue, not a grasp abstraction.

        Returns:
            True and activates the weld iff both gates pass. False,
            logging the specific reason, otherwise -- including when `arm`
            already holds something (release it first) or `object_name` is
            not one of `GRASPABLE_OBJECTS`.
        """
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")

        key = (arm, object_name)
        if key not in self._eq_ids:
            logger.info(
                "attempt_grasp refused: arm=%s object=%r is not one of the pre-declared "
                "graspable objects %s",
                arm, object_name, GRASPABLE_OBJECTS,
            )
            return False

        if self.active_welds[arm] is not None:
            logger.info(
                "attempt_grasp refused: arm=%s already holds %r -- call release(%r) first",
                arm, self.active_welds[arm], arm,
            )
            return False

        # ---- Gate 1: closure -------------------------------------------
        qpos_adr = self._gripper_joint_qpos_adr[arm]
        gripper_joint_qpos = float(self.data.qpos[qpos_adr])
        if gripper_joint_qpos >= closure_threshold:
            logger.info(
                "attempt_grasp refused: arm=%s object=%r gripper not closed enough "
                "(joint qpos=%.4f rad >= closure_threshold=%.4f rad; jaws must be "
                "closing, i.e. qpos below threshold)",
                arm, object_name, gripper_joint_qpos, closure_threshold,
            )
            return False

        # ---- Gate 2: proximity -------------------------------------------
        # Measured from the PINCH POINT (the midpoint of the fixed and moving
        # jaw bodies), not the gripper body alone -- see this method's
        # docstring's "Bug history" note. This mirrors `ik.py`'s own
        # `_pinch_point()` computation (`solve_position_ik`'s IK target)
        # exactly: `0.5 * (xpos[fixed_jaw] + xpos[moving_jaw])`. Duplicated
        # here (rather than imported) because `ik.py`'s `_pinch_point` is a
        # local closure inside `solve_position_ik`, not a module-level
        # function -- this comment is the pointer back to that source of
        # truth (ik.py's `solve_position_ik`, `_pinch_point`) so the two
        # never silently drift apart.
        gripper_body_id = self._gripper_body_id[arm]
        moving_jaw_body_id = self._moving_jaw_body_id[arm]
        object_body_id = self._object_body_id[object_name]
        gripper_pos = self.data.xpos[gripper_body_id].copy()
        moving_jaw_pos = self.data.xpos[moving_jaw_body_id].copy()
        pinch_pos = 0.5 * (gripper_pos + moving_jaw_pos)
        object_pos = self.data.xpos[object_body_id].copy()
        distance_m = float(np.linalg.norm(object_pos - pinch_pos))
        if distance_m >= distance_threshold_m:
            logger.info(
                "attempt_grasp refused: arm=%s object=%r too far (pinch-point "
                "distance=%.4f m >= distance_threshold_m=%.4f m)",
                arm, object_name, distance_m, distance_threshold_m,
            )
            return False

        # ---- Both gates passed: attach -----------------------------------
        # THE ORDER BELOW MATTERS (this module's docstring's "teleport
        # gotcha"): compute and write the CURRENT relative pose into
        # eq_data BEFORE setting eq_active, then mj_forward, then verify.
        gripper_quat = self.data.xquat[gripper_body_id].copy()
        object_quat = self.data.xquat[object_body_id].copy()

        neg_gripper_quat = np.zeros(4)
        _quat_conj(neg_gripper_quat, gripper_quat)
        anchor = np.zeros(3)
        # object's position, expressed in the gripper body's own local frame
        # (rotate the world-frame offset by the gripper's INVERSE
        # orientation) -- see module docstring's empirically-derived formula.
        mujoco.mju_rotVecQuat(anchor, object_pos - gripper_pos, neg_gripper_quat)

        neg_object_quat = np.zeros(4)
        _quat_conj(neg_object_quat, object_quat)
        relpose_quat = np.zeros(4)
        # the GRIPPER's orientation expressed in the OBJECT's frame (reversed
        # order relative to the naive "object relative to gripper" -- see
        # module docstring for how this sign was caught).
        mujoco.mju_mulQuat(relpose_quat, neg_object_quat, gripper_quat)

        eq_id = self._eq_ids[key]
        pos_before = object_pos.copy()

        self.model.eq_data[eq_id, 0:3] = anchor
        self.model.eq_data[eq_id, 3:6] = 0.0
        self.model.eq_data[eq_id, 6:10] = relpose_quat
        self.model.eq_data[eq_id, 10] = 1.0
        self.data.eq_active[eq_id] = 1
        mujoco.mj_forward(self.model, self.data)

        pos_after = self.data.xpos[object_body_id].copy()
        teleport_m = float(np.linalg.norm(pos_after - pos_before))
        logger.info(
            "attempt_grasp attached: arm=%s object=%r distance=%.4f m closure_qpos=%.4f rad "
            "pos_before=%s pos_after=%s teleport=%.6f m",
            arm, object_name, distance_m, gripper_joint_qpos, pos_before, pos_after, teleport_m,
        )
        if teleport_m > TELEPORT_WARNING_THRESHOLD_M:
            logger.warning(
                "attempt_grasp: teleport of %.6f m on attach exceeds the %.3f m tripwire -- "
                "the eq_data relative-pose convention may be wrong (see this module's "
                "docstring's 'teleport gotcha' section)",
                teleport_m, TELEPORT_WARNING_THRESHOLD_M,
            )

        self.active_welds[arm] = object_name
        return True

    def release(self, arm: str) -> bool:
        """Deactivate `arm`'s active weld, if any.

        Returns:
            True if a weld was active and is now deactivated. False if `arm`
            was not holding anything (no-op).
        """
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")

        held = self.active_welds[arm]
        if held is None:
            logger.info("release no-op: arm=%s was not holding anything", arm)
            return False

        eq_id = self._eq_ids[(arm, held)]
        self.data.eq_active[eq_id] = 0
        mujoco.mj_forward(self.model, self.data)
        logger.info("release: arm=%s released %r", arm, held)
        self.active_welds[arm] = None
        return True

    def is_holding(self, arm: str) -> str | None:
        """The object name `arm` currently holds via an active weld, or None."""
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
        return self.active_welds[arm]

    def reset(self) -> None:
        """Clear this instance's held-object bookkeeping after `env.reset()`.

        **The bug this method exists to fix (ADR-047).** `self.active_welds`
        is a plain Python `dict` on THIS instance, set by a successful
        `attempt_grasp` and cleared only by `release()` -- nothing else ever
        touches it. `env.reset()` (`TableSettingEnv.reset()`,
        `src/bimanual/sim/env.py`) calls `mujoco.mj_resetData` (and, if a
        "home" keyframe exists, `mj_resetDataKeyframe`), which DOES reset
        MuJoCo's own `data.eq_active` for every weld back to the compiled
        model's inactive default -- but `mj_resetData` only ever touches the
        `mujoco.MjData` buffer; it has no way to reach into a Python object
        it does not know exists. So after `env.reset()`, MuJoCo believes
        nothing is welded while `self.active_welds` still remembers whatever
        was held the moment before reset. A caller that reuses one
        `WeldGrasp` across repeated trials against the SAME `env` --
        `ScriptedSkillExecutor._ensure_weld` does exactly this by design,
        ADR-030 -- and calls `env.reset()` directly between trials (instead
        of this method) hits `attempt_grasp`'s "already holds" refusal on
        the very next grasp for that (arm, object) pair, which surfaces to a
        caller as `weld_attach_failed_after_N_frames`: indistinguishable
        from a genuine reachability/grasp failure, silently corrupting any
        multi-trial evaluation (`docs/hardware/m10-pre-m07-audit.md`'s
        zero-th finding; this is that finding's fix).

        **What this clears, and what it deliberately does not touch.**
        Resets `self.active_welds` to `{'A': None, 'B': None}` (this
        instance's own bookkeeping) and, for every pre-declared weld this
        instance resolved at construction (`self._eq_ids.values()`), sets
        `self.data.eq_active[eq_id] = 0` -- belt-and-suspenders with
        `env.reset()`'s own `mj_resetData`, which should already have zeroed
        every one of these (this method does not assume that and clears
        them itself regardless, in case a future scene ever carries
        `eq_active` state in its "home" keyframe). Per this module's own
        docstring, this class "only ever toggles `data.eq_active` and
        rewrites `model.eq_data`" -- this method never writes
        `model.eq_active0` (the model's COMPILED initial value, which would
        persist across resets, a new and unwanted side effect) or
        `model.eq_data` (no relative pose needs restating; the constraint is
        simply inactive).

        **Call this immediately after every `env.reset()` call against the
        SAME `env` this `WeldGrasp` was constructed against** (this
        instance's `self.env`/`self.model`/`self.data` are bound once, at
        construction, to one compiled model -- see `__init__`'s docstring --
        and are never rebound by this method). `ScriptedSkillExecutor.reset`
        (`executor.py`, ADR-047) is the intended call site for ordinary
        skill-execution code; this method is also safe to call directly
        (e.g. from a probe script) when no executor is involved. A no-op,
        safely, if nothing was held and no weld was active.
        """
        self.active_welds = {"A": None, "B": None}
        for eq_id in self._eq_ids.values():
            self.data.eq_active[eq_id] = 0
        mujoco.mj_forward(self.model, self.data)
        logger.info("WeldGrasp.reset: cleared active_welds and all eq_active constraints")
