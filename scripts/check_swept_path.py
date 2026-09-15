"""Redesign Stage 4, Step 1: the swept-path collision gate (ADR-061).

**Why this gate is needed at all.** Stage 2's `verify_stage2_gate.py` checked
that every waypoint TARGET (the still endpoint of a hop -- a hover point, a
grasp point, the handoff point) is IK-reachable and collision-free once the
arm is sitting there. It never checked the MOTION between two consecutive
waypoints. `ik.solve_position_ik` has no obstacle term at all (ADR-024) and
`_drive_to_target`'s incremental re-solve (`skills_scripted.py`) walks the
arm toward a target from wherever it currently is, one small IK step at a
time -- so two endpoints can both be individually reachable and collision-free
while the straight-through-joint-space path between them clips a prop, the
table, or the other arm. Stage 3 (ADR-060) found exactly this: three skills
newly regressed on identical endpoints, traced to a "waypoint-1 approach
collision" that no existing gate checked for. This module is the missing
check.

**Interpolation is in JOINT SPACE, deliberately, not Cartesian.** The real
controller (`_drive_to_target`) re-solves IK every physics step from the
arm's CURRENT joint configuration toward the fixed target -- it never
re-plans a Cartesian line and re-solves per waypoint. A joint-space
interpolation between the start configuration and the (single) IK solution
at the target is the closer proxy to what the controller actually executes:
a monotonic walk from one joint configuration to another. A Cartesian
interpolation with a fresh IK re-solve at every interpolated point can jump
to a DIFFERENT joint-space branch at each step (the 5-DoF solve is
redundant against a 3-DoF position target -- ADR-024) and would silently
approve or reject a path the real arm never takes. This was an explicit
task requirement, not a style choice.

**RAW contact enumeration only -- the `table_top` mesh penetration filter
used elsewhere in this repo is NEVER applied here.** ADR-058 found that
filter produces convex-hull false positives on perfectly good, converged
poses (-0.39 m on `pick(A, fork)`'s own working grasp target; ADR-035 has an
independent -0.222 m instance). This module reads `data.contact[i].dist`
directly, exactly as `search_home_keyframe.py::evaluate_candidate` and
`verify_stage2_gate.py` already do -- no other filter is trusted repo-wide,
and none is introduced here.

**Finger-pad-vs-target exemption, only near the end of a sweep.** A GRIP or
a DESCEND-onto-a-prop waypoint is SUPPOSED to end with the gripper's finger
pads touching the object being grasped -- that contact is the point of the
motion, not a defect. Excluding it everywhere would hide a real mid-path
collision; excluding it only in the last `final_exempt_steps` interpolation
steps (near the target end of the sweep) keeps that protection for the
approach corridor while not flagging the intended terminal touch. If the
caller knows exactly which geoms the arm is about to grasp
(`target_geom_names`), the exemption is restricted to just those geoms --
otherwise it falls back to exempting finger-pad contact against ANY prop
geom near the end of the sweep (still never against the table or the other
arm).

**`num_endpoint_seeds` -- why a SINGLE IK solution at the target is not
enough, measured directly (`scripts/validate_swept_path_gate.py`'s own dev
log, not assumed).** `ik.solve_position_ik` is a 5-DoF damped-least-squares
solve against a 3-DoF position target (ADR-024) -- the two spare degrees of
freedom form a REDUNDANT null space: infinitely many joint configurations
place the pinch point at the same target, differing only in elbow/wrist
pose. The solver has no null-space regularization term, so which particular
member of that family it lands on depends on the STARTING joint angles it
is seeded from. `_drive_to_target`'s real closed loop re-seeds from
`env.data`'s CURRENT (continuously, physically evolving) qpos every single
physics step -- so over an up-to-500-step waypoint it silently drifts
through this null space while the task-space (pinch-point) error stays
small throughout. Measured directly for `pick(A, fork)` waypoint 1 on
redesign's geometry: a single unperturbed solve from `start_qpos` lands on
a config whose straight-line joint path is genuinely collision-free (0/101
interpolated steps), while the REAL 500-step closed loop (same start, same
target) drifts to a DIFFERENT null-space member and stalls in a real
`armA-vs-table_top` + cross-arm collision by step 168 -- exactly what Stage
3 (ADR-060) reported. A 30-trial sweep of RANDOMLY PERTURBED starting seeds
(same start pose, same target, only the null-space branch varies) found
8/30 land on configs that collide (`armA_moving_finger_pad`/`armA_wrist`
vs. `armB_upper_arm`) -- the SAME collision family (cross-arm, the shared
band ADR-058 already measured as too narrow), just a different specific
geom pair than the one the 500-step trace happened to stall at. A single
endpoint sample is therefore not representative of what the real controller
can actually reach; `num_endpoint_seeds > 1` solves the SAME fixed target
from `num_endpoint_seeds` different starting perturbations (seed 0 is
ALWAYS the unperturbed solve, exactly reproducing the `num_endpoint_seeds=1`
behaviour -- the same "seed 0 is the unperturbed configuration" convention
`ik.py`'s own ADR-056 restart mechanism already uses), sweeps EACH resulting
joint-space path from the SAME real `start_qpos`, and reports a collision if
ANY sampled branch collides. This is still a single fixed target xyz and
still pure joint-space interpolation per waypoint -- it does not resolve
per Cartesian step, it samples which of several EQUALLY VALID endpoints the
real redundant solver might land the arm on.
"""
from __future__ import annotations

import pathlib
import sys
from dataclasses import dataclass, field

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

import mujoco  # noqa: E402

from bimanual.control import ik  # noqa: E402

# ---------------------------------------------------------------------------
# Geometry vocabulary used by the exemption rule. These names are read from
# the compiled model wherever possible (see `_geom_label`); the two constant
# sets below only need to name geoms whose special TREATMENT (exempt near a
# target) matters, not enumerate every geom in the scene.
# ---------------------------------------------------------------------------
FINGER_PAD_GEOM_NAMES = {
    "armA_static_finger_pad", "armA_moving_finger_pad",
    "armB_static_finger_pad", "armB_moving_finger_pad",
}
#: Every pickable prop's own geoms (verified scene geometry,
#: `scripts/gen_dual_scene.py`). Used as the exemption's default "what is a
#: prop the gripper might legitimately be touching" set when the caller does
#: not name the specific target geoms.
PROP_GEOM_NAMES = {
    "plate_foot", "plate_dish",
    "mug_body", "mug_handle",
    "fork_handle", "fork_head",
    "spoon_handle", "spoon_bowl",
    "water_bottle_body", "water_bottle_cap",
}

#: Per-prop breakdown of `PROP_GEOM_NAMES`, so a caller checking one
#: specific skill's target (e.g. "this waypoint's target is the mug") can
#: pass ONLY that prop's own geoms as `target_geom_names`, rather than
#: exempting every prop in the scene at once.
PROP_GEOMS_BY_BODY = {
    "plate": {"plate_foot", "plate_dish"},
    "mug": {"mug_body", "mug_handle"},
    "fork": {"fork_handle", "fork_head"},
    "spoon": {"spoon_handle", "spoon_bowl"},
    "water_bottle": {"water_bottle_body", "water_bottle_cap"},
}


def _geom_label(model, gid: int) -> str:
    """Geom name if the compiled model has one, else a body-qualified
    fallback (`so101_dual_table.xml`'s upstream arm meshes are largely
    unnamed -- see the geom dump in this stage's own diagnostic notes) so
    every contact is still diagnosable by WHICH BODY it belongs to, even
    when MuJoCo assigned no geom name.
    """
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
    if name:
        return name
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid]))
    return f"{body_name or 'unknown'}#geom{gid}"


#: Mirrors `skills_scripted.CRUSH_THRESHOLD_M` exactly (frozen file, value
#: read from there at import time so the two can never drift independently
#: -- see `_effective_tol`'s docstring for why this gate needs it too).
_CRUSH_THRESHOLD_M = -0.02


def _is_exempt_pair(n1: str, n2: str, target_geom_names) -> bool:
    """True if this contact is a finger-pad-vs-legitimate-grasp-target pair
    that should not count against the gate NEAR the end of a sweep (see
    module docstring). Never exempts anything else -- table contact,
    cross-arm contact and arm self-contact are always counted.
    """
    pair = {n1, n2}
    finger = pair & FINGER_PAD_GEOM_NAMES
    other = pair - FINGER_PAD_GEOM_NAMES
    if not finger or len(other) != 1:
        return False
    other_name = next(iter(other))
    allowed = target_geom_names if target_geom_names is not None else PROP_GEOM_NAMES
    return other_name in allowed


def _effective_tol(n1: str, n2: str, arm: str, target_geom_names, penetration_tol: float) -> float:
    """The penetration bar THIS contact pair must clear -- `penetration_tol`
    for everything, EXCEPT a contact between (any geom of) the arm being
    swept and (a geom of) its OWN current grasp target, which gets the
    looser `skills_scripted.CRUSH_THRESHOLD_M` bar instead.

    Why this exists (found running Step 3's geometry search, not assumed):
    an early version of this gate only exempted FINGER-PAD-vs-target
    contact, matching the module docstring's original framing ("the gripper
    is meant to touch what it grasps"). In practice, several genuinely fine
    candidate configurations were flagged purely because the arm's WRIST
    segment (not the finger pads) swings close to a wide/tall prop
    (`plate`'s overhanging dish, the `mug` body, the `water_bottle`) during
    the final approach into its own grasp point -- exactly the situation
    `skills_scripted._prop_collision_violations`'s `target_body` parameter
    ALREADY treats as expected, at EVERY waypoint (APPROACH/DESCEND/GRIP/
    RETREAT alike, per that function's own docstring), using the SAME
    `CRUSH_THRESHOLD_M` bar this function reuses. Mirroring that existing,
    already-adopted real-controller behaviour makes this gate a more
    faithful predictor of what the real controller will actually accept,
    rather than a stricter proxy that rejects layouts the real skill would
    happily run. Bystander props, the table and the other arm are NEVER
    given this looser bar -- only the swept arm's own declared target.
    """
    if target_geom_names:
        pair = {n1, n2}
        this_arm_prefix = f"arm{arm}_"
        is_this_arm = [p.startswith(this_arm_prefix) for p in pair]
        is_target = [p in target_geom_names for p in pair]
        if any(is_this_arm) and any(is_target) and not (is_this_arm[0] and is_this_arm[1]):
            return _CRUSH_THRESHOLD_M
    return penetration_tol


@dataclass
class ContactViolation:
    step: int
    geom1: str
    geom2: str
    dist: float
    seed: int = 0
    suspected_artifact: bool = False

    def as_dict(self) -> dict:
        return {
            "seed": self.seed, "step": self.step, "geom1": self.geom1, "geom2": self.geom2,
            "dist": self.dist, "suspected_artifact": self.suspected_artifact,
        }


def swept_path_clear(
    model,
    data,
    arm: str,
    start_qpos: np.ndarray,
    target_xyz,
    n_steps: int = 25,
    penetration_tol: float = -0.005,
    target_geom_names: "set[str] | None" = None,
    final_exempt_steps: int = 3,
    num_endpoint_seeds: int = 8,
    seed_noise_rad: float = 0.6,
    implausible_penetration_m: float = -0.15,
) -> "tuple[bool, float, int, list[dict]]":
    """Check whether the STRAIGHT joint-space path from `start_qpos` to the
    IK solution for `arm` at `target_xyz` is collision-clear.

    Args:
        model: compiled `mujoco.MjModel`.
        data: accepted for the exact signature the task specifies and to
            match every other gate function in this repo
            (`verify_stage2_gate.py`, `search_home_keyframe.py`'s
            `evaluate_candidate`) that takes a live `mujoco.MjData` -- but
            NEVER read or mutated here: every pose this function evaluates
            (the IK seed, every interpolated step, every endpoint-seed
            perturbation) is built on its own fresh scratch `mujoco.MjData`
            from `start_qpos`, so a caller with no live `data` handy (e.g. a
            geometry-search loop scoring many candidates, never itself
            stepping a real episode) may pass `None`.
        arm: "A" or "B" -- which arm is swept. Every OTHER dof (the other
            arm, the props, the drawer) is held fixed at `start_qpos`'s
            value throughout the sweep.
        start_qpos: full-length `qpos` vector (e.g. a copy of `env.data.qpos`
            at the pose the real controller would start this waypoint from).
        target_xyz: world-frame xyz the waypoint is driving toward (the same
            argument a real `_run_waypoint`/`_drive_to_target` call would
            receive).
        n_steps: number of interpolated configurations checked, INCLUDING
            both endpoints (`np.linspace(0, 1, n_steps)`).
        penetration_tol: a contact with `dist < penetration_tol` counts as a
            genuine collision. Matches `verify_stage2_gate.py`'s
            `CROSS_ARM_DEPTH_TOL_M` (-0.005 m) by default -- deliberately
            looser than `skills_scripted.TABLE_COLLISION_DEPTH_TOL_M`
            (-0.001 m), since this gate is a PLACEMENT decision tool, not a
            per-step runtime abort, and a mm-scale contact-solver graze
            should not by itself veto a whole layout.
        target_geom_names: see `_is_exempt_pair`. `None` (default) exempts
            finger-pad contact against ANY prop geom near the end of the
            sweep; pass the specific grasp target's geom name(s) for a
            tighter check.
        final_exempt_steps: how many of the LAST interpolated steps get the
            finger-pad exemption (see module docstring).
        num_endpoint_seeds: how many candidate endpoint IK solutions to try
            (see module docstring's "why a single IK solution is not
            enough"). Seed 0 is ALWAYS the unperturbed solve from
            `start_qpos` exactly -- `num_endpoint_seeds=1` reproduces the
            literal "interpolate to THE IK solution" behaviour with no
            multi-seed search at all. Seeds 1..N-1 solve the SAME target
            from `start_qpos` with a per-joint uniform perturbation applied
            to the STARTING angles only (never the target), so each finds a
            different member of the position solve's redundant null space.
        seed_noise_rad: half-width, radians, of seeds 1..N-1's starting-angle
            perturbation. Deterministic across seeds/runs/machines (see
            `_seed_rng` below), matching `ik.py` ADR-056's own
            reproducibility convention.
        implausible_penetration_m: a contact with `dist` DEEPER than this
            (e.g. -0.15 m in a scene where every link and prop is a few cm
            across) is excluded from `clear`/`worst_penetration` and instead
            reported separately (see `suspected_mesh_artifacts` below,
            surfaced through `contact_pairs`' own `"suspected_artifact"`
            flag). Found NECESSARY empirically while running Step 3's
            geometry search (`search_home_keyframe_stage4.py`): a broad
            joint-angle search occasionally lands an interpolated pose on
            the SAME class of MuJoCo convex-hull mesh artifact ADR-058
            documented (-0.39 m on a converged, working `pick(A, fork)`
            pose) and ADR-035 independently recorded (-0.222 m) -- this is
            NOT the distrusted `table_top` penetration FILTER the task names
            (a different, derived computation this module never imports or
            reuses); it is the SAME raw `data.contact[i].dist` the task
            requires, at a MAGNITUDE no real link-scale collision in this
            scene could produce without the physics engine itself exploding
            on the very next `mj_step`. -0.15 m sits comfortably above every
            MEASURED genuine collision this stage's own diagnosis found
            (worst genuine one: -0.098 m, `armA_wrist` swept directly
            through the water bottle's own body) and comfortably below both
            documented artifact magnitudes (-0.222 m, -0.39 m), so it
            separates the two classes without needing to guess a value.

    Returns:
        (clear, worst_penetration, worst_step, contact_pairs)
        - clear: True iff, for EVERY sampled endpoint seed, no NON-EXEMPT
          contact anywhere on that seed's sweep has `dist < penetration_tol`.
        - worst_penetration: the most negative `dist` seen among all
          non-exempt contacts across every sampled seed's sweep (0.0 if none
          were ever negative -- i.e. every sampled branch was totally clean).
        - worst_step: the interpolation step index (0..n_steps-1) at which
          `worst_penetration` occurred, or -1 if none.
        - contact_pairs: list of dicts (`seed`, `step`, `geom1`, `geom2`,
          `dist`, `suspected_artifact`) for every non-exempt contact that
          breached `penetration_tol`, across every sampled seed -- the
          diagnosable violation list, NOT every negative-dist contact (a
          converged resting pose has plenty of harmless micro-grazes).
          Entries deeper than `implausible_penetration_m` are still
          INCLUDED here (never silently hidden) but flagged
          `suspected_artifact=True` and do NOT count toward `clear`/
          `worst_penetration`.
    """
    target = np.asarray(target_xyz, dtype=np.float64).reshape(3)
    start_qpos = np.asarray(start_qpos, dtype=np.float64).reshape(-1)

    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ik.arm_joint_names(arm)]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    joint_ranges = [tuple(model.jnt_range[j]) for j in joint_ids]
    start_angles = np.array([start_qpos[a] for a in qpos_adrs], dtype=np.float64)

    # Deterministic per-(arm, target) RNG, same integer-only formula
    # convention `ik.py`'s ADR-056 restart mechanism uses -- reproducible on
    # any machine regardless of PYTHONHASHSEED (never Python's built-in
    # hash()).
    seed_int = (ord(arm) * 1_000_003) ^ int(round(target[0] * 1e6)) ^ int(round(target[1] * 1e6) * 7) \
        ^ int(round(target[2] * 1e6) * 13)
    rng = np.random.default_rng(abs(seed_int) % (2**32))

    worst_penetration = 0.0
    worst_step = -1
    violations: list[ContactViolation] = []
    n_steps = max(2, int(n_steps))
    num_endpoint_seeds = max(1, int(num_endpoint_seeds))

    for seed_idx in range(num_endpoint_seeds):
        seed_start_angles = start_angles.copy()
        if seed_idx > 0:
            noise = rng.uniform(-seed_noise_rad, seed_noise_rad, size=len(qpos_adrs))
            for k, (lo, hi) in enumerate(joint_ranges):
                seed_start_angles[k] = float(np.clip(seed_start_angles[k] + noise[k], lo, hi))

        seed_data = mujoco.MjData(model)
        seed_data.qpos[:] = start_qpos
        for k, qadr in enumerate(qpos_adrs):
            seed_data.qpos[qadr] = seed_start_angles[k]
        seed_data.qvel[:] = 0.0
        mujoco.mj_forward(model, seed_data)
        solution = ik.solve_position_ik(model, seed_data, arm, target)
        if seed_idx > 0 and solution.position_error_m >= ik.IK_POSITION_TOLERANCE_M:
            # This perturbed seed did not even converge to the target -- it
            # is not a real alternative endpoint the controller could settle
            # on, so it is skipped rather than swept (sweeping a path to a
            # point that is not actually the requested target would be
            # checking the wrong motion).
            continue
        end_angles = np.asarray(solution.joint_angles, dtype=np.float64)

        # ALWAYS interpolate from the REAL start_qpos (never from the
        # perturbed seed, which only exists to pick which null-space branch
        # of the target the arm might land on) -- the arm genuinely starts
        # this waypoint at start_qpos regardless of which branch it drifts
        # toward.
        for step_idx, t in enumerate(np.linspace(0.0, 1.0, n_steps)):
            step_qpos = start_qpos.copy()
            interp_angles = start_angles + t * (end_angles - start_angles)
            for k, qadr in enumerate(qpos_adrs):
                step_qpos[qadr] = interp_angles[k]

            step_data = mujoco.MjData(model)
            step_data.qpos[:] = step_qpos
            step_data.qvel[:] = 0.0
            mujoco.mj_forward(model, step_data)

            is_final = step_idx >= n_steps - final_exempt_steps
            for c in range(step_data.ncon):
                con = step_data.contact[c]
                dist = float(con.dist)
                if dist >= 0.0:
                    continue  # not actually penetrating -- within contact margin only
                g1, g2 = int(con.geom1), int(con.geom2)
                n1, n2 = _geom_label(model, g1), _geom_label(model, g2)
                # Full exemption (finger-pad-vs-target, no depth limit) only
                # right at the end of the sweep -- the intended terminal
                # pinch. `_effective_tol` (below) additionally applies the
                # SAME `CRUSH_THRESHOLD_M` bar the real controller already
                # uses for ANY of the swept arm's geoms against its own
                # declared target, at every step (not just the final ones) --
                # see `_effective_tol`'s own docstring.
                if is_final and _is_exempt_pair(n1, n2, target_geom_names):
                    continue
                effective_tol = _effective_tol(n1, n2, arm, target_geom_names, penetration_tol)
                if dist >= effective_tol:
                    continue  # within whichever bar applies to THIS pair -- not a violation
                is_artifact = dist < implausible_penetration_m
                if not is_artifact and dist < worst_penetration:
                    worst_penetration = dist
                    worst_step = step_idx
                violations.append(
                    ContactViolation(step_idx, n1, n2, dist, seed=seed_idx, suspected_artifact=is_artifact)
                )

    clear = worst_penetration >= penetration_tol
    # Worst-first ordering so a Tester reading a truncated printout sees the
    # deepest genuine penetration first; suspected artifacts sort after every
    # genuine violation regardless of their (possibly enormous) magnitude.
    violations.sort(key=lambda v: (v.suspected_artifact, v.dist))
    contact_pairs = [v.as_dict() for v in violations]
    return clear, worst_penetration, worst_step, contact_pairs


if __name__ == "__main__":
    print(__doc__)
    print("This module exposes swept_path_clear(); see scripts/validate_swept_path_gate.py "
          "for the ground-truth validation run, and scripts/diagnose_swept_path.py for the "
          "eight-skill diagnostic sweep.")
