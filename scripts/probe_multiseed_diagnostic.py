"""ADR-057 diagnostic probe (measurement only, no source change): for five
targets known or suspected to fail `solve_position_ik`'s tolerance, measure
the FULL per-seed residual distribution ADR-056's `num_seeds` random-restart
mechanism samples -- not just the single "best of N" number
`solve_position_ik` itself returns -- so Commit 3 can tell "local minimum,
rescuable by multi-seed" apart from "genuine kinematic boundary, not
rescuable" on a per-target basis, per the interpretation rule in
docs/hardware/m11-multiseed-diagnostic.md.

**Why this script exists instead of reading `IKSolution.winning_seed` alone
(Commit 1's approach).** `solve_position_ik` only returns the BEST result
across its internal restarts -- there is no way to recover what every OTHER
seed's residual was from its return value alone. Per this task's own
implementation note, `ik.py` stays completely untouched: this script
reconstructs each perturbed starting configuration ITSELF (a scratch
`mujoco.MjData` with this arm's 5 joints overridden, exactly the state
`ik.py`'s own internal `_solve_from(seed_overrides)` would have solved from)
and calls `ik.solve_position_ik(..., num_seeds=1)` from each one individually
-- so every one of the 32 residuals is produced by `ik.py`'s own unmodified
DLS loop, just invoked 32 times from outside instead of once from inside.

**Mirroring ADR-056's perturbation EXACTLY, not approximately.** The seed
derivation (`(ord(arm)*1_000_003 + micron_x*7 + micron_y*13 + micron_z*17) %
2**32`, see `ik.py`'s own "Reproducibility" docstring section) and the draw
sequence (one `rng.uniform(-seed_noise_rad, seed_noise_rad, size=5)` call per
seed index, in order, from a single `numpy.random.default_rng(seed)`) are
reproduced verbatim here. Because `numpy.random.default_rng`'s stream is
sequential and deterministic, seeds 1..7 drawn here are IDENTICAL to the
first 7 restart draws `ik.py` itself would make internally for
`num_seeds=8` on the SAME (arm, target) pair -- so slicing this script's own
32-long residual list at [:1], [:8], [:32] reproduces exactly what
`num_seeds=1`, `num_seeds=8`, `num_seeds=32` would each have found, without
calling `solve_position_ik` with `num_seeds>1` even once.

Five targets (see the module docstring cross-referenced in the report for
per-target provenance): (a) `place(A, water_bottle, table)` destination
approach (Commit 1's own known-failing target, reused verbatim as a sanity
check that this script's independent reimplementation of the perturbation
agrees with `ik.py`'s own `num_seeds=32` path); (b) `pick(A, mug)`
waypoint-1 approach; (c) `handoff(B->A, fork)` Phase 1 pick approach
(ADR-037); (d) the pick->handoff chain's Phase 3 `to_arm` approach
(ADR-054/overnight-batch-log.md), seeded from the actual chained-demo
state, not home; (e) `pick(A, water_bottle)` at the seeds ADR-053's 20-seed
sweep found failing in the 10-19 range.

Read-only with respect to every file this batch's constraints name:
`ik.py`, `skills_scripted.py`, `grasp.py`, `executor.py`, `env.py`,
`randomization.py` are all IMPORTED, never edited. This script calls a
handful of underscore-prefixed helpers in `skills_scripted.py`
(`_hold_ctrl`, `_contact_counts`, `_run_waypoint`, `_run_approach_with_staging`,
`_body_id`) to reconstruct the exact intermediate states those modules'
own skills pass through -- reading/calling private helpers from a diagnostic
script is not the same as modifying the module they live in, and nothing
here writes to any of those files.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`.

Usage:
    python scripts/probe_multiseed_diagnostic.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.control.executor import ScriptedSkillExecutor  # noqa: E402
from bimanual.language.skills import SkillCall  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402
from bimanual.sim.randomization import ScenarioRandomizer, SKILL_ENVELOPES  # noqa: E402

NUM_SEEDS_MAX = 32
SEED_NOISE_RAD = 0.15  # ik.py's own default -- mirrored, not re-guessed.

RESULTS_PATH = Path(__file__).resolve().parent.parent / "docs" / "hardware" / "m11-raw-results.json"


def perturbed_residuals(model, data, arm: str, target_pos) -> list[float]:
    """Return `NUM_SEEDS_MAX` residuals: index 0 is the unperturbed seed-0
    solve (identical to what `ik.solve_position_ik(..., num_seeds=1)` alone
    would return); indices 1..31 are drawn with the EXACT SAME RNG formula
    and per-seed `uniform(size=5)` call `ik.py`'s own `num_seeds>1` branch
    uses internally (see module docstring). Each of the 32 residuals comes
    from an independent `ik.solve_position_ik(..., num_seeds=1)` call --
    `ik.py` itself is never modified or monkeypatched.
    """
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in ik.arm_joint_names(arm)
    ]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    joint_ranges = [tuple(model.jnt_range[j]) for j in joint_ids]

    target = np.asarray(target_pos, dtype=np.float64).reshape(3)

    # Seed 0: the unperturbed current configuration -- byte-for-byte the same
    # call `ik.py` itself makes first, unconditionally, for any num_seeds.
    sol0 = ik.solve_position_ik(model, data, arm, target, num_seeds=1)
    residuals = [sol0.position_error_m]

    # Exact mirror of ik.py's "Seed derivation" comment (solve_position_ik,
    # ~line 453 in ik.py at ADR-056/057): pure integer arithmetic on
    # ord(arm) and the target's micron-rounded xyz -- deliberately not
    # Python's hash(), same reasoning ADR-056/ADR-047 already give.
    target_micron = np.round(target * 1_000_000.0).astype(np.int64)
    seed = int(
        (
            ord(arm) * 1_000_003
            + int(target_micron[0]) * 7
            + int(target_micron[1]) * 13
            + int(target_micron[2]) * 17
        )
        % (2**32)
    )
    rng = np.random.default_rng(seed)
    base_qpos = np.array([float(data.qpos[a]) for a in qpos_adrs], dtype=np.float64)

    for seed_idx in range(1, NUM_SEEDS_MAX):
        noise = rng.uniform(-SEED_NOISE_RAD, SEED_NOISE_RAD, size=len(qpos_adrs))
        perturbed = np.array(
            [
                float(np.clip(base_qpos[k] + noise[k], joint_ranges[k][0], joint_ranges[k][1]))
                for k in range(len(qpos_adrs))
            ],
            dtype=np.float64,
        )
        # Build the perturbed starting MjData ourselves -- exactly the state
        # ik.py's own _solve_from(seed_overrides) would solve from -- then
        # call solve_position_ik(..., num_seeds=1) so ik.py's OWN unmodified
        # DLS loop produces this residual, not a reimplementation of it.
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        for k, qadr in enumerate(qpos_adrs):
            scratch.qpos[qadr] = perturbed[k]
        scratch.qvel[:] = 0.0
        mujoco.mj_forward(model, scratch)
        sol = ik.solve_position_ik(model, scratch, arm, target, num_seeds=1)
        residuals.append(sol.position_error_m)

    return residuals


def summarize(residuals: list[float]) -> dict:
    """Per-`num_seeds` (1, 8, 32) summary: best residual, variance across
    that many seeds, count of distinct values (rounded to 1e-5), winning
    seed index -- the exact quantities the report needs, computed by
    slicing the SAME 32-long residual list (see module docstring on why
    slicing is valid: the RNG stream is sequential and deterministic).
    """
    out = {}
    arr_all = np.asarray(residuals, dtype=np.float64)
    for n in (1, 8, 32):
        arr = arr_all[:n]
        best = float(np.min(arr))
        variance = float(np.var(arr))  # population variance; 0.0 for n=1 by construction
        distinct = len(set(np.round(arr, 5).tolist()))
        winning_seed = int(np.argmin(arr))
        out[f"num_seeds={n}"] = {
            "best_residual_m": round(best, 5),
            "variance": variance,
            "distinct_values": distinct,
            "winning_seed": winning_seed,
        }
    return out


def _append_result(label: str, payload: dict) -> None:
    """Write incrementally (task constraint: a mid-run failure must not
    lose earlier targets' results). Appends one JSON line per target.
    """
    RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps({"target": label, **payload}) + "\n")
    print(f"\n=== {label} ===")
    print(json.dumps(payload, indent=2))


def target_a() -> None:
    """place(A, water_bottle, table) destination approach -- Commit 1's own
    known-failing target (ADR-056/034), reused verbatim as a cross-check
    that this script's independent perturbation reimplementation agrees
    with ik.py's own num_seeds=32 path (both should read 0.0138 m,
    winning_seed=0, per ADR-056's own probe)."""
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    try:
        executor = ScriptedSkillExecutor()
        body_id = sk._body_id(env.model, "water_bottle")
        pick_call = SkillCall(skill="pick", arm="A", target_object="bottle", params={})
        pick_result = executor.execute(pick_call, env)

        obj_xy = np.array(env.data.xpos[body_id][:2], dtype=np.float64, copy=True)
        dest_xy = obj_xy + np.array(sk.PLACE_OFFSET_XY_M)
        dest_xy[0] = float(np.clip(dest_xy[0], -0.30, 0.30))
        dest_xy[1] = float(np.clip(dest_xy[1], -0.18, 0.18))
        target = np.array([dest_xy[0], dest_xy[1], sk.TABLE_SURFACE_Z + sk.CLEARANCE_HEIGHT_M])

        residuals = perturbed_residuals(env.model, env.data, "A", target)
        _append_result(
            "a_place_water_bottle_destination_approach",
            {
                "pick_success": bool(pick_result.success),
                "target": target.tolist(),
                "residuals_all_32": [round(r, 5) for r in residuals],
                "summary": summarize(residuals),
            },
        )
    finally:
        env.close()


def target_b() -> None:
    """pick(A, mug) waypoint-1 approach -- fresh measurement, no prior known
    residual. Entry state = HOME (arm A has not moved since reset), same
    methodology as target (a): a one-shot solve from the state the physical
    waypoint-1 drive would itself begin from."""
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)
    try:
        body_id = sk._body_id(env.model, "mug")
        obj_pos0 = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
        offset = sk.GRASP_POINT_OFFSET_M.get("mug", np.zeros(3))
        grasp_point = obj_pos0 + offset
        top_local_z = sk.OBJECT_TOP_LOCAL_Z_M.get("mug", offset[2])
        hover_z = max(float(grasp_point[2]), float(obj_pos0[2] + top_local_z)) + sk.CLEARANCE_HEIGHT_M
        hover = np.array([grasp_point[0], grasp_point[1], hover_z])

        residuals = perturbed_residuals(env.model, env.data, "A", hover)
        _append_result(
            "b_pick_mug_waypoint1_approach",
            {
                "target": hover.tolist(),
                "residuals_all_32": [round(r, 5) for r in residuals],
                "summary": summarize(residuals),
            },
        )
    finally:
        env.close()


def target_c() -> None:
    """handoff(B->A, fork) Phase 1 pick approach -- known 0.0954 m (ADR-037,
    scripts/chained_demo.py's own documented mirror run). Reproduces
    run_pick(env, 'B', 'fork', ...)'s own waypoint-1 target computation from
    a fresh env.reset(seed=0) with NO randomizer (ADR-037's own VERIFY runs
    and chained_demo.py both use this fixed-layout convention). Entry state
    = HOME (arm B's first move in this call). Additionally runs the REAL
    500-step waypoint-1 drive (sk._run_waypoint, unmodified) as a
    cross-check that the documented 0.0954 m figure is reproduced, and
    measures the 32-seed perturbation from WHICHEVER state (entry vs.
    post-drive-terminal) actually matches it -- reported explicitly either
    way, not assumed.
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)  # no randomizer: fixed default layout (ADR-048/chained_demo.py convention)
    try:
        body_id = sk._body_id(env.model, "fork")
        obj_pos0 = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
        offset = sk.GRASP_POINT_OFFSET_M.get("fork", np.zeros(3))
        grasp_point = obj_pos0 + offset
        top_local_z = sk.OBJECT_TOP_LOCAL_Z_M.get("fork", offset[2])
        hover_z = max(float(grasp_point[2]), float(obj_pos0[2] + top_local_z)) + sk.CLEARANCE_HEIGHT_M
        hover = np.array([grasp_point[0], grasp_point[1], hover_z])

        entry_sol = ik.solve_position_ik(env.model, env.data, "B", hover, num_seeds=1)
        entry_residuals = perturbed_residuals(env.model, env.data, "B", hover)

        # Cross-check: run the REAL waypoint-1 drive (same call run_pick makes
        # internally) to see the actual post-drive terminal residual.
        baseline = sk._contact_counts(env)
        ok, used, reason = sk._run_waypoint(
            env, "B", hover, sk.GRIPPER_OPEN_FRACTION,
            min(sk.APPROACH_DESCENT_STEPS, ik.DEFAULT_STEP_BUDGET), baseline, target_object="fork",
        )
        terminal_sol = ik.solve_position_ik(env.model, env.data, "B", hover, num_seeds=1)
        terminal_residuals = perturbed_residuals(env.model, env.data, "B", hover)

        _append_result(
            "c_handoff_B_to_A_fork_phase1_pick_approach",
            {
                "target": hover.tolist(),
                "documented_residual_m": 0.0954,
                "entry_state_one_shot_residual_m": round(entry_sol.position_error_m, 5),
                "entry_state_residuals_all_32": [round(r, 5) for r in entry_residuals],
                "entry_state_summary": summarize(entry_residuals),
                "real_drive_waypoint_ok": ok,
                "real_drive_frames_used": used,
                "real_drive_reason": reason,
                "terminal_state_one_shot_residual_m": round(terminal_sol.position_error_m, 5),
                "terminal_state_residuals_all_32": [round(r, 5) for r in terminal_residuals],
                "terminal_state_summary": summarize(terminal_residuals),
            },
        )
    finally:
        env.close()


def target_d() -> None:
    """Chain Phase 3 to_arm approach -- known 0.0875 m
    (overnight-batch-log.md's ADR-054 entry). Seeded from the ACTUAL
    chained-demo state: pick(A, fork) run for real (weld-backed), then
    Phase 1 skipped (already_held guard, ADR-054) and Phase 2
    (from_arm approaches transfer_point) run for real via
    sk._run_approach_with_staging -- exactly scripts/chained_demo.py's own
    step 1 + run_handoff's own Phase 1/2, reproduced by calling the same
    unmodified helper functions in the same order run_handoff itself does.
    Entry state for Phase 3 = qpos the INSTANT Phase 2 finishes (to_arm/B
    has not moved yet -- still at HOME). Also runs the REAL Phase-3
    waypoint drive as a cross-check against the documented 0.0875 m,
    exactly as target_c does.
    """
    env = TableSettingEnv(cameras=None)
    env.reset(seed=0)  # chained_demo.py's own SEED=0, no randomizer
    weld = WeldGrasp(env)
    try:
        r1 = sk.run_pick(env, "A", "fork", weld=weld)
        if not (r1.success and weld.is_holding("A") == "fork"):
            _append_result(
                "d_chain_phase3_to_arm_approach",
                {"error": f"step 1 (pick) did not reach the expected state: {r1.reason}"},
            )
            return

        hx, hy, hz = sk.HANDOFF_POSITION_XYZ
        transfer_point = np.array([hx, hy, hz])
        to_arm_hold = sk._hold_ctrl(env)
        baseline = sk._contact_counts(env)
        ok2, used2, reason2 = sk._run_approach_with_staging(
            env, "A", transfer_point, sk.GRIPPER_CLOSE_FRACTION, ik.DEFAULT_STEP_BUDGET, baseline,
            target_object="fork", hold_ctrl_base=to_arm_hold,
        )
        if not ok2:
            _append_result(
                "d_chain_phase3_to_arm_approach",
                {"error": f"phase 2 (from_arm approach) did not converge as documented: {reason2}"},
            )
            return

        from_arm_hold = sk._hold_ctrl(env)
        side = 1.0  # to_arm == "B": arm B base at y=+0.25, receiving point on the OPPOSITE side
        receiving_point = transfer_point + np.array([0.0, side * sk.HANDOFF_SIDE_OFFSET_M, 0.0])

        entry_sol = ik.solve_position_ik(env.model, env.data, "B", receiving_point, num_seeds=1)
        entry_residuals = perturbed_residuals(env.model, env.data, "B", receiving_point)

        # Cross-check: the REAL Phase-3 direct-shot waypoint drive (the exact
        # first call _run_approach_with_staging makes for to_arm).
        ok3, used3, reason3 = sk._run_waypoint(
            env, "B", receiving_point, sk.GRIPPER_OPEN_FRACTION, sk.APPROACH_DESCENT_STEPS, baseline,
            target_object="fork", hold_ctrl_base=from_arm_hold,
        )
        terminal_sol = ik.solve_position_ik(env.model, env.data, "B", receiving_point, num_seeds=1)
        terminal_residuals = perturbed_residuals(env.model, env.data, "B", receiving_point)

        _append_result(
            "d_chain_phase3_to_arm_approach",
            {
                "target": receiving_point.tolist(),
                "documented_residual_m": 0.0875,
                "entry_state_one_shot_residual_m": round(entry_sol.position_error_m, 5),
                "entry_state_residuals_all_32": [round(r, 5) for r in entry_residuals],
                "entry_state_summary": summarize(entry_residuals),
                "real_drive_waypoint_ok": ok3,
                "real_drive_frames_used": used3,
                "real_drive_reason": reason3,
                "terminal_state_one_shot_residual_m": round(terminal_sol.position_error_m, 5),
                "terminal_state_residuals_all_32": [round(r, 5) for r in terminal_residuals],
                "terminal_state_summary": summarize(terminal_residuals),
            },
        )
    finally:
        env.close()


def target_e() -> None:
    """pick(A, water_bottle) at the 7 env-seeds (10,12,13,15,17,18,19) that
    ADR-053's 20-seed Track A sweep found FAILING in the 10-19 range (per
    docs/hardware/m08-extended-eval.md's own seed table: all 7 fail with
    reason weld_attach_failed_after_300_frames, frames_used=1300 -- i.e.
    waypoint 1 (approach) and waypoint 2 (descend) ALREADY CONVERGE; the
    documented failure is a GRIP/weld-mechanism failure, not an IK
    convergence failure). Measured anyway, for completeness and honesty:
    this reports whether waypoint-1's own IK residual is trivial at each of
    these 7 seeds (expected), which is itself the finding that matters for
    Commit 3 -- multi-seed IK restarts cannot rescue a failure that never
    was an IK failure.
    """
    failing_seeds = [10, 12, 13, 15, 17, 18, 19]
    randomizer = ScenarioRandomizer(envelopes={"water_bottle": SKILL_ENVELOPES["pick_bottle"]["water_bottle"]})
    per_seed = {}
    for seed in failing_seeds:
        env = TableSettingEnv(cameras=None)
        env.reset(seed=seed, cameras=None, randomizer=randomizer)
        try:
            body_id = sk._body_id(env.model, "water_bottle")
            obj_pos0 = np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)
            offset = sk.GRASP_POINT_OFFSET_M.get("bottle", np.zeros(3))
            grasp_point = obj_pos0 + offset
            top_local_z = sk.OBJECT_TOP_LOCAL_Z_M.get("bottle", offset[2])
            hover_z = max(float(grasp_point[2]), float(obj_pos0[2] + top_local_z)) + sk.CLEARANCE_HEIGHT_M
            hover = np.array([grasp_point[0], grasp_point[1], hover_z])

            residuals = perturbed_residuals(env.model, env.data, "A", hover)
            per_seed[str(seed)] = {
                "target": hover.tolist(),
                "residuals_all_32": [round(r, 5) for r in residuals],
                "summary": summarize(residuals),
            }
        finally:
            env.close()

    _append_result(
        "e_pick_water_bottle_20seed_sweep_failing_seeds",
        {
            "note": (
                "Documented failure mode at all 7 seeds is "
                "weld_attach_failed_after_300_frames (m08-extended-eval.md), "
                "NOT an IK convergence failure -- frames_used=1300 there "
                "means waypoint 1 (approach) and waypoint 2 (descend) BOTH "
                "already converged before GRIP exhausted its 300-frame "
                "budget. Residuals below are waypoint-1's own one-shot IK "
                "residual at each seed, measured for completeness."
            ),
            "per_seed": per_seed,
        },
    )


def main() -> None:
    # Fresh file each run (task constraint: incremental writes so a mid-run
    # failure loses nothing -- each target below appends one line as soon as
    # it completes, so this only needs to start clean once per invocation).
    if RESULTS_PATH.exists():
        RESULTS_PATH.unlink()

    target_a()
    target_b()
    target_c()
    target_d()
    target_e()

    print(f"\nAll targets complete. Raw results: {RESULTS_PATH}")


if __name__ == "__main__":
    main()
