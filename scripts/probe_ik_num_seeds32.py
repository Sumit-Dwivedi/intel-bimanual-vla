"""ADR-056 measurement probe (no source change, no skill wiring): does
`solve_position_ik`'s new `num_seeds` random-restart option find a lower
residual than the single-seed solve on a target ALREADY KNOWN to fail its
closed-loop tolerance -- `place(A, water_bottle, table)`'s own destination
approach waypoint, which ADR-034 measured at residual 0.0138 m against the
0.010 m tolerance (a plain kinematic IK-solver miss, not a physics/servo
issue: ADR-034's own diagnostic, `scripts/probe_place_waypoint1_diag.py`,
already showed the residual plateaus rather than shrinking with more
driving steps -- the signature of a local minimum or a true reach-limit,
not "just needs more time").

This script does NOT call `_drive_to_target` (the physical, step-by-step
closed loop `place`'s own code uses) and does NOT touch `skills_scripted.py`
in any way. It reproduces ADR-034's own diagnostic's approach -- pick the
bottle, compute the exact same `approach_above_dest` target, then call
`ik.solve_position_ik` directly on the CURRENT qpos -- and compares the
single-seed (`num_seeds=1`, the default, unchanged) residual against the
`num_seeds=32` residual from the SAME starting qpos and SAME target. CuRobo
(cited in ADR-056) defaults to exactly `num_seeds=32`, which is why 32 was
chosen here rather than some other count.

This is a measurement only. Nothing here is wired into any skill, and
`place`'s own closed-loop `_drive_to_target` call inside `skills_scripted.py`
is untouched -- so `place(A, water_bottle, table)` still fails exactly as
ADR-034 documented, in the shipped code path, regardless of what this probe
finds.
"""

from __future__ import annotations

import numpy as np

from bimanual.control.executor import ScriptedSkillExecutor
from bimanual.control import ik
from bimanual.control.skills_scripted import (
    TABLE_SURFACE_Z, CLEARANCE_HEIGHT_M, PLACE_OFFSET_XY_M, _body_id,
)
from bimanual.language.skills import SkillCall
from bimanual.sim.env import TableSettingEnv

SEED = 0
NUM_SEEDS = 32


def main() -> None:
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)
    try:
        executor = ScriptedSkillExecutor()
        body_id = _body_id(env.model, "water_bottle")

        pick_call = SkillCall(skill="pick", arm="A", target_object="bottle", params={})
        pick_result = executor.execute(pick_call, env)
        print(f"pick success={pick_result.success}")

        # Exactly ADR-034's own destination-approach target computation
        # (probe_place_waypoint1_diag.py), reproduced here unchanged so
        # this measurement is directly comparable to the 0.0138 m figure.
        obj_xy = np.array(env.data.xpos[body_id][:2], dtype=np.float64, copy=True)
        dest_xy = obj_xy + np.array(PLACE_OFFSET_XY_M)
        dest_xy[0] = float(np.clip(dest_xy[0], -0.30, 0.30))
        dest_xy[1] = float(np.clip(dest_xy[1], -0.18, 0.18))
        approach_above_dest = np.array(
            [dest_xy[0], dest_xy[1], TABLE_SURFACE_Z + CLEARANCE_HEIGHT_M]
        )
        print(f"approach_above_dest target = {approach_above_dest}")

        # Baseline: single-seed solve (num_seeds=1, the default -- ADR-056
        # is a genuine no-op here too), from the CURRENT qpos right after
        # pick (same as ADR-034's diagnostic's own last line).
        single = ik.solve_position_ik(env.model, env.data, "A", approach_above_dest)
        print(
            f"num_seeds=1  residual={single.position_error_m:.4f} m "
            f"converged={single.converged} iterations={single.iterations} "
            f"winning_seed={single.winning_seed}"
        )

        # ADR-056 measurement: 32 restart seeds, same target, same starting
        # qpos (each seed perturbs from data's CURRENT qpos independently --
        # data itself is never mutated by solve_position_ik).
        multi = ik.solve_position_ik(
            env.model, env.data, "A", approach_above_dest, num_seeds=NUM_SEEDS
        )
        print(
            f"num_seeds={NUM_SEEDS} residual={multi.position_error_m:.4f} m "
            f"converged={multi.converged} iterations={multi.iterations} "
            f"winning_seed={multi.winning_seed}"
        )

        tol = ik.IK_POSITION_TOLERANCE_M
        print(f"tolerance = {tol:.4f} m")
        print(
            f"improvement: {single.position_error_m - multi.position_error_m:.4f} m "
            f"({'CLEARS' if multi.converged else 'still MISSES'} the {tol:.3f} m tolerance)"
        )
    finally:
        env.close()


if __name__ == "__main__":
    main()
