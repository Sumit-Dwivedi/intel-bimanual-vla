"""Proves ADR-047: `WeldGrasp.reset()` / `ScriptedSkillExecutor.reset()` fix
the cross-trial state-corruption bug the pre-M07/M08 audit found
(`docs/hardware/m10-pre-m07-audit.md`'s "Zero-th finding").

**The bug, restated.** `WeldGrasp.active_welds` is a plain Python `dict` on
the `WeldGrasp` instance, set by `attempt_grasp` and cleared only by
`release()`. `env.reset()` resets MuJoCo's own `data` (`mj_resetData`, plus
the "home" keyframe overlay) -- which DOES clear `data.eq_active` for every
weld back to its compiled inactive default -- but has no way to reach into a
Python object it does not know exists, so `active_welds` survives a bare
`env.reset()` untouched. `ScriptedSkillExecutor._ensure_weld` (ADR-030)
deliberately reuses one `WeldGrasp` across repeated calls against the same
`env` -- correct for a `TaskPlan` run, wrong for a multi-trial/multi-seed
harness (M08's own shape) that calls `env.reset()` directly between trials:
the second trial's grasp attempt for whatever (arm, object) pair was held at
reset time is refused at `attempt_grasp`'s "already holds" gate, surfacing
as `weld_attach_failed_after_N_frames` -- indistinguishable from a genuine
grasp failure.

**What this script does, in one process, against one `TableSettingEnv` +
one `ScriptedSkillExecutor` (the exact reuse pattern that triggers the bug):**

  1. `pick(A, fork)` via the executor -- succeeds, records `final_z_run1`.
  2. **Demonstrates the bug**: calls `env.reset(...)` DIRECTLY (bypassing
     the new `executor.reset()`), then shows the desync directly --
     `weld.is_holding('A')` still reports `'fork'` (stale Python state)
     while `env.data.eq_active[fork_eq_id]` is already `0` (MuJoCo's own
     reset correctly cleared it) -- and shows the practical consequence: a
     second `pick(A, fork)` through the SAME executor now fails with
     `weld_attach_failed_after_N_frames`, even though nothing about the
     scene or the arm's reachability changed.
  3. **Demonstrates the fix**: calls `executor.reset(env, seed=0,
     cameras=None)` (ADR-047) instead, and asserts `is_holding('A') is
     None` and `data.eq_active[fork_eq_id] == 0`.
  4. Re-runs `pick(A, fork)` through the SAME executor and confirms it
     succeeds again, with `final_z` IDENTICAL (not merely close) to
     `final_z_run1` -- the real proof that the corruption is gone and the
     simulation is exactly as reproducible as a fresh env would have been.

Run on bm-ptl (`ov_env`, per ADR-020 -- `mujoco` does not reliably load on
every laptop): `ov_env\\Scripts\\python.exe scripts\\probe_weld_reset.py`.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402

from bimanual.control.executor import ScriptedSkillExecutor  # noqa: E402
from bimanual.language.skills import SkillCall  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

SEED = 0


def _body_z(env: TableSettingEnv, body_name: str) -> float:
    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(env.data.xpos[body_id][2])


def _pick_fork(executor: ScriptedSkillExecutor, env: TableSettingEnv):
    return executor.execute(
        SkillCall(skill="pick", arm="A", target_object="fork", params={}), env
    )


def main() -> int:
    print("=" * 70)
    print("PROBE: WeldGrasp / ScriptedSkillExecutor cross-trial reset (ADR-047)")
    print("=" * 70)

    env = TableSettingEnv(cameras=None)
    executor = ScriptedSkillExecutor()

    # ---- Run 1: pick(A, fork) against a freshly-reset env -----------------
    env.reset(seed=SEED, cameras=None)
    z0_run1 = _body_z(env, "fork")
    result1 = _pick_fork(executor, env)
    final_z_run1 = _body_z(env, "fork")
    print("\n[Run 1] pick(A, fork) on a fresh env")
    print(f"    success={result1.success}  holding={executor.weld.is_holding('A')!r}")
    print(f"    fork z: {z0_run1:.4f} -> {final_z_run1:.4f}")
    print(f"    reason: {result1.reason}")
    assert result1.success, f"Run 1 must succeed as a precondition for this probe; got {result1}"
    assert executor.weld.is_holding("A") == "fork"

    fork_eq_id = executor.weld._eq_ids[("A", "fork")]

    # ---- Step 2: DEMONSTRATE THE BUG --------------------------------------
    # Calls env.reset() DIRECTLY, bypassing the new executor.reset() (i.e.
    # exactly what every pre-ADR-047 caller did). This is the buggy path.
    print("\n[Bug demonstration] env.reset() called directly (weld.reset() skipped)")
    env.reset(seed=SEED, cameras=None)
    stale_holding = executor.weld.is_holding("A")
    mujoco_eq_active = int(env.data.eq_active[fork_eq_id])
    print(f"    weld.is_holding('A') after env.reset() = {stale_holding!r}  (STALE -- should be None)")
    print(f"    env.data.eq_active[fork_eq_id] after env.reset() = {mujoco_eq_active}  (MuJoCo's own state, correctly 0)")
    assert stale_holding == "fork", (
        "expected the bug to reproduce: WeldGrasp.active_welds must still claim 'fork' is "
        f"held after a bare env.reset(); got {stale_holding!r} -- has the bug already been "
        "fixed some other way, or did this probe's setup change?"
    )
    assert mujoco_eq_active == 0, (
        "expected MuJoCo's own data.eq_active to already be 0 immediately after env.reset() "
        f"(mj_resetData's job, independent of this fix); got {mujoco_eq_active}"
    )
    print("    DESYNC CONFIRMED: WeldGrasp believes arm A still holds the fork; MuJoCo disagrees.")

    result_bug = _pick_fork(executor, env)
    print(f"    pick(A, fork) retried in this corrupted state: success={result_bug.success}")
    print(f"    reason: {result_bug.reason}")
    assert result_bug.success is False, (
        "expected the corrupted state to cause a spurious pick failure; it did not -- "
        f"got {result_bug}"
    )
    assert "weld_attach_failed" in result_bug.reason, (
        "expected the specific 'already holds' -> weld_attach_failed_after_N_frames symptom "
        f"the audit documented; got reason={result_bug.reason!r}"
    )
    print("    BUG DEMONSTRATED: pick(A, fork) fails with "
          "'weld_attach_failed_after_N_frames' -- indistinguishable from a genuine grasp "
          "failure, even though nothing about the scene changed.")

    # ---- Step 3: THE FIX ---------------------------------------------------
    print("\n[Fix] executor.reset(env, seed=0, cameras=None) (ADR-047)")
    executor.reset(env, seed=SEED, cameras=None)
    fixed_holding = executor.weld.is_holding("A")
    fixed_eq_active = int(env.data.eq_active[fork_eq_id])
    print(f"    weld.is_holding('A') after executor.reset() = {fixed_holding!r}")
    print(f"    env.data.eq_active[fork_eq_id] after executor.reset() = {fixed_eq_active}")
    assert fixed_holding is None, f"expected is_holding('A') is None after the fix; got {fixed_holding!r}"
    assert fixed_eq_active == 0, f"expected data.eq_active[fork_eq_id] == 0 after the fix; got {fixed_eq_active}"
    print("    FIXED: WeldGrasp and MuJoCo agree again -- nothing held, no active weld.")

    # ---- Step 4: Run 2, through the SAME executor, must reproduce Run 1 ---
    result2 = _pick_fork(executor, env)
    final_z_run2 = _body_z(env, "fork")
    print("\n[Run 2] pick(A, fork) again, same executor, after the fix")
    print(f"    success={result2.success}  holding={executor.weld.is_holding('A')!r}")
    print(f"    fork z: -> {final_z_run2:.6f}  (Run 1 was {final_z_run1:.6f})")
    print(f"    reason: {result2.reason}")
    assert result2.success, f"Run 2 must succeed after the fix; got {result2}"
    assert final_z_run2 == final_z_run1, (
        "THE REAL PROOF: Run 2's final z must be IDENTICAL to Run 1's -- any difference would "
        f"mean the reset did not fully restore a clean trial; run1={final_z_run1!r} "
        f"run2={final_z_run2!r}"
    )
    print("    CONFIRMED: Run 2's final z is bit-identical to Run 1's -- the corruption is gone "
          "and this executor is safe to reuse across trials via executor.reset().")

    env.close()

    print("\n" + "=" * 70)
    print("ALL ASSERTIONS PASSED: WeldGrasp.reset() / ScriptedSkillExecutor.reset() (ADR-047)")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
