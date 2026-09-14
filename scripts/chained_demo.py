"""chained_demo.py -- Fix C: pick -> handoff -> place, chained in ONE continuous
episode (batch brief's "chained skills" demonstration; candidate ADR-054).

**Shape, and why it differs from run_demo.py / run_grounded_demo.py.** Both of
those scripts give EACH skill call a fresh `TableSettingEnv` + fresh
`ScriptedSkillExecutor`/`WeldGrasp` (ADR-047's safe pattern for a set of
INDEPENDENT trials). This script is deliberately the opposite: ONE
`env.reset()`, ONE `WeldGrasp` instance, three skill calls run back to back
against the SAME live physics state -- because the whole point of this demo
is to show the three skills chained within a single episode, not three
isolated single-skill measurements. ADR-047's own hazard (`WeldGrasp.reset()`
must run alongside `env.reset()` or `active_welds` desyncs from MuJoCo's own
`eq_active`) does not apply here: `env.reset()` is called exactly once, before
anything else touches `env` or `weld`, and this script never resets `env`
again or constructs a second `WeldGrasp` against it.

**Randomization: fixed default layout, no `ScenarioRandomizer` passed to
`reset()` -- deliberately, not by omission.** `randomization.py`'s own
`SKILL_ENVELOPES` records fork's per-skill measured rectangles for
`pick_fork`, `place_fork` and `handoff`; ADR-048 already intersected all
three (the module-level `ENVELOPES` export) and found the intersection
collapses to the single point (0, 0, 0, 0) -- i.e. once a chain includes
`handoff`, there is NO non-degenerate (dx, dy) offset for `fork` that is
simultaneously safe for every skill in this chain. Passing
`ScenarioRandomizer(envelopes={"fork": SKILL_ENVELOPES["pick_fork"]["fork"]})`
the way `run_demo.py` does for its INDEPENDENT `pick(A, fork)` step would
offer a real (dx, dy) range for that one step, but this chain does not stop
after `pick` -- the SAME reset must also be safe for the `handoff` call two
steps later, and `handoff`'s own envelope is the single point. So this
script calls `env.reset(seed=SEED)` with `randomizer=None` (the default),
which is BYTE-IDENTICAL across any `seed` value per `TableSettingEnv.reset()`'s
own documented contract when no randomizer is given (ADR-048) -- i.e.
`SEED`'s actual value is inert here, kept only so the call site reads the
same as every other script in this repo. This is stated here, loudly, so a
reader does not mistake "seed=0" for "this scene varies with the seed" --
it does not, on this path, and that is the correct, evidence-based choice
for a chain that includes `handoff`, not an oversight (see this repo's Fix C
task brief, correction 3, and `run_demo.py`'s own handoff step for the same
reasoning applied to a single skill).

**Do not reset `env` between steps.** That is the one continuous-episode
property this whole script exists to demonstrate; each step below acts on
whatever physical state the previous step left behind.

**Known, already-documented blocker this script is very likely to hit at
step 2 (not fixed here -- `skills_scripted.py` is off-limits for this
batch).** `run_handoff`'s Phase 1 (`skills_scripted.py` around line 2136)
calls `run_pick(env, from_arm, target_object, ...)` UNCONDITIONALLY, with no
`already_held` guard -- unlike `run_place`, which ADR-034 fixed to skip its
own nested pick when the object is already held. Because step 1 below
already leaves arm A holding the fork via `weld`, step 2's internal re-pick
targets an object that is airborne in arm A's own grip, and its GRIP
waypoint calls `weld.attempt_grasp("A", "fork")` every step -- which
`grasp.py`'s own "already holds -- release first" gate refuses immediately,
every single step, because `weld.active_welds["A"]` is already `"fork"`.
The attach frame count then exhausts `GRIP_HOLD_FRAMES` (300) with no
attach, producing `weld_attach_failed_after_300_frames`. This exact failure
was already found and documented by `scripts/run_grounded_demo.py`'s own
module docstring (finding 2), for the equivalent two-clause
pick-then-handoff composition. If it reproduces here, this script reports
it as that known architectural gap, not as a bug to chase, and does not
attempt to patch `run_handoff`.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`, which imports
`mujoco`, which cannot load on the developer's Windows laptop.

Usage:
    python scripts/chained_demo.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np  # noqa: E402

from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402
from bimanual.sim.grasp import WeldGrasp  # noqa: E402

# Inert on this path (no randomizer passed to reset()) -- see this module's
# docstring's "Randomization" section for why that is the correct choice for
# a chain that includes handoff. Kept only so this call site reads like
# every other script in this repo.
SEED = 0


def _fork_pos(env) -> np.ndarray:
    body_id = sk._body_id(env.model, "fork")
    return np.array(env.data.xpos[body_id], dtype=np.float64, copy=True)


def _fork_z(env) -> float:
    return float(_fork_pos(env)[2])


def _report_scene(env, weld: WeldGrasp, label: str) -> None:
    fork_pos = _fork_pos(env)
    holding_a = weld.is_holding("A")
    holding_b = weld.is_holding("B")
    print(
        f"  [scene @ {label}] fork_pos={fork_pos}  "
        f"is_holding('A')={holding_a!r}  is_holding('B')={holding_b!r}"
    )


def main() -> int:
    print("=" * 72)
    print("CHAINED DEMO (Fix C): pick(A, fork) -> handoff(A->B, fork) -> place(B, fork, table)")
    print("ONE continuous episode: single env.reset(), single WeldGrasp, no resets between steps.")
    print("=" * 72)
    print()
    print("RANDOMIZATION: env.reset(seed=%d) called with randomizer=None (fixed default layout)."
          % SEED)
    print(
        "  Reasoning (ADR-048): fork's cross-skill-safe envelope -- the intersection of "
        "SKILL_ENVELOPES['pick_fork']['fork'], ['place_fork']['fork'] and ['handoff']['fork'] "
        "in randomization.py -- collapses to the single point (0,0,0,0) once handoff is part "
        "of the chain. There is no non-degenerate offset safe for every skill this chain "
        "calls, so this script deliberately runs at the fixed default scene rather than "
        "passing a ScenarioRandomizer that would only be safe for one of the three steps. "
        "Per TableSettingEnv.reset()'s own documented contract, this makes the scene "
        "byte-identical regardless of SEED's value."
    )
    print()

    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)
    # ONE WeldGrasp for the whole episode -- ADR-047: never construct a
    # second WeldGrasp against the same env within one unreset episode.
    weld = WeldGrasp(env)

    _report_scene(env, weld, "initial (post-reset)")
    results: dict[str, object] = {}

    # ------------------------------------------------------------------
    # Step 1: pick(A, fork)
    # ------------------------------------------------------------------
    print("\n--- STEP 1: pick(A, fork) ---")
    z_before_1 = _fork_z(env)
    r1 = sk.run_pick(env, "A", "fork", weld=weld)
    print(f"  success={r1.success}  reason={r1.reason}")
    print(f"  frames_used={r1.frames_used}  fork z: {z_before_1:.4f} -> {_fork_z(env):.4f}")
    _report_scene(env, weld, "after step 1")
    results["step1_pick_A_fork"] = r1
    step1_ok = bool(r1.success) and weld.is_holding("A") == "fork"
    print(f"  VERIFY is_holding('A') == 'fork': {weld.is_holding('A') == 'fork'} "
          f"-> {'PASS' if step1_ok else 'FAIL'}")

    if not step1_ok:
        print("\nSTOPPING: step 1 (pick) did not succeed -- the chain cannot continue honestly.")
        _print_summary(results, failed_at=1)
        env.close()
        return 1

    # ------------------------------------------------------------------
    # Step 2: handoff(A -> B, fork), receiver-first call per this repo's
    # convention: run_handoff(env, to_arm, from_arm, target_object, ...).
    # ------------------------------------------------------------------
    print("\n--- STEP 2: handoff(A->B, fork)  [run_handoff(env, 'B', 'A', 'fork', weld=weld)] ---")
    r2 = sk.run_handoff(env, "B", "A", "fork", weld=weld)
    print(f"  success={r2.success}  reason={r2.reason}")
    print(f"  frames_used={r2.frames_used}")
    _report_scene(env, weld, "after step 2")
    results["step2_handoff_A_to_B_fork"] = r2
    step2_ok = bool(r2.success) and weld.is_holding("B") == "fork" and weld.is_holding("A") is None
    print(
        f"  VERIFY is_holding('B')=='fork' and is_holding('A') is None: "
        f"{weld.is_holding('B') == 'fork'} and {weld.is_holding('A') is None} "
        f"-> {'PASS' if step2_ok else 'FAIL'}"
    )

    if not step2_ok:
        print("\nSTEP 2 (handoff) FAILED.")
        if "phase 1" in r2.reason and "weld_attach_failed" in r2.reason:
            print(
                "  This matches the KNOWN, already-documented re-pick gap (see this script's "
                "own module docstring, and scripts/run_grounded_demo.py's finding 2):"
            )
            print(
                "  run_handoff's Phase 1 (skills_scripted.py:2136) calls run_pick(env, from_arm, "
                "target_object, ...) UNCONDITIONALLY -- no already_held guard, unlike run_place's "
                "ADR-034 fix. Arm A already holds the fork from step 1, so the nested pick's GRIP "
                "waypoint's weld.attempt_grasp('A', 'fork') is refused every step by grasp.py's "
                "'already holds -- release first' gate, exhausting GRIP_HOLD_FRAMES with no "
                "attach -> weld_attach_failed_after_300_frames."
            )
            print("  NOT fixed here: skills_scripted.py is off-limits for this batch.")
        else:
            print(f"  Failure reason does not match the expected known gap verbatim; reason={r2.reason!r}")
        print(f"\n  Final fork position at failure: {_fork_pos(env)}")
        print(f"  is_holding('A')={weld.is_holding('A')!r}  is_holding('B')={weld.is_holding('B')!r}")

        # Optional fallback: if arm A still genuinely holds the fork (the
        # expected state after this exact failure -- Phase 1's failed nested
        # pick never displaces the ORIGINAL weld from step 1), demonstrate
        # that pick+place alone still works. Explicitly labelled as a
        # FALLBACK, not as the chain succeeding (Fix C task brief item 5).
        if weld.is_holding("A") == "fork":
            print("\n--- FALLBACK (NOT the 3-step chain; the chain FAILED at step 2 above) ---")
            print("--- place(A, fork, table): arm A still holds the fork from step 1 ---")
            z_before_fb = _fork_z(env)
            r_fb = sk.run_place(env, "A", "fork", "table", weld=weld)
            print(f"  success={r_fb.success}  reason={r_fb.reason}")
            print(f"  frames_used={r_fb.frames_used}  fork z: {z_before_fb:.4f} -> {_fork_z(env):.4f}")
            _report_scene(env, weld, "after FALLBACK place(A)")
            results["FALLBACK_place_A_fork_table"] = r_fb

        _print_summary(results, failed_at=2)
        env.close()
        return 1

    # ------------------------------------------------------------------
    # Step 3: place(B, fork, table)
    # ------------------------------------------------------------------
    print("\n--- STEP 3: place(B, fork, table) ---")
    z_before_3 = _fork_z(env)
    r3 = sk.run_place(env, "B", "fork", "table", weld=weld)
    print(f"  success={r3.success}  reason={r3.reason}")
    print(f"  frames_used={r3.frames_used}  fork z: {z_before_3:.4f} -> {_fork_z(env):.4f}")
    _report_scene(env, weld, "after step 3")
    results["step3_place_B_fork_table"] = r3
    step3_ok = bool(r3.success) and weld.is_holding("B") is None
    print(f"  VERIFY fork at rest, weld released: is_holding('B') is None: "
          f"{weld.is_holding('B') is None} -> {'PASS' if step3_ok else 'FAIL'}")

    if not step3_ok:
        print(f"\n  Failing waypoint / reason: {r3.reason}")
        print(f"  Final fork position: {_fork_pos(env)}")

    ok = step1_ok and step2_ok and step3_ok
    _print_summary(results, failed_at=None if ok else 3)

    print(f"\nFINAL FORK POSITION: {_fork_pos(env)}")
    env.close()
    return 0 if ok else 1


def _print_summary(results: dict, failed_at) -> None:
    print("\n" + "=" * 72)
    if failed_at is None:
        print("SUMMARY: chained demo SUCCEEDED -- pick -> handoff -> place, all three PASS.")
    else:
        print(f"SUMMARY: chained demo attempted, FAILS at step {failed_at}.")
    print("=" * 72)
    for key, r in results.items():
        status = "PASS" if getattr(r, "success", False) else "FAIL"
        print(f"  [{status}] {key}: {getattr(r, 'reason', r)}")


if __name__ == "__main__":
    sys.exit(main())
