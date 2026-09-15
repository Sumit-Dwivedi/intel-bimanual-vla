"""Redesign Stage 3, Step 2: eight-skill retest at the new 0.40 m geometry,
new home pose (ADR-060), new handoff constant (ADR-060).

Seed 0, oracle perception. **Note on `cameras=[]` vs `cameras=None`:** the
task brief says `cameras=[]`; `bimanual.control.executor.py:212` (frozen,
unmodified) asserts `default_cameras is None` for oracle-mode
`ScriptedSkillExecutor` and raises if `TableSettingEnv` was constructed with
ANY non-`None` camera list, including an empty one -- confirmed by hitting
that exact `AssertionError` with `cameras=[]` before this fix. `cameras=None`
is therefore the only value this frozen assertion accepts, and is also what
every other oracle-mode script in this repo already uses
(`run_skill.py`, `verify_adr038_skills.py`, `probe_multiseed_diagnostic.py`)
-- both mean the same thing operationally (zero camera renders per step,
oracle-only state), so this script uses `cameras=None` to satisfy the
frozen assertion rather than editing `executor.py`. A FRESH
`TableSettingEnv` and a FRESH `ScriptedSkillExecutor` (and, transitively, a
FRESH `WeldGrasp` -- ADR-030's `_ensure_weld` constructs one lazily on first
`execute()` call against a never-before-seen `env`) are built per skill,
never reused -- ADR-047's own lesson (`WeldGrasp.active_welds` survives
`env.reset()` and poisons later trials) applies to ANY state carried across
skills in one process, not just `env.reset()` specifically, so this script
sidesteps the whole class of bug by never sharing an executor or env across
skills at all.

Read-only with respect to every frozen module this stage's rules name
(`skills_scripted.py` beyond Step 1's single constant, `grasp.py`, `ik.py`,
`executor.py`, `env.py`) -- all imported, never edited.

Run on bm-ptl:
    python scripts/stage3_eight_skill_retest.py
"""
from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402

from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.control.executor import ScriptedSkillExecutor  # noqa: E402
from bimanual.language.skills import SkillCall  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_PATH = REPO_ROOT / "out" / "stage3_eight_skill_retest.json"

SEED = 0

# (label, skill, arm, target_object, params)
SKILLS = [
    ("pick(A, fork)", "pick", "A", "fork", {}),
    ("place(A, fork, table)", "place", "A", "fork", {"destination": "table"}),
    ("pick(A, water_bottle)", "pick", "A", "bottle", {}),
    ("place(A, water_bottle, table)", "place", "A", "bottle", {"destination": "table"}),
    ("pick(A, mug)", "pick", "A", "mug", {}),
    ("place(A, mug, table)", "place", "A", "mug", {"destination": "table"}),
    # handoff is receiver-first: run_handoff(env, to_arm, from_arm, obj).
    # A->B: to_arm=B, from_arm=A. B->A: to_arm=A, from_arm=B.
    ("handoff(A->B, fork)", "handoff", "B", "fork", {"from_arm": "A"}),
    ("handoff(B->A, fork)", "handoff", "A", "fork", {"from_arm": "B"}),
]


def obj_z(env, body_name: str) -> float:
    bid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(env.data.xpos[bid][2])


def run_one(label, skill, arm, target_object, params):
    # FRESH env + FRESH executor, every skill (ADR-047). cameras=None, not
    # cameras=[] -- see module docstring's note on executor.py:212's
    # assertion.
    env = TableSettingEnv(cameras=None)
    executor = ScriptedSkillExecutor()
    env.reset(seed=SEED)

    body_name = sk.OBJECT_BODY_NAME.get(target_object)
    initial_z = obj_z(env, body_name) if body_name else None

    call = SkillCall(skill=skill, arm=arm, target_object=target_object, params=params)
    result = executor.execute(call, env)

    final_z = obj_z(env, body_name) if body_name else None
    holding_arm = executor.weld.is_holding(arm) if executor.weld is not None else None
    other_arm = params.get("from_arm") if skill == "handoff" else None
    holding_other = executor.weld.is_holding(other_arm) if (executor.weld is not None and other_arm) else None

    record = {
        "label": label,
        "skill": skill,
        "arm": arm,
        "target_object": target_object,
        "params": params,
        "seed": SEED,
        "success": bool(result.success),
        "frames_used": int(result.frames_used),
        "reason": result.reason,
        "weld_attach_frame": result.weld_attach_frame,
        "weld_active_at_end": result.weld_active_at_end,
        "initial_z": initial_z,
        "final_z": final_z,
        "holding_acting_arm": holding_arm,
        "holding_other_arm": holding_other,
    }
    env.close()
    return record


def main() -> None:
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    if OUT_PATH.exists():
        OUT_PATH.unlink()

    results = []
    for label, skill, arm, target_object, params in SKILLS:
        print("=" * 70, flush=True)
        print(f"RUNNING: {label}", flush=True)
        rec = run_one(label, skill, arm, target_object, params)
        results.append(rec)
        # Incremental write -- a mid-run crash must not lose earlier results.
        OUT_PATH.write_text(json.dumps(results, indent=2, default=str), newline="\n", encoding="utf-8")
        print(json.dumps(rec, indent=2, default=str), flush=True)

    print("\n" + "=" * 70)
    print("SUMMARY")
    for r in results:
        print(f"  {r['label']:32s} success={r['success']!s:5s} frames_used={r['frames_used']:6d} "
              f"final_z={r['final_z']}")
    print(f"\nFull results: {OUT_PATH}")


if __name__ == "__main__":
    main()
