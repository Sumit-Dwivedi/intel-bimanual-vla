"""Redesign Stage 4, Step 4: eight-skill retest at the swept-path-gated
geometry (ADR-061) -- new home fold from `search_home_keyframe_stage4.py`,
0.40 m base separation and prop placement otherwise unchanged from Stage 3
(ADR-060), `HANDOFF_POSITION_XYZ` unchanged.

Byte-identical harness to `scripts/stage3_eight_skill_retest.py` (SAME
skill list, SAME seed, SAME `cameras=None` rationale, SAME fresh-env-per-
skill rule, ADR-047) -- only the geometry the imported modules load has
changed. Kept as a SEPARATE file (not a diff-in-place edit of the Stage 3
script) so both stages' raw run outputs stay independently reproducible
side by side.

Run on bm-ptl:
    python scripts/stage4_eight_skill_retest.py
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
OUT_PATH = REPO_ROOT / "out" / "stage4_eight_skill_retest.json"

SEED = 0

# (label, skill, arm, target_object, params) -- identical to Stage 3's list.
SKILLS = [
    ("pick(A, fork)", "pick", "A", "fork", {}),
    ("place(A, fork, table)", "place", "A", "fork", {"destination": "table"}),
    ("pick(A, water_bottle)", "pick", "A", "bottle", {}),
    ("place(A, water_bottle, table)", "place", "A", "bottle", {"destination": "table"}),
    ("pick(A, mug)", "pick", "A", "mug", {}),
    ("place(A, mug, table)", "place", "A", "mug", {"destination": "table"}),
    ("handoff(A->B, fork)", "handoff", "B", "fork", {"from_arm": "A"}),
    ("handoff(B->A, fork)", "handoff", "A", "fork", {"from_arm": "B"}),
]


def obj_z(env, body_name: str) -> float:
    bid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(env.data.xpos[bid][2])


def run_one(label, skill, arm, target_object, params):
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
        OUT_PATH.write_text(json.dumps(results, indent=2, default=str), newline="\n", encoding="utf-8")
        print(json.dumps(rec, indent=2, default=str), flush=True)

    print("\n" + "=" * 70)
    print("SUMMARY")
    n_pass = 0
    for r in results:
        print(f"  {r['label']:32s} success={r['success']!s:5s} frames_used={r['frames_used']:6d} "
              f"final_z={r['final_z']}")
        n_pass += 1 if r["success"] else 0
    print(f"\n{n_pass}/{len(results)} passed.")
    print(f"Full results: {OUT_PATH}")


if __name__ == "__main__":
    main()
