"""scripts/run_skill.py -- M06a CLI: run one scripted skill against
TableSettingEnv and report the measured outcome.

Example (PLAN.md M06 done-when 1):

    python scripts/run_skill.py --skill pick --object plate --arm A --seed 0

Runs only on bm-ptl (ADR-020): this script imports `bimanual.sim.env`, which
imports `mujoco`, which cannot load on the developer's laptop (Windows Smart
App Control blocks the unsigned `mujoco.dll`).

Prints:
  - the SkillResult (success, reason, frames_used),
  - a measured before/after state delta for the object (or `drawer_slide`
    qpos for `open_drawer`),
  - MuJoCo contact/limit warning counts and the largest joint-limit
    violation observed, so joint-limit/self-collision issues are visible in
    the log rather than silently absorbed by the constraint solver (PLAN.md
    M06 done-when 4).

Exit code is 0 on skill success, 1 on skill failure (so this script is
usable as a scripted smoke test, e.g. in a CI-less "did it still work"
check).
"""

from __future__ import annotations

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import mujoco  # noqa: E402

from bimanual.control import ik  # noqa: E402
from bimanual.control.executor import ScriptedSkillExecutor  # noqa: E402
from bimanual.control.skills_scripted import OBJECT_BODY_NAME  # noqa: E402
from bimanual.language.skills import SkillCall  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

_SKILLS = ("open_drawer", "pick", "place", "handoff")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--skill", required=True, choices=_SKILLS)
    parser.add_argument(
        "--object",
        default=None,
        help="target object (plate, mug, fork, spoon, bottle); required for pick/place/handoff",
    )
    parser.add_argument(
        "--arm",
        default="A",
        choices=["A", "B"],
        help="acting arm; for handoff this is the RECEIVING arm",
    )
    parser.add_argument(
        "--from-arm",
        default=None,
        choices=["A", "B"],
        help="handoff only: the origin arm; defaults to the other of the two arms",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--step-budget", type=int, default=ik.DEFAULT_STEP_BUDGET)
    return parser


def collect_mj_diagnostics(model, data) -> dict:
    """MuJoCo contact/limit warning counts plus the largest joint-limit
    violation observed at the CURRENT state (PLAN.md M06 done-when 4: these
    must be visible to Tester, not silently swallowed).
    """
    warnings = {}
    for i, warning_stat in enumerate(data.warning):
        if int(warning_stat.number) > 0:
            try:
                name = mujoco.mjtWarning(i).name
            except ValueError:
                name = f"warning_index_{i}"
            warnings[name] = int(warning_stat.number)

    max_violation = 0.0
    for jid in range(model.njnt):
        if not model.jnt_limited[jid]:
            continue
        qadr = int(model.jnt_qposadr[jid])
        lo, hi = model.jnt_range[jid]
        q = data.qpos[qadr]
        max_violation = max(max_violation, float(lo - q), float(q - hi), 0.0)

    return {"mj_warnings": warnings, "max_joint_limit_violation": max_violation}


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    if args.skill in ("pick", "place", "handoff") and not args.object:
        print(f"error: --object is required for --skill {args.skill}", file=sys.stderr)
        return 2

    target_object = args.object if args.object else "drawer"

    if args.skill == "handoff":
        from_arm = args.from_arm or ("B" if args.arm == "A" else "A")
        if from_arm == args.arm:
            print("error: handoff requires --arm and --from-arm to differ", file=sys.stderr)
            return 2
        params = {"from_arm": from_arm}
    elif args.skill == "place":
        params = {"destination": "table"}
    else:
        params = {}

    skill_call = SkillCall(skill=args.skill, arm=args.arm, target_object=target_object, params=params)

    # Vision-based skills out of scope per ADR-023. All skills execute
    # state-only for ~0.20ms/step budget.
    env = TableSettingEnv(cameras=None)
    env.reset(seed=args.seed)

    body_name = OBJECT_BODY_NAME.get(target_object) if args.skill in ("pick", "place", "handoff") else None
    initial_z = None
    if body_name is not None:
        body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        initial_z = float(env.data.xpos[body_id][2])

    initial_drawer_qpos = None
    if args.skill == "open_drawer":
        drawer_jid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, "drawer_slide")
        drawer_qadr = int(env.model.jnt_qposadr[drawer_jid])
        initial_drawer_qpos = float(env.data.qpos[drawer_qadr])

    executor = ScriptedSkillExecutor()
    result = executor.execute(skill_call, env, step_budget=args.step_budget)

    print(f"skill={args.skill} arm={args.arm} object={target_object} params={params} seed={args.seed}")
    print(f"result: success={result.success} frames_used={result.frames_used}")
    print(f"reason: {result.reason}")

    if body_name is not None and initial_z is not None:
        body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        final_z = float(env.data.xpos[body_id][2])
        print(
            f"measured: {body_name} z: initial={initial_z:.4f} final={final_z:.4f} "
            f"delta={final_z - initial_z:+.4f}"
        )

    if initial_drawer_qpos is not None:
        drawer_jid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, "drawer_slide")
        drawer_qadr = int(env.model.jnt_qposadr[drawer_jid])
        final_drawer_qpos = float(env.data.qpos[drawer_qadr])
        print(f"measured: drawer_slide qpos: initial={initial_drawer_qpos:.4f} final={final_drawer_qpos:.4f} (limit 0.15)")

    diagnostics = collect_mj_diagnostics(env.model, env.data)
    print(f"diagnostics: {diagnostics}")

    env.close()
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
