"""Diagnostic-only probe for the pre-M07/M08 audit
(docs/hardware/m10-pre-m07-audit.md). FINDINGS ONLY -- this script never
modifies skills_scripted.py, grasp.py, ik.py, executor.py, env.py,
scenes/so101/, gen_dual_scene.py, posenet.py, dataset.py, the checkpoint, or
the IR. It only calls the existing, frozen `ScriptedSkillExecutor` /
`TableSettingEnv` / `PoseNetInference` API surface many times over and
records what happens.

Why this script exists at all, rather than just looping `reset(seed=N)`
------------------------------------------------------------------------
`TableSettingEnv.reset(seed=N)` is BYTE-IDENTICAL for every `N` on the
scripted-controller path today -- see that method's own docstring:
"the seed governs only this env's own RNG stream for future randomized
use." M07 (the randomizer that will actually consume that RNG stream) is
not built yet. Looping seeds 0-9 through `reset()` alone would therefore
run the identical deterministic scenario ten times and report a 10/10 or
0/10 that carries no robustness information at all.

So this script does its OWN randomization, using the exact runtime
mechanism `scripts/generate_posenet_data.py` already uses (and this
project's own ADR-038 history already established is the safe one):
overwrite ONE prop's free-joint qpos x/y AFTER `reset()`, leave z and the
quaternion exactly as `reset()` set them, then call `mujoco.mj_forward` to
recompute derived quantities before running a skill. This is NEVER done by
regenerating the scene XML (ADR-038 found that breaks `handoff`). This is
this AUDIT's own randomization, not a feature of `reset(seed=)` -- every
subcommand below prints and records the exact per-seed prop position it
used, so the numbers this script produces are independently reproducible.

Subcommands
-----------
  robustness  -- audit variables 1 & 2. `--skill {handoff,pick_fork,
                 place_fork,pick_bottle} --seeds 0-9 [--jitter 0.05]`.
                 Randomizes the ONE prop that skill targets (see "Scope
                 note" below) by +/-`jitter` metres in x,y around its own
                 default resting position (z untouched), runs the skill via
                 the real `ScriptedSkillExecutor` in oracle mode
                 (`TableSettingEnv(cameras=None)`), and appends one JSON
                 line per seed to `--out` immediately.
  grid        -- audit variable 6. `--skill {...} --step 0.02 --range 0.05`.
                 Same single-prop randomization mechanism, swept on a full
                 grid (not sampled), so the pass/fail ENVELOPE is visible,
                 not just an aggregate success rate.
  posenet_acc -- audit variable 3. Runs 5 skill executions, truncating each
                 one's `step_budget` to ~2%/25%/50%/75% of ITS OWN full
                 `frames_used` (measured first, oracle-mode, full budget) to
                 capture a `posenet_cam` frame at each of 4 mid-execution
                 points. Each frame is run through the real
                 `PoseNetInference` (GPU FP16, ADR-045's default) and
                 compared against the oracle `env.data.xpos` for
                 fork/water_bottle/mug at that same instant.

Scope note (robustness/grid) -- read before citing a number from either.
Each run randomizes ONLY the skill's own target prop, not all five props
simultaneously. `scripts/generate_posenet_data.py`'s
`sample_nonoverlapping_positions` already exists for a full-scene,
collision-avoiding, multi-prop randomization, but exercising THAT here
would silently fold a second, unaudited randomizer's own behaviour into a
report about whether ONE skill tolerates ITS OWN target prop moving --
two unmeasured things conflated into one number. Left for M07 itself to
build properly; flagged in the audit report, not attempted here.

Every subcommand writes to `--out` immediately after each trial completes
(`f.write(...); f.flush()`), per this audit's own SSH-hygiene instruction:
a killed mid-sweep run must leave a valid, partial JSONL file behind, not
lose everything.

**A FRESH `TableSettingEnv` and a FRESH `ScriptedSkillExecutor` are
constructed for EVERY trial in every subcommand below -- this is not free
(a few hundred ms of MuJoCo model compile per trial) but it is required for
CORRECT numbers, not just tidiness. Found during this audit's own dry run,
independently of any of the eight audit variables: `WeldGrasp.active_welds`
(`src/bimanual/sim/grasp.py`) is a plain Python dict on the `WeldGrasp`
INSTANCE, set by a successful `attempt_grasp` and cleared only by
`release()` -- `env.reset()` resets MuJoCo's `data` (which is where
`eq_active`, the constraint's physical on/off flag, actually lives) but has
no way to reach into a `WeldGrasp` Python object it does not even know
exists, so `active_welds` survives a `reset()` untouched. Since
`ScriptedSkillExecutor._ensure_weld` deliberately REUSES one `WeldGrasp`
"across repeated calls against the SAME env" (`executor.py`'s own
docstring), an early trial in a loop that reuses one executor+env across
many `reset()`s can succeed once, and EVERY subsequent trial's
`attempt_grasp` for that same (arm, object) then refuses unconditionally
(`"arm=A already holds 'fork' -- call release(...) first"`) even though the
object is physically back on the table after `reset()` -- silently
deflating every later trial's success rate to look like a reachability
failure (`weld_attach_failed_after_300_frames`) that has nothing to do with
position. Reproduced directly and reported in the audit doc's own findings
(it is exactly the kind of state a future M08 harness must get right if it
reuses one executor/env pair across a seed loop, per this audit's remit).
This script's own subcommands avoid it entirely by never reusing a
`WeldGrasp` across trials -- one fresh env + one fresh executor per trial,
below.**
"""

from __future__ import annotations

import argparse
import json
import pathlib
import time

import mujoco
import numpy as np

from bimanual.control import ik
from bimanual.control.executor import ScriptedSkillExecutor
from bimanual.language.skills import SkillCall
from bimanual.sim.env import TableSettingEnv

RESET_SEED = 0  # every env.reset() below uses this -- deterministic baseline
# (env.py's own reset() docstring); this script's own jitter is layered on
# top, exactly per this module's docstring.

# body name (as used by OBJECT_BODY_NAME / the MJCF), keyed by this script's
# own short skill-config names below.
_PROP_BODY_NAME = {
    "fork": "fork",
    "water_bottle": "water_bottle",
}


def _skill_call(skill_name: str) -> SkillCall:
    if skill_name == "handoff":
        return SkillCall(skill="handoff", arm="B", target_object="fork", params={"from_arm": "A"})
    if skill_name == "pick_fork":
        return SkillCall(skill="pick", arm="A", target_object="fork", params={})
    if skill_name == "place_fork":
        return SkillCall(skill="place", arm="A", target_object="fork", params={"destination": "table"})
    if skill_name == "pick_bottle":
        return SkillCall(skill="pick", arm="A", target_object="bottle", params={})
    raise ValueError(f"unknown --skill {skill_name!r}")


_SKILL_TARGET_PROP = {
    "handoff": "fork",
    "pick_fork": "fork",
    "place_fork": "fork",
    "pick_bottle": "water_bottle",
}


def _prop_qpos_adr(model, body_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{body_name}_free")
    if jid == -1:
        raise RuntimeError(f"expected free joint '{body_name}_free' not found in compiled model")
    return int(model.jnt_qposadr[jid])


def _default_prop_xy(env: TableSettingEnv, body_name: str) -> tuple[float, float]:
    """Read the prop's own default resting (x, y) LIVE off a fresh
    `reset(seed=RESET_SEED)`, rather than hardcoding a duplicate of
    `gen_dual_scene.py`'s position constants -- same discipline
    `generate_posenet_data.py`'s "Correction 3" already used.
    """
    env.reset(seed=RESET_SEED, cameras=None)
    adr = _prop_qpos_adr(env.model, body_name)
    return float(env.data.qpos[adr]), float(env.data.qpos[adr + 1])


def _apply_jitter(env: TableSettingEnv, body_name: str, x: float, y: float) -> None:
    """Reset to the deterministic baseline, then overwrite ONE prop's
    free-joint x/y in place (z and quaternion left exactly as reset() set
    them), then `mj_forward` to recompute `data.xpos` etc. before any skill
    runs. Same mechanism as `scripts/generate_posenet_data.py`'s per-sample
    loop.
    """
    env.reset(seed=RESET_SEED, cameras=None)
    adr = _prop_qpos_adr(env.model, body_name)
    env.data.qpos[adr] = x
    env.data.qpos[adr + 1] = y
    mujoco.mj_forward(env.model, env.data)


def _body_xyz(env: TableSettingEnv, body_name: str) -> list[float]:
    bid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return [float(v) for v in env.data.xpos[bid]]


def _gripperframe_xyz(env: TableSettingEnv, arm: str) -> list[float]:
    sid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, f"arm{arm}_gripperframe")
    return [float(v) for v in env.data.site_xpos[sid]]


def _write_jsonl(path: pathlib.Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")
        f.flush()


# ---------------------------------------------------------------------------
# Subcommand: robustness (audit variables 1 & 2)
# ---------------------------------------------------------------------------
def cmd_robustness(args: argparse.Namespace) -> int:
    skill_name = args.skill
    body_name = _SKILL_TARGET_PROP[skill_name]
    seeds = range(args.seeds[0], args.seeds[1] + 1)
    out_path = pathlib.Path(args.out)

    probe_env = TableSettingEnv(cameras=None)
    default_x, default_y = _default_prop_xy(probe_env, body_name)
    probe_env.close()
    print(f"[{skill_name}] target prop={body_name!r} default (x,y)=({default_x:.4f},{default_y:.4f}) jitter=+/-{args.jitter}")

    n_success = 0
    n_total = 0
    for seed in seeds:
        # FRESH env + FRESH executor (-> fresh WeldGrasp) every trial --
        # required for correct numbers, see this module's docstring's
        # `active_welds` state-leak paragraph. Reusing either across a
        # `reset()` loop silently poisons every trial after the first
        # success.
        env = TableSettingEnv(cameras=None)
        executor = ScriptedSkillExecutor()

        rng = np.random.default_rng(seed)
        dx, dy = rng.uniform(-args.jitter, args.jitter, size=2)
        x, y = default_x + dx, default_y + dy
        _apply_jitter(env, body_name, x, y)

        t0 = time.time()
        result = executor.execute(_skill_call(skill_name), env, step_budget=ik.DEFAULT_STEP_BUDGET)
        wall_s = time.time() - t0

        record = {
            "skill": skill_name,
            "seed": int(seed),
            "prop": body_name,
            "prop_xy_used": [float(x), float(y)],
            "prop_xy_default": [default_x, default_y],
            "jitter_dxdy": [float(dx), float(dy)],
            "success": bool(result.success),
            "reason": result.reason,
            "frames_used": int(result.frames_used),
            "wall_s": wall_s,
        }
        if skill_name == "handoff":
            record["final_prop_xyz"] = _body_xyz(env, "fork")
            record["armA_gripperframe_xyz"] = _gripperframe_xyz(env, "A")
            record["armB_gripperframe_xyz"] = _gripperframe_xyz(env, "B")
            record["inter_arm_separation_m"] = float(
                np.linalg.norm(
                    np.array(record["armA_gripperframe_xyz"]) - np.array(record["armB_gripperframe_xyz"])
                )
            )

        n_total += 1
        n_success += int(result.success)
        _write_jsonl(out_path, record)
        print(
            f"  seed={seed} xy=({x:.4f},{y:.4f}) success={result.success} "
            f"frames={result.frames_used} wall={wall_s:.2f}s reason={result.reason}"
        )
        env.close()

    print(f"[{skill_name}] {n_success}/{n_total} succeeded. Appended to {out_path}")
    return 0


# ---------------------------------------------------------------------------
# Subcommand: grid (audit variable 6)
# ---------------------------------------------------------------------------
def cmd_grid(args: argparse.Namespace) -> int:
    skill_name = args.skill
    body_name = _SKILL_TARGET_PROP[skill_name]
    out_path = pathlib.Path(args.out)

    offsets = np.round(np.arange(-args.range, args.range + 1e-9, args.step), 6)
    offsets = [float(v) for v in offsets if abs(v) > 1e-9 or args.include_zero]

    probe_env = TableSettingEnv(cameras=None)
    default_x, default_y = _default_prop_xy(probe_env, body_name)
    probe_env.close()
    print(
        f"[{skill_name}] grid target prop={body_name!r} default=({default_x:.4f},{default_y:.4f}) "
        f"offsets={offsets} ({len(offsets)}x{len(offsets)}={len(offsets)**2} points)"
    )

    t_sweep_start = time.time()
    n_success = 0
    n_total = 0
    for dx in offsets:
        for dy in offsets:
            # FRESH env + FRESH executor every grid point -- see this
            # module's docstring's `active_welds` state-leak paragraph.
            env = TableSettingEnv(cameras=None)
            executor = ScriptedSkillExecutor()

            x, y = default_x + dx, default_y + dy
            _apply_jitter(env, body_name, x, y)

            t0 = time.time()
            result = executor.execute(_skill_call(skill_name), env, step_budget=ik.DEFAULT_STEP_BUDGET)
            wall_s = time.time() - t0

            record = {
                "skill": skill_name,
                "prop": body_name,
                "dx": dx,
                "dy": dy,
                "prop_xy_used": [float(x), float(y)],
                "success": bool(result.success),
                "reason": result.reason,
                "frames_used": int(result.frames_used),
                "wall_s": wall_s,
            }
            n_total += 1
            n_success += int(result.success)
            _write_jsonl(out_path, record)
            print(f"  dx={dx:+.3f} dy={dy:+.3f} success={result.success} frames={result.frames_used} wall={wall_s:.2f}s")
            env.close()

    sweep_wall_s = time.time() - t_sweep_start
    print(f"[{skill_name}] grid done: {n_success}/{n_total} succeeded. sweep_wall_s={sweep_wall_s:.1f}")
    _write_jsonl(out_path, {"skill": skill_name, "summary": True, "n_success": n_success, "n_total": n_total, "sweep_wall_s": sweep_wall_s})
    return 0


# ---------------------------------------------------------------------------
# Subcommand: posenet_acc (audit variable 3)
# ---------------------------------------------------------------------------
_POSENET_PROPS = ["fork", "water_bottle", "mug"]  # ADR-041's 3-prop scope


def _fresh_env_224() -> TableSettingEnv:
    return TableSettingEnv(cameras=None, render_width=224, render_height=224)


def _run_full(skill_call: SkillCall, jitter_xy: tuple[float, float] | None) -> int:
    """FRESH env + FRESH executor, run to completion (or step-budget
    exhaustion) at full budget, return frames_used. Used only to learn each
    execution's own total frame count so the 4 capture points can be
    expressed as fractions of it, per this module's docstring. A fresh
    executor per call is required, not optional -- see this module's
    docstring's `active_welds` state-leak paragraph.
    """
    env = _fresh_env_224()
    executor = ScriptedSkillExecutor()
    if jitter_xy is not None:
        _apply_jitter(env, _SKILL_TARGET_PROP["handoff"], *jitter_xy)
    else:
        env.reset(seed=RESET_SEED, cameras=None)
    result = executor.execute(skill_call, env, step_budget=ik.DEFAULT_STEP_BUDGET)
    env.close()
    return result.frames_used


def cmd_posenet_acc(args: argparse.Namespace) -> int:
    from bimanual.perception.inference import PoseNetInference

    out_path = pathlib.Path(args.out)

    probe_env = TableSettingEnv(cameras=None)
    default_fork_x, default_fork_y = _default_prop_xy(probe_env, "fork")
    probe_env.close()
    rng = np.random.default_rng(0)
    jdx, jdy = rng.uniform(-args.jitter, args.jitter, size=2)
    jittered_fork_xy = (default_fork_x + jdx, default_fork_y + jdy)

    # 5 skill executions, per this module's docstring. A second handoff at a
    # jittered fork start position is included for variety (not just the
    # deterministic default position 4 times over).
    executions = [
        ("pick_fork", _skill_call("pick_fork"), None),
        ("place_fork", _skill_call("place_fork"), None),
        ("pick_bottle", _skill_call("pick_bottle"), None),
        ("handoff_fork", _skill_call("handoff"), None),
        ("handoff_fork_jittered", _skill_call("handoff"), jittered_fork_xy),
    ]

    fractions = [0.02, 0.25, 0.50, 0.75]  # "start" is the earliest frame with
    # visible arm motion (frac=0 would be the pre-motion home pose, already
    # covered by ADR-044's own held-out set) -- see report for this choice.

    inference = PoseNetInference(device=args.device)
    print(f"PoseNetInference ready: device={inference.device} execution_devices={inference.execution_devices}")

    frame_idx = 0
    for exec_name, skill_call, jitter_xy in executions:
        total_frames = _run_full(skill_call, jitter_xy)
        print(f"[{exec_name}] total_frames={total_frames}")

        for frac in fractions:
            step_budget = max(1, round(frac * total_frames))
            # FRESH env + FRESH executor for every capture point too -- same
            # reason as `_run_full`, plus it lets each capture point be
            # driven from a clean start rather than accumulating state
            # across fractions within one execution.
            env = _fresh_env_224()
            executor = ScriptedSkillExecutor()
            if jitter_xy is not None:
                _apply_jitter(env, _SKILL_TARGET_PROP["handoff"], *jitter_xy)
            else:
                env.reset(seed=RESET_SEED, cameras=None)
            partial_result = executor.execute(skill_call, env, step_budget=step_budget)

            # Escape hatch: render regardless of the env's cameras= opt-in
            # setting (env.py's render() docstring).
            frame = env.render("posenet_cam")
            oracle_xyz = {p: _body_xyz(env, p) for p in _POSENET_PROPS}
            pred = inference.predict(frame)

            per_prop_error = {}
            for p in _POSENET_PROPS:
                oracle = np.array(oracle_xyz[p])
                predicted = pred[p]
                per_prop_error[p] = {
                    "oracle_xyz": [float(v) for v in oracle],
                    "predicted_xyz": [float(v) for v in predicted],
                    "abs_error_xy_m": float(np.linalg.norm(oracle[:2] - predicted[:2])),
                    "abs_error_xyz_m": float(np.linalg.norm(oracle - predicted)),
                }

            record = {
                "execution": exec_name,
                "frame_idx": frame_idx,
                "frac_of_total": frac,
                "step_budget_used": step_budget,
                "total_frames": total_frames,
                "partial_frames_used": int(partial_result.frames_used),
                "partial_success": bool(partial_result.success),
                "partial_reason": partial_result.reason,
                "per_prop_error": per_prop_error,
            }
            _write_jsonl(out_path, record)
            print(
                f"  frac={frac:.2f} step_budget={step_budget} "
                + " ".join(f"{p}={per_prop_error[p]['abs_error_xy_m']*1000:.1f}mm" for p in _POSENET_PROPS)
            )
            frame_idx += 1
            env.close()

    print(f"posenet_acc done: {frame_idx} frames appended to {out_path}")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    p_rob = sub.add_parser("robustness", help="audit variables 1 & 2")
    p_rob.add_argument("--skill", required=True, choices=list(_SKILL_TARGET_PROP))
    p_rob.add_argument("--seeds", type=int, nargs=2, default=[0, 9], metavar=("LO", "HI"))
    p_rob.add_argument("--jitter", type=float, default=0.05)
    p_rob.add_argument("--out", required=True)
    p_rob.set_defaults(func=cmd_robustness)

    p_grid = sub.add_parser("grid", help="audit variable 6")
    p_grid.add_argument("--skill", required=True, choices=list(_SKILL_TARGET_PROP))
    p_grid.add_argument("--step", type=float, default=0.02)
    p_grid.add_argument("--range", type=float, default=0.05)
    p_grid.add_argument("--include-zero", action="store_true")
    p_grid.add_argument("--out", required=True)
    p_grid.set_defaults(func=cmd_grid)

    p_pn = sub.add_parser("posenet_acc", help="audit variable 3")
    p_pn.add_argument("--out", required=True)
    p_pn.add_argument("--device", default="GPU")
    p_pn.add_argument("--jitter", type=float, default=0.05)
    p_pn.set_defaults(func=cmd_posenet_acc)

    return p


if __name__ == "__main__":
    args = build_arg_parser().parse_args()
    raise SystemExit(args.func(args))
