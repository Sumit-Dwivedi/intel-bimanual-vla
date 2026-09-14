"""M10 FIX A -- DIAGNOSTIC ONLY, no skill-logic change (ADR-051 candidate).

**Question.** M08 (ADR-049) reported `handoff(A->B, fork)` Track A 10/10, but
Track A's own envelope for `handoff` is a single point (`(0,0,0,0)`) -- every
seed drew the SAME zero prop offset, so that number measures determinism,
not robustness. This script asks a different question: does `run_handoff`
tolerate a small perturbation that is NOT prop placement -- specifically,
noise on each arm's HOME pose (`shoulder_lift`, `elbow_flex`) immediately
after `env.reset()`, before the skill ever starts moving?

**What this is not.** A physics-timestep sweep was also proposed for this
task and is deliberately NOT run here. No `timestep` is set anywhere in the
scene XML or `gen_dual_scene.py`, so the model runs at MuJoCo's default
(0.002 s), and every skill's step budget in this repo
(`ik.DEFAULT_STEP_BUDGET` et al.) is a FRAME count tuned at that default.
Changing `dt` without rescaling the frame budget does not perturb the
skill -- it changes how much simulated time the same frame budget buys, plus
integrator accuracy and contact resolution, and would produce a result that
looks like a robustness measurement but actually just says "this skill is
frame-budget-tuned for dt=0.002" (already known). Skipped for that reason,
not for lack of time.

**Method.** For each of 5 seeds (0-4) at a given noise magnitude:
  1. Fresh `TableSettingEnv(cameras=None)`, `env.reset(seed=seed, cameras=None)`.
  2. Draw one independent U(-magnitude, +magnitude) sample per joint for
     `armA_shoulder_lift`, `armA_elbow_flex`, `armB_shoulder_lift`,
     `armB_elbow_flex` from a seed+magnitude-keyed RNG (reproducible), add
     directly to `env.data.qpos` at each joint's own `jnt_qposadr` slot (a
     1-DOF hinge joint, unlike a prop's 7-wide free joint -- see
     `env.py`'s `_prop_free_joint_qpos_adr` for the analogous, wider case
     this mirrors for a single-DOF joint instead).
  3. `mujoco.mj_forward(env.model, env.data)` -- re-derive every dependent
     site/body xpos the skill's targeting and Phase-5 gate both read, the
     same pattern `env.py`'s own `reset()` randomizer branch already uses
     after perturbing a prop's qpos, applied here to joint qpos instead.
  4. A FRESH `WeldGrasp(env)` per trial (never reused across seeds/trials --
     ADR-047's fix exists precisely because reusing one across a `reset()`
     loop silently corrupts `active_welds`). This mirrors
     `scripts/verify_adr038_skills.py`'s own `fresh()` helper
     (`WeldGrasp(env)` constructed directly, not via
     `ScriptedSkillExecutor`), which is this repo's established pattern for
     a bare, direct `run_handoff` call outside the `SkillCall`/executor
     dispatch path.
  5. Call receiver-first, exactly as `verify_adr038_skills.py`'s own comment
     states it and PLAN.md's task brief requires:
     `run_handoff(env, "B", "A", "fork", weld=weld)`.

Widens the magnitude (0.005 -> 0.01 -> 0.02 rad) only if the previous,
smaller magnitude passed all 5 seeds -- finding where it breaks is the goal,
not a pass count at one level.

Modifies NOTHING in `skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`,
`env.py`, `randomization.py`, `scenes/so101/`, or `gen_dual_scene.py` --
reads `SkillResult.reason` (which already embeds
`from_arm_retreat_dist=X.XXXX` at every point past Phase 5a, success or
failure, per `skills_scripted.py`'s own f-strings) rather than adding any new
instrumentation to that module.

Usage
-----
  python scripts/probe_handoff_perturbation.py --out out/m10_handoff_perturbation.jsonl
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import time

import mujoco
import numpy as np

from bimanual.control import skills_scripted as skills
from bimanual.sim.env import TableSettingEnv
from bimanual.sim.grasp import WeldGrasp

#: Widening ladder -- run in order, stop at the first magnitude that does not
#: pass all 5 seeds. Overridable via --magnitudes (ADR-052 extension: a
#: finer, single-magnitude condition needed a way to run 0.003 rad alone
#: without disturbing this default ladder for any other caller).
NOISE_LEVELS_RAD = (0.005, 0.01, 0.02)
SEEDS = (0, 1, 2, 3, 4)
#: Default joint suffixes noised together (ADR-051's original method).
#: Overridable via --joints (ADR-052 extension: isolating shoulder_lift or
#: elbow_flex alone was a genuinely new question, not covered by the
#: combined default -- added as a parameter rather than a second script,
#: per this task's "reuse it; do not rewrite it" instruction).
NOISY_JOINT_SUFFIXES = ("shoulder_lift", "elbow_flex")

#: `skills_scripted.run_handoff`'s own f-strings embed this token at every
#: return point past Phase 5a (see that function's Phase 5/5b blocks) --
#: parsed out here rather than duplicating the measurement.
_RETREAT_RE = re.compile(r"from_arm_retreat_dist=([0-9.]+)")


def _noisy_joint_qpos_addrs(model, joint_suffixes=NOISY_JOINT_SUFFIXES) -> list[tuple[str, int]]:
    """Resolve the (armA/armB x joint_suffixes) qpos addresses.

    Each is a single-DOF hinge joint, so `jnt_qposadr` gives that joint's one
    and only `qpos` slot directly.
    """
    addrs = []
    for arm in ("A", "B"):
        for suffix in joint_suffixes:
            name = f"arm{arm}_{suffix}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid == -1:
                raise KeyError(f"no joint named {name!r} in the compiled model")
            addrs.append((name, int(model.jnt_qposadr[jid])))
    return addrs


def _write_jsonl(path: pathlib.Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")
        f.flush()


def _run_trial(
    seed: int,
    magnitude_rad: float,
    out_path: pathlib.Path,
    levels: tuple = NOISE_LEVELS_RAD,
    joint_suffixes: tuple = NOISY_JOINT_SUFFIXES,
) -> dict:
    # Deterministic, distinct stream per (magnitude, seed) so a re-run
    # reproduces the identical noise vector -- keyed so no two (magnitude,
    # seed) pairs in the whole grid ever collide. `levels` defaults to the
    # original ladder so the default CLI invocation reproduces ADR-051's
    # exact seeding bit-for-bit; a caller passing a custom --magnitudes list
    # (ADR-052) gets its own, still-reproducible, still-collision-free
    # index space.
    level_idx = levels.index(magnitude_rad)
    rng = np.random.default_rng(seed=1000 * level_idx + seed)

    env = TableSettingEnv(cameras=None)
    env.reset(seed=seed, cameras=None)

    addrs = _noisy_joint_qpos_addrs(env.model, joint_suffixes)
    noise = rng.uniform(-magnitude_rad, magnitude_rad, size=len(addrs))
    applied = {}
    for (name, adr), delta in zip(addrs, noise):
        env.data.qpos[adr] += float(delta)
        applied[name] = float(delta)
    mujoco.mj_forward(env.model, env.data)

    # Fresh WeldGrasp per trial (ADR-047), constructed directly -- same
    # pattern as scripts/verify_adr038_skills.py's `fresh()` helper, not via
    # ScriptedSkillExecutor (this is a bare run_handoff call, outside the
    # SkillCall/executor dispatch path).
    weld = WeldGrasp(env)

    t0 = time.time()
    result = skills.run_handoff(env, "B", "A", "fork", weld=weld)
    wall_s = time.time() - t0

    m = _RETREAT_RE.search(result.reason)
    from_arm_retreat_dist = float(m.group(1)) if m else None

    record = {
        "magnitude_rad": magnitude_rad,
        "seed": seed,
        "noise_applied_rad": applied,
        "success": bool(result.success),
        "reason": result.reason,
        "frames_used": int(result.frames_used),
        "from_arm_retreat_dist": from_arm_retreat_dist,
        "wall_s": wall_s,
    }
    env.close()
    _write_jsonl(out_path, record)
    print(
        f"  magnitude={magnitude_rad} seed={seed} noise={applied} success={result.success} "
        f"retreat_dist={from_arm_retreat_dist} frames={result.frames_used} wall={wall_s:.2f}s\n"
        f"    reason={result.reason}"
    )
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="out/m10_handoff_perturbation.jsonl")
    # ADR-052 extension (both default to ADR-051's exact original values, so
    # the bare `python scripts/probe_handoff_perturbation.py` invocation is
    # byte-for-bit unchanged from last night's run):
    parser.add_argument(
        "--magnitudes",
        default=",".join(str(m) for m in NOISE_LEVELS_RAD),
        help="comma-separated widening ladder in rad, e.g. '0.003' for a single fixed magnitude",
    )
    parser.add_argument(
        "--joints",
        default=",".join(NOISY_JOINT_SUFFIXES),
        help="comma-separated joint suffixes to noise on both arms, e.g. 'shoulder_lift'",
    )
    args = parser.parse_args()
    out_path = pathlib.Path(args.out)
    levels = tuple(float(x) for x in args.magnitudes.split(","))
    joint_suffixes = tuple(x.strip() for x in args.joints.split(","))

    all_records: list[dict] = []
    levels_run: list[float] = []
    for magnitude in levels:
        print(f"=== magnitude={magnitude} rad joints={joint_suffixes} (seeds {SEEDS}) ===")
        levels_run.append(magnitude)
        level_records = [
            _run_trial(seed, magnitude, out_path, levels=levels, joint_suffixes=joint_suffixes)
            for seed in SEEDS
        ]
        all_records.extend(level_records)
        n_pass = sum(r["success"] for r in level_records)
        print(f"[magnitude={magnitude}] {n_pass}/{len(SEEDS)}")
        if n_pass < len(SEEDS):
            print(f"magnitude={magnitude} did not pass all {len(SEEDS)} seeds -- stopping the widening ladder here.")
            break

    n_success = sum(r["success"] for r in all_records)
    summary = {
        "eval_summary": True,
        "n_success": n_success,
        "n_total": len(all_records),
        "levels_run_rad": levels_run,
        "joints": list(joint_suffixes),
    }
    _write_jsonl(out_path, summary)
    print(f"\nTOTAL {n_success}/{len(all_records)}")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
