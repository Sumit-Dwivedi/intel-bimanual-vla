"""run_demo.py -- reproducible demo entry point for evaluators.

`./run_demo.sh` (repo root) execs this file; the wrapper exists only because
`README.md` and `CONTRIBUTING.md` already tell an evaluator to run
`./run_demo.sh`, and bm-ptl (this project's only MuJoCo-capable machine,
ADR-020) is Windows with no native bash to run a `.sh` file directly -- so
the real, portable logic lives HERE, in a plain Python script that runs
identically wherever `python`/`python3` runs, and `run_demo.sh` is a thin
`exec` wrapper around it. See `docs/SETUP.md` for full setup instructions
this docstring does not repeat.

What this script does, in order (mirrors this module's own task brief):
  1. Environment check -- python/mujoco/openvino/torch versions. mujoco is a
     HARD requirement (the four skills run inside a live MuJoCo sim) and a
     missing/broken install fails this script early with a message naming
     which requirements file fixes it. openvino and torch are reported but
     their absence is only a WARNING: oracle mode (ADR-046's default, and
     the only mode this script drives) never calls either. torch in
     particular lives only in the separate `train_env` venv
     (`scripts/requirements-train.txt`), never in `ov_env`
     (`scripts/requirements-bmptl.txt`) -- see that file's own docstring --
     so its absence here is the EXPECTED, documented state, not a defect.
  2. Asset check -- `scenes/so101/`, the packaged dual-arm scene XML, and
     the (gitignored, bm-ptl-only) PoseNet OpenVINO IR under
     `artifacts/posenet_ir/`. The scene assets are required (this script
     fails without them); the IR is not (ADR-046: oracle is the default,
     and this script never opts into perception) -- its absence is reported
     as informational, not fatal, since a fresh clone will never have it.
  3. Prints the planned four-skill sequence, honestly labelled per this
     module's task brief correction 1 (see `build_plan`'s docstring): three
     skills (`pick(A, fork)`, `place(A, fork, table)`, `pick(A, bottle)`)
     genuinely vary prop placement by `--seed`, drawn from that skill's own
     measured envelope (`SKILL_ENVELOPES`, ADR-048/ADR-049 Track A);
     `handoff` runs at a FIXED default layout and says so explicitly,
     because its own measured envelope is a single point (ADR-048) -- a
     `--seed` label on it would imply variation that is not happening.
  4. Executes the four skills in oracle mode (`ScriptedSkillExecutor
     (inference=None)`, ADR-046's default -- no OpenVINO/PoseNet call on
     this path at all). Each skill gets a FRESH `TableSettingEnv` + FRESH
     `ScriptedSkillExecutor` (this module's task brief correction 3, option
     1) -- the same pattern `scripts/verify_adr038_skills.py` and
     `scripts/eval_m08.py` already use, and for the identical reason
     (ADR-047): `WeldGrasp.active_welds` survives a bare `env.reset()`
     across a reused executor and silently poisons the next trial's grasp.
     Four independent skills, not a chained `TaskPlan`, is exactly this
     script's shape, so "fresh env+executor per skill" is the simpler of
     the two ADR-047-safe options and is what this script does throughout.
     `handoff` is run receiver-first, per this module's task brief
     correction 4: `run_handoff(env, "B", "A", "fork", weld=weld)` for an
     A->B handoff, reached here via
     `SkillCall(skill="handoff", arm="B", target_object="fork",
     params={"from_arm": "A"})` -- `ScriptedSkillExecutor._dispatch` turns
     that into `to_arm=skill_call.arm ("B"), from_arm=params["from_arm"]
     ("A")`, i.e. exactly the receiver-first call this module's task brief
     names. Getting `arm`/`from_arm` backwards here would silently run
     B->A instead and likely just fail differently -- this ordering is
     therefore deliberate, not incidental.
  5. Prints one result line per skill.
  6. Prints a summary (N/4 passed) with pointers to the benchmark and
     robustness docs.
  7. Exits 0 if all four skills passed, 1 otherwise (including on a hard
     environment/asset failure).

Usage:
    python run_demo.py
    python run_demo.py --seed 3
    ./run_demo.sh --seed 3
"""

from __future__ import annotations

import argparse
import platform
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent

# So `import bimanual...` works from a plain `python run_demo.py` at the
# repo root without requiring `pip install -e .` first -- the same pattern
# `scripts/run_grounded_demo.py` uses (`sys.path.insert(0, ".../src")`).
# Inserted at position 0 so a local checkout always wins over any installed
# copy of the same package.
sys.path.insert(0, str(REPO_ROOT / "src"))

DOC_BENCHMARK = "docs/hardware/m10-phase4-benchmark.md"
DOC_ROBUSTNESS = "docs/hardware/m08-eval.md"


# ---------------------------------------------------------------------------
# 1. Environment check
# ---------------------------------------------------------------------------
def check_environment() -> bool:
    """Print installed versions of python/mujoco/openvino/torch.

    Returns True if every HARD requirement is satisfied (mujoco only --
    see this module's docstring for why openvino/torch are soft). A caller
    should treat a False return as fatal and stop before touching MuJoCo.
    """
    print("=" * 72)
    print("1. ENVIRONMENT CHECK")
    print("=" * 72)
    print(f"  python   : {platform.python_version()} ({sys.executable})")
    print(f"  host     : {platform.node()} ({platform.system()} {platform.release()})")

    hard_ok = True

    # mujoco -- HARD requirement. All four scripted skills this demo runs
    # execute inside a live MuJoCo simulation; nothing else in this script
    # can proceed without it.
    try:
        import mujoco

        print(f"  mujoco   : {mujoco.__version__}  (OK -- required)")
    except Exception as exc:  # noqa: BLE001 -- deliberately broad, classified below
        print(f"  mujoco   : NOT AVAILABLE -- {type(exc).__name__}: {exc}")
        print()
        print(
            "FATAL: mujoco is required -- every skill this demo runs executes inside a "
            "live MuJoCo simulation, and none of the four can proceed without it."
        )
        print(
            "Fix: install scripts/requirements-bmptl.txt into the venv you are running "
            "this script with, e.g. on bm-ptl:"
        )
        print(
            r"  C:\Users\devcloud\project\ov_env\Scripts\python.exe -m pip install "
            "-r scripts/requirements-bmptl.txt"
        )
        print(
            "If this is the authoring laptop (Windows), mujoco is EXPECTED to fail to "
            "import there (ARCHITECTURE.md ADR-020 -- Windows Smart App Control blocks "
            "the unsigned mujoco.dll). This demo only ever runs on bm-ptl for that "
            "reason; see docs/SETUP.md."
        )
        hard_ok = False

    # openvino -- SOFT for this script. Oracle mode (the only mode this
    # script drives, ADR-046's default) never calls OpenVINO; its absence
    # only means the separate OpenVINO benchmark/quantization scripts are
    # unavailable in this venv, not that this demo cannot run.
    try:
        import openvino as ov

        print(f"  openvino : {ov.__version__}  (installed; not used by this demo's oracle-mode path)")
    except Exception as exc:  # noqa: BLE001
        print(f"  openvino : NOT AVAILABLE -- {type(exc).__name__}: {exc}")
        print(
            "      NOTE: not required by this demo (oracle mode, ADR-046 default, never "
            "calls OpenVINO). Required separately for scripts/quantize_posenet.py and the "
            "OpenVINO device benchmarks -- install scripts/requirements-bmptl.txt for those."
        )

    # torch -- lives ONLY in the separate train_env venv
    # (scripts/requirements-train.txt), never in ov_env -- see that file's
    # own docstring for why (ADR-037 already had one accidental transitive
    # upgrade from an unscoped install into ov_env; torch stays isolated).
    # Its absence in ov_env is therefore the EXPECTED, documented state and
    # must be a warning, never a failure.
    try:
        import torch

        print(f"  torch    : {torch.__version__}  (installed; this demo's four skills do not need it)")
    except Exception:
        print(
            "  torch    : NOT AVAILABLE -- expected in ov_env (this demo's venv). torch "
            "lives only in the separate train_env (scripts/requirements-train.txt); see "
            "that file's own docstring."
        )
        print(
            "      This demo's four skills need only mujoco (ov_env). OpenVINO PoseNet "
            "training/benchmarks that depend on torch are unavailable in this environment "
            "-- that is expected here, not a failure of this demo."
        )

    print()
    return hard_ok


# ---------------------------------------------------------------------------
# 2. Asset check
# ---------------------------------------------------------------------------
def check_assets() -> bool:
    """Print presence/absence of the scene assets and the (optional) IR.

    Returns True if every asset REQUIRED to run the demo is present. The
    PoseNet IR is checked and reported but never affects the return value:
    it is gitignored (~67 MB build artifact, see `.gitignore`) and exists
    only on bm-ptl where it was generated -- a fresh clone will never have
    it, and ADR-046 makes oracle (no perception) the default this script
    always uses, so its absence does not block anything this script does.
    """
    print("=" * 72)
    print("2. ASSET CHECK")
    print("=" * 72)
    ok = True

    so101_dir = REPO_ROOT / "scenes" / "so101"
    if so101_dir.is_dir() and any(so101_dir.iterdir()):
        n = sum(1 for _ in so101_dir.iterdir())
        print(f"  scenes/so101/                                    : present ({n} entries)")
    else:
        print("  scenes/so101/                                    : MISSING")
        ok = False

    scene_xml = REPO_ROOT / "src" / "bimanual" / "sim" / "assets" / "so101_dual_table.xml"
    rel = scene_xml.relative_to(REPO_ROOT)
    if scene_xml.exists():
        print(f"  {rel} : present")
    else:
        print(f"  {rel} : MISSING")
        ok = False

    ir_dir = REPO_ROOT / "artifacts" / "posenet_ir"
    ir_files = sorted(p.name for p in ir_dir.glob("*.xml")) if ir_dir.is_dir() else []
    if ir_files:
        print(f"  artifacts/posenet_ir/                            : present ({', '.join(ir_files)})")
    else:
        print("  artifacts/posenet_ir/                            : not present -- perception")
        print(
            "      benchmarks unavailable, demo runs in oracle mode (ADR-046: oracle is "
            "the default). This directory is gitignored and exists only on bm-ptl, where "
            "it was generated (scripts/posenet_to_openvino.py); a fresh clone never has "
            "it, and this script does not need it."
        )

    print()
    if not ok:
        print("FATAL: required scene assets are missing -- cannot construct the simulation.")
        print()
    return ok


# ---------------------------------------------------------------------------
# 3/4/5. Planned sequence + execution (oracle mode)
# ---------------------------------------------------------------------------
@dataclass
class DemoStep:
    """One of the four skills this demo runs.

    `randomizer` is `None` for a step that intentionally runs at a FIXED
    default layout (currently only `handoff` -- see `build_plan`'s
    docstring). `seed_note` is the ONE sentence printed for this step
    before it runs, and it must never claim variation that is not
    happening (this module's task brief correction 1).
    """

    label: str
    skill_call: Any  # bimanual.language.skills.SkillCall, imported lazily -- see run_demo()
    randomizer: Any  # bimanual.sim.randomization.ScenarioRandomizer | None
    target_prop: str  # MuJoCo body name to read z from (e.g. "fork", "water_bottle")
    z_label: str  # human label for the printed result line, e.g. "fork z"
    seed_note: str


def build_plan(seed: int, ScenarioRandomizer, SKILL_ENVELOPES, SkillCall) -> list[DemoStep]:
    """Build the four-step demo plan, honestly labelled per correction 1.

    Three skills get a per-seed `ScenarioRandomizer` built from
    `SKILL_ENVELOPES[skill][target]` -- the INDIVIDUAL, non-intersected
    envelope M08 Track A used (`scripts/eval_m08.py`'s own
    `_track_a_randomizer`, same construction) -- not `randomization.py`'s
    module-level `ENVELOPES` export, which is intentionally empty (ADR-048:
    every per-prop envelope collapses to a single point once `handoff`'s
    cross-prop fragility is folded in). Passing `SKILL_ENVELOPES` directly,
    scoped to one skill's own target, is exactly what `ScenarioRandomizer`'s
    `envelopes=` constructor argument exists for (see `randomization.py`'s
    own docstring) and is additive, not a change to the shipped default.

    `pick_fork`, `place_fork` and `pick_bottle` each have a real 1-D range
    (ADR-048's measured rectangles), so `--seed` genuinely moves their
    target prop and the printed offset is real. `handoff`'s own measured
    envelope (`SKILL_ENVELOPES["handoff"]["fork"]`) is `(0, 0, 0, 0)` -- a
    single point -- so it is run WITHOUT a randomizer, at the fixed default
    layout, and labelled as such rather than printed with a seed that would
    select nothing.
    """
    pick_fork_rect = SKILL_ENVELOPES["pick_fork"]["fork"]
    place_fork_rect = SKILL_ENVELOPES["place_fork"]["fork"]
    pick_bottle_rect = SKILL_ENVELOPES["pick_bottle"]["water_bottle"]

    return [
        DemoStep(
            label="pick(A, fork)",
            skill_call=SkillCall(skill="pick", arm="A", target_object="fork", params={}),
            randomizer=ScenarioRandomizer(envelopes={"fork": pick_fork_rect}),
            target_prop="fork",
            z_label="fork z",
            seed_note=(
                f"seed={seed} varies fork's own (x, y) placement within its measured "
                f"envelope dx/dy in {pick_fork_rect} m (ADR-048; ADR-049 Track A method)."
            ),
        ),
        DemoStep(
            label="place(A, fork, table)",
            skill_call=SkillCall(
                skill="place", arm="A", target_object="fork", params={"destination": "table"}
            ),
            randomizer=ScenarioRandomizer(envelopes={"fork": place_fork_rect}),
            target_prop="fork",
            z_label="fork z",
            seed_note=(
                f"seed={seed} varies fork's own (x, y) placement within its measured "
                f"envelope dx/dy in {place_fork_rect} m (ADR-048; ADR-049 Track A method)."
            ),
        ),
        DemoStep(
            label="pick(A, bottle)",
            skill_call=SkillCall(skill="pick", arm="A", target_object="bottle", params={}),
            randomizer=ScenarioRandomizer(envelopes={"water_bottle": pick_bottle_rect}),
            target_prop="water_bottle",
            z_label="bottle z",
            seed_note=(
                f"seed={seed} varies water_bottle's own (x, y) placement within its "
                f"measured envelope dx/dy in {pick_bottle_rect} m (ADR-048; ADR-049 Track A method)."
            ),
        ),
        DemoStep(
            label="handoff(A->B, fork)",
            skill_call=SkillCall(
                skill="handoff", arm="B", target_object="fork", params={"from_arm": "A"}
            ),
            randomizer=None,
            target_prop="fork",
            z_label="fork z",
            seed_note=(
                "fixed default layout (randomization envelopes are degenerate -- see "
                "ADR-048): handoff's own measured envelope is a single point, "
                f"(0, 0, 0, 0), so --seed={seed} is NOT applied here and no seed label is "
                "printed for this step's placement -- printing one would imply variation "
                "that is not happening."
            ),
        ),
    ]


def print_plan(steps: list[DemoStep]) -> None:
    print("=" * 72)
    print("3. PLANNED SEQUENCE (oracle mode, ADR-046 default; fresh env+executor per skill, ADR-047)")
    print("=" * 72)
    for i, step in enumerate(steps, start=1):
        print(f"  [{i}] {step.label}")
        print(f"      {step.seed_note}")
    print()


def run_step(step: DemoStep, seed: int, TableSettingEnv, ScriptedSkillExecutor, sk) -> dict:
    """Run one skill against a FRESH env + FRESH executor (ADR-047 --
    reusing either across a reset()-based loop can silently corrupt
    `WeldGrasp.active_welds`; see this module's docstring's step 4).
    """
    import numpy as np

    def body_z(env, name: str) -> float:
        return float(env.data.xpos[sk._body_id(env.model, name)][2])

    def pinch_point(env, arm: str):
        fixed = env.data.xpos[sk._body_id(env.model, f"arm{arm}_gripper")]
        moving = env.data.xpos[sk._body_id(env.model, f"arm{arm}_moving_jaw_so101_v1")]
        return 0.5 * (np.asarray(fixed) + np.asarray(moving))

    env = TableSettingEnv(cameras=None)
    executor = ScriptedSkillExecutor()  # inference=None -- oracle mode, ADR-046 default

    if step.randomizer is not None:
        env.reset(seed=seed, randomizer=step.randomizer)
        offsets_used = step.randomizer.randomize(seed)
    else:
        # No randomizer passed -> byte-identical fixed default scene
        # regardless of `seed` (ADR-048's own documented reset() contract).
        env.reset(seed=seed)
        offsets_used = {}

    z_before = body_z(env, step.target_prop)
    result = executor.execute(step.skill_call, env)
    z_after = body_z(env, step.target_prop)

    extra: dict[str, float] = {}
    if step.skill_call.skill == "handoff":
        pa = pinch_point(env, "A")
        pb = pinch_point(env, "B")
        extra["lateral_separation_m"] = float(abs(pa[1] - pb[1]))

    env.close()

    return {
        "label": step.label,
        "z_label": step.z_label,
        "success": bool(result.success),
        "reason": result.reason,
        "frames_used": int(result.frames_used),
        "z_before": z_before,
        "z_after": z_after,
        "offsets_used": offsets_used,
        "extra": extra,
    }


def print_result(r: dict) -> None:
    status = "PASS" if r["success"] else "FAIL"
    line = f"[{status}] {r['label']}: frames={r['frames_used']}, {r['z_label']}={r['z_after']:.4f}"
    if "lateral_separation_m" in r["extra"]:
        line += f", lateral_separation={r['extra']['lateral_separation_m']:.4f} m"
    if not r["success"]:
        line += f" -- reason: {r['reason']}"
    print(line)


def print_summary(results: list[dict]) -> int:
    n_pass = sum(1 for r in results if r["success"])
    print()
    print("=" * 72)
    print("6. SUMMARY")
    print("=" * 72)
    print(f"  Skills completed: {n_pass}/4")
    for r in results:
        print(f"    [{'PASS' if r['success'] else 'FAIL'}] {r['label']}")
    print()
    print(f"  OpenVINO device/precision benchmark (CPU/iGPU/NPU, FP32/FP16/INT8): {DOC_BENCHMARK}")
    print(f"  10-seed robustness evaluation (Track A own-prop / Track B multi-prop): {DOC_ROBUSTNESS}")
    print()
    return n_pass


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--seed",
        type=int,
        default=3,
        help=(
            "Seed for the three skills with a real, measured placement envelope "
            "(pick_fork, place_fork, pick_bottle -- ADR-048/ADR-049 Track A). handoff "
            "always runs at its fixed default layout regardless of this flag -- its own "
            "measured envelope is a single point (ADR-048). Default: 3 -- chosen because "
            "it is one of the 6/10 seeds where ADR-049 Track A's own measurement found "
            "pick(A, bottle) passes (seeds 0, 1, 2, 4 are documented failures for that "
            "skill alone, see docs/hardware/m08-eval.md); this is a demo default choice, "
            "not a claim that every seed passes -- pass --seed 0 to see the documented "
            "pick(A, bottle) failure mode directly."
        ),
    )
    parser.add_argument(
        "--skip-env-check",
        action="store_true",
        help="Skip step 1 (environment check). Not recommended -- present for debugging only.",
    )
    args = parser.parse_args(argv)

    if not args.skip_env_check:
        if not check_environment():
            return 1
    else:
        print("Skipping environment check (--skip-env-check).\n")

    if not check_assets():
        return 1

    # Every import below touches mujoco (directly or transitively) and is
    # deferred to this point deliberately: check_environment() above is
    # what turns a missing/broken mujoco install into this script's own
    # clear, actionable message, not a raw ImportError traceback here.
    from bimanual.control import skills_scripted as sk
    from bimanual.control.executor import ScriptedSkillExecutor
    from bimanual.language.skills import SkillCall
    from bimanual.sim.env import TableSettingEnv
    from bimanual.sim.randomization import SKILL_ENVELOPES, ScenarioRandomizer

    steps = build_plan(args.seed, ScenarioRandomizer, SKILL_ENVELOPES, SkillCall)
    print_plan(steps)

    print("=" * 72)
    print("4/5. EXECUTION (oracle mode) + PER-SKILL RESULTS")
    print("=" * 72)
    results = []
    for step in steps:
        r = run_step(step, args.seed, TableSettingEnv, ScriptedSkillExecutor, sk)
        print_result(r)
        results.append(r)

    n_pass = print_summary(results)
    return 0 if n_pass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
