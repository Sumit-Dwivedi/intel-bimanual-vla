"""M08: formal 10-seed robustness evaluation of the four ADR-038-gated
skills (ADR-049).

**Two tracks, reported separately -- see `docs/hardware/m08-eval.md` for the
full write-up and why they cannot be collapsed into one table.**

**Track A -- own-prop randomization.** Only the skill's own target prop is
displaced, drawn from that skill's OWN measured envelope
(`randomization.py`'s `SKILL_ENVELOPES[skill][target_prop]` -- the per-skill
rectangle, NOT the cross-skill-intersected, module-level `ENVELOPES` export,
which is empty by design, see ADR-048). Every other prop stays at its
deterministic default. This is the method this task's brief specifies.
`handoff`'s own envelope is a single point (`(0,0,0,0)`) -- every trial
therefore draws the SAME (0, 0) offset, so this track measures determinism
for `handoff`, not robustness. `pick_fork`, `place_fork` and `pick_bottle`
each have a genuine 1-D range, so their Track A numbers carry real
variation.

**Track B -- multi-prop randomization (Round 1's method,
`docs/hardware/m07-envelopes.md`'s Task 3 Round 1).** The SAME randomizer
config runs underneath EVERY skill's 10 seeds, regardless of what that
skill targets: `water_bottle` alone, at its own individually-measured
envelope `(0, 0, -0.010, +0.010)` -- `fork` was already excluded from this
config (its cross-skill intersection collapses to a point independent of
the water_bottle/handoff finding, see `docs/hardware/m07-envelopes.md`'s
Task 2). This is what surfaces `handoff`'s cross-prop coupling: `handoff`
never touches `water_bottle`, yet fails when it moves, through physics
coupling documented in ADR-038 and re-confirmed at 0.1 mm resolution in
`m07-envelopes.md`.

Both tracks: seeds 0-9, oracle mode (no perception, `ScriptedSkillExecutor
(inference=None)`, the default), a FRESH `TableSettingEnv` + FRESH
`ScriptedSkillExecutor` per trial (ADR-047: reusing either across a
`reset()`-based loop silently corrupts `WeldGrasp.active_welds` -- the exact
bug that ADR-047 fixed and the exact reason `probe_envelope.py`'s own
`randomized_eval` and `sweep` never reuse either across trials). Results are
written one JSON line at a time, flushed immediately, so a mid-run failure
loses at most the one in-flight trial (per this task's own "write results
incrementally" instruction).

This script never modifies `skills_scripted.py`, `grasp.py`, `ik.py`,
`executor.py`, `env.py`, `randomization.py`, `scenes/so101/`, or
`gen_dual_scene.py` -- it only reads `SKILL_ENVELOPES` from
`randomization.py` and constructs `ScenarioRandomizer` instances with
explicit, script-local `envelopes=` dicts (a supported, documented
constructor argument -- see `randomization.py`'s `ScenarioRandomizer.__init__`).
`randomization.py`'s own module-level `ENVELOPES` stays empty and untouched.

Usage
-----
  python scripts/eval_m08.py --track A --skill pick_fork --out out/m08/trackA_pick_fork.jsonl
  python scripts/eval_m08.py --track B --skill handoff --out out/m08/trackB_handoff.jsonl
  python scripts/eval_m08.py --track both --skill all --out-dir out/m08
"""

from __future__ import annotations

import argparse
import json
import pathlib
import time

from bimanual.control import ik
from bimanual.control.executor import ScriptedSkillExecutor
from bimanual.language.skills import SkillCall
from bimanual.sim.env import TableSettingEnv
from bimanual.sim.randomization import ScenarioRandomizer, SKILL_ENVELOPES

ALL_SKILLS = ("pick_fork", "place_fork", "handoff", "pick_bottle")

# Track B's fixed config, cited verbatim from docs/hardware/m07-envelopes.md's
# Task 3 "Round 1" (`ENVELOPES = {"water_bottle": (0,0,-0.010,+0.010)}`,
# fork already excluded independent of the water_bottle/handoff finding).
# Applied identically underneath every skill's 10-seed run -- this is the
# entire point of Track B: props the skill does not manipulate still move.
TRACK_B_ENVELOPES: dict[str, tuple[float, float, float, float]] = {
    "water_bottle": (0.000, 0.000, -0.010, 0.010),
}

_TARGET_PROP = {
    "pick_fork": "fork",
    "place_fork": "fork",
    "handoff": "fork",
    "pick_bottle": "water_bottle",
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
    raise ValueError(f"unknown skill {skill_name!r}")


def _track_a_randomizer(skill_name: str) -> ScenarioRandomizer:
    """Own-prop-only randomizer: exactly this skill's own SKILL_ENVELOPES
    entry for its own target prop, nothing else. Constructed directly from
    `SKILL_ENVELOPES` (not the module's cross-skill-intersected, degenerate-
    excluding `ENVELOPES`) so a degenerate (single-point) rectangle, e.g.
    `handoff`'s `(0,0,0,0)`, is used AS-IS -- deliberately not filtered out,
    because the whole point of Track A's `handoff` row is to show the
    degenerate range, not to hide it.
    """
    target = _TARGET_PROP[skill_name]
    rect = SKILL_ENVELOPES[skill_name][target]
    return ScenarioRandomizer(envelopes={target: rect})


def _track_b_randomizer() -> ScenarioRandomizer:
    return ScenarioRandomizer(envelopes=dict(TRACK_B_ENVELOPES))


def _write_jsonl(path: pathlib.Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")
        f.flush()


def run_track(track: str, skill_name: str, out_path: pathlib.Path) -> dict:
    """Run one (track, skill) combination across seeds 0-9. Fresh env +
    fresh executor per trial (ADR-047). Returns the summary dict, which is
    also appended to `out_path` as the final line.
    """
    randomizer = _track_a_randomizer(skill_name) if track == "A" else _track_b_randomizer()
    print(f"=== track {track} skill={skill_name} envelopes={randomizer.envelopes} ===")

    n_success = 0
    frames_on_success: list[int] = []
    failure_reasons: dict[str, int] = {}
    per_seed_rows = []

    for seed in range(10):
        env = TableSettingEnv(cameras=None)
        executor = ScriptedSkillExecutor()
        env.reset(seed=seed, cameras=None, randomizer=randomizer)
        # Pure function of seed -- reset() above already applied these
        # offsets; calling it again here is for RECORD-KEEPING only (same
        # pattern as probe_envelope.py's randomized_eval).
        offsets_used = randomizer.randomize(seed)

        t0 = time.time()
        result = executor.execute(_skill_call(skill_name), env, step_budget=ik.DEFAULT_STEP_BUDGET)
        wall_s = time.time() - t0

        record = {
            "track": track,
            "skill": skill_name,
            "seed": seed,
            "offsets_used": {k: list(v) for k, v in offsets_used.items()},
            "success": bool(result.success),
            "reason": result.reason,
            "frames_used": int(result.frames_used),
            "wall_s": wall_s,
        }
        n_success += int(result.success)
        if result.success:
            frames_on_success.append(result.frames_used)
        else:
            failure_reasons[result.reason] = failure_reasons.get(result.reason, 0) + 1
        per_seed_rows.append(record)
        _write_jsonl(out_path, record)
        print(
            f"  seed={seed} offsets={offsets_used} success={result.success} "
            f"frames={result.frames_used} wall={wall_s:.2f}s reason={result.reason}"
        )
        env.close()

    mean_frames = (sum(frames_on_success) / len(frames_on_success)) if frames_on_success else None
    summary = {
        "eval_summary": True,
        "track": track,
        "skill": skill_name,
        "n_success": n_success,
        "n_total": 10,
        "mean_frames_on_success": mean_frames,
        "failure_reasons": failure_reasons,
        "envelope_used": {k: list(v) for k, v in randomizer.envelopes.items()},
    }
    _write_jsonl(out_path, summary)
    print(
        f"[track {track}][{skill_name}] {n_success}/10, "
        f"mean_frames_on_success={mean_frames}, failures={failure_reasons}"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", choices=["A", "B", "both"], required=True)
    parser.add_argument("--skill", choices=list(ALL_SKILLS) + ["all"], required=True)
    parser.add_argument("--out-dir", default="out/m08_eval", help="Directory for per-run JSONL files")
    args = parser.parse_args()

    tracks = ["A", "B"] if args.track == "both" else [args.track]
    skills = list(ALL_SKILLS) if args.skill == "all" else [args.skill]
    out_dir = pathlib.Path(args.out_dir)

    all_summaries = []
    for track in tracks:
        for skill_name in skills:
            out_path = out_dir / f"track{track}_{skill_name}.jsonl"
            summary = run_track(track, skill_name, out_path)
            all_summaries.append(summary)

    # Combined summary file, written last (all per-(track,skill) files are
    # already durable on disk by this point, this is a convenience index).
    combined_path = out_dir / "summary_all.json"
    combined_path.parent.mkdir(parents=True, exist_ok=True)
    with open(combined_path, "w", encoding="utf-8") as f:
        json.dump(all_summaries, f, indent=2)
    print(f"\nWrote combined summary: {combined_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
