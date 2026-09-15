"""Redesign Stage 4, Step 2: run the validated swept-path gate
(`check_swept_path.swept_path_clear`) against every waypoint of every one of
the eight skills' APPROACH sequences, from the pose that precedes it, and
classify each collision found.

**How the real waypoint sequence is captured, without hand-deriving
formulas a second time.** `skills_scripted.py` (frozen this stage) already
builds each skill's exact waypoint targets inside `run_pick`/`run_place`/
`run_handoff` -- re-deriving them by hand here would risk silently
diverging from what those functions actually do (e.g. `run_handoff`'s
target-interpolation staging, ADR-035). Instead, this script MONKEYPATCHES
the module-level `skills_scripted._run_waypoint` symbol (never edits the
file: `git diff` against it stays empty, verified at the end of this
script) with a wrapper that:
  1. Records `(skill_label, waypoint_index, arm, start_qpos, target_pos,
     target_object)` -- `start_qpos` is `env.data.qpos` at the EXACT instant
     this waypoint begins, i.e. "the preceding pose" the task asks for.
  2. Solves IK once (the SAME call the real controller's first physics step
     of this waypoint would make) and TELEPORTS that arm's 5 joints directly
     to the solution (skipping the up to 500-step physics loop -- this
     script only needs the CHAIN of waypoint start/target poses, not a
     physically accurate trajectory; `swept_path_clear` re-solves its own
     IK from each captured `start_qpos` anyway, so a physically-realized
     endpoint is not required here).
  3. ALSO writes that arm's actuator `ctrl` entries to match, and zeroes
     `qvel`, so any REAL `env.step()` calls elsewhere in the skill (GRIP/
     RELEASE dwells, which this script does NOT patch -- they hold position,
     not sweep it) do not fight a stale ctrl target and yank the arm back.
  4. ALWAYS reports `(ok=True, ...)` regardless of the real controller's
     collision/convergence bars, so `run_pick`/`run_place`/`run_handoff`
     advance to their NEXT waypoint unconditionally -- this is what makes
     it possible to capture a currently-FAILING skill's entire intended
     approach sequence, not just the prefix up to its first failure.

`weld=None` throughout (no `WeldGrasp`): this script only needs the
waypoint CHAIN, and the `already_held`/weld-gated branches in `run_place`/
`run_handoff` are simplest and most deterministic with weld disabled
(confirmed by reading those functions: `weld is None` skips every
weld-conditional branch and always takes the "pick/grip/release for real"
path, which is what produces a full waypoint chain).

Each captured waypoint is then run through `swept_path_clear` at the
DEFAULT settings validated in Step 1 (`num_endpoint_seeds=8`), and every
collision is classified into one of the four categories the task names:
  (a) arm-vs-prop   -- a prop geom (not the skill's own current target,
                        outside the finger-pad exemption) is hit.
  (b) arm-vs-table  -- `table_top` is hit.
  (c) arm-vs-arm    -- the OTHER arm's geoms are hit (cross-arm).
  (d) self-collision -- two geoms of the SAME arm being swept are hit.
A single waypoint's violation list can span more than one category; the
per-skill/per-waypoint table records every category present, and the
overall "dominant category" tally (Step 2's deliverable) counts a category
once per COLLIDING WAYPOINT it appears in (not once per raw contact row),
so one badly-clipped waypoint cannot out-vote several separately-diagnosed
ones.

Run on bm-ptl:
    python scripts/diagnose_swept_path.py
"""
from __future__ import annotations

import json
import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import mujoco  # noqa: E402

from check_swept_path import swept_path_clear, PROP_GEOM_NAMES  # noqa: E402
from bimanual.control import ik  # noqa: E402
from bimanual.control import skills_scripted as sk  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

OUT_PATH = REPO_ROOT / "out" / "stage4_swept_path_diagnosis.json"
SEED = 0

# Same eight skills, same calling convention as
# scripts/stage3_eight_skill_retest.py (handoff is receiver-first).
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

_ORIGINAL_RUN_WAYPOINT = sk._run_waypoint
_ORIGINAL_RUN_DWELL = sk._run_dwell
_ORIGINAL_RUN_PICK = sk.run_pick
_CAPTURED: list[dict] = []
_WAYPOINT_COUNTER = {"n": 0}


def _capturing_run_waypoint(env, arm, target_pos, gripper_fraction, max_steps, baseline,
                             pos_tol=sk.POS_CONVERGENCE_TOL_M, target_object=None, hold_ctrl_base=None):
    model, data = env.model, env.data
    start_qpos = np.array(data.qpos, dtype=np.float64, copy=True)
    target = np.asarray(target_pos, dtype=np.float64).reshape(3)

    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ik.arm_joint_names(arm)]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
    actuator_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ik.arm_joint_names(arm)]

    solution = ik.solve_position_ik(model, data, arm, target)

    _WAYPOINT_COUNTER["n"] += 1
    _CAPTURED.append({
        "waypoint_seq": _WAYPOINT_COUNTER["n"],
        "arm": arm,
        "start_qpos": start_qpos.tolist(),
        "target_xyz": target.tolist(),
        "target_object": target_object,
        "ik_residual_m": solution.position_error_m,
    })

    # Teleport: write the solved joint angles directly into qpos AND ctrl
    # for this arm (see module docstring point 3), zero qvel, refresh
    # derived quantities.
    for k, qadr in enumerate(qpos_adrs):
        data.qpos[qadr] = solution.joint_angles[k]
    for k, aid in enumerate(actuator_ids):
        data.ctrl[aid] = solution.joint_angles[k]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    # Always report success so the caller advances to the next waypoint
    # unconditionally (module docstring point 4).
    return True, 1, None


def _noop_run_dwell(env, arm, hold_pos, gripper_fraction, n_steps, baseline,
                     target_object=None, weld=None, weld_object_name=None, hold_ctrl_base=None):
    """GRIP/RELEASE dwells hold position and close/open the jaw -- they are
    NOT a swept motion (start == target, by construction: `_dwell` freezes
    the arm's positioning ctrl for the whole dwell, ADR-031), so
    `swept_path_clear` has nothing to check for them. They CAN still fail
    for real (a genuine crush, or -- as found empirically running this
    diagnostic with `_run_dwell` unpatched -- a real physics GRIP dwell
    colliding against the table right after a DESCEND waypoint that lands
    very close to it), which would silently truncate the captured waypoint
    chain before later, real APPROACH/DESCEND/RETREAT waypoints ever run.
    Since Stage 3's own diagnostics already separately analyse GRIP/weld
    failures as their own mechanism (ADR-060: `pick(A, mug)`'s
    `weld_attach_failed`), this diagnostic script deliberately does not
    re-litigate them here -- it only sets the commanded gripper ctrl,
    refreshes derived quantities once, and always reports success so the
    REST of the intended approach sequence still gets captured and
    swept-path-checked.
    """
    gripper_ctrl = sk._gripper_ctrl(env.model, arm, gripper_fraction)
    aid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_ACTUATOR, ik.gripper_joint_name(arm))
    env.data.ctrl[aid] = gripper_ctrl
    mujoco.mj_forward(env.model, env.data)
    return True, 1, None, None


def _forced_success_run_pick(env, arm, target_object, step_budget=ik.DEFAULT_STEP_BUDGET, weld=None,
                              hold_ctrl_base=None, position_provider=None):
    """`run_place`'s and `run_handoff`'s Phase-1 code both gate their OWN
    continuation on `pick_result.success` (`if not pick_result.success:
    return SkillResult(False, "... pick failed ...")`). With
    `_run_dwell` a no-op (see `_noop_run_dwell`) the object is never
    physically lifted, so the REAL lift-margin success check is always
    False here regardless of geometry -- which would truncate every nested
    `place`/`handoff` capture at Phase 1, before their own
    (geometrically distinct) later waypoints ever run. This wrapper runs
    the REAL `run_pick` (so every `_run_waypoint` call inside it is still
    captured, unaffected) and only overrides the returned `success` flag so
    the caller's gate passes -- nothing about waypoint capture changes.
    """
    result = _ORIGINAL_RUN_PICK(
        env, arm, target_object, step_budget=step_budget, weld=weld,
        hold_ctrl_base=hold_ctrl_base, position_provider=position_provider,
    )
    if result.success:
        return result
    return sk.SkillResult(
        True, result.reason + " [diagnostic: success forced for waypoint-chain capture]",
        result.frames_used, weld_attach_frame=result.weld_attach_frame,
        weld_active_at_end=result.weld_active_at_end,
    )


def _skill_marker(label: str) -> None:
    _CAPTURED.append({"skill_marker": label})


def _run_skill_and_capture(label, skill, arm, target_object, params) -> list[dict]:
    """Fresh env per skill (ADR-047), captures every `_run_waypoint` call
    made during this ONE skill invocation.
    """
    start_idx = len(_CAPTURED)
    env = TableSettingEnv(cameras=None)
    env.reset(seed=SEED)

    if skill == "pick":
        sk.run_pick(env, arm, target_object, weld=None)
    elif skill == "place":
        sk.run_place(env, arm, target_object, destination=params.get("destination", "table"), weld=None)
    elif skill == "handoff":
        sk.run_handoff(env, arm, params["from_arm"], target_object, weld=None)
    else:
        raise ValueError(f"unknown skill {skill!r}")

    env.close()
    return _CAPTURED[start_idx:]


def _classify(model, arm: str, contact_pairs: list[dict], target_object: str | None) -> set[str]:
    """Map a waypoint's `contact_pairs` (geom-name pairs) to the task's four
    categories. A geom belongs to "this arm" if its name/body starts with
    `arm{arm}_`; "other arm" if it starts with `arm{other}_`.
    """
    other_arm = "B" if arm == "A" else "A"
    this_prefix = f"arm{arm}_"
    other_prefix = f"arm{other_arm}_"
    cats: set[str] = set()
    for c in contact_pairs:
        n1, n2 = c["geom1"], c["geom2"]
        pair = (n1, n2)
        # `_geom_label` (check_swept_path.py) names unnamed geoms
        # "{body_name}#geom{gid}" -- a plain `startswith` on the arm-prefix
        # catches both named and fallback-labelled geoms uniformly.
        is_this = [n.startswith(this_prefix) for n in pair]
        is_other = [n.startswith(other_prefix) for n in pair]
        is_table = [n == "table_top" for n in pair]
        is_prop = [n in PROP_GEOM_NAMES for n in pair]
        if any(is_table):
            cats.add("b_arm_vs_table")
        if any(is_other):
            cats.add("c_arm_vs_arm")
        if any(is_prop):
            cats.add("a_arm_vs_prop")
        if is_this[0] and is_this[1]:
            cats.add("d_self_collision")
    return cats


def main() -> int:
    print("branch check skipped (diagnostic script, read-only against skills_scripted.py "
          "beyond the frozen monkeypatch scope) -- verified separately via git diff.")
    sk._run_waypoint = _capturing_run_waypoint  # module-global monkeypatch, not a file edit
    sk._run_dwell = _noop_run_dwell  # see _noop_run_dwell's own docstring for why
    sk.run_pick = _forced_success_run_pick  # see _forced_success_run_pick's own docstring for why

    per_skill_report = []
    category_tally = {"a_arm_vs_prop": 0, "b_arm_vs_table": 0, "c_arm_vs_arm": 0, "d_self_collision": 0}
    total_waypoints_checked = 0
    total_waypoints_colliding = 0

    for label, skill, arm, target_object, params in SKILLS:
        print("=" * 70, flush=True)
        print(f"CAPTURING: {label}", flush=True)
        _skill_marker(label)
        waypoints = _run_skill_and_capture(label, skill, arm, target_object, params)
        print(f"  {len(waypoints)} waypoint(s) captured", flush=True)

        env = TableSettingEnv(cameras=None)
        env.reset(seed=SEED)
        model = env.model

        wp_results = []
        for wp in waypoints:
            clear, worst_pen, worst_step, contacts = swept_path_clear(
                model, env.data, wp["arm"],
                np.array(wp["start_qpos"], dtype=np.float64),
                np.array(wp["target_xyz"], dtype=np.float64),
                target_geom_names=None,
            )
            cats = _classify(model, wp["arm"], contacts, wp["target_object"]) if not clear else set()
            total_waypoints_checked += 1
            if not clear:
                total_waypoints_colliding += 1
                for cat in cats:
                    category_tally[cat] += 1
            wp_results.append({
                "waypoint_seq": wp["waypoint_seq"],
                "arm": wp["arm"],
                "target_object": wp["target_object"],
                "target_xyz": wp["target_xyz"],
                "ik_residual_m": wp["ik_residual_m"],
                "clear": clear,
                "worst_penetration_m": worst_pen,
                "worst_step": worst_step,
                "categories": sorted(cats),
                "contact_pairs": contacts[:10],
            })
            status = "CLEAR" if clear else f"COLLISION {sorted(cats)}"
            print(f"    wp{wp['waypoint_seq']:3d} arm={wp['arm']} target={np.round(wp['target_xyz'],4).tolist()} "
                  f"-> {status} (worst_pen={worst_pen:.5f})", flush=True)
        env.close()

        per_skill_report.append({"label": label, "waypoints": wp_results})

    sk._run_waypoint = _ORIGINAL_RUN_WAYPOINT  # restore, though this process exits anyway
    sk._run_dwell = _ORIGINAL_RUN_DWELL
    sk.run_pick = _ORIGINAL_RUN_PICK

    report = {
        "total_waypoints_checked": total_waypoints_checked,
        "total_waypoints_colliding": total_waypoints_colliding,
        "category_tally": category_tally,
        "per_skill": per_skill_report,
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(report, indent=2, default=str), newline="\n", encoding="utf-8")

    print("\n" + "=" * 70)
    print("CATEGORY TALLY (count of COLLIDING WAYPOINTS each category appears in):")
    for cat, count in sorted(category_tally.items()):
        print(f"    {cat}: {count}")
    dominant = max(category_tally, key=lambda k: category_tally[k]) if total_waypoints_colliding else None
    print(f"\nTotal waypoints checked: {total_waypoints_checked}, colliding: {total_waypoints_colliding}")
    print(f"DOMINANT CATEGORY: {dominant}")
    print(f"\nFull report: {OUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
