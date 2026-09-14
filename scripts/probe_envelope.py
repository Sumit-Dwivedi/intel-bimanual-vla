"""M07 fine-grid placement-envelope measurement (ADR-048).

Extends the pre-M07/M08 audit's own single-prop grid mechanism
(`scripts/probe_pre_m07_audit.py`'s `grid` subcommand, variable 6) to the
resolution and range M07 actually needs to build a defensible
`ScenarioRandomizer` on top of: **7x7 grid, 1 cm step, +/-3 cm range**,
around each prop's own default (x, y), for the four skills the ADR-038
regression gate already treats as "working": `pick(A, fork)`,
`place(A, fork, table)`, `handoff(B, A, fork)`, `pick(A, 'bottle')`.

This script is diagnostic/measurement only -- like its audit predecessor, it
never modifies `skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`,
`env.py`'s pre-existing behaviour (env.py's own change in this commit is an
OPT-IN `randomizer=` argument to `reset()`, never exercised by this script,
which only ever calls the plain deterministic `reset(seed=...)` and then
overwrites one prop's qpos directly, exactly the mechanism
`scripts/generate_posenet_data.py` and the pre-M07 audit already used),
`scenes/so101/`, `gen_dual_scene.py`, `posenet.py`, `dataset.py`, the
checkpoint, or the IR.

**Why "5 seeds per cell" measures something real, even though this sim has
no execution-time randomness on this path.** Each grid CELL fixes the
target prop's (dx, dy) exactly -- that is the whole point of a grid, as
opposed to a jittered sample. `TableSettingEnv.reset(seed=N)` is
byte-identical for every `N` given a fixed final qpos (no randomizer is
passed here), and nothing in `skills_scripted.py`/`ik.py`/`grasp.py` draws
from any RNG on the scripted-controller path (grepped, confirmed, same
finding the pre-M07 audit already made). So the 5 reps per cell are 5
INDEPENDENT fresh-env-fresh-executor repetitions (seeds 0-4, forwarded to
`env.reset(seed=...)` for bookkeeping even though they provably do not
change the outcome) rather than 5 draws of a fifth randomization axis. This
is a deliberate defensive design, not an oversight: it is a live regression
check against the exact class of bug ADR-047 fixed (a cross-trial state
leak that made supposedly-identical repeated trials disagree) -- every cell
is EXPECTED to report 0/5 or 5/5. A cell reporting anything else (1/5-4/5)
is not "the success threshold wasn't met," it is evidence of a live
nondeterminism bug in the harness or the underlying code, and is flagged as
such in this script's own output rather than silently averaged into a
percentage.

**Rejection (off-table / intersecting another prop), and why it uses real
MuJoCo contact detection, not `generate_posenet_data.py`'s bounding-circle
heuristic.** That script's `FOOTPRINT_RADIUS_M`/`CLEARANCE_MARGIN_M`
pairwise-distance check is explicitly documented there as a RENDER-
LEGIBILITY check for a from-scratch multi-prop layout generator, "not a
physics settle." Reusing it here would be wrong: the scene's own SHIPPED
default already has some prop pairs (e.g. fork/spoon) closer together than
that heuristic's forbidden-disk sum, because the heuristic's circles are
deliberately conservative, isotropic over-approximations of irregular prop
shapes that happen not to touch at their ACTUAL relative bearing in the
shipped layout. Applying that heuristic near a prop's own default would
reject cells the real, working scene already occupies. Instead, after
writing the candidate (x, y) into the target prop's qpos and calling
`mj_forward` (which runs MuJoCo's own broad+narrow-phase collision
detection as part of computing forward kinematics), this script inspects
`env.data.contact` directly for any PROP-vs-PROP contact whose penetration
depth (`contact.dist`) is more negative than `-COLLISION_DIST_TOL_M` --  a
real, geometry-accurate overlap check, not a heuristic one. Off-table uses
`FOOTPRINT_RADIUS_M` (cited from `generate_posenet_data.py:276-281`, copied
rather than imported since that script is not a package module) only as a
margin against the table surface's own half-extents (read from
`gen_dual_scene.py:1037`'s declared geom size, cited not re-derived) -- a
reasonable use of that same conservative radius, since "close to the table
edge" is exactly the kind of coarse, orientation-agnostic bound a bounding
circle is good at.

Subcommands
-----------
  sweep   -- the main 7x7 measurement. `--skill {pick_fork,place_fork,
             handoff,pick_bottle} --out PATH [--early-stop-empty-rows N]`.
             Writes one JSON line per TRIAL immediately (`f.write(...);
             f.flush()`), plus one per-CELL summary line after each cell's
             (up to) 5 trials complete, plus one per-SKILL summary line at
             the end (or at early-stop). A killed mid-sweep run therefore
             never loses a completed cell.
  report  -- reads a `sweep` JSONL file back and prints the 7x7 pass/fail
             grid, the chosen envelope rectangle (largest axis-aligned
             all-PASS rectangle containing the origin cell, brute-forced
             over the 7x7 grid's own 4x4x4x4=256 possible extents -- small
             enough to enumerate exhaustively, no heuristic needed), and
             that rectangle's own coverage (trivially 100% by
             construction, reported anyway for transparency against the
             full 49-cell measurement).
  timing  -- runs a handful of real cells (`--skill ... --n-cells N`) and
             reports measured per-trial and per-cell wall time, plus a
             projected total sweep time for all four skills at the full
             7x7x5 grid -- run this BEFORE `sweep` and report the
             projection, per this module's own task instruction.
  randomized_eval -- Task 3: seeds 0-9, oracle mode, `bimanual.sim.
             randomization.ScenarioRandomizer` opted IN via
             `env.reset(seed=N, randomizer=...)`. `--skill {...} --out
             PATH`. One fresh env+executor per seed (same zero-th-finding
             discipline as `sweep`). Records which props were actually
             offset (from `ScenarioRandomizer.randomize(seed)` -- possibly
             none, if `ENVELOPES` is empty for every prop this skill
             targets) and by how much, alongside success/reason/frames, so
             a failure can be read against the exact offsets that produced
             it.
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

RESET_SEED_BASE = 0  # forwarded to env.reset(seed=...) per trial; see module
# docstring's "5 seeds per cell" section for why this provably does not
# change the deterministic outcome on this code path -- seeds 0..4 used
# below are RESET_SEED_BASE + rep index, not a second env.reset() baseline.

N_SEEDS_PER_CELL = 5
SUCCESS_THRESHOLD = 0.80  # >= 4/5

# 1 cm step, +/-3 cm range -> 7 offsets, 49 grid cells.
GRID_STEP_M = 0.01
GRID_RANGE_M = 0.03
OFFSETS_M = [round(GRID_STEP_M * i, 6) for i in range(-3, 4)]
assert OFFSETS_M == [-0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.03]

_SKILL_TARGET_PROP = {
    "handoff": "fork",
    "pick_fork": "fork",
    "place_fork": "fork",
    "pick_bottle": "water_bottle",
}

ALL_PROP_BODIES = ("plate", "mug", "fork", "spoon", "water_bottle")

# Cited from scripts/generate_posenet_data.py:276-281 (FOOTPRINT_RADIUS_M) --
# copied, not imported (that script is a standalone CLI, not a package
# module, and this file must not add an import-path dependency on it). Used
# HERE only as a conservative margin against the table edge (off-table
# check) -- never as the prop-vs-prop overlap check, see module docstring.
FOOTPRINT_RADIUS_M = {
    "plate": 0.06,
    "mug": 0.06,
    "fork": 0.08,
    "spoon": 0.07,
    "water_bottle": 0.03,
}

# Cited from scripts/gen_dual_scene.py:1037's declared table_top geom:
#   <geom name="table_top" type="box" size="0.40 0.25 0.01" pos="0 0 0.34" .../>
# `size` for a MuJoCo box is HALF-extents, so the table surface spans
# x in [-0.40, 0.40], y in [-0.25, 0.25], centred at the world origin.
TABLE_HALF_EXTENT_M = {"x": 0.40, "y": 0.25}

# A contact penetration deeper than this (metres) between two DIFFERENT
# prop bodies is treated as a genuine overlap, not numerical/margin fuzz.
# MuJoCo's default contact margin can report small near-zero (or slightly
# negative, within its own solver tolerance) contacts for objects legitimately
# resting near each other without either being "colliding" in any real sense;
# 2 mm is comfortably below the smallest FOOTPRINT_RADIUS_M (0.03 m,
# water_bottle) and comfortably above typical solver/margin fuzz.
COLLISION_DIST_TOL_M = 0.002


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


def _prop_qpos_adr(model, body_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{body_name}_free")
    if jid == -1:
        raise RuntimeError(f"expected free joint '{body_name}_free' not found in compiled model")
    return int(model.jnt_qposadr[jid])


def _default_prop_xy(env: TableSettingEnv, body_name: str) -> tuple[float, float]:
    """Read the prop's own default resting (x, y) LIVE off a fresh
    `reset(seed=RESET_SEED_BASE)`, same discipline as the pre-M07 audit
    (`probe_pre_m07_audit.py:_default_prop_xy`) and `generate_posenet_data.py`'s
    Correction 3 -- never a hardcoded duplicate of `gen_dual_scene.py`'s
    position constants.
    """
    env.reset(seed=RESET_SEED_BASE, cameras=None)
    adr = _prop_qpos_adr(env.model, body_name)
    return float(env.data.qpos[adr]), float(env.data.qpos[adr + 1])


def _set_target_xy(env: TableSettingEnv, seed: int, body_name: str, x: float, y: float) -> None:
    """Reset to this trial's own seed, then overwrite ONE prop's free-joint
    x/y in place (z and quaternion left exactly as reset() set them), then
    mj_forward -- identical mechanism to `probe_pre_m07_audit.py`'s
    `_apply_jitter`, generalised to accept the trial's own seed rather than
    a single fixed baseline (see module docstring's "5 seeds per cell").
    """
    env.reset(seed=seed, cameras=None)
    adr = _prop_qpos_adr(env.model, body_name)
    env.data.qpos[adr] = x
    env.data.qpos[adr + 1] = y
    mujoco.mj_forward(env.model, env.data)


def _body_xyz(env: TableSettingEnv, body_name: str) -> list[float]:
    bid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return [float(v) for v in env.data.xpos[bid]]


def _is_off_table(x: float, y: float, body_name: str) -> bool:
    r = FOOTPRINT_RADIUS_M[body_name]
    x_ok = -TABLE_HALF_EXTENT_M["x"] + r <= x <= TABLE_HALF_EXTENT_M["x"] - r
    y_ok = -TABLE_HALF_EXTENT_M["y"] + r <= y <= TABLE_HALF_EXTENT_M["y"] - r
    return not (x_ok and y_ok)


def _check_prop_overlap(env: TableSettingEnv, target_body: str) -> tuple[bool, str | None]:
    """After `mj_forward` has already run at the candidate pose, inspect
    `env.data.contact` for any PROP-vs-PROP contact deeper than
    `COLLISION_DIST_TOL_M`. Real MuJoCo collision geometry, not a bounding-
    circle heuristic -- see module docstring.

    Returns (is_overlapping, detail_string_or_None).
    """
    body_ids = {
        name: mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, name) for name in ALL_PROP_BODIES
    }
    id_to_name = {v: k for k, v in body_ids.items()}

    for i in range(env.data.ncon):
        contact = env.data.contact[i]
        b1 = int(env.model.geom_bodyid[contact.geom1])
        b2 = int(env.model.geom_bodyid[contact.geom2])
        if b1 == b2:
            continue
        if b1 not in id_to_name or b2 not in id_to_name:
            continue  # not a prop-vs-prop contact (e.g. prop-vs-table, prop-vs-arm)
        if float(contact.dist) < -COLLISION_DIST_TOL_M:
            n1, n2 = id_to_name[b1], id_to_name[b2]
            return True, f"{n1}<->{n2} penetration dist={float(contact.dist):.4f}m"
    return False, None


def _write_jsonl(path: pathlib.Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")
        f.flush()


def _run_one_trial(skill_name: str, body_name: str, seed: int, x: float, y: float) -> dict:
    """Fresh `TableSettingEnv` + fresh `ScriptedSkillExecutor` for this ONE
    trial (ADR-047 / the pre-M07 audit's zero-th finding: reusing either
    across a `reset()`-based loop silently poisons later trials via
    `WeldGrasp.active_welds` surviving `env.reset()`).
    """
    env = TableSettingEnv(cameras=None)
    executor = ScriptedSkillExecutor()
    _set_target_xy(env, seed, body_name, x, y)

    t0 = time.time()
    result = executor.execute(_skill_call(skill_name), env, step_budget=ik.DEFAULT_STEP_BUDGET)
    wall_s = time.time() - t0

    record = {
        "skill": skill_name,
        "prop": body_name,
        "seed": int(seed),
        "xy_used": [float(x), float(y)],
        "success": bool(result.success),
        "reason": result.reason,
        "frames_used": int(result.frames_used),
        "wall_s": wall_s,
    }
    env.close()
    return record


def cmd_sweep(args: argparse.Namespace) -> int:
    skill_name = args.skill
    body_name = _SKILL_TARGET_PROP[skill_name]
    out_path = pathlib.Path(args.out)

    probe_env = TableSettingEnv(cameras=None)
    default_x, default_y = _default_prop_xy(probe_env, body_name)
    print(
        f"[{skill_name}] target prop={body_name!r} default=({default_x:.4f},{default_y:.4f}) "
        f"grid: {len(OFFSETS_M)}x{len(OFFSETS_M)}={len(OFFSETS_M)**2} cells x {N_SEEDS_PER_CELL} seeds "
        f"= {len(OFFSETS_M)**2 * N_SEEDS_PER_CELL} trials (budget, before any rejection)"
    )

    consecutive_empty_rows = 0
    n_cells_measured = 0
    n_cells_passed = 0
    n_cells_rejected = 0
    n_trials_total = 0
    t_sweep_start = time.time()
    stopped_early = False

    for dx in OFFSETS_M:
        row_had_pass = False
        row_had_nonrejected_cell = False
        for dy in OFFSETS_M:
            x, y = default_x + dx, default_y + dy

            # --- Rejection checks: off-table, or intersecting another prop.
            if _is_off_table(x, y, body_name):
                _write_jsonl(out_path, {
                    "skill": skill_name, "prop": body_name, "dx": dx, "dy": dy,
                    "xy_used": [x, y], "cell_summary": True, "rejected": True,
                    "reject_reason": "off_table",
                })
                print(f"  dx={dx:+.3f} dy={dy:+.3f} REJECTED (off_table)")
                n_cells_rejected += 1
                continue

            # Probe the collision check with a fresh env at seed 0 for this
            # cell -- the check itself needs no skill execution, just the
            # candidate pose. If it flags an overlap, every one of this
            # cell's 5 trials would start from an invalid configuration, so
            # none are run.
            check_env = TableSettingEnv(cameras=None)
            _set_target_xy(check_env, RESET_SEED_BASE, body_name, x, y)
            overlapping, detail = _check_prop_overlap(check_env, body_name)
            check_env.close()
            if overlapping:
                _write_jsonl(out_path, {
                    "skill": skill_name, "prop": body_name, "dx": dx, "dy": dy,
                    "xy_used": [x, y], "cell_summary": True, "rejected": True,
                    "reject_reason": "intersects_another_prop", "reject_detail": detail,
                })
                print(f"  dx={dx:+.3f} dy={dy:+.3f} REJECTED (intersects_another_prop: {detail})")
                n_cells_rejected += 1
                continue

            row_had_nonrejected_cell = True

            # --- Real cell: N_SEEDS_PER_CELL fresh trials.
            n_success = 0
            for rep in range(N_SEEDS_PER_CELL):
                seed = RESET_SEED_BASE + rep
                trial = _run_one_trial(skill_name, body_name, seed, x, y)
                trial["dx"], trial["dy"] = dx, dy
                n_success += int(trial["success"])
                n_trials_total += 1
                _write_jsonl(out_path, trial)

            cell_pass = (n_success / N_SEEDS_PER_CELL) >= SUCCESS_THRESHOLD
            n_cells_measured += 1
            n_cells_passed += int(cell_pass)
            if n_success not in (0, N_SEEDS_PER_CELL):
                print(
                    f"  ** NONDETERMINISM FLAG ** dx={dx:+.3f} dy={dy:+.3f} "
                    f"{n_success}/{N_SEEDS_PER_CELL} -- identical repeated trials disagreed; "
                    f"see module docstring's '5 seeds per cell' section."
                )
            if cell_pass:
                row_had_pass = True
            _write_jsonl(out_path, {
                "skill": skill_name, "prop": body_name, "dx": dx, "dy": dy,
                "xy_used": [x, y], "cell_summary": True, "rejected": False,
                "n_success": n_success, "n_total": N_SEEDS_PER_CELL, "cell_pass": cell_pass,
            })
            print(f"  dx={dx:+.3f} dy={dy:+.3f} {n_success}/{N_SEEDS_PER_CELL} pass={cell_pass}")

        # End of one dx "row" (all 7 dy values tried).
        if row_had_nonrejected_cell and not row_had_pass:
            consecutive_empty_rows += 1
        elif row_had_nonrejected_cell:
            consecutive_empty_rows = 0
        # (a fully-rejected row is not evidence either way; counter untouched)

        if args.early_stop_empty_rows and consecutive_empty_rows >= args.early_stop_empty_rows:
            elapsed = time.time() - t_sweep_start
            print(
                f"[{skill_name}] EARLY STOP: {consecutive_empty_rows} consecutive all-fail rows "
                f"(dx up through {dx:+.3f}) -- per this task's own instruction, stopping this "
                f"sweep rather than spending further wall-clock confirming emptiness. "
                f"{n_cells_measured} cells measured, {n_cells_passed} passed, "
                f"{n_cells_rejected} rejected, {elapsed:.1f}s elapsed."
            )
            stopped_early = True
            break

    sweep_wall_s = time.time() - t_sweep_start
    summary = {
        "skill": skill_name, "prop": body_name, "sweep_summary": True,
        "n_cells_measured": n_cells_measured, "n_cells_passed": n_cells_passed,
        "n_cells_rejected": n_cells_rejected, "n_trials_total": n_trials_total,
        "sweep_wall_s": sweep_wall_s, "stopped_early": stopped_early,
    }
    _write_jsonl(out_path, summary)
    print(
        f"[{skill_name}] sweep done: {n_cells_passed}/{n_cells_measured} cells passed "
        f"({n_cells_rejected} rejected), {n_trials_total} trials, {sweep_wall_s:.1f}s, "
        f"stopped_early={stopped_early}. Appended to {out_path}"
    )
    probe_env.close()
    return 0


def cmd_timing(args: argparse.Namespace) -> int:
    """Run --n-cells real cells (all N_SEEDS_PER_CELL trials each) near the
    default position, measure real wall time, and project the full 4-skill
    sweep -- run BEFORE `sweep`, per this task's own instruction to report a
    projection before committing to the full run.
    """
    skill_name = args.skill
    body_name = _SKILL_TARGET_PROP[skill_name]

    probe_env = TableSettingEnv(cameras=None)
    default_x, default_y = _default_prop_xy(probe_env, body_name)
    probe_env.close()

    cells = [(OFFSETS_M[i], OFFSETS_M[i]) for i in range(min(args.n_cells, len(OFFSETS_M)))]
    print(f"[timing] skill={skill_name} sampling {len(cells)} cells x {N_SEEDS_PER_CELL} seeds")

    t0 = time.time()
    n_trials = 0
    for dx, dy in cells:
        x, y = default_x + dx, default_y + dy
        for rep in range(N_SEEDS_PER_CELL):
            seed = RESET_SEED_BASE + rep
            trial = _run_one_trial(skill_name, body_name, seed, x, y)
            n_trials += 1
            print(f"  dx={dx:+.3f} dy={dy:+.3f} seed={seed} success={trial['success']} "
                  f"frames={trial['frames_used']} wall={trial['wall_s']:.2f}s")
    elapsed = time.time() - t0
    per_trial = elapsed / n_trials
    per_cell = per_trial * N_SEEDS_PER_CELL
    full_cells = len(OFFSETS_M) ** 2
    projected_this_skill = per_cell * full_cells
    print(
        f"[timing] {n_trials} trials in {elapsed:.1f}s -> {per_trial:.2f}s/trial, "
        f"{per_cell:.1f}s/cell -> projected {projected_this_skill:.0f}s "
        f"({projected_this_skill/60:.1f} min) for this skill's full {full_cells}-cell sweep "
        f"(ignoring rejected cells, which cost ~0 execution time)."
    )
    return 0


def _load_cells(path: pathlib.Path, skill_name: str) -> dict[tuple[float, float], dict]:
    cells: dict[tuple[float, float], dict] = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("skill") != skill_name or not rec.get("cell_summary"):
                continue
            cells[(rec["dx"], rec["dy"])] = rec
    return cells


def _largest_all_pass_rectangle(cells: dict[tuple[float, float], dict]) -> tuple[int, int, int, int] | None:
    """Brute-force over all (a, b, c, d) extents (each 0..3 grid steps) from
    the origin cell (dx=dy=0), where the rectangle is
    dx in OFFSETS_M[3-a : 4+b], dy in OFFSETS_M[3-c : 4+d]. Returns the
    (a, b, c, d) with the largest cell-count (a+b+1)*(c+d+1) such that EVERY
    cell inside is present, not rejected, and cell_pass True. 256 candidates
    -- exhaustive, no heuristic. Returns None if the origin itself is not a
    passing cell.
    """
    origin = cells.get((0.0, 0.0))
    if origin is None or origin.get("rejected") or not origin.get("cell_pass"):
        return None

    best = None
    best_area = 0
    for a in range(4):
        for b in range(4):
            for c in range(4):
                for d in range(4):
                    ok = True
                    for i in range(3 - a, 4 + b):
                        for j in range(3 - c, 4 + d):
                            cell = cells.get((OFFSETS_M[i], OFFSETS_M[j]))
                            if cell is None or cell.get("rejected") or not cell.get("cell_pass"):
                                ok = False
                                break
                        if not ok:
                            break
                    if not ok:
                        continue
                    area = (a + b + 1) * (c + d + 1)
                    if area > best_area:
                        best_area = area
                        best = (a, b, c, d)
    return best


def cmd_report(args: argparse.Namespace) -> int:
    skill_name = args.skill
    path = pathlib.Path(args.out)
    cells = _load_cells(path, skill_name)

    print(f"=== {skill_name} 7x7 grid (rows=dx, cols=dy, cm) ===")
    header = "dx\\dy".rjust(7) + "".join(f"{dy*100:+.0f}".rjust(8) for dy in OFFSETS_M)
    print(header)
    for dx in OFFSETS_M:
        row = f"{dx*100:+.0f}".rjust(7)
        for dy in OFFSETS_M:
            cell = cells.get((dx, dy))
            if cell is None:
                cell_str = "  ?"
            elif cell.get("rejected"):
                cell_str = "REJ"
            else:
                cell_str = f"{cell['n_success']}/{cell['n_total']}"
            row += cell_str.rjust(8)
        print(row)

    n_measured = sum(1 for c in cells.values() if not c.get("rejected"))
    n_passed = sum(1 for c in cells.values() if not c.get("rejected") and c.get("cell_pass"))
    n_rejected = sum(1 for c in cells.values() if c.get("rejected"))
    print(f"totals: {n_passed}/{n_measured} cells passed, {n_rejected} rejected, "
          f"{49 - n_measured - n_rejected} not measured (early stop or incomplete)")

    rect = _largest_all_pass_rectangle(cells)
    if rect is None:
        print("chosen envelope rectangle: NONE (origin cell dx=dy=0 did not pass -- "
              "cannot anchor a rectangle there).")
    else:
        a, b, c, d = rect
        dx_lo, dx_hi = OFFSETS_M[3 - a], OFFSETS_M[3 + b]
        dy_lo, dy_hi = OFFSETS_M[3 - c], OFFSETS_M[3 + d]
        area = (a + b + 1) * (c + d + 1)
        print(
            f"chosen envelope rectangle: dx in [{dx_lo:+.3f}, {dx_hi:+.3f}] m, "
            f"dy in [{dy_lo:+.3f}, {dy_hi:+.3f}] m ({area} cells, all-PASS by construction, "
            f"coverage=100% of its own {area} cells; {n_passed}/{n_measured} overall measured "
            f"pass rate across the full 7x7 grid for context)."
        )
    return 0


def cmd_randomized_eval(args: argparse.Namespace) -> int:
    """Task 3: seeds 0-9, oracle mode, `ScenarioRandomizer` opted in.

    Unlike `sweep` (which moves ONE prop to an exact grid cell and leaves
    every other prop at its default), this exercises the SHIPPED randomizer
    exactly as a future caller would: every prop `ScenarioRandomizer.
    ENVELOPES` covers moves together, drawn from ONE `rng =
    np.random.default_rng(seed)` per seed (see `randomization.py`'s own
    docstring for the exact draw order/determinism contract).
    """
    from bimanual.sim.randomization import ScenarioRandomizer

    skill_name = args.skill
    out_path = pathlib.Path(args.out)
    randomizer = ScenarioRandomizer()
    print(f"[{skill_name}] randomized_eval, ScenarioRandomizer.envelopes={randomizer.envelopes}")

    n_success = 0
    frames_on_success = []
    for seed in range(10):
        env = TableSettingEnv(cameras=None)
        executor = ScriptedSkillExecutor()
        env.reset(seed=seed, cameras=None, randomizer=randomizer)
        offsets_used = randomizer.randomize(seed)  # same seed -> same offsets env.reset() just applied

        t0 = time.time()
        result = executor.execute(_skill_call(skill_name), env, step_budget=ik.DEFAULT_STEP_BUDGET)
        wall_s = time.time() - t0

        record = {
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
        _write_jsonl(out_path, record)
        print(f"  seed={seed} offsets={offsets_used} success={result.success} "
              f"frames={result.frames_used} wall={wall_s:.2f}s reason={result.reason}")
        env.close()

    mean_frames = (sum(frames_on_success) / len(frames_on_success)) if frames_on_success else None
    summary = {
        "skill": skill_name, "randomized_eval_summary": True,
        "n_success": n_success, "n_total": 10, "mean_frames_on_success": mean_frames,
    }
    _write_jsonl(out_path, summary)
    print(f"[{skill_name}] randomized_eval done: {n_success}/10, mean_frames_on_success={mean_frames}")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    p_sweep = sub.add_parser("sweep", help="7x7 fine-grid envelope measurement")
    p_sweep.add_argument("--skill", required=True, choices=list(_SKILL_TARGET_PROP))
    p_sweep.add_argument("--out", required=True)
    p_sweep.add_argument("--early-stop-empty-rows", type=int, default=0,
                          help="stop the sweep after this many consecutive all-fail dx rows "
                               "(0 = never stop early)")
    p_sweep.set_defaults(func=cmd_sweep)

    p_timing = sub.add_parser("timing", help="measure real per-cell cost, project full-sweep total")
    p_timing.add_argument("--skill", required=True, choices=list(_SKILL_TARGET_PROP))
    p_timing.add_argument("--n-cells", type=int, default=3)
    p_timing.set_defaults(func=cmd_timing)

    p_report = sub.add_parser("report", help="print the 7x7 grid + chosen rectangle from a sweep's JSONL")
    p_report.add_argument("--skill", required=True, choices=list(_SKILL_TARGET_PROP))
    p_report.add_argument("--out", required=True)
    p_report.set_defaults(func=cmd_report)

    p_reval = sub.add_parser("randomized_eval", help="Task 3: seeds 0-9, ScenarioRandomizer opted in")
    p_reval.add_argument("--skill", required=True, choices=list(_SKILL_TARGET_PROP))
    p_reval.add_argument("--out", required=True)
    p_reval.set_defaults(func=cmd_randomized_eval)

    return p


if __name__ == "__main__":
    args = build_arg_parser().parse_args()
    raise SystemExit(args.func(args))
