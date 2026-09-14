"""ScenarioRandomizer: deterministic, opt-in, scoped placement randomization
for M07 (ADR-048).

**What this module is, precisely.** A pure function of an integer seed to a
dict of per-prop (dx, dy) offsets in metres, PLUS a table of measured
per-prop envelopes (`ENVELOPES` below) those offsets are drawn from.
`TableSettingEnv.reset(seed=N, randomizer=an_instance_of_this_class)`
(`src/bimanual/sim/env.py`, ADR-048) is the only consumer: `reset()` applies
the returned offsets to each named prop's free-joint `qpos` (x, y only; z
and orientation are left exactly as the deterministic "home"-keyframe reset
already set them) via a direct `data.qpos` write followed by one
`mujoco.mj_forward` -- the SAME runtime mechanism
`scripts/generate_posenet_data.py` and the pre-M07 audit's own probe already
used, never by regenerating the scene XML (ADR-038 found that breaks
`handoff`). This module itself never touches `mujoco`, `env.py`, or any
prop's qpos directly -- it only computes offsets and hands them back.

**Why "opt-in", and why this matters more than it looks.** Two callers in
this repo depend on `TableSettingEnv.reset(seed=N)` producing the exact
FIXED baseline scene today: `scripts/verify_adr038_skills.py:19` (the
four-skill ADR-038 regression gate: pick(A,fork) z=0.3989,
place(A,fork,table) z=0.3588, pick(A,'bottle') z=0.6192, handoff(A->B,fork)
lateral sep=0.1946) and `tests/test_skills.py:76` (the 4-passed/4-failed
pytest baseline). Neither passes `randomizer=`, so `reset()`'s default
(`randomizer=None`) must leave both byte-identical -- verified on bm-ptl for
this exact commit (see `docs/hardware/m07-envelopes.md`). This mirrors the
"default OFF, explicit opt-in" shape ADR-046 already established for
perception (`ScriptedSkillExecutor(inference=None)`).

**Where `ENVELOPES` comes from.** `scripts/probe_envelope.py`'s `sweep`
subcommand measured a 7x7 grid (1 cm step, +/-3 cm range) around each of
`fork`'s and `water_bottle`'s own default (x, y), 5 repeated trials per
cell, for the four skills the ADR-038 gate already treats as working:
`pick(A, fork)`, `place(A, fork, table)`, `handoff(B, A, fork)`,
`pick(A, 'bottle')`. Full per-cell grids, the chosen rectangle per skill,
and this measurement's own methodology live in
`docs/hardware/m07-envelopes.md` -- read that document before changing
`SKILL_ENVELOPES` below; it is the ONLY source `SKILL_ENVELOPES` is allowed
to disagree with (never invented, never guessed).

**Per-prop scoping: intersection across every skill that targets that
prop, not a union, and not measured independently per skill.** `fork` is
targeted by THREE of the four measured skills (`pick`, `place`, `handoff`);
`ScenarioRandomizer` must not offer a randomized fork position any of the
three would reject, so `ENVELOPES["fork"]` is the RECTANGLE INTERSECTION of
`SKILL_ENVELOPES["pick_fork"]["fork"]`, `SKILL_ENVELOPES["place_fork"]["fork"]`
and `SKILL_ENVELOPES["handoff"]["fork"]`. `plate`, `mug` and `spoon` are not
targeted by ANY of the four skills this M07 pass measured (no skill in this
repo's current executor exercises them as a movable pick/place/handoff
target), so they have no measured envelope at all and are DELIBERATELY
EXCLUDED from `ENVELOPES` -- see `_intersect_all`'s docstring for the exact
rule and why an empty intersection is handled the same way, not papered
over.

**A second, CROSS-PROP constraint on `water_bottle`, found empirically
during Task 3 (`docs/hardware/m07-envelopes.md`'s Task 3 section), not
assumed at Task-2 design time.** `water_bottle` is targeted (as a
pick/place/handoff OBJECT) by only `pick_bottle`, so a first-cut reading of
"intersection across every skill that USES that prop" would give it
`pick_bottle`'s own measured rectangle, `(0, 0, -0.010, +0.010)`, and stop
there. Task 3's `randomized_eval` (seeds 0-9, `ScenarioRandomizer` opted in)
found this reading is not sufficient: `handoff` -- which never touches
`water_bottle` as its OWN target -- FAILED 0/10 with `water_bottle`
randomized, byte-identical every seed (same IK residual, same frame count)
at phase 3 (the cross-arm transfer staging), even though `fork` itself
never moved. Directly probed at finer resolution (`dy` in
{+-0.0001, +-0.001, +-0.005} m, `dx=0`): EVERY nonzero offset tested
reproduces the identical phase-3 failure; `dy=0.0` exactly reproduces the
unperturbed baseline exactly (`frames_used=6610`,
`from_arm_retreat_dist=0.2263`, both bit-identical to
`scripts/verify_adr038_skills.py`'s own numbers). This is a genuine binary
cliff, not a smoothly shrinkable range -- MuJoCo's simulation is a fully
globally-coupled system (every contact/force computation each step touches
the WHOLE state vector), so a prop's position can perturb a marginal,
already-knife-edge collision check (ADR-037's own term for this exact
corridor) elsewhere in the scene even with zero direct geometric proximity.
Because `handoff` is one of the four skills the ADR-038 regression gate
protects, and a global (skill-agnostic) randomizer cannot know which skill
will run next after `reset()`, `water_bottle`'s cross-skill-SAFE envelope
must also satisfy `handoff`'s empirically-found constraint -- represented
below as `SKILL_ENVELOPES["handoff"]["water_bottle"] = (0, 0, 0, 0)`, a
second, explicit fact this table records (not a target-object entry, an
empirically-measured COMPATIBILITY constraint) alongside `handoff`'s own
`fork` entry.

**The measured result, stated plainly (see `docs/hardware/m07-envelopes.md`
for the full grids and the Task 3 write-up).** `fork`'s three-way
intersection (`pick`, `place`, `handoff`) comes back a SINGLE POINT --
`handoff`'s own fine-grid envelope turned out to be exactly one passing
cell out of 29 measured (the unperturbed default itself), and intersecting
that with `pick_fork`'s and `place_fork`'s own (wider, but still
dy-zero-width) rectangles leaves zero width on BOTH axes. `water_bottle`'s
intersection, once `handoff`'s cross-prop constraint above is included,
ALSO collapses to the single point `(0, 0, 0, 0)`. `_is_degenerate_point`
(below) catches both and EXCLUDES them from `ENVELOPES` -- neither is given
a `(0, 0, 0, 0)` "range" that would call `rng.uniform(0, 0)` and always
return exactly zero, which is not randomization at all, just a disguised
no-op. **Net effect: `ENVELOPES` is empty.** This M07 pass's
`ScenarioRandomizer`, run against the evidence actually measured, and
safety-checked against every one of the four ADR-038-gated skills (not just
the skill that happens to target a given prop), randomizes NOTHING --
every prop stays at its deterministic default on every `reset()`,
randomizer-on or not, until a future pass either finds a real, non-
degenerate, cross-skill-safe range for some prop, or fixes the underlying
`handoff` fragility this Task 3 run surfaced. This is reported as the
headline finding, not hidden as a null result: it precisely quantifies how
little placement margin the currently-shipped `handoff` choreography has --
not just to its OWN target's placement (already known, ADR-032..038), but
to any other prop's position anywhere in the shared scene.
"""

from __future__ import annotations

import numpy as np

# ---------------------------------------------------------------------------
# Measured per-skill envelopes, transcribed from docs/hardware/m07-envelopes.md
# ("chosen envelope rectangle" per skill). Each rectangle is
# (dx_min, dx_max, dy_min, dy_max) in METRES, an axis-aligned box around the
# prop's own default (x, y) inside which `scripts/probe_envelope.py`'s sweep
# found >=80% (4/5 or 5/5) trial success at every one of its own grid cells.
# `None` means the measured rectangle was empty (the origin cell itself did
# not pass, or the sweep found no all-pass rectangle worth using) -- NOT
# zero-width, and NOT omitted; see `_intersect_all` below for why the
# distinction matters.
# ---------------------------------------------------------------------------
# Measured on bm-ptl, this commit, via `scripts/probe_envelope.py sweep`
# (7x7 grid, 1 cm step, +/-3 cm range, 5 reps/cell, >=80% pass threshold) --
# see `docs/hardware/m07-envelopes.md` for the full per-cell grids these
# rectangles were read off of.
#   pick_fork:   dx in [-0.010, +0.020] m, dy in [+0.000, +0.000] m (4/29 measured cells passed)
#   place_fork:  dx in [-0.010, +0.000] m, dy in [+0.000, +0.000] m (2/29 measured cells passed)
#   handoff:     dx in [+0.000, +0.000] m, dy in [+0.000, +0.000] m -- exactly ONE passing
#     cell in the full 7x7 grid: the unperturbed default itself (1/29). Every
#     other cell measured -- including both of the sweep's first two dx rows,
#     which is why an early-stop-permitted first attempt (superseded, see the
#     doc) nearly reported this as fully empty before a full re-sweep found
#     the origin's own pass -- FAILED. This is a genuine single-point
#     envelope, not a measurement gap.
#   pick_bottle: dx in [+0.000, +0.000] m, dy in [-0.010, +0.010] m (8/49 measured cells passed,
#     scattered elsewhere in the grid with no other contiguous rectangle worth
#     using -- see the doc's full grid).
#
# `handoff`'s SECOND entry below (`water_bottle`) is not a target-object
# envelope -- `handoff` never touches `water_bottle` -- it is the Task-3
# cross-prop COMPATIBILITY constraint this module's docstring's "A second,
# CROSS-PROP constraint" section explains: measured via
# `scripts/probe_envelope.py randomized_eval --skill handoff` (10/10 seeds
# FAILED with `water_bottle` randomized) and confirmed via direct probing at
# dy in {+-0.0001, +-0.001, +-0.005} m (every nonzero value reproduces the
# identical phase-3 failure; dy=0.0 exactly reproduces the true baseline).
SKILL_ENVELOPES: dict[str, dict[str, tuple[float, float, float, float] | None]] = {
    "pick_fork": {"fork": (-0.010, 0.020, 0.000, 0.000)},
    "place_fork": {"fork": (-0.010, 0.000, 0.000, 0.000)},
    "handoff": {
        "fork": (0.000, 0.000, 0.000, 0.000),
        "water_bottle": (0.000, 0.000, 0.000, 0.000),
    },
    "pick_bottle": {"water_bottle": (0.000, 0.000, -0.010, 0.010)},
}

# A rectangle intersection narrower than this (metres) on an axis is treated
# as "zero width" for the DEGENERATE-POINT check below -- purely a float-
# equality guard (the rectangles above are already round 1 cm/2 cm values,
# so this only protects against accumulated floating-point noise from
# `max()`/`min()`, not a second, looser tolerance being smuggled in).
_ZERO_WIDTH_TOL_M = 1e-9


def _intersect_all(
    rects: list[tuple[float, float, float, float] | None],
) -> tuple[float, float, float, float] | None:
    """Intersect a list of (dx_min, dx_max, dy_min, dy_max) rectangles.

    Returns `None` (explicitly, not a zero-width rectangle and not a raised
    exception) if:
      - any input rectangle is itself `None` (that skill's own measured
        envelope was empty -- a prop no skill can tolerate moving AT ALL
        cannot be given a non-empty randomization range just because
        OTHER skills that share the prop have room), or
      - every input is non-`None` but their geometric intersection is empty
        (dx_min > dx_max or dy_min > dy_max after intersecting).

    This is the exact rule Task 2 of this M07 pass specifies: "if handoff's
    fork envelope is empty, the intersection is empty, and the right answer
    is to say so and exclude that prop from randomization rather than
    silently emitting a zero-width range or crashing." A caller (`ENVELOPES`
    below, built at import time) is expected to simply OMIT any prop this
    function returns `None` for.
    """
    if not rects:
        return None
    if any(r is None for r in rects):
        return None
    dx_min = max(r[0] for r in rects)
    dx_max = min(r[1] for r in rects)
    dy_min = max(r[2] for r in rects)
    dy_max = min(r[3] for r in rects)
    if dx_min > dx_max or dy_min > dy_max:
        return None
    return (dx_min, dx_max, dy_min, dy_max)


def _is_degenerate_point(rect: tuple[float, float, float, float]) -> bool:
    """True if `rect` has (numerically) zero width on BOTH axes -- i.e. it
    permits exactly one (dx, dy) value, always the same one, regardless of
    seed. Distinguished deliberately from a rectangle that is zero-width on
    ONLY one axis (e.g. `water_bottle`'s measured `(0, 0, -0.01, 0.01)`:
    real, non-degenerate variation in dy even though dx never moves) --
    that case is a legitimate, if narrow, randomization axis and is kept.
    A both-axes-degenerate rectangle is NOT geometrically empty (dx_min <=
    dx_max and dy_min <= dy_max both hold, usually at 0), so
    `_intersect_all` alone would happily return it -- this is the second,
    separate check Task 2 asks for: "exclude that prop from randomization
    rather than silently emitting a zero-width range."
    """
    dx_min, dx_max, dy_min, dy_max = rect
    return (dx_max - dx_min) < _ZERO_WIDTH_TOL_M and (dy_max - dy_min) < _ZERO_WIDTH_TOL_M


def _build_envelopes() -> dict[str, tuple[float, float, float, float]]:
    """Build the prop -> intersected-rectangle table `ENVELOPES` exports.

    Groups `SKILL_ENVELOPES` by PROP (not by skill), intersects every
    skill's rectangle for that prop via `_intersect_all`, and keeps only the
    props whose intersection came back non-`None` AND is not degenerate to a
    single point on BOTH axes (`_is_degenerate_point` -- see its own
    docstring for why a single-axis-zero-width rectangle, e.g.
    `water_bottle`'s, is kept while a both-axes one, e.g. `fork`'s, is not).
    A prop no measured skill ever targets simply never appears as a key in
    the grouped-by-prop dict below, so it is excluded the same way an empty
    or degenerate intersection is -- uniformly, with no special-cased
    "skip this one" logic.
    """
    by_prop: dict[str, list[tuple[float, float, float, float] | None]] = {}
    for skill_rects in SKILL_ENVELOPES.values():
        for prop, rect in skill_rects.items():
            by_prop.setdefault(prop, []).append(rect)

    envelopes: dict[str, tuple[float, float, float, float]] = {}
    excluded: dict[str, str] = {}
    for prop, rects in by_prop.items():
        intersected = _intersect_all(rects)
        if intersected is None:
            excluded[prop] = (
                f"empty intersection across {len(rects)} skill envelope(s) "
                f"({rects!r}) -- excluded from randomization"
            )
        elif _is_degenerate_point(intersected):
            excluded[prop] = (
                f"intersection {intersected!r} across {len(rects)} skill envelope(s) "
                f"({rects!r}) is a single point (zero width on BOTH axes) -- every "
                f"skill that targets this prop tolerates it moving in at most one "
                f"axis and only down to a single measured value, so there is no "
                f"real randomization range left once all of them must agree; "
                f"excluded from randomization rather than offered as a fake "
                f"always-zero 'range'"
            )
        else:
            envelopes[prop] = intersected
    return envelopes, excluded


#: prop name -> (dx_min, dx_max, dy_min, dy_max) in metres, the intersection
#: across every measured skill that targets that prop. Built once at import
#: time from `SKILL_ENVELOPES` above via `_intersect_all` -- see this
#: module's docstring's "per-prop scoping" section. `plate`, `mug`, `spoon`
#: never appear here (no measured skill targets them this pass); any prop
#: whose intersection came back empty (see `EXCLUDED_PROPS` below) is also
#: absent, not present with a zero-width range.
ENVELOPES, EXCLUDED_PROPS = _build_envelopes()


class ScenarioRandomizer:
    """`randomize(seed) -> {prop_name: (dx, dy)}`, deterministic in `seed`.

    Only draws offsets for props present in `self.envelopes` (default:
    module-level `ENVELOPES`, i.e. only props with a non-empty measured
    intersection across every skill that targets them -- see module
    docstring). A prop excluded from `self.envelopes` (whether because no
    skill targets it, or because its measured intersection was empty) is
    simply never a key in the returned dict; `TableSettingEnv.reset()`
    therefore leaves that prop at its deterministic default position,
    unchanged, exactly as if no randomizer had been passed at all.
    """

    def __init__(
        self,
        envelopes: dict[str, tuple[float, float, float, float]] | None = None,
    ) -> None:
        self.envelopes = dict(ENVELOPES) if envelopes is None else dict(envelopes)

    def randomize(self, seed: int) -> dict[str, tuple[float, float]]:
        """Return {prop_name: (dx, dy)} for every prop in `self.envelopes`.

        Deterministic in `seed` alone: constructs a fresh
        `np.random.default_rng(seed)` (never touches any global numpy RNG
        state or `TableSettingEnv.np_random` -- see `env.py`'s own comment
        on why this stays a pure function of `seed`), then draws props IN
        SORTED-NAME ORDER (not dict insertion order, which callers could
        vary without meaning to) so the exact sequence of `rng.uniform(...)`
        calls -- and therefore the exact offsets returned for a given
        `seed` -- is reproducible across processes and across any future
        reordering of `self.envelopes`'s own construction.
        """
        rng = np.random.default_rng(seed)
        offsets: dict[str, tuple[float, float]] = {}
        for prop in sorted(self.envelopes):
            dx_min, dx_max, dy_min, dy_max = self.envelopes[prop]
            dx = float(rng.uniform(dx_min, dx_max))
            dy = float(rng.uniform(dy_min, dy_max))
            offsets[prop] = (dx, dy)
        return offsets


if __name__ == "__main__":
    # Minimal manual smoke check -- stdlib + numpy only, no `mujoco` import,
    # runs on the laptop (same discipline as `scripts/gen_dual_scene.py`
    # under ADR-021: this module's own pure-function half must not require
    # bm-ptl to exercise).
    print(f"ENVELOPES = {ENVELOPES}")
    print(f"EXCLUDED_PROPS = {EXCLUDED_PROPS}")
    randomizer = ScenarioRandomizer()
    for seed in range(3):
        print(f"seed={seed}: {randomizer.randomize(seed)}")
    # Determinism check: same seed, two calls, byte-identical.
    a = randomizer.randomize(0)
    b = randomizer.randomize(0)
    assert a == b, f"randomize(0) not deterministic: {a} != {b}"
    print("OK: randomize(seed) is deterministic across repeated calls.")
