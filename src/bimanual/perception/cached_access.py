"""CachedPropPositions -- the ONE seam between a scripted skill and PoseNet
(M10 Phase 5, ADR-046).

**Why a cache at all, rather than calling `inference.predict()` inline
everywhere a position is needed.** A single `run_pick`/`run_place` call
already reads an object's position more than once across its own waypoints
in the pre-existing oracle code (see `skills_scripted.py`'s `obj_pos0`/
`obj_xy` reads). Re-rendering `posenet_cam` and re-running the network on
every one of those reads would multiply an already-measured per-render cost
(ADR-022) by however many times a skill happens to read a position -- pure
waste, since the object has not moved between two reads taken microseconds
apart in the SAME `env.step()`-free stretch of Python. This class renders
and infers AT MOST ONCE per "cache generation", and every `get()` call
within that generation reuses the same prediction. `invalidate()` starts a
new generation; the next `get()` after it pays one fresh render+infer.

**Why this class, and not `PoseNetInference` itself, decides when to fall
back to oracle (ADR-046's Correction 2).** ADR-044's caveat 1 records that
PoseNet's z output carries no real signal: every training image shows a
prop resting on the table, so z is a per-prop constant the network
memorized, not a quantity it can generalize. The moment an arm's weld picks
an object up, its TRUE z rises well above that resting constant (the water
bottle, for example, ends a full lift's worth higher, ADR-034's measured
final z=0.6192 against a resting z=0.4400 -- roughly 180 mm) while PoseNet,
shown a fresh render, would keep confidently predicting the resting height
it always predicts for that prop. Serving that stale estimate as if it were
current would inject a ~180 mm error into whichever skill reads it next
-- worse than not having perception at all, because it would be silently
wrong rather than visibly absent. **This class's `get()` therefore refuses
to serve a PoseNet estimate for any prop currently held by either arm's
`WeldGrasp`, returning `None` (the same "fall through to oracle" signal used
for any prop outside PoseNet's 3-prop scope) instead.** Concretely this
means: a targeting call site wired to `get()` (e.g. `run_place`'s
destination-offset read, which by construction always runs on an object the
gripper is already holding) is *mechanically* wired to perception, but at
runtime is *served* by the oracle fallback for exactly the reason above --
not because that call site was left oracle, but because the safety rule
inside `get()` recognizes the object is airborne and refuses to guess.

**Why NOT auto-refresh on `attempt_grasp` succeeding (a real behavioural
choice, stated plainly).** An earlier design considered by this module's
task brief invalidates the cache "after weld attempt_grasp succeeds or
release fires". For release that is correct (the object is back on the
table, at rest -- exactly PoseNet's training distribution). For an ATTACH it
is actively harmful for the reason above: a refresh immediately after a
successful grasp would render the object mid-lift and get back its resting
z again, this time freshly computed and therefore *more* convincing to a
future reader of a log than a merely-stale cached value would be. This
class does NOT invalidate on attach. It relies entirely on the held-object
check in `get()` (previous paragraph) to keep a held object's position off
the perception path altogether, for as long as it stays held -- "skip the
refresh while holding" and "fall through to oracle for a held object" are
the same rule, applied from two different mechanisms; this class implements
the second because it composes correctly with all THREE call sites
(`run_pick`'s pre-grasp targeting, `run_place`'s post-grasp targeting,
`run_handoff`'s nested pick) without any of them needing to know which
mechanism is in effect.

**Diagnostic logging, never control.** Every actual refresh logs, per prop,
the PoseNet estimate against the oracle ground truth and their Euclidean
delta (`get_all_with_oracle` exposes the same pairing for a caller that
wants it structured rather than just logged). ADR-045's ~1.4e-4 figure is
OpenVINO-vs-PyTorch conversion fidelity, not this; the honest number to
compare these deltas against is ADR-044's held-out MAE (fork 3.2 mm,
water_bottle 2.6 mm, mug 2.8 mm, x/y only). This log is for a human reading
`docs/hardware/m10-phase5-integration.md` after the fact -- it is NEVER read
by any skill's success check, which per ADR-046's Correction 1 stays oracle
everywhere, unconditionally.
"""

from __future__ import annotations

import logging

import mujoco
import numpy as np

logger = logging.getLogger(__name__)

# NOT imported from `bimanual.perception.posenet`: that module does `import
# torch` at module scope, and `torch` is not installed in `ov_env`
# (scripts/requirements-bmptl.txt) -- the one bm-ptl venv with both `mujoco`
# and `openvino`, and therefore the only venv this class can actually run
# in. See `inference.py`'s identical note; duplicated (not imported) for the
# same reason, verified byte-identical to `posenet.py`'s own `PROP_ORDER` as
# of this module.
PROP_ORDER = ["fork", "water_bottle", "mug"]

#: Both arms this project ever has (matches bimanual.sim.grasp.ARMS,
#: duplicated here rather than imported to keep this perception module
#: import-independent of bimanual.sim -- see this module's own import list).
_ARMS = ("A", "B")


def _body_id(model, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id == -1:
        raise ValueError(f"body {name!r} not found in the compiled model")
    return body_id


class CachedPropPositions:
    """Per-(env, weld) cached PoseNet position provider with an oracle
    fallback for out-of-scope or currently-held props.

    Args:
        env: A `TableSettingEnv` constructed with `cameras=["posenet_cam"]`
            (ADR-022's opt-in rendering: `render(camera)` will not implicitly
            work around a missing camera name, and per this module's task
            brief, perception-mode wiring is expected to construct the env
            with `posenet_cam` in its default camera set so the per-render
            cost is visible in the same place the behavioural change is).
        inference: A `PoseNetInference` instance (already compiled).
        weld: Optional `bimanual.sim.grasp.WeldGrasp` bound to the SAME
            `env`. If given, `get()` refuses to serve a perception estimate
            for whichever prop either arm currently holds (see this
            module's docstring). If `None`, the held-object safety check is
            skipped entirely -- only appropriate for a caller with no
            grasping in play (e.g. a pure perception-accuracy probe); every
            real skill call site in this repo supplies a `weld`.
        camera: The camera name to render for each refresh. Default
            `"posenet_cam"`, PoseNet's own training camera (ADR-040/041) --
            deliberately NOT `"front"`, which was measured too small/wide
            for this model's training distribution (`docs/hardware/
            m10-camera-comparison.md`).
    """

    def __init__(self, env, inference, weld=None, camera: str = "posenet_cam") -> None:
        self.env = env
        self.inference = inference
        self.weld = weld
        self.camera = camera

        self._cache: dict[str, np.ndarray] | None = None

        # Per-instance counters, read by a caller (e.g. a verification
        # script) after a skill call to report "N refreshes, M inference
        # calls" -- one inference call per refresh today (a single-camera,
        # single-frame, all-props-at-once model), but counted separately
        # rather than assumed equal, in case a future camera-fusion variant
        # ever calls `inference.predict()` more than once per refresh.
        self.refresh_count = 0
        self.inference_count = 0

        # Transparency log: one entry per (refresh, prop), never consumed by
        # any control decision -- see this module's docstring.
        self.deltas: list[dict] = []

    def invalidate(self) -> None:
        """Drop the current cache generation. The next `get()` call (for any
        prop) triggers exactly one fresh render + `inference.predict()`.
        """
        self._cache = None

    def _oracle_position(self, body_name: str) -> np.ndarray:
        """The SAME privileged read every skill used before M10 Phase 5:
        `env.data.xpos` for `body_name`, world frame, metres. This is the
        ground truth `get()` falls through to -- never itself replaced by
        perception (ADR-046's Correction 1 is about skill success checks,
        which never call this class at all; this method exists so `get()`'s
        own fallback and the diagnostic delta log both read the identical
        ground truth a verification script would).
        """
        body_id = _body_id(self.env.model, body_name)
        return np.array(self.env.data.xpos[body_id], dtype=np.float64, copy=True)

    def _is_held(self, body_name: str) -> bool:
        if self.weld is None:
            return False
        return any(self.weld.is_holding(arm) == body_name for arm in _ARMS)

    def _refresh(self) -> None:
        image = self.env.render(self.camera)
        predictions = self.inference.predict(image)
        self.inference_count += 1
        self.refresh_count += 1
        self._cache = predictions

        for prop, posenet_pos in predictions.items():
            oracle_pos = self._oracle_position(prop)
            delta_m = float(np.linalg.norm(np.asarray(posenet_pos) - oracle_pos))
            entry = {
                "refresh_index": self.refresh_count,
                "prop": prop,
                "oracle_xyz": oracle_pos.tolist(),
                "posenet_xyz": [float(v) for v in posenet_pos],
                "delta_m": delta_m,
            }
            self.deltas.append(entry)
            logger.info(
                "perception refresh #%d: prop=%s oracle=%s posenet=%s delta=%.4f m "
                "(never used for control, ADR-046)",
                self.refresh_count, prop, oracle_pos, posenet_pos, delta_m,
            )

    def get(self, body_name: str) -> np.ndarray | None:
        """Return a cached PoseNet (x, y, z) estimate for `body_name`, or
        `None` to signal "fall through to the oracle read" -- either because
        `body_name` is outside PoseNet's 3-prop scope (`plate`, `spoon`,
        `drawer` were never labelled or predicted, ADR-041), or because
        `body_name` is currently held by either arm (this module's docstring,
        Correction 2). Triggers at most one render+infer per cache
        generation (see `invalidate()`).

        Callers NEVER need to duplicate the held-object check themselves --
        that is the entire point of centralizing it here (this module's
        docstring).
        """
        if self._is_held(body_name):
            return None
        if body_name not in PROP_ORDER:
            return None
        if self._cache is None:
            self._refresh()
        return np.array(self._cache[body_name], dtype=np.float64, copy=True)

    def get_all_with_oracle(self) -> dict[str, dict]:
        """Diagnostics only (never called by a skill): refresh if needed,
        then return, for every one of PoseNet's 3 props, both estimates and
        their delta -- regardless of held state, unlike `get()`. Useful for
        a verification script that wants to report "how far off was
        perception this step" even for a prop `get()` itself would have
        refused to serve.
        """
        if self._cache is None:
            self._refresh()
        out: dict[str, dict] = {}
        for prop in PROP_ORDER:
            posenet_pos = np.array(self._cache[prop], dtype=np.float64, copy=True)
            oracle_pos = self._oracle_position(prop)
            out[prop] = {
                "posenet": posenet_pos,
                "oracle": oracle_pos,
                "delta_m": float(np.linalg.norm(posenet_pos - oracle_pos)),
                "held": self._is_held(prop),
            }
        return out
