"""TableSettingEnv: MuJoCo wrapper for the dual-SO-101 table-setting scene (M02).

Implements the `TableSettingEnv` contract described in
`src/bimanual/sim/__init__.py` and `ARCHITECTURE.md` section 1's component
table:

    reset(seed) -> obs
    step(action) -> (obs, done, info)
    render(camera) -> rgb ndarray
    get_state() -> ndarray
    close()

Per ADR-020, MuJoCo cannot be imported on the developer's laptop (Windows
Smart App Control blocks the unsigned `mujoco.dll`); this module is developed
here but only ever *run* on bm-ptl. Importing `bimanual.sim.env` itself will
raise ImportError on the laptop the moment `import mujoco` executes -- that is
expected and is why `bimanual.sim.__init__` stays side-effect free (see its
docstring) so `import bimanual` keeps working everywhere.

A note on "seed" for a robotics newcomer: MuJoCo's physics stepping itself is
deterministic given a fixed model and initial state -- there is no per-step
dice roll unless noise is explicitly added. What *is* random is anything we
layer on top: initial joint-angle jitter, domain randomization (M07), etc.
`reset(seed=...)` therefore does two things: (1) seeds a `numpy.random.Generator`
owned by this env so that any future randomized reset logic draws from a
reproducible stream, and (2) resets MuJoCo's internal data (qpos, qvel, time,
contacts, ...) via `mj_resetData`, then -- if the compiled model defines a
keyframe named "home" -- overlays that keyframe's qpos/qvel/ctrl via
`mj_resetDataKeyframe` (ADR-026). Two `reset(seed=0)` calls in fresh
processes therefore produce byte-identical observations, which is the
reproducibility property ADR-012 depends on later.

ADR-026 note: the scene's compiled default (qpos0, all arm joints at 0 rad,
unmodified per ADR-016) is NOT the pose this env resets to. It was measured
to put both arms 34 contacts deep into each other (29 armA<->armB, deepest
-0.0597 m) because both arms' zero-angle configuration extends fully
forward into the shared handoff band. `scripts/gen_dual_scene.py` bakes an
additional named "home" keyframe (arms folded back) into the generated
scene, and `reset()` applies it explicitly -- `qpos0` itself is left alone
so the upstream asset stays byte-for-byte unmodified (ADR-016). A
`scene_path=` that has no "home" key (e.g. a bare single-arm scene) falls
back to the plain `mj_resetData` pose rather than raising, so this class
stays usable outside the packaged dual-arm scene.

Camera rendering is opt-in (M02 refactor). `reset()`/`step()` return camera
pixels in `obs` ONLY for the camera names explicitly requested -- either the
`cameras=` list passed to `__init__` (the instance default) or a per-call
`cameras=` override. Passing `cameras=None` everywhere (the default) is the
"state-only" fast path: no image is rendered and, more importantly, no
offscreen GL renderer is even constructed, because constructing
`mujoco.Renderer` itself costs real wall-clock (GL/EGL setup) independent of
how many frames are later rendered through it. This matters because M09
(demonstration collection) and M08's evaluation harness both read privileged
state (ADR-005) and never touch pixels -- forcing a renderer to exist for
those call sites would waste GL setup and per-camera render time on every
`reset()`/`step()` for streams nobody reads. See
`docs/hardware/m02-render-cost.md` for measured numbers and ADR-022 for the
decision record.
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

# Default scene shipped with the package. Resolved relative to this file, NOT
# the process working directory -- a caller may `import bimanual.sim.env` from
# any cwd (see module docstring / task instructions) and this must still find
# the asset.
_DEFAULT_SCENE_PATH = (
    pathlib.Path(__file__).resolve().parent / "assets" / "so101_dual_table.xml"
)

# NOTE ON CAMERA VALIDATION. There is deliberately no hardcoded list of
# expected camera names here. Validation is always performed against
# `self._camera_names`, discovered from the compiled model via `mj_id2name`
# in `__init__`. A hardcoded tuple would silently re-break the property
# established in commit 80f7316 ("standardize camera obs keys to match MJCF
# camera names"): that adding a camera to the MJCF requires no env.py edit.
# The model is the single source of truth.


class TableSettingEnv:
    """Thin MuJoCo wrapper around the dual-SO-101 table-setting scene.

    Not a full Gym/Gymnasium environment (no action/observation `Space`
    objects, no reward) -- PLAN.md M02 only asks for reset/step/render/
    get_state/close. A Gym-compatible shim can be layered on top later if a
    training library needs one, without touching this class.
    """

    def __init__(
        self,
        scene_path: str | pathlib.Path | None = None,
        cameras: list[str] | None = None,
        render_width: int = 640,
        render_height: int = 480,
    ) -> None:
        """Load the MJCF scene. Camera rendering is opt-in (see module docstring).

        Args:
            scene_path: Path to an MJCF scene file. Defaults to the packaged
                `so101_dual_table.xml` (M02's dual-arm table scene), resolved
                relative to this source file's location.
            cameras: List of camera names to render into `obs` on every
                `reset()`/`step()` call that does not itself pass a
                `cameras=` override. `None` (the default) means NO rendering
                -- the fast, state-only path: `obs` will contain only 'qpos'
                and 'qvel'. Every name is validated against the cameras
                actually discovered in the compiled model (see
                `_camera_names` below); an unknown name raises `ValueError`
                naming the offending camera and listing every valid name.
            render_width, render_height: Offscreen render resolution, used
                only if/when a renderer is actually constructed (see
                `_ensure_renderer`). Clamped internally to the scene's
                declared `<visual><global offwidth/offheight/></visual>`
                framebuffer size (the scene declares 1280x720; see
                `scripts/gen_dual_scene.py`'s TODO(M02) comment and
                `scripts/probe_render.py` for why this clamp exists --
                requesting more than the framebuffer size raises inside
                MuJoCo before any GL call happens).
        """
        self.scene_path = pathlib.Path(scene_path) if scene_path is not None else _DEFAULT_SCENE_PATH
        if not self.scene_path.exists():
            raise FileNotFoundError(
                f"MJCF scene not found at {self.scene_path}. "
                "Pass scene_path= explicitly, or check the package install."
            )

        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)

        # Discover every camera defined in the scene by name, rather than
        # hardcoding a list, so this class keeps working if the
        # hand-authored camera section of the scene changes. This is the
        # ONE source of truth for camera-name validation everywhere in this
        # class -- see the camera-validation note at the top of this module.
        self._camera_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            for i in range(self.model.ncam)
        ]

        # Record the requested offscreen size now (used only if a renderer
        # is later actually constructed -- see _ensure_renderer). Clamping
        # to the compiled framebuffer happens at that point too, since it
        # needs model.vis, which is already available here but the clamp
        # itself has no side effect worth paying before it's needed.
        fb_w = int(self.model.vis.global_.offwidth)
        fb_h = int(self.model.vis.global_.offheight)
        self._render_width = min(render_width, fb_w)
        self._render_height = min(render_height, fb_h)

        # Opt-in camera rendering (M02 refactor -- see module docstring and
        # ADR-022). `cameras=None` is the fast, state-only default: no
        # renderer is constructed at all until something actually asks to
        # render (an explicit `render()` call, or a `reset()`/`step()` that
        # requests cameras via the instance default or a per-call override).
        # Stored as a defensive copy so a caller mutating the list they
        # passed in cannot retroactively change this instance's default.
        self._validate_cameras(cameras)
        self._default_cameras: list[str] | None = list(cameras) if cameras is not None else None

        # mujoco.Renderer owns a GL/offscreen buffer and its construction
        # itself costs real wall-clock (GL/EGL setup) independent of how
        # many frames are ever rendered through it -- see
        # docs/hardware/m02-render-cost.md. Lazily constructed on first
        # actual render via `_ensure_renderer()`, not here, so the
        # state-only path (`cameras=None` everywhere) never pays that cost.
        self._renderer: mujoco.Renderer | None = None

        # Owned RNG, seeded in reset(). Nothing in step()/render() consumes
        # it, and M07's shipped `ScenarioRandomizer` (ADR-048) does not
        # either -- it seeds its OWN `np.random.default_rng(seed)` directly
        # from the same `seed` int `reset()` receives, rather than reaching
        # into this instance attribute, so `randomize(seed)` stays a pure
        # function of `seed` alone (testable and reproducible with no `env`
        # object at all). `self.np_random` is kept anyway as the documented
        # seam ADR-012 reserved ("nothing in the control or perception path
        # may consume the randomization RNG" outside this seam) in case a
        # future caller wants an env-bound stream instead of a pure one.
        self.np_random: np.random.Generator | None = None
        self._seed: int | None = None

    # ------------------------------------------------------------------
    # Core contract
    # ------------------------------------------------------------------

    def reset(
        self,
        seed: int = 0,
        cameras: list[str] | None = None,
        randomizer=None,
    ) -> dict:
        """Reset physics to the model's compiled initial state.

        Obs schema: always 'qpos' (nq,) float64 and 'qvel' (nv,) float64.
        Camera entries appear under their bare MJCF name (e.g. 'front',
        'overhead', ...) as an (H, W, 3) uint8 RGB ndarray, but ONLY for the
        cameras actually requested this call -- see the `cameras` arg below
        and the module docstring's "opt-in" note.

        Args:
            seed: Seeds `self.np_random` reproducibly. Two `reset(seed=0)`
                calls in independent processes produce byte-identical
                `qpos`/`qvel`/camera images because MuJoCo's own state reset
                (`mj_resetData` plus the "home" keyframe overlay, see below)
                is itself deterministic; the seed governs only this env's
                own RNG stream for future randomized use.
            cameras: If not `None`, overrides the instance's default camera
                list (set in `__init__`) FOR THIS CALL ONLY -- the instance
                default itself is never mutated by this argument. If `None`
                (the default), the instance default is used as-is. Every
                name is validated against the model's discovered cameras;
                an unknown name raises `ValueError` naming the offending
                camera and listing every valid name.
            randomizer: **M07 (ADR-048), opt-in, `None` by default.** `None`
                (the default) means this method is BYTE-IDENTICAL to its
                pre-M07 behaviour for every `seed` -- this is deliberate,
                not an oversight: `scripts/verify_adr038_skills.py:19` and
                `tests/test_skills.py:76` both call `reset(seed=N)` today and
                depend on it producing the exact fixed baseline scene (the
                30-point regression evidence and the 4-passed/4-failed
                pytest baseline). Making randomization the default the
                moment a caller passes a `seed` would silently invalidate
                both without raising anything -- the same "default OFF,
                explicit opt-in" shape ADR-046 already established for
                perception (`ScriptedSkillExecutor(inference=None)`).
                When not `None`, `randomizer` must implement
                `randomize(seed) -> dict[str, tuple[float, float]]`
                (`bimanual.sim.randomization.ScenarioRandomizer` is the one
                shipped implementation) mapping a prop's body name to an
                (dx, dy) offset in METRES, applied ON TOP OF whatever x/y the
                deterministic reset above already wrote via the compiled
                model's own "home" keyframe -- never by regenerating the
                scene XML (ADR-038 found that breaks `handoff`). Applied via
                a direct `data.qpos` write to that prop's own free joint,
                exactly the runtime mechanism `scripts/generate_posenet_data.py`
                and the pre-M07 audit's own probe already used, followed by
                one extra `mj_forward` to recompute derived kinematics
                (`data.xpos` etc.) before this call builds its observation.
                A prop name `randomizer.randomize()` returns that has no
                `"<name>_free"` joint in the compiled model raises
                `KeyError` immediately -- fail loud, not a silently-ignored
                offset.

        Returns:
            Observation dict; see the obs schema note above.
        """
        self._seed = seed
        self.np_random = np.random.default_rng(seed)

        mujoco.mj_resetData(self.model, self.data)
        # ADR-026: apply the "home" keyframe (arms folded back) if the
        # compiled model defines one, instead of leaving qpos at qpos0
        # (both arms fully extended -- measured to interpenetrate, see
        # module docstring). `mj_name2id` returns -1 for a missing name
        # rather than raising, so a scene without a "home" key (not the
        # packaged dual-arm scene) silently keeps the plain mj_resetData
        # pose instead of erroring.
        home_key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if home_key_id != -1:
            mujoco.mj_resetDataKeyframe(self.model, self.data, home_key_id)
        mujoco.mj_forward(self.model, self.data)

        # M07 (ADR-048): opt-in scoped randomization, applied AFTER the
        # deterministic baseline above and BEFORE the observation is built,
        # so a caller that never passes `randomizer=` sees no change at all
        # (the `if` below is simply never entered) -- see this method's
        # `randomizer` arg docstring for why that byte-identical guarantee
        # matters.
        if randomizer is not None:
            offsets = randomizer.randomize(seed)
            for prop_name, (dx, dy) in offsets.items():
                adr = self._prop_free_joint_qpos_adr(prop_name)
                self.data.qpos[adr] += dx
                self.data.qpos[adr + 1] += dy
            # Recompute data.xpos/xquat/etc. from the modified qpos -- the
            # skill layer (skills_scripted.py) reads xpos, not qpos, for
            # every targeting decision, so skipping this would leave every
            # randomized prop's derived pose stale until the first step().
            mujoco.mj_forward(self.model, self.data)

        effective_cameras = self._resolve_cameras(cameras)
        return self._build_obs(effective_cameras)

    def _prop_free_joint_qpos_adr(self, prop_name: str) -> int:
        """Resolve `prop_name`'s free-joint `qpos` address by name.

        Every movable prop in the packaged scene (`gen_dual_scene.py`) has a
        free joint literally named `"<body_name>_free"` -- the same lookup
        `scripts/generate_posenet_data.py` and
        `scripts/probe_pre_m07_audit.py` already perform, repeated here so
        `reset()`'s randomizer hook needs no hardcoded prop list of its own
        (the model, via `randomizer.randomize()`'s returned keys, is the
        source of truth for WHICH props get offset). `mj_name2id` returns
        -1 for an unknown joint name rather than raising -- turned into a
        loud `KeyError` here instead of a silently-ignored offset.
        """
        joint_name = f"{prop_name}_free"
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid == -1:
            raise KeyError(
                f"randomizer offset requested for prop {prop_name!r}, but this model has "
                f"no free joint named {joint_name!r}. Valid prop names have a "
                f"'<name>_free' joint in the compiled scene."
            )
        return int(self.model.jnt_qposadr[jid])

    def step(self, action, cameras: list[str] | None = None) -> tuple[dict, bool, dict]:
        """Advance physics by one timestep under `action`.

        Obs schema: identical to `reset()`'s -- see that docstring's "Obs
        schema" note, including the opt-in `cameras` override semantics
        (not `None` overrides the instance default FOR THIS CALL ONLY,
        without mutating it).

        Args:
            action: length-`model.nu` (12: 6 actuators x 2 arms) array-like
                of position-actuator targets, written directly into
                `data.ctrl` and held constant for exactly one `mj_step`.
            cameras: Same per-call override semantics as `reset()`'s
                `cameras` argument.

        Returns:
            (obs, done, info) where `obs` has the same shape as `reset()`'s
            return value, `done` is always False (this env defines no
            terminal condition -- that is the Coordinator/skill layer's job
            in M06, not the physics wrapper's), and `info` is always `{}`
            (reserved for future per-step diagnostics).
        """
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape[0] != self.model.nu:
            raise ValueError(
                f"action has length {action.shape[0]}, expected {self.model.nu} "
                f"(one target per actuator: {self.model.nu} actuators total)."
            )

        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)

        effective_cameras = self._resolve_cameras(cameras)
        return self._build_obs(effective_cameras), False, {}

    def render(self, camera: str = "front") -> np.ndarray:
        """Render one named camera and return an (H, W, 3) uint8 RGB array.

        This is the explicit escape hatch: it renders regardless of the
        `cameras=` opt-in setting configured in `__init__`/`reset()`/
        `step()`. `scripts/view_scene.py` and other probes call this
        directly when they want one specific frame without opting the whole
        env into per-step rendering.
        """
        if camera not in self._camera_names:
            raise ValueError(
                f"unknown camera {camera!r}; scene defines {self._camera_names}"
            )
        renderer = self._ensure_renderer()
        renderer.update_scene(self.data, camera=camera)
        return renderer.render()

    def get_state(self) -> np.ndarray:
        """Return the full internal MuJoCo state as one flat float64 ndarray.

        Uses `mujoco.mj_getState` with `mjSTATE_FULLPHYSICS`, which covers
        simulation time, qpos, qvel and actuator activation state (act) --
        everything needed to exactly resume physics from this point, as
        opposed to `get_obs`'s qpos/qvel-only view meant for a policy.
        """
        spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        size = mujoco.mj_stateSize(self.model, spec)
        state = np.empty(size, dtype=np.float64)
        mujoco.mj_getState(self.model, self.data, state, spec)
        return state

    def close(self) -> None:
        """Release the offscreen renderer's GL resources, if one was ever built."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _validate_cameras(self, cameras: list[str] | None) -> None:
        """Raise ValueError if `cameras` names anything not in the model.

        Validates against `self._camera_names` (discovered from the
        compiled model), never against a hardcoded list -- see the
        camera-validation note at the top of this module for why.
        """
        if cameras is None:
            return
        for name in cameras:
            if name not in self._camera_names:
                raise ValueError(
                    f"unknown camera {name!r}; valid cameras are {self._camera_names}"
                )

    def _resolve_cameras(self, cameras: list[str] | None) -> list[str] | None:
        """Resolve a reset()/step() per-call `cameras` argument.

        `cameras is None` means "no override": fall back to the instance
        default set in `__init__` (itself possibly `None`, i.e. state-only).
        A non-`None` value is validated fresh (the instance default was
        already validated once in `__init__`, but an override is new input
        and must be checked again) and used for THIS CALL ONLY -- the
        instance default is never written to here.
        """
        if cameras is None:
            return self._default_cameras
        self._validate_cameras(cameras)
        return list(cameras)

    def _ensure_renderer(self) -> mujoco.Renderer:
        """Lazily construct the offscreen renderer on first actual use.

        `mujoco.Renderer.__init__` performs GL/offscreen-buffer setup that
        costs real wall-clock (see docs/hardware/m02-render-cost.md) even
        before any frame is rendered. Deferring this until a render is
        actually requested is what makes `cameras=None` genuinely fast
        rather than "fast except for the renderer nobody asked for."
        """
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self._render_height, width=self._render_width
            )
        return self._renderer

    def _build_obs(self, cameras: list[str] | None) -> dict:
        # Always include privileged state (ADR-005: qpos/qvel are free --
        # they come straight out of MuJoCo's data struct with no rendering
        # involved). Camera keys are the MJCF camera names themselves (no
        # "_rgb" suffix), added ONLY for the cameras this call actually
        # requested (see `_resolve_cameras`) -- this is the opt-in contract
        # this refactor exists to establish.
        obs = {
            "qpos": np.array(self.data.qpos, dtype=np.float64, copy=True),
            "qvel": np.array(self.data.qvel, dtype=np.float64, copy=True),
        }
        if cameras:
            for name in cameras:
                obs[name] = self.render(name)
        return obs

    def __enter__(self) -> "TableSettingEnv":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


if __name__ == "__main__":
    # Minimal self-test / smoke run, updated for the opt-in camera refactor:
    # (1) the state-only fast path (cameras=None, the default -- no renderer
    #     ever constructed), (2) a reset() with an explicit per-call camera
    #     override, and (3) render() as the always-available escape hatch.
    # This is a manual developer check, not the module's automated test
    # (there is no local test path per ADR-020 -- run this on bm-ptl).
    env = TableSettingEnv()
    print(f"scene: {env.scene_path}")
    print(f"nq={env.model.nq} nv={env.model.nv} nu={env.model.nu} "
          f"ncam={env.model.ncam} cameras={env._camera_names}")

    obs = env.reset(seed=0)
    print("reset(seed=0), cameras=None (state-only, default):")
    for key, val in obs.items():
        print(f"  obs[{key!r}].shape = {val.shape}  dtype={val.dtype}")
    assert env._renderer is None, "state-only reset() must not construct a renderer"
    print(f"  renderer constructed? {env._renderer is not None} (expected False)")

    zero_action = np.zeros(env.model.nu, dtype=np.float64)
    obs, done, info = env.step(zero_action, cameras=["front"])
    print("step(zero_action, cameras=['front']) -- per-call override:")
    for key, val in obs.items():
        print(f"  obs[{key!r}].shape = {val.shape}  dtype={val.dtype}")
    assert env._renderer is not None, "requesting a camera must construct the renderer"

    state = env.get_state()
    print(f"get_state() -> shape {state.shape}, dtype {state.dtype}")

    frame = env.render("front")
    print(f"render('front') -> shape {frame.shape}, dtype {frame.dtype} (escape hatch, always works)")

    try:
        env.reset(seed=0, cameras=["not_a_camera"])
    except ValueError as exc:
        print(f"unknown-camera override correctly raised: {exc}")
    else:
        raise AssertionError("expected ValueError for unknown camera override")

    env.close()
    print("OK")
