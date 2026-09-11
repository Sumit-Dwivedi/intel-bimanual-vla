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
contacts, ...) back to the model's compiled defaults via `mj_resetData`. Two
`reset(seed=0)` calls in fresh processes therefore produce byte-identical
observations, which is the reproducibility property ADR-012 depends on later.
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

# Cameras rendered into every observation by default. Discovered from the
# model at load time in practice (see `_camera_names`), but declared here for
# reference: this is the M02 scene's full camera set (ARCHITECTURE.md section
# 1, "overhead camera, front camera, per-arm wrist cameras", plus the M02(e)
# drawer_view camera added for demo visibility of the drawer-open action).
_EXPECTED_CAMERAS = ("overhead", "front", "armA_wrist", "armB_wrist", "drawer_view")


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
        render_width: int = 640,
        render_height: int = 480,
    ) -> None:
        """Load the MJCF scene and construct one offscreen renderer per camera.

        Args:
            scene_path: Path to an MJCF scene file. Defaults to the packaged
                `so101_dual_table.xml` (M02's dual-arm table scene), resolved
                relative to this source file's location.
            render_width, render_height: Offscreen render resolution. Clamped
                internally to the scene's declared `<visual><global
                offwidth/offheight/></visual>` framebuffer size (the scene
                declares 1280x720; see `scripts/gen_dual_scene.py`'s
                TODO(M02) comment and `scripts/probe_render.py` for why this
                clamp exists -- requesting more than the framebuffer size
                raises inside MuJoCo before any GL call happens).
        """
        self.scene_path = pathlib.Path(scene_path) if scene_path is not None else _DEFAULT_SCENE_PATH
        if not self.scene_path.exists():
            raise FileNotFoundError(
                f"MJCF scene not found at {self.scene_path}. "
                "Pass scene_path= explicitly, or check the package install."
            )

        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)

        # Clamp requested render size to the scene's compiled offscreen
        # framebuffer, exactly as scripts/probe_render.py does, so a caller
        # asking for more than the scene declares gets a smaller image
        # instead of a MuJoCo exception that looks like a GL failure.
        fb_w = int(self.model.vis.global_.offwidth)
        fb_h = int(self.model.vis.global_.offheight)
        self._render_width = min(render_width, fb_w)
        self._render_height = min(render_height, fb_h)

        # Discover every camera defined in the scene by name, rather than
        # hardcoding _EXPECTED_CAMERAS, so this class keeps working if the
        # hand-authored camera section of the scene changes.
        self._camera_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            for i in range(self.model.ncam)
        ]

        # One Renderer per camera. mujoco.Renderer owns its own GL/offscreen
        # buffer; each render() call re-points it at a different camera via
        # update_scene(camera=...), so a single renderer instance is reused
        # across cameras rather than allocating one per camera name.
        self._renderer = mujoco.Renderer(
            self.model, height=self._render_height, width=self._render_width
        )

        # Owned RNG, seeded in reset(). Nothing in step()/render() consumes
        # it yet -- it exists so M07's Randomizer has a documented, seeded
        # source to draw from without reaching into global numpy state
        # (ARCHITECTURE.md ADR-012: "nothing in the control or perception
        # path may consume the randomization RNG" outside this seam).
        self.np_random: np.random.Generator | None = None
        self._seed: int | None = None

    # ------------------------------------------------------------------
    # Core contract
    # ------------------------------------------------------------------

    def reset(self, seed: int = 0) -> dict:
        """Reset physics to the model's compiled initial state.

        Obs schema (identical to `step()`'s, and derived from the scene's
        cameras at load time, not hardcoded -- see `_build_obs`): 'qpos'
        (nq,) float64, 'qvel' (nv,) float64, plus one `<camera_name>` key per
        camera defined in the MJCF (currently 'overhead', 'front',
        'armA_wrist', 'armB_wrist', 'drawer_view'), each an (H, W, 3) uint8
        RGB ndarray.

        Args:
            seed: Seeds `self.np_random` reproducibly. Two `reset(seed=0)`
                calls in independent processes produce byte-identical
                `qpos`/`qvel`/camera images because MuJoCo's own state reset
                (`mj_resetData`) is itself deterministic; the seed governs
                only this env's own RNG stream for future randomized use.

        Returns:
            Observation dict; see the obs schema note above.
        """
        self._seed = seed
        self.np_random = np.random.default_rng(seed)

        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        return self._build_obs()

    def step(self, action) -> tuple[dict, bool, dict]:
        """Advance physics by one timestep under `action`.

        Obs schema: identical to `reset()`'s -- see that docstring's "Obs
        schema" note.

        Args:
            action: length-`model.nu` (12: 6 actuators x 2 arms) array-like
                of position-actuator targets, written directly into
                `data.ctrl` and held constant for exactly one `mj_step`.

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

        return self._build_obs(), False, {}

    def render(self, camera: str = "front") -> np.ndarray:
        """Render one named camera and return an (H, W, 3) uint8 RGB array."""
        if camera not in self._camera_names:
            raise ValueError(
                f"unknown camera {camera!r}; scene defines {self._camera_names}"
            )
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

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
        """Release the offscreen renderer's GL resources."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _build_obs(self) -> dict:
        # Obs keys are the MJCF camera names themselves (no "_rgb" suffix),
        # derived from `self._camera_names` (discovered from the model, see
        # __init__) rather than hardcoded -- adding a camera to the scene
        # (e.g. M02(e)'s drawer_view) therefore shows up in obs automatically
        # with no change needed here.
        obs = {
            "qpos": np.array(self.data.qpos, dtype=np.float64, copy=True),
            "qvel": np.array(self.data.qvel, dtype=np.float64, copy=True),
        }
        for name in self._camera_names:
            obs[name] = self.render(name)
        return obs

    def __enter__(self) -> "TableSettingEnv":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


if __name__ == "__main__":
    # Minimal self-test / smoke run: reset, step once, print observation
    # shapes, and confirm get_state()/render() work. This is a manual
    # developer check, not the module's automated test (there is no local
    # test path per ADR-020 -- run this on bm-ptl).
    env = TableSettingEnv()
    obs = env.reset(seed=0)
    print(f"scene: {env.scene_path}")
    print(f"nq={env.model.nq} nv={env.model.nv} nu={env.model.nu} "
          f"ncam={env.model.ncam} cameras={env._camera_names}")
    for key, val in obs.items():
        print(f"  obs[{key!r}].shape = {val.shape}  dtype={val.dtype}")

    zero_action = np.zeros(env.model.nu, dtype=np.float64)
    obs, done, info = env.step(zero_action)
    print(f"after 1 step: done={done} info={info}")

    state = env.get_state()
    print(f"get_state() -> shape {state.shape}, dtype {state.dtype}")

    frame = env.render("front")
    print(f"render('front') -> shape {frame.shape}, dtype {frame.dtype}")

    env.close()
    print("OK")
