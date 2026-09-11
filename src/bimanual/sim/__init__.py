"""MuJoCo simulation: TableSettingEnv and domain randomization (M02, M07).

TableSettingEnv wraps the dual-SO-101 table-setting MJCF scene with
reset(seed), step(action), render(camera), get_state(). Randomizer is a pure
function seed -> SceneConfig (ARCHITECTURE.md ADR-012); nothing in the
control or perception path may consume the randomization RNG.

Per ARCHITECTURE.md ADR-020, MuJoCo cannot be imported on this laptop
(Windows Smart App Control blocks the unsigned mujoco.dll). All simulation
work in this package is developed and run on bm-ptl. Importing
`bimanual.sim` itself must stay side-effect free so `import bimanual` keeps
working on the laptop; only importing the MuJoCo-backed submodules added in
M02 will fail here, as expected.

`assets/` (populated in M02) holds MJCF/mesh files for the assembled scene;
it is not a Python package.
"""
