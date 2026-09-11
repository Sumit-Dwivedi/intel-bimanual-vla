# Learning Index

Short notes explaining what each module does and why, written for a software engineer
with no robotics background. One note per module, in build order. Terms are defined in
[GLOSSARY.md](GLOSSARY.md).

| # | Note | Covers |
|---|------|--------|
| 01 | [MuJoCo scene basics](docs/learn/01-mujoco-scene-basics.md) | MJCF, model compilation, `nq`/`nv`/`nu`/`nbody`/`ngeom`, actuators in `scenes/so101/` |
| 02 | [Env wrapper and physics stability](docs/learn/02-mujoco-env-and-physics.md) | `TableSettingEnv` reset/step/render/get_state, free joints vs hinges (`nq=48`, `nv=43`), the 1000-step zero-action probe |
