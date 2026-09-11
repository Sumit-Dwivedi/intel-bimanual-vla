# Learning Index

Short notes explaining what each module does and why, written for a software engineer
with no robotics background. One note per module, in build order. Terms are defined in
[GLOSSARY.md](GLOSSARY.md).

| # | Note | Covers |
|---|------|--------|
| 01 | [MuJoCo scene basics](docs/learn/01-mujoco-scene-basics.md) | MJCF, model compilation, `nq`/`nv`/`nu`/`nbody`/`ngeom`, actuators in `scenes/so101/` |
| 02 | [Env wrapper and physics stability](docs/learn/02-mujoco-env-and-physics.md) | `TableSettingEnv` reset/step/render/get_state, free joints vs hinges (`nq=48`, `nv=43`), the 1000-step zero-action probe |
| 03 | [OpenVINO conversion](docs/learn/03-openvino-conversion.md) | Model conversion to IR (`.xml`+`.bin`), `compile_model` and device plugins, why M03 smoke-tests the path before the real model (ADR-014) — *pre-note, written before M03 ran* |
| 04 | [CommandSource abstraction](docs/learn/04-command-source.md) | Strategy pattern + dependency injection over `CommandSource`/`CommandEvent`, why the VLA never sees audio (ADR-002) and the grep gate that enforces it — *pre-note, written before M04 was built* |
| 05 | [Rule grounder](docs/learn/05-rule-grounder.md) | `ground(CommandEvent, SceneBelief) -> TaskPlan` as a DSL front end, the `open_drawer`/`pick`/`place`/`handoff`/`pour` vocabulary with explicit arm assignment, rules as the deterministic floor under ADR-003, and failing loudly via `UngroundedCommandError` — *pre-note, written before M05 was built* |
