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
| 06 | [IK and closed-loop skills](docs/learn/06-ik-and-skills.md) | Forward vs. inverse kinematics, why SO-101's 5 positioning DoF cannot satisfy a 6-DoF pose target, the observe→IK→`step()`→check loop behind `control/ik.py` and `control/skills_scripted.py`, and why gripper timing makes `handoff` the hardest skill — *pre-note, written before M06 was built* |
| 07 | [The day things broke](docs/learn/07-the-day-things-broke.md) | M06 day narrative: the 6 cm inter-arm interpenetration the `home` keyframe fixed (ADR-026), collision-blind IK solving through the table (ADR-024) and the waypoint staging that fixed it (ADR-027), the four-fix grasp ladder that failed cumulatively, and MuJoCo's convex-hull collapse of the jaw meshes (ADR-028) — *written after M06a ran* |
| 08 | [When first principles plateau](docs/learn/08-when-first-principles-plateau.md) | The methodology lesson from note 07: how to recognise that reasoning has stopped working, instrumenting before fixing, searching the tool's issue tracker, shape-vs-parameters — and the defensible two-hour cost of missing the signal |
