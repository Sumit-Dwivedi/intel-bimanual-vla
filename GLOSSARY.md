# Glossary

One line per term. Alphabetical. Added to as terms appear in code — the note that
introduces a term is in [LEARN.md](LEARN.md).

| Term | Meaning |
|------|---------|
| action space | The shape and meaning of what you may pass to `step()`; here a length-`nu` (12) vector of position-actuator targets written straight into `data.ctrl`. |
| actuator | Element that turns a control number into force on a joint; SO-101 uses `<position>` actuators, so the control input is a target angle. |
| body | A rigid link in the kinematic tree; MuJoCo nests them, and each child connects to its parent through a joint. |
| DoF (degree of freedom) | One independent way the system can move; counted by `nv`. A hinge has 1, a free-floating object has 6. |
| env (environment) | The object owning the simulator state and exposing `reset`/`step`/`render`; `TableSettingEnv` is one, though it is not a `gym.Env` (no `Space` objects, no reward). |
| free joint | A joint giving a body full 6-DoF motion relative to the world; contributes 7 to `nq` (3 position + 4 quaternion) but 6 to `nv`. The five props use `<freejoint>`. |
| geom | A geometric shape attached to a body, used for collision, for rendering, or both; counted by `ngeom`. |
| gripper | The two-finger end of the arm; also the name of the joint/actuator that opens and closes the moving jaw. |
| gripper (naming caveat, 1/2) | `<body name="gripper">` (`so101_new_calib.xml`:100) is the physical gripper body, but the joint that moves *it* is `wrist_roll`; the joint/actuator named `gripper` sits on its child body `moving_jaw_so101_v1`. |
| gripper (naming caveat, 2/2) | So a policy output addressed to `gripper` drives the jaw open/closed, **not** the wrist body — indexing `ctrl` by the name `gripper` never rotates the wrist. |
| hinge | A joint that rotates about one fixed axis, like a door — 1 position value and 1 DoF. All six SO-101 joints are hinges. |
| interpenetration | Geoms overlapping more than the contact model intends — usually from bad initial placement, and the solver answers with a large separation impulse that flings them apart. |
| joint | The connection that allows relative motion between a body and its parent, and the variable describing that motion. |
| kp / kv | Proportional and damping gains of a `<position>` actuator — how hard it pulls toward the target angle and how much it resists overshoot. |
| MJCF | MuJoCo XML Configuration Format: the XML source language describing a scene's bodies, joints, geoms, and actuators. |
| `MjData` | The mutable per-timestep state (positions, velocities, controls, contacts) that evolves as the simulation steps. |
| `MjModel` | The immutable compiled scene produced from MJCF; holds the flat arrays MuJoCo simulates against. |
| NaN cascade | One non-finite force makes one state entry `NaN`; because each step reads the previous state, it spreads through `qpos`/`qvel` and never clears — so the stability probe stops at the first sighting. |
| `nbody` | Number of bodies in the compiled model, including the implicit `world` body. |
| `ngeom` | Number of geoms in the compiled model. |
| `nq` | Length of the position vector `qpos`. Equals `nv` only when no joint uses a quaternion. |
| `nu` | Number of actuators, i.e. the length of the control vector `data.ctrl`. |
| `nv` | Length of the velocity vector `qvel`; the true degree-of-freedom count. |
| `qpos` / `qvel` | The position and velocity vectors inside `MjData`, lengths `nq` and `nv`; the env copies both into every observation dict. |
| quaternion | A 4-number encoding of a 3D orientation, constrained to unit length so it carries only 3 independent DoF; the reason `nq` can exceed `nv`. |
| slide joint | A joint that translates along one fixed axis — 1 position value and 1 DoF. The drawer uses one (`drawer_slide`). |
| SO-101 | The 6-joint open-source robot arm used in this project; asset from TheRobotStudio/SO-ARM100. |
| STL | Triangle-mesh file format; the 13 files in `scenes/so101/assets/` supply the arm's shapes. |
| tunneling | An object moving far enough in one timestep that collision detection never sees the surface between start and end, so it passes straight through. |
| `world` / worldbody | The fixed root of the body tree; everything is positioned relative to it and it never moves. |
