# Glossary

One line per term. Alphabetical. Added to as terms appear in code — the note that
introduces a term is in [LEARN.md](LEARN.md).

| Term | Meaning |
|------|---------|
| action space | The shape and meaning of what you may pass to `step()`; here a length-`nu` (12) vector of position-actuator targets written straight into `data.ctrl`. |
| actuator | Element that turns a control number into force on a joint; SO-101 uses `<position>` actuators, so the control input is a target angle. |
| body | A rigid link in the kinematic tree; MuJoCo nests them, and each child connects to its parent through a joint. |
| closed-loop control | Observe state, compute the next targets, apply, observe again — repeating until the goal is met or a step budget expires. The re-observation is what absorbs friction, small collisions and timing error; open-loop sends targets once and hopes. |
| `CommandEvent` | The only type allowed to cross out of `src/bimanual/command/`: a command as `text` plus `timestamp`, `source_id`, optional `confidence` and `raw_meta`. Never audio (ADR-002). |
| `CommandSource` | Abstract base for "where a command came from", with a non-blocking `poll() -> CommandEvent \| None` and `close()`; `TextCommandSource` and `VoiceCommandSource` are interchangeable behind it. Callers are handed one and never branch on which. |
| `compile_model` | OpenVINO call that hands IR to one device's plugin and returns an executable for that device; the backend pass of the toolchain. Compiling for CPU, GPU and NPU from the same IR yields three different executables. |
| device plugin | The per-device backend inside OpenVINO (CPU, GPU, NPU here) that turns IR into kernels for that silicon and decides what it will and will not accept. |
| DoF (degree of freedom) | One independent way the system can move; counted by `nv`. A hinge has 1, a free-floating object has 6. |
| end-effector | The business end of the arm — the point IK aims at. On SO-101 the intended one is the gripper's grasp point, which is *not* the same thing as `<body name="gripper">` (see the gripper naming caveats). |
| env (environment) | The object owning the simulator state and exposing `reset`/`step`/`render`; `TableSettingEnv` is one, though it is not a `gym.Env` (no `Space` objects, no reward). |
| forward kinematics | Joint angles → end-effector pose. Always one answer, always cheap; it is evaluating an expression, and MuJoCo does it for you every step. |
| free joint | A joint giving a body full 6-DoF motion relative to the world; contributes 7 to `nq` (3 position + 4 quaternion) but 6 to `nv`. The five props use `<freejoint>`. |
| geom | A geometric shape attached to a body, used for collision, for rendering, or both; counted by `ngeom`. |
| grep gate | A done-when condition expressed as a search that must return zero matches — e.g. M04's `grep -ri "audio\|pcm\|wav\|microphone" src/bimanual/language src/bimanual/policy`. Makes an architectural boundary mechanically checkable instead of a convention. |
| gripper | The two-finger end of the arm; also the name of the joint/actuator that opens and closes the moving jaw. |
| gripper (naming caveat, 1/2) | `<body name="gripper">` (`so101_new_calib.xml`:100) is the physical gripper body, but the joint that moves *it* is `wrist_roll`; the joint/actuator named `gripper` sits on its child body `moving_jaw_so101_v1`. |
| gripper (naming caveat, 2/2) | So a policy output addressed to `gripper` drives the jaw open/closed, **not** the wrist body — indexing `ctrl` by the name `gripper` never rotates the wrist. |
| grounder | The component mapping one natural-language command to an executable plan: `ground(CommandEvent, SceneBelief) -> TaskPlan`. `RuleGrounder` is the demo default; `VlmGrounder` is a stretch behind the same interface (ADR-003). |
| hinge | A joint that rotates about one fixed axis, like a door — 1 position value and 1 DoF. All six SO-101 joints are hinges. |
| interpenetration | Geoms overlapping more than the contact model intends — usually from bad initial placement, and the solver answers with a large separation impulse that flings them apart. |
| inverse kinematics (IK) | End-effector pose → joint angles: solving for *x* rather than evaluating. Often several solutions (elbow up/down), so you take the one nearest the current pose; with SO-101's 5 positioning DoF against a 6-DoF task space, position *and* orientation generally cannot both be satisfied. |
| IR (OpenVINO Intermediate Representation) | OpenVINO's device-neutral model format: an `.xml` holding the graph topology and a `.bin` holding the weights. Analogous to LLVM IR — one front end per framework, one backend per device. |
| joint | The connection that allows relative motion between a body and its parent, and the variable describing that motion. |
| kp / kv | Proportional and damping gains of a `<position>` actuator — how hard it pulls toward the target angle and how much it resists overshoot. |
| manipulation primitive | A short, self-contained unit of physical action (1–3 s of closed-loop control) that a skill name refers to — the level at which this project can actually learn or script behaviour (ADR-001). |
| MJCF | MuJoCo XML Configuration Format: the XML source language describing a scene's bodies, joints, geoms, and actuators. |
| `MjData` | The mutable per-timestep state (positions, velocities, controls, contacts) that evolves as the simulation steps. |
| `MjModel` | The immutable compiled scene produced from MJCF; holds the flat arrays MuJoCo simulates against. |
| model conversion | Translating a trained model from its training framework (PyTorch, ONNX, TensorFlow) into OpenVINO IR. It is a graph translation, not retraining; small numeric differences from the original are expected, which is why M03 records max absolute deviation. |
| NaN cascade | One non-finite force makes one state entry `NaN`; because each step reads the previous state, it spreads through `qpos`/`qvel` and never clears — so the stability probe stops at the first sighting. |
| `nbody` | Number of bodies in the compiled model, including the implicit `world` body. |
| `ngeom` | Number of geoms in the compiled model. |
| NPU (Neural Processing Unit) | A fixed-function accelerator for neural-network math, separate from CPU and GPU; on bm-ptl it is the Panther Lake NPU5010, exposed to OpenVINO as `NPU` (Intel AI Boost). Typically the most constrained device — e.g. it may demand static input shapes. |
| `nq` | Length of the position vector `qpos`. Equals `nv` only when no joint uses a quaternion. |
| `nu` | Number of actuators, i.e. the length of the control vector `data.ctrl`. |
| `nv` | Length of the velocity vector `qvel`; the true degree-of-freedom count. |
| position gain | The `kp` of a `<position>` actuator — how hard the built-in spring pulls the joint toward the target angle you wrote into `data.ctrl`. SO-101's class default is `kp="998.22"` with `kv="2.731"`; raising it makes the joint arrive faster and overshoot. |
| `qpos` / `qvel` | The position and velocity vectors inside `MjData`, lengths `nq` and `nv`; the env copies both into every observation dict. |
| quantization | Re-expressing weights/activations in a lower-precision numeric type (FP32 → FP16 → INT8) to cut memory and latency, at some accuracy cost. Not part of M03, which stays FP32; it arrives later via NNCF (`ARCHITECTURE.md` ADR-013). |
| quaternion | A 4-number encoding of a 3D orientation, constrained to unit length so it carries only 3 independent DoF; the reason `nq` can exceed `nv`. |
| skill | One named action in the fixed vocabulary the grounder may emit — `open_drawer`, `pick`, `place`, `handoff`, `pour` — each implemented as a manipulation primitive in M06. `handoff` is the one that moves an object between arms. |
| `SkillCall` | One step of a plan as a dataclass: `skill`, `arm`, `target_object`, `params: dict`. `arm` is mandatory and never `None` (`PLAN.md` M05 done-when 3). |
| slide joint | A joint that translates along one fixed axis — 1 position value and 1 DoF. The drawer uses one (`drawer_slide`). |
| SO-101 | The 6-joint open-source robot arm used in this project; asset from TheRobotStudio/SO-ARM100. |
| STL | Triangle-mesh file format; the 13 files in `scenes/so101/assets/` supply the arm's shapes. |
| subgoal | One stage inside a skill — e.g. `pick` decomposes into approach pose, grip pose, retreat pose. The controller holds one subgoal at a time and advances only once a reached-test passes. |
| `TaskPlan` | The grounder's output: an ordered sequence of `SkillCall`s for one command. The typed IR between language and control — produced once per command, then consumed by the Coordinator. |
| throughput vs latency | Latency is time for one inference end to end; throughput is inferences per second in aggregate. Batching and parallelism can raise throughput while making latency worse — a closed-loop robot cares about latency. |
| tunneling | An object moving far enough in one timestep that collision detection never sees the surface between start and end, so it passes straight through. |
| `UngroundedCommandError` | The typed error a grounder raises when a command falls outside the documented grammar. Required by `PLAN.md` M05 done-when 2 so an unsupported command refuses loudly instead of yielding a partial plan. |
| waypoint | A target pose on the way to a subgoal; a skill is largely a list of them. Going through waypoints rather than straight to the goal is how you avoid dragging the gripper through the table or the other arm. |
| `world` / worldbody | The fixed root of the body tree; everything is positioned relative to it and it never moves. |
