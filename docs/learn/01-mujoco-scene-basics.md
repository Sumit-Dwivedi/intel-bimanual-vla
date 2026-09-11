# 01 — MuJoCo Scene Basics

**Problem.** Before a policy can move anything, the simulator needs a machine-readable
description of the robot. `scenes/so101/` is that description for one SO-101 arm.

**Key concept: MJCF is source, MuJoCo is a compiler.** An MJCF file is XML declaring
bodies, joints, geoms and actuators. `MjModel.from_xml_path` parses it, resolves
`<include>` and `<default>` inheritance, loads meshes, and emits a fixed-layout compiled
struct — an AST lowered to flat arrays.

**How it works.** `scenes/so101/scene.xml` is a wrapper: it `<include>`s
`so101_new_calib.xml`, then adds a skybox, a light, and one floor geom. The real arm is
`so101_new_calib.xml` — seven nested `<body>` elements, `base` down to
`moving_jaw_so101_v1`, each joined to its parent by one hinge. `scripts/probe_render.py`
(line 82) printed the compiled sizes:

| Symbol | Value | Meaning |
|---|---|---|
| `nq`  | 6  | position coordinates |
| `nv`  | 6  | velocity coordinates (degrees of freedom) |
| `nu`  | 6  | actuators (control inputs) |
| `nbody` | 8 | bodies, including the implicit `world` |
| `ngeom` | 31 | collision/visual shapes, including the floor |

`nq == nv` **only because every joint here is a hinge** (one number, one rate). This is
not general: a free-floating body stores orientation as a 4-number quaternion but has 3
rotational DoF, so `nq > nv`. Expect M02's props to break the equality.

**Actuator.** In MJCF an actuator is the thing that converts a control number into joint
force. All six here are `<position>` actuators (lines 157-162): you write a *target
angle* into `data.ctrl`, and MuJoCo runs an internal spring-damper (`kp="998.22"`,
`kv="2.731"` from the `sts3215` default class, with each actuator overriding
`forcerange` to `-3.35 3.35`) to drive the
joint there. You command where to go, not how hard to push. The six are `shoulder_pan`,
`shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`.

**Try this.** MuJoCo does not run on the laptop (ADR-020: Smart App Control blocks the
unsigned `mujoco.dll`), so run this on bm-ptl:

```
ssh -J guest@146.152.207.201 devcloud@192.168.2.2
cd C:\Users\devcloud\intel-bimanual-vla
C:\Users\devcloud\project\ov_env\Scripts\python.exe -c "import mujoco; m=mujoco.MjModel.from_xml_path(r'scenes/so101/so101_new_calib.xml'); print(m.nq, m.nv, m.nu, m.nbody, m.ngeom)"
```

That compiles the arm *without* `scene.xml`'s wrapper. Predict all five numbers before
you hit enter. Four are unchanged; exactly one moves, and the diff tells you precisely
what `scene.xml` contributes.
