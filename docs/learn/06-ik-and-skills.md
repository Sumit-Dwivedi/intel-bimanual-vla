# 06 — IK and Closed-Loop Skills: Turning a `SkillCall` Into Motion

*Written before M06 is built. This is the design about to be built, not a result.*

**Problem.** Note 05 ended at a `TaskPlan` of `SkillCall`s — `pick(plate, arm=A)` is a
noun phrase, not a motion. M06 turns each call into joint-angle targets the env can
actually execute, and under ADR-023 it is the *only* controller this submission ships.

**Key concept: inverse kinematics.** Forward kinematics is evaluating an expression —
given joint angles, MuJoCo tells you where the end-effector lands. IK is solving for *x*:
given a desired end-effector pose, find joint angles that reach it. Several answers
usually exist (elbow up vs. elbow down), so you pick the one nearest the current pose.

**The catch specific to this arm.** SO-101 has six `<position>` actuators, but `gripper`
only opens the jaw — that leaves **5 positioning DoF against a 6-DoF task space** (3
position + 3 orientation). A 5-DoF arm generally cannot hit an arbitrary position *and*
orientation. Something must be relaxed: solve for position and accept the resulting
orientation, or constrain one axis only. Also aim at the right body — per GLOSSARY note
01, `<body name="gripper">` is driven by `wrist_roll`, so a target on the wrong body
silently reaches the wrong place.

**Closed-loop, not fire-and-forget.** Each timestep: read `get_state()`, pick the current
subgoal (approach → grip → retreat), solve IK, write targets via `env.step()`, test
whether the subgoal is met, advance or time out (`control/ik.py`,
`control/skills_scripted.py`, `control/executor.py`). The loop is what absorbs friction
and small collisions. It is affordable only because ADR-022 made rendering opt-in: physics
alone is 0.20 ms/step, three cameras ~456 ms (`docs/hardware/m02-render-cost.md`).

**Handoff is the hard one.** The arms sit at y=±0.25 and overlap in roughly a 0.10 m band
at the table centre — the only place a handoff can physically happen. One gripper must
open exactly when the other has closed; early drops the object, late crushes it.

**Try this.** When M06 lands, raise the position gain (`kp`, default `998.22`) on
`shoulder_pan`. It will overshoot its targets. That is why closed-loop control with modest
gains beats open-loop with high gains.
