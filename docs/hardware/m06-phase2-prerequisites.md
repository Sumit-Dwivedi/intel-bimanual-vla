# M06 Phase 2 prerequisites: IK regime and ctrl-holding audit

**Read-only audit.** No edits to `skills_scripted.py`, `env.py`, `ik.py`, `grasp.py`, or any scene file. No probes run. This document answers two specific questions about code already committed, ahead of Phase 2 (wiring weld-based grasping into the scripted skills).

## Q1 verdict: every skill, every waypoint type, uses the INCREMENTAL (closed-loop, re-solved-every-step) regime. There is no single-shot IK anywhere in this module.

`open_drawer`, `pick`, `place`, and `handoff` are all built exclusively from two shared helpers, and neither one solves IK once and holds the result:

- `_drive_to_target` (`src/bimanual/control/skills_scripted.py:475-555`) drives APPROACH/DESCEND/RETREAT/INSERT/PULL waypoints. Its loop body:

  ```
  while steps < max_steps:
      solution = ik.solve_position_ik(env.model, env.data, arm, target)   # skills_scripted.py:541
      ctrl = _hold_ctrl(env)
      _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
      env.step(ctrl)                                                      # skills_scripted.py:544
      steps += 1
      ...
  ```
  (`skills_scripted.py:540-555`). `ik.solve_position_ik` is called fresh, against `env.data`'s CURRENT state, on every single iteration of this `while` loop -- not once before the loop.

- `_dwell` (`skills_scripted.py:558-606`), used for every GRIP/RELEASE waypoint, has the identical structure:

  ```
  for i in range(n_steps):
      solution = ik.solve_position_ik(env.model, env.data, arm, target)   # skills_scripted.py:588
      ctrl = _hold_ctrl(env)
      _write_arm_ctrl(ctrl, env.model, arm, solution.joint_angles, gripper_ctrl)
      env.step(ctrl)                                                      # skills_scripted.py:591
      ...
  ```
  (`skills_scripted.py:587-591`).

- `_run_waypoint` (`skills_scripted.py:834-892`) and `_run_dwell` (`skills_scripted.py:895-931`) are thin wrappers that call `_drive_to_target`/`_dwell` and then run `_validate_against_baseline`'s post-hoc IK-residual/collision check -- they add no additional IK solve of their own inside the step loop.

Every skill is built ONLY from calls to `_run_waypoint`/`_run_dwell` -- there is no other code path in this file that calls `ik.solve_position_ik` or `env.step()`:

- `run_pick` (`skills_scripted.py:939-1016`): waypoint 1 APPROACH (`_run_waypoint`, :967), waypoint 2 DESCEND (`_run_waypoint`, :976), waypoint 3 GRIP (`_run_dwell`, :986), waypoint 4 RETREAT (`_run_waypoint`, :1000).
- `run_place` (`skills_scripted.py:1019-1128`): nested `run_pick` (:1053), waypoint 1 APPROACH (`_run_waypoint`, :1076), waypoint 2 DESCEND (`_run_waypoint`, :1088), waypoint 3 RELEASE (`_run_dwell`, :1098), waypoint 4 RETREAT (`_run_waypoint`, :1109).
- `run_handoff` (`skills_scripted.py:1131-1292`): nested `run_pick` (:1172), then waypoints 1-8 alternate `_run_waypoint`/`_run_dwell` exclusively (:1190, :1200, :1212, :1222, :1232, :1242, :1254, :1268).
- `run_open_drawer` (`skills_scripted.py:1295-1421`): APPROACH (`_run_waypoint`, :1372), INSERT (`_run_waypoint`, :1382), GRIP (`_run_dwell`, :1389), PULL (`_run_waypoint`, :1396), RELEASE (`_run_dwell`, :1403), RETREAT (`_run_waypoint`, :1410).

There is no mix, and no waypoint type (APPROACH/DESCEND/GRIP/RELEASE/INSERT/PULL/RETREAT) that departs from this pattern: **every waypoint, in every skill, re-solves IK against the current arm state on every physics step**, for as many as `APPROACH_DESCENT_STEPS` (500) or `GRIP_HOLD_FRAMES` (300) steps.

### Interpretation against the two probes

The two probe documents used two DIFFERENT regimes, and each says so about itself:

- `docs/hardware/m06-ik-lift-diagnostic.md:46`: *"This probe... solves IK ONCE per target (per this task's own instructions), toward a much larger, immediately-distant target, then holds that one joint solution for 300 steps."* This is single-shot. It found the pinch point and gripper body rising together (50-79%/centimetre-scale).
- `docs/hardware/m06-weld-verification.md:20`: describes *"re-targeting the pinch point progressively higher... continuously re-anchored from the current pinch point"* -- i.e. IK re-solved every step against a small, incrementally-advancing target -- and found the gripper body FALLING (millimetre-scale) while the pinch point tracked, because the wrist absorbed each small correction.

The lift-diagnostic document itself already flags this exact ambiguity, verbatim: *"Both findings are real; they describe different control methodologies (single large-step solve vs. many small closed-loop re-solves) applied to the same underlying redundant, orientation-unconstrained IK, and Phase 2 should not assume either one generalizes to the other without checking which regime the real skills' closed loop (`skills_scripted.py`) actually operates in."* (`m06-ik-lift-diagnostic.md:46`)

Having now checked: **the real skills' closed loop is the incremental regime, not the single-shot one.** `skills_scripted.py`'s `_drive_to_target`/`_dwell` re-solve `ik.solve_position_ik` against `env.data`'s current state on every `env.step()` call, exactly matching `m06-weld-verification.md`'s methodology (re-anchored, small-increment re-solves), not `m06-ik-lift-diagnostic.md`'s (one large solve, held for 300 steps).

**Verdict: `docs/hardware/m06-weld-verification.md`'s finding governs Phase 2** -- millimetre-scale lift, with the gripper body (the weld attach frame) potentially FALLING while the IK-tracked pinch point rises, because the wrist absorbs each small per-step correction. The lift-diagnostic's 50-79% centimetre-scale result describes a regime (single-shot, large target delta, held) that the scripted skills do not use anywhere, and should not be used to size Phase 2's expected lift margins.

## Q2 verdict: `step()` requires exactly a full 12-vector every call; it is a bare overwrite with no merge or hold mechanism of its own. Holding the idle arm is entirely the CALLER's responsibility, done today only inside `skills_scripted.py`'s `_hold_ctrl`, never inside `env.py`.

Confirmed from `src/bimanual/sim/env.py`:

```
action = np.asarray(action, dtype=np.float64).reshape(-1)
if action.shape[0] != self.model.nu:
    raise ValueError(
        f"action has length {action.shape[0]}, expected {self.model.nu} "
        f"(one target per actuator: {self.model.nu} actuators total)."
    )

self.data.ctrl[:] = action                    # env.py:260
mujoco.mj_step(self.model, self.data)          # env.py:261
```
(`env.py:253-261`.) `self.model.nu` is 12 (6 armA + 6 armB actuators, per the docstring at `env.py:240-242`). `step()` raises `ValueError` if `action` is not exactly length 12 (`env.py:254-258`) -- so yes, a full 12-vector is required, not a 6-vector for "the arm being driven" plus an implicit default for the rest.

`self.data.ctrl[:] = action` (`env.py:260`) is a **full, unconditional overwrite** of every actuator's ctrl target, every call. There is no merge with the previous `data.ctrl`, no "hold last commanded value for actuators the caller didn't touch," and no reference to `qpos` inside `step()` at all. `TableSettingEnv` has no idle-arm-holding logic anywhere in its own code.

**The user's stated hypothesis is correct as read, and the drawn consequence is correct in exactly the case it describes -- but that case does not occur inside `skills_scripted.py`'s current code, because a separate caller-side mechanism (not part of `env.py`) prevents it.** Specifically:

- Confirmed: `so101_dual_table.xml`'s `home` keyframe declares `qpos` only (`so101_dual_table.xml:427`), no `ctrl` attribute anywhere in the file:
  ```
  grep -n "ctrl=" so101_dual_table.xml   ->  0 matches
  ```
  `mj_resetDataKeyframe` therefore leaves `data.ctrl` at its default (zero) after `reset()`, while `qpos` sits at the folded home pose (e.g. `shoulder_lift=-1.2`, `elbow_flex=-1.6`, `wrist_flex=1.745329` per arm, from the keyframe string at `so101_dual_table.xml:427`) -- non-zero and far from each arm's `qpos=0` fully-extended configuration.
- Confirmed: if a caller called `env.step(action)` with `action[6:]` (arm B's slice) left at 0 while only arm A's slice carries real targets, `env.py:260` would write `0.0` into every one of arm B's 6 position-actuator ctrl targets, every step -- commanding arm B's joints toward `qpos=0` (fully extended), which is exactly the configuration ADR-026 measured at 34 total contacts, 29 armA<->armB, deepest -0.0597 m, per `env.py:33-37`'s own module docstring.
- **However, `skills_scripted.py` never does this.** Every one of its `env.step()` calls is preceded by `ctrl = _hold_ctrl(env)` (`skills_scripted.py:542, 589`), which builds the FULL ctrl vector by reading `env.data.qpos` at EVERY actuator's own joint address, fresh, on that same step (`skills_scripted.py:419-430`):
  ```python
  def _hold_ctrl(env) -> np.ndarray:
      ctrl = np.zeros(env.model.nu, dtype=np.float64)
      for aid in range(env.model.nu):
          jid = int(env.model.actuator_trnid[aid, 0])
          qadr = int(env.model.jnt_qposadr[jid])
          ctrl[aid] = env.data.qpos[qadr]
      return ctrl
  ```
  Only AFTER this is the active arm's own 6-slot slice overwritten with the freshly-solved IK joint targets, via `_write_arm_ctrl` (`skills_scripted.py:433-439, 543, 590`). So today, within this module, the idle arm's ctrl target is re-anchored to wherever it CURRENTLY sits, every single step -- not frozen at its reset-time pose, and never zeroed.

**Verdict: `env.step()` itself does no holding -- it is a dumb, full 12-length overwrite, and the "zero action[6:] pulls arm B toward qpos=0 / the ADR-026 interpenetration pose" scenario is real and would happen exactly as described if invoked that way.** It does not happen today only because `skills_scripted.py`'s own `_hold_ctrl` helper (called inside the IK loop, not inside `env.py`) supplies a live, per-step "hold current qpos" value for every actuator before each skill's active arm slice is written on top of it. This is an important asymmetry for Phase 2: any NEW caller (e.g. code driving `grasp.py`'s `WeldGrasp` mechanism directly against `env.step()`, or any Phase 2 wiring path that bypasses `_hold_ctrl`) that forgets to reproduce this "read current qpos into every idle slot first" step will reintroduce the ADR-026 cross-arm interpenetration failure mode the very first time it drives one arm while leaving the other's slice at its language-model/default value of 0.

## Implications for Phase 2 wiring

1. **Any Phase 2 code that calls `env.step()` must route through (or replicate) `_hold_ctrl`'s per-step "read current qpos, write it back" pattern for every actuator it is not actively driving.** There is no environment-level safety net; `env.py:260` will happily accept and apply a naive zero-filled idle-arm slice, silently reproducing the ADR-026 34-contact interpenetration.
2. **Phase 2's expected lift/grasp behaviour should be modelled on `m06-weld-verification.md`'s incremental-regime finding (millimetre-scale, gripper-body-may-fall-while-pinch-tracks), not `m06-ik-lift-diagnostic.md`'s single-shot 50-79% figure**, because the scripted skills' actual `_drive_to_target`/`_dwell` loops re-solve IK every step against a continuously-advancing target -- the same methodology the weld-verification probe used, not the lift-diagnostic's one-shot-then-hold methodology.
3. Both findings above compound: if Phase 2 wires `WeldGrasp` into `pick`/`place`/`handoff`'s existing GRIP/RETREAT waypoints (which already run through `_drive_to_target`/`_dwell` and therefore already inherit both the incremental IK regime and the `_hold_ctrl` idle-arm protection), no new interpenetration or lift-margin surprise should appear beyond what `m06-weld-verification.md` already measured. The risk is specific to any NEW, separate call path that talks to `env.step()` directly.

**Recommended Phase 2 approach (one sentence):** wire `WeldGrasp` into the existing `_drive_to_target`/`_dwell`/`_run_waypoint`/`_run_dwell` machinery (so it automatically inherits both the incremental every-step IK regime and the `_hold_ctrl` idle-arm-holding pattern already proven safe by every current skill) rather than adding any new code path that calls `env.step()` on its own, and size Phase 2's expected lift margins from `m06-weld-verification.md`'s millimetre-scale, wrist-absorbs-the-correction result -- not `m06-ik-lift-diagnostic.md`'s single-shot centimetre-scale one.
