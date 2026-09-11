# 02 — Env Wrapper and Physics Stability

**Problem.** Note 01 ended with a compiled `MjModel`, which is inert data. Something must
own the mutable `MjData`, advance it in time, and hand out observations a policy can eat.

**Key concept: the env wrapper is a state machine behind a fixed API.**
`TableSettingEnv` (`src/bimanual/sim/env.py`) exposes `reset/step/render/get_state/close`.
It is deliberately *not* a `gym.Env` — no `Space` objects, no reward, and `step` returns
`(obs, done, info)` with `done` hardcoded `False`.

**How it works.** `reset(seed)` calls `mj_resetData` (wipes qpos, qvel, time and contacts
to compiled defaults) then `mj_forward` (recomputes derived quantities without advancing
time); the seed only feeds `self.np_random`, reserved for later randomization. `step`
writes 12 floats into `data.ctrl` and calls `mj_step` once. `get_state()` returns a flat
`(92,)` array via `mj_getState(mjSTATE_FULLPHYSICS)` — enough to resume exactly, versus
the observation dict's smaller policy-facing `qpos`/`qvel` plus per-camera RGB.

**Free joints vs hinges.** This scene compiles to `nq=48`, `nv=43`: 12 arm hinges + 1
drawer slide + 5 `<freejoint>` props. Each free joint stores 7 positions
`[x, y, z, qw, qx, qy, qz]` but only 6 velocities (3 linear, 3 angular). 12+1+35=48;
12+1+30=43. The extra number is the quaternion's: 4 components pinned to unit length
encode just 3 rotational DoF. That constraint is precisely why `nq > nv` here.

**Why 1000 zero-action steps.** `scripts/probe_physics_stability.py` resets at seed 0 and
steps 1000 times with `ctrl = 0`, checking for NaN and for any prop below
`FLOOR_Z = 0.30 m`. A do-nothing rollout is the cheapest test because a correct scene
should be boring. It catches **tunneling** (a thin geom moves far enough in one timestep
to skip collision and end up under the table), **NaN cascades** (one infinite force
poisons the state, and since each step reads the last, it never clears — hence the
immediate `break` at line 106), and **interpenetration** (geoms spawned overlapping, so
the solver fires a huge separation impulse and props launch).

It passed 1000/1000. The instructive number: the plate's minimum z is **0.34990**, ~0.1 mm
*below* the 0.35 m tabletop. Not a bug — MuJoCo contacts are soft, so resting objects sink
microscopically until contact force balances weight. That is exactly why the threshold is
0.30, not 0.35. Numbers in `docs/hardware/m02-physics-stability.md`.

**Try this.** On bm-ptl (MuJoCo will not import on the laptop, ADR-020):

```
cd C:\Users\devcloud\intel-bimanual-vla
C:\Users\devcloud\project\ov_env\Scripts\python.exe scripts/probe_physics_stability.py
```

Then change `FLOOR_Z` to `0.35` and rerun. Predict first: which bodies trip, and at which
step? You will see a "tunneling event" reported for an object that never moved.
