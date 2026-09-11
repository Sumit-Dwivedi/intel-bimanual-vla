# M02 render cost — opt-in camera rendering, measured on bm-ptl

Measured Sept 11, 2026 on bm-ptl (`WIN-GLILH4PFDLN`), commit `731f0aa`,
mujoco 3.2.7, `ov_env` venv. Scene: `src/bimanual/sim/assets/so101_dual_table.xml`
(nq=48, nv=43, nu=12, nbody=23, ngeom=80). Cameras render at the env default
640x480. Each configuration warmed up 5 steps, then timed with
`time.perf_counter()` over 100 steps (state-only) or 30 steps (camera configs).

## Results

| configuration | ms/step | vs state-only |
|---|---:|---:|
| `cameras=None` (state-only) | **0.20** | 1x |
| `cameras=['front']` | **152.25** | 761x |
| all five cameras | **809.86** | **4049x** |

## Rendering hardware

Not a software fallback. `scripts/probe_gl_renderer.py` reports:

```
GL_VENDOR:   Intel
GL_RENDERER: Intel(R) Arc(TM) B390 GPU
GL_VERSION:  4.6.0 - Build 32.0.101.8860
```

The offscreen context is the real Arc B390 iGPU with a genuine OpenGL 4.6
driver. The per-frame cost below is actual GPU-path time, not CPU rasterization.

## Interpretation

**Physics is effectively free; rendering is the entire cost.** A full `mj_step`
plus observation assembly on this scene takes 0.20 ms. Every millisecond beyond
that is camera rendering.

**Per-camera cost, and how it scales.** Subtracting the 0.20 ms physics floor:
one camera costs 152.05 ms; five cost 809.66 ms, i.e. 161.93 ms each. Five
cameras priced at the one-camera rate would predict 760.25 ms, so the measured
809.66 ms is 6.5% above linear. **Cameras scale linearly — there is no large
fixed setup being amortised.** Adding the Nth camera costs roughly as much as
the first. This is the load-bearing fact for the decision in ADR-022: there is
no threshold below which "just render them all" becomes cheap.

**Speedup: 4049x** between state-only and all-five. Concretely, for one
1000-step episode:

| configuration | wall clock per 1000-step episode |
|---|---|
| state-only | 0.2 s |
| one camera | 2 min 32 s |
| all five | 13 min 30 s |

The ADR component table budgets 50 ms/step. State-only uses 0.4% of that
budget; a single camera already exceeds it by 3x.

## Investigation deferred

Investigation deferred: the ~133ms per-frame cost on Arc B390 is
~100x slower than expected. Confirmed not a software fallback
(GL_RENDERER: Intel(R) Arc(TM) B390 GPU with genuine OpenGL 4.6).
Suspects, in order of likelihood: (a) per-frame GPU-to-CPU pixel
readback stall, (b) update_scene rebuilding scene geometry each
call, (c) 13 STL meshes per arm re-uploaded rather than cached.
Not investigated further within the 5-day hackathon window --
opt-in rendering satisfies the immediate need (M09 requires no
cameras per ADR-005; M11 vision training on Kaggle absorbs the
cost).

## Reproducing

```
# on bm-ptl, from the repo root
C:\Users\devcloud\project\ov_env\Scripts\python.exe scripts\probe_gl_renderer.py
```

The three-configuration benchmark instantiates `TableSettingEnv(cameras=...)`
for each of `None`, `['front']`, and all five camera names, and times
`step(np.zeros(12))` after a 5-step warmup.

## Note on fork/spoon materials

Note: fork and spoon share fork_material by design. They are
distinguishable by shape and position in the scene, not by color. The
front camera resolution renders them as small pale slivers; higher-
resolution or closer camera angles used for the cover image will
disambiguate them.
