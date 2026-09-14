# M06 handoff — perturbation robustness diagnostic (ADR-051)

**Diagnostic only. No skill logic changed.** Not modified by this task:
`skills_scripted.py`, `grasp.py`, `ik.py`, `executor.py`, `env.py`,
`randomization.py`, `scenes/so101/`, `gen_dual_scene.py`. New files added:
`scripts/probe_handoff_perturbation.py` (this measurement) and this document.

**Provenance.** Every number below ran on bm-ptl,
`C:\Users\devcloud\project\ov_env\Scripts\python.exe`. ADR-047 recorded a
pre-existing float divergence between the laptop and bm-ptl; laptop numbers
must not be reported and none are — this document is bm-ptl only.

**Interpretation bar, fixed before the run and not revisited after seeing the
data:** more than 10/15 trials passing supports a robustness statement in the
demo video; fewer than 10/15 means we do not make one.

**Answer, stated up front: the result landed on the "do not make the claim"
side, decisively (0/5 at the very first, smallest magnitude tested).**

---

## 1. The question

M08 (ADR-049) reported `handoff(B, A, fork)` Track A 10/10, but that
envelope is a single point (`SKILL_ENVELOPES["handoff"]["fork"] = (0,0,0,0)`)
— every one of those ten trials was the identical deterministic scenario,
so 10/10 measures determinism, not robustness. Track B (a prop the skill
never touches moving) gave 0/10. This diagnostic asks whether `handoff`
tolerates a *different* class of small perturbation: noise on each arm's own
home pose, rather than a prop's position.

## 2. What was NOT run, and why

The task brief proposed a physics-timestep sweep (0.001 / 0.002 / 0.003 s)
alongside the noise sweep. **This was deliberately skipped.** No `timestep`
is set anywhere in the scene XML or `gen_dual_scene.py`, so the model runs at
MuJoCo's default, 0.002 s. Every skill's step budget in this repo —
`handoff`'s included, 6610 frames on the zero-noise control below — is a
FRAME count tuned at that default, i.e. 6610 frames = 13.2 s of simulated
time at dt=0.002 s. Changing `dt` alone, without rescaling the frame budget
by `0.002/dt`, does not perturb the skill physically: at dt=0.001 s the same
6610-frame budget only buys 6.6 s of simulated time (likely a manufactured
timeout, not a physics finding), and at dt=0.003 s it buys 19.8 s (extra,
free budget the skill was never tuned to need), on top of changed integrator
accuracy and contact resolution at each `dt`. A near-certain failure or an
easy pass under either skew would say "this skill is frame-budget-tuned for
dt=0.002," which ADR-046/ADR-049 already established, dressed up as a
robustness result. Not run, for that reason — not for lack of time inside
this task's 60-minute cap.

## 3. Method

- **Perturbation.** Independent noise, drawn per (magnitude, seed) from
  `numpy.random.default_rng(seed=1000*level_idx + seed)`, applied to
  `env.data.qpos` at each of `armA_shoulder_lift`, `armA_elbow_flex`,
  `armB_shoulder_lift`, `armB_elbow_flex`'s own `jnt_qposadr` slot (each a
  1-DOF hinge joint), immediately after `env.reset(seed=seed, cameras=None)`,
  followed by `mujoco.mj_forward(env.model, env.data)` — the same
  re-derivation step `env.py`'s own randomizer branch already uses after
  perturbing a prop's `qpos`, applied here to arm joint `qpos` instead.
- **Call.** Fresh `TableSettingEnv(cameras=None)` + fresh `WeldGrasp(env)`
  per trial (ADR-047: reusing either across a `reset()`-based loop silently
  corrupts `WeldGrasp.active_welds`), then receiver-first, exactly as
  `scripts/verify_adr038_skills.py`'s own established direct-call pattern
  (its `fresh()` helper + its own comment "receiver-first: A->B is (B, A)"):
  `run_handoff(env, "B", "A", "fork", weld=weld)`.
- **Grid.** 5 seeds (0-4) per magnitude, magnitudes `[0.005, 0.01, 0.02]` rad,
  widening to the next magnitude ONLY if the previous one passed all 5 seeds
  — finding the breaking point, not a pass count at one level.
- **Script:** `scripts/probe_handoff_perturbation.py`. Raw per-trial JSONL:
  `out/m10_handoff_perturbation.jsonl` on bm-ptl (gitignored, not committed —
  full contents transcribed below).

## 4. Sanity control: zero noise, same harness

Before trusting any noisy-trial result, the harness itself was checked
against the known baseline: same direct-call pattern, same fresh-env/fresh-
weld construction, `mujoco.mj_forward` called with zero noise added, 5 seeds:

| seed | success | frames | from_arm_retreat_dist |
|---:|---|---:|---:|
| 0 | True | 6610 | 0.2263 |
| 1 | True | 6610 | 0.2263 |
| 2 | True | 6610 | 0.2263 |
| 3 | True | 6610 | 0.2263 |
| 4 | True | 6610 | 0.2263 |

All 5: `held by arm B: z=0.5498 (initial 0.3560) dist_to_armA=0.1568
dist_to_armB=0.0582 (to_arm=B) weld_holding_to_arm=True
weld_holding_from_arm=False from_arm_retreat_dist=0.2263
from_arm_clear=True`.

This is byte-identical to ADR-046's own recorded oracle baseline
(`from_arm_retreat_dist=0.2263 m`, PICK_LIFT-cleared `z`, `frames_used`
consistent with the 6610-frame budget the task brief itself cites for this
skill). **The harness is sound** — the primary experiment's failures below
are not an artifact of this script's own construction.

## 5. Primary experiment: home-pose noise, ±0.005 rad

5/5 seeds, all FAILED. The ladder therefore did not widen to ±0.01 or
±0.02 rad — **the breaking point is at or below ±0.005 rad** (roughly
±0.3 degrees on two joints per arm).

| seed | armA shoulder_lift (rad) | armA elbow_flex (rad) | armB shoulder_lift (rad) | armB elbow_flex (rad) | success | phase | frames | IK residual (m) | from_arm_retreat_dist |
|---:|---:|---:|---:|---:|---|---|---:|---:|---:|
| 0 | +0.001370 | -0.002302 | -0.004590 | -0.004835 | FAIL | 3 (to_arm approach) | 3155 | 0.0875 | N/A (fails before Phase 5) |
| 1 | +0.000118 | +0.004505 | -0.003558 | +0.004486 | FAIL | 3 (to_arm approach) | 3155 | 0.0875 | N/A (fails before Phase 5) |
| 2 | -0.002384 | -0.002015 | +0.003142 | -0.004081 | FAIL | 3 (to_arm approach) | 3155 | 0.0875 | N/A (fails before Phase 5) |
| 3 | -0.004144 | -0.002632 | +0.003013 | +0.000822 | FAIL | 3 (to_arm approach) | 3155 | 0.0875 | N/A (fails before Phase 5) |
| 4 | +0.004431 | +0.000113 | +0.004762 | -0.004192 | FAIL | 3 (to_arm approach) | 3155 | 0.0875 | N/A (fails before Phase 5) |

**Result: 0/5.**

Verbatim failure reason, identical across all 5 trials:

```
phase 3 (to_arm approach) failed [direct approach failed [convergence
(IK residual=0.0875 m >= 0.01 m)]; staging to y=-0.06 also failed
[collision (cross_arm contacts=1 vs baseline 0; armB-vs-table_top
contacts=0 vs baseline 0)]]
```

`from_arm_retreat_dist` — handoff's own known knife-edge metric (ADR-046
found a 2.2 mm perception offset collapsed it from 0.2263 m to 0.0540 m
against the 0.1 m gate) — is **not reached at all** in any of these 5
trials: the skill fails earlier, at Phase 3 (`to_arm` approach), before
Phase 5's retreat gate is ever evaluated.

### The identical-failure pattern is itself the finding

Five different random noise vectors (both magnitude within ±0.005 rad and
sign varying independently per joint per seed) produced the SAME phase,
SAME IK residual to four decimal places (0.0875 m), and the SAME frame
count (3155) at failure. This is not five different near-misses — it is one
outcome, reproduced exactly. That pattern matches, word for word, what
ADR-049 already documented for `water_bottle` placement noise: "any nonzero
`water_bottle` offset reproduces the identical phase-3 failure, only exactly
zero reproduces baseline" (ARCHITECTURE.md ADR-049). Combined with section 4's
control (exactly zero noise passes, exactly and only), the evidence reads the
same way here: `handoff`'s Phase 3 cross-arm corridor (ADR-037's own
"joint-limit knife-edge" language) is a cliff with respect to home-pose
noise, structurally the same cliff already found for prop placement — not
a new, independent failure mode, and not a harness artifact.

## 6. Interpretation

**0/5 at the first (smallest) magnitude tested. Widening to ±0.01/±0.02 rad
never triggered, so the full grid actually run is 5 trials, not 15
(N=5/15).** Against the bar fixed before this run — more than 10/15 supports
a robustness claim, fewer than 10/15 means none is made — **this is
decisively on the "do not make the claim" side.**

**Plain statement for the video/README:** `handoff(A, B, fork)` reproduces
deterministically at its exact tuned configuration (ADR-049's Track A
10/10, and this diagnostic's own 5/5 zero-noise control). It does **not**
tolerate the smallest home-pose arm-angle noise tested here (±0.005 rad on
`shoulder_lift`/`elbow_flex`, both arms), nor does it tolerate prop
placement beyond the single point ADR-049 already measured (Track B: 0/10).
No general robustness claim should be made for `handoff` in the demo video
or submission materials beyond "reproduces exactly at its tuned
configuration." This is useful negative information, not a gap in the
measurement: it confirms `handoff` is, as ADR-037 through ADR-049 already
characterised it, the most kinematically fragile skill in this repo, on a
knife-edge with respect to essentially any perturbation tried against it so
far — prop placement (ADR-049) and now home-pose joint noise (this
document) alike.

## 7. Regression gate

`pytest tests/test_skills.py`, bm-ptl, immediately before this diagnostic's
run:

```
4 passed / 4 failed
FAILED tests/test_skills.py::test_open_drawer_reaches_near_limit
FAILED tests/test_skills.py::test_pick_plate_lifts_above_table
FAILED tests/test_skills.py::test_place_plate_returns_to_table_rest
FAILED tests/test_skills.py::test_handoff_mug_ends_held_by_arm_b
```

Same four tests and the same failure reasons as ADR-047/ADR-048/ADR-049 —
unchanged by this diagnostic, as expected, since nothing in the skill or
executor layer was touched.

## 8. Constraints honoured

- No modification to `skills_scripted.py`, `grasp.py`, `ik.py`,
  `executor.py`, `env.py`, `randomization.py`, `scenes/so101/`, or
  `gen_dual_scene.py`.
- Fresh `TableSettingEnv` + fresh `WeldGrasp` per trial throughout (ADR-047).
- Results written one JSON line at a time, flushed immediately
  (`scripts/probe_handoff_perturbation.py`'s `_write_jsonl`) — a mid-run
  failure would have lost at most the one in-flight trial. In fact all 5
  trials plus the summary line completed without any harness failure.
- Only `scripts/probe_handoff_perturbation.py` and this document were added.
- ADR-051 recorded in `ARCHITECTURE.md` and mirrored in `DECISIONS.md` —
  a real decision emerged (whether to make a robustness claim in the demo
  video), so it was not skipped.
