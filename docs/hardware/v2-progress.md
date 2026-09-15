# REDESIGN v2 — progress and resume point

Work paused after Stage 3. This file is the resume point: what is done, what
was verified, what was corrected, and exactly what Stage 4 must do.

## State

| | |
|---|---|
| Branch | `redesign-v2` (pushed to origin) |
| HEAD | `51a4545` |
| `master` | **FROZEN** at `238cfed` — verified unchanged at every stage |
| `redesign` (v1) | untouched, preserved as evidence |
| Stages complete | 1, 2, 3 (each committed, each independently verified) |
| Stages remaining | **4** (skill rewrite), **5** (evaluation + video) |

### Commits on this branch

```
51a4545  v2 Stage 3 correction: handoff line z=0.40 -> 0.46
f53ffdd  v2 Stage 3: folded home pose, scene validated under top-down IK (ADR-072)
c50a71a  v2 Stage 2 addendum: correct tracking headline metric, drift gate mis-specified
ce85803  v2 Stage 2: cubic-spline joint-space motion primitive (ADR-071)
3c4d4ca  v2 Stage 1: geometric top-down IK for SO-101 (ADR-070)
238cfed  (master) Full-sequence demo video
```

### New code (all additive — nothing on master was rewritten)

- `src/bimanual/control/ik_geometric.py` — `solve_topdown_ik(model, data, arm, target_xyz, yaw=0.0)`
- `src/bimanual/control/motion.py` — `move_to_config(...)`, `set_gripper(...)`, `MotionResult`
- `scripts/v2_probe_kinematics.py`, `v2_validate_ik.py`, `v2_validate_motion.py`, `v2_validate_scene.py`
- `scripts/v2_verify_stage1_independent.py` — orchestrator's independent IK check
- `docs/hardware/v2-topdown-workspace.md`, `v2-tracking.md`, `v2-scene-validation.md`, `v2-reference-notes.md`

`ik.py`, `skills_scripted.py`, `grasp.py`, `env.py`, `executor.py`, `scenes/so101/`
and `gen_dual_scene.py` are all **unmodified** — confirmed by `git diff --stat`
against `238cfed` at every stage.

## Verified results

**Stage 1 — geometric top-down IK.** Round-trip error mean **2.0e-16 m**, max
**5.9e-16 m** over 500 targets (gate: mean<2mm, max<5mm). Independently
re-measured by the orchestrator with a different RNG seed and its own readback:
2.02e-16 / 5.90e-16 (arm A), 2.23e-16 / 4.87e-16 (arm B), **0 joint-limit
violations in 800 solutions**. Not circular — targets are random Cartesian
points, readback is a throwaway `MjData` + `mj_forward` on real body positions.

Orientation: the stage reported min dot 0.998258 using the wrist→pinch vector,
which includes the lateral pinch offset and therefore *cannot* be vertical. The
gripper body's own z-axis dots to **exactly -1.000000** (min = mean = max over
800 solves) — the top-down constraint is satisfied **exactly**. Confirmed
visually (jaws-down onto the fork) and structurally (grasp and hover solves
share identical `shoulder_pan` and `wrist_roll`).

Measured constants (from the MJCF, not a drawing):
`L1 = 0.11600 m`, `L2 = 0.13500 m`, tool offset `0.07836 m`,
arm A base `[0, -0.25, 0.35]`, `shoulder_pan` axis `[0,0,-1]`,
`shoulder_lift`/`elbow_flex`/`wrist_flex` axes all `[-1,0,0]` (the planar 3R chain).

**Stage 2 — cubic-spline motion.** Peak velocity/acceleration match the closed
forms `1.5*delta/T` and `6*delta/T^2` to 1.3e-6 and 2.0e-3 relative error.

The honest headline (orchestrator correction — see `v2-tracking.md`):
spline vs one-shot direct command, measured from `qpos`:

| | peak ACTUAL velocity | peak ACTUAL acceleration | final error |
|---|---|---|---|
| Spline 2.0 s | **0.451 rad/s** | **9.9 rad/s²** | 0.00014 rad |
| Direct | **5.350 rad/s** | **211.4 rad/s²** | 0.00014 rad |

**11.9x lower peak velocity, 21x lower peak acceleration.** Do NOT quote the
"~900x tracking error" figure: for a one-shot command the peak error is the
size of the move by construction.

**Stage 3 — home pose and scene.** Current home keyframe (`shoulder_lift=-1.2,
elbow_flex=-1.6`) KEPT; all contact gates pass with zero counts. All-zeros
(Lab 8's home) measured at **19 cross-arm contacts, deepest -0.0597 m** — it
does not transfer to a dual-arm scene.

## Corrections made to the brief (all measured, all already applied)

1. **Render stride.** `timestep = 0.002 s`, so Stage 5's "every 6th step at
   30 fps = real motion speed" is wrong — stride 6 gives **0.360x** real speed.
   Real time at 30 fps needs **stride 17** (16.67 exact).
2. **Home poses were inverted in the brief.** Current home is already folded;
   all-zeros is the extended pose that collides. No "extended → folded" change
   was needed or made.
3. **ADR-058 collision.** On `master`/`redesign-v2`, ADR-058 is Speechmatics
   voice. The position-only workspace is `redesign` branch ADR-058. Always
   qualify by branch. (v2 numbering starts at 070 for this reason.)
4. **Handoff height.** Stage 3's z=0.40 puts `armB_gripper` **-0.0023 m into
   the drawer** and -0.0152 m into the mug. Clean reachable band is
   **z = 0.435–0.485**; corrected to **z = 0.46** (midpoint, 2.5 cm margin).
5. **Drift gate is mis-specified.** See below.
6. **MuJoCo runs on the laptop.** Verified: 3.2.7 imports, builds the scene,
   and renders offscreen at 1280x720 in 1.7 s. ADR-020's premise is stale.
   Iterate locally; bm-ptl stays authoritative for numbers (ADR-047).
   **Stage 5 no longer needs 461 MB / 9-minute frame transfers.**

## Open issue for whoever sets Stage 4/5 gates

The `< 0.001 rad` idle-arm drift gate **fails at 0.001110 rad**, and the gate
is wrong, not the code. Max drift occurs at **step 12 of 1000**, then settles
to **exactly 0.000774** by step ~200 and holds for the remaining 800 steps —
a damped transient, and 0.000774 is ADR-037's own steady-state figure. ADR-037
sampled at phase boundaries, which can never see a step-12 transient. Any
snapshot-once PD hold must sag briefly: a gravity-loaded joint snapshotted at
zero position error has to move before the PD term restores it.

**Recommended restatement (NOT yet applied — user's call):** gate *settled*
drift at `< 0.001 rad` (passes at 0.000774) and disclose the bounded start-up
transient of 0.001110 rad decaying within ~200 steps.

## Key facts Stage 4 needs

- **Handoff points:** `P_from = (-0.03, 0.0, 0.46)` (arm A),
  `P_to = (0.03, 0.0, 0.46)` (arm B). 6 cm apart along **world +x**.
  **Cross-arm contacts = 0** — the failure that killed master's Phase 3 and
  defeated v1 is genuinely solved. Pinch points land on target to 4 dp.
- **Fork:** `xquat = [1,0,0,0]`, long axis is **world +x**, so grasp
  **yaw = pi/2**. The 6 cm offset is along the corridor's WIDE axis (0.36 m),
  not the tight y axis (0.12 m).
- **Two-arm corridor:** z=0.38–0.45 gives ~0.12 m in y by 0.36 m in x. The
  separation sweep was deliberately **not** run and should not be —
  0.5 m separation works under top-down IK.
- **Hover height 0.08 m is safe** for fork, mug and plate (reachable at every
  height 0→0.10 m). Lab 8 uses 0.03 m and warns reachability narrows with
  height; checked, it does not bite here.
- **water_bottle is OUT OF SCOPE for v2** — unreachable by both arms at every
  height (horizontal reach, not height). Master reached it 9/20 under
  position-only IK. **This is a real regression and must stay disclosed.**

## Next task — Stage 4 (cap 4 hours)

Write `src/bimanual/control/skills_v2.py` (new file; `skills_scripted.py` stays
for master's tests). Every skill is a sequence of `move_to_config` /
`set_gripper` primitives. **No IK inside the control loop** — solve once per
phase, then spline to it. `executor.py` gets a flag to route to `skills_v2`
instead of `skills_scripted` (that flag is the only permitted edit to
`executor.py`).

- **PICK(arm, obj):** hover (grasp+[0,0,0.08], 2.0 s, gripper open) → descend
  to grasp (1.0 s) → close (0.6 s) with `weld.attempt_grasp` during closure →
  lift back to hover (1.0 s). Yaw = object long-axis heading + 90°, read from
  `data.xquat`.
- **PLACE(arm, obj, target):** hover (2.0 s) → descend to
  `target+[0,0,obj_half_height]` (1.0 s) → `weld.release` then open (0.6 s) →
  retreat (1.0 s).
- **HANDOFF(from, to, obj):** from_arm to above `P_from` then descend;
  to_arm held at home → to_arm to above `P_to` then descend; from_arm held,
  still welded → to_arm closes, `weld.attempt_grasp(to_arm)` → verify to_arm
  holds, ONLY then from_arm releases and opens → from_arm retreats to home →
  to_arm lifts. Idle arm held at every phase.

**Five-criterion pass definition — all must hold (this is what master got
wrong and is non-negotiable):**

(a) target object ends where intended;
(b) NO non-target prop moved > 5 mm from its start position (record every
    prop's `xpos` at skill start, compare at end);
(c) NO contact between either arm and any non-target prop during the skill
    (enumerate `data.contact` every step; flag penetration > 1 mm);
(d) NO cross-arm contact during the skill;
(e) peak joint velocity < a threshold chosen from Stage 2b and justified.
    **Suggested: 1.0 rad/s** — the spline's measured peak at 2.0 s is
    0.451 rad/s, and the direct-command baseline was 5.350 rad/s, so 1.0 rad/s
    sits comfortably above normal operation and far below the old behaviour.

Criteria (b)–(e) are the ones that would have failed master's demo video,
where the mug was knocked onto its side and the water bottle fell off the
table. Add **ADR-073**, mirrored in `DECISIONS.md` and `ARCHITECTURE.md`.

Commit: `v2 Stage 4: phase-based skills with cubic splines and scene-integrity success criteria (ADR-073).`

## Then Stage 5 (cap 3 hours)

All skills at seed 0 against all five criteria; then envelope + 20 seeds for
every passing skill; then the video — one continuous fixed-camera shot,
azimuth 130 / elevation -22, `pick(A,fork)` → `handoff(A→B,fork)` →
`place(B,fork,table)`, **stride 17** (not 6) for real-time at 30 fps, encoded
on the laptop. Extract 8 frames and answer the five questions in the brief. If
any answer is no, that is a Stage 4 bug — fix it there, do not move the camera.
Write `docs/hardware/v2-final-eval.md` with the master-vs-v2 comparison, and
**ADR-074** as the v2 verdict.

## Running things

MuJoCo works locally (fast iteration). bm-ptl is authoritative for any number
that lands in an ADR or doc:

```
KEY=C:/Users/dwive/.ssh/intel_hackathon
PJ="ssh -i $KEY -o BatchMode=yes -o StrictHostKeyChecking=accept-new -W %h:%p guest@146.152.207.201"
ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -i "$KEY" \
    -o ProxyCommand="$PJ" devcloud@192.168.2.2 \
    "cd C:\\Users\\devcloud\\intel-bimanual-vla && C:\\Users\\devcloud\\project\\ov_env\\Scripts\\python.exe scripts\\X.py"
```

bm-ptl's checkout is BEHIND this branch — `scp` files across rather than
git-pulling (pulling needs a PAT). `ik_geometric.py` and `v2_validate_scene.py`
are already there. Verify every upload by md5. **Batch ssh calls aggressively**
— ~8 agents have been lost to jump-host rate limiting. `timeout /t` fails over
ssh; use `ping -n N 127.0.0.1 >nul`.
