# DECISIONS

User-ratified decisions, newest first. Each entry mirrors an ADR in
`ARCHITECTURE.md` section 4, which holds the full context, options and
consequences. Where the two disagree, `ARCHITECTURE.md` is authoritative and the
disagreement is a defect to be fixed.

ADR-001 through ADR-015 were authored by the planner and live only in
`ARCHITECTURE.md`. This file starts at ADR-016, the point at which decisions began
being ratified by the user rather than proposed.

---

## M02 — TableSettingEnv, view_scene.py, physics probe, front-camera fix (fulfills ADR-020, ADR-021)

**Recorded:** Sept 11, 2026 · **Module:** M02 (dual-SO-101 table scene v0) ·
**Relates to:** ADR-020 (sim runs on bm-ptl), ADR-021 (generator, not hand-edit)

Compliance-reviewer found M02 done-when criteria 1, 2 and 4 unmet (`src/bimanual/sim/env.py`
and `scripts/view_scene.py` did not exist yet). Closed in four separately committed steps,
each run and verified on bm-ptl per ADR-020 (mujoco cannot import on the laptop):

1. `src/bimanual/sim/env.py` — `TableSettingEnv` with `reset(seed)`, `step(action)`,
   `render(camera)`, `get_state()`, `close()`. Default scene path resolves via
   `Path(__file__).resolve().parent`, not the process cwd; verified importable and running
   from a cwd other than the repo root (`C:\Users\devcloud\import_check.py`, run from
   `C:\Users\devcloud`). Measured on bm-ptl (mujoco 3.2.7): `nq=48 nv=43 nu=12`, 4 cameras
   discovered from the model (`overhead`, `front`, `armA_wrist`, `armB_wrist`).
2. `scripts/view_scene.py` — `python scripts/view_scene.py --headless --save out/scene.png`
   writes a 1280x720 PNG (clamped to the scene's declared framebuffer) via `TableSettingEnv`.
3. `scripts/probe_physics_stability.py` — reset(seed=0), 1000 steps of zero action, checked
   for NaN and free-body tunneling at every step. **PASS**: no NaN, no free body dropped
   below `FLOOR_Z=0.30` m (5 cm below the 0.35 m tabletop surface — chosen so ordinary
   millimetre-scale contact settling cannot trip it; see
   `docs/hardware/m02-physics-stability.md` for the full derivation and the measured per-body
   minimum z values). Per the task's stop rule, this gated whether subtask (d) proceeded —
   it passed, so (d) went ahead.
4. Front-camera clipping fix. The camera used `mode="targetbody" target="table"`, which
   aims at the table body's own origin (z=0); measured on bm-ptl, the highest arm geometry
   at the reset pose reaches z≈0.667 against a 0.35 m tabletop, so the frame cropped both
   arms above the gripper. The camera is emitted by `scripts/gen_dual_scene.py`'s template
   string, not hand-typed into the generated XML, so per ADR-021's consequences the fix went
   into the generator (`FRONT_CAM_POS`/`FRONT_CAM_TARGET`/`look_at_xyaxes`, a stdlib-only
   look-at basis construction) and the scene was regenerated, not hand-patched. `git diff`
   on the regenerated XML shows only the one `<camera name="front".../>` line changed.
   Re-rendered `docs/images/m02-scene.png`; both arms fully visible on inspection. Overhead
   and wrist cameras were not touched (re-rendered for comparison; identical framing).
   `git diff --stat -- scenes/so101/` is empty throughout.

The package was installed editable on bm-ptl (`pip install -e .`) so `import bimanual` and
`from bimanual.sim.env import TableSettingEnv` resolve from any working directory, per the
task's tester-readiness requirement.

---

## ADR-022 — Opt-in camera rendering in `TableSettingEnv`

**Ratified:** Sept 11, 2026 · **Evidence:** `docs/hardware/m02-render-cost.md`

Rendering all five cameras on every step cost ~810 ms; physics alone is 0.20 ms.
Measured on bm-ptl: state-only 0.20 ms/step, one camera 152.25, all five 809.86 —
a **4049x** spread. Confirmed real GPU time (`Intel(R) Arc(TM) B390 GPU`, OpenGL
4.6), and cameras scale linearly, so there is no fixed cost to amortise.

Rejected: keeping unconditional rendering (dominates step cost for callers that
never read an image); frame caching keyed on unchanged action (policy rollouts
change action every step, so it would never hit).

**Decision.** `cameras=None` by default — render nothing. `reset()` and `step()`
take a per-call `cameras=` override that does not mutate the instance default.
`render()` stays ungated as an explicit escape hatch. Names validate against the
model's discovered cameras, not a hardcoded list.

M09 collects demonstrations with no cameras (ADR-005 uses privileged state, so
every frame it rendered was discarded); M11 trains on
`['front','armA_wrist','armB_wrist']`; M08 scores GATE-1 state-only and enables
cameras only for the final recorded run.

---

## ADR-021 — Dual-arm scene composed by scripted renaming, not MJCF `<include>`

**Recorded:** Sept 11, 2026 · **Module:** M02 (dual-SO-101 table scene)

`PLAN.md` M02 named two candidate ways to duplicate the unmodified SO-101 arm for the
bimanual scene: MJCF `<include>`, or hand-copying the body tree. MuJoCo requires every
body, joint, site and actuator name to be unique across the whole compiled model.

Tested empirically on bm-ptl before deciding, via `scripts/probe_include_namespace.py`
(mujoco 3.2.7): `<include>`-ing `scenes/so101/so101_new_calib.xml` twice does not fail on
a naming collision — it fails earlier, because MuJoCo's compiler refuses to include the
same file twice at all: `ValueError: XML Error: File 'scenes/so101/so101_new_calib.xml'
already included / Element 'include', line 4`. `<include>` has no prefix attribute, so
even a byte-identical second copy under a different filename would still collide on every
body/joint/site/actuator name. `<include>` is therefore not viable for arm duplication,
full stop — this is the deciding factor, established by measurement rather than by reading
the MJCF spec alone.

Hand-copying the ~120-line body tree twice by hand was rejected in favour of
`scripts/gen_dual_scene.py`: a stdlib-only (`xml.etree.ElementTree`, no `mujoco` import,
runs on the laptop despite ADR-020) script that parses the unmodified upstream
`so101_new_calib.xml`, deep-copies its body tree twice, prefixes every body/joint/site name
(`armA_`/`armB_`), repositions each copy, generates a matching renamed actuator pair, and
splices the result into a hand-authored table/drawer/props/cameras template. Mesh and
material definitions are declared once, shared by both arm copies, since geometry is not
per-instance data. The deciding factor over hand-copying: a script guarantees the two arm
copies stay byte-faithful to upstream and to each other, resynchronized by a single command
rather than by two independent manual edits with nothing to catch a missed rename — the same
transcription-drift failure mode ADR-016 already flagged for the single-arm case.

Layout: SO-ARM100 reach is approximately 0.30 m (stated assumption), so arm bases are
placed 0.50 m apart on the table's two long edges (0.25 m off centreline each), leaving a
roughly 0.10 m wide band at the table centre reachable by both arms, where the five props
sit. Full context, options and consequences in `ARCHITECTURE.md` ADR-021.

---

## M01 — RISK-07 resolved: `openvino-telemetry` pin unified to `2025.2`

**Recorded:** Sept 11, 2026 · **Closes:** RISK-07 · **Module:** M01 (repo scaffold and
pinned environments)

`PLAN.md` RISK-07 flagged a discrepancy: `scripts/requirements-bmptl.txt:3` pinned
`openvino-telemetry==2025.2` while `benchmarks/bmptl-environment.txt:3` recorded
`openvino-telemetry==2025.2.` with a trailing period.

Read both files directly during M01: as of commit `e5d74eb` (before this module's own
work began) both already read `openvino-telemetry==2025.2`, with no trailing period and
no other differences on that line. The trailing-period form was never a real,
installable pin — `pip index versions openvino-telemetry` lists PyPI releases as
`2025.2.0`, `2025.1.0`, `2025.0.1`, etc. (three-component versions only); `2025.2.` is
not one of them and is not valid PEP 440 syntax as a distinct release. `2025.2` is a
valid pin and resolves to `2025.2.0` under PEP 440's version-normalization rules (a
trailing implicit zero), so `2025.2` — not `2025.2.` — is the correct value, and it is
the one both files share.

No further edit to either file was needed; this entry exists because the done-when
criterion for M01 requires the resolution to be *recorded* here, not only present in the
files. `scripts/verify_env.py` (this module's other output) checks
`scripts/requirements-bmptl.txt` package-by-package against installed versions
whenever it is pointed at that file, so a future re-introduction of the mismatched
trailing-period form on either file would surface as a plain string mismatch against
whichever file is passed to `--requirements`, not as a silent drift.

---

## ADR-020 — Simulation runs on bm-ptl, not the laptop

**Ratified:** Sept 11, 2026 · **Closes:** RISK-11 · **Promotes:** RISK-03 to blocking

`mujoco.MjModel.from_xml_path` fails on the laptop with `OSError: [WinError 4551]` —
Windows Smart App Control blocks the unsigned `mujoco.dll`. Reproduced on `mujoco==3.13.0`
and `3.2.7`, so it is an OS policy issue, not an asset or package defect
(`scenes/so101/VERIFICATION.md`).

Options: (a) disable Smart App Control locally — one-way and a standing security
regression on the daily machine, **rejected**; (b) run simulation on bm-ptl — dev matches
the deployment target the brief requires, gains Arc B390 and 32 GB, costs an SSH iteration
tax, **chosen**; (c) add WSL2 — another environment to pin plus EGL quirks, **deferred**
but retained as the escape hatch.

**Decision.** The laptop is for code, git and the Speechmatics client. bm-ptl runs all
MuJoCo work.

The outstanding MuJoCo compile-check of the SO-101 asset moves to bm-ptl, as does M02's
rendered PNG and every sim module's iteration loop. Note the cost: with (a) rejected and
(c) unbuilt, **RISK-03 (offscreen rendering on bm-ptl) now has no fallback** and the M03
rendering probe becomes a blocking prerequisite. bm-ptl's Sept 17 00:15 expiry now bounds
simulation development, not just benchmarking, so work must stay pushed to git rather than
living on the instance.

---

## M02 prerequisite: SO-101 asset acquisition — adopted from TheRobotStudio/SO-ARM100 (fulfills ADR-016)

**Recorded:** Sept 11, 2026 · **Closes:** RISK-01 · **Fulfills:** ADR-016

Per the user-specified search order for this prerequisite, option (a) — the Intel Hack-a-thon Resources
bundle referenced by a button on the challenge platform page
(`docs/challenge/Screenshot 2026-09-11 132213.png`) — could not be reached: no URL for
it appears anywhere in the challenge-brief PDF (checked programmatically for link
annotations, none found) or elsewhere in this repository. Proceeded to option (b):
**TheRobotStudio/SO-ARM100**, `https://github.com/TheRobotStudio/SO-ARM100`, commit
`eecbe3e0a9ebb23e25ad7b2759b03884c6660903`, files under `Simulation/SO101/`. License
**Apache-2.0**, permissive and public-repo-compatible; text copied to
`scenes/so101/LICENSE`. Full provenance in `scenes/so101/PROVENANCE.md`.

Per ADR-016, the asset's shipped kinematics were taken unmodified. Actuated DoF per
arm, read from `so101_new_calib.xml`'s `<actuator>` block: **6** — `shoulder_pan`,
`shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`, all hinge joints
under `<position>` actuators. This closes RISK-02's remaining "what is the number"
question with a measurement; it feeds ARCHITECTURE.md section 1's contract table at
GATE-1 as ADR-016 specified.

`mujoco.MjModel.from_xml_path('scenes/so101/scene.xml')` could not be executed
successfully on this laptop: Windows Smart App Control (enforced, not evaluation
mode) blocks loading `mujoco.dll` because it is an unsigned binary with insufficient
Microsoft cloud reputation — confirmed via Windows Event Log
`Microsoft-Windows-CodeIntegrity/Operational` (Event ID 3077/3118) and
`Get-AuthenticodeSignature` reporting `NotSigned`. This reproduces identically on
`mujoco==3.13.0` and `mujoco==3.2.7`, so it is an OS policy issue, not an asset or
package defect. A plain-XML structural check (no MuJoCo) confirms the files are
well-formed, all 6 actuated joints and all 13 referenced mesh files are present and
resolvable. Full verbatim command/output and diagnosis in
`scenes/so101/VERIFICATION.md`. Flagged as a new, previously unlisted risk
(**RISK-11**, not yet added to `PLAN.md` — planner's file, not builder's to edit) for
the planner/user to decide: disable Smart App Control on this laptop, verify on
bm-ptl instead, or add WSL as a local dev path.

---

## ADR-019 — Speechmatics credentials via gitignored `.env`

**Ratified:** Sept 11, 2026 · **Closes:** RISK-04 (handling) · **Relates to:** ADR-002

The Speechmatics API key lives in `.env`, which is gitignored. Code reads it from the
environment as `SPEECHMATICS_API_KEY`, never from a literal. A committed `.env.example`
names the variable with an empty value so setup stays reproducible. A missing key fails
loudly at startup; it must not silently disable voice.

Verified at ratification: `.gitignore:23` ignores `.env`; the local `.env` is untracked;
no `.env`, `*.pem` or `.kaggle` path appears in git history. A credential scan runs before
the repo is flipped public on Sept 15.

---

## ADR-018 — 10-seed evaluation reports the true success rate

**Ratified:** Sept 11, 2026 · **Closes:** RISK-06 · **Strengthens:** ADR-015 rule 4

The 10 evaluation seeds are declared in the eval config before the run and are not
changed afterward to improve the result. The reported rate is successes over those 10.
Cherry-picking 10 successes from a larger pool is prohibited. Failure modes must be
shown and narrated in the video, not merely tabulated in the repo — if 7 of 10 succeed,
the video says 7 of 10 and shows what the other 3 did.

Re-running a seed after a code change is normal iteration. Swapping the seed set after
seeing results is not; the committed seed list makes the difference auditable.

---

## ADR-017 — `pour` is a tilt-and-position motion, with mandatory disclosure

**Ratified:** Sept 11, 2026 · **Closes:** RISK-08 · **Confirms:** ADR-011

Arm B holds the mug, arm A brings the bottle over it and achieves a tilt-and-hold pose
within a stated tolerance. Success is pose achievement. No fluid, particle or volume is
simulated.

The absence of fluid must be stated in `README.md` and spoken in the video narration —
not a caption, not a footnote, not repo-only prose, because the video reaches judges who
may never open the repo. Compliance-reviewer treats a missing statement as a defect.

---

## ADR-016 — SO-101 DoF is whatever the asset ships, unmodified

**Ratified:** Sept 11, 2026 · **Closes:** RISK-02 · **Feeds:** M01, M02, ADR-013

The adopted SO-101 asset's shipped DoF is authoritative. The kinematics are not edited to
suit our controller — no locked joints, no added DoF. IK, ACT action dimensions and
OpenVINO input shapes adapt to the asset instead. Builder reports the actual DoF in
M01/M02 and it is written into the `ARCHITECTURE.md` section 1 contract table then.

Scene composition — arm placement, table, objects, cameras — is ours. The arm model is not.
