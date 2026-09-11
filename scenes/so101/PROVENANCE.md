# SO-101 asset provenance (M01)

## Source chosen

Option (b) from M01's search order: **TheRobotStudio/SO-ARM100** GitHub repository.
Option (a), the "Intel Hack-a-thon Resources" bundle referenced by a button on the
challenge platform page (`docs/challenge/Screenshot 2026-09-11 132213.png`), was tried
first and could not be reached: the screenshot shows only a button labelled
"Hack-a-thon Resources" with no visible URL, the challenge-brief PDF's link
annotations were inspected programmatically (`pypdf`) and contain zero embedded URIs,
and no URL for that bundle appears anywhere else in the repository. Without a URL,
option (a) is not actionable, so per the task's fallback instruction I proceeded to
option (b) rather than guess a URL.

## Repository

- **Source URL:** https://github.com/TheRobotStudio/SO-ARM100
- **Commit taken:** `eecbe3e0a9ebb23e25ad7b2759b03884c6660903` (default branch `main`,
  authored 2026-09-06T12:46:39Z — the HEAD of `main` at the time of retrieval,
  2026-09-11).
- **Files taken from:** `Simulation/SO101/` in that commit — `scene.xml`,
  `so101_new_calib.xml` (adopted calibration), `so101_old_calib.xml`,
  `so101_new_calib_camera.xml` (wrist-camera variant, for M02's camera-placement
  decision), `joints_properties.xml`, `README.md` (copied here as
  `UPSTREAM_README.md`), `LICENSE`, and all 13 `.stl` mesh files referenced by
  `so101_new_calib.xml`'s `<asset>` block, from `Simulation/SO101/assets/`.
- **Retrieved via:** `raw.githubusercontent.com` at the pinned commit SHA above (not
  `main`, so this is reproducible even if upstream `main` moves).

## License

**Apache License 2.0** (SPDX `Apache-2.0`), as reported by the GitHub repository API
(`license.spdx_id: Apache-2.0`) and as the literal license text copied to
`scenes/so101/LICENSE` from the repository root at the pinned commit. Apache-2.0 is
permissive and compatible with inclusion in a public GitHub repository, provided
attribution and the license text are retained — both are satisfied by this file and by
the co-located `LICENSE` file. No separate license file exists inside
`Simulation/SO101/`; the repository-root `LICENSE` governs the whole repository
including these simulation assets.

## What was NOT found (RISK-01, option (a))

The Intel Hack-a-thon Resources bundle mentioned in the challenge page
(`docs/challenge/Screenshot 2026-09-11 132213.png`, tag "Hack-a-thon Resources")
could not be located within the 90-minute budget. No URL for it exists in:
- the challenge-brief PDF text or its link annotations (checked with `pypdf`,
  zero URIs found across all 5 pages' annotations),
- any file already in this repository (`grep -ri "hack-a-thon\|hackathon" README.md
  SUBMISSION.md DECISIONS.md` found no URL, only the phrase itself in SUBMISSION.md).

This is reported honestly rather than guessed. If the user has the actual URL (e.g.
from the original challenge platform login), option (a) can be revisited — but option
(b) is a complete, working, permissively-licensed substitute and unblocks M02, so no
further time was spent searching for (a).

## Actuated DoF per arm (RISK-02 / ADR-016)

Read directly from `so101_new_calib.xml`'s `<actuator>` block (unmodified, as
ADR-016 requires):

| # | Actuator name | Joint | Type | ctrlrange (rad) |
|---|---|---|---|---|
| 1 | `shoulder_pan` | `shoulder_pan` | hinge, position-controlled | -1.91986 to 1.91986 |
| 2 | `shoulder_lift` | `shoulder_lift` | hinge, position-controlled | -1.74533 to 1.74533 |
| 3 | `elbow_flex` | `elbow_flex` | hinge, position-controlled | -1.69 to 1.69 |
| 4 | `wrist_flex` | `wrist_flex` | hinge, position-controlled | -1.65806 to 1.65806 |
| 5 | `wrist_roll` | `wrist_roll` | hinge, position-controlled | -2.74385 to 2.84121 |
| 6 | `gripper` | `gripper` | hinge, position-controlled | -0.17453 to 1.74533 |

**Actuated DoF per arm = 6** (5 arm joints + 1 gripper joint), all revolute (hinge),
all driven by MuJoCo `<position>` actuators. Two arms => 12 actuated DoF total for the
bimanual scene, before any objects (drawer slide, etc.) are added in M02. This value
is unmodified from the upstream asset per ADR-016 and should be written into
`ARCHITECTURE.md` section 1's contract table at GATE-1 as planned.

Note for M02: the gripper joint here is a hinge, not the linear 0–100 "open/closed"
convention LeRobot uses — the upstream `UPSTREAM_README.md` (Gripper Note) says this
mapping is not yet reflected in the MJCF/URDF. M02's IK/skill code must convert
between these conventions explicitly rather than assume LeRobot's linear gripper unit.

## Verification status

See `scenes/so101/VERIFICATION.md` for the exact `mujoco.MjModel.from_xml_path`
command run and its result on this laptop.

## Apache-2.0 §4(d) — NOTICE file

Checked 2026-09-11. The upstream repository at commit
`eecbe3e0a9ebb23e25ad7b2759b03884c6660903` contains **no NOTICE file**, so §4(d) imposes
no carry-forward obligation.

Evidence: `GET https://api.github.com/repos/TheRobotStudio/SO-ARM100/git/trees/eecbe3e0a9ebb23e25ad7b2759b03884c6660903?recursive=1`
returned 293 tree entries with `truncated: false`; zero paths match `/notice/i`; the only
license file in the entire tree is the root `LICENSE`.

Integrity: our `scenes/so101/LICENSE` reproduces that file verbatim — git blob sha
`261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64`, 11357 bytes, identical to the upstream blob
sha at the same commit. A provenance note is appended below the license text, separated by
a horizontal rule; the Apache-2.0 terms themselves are unaltered.
