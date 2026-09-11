# SO-101 asset verification (M01)

## What was run

```
python -m pip install mujoco     # installed mujoco 3.13.0, then also tried 3.2.7
python -c "import mujoco; m = mujoco.MjModel.from_xml_path('scenes/so101/scene.xml'); print('OK', m.nq)"
```

Working directory: repo root
(`C:\Users\dwive\Documents\Hackathon\AI Infra Summit Hackathon\intel-bimanual-vla`).
Python: 3.11.9 (`C:\Users\dwive\AppData\Local\Programs\Python\Python311\python.exe`).

## Verbatim result — BLOCKED, not a MuJoCo/asset problem

```
Traceback (most recent call last):
  File "<string>", line 1, in <module>
  File "C:\Users\dwive\AppData\Local\Programs\Python\Python311\Lib\site-packages\mujoco\__init__.py", line 35, in <module>
    ctypes.WinDLL(os.path.join(os.path.dirname(__file__), 'mujoco.dll'))
  File "C:\Users\dwive\AppData\Local\Programs\Python\Python311\Lib\ctypes\__init__.py", line 376, in __init__
    self._handle = _dlopen(self._name, mode)
                   ^^^^^^^^^^^^^^^^^^^^^^^^^
OSError: [WinError 4551] An Application Control policy has blocked this file
```

This is **not** an error in the SO-101 asset or in the `mujoco` package. It happens
before any XML is parsed — it happens while Python is loading `mujoco.dll` itself, on
both `mujoco==3.13.0` and `mujoco==3.2.7` (tried both to rule out a version-specific
packaging defect).

## Root cause, diagnosed

This machine has **Windows 11 Smart App Control** turned on
(`HKLM\SYSTEM\CurrentControlSet\Control\CI\Policy!VerifiedAndReputablePolicyState = 1`,
i.e. enforced, not evaluation mode). Windows Event Log
`Microsoft-Windows-CodeIntegrity/Operational` records the block directly:

```
Event ID 3077: Code Integrity determined that a process
(...\Python311\python.exe) attempted to load
...\site-packages\mujoco\mujoco.dll that did not meet the Enterprise signing level
requirements or violated code integrity policy (Policy ID:
{0283ac0f-fff1-49ae-ada1-8a933130cad6}).
Event ID 3118: Smart App Control Block Details
```

`Get-AuthenticodeSignature` on `mujoco.dll` reports `NotSigned`. Smart App Control
blocks unsigned binaries that lack sufficient Microsoft cloud reputation; `numpy` and
`scipy`'s compiled extensions loaded without issue in the same environment (both
unsigned too), so this is a reputation/prevalence judgement Smart App Control makes
per-file, not a blanket "no native code" policy — `mujoco.dll` specifically has not
been recognized. This is a laptop-local OS security setting, independent of MuJoCo,
the SO-101 asset, or this repository.

## What this does and does not mean

- It does **not** mean the SO-101 asset is broken. The static structural check below
  passed, and this exact asset (an unmodified pull from a widely-used upstream repo)
  is loaded successfully by MuJoCo on any machine without this OS restriction.
- It **does** mean `mujoco.MjModel.from_xml_path` cannot be executed on *this specific
  laptop* right now to produce a live-loaded confirmation, and by extension
  `scripts/view_scene.py` (M02) will hit the same block here until this is resolved.
- It has **not** been checked whether bm-ptl has Smart App Control enabled. That
  should be checked early in M02/M03 rather than assumed either way — see
  "Recommendation" below.

## Static structural check performed instead (not a substitute, but partial evidence)

Ran a plain-XML structural check (`xml.etree.ElementTree`, no MuJoCo) against
`scenes/so101/scene.xml` and `scenes/so101/so101_new_calib.xml`:

- Both files parse as well-formed XML.
- `so101_new_calib.xml` model name: `so101_new_calib`.
- 6 named joints found: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`,
  `wrist_roll`, `gripper` (plus 3 unnamed `<joint>` elements inside `<default>` class
  blocks, which is expected MJCF and not a defect).
- 6 actuators found, one per named joint, all `<position>` type.
- 13 mesh files are referenced in `<asset>`; all 13 are present under
  `scenes/so101/assets/` (verified by filesystem check, zero missing).

This confirms the file is well-formed and internally consistent (no dangling mesh
references), which is the most that can be checked without a working MuJoCo runtime
on this machine. It does **not** confirm the model compiles (body tree validity,
inertial consistency, collision geometry, etc.) — only `mujoco.MjModel.from_xml_path`
or the `simulate` viewer can confirm that, and neither ran successfully here.

## Recommendation (for the user / next module)

Pick one before M02 starts in earnest:
1. Disable Smart App Control on this laptop (Settings > Privacy & security > Windows
   Security > App & browser control > Smart App Control — **this is a one-way
   action**: Microsoft does not allow re-enabling it without a clean Windows
   reinstall). Requires the user's explicit decision; not something to change
   unilaterally.
2. Verify mujoco on bm-ptl instead (it is a separate Windows machine and may not have
   Smart App Control enabled, or may be in Evaluation mode which behaves
   differently) — cheap to check, do it during the M03 bm-ptl login.
3. Install WSL (not currently present on this laptop — checked, zero distributions
   registered) and run MuJoCo there for local dev/verification, keeping bm-ptl as the
   Windows-native target. Adds a new dependency/toolchain not currently in the plan;
   flagging rather than doing unilaterally.

This is a new, previously unknown-unknown risk, not covered by RISK-01 through
RISK-10 in `PLAN.md`. It should be escalated to the planner as a new risk (working
title: **RISK-11 — Smart App Control blocks native MuJoCo bindings on the primary dev
laptop**) rather than silently worked around.
