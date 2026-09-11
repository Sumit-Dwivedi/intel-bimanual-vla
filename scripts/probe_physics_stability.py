"""M02 done-when #4: physics stability probe.

Resets TableSettingEnv(seed=0), steps 1000 times with a zero action, and
checks at EVERY step that:
  1. No NaN appears anywhere in qpos or qvel.
  2. No free-body (the five props: plate, mug, fork, spoon, water_bottle)
     tunnels through the table -- i.e. its z-coordinate never drops below a
     "floor" threshold that a settling prop could never legitimately reach.

Per the task instructions, this is a blocker check: if physics is unstable,
this script reports FAIL and the module stops here (see
docs/hardware/m02-physics-stability.md for the written verdict). No attempt
is made to patch the scene from this script.

Threshold derivation (also recorded in the report):
  The tabletop geom is a box centered at z=0.34 with half-thickness 0.01, so
  its top surface is at z=0.35 (src/bimanual/sim/assets/so101_dual_table.xml,
  "table_top" geom). Every prop's resting geom half-height is at least
  0.004 m (the thinnest is the fork/spoon handle capsule, radius 0.004 m;
  the plate is 0.006 m half-height). A prop settling under gravity and
  contact damping can lose at most a millimetre or two of height as contact
  penetration transiently resolves -- that is normal MuJoCo soft-contact
  behaviour, not tunneling. A prop that has fallen THROUGH the table lands
  on the floor plane at z=0, or on the drawer/table-leg geometry well below
  the tabletop.

  We therefore set FLOOR_Z = 0.30 m: five centimetres below the tabletop
  surface (0.35 m) and comfortably below any plausible settling depth
  (millimetres), but well above the floor (z=0) or any other post-tunneling
  resting height. A false positive would require a prop to sink 5 cm into a
  rigid table it is resting on, which does not happen under normal MuJoCo
  contact resolution at this model's damping/stiffness settings -- so this
  threshold cannot be tripped by ordinary settling, only by an actual
  tunneling event.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import numpy as np  # noqa: E402

from bimanual.sim.env import TableSettingEnv  # noqa: E402

N_STEPS = 1000
SEED = 0
FLOOR_Z = 0.30  # metres; see module docstring for derivation

# The five manipulable props, each a free-jointed body (ADR-021 / gen_dual_scene.py
# scene comments). Free-joint qpos layout per body is [x, y, z, qw, qx, qy, qz],
# so the z-coordinate is qpos index 2 within that body's 7-wide slice.
FREE_BODIES = ["plate", "mug", "fork", "spoon", "water_bottle"]

REPORT_PATH = pathlib.Path(__file__).resolve().parent.parent / "docs" / "hardware" / "m02-physics-stability.md"


def free_body_z_indices(model) -> dict[str, int]:
    """Map each free body name to the qpos index of its z-coordinate.

    MuJoCo lays out qpos as: all non-free joints in body-tree order first
    complications aside, each free joint contributes 7 contiguous qpos
    values starting at `model.jnt_qposadr[joint_id]`. We look up the
    free-joint id owned by each named body via its joint address and read
    the z offset (index 2 of the 7).
    """
    import mujoco

    z_idx = {}
    for name in FREE_BODIES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id == -1:
            raise ValueError(f"body {name!r} not found in model")
        # Each body has model.body_jntnum[body_id] joints starting at
        # model.body_jntadr[body_id]. Free-joint bodies in this scene have
        # exactly one joint (the freejoint).
        jnt_adr = model.body_jntadr[body_id]
        n_jnt = model.body_jntnum[body_id]
        assert n_jnt == 1, f"expected exactly one joint (freejoint) on {name!r}, found {n_jnt}"
        qpos_adr = model.jnt_qposadr[jnt_adr]
        z_idx[name] = qpos_adr + 2  # [x, y, z, qw, qx, qy, qz] -> z is offset 2
    return z_idx


def main() -> int:
    # cameras=None: this probe reads only qpos/qvel, never pixels (see module
    # docstring). Camera rendering is opt-in (M02 refactor); passing
    # cameras=None explicitly takes the fast, state-only path -- no
    # offscreen renderer is even constructed for these 1000 steps. See
    # docs/hardware/m02-render-cost.md for measured cost and ADR-022.
    env = TableSettingEnv(cameras=None)
    z_idx = free_body_z_indices(env.model)
    zero_action = np.zeros(env.model.nu, dtype=np.float64)

    obs = env.reset(seed=SEED, cameras=None)

    nan_step = None
    tunnel_step = None
    tunnel_body = None
    min_z_seen = {name: float(obs["qpos"][idx]) for name, idx in z_idx.items()}

    for step in range(N_STEPS):
        obs, done, info = env.step(zero_action, cameras=None)
        qpos = obs["qpos"]
        qvel = obs["qvel"]

        if nan_step is None and (np.isnan(qpos).any() or np.isnan(qvel).any()):
            nan_step = step
            break  # stop immediately; downstream numbers are meaningless once NaN appears

        for name, idx in z_idx.items():
            z = float(qpos[idx])
            if z < min_z_seen[name]:
                min_z_seen[name] = z
            if tunnel_step is None and z < FLOOR_Z:
                tunnel_step = step
                tunnel_body = name
                break
        if tunnel_step is not None:
            break

    env.close()

    passed = nan_step is None and tunnel_step is None
    steps_completed = (nan_step if nan_step is not None else
                       tunnel_step if tunnel_step is not None else N_STEPS)

    lines = []
    lines.append("# M02 physics stability probe")
    lines.append("")
    lines.append(f"Verdict: **{'PASS' if passed else 'FAIL'}**")
    lines.append("")
    lines.append(f"- Seed: {SEED}")
    lines.append(f"- Steps requested: {N_STEPS}")
    lines.append(f"- Steps completed before stopping: {steps_completed}")
    lines.append(f"- Action: zero action vector (length {env.model.nu}) every step")
    lines.append(f"- Floor threshold (tunneling check): FLOOR_Z = {FLOOR_Z} m")
    lines.append(
        "  - Justification: tabletop surface is at z=0.35 (box geom center "
        "z=0.34, half-thickness 0.01, src/bimanual/sim/assets/so101_dual_table.xml "
        "\"table_top\"). Normal contact settling loses at most ~1-2 mm of "
        "height, never 5 cm. FLOOR_Z=0.30 is 5 cm below the tabletop surface: "
        "unreachable by settling, but well above the floor (z=0) or other "
        "sub-table geometry a tunneled prop would land on."
    )
    lines.append("")
    lines.append("## NaN check")
    if nan_step is None:
        lines.append("No NaN observed in qpos or qvel at any completed step.")
    else:
        lines.append(f"**NaN first observed at step index {nan_step}** (0-indexed).")
    lines.append("")
    lines.append("## Tunneling check")
    if tunnel_step is None:
        lines.append("No free body's z-coordinate dropped below the floor threshold.")
    else:
        lines.append(
            f"**Tunneling detected**: body `{tunnel_body}` first crossed below "
            f"FLOOR_Z={FLOOR_Z} m at step index {tunnel_step} (0-indexed)."
        )
    lines.append("")
    lines.append("## Minimum z observed per free body (over completed steps)")
    lines.append("")
    lines.append("| body | min z (m) |")
    lines.append("|---|---|")
    for name in FREE_BODIES:
        lines.append(f"| {name} | {min_z_seen[name]:.5f} |")
    lines.append("")

    report = "\n".join(lines) + "\n"
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report, newline="\n")

    print(report)
    print(f"(report written to {REPORT_PATH})")

    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
