"""ADR-021 empirical probe: does MJCF <include> support namespacing?

Writes a throwaway scene that <include>s scenes/so101/so101_new_calib.xml
TWICE with no renaming, and attempts to compile it with mujoco.MjModel. The
prediction (from reading the MJCF docs: <include> is a plain textual splice
with no prefix attribute) is that this fails with a duplicate-name error on
the first colliding body/joint/site/actuator name. This script exists to
turn that prediction into a measurement before ADR-021 relies on it.

Run from the repo root on bm-ptl (mujoco cannot import on the laptop,
ADR-020):
    python scripts/probe_include_namespace.py

Exit codes: 0 = it compiled (namespacing worked, ADR-021 assumption wrong,
stop and re-examine); 1 = it failed to compile (confirms the ADR-021
deciding factor, expected outcome).
"""

import os
import sys
import tempfile
import traceback

THROWAWAY = """<mujoco model="include_namespace_probe">
  <compiler angle="radian" meshdir="scenes/so101/assets" autolimits="true"/>
  <include file="scenes/so101/so101_new_calib.xml"/>
  <include file="scenes/so101/so101_new_calib.xml"/>
</mujoco>
"""

# MuJoCo resolves nested <include> paths relative to the including file's own
# directory, so this throwaway file must live at the repo root (same level
# scripts/probe_render.py already assumes scenes/so101/... is relative to).
path = os.path.join(os.getcwd(), "_probe_include_namespace.xml")
with open(path, "w") as f:
    f.write(THROWAWAY)

print("Wrote throwaway double-<include> scene to", path)
print("Attempting mujoco.MjModel.from_xml_path(...) ...")
print()

import mujoco  # noqa: E402

try:
    model = mujoco.MjModel.from_xml_path(path)
    print("COMPILED OK: nq=%d nv=%d nu=%d" % (model.nq, model.nv, model.nu))
    print()
    print("UNEXPECTED: <include> namespaced automatically. ADR-021 needs review.")
    os.remove(path)
    sys.exit(0)
except Exception:
    traceback.print_exc()
    print()
    print("EXPECTED: <include>-based duplication fails on duplicate names.")
    print("This is the empirical deciding factor for ADR-021's hand-copy choice.")
    os.remove(path)
    sys.exit(1)
