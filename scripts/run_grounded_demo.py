"""M10 Phase 5, Task 5: grounder -> TaskPlan -> ScriptedSkillExecutor, end to
end, rendered as `docs/images/m10-demo-end-to-end.png`.

**Two findings, reported here rather than silently worked around -- NEITHER
is a perception regression; both reproduce identically in oracle mode.**

1. **Grammar gap.** This module's task brief's literal command, "Pick up the
   fork with arm A and hand it to arm B", does NOT parse under M05's frozen
   `RuleGrounder` grammar: `docs/command-grammar.md`'s `handoff` row accepts
   only `hand off`/`handoff`/`pass`/`give` as the verb (`rule_grounder.py`'s
   `_R_HANDOFF` pattern requires literal `hand\s*off`) -- plain "hand it to
   arm B" matches none of the four and raises `UngroundedCommandError`.

2. **A two-clause grounding of "pick up X, then hand it to Y" does not
   compose with the frozen `run_handoff` implementation, in EITHER
   perception mode.** Grounding "Pick up the fork with arm A and give it to
   arm B." (the grammar-supported form of finding 1) produces TWO
   `SkillCall`s: `pick(A, fork)` then `handoff(to_arm=B, from_arm=A, fork)`.
   Executing them in sequence FAILS at handoff's own Phase 1 --
   `weld_attach_failed_after_300_frames` -- because `run_handoff`'s Phase 1
   (ADR-037's docstring) unconditionally re-picks the object via a NESTED
   `run_pick`, assuming it starts at rest on the table. By the time the
   standalone `pick(A, fork)` SkillCall has already run, the fork is
   airborne, held by arm A -- exactly the state `run_place` has an explicit
   `already_held` branch to detect and skip (see `skills_scripted.py`'s
   `run_place` docstring), but `run_handoff` has no equivalent branch.
   Verified directly on bm-ptl, oracle mode, before touching perception:
   `pick(A, fork)` succeeds (final_z=0.3989), then
   `handoff(B, A, fork)` fails immediately with
   `weld_attach_failed_after_300_frames`. This is a genuine, pre-existing
   gap in M06/ADR-037's frozen handoff semantics, not a perception defect,
   and fixing `run_handoff` is out of this module's scope (M10 Phase 5 is
   perception wiring, not an M06 skill-semantics change).

   **This script therefore grounds a SINGLE clause** that already carries
   the same semantics the brief's compound sentence intended
   ("pick up the fork with arm A" is exactly what `run_handoff`'s own Phase 1
   does internally) -- `to_arm`'s destination is mandatory and `from_arm`
   defaults to "the other arm" (`docs/command-grammar.md`'s handoff row),
   so a single "give the fork to arm B" already means "arm A picks up the
   fork and hands it to arm B":

       "Give the fork to arm B."

   which grounds to ONE `SkillCall` (`handoff(to_arm=B, from_arm=A, fork)`),
   composes correctly with `run_handoff` exactly as `run_handoff` was built
   to be invoked, and is verified end to end below.

Runs only on bm-ptl (ADR-020): imports `bimanual.sim.env`.
"""

from __future__ import annotations

import argparse
import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from bimanual.command.events import CommandEvent  # noqa: E402
from bimanual.control.executor import ScriptedSkillExecutor  # noqa: E402
from bimanual.language.rule_grounder import RuleGrounder  # noqa: E402
from bimanual.sim.env import TableSettingEnv  # noqa: E402

# The grammar-supported, composition-correct substitute for the brief's
# literal sentence -- see this module's docstring for both findings this
# rests on.
DEMO_COMMAND = "Give the fork to arm B."

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "docs" / "images" / "m10-demo-end-to-end.png"


def write_png(path: Path, rgb) -> None:
    """Stdlib-only PNG writer, copied verbatim from `scripts/probe_render.py`
    (Pillow is not installed in `ov_env`, the venv this script runs in).
    """
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command", default=DEMO_COMMAND)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--camera", default="front",
                         help="Render camera for the end-state screenshot. 'front' "
                         "(default) is the wide, legible establishing shot used for "
                         "the M06 handoff renders (ADR-038); 'posenet_cam' is a "
                         "close, downward, perception-only angle and is NOT "
                         "recommended for a human-facing demo image.")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    print(f"Command: {args.command!r}")
    grounder = RuleGrounder()
    plan = grounder.ground(CommandEvent(text=args.command, timestamp=0.0, source_id="demo"))
    print(f"TaskPlan ({len(plan.skills)} skills):")
    for call in plan.skills:
        print(f"  {call}")

    env = TableSettingEnv(cameras=None)
    env.reset(seed=args.seed)
    executor = ScriptedSkillExecutor()  # oracle mode -- ADR-046 default

    for call in plan.skills:
        result = executor.execute(call, env)
        print(f"executed {call.skill}({call.arm}, {call.target_object}): "
              f"success={result.success} reason={result.reason}")
        if not result.success:
            print(f"STOPPING: {call.skill} did not succeed -- the plan cannot continue honestly.")
            env.close()
            return 1

    weld = executor.weld
    holding_a = weld.is_holding("A")
    holding_b = weld.is_holding("B")
    print(f"\nFinal state: is_holding('A')={holding_a!r} (expect None), "
          f"is_holding('B')={holding_b!r} (expect 'fork')")

    ok = holding_a is None and holding_b == "fork"
    print(f"VERIFICATION: {'PASS' if ok else 'FAIL'}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    frame = env.render(args.camera)
    write_png(args.out, frame)
    print(f"Wrote {args.out} (camera={args.camera}, shape={frame.shape})")

    env.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
