"""Render the M02 dual-arm table scene to a PNG (PLAN.md M02 done-when #1).

Usage:
    python scripts/view_scene.py --headless --save out/scene.png
    python scripts/view_scene.py --headless --save out/scene.png --camera overhead
    python scripts/view_scene.py --headless --save out/scene.png --scene <path.xml>

Per ADR-020, MuJoCo cannot import on the laptop; this script only runs on
bm-ptl. `--headless` is accepted (and is the only mode actually implemented)
because MuJoCo's interactive viewer needs a display and a running event loop,
neither of which exists over the SSH session this project develops through
-- there is no windowed mode to fall back to here, so the flag is present
for interface clarity and for a future local run rather than to select
between two real code paths today.
"""

from __future__ import annotations

import argparse
import pathlib
import sys

# Make `bimanual` importable when this script is run directly (`python
# scripts/view_scene.py`) without the package having been pip-installed,
# mirroring the other scripts/ probes in this repo.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from bimanual.sim.env import TableSettingEnv  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Render offscreen with no display (the only supported mode; see module docstring).",
    )
    parser.add_argument(
        "--save",
        type=pathlib.Path,
        required=True,
        help="Output PNG path. Parent directories are created if missing.",
    )
    parser.add_argument(
        "--camera",
        default="front",
        help="Camera to render from (default: front).",
    )
    parser.add_argument(
        "--scene",
        type=pathlib.Path,
        default=None,
        help="Path to an MJCF scene file. Defaults to the packaged dual-arm table scene.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Reset seed before rendering (default: 0).",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Render width (default: 1280, the scene's declared framebuffer size).",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Render height (default: 720, the scene's declared framebuffer size).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.headless:
        print(
            "This script only supports --headless rendering (no interactive "
            "viewer is available over the bm-ptl SSH session; see module "
            "docstring). Pass --headless to proceed.",
            file=sys.stderr,
        )
        return 1

    env = TableSettingEnv(
        scene_path=args.scene, render_width=args.width, render_height=args.height
    )
    try:
        env.reset(seed=args.seed)
        frame = env.render(args.camera)

        args.save.parent.mkdir(parents=True, exist_ok=True)
        _write_png(args.save, frame)

        print(
            f"wrote {args.save} ({frame.shape[1]}x{frame.shape[0]}) "
            f"from camera '{args.camera}', scene={env.scene_path}"
        )
        return 0
    finally:
        env.close()


def _write_png(path: pathlib.Path, rgb) -> None:
    """Write an HxWx3 uint8 array as a PNG using only the standard library.

    Mirrors scripts/probe_render.py's writer so this script has no extra
    image-library dependency beyond numpy/mujoco, which are already pinned
    for bm-ptl.
    """
    import struct
    import zlib

    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
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


if __name__ == "__main__":
    raise SystemExit(main())
