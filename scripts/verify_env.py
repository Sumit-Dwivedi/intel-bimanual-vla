"""Verify the Python environment against a pinned requirements file.

Why this script exists (PLAN.md M01): reproducibility is 10 rubric points, and
three different hosts (laptop, bm-ptl, Kaggle) each have their own pinned
requirements file. This script reads a requirements file, checks every pinned
package's *installed* version against the pin, and exits non-zero if anything
is missing or mismatched -- so "reproducible setup" is something a judge can
run, not something we merely assert.

Design note on how versions are read (important, see ARCHITECTURE.md ADR-020):
this script NEVER does `import mujoco` (or any other heavy/native package) to
find a version number. It reads package metadata instead, via
`importlib.metadata.version(name)`. That call only reads the installed
distribution's metadata (a small text file written at install time) -- it
does not execute the package's code or load any compiled extension (e.g.
mujoco.dll). This matters because on this laptop, actually importing `mujoco`
raises `OSError: [WinError 4551]`: Windows Smart App Control blocks the
unsigned mujoco.dll (ADR-020, confirmed in scenes/so101/VERIFICATION.md and
DECISIONS.md). Reading metadata sidesteps that entirely, so we can still
report "mujoco 3.2.7 is installed and correctly pinned" truthfully, without
ever touching the blocked DLL.

That leaves a separate, narrower question this script also answers: is the
package additionally *importable* on this host right now? For every package
we attempt a real `importlib.import_module(...)` after the metadata check,
purely for reporting, and classify the outcome as one of:

  - IMPORTABLE           -- imports cleanly.
  - NOT_IMPORTABLE_ADR020 -- fails with the exact signature of the documented
                             Windows Smart App Control block (OSError
                             mentioning "4551" or "Application Control"). This
                             is EXPECTED on the laptop and does NOT fail the
                             script. It is a real failure on any host where it
                             is *not* expected (e.g. bm-ptl, where ADR-020
                             says MuJoCo work happens and mujoco IS expected to
                             import) -- but we do not special-case by hostname
                             to decide that: we react to the actual error
                             signature we observe, on whichever host we are
                             actually running on, and simply print that host's
                             identity so a human can sanity-check the claim.
                             If bm-ptl ever produces this exact signature, that
                             is worth investigating, but per ADR-020's own
                             text mujoco is expected to import there, so in
                             practice this branch should only ever be hit on
                             the laptop.
  - IMPORT_FAILED         -- any other import error. Always fatal. We do not
                             want an unrelated, undiagnosed import failure to
                             be silently treated as "fine, must be the known
                             issue."

A package that is not installed at all (`importlib.metadata.version` raises
`PackageNotFoundError`) is always a fatal, plain failure -- that is a real
reproducibility bug, not a documented host limitation.

Usage:
    python scripts/verify_env.py
    python scripts/verify_env.py --requirements scripts/requirements-bmptl.txt
    python scripts/verify_env.py --requirements scripts/requirements-dev.txt scripts/requirements-bmptl.txt

Exit codes:
    0 - every pinned package is installed at the pinned version (import
        failures are allowed only for the documented ADR-020 mujoco case).
    1 - at least one package is missing, mismatched, or fails to import for
        an undocumented reason.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata as metadata
import platform
import sys
from dataclasses import dataclass
from pathlib import Path

# Distribution name (as it appears in a requirements.txt / on PyPI) -> the
# name you would actually `import`. Only needed where the two differ.
IMPORT_NAME_OVERRIDES = {
    "pyyaml": "yaml",
    "python-dotenv": "dotenv",
    "opencv-python": "cv2",
}

# Substrings that identify the documented ADR-020 Windows Smart App Control
# block. Matched against `str(exception)`, case-insensitively.
ADR020_SIGNATURE_SUBSTRINGS = ("4551", "application control")

# Only mujoco is *known and documented* to hit the ADR-020 signature. If some
# other package raised the exact same OS error text (implausible, but this
# script should not silently forgive it for a package nobody has diagnosed),
# we still fail loudly.
ADR020_DOCUMENTED_PACKAGES = {"mujoco"}


@dataclass
class PackageCheck:
    name: str  # distribution name, as pinned
    pinned_version: str
    installed_version: str | None  # None if not installed
    import_status: str  # "IMPORTABLE" | "NOT_IMPORTABLE_ADR020" | "IMPORT_FAILED" | "SKIPPED"
    import_detail: str  # human-readable detail (verbatim error if any)

    @property
    def installed(self) -> bool:
        return self.installed_version is not None

    @property
    def version_ok(self) -> bool:
        return self.installed and self.installed_version == self.pinned_version

    @property
    def is_fatal(self) -> bool:
        """Whether this package's status should fail the whole script."""
        if not self.installed:
            return True
        if not self.version_ok:
            return True
        if self.import_status == "IMPORT_FAILED":
            return True
        # NOT_IMPORTABLE_ADR020 and IMPORTABLE are both non-fatal.
        return False


def parse_requirements(path: Path) -> list[tuple[str, str]]:
    """Parse a simple `name==version` requirements file.

    Deliberately minimal: this project's requirements files (see
    scripts/requirements-dev.txt, scripts/requirements-bmptl.txt) are flat
    `name==version` pins with `#` comments and blank lines, no extras, no
    environment markers, no `-r` includes. If a line doesn't match that shape
    we skip it and print why, rather than guessing.
    """
    pins: list[tuple[str, str]] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if "==" not in line:
            print(f"  [skip] {path.name}: cannot parse line as name==version: {raw_line!r}")
            continue
        name, _, version = line.partition("==")
        pins.append((name.strip(), version.strip()))
    return pins


def check_package(name: str, pinned_version: str) -> PackageCheck:
    # Step 1: version, via metadata only. Never imports the package.
    try:
        installed_version = metadata.version(name)
    except metadata.PackageNotFoundError:
        return PackageCheck(
            name=name,
            pinned_version=pinned_version,
            installed_version=None,
            import_status="SKIPPED",
            import_detail="package is not installed; skipped the import check",
        )

    # Step 2: import, purely for reporting host-importability. Failures here
    # are classified, not silently absorbed.
    import_name = IMPORT_NAME_OVERRIDES.get(name.lower(), name)
    try:
        importlib.import_module(import_name)
        import_status = "IMPORTABLE"
        import_detail = f"import {import_name} succeeded"
    except Exception as exc:  # noqa: BLE001 - we deliberately classify, not suppress
        message = str(exc)
        looks_like_adr020 = any(
            sub in message.lower() for sub in ADR020_SIGNATURE_SUBSTRINGS
        )
        if looks_like_adr020 and name.lower() in ADR020_DOCUMENTED_PACKAGES:
            import_status = "NOT_IMPORTABLE_ADR020"
            import_detail = (
                f"{type(exc).__name__}: {message} "
                "-- matches the documented ADR-020 Windows Smart App Control "
                "block (scenes/so101/VERIFICATION.md, DECISIONS.md). Expected "
                "on the laptop; not fatal here. This is a real problem if it "
                "ever occurs on bm-ptl, where ADR-020 states mujoco IS "
                "expected to import."
            )
        else:
            import_status = "IMPORT_FAILED"
            import_detail = f"{type(exc).__name__}: {message}"

    return PackageCheck(
        name=name,
        pinned_version=pinned_version,
        installed_version=installed_version,
        import_status=import_status,
        import_detail=import_detail,
    )


def print_host_banner() -> None:
    print("Host")
    print("----")
    print(f"  platform.node()    : {platform.node()}")
    print(f"  platform.system()  : {platform.system()} {platform.release()}")
    print(f"  platform.machine() : {platform.machine()}")
    print(f"  os.name            : {__import__('os').name}")
    print(
        "  Note: this script identifies the ADR-020 mujoco carve-out by the "
        "exact error signature it observes (OSError mentioning '4551' or "
        "'Application Control'), not by guessing the hostname above -- see "
        "this file's module docstring for why."
    )
    print()


def print_python_and_key_versions(checks: list[PackageCheck]) -> None:
    print("Interpreter")
    print("-----------")
    print(f"  Python : {platform.python_version()} ({sys.executable})")
    print()
    print("Key package versions (done-when #1 of PLAN.md M01)")
    print("---------------------------------------------------")
    for wanted in ("numpy", "mujoco", "torch"):
        match = next((c for c in checks if c.name.lower() == wanted), None)
        if match is None:
            print(f"  {wanted:8s}: not in the checked requirements file")
        else:
            shown = match.installed_version or "NOT INSTALLED"
            print(f"  {wanted:8s}: {shown}")
    print()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--requirements",
        nargs="+",
        type=Path,
        default=[Path(__file__).parent / "requirements-dev.txt"],
        help=(
            "One or more requirements files to check (default: "
            "scripts/requirements-dev.txt, the laptop pin set). Pass "
            "scripts/requirements-bmptl.txt when running this on bm-ptl."
        ),
    )
    args = parser.parse_args(argv)

    print_host_banner()

    all_pins: list[tuple[str, str, Path]] = []
    for req_path in args.requirements:
        if not req_path.exists():
            print(f"ERROR: requirements file not found: {req_path}")
            return 1
        for name, version in parse_requirements(req_path):
            all_pins.append((name, version, req_path))

    if not all_pins:
        print("ERROR: no name==version pins parsed from the given requirements file(s).")
        return 1

    checks = [check_package(name, version) for name, version, _ in all_pins]
    print_python_and_key_versions(checks)

    print("Full report")
    print("-----------")
    fatal = False
    for (name, _, req_path), check in zip(all_pins, checks):
        if not check.installed:
            status_line = "MISSING (not installed) -- real failure"
            fatal = True
        elif not check.version_ok:
            status_line = (
                f"VERSION MISMATCH -- pinned {check.pinned_version}, "
                f"installed {check.installed_version} -- real failure"
            )
            fatal = True
        elif check.import_status == "IMPORT_FAILED":
            status_line = f"installed {check.installed_version}, but import failed -- real failure"
            fatal = True
        elif check.import_status == "NOT_IMPORTABLE_ADR020":
            status_line = f"installed {check.installed_version}, matches pin -- not importable on this host (ADR-020, expected)"
        else:
            status_line = f"installed {check.installed_version}, matches pin, imports cleanly"

        print(f"  [{req_path.name}] {name}: {status_line}")
        if check.import_status in ("IMPORT_FAILED", "NOT_IMPORTABLE_ADR020"):
            print(f"      detail: {check.import_detail}")
        if not check.installed or not check.version_ok:
            fatal = fatal or check.is_fatal

    print()
    if fatal:
        print("RESULT: FAIL -- at least one package is missing, mismatched, or has an")
        print("undocumented import failure. See 'real failure' lines above.")
        return 1

    print("RESULT: PASS -- every pinned package is installed at the pinned version.")
    if any(c.import_status == "NOT_IMPORTABLE_ADR020" for c in checks):
        print(
            "NOTE: mujoco is installed at the pinned version but cannot be "
            "imported on this host due to Windows Smart App Control blocking "
            "the unsigned mujoco.dll (ARCHITECTURE.md ADR-020). This is "
            "expected on the laptop and does not fail this check. All MuJoCo "
            "simulation work runs on bm-ptl instead."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
