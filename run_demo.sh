#!/usr/bin/env bash
# Thin wrapper around run_demo.py.
#
# Why this file exists separately from run_demo.py: README.md and
# CONTRIBUTING.md already tell an evaluator to run `./run_demo.sh`, so that
# filename must exist at the repo root or those docs break on day one. But
# bm-ptl -- this project's only MuJoCo-capable machine (ARCHITECTURE.md
# ADR-020) -- is Windows, with no native bash to run a `.sh` file directly.
# So the real, portable logic lives in `run_demo.py` (plain Python, runs
# identically on Linux, macOS, or Windows via `py`/`python`/`python3`), and
# this script's only job is to find a Python interpreter and exec it,
# forwarding every argument unchanged.
#
# Usage:
#   ./run_demo.sh
#   ./run_demo.sh --seed 3
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# Try python3 then python, in that order -- but a name being ON PATH is not
# enough to trust it: Windows ships a `python3`/`python` App Execution Alias
# stub (Microsoft Store) on PATH by default that `command -v` happily finds
# but which does not actually run Python at all (it prints a Store prompt
# and exits 0 -- confirmed on this project's own dev laptop). So each
# candidate is verified with a real `--version` invocation before being
# trusted, and the loop keeps trying candidates rather than stopping at the
# first PATH match. Neither name is hardcoded to a specific venv path --
# activate the right environment (see docs/SETUP.md) before running this.
PYTHON_BIN="${PYTHON_BIN:-}"
if [ -z "$PYTHON_BIN" ]; then
  for candidate in python3 python; do
    if command -v "$candidate" >/dev/null 2>&1 && "$candidate" --version >/dev/null 2>&1; then
      PYTHON_BIN="$candidate"
      break
    fi
  done
fi

if [ -z "$PYTHON_BIN" ]; then
  echo "FATAL: no working python3 or python found on PATH (a Windows Store alias stub" >&2
  echo "does not count and is detected and skipped). Activate the venv described in" >&2
  echo "docs/SETUP.md (ov_env on bm-ptl, or scripts/requirements-dev.txt on Linux/macOS)" >&2
  echo "before running this script, or set PYTHON_BIN explicitly." >&2
  exit 1
fi

exec "$PYTHON_BIN" "$SCRIPT_DIR/run_demo.py" "$@"
