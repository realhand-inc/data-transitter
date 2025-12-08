"""Launcher for the RealHand desktop UI.

This module ensures the repository root is on sys.path, then imports and
runs the existing Tkinter UI in scripts/RH/adb_control_gui.py. It is meant
to be the PyInstaller entrypoint and can also be invoked directly for
development.
"""

from __future__ import annotations

import importlib
import os
import sys
import traceback
from pathlib import Path
from typing import Optional


def _add_repo_root_to_sys_path() -> Path:
    """Add repo root and legacy script folder to sys.path for imports."""
    # launcher.py -> realhand_app -> src -> application -> repo root
    repo_root = Path(__file__).resolve().parents[3]
    repo_str = str(repo_root)
    scripts_rh = repo_root / "scripts" / "RH"

    if repo_str not in sys.path:
        sys.path.insert(0, repo_str)
    scripts_rh_str = str(scripts_rh)
    if scripts_rh_str not in sys.path:
        sys.path.insert(1, scripts_rh_str)  # before repo root so `import test_adb_simple` works

    return repo_root


def _import_adb_gui():
    """Import the existing adb_control_gui module."""
    return importlib.import_module("scripts.RH.adb_control_gui")


def _pause_if_tty():
    """Pause in interactive shells (helps when double-clicking a .sh)."""
    try:
        if sys.stdin.isatty():
            input("Press Enter to exit...")
    except Exception:
        pass


def main() -> Optional[int]:
    """Entrypoint used by both development and packaged builds."""
    repo_root = _add_repo_root_to_sys_path()

    # Align working directory with repo root so relative paths behave as expected.
    try:
        os.chdir(repo_root)
    except Exception:
        # If we cannot change directories, continue anyway but log for visibility.
        print(f"Warning: failed to change working directory to {repo_root}", file=sys.stderr)

    # Basic guard for environments without a display
    if os.name != "nt" and not os.environ.get("DISPLAY"):
        print("No DISPLAY detected. Please run from a graphical session.", file=sys.stderr)
        _pause_if_tty()
        return 1

    try:
        adb_gui = _import_adb_gui()
    except Exception:
        print("Failed to import scripts.RH.adb_control_gui", file=sys.stderr)
        traceback.print_exc()
        _pause_if_tty()
        return 1

    if not hasattr(adb_gui, "main"):
        print("scripts.RH.adb_control_gui is missing a main() function to launch the UI.", file=sys.stderr)
        _pause_if_tty()
        return 1

    try:
        adb_gui.main()
    except Exception:
        traceback.print_exc()
        _pause_if_tty()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
