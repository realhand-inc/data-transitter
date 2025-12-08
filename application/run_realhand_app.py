"""
Double-clickable launcher for the RealHand desktop UI.

This thin wrapper adds `application/src` to sys.path so Python can find the
`realhand_app` package, then delegates to its launcher. It keeps relative paths
stable for both development and future packaging.
"""

from __future__ import annotations

import sys
from pathlib import Path


def _bootstrap_paths():
    here = Path(__file__).resolve().parent
    src_dir = here / "src"
    src_str = str(src_dir)
    if src_str not in sys.path:
        sys.path.insert(0, src_str)


def main():
    _bootstrap_paths()
    from realhand_app.launcher import main as launch_main

    return launch_main()


if __name__ == "__main__":
    raise SystemExit(main() or 0)
