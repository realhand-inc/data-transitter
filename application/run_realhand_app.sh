#!/usr/bin/env bash
# Simple double-clickable launcher for Linux file managers.
# Uses repo venv if present, otherwise falls back to system python3.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -x "${REPO_ROOT}/venv/bin/python" ]]; then
  PY="${REPO_ROOT}/venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
  PY="$(command -v python3)"
else
  echo "python3 not found; please install Python 3.10+ or create venv." >&2
  exit 1
fi

export PYTHONPATH="${SCRIPT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
cd "${REPO_ROOT}"
exec "${PY}" "${SCRIPT_DIR}/src/realhand_app/launcher.py" "$@"
