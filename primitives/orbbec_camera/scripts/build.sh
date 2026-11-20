#!/usr/bin/env bash
set -euo pipefail
CAM_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CAM_API="$(rbnx path robonix-api)"
if [[ ! -x "$CAM_PKG/rbnx-build/venv/bin/python" ]]; then
  uv venv --python 3.11 "$CAM_PKG/rbnx-build/venv"
fi
uv pip install --python "$CAM_PKG/rbnx-build/venv/bin/python" -r "$CAM_PKG/requirements.txt" "$CAM_API"
if command -v powershell.exe >/dev/null 2>&1; then
  "$CAM_PKG/rbnx-build/venv/bin/python" "$CAM_PKG/scripts/setup_windows.py"
else
  uv pip install --python "$CAM_PKG/rbnx-build/venv/bin/python" -r "$CAM_PKG/requirements-sdk.txt"
fi
RBNX_CODEGEN_PYTHON="$CAM_PKG/rbnx-build/venv/bin/python" rbnx codegen -p "$CAM_PKG" --mcp
