#!/usr/bin/env bash
set -euo pipefail
GRASP_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
GRASP_API="$(rbnx path robonix-api)"
if [[ ! -x "$GRASP_PKG/rbnx-build/venv/bin/python" ]]; then
  uv venv --python 3.11 "$GRASP_PKG/rbnx-build/venv"
fi
uv pip install --python "$GRASP_PKG/rbnx-build/venv/bin/python" \
  -r "$GRASP_PKG/requirements.txt" "$GRASP_API"
RBNX_CODEGEN_PYTHON="$GRASP_PKG/rbnx-build/venv/bin/python" rbnx codegen -p "$GRASP_PKG" --mcp
