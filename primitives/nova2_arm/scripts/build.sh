#!/usr/bin/env bash
set -euo pipefail
NOVA2_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
NOVA2_API="$(rbnx path robonix-api)"
if [[ ! -x "$NOVA2_PKG/rbnx-build/venv/bin/python" ]]; then
  uv venv --python 3.11 "$NOVA2_PKG/rbnx-build/venv"
fi
uv pip install --python "$NOVA2_PKG/rbnx-build/venv/bin/python" \
  -r "$NOVA2_PKG/requirements.txt" "$NOVA2_API"
RBNX_CODEGEN_PYTHON="$NOVA2_PKG/rbnx-build/venv/bin/python" rbnx codegen -p "$NOVA2_PKG" --mcp
