#!/usr/bin/env bash
set -euo pipefail
TG_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
TG_API="$(rbnx path robonix-api)"
if [[ ! -x "$TG_PKG/rbnx-build/venv/bin/python" ]]; then
  uv venv --python 3.11 "$TG_PKG/rbnx-build/venv"
fi
uv pip install --python "$TG_PKG/rbnx-build/venv/bin/python" -r "$TG_PKG/requirements.txt" "$TG_API"
RBNX_CODEGEN_PYTHON="$TG_PKG/rbnx-build/venv/bin/python" rbnx codegen -p "$TG_PKG" --mcp
