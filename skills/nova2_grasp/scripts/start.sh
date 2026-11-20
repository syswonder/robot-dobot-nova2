#!/usr/bin/env bash
set -euo pipefail
GRASP_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
export ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-127.0.0.1}"
export ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}"
export PYTHONPATH="$GRASP_PKG:$GRASP_PKG/rbnx-build/codegen/proto_gen:$GRASP_PKG/rbnx-build/codegen/robonix_mcp_types${PYTHONPATH:+:$PYTHONPATH}"
cd "$GRASP_PKG"
exec "$GRASP_PKG/rbnx-build/venv/bin/python" -m nova2_grasp.driver
