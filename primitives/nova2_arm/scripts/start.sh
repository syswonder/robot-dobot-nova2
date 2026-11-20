#!/usr/bin/env bash
set -euo pipefail
NOVA2_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
export ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-127.0.0.1}"
export ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}"
export PYTHONPATH="$NOVA2_PKG:$NOVA2_PKG/rbnx-build/codegen/proto_gen:$NOVA2_PKG/rbnx-build/codegen/robonix_mcp_types${PYTHONPATH:+:$PYTHONPATH}"
cd "$NOVA2_PKG"
exec "$NOVA2_PKG/rbnx-build/venv/bin/python" -m nova2_arm.driver
