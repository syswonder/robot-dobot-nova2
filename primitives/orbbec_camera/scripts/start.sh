#!/usr/bin/env bash
set -euo pipefail
CAM_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
export ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-127.0.0.1}"
export ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}"
export PYTHONPATH="$CAM_PKG:$CAM_PKG/rbnx-build/codegen/proto_gen:$CAM_PKG/rbnx-build/codegen/robonix_mcp_types${PYTHONPATH:+:$PYTHONPATH}"
cd "$CAM_PKG"
exec "$CAM_PKG/rbnx-build/venv/bin/python" -m orbbec_usb.driver
