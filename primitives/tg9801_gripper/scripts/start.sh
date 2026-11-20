#!/usr/bin/env bash
set -euo pipefail
TG_PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
export ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-127.0.0.1}"
export ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}"
export PYTHONPATH="$TG_PKG:$TG_PKG/rbnx-build/codegen/proto_gen:$TG_PKG/rbnx-build/codegen/robonix_mcp_types${PYTHONPATH:+:$PYTHONPATH}"
cd "$TG_PKG"
exec "$TG_PKG/rbnx-build/venv/bin/python" -m tg9801.driver
