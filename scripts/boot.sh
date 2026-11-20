#!/usr/bin/env bash
set -euo pipefail
NOVA2_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$NOVA2_ROOT"
if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi
: "${VLM_BASE_URL:?Fill VLM_BASE_URL in .env or the shell environment}"
: "${VLM_API_KEY:?Fill VLM_API_KEY in .env or the shell environment}"
: "${VLM_MODEL:?Fill VLM_MODEL in .env or the shell environment}"
if [[ "$VLM_API_KEY" == replace-me || "$VLM_MODEL" == replace-with-your-model ]]; then
  echo "Edit .env and replace the example model credentials before boot." >&2
  exit 1
fi
exec rbnx boot --no-update-check "$@"
