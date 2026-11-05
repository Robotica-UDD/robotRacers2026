#!/bin/bash
set -euo pipefail
ROBOTRACERS_CAMERA_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROBOTRACERS_CAMERA_DIR"
if [ ! -x .venv/bin/python ]; then
  if command -v uv >/dev/null 2>&1; then
    uv sync --frozen
  else
    echo 'Instala uv y ejecuta uv sync --frozen en la carpeta del proyecto.'
    exit 1
  fi
fi
exec .venv/bin/python -m simulation.webcam "$@"
