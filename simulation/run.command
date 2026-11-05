#!/bin/bash
set -euo pipefail
ROBOTRACERS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROBOTRACERS_DIR"
if [ ! -x .venv/bin/python ] || [ ! -d simulation/ui/dist ]; then
  echo 'Primero ejecuta simulation/setup.sh.'
  exit 1
fi
exec .venv/bin/python simulation/launch.py "$@"
