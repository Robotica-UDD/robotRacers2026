#!/bin/bash
set -u
ROBOTRACERS_UPLOAD_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROBOTRACERS_UPLOAD_DIR" || exit 1
if [ ! -x .venv/bin/python ]; then
  echo 'No se encontró el entorno Python del proyecto. Ejecuta primero la instalación de RobotRacers.'
  ROBOTRACERS_UPLOAD_RESULT=1
else
  .venv/bin/python -m simulation.webcam.upload_robot "$@"
  ROBOTRACERS_UPLOAD_RESULT=$?
fi
if [ -t 0 ]; then
  printf '\nPulsa Enter para cerrar esta ventana. '
  read -r ROBOTRACERS_UPLOAD_REPLY
fi
exit "$ROBOTRACERS_UPLOAD_RESULT"
