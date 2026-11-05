#!/bin/bash
set -euo pipefail
ROBOTRACERS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROBOTRACERS_DIR"
if [ "$(uname -s)" != Darwin ]; then
  echo 'Este instalador está preparado para macOS. Consulta README para otros sistemas.'
  exit 1
fi
if ! command -v clang++ >/dev/null 2>&1; then
  echo 'Faltan las herramientas de compilación de Apple. Instala Command Line Tools (xcode-select --install) y repite.'
  exit 1
fi
if ! command -v npm >/dev/null 2>&1; then
  if command -v brew >/dev/null 2>&1; then
    HOMEBREW_NO_AUTO_UPDATE=1 brew install node
  else
    echo 'Instala Node.js 24 LTS y vuelve a ejecutar este archivo.'
    exit 1
  fi
fi
if ! command -v uv >/dev/null 2>&1; then
  if command -v brew >/dev/null 2>&1; then
    HOMEBREW_NO_AUTO_UPDATE=1 brew install uv
  else
    echo 'Instala uv desde https://docs.astral.sh/uv/getting-started/installation/ y vuelve a ejecutar este archivo.'
    exit 1
  fi
fi
uv python install 3.12
uv sync --frozen --group dev
export WEBOTS_HOME="${WEBOTS_HOME:-/Applications/Webots.app/Contents}"
if [ ! -d "$WEBOTS_HOME" ]; then
  if [ "$WEBOTS_HOME" != /Applications/Webots.app/Contents ]; then
    echo 'La ruta WEBOTS_HOME indicada no existe. Instala Webots R2025a allí o utiliza la ruta predeterminada.'
    exit 1
  fi
  ROBOTRACERS_DMG="${TMPDIR:-/tmp}/robotracers-webots-R2025a.dmg"
  curl --fail --location --retry 3 --output "$ROBOTRACERS_DMG" https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a.dmg
  hdiutil attach -nobrowse -readonly "$ROBOTRACERS_DMG"
  ditto /Volumes/Webots/Webots.app /Applications/Webots.app
  hdiutil detach /Volumes/Webots
fi
if [ "$(cat "$WEBOTS_HOME/Resources/version.txt")" != 'R2025a' ]; then
  echo 'Este proyecto requiere Webots R2025a. Conserva tu otra versión e instala R2025a en una ruta definida mediante WEBOTS_HOME.'
  exit 1
fi
.venv/bin/python simulation/tools/vendor_webots_web.py
.venv/bin/python simulation/tools/build_controller.py
.venv/bin/python simulation/tools/build_world.py
cd simulation/ui
npm ci
npm run build
echo 'Preparado. Ejecuta simulation/run.command desde la carpeta del proyecto.'
