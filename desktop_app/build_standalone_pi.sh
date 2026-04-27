#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UI_DIR="$ROOT_DIR/nextjs-gui/out"
NATIVE_LIB="$ROOT_DIR/native/build/libbubble_detect.so"

if [ ! -f "$UI_DIR/index.html" ]; then
  echo "Frontend export not found."
  echo "Run: cd \"$ROOT_DIR/nextjs-gui\" && npm install && npm run build"
  exit 1
fi

if [ ! -f "$NATIVE_LIB" ]; then
  echo "Native detector not found."
  echo "Run: cd \"$ROOT_DIR/native\" && ./build_pi.sh"
  exit 1
fi

python3 -m pip install \
  -r "$ROOT_DIR/desktop_app/requirements-app.txt" \
  -r "$ROOT_DIR/desktop_app/requirements-build.txt"
pyinstaller "$ROOT_DIR/desktop_app/pi_bubble_app.spec" --noconfirm --clean

echo "Standalone app created under $ROOT_DIR/dist/PiBubbleMissionControl"
