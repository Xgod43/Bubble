#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-"$ROOT_DIR/.venv"}"

cd "$ROOT_DIR"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "Python command not found: $PYTHON_BIN" >&2
  exit 1
fi

if [ ! -d "$VENV_DIR" ]; then
  echo "Creating venv with apt package visibility: $VENV_DIR"
  if ! "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"; then
    echo "Could not create venv. Install python3-venv/python3-full first:" >&2
    echo "  sudo apt update" >&2
    echo "  sudo apt install -y python3-venv python3-full" >&2
    exit 1
  fi
fi

"$VENV_DIR/bin/python" -m pip install -U pip setuptools wheel
"$VENV_DIR/bin/python" -m pip install -r "$ROOT_DIR/requirements-pi-bookworm.txt"

echo
echo "Done. Launch with:"
echo "  $VENV_DIR/bin/python main.py"
echo "or:"
echo "  ./launch_bubble_app.sh"
