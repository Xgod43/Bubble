#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ESCAPED_ROOT="${ROOT_DIR// /\\ }"
TARGET_DIR="${HOME}/Desktop"
TARGET_FILE="$TARGET_DIR/PiBubbleMissionControl.desktop"

mkdir -p "$TARGET_DIR"
cat > "$TARGET_FILE" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Pi Bubble Mission Control
Comment=Soft gripper bubble console
Exec=python3 $ESCAPED_ROOT/main.py
Path=$ROOT_DIR
Terminal=false
Categories=Utility;Engineering;
EOF

chmod +x "$TARGET_FILE"

echo "Desktop launcher installed at $TARGET_FILE"
