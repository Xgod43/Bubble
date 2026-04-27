#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"
mkdir -p "$BUILD_DIR"

gcc \
  -O3 \
  -ffast-math \
  -fPIC \
  -shared \
  "$ROOT_DIR/blob_detector.c" \
  -o "$BUILD_DIR/libbubble_detect.so"

echo "Built $BUILD_DIR/libbubble_detect.so"
