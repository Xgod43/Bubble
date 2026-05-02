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

STEPPER_OUT="$BUILD_DIR/stepper_runner"
if pkg-config --exists libgpiod; then
  gcc \
    -O3 \
    "$ROOT_DIR/stepper_runner.c" \
    -o "$STEPPER_OUT" \
    $(pkg-config --cflags --libs libgpiod)
  echo "Built $STEPPER_OUT"
elif gcc -O3 "$ROOT_DIR/stepper_runner.c" -o "$STEPPER_OUT" -lgpiod >/tmp/pi_bubble_stepper_build.log 2>&1; then
  echo "Built $STEPPER_OUT"
else
  echo "Stepper native runner was not built. Install libgpiod-dev to enable it."
  cat /tmp/pi_bubble_stepper_build.log
fi
