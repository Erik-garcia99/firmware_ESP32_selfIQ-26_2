#!/usr/bin/env bash
# Fase 5 E1: sin ESP-IDF, cJSON, Python ni hardware.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$(mktemp -d "${TMPDIR:-/tmp/opencode}/selfiq-vision.XXXXXX")"
trap 'rm -rf "$OUT"' EXIT
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
  -fsanitize=address,undefined -fno-sanitize-recover=all \
  "$ROOT/test/test_vision_core.c" \
  "$ROOT/main/modules/vision/vision_core.c" \
  "$ROOT/main/modules/calibration/calibration_core.c" -o "$OUT/vision"
"$OUT/vision"
