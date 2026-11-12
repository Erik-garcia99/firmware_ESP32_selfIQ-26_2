#!/usr/bin/env bash
# Prueba host del almacén NVS (borrador, doble banco, selector y CRC).
# Usa los stubs de ESP-IDF de test/stubs, pero la cabecera ROI REAL: el stub
# test/stubs/modules/roi/roi_core.h tiene otro layout y corrompe el test.
# Uso: test/run_host.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MOD="$ROOT/main/modules/calibration"
OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT
cc -D_POSIX_C_SOURCE=200809L -std=c11 -Wall -Wextra -Werror -O1 -g \
  -Wno-error=deprecated-declarations \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$ROOT/main" -I"$MOD" -I"$ROOT/test/stubs" \
  "$MOD/calibration_store.c" "$ROOT/test/test_calibration_store.c" \
  "$MOD/calibration_core.c" -o "$OUT"
"$OUT"
