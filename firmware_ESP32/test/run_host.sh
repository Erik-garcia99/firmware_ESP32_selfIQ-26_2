#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MOD="$ROOT/main/modules/calibration"
OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT
cc -D_POSIX_C_SOURCE=200809L -std=c11 -Wall -Wextra -Werror -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$ROOT/tests/stubs" -I"$MOD" \
  "$MOD/calibration_store.c" "$ROOT/tests/test_calibration_store.c" \
  "$MOD/calibration_core.c" -o "$OUT"
"$OUT"
