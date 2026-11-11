#!/usr/bin/env bash
# Pruebas host del firmware de calibración (E5.0/E5.1/E5.2). No requiere ESP-IDF.
# Uso:  test/run_host_tests.sh [ruta/a/cJSON]
# Layout esperado: firmware_ESP32/{main/modules/calibration, test, test/third_party/cjson}
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FW_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$FW_ROOT"

# cJSON: argumento > $CJSON_DIR > búsqueda en test/third_party/cjson y similares
CJSON="${1:-${CJSON_DIR:-}}"
if [[ -z "$CJSON" ]]; then
  for c in "$SCRIPT_DIR/third_party/cjson" "$SCRIPT_DIR/third_party" \
           "third_party/cjson" "third_party"; do
    if [[ -f "$c/cJSON.c" && -f "$c/cJSON.h" ]]; then CJSON="$c"; break; fi
  done
fi
if [[ -z "$CJSON" || ! -f "$CJSON/cJSON.c" || ! -f "$CJSON/cJSON.h" ]]; then
  echo "Falta cJSON.c/.h en test/third_party/cjson/ (o pasa la carpeta / CJSON_DIR)." >&2
  exit 2
fi
CJSON="$(cd "$CJSON" && pwd)"

M="main/modules/calibration"
T="$SCRIPT_DIR"

CFLAGS=(-std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined
        -fno-sanitize-recover=all -I"$CJSON" -I"$FW_ROOT/main" -I"$M")

OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
fail=0
run() {
  local name="$1"; shift
  if gcc "${CFLAGS[@]}" "$@" -lm -o "$OUT/$name" && "$OUT/$name"; then :; else fail=1; fi
}

# El core se compila siempre (detecta regresiones de cabecera/macros).
gcc "${CFLAGS[@]}" -c "$M/calibration_core.c" -o "$OUT/core.o"
echo "calibration_core: compila limpio con -Werror"

[[ -f "$T/test_calibration_core.c" ]] &&
  run core "$T/test_calibration_core.c" "$M/calibration_core.c"

run frame "$T/test_calibration_frame.c" "$M/calibration_frame.c"
run capture "$T/test_calibration_capture.c" "$M/calibration_capture.c" \
            "$M/calibration_core.c" "$M/calibration_sha256.c"
run cmd   "$T/test_calibration_cmd.c"   "$M/calibration_cmd.c" "$CJSON/cJSON.c"
run flow  "$T/test_calibration_flow.c"  "$M/calibration_cmd.c" "$M/calibration_core.c" \
          "$M/calibration_frame.c" "$CJSON/cJSON.c"
run e53_flow "$T/test_calibration_e53_flow.c" "$M/calibration_capture.c" \
             "$M/calibration_core.c" "$M/calibration_sha256.c" \
             "$M/calibration_frame.c"

[[ $fail -eq 0 ]] && echo "TODAS LAS PRUEBAS HOST PASAN" || { echo "HAY FALLOS"; exit 1; }