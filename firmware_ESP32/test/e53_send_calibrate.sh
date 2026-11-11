#!/usr/bin/env bash
# E5.3 — cliente MQTT de prueba para una captura de calibración.
#
# NO se ejecuta durante la implementación de E5.3. Se usa al final de la Fase 4
# (E5.7), con el nodo encendido, suscrito y con la config_version vigente.
#
# Uso:
#   BRANCH=<uuid-sucursal> \
#   NODE=<uuid-nodo> \
#   ROI=<uuid-roi> \
#   CONFIG_VERSION=<version-aplicada> \
#   CONTEXT_HASH=<64-hex-de-la-huella> \
#   KIND=empty|full \
#   BROKER_HOST=localhost BROKER_PORT=1883 \
#   ./test/e53_send_calibrate.sh
#
# El comando usa el sobre del Anexo A.7.1. En E5.3 el firmware NO publica el
# ACK de éxito (status=stored corresponde a E5.4). Por eso el script escucha
# unos segundos y luego recuerda revisar el monitor serie:
#
#   [CAL] comando aceptado
#   [CAL] captura solicitada
#   CAL: captura encolada
#   CAL_CAPTURE_OK: ... crc=XXXXXXXX bytes=... sha=... upload=queued
#   CAL_UPLOAD_OK: ... HTTP 200 ...
#
# Si ya hay una captura pendiente, el nodo sí responde un ACK de error `busy`.
set -euo pipefail

: "${BRANCH:?Falta BRANCH (UUID de sucursal)}"
: "${NODE:?Falta NODE (UUID del nodo)}"
: "${ROI:?Falta ROI (UUID de la zona)}"
: "${CONFIG_VERSION:?Falta CONFIG_VERSION (versión aplicada en el nodo)}"
: "${CONTEXT_HASH:?Falta CONTEXT_HASH (64 hex de la huella vigente)}"
KIND="${KIND:-empty}"
BROKER_HOST="${BROKER_HOST:-localhost}"
BROKER_PORT="${BROKER_PORT:-1883}"
ACK_TIMEOUT="${ACK_TIMEOUT:-8}"

for tool in uuidgen mosquitto_pub mosquitto_sub date; do
  command -v "$tool" >/dev/null 2>&1 || {
    echo "Falta la herramienta: $tool" >&2
    exit 2
  }
done
case "$KIND" in empty|full) ;; *) echo "KIND debe ser empty o full" >&2; exit 2;; esac
[[ "$CONTEXT_HASH" =~ ^[0-9a-fA-F]{64}$ ]] || {
  echo "CONTEXT_HASH debe tener 64 hex" >&2
  exit 2
}

OPERATION_ID="$(uuidgen)"
REQ_ID="$(uuidgen)"
CALIBRATION_ID="$(uuidgen)"
CAPTURE_ID="$(uuidgen)"
ISSUED_AT="$(date -u +%Y-%m-%dT%H:%M:%S.000Z)"
EXPIRES_AT="$(date -u -d '+10 seconds' +%Y-%m-%dT%H:%M:%S.000Z)"

TOPIC="selfiq/${BRANCH}/${NODE}/shelf_cam/command"
ACK_TOPIC="selfiq/${BRANCH}/${NODE}/shelf_cam/command/ack"

PAYLOAD=$(cat <<JSON
{"schema_version":1,"type":"calibrate","id":"${OPERATION_ID}","req_id":"${REQ_ID}","calibration_id":"${CALIBRATION_ID}","calibration_version":1,"roi_id":"${ROI}","config_version":${CONFIG_VERSION},"context_hash":"${CONTEXT_HASH}","capture_id":"${CAPTURE_ID}","kind":"${KIND}","format_version":1,"issued_at":"${ISSUED_AT}","expires_at":"${EXPIRES_AT}"}
JSON
)

echo "Tópico : ${TOPIC}"
echo "ACK    : ${ACK_TOPIC}"
echo "Payload: ${PAYLOAD}"
echo
echo "Escuchando ${ACK_TIMEOUT}s (E5.3 no emite ACK de éxito; revisa el serie)."
mosquitto_sub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "$ACK_TOPIC" \
  -q 1 -W "$ACK_TIMEOUT" -v &
SUB_PID=$!
sleep 0.3
mosquitto_pub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "$TOPIC" \
  -q 1 -m "$PAYLOAD"
wait "$SUB_PID" || true
echo
echo "Serie esperada: CAL_CAPTURE_OK ... upload=queued y CAL_UPLOAD_OK HTTP 200."
echo "En E5.3 no se confirma pareja ni se escribe NVS; eso es E5.4/E5.5."
