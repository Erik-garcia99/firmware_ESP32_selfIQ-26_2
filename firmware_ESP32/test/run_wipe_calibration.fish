#!/usr/bin/env fish
# Wipe protegido de calibraciones locales del ESP32-CAM (selfiq_cal).
#
# Uso:
#   fish test/run_wipe_calibration.fish <ROI_ID>
#       Solo consulta el estado del nodo (no borra nada).
#   fish test/run_wipe_calibration.fish <ROI_ID> --execute
#       Consulta y despues ejecuta el borrado. Borra TODAS las calibraciones
#       de esa camara, no solo la ROI indicada: --roi solo correlaciona la
#       solicitud con la respuesta. Deja constancia en un JSONL local.
#
# Requiere: .venv-vision con paho-mqtt>=2,<3 y el nodo encendido en la LAN.
# No modifica la instalacion de la Raspberry ni publica stock.

set -l roi $argv[1]
set -l execute false
if contains -- --execute $argv
    set execute true
end

if test -z "$roi"
    echo "Uso: fish test/run_wipe_calibration.fish <ROI_ID> [--execute]" >&2
    echo "Acepta 'cal_keys_removed=0' como valido si ya estaba vacio." >&2
    exit 2
end

set -l broker 192.168.1.98
set -l branch d07018c7-99c1-490f-95b9-dd54589cf60b
set -l node 5c485efa-cc20-4f9a-91eb-ceeba0f99289
set -l python .venv-vision/bin/python
set -l log vision_trial_nvs_wipe_(date -u +%Y%m%dT%H%M%SZ).jsonl
set -l session ($python -c 'import uuid; print(uuid.uuid4())')

if not test -x $python
    echo "Falta $python. Crear el entorno con paho-mqtt>=2,<3." >&2
    exit 2
end

echo "broker=$broker"
echo "branch=$branch"
echo "node=$node"
echo "roi=$roi"
echo "session=$session"
echo "log=$log"
echo

set -l common --broker $broker --branch $branch --node $node --roi $roi --session $session --log $log

$python test/vision_trial_client.py $common \
    --action status --condition antes_del_wipe
or begin
    echo "El estado previo fallo; no se ejecuta el wipe." >&2
    exit 1
end

if test $execute = false
    echo
    echo "Consulta solamente. Para borrar, repetir con --execute."
    exit 0
end

echo
echo "ATENCION: se borraran TODAS las calibraciones locales de esta camara."
read -l -P "Escribir ERASE_CAL para continuar: " reply
if test "$reply" != ERASE_CAL
    echo "Cancelado."
    exit 1
end

$python test/vision_trial_client.py $common \
    --action wipe_calibration --confirm-wipe-cal ERASE_CAL \
    --condition borrado_para_montaje_fijo
