# vision_core — contrato Fase 5 / E1

Implementado en PC el 12/11/2026. **No integrado ni habilitado en cámara**:
no cambia CMake, tareas, NVS, resolución, MQTT ni comportamiento desplegado.
Los parámetros son candidatos reproducibles, no parámetros de piloto aprobados.

## API y memoria

`vision_measure(input, empty, full, out)` es C11 puro. Recibe gris de 8 bits,
longitud, ancho/alto/stride, ROI en millonésimas (`cal_roi_t`), contexto esperado,
tiempo monotónico en ms y dos vistas prestadas de los bytes persistidos. No
copia el frame ni asigna heap; usa memoria automática acotada (tres descriptores
de calibración y tres arreglos de 64 enteros, además de locales). No hace I/O.
El presupuesto real de stack/tiempo en ESP sigue pendiente de E3; no asumir que
el margen del worker observado antes de visión basta.

Todas las entradas deben permanecer inmutables durante la llamada, `out` no
debe solaparse con ellas. No hay estado global mutable; admite llamadas
independientes. `out` se limpia, incluido padding: idénticas entradas producen
resultado idéntico byte a byte. El tiempo lo aporta el llamador, no se consulta
un reloj. `out == NULL` devuelve `bad_argument` sin escritura.

`VISION_OK` es la única autorización para usar el porcentaje. En cualquier
rechazo, `occupancy_pct=255` (sentinela, **no telemetría**), `confidence=0` y
`status` explica el motivo. Métricas no alcanzadas quedan en cero. No representar
un rechazo como 0%, ni publicar eventos de error por frame.

## Contexto y compatibilidad

- Formato exacto: 66 bytes `[1][8][64 medias]`. Versiones desconocidas, 16×16,
  longitudes distintas y CRC32 IEEE incorrecto se rechazan. Se reutilizan
  `cal_descriptor_decode` y `cal_descriptor_crc32`; no se modifica el formato.
- Contexto esperado, empty y full deben coincidir en ROI UUID, hash de contexto,
  configuración y versión de calibración. UUID/hash canónicos en minúsculas,
  terminados en NUL; versiones positivas. El core compara campos, no padding.
- `NULL` en una referencia significa `uncalibrated`; bytes inválidos en una
  referencia presente significan `invalid_reference`.
- El core **no puede demostrar** que un contexto proviene del banco activo,
  ni autenticar CRC/hash proporcionados por el llamador. E3 debe obtener un
  snapshot coherente de ROI + pareja activa validada, verificar captura/commit
  y vinculación al contexto real (geometría/producto/cámara). No fabricar el
  contexto esperado copiándolo de la referencia. No usar borradores.

## Extracción y calidad

Se llama directamente a `cal_descriptor_extract(..., 8, ...)`: mismo recorte
floor(inicio)/ceil(fin), límites semiabiertos y medias redondeadas de calibración.
No hay segunda conversión de ROI. El core permite tamaños host arbitrarios
válidos con ROI de al menos 8×8 píxeles, `stride >= width` y
`length >= stride*height`; ignora padding fuera del ancho.

En E3 el adaptador debe exigir **VGA gris 640×480, stride 640, longitud exacta
307200**, formato y edad válidos según `calibration_frame`. Una matriz de bytes
no permite al core detectar JPEG, edad o un buffer liberado. Estos chequeos no
se sustituyen por los del core.

Media = media redondeada de las 64 celdas; contraste = max−min de celdas;
saturación = porcentaje de píxeles <=5 o >=250, redondeado a entero por
`calibration_core`. Los bytes persistidos no contienen saturación original:
**no se inventa ni se usa como calidad de referencia**. Referencias con
contraste/MAD insuficientes o no normalizables se rechazan.

## Algoritmo entero congelado para evaluación E2

Para cada descriptor, usando sus celdas `v[i]` y media entera `m`:

```text
S = sum(abs(v[i] - m))                  # MAD = S/64
n[i] = round_signed((v[i]-m)*32*256*64 / S)
```

`round_signed` redondea la magnitud al entero más cercano (empate alejándose de
cero). Los numeradores usan `int64_t`. Los normalizados son enteros Q8 con centro
cero y MAD objetivo 32. La ganancia permitida es `[0.5, 2]`, no se fuerza una
ganancia extrema ni se recortan celdas para ocultar errores. Cada descriptor se
normaliza independientemente y las referencias originales nunca se escriben.

```text
MAE_Q8(a,b) = (sum(abs(a[i]-b[i])) + 32) / 64   # división entera
separation = MAE_Q8(empty, full)
d_empty = MAE_Q8(current, empty)
d_full = MAE_Q8(current, full)
D = d_empty + d_full
occupancy_pct = clamp((100*d_empty + D/2)/D, 0, 100)
```

Umbrales candidatos (inclusive en límites aceptados):

| Control | Regla / rechazo |
|---|---|
| Contraste de celdas | >=16; frame: `low_contrast`, referencia: `invalid_reference` |
| MAD | 16..64; frame: `normalization_rejected`, referencia: `invalid_reference` |
| Saturación actual | <=20% entero; si excede: `excessive_clipping` |
| Media actual | entre min(media empty/full)−24 y max(media empty/full)+24 |
| Separación normalizada | >=4 grises (1024 Q8); si falla: `indistinguishable_references` |
| Denominador | >=1024 Q8, guardia explícita incluso si la separación ya lo garantiza |

Precedencia: argumentos/contexto esperado → presencia de referencias → coincidencia
de contextos → formato/CRC → extracción/frame/ROI → saturación actual → calidad de
referencias → separación → deriva de media → normalización actual → distancias.
Cuando varias condiciones fallan se devuelve la primera, no una lista.

La confianza es consistencia con el segmento de referencias, **no probabilidad
ni precisión**:

```text
consistency = min(100, round(100*separation/D))
confidence = round(consistency*(100-clipped_pct)/100)
```

Un punto intermedio compatible puede tener confianza 100. Una escena ajena puede
tener confianza menor; ni una confianza alta demuestra que no se movió la cámara.
No implementa histéresis, suavizado, estados de stock ni detección `cam_moved`.

### Limitación deliberada que E2 debe evaluar

Centrar/escalar elimina información absoluta de brillo/contraste. Si vacío y
lleno solo difieren en esas variables, su separación desaparece: se rechaza la
pareja. Un nivel intermedio sin textura también puede rechazarse. La invariancia
a iluminación es local y condicionada, **no una garantía general**. El cociente
de distancias tampoco demuestra conteo ni linealidad con unidades. Si las fotos
reales no sostienen el algoritmo, ajustar montaje/ROI o revisar esta decisión en
E2 y registrar evidencia antes de integrar. No cambiar a 16×16 por intuición.

## Pruebas reproducibles

Desde `firmware_ESP32_SelfIQ-26-2/firmware_ESP32`:

```bash
bash test/run_vision_host_tests.sh
CC=clang bash test/run_vision_host_tests.sh
bash test/run_host_tests.sh
```

El runner dedicado usa C11, `-Wall -Wextra -Werror -pedantic`, ASan y UBSan;
no depende de ESP-IDF ni cJSON. Usa `${TMPDIR:-/tmp/opencode}` para temporales
(crear/cambiar TMPDIR si se ejecuta en otra máquina). La suite general incluye
visión y las regresiones de calibración, y sí requiere el cJSON existente.

Casos: 0/25/50/75/100 sintéticos en VGA, redondeo, cambios leves de brillo y
ganancia, determinismo byte a byte, referencias inmutables, contexto/CRC/formato,
ROI inválidas/límite, stride con padding, frames incompletos/extremos, pareja
idéntica/indistinguible, calidad y límites de ganancia/media/saturación/separación.
Estos patrones son controles matemáticos con textura compartida y señal de
ocupación independiente; **no son un dataset de aceptación ni evidencia de
error real o tiempo de ciclo en placa**.

Próximo paso E2: reunir fotos etiquetadas reales 0/25/50/75/100 con ROI/pareja
aplicadas, comparar resultados por nivel y documentar fallos, error y duración.
Los umbrales y la política de publicación delta/heartbeat quedan pendientes de
aprobar con esa evidencia. La prueba cámara→app corresponde a E3–E6.
