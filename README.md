# Firmware ESP32-CAM — SelfIQ

Firmware del módulo de monitoreo de repisas del sistema **SelfIQ**.

Este repositorio contiene exclusivamente el firmware correspondiente a los módulos
basados en **ESP32-CAM + OV2640**.

El objetivo de este módulo es detectar cambios de ocupación en los espacios de una
repisa mediante procesamiento local de imágenes y comunicar los eventos detectados
hacia el gateway del sistema mediante MQTT.

> Este repositorio no contiene el firmware correspondiente a los módulos de aforo,
> monitoreo ambiental o guía mediante LEDs. Dichos módulos se mantienen de forma
> independiente.

---

## Descripción

El módulo ESP32-CAM forma parte de la capa IoT local del sistema SelfIQ.

Cada dispositivo se instala orientado hacia una repisa o zona de almacenamiento y
captura imágenes mediante la cámara OV2640.

En lugar de enviar imágenes completas hacia la nube, el procesamiento se realiza
localmente en el microcontrolador.

El análisis se basa en regiones de interés o **ROI (Regions of Interest)** previamente
configuradas.

Sobre estas regiones se realizan operaciones simples sobre los píxeles para detectar
cambios relacionados con la ocupación del espacio observado.

Cuando se detecta un evento relevante, el dispositivo publica la información mediante
MQTT hacia el broker Mosquitto ejecutado en la Raspberry Pi Zero 2 W.

---

## Función dentro de la arquitectura

El ESP32-CAM no se comunica directamente con servicios de Internet.

El flujo correspondiente a este módulo es:

```text
        Repisa
           │
           ▼
      ┌───────────┐
      │  OV2640   │
      └─────┬─────┘
            │
            ▼
    ┌────────────────┐
    │    ESP32-CAM   │
    │                │
    │ Captura imagen │
    │ Procesa ROI    │
    │ Detecta cambio │
    └───────┬────────┘
            │
            │ MQTT
            ▼
    ┌────────────────┐
    │ Raspberry Pi   │
    │ Zero 2 W       │
    │                │
    │ Mosquitto      │
    │ SQLite         │
    │ Gateway Edge   │
    └───────┬────────┘
            │
            │ HTTPS / TLS
            ▼
        Backend Cloud
