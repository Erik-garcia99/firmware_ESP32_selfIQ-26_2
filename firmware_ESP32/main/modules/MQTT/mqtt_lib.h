#ifndef SELFIQ_MQTT_LIB_H
#define SELFIQ_MQTT_LIB_H
#include <stdint.h>
#include "esp_err.h"

/* Se llama una sola vez DESPUES de conectarse al Wi-Fi definitivo. */
esp_err_t mqtt_register_start(const char *broker_ip, uint16_t broker_port);
#endif
