#ifndef STUB_ESP_ERR_H
#define STUB_ESP_ERR_H
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_SIZE 0x103
#define ESP_ERR_INVALID_STATE 0x104
#define ESP_ERR_INVALID_CRC 0x109
#define ESP_ERR_NVS_NOT_FOUND 0x1102
static inline const char *esp_err_to_name(esp_err_t err) {(void)err;return "stub";}
#endif
