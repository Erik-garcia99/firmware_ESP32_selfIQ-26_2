#ifndef SELFIQ_CAMERA_LIB_H
#define SELFIQ_CAMERA_LIB_H

#include "esp_camera.h"
#include "esp_err.h"

/* AI-Thinker ESP32-CAM, compatible con OV2640/OV3660 si usan ese pinout. */
esp_err_t camera_init(void);
camera_fb_t *camera_capture(void);
void camera_release(camera_fb_t *frame);
esp_err_t camera_set_resolution(framesize_t resolution);
esp_err_t camera_set_quality(int quality); /* 0..63, menor = mejor JPEG */
esp_err_t camera_deinit(void);

#endif
