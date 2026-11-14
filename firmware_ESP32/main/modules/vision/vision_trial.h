#ifndef SELFIQ_VISION_TRIAL_H
#define SELFIQ_VISION_TRIAL_H
#include <stdbool.h>
#include "cJSON.h"
#include "esp_err.h"

/* Isolated, opt-in single-ROI pilot. Camera task is sole owner of state/NVS. */
esp_err_t vision_trial_init(void);
esp_err_t vision_trial_submit(const cJSON *root, bool retained);
bool vision_trial_service(void);
/* Non-retained, local diagnostic topic; never production occupancy. */
esp_err_t mqtt_publish_trial_json(const char *payload);
#endif
