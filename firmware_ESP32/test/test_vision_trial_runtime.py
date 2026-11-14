"""Compile the actual device adapter with mock hardware/NVS, not a rewrite."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUBS = {
    'esp_err.h': '''
#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 1
#define ESP_ERR_INVALID_ARG 2
#define ESP_ERR_INVALID_STATE 3
#define ESP_ERR_INVALID_SIZE 4
#define ESP_ERR_NVS_NOT_FOUND 4354
#define ESP_ERR_NVS_NOT_ENOUGH_SPACE 4357
const char *esp_err_to_name(int);
''',
    'esp_heap_caps.h': '''
#pragma once
#include <stddef.h>
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_8BIT 2
#define MALLOC_CAP_INTERNAL 4
void *heap_caps_calloc(size_t,size_t,unsigned);
void *heap_caps_malloc(size_t,unsigned);
size_t heap_caps_get_free_size(unsigned);
''',
    'esp_log.h': '''
#pragma once
void mock_log(const char *,const char *,...);
#define ESP_LOGI(...) mock_log(__VA_ARGS__)
#define ESP_LOGW(...) mock_log(__VA_ARGS__)
#define ESP_LOGE(...) mock_log(__VA_ARGS__)
''',
    'esp_timer.h': '#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',
    'freertos/FreeRTOS.h': '''
#pragma once
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) (ms)
''',
    'freertos/queue.h': '''
#pragma once
#include <stddef.h>
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned,size_t);
int xQueueSend(QueueHandle_t,const void *,unsigned);
int xQueueReceive(QueueHandle_t,void *,unsigned);
''',
    'freertos/task.h': '#pragma once\nvoid vTaskDelay(unsigned);\n',
    'nvs.h': '''
#pragma once
#include <stddef.h>
#include "esp_err.h"
typedef int nvs_handle_t;
typedef struct {size_t used_entries,free_entries,available_entries,total_entries,namespace_count;} nvs_stats_t;
#define NVS_READWRITE 1
#define NVS_READONLY 0
int nvs_open(const char *,int,nvs_handle_t *);
int nvs_set_blob(nvs_handle_t,const char *,const void *,size_t);
int nvs_get_blob(nvs_handle_t,const char *,void *,size_t *);
int nvs_commit(nvs_handle_t);
int nvs_erase_key(nvs_handle_t,const char *);
void nvs_close(nvs_handle_t);
int nvs_get_stats(const char *,nvs_stats_t *);
''',
    'modules/roi/roi_store.h': '''
#pragma once
#include "modules/roi/roi_core.h"
bool roi_store_snapshot(roi_blob_t *);
''',
    'modules/camera/camera_lib.h': '''
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>
#include "modules/calibration/calibration_core.h"
#define CAMERA_CAL_FRAME_WIDTH 640U
#define CAMERA_CAL_FRAME_HEIGHT 480U
#define PIXFORMAT_GRAYSCALE 1
#define OV3660_PID 0x3660
typedef struct sensor_s sensor_t;
struct sensor_s {struct {int PID;} id; int (*get_reg)(sensor_t *,int,int);};
sensor_t *esp_camera_sensor_get(void);
typedef struct {uint8_t *buf; size_t len; uint32_t width,height;
                int format; struct timeval timestamp;} camera_fb_t;
camera_fb_t *camera_capture(void);
void camera_release(camera_fb_t *);
''',
}


class VisionTrialRuntimeTests(unittest.TestCase):
    def test_actual_adapter(self):
        with tempfile.TemporaryDirectory(dir='/tmp/opencode') as directory:
            temp = Path(directory)
            for name, text in STUBS.items():
                path = temp / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
            command = [
                os.environ.get('CC', 'gcc'), '-std=c11', '-O1', '-g',
                '-Wall', '-Wextra', '-Werror', '-pedantic',
                '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                '-I', str(temp), '-I', str(ROOT / 'main'),
                '-I', str(ROOT / 'managed_components/espressif__cjson/cJSON'),
                str(ROOT / 'test/vision_trial_runtime.c'),
                str(ROOT / 'main/modules/vision/local_pattern.c'),
                str(ROOT / 'main/modules/vision/local_guard.c'),
                str(ROOT / 'main/modules/vision/vision_runtime.c'),
                str(ROOT / 'main/modules/calibration/calibration_core.c'),
                str(ROOT / 'main/modules/roi/roi_core.c'),
                str(ROOT / 'managed_components/espressif__cjson/cJSON/cJSON.c'),
                '-lm', '-o', str(temp / 'trial'),
            ]
            subprocess.run(command, check=True, capture_output=True, text=True)
            result = subprocess.run([str(temp / 'trial')], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
