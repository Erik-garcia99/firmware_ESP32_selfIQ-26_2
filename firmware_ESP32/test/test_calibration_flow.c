/* Integración host E5.0/E5.1: frame -> filtro -> descriptor 8x8 v1 -> ACK. */
#include <stdio.h>
#include <string.h>

#include "../main/modules/calibration/calibration_cmd.h"
#include "../main/modules/calibration/calibration_core.h"
#include "../main/modules/calibration/calibration_frame.h"

static int g_fail = 0, g_run = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    ++g_run;                                                           \
    if (!(c)) {                                                        \
      ++g_fail;                                                        \
      printf("FALLO %s:%d: %s\n", __FILE__, __LINE__, #c);             \
    }                                                                  \
  } while (0)

#define OP "11111111-2222-4333-8444-555555555555"
#define REQ "aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee"
#define CAL "9f1c6a4e-0f6d-4a63-9a1b-2a5c7d1e0b11"
#define ROI "3a3e7d54-9b9f-4d4e-8c6b-1f0a2b3c4d5e"
#define CAP "c0ffee00-0000-4000-8000-000000000001"
#define H64 "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"

static uint8_t frame[320 * 240];

int main(void) {
  /* frame sintético con gradiente */
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 320; ++x)
      frame[y * 320 + x] = (uint8_t)((x * 255) / 319);

  /* 1) el filtro acepta el frame de 320x240 y rechaza el de 72960 B */
  cal_frame_info_t info = {true, 320, 240, sizeof frame, 2000000};
  cal_frame_stats_t stats;
  memset(&stats, 0, sizeof stats);
  CHECK(cal_frame_check(&info, 320, 240, 1000000, 2100000, CAL_FRAME_MAX_AGE_MS, &stats) == CAL_FRAME_OK);
  info.len = 72960;
  CHECK(cal_frame_check(&info, 320, 240, 1000000, 2100000, CAL_FRAME_MAX_AGE_MS, &stats) == CAL_FRAME_E_LENGTH);
  info.len = sizeof frame;

  /* 2) comando -> ROI en pixeles -> descriptor v1 */
  const char *cmd_json =
      "{\"schema_version\":1,\"type\":\"calibrate\",\"id\":\"" OP
      "\",\"req_id\":\"" REQ "\",\"calibration_id\":\"" CAL
      "\",\"calibration_version\":3,\"roi_id\":\"" ROI
      "\",\"config_version\":8,\"context_hash\":\"" H64
      "\",\"capture_id\":\"" CAP
      "\",\"kind\":\"empty\",\"format_version\":1,"
      "\"issued_at\":\"2026-11-09T21:40:00.000Z\","
      "\"expires_at\":\"2026-11-09T21:40:10.000Z\"}";
  cal_cmd_t cmd;
  cal_cmd_status_t st;
  CHECK(cal_cmd_parse(cmd_json, strlen(cmd_json), &cmd, &st) == CAL_CMD_OK);

  /* bbox de la ROI activa del log (millonesimas) */
  cal_roi_t roi = {79044, 518627, 599265, 477941};
  cal_rect_t rect;
  CHECK(cal_roi_to_rect(320, 240, roi, &rect) == CAL_OK);
  CHECK(rect.left == 25 && rect.top == 124 && rect.right == 218 && rect.bottom == 240);

  cal_descriptor_t d;
  CHECK(cal_descriptor_extract(frame, sizeof frame, 320, 240, 320, roi,
                               CAL_DESCRIPTOR_GRID_WIDTH, &d) == CAL_OK);
  uint8_t bytes[CAL_DESCRIPTOR_MAX_BYTES];
  size_t n = 0;
  CHECK(cal_descriptor_encode(&d, bytes, sizeof bytes, &n) == CAL_OK);
  CHECK(n == 66U); /* 2 + 8*8, contrato v1 */
  CHECK(bytes[0] == CAL_CMD_FORMAT_VERSION && bytes[1] == 8U);

  /* 3) el ACK usa el CRC del descriptor y cabe en 1024 B */
  cal_capture_ack_t cap;
  memset(&cap, 0, sizeof cap);
  cap.descriptor_crc = cal_descriptor_crc32(&d);
  memset(cap.image_sha256, 'a', CAL_HASH_HEX_LEN);
  cap.image_bytes = 2021;
  cap.width = 320;
  cap.height = 240;
  cap.quality.mean = d.mean;
  cap.quality.contrast = d.contrast;
  cap.quality.clipped_pct = d.clipped_pct;
  cap.clock_quality = "unknown";
  cap.uptime_ms = 4000000;
  cap.persisted = true;
  char ack[CAL_ACK_JSON_SIZE];
  CHECK(cal_ack_capture(ack, sizeof ack, &cmd, &cap));
  char crc_hex[CAL_CRC_HEX_SIZE];
  snprintf(crc_hex, sizeof crc_hex, "%08lx", (unsigned long)cap.descriptor_crc);
  CHECK(cal_crc_hex_valid(crc_hex) && strstr(ack, crc_hex));

  /* 4) decodificar de vuelta da el mismo CRC (ida y vuelta íntegra) */
  cal_descriptor_t back;
  CHECK(cal_descriptor_decode(bytes, n, &back) == CAL_OK);
  CHECK(cal_descriptor_crc32(&back) == cap.descriptor_crc);

  /* 5) ROI de una version distinta del conjunto -> config_mismatch */
  CHECK(cal_cmd_check_context(&cmd, true, 8) == CAL_CMD_OK);
  CHECK(cal_cmd_check_context(&cmd, true, 7) == CAL_CMD_E_CONFIG_MISMATCH);

  printf("calibration_flow: %d comprobaciones, %d fallos (ACK %zu B)\n", g_run, g_fail, strlen(ack));
  return g_fail ? 1 : 0;
}
