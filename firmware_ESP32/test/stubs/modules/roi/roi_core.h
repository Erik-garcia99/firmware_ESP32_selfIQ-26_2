#ifndef MOCK_ROI_CORE_H
#define MOCK_ROI_CORE_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef struct {char id[37];} roi_def_t;
typedef struct {uint32_t config_version;size_t count;roi_def_t rois[8];} roi_blob_t;
bool roi_uuid_valid(const char *);
#endif
