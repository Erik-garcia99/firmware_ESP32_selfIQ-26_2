#ifndef MOCK_ROI_STORE_H
#define MOCK_ROI_STORE_H
#include "roi_core.h"
const roi_blob_t *roi_store_current(void);
bool roi_store_snapshot(roi_blob_t *out);
#endif
