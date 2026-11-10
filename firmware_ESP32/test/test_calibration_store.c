#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "nvs.h"
#include "../main/modules/roi/roi_store.h"
#include "calibration_store.h"

static roi_blob_t configured;
const roi_blob_t *roi_store_current(void) {return &configured;}
bool roi_uuid_valid(const char *p) {
    if (!p || strlen(p) != 36) return false;
    for (unsigned i=0;i<36;i++) {
        if (i==8||i==13||i==18||i==23) {if(p[i]!='-')return false;}
        else if(!((p[i]>='0'&&p[i]<='9')||(p[i]>='a'&&p[i]<='f')))return false;
    }
    return true;
}
#define MAX_KEYS 50
#define MAX_PAYLOAD 512
typedef struct {char key[16]; unsigned char buf[MAX_PAYLOAD]; size_t len; bool occupied;} entry_t;
static entry_t entries[MAX_KEYS];
static int fail_selector_commit;
static char pending_selector[16];
static uint8_t pending_selector_value;
static bool has_pending_selector;
static entry_t *entry(const char *key, bool allocate) {
    for(int i=0;i<MAX_KEYS;i++)if(entries[i].occupied && !strcmp(entries[i].key,key))return &entries[i];
    if(allocate)for(int i=0;i<MAX_KEYS;i++)if(!entries[i].occupied){
        entries[i].occupied=true; snprintf(entries[i].key,sizeof(entries[i].key),"%s",key);
        return &entries[i];}
    return NULL;
}
esp_err_t nvs_open(const char *ns,int mode,nvs_handle_t *h){(void)ns;(void)mode;*h=1;return ESP_OK;}
void nvs_close(nvs_handle_t h){(void)h;}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *data,size_t *len){
    (void)h;entry_t *e=entry(key,false);if(!e)return ESP_ERR_NVS_NOT_FOUND;
    if(*len<e->len)return ESP_ERR_INVALID_SIZE;
    memcpy(data,e->buf,e->len);*len=e->len;return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t len){
    (void)h;if(len>MAX_PAYLOAD)return ESP_ERR_INVALID_SIZE;entry_t *e=entry(key,true);
    if(!e)return ESP_ERR_NO_MEM;
    memcpy(e->buf,data,len);e->len=len;return ESP_OK;
}
esp_err_t nvs_get_u8(nvs_handle_t h,const char *key,uint8_t *v){
    (void)h;entry_t *e=entry(key,false);if(!e)return ESP_ERR_NVS_NOT_FOUND;
    if(e->len!=1)return ESP_ERR_INVALID_SIZE;
    *v=e->buf[0];return ESP_OK;
}
esp_err_t nvs_set_u8(nvs_handle_t h,const char *key,uint8_t v){
    (void)h;snprintf(pending_selector,sizeof(pending_selector),"%s",key);
    pending_selector_value=v;has_pending_selector=true;return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h){
    (void)h;if(has_pending_selector){
        if(fail_selector_commit){has_pending_selector=false;fail_selector_commit=0;return ESP_FAIL;}
        entry_t *e=entry(pending_selector,true);if(!e)return ESP_ERR_NO_MEM;
        e->buf[0]=pending_selector_value;e->len=1;has_pending_selector=false;
    }
    return ESP_OK;
}
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key){
    (void)h;entry_t *e=entry(key,false);if(!e)return ESP_ERR_NVS_NOT_FOUND;
    e->occupied=false;return ESP_OK;
}
static const char *ROI="06c87f97-8fad-4684-b80e-1f179dc3b570";
static const char *CAL1="11111111-1111-4111-8111-111111111111";
static const char *CAL2="22222222-2222-4222-8222-222222222222";
static const char *E1="33333333-3333-4333-8333-333333333333";
static const char *F1="44444444-4444-4444-8444-444444444444";
static const char *E2="55555555-5555-4555-8555-555555555555";
static const char *F2="66666666-6666-4666-8666-666666666666";
static char context[65];
static cal_descriptor_t descriptor(uint8_t n){
    cal_descriptor_t d={0};d.grid_side=8;d.cell_count=64;
    memset(d.cells,n,64);return d;
}
static esp_err_t save(const char *cal,const char *capture,int kind, uint8_t n,uint32_t ver){
    cal_descriptor_t d=descriptor(n);
    cal_store_reference_t r={0};
    memcpy(r.roi_id,ROI,37);memcpy(r.calibration_id,cal,37);
    memcpy(r.context_hash,context,65);memcpy(r.capture_id,capture,37);
    r.config_version=7;r.calibration_version=ver;r.kind=kind;r.descriptor=&d;
    return calibration_store_save_draft(&r);
}
int main(void){
    memcpy(configured.rois[0].id,ROI,37);configured.count=1;configured.config_version=7;
    memset(context,'a',64);context[64]=0;
    assert(calibration_store_init()==ESP_OK);
    cal_store_pair_t p;
    assert(calibration_store_get_active(ROI,&p)==ESP_ERR_NVS_NOT_FOUND);
    assert(save(CAL1,E1,CAL_STORE_EMPTY,10,1)==ESP_OK);
    assert(calibration_store_get_active(ROI,&p)==ESP_ERR_NVS_NOT_FOUND);
    assert(calibration_store_get_draft(ROI,&p)==ESP_OK && p.has_empty && !p.has_full);
    assert(calibration_store_commit(ROI,CAL1,E1,F1)==ESP_ERR_INVALID_STATE);
    assert(save(CAL1,F1,CAL_STORE_FULL,150,1)==ESP_OK);
    assert(calibration_store_commit(ROI,CAL1,E1,F1)==ESP_OK);
    assert(calibration_store_init()==ESP_OK); /* reinicio logico */
    assert(calibration_store_get_active(ROI,&p)==ESP_OK && p.calibration_version==1);
    assert(calibration_store_commit(ROI,CAL1,E1,F1)==ESP_OK); /* idempotencia */
    assert(save(CAL1,E1,CAL_STORE_EMPTY,11,1)==ESP_ERR_INVALID_STATE); /* ID reutilizado */
    assert(calibration_store_cancel(ROI,CAL1)==ESP_OK);
    assert(calibration_store_get_active(ROI,&p)==ESP_OK);
    assert(save(CAL2,E2,CAL_STORE_EMPTY,20,2)==ESP_OK);
    assert(save(CAL2,F2,CAL_STORE_FULL,170,2)==ESP_OK);
    fail_selector_commit=1;
    assert(calibration_store_commit(ROI,CAL2,E2,F2)==ESP_FAIL);
    assert(calibration_store_get_active(ROI,&p)==ESP_OK && p.calibration_version==1);
    assert(calibration_store_commit(ROI,CAL2,E2,F2)==ESP_OK);
    assert(calibration_store_init()==ESP_OK);
    assert(calibration_store_get_active(ROI,&p)==ESP_OK && p.calibration_version==2);
    configured.config_version=8;
    assert(calibration_store_get_active(ROI,&p)==ESP_ERR_INVALID_STATE);
    assert(calibration_store_cancel(ROI,CAL2)==ESP_OK); /* limpiar borrador obsoleto */
    puts("PASS: borrador, pareja, reinicio logico, replay, cancel, fallo selector, version ROI");
    return 0;
}
