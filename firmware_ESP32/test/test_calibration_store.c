#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "nvs.h"
#include "../main/modules/roi/roi_store.h"
#include "calibration_store.h"
#ifdef CAL_CONTRACT_ACKS
#include "calibration_cmd.h"
#endif

static roi_blob_t configured;
const roi_blob_t *roi_store_current(void) {return &configured;}
bool roi_store_snapshot(roi_blob_t *out) {*out = configured; return true;}
bool roi_uuid_valid(const char *p) {
    if (!p || strlen(p) != 36) return false;
    for (unsigned i=0;i<36;i++) {
        if (i==8||i==13||i==18||i==23) {if(p[i]!='-')return false;}
        else if(!((p[i]>='0'&&p[i]<='9')||(p[i]>='a'&&p[i]<='f')))return false;
    }
    return true;
}
#define MAX_KEYS 50
#define MAX_PAYLOAD 6144
typedef struct {char key[16]; unsigned char buf[MAX_PAYLOAD]; size_t len; bool occupied;} entry_t;
static entry_t entries[MAX_KEYS];
static int fail_selector_commit;
static int corrupt_selector_commit;
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
    if(!data){*len=e->len;return ESP_OK;} /* igual que ESP-IDF: solo longitud */
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
    uint32_t revision = calibration_store_revision();
    assert(revision & 1U);
    assert(calibration_store_commit("invalid", "invalid", "invalid", "invalid") ==
           ESP_ERR_INVALID_STATE); /* reentrant writer is rejected before NVS */
    assert(calibration_store_revision() == revision);
    (void)h;snprintf(pending_selector,sizeof(pending_selector),"%s",key);
    pending_selector_value=v;has_pending_selector=true;return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h){
    (void)h;if(has_pending_selector){
        if(fail_selector_commit){has_pending_selector=false;fail_selector_commit=0;return ESP_FAIL;}
        entry_t *e=entry(pending_selector,true);if(!e)return ESP_ERR_NO_MEM;
        e->buf[0]=corrupt_selector_commit ? 2 : pending_selector_value;
        corrupt_selector_commit=0;e->len=1;has_pending_selector=false;
    }
    return ESP_OK;
}
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key){
    (void)h;entry_t *e=entry(key,false);if(!e)return ESP_ERR_NVS_NOT_FOUND;
    e->occupied=false;return ESP_OK;
}
/* Enumeracion y estadisticas: lo justo para ejercitar el borrado total sin
 * depender de ESP-IDF. Todas las claves del banco de pruebas son de selfiq_cal. */
#define MAX_ITER 4
struct nvs_opaque_iterator_t {int index;};
static struct nvs_opaque_iterator_t iterators[MAX_ITER];
static bool iterator_used[MAX_ITER];
esp_err_t nvs_entry_find(const char *part,const char *ns,nvs_type_t type,
                         nvs_iterator_t *out){
    /* ESP-IDF real rechaza part_name NULL con ESP_ERR_INVALID_ARG. */
    if(!part||!out)return ESP_ERR_INVALID_ARG;
    (void)ns;(void)type;
    *out=NULL;
    for(int slot=0;slot<MAX_ITER;slot++){
        if(iterator_used[slot])continue;
        for(int i=0;i<MAX_KEYS;i++){
            if(!entries[i].occupied)continue;
            iterator_used[slot]=true;iterators[slot].index=i;
            *out=&iterators[slot];return ESP_OK;
        }
        break;
    }
    return ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvs_entry_next(nvs_iterator_t *it){
    if(!it||!*it)return ESP_ERR_INVALID_ARG;
    for(int i=(*it)->index+1;i<MAX_KEYS;i++){
        if(!entries[i].occupied)continue;
        (*it)->index=i;return ESP_OK;
    }
    return ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvs_entry_info(const nvs_iterator_t it,nvs_entry_info_t *info){
    if(!it||!info)return ESP_ERR_INVALID_ARG;
    if(it->index<0||it->index>=MAX_KEYS||!entries[it->index].occupied)
        return ESP_ERR_NVS_NOT_FOUND;
    memset(info,0,sizeof(*info));
    snprintf(info->key,sizeof(info->key),"%s",entries[it->index].key);
    return ESP_OK;
}
void nvs_release_iterator(nvs_iterator_t it){
    if(!it)return;
    for(int slot=0;slot<MAX_ITER;slot++)
        if(&iterators[slot]==it)iterator_used[slot]=false;
}
esp_err_t nvs_get_stats(const char *part,nvs_stats_t *stats){
    (void)part;if(!stats)return ESP_ERR_INVALID_ARG;
    size_t used=0;
    for(int i=0;i<MAX_KEYS;i++)
        if(entries[i].occupied)used+=1+(entries[i].len+31)/32;
    memset(stats,0,sizeof(*stats));
    stats->used_entries=used;stats->total_entries=756;
    stats->free_entries=used<756?756-used:0;
    stats->available_entries=stats->free_entries>126?stats->free_entries-126:0;
    stats->namespace_count=3;
    return ESP_OK;
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
static void report_status(void) {
    char body[768];
    assert(calibration_store_status_body(ROI, body, sizeof(body)) == ESP_OK);
    printf("STATUS_BODY {%s}\n", body);
#ifdef CAL_CONTRACT_ACKS
    cal_cmd_t cmd;
    cal_cmd_status_t status;
    char request[512];
    int n = snprintf(request, sizeof(request),
                     "{\"schema_version\":1,\"type\":\"calibration.status\","
                     "\"id\":\"%s\",\"req_id\":\"%s\",\"calibration_id\":\"%s\","
                     "\"roi_id\":\"%s\",\"calibration_version\":2147483647}",
                     E1, F1, CAL1, ROI);
    assert(n > 0 && (size_t)n < sizeof(request));
    assert(cal_cmd_parse(request, strlen(request), &cmd, &status) == CAL_CMD_OK);
    assert(cmd.calibration_version == 2147483647U);
    char ack[CAL_ACK_JSON_SIZE];
    assert(cal_ack_status(ack, sizeof(ack), &cmd, body));
    printf("ACK_JSON %s\n", ack);
    cmd.action = CAL_ACTION_CANCEL;
    memcpy(cmd.context_hash, context, 65);
    assert(cal_ack_cancel(ack, sizeof(ack), &cmd));
    printf("ACK_JSON %s\n", ack);
#endif
}
int main(void){
    memcpy(configured.rois[0].id,ROI,37);configured.count=1;configured.config_version=7;
    memset(context,'a',64);context[64]=0;
    assert(calibration_store_init()==ESP_OK);
    cal_store_pair_t p;
    assert(calibration_store_get_active(ROI,&p)==ESP_ERR_NVS_NOT_FOUND);
    report_status();
    assert(save(CAL1,E1,CAL_STORE_EMPTY,10,1)==ESP_OK);
    assert(calibration_store_get_active(ROI,&p)==ESP_ERR_NVS_NOT_FOUND);
    assert(calibration_store_get_draft(ROI,&p)==ESP_OK && p.has_empty && !p.has_full);
    report_status(); /* full_crc=null, no CRC inventado */
    assert(save(CAL2,F2,CAL_STORE_FULL,170,2)==ESP_ERR_INVALID_STATE);
    assert(save(CAL2,E2,CAL_STORE_EMPTY,20,1)==ESP_ERR_INVALID_STATE);
    uint32_t revision_before = calibration_store_revision();
    assert(!(revision_before & 1U));
    assert(calibration_store_commit(ROI,CAL1,E1,F1)==ESP_ERR_INVALID_STATE);
    assert(calibration_store_revision() == revision_before + 2U);
    assert(save(CAL1,F1,CAL_STORE_FULL,150,1)==ESP_OK);
    report_status();
    revision_before = calibration_store_revision();
    assert(calibration_store_commit(ROI,CAL1,E1,F1)==ESP_OK);
    assert(calibration_store_revision() == revision_before + 2U);
    assert(calibration_store_init()==ESP_OK); /* reinicio logico */
    assert(calibration_store_get_active(ROI,&p)==ESP_OK && p.calibration_version==1);
    report_status();
    assert(calibration_store_commit(ROI,CAL1,E1,F1)==ESP_OK); /* idempotencia */
    assert(save(CAL1,E1,CAL_STORE_EMPTY,11,1)==ESP_ERR_INVALID_STATE); /* ID reutilizado */
    assert(calibration_store_cancel(ROI,CAL1)==ESP_OK);
    assert(calibration_store_get_active(ROI,&p)==ESP_OK);
    assert(save(CAL2,E2,CAL_STORE_EMPTY,20,2)==ESP_OK);
    report_status();
    assert(save(CAL1,E1,CAL_STORE_EMPTY,10,1)==ESP_ERR_INVALID_STATE);
    assert(save(CAL2,F2,CAL_STORE_FULL,170,2)==ESP_OK);
    fail_selector_commit=1;
    revision_before = calibration_store_revision();
    assert(calibration_store_commit(ROI,CAL2,E2,F2)==ESP_FAIL);
    assert(calibration_store_revision() == revision_before + 2U);
    assert(calibration_store_get_active(ROI,&p)==ESP_OK && p.calibration_version==1);
    assert(calibration_store_commit(ROI,CAL2,E2,F2)==ESP_OK);
    assert(calibration_store_init()==ESP_OK);
    assert(calibration_store_get_active(ROI,&p)==ESP_OK && p.calibration_version==2);
    report_status();
    /* Nueva sesión sin recibir cancel de la anterior: solo empty de versión
     * mayor puede sustituir el borrador; la pareja activa sigue intacta. */
    assert(save(CAL1,E1,CAL_STORE_EMPTY,30,3)==ESP_OK);
    assert(calibration_store_get_draft(ROI,&p)==ESP_OK && !p.has_full);
    assert(calibration_store_get_active(ROI,&p)==ESP_OK && p.calibration_version==2);
    assert(save(CAL2,F2,CAL_STORE_FULL,170,2)==ESP_ERR_INVALID_STATE);
    assert(save(CAL1,F1,CAL_STORE_FULL,180,3)==ESP_OK);
    corrupt_selector_commit=1;
    assert(calibration_store_commit(ROI,CAL1,E1,F1)==ESP_ERR_INVALID_STATE);
    char body[768];
    assert(calibration_store_status_body(ROI,body,sizeof(body))==ESP_ERR_INVALID_CRC);
    assert(body[0]=='\0'); /* NVS corrupta no se presenta como active=null */
    /* Restaurar selector previo para las siguientes comprobaciones. */
    for (int i=0;i<MAX_KEYS;i++)
        if (entries[i].occupied && entries[i].key[0]=='s') entries[i].buf[0]=1;
    char tiny[8];
    assert(calibration_store_status_body(ROI,tiny,sizeof(tiny))==ESP_ERR_INVALID_SIZE);
    assert(tiny[0]=='\0');
    configured.config_version=8;
    assert(calibration_store_get_active(ROI,&p)==ESP_ERR_INVALID_STATE);
    assert(calibration_store_status_body(ROI,body,sizeof(body))==ESP_OK);
    assert(calibration_store_cancel(ROI,CAL1)==ESP_OK); /* limpiar borrador obsoleto */
    /* Referencia local del kernel, una por ROI: formato, version y caducidad. */
    {
        const uint32_t CFG = 8U, CV = 3U;
        uint8_t tiles[CAL_KERNEL_TILE_BYTES * 40U];
        for (size_t i = 0; i < sizeof(tiles); ++i) tiles[i] = (uint8_t)(i * 7U + 1U);
        cal_kernel_ref_t ref;
        memset(&ref, 0, sizeof(ref));
        memcpy(ref.roi_id, ROI, 37);
        ref.config_version = CFG;
        ref.calibration_version = CV;
        ref.rect[0] = 10U; ref.rect[1] = 20U; ref.rect[2] = 160U; ref.rect[3] = 120U;
        ref.tile_count = 32U;
        ref.guard_count = 8U;
        ref.tiles = tiles;
        ref.tiles_bytes = sizeof(tiles);

        cal_kernel_ref_t probe;
        uint8_t readback[CAL_KERNEL_TILE_BYTES * 40U];
#define KERNEL_PROBE()                                                        \
        do {                                                                  \
            memset(&probe, 0, sizeof(probe));                                 \
            memcpy(probe.roi_id, ROI, 37);                                    \
            probe.config_version = CFG;                                       \
            probe.calibration_version = CV;                                   \
        } while (0)

        /* Sin guardar todavia: ausencia declarada, nunca exito vacio. */
        KERNEL_PROBE();
        assert(calibration_store_get_kernel_ref(&probe) == ESP_ERR_NVS_NOT_FOUND);

        /* Argumentos incoherentes y configuracion no aplicada. */
        ref.tiles_bytes = 10U;
        assert(calibration_store_save_kernel_ref(&ref) == ESP_ERR_INVALID_ARG);
        ref.tiles_bytes = sizeof(tiles);
        ref.tile_count = 0U;
        assert(calibration_store_save_kernel_ref(&ref) == ESP_ERR_INVALID_ARG);
        ref.tile_count = 32U;
        ref.full_pixels = 160U * 120U + 1U;
        assert(calibration_store_save_kernel_ref(&ref) == ESP_ERR_INVALID_ARG);
        ref.full_pixels = 0U;
        ref.config_version = 7U; /* la configuracion aplicada es la 8 */
        assert(calibration_store_save_kernel_ref(&ref) == ESP_ERR_INVALID_STATE);
        ref.config_version = CFG;

        /* Guardado: primero la referencia de vacio, sin lleno aun. */
        assert(calibration_store_save_kernel_ref(&ref) == ESP_OK);

        /* Lectura de metadatos sin pedir tiles. */
        KERNEL_PROBE();
        assert(calibration_store_get_kernel_ref(&probe) == ESP_OK);
        assert(probe.tiles == NULL && probe.tiles_bytes == sizeof(tiles));
        assert(probe.tile_count == 32U && probe.guard_count == 8U);
        assert(probe.rect[0] == 10U && probe.rect[3] == 120U);
        assert(probe.full_pixels == 0U);
        assert(!strcmp(probe.roi_id, ROI));

        /* Destino pequeno no se acepta a medias. */
        probe.tiles = readback;
        probe.tiles_capacity = 8U;
        assert(calibration_store_get_kernel_ref(&probe) == ESP_ERR_INVALID_SIZE);
        probe.tiles_capacity = sizeof(readback);
        assert(calibration_store_get_kernel_ref(&probe) == ESP_OK);
        assert(!memcmp(readback, tiles, sizeof(tiles)));

        /* Recalibrar invalida sin borrar: la version manda. */
        probe.calibration_version = CV + 1U;
        assert(calibration_store_get_kernel_ref(&probe) == ESP_ERR_INVALID_STATE);
        probe.calibration_version = CV;
        assert(calibration_store_get_kernel_ref(&probe) == ESP_OK);

        /* Lleno tardio: lectura-modificacion-escritura sobre el mismo blob. */
        assert(calibration_store_set_kernel_full_pixels(ROI, CFG, CV, 0U) == ESP_ERR_INVALID_ARG);
        assert(calibration_store_set_kernel_full_pixels(ROI, CFG, CV + 1U, 9000U) == ESP_ERR_INVALID_STATE);
        assert(calibration_store_set_kernel_full_pixels(ROI, CFG, CV, 160U * 120U + 1U) == ESP_ERR_INVALID_ARG);
        assert(calibration_store_set_kernel_full_pixels(ROI, CFG, CV, 9000U) == ESP_OK);
        KERNEL_PROBE();
        assert(calibration_store_get_kernel_ref(&probe) == ESP_OK);
        assert(probe.full_pixels == 9000U);
        assert(probe.tiles_bytes == sizeof(tiles)); /* los tiles sobreviven */

        /* Reescribir el mismo ROI no crea claves nuevas ni pierde datos. */
        unsigned kernel_keys = 0;
        for (int i = 0; i < MAX_KEYS; ++i)
            if (entries[i].occupied && entries[i].key[0] == 'k') ++kernel_keys;
        assert(kernel_keys == 1U);
        assert(calibration_store_save_kernel_ref(&ref) == ESP_OK);
        kernel_keys = 0;
        for (int i = 0; i < MAX_KEYS; ++i)
            if (entries[i].occupied && entries[i].key[0] == 'k') ++kernel_keys;
        assert(kernel_keys == 1U);

        /* Un byte alterado invalida el blob; se reescribe sin intervencion. */
        for (int i = 0; i < MAX_KEYS; ++i)
            if (entries[i].occupied && entries[i].key[0] == 'k')
                entries[i].buf[100] ^= 0xFFU;
        KERNEL_PROBE();
        assert(calibration_store_get_kernel_ref(&probe) == ESP_ERR_INVALID_CRC);
        assert(calibration_store_save_kernel_ref(&ref) == ESP_OK);
        KERNEL_PROBE();
        assert(calibration_store_get_kernel_ref(&probe) == ESP_OK);

        /* Borrado independiente de la pareja, e idempotente. */
        assert(calibration_store_drop_kernel_ref(ROI) == ESP_OK);
        assert(calibration_store_drop_kernel_ref(ROI) == ESP_OK);
        KERNEL_PROBE();
        assert(calibration_store_get_kernel_ref(&probe) == ESP_ERR_NVS_NOT_FOUND);
        bool pair_alive = false;
        for (int i = 0; i < MAX_KEYS; ++i)
            if (entries[i].occupied && (entries[i].key[0] == 'a' || entries[i].key[0] == 'b'))
                pair_alive = true;
        assert(pair_alive); /* borrar el kernel no toca la pareja de calibracion */
        /* Dejar una referencia viva para comprobar el borrado total. */
        assert(calibration_store_save_kernel_ref(&ref) == ESP_OK);
#undef KERNEL_PROBE
    }
    /* Mantenimiento: borrado total de selfiq_cal. */
    unsigned keys_before = 0;
    for (int i=0;i<MAX_KEYS;i++) if (entries[i].occupied) keys_before++;
    assert(keys_before > 0);
    cal_store_erase_report_t report;
    assert(calibration_store_erase_all(&report) == ESP_OK);
    assert(report.keys_removed == keys_before);
    assert(report.verified_empty);
    assert(report.available_entries_after >= report.available_entries_before);
    for (int i=0;i<MAX_KEYS;i++) assert(!entries[i].occupied);
    /* Repetir no es error: no quedan claves y se informa el estado real. */
    assert(calibration_store_erase_all(&report) == ESP_OK);
    assert(report.keys_removed == 0 && report.verified_empty);
    /* Sin pareja: "sin datos", nunca corrupcion ni porcentajes inventados. */
    assert(calibration_store_get_active(ROI,&p) == ESP_ERR_NVS_NOT_FOUND);
    assert(calibration_store_get_draft(ROI,&p) == ESP_ERR_NVS_NOT_FOUND);
    assert(calibration_store_status_body(ROI,body,sizeof(body)) == ESP_OK);
    assert(!strcmp(body,"\"active\":null,\"draft\":null"));
    /* La geometria aplicada (otro namespace) sigue intacta. */
    assert(roi_store_snapshot(&configured) && configured.count == 1);
    puts("PASS: borrado total de calibracion, idempotencia y ROI conservada");
    puts("PASS: borrador, pareja, reinicio logico, replay, cancel, fallo selector, version ROI");
    return 0;
}
