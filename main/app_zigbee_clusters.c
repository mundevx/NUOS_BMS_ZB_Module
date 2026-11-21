#include "app_zigbee_clusters.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs.h"

#if defined(__has_include)
# if __has_include("aps/esp_zigbee_aps.h")
#  include "aps/esp_zigbee_aps.h"
# elif __has_include("esp_zigbee_aps.h")
#  include "esp_zigbee_aps.h"
# endif
#endif
#ifndef ESP_ZB_ZCL_CMD_REPORT_ATTRIB
#define ESP_ZB_ZCL_CMD_REPORT_ATTRIB 0x0A
#endif
#ifndef NUOS_APSDE_REQUEST
#define NUOS_APSDE_REQUEST esp_zb_aps_data_request
#endif

static const char *TAG="ZB_CLUSTER";
#define CFG_NS "nuos_cfg"
#define CFG_META "meta"
#define CFG_MAGIC 0x4E5A4346u
#define CFG_VERSION 3u
#define CFG_REBUILD_KEY "rebuild"

typedef struct { uint8_t id; uint8_t type; uint8_t value[16]; bool configured; bool changed; } dyn_attr_t;
typedef struct { bool valid; uint8_t device_type; uint16_t cluster_id; uint16_t count; dyn_attr_t *attrs; } dyn_ep_t;
static dyn_ep_t g_active[NUOS_TOTAL_ENDPOINTS];
static dyn_ep_t g_pending[NUOS_TOTAL_ENDPOINTS];
static bool g_pending_seen[NUOS_TOTAL_ENDPOINTS];
static uint8_t g_zcl_seq;
static volatile bool g_zigbee_network_ready;

typedef struct __attribute__((packed)) { uint32_t magic; uint16_t version; uint16_t count; uint8_t device_type; uint8_t reserved; uint16_t cluster_id; } cfg_ep_hdr_t;
typedef struct __attribute__((packed)) { uint32_t magic; uint16_t version; uint8_t endpoint_count; uint8_t reserved; } cfg_meta_t;

static uint8_t zcl_type(uint8_t t){switch(t){case NUOS_TYPE_BOOL:return 0x10;case NUOS_TYPE_UINT16:return ESP_ZB_ZCL_ATTR_TYPE_U16;case NUOS_TYPE_INT16:return 0x29;case NUOS_TYPE_UINT32:return ESP_ZB_ZCL_ATTR_TYPE_U32;case NUOS_TYPE_INT32:return 0x2B;case NUOS_TYPE_FLOAT32:return 0x2A;case NUOS_TYPE_UINT64:return 0x27;case NUOS_TYPE_INT64:return 0x2F;case NUOS_TYPE_STRING:return 0x42;case NUOS_TYPE_UINT8:return 0x20;case NUOS_TYPE_INT8:return 0x28;case NUOS_TYPE_UINT24:return ESP_ZB_ZCL_ATTR_TYPE_U32;case NUOS_TYPE_UINT48:return 0x27;default:return ESP_ZB_ZCL_ATTR_TYPE_U32;}}
static uint8_t type_width(uint8_t t){switch(t){case NUOS_TYPE_BOOL:case NUOS_TYPE_UINT8:case NUOS_TYPE_INT8:return 1;case NUOS_TYPE_UINT16:case NUOS_TYPE_INT16:return 2;case NUOS_TYPE_UINT32:case NUOS_TYPE_INT32:case NUOS_TYPE_FLOAT32:return 4;case NUOS_TYPE_UINT64:case NUOS_TYPE_INT64:return 8;case NUOS_TYPE_UINT24:return 3;case NUOS_TYPE_UINT48:return 6;default:return 4;}}
static uint8_t report_width(uint8_t t){return t==NUOS_TYPE_UINT24?4:t==NUOS_TYPE_UINT48?8:type_width(t);}
static uint8_t zcl_decl_type(uint8_t t){return zcl_type(t);}
static void free_ep(dyn_ep_t *e){if(!e)return;free(e->attrs);memset(e,0,sizeof(*e));}
static void free_table(dyn_ep_t *t){for(uint8_t i=0;i<NUOS_TOTAL_ENDPOINTS;i++)free_ep(&t[i]);}
static dyn_attr_t *find_attr(dyn_ep_t *e,uint8_t id){if(!e||!e->attrs)return NULL;for(uint16_t i=0;i<e->count;i++)if(e->attrs[i].id==id)return &e->attrs[i];return NULL;}
static bool valid_type(uint8_t t){return t<=NUOS_TYPE_UINT48;}

static esp_zb_attribute_list_t *create_basic_cluster(void){esp_zb_basic_cluster_cfg_t cfg={.power_source=ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,.zcl_version=ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE};esp_zb_attribute_list_t*b=esp_zb_basic_cluster_create(&cfg);if(!b)return NULL;static char model[]={18,'N','U','O','S','-','B','M','S','-','H','2','-','R','O','U','T','E','R'};static char man[]={4,'N','U','O','S'};esp_zb_basic_cluster_add_attr(b,ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,model);esp_zb_basic_cluster_add_attr(b,ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,man);return b;}

static esp_zb_attribute_list_t *create_custom_cluster(uint8_t ep){dyn_ep_t*e=&g_active[ep-1];if(!e->valid||!e->cluster_id||!e->count)return NULL;esp_zb_attribute_list_t*cl=esp_zb_zcl_attr_list_create(e->cluster_id);if(!cl)return NULL;for(uint16_t i=0;i<e->count;i++){dyn_attr_t*a=&e->attrs[i];esp_err_t r=esp_zb_custom_cluster_add_custom_attr(cl,a->id,zcl_decl_type(a->type),ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE|ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,a->value);if(r!=ESP_OK){ESP_LOGE(TAG,"EP%u attr 0x%02X create failed 0x%x",ep,a->id,(unsigned)r);return NULL;}}return cl;}

static esp_zb_cluster_list_t *create_minimal_commissioning_cluster_list(void)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    if (!cl) return NULL;

    esp_zb_attribute_list_t *b = create_basic_cluster();
    if (!b) return NULL;
    esp_zb_cluster_list_add_basic_cluster(cl, b, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t icfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
    };
    esp_zb_attribute_list_t *id = esp_zb_identify_cluster_create(&icfg);
    if (!id) return NULL;
    esp_zb_cluster_list_add_identify_cluster(cl, id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    return cl;
}

esp_zb_ep_list_t *nuos_init_zb_clusters(void){
    esp_zb_ep_list_t *list = esp_zb_ep_list_create();
    if (!list) return NULL;

    uint8_t created = 0;
    bool have_configured_endpoint = false;
    for (uint8_t ep = 1; ep <= NUOS_TOTAL_ENDPOINTS; ep++) {
        dyn_ep_t *e = &g_active[ep - 1];
        if (!e->valid || !e->cluster_id || !e->count) continue;
        have_configured_endpoint = true;

        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
        if (!cl) return NULL;
        if (ep == 1) {
            esp_zb_attribute_list_t *b = create_basic_cluster();
            if (!b) return NULL;
            esp_zb_cluster_list_add_basic_cluster(cl, b, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        }
        esp_zb_identify_cluster_cfg_t icfg = {
            .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
        };
        esp_zb_attribute_list_t *id = esp_zb_identify_cluster_create(&icfg);
        if (!id) return NULL;
        esp_zb_cluster_list_add_identify_cluster(cl, id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_attribute_list_t *custom = create_custom_cluster(ep);
        if (!custom) return NULL;
        esp_zb_cluster_list_add_custom_cluster(cl, custom, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_endpoint_config_t ec = {
            .endpoint = ep,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
            .app_device_version = 0
        };
        if (esp_zb_ep_list_add_ep(list, cl, ec) != ESP_OK) return NULL;
        created++;
    }

    if (!have_configured_endpoint) {
        /* First boot / empty NVS compatibility path.
         * Register only Basic + Identify so the router can join without
         * inventing a 16-attribute application cluster. C6 configuration
         * arrives over UART and a controlled restart then creates the exact
         * configured endpoint/attribute map. */
        esp_zb_cluster_list_t *cl = create_minimal_commissioning_cluster_list();
        if (!cl) return NULL;
        esp_zb_endpoint_config_t ec = {
            .endpoint = 1,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
            .app_device_version = 0
        };
        if (esp_zb_ep_list_add_ep(list, cl, ec) != ESP_OK) return NULL;
        ESP_LOGW(TAG, "No persisted C6 configuration; registering minimal commissioning EP1 (no dummy attributes)");
        created = 1;
    }

    ESP_LOGI(TAG, "Zigbee endpoints created=%u (configured map when available)", created);
    return list;
}

static esp_err_t send_report_chunk_locked(uint8_t ep,
                                           dyn_ep_t *e,
                                           uint8_t *asdu,
                                           uint16_t *pos,
                                           uint8_t first_attr,
                                           uint8_t last_attr,
                                           uint8_t report_no)
{
    if (!e || !asdu || !pos || *pos <= 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t count = 0;
    for (uint8_t i = first_attr; i <= last_attr; i++) {
        if (e->attrs[i].changed) count++;
    }
    if (!count) return ESP_OK;

    esp_zb_apsde_data_req_t req = {
        .dst_addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .dst_addr.addr_short = NUOS_REPORT_DST_SHORT,
        .dst_endpoint = NUOS_REPORT_DST_EP,
        .src_endpoint = ep,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = e->cluster_id,
        .asdu_length = *pos,
        .asdu = asdu,
        /* The report is already packed to a conservative size.  Keep
         * fragmentation permitted as an additional safety net for stacks
         * whose security/header overhead leaves less room in one NWK frame. */
        .tx_options = ESP_ZB_APSDE_TX_OPT_ACK_TX | ESP_ZB_APSDE_TX_OPT_FRAG_PERMITTED,
        .use_alias = false,
        .radius = 0
    };

    ESP_LOGI(TAG,
             "REPORT TX EP%u report=%u attrs=%u slots=%u..%u ASDU=%u cluster=0x%04X",
             ep, report_no, count, first_attr, last_attr,
             (unsigned)*pos, e->cluster_id);

    esp_err_t r = NUOS_APSDE_REQUEST(&req);
    if (r != ESP_OK) {
        ESP_LOGE(TAG,
                 "REPORT TX FAILED EP%u report=%u result=0x%x (%s); changed flags retained",
                 ep, report_no, (unsigned)r, esp_err_to_name(r));
        return r;
    }

    /* Clear only attributes that were actually included in this successful
     * APSDE request. Attributes belonging to later reports remain pending. */
    for (uint8_t i = first_attr; i <= last_attr; i++) {
        if (e->attrs[i].changed) e->attrs[i].changed = false;
    }

    return ESP_OK;
}

static esp_err_t send_report_locked(uint8_t ep)
{
    dyn_ep_t *e = &g_active[ep - 1];
    if (!e->valid || !e->cluster_id || !e->count) return ESP_ERR_NOT_FOUND;
    if (!g_zigbee_network_ready) return ESP_ERR_INVALID_STATE;

    /*
     * A ZCL Report Attributes record is:
     *     Attribute ID  : 2 bytes
     *     Data type     : 1 byte
     *     Value         : 1..8 bytes for the supported numeric types
     *
     * Keep each individual APSDE ASDU <= 72 bytes for broad coordinator
     * compatibility. 52/64 attributes are therefore sent as several ZCL
     * Report Attributes messages. APS fragmentation is also permitted as a
     * safety net by send_report_chunk_locked().
     */
    uint8_t asdu[NUOS_MAX_ASDU_LEN];
    uint16_t pos = 3;
    uint8_t report_no = 0;
    uint8_t chunk_first = 0;
    uint8_t chunk_last = 0;
    bool chunk_has_attr = false;
    uint16_t total_changed = 0;
    uint16_t total_sent = 0;
    esp_err_t final = ESP_OK;

    memset(asdu, 0, sizeof(asdu));

    for (uint16_t i = 0; i < e->count; i++) {
        if (e->attrs[i].changed) total_changed++;
    }

    if (!total_changed) {
        ESP_LOGI(TAG, "REPORT END EP%u: no changed attributes", ep);
        return ESP_OK;
    }

    ESP_LOGI(TAG,
             "REPORT START EP%u cluster=0x%04X configured=%u changed=%u max_asdu=%u",
             ep, e->cluster_id, e->count, total_changed,
             (unsigned)sizeof(asdu));

    asdu[0] = 0x18; /* ZCL frame control: global command, server-to-client. */
    asdu[2] = ESP_ZB_ZCL_CMD_REPORT_ATTRIB;

    for (uint16_t i = 0; i < e->count; i++) {
        dyn_attr_t *a = &e->attrs[i];
        if (!a->changed) continue;

        uint8_t w = report_width(a->type);
        uint16_t rec_len = (uint16_t)(3u + w);

        if (rec_len > (sizeof(asdu) - 3u)) {
            ESP_LOGE(TAG,
                     "REPORT FAIL EP%u attr=0x%02X record=%u exceeds ASDU capacity=%u",
                     ep, a->id, (unsigned)rec_len,
                     (unsigned)(sizeof(asdu) - 3u));
            return ESP_ERR_INVALID_SIZE;
        }



        /* Start a new report when the next complete attribute would not fit. */
        if (chunk_has_attr && ((uint16_t)pos + rec_len > sizeof(asdu))) {
            asdu[1] = g_zcl_seq++;
            esp_err_t r = send_report_chunk_locked(ep, e, asdu, &pos,
                                                   chunk_first, chunk_last,
                                                   report_no);
            if (r != ESP_OK) final = r;
            else total_sent += 1;
            report_no++;

            if (r != ESP_OK) {
                /* Do not continue after a failed chunk: its changed flags are
                 * retained so the next poll can retry without data loss. */
                return r;
            }

            pos = 3;
            chunk_has_attr = false;
            memset(&asdu[3], 0, sizeof(asdu) - 3);
        }

        if (!chunk_has_attr) chunk_first = (uint8_t)i;
        chunk_last = (uint8_t)i;

        asdu[pos++] = a->id;
        asdu[pos++] = 0x00;
        asdu[pos++] = zcl_type(a->type);

        /* dyn_attr_t::value is kept in native little-endian form. UINT24 and
         * UINT48 are widened to UINT32/UINT64 in that same representation. */
        memcpy(&asdu[pos], a->value, w);
        pos += w;
        chunk_has_attr = true;
    }

    if (chunk_has_attr) {
        asdu[1] = g_zcl_seq++;
        esp_err_t r = send_report_chunk_locked(ep, e, asdu, &pos,
                                               chunk_first, chunk_last,
                                               report_no);
        if (r != ESP_OK) final = r;
        else total_sent += 1;
        report_no++;
    }

    ESP_LOGI(TAG,
             "REPORT END EP%u changed=%u reports=%u successful=%u result=0x%x (%s)",
             ep, total_changed, report_no, total_sent,
             (unsigned)final, esp_err_to_name(final));

    
        ESP_LOG_BUFFER_HEX_LEVEL(
            TAG,
            &asdu[0],
            pos,
            ESP_LOG_INFO
        );         
    return final;
}

static esp_err_t apply_core(uint8_t ep, const uint8_t *p, uint16_t len)
{
    if (ep < 1 || ep > NUOS_TOTAL_ENDPOINTS || !p || len < 3) {
        return ESP_ERR_INVALID_ARG;
    }

    dyn_ep_t *e = &g_active[ep - 1];
    if (!e->valid || !e->cluster_id || !e->count) {
        ESP_LOGW(TAG, "EP_DATA EP%u rejected: endpoint is not configured", ep);
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_zigbee_network_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    /* C6 payload format: [version=1][attribute_count_le16][records...] */
    if (p[0] != 1u) {
        ESP_LOGW(TAG, "EP_DATA EP%u rejected: unsupported payload version=%u", ep, p[0]);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t count = nuos_link_get_u16(&p[1]);
    uint16_t pos = 3;
    bool any_new_change = false;

    ESP_LOGI(TAG,
             "EP_DATA APPLY START EP%u len=%u attr_count=%u cluster=0x%04X configured=%u",
             ep, len, count, e->cluster_id, e->count);

    for (uint16_t n = 0; n < count; n++) {
        if ((uint32_t)pos + 3u > len) {
            ESP_LOGW(TAG, "EP_DATA EP%u truncated record header at index=%u", ep, n);
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t id = p[pos++];
        uint8_t dt = p[pos++];
        uint8_t vl = p[pos++];

        if ((uint32_t)pos + vl > len || vl > 16u) {
            ESP_LOGW(TAG,
                     "EP_DATA EP%u invalid value length attr=0x%02X len=%u remaining=%u",
                     ep, id, vl, (unsigned)(len - pos));
            return ESP_ERR_INVALID_SIZE;
        }

        dyn_attr_t *a = find_attr(e, id);
        if (!a || !a->configured) {
            ESP_LOGW(TAG,
                     "EP_DATA EP%u attr 0x%02X is not in the active C6 configuration",
                     ep, id);
            return ESP_ERR_NOT_FOUND;
        }

        if (!valid_type(dt)) {
            ESP_LOGW(TAG, "EP_DATA EP%u attr 0x%02X invalid type=%u", ep, id, dt);
            return ESP_ERR_INVALID_ARG;
        }

        /* The ZCL attribute declaration is fixed when the endpoint is built.
         * Reject a runtime type change instead of silently changing only the
         * RAM-side type and leaving the registered ZCL type inconsistent. */
        if (a->type != dt) {
            ESP_LOGE(TAG,
                     "EP_DATA EP%u attr=0x%02X type mismatch: configured=%u received=%u",
                     ep, id, a->type, dt);
            return ESP_ERR_INVALID_ARG;
        }

        uint8_t expected_width = type_width(dt);
        if (dt != NUOS_TYPE_STRING && vl != expected_width) {
            ESP_LOGW(TAG,
                     "EP_DATA EP%u attr=0x%02X invalid type length: type=%u len=%u expected=%u",
                     ep, id, dt, vl, expected_width);
            return ESP_ERR_INVALID_SIZE;
        }

        ESP_LOGI(TAG,
                 "EP_DATA VALUE EP%u ATTR=0x%02X DT=0x%02X LEN=%u "
                 "RX=%02X %02X %02X %02X "
                 "OLD=%02X %02X %02X %02X",
                 ep, id, dt, vl,
                 p[pos],
                 vl > 1 ? p[pos + 1] : 0,
                 vl > 2 ? p[pos + 2] : 0,
                 vl > 3 ? p[pos + 3] : 0,
                 a->value[0], a->value[1], a->value[2], a->value[3]);

        /* Construct the normalized replacement separately. Do NOT clear
         * a->value here: the old bytes are required for change detection. */
        uint8_t new_value[16] = {0};
        switch (dt) {
        case NUOS_TYPE_UINT16:
        case NUOS_TYPE_INT16:
        case NUOS_TYPE_UINT32:
        case NUOS_TYPE_INT32:
        case NUOS_TYPE_FLOAT32:
        case NUOS_TYPE_UINT64:
        case NUOS_TYPE_INT64:
            for (uint8_t i = 0; i < vl; i++) {
                new_value[i] = p[pos + vl - 1u - i];
            }
            break;

        case NUOS_TYPE_UINT24:
            /* Link value is 3-byte big-endian; ZCL declaration is UINT32. */
            new_value[0] = p[pos + 2];
            new_value[1] = p[pos + 1];
            new_value[2] = p[pos];
            new_value[3] = 0;
            break;

        case NUOS_TYPE_UINT48:
            /* Link value is 6-byte big-endian; ZCL declaration is UINT64. */
            for (uint8_t i = 0; i < 6; i++) {
                new_value[i] = p[pos + 5u - i];
            }
            new_value[6] = 0;
            new_value[7] = 0;
            break;

        default:
            /* BOOL, UINT8, INT8 and STRING are byte-oriented in this ABI. */
            memcpy(new_value, &p[pos], vl);
            break;
        }

        /* Compare before storage is changed. This catches nonzero -> zero. */
        bool changed = memcmp(a->value, new_value, sizeof(new_value)) != 0;
        uint8_t old_value[sizeof(a->value)];
        memcpy(old_value, a->value, sizeof(old_value));
        memcpy(a->value, new_value, sizeof(a->value));

        uint64_t zcl_value = 0;
        uint8_t zcl_width = report_width(dt);
        if (zcl_width > sizeof(zcl_value)) {
            memcpy(a->value, old_value, sizeof(a->value));
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(&zcl_value, new_value, zcl_width);

        esp_zb_zcl_status_t st = esp_zb_zcl_set_attribute_val(
            ep,
            e->cluster_id,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            a->id,
            &zcl_value,
            false);

        ESP_LOGI(TAG,
                 "ATTR STORAGE EP%u ID=0x%02X DATA=%02X %02X %02X %02X",
                 ep, a->id,
                 a->value[0], a->value[1], a->value[2], a->value[3]);
        ESP_LOGI(TAG,
                 "ZCL SET EP%u cluster=0x%04X attr=0x%02X status=0x%02X",
                 ep, e->cluster_id, a->id, st);

        if (st != ESP_ZB_ZCL_STATUS_SUCCESS) {
            /* A failed ZCL setter must not be ACKed as successful to C6.
             * Restore storage so C6's uncommitted value will be retried. */
            memcpy(a->value, old_value, sizeof(a->value));
            ESP_LOGE(TAG,
                     "ZCL SET failed EP%u attr=0x%02X status=0x%02X; old value restored",
                     ep, a->id, st);
            return ESP_FAIL;
        }

        if (changed) {
            a->changed = true;
            any_new_change = true;
        }

        pos += vl;
    }

    if (pos != len) {
        ESP_LOGW(TAG,
                 "EP_DATA EP%u trailing payload bytes: parsed=%u total=%u",
                 ep, pos, len);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!any_new_change) {
        ESP_LOGI(TAG,
                 "EP_DATA EP%u: no new value changes; checking pending report retries",
                 ep);
    }

    /* This sends only attributes with changed=true and retries flags retained
     * after a previous APSDE request failure. It does not force all attributes. */
    return send_report_locked(ep);
}

esp_err_t nuos_apply_endpoint_payload(uint8_t ep,const uint8_t*p,uint16_t len){if(!esp_zb_lock_acquire(portMAX_DELAY))return ESP_ERR_TIMEOUT;esp_err_t r=apply_core(ep,p,len);esp_zb_lock_release();return r;}
esp_err_t nuos_apply_endpoint_payload_from_zigbee_task(uint8_t ep,const uint8_t*p,uint16_t len){return apply_core(ep,p,len);}
esp_err_t nuos_send_bulk_report(uint8_t ep){if(ep<1||ep>NUOS_TOTAL_ENDPOINTS)return ESP_ERR_INVALID_ARG;if(!esp_zb_lock_acquire(portMAX_DELAY))return ESP_ERR_TIMEOUT;esp_err_t r=send_report_locked(ep);esp_zb_lock_release();return r;}
esp_err_t nuos_send_raw_asdu(uint8_t src,uint16_t cluster,uint16_t dst,uint8_t dst_ep,const uint8_t*asdu,uint8_t len){if(!asdu||!len||src<1||src>NUOS_TOTAL_ENDPOINTS||len>NUOS_MAX_ASDU_LEN)return ESP_ERR_INVALID_ARG;if(!esp_zb_lock_acquire(portMAX_DELAY))return ESP_ERR_TIMEOUT;static uint8_t buf[NUOS_MAX_ASDU_LEN];memcpy(buf,asdu,len);esp_zb_apsde_data_req_t req={.dst_addr_mode=ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,.dst_addr.addr_short=dst,.dst_endpoint=dst_ep,.src_endpoint=src,.profile_id=ESP_ZB_AF_HA_PROFILE_ID,.cluster_id=cluster,.asdu_length=len,.asdu=buf,.tx_options=ESP_ZB_APSDE_TX_OPT_ACK_TX,.use_alias=false,.radius=0};esp_err_t r=NUOS_APSDE_REQUEST(&req);esp_zb_lock_release();return r;}
esp_err_t send_report(uint8_t ep,uint16_t cluster,uint16_t attr){(void)cluster;(void)attr;return nuos_send_bulk_report(ep);}

void nuos_config_begin(void){free_table(g_pending);memset(g_pending_seen,0,sizeof(g_pending_seen));ESP_LOGI(TAG,"ZB_CONFIG: begin - cleared pending dynamic configuration");}
esp_err_t nuos_configure_endpoint_chunk(uint8_t ep,
                                        uint8_t device_type,
                                        uint16_t cluster,
                                        uint16_t start,
                                        const uint8_t *pairs,
                                        uint16_t count)
{
    if (ep < 1 || ep > NUOS_TOTAL_ENDPOINTS || !pairs || !cluster || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* start + count is evaluated as 32-bit so there is no unsigned-16-bit
     * wraparound and no impossible comparison against a uint16_t value. */
    uint32_t needed = (uint32_t)start + (uint32_t)count;
    if (needed > NUOS_LINK_MAX_ATTRIBUTES) {
        ESP_LOGW(TAG, "ZB_CONFIG EP%u exceeds protocol attribute ceiling: start=%u count=%u", ep, start, count);
        return ESP_ERR_INVALID_SIZE;
    }

    dyn_ep_t *e = &g_pending[ep - 1];

    if (!e->valid) {
        e->valid = true;
        e->device_type = device_type;
        e->cluster_id = cluster;
        e->count = 0;
        e->attrs = NULL;
    } else if (e->device_type != device_type || e->cluster_id != cluster) {
        return ESP_ERR_INVALID_ARG;
    }

    if (needed > e->count) {
        dyn_attr_t *n = realloc(e->attrs,
                                (size_t)needed * sizeof(dyn_attr_t));
        if (!n) {
            return ESP_ERR_NO_MEM;
        }

        memset(n + e->count,
               0,
               (size_t)(needed - e->count) * sizeof(dyn_attr_t));
        e->attrs = n;
        e->count = (uint16_t)needed;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint8_t id = pairs[i * 2];
        uint8_t dt = pairs[i * 2 + 1];

        if (!valid_type(dt)) {
            return ESP_ERR_INVALID_ARG;
        }

        /* Attribute ID 0 is valid. Do not use zero-initialized entries as
         * evidence that an attribute is already configured. Only compare
         * against slots that were actually received from C6. */
        for (uint16_t j = 0; j < e->count; j++) {
            if (j != (uint16_t)(start + i) &&
                e->attrs[j].configured && e->attrs[j].id == id) {
                ESP_LOGW(TAG,
                         "ZB_CONFIG EP%u duplicate attr id=0x%02X at index=%u (existing=%u)",
                         ep, id, (unsigned)(start + i), (unsigned)j);
                return ESP_ERR_INVALID_SIZE;
            }
        }

        e->attrs[start + i].id = id;
        e->attrs[start + i].type = dt;
        e->attrs[start + i].configured = true;
        e->attrs[start + i].changed = false;
    }

    g_pending_seen[ep - 1] = true;

    ESP_LOGI(TAG,
             "ZB_CONFIG EP%u chunk start=%u count=%u total=%u cluster=0x%04X",
             ep, start, count, e->count, cluster);

    return ESP_OK;
}

esp_err_t nuos_config_validate_pending(uint8_t expected)
{
    uint8_t seen = 0;

    for (uint8_t ep = 0; ep < NUOS_TOTAL_ENDPOINTS; ep++) {
        if (!g_pending_seen[ep]) {
            continue;
        }

        seen++;
        dyn_ep_t *e = &g_pending[ep];
        if (e->count > NUOS_LINK_MAX_ATTRIBUTES) return ESP_ERR_INVALID_SIZE;
        if (!e->valid || !e->cluster_id || !e->count || !e->attrs) {
            ESP_LOGW(TAG, "ZB_CONFIG validation EP%u invalid/empty", ep + 1);
            return ESP_ERR_INVALID_STATE;
        }

        for (uint16_t i = 0; i < e->count; i++) {
            if (!e->attrs[i].configured) {
                ESP_LOGW(TAG,
                         "ZB_CONFIG validation EP%u missing attribute index=%u",
                         ep + 1, i);
                return ESP_ERR_INVALID_STATE;
            }

            for (uint16_t j = i + 1; j < e->count; j++) {
                if (e->attrs[j].configured &&
                    e->attrs[i].id == e->attrs[j].id) {
                    return ESP_ERR_INVALID_SIZE;
                }
            }
        }
    }

    if (seen != expected) {
        ESP_LOGW(TAG,
                 "ZB_CONFIG validation endpoint count mismatch seen=%u expected=%u",
                 seen, expected);
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t save_one(nvs_handle_t h,uint8_t ep,const dyn_ep_t*e){char key[8];snprintf(key,sizeof(key),"ep%u",ep);if(!e->valid||!e->count){esp_err_t er=nvs_erase_key(h,key);return er==ESP_ERR_NVS_NOT_FOUND?ESP_OK:er;}size_t sz=sizeof(cfg_ep_hdr_t)+(size_t)e->count*2;uint8_t*b=malloc(sz);if(!b)return ESP_ERR_NO_MEM;cfg_ep_hdr_t hdr={CFG_MAGIC,CFG_VERSION,e->count,e->device_type,0,e->cluster_id};memcpy(b,&hdr,sizeof(hdr));for(uint16_t i=0;i<e->count;i++){b[sizeof(hdr)+i*2]=e->attrs[i].id;b[sizeof(hdr)+i*2+1]=e->attrs[i].type;}esp_err_t r=nvs_set_blob(h,key,b,sz);free(b);return r;}

static bool ep_config_matches(const dyn_ep_t *a,const dyn_ep_t *b)
{
    if (!a || !b) return false;
    if (a->valid != b->valid) return false;
    if (!a->valid) return true;
    if (a->device_type != b->device_type ||
        a->cluster_id != b->cluster_id ||
        a->count != b->count) return false;
    if (a->count == 0) return true;
    for (uint16_t i = 0; i < a->count; i++) {
        if (!a->attrs || !b->attrs ||
            a->attrs[i].id != b->attrs[i].id ||
            a->attrs[i].type != b->attrs[i].type ||
            !a->attrs[i].configured || !b->attrs[i].configured) {
            return false;
        }
    }
    return true;
}

esp_err_t nuos_config_save_persisted(uint8_t endpoint_count)
{
    if (endpoint_count > NUOS_TOTAL_ENDPOINTS) return ESP_ERR_INVALID_ARG;

    esp_err_t v = nuos_config_validate_pending(endpoint_count);
    if (v != ESP_OK) return v;

    nvs_handle_t h;
    esp_err_t e = nvs_open(CFG_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;

    for (uint8_t ep = 1; ep <= NUOS_TOTAL_ENDPOINTS; ep++) {
        e = save_one(h, ep, &g_pending[ep - 1]);
        if (e != ESP_OK) break;
    }

    cfg_meta_t m = {CFG_MAGIC, CFG_VERSION, endpoint_count, 0};
    if (e == ESP_OK) e = nvs_set_blob(h, CFG_META, &m, sizeof(m));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);

    if (e != ESP_OK) return e;

    free_table(g_active);
    for (uint8_t ep = 0; ep < NUOS_TOTAL_ENDPOINTS; ep++) {
        if (!g_pending[ep].valid) continue;
        g_active[ep] = g_pending[ep];
        g_pending[ep].attrs = NULL;
        g_pending[ep].count = 0;
        g_pending[ep].valid = false;
    }
    memset(g_pending_seen, 0, sizeof(g_pending_seen));

    return ESP_OK;
}

esp_err_t nuos_config_load_persisted(uint8_t*out){free_table(g_active);if(out)*out=0;nvs_handle_t h;esp_err_t e=nvs_open(CFG_NS,NVS_READONLY,&h);if(e!=ESP_OK)return ESP_OK;cfg_meta_t m={0};size_t ms=sizeof(m);e=nvs_get_blob(h,CFG_META,&m,&ms);if(e!=ESP_OK||ms!=sizeof(m)||m.magic!=CFG_MAGIC||m.version!=CFG_VERSION){nvs_close(h);return ESP_OK;}for(uint8_t ep=1;ep<=NUOS_TOTAL_ENDPOINTS;ep++){char key[8];snprintf(key,sizeof(key),"ep%u",ep);size_t sz=0;if(nvs_get_blob(h,key,NULL,&sz)!=ESP_OK||sz<sizeof(cfg_ep_hdr_t))continue;uint8_t*b=malloc(sz);if(!b){nvs_close(h);return ESP_ERR_NO_MEM;}size_t got=sz;e=nvs_get_blob(h,key,b,&got);if(e!=ESP_OK){free(b);continue;}cfg_ep_hdr_t hdr;memcpy(&hdr,b,sizeof(hdr));size_t need=sizeof(hdr)+(size_t)hdr.count*2;if(hdr.magic==CFG_MAGIC&&hdr.version==CFG_VERSION&&need==got&&hdr.count&&hdr.cluster_id){dyn_ep_t*d=&g_active[ep-1];d->valid=true;d->device_type=hdr.device_type;d->cluster_id=hdr.cluster_id;d->count=hdr.count;d->attrs=calloc(d->count,sizeof(dyn_attr_t));if(!d->attrs){free(b);nvs_close(h);return ESP_ERR_NO_MEM;}for(uint16_t i=0;i<d->count;i++){d->attrs[i].id=b[sizeof(hdr)+i*2];d->attrs[i].type=b[sizeof(hdr)+i*2+1];d->attrs[i].configured=true;}}free(b);}nvs_close(h);if(out)*out=m.endpoint_count;uint32_t total=0;for(uint8_t ep=0;ep<NUOS_TOTAL_ENDPOINTS;ep++)if(g_active[ep].valid)total+=g_active[ep].count;ESP_LOGI(TAG,"ZB_CONFIG NVS: restored endpoints=%u configured_attrs=%lu",m.endpoint_count,(unsigned long)total);return ESP_OK;}

void nuos_config_dump(void){for(uint8_t ep=0;ep<NUOS_TOTAL_ENDPOINTS;ep++){dyn_ep_t*e=&g_active[ep];if(!e->valid)continue;ESP_LOGI(TAG,"EP%u configuration attrs=%u device_type=%u cluster=0x%04X",ep+1,e->count,e->device_type,e->cluster_id);for(uint16_t i=0;i<e->count;i++)ESP_LOGI(TAG,"  attr[%u] id=0x%02X type=%u zcl=0x%02X",i,e->attrs[i].id,e->attrs[i].type,zcl_type(e->attrs[i].type));}}

void nuos_zigbee_network_ready_set(bool ready){
    g_zigbee_network_ready=ready;ESP_LOGI(TAG,"Zigbee network readiness=%d (router-joined semantics)",ready?1:0);
}
bool nuos_zigbee_network_ready(void){return g_zigbee_network_ready;}

bool nuos_config_rebuild_requested(void){nvs_handle_t h;esp_err_t e=nvs_open(CFG_NS,NVS_READONLY,&h);if(e!=ESP_OK)return false;uint8_t v=0;e=nvs_get_u8(h,CFG_REBUILD_KEY,&v);nvs_close(h);return e==ESP_OK&&v!=0;}
esp_err_t nuos_config_request_rebuild(void){nvs_handle_t h;esp_err_t e=nvs_open(CFG_NS,NVS_READWRITE,&h);if(e!=ESP_OK)return e;e=nvs_set_u8(h,CFG_REBUILD_KEY,1);if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);return e;}
void nuos_config_clear_rebuild_request(void){nvs_handle_t h;if(nvs_open(CFG_NS,NVS_READWRITE,&h)!=ESP_OK)return;(void)nvs_erase_key(h,CFG_REBUILD_KEY);(void)nvs_commit(h);nvs_close(h);}

bool nuos_init_sequence(void){uint8_t count=0;if(nuos_config_load_persisted(&count)!=ESP_OK)return false;ESP_LOGI(TAG,"H2 Zigbee Router: dynamic configuration endpoints=%u (no attribute maximum)",count);if(count)nuos_config_dump();return true;}
