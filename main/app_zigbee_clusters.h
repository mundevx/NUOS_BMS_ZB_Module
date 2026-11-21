#ifndef APP_ZIGBEE_CLUSTERS_H
#define APP_ZIGBEE_CLUSTERS_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nuos_link_proto.h"
#define NUOS_FIRST_ENDPOINT 1
#define NUOS_TOTAL_ENDPOINTS 50
#define NUOS_FIRST_ATTR_ID 0x0000
#define NUOS_REPORT_DST_SHORT 0x0000
#define NUOS_REPORT_DST_EP 1
#define NUOS_MAX_ASDU_LEN 72  /* conservative per-report ASDU; 52/64 attrs are packed across multiple reports */

/* No application-defined attribute-count limit exists on H2. Every endpoint
 * owns a heap allocation sized to the C6 configuration. */
esp_zb_ep_list_t *nuos_init_zb_clusters(void);
void nuos_config_begin(void);
esp_err_t nuos_configure_endpoint_chunk(uint8_t endpoint,uint8_t device_type,uint16_t cluster_id,uint16_t start_index,const uint8_t *pairs,uint16_t count);
esp_err_t nuos_config_validate_pending(uint8_t expected_endpoint_count);
esp_err_t nuos_config_save_persisted(uint8_t endpoint_count);
esp_err_t nuos_config_load_persisted(uint8_t *endpoint_count_out);
void nuos_config_dump(void);
bool nuos_config_rebuild_requested(void);
esp_err_t nuos_config_request_rebuild(void);
void nuos_config_clear_rebuild_request(void);
void nuos_zigbee_network_ready_set(bool ready);
bool nuos_zigbee_network_ready(void);
esp_err_t nuos_apply_endpoint_payload(uint8_t endpoint,const uint8_t *payload,uint16_t len);
esp_err_t nuos_apply_endpoint_payload_from_zigbee_task(uint8_t endpoint,const uint8_t *payload,uint16_t len);
esp_err_t nuos_send_bulk_report(uint8_t endpoint);
esp_err_t nuos_send_raw_asdu(uint8_t src_endpoint,uint16_t cluster_id,uint16_t dst_short,uint8_t dst_endpoint,const uint8_t *asdu,uint8_t asdu_len);
esp_err_t send_report(uint8_t endpoint,uint16_t cluster_id,uint16_t attr_id);
bool nuos_init_sequence(void);
#endif
