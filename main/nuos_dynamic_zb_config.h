#ifndef NUOS_DYNAMIC_ZB_CONFIG_H
#define NUOS_DYNAMIC_ZB_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define NUOS_DYNAMIC_MAX_ENDPOINTS 50
#define NUOS_DYNAMIC_MAX_ATTRIBUTES 64

typedef struct {
    uint8_t attribute_id;
    uint8_t data_type;
} nuos_dynamic_attr_t;

typedef struct {
    bool valid;
    uint8_t endpoint_id;
    uint8_t device_type;
    uint16_t cluster_id;
    uint16_t attribute_count;
    nuos_dynamic_attr_t attr[NUOS_DYNAMIC_MAX_ATTRIBUTES];
} nuos_dynamic_endpoint_t;

esp_err_t nuos_dynamic_config_apply(const nuos_dynamic_endpoint_t *cfg,
                                    bool *configuration_changed);

uint16_t nuos_dynamic_cluster_for_endpoint(uint8_t endpoint_id);

const nuos_dynamic_endpoint_t *
nuos_dynamic_config_get(uint8_t endpoint_id);

void nuos_dynamic_config_clear(void);

#endif
