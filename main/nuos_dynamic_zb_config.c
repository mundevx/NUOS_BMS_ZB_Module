#include "nuos_dynamic_zb_config.h"

#include <string.h>
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "NUOS_DYN_CFG";

static nuos_dynamic_endpoint_t g_cfg[NUOS_DYNAMIC_MAX_ENDPOINTS];
static bool g_loaded;

static void ensure_loaded(void)
{
    if (g_loaded) return;

    memset(g_cfg, 0, sizeof(g_cfg));

    nvs_handle_t h;
    if (nvs_open("nuos_zb_cfg", NVS_READONLY, &h) == ESP_OK) {
        size_t sz = sizeof(g_cfg);
        if (nvs_get_blob(h, "endpoints", g_cfg, &sz) != ESP_OK ||
            sz != sizeof(g_cfg)) {
            memset(g_cfg, 0, sizeof(g_cfg));
        }
        nvs_close(h);
    }

    g_loaded = true;
}

static esp_err_t save_config(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("nuos_zb_cfg", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, "endpoints", g_cfg, sizeof(g_cfg));
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    return err;
}

static bool config_equal(const nuos_dynamic_endpoint_t *a,
                         const nuos_dynamic_endpoint_t *b)
{
    if (!a || !b) return false;

    if (a->valid != b->valid ||
        a->endpoint_id != b->endpoint_id ||
        a->device_type != b->device_type ||
        a->cluster_id != b->cluster_id ||
        a->attribute_count != b->attribute_count) {
        return false;
    }

    return memcmp(a->attr, b->attr, sizeof(a->attr)) == 0;
}

esp_err_t nuos_dynamic_config_apply(const nuos_dynamic_endpoint_t *cfg,
                                    bool *configuration_changed)
{
    if (!cfg ||
        cfg->endpoint_id >= NUOS_DYNAMIC_MAX_ENDPOINTS ||
        cfg->attribute_count > NUOS_DYNAMIC_MAX_ATTRIBUTES ||
        cfg->cluster_id == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ensure_loaded();

    nuos_dynamic_endpoint_t next = *cfg;
    next.valid = true;

    bool changed = !config_equal(&g_cfg[next.endpoint_id], &next);

    g_cfg[next.endpoint_id] = next;

    esp_err_t err = save_config();

    if (configuration_changed) {
        *configuration_changed = changed;
    }

    ESP_LOGI(TAG,
             "EP%u device=%u cluster=0x%04X attrs=%u changed=%s",
             next.endpoint_id,
             next.device_type,
             next.cluster_id,
             next.attribute_count,
             changed ? "YES" : "NO");

    return err;
}

uint16_t nuos_dynamic_cluster_for_endpoint(uint8_t endpoint_id)
{
    ensure_loaded();

    if (endpoint_id >= NUOS_DYNAMIC_MAX_ENDPOINTS ||
        !g_cfg[endpoint_id].valid) {
        return 0;
    }

    return g_cfg[endpoint_id].cluster_id;
}

const nuos_dynamic_endpoint_t *
nuos_dynamic_config_get(uint8_t endpoint_id)
{
    ensure_loaded();

    if (endpoint_id >= NUOS_DYNAMIC_MAX_ENDPOINTS ||
        !g_cfg[endpoint_id].valid) {
        return NULL;
    }

    return &g_cfg[endpoint_id];
}

void nuos_dynamic_config_clear(void)
{
    memset(g_cfg, 0, sizeof(g_cfg));
    g_loaded = true;
    (void)save_config();
}
