/*
 * NUOS BMS - Section 2: ESP32-H2 Zigbee board
 *
 * Zigbee endpoints, cluster IDs and attributes are loaded dynamically from
 * the configuration persisted by the ESP32-C6. H2 does not contain a
 * device-specific cluster map or an application-defined attribute ceiling.
 *
 * There is no Modbus master here. The ESP32-C6 polls RS485 and sends endpoint
 * data over the existing UART link. H2 applies only attributes that were
 * declared by the C6 configuration and reports them on that endpoint cluster.
 *
 * Zigbee role: ROUTER (ESP_ZB_ZR_CONFIG).
 *
 * UART link to the ESP32-C6 uses the existing project UART configuration.
 * Status LED = WS2812 addressable RGB on GPIO8
 */

#define DECLARE_MAIN

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_controller_main.h"
#include "app_zigbee_clusters.h"
#include "nuos_uart_link.h"


static const char *TAG = "ZB_MAIN_FILE";

/*
 * Router commissioning policy:
 * - Factory-new devices start network steering.
 * - A device that already has Zigbee network information first performs
 *   normal BDB initialization/rejoin.
 * - If it is still not joined after a short grace period, network steering
 *   is started as a recovery path.
 * - If the parent/coordinator disappears or the device leaves the network,
 *   steering is retried automatically.
 */
#define NUOS_JOIN_RECOVERY_DELAY_MS 8000
#define NUOS_STEERING_RETRY_MS      5000

/* Set to 1 for ONE firmware build when you need to clear the H2 Zigbee
 * network credentials. After flashing/booting once, set it back to 0 and
 * rebuild/flash. Do NOT leave this enabled or the router will factory-reset
 * its Zigbee network state on every boot. */
#ifndef NUOS_ZIGBEE_FACTORY_RESET_ON_BOOT
#define NUOS_ZIGBEE_FACTORY_RESET_ON_BOOT 0
#endif

static void nuos_start_steering(void);
static void nuos_join_recovery_cb(uint8_t unused);

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_err_t ret =
        esp_zb_bdb_start_top_level_commissioning(mode_mask);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "Commissioning start failed mode=0x%02X: %s (0x%x)",
                 mode_mask, esp_err_to_name(ret), ret);
    } else {
        ESP_LOGI(TAG, "Commissioning requested mode=0x%02X", mode_mask);
    }
}

static void nuos_start_steering(void)
{
    nuos_status_led_set_state(NUOS_LED_JOINING);
    ESP_LOGI(TAG, "ROUTER: starting network steering (join existing coordinator network)");
    esp_err_t ret = esp_zb_bdb_start_top_level_commissioning(
        ESP_ZB_BDB_MODE_NETWORK_STEERING);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ROUTER: network steering request failed: %s (0x%x)",
                 esp_err_to_name(ret), ret);
        esp_zb_scheduler_alarm(
            bdb_start_top_level_commissioning_cb,
            ESP_ZB_BDB_MODE_NETWORK_STEERING,
            NUOS_STEERING_RETRY_MS);
    }
}

static void nuos_join_recovery_cb(uint8_t unused)
{
    (void)unused;

    if (esp_zb_bdb_dev_joined()) {
        nuos_zigbee_network_ready_set(true);
        nuos_status_led_set_state(NUOS_LED_JOINED);
         esp_zb_ieee_addr_t local_ieee_addr;
            esp_zb_get_long_address(local_ieee_addr);

            ESP_LOGI(TAG,
                     "ROUTER: PAN=0x%04X channel=%u short=0x%04X IEEE=0x%02X%02X%02X%02X%02X%02X%02X%02X",
                     esp_zb_get_pan_id(),
                     esp_zb_get_current_channel(),
                     esp_zb_get_short_address(),
                     local_ieee_addr[7], local_ieee_addr[6],
                     local_ieee_addr[5], local_ieee_addr[4],
                     local_ieee_addr[3], local_ieee_addr[2],
                     local_ieee_addr[1], local_ieee_addr[0]);
        // ESP_LOGI(TAG,
        //          "ROUTER: already joined/rejoined; PAN=0x%04X channel=%u short=0x%04X",
        //          esp_zb_get_pan_id(),
        //          esp_zb_get_current_channel(),
        //          esp_zb_get_short_address());
        return;
    }

    nuos_zigbee_network_ready_set(false);

    ESP_LOGW(TAG,
             "ROUTER: not joined after initialization; starting network steering");
    nuos_start_steering();
}


void esp_zb_app_signal_handler(
    esp_zb_app_signal_t *signal_struct)
{
    if (signal_struct == NULL) {
        return;
    }

    uint32_t *p_sg_p =
        signal_struct->p_app_signal;

    esp_err_t err_status =
        signal_struct->esp_err_status;

    esp_zb_app_signal_type_t sig_type =
        *p_sg_p;

    switch (sig_type) {

    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:

        nuos_zigbee_network_ready_set(false);
        ESP_LOGI(TAG, "Initialize Zigbee stack");

        esp_zb_bdb_start_top_level_commissioning(
            ESP_ZB_BDB_MODE_INITIALIZATION);

        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:

        if (err_status == ESP_OK) {

            ESP_LOGI(
                TAG,
                "Zigbee started, factory_new=%s",
                esp_zb_bdb_is_factory_new() ? "YES" : "NO");

            /*
             * Nothing to start here any more.
             *
             * The UART link is already running (brought up in app_main), and
             * the heartbeat task tells the C6 when the network is joined.
             * The C6 then waits its settle time and begins polling.
             */

            if (esp_zb_bdb_is_factory_new()) {

                nuos_zigbee_network_ready_set(false);
                nuos_status_led_set_state(NUOS_LED_JOINING);
                ESP_LOGI(TAG,
                         "ROUTER: factory-new device; steering will search for an existing coordinator network");

                esp_zb_scheduler_alarm(
                    bdb_start_top_level_commissioning_cb,
                    ESP_ZB_BDB_MODE_NETWORK_STEERING,
                    1000);

            } else {

                nuos_zigbee_network_ready_set(false);
                nuos_status_led_set_state(NUOS_LED_REJOINING);
                ESP_LOGI(TAG,
                         "ROUTER: existing network information found; waiting for normal rejoin");

                /* Give BDB initialization/rejoin time to restore the parent.
                 * If that does not happen, fall back to active network steering.
                 */
                esp_zb_scheduler_alarm(
                    nuos_join_recovery_cb,
                    0,
                    NUOS_JOIN_RECOVERY_DELAY_MS);
            }

        } else {

            nuos_zigbee_network_ready_set(false);
            nuos_status_led_set_state(NUOS_LED_ERROR);
            ESP_LOGW(TAG,
                     "Zigbee startup failed: %s",
                     esp_err_to_name(err_status));

            esp_zb_scheduler_alarm(
                bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_INITIALIZATION,
                1000);
        }

        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:

        if (err_status == ESP_OK) {

            nuos_zigbee_network_ready_set(true);
            nuos_status_led_set_state(NUOS_LED_JOINED);
            ESP_LOGI(TAG,
                     "ROUTER: JOINED Zigbee coordinator network successfully");
            esp_zb_ieee_addr_t local_ieee_addr;
            esp_zb_get_long_address(local_ieee_addr);

            ESP_LOGI(TAG,
                     "ROUTER: PAN=0x%04X channel=%u short=0x%04X IEEE=0x%02X%02X%02X%02X%02X%02X%02X%02X",
                     esp_zb_get_pan_id(),
                     esp_zb_get_current_channel(),
                     esp_zb_get_short_address(),
                     local_ieee_addr[7], local_ieee_addr[6],
                     local_ieee_addr[5], local_ieee_addr[4],
                     local_ieee_addr[3], local_ieee_addr[2],
                     local_ieee_addr[1], local_ieee_addr[0]);

        } else {

            nuos_zigbee_network_ready_set(false);
            nuos_status_led_set_state(NUOS_LED_JOINING);
            ESP_LOGW(TAG,
                     "ROUTER: network steering failed: %s; retrying in %u ms",
                     esp_err_to_name(err_status),
                     (unsigned)NUOS_STEERING_RETRY_MS);

            esp_zb_scheduler_alarm(
                bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING,
                NUOS_STEERING_RETRY_MS);
        }

        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
    {
        /* This payload identifies the remote Zigbee device that joined/rejoined. */
        esp_zb_zdo_signal_device_annce_params_t *dev_annce =
            (esp_zb_zdo_signal_device_annce_params_t *)
                esp_zb_app_signal_get_params(p_sg_p);

        if (dev_annce != NULL) {
            ESP_LOGI(TAG,
                     "ZIGBEE DEVICE JOINED/ANNOUNCED: short=0x%04X IEEE=0x%02X%02X%02X%02X%02X%02X%02X%02X capability=0x%02X",
                     dev_annce->device_short_addr,
                     dev_annce->ieee_addr[7], dev_annce->ieee_addr[6],
                     dev_annce->ieee_addr[5], dev_annce->ieee_addr[4],
                     dev_annce->ieee_addr[3], dev_annce->ieee_addr[2],
                     dev_annce->ieee_addr[1], dev_annce->ieee_addr[0],
                     dev_annce->capability);
        } else {
            ESP_LOGW(TAG, "DEVICE_ANNCE received without announcement parameters");
        }

        break;
    }

    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:

        nuos_zigbee_network_ready_set(false);
        nuos_status_led_set_state(NUOS_LED_REJOINING);
        ESP_LOGW(TAG,
                 "ROUTER: parent/coordinator unavailable; scheduling steering recovery");

        esp_zb_scheduler_alarm(
            bdb_start_top_level_commissioning_cb,
            ESP_ZB_BDB_MODE_NETWORK_STEERING,
            NUOS_STEERING_RETRY_MS);

        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:

        nuos_zigbee_network_ready_set(false);
        nuos_status_led_set_state(NUOS_LED_REJOINING);
        ESP_LOGW(TAG,
                 "ROUTER: left Zigbee network; restarting network steering");

        esp_zb_scheduler_alarm(
            bdb_start_top_level_commissioning_cb,
            ESP_ZB_BDB_MODE_NETWORK_STEERING,
            1000);

        break;

    default:
        break;
    }
}

// // Define the standard Zigbee HA Link Key
// uint8_t standard_ha_link_key[] = {0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, 
//                                   0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39};


static void esp_zb_task(void *pvParameters)
{
    (void)pvParameters;

    /*
     * ROUTER, not coordinator. This board joins the existing network via
     * network steering and reports up to the coordinator at 0x0000.
     */

    esp_zb_cfg_t zb_nwk_cfg =
        ESP_ZB_ZR_CONFIG();

    esp_zb_init(&zb_nwk_cfg);

    // esp_err_t key_err = esp_zb_secur_multi_TC_standard_preconfigure_key_add(
    //     standard_ha_link_key);
    // if (key_err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to register Trust Center link key: %s (0x%x)",
    //              esp_err_to_name(key_err), key_err);
    // }


    nuos_zigbee_network_ready_set(false);

    ESP_LOGI(TAG, "Free heap before endpoint registration: %u", (unsigned)esp_get_free_heap_size());

    esp_zb_ep_list_t *ep_list =
        nuos_init_zb_clusters();

    if (ep_list == NULL) {
        ESP_LOGE(TAG, "Zigbee endpoint list allocation failed; free heap=%u",
                 (unsigned)esp_get_free_heap_size());
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Free heap after endpoint creation: %u", (unsigned)esp_get_free_heap_size());

    esp_zb_device_register(ep_list);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);



#if NUOS_ZIGBEE_FACTORY_RESET_ON_BOOT
    ESP_LOGW(TAG, "*** ZIGBEE FACTORY RESET ENABLED: clearing stored network credentials ***");
    esp_zb_factory_reset();
    ESP_LOGW(TAG, "*** ZIGBEE FACTORY RESET REQUESTED; reboot/pair as a new router ***");
    return;
#endif

    // esp_zb_factory_reset();
    ESP_ERROR_CHECK(
        esp_zb_start(false));

    /* From here on the link tasks may take the Zigbee lock. */
    nuos_link_notify_zb_started();
    nuos_status_led_set_state(NUOS_LED_ZIGBEE_STARTED);

    esp_zb_stack_main_loop();
    
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode =
                ZB_HOST_CONNECTION_MODE_NONE,
        },
    };

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NOT_FOUND ||
        ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());

    } else if (ret != ESP_OK) {

        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(
        esp_zb_platform_config(&config));

    if (!nuos_init_sequence()) {

        ESP_LOGE(TAG,
                 "Application initialization failed");
        return;
    }

    /*
     * Bring the UART link up before the stack. Frames that arrive while the
     * network is still forming are answered with NUOS_LINK_ST_NOT_JOINED,
     * which is exactly what keeps the C6 from polling too early.
     */
    ESP_ERROR_CHECK(nuos_link_init());

    if (xTaskCreate(
            esp_zb_task,
            "Zigbee_main",
            16384,
            NULL,
            5,
            NULL) != pdPASS) {

        ESP_LOGE(TAG,
                 "Failed to create Zigbee task");
    }
}
