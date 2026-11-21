#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "nuos_uart_link.h"
#include "app_zigbee_clusters.h"
#include "light_driver.h"
#include "nvs.h"
#include "esp_system.h"

static const char *TAG = "NUOS_LINK";

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t ep;
    uint16_t len;
    uint8_t *payload;
} work_t;

static QueueHandle_t q;
static SemaphoreHandle_t txm;
static nuos_link_parser_t parser;
static volatile bool zb_started;
static volatile uint32_t rx_count;
static volatile uint32_t applied_count;
static volatile uint32_t crc_count;
static volatile uint32_t drop_count;
static uint8_t hb_seq;
static volatile uint32_t s_config_hash;
static volatile uint8_t s_config_endpoint_count;
static volatile uint32_t s_pending_config_hash;
static volatile uint8_t s_pending_config_endpoint_count;
static bool s_config_in_progress;

#define NUOS_LINK_NVS_NS       "nuos_link"
#define NUOS_LINK_NVS_HASH     "cfg_hash"
#define NUOS_LINK_NVS_ENDPOINT "cfg_eps"

/* H2 has one addressable WS2812 on GPIO8.
 *
 * There is no steady LED state. A dedicated LED task continuously blinks the
 * current status color. Zigbee/UART callbacks only change the state variable,
 * so light_driver is never driven concurrently by callbacks.
 *
 * Colors:
 *   WHITE   = boot / initializing
 *   RED     = UART not detected
 *   ORANGE  = UART detected before Zigbee start
 *   BLUE    = Zigbee stack started, waiting for join
 *   PURPLE  = router joining / commissioning
 *   GREEN   = Zigbee joined, UART not currently detected
 *   CYAN    = Zigbee joined + C6 UART OK
 *   MAGENTA = rejoin / parent lost recovery
 *   RED FAST= fatal initialization error
 */
static volatile nuos_led_state_t s_led_state = NUOS_LED_BOOT;
static volatile int64_t last_c6_rx_us;

typedef struct {
    uint8_t r, g, b;
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t pulses;
    uint16_t gap_ms;
} led_pattern_t;

static bool joined(void);

static led_pattern_t led_pattern(nuos_led_state_t state)
{
    switch (state) {
    case NUOS_LED_UART_NOT_DETECTED:
        return (led_pattern_t){255, 0, 0, 200, 800, 1, 0};
    case NUOS_LED_UART_OK:
        return (led_pattern_t){255, 80, 0, 200, 200, 2, 1000};
    case NUOS_LED_ZIGBEE_STARTED:
        return (led_pattern_t){0, 0, 255, 600, 600, 1, 0};
    case NUOS_LED_JOINING:
        return (led_pattern_t){160, 0, 255, 100, 100, 1, 0};
    case NUOS_LED_JOINED:
        return (led_pattern_t){0, 255, 0, 200, 1800, 1, 0};
    case NUOS_LED_JOINED_UART_OK:
        return (led_pattern_t){0, 255, 255, 200, 200, 2, 1400};
    case NUOS_LED_REJOINING:
        return (led_pattern_t){255, 0, 180, 600, 200, 2, 1000};
    case NUOS_LED_ERROR:
        return (led_pattern_t){255, 0, 0, 50, 50, 1, 0};
    case NUOS_LED_BOOT:
    default:
        return (led_pattern_t){255, 255, 255, 200, 200, 2, 1200};
    }
}

void nuos_status_led_set_state(nuos_led_state_t state)
{
    s_led_state = state;
}

static void status_led_task(void *arg)
{
    (void)arg;
    nuos_led_state_t previous = (nuos_led_state_t)0xFF;
    uint64_t state_start = esp_timer_get_time() / 1000ULL;
    bool output_on = false;

    for (;;) {
        nuos_led_state_t state = s_led_state;
        if (state != previous) {
            previous = state;
            state_start = esp_timer_get_time() / 1000ULL;
            output_on = false;
            light_driver_set_power(false);
        }

        led_pattern_t p = led_pattern(state);
        uint32_t pulse_len = (uint32_t)p.on_ms + (uint32_t)p.off_ms;
        uint32_t cycle_len = pulse_len * p.pulses + p.gap_ms;
        uint64_t elapsed = (esp_timer_get_time() / 1000ULL) - state_start;
        uint32_t phase = cycle_len ? (uint32_t)(elapsed % cycle_len) : 0;
        bool should_on = false;

        if (pulse_len != 0 && phase < pulse_len * p.pulses) {
            uint32_t pulse_phase = phase % pulse_len;
            should_on = pulse_phase < p.on_ms;
        }

        if (should_on != output_on) {
            output_on = should_on;
            if (output_on) {
                light_driver_set_color_RGB(p.r, p.g, p.b);
                light_driver_set_power(true);
            } else {
                light_driver_set_power(false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static bool c6_uart_recent(void)
{
    int64_t t = last_c6_rx_us;
    if (t == 0) {
        return false;
    }
    return (esp_timer_get_time() - t) < 5000000LL;
}

static void status_led_update(void)
{
    bool c6_seen = c6_uart_recent();
    nuos_led_state_t state = s_led_state;

    /* Automatic UART indication is only used before Zigbee starts and after
     * the router is joined. Explicit JOINING/REJOINING/ERROR states always
     * have priority, so the LED accurately represents Zigbee state. */
    if (state == NUOS_LED_BOOT || state == NUOS_LED_UART_NOT_DETECTED ||
        state == NUOS_LED_UART_OK) {
        if (zb_started) {
            state = joined() ?
                (c6_seen ? NUOS_LED_JOINED_UART_OK : NUOS_LED_JOINED) :
                NUOS_LED_ZIGBEE_STARTED;
        } else {
            state = c6_seen ? NUOS_LED_UART_OK : NUOS_LED_UART_NOT_DETECTED;
        }
    } else if (state == NUOS_LED_JOINED && c6_seen) {
        state = NUOS_LED_JOINED_UART_OK;
    } else if (state == NUOS_LED_JOINED_UART_OK && !c6_seen) {
        state = NUOS_LED_JOINED;
    }

    s_led_state = state;
}

static bool joined(void)
{
    /*
     * H2 is a Zigbee ROUTER and joins an existing coordinator network.
     * The router signal handler updates the shared readiness state only after
     * successful BDB steering/rejoin. This keeps C6 Modbus polling blocked
     * until the H2 has a valid parent/network attachment.
     */
    return zb_started && nuos_zigbee_network_ready();
}

static esp_err_t load_persisted_link_config(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NUOS_LINK_NVS_NS, NVS_READONLY, &h);
    if (e != ESP_OK) {
        s_config_hash = 0;
        s_config_endpoint_count = 0;
        ESP_LOGI(TAG, "UART config NVS: no persisted link hash");
        return ESP_OK;
    }

    uint32_t hash = 0;
    uint8_t eps = 0;
    esp_err_t eh = nvs_get_u32(h, NUOS_LINK_NVS_HASH, &hash);
    esp_err_t ee = nvs_get_u8(h, NUOS_LINK_NVS_ENDPOINT, &eps);
    nvs_close(h);

    if (eh == ESP_OK && ee == ESP_OK && eps <= NUOS_TOTAL_ENDPOINTS) {
        s_config_hash = hash;
        s_config_endpoint_count = eps;
        ESP_LOGI(TAG, "UART config NVS: restored hash=0x%08lX endpoints=%u",
                 (unsigned long)hash, eps);
    } else {
        s_config_hash = 0;
        s_config_endpoint_count = 0;
        ESP_LOGI(TAG, "UART config NVS: no valid persisted link hash");
    }
    return ESP_OK;
}

static esp_err_t save_persisted_link_config(uint32_t hash, uint8_t endpoints)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NUOS_LINK_NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;

    e = nvs_set_u32(h, NUOS_LINK_NVS_HASH, hash);
    if (e == ESP_OK) e = nvs_set_u8(h, NUOS_LINK_NVS_ENDPOINT, endpoints);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);

    if (e != ESP_OK) {
        ESP_LOGE(TAG, "UART config NVS: save failed: %s", esp_err_to_name(e));
    }
    return e;
}

static uint8_t status_flags(void)
{
    uint8_t flags = 0;

    if (joined()) {
        flags |= NUOS_LINK_FLAG_ZB_JOINED;
    }
    if (zb_started) {
        flags |= NUOS_LINK_FLAG_ZB_STARTED;
    }

    return flags;
}

static esp_err_t tx_frame(uint8_t type, uint8_t seq, uint8_t ep,
                          const uint8_t *p, uint16_t len)
{
    uint8_t *frame = malloc(NUOS_LINK_MAX_FRAME);
    if (!frame) {
        ESP_LOGE(TAG, "TX allocation failed type=0x%02X len=%u", type, len);
        return ESP_ERR_NO_MEM;
    }

    uint16_t n = nuos_link_build(frame, type, seq, ep, p, len);
    if (!n) {
        free(frame);
        ESP_LOGE(TAG, "TX frame build failed type=0x%02X len=%u", type, len);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(txm, pdMS_TO_TICKS(1000)) != pdTRUE) {
        free(frame);
        ESP_LOGW(TAG, "TX mutex timeout type=0x%02X", type);
        return ESP_ERR_TIMEOUT;
    }

    int w = uart_write_bytes(NUOS_LINK_UART, frame, n);
    esp_err_t wait_ret = uart_wait_tx_done(NUOS_LINK_UART, pdMS_TO_TICKS(1000));
    xSemaphoreGive(txm);

    if (w != (int)n || wait_ret != ESP_OK) {
        ESP_LOGW(TAG, "TX failed type=0x%02X seq=%u ep=%u bytes=%d/%u wait=0x%x",
                 type, seq, ep, w, n, (unsigned)wait_ret);
        free(frame);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TX type=0x%02X seq=%u ep=%u len=%u", type, seq, ep, len);
    free(frame);
    return ESP_OK;
}

static void put_u32_le_local(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void send_status(const char *reason)
{
    uint8_t flags = status_flags();
    /* STATUS payload: [flags][4-byte C6 configuration hash][crc8].
     * C6 uses bytes 1..4 as the synchronized configuration hash. */
    uint8_t p[6];
    p[0] = flags;
    put_u32_le_local(&p[1], s_config_hash);
    p[5] = (uint8_t)(parser.crc_errors & 0xFFu);

    ESP_LOGI(TAG, "STATUS TX reason=%s flags=0x%02X started=%d joined=%d config_hash=0x%08lX crc=%lu",
             reason, flags, zb_started, (flags & NUOS_LINK_FLAG_ZB_JOINED) != 0,
             (unsigned long)s_config_hash, (unsigned long)parser.crc_errors);

    (void)tx_frame(NUOS_LINK_TYPE_STATUS, hb_seq++, 0, p, sizeof(p));
}

void nuos_link_notify_zb_started(void)
{
    zb_started = true;
    /* Called from the Zigbee task after esp_zb_start(); do not acquire the
     * Zigbee lock here. The LED task/heartbeat will refine this to JOINED or
     * JOINED+UART when the network state is available. */
    nuos_status_led_set_state(NUOS_LED_ZIGBEE_STARTED);
    ESP_LOGI(TAG, "Zigbee stack started; sending immediate H2 status");
    send_status("zigbee-started");
}

static void ack(uint8_t seq, uint8_t ep, uint8_t status)
{
    uint8_t p[1] = { status };
    esp_err_t r = tx_frame(NUOS_LINK_TYPE_ACK, seq, ep, p, sizeof(p));
    ESP_LOGI(TAG, "ACK TX seq=%u ep=%u status=0x%02X result=%s",
             seq, ep, status, esp_err_to_name(r));
}

static void handle_frame(const nuos_link_frame_t *f)
{
    rx_count++;
    last_c6_rx_us = esp_timer_get_time();
    status_led_update();

    ESP_LOGI(TAG, "RX type=0x%02X seq=%u ep=%u len=%u",
             f->type, f->seq, f->endpoint, f->len);

    if (f->type == NUOS_LINK_TYPE_PING) {
        send_status("ping");
        return;
    }

    /* Configuration handshake from C6. These messages are ACKed immediately
     * because the C6 waits for an ACK before sending the next phase. */
    if (f->type == NUOS_LINK_TYPE_ZB_CONFIG_BEGIN) {
        if (f->len != 6 || f->payload[0] != NUOS_ZB_CONFIG_VERSION) {
            ack(f->seq, f->endpoint, NUOS_LINK_ST_BAD_LENGTH);
            return;
        }
        uint32_t hash = (uint32_t)f->payload[1] | ((uint32_t)f->payload[2] << 8) |
                        ((uint32_t)f->payload[3] << 16) | ((uint32_t)f->payload[4] << 24);
        uint8_t endpoints = f->payload[5];
        if (endpoints > NUOS_TOTAL_ENDPOINTS) {
            ack(f->seq, f->endpoint, NUOS_LINK_ST_BAD_LENGTH);
            return;
        }
        s_config_in_progress = true;
        nuos_config_begin();
        s_pending_config_hash = hash;
        s_pending_config_endpoint_count = endpoints;
        ESP_LOGI(TAG, "ZB_CONFIG BEGIN hash=0x%08lX endpoints=%u",
                 (unsigned long)hash, endpoints);
        ack(f->seq, f->endpoint, NUOS_LINK_ST_OK);
        return;
    }

    if (f->type == NUOS_LINK_TYPE_ZB_CONFIG_EP) {
        /* Payload: version, device_type, cluster_id(u16), start_index(u16), count(u16), pairs... */
        if (!s_config_in_progress || f->endpoint < 1 || f->endpoint > NUOS_TOTAL_ENDPOINTS || f->len < 8 ||
            f->payload[0] != NUOS_ZB_CONFIG_VERSION) {
            ack(f->seq, f->endpoint, NUOS_LINK_ST_BAD_LENGTH);
            return;
        }
        uint8_t device_type=f->payload[1];
        uint16_t cluster=(uint16_t)f->payload[2] | ((uint16_t)f->payload[3]<<8);
        uint16_t start=(uint16_t)f->payload[4] | ((uint16_t)f->payload[5]<<8);
        uint16_t count=(uint16_t)f->payload[6] | ((uint16_t)f->payload[7]<<8);
        if (!cluster || f->len != (uint16_t)(8u + count*2u) || count == 0) {
            ESP_LOGW(TAG,
                     "ZB_CONFIG EP%u rejected: start=%u count=%u cluster=0x%04X len=%u expected_len=%u",
                     f->endpoint, start, count, cluster, f->len,
                     (unsigned)(8u + count * 2u));
            ack(f->seq,f->endpoint,NUOS_LINK_ST_BAD_LENGTH);return;
        }
        ESP_LOGI(TAG,
                 "ZB_CONFIG EP%u RX chunk seq=%u device_type=%u cluster=0x%04X start=%u count=%u len=%u",
                 f->endpoint, f->seq, device_type, cluster, start, count, f->len);
        esp_err_t r=nuos_configure_endpoint_chunk(f->endpoint,device_type,cluster,start,&f->payload[8],count);
        ESP_LOGI(TAG, "ZB_CONFIG EP%u chunk result=%s; sending ACK",
                 f->endpoint, esp_err_to_name(r));
        ack(f->seq,f->endpoint,r==ESP_OK?NUOS_LINK_ST_OK:NUOS_LINK_ST_ZB_ERROR);
        return;
    }

    if (f->type == NUOS_LINK_TYPE_ZB_CONFIG_COMMIT) {
        if (!s_config_in_progress || f->len != 6 ||
            f->payload[0] != NUOS_ZB_CONFIG_VERSION) {
            ack(f->seq, f->endpoint, NUOS_LINK_ST_BAD_LENGTH);
            return;
        }
        uint32_t hash = (uint32_t)f->payload[1] | ((uint32_t)f->payload[2] << 8) |
                        ((uint32_t)f->payload[3] << 16) | ((uint32_t)f->payload[4] << 24);
        uint8_t endpoints = f->payload[5];
        if (hash != s_pending_config_hash || endpoints != s_pending_config_endpoint_count) {
            ESP_LOGW(TAG, "ZB_CONFIG COMMIT mismatch begin=0x%08lX/%u commit=0x%08lX/%u",
                     (unsigned long)s_pending_config_hash, s_pending_config_endpoint_count,
                     (unsigned long)hash, endpoints);
            ack(f->seq, f->endpoint, NUOS_LINK_ST_BAD_LENGTH);
            return;
        }

        ESP_LOGI(TAG,
                 "ZB_CONFIG COMMIT: validating and storing %u endpoints in NVS",
                 endpoints);
        esp_err_t save_types = nuos_config_save_persisted(endpoints);
        if (save_types != ESP_OK) {
            ESP_LOGW(TAG, "ZB_CONFIG COMMIT rejected: pending configuration invalid/save failed: %s", esp_err_to_name(save_types));
            ack(f->seq, f->endpoint, NUOS_LINK_ST_ZB_ERROR);
            return;
        }
        ESP_LOGI(TAG,
                 "ZB_CONFIG NVS endpoint map stored successfully: endpoints=%u",
                 endpoints);
        esp_err_t save_hash = save_persisted_link_config(hash, endpoints);
        if (save_hash != ESP_OK) {
            ESP_LOGE(TAG,
                     "ZB_CONFIG NVS hash save failed hash=0x%08lX: %s",
                     (unsigned long)hash, esp_err_to_name(save_hash));
            ack(f->seq, f->endpoint, NUOS_LINK_ST_ZB_ERROR);
            return;
        }
        ESP_LOGI(TAG,
                 "ZB_CONFIG NVS hash stored successfully: hash=0x%08lX endpoints=%u",
                 (unsigned long)hash, endpoints);

        /* nuos_config_save_persisted() already records the rebuild request when
         * the persisted endpoint map changes. Do not request it a second time
         * here; the COMMIT path must stay deterministic and fast. */
        s_config_hash = hash;
        s_config_endpoint_count = endpoints;
        s_config_in_progress = false;
        ESP_LOGI(TAG, "ZB_CONFIG COMMIT accepted + persisted hash=0x%08lX endpoints=%u",
                 (unsigned long)s_config_hash, s_config_endpoint_count);
        nuos_config_dump();

        /* IMPORTANT: ACK only after both the endpoint configuration and the
         * configuration hash have been committed to NVS. The C6 uses this ACK
         * as proof that the configuration is safely stored.
         *
         * Keep the reboot AFTER the ACK. Rebooting before the UART ACK reaches
         * the C6 causes the C6 to report ESP_ERR_TIMEOUT even though NVS may
         * already contain the new configuration. */
        ack(f->seq, f->endpoint, NUOS_LINK_ST_OK);
        send_status("config-commit");

        /* Cluster objects are created at Zigbee startup. Give the UART TX
         * path time to finish the ACK/status before restarting H2. */
        vTaskDelay(pdMS_TO_TICKS(750));
        esp_restart();
        return;
    }

    if (f->type != NUOS_LINK_TYPE_EP_DATA) {
        return;
    }

    if (f->endpoint < 1 || f->endpoint > NUOS_TOTAL_ENDPOINTS) {
        ack(f->seq, f->endpoint, NUOS_LINK_ST_BAD_ENDPOINT);
        return;
    }

    if (!joined()) {
        ack(f->seq, f->endpoint, NUOS_LINK_ST_NOT_JOINED);
        return;
    }

    if (f->len < 3) {
        ack(f->seq, f->endpoint, NUOS_LINK_ST_BAD_LENGTH);
        return;
    }

    work_t w = {
        .type = f->type,
        .seq = f->seq,
        .ep = f->endpoint,
        .len = f->len,
        .payload = malloc(f->len),
    };

    if (!w.payload) {
        ack(f->seq, f->endpoint, NUOS_LINK_ST_BUSY);
        return;
    }

    memcpy(w.payload, f->payload, f->len);

    if (xQueueSend(q, &w, 0) != pdTRUE) {
        free(w.payload);
        drop_count++;
        ack(f->seq, f->endpoint, NUOS_LINK_ST_BUSY);
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t b[512];
    nuos_link_parser_init(&parser);

    for (;;) {
        int n = uart_read_bytes(NUOS_LINK_UART, b, sizeof(b), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            nuos_link_frame_t f;
            if (nuos_link_parser_push(&parser, b[i], &f)) {
                handle_frame(&f);
            }
        }
    }
}


static void work_task(void *arg)
{
    (void)arg;
    work_t w;

    for (;;) {
        if (xQueueReceive(q, &w, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_LOGI(TAG, "WORK APPLY START seq=%u ep=%u len=%u", w.seq, w.ep, w.len);

        /* This project uses the ESP-Zigbee SDK v1.x compatibility API
         * (esp_zb_*). The v2-only esp_zigbee_task_queue_post() API must not
         * be used here. The UART worker is a normal FreeRTOS task, so the
         * public v1.x pattern is to call the H2 apply function, which acquires
         * and releases the Zigbee lock around all Zigbee API access. */
        esp_err_t r = nuos_apply_endpoint_payload(w.ep, w.payload, w.len);

        ESP_LOGI(TAG, "WORK APPLY END seq=%u ep=%u result=0x%x (%s)",
                 w.seq, w.ep, (unsigned)r, esp_err_to_name(r));

        if (r == ESP_OK) {
            applied_count++;
            ack(w.seq, w.ep, NUOS_LINK_ST_OK);
        } else if (r == ESP_ERR_INVALID_STATE) {
            ack(w.seq, w.ep, NUOS_LINK_ST_NOT_JOINED);
        } else if (r == ESP_ERR_INVALID_SIZE || r == ESP_ERR_INVALID_ARG) {
            ack(w.seq, w.ep, NUOS_LINK_ST_BAD_LENGTH);
        } else {
            ack(w.seq, w.ep, NUOS_LINK_ST_ZB_ERROR);
        }

        free(w.payload);
    }
}

static void hb_task(void *arg)
{
    uint32_t cc = 59;
    (void)arg;

    /* Send a heartbeat even before Zigbee has joined. This proves the UART
     * link is alive independently of Zigbee network state. */
    vTaskDelay(pdMS_TO_TICKS(250));
    status_led_update();
    send_status("startup");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(NUOS_LINK_HEARTBEAT_MS));
        if(cc++ >= 2){
            cc = 0;
            crc_count = parser.crc_errors;
            status_led_update();
            send_status("heartbeat");
        }
    }
}

esp_err_t nuos_link_init(void)
{
    /* Use Espressif's common/light_driver component for the single WS2812.
     * GPIO and LED count are supplied through CONFIG_EXAMPLE_STRIP_LED_GPIO
     * and CONFIG_EXAMPLE_STRIP_LED_NUMBER in sdkconfig.defaults.
     */
    light_driver_init(LIGHT_DEFAULT_OFF);
    nuos_status_led_set_state(NUOS_LED_BOOT);
    if (xTaskCreate(status_led_task, "nuos_status_led", 3072, NULL, 3, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
         "WS2812 status LED ready via light_driver GPIO=%d (always blinking)", NUOS_WS2812_GPIO);

    txm = xSemaphoreCreateMutex();
    q = xQueueCreate(NUOS_LINK_QUEUE_DEPTH, sizeof(work_t));
    (void)load_persisted_link_config();

    if (!txm || !q) {
        nuos_status_led_set_state(NUOS_LED_ERROR);
        return ESP_ERR_NO_MEM;
    }

    uart_config_t c = {
        .baud_rate = NUOS_LINK_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t r = uart_driver_install(NUOS_LINK_UART, NUOS_LINK_RX_BUF,
                                      NUOS_LINK_TX_BUF, 0, NULL, 0);
    if (r != ESP_OK) {
        nuos_status_led_set_state(NUOS_LED_ERROR);
        return r;
    }

    r = uart_param_config(NUOS_LINK_UART, &c);
    if (r != ESP_OK) {
        nuos_status_led_set_state(NUOS_LED_ERROR);
        return r;
    }

    r = uart_set_pin(NUOS_LINK_UART, NUOS_LINK_TX_PIN, NUOS_LINK_RX_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (r != ESP_OK) {
        nuos_status_led_set_state(NUOS_LED_ERROR);
        return r;
    }

    if (xTaskCreate(rx_task, "nuos_link_rx", 6144, NULL, 7, NULL) != pdPASS) {
        nuos_status_led_set_state(NUOS_LED_ERROR);
        return ESP_FAIL;
    }

    if (xTaskCreate(work_task, "nuos_link_work", 8192, NULL, 6, NULL) != pdPASS) {
        nuos_status_led_set_state(NUOS_LED_ERROR);
        return ESP_FAIL;
    }

    if (xTaskCreate(hb_task, "nuos_link_hb", 3072, NULL, 4, NULL) != pdPASS) {
        nuos_status_led_set_state(NUOS_LED_ERROR);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "H2 UART1 ready TX=%d RX=%d 115200 8N1",
             NUOS_LINK_TX_PIN, NUOS_LINK_RX_PIN);
    ESP_LOGI(TAG, "H2 runtime: persisted config is restored before Zigbee endpoint creation; no Zigbee report is sent on join");
    return ESP_OK;
}

bool nuos_link_peer_present(void)
{
    return c6_uart_recent();
}
