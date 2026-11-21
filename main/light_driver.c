#include "light_driver.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"


static const char *TAG = "LIGHT_DRV";
static led_strip_handle_t s_strip;
static SemaphoreHandle_t s_mutex;
static uint8_t s_r, s_g, s_b;
static bool s_power;

static esp_err_t refresh_locked(void)
{
    if (!s_strip) return ESP_ERR_INVALID_STATE;
    uint8_t r = s_power ? s_r : 0;
    uint8_t g = s_power ? s_g : 0;
    uint8_t b = s_power ? s_b : 0;
    esp_err_t e = led_strip_set_pixel(s_strip, 0, r, g, b);
    if (e == ESP_OK) e = led_strip_refresh(s_strip);
    return e;
}

esp_err_t light_driver_init(int initial_state)
{
    (void)initial_state;
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    if (s_strip) return ESP_OK;

    const led_strip_config_t cfg = {
        .strip_gpio_num = NUOS_WS2812_GPIO,
        .max_leds = NUOS_WS2812_COUNT,
    };
    const led_strip_rmt_config_t rmt = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    esp_err_t e = led_strip_new_rmt_device(&cfg, &rmt, &s_strip);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 init failed: %s", esp_err_to_name(e));
        return e;
    }
    s_r = s_g = s_b = 0;
    s_power = false;
    e = led_strip_clear(s_strip);
    ESP_LOGI(TAG, "WS2812 status LED initialized on GPIO%d", NUOS_WS2812_GPIO);
    return e;
}

esp_err_t light_driver_set_color_RGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_r = r; s_g = g; s_b = b;
    esp_err_t e = refresh_locked();
    xSemaphoreGive(s_mutex);
    return e;
}

esp_err_t light_driver_set_power(bool on)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_power = on;
    esp_err_t e = refresh_locked();
    xSemaphoreGive(s_mutex);
    return e;
}
