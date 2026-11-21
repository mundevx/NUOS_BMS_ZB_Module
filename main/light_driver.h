#ifndef NUOS_LIGHT_DRIVER_H
#define NUOS_LIGHT_DRIVER_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Board status LED: WS2812 on ESP32-H2 GPIO8. */
#define NUOS_WS2812_GPIO 8
#define NUOS_WS2812_COUNT 1
#define LIGHT_DEFAULT_OFF 0
esp_err_t light_driver_init(int initial_state);
esp_err_t light_driver_set_power(bool on);
esp_err_t light_driver_set_color_RGB(uint8_t r, uint8_t g, uint8_t b);
#endif
