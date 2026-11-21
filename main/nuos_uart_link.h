#ifndef NUOS_UART_LINK_H
#define NUOS_UART_LINK_H
#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "nuos_link_proto.h"
#define NUOS_LINK_UART UART_NUM_1
#define NUOS_LINK_TX_PIN 5
#define NUOS_LINK_RX_PIN 4
#define NUOS_LINK_RX_BUF 4096
#define NUOS_LINK_TX_BUF 4096
#define NUOS_LINK_QUEUE_DEPTH 8
#define NUOS_LINK_HEARTBEAT_MS 1000
esp_err_t nuos_link_init(void);
void nuos_link_notify_zb_started(void);
bool nuos_link_peer_present(void);

/* Status LED state machine. The LED is always blinking; the LED task is the
 * only code that drives light_driver. */
typedef enum {
    NUOS_LED_BOOT = 0,
    NUOS_LED_UART_NOT_DETECTED,
    NUOS_LED_UART_OK,
    NUOS_LED_ZIGBEE_STARTED,
    NUOS_LED_JOINING,
    NUOS_LED_JOINED,
    NUOS_LED_JOINED_UART_OK,
    NUOS_LED_REJOINING,
    NUOS_LED_ERROR
} nuos_led_state_t;

void nuos_status_led_set_state(nuos_led_state_t state);
#endif
