idf_component_register(
    SRCS
        "esp_zb_controller_main.c"
        "app_zigbee_clusters.c"
        "nuos_dynamic_zb_config.c"
        "nuos_uart_link.c"
        "light_driver.c"
    INCLUDE_DIRS "."
    REQUIRES led_strip espressif__esp-zboss-lib mbedtls nvs_flash esp_timer app_update
)
