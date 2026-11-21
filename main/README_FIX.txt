NUOS H2 Zigbee Join Fix
=======================

Baseline: last known-good H2 firmware dated 2026-09-26 09:21/09:22.

Changes:
1. esp_zb_controller_main.c
   - Removed Trust Center security-key code/include from the startup path.
   - Removed the periodic dynamic endpoint rebuild watcher.
   - Removed the startup-time factory-reset/rebuild path caused by the rebuild flag.
   - Preserved the known-good BDB initialization -> factory-new steering / existing-network rejoin sequence.
   - Preserved network_ready state updates used by the UART/application layer.

2. app_zigbee_clusters.c
   - Dynamic configured endpoints are still used when persisted C6 configuration exists.
   - If persisted endpoint count is zero, EP1 temporary fallback is registered so the router has a valid Zigbee endpoint and can commission/join.
   - The fallback is only for the empty-NVS/first-boot case. A C6 configuration COMMIT restarts H2, and the next boot instantiates the configured endpoint map.
   - Removed redundant rebuild-flag request from config save because the UART COMMIT already restarts H2.

3. nuos_uart_link.c
   - Configuration COMMIT continues to persist the map and then calls esp_restart().
   - Removed the redundant second rebuild-request operation.

Expected first-boot log:
  ZB_CONFIG NVS: restored endpoints=0 ...
  No persisted C6 configuration; using temporary EP1 fallback for Zigbee commissioning
  Zigbee endpoints created=1 ...
  Zigbee started, factory_new=YES
  ... network steering ...

Expected configured-boot behavior:
  ZB_CONFIG NVS: restored endpoints=<N> ...
  Zigbee endpoints created=<N> ...
  ... normal BDB rejoin/steering ...

Do not enable NUOS_ZIGBEE_FACTORY_RESET_ON_BOOT for normal operation.
