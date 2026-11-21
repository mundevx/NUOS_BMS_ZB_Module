NUOS H2 ROUTER / JOIN COORDINATOR FIX
====================================

Role:
- ESP32-H2 is a Zigbee ROUTER.
- The H2 joins an existing Zigbee coordinator network using network steering.
- It does NOT form or own the Zigbee network.

Fixes:
1. Restored ESP_ZB_ZR_CONFIG() in esp_zb_controller_main.c.
2. Restored factory-new network steering and rejoin/recovery behavior.
3. H2 shared Zigbee readiness is set TRUE only after successful steering/rejoin.
4. H2 shared Zigbee readiness is cleared on startup, steering failure,
   parent unavailable and leave.
5. C6 STATUS therefore becomes flags=0x03 only after H2 is actually joined.
6. Removed coordinator-only factory-reset/migration logic.
7. Dynamic endpoint configuration rebuild remains supported.
8. Removed incompatible scheduler callback casts in the controller.

Expected runtime sequence:
    H2 boot
      -> Zigbee stack initialize
      -> factory-new: NETWORK STEERING
         or existing credentials: normal rejoin, then recovery steering if needed
      -> ESP_ZB_BDB_SIGNAL_STEERING success
      -> network_ready=true
      -> C6 H2 STATUS flags=0x03
      -> C6 EP_DATA transmissions accepted

Build:
    idf.py fullclean
    idf.py build
    idf.py flash monitor

Do NOT erase NVS as part of this fix. The existing router network credentials
must remain available for rejoin.
