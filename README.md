# NUOS BMS - split into two boards

The single-chip firmware (Modbus master + 30 Zigbee endpoints on one ESP32)
is now two independent IDF projects joined by a framed UART link.

```
   RS485 bus                    UART link                   Zigbee (2.4 GHz)
 ───────────────┐        ┌────────────────────────┐        ┌─────────────────
                │        │                        │        │
   Modbus  ─────┤ ESP32  │  A5 5A ... 49 bytes    │ ESP32  │  cluster 0xFCE0
   slave        │  -C6   ├───────────────────────►│  -H2   ├───► coordinator
   (FC03)  ◄────┤        │◄───── ACK / STATUS ────┤        │     / Tuya bridge
                │ UART1  │        UART0 <-> UART1 │ UART1  │
 ───────────────┘        └────────────────────────┘        └─────────────────
   Section 1: no Zigbee stack           Section 2: no Modbus master
```

| | Section 1 | Section 2 |
| --- | --- | --- |
| Directory | `01_esp32c6_modbus_uart` | `02_esp32h2_zigbee_bridge` |
| Target | `esp32c6` | `esp32h2` |
| Project name | `nuos_modbus_uart_c6` | `nuos_zigbee_bridge_h2` |
| Owns | RS485 FC03 polling, endpoint sequencing | 30 endpoints, ZCL attributes, APS reports |
| Dependencies | none (hand-rolled FC03) | `esp-zigbee-lib`, `esp-zboss-lib` |

## Wiring

| ESP32-C6 | direction | ESP32-H2 |
| --- | --- | --- |
| GPIO16 - UART0 TX | ──► | GPIO11 - UART1 RX |
| GPIO17 - UART0 RX | ◄── | GPIO10 - UART1 TX |
| GND | ─── | GND |

RS485 stays exactly where it was on the C6: UART1, TX GPIO3, RX GPIO2,
DE GPIO5, 9600 8N1, slave 1, FC03 register 0.

Pins are single `#define`s at the top of each `nuos_uart_link.h`. To make it
UART0 on both sides, set `NUOS_LINK_UART` to `UART_NUM_0` on the H2, use pins
24/23 (its UART0 defaults), and move that console to USB-Serial-JTAG the same
way the C6 `sdkconfig.defaults` does.

## The one thing that will bite you

**UART0 on the C6 is the console.** Every `ESP_LOGI` would be injected into the
middle of the data frames. `sdkconfig.defaults` therefore sets:

```
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
CONFIG_ESP_CONSOLE_SECONDARY_NONE=y
```

Flash and monitor the C6 over its USB port after that. The ROM bootloader
still emits a few bytes on UART0 at reset; the H2 parser treats them as junk
and resynchronises on the next `A5 5A`, so no special handling is needed.

## Link protocol

Defined once in `nuos_link_proto.h`, which **must stay byte-identical in both
projects** (they are identical copies today - `md5sum` them after any edit).
The file is pure C with no IDF dependency, so it also builds on a host for
testing.

```
off  size  field
 0    1    SOF0        0xA5
 1    1    SOF1        0x5A
 2    1    VER         0x01
 3    1    TYPE
 4    1    SEQ         echoed in the ACK
 5    1    ENDPOINT    1..30
 6    1    LEN         payload length
 7   LEN   PAYLOAD
7+L   2    CRC16       CRC-16/MODBUS over offsets 2..6+LEN, little-endian
```

| TYPE | Direction | Payload |
| --- | --- | --- |
| `0x01` EP_DATA | C6 → H2 | 10 × uint32 LE = 40 B (frame total 49 B) |
| `0x02` ACK | H2 → C6 | 1 B status |
| `0x03` NAK | H2 → C6 | 1 B status |
| `0x04` RAW_ASDU | C6 → H2 | cluster(2) + dst short(2) + dst EP(1) + ASDU |
| `0x10` STATUS | H2 → C6 | 1 B flags, bit0 = Zigbee joined |
| `0x11` PING | C6 → H2 | none - asks for a STATUS |

The CRC reuses the polynomial and seed of the existing Modbus CRC, so there is
one CRC routine in the product, not two.

### Flow control and gating

- The H2 sends a STATUS heartbeat every 2 s. The C6 polls only while
  `joined == true` **and** the last frame is under 7 s old (three missed
  beats), which is what replaced `esp_zb_bdb_dev_joined()` on that chip.
- Every EP_DATA frame is acknowledged. The C6 waits up to 300 ms, retries
  once, and moves on to the next endpoint rather than stalling the cycle.
- `NUOS_LINK_ST_NOT_JOINED` is not retried - the C6 goes back to waiting.
- On the H2 the parser never blocks on Zigbee: frames go into an 8-deep
  queue and a worker task does the ZCL work. A full queue is answered with
  `NUOS_LINK_ST_BUSY` instead of overrunning the UART.

### Timing

49 bytes at 115200 8N1 is ~4.3 ms, against ~16 ms for the FC03 exchange at
9600 baud and two APS frames on the Zigbee side. The link is not the
bottleneck; raise `NUOS_LINK_BAUDRATE` in the shared header (both sides) if
the payload ever grows past 10 registers.

## What changed in the code

**Section 1 (C6)**

- `modbus_master.c` - Modbus logic untouched. `esp_zigbee_core.h`,
  `esp_zb_bdb_dev_joined()` and `nuos_update_endpoint_from_modbus()` are
  replaced by `nuos_link_peer_joined()` and `nuos_link_send_endpoint()`.
  The old `count++ > 3000` / 10 ms loop is now `JOIN_SETTLE_MS 30000`, the
  same ~30 s settle written once.
- `nuos_uart_link.c` - framing, TX with ACK wait, RX task for ACK/STATUS.
- `main.c` - link first, then Modbus. No NVS, no Zigbee, no `app_config.h`
  (that file only selected lighting device types and is not used by either
  section - keep it in your tree if other products share the repo).

**Section 2 (H2)**

- `app_zigbee_clusters.c/.h` - the endpoint model, the 72-byte APS limit and
  the 5-attributes-per-frame chunking are unchanged, because that knowledge
  belongs on the chip that owns the radio. Renamed
  `nuos_update_endpoint_from_modbus` → `nuos_update_endpoint_values` and
  `nuos_set_modbus_value` → `nuos_set_attr_value`; both old names still work
  through macros. Added `nuos_send_raw_asdu()` for passthrough mode.
- `nuos_uart_link.c` - RX parser task, Zigbee worker task, STATUS heartbeat.
- `esp_zb_controller_main.c` - `modbus_master_init()` is gone from the signal
  handler; `nuos_link_init()` runs in `app_main` before the stack so early
  frames are answered with "not joined". `nuos_link_notify_zb_started()` is
  called after `esp_zb_start()` because `esp_zb_lock_acquire()` must not be
  touched before that.
- `app_zigbee_clusters.h` has `_Static_assert`s tying `NUOS_ATTR_COUNT`,
  `NUOS_FIRST_ENDPOINT` and `NUOS_TOTAL_ENDPOINTS` to the link constants, so
  the two firmwares cannot silently drift apart.

## Two modes for "send the payload over Zigbee"

**EP_DATA (default, recommended).** The C6 sends 10 raw values; the H2 writes
them into the ZCL database with `esp_zb_zcl_set_attribute_val()` and builds
the Report Attributes frames. Attribute reads from the coordinator keep
working, and the APS size limit stays on the chip that measured it.

**RAW_ASDU (`0x04`, optional).** The C6 builds the ZCL frame itself and the H2
relays it verbatim through `esp_zb_aps_data_request()`. Call
`nuos_link_send_raw_asdu()` instead of `nuos_link_send_endpoint()` if you want
this. Two caveats: the local attribute database on the H2 is then never
updated, so reads return stale values; and the ZCL transaction sequence
numbers come from a chip that knows nothing about the Zigbee state. Max ASDU
in one link frame is 59 bytes.

## Build

```bash
# Section 1
cd 01_esp32c6_modbus_uart
idf.py set-target esp32c6
idf.py build flash monitor          # monitor over USB-Serial-JTAG

# Section 2
cd ../02_esp32h2_zigbee_bridge
idf.py set-target esp32h2
idf.py build flash monitor
```

The H2 project ships `partitions.csv` with the `zb_storage` and `zb_fct`
partitions the Zigbee stack requires. If you had `override_path` pointing at a
local `esp-zboss-lib` checkout, put that back in
`02_esp32h2_zigbee_bridge/main/idf_component.yml`.

## Bring-up checklist

1. Flash the H2 first and commission it. Its log shows
   `Network steering successful`.
2. Flash the C6. Its log shows `Peer reports Zigbee joined`, then the 30 s
   settle, then `REG0 = ...` and `EP1: 10 values acknowledged by Zigbee board`.
3. The H2 log shows `EP1 <- link: [...]` followed by
   `EP1 reported 10 attrs in 2 frame(s)`.
4. `link stats:` is printed once per cycle on the C6. `crc_err` climbing means
   a cabling or baud problem; `timeout` climbing means the H2 is not answering;
   `busy_drop` on the H2 means the Zigbee side is slower than the poll rate -
   raise `INTER_CYCLE_MS` or `NUOS_LINK_QUEUE_DEPTH`.

Nothing here has been compiled against ESP-IDF - only `nuos_link_proto.h` was
built and round-trip tested on a host (junk resync, CRC rejection, 10-value
pack/unpack). Expect the usual small fixes on the first `idf.py build`,
especially around the APSDE struct names, which move between esp-zigbee-sdk
releases (`NUOS_APSDE_REQUEST` at the top of `app_zigbee_clusters.c` is the
knob for that).
#   N U O S _ B M S _ M O D B U S  
 