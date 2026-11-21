#ifndef NUOS_LINK_PROTO_H
#define NUOS_LINK_PROTO_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#define NUOS_LINK_SOF0 0xA5u
#define NUOS_LINK_SOF1 0x5Au
#define NUOS_LINK_VERSION 0x02u
#define NUOS_LINK_HDR_LEN 8u
#define NUOS_LINK_CRC_LEN 2u
#define NUOS_LINK_MAX_PAYLOAD 1024u
#define NUOS_LINK_MAX_FRAME (NUOS_LINK_HDR_LEN+NUOS_LINK_MAX_PAYLOAD+NUOS_LINK_CRC_LEN)
#define NUOS_LINK_TYPE_EP_DATA 0x20u
#define NUOS_LINK_TYPE_ACK 0x02u
#define NUOS_LINK_TYPE_NAK 0x03u
#define NUOS_LINK_TYPE_STATUS 0x10u
#define NUOS_LINK_TYPE_PING 0x11u
#define NUOS_LINK_TYPE_WRITE_RESULT 0x21u
/* C6 <-> H2 dynamic Zigbee configuration messages */
#define NUOS_LINK_TYPE_ZB_CONFIG_BEGIN 0x30u
#define NUOS_LINK_TYPE_ZB_CONFIG_EP 0x31u
#define NUOS_LINK_TYPE_ZB_CONFIG_COMMIT 0x32u
#define NUOS_ZB_CONFIG_VERSION 0x02u
#define NUOS_LINK_ST_OK 0x00u
#define NUOS_LINK_ST_NOT_JOINED 0x01u
#define NUOS_LINK_ST_BAD_ENDPOINT 0x02u
#define NUOS_LINK_ST_BAD_LENGTH 0x03u
#define NUOS_LINK_ST_ZB_ERROR 0x04u
#define NUOS_LINK_ST_BUSY 0x05u
#define NUOS_LINK_FLAG_ZB_JOINED 0x01u
#define NUOS_LINK_FLAG_ZB_STARTED 0x02u
#define NUOS_LINK_FIRST_ENDPOINT 1u
#define NUOS_LINK_TOTAL_ENDPOINTS 50u
#define NUOS_LINK_MAX_ATTRIBUTES 64u /* not a per-endpoint allocation; actual count is dynamic */
#define NUOS_LINK_BAUDRATE 115200

/* Shared C6/H2 data-type IDs. These IDs are part of the UART ABI. */
#define NUOS_TYPE_BOOL     0u
#define NUOS_TYPE_UINT16   1u
#define NUOS_TYPE_INT16    2u
#define NUOS_TYPE_UINT32   3u
#define NUOS_TYPE_INT32    4u
#define NUOS_TYPE_FLOAT32  5u
#define NUOS_TYPE_UINT64   6u
#define NUOS_TYPE_INT64    7u
#define NUOS_TYPE_STRING   8u
#define NUOS_TYPE_UINT8    9u
#define NUOS_TYPE_INT8     10u
#define NUOS_TYPE_UINT24   11u
#define NUOS_TYPE_UINT48   12u

typedef struct { uint8_t type,seq,endpoint; uint16_t len; const uint8_t *payload; } nuos_link_frame_t;
typedef struct { uint8_t buf[NUOS_LINK_MAX_FRAME]; uint16_t idx,need; uint32_t crc_errors,resyncs; } nuos_link_parser_t;
static inline uint16_t nuos_link_crc16(const uint8_t *b,uint16_t n){uint16_t c=0xFFFF;for(uint16_t p=0;p<n;p++){c^=b[p];for(uint8_t i=0;i<8;i++)c=(c&1)?(c>>1)^0xA001:(c>>1);}return c;}

/* CRC32 used for C6 <-> H2 Zigbee configuration hash.
 * Polynomial: CRC-32/IEEE 802.3 (0xEDB88320 reflected)
 */
static inline uint32_t nuos_link_crc32_update(uint32_t crc,
                                              const uint8_t *data,
                                              size_t len)
{
    while (len--) {
        crc ^= (uint32_t)(*data++);

        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static inline uint32_t nuos_link_crc32_finish(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}
static inline uint16_t nuos_link_get_u16(const uint8_t *p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
static inline uint16_t nuos_link_build(uint8_t *out, uint8_t type, uint8_t seq, uint8_t ep,
                                        const uint8_t *payload, uint16_t len)
{
    if (out == NULL || len > NUOS_LINK_MAX_PAYLOAD) {
        return 0;
    }
    if (len > 0 && payload == NULL) {
        return 0;
    }

    out[0] = NUOS_LINK_SOF0;
    out[1] = NUOS_LINK_SOF1;
    out[2] = NUOS_LINK_VERSION;
    out[3] = type;
    out[4] = seq;
    out[5] = ep;
    out[6] = (uint8_t)len;
    out[7] = (uint8_t)(len >> 8);
    if (len > 0) {
        memcpy(&out[8], payload, len);
    }
    uint16_t c = nuos_link_crc16(&out[2], (uint16_t)(6 + len));
    out[8 + len] = (uint8_t)c;
    out[9 + len] = (uint8_t)(c >> 8);
    return (uint16_t)(10 + len);
}

static inline void nuos_link_parser_init(nuos_link_parser_t *p){memset(p,0,sizeof(*p));}
static inline bool nuos_link_parser_push(nuos_link_parser_t *p,uint8_t b,nuos_link_frame_t *f){if(!p||!f)return false;if(p->idx==0){if(b==NUOS_LINK_SOF0)p->buf[p->idx++]=b;return false;}if(p->idx==1){if(b==NUOS_LINK_SOF1)p->buf[p->idx++]=b;else p->idx=(b==NUOS_LINK_SOF0)?1:0;return false;}if(p->idx>=sizeof(p->buf)){p->idx=0;p->need=0;p->resyncs++;return false;}p->buf[p->idx++]=b;if(p->idx==3&&b!=NUOS_LINK_VERSION){p->idx=0;p->need=0;p->resyncs++;return false;}if(p->idx==8){uint16_t l=nuos_link_get_u16(&p->buf[6]);if(l>NUOS_LINK_MAX_PAYLOAD){p->idx=0;p->need=0;p->resyncs++;return false;}p->need=(uint16_t)(10+l);}if(p->need==0||p->idx<p->need)return false;uint16_t l=nuos_link_get_u16(&p->buf[6]);uint16_t calc=nuos_link_crc16(&p->buf[2],(uint16_t)(6+l));uint16_t rx=nuos_link_get_u16(&p->buf[8+l]);p->idx=0;p->need=0;if(calc!=rx){p->crc_errors++;return false;}f->type=p->buf[3];f->seq=p->buf[4];f->endpoint=p->buf[5];f->len=l;f->payload=&p->buf[8];return true;}

uint32_t nuos_link_crc32_update(uint32_t crc,
                                const uint8_t *data,
                                size_t len);

uint32_t nuos_link_crc32_finish(uint32_t crc);
#endif
